//! Viewer-side TCP frame server (stage 5b).
//!
//! The viewer LISTENS (default `127.0.0.1:39201`); sims dial in with
//! [`NetVis`](crate::net_vis::NetVis) streams (`[u32 LE len][postcard]`,
//! see [`crate::proto`]). One background thread accepts and keeps only the
//! LATEST frame — no interpolation, no queue. At 120Hz ticks sampled by a
//! 60Hz viewer this matches the old in-process look; a slow viewer drops
//! frames by construction, never lags.
//!
//! Sim disconnect (exit/restart) returns to `Waiting` and the accept loop
//! takes the next sim; the viewer keeps showing the last frame meanwhile.

use std::{
    io::Read,
    net::{SocketAddr, TcpListener, TcpStream},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::JoinHandle,
    time::Duration,
};

use crate::proto::{VisFrame, decode_frame};

/// Connection state for the overlay.
#[derive(Clone, Debug)]
pub enum ConnStatus {
    /// No sim yet (or lost it) — still accepting.
    Waiting,
    /// Streaming from a sim.
    Connected {
        /// Sim address string.
        addr: String,
    },
}

/// Latest-frame receiver. Spawn with [`FrameClient::connect`]; read with
/// [`FrameClient::latest`] (clones the newest `VisFrame`, if any).
pub struct FrameClient {
    frame: Arc<Mutex<Option<VisFrame>>>,
    status: Arc<Mutex<ConnStatus>>,
    /// Total frames decoded (overlay diagnostics).
    received: Arc<Mutex<u64>>,
    local_addr: SocketAddr,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl FrameClient {
    /// Binds `addr` and starts the background accept loop. Panics if the
    /// port is busy (another viewer is already up — connect the sim to it
    /// instead of starting a second viewer).
    pub fn connect(addr: &str) -> Self {
        let listener = TcpListener::bind(addr)
            .unwrap_or_else(|e| panic!("vis_viewer: cannot listen on {addr} ({e})"));
        listener
            .set_nonblocking(true)
            .expect("vis_viewer: nonblocking listen failed");
        let local_addr = listener.local_addr().expect("vis_viewer: no local addr");
        let frame = Arc::new(Mutex::new(None));
        let status = Arc::new(Mutex::new(ConnStatus::Waiting));
        let received = Arc::new(Mutex::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let worker = {
            let (frame, status, received, stop) = (
                frame.clone(),
                status.clone(),
                received.clone(),
                stop.clone(),
            );
            std::thread::spawn(move || accept_loop(listener, &frame, &status, &received, &stop))
        };
        Self {
            frame,
            status,
            received,
            local_addr,
            stop,
            worker: Some(worker),
        }
    }

    /// Address actually bound (useful with port 0 in tests).
    pub fn local_addr(&self) -> SocketAddr {
        self.local_addr
    }

    /// Clones the newest frame, or `None` if no sim has sent one yet.
    /// Cloning an `ArenaState` per viewer frame (~120Hz state, few KB) is
    /// cheaper than holding the lock through the draw.
    pub fn latest(&self) -> Option<VisFrame> {
        self.frame.lock().ok().and_then(|g| g.clone())
    }

    /// Current connection state (for the overlay).
    pub fn status(&self) -> ConnStatus {
        self.status
            .lock()
            .map(|g| g.clone())
            .unwrap_or(ConnStatus::Waiting)
    }

    /// Total frames decoded so far.
    pub fn received(&self) -> u64 {
        self.received.lock().map(|g| *g).unwrap_or(0)
    }

    /// Stops the receiver. Called automatically on drop.
    pub fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.worker.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for FrameClient {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn accept_loop(
    listener: TcpListener,
    frame: &Mutex<Option<VisFrame>>,
    status: &Mutex<ConnStatus>,
    received: &Mutex<u64>,
    stop: &AtomicBool,
) {
    while !stop.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, _)) => {
                *status.lock().unwrap() = ConnStatus::Connected {
                    addr: stream
                        .peer_addr()
                        .map(|a| a.to_string())
                        .unwrap_or_default(),
                };
                pump(stream, frame, received, stop);
                *status.lock().unwrap() = ConnStatus::Waiting;
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                std::thread::sleep(Duration::from_millis(50));
            }
            Err(_) => {
                std::thread::sleep(Duration::from_millis(200));
            }
        }
    }
}

/// Reads length-prefixed frames until EOF/error/stop, keeping the newest.
fn pump(
    mut stream: TcpStream,
    frame: &Mutex<Option<VisFrame>>,
    received: &Mutex<u64>,
    stop: &AtomicBool,
) {
    let _ = stream.set_read_timeout(Some(Duration::from_millis(200)));
    let mut buf = Vec::new();
    let mut chunk = [0u8; 65536];
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        match stream.read(&mut chunk) {
            Ok(0) => return, // sim closed
            Ok(n) => {
                buf.extend_from_slice(&chunk[..n]);
                while let Ok(Some((decoded, consumed))) = decode_frame(&buf) {
                    if let Ok(mut slot) = frame.lock() {
                        *slot = Some(decoded);
                    }
                    if let Ok(mut count) = received.lock() {
                        *count += 1;
                    }
                    buf.drain(..consumed);
                }
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::TimedOut
                    || e.kind() == std::io::ErrorKind::WouldBlock =>
            {
                continue;
            }
            Err(_) => return,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{io::Write, time::Instant};

    use super::*;
    use crate::proto::encode_frame;

    #[test]
    fn server_stores_latest_frame() {
        crate::photo::init_meshes();
        let client = FrameClient::connect("127.0.0.1:0");
        let addr = client.local_addr();

        // Sim side: dial in and send three ticks back-to-back, then close.
        let mut stream = TcpStream::connect(addr).unwrap();
        let mut arena = crate::photo::build_arena();
        for _ in 0..3 {
            arena.step_tick();
            let bytes = encode_frame(&crate::proto::VisFrame::new(
                arena.get_arena_state(),
                1.0 / 120.0,
            ));
            stream.write_all(&bytes).unwrap();
        }
        drop(stream);

        // Tick 3 must arrive (10s budget, polls the lock).
        let deadline = Instant::now() + Duration::from_secs(10);
        let tick = loop {
            if let Some(frame) = client.latest()
                && frame.state.tick_count == 3
            {
                break frame.state.tick_count;
            }
            assert!(Instant::now() < deadline, "tick 3 never arrived");
            std::thread::sleep(Duration::from_millis(20));
        };
        assert_eq!(tick, 3);
        assert!(client.received() >= 3);
    }

    #[test]
    fn server_waits_for_sim() {
        // Bound but nobody dials: status stays Waiting, latest is None,
        // and nothing blocks or panics.
        let client = FrameClient::connect("127.0.0.1:0");
        std::thread::sleep(Duration::from_millis(200));
        assert!(client.latest().is_none());
        assert!(matches!(client.status(), ConnStatus::Waiting));
        assert_eq!(client.received(), 0);
    }
}
