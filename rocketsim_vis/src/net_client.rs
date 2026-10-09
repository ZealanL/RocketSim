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
    io::{Read, Write},
    net::{SocketAddr, TcpListener, TcpStream},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::JoinHandle,
    time::Duration,
};

use crate::proto::{ControlMsg, VisFrame, decode_frame, encode_control};

/// Viewer→sim control sender, viewer side (stage 5c).
///
/// The SIM listens (see `ControlServer` in [`crate::net_vis`]); this dials
/// the sim's control port lazily with `TCP_NODELAY` and streams [`ControlMsg`]
/// frames (`[u32 LE len][postcard]`, same framing as [`crate::proto`]).
/// Intended use (viewer integration — NOT wired yet, see drive.rs docs):
/// the viewer captures keyboard/mouse via egui input (works focused on
/// Wayland, unlike global `device_query` reads), bumps a sequence number,
/// and calls [`ControlSender::send`] once per viewer frame.
///
/// Latest-only slot + backoff reconnect mirror [`crate::net_vis::NetVis`]:
/// [`ControlSender::send`] overwrites with `try_lock` (drops the tick rather
/// than stalling the viewer when the sender thread momentarily holds the
/// slot); the background thread pumps newest-first and reconnects. Drop
/// stops the thread (`stop` + `join`); construction never blocks, fails, or
/// panics.
pub struct ControlSender {
    slot: Arc<Mutex<Option<ControlMsg>>>,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl ControlSender {
    /// Starts the background dialer for the sim's control port (e.g.
    /// `"127.0.0.1:39202"`). Never blocks or fails: with no sim listening
    /// the thread retries with backoff and inputs are dropped, never queued.
    pub fn connect(addr: &str) -> Self {
        let slot = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let worker = {
            let (slot, stop, addr) = (slot.clone(), stop.clone(), addr.to_string());
            std::thread::spawn(move || control_sender_loop(&addr, &slot, &stop))
        };
        Self {
            slot,
            stop,
            worker: Some(worker),
        }
    }

    /// Queues `msg` for sending (overwrites any unsent older message —
    /// latest-only, so rapid viewer frames converge instead of queueing).
    /// Lock-only via `try_lock`; never blocks the viewer, never panics.
    pub fn send(&self, msg: ControlMsg) {
        if let Ok(mut slot) = self.slot.try_lock() {
            *slot = Some(msg);
        }
    }

    /// Stops the sender. Called automatically on drop.
    pub fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.worker.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for ControlSender {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Dials with backoff until `stop`; on connect pumps newest-first, then
/// reconnects (mirrors the [`crate::net_vis`] sender loop).
fn control_sender_loop(addr: &str, slot: &Mutex<Option<ControlMsg>>, stop: &AtomicBool) {
    let mut backoff = Duration::from_millis(500);
    let max_backoff = Duration::from_secs(5);
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        match TcpStream::connect(addr) {
            Ok(stream) => {
                backoff = Duration::from_millis(500);
                let _ = stream.set_nodelay(true);
                control_sender_pump(stream, slot, stop);
                // pump returned = connection lost; reconnect with backoff.
            }
            Err(_) => {
                std::thread::sleep(backoff);
                backoff = (backoff * 2).min(max_backoff);
                continue;
            }
        }
    }
}

/// Sends newest-first until the connection breaks or `stop` is set.
fn control_sender_pump(mut stream: TcpStream, slot: &Mutex<Option<ControlMsg>>, stop: &AtomicBool) {
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        let msg = slot.lock().map(|mut s| s.take()).unwrap_or(None);
        match msg {
            Some(msg) => {
                let bytes = encode_control(&msg);
                if stream.write_all(&bytes).is_err() {
                    // Put the message back only if nothing newer arrived;
                    // never overwrite a newer message with an older one.
                    if let Ok(mut s) = slot.lock()
                        && s.is_none()
                    {
                        *s = Some(msg);
                    }
                    return;
                }
            }
            None => std::thread::sleep(Duration::from_millis(2)),
        }
    }
}

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

    /// Reads control messages until `want_seq` arrives or `deadline` passes;
    /// returns the newest seq seen.
    fn wait_for_seq(mut stream: TcpStream, want_seq: u64, deadline: Instant) -> Option<u64> {
        use crate::proto::decode_control;
        stream
            .set_read_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        let mut buf = Vec::new();
        let mut chunk = [0u8; 4096];
        let mut last = None;
        loop {
            if Instant::now() > deadline {
                return last;
            }
            match stream.read(&mut chunk) {
                Ok(0) => return last, // sender closed
                Ok(n) => {
                    buf.extend_from_slice(&chunk[..n]);
                    while let Some((msg, consumed)) = decode_control(&buf).expect("valid controls")
                    {
                        last = Some(msg.seq);
                        buf.drain(..consumed);
                        if last == Some(want_seq) {
                            return last;
                        }
                    }
                }
                Err(e)
                    if e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    continue;
                }
                Err(_) => return last,
            }
        }
    }

    #[test]
    fn control_sender_converges_to_newest() {
        // Fake sim: listen; the sender dials lazily (10s budget for dial).
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let sender = ControlSender::connect(&addr);

        // Rapid viewer frames: 50 sends back-to-back. Latest-only slot means
        // the sim side converges to seq 50 (intermediate drops are fine and
        // expected — the test only requires the NEWEST to arrive).
        for seq in 1..=50u64 {
            sender.send(ControlMsg::new(
                0,
                rocketsim::CarControls::default().with_throttle(seq as f32 / 50.0),
                0,
                seq,
            ));
        }

        let (tx, rx) = std::sync::mpsc::channel();
        let server = std::thread::spawn(move || {
            listener
                .set_nonblocking(false)
                .expect("test listener blocking");
            let (stream, _) = listener.accept().unwrap();
            let last = wait_for_seq(stream, 50, Instant::now() + Duration::from_secs(10));
            let _ = tx.send(last);
        });
        let last = rx.recv_timeout(Duration::from_secs(15)).unwrap();
        assert_eq!(last, Some(50), "must converge to newest control");
        drop(sender);
        server.join().unwrap();
    }

    #[test]
    fn control_send_without_sim_never_blocks() {
        // Port 1 never accepts: every send must still be fast (dropped).
        // 2s budget for 20k sends = <100µs each, same bar as NetVis::update.
        let sender = ControlSender::connect("127.0.0.1:1");
        let t = Instant::now();
        for seq in 1..=20_000u64 {
            sender.send(ControlMsg::new(
                0,
                rocketsim::CarControls::default(),
                0,
                seq,
            ));
        }
        assert!(
            t.elapsed() < Duration::from_secs(2),
            "control send blocked: {:?}",
            t.elapsed()
        );
    }
}
