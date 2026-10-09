//! Detached-viewer bridge: a [`rocketsim::Vis`] that forwards ticks to a
//! separate viewer process over TCP without ever blocking the sim.
//!
//! The sim side pays one `ArenaState` clone per tick plus a `try_lock`;
//! serialization and sockets live on a background sender thread with a
//! latest-only slot, so a slow or missing viewer can only cause dropped
//! frames, never a stalled tick.
//!
//! Wire format: [`crate::proto`] (`[u32 LE len][postcard(VisFrame)]`).

use std::{
    io::Write,
    net::TcpStream,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::JoinHandle,
    time::Duration,
};

use rocketsim::{ArenaState, Vis};

use crate::proto::{VisFrame, encode_frame};

/// [`rocketsim::Vis`] implementation that streams `(ArenaState, dt)` to a
/// viewer process listening on `addr` (e.g. `"127.0.0.1:39201"`).
///
/// Construct with [`NetVis::connect`], or attach to an arena with
/// `ArenaVisExt::set_net_vis_enabled`.
/// Dropping the instance stops the sender thread and detaches the viewer.
pub struct NetVis {
    slot: Arc<Mutex<Option<VisFrame>>>,
    stop: Arc<AtomicBool>,
    sender: Option<JoinHandle<()>>,
}

impl NetVis {
    /// Starts the background sender; never blocks or fails. If no viewer
    /// listens, the sender thread retries with backoff until one appears —
    /// ticks in the meantime are dropped, never queued.
    pub fn connect(addr: &str) -> Self {
        let slot = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let sender = {
            let (slot, stop, addr) = (slot.clone(), stop.clone(), addr.to_string());
            std::thread::spawn(move || sender_loop(&addr, &slot, &stop))
        };
        Self {
            slot,
            stop,
            sender: Some(sender),
        }
    }
}

impl Drop for NetVis {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.sender.take() {
            let _ = handle.join();
        }
    }
}

impl Vis for NetVis {
    fn update(&mut self, arena_state: &ArenaState, dt: f32) {
        // Clone is ~0.1 µs (6 cars); encode happens on the sender thread.
        // try_lock: if the sender is momentarily holding the slot, drop this
        // tick rather than stall the sim. The slot always keeps the NEWEST
        // frame (overwrite), so the viewer converges instead of lagging.
        if let Ok(mut slot) = self.slot.try_lock() {
            *slot = Some(VisFrame::new(arena_state.clone(), dt));
        }
    }
}

/// Makes `cargo run --example …` Just Work: if nothing listens on `addr`,
/// spawns the sibling `vis_viewer` binary (next to the current executable,
/// i.e. `target/{debug,release}/`) and waits for the port to accept.
/// Prints what it is doing; never fails (worst case the sim streams to a
/// viewer started later — [`NetVis`] connects lazily).
pub fn ensure_viewer_running(addr: &str) {
    if std::net::TcpStream::connect(addr).is_ok() {
        eprintln!("rocketsim_vis: viewer already listening on {addr}");
        return;
    }
    let viewer = std::env::current_exe().ok().and_then(|exe| {
        exe.parent()?
            .parent()
            .map(|dir| dir.join(bin_name("vis_viewer")))
    });
    match viewer {
        Some(path) if path.exists() => {
            eprintln!("rocketsim_vis: starting viewer {path:?} ...");
            match std::process::Command::new(&path)
                .arg(port_of(addr).to_string())
                .spawn()
            {
                Ok(child) => {
                    // Remember OUR viewer (not a pre-existing one) so
                    // `set_vis_enabled(false)` can close what it opened.
                    *spawned_viewer().lock().unwrap() = Some(child);
                }
                Err(e) => {
                    eprintln!(
                        "rocketsim_vis: could not spawn viewer ({e}); start it manually: vis_viewer"
                    );
                    return;
                }
            }
            for _ in 0..100 {
                if std::net::TcpStream::connect(addr).is_ok() {
                    eprintln!("rocketsim_vis: viewer up on {addr}");
                    return;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
            eprintln!("rocketsim_vis: viewer did not accept {addr} in time");
        }
        _ => eprintln!(
            "rocketsim_vis: no viewer on {addr} — start one: cargo run -p rocketsim_vis --bin vis_viewer"
        ),
    }
}

/// Viewer process WE spawned via [`ensure_viewer_running`] (never a
/// pre-existing one). `set_vis_enabled(false)` closes what it opened.
fn spawned_viewer() -> &'static Mutex<Option<std::process::Child>> {
    static SPAWNED: std::sync::OnceLock<Mutex<Option<std::process::Child>>> =
        std::sync::OnceLock::new();
    SPAWNED.get_or_init(|| Mutex::new(None))
}

/// Stops the auto-spawned viewer, if it is still alive. Never touches a
/// viewer the user started themselves (no handle = not ours).
pub(crate) fn stop_spawned_viewer() {
    let mut guard = spawned_viewer().lock().unwrap();
    if let Some(mut child) = guard.take() {
        // Already exited (or never really started): ignore the status.
        let _ = child.try_wait();
        let _ = child.kill();
    }
}

fn bin_name(base: &str) -> String {
    if cfg!(windows) {
        format!("{base}.exe")
    } else {
        base.to_string()
    }
}

fn port_of(addr: &str) -> u16 {
    addr.rsplit(':')
        .next()
        .and_then(|port| port.parse().ok())
        .unwrap_or(crate::proto::DEFAULT_PORT)
}

fn sender_loop(addr: &str, slot: &Mutex<Option<VisFrame>>, stop: &AtomicBool) {
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
                pump(stream, slot, stop);
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
fn pump(mut stream: TcpStream, slot: &Mutex<Option<VisFrame>>, stop: &AtomicBool) {
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        let frame = slot.lock().map(|mut s| s.take()).unwrap_or(None);
        match frame {
            Some(frame) => {
                let bytes = encode_frame(&frame);
                if stream.write_all(&bytes).is_err() {
                    // Put the frame back only if nothing newer arrived;
                    // never overwrite a newer frame with an older one.
                    if let Ok(mut s) = slot.lock()
                        && s.is_none()
                    {
                        *s = Some(frame);
                    }
                    return;
                }
            }
            None => std::thread::sleep(Duration::from_millis(2)),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{io::Read, net::TcpListener, time::Instant};

    use super::*;
    use crate::proto::decode_frame;

    /// Reads chunks until a frame with `want_tick` arrives or `deadline`
    /// passes. Returns the newest tick seen (proves latest-only convergence).
    fn wait_for_tick(mut stream: TcpStream, want_tick: u64, deadline: Instant) -> Option<u64> {
        stream
            .set_read_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        let mut buf = Vec::new();
        let mut chunk = [0u8; 65536];
        let mut last = None;
        loop {
            if Instant::now() > deadline {
                return last;
            }
            match stream.read(&mut chunk) {
                Ok(0) => return last, // sender closed
                Ok(n) => {
                    buf.extend_from_slice(&chunk[..n]);
                    while let Some((frame, consumed)) = decode_frame(&buf).expect("valid frames") {
                        last = Some(frame.state.tick_count);
                        buf.drain(..consumed);
                        if last == Some(want_tick) {
                            return last;
                        }
                    }
                }
                Err(e)
                    if e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    continue; // sender still connecting/encoding
                }
                Err(_) => return last,
            }
        }
    }

    #[test]
    fn loopback_delivers_latest_frame() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let mut vis = NetVis::connect(&addr);
        // Server thread owns the read end and reports the newest tick seen;
        // the main thread must NOT drop `vis` first (that would stop the
        // sender before it pumps — the exact race this ordering avoids).
        let (tx, rx) = std::sync::mpsc::channel();
        let server = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            let last = wait_for_tick(stream, 2, Instant::now() + Duration::from_secs(10));
            let _ = tx.send(last);
        });

        crate::photo::init_meshes();
        let mut arena = crate::photo::build_arena();
        // Two rapid updates; the viewer must converge to the newest (tick 2).
        for _ in 0..2 {
            arena.step_tick();
            let state = arena.get_arena_state();
            vis.update(&state, 1.0 / 120.0);
        }

        let last = rx.recv_timeout(Duration::from_secs(10)).unwrap();
        assert_eq!(last, Some(2), "must converge to newest tick");
        drop(vis);
        server.join().unwrap();
    }

    #[test]
    fn update_without_viewer_never_blocks() {
        // Port 1 never accepts: every update must still be fast (dropped).
        let mut vis = NetVis::connect("127.0.0.1:1");
        crate::photo::init_meshes();
        let arena = crate::photo::build_arena();
        let state = arena.get_arena_state();
        for _ in 0..1000 {
            let t = Instant::now();
            vis.update(&state, 1.0 / 120.0);
            assert!(t.elapsed() < Duration::from_micros(100), "update blocked!");
        }
    }
}
