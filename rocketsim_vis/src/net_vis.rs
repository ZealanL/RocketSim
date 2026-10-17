//! Detached-viewer bridge: a [`rocketsim::Vis`] that forwards ticks to a
//! separate viewer process over TCP without ever blocking the sim.
//! Latest-only slot on a background thread: a slow or missing viewer costs
//! dropped frames, never a stalled tick. Wire format: [`crate::proto`].

use std::{
    io::Read,
    net::{SocketAddr, TcpListener, TcpStream},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU8, Ordering},
    },
    thread::JoinHandle,
    time::Duration,
};

use rocketsim::{ArenaState, Vis};

use crate::{
    net_client::OutgoingBuffer,
    proto::{ControlMsg, DebugDraw, SceneMetadata, VisFrame, decode_control, encode_frame},
};

/// [`rocketsim::Vis`] implementation that streams `(ArenaState, dt)` to a
/// viewer process listening on `addr` (e.g. `"127.0.0.1:39201"`).
///
/// Construct with [`NetVis::connect`], or attach to an arena with
/// `ArenaVisExt::set_net_vis_enabled`.
/// Dropping the instance stops the sender thread and detaches the viewer.
pub struct NetVis {
    slot: Arc<Mutex<Option<VisFrame>>>,
    controls: Arc<Mutex<Option<(ControlMsg, std::time::Instant)>>>,
    actions: Arc<AtomicU8>,
    debug: Arc<Mutex<DebugDraw>>,
    scene: Arc<Mutex<Option<SceneMetadata>>>,
    address: String,
    connected: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    sender: Option<JoinHandle<()>>,
    pub(crate) lease: Option<crate::launcher::ViewerLease>,
}

impl NetVis {
    /// Starts the background sender; never blocks or fails. `addr` must be
    /// a numeric socket address because DNS lookup cannot be cancelled. If no viewer
    /// listens, the sender thread retries with backoff until one appears —
    /// ticks in the meantime are dropped, never queued.
    pub fn connect(addr: &str) -> Self {
        let slot = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let controls = Arc::new(Mutex::new(None));
        let actions = Arc::new(AtomicU8::new(0));
        let connected = Arc::new(AtomicBool::new(false));
        let debug = Arc::new(Mutex::new(DebugDraw::default()));
        let scene = Arc::new(Mutex::new(None));
        let address = addr.to_string();
        let sender = {
            let addr = address.clone();
            let shared = SenderState {
                slot: slot.clone(),
                stop: stop.clone(),
                controls: controls.clone(),
                actions: actions.clone(),
                connected: connected.clone(),
                scene: scene.clone(),
            };
            std::thread::spawn(move || sender_loop(&addr, shared))
        };
        Self {
            slot,
            controls,
            actions,
            debug,
            scene,
            address,
            connected,
            stop,
            sender: Some(sender),
            lease: None,
        }
    }

    pub fn address(&self) -> &str {
        &self.address
    }
    pub fn connected(&self) -> bool {
        self.connected.load(Ordering::Acquire)
    }
    pub fn latest_controls(&self) -> Option<ControlMsg> {
        self.controls
            .lock()
            .ok()
            .and_then(|g| *g)
            .and_then(|(msg, at)| (at.elapsed() <= Duration::from_millis(250)).then_some(msg))
    }
    pub fn take_actions(&self) -> u8 {
        self.actions.swap(0, Ordering::AcqRel)
    }
    pub fn set_debug(&self, debug: DebugDraw) {
        if debug.validate()
            && let Ok(mut d) = self.debug.lock()
        {
            *d = debug;
        }
    }
    pub fn set_scene_metadata(&self, scene: SceneMetadata) {
        if scene.validate()
            && let Ok(mut s) = self.scene.lock()
        {
            *s = Some(scene);
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
        // Latest-only overwrite via try_lock: drops the tick rather than
        // stalling the sim when the sender holds the slot.
        if let Ok(mut slot) = self.slot.try_lock() {
            let mut frame = VisFrame::new(arena_state.clone(), dt);
            if let Ok(d) = self.debug.lock() {
                frame.debug = d.clone();
            }

            *slot = Some(frame);
        }
    }
}

/// Viewer→sim control channel, sim side.
/// The SIM listens here (default `127.0.0.1:39202`); the viewer dials lazily
/// and streams [`ControlMsg`] frames. One background thread accepts and
/// keeps only the LATEST message — poll each tick with
/// [`ControlServer::latest`] (level state: same message until a newer one
/// arrives, so failsafe to neutral on age). Construction never blocks,
/// fails, or panics (the bind itself retries on the worker thread).
/// Drop stops the thread.
///
/// ```rust
/// use std::{
///     io::Write,
///     net::TcpStream,
///     time::{Duration, Instant},
/// };
///
/// use rocketsim::CarControls;
/// use rocketsim_vis::{
///     net_vis::ControlServer,
///     proto::{ControlMsg, encode_control},
/// };
///
/// // Sim side: bind (port 0 picks a free port; read it back).
/// let server = ControlServer::bind("127.0.0.1:0");
/// let deadline = Instant::now() + Duration::from_secs(10);
/// let addr = loop {
///     if let Some(addr) = server.local_addr() {
///         break addr;
///     }
///     assert!(Instant::now() < deadline, "listener never bound");
///     std::thread::sleep(Duration::from_millis(5));
/// };
///
/// // Viewer side: dial in and send one input.
/// let mut stream = TcpStream::connect(addr).unwrap();
/// stream
///     .write_all(&encode_control(&ControlMsg::new(
///         0,
///         CarControls::default(),
///         0b001,
///         7,
///     )))
///     .unwrap();
///
/// // Sim poll loop: the newest message arrives.
/// let msg = loop {
///     if let Some(msg) = server.latest() {
///         break msg;
///     }
///     assert!(Instant::now() < deadline, "control never arrived");
///     std::thread::sleep(Duration::from_millis(5));
/// };
/// assert_eq!(msg.seq, 7);
/// assert_eq!(msg.buttons, 0b001);
/// ```
pub struct ControlServer {
    slot: Arc<Mutex<Option<ControlMsg>>>,
    /// Total control messages decoded (overlay diagnostics).
    received: Arc<Mutex<u64>>,
    /// Bound address once the worker's bind succeeds (`None` while
    /// retrying — e.g. the port is still busy).
    local_addr: Arc<Mutex<Option<SocketAddr>>>,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl ControlServer {
    /// Starts the background listener on `addr` (e.g. `"127.0.0.1:39202"`).
    /// Returns immediately; while no viewer is connected [`ControlServer::latest`]
    /// is `None` (drive with neutral inputs until then).
    pub fn bind(addr: &str) -> Self {
        let slot = Arc::new(Mutex::new(None));
        let received = Arc::new(Mutex::new(0));
        let local_addr = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let worker = {
            let (slot, received, local_addr, stop, addr) = (
                slot.clone(),
                received.clone(),
                local_addr.clone(),
                stop.clone(),
                addr.to_string(),
            );
            std::thread::spawn(move || {
                control_accept_loop(&addr, &slot, &received, &local_addr, &stop)
            })
        };
        Self {
            slot,
            received,
            local_addr,
            stop,
            worker: Some(worker),
        }
    }

    /// Clones the newest control message, or `None` if none arrived yet.
    /// Lock-only, never blocks on the network.
    pub fn latest(&self) -> Option<ControlMsg> {
        self.slot.lock().ok().and_then(|g| *g)
    }

    /// Bound address, once the worker bind succeeds (`None` while retrying).
    pub fn local_addr(&self) -> Option<SocketAddr> {
        self.local_addr.lock().ok().and_then(|g| *g)
    }

    /// Total control messages decoded so far.
    pub fn received(&self) -> u64 {
        self.received.lock().map(|g| *g).unwrap_or(0)
    }

    /// Stops the listener. Called automatically on drop.
    pub fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.worker.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for ControlServer {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Binds (with backoff — the port may still be held by a previous sim)
/// then serves one viewer at a time, keeping only the newest message.
fn control_accept_loop(
    addr: &str,
    slot: &Mutex<Option<ControlMsg>>,
    received: &Mutex<u64>,
    local_addr: &Mutex<Option<SocketAddr>>,
    stop: &AtomicBool,
) {
    let mut backoff = Duration::from_millis(200);
    let max_backoff = Duration::from_secs(5);
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        match TcpListener::bind(addr) {
            Ok(listener) => {
                backoff = Duration::from_millis(200);
                if let Ok(bound) = listener.local_addr() {
                    *local_addr.lock().unwrap() = Some(bound);
                }
                listener
                    .set_nonblocking(true)
                    .expect("control listener: nonblocking failed");
                control_serve(listener, slot, received, stop);
                // serve returned = stop set (listener closed on drop);
                // a lost viewer just ends its pump inside serve.
                *local_addr.lock().unwrap() = None;
            }
            Err(_) => {
                sleep_interruptible(stop, backoff);
                backoff = (backoff * 2).min(max_backoff);
                continue;
            }
        }
    }
}

/// Accepts viewers sequentially; each connection pumps newest-only until
/// EOF/error/stop, then the loop takes the next viewer.
fn control_serve(
    listener: TcpListener,
    slot: &Mutex<Option<ControlMsg>>,
    received: &Mutex<u64>,
    stop: &AtomicBool,
) {
    while !stop.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, _)) => {
                control_pump(stream, slot, received, stop);
                if let Ok(mut latest) = slot.lock() {
                    *latest = None;
                }
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

/// Reads length-prefixed control frames until EOF/error/stop, keeping the
/// newest (overwrite — stale inputs drop by construction).
fn control_pump(
    mut stream: TcpStream,
    slot: &Mutex<Option<ControlMsg>>,
    received: &Mutex<u64>,
    stop: &AtomicBool,
) {
    let _ = stream.set_nodelay(true);
    let _ = stream.set_read_timeout(Some(Duration::from_millis(200)));
    let mut buf = Vec::new();
    let mut chunk = [0u8; 4096];
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        match stream.read(&mut chunk) {
            Ok(0) => return, // viewer closed
            Ok(n) => {
                buf.extend_from_slice(&chunk[..n]);
                loop {
                    match decode_control(&buf) {
                        Ok(Some((decoded, consumed))) => {
                            if let Ok(mut s) = slot.lock() {
                                *s = Some(decoded);
                            }
                            if let Ok(mut count) = received.lock() {
                                *count += 1;
                            }
                            buf.drain(..consumed);
                        }
                        // Corrupt framing (e.g. version mismatch): drop the
                        // connection so both ends reconnect cleanly instead
                        // of wedging on garbage bytes forever.
                        Err(_) => return,
                        Ok(None) => break,
                    }
                }
                if buf.len() > crate::proto::MAX_CONTROL_BYTES + 4 {
                    return;
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

struct SenderState {
    slot: Arc<Mutex<Option<VisFrame>>>,
    stop: Arc<AtomicBool>,
    controls: Arc<Mutex<Option<(ControlMsg, std::time::Instant)>>>,
    actions: Arc<AtomicU8>,
    connected: Arc<AtomicBool>,
    scene: Arc<Mutex<Option<SceneMetadata>>>,
}

fn sender_loop(addr: &str, shared: SenderState) {
    let mut backoff = Duration::from_millis(500);
    let max_backoff = Duration::from_secs(5);
    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return;
        }
        match connect_bounded(addr) {
            Ok(stream) => {
                backoff = Duration::from_millis(500);
                let _ = stream.set_nodelay(true);
                shared.connected.store(true, Ordering::Release);
                pump(stream, &shared);
                shared.connected.store(false, Ordering::Release);
                if let Ok(mut c) = shared.controls.lock() {
                    *c = None;
                }
                shared.actions.store(0, Ordering::Release);
                // pump returned = connection lost; reconnect with backoff.
            }
            Err(_) => {
                sleep_interruptible(&shared.stop, backoff);
                backoff = (backoff * 2).min(max_backoff);
                continue;
            }
        }
    }
}

fn connect_bounded(addr: &str) -> std::io::Result<TcpStream> {
    let socket_addr: SocketAddr = addr.parse().map_err(|e| {
        std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!("address must be a numeric socket address: {e}"),
        )
    })?;
    TcpStream::connect_timeout(&socket_addr, Duration::from_millis(250))
}
fn sleep_interruptible(stop: &AtomicBool, d: Duration) {
    let end = std::time::Instant::now() + d;
    while !stop.load(Ordering::Acquire) && std::time::Instant::now() < end {
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// Sends latest frame and receives controls on the same accepted stream.
fn pump(mut stream: TcpStream, shared: &SenderState) {
    if stream.set_nonblocking(true).is_err() {
        return;
    }
    let _ = stream.set_write_timeout(Some(Duration::from_millis(250)));
    let peer = stream
        .peer_addr()
        .map(|a| a.to_string())
        .unwrap_or_default();
    let mut input = Vec::new();
    let mut chunk = [0u8; 2048];
    let mut metadata_sent = false;
    let mut queued_scene = false;
    let mut output = OutgoingBuffer::default();
    loop {
        if shared.stop.load(Ordering::Acquire) {
            let _ = stream.shutdown(std::net::Shutdown::Both);
            return;
        }
        if !output.is_empty() {
            match output.flush(&mut stream) {
                Ok(true) => {
                    metadata_sent |= queued_scene;
                    queued_scene = false;
                }
                Ok(false) => {}
                Err(_) => return,
            }
        }
        match stream.read(&mut chunk) {
            Ok(0) => return,
            Ok(n) => {
                input.extend_from_slice(&chunk[..n]);
                loop {
                    match decode_control(&input) {
                        Ok(Some((mut msg, used))) => {
                            shared.actions.fetch_or(msg.buttons & 7, Ordering::AcqRel);
                            msg.buttons = 0;
                            if let Ok(mut c) = shared.controls.lock() {
                                *c = Some((msg, std::time::Instant::now()));
                            }
                            input.drain(..used);
                        }
                        Ok(None) => break,
                        Err(error) => {
                            eprintln!(
                                "rocketsim_vis: closing invalid control stream from {peer}: {error}"
                            );
                            return;
                        }
                    }
                }
                if input.len() > crate::proto::MAX_CONTROL_BYTES + 4 {
                    return;
                }
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(_) => return,
        }
        let frame = if output.is_empty() {
            shared.slot.lock().map(|mut s| s.take()).unwrap_or(None)
        } else {
            None
        };
        match frame {
            Some(mut frame) => {
                if metadata_sent {
                    frame.scene = None;
                } else if let Ok(s) = shared.scene.lock()
                    && let Some(meta) = s.as_ref()
                {
                    frame.scene = Some(meta.clone());
                }
                queued_scene = frame.scene.is_some();
                let bytes = encode_frame(&frame);
                if bytes.len() > crate::proto::MAX_FRAME_BYTES + 4 {
                    return;
                }
                output.queue(bytes);
            }
            None => std::thread::sleep(Duration::from_millis(1)),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{
        io::{Read, Write},
        net::TcpListener,
        time::Instant,
    };

    use super::*;
    use crate::proto::{ControlMsg, decode_frame, encode_control};

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
    fn same_stream_controls_preserve_action_pulses() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let vis = NetVis::connect(&addr);
        let (mut stream, _) = listener.accept().unwrap();
        let first = encode_control(&ControlMsg::new(0, rocketsim::CarControls::default(), 1, 1));
        let second = encode_control(&ControlMsg::new(0, rocketsim::CarControls::default(), 4, 2));
        stream.write_all(&first).unwrap();
        stream.write_all(&second).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while vis.latest_controls().is_none_or(|m| m.seq < 2) {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(vis.take_actions(), 5);
        assert_eq!(vis.take_actions(), 0);
        assert!(vis.connected());
        drop(listener);
        drop(stream);
        let deadline = Instant::now() + Duration::from_secs(2);
        while vis.connected() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        assert!(vis.latest_controls().is_none());
        assert_eq!(vis.take_actions(), 0);
    }

    #[test]
    fn two_netvis_sessions_keep_controls_and_actions_isolated() {
        let listener_a = TcpListener::bind("127.0.0.1:0").unwrap();
        let listener_b = TcpListener::bind("127.0.0.1:0").unwrap();
        let vis_a = NetVis::connect(&listener_a.local_addr().unwrap().to_string());
        let vis_b = NetVis::connect(&listener_b.local_addr().unwrap().to_string());
        let (mut stream_a, _) = listener_a.accept().unwrap();
        let (mut stream_b, _) = listener_b.accept().unwrap();
        stream_a
            .write_all(&encode_control(&ControlMsg::new(
                0,
                rocketsim::CarControls::default(),
                1,
                11,
            )))
            .unwrap();
        stream_b
            .write_all(&encode_control(&ControlMsg::new(
                0,
                rocketsim::CarControls::default(),
                4,
                22,
            )))
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while vis_a.latest_controls().is_none_or(|m| m.seq != 11)
            || vis_b.latest_controls().is_none_or(|m| m.seq != 22)
        {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(vis_a.take_actions(), 1);
        assert_eq!(vis_b.take_actions(), 4);
        assert_eq!(vis_a.latest_controls().unwrap().seq, 11);
        assert_eq!(vis_b.latest_controls().unwrap().seq, 22);
    }

    #[test]
    fn loopback_delivers_latest_frame() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let mut vis = NetVis::connect(&addr);
        // Server owns the read end; main thread must NOT drop `vis` first
        // (that would stop the sender before it pumps).
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
        // Port 1 never accepts: updates stay a clone + try_lock.
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

    /// Waits until `server.latest()` reports `want_seq` (or `deadline`
    /// passes); returns the newest seq seen.
    fn wait_for_seq(server: &ControlServer, want_seq: u64, deadline: Instant) -> Option<u64> {
        let mut last = None;
        loop {
            if let Some(msg) = server.latest() {
                last = Some(msg.seq);
                if last == Some(want_seq) {
                    return last;
                }
            }
            if Instant::now() > deadline {
                return last;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn control_server_converges_to_newest() {
        // Bind on port 0; the worker reports the real port via local_addr.
        let server = ControlServer::bind("127.0.0.1:0");
        let deadline = Instant::now() + Duration::from_secs(10);
        let addr = loop {
            if let Some(addr) = server.local_addr() {
                break addr;
            }
            assert!(Instant::now() < deadline, "control listener never bound");
            std::thread::sleep(Duration::from_millis(5));
        };

        // Fake viewer: 50 inputs back-to-back, then close. Server keeps
        // the newest (seq 50), dropping stale.
        let mut stream = TcpStream::connect(addr).unwrap();
        for seq in 1..=50u64 {
            let msg = ControlMsg::new(
                0,
                rocketsim::CarControls::default().with_throttle(seq as f32 / 50.0),
                0,
                seq,
            );
            stream
                .write_all(&crate::proto::encode_control(&msg))
                .unwrap();
        }
        let last = wait_for_seq(&server, 50, Instant::now() + Duration::from_secs(10));
        assert_eq!(last, Some(50), "must converge to newest control");
        drop(stream);
        assert!(server.received() >= 50);
        let msg = server.latest().unwrap();
        assert!((msg.controls.throttle - 1.0).abs() < 1e-6);
    }

    #[test]
    fn control_poll_without_viewer_never_blocks() {
        // Nobody dials: polls are fast lock-only reads.
        let server = ControlServer::bind("127.0.0.1:0");
        let t = Instant::now();
        for _ in 0..20_000 {
            let _ = server.latest();
        }
        assert!(
            t.elapsed() < Duration::from_secs(2),
            "control poll blocked: {:?}",
            t.elapsed()
        );
        assert!(server.latest().is_none());
        assert_eq!(server.received(), 0);
    }
}
