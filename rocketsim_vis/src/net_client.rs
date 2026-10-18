//! Viewer-side TCP frame server.
//! The viewer LISTENS; sims dial in. One background thread accepts and
//! keeps only the LATEST frame (no queue: a slow viewer drops frames,
//! never lags). On sim disconnect the loop takes the next sim; the viewer
//! keeps showing the last frame meanwhile.

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

use crate::proto::{ControlMsg, VisFrame, decode_sim, encode_control};

/// Viewer→sim control sender, viewer side.
/// Dials the sim's control port lazily and streams [`ControlMsg`] frames.
/// [`ControlSender::send`] overwrites any unsent message (latest-only), so
/// it never blocks the viewer; with no sim listening the thread retries
/// with backoff and inputs are dropped. Drop stops the thread.
/// Construction never blocks, fails, or panics.
///
/// ```rust
/// use rocketsim::CarControls;
/// use rocketsim_vis::{net_client::ControlSender, proto::ControlMsg};
///
/// // Port 1 never accepts: sends are dropped, never block.
/// let sender = ControlSender::connect("127.0.0.1:1");
/// sender.send(ControlMsg::new(0, CarControls::default(), 0, 1));
/// assert!(!sender.connected());
/// ```
pub struct ControlSender {
    slot: Arc<Mutex<Option<ControlMsg>>>,
    stop: Arc<AtomicBool>,
    /// Set while the worker holds a live sim connection (overlay status).
    connected: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl ControlSender {
    /// Starts the background dialer for the sim's control port (e.g.
    /// `"127.0.0.1:39202"`). Use a numeric socket address; DNS lookup cannot be cancelled.
    /// Never blocks or fails: with no sim listening
    /// the thread retries with backoff and inputs are dropped, never queued.
    pub fn connect(addr: &str) -> Self {
        let slot = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let connected = Arc::new(AtomicBool::new(false));
        let worker = {
            let (slot, stop, connected, addr) = (
                slot.clone(),
                stop.clone(),
                connected.clone(),
                addr.to_string(),
            );
            std::thread::spawn(move || control_sender_loop(&addr, &slot, &stop, &connected))
        };
        Self {
            slot,
            stop,
            connected,
            worker: Some(worker),
        }
    }

    /// Queues `msg` for sending (overwrites any unsent older message).
    /// Lock-only via `try_lock`; never blocks, never panics.
    pub fn send(&self, msg: ControlMsg) {
        if let Ok(mut slot) = self.slot.try_lock() {
            *slot = Some(msg);
        }
    }

    /// True while the worker holds a live sim connection. False means
    /// inputs are dropped (no sim listening).
    pub fn connected(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
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

fn control_sender_loop(
    addr: &str,
    slot: &Mutex<Option<ControlMsg>>,
    stop: &AtomicBool,
    connected: &AtomicBool,
) {
    let mut backoff = Duration::from_millis(500);
    let max_backoff = Duration::from_secs(5);
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        match connect_bounded(addr) {
            Ok(stream) => {
                backoff = Duration::from_millis(500);
                let _ = stream.set_nodelay(true);
                connected.store(true, Ordering::Relaxed);
                control_sender_pump(stream, slot, stop);
                connected.store(false, Ordering::Relaxed);
                // pump returned = connection lost; reconnect with backoff.
            }
            Err(_) => {
                sleep_interruptible(stop, backoff);
                backoff = (backoff * 2).min(max_backoff);
                continue;
            }
        }
    }
}

/// Sends newest-first until the connection breaks or `stop` is set.
fn control_sender_pump(mut stream: TcpStream, slot: &Mutex<Option<ControlMsg>>, stop: &AtomicBool) {
    let _ = stream.set_write_timeout(Some(Duration::from_millis(200)));
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
/// [`FrameClient::latest`] (clones the newest `VisFrame`, if any) and
/// [`FrameClient::debug_scene`] (cheap `Arc` of retained groups + assets,
/// independent of the overwritten physics slot).
#[derive(Clone)]
struct ClientSession {
    generation: u64,
    status: ConnStatus,
    frame: Option<VisFrame>,
    scene: Option<Arc<crate::proto::SceneMetadata>>,
    debug: Arc<crate::debug::DebugScene>,

    /// Debug transfers applied this session (diagnostics).
    debug_transfers: u64,
    controls: Option<ControlMsg>,
    pending_buttons: u8,
}

impl ClientSession {
    fn reset(&mut self, status: ConnStatus) {
        self.generation = self.generation.wrapping_add(1);
        self.status = status;
        self.frame = None;
        self.scene = None;
        self.debug = Arc::new(crate::debug::DebugScene::empty());
        self.debug_transfers = 0;
        self.controls = None;
        self.pending_buttons = 0;
    }
}

fn display_debug_scene(session: &ClientSession) -> Arc<crate::debug::DebugScene> {
    let now_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64;
    let clock = session.frame.as_ref().map_or(0, |frame| frame.debug_tick);
    let has_snapshot = session.frame.is_some();
    let groups = session
        .debug
        .groups
        .iter()
        .filter(|(_, group)| match group.lifetime {
            crate::debug::Lifetime::Persistent => true,
            crate::debug::Lifetime::Frame => {
                has_snapshot
                    && group.expires_tick.is_some_and(|deadline| {
                        clock >= deadline.saturating_sub(1) && clock < deadline
                    })
            }
            crate::debug::Lifetime::Ticks(_) => {
                group.expires_tick.is_none_or(|deadline| clock < deadline)
            }
            crate::debug::Lifetime::Seconds(_) => {
                group.expires_at_ms.is_none_or(|deadline| now_ms < deadline)
            }
        })
        .map(|(name, group)| (name.clone(), group.clone()))
        .collect();
    Arc::new(crate::debug::DebugScene {
        groups,
        meshes: session.debug.meshes.clone(),
        textures: session.debug.textures.clone(),
        revision: session.debug.revision,
    })
}

pub struct FrameClient {
    session: Arc<Mutex<ClientSession>>,
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
        Self::try_connect(addr)
            .unwrap_or_else(|e| panic!("rocketsim-viewer: cannot listen on {addr} ({e})"))
    }

    pub fn try_connect(addr: &str) -> std::io::Result<Self> {
        let listener = TcpListener::bind(addr)?;
        listener.set_nonblocking(true)?;
        let local_addr = listener.local_addr()?;
        let session = Arc::new(Mutex::new(ClientSession {
            generation: 0,
            status: ConnStatus::Waiting,
            frame: None,
            scene: None,
            debug: Arc::new(crate::debug::DebugScene::empty()),

            debug_transfers: 0,
            controls: None,
            pending_buttons: 0,
        }));
        let received = Arc::new(Mutex::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let worker = {
            let (session, received, stop) = (session.clone(), received.clone(), stop.clone());
            std::thread::spawn(move || accept_loop(listener, &session, &received, &stop))
        };
        Ok(Self {
            session,
            received,
            local_addr,
            stop,
            worker: Some(worker),
        })
    }

    pub fn send_controls(&self, msg: ControlMsg) {
        let generation = self.generation();
        let _ = self.send_controls_for_generation(generation, msg);
    }

    /// Enqueue controls only if `generation` still names the active sim session.
    /// Returns false for stale generations or when no sim is connected.
    pub fn send_controls_for_generation(&self, generation: u64, mut msg: ControlMsg) -> bool {
        let Ok(mut session) = self.session.lock() else {
            return false;
        };
        if session.generation != generation
            || !matches!(session.status, ConnStatus::Connected { .. })
        {
            return false;
        }
        session.pending_buttons |= msg.buttons & 7;
        msg.buttons = 0;
        msg.generation = generation;
        session.controls = Some(msg);
        true
    }

    /// Return one coherent session generation, frame, and shared scene.
    /// The frame has no scene field; use the returned `Arc` for metadata.
    pub fn snapshot(
        &self,
    ) -> (
        u64,
        Option<VisFrame>,
        Option<Arc<crate::proto::SceneMetadata>>,
    ) {
        self.session
            .lock()
            .map(|s| (s.generation, s.frame.clone(), s.scene.clone()))
            .unwrap_or((0, None, None))
    }

    pub fn controls_connected(&self) -> bool {
        self.session
            .lock()
            .map(|s| matches!(s.status, ConnStatus::Connected { .. }))
            .unwrap_or(false)
    }
    pub fn generation(&self) -> u64 {
        self.session.lock().map(|s| s.generation).unwrap_or(0)
    }
    pub fn scene(&self) -> Option<Arc<crate::proto::SceneMetadata>> {
        self.session.lock().ok().and_then(|s| s.scene.clone())
    }
    /// Shared retained scene filtered for the latest displayed simulation snapshot.
    /// Canonical sender state remains unchanged by lifetime filtering.
    pub fn debug_scene(&self) -> Arc<crate::debug::DebugScene> {
        self.session
            .lock()
            .map(|s| display_debug_scene(&s))
            .unwrap_or_else(|_| Arc::new(crate::debug::DebugScene::empty()))
    }
    /// Generation plus display-filtered retained scene.
    pub fn debug_snapshot(&self) -> (u64, Arc<crate::debug::DebugScene>) {
        self.session
            .lock()
            .map(|s| (s.generation, display_debug_scene(&s)))
            .unwrap_or_else(|_| (0, Arc::new(crate::debug::DebugScene::empty())))
    }
    /// Current retained-debug revision (sender-global; changes on any
    /// publication). Useful for GPU cache invalidation.
    pub fn debug_revision(&self) -> u64 {
        self.session.lock().map(|s| s.debug.revision).unwrap_or(0)
    }
    /// Debug transfers applied this session (diagnostics).
    pub fn debug_transfers(&self) -> u64 {
        self.session.lock().map(|s| s.debug_transfers).unwrap_or(0)
    }

    /// Address actually bound (useful with port 0 in tests).
    pub fn local_addr(&self) -> SocketAddr {
        self.local_addr
    }

    /// Clones the newest frame, or `None` if no sim has sent one yet.
    pub fn latest(&self) -> Option<VisFrame> {
        self.session.lock().ok().and_then(|s| s.frame.clone())
    }

    /// Current connection state (for the overlay).
    pub fn status(&self) -> ConnStatus {
        self.session
            .lock()
            .map(|s| s.status.clone())
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
    session: &Mutex<ClientSession>,
    received: &Mutex<u64>,
    stop: &AtomicBool,
) {
    while !stop.load(Ordering::Acquire) {
        match listener.accept() {
            Ok((stream, _)) => {
                let active = Arc::new(AtomicBool::new(true));
                let extra_active = active.clone();
                let extra_listener = match listener.try_clone() {
                    Ok(l) => l,
                    Err(_) => continue,
                };
                let extra_worker = std::thread::spawn(move || {
                    while extra_active.load(Ordering::Acquire) {
                        match extra_listener.accept() {
                            Ok((extra, _)) => drop(extra),
                            Err(_) => std::thread::sleep(Duration::from_millis(10)),
                        }
                    }
                });
                let peer = stream
                    .peer_addr()
                    .map(|a| a.to_string())
                    .unwrap_or_default();
                if let Ok(mut current) = session.lock() {
                    current.reset(ConnStatus::Connected { addr: peer.clone() });
                }
                pump(stream, session, received, stop, &peer);
                active.store(false, Ordering::Release);
                let _ = extra_worker.join();
                if let Ok(mut current) = session.lock() {
                    current.reset(ConnStatus::Waiting);
                }
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                std::thread::sleep(Duration::from_millis(20))
            }
            Err(_) => std::thread::sleep(Duration::from_millis(50)),
        }
    }
}

#[derive(Default)]
pub(crate) struct OutgoingBuffer {
    bytes: Vec<u8>,
    offset: usize,
    last_progress: Option<std::time::Instant>,
}
impl OutgoingBuffer {
    pub(crate) fn is_empty(&self) -> bool {
        self.offset == self.bytes.len()
    }
    pub(crate) fn queue(&mut self, bytes: Vec<u8>) {
        debug_assert!(self.is_empty());
        self.bytes = bytes;
        self.offset = 0;
        self.last_progress = Some(std::time::Instant::now());
    }
    pub(crate) fn flush<W: Write>(&mut self, writer: &mut W) -> std::io::Result<bool> {
        while !self.is_empty() {
            match writer.write(&self.bytes[self.offset..]) {
                Ok(0) => return Err(std::io::ErrorKind::WriteZero.into()),
                Ok(n) => {
                    self.offset += n;
                    self.last_progress = Some(std::time::Instant::now());
                }
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => return Ok(false),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    if self
                        .last_progress
                        .is_some_and(|at| at.elapsed() >= Duration::from_millis(250))
                    {
                        return Err(std::io::ErrorKind::TimedOut.into());
                    }
                    return Ok(false);
                }
                Err(e) => return Err(e),
            }
        }
        self.bytes.clear();
        self.offset = 0;
        self.last_progress = None;
        Ok(true)
    }
}

/// One in-progress debug transfer (per connection; frames interleave).
struct DebugReassembly {
    transfer_id: u64,
    total: usize,
    parts: Vec<Option<Vec<u8>>>,
    received: usize,
}

/// Apply a validated `Full` atomically (fresh session state).
fn apply_debug_full(session: &Mutex<ClientSession>, full: crate::proto::DebugFull) -> bool {
    if let Ok(mut s) = session.lock() {
        let scene = crate::debug::DebugScene {
            groups: full
                .groups
                .into_iter()
                .map(|g| (g.name.clone(), Arc::new(g)))
                .collect(),
            meshes: full
                .meshes
                .into_iter()
                .map(|m| (m.id.clone(), Arc::new(m)))
                .collect(),
            textures: full
                .textures
                .into_iter()
                .map(|t| (t.id.clone(), Arc::new(t)))
                .collect(),
            revision: full.rev.max(1),
        };
        if scene.validate().is_err() {
            return false;
        }

        s.debug = Arc::new(scene);
        s.debug_transfers = s.debug_transfers.wrapping_add(1);
        true
    } else {
        false
    }
}

/// Apply a validated `Delta` atomically (ordered stream). Invalid results
/// (over budget after merge) are dropped without publishing.
fn apply_debug_delta(session: &Mutex<ClientSession>, delta: crate::proto::DebugDelta) -> bool {
    if let Ok(mut s) = session.lock() {
        let mut next = (*s.debug).clone();
        for g in delta.upsert_groups {
            next.groups.insert(g.name.clone(), Arc::new(g));
        }
        for patch in delta.group_patches {
            let Some(existing) = next.groups.get(&patch.name) else {
                return false;
            };
            let mut group = (**existing).clone();
            group.visible = patch.visible;
            group.anchor = patch.anchor;
            group.transform = patch.transform;
            group.lifetime = patch.lifetime;
            group.expires_tick = patch.expires_tick;
            group.expires_at_ms = patch.expires_at_ms;
            group.revision = patch.revision;
            next.groups.insert(patch.name, Arc::new(group));
        }
        for name in delta.removed_groups {
            next.groups.remove(&name);
        }
        for m in delta.upsert_meshes {
            next.meshes.insert(m.id.clone(), Arc::new(m));
        }
        for id in delta.removed_meshes {
            next.meshes.remove(&id);
        }
        for t in delta.upsert_textures {
            next.textures.insert(t.id.clone(), Arc::new(t));
        }
        for id in delta.removed_textures {
            next.textures.remove(&id);
        }
        next.revision = delta.rev.max(1);
        if next.validate().is_ok() {
            s.debug = Arc::new(next);
            s.debug_transfers = s.debug_transfers.wrapping_add(1);
            true
        } else {
            false
        }
        // Do not accept later deltas against a state that missed this one.
    } else {
        false
    }
}

fn pump(
    mut stream: TcpStream,
    session: &Mutex<ClientSession>,
    received: &Mutex<u64>,
    stop: &AtomicBool,
    peer: &str,
) {
    if stream.set_nonblocking(true).is_err() {
        return;
    }
    let mut input = Vec::new();
    let mut chunk = [0u8; 65536];
    let mut output = OutgoingBuffer::default();
    let mut reassembly: Option<DebugReassembly> = None;
    loop {
        if stop.load(Ordering::Acquire) {
            let _ = stream.shutdown(std::net::Shutdown::Both);
            return;
        }
        if !output.is_empty() && output.flush(&mut stream).is_err() {
            return;
        }
        if output.is_empty() {
            let next = session.lock().ok().and_then(|mut s| {
                s.controls.take().map(|mut msg| {
                    msg.generation = s.generation;
                    msg.buttons = s.pending_buttons;
                    s.pending_buttons = 0;
                    msg
                })
            });
            if let Some(msg) = next {
                output.queue(crate::proto::encode_control(&msg));
                continue;
            }
        }
        match stream.read(&mut chunk) {
            Ok(0) => return,
            Ok(n) => {
                input.extend_from_slice(&chunk[..n]);
                loop {
                    match decode_sim(&input) {
                        Ok(Some((msg, consumed))) => {
                            match msg {
                                crate::proto::SimMsg::Frame(mut frame) => {
                                    let metadata = frame.scene.take();
                                    if let Ok(mut s) = session.lock() {
                                        if let Some(metadata) = metadata {
                                            s.scene = Some(Arc::new(metadata));
                                        }
                                        s.frame = Some(frame);
                                    }
                                    if let Ok(mut count) = received.lock() {
                                        *count += 1;
                                    }
                                }
                                crate::proto::SimMsg::DebugChunk(c) => {
                                    let total = c.total as usize;
                                    let index = c.index as usize;
                                    let fresh = reassembly
                                        .as_ref()
                                        .is_none_or(|r| r.transfer_id != c.transfer_id);
                                    if fresh {
                                        if total == 0
                                            || total > crate::proto::MAX_CHUNKS_PER_TRANSFER
                                        {
                                            eprintln!(
                                                "rocketsim-viewer: closing invalid debug stream from {peer}: bad total"
                                            );
                                            return;
                                        }
                                        reassembly = Some(DebugReassembly {
                                            transfer_id: c.transfer_id,
                                            total,
                                            parts: (0..total).map(|_| None).collect(),
                                            received: 0,
                                        });
                                    }
                                    let done = if let Some(r) = reassembly.as_mut() {
                                        if r.total != total || index >= r.total {
                                            eprintln!(
                                                "rocketsim-viewer: closing invalid debug stream from {peer}: bad index"
                                            );
                                            return;
                                        }
                                        if r.parts[index].is_none() {
                                            r.parts[index] = Some(c.bytes);
                                            r.received += 1;
                                        }
                                        r.received == r.total
                                    } else {
                                        false
                                    };
                                    if done {
                                        let r = reassembly.take().unwrap();
                                        let mut bytes = Vec::new();
                                        for part in r.parts.into_iter().flatten() {
                                            bytes.extend_from_slice(&part);
                                        }
                                        match crate::proto::decode_debug_sync(&bytes) {
                                            Ok(crate::proto::DebugSyncMsg::Full(full)) => {
                                                if !apply_debug_full(session, full) {
                                                    return;
                                                }
                                            }
                                            Ok(crate::proto::DebugSyncMsg::Delta(delta)) => {
                                                if !apply_debug_delta(session, delta) {
                                                    return;
                                                }
                                            }
                                            Err(error) => {
                                                eprintln!(
                                                    "rocketsim-viewer: closing invalid debug stream from {peer}: {error}"
                                                );
                                                return;
                                            }
                                        }
                                    }
                                }
                            }
                            input.drain(..consumed);
                        }
                        Ok(None) => break,
                        Err(error) => {
                            eprintln!(
                                "rocketsim-viewer: closing invalid frame stream from {peer}: {error}"
                            );
                            return;
                        }
                    }
                }
                if input.len() > crate::proto::MAX_SIM_MSG_BYTES + 4 {
                    return;
                }
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(_) => return,
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

#[cfg(test)]
mod tests {
    use std::{io::Write, time::Instant};

    use super::*;
    use crate::proto::{ControlMsg, SimMsg, decode_control, encode_sim};

    struct ScriptedWriter {
        steps: Vec<std::io::Result<usize>>,
        bytes: Vec<u8>,
    }
    impl Write for ScriptedWriter {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            let next = self.steps.remove(0)?;
            if next == 0 {
                return Ok(0);
            }
            let n = next.min(buf.len());
            self.bytes.extend_from_slice(&buf[..n]);
            Ok(n)
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn display_expiry_does_not_remove_canonical_groups_or_block_late_patches() {
        use crate::debug::{DebugGroup, DrawList, GroupOptions, Lifetime};
        crate::photo::init_meshes();
        let arena = crate::photo::build_arena();
        let state = arena.get_arena_state();
        let mut group = DebugGroup::new(
            "frame",
            DrawList::new(),
            GroupOptions::default().lifetime(Lifetime::Frame),
        )
        .unwrap();
        group.expires_tick = Some(12);
        let mut scene = crate::debug::DebugScene::empty();
        scene.groups.insert("frame".into(), Arc::new(group));
        let session = Mutex::new(ClientSession {
            generation: 1,
            status: ConnStatus::Connected {
                addr: "local".into(),
            },
            frame: None,
            scene: None,
            debug: Arc::new(scene),
            debug_transfers: 0,
            controls: None,
            pending_buttons: 0,
        });
        assert!(
            display_debug_scene(&session.lock().unwrap())
                .groups
                .is_empty()
        );
        {
            let mut s = session.lock().unwrap();
            let mut frame = VisFrame::new(state, 1.0 / 120.0);
            frame.debug_tick = 11;
            s.frame = Some(frame);
        }
        assert!(
            display_debug_scene(&session.lock().unwrap())
                .groups
                .contains_key("frame")
        );
        std::thread::sleep(Duration::from_millis(275));
        assert!(
            display_debug_scene(&session.lock().unwrap())
                .groups
                .contains_key("frame")
        );
        {
            let mut s = session.lock().unwrap();
            s.frame.as_mut().unwrap().debug_tick = 12;
        }
        assert!(
            display_debug_scene(&session.lock().unwrap())
                .groups
                .is_empty()
        );
        assert!(session.lock().unwrap().debug.groups.contains_key("frame"));
        let patch = crate::proto::DebugGroupPatch {
            name: "frame".into(),
            visible: true,
            anchor: crate::debug::Anchor::World,
            transform: crate::debug::GroupTransform::identity(),
            lifetime: Lifetime::Frame,
            expires_tick: Some(12),
            expires_at_ms: None,
            revision: 2,
        };
        let delta = crate::proto::DebugDelta {
            group_patches: vec![patch],
            rev: 2,
            ..Default::default()
        };
        assert!(apply_debug_delta(&session, delta));
        let before = session.lock().unwrap().debug.revision;
        let bad = crate::proto::DebugDelta {
            group_patches: vec![crate::proto::DebugGroupPatch {
                name: "missing".into(),
                visible: true,
                anchor: crate::debug::Anchor::World,
                transform: crate::debug::GroupTransform::identity(),
                lifetime: Lifetime::Frame,
                expires_tick: Some(2),
                expires_at_ms: None,
                revision: 3,
            }],
            rev: 3,
            ..Default::default()
        };
        assert!(!apply_debug_delta(&session, bad));
        assert_eq!(session.lock().unwrap().debug.revision, before);
    }

    #[test]
    fn outgoing_buffer_resumes_short_writes_after_would_block() {
        let mut buffer = OutgoingBuffer::default();
        buffer.queue(b"complete-frame".to_vec());
        let mut writer = ScriptedWriter {
            steps: vec![Ok(3), Err(std::io::ErrorKind::WouldBlock.into()), Ok(4096)],
            bytes: Vec::new(),
        };
        assert!(!buffer.flush(&mut writer).unwrap());
        assert_eq!(buffer.offset, 3);
        assert!(buffer.flush(&mut writer).unwrap());
        assert_eq!(writer.bytes, b"complete-frame");
        assert!(buffer.is_empty());
    }

    #[test]
    fn outgoing_buffer_closes_after_a_stalled_write_deadline() {
        let mut buffer = OutgoingBuffer::default();
        buffer.queue(vec![1, 2, 3]);
        buffer.last_progress = Some(Instant::now() - Duration::from_millis(300));
        let mut writer = ScriptedWriter {
            steps: vec![Err(std::io::ErrorKind::WouldBlock.into())],
            bytes: Vec::new(),
        };
        assert_eq!(
            buffer.flush(&mut writer).unwrap_err().kind(),
            std::io::ErrorKind::TimedOut
        );
    }

    #[test]
    fn server_stores_latest_frame() {
        crate::photo::init_meshes();
        let client = FrameClient::connect("127.0.0.1:0");
        let addr = client.local_addr();

        // Sim side: three ticks back-to-back, then close.
        let mut stream = TcpStream::connect(addr).unwrap();
        let mut arena = crate::photo::build_arena();
        for _ in 0..3 {
            arena.step_tick();
            let bytes = encode_sim(&SimMsg::Frame(crate::proto::VisFrame::new(
                arena.get_arena_state(),
                1.0 / 120.0,
            )));
            stream.write_all(&bytes).unwrap();
        }

        // Tick 3 must arrive before disconnect clears this session's frame.
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
        drop(stream);
    }

    #[test]
    fn controls_share_the_accepted_frame_stream_and_retain_button_pulses() {
        let client = FrameClient::try_connect("127.0.0.1:0").unwrap();
        crate::photo::init_meshes();
        let mut stream = TcpStream::connect(client.local_addr()).unwrap();
        let arena = crate::photo::build_arena();
        let mut initial = crate::proto::VisFrame::new(arena.get_arena_state(), 1.0 / 120.0);
        initial.scene = Some(crate::proto::SceneMetadata::empty(
            rocketsim::GameMode::Soccar,
        ));
        stream
            .write_all(&encode_sim(&SimMsg::Frame(initial)))
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while !client.controls_connected()
            || client.snapshot().1.is_none()
            || client.scene().is_none()
        {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        let (generation, frame, snapshot_scene) = client.snapshot();
        assert!(frame.is_some());
        let scene = client.scene().unwrap();
        assert!(Arc::ptr_eq(&scene, &snapshot_scene.unwrap()));
        assert!(
            client.latest().unwrap().scene.is_none(),
            "scene must not be cloned into each latest frame"
        );
        assert!(client.send_controls_for_generation(
            generation,
            ControlMsg::new(0, rocketsim::CarControls::default(), 1, 1)
        ));
        assert!(client.send_controls_for_generation(
            generation,
            ControlMsg::new(0, rocketsim::CarControls::default(), 4, 2)
        ));
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let mut buf = [0u8; 1024];
        let n = stream.read(&mut buf).unwrap();
        let (msg, _) = decode_control(&buf[..n]).unwrap().unwrap();
        assert_eq!(msg.generation, generation);
        assert_eq!(msg.buttons, 5);
        drop(stream);
        let deadline = Instant::now() + Duration::from_secs(2);
        while client.controls_connected() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        let (disconnected_generation, stale_frame, stale_scene) = client.snapshot();
        assert!(disconnected_generation > generation);
        assert!(
            stale_frame.is_none() && stale_scene.is_none(),
            "disconnect clears session data atomically"
        );
        assert!(!client.send_controls_for_generation(
            generation,
            ControlMsg::new(0, rocketsim::CarControls::default(), 7, 3)
        ));
        let mut next_sim = TcpStream::connect(client.local_addr()).unwrap();
        next_sim
            .write_all(&encode_sim(&SimMsg::Frame(crate::proto::VisFrame::new(
                arena.get_arena_state(),
                1.0 / 120.0,
            ))))
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while client.generation() <= generation || client.latest().is_none() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        drop(next_sim);
    }

    #[test]
    fn competing_sim_connection_is_rejected_without_replacing_active_session() {
        crate::photo::init_meshes();
        let client = FrameClient::try_connect("127.0.0.1:0").unwrap();
        let mut owner = TcpStream::connect(client.local_addr()).unwrap();
        let arena = crate::photo::build_arena();
        owner
            .write_all(&encode_sim(&SimMsg::Frame(crate::proto::VisFrame::new(
                arena.get_arena_state(),
                1.0 / 120.0,
            ))))
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while !client.controls_connected() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        let mut extra = TcpStream::connect(client.local_addr()).unwrap();
        extra
            .set_read_timeout(Some(Duration::from_secs(1)))
            .unwrap();
        let mut byte = [0u8; 1];
        match extra.read(&mut byte) {
            Ok(0) | Err(_) => {}
            Ok(n) => panic!("competing connection received {n} bytes"),
        }
        assert!(client.controls_connected());
        assert_eq!(client.generation(), 1);
        drop(owner);
    }

    #[test]
    fn dropping_with_a_stalled_peer_is_bounded() {
        let mut client = FrameClient::try_connect("127.0.0.1:0").unwrap();
        let mut peer = TcpStream::connect(client.local_addr()).unwrap();
        peer.write_all(&[0x20, 0x00]).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        let start = Instant::now();
        client.shutdown();
        assert!(start.elapsed() < Duration::from_secs(1));
    }

    #[test]
    fn server_waits_for_sim() {
        // Bound but nobody dials: Waiting, latest None, no block or panic.
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
        // Fake sim listens; the sender dials lazily.
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let sender = ControlSender::connect(&addr);

        // 50 rapid sends: latest-only slot converges to seq 50 (drops fine).
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
        // Port 1 never accepts: sends must stay fast (dropped).
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
