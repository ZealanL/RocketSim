//! Accept simulation connections and retain their latest viewer state.
//!
//! A listener accepts one simulation at a time. It stores the latest frame,
//! canonical scene metadata, and retained debug scene. A slow viewer can miss
//! physics frames. Retained scene transfers are applied atomically. A new
//! connection starts a new generation and resets its session state.
//!
//! Use this module to build a viewer. Simulation applications normally use
//! [`crate::ArenaVisExt`] instead.
//!
//! # Receive frames and return controls on one connection
//!
//! Bind a listener, attach an arena to its address, and use the generation from
//! [`ViewerListener::snapshot`] when you send controls.
//!
//! ```
//! use std::{
//!     thread,
//!     time::{Duration, Instant},
//! };
//!
//! use rocketsim::{Arena, CarBodyConfig, CarControls, GameMode, Team};
//! use rocketsim_vis::{ArenaVisExt, proto::ControlMsg, transport::viewer::ViewerListener};
//!
//! let listener = ViewerListener::try_bind("127.0.0.1:0")?;
//! let mut arena = Arena::new(GameMode::TheVoid);
//! let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
//! arena.try_set_net_vis(Some(&listener.local_addr().to_string()))?;
//! let deadline = Instant::now() + Duration::from_secs(5);
//! let snapshot = loop {
//!     arena.step_tick();
//!     let snapshot = listener.snapshot();
//!     if snapshot.frame.is_some() {
//!         break snapshot;
//!     }
//!     assert!(Instant::now() < deadline, "no frame received");
//!     thread::sleep(Duration::from_millis(2));
//! };
//! let controls = CarControls::default().with_throttle(1.0);
//! assert!(listener.send_controls_for_generation(
//!     snapshot.generation,
//!     ControlMsg::new(car as u32, controls, 0, 1),
//! ));
//! # Ok::<(), std::io::Error>(())
//! ```
//!
//! The listener rejects extra connections while one simulation is active.
//! The generation changes on connect and disconnect. This protocol has no
//! authentication or encryption. Use non-loopback addresses only on trusted
//! networks.

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

use super::socket::OutgoingBuffer;
use crate::{
    debug::{DebugScene, Lifetime},
    proto::{
        ControlMsg, DebugSyncMsg, SceneMetadata, SimMsg, VisFrame, decode_sim, encode_control,
    },
    sync::{ChunkAssembler, SyncError, apply_sync},
};

/// Connection state for a viewer listener.
#[derive(Clone, Debug)]
pub enum ConnStatus {
    /// No simulation is connected. The listener continues to accept connections.
    Waiting,
    /// A simulation is connected.
    Connected {
        /// Simulation peer address.
        addr: String,
    },
}

/// Consistent view of the listener state at one point in time.
///
/// `frame`, `scene`, and `debug` come from one session lock. The frame does not
/// contain scene metadata. The scene is canonical and is not changed by debug
/// lifetime filtering. The debug scene is filtered against the same frame.
#[derive(Clone, Debug)]
pub struct ViewerSnapshot {
    /// Current connection generation. It changes on connect and disconnect.
    pub generation: u64,
    /// Current connection state.
    pub status: ConnStatus,
    /// Latest physics frame, if one has arrived in this session.
    pub frame: Option<Arc<VisFrame>>,
    /// Canonical scene metadata received in the session.
    pub scene: Option<Arc<SceneMetadata>>,
    /// Retained scene filtered against this snapshot's frame and time.
    pub debug: Arc<DebugScene>,
    /// Total frames decoded since this listener started.
    pub received: u64,
    /// Retained-scene transfers applied in this session.
    pub debug_transfers: u64,
}

struct DisplayCache {
    revision: u64,
    tick: u64,
    has_frame: bool,
    checked_at_ms: u64,
    valid_until_tick: Option<u64>,
    valid_until_ms: Option<u64>,
    scene: Arc<DebugScene>,
}

struct ClientSession {
    generation: u64,
    status: ConnStatus,
    frame: Option<Arc<VisFrame>>,
    scene: Option<Arc<SceneMetadata>>,
    debug: Arc<DebugScene>,
    debug_transfers: u64,
    controls: Option<ControlMsg>,
    pending_buttons: u8,
    received: u64,
    display_cache: Option<DisplayCache>,
}

impl ClientSession {
    fn new() -> Self {
        Self {
            generation: 0,
            status: ConnStatus::Waiting,
            frame: None,
            scene: None,
            debug: Arc::new(DebugScene::empty()),
            debug_transfers: 0,
            controls: None,
            pending_buttons: 0,
            received: 0,
            display_cache: None,
        }
    }

    fn reset(&mut self, status: ConnStatus) {
        self.generation = self.generation.wrapping_add(1);
        self.status = status;
        self.frame = None;
        self.scene = None;
        self.debug = Arc::new(DebugScene::empty());
        self.debug_transfers = 0;
        self.controls = None;
        self.pending_buttons = 0;
        self.display_cache = None;
    }
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
        .min(u64::MAX as u128) as u64
}

fn display_debug_scene(session: &mut ClientSession, now_ms: u64) -> Arc<DebugScene> {
    let has_frame = session.frame.is_some();
    let tick = session.frame.as_ref().map_or(0, |frame| frame.debug_tick);
    if let Some(cache) = &session.display_cache
        && cache.revision == session.debug.revision
        && tick >= cache.tick
        && cache
            .valid_until_tick
            .is_none_or(|deadline| tick < deadline)
        && cache.has_frame == has_frame
        && now_ms >= cache.checked_at_ms
        && cache
            .valid_until_ms
            .is_none_or(|deadline| now_ms < deadline)
    {
        return cache.scene.clone();
    }

    let visible = |group: &crate::debug::DebugGroup| match group.lifetime {
        Lifetime::Persistent => true,
        Lifetime::Frame => {
            has_frame
                && group
                    .expires_tick
                    .is_some_and(|deadline| tick >= deadline.saturating_sub(1) && tick < deadline)
        }
        Lifetime::Ticks(_) => group.expires_tick.is_none_or(|deadline| tick < deadline),
        Lifetime::Seconds(_) => group.expires_at_ms.is_none_or(|deadline| now_ms < deadline),
    };
    let mut next_expiry = None;
    let mut next_tick = None;
    let mut filtered = false;
    for group in session.debug.groups.values() {
        filtered |= !visible(group);
        let tick_deadline = match group.lifetime {
            Lifetime::Frame => group.expires_tick.map(|deadline| {
                let start = deadline.saturating_sub(1);
                if tick < start { start } else { deadline }
            }),
            Lifetime::Ticks(_) => group.expires_tick,
            _ => None,
        };
        if let Some(deadline) = tick_deadline.filter(|deadline| *deadline > tick) {
            next_tick = Some(next_tick.map_or(deadline, |current: u64| current.min(deadline)));
        }
        if let Some(deadline) = group.expires_at_ms.filter(|deadline| *deadline > now_ms) {
            next_expiry = Some(next_expiry.map_or(deadline, |current: u64| current.min(deadline)));
        }
    }
    let scene = if filtered {
        Arc::new(DebugScene {
            groups: session
                .debug
                .groups
                .iter()
                .filter(|(_, group)| visible(group))
                .map(|(name, group)| (name.clone(), group.clone()))
                .collect(),
            meshes: session.debug.meshes.clone(),
            textures: session.debug.textures.clone(),
            revision: session.debug.revision,
        })
    } else {
        session.debug.clone()
    };
    session.display_cache = Some(DisplayCache {
        revision: session.debug.revision,
        tick,
        has_frame,
        checked_at_ms: now_ms,
        valid_until_tick: next_tick,
        valid_until_ms: next_expiry,
        scene: scene.clone(),
    });
    scene
}

/// Viewer-side TCP listener for frames, controls, and retained scene data.
pub struct ViewerListener {
    session: Arc<Mutex<ClientSession>>,
    local_addr: SocketAddr,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl ViewerListener {
    /// Bind a listener and start its receiver thread.
    ///
    /// Use port `0` to select an available port. Use [`Self::try_bind`] to
    /// handle bind and socket setup errors.
    ///
    /// # Panics
    ///
    /// Panic if binding fails or the operating system cannot create the worker.
    pub fn bind(addr: &str) -> Self {
        Self::try_bind(addr)
            .unwrap_or_else(|error| panic!("rocketsim-viewer: cannot listen on {addr} ({error})"))
    }

    /// Bind a listener and start its receiver thread.
    ///
    /// Use port `0` to select an available port, then read the selected address
    /// with [`Self::local_addr`]. Use non-loopback listeners only on trusted
    /// networks.
    ///
    /// # Errors
    ///
    /// Return bind, address-resolution, or socket setup errors.
    ///
    /// # Panics
    ///
    /// Panic if the operating system cannot create the receiver thread.
    pub fn try_bind(addr: &str) -> std::io::Result<Self> {
        let listener = TcpListener::bind(addr)?;
        listener.set_nonblocking(true)?;
        let local_addr = listener.local_addr()?;
        let session = Arc::new(Mutex::new(ClientSession::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let worker = {
            let (session, stop) = (session.clone(), stop.clone());
            std::thread::spawn(move || accept_loop(listener, &session, &stop))
        };
        Ok(Self {
            session,
            local_addr,
            stop,
            worker: Some(worker),
        })
    }

    /// Return one consistent snapshot of the current listener state.
    pub fn snapshot(&self) -> ViewerSnapshot {
        self.session
            .lock()
            .map(|mut session| {
                let debug = display_debug_scene(&mut session, now_ms());
                ViewerSnapshot {
                    generation: session.generation,
                    status: session.status.clone(),
                    frame: session.frame.clone(),
                    scene: session.scene.clone(),
                    debug,
                    received: session.received,
                    debug_transfers: session.debug_transfers,
                }
            })
            .unwrap_or_else(|_| ViewerSnapshot {
                generation: 0,
                status: ConnStatus::Waiting,
                frame: None,
                scene: None,
                debug: Arc::new(DebugScene::empty()),
                received: 0,
                debug_transfers: 0,
            })
    }

    /// Queue controls for the current connection.
    ///
    /// Return `false` if no simulation is connected. This method combines
    /// action bits with pending requests and replaces pending analog controls.
    pub fn send_controls(&self, msg: ControlMsg) -> bool {
        let generation = self
            .session
            .lock()
            .map(|session| session.generation)
            .unwrap_or(0);
        self.send_controls_for_generation(generation, msg)
    }

    /// Queue controls only for the specified connection generation.
    ///
    /// Replace pending analog controls and combine action bits. The method
    /// writes the active generation into the message. Return `true` when the
    /// listener accepts the message. This does not confirm delivery.
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

    /// Return the address bound by this listener.
    pub fn local_addr(&self) -> SocketAddr {
        self.local_addr
    }

    /// Stop the listener thread.
    ///
    /// This method runs automatically when the listener is dropped.
    pub fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

impl Drop for ViewerListener {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn accept_loop(listener: TcpListener, session: &Mutex<ClientSession>, stop: &AtomicBool) {
    while !stop.load(Ordering::Acquire) {
        match listener.accept() {
            Ok((stream, _)) => {
                let active = Arc::new(AtomicBool::new(true));
                let extra_active = active.clone();
                let extra_listener = match listener.try_clone() {
                    Ok(listener) => listener,
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
                    .map(|address| address.to_string())
                    .unwrap_or_default();
                if let Ok(mut current) = session.lock() {
                    current.reset(ConnStatus::Connected { addr: peer.clone() });
                }
                pump(stream, session, stop, &peer);
                active.store(false, Ordering::Release);
                let _ = extra_worker.join();
                if let Ok(mut current) = session.lock() {
                    current.reset(ConnStatus::Waiting);
                }
            }
            Err(error)
                if error.kind() == std::io::ErrorKind::WouldBlock
                    || error.kind() == std::io::ErrorKind::TimedOut =>
            {
                std::thread::sleep(Duration::from_millis(20));
            }
            Err(_) => std::thread::sleep(Duration::from_millis(50)),
        }
    }
}

fn apply_transfer(session: &Mutex<ClientSession>, message: DebugSyncMsg) -> Result<(), SyncError> {
    let mut current = session.lock().map_err(|_| SyncError::Poisoned)?;
    let next = apply_sync(&current.debug, message)?;
    current.debug = Arc::new(next);
    current.debug_transfers = current.debug_transfers.wrapping_add(1);
    current.display_cache = None;
    Ok(())
}

fn pump(mut stream: TcpStream, session: &Mutex<ClientSession>, stop: &AtomicBool, peer: &str) {
    if stream.set_nonblocking(true).is_err() {
        return;
    }
    let mut input = Vec::new();
    let mut chunk = [0u8; 65536];
    let mut output = OutgoingBuffer::default();
    let mut assembler = ChunkAssembler::default();
    loop {
        if stop.load(Ordering::Acquire) {
            let _ = stream.shutdown(std::net::Shutdown::Both);
            return;
        }
        if !output.is_empty() && output.flush(&mut stream).is_err() {
            return;
        }
        if output.is_empty() {
            let next = session.lock().ok().and_then(|mut state| {
                state.controls.take().map(|mut msg| {
                    msg.generation = state.generation;
                    msg.buttons = state.pending_buttons;
                    state.pending_buttons = 0;
                    msg
                })
            });
            if let Some(message) = next {
                output.queue(encode_control(&message));
                continue;
            }
        }
        match stream.read(&mut chunk) {
            Ok(0) => return,
            Ok(count) => {
                input.extend_from_slice(&chunk[..count]);
                loop {
                    match decode_sim(&input) {
                        Ok(Some((message, consumed))) => {
                            match message {
                                SimMsg::Frame(mut frame) => {
                                    let metadata = frame.scene.take().map(Arc::new);
                                    if let Ok(mut state) = session.lock() {
                                        if let Some(metadata) = metadata {
                                            state.scene = Some(metadata);
                                        }
                                        state.frame = Some(Arc::new(frame));
                                        state.received = state.received.wrapping_add(1);
                                    }
                                }
                                SimMsg::DebugChunk(chunk) => match assembler.push(chunk) {
                                    Ok(Some(message)) => {
                                        if let Err(error) = apply_transfer(session, message) {
                                            eprintln!(
                                                "rocketsim-viewer: closing invalid debug stream from {peer}: {error}"
                                            );
                                            return;
                                        }
                                    }
                                    Ok(None) => {}
                                    Err(error) => {
                                        eprintln!(
                                            "rocketsim-viewer: closing invalid debug stream from {peer}: {error}"
                                        );
                                        return;
                                    }
                                },
                            }
                            input.drain(..consumed);
                        }
                        Ok(None) => break,
                        Err(error) => {
                            eprintln!(
                                "rocketsim-viewer: closing invalid sim stream from {peer}: {error}"
                            );
                            return;
                        }
                    }
                }
                if input.len() > crate::proto::MAX_SIM_MSG_BYTES + 4 {
                    return;
                }
            }
            Err(error)
                if error.kind() == std::io::ErrorKind::WouldBlock
                    || error.kind() == std::io::ErrorKind::Interrupted => {}
            Err(_) => return,
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

#[cfg(test)]
mod tests {
    use std::{
        io::{Read, Write},
        net::TcpStream,
        time::Instant,
    };

    use super::*;
    use crate::proto::encode_sim;

    #[test]
    fn display_cache_expires_by_time_and_keeps_canonical_scene() {
        use crate::debug::{DebugGroup, DrawList, GroupOptions};

        let mut group = DebugGroup::new(
            "temporary",
            DrawList::new(),
            GroupOptions::default().lifetime(Lifetime::Seconds(1.0)),
        )
        .unwrap();
        group.expires_at_ms = Some(1080);
        let mut debug = DebugScene::empty();
        debug.groups.insert("temporary".into(), Arc::new(group));
        debug.groups.insert(
            "persistent".into(),
            Arc::new(
                DebugGroup::new("persistent", DrawList::new(), GroupOptions::default()).unwrap(),
            ),
        );
        debug.revision = 7;
        let scene = Arc::new(SceneMetadata::empty(rocketsim::GameMode::TheVoid));
        let mut session = ClientSession::new();
        session.debug = Arc::new(debug);
        session.scene = Some(scene.clone());
        let first = display_debug_scene(&mut session, 1000);
        let second = display_debug_scene(&mut session, 1001);
        assert!(Arc::ptr_eq(&first, &second));
        assert!(first.groups.contains_key("temporary"));
        assert!(session.debug.groups.contains_key("temporary"));
        let expired = display_debug_scene(&mut session, 1100);
        assert!(!expired.groups.contains_key("temporary"));
        assert!(expired.groups.contains_key("persistent"));
        assert!(session.debug.groups.contains_key("temporary"));
        assert!(Arc::ptr_eq(session.scene.as_ref().unwrap(), &scene));

        let patched = apply_sync(
            &session.debug,
            DebugSyncMsg::Delta(crate::proto::DebugDelta {
                group_patches: vec![crate::proto::DebugGroupPatch {
                    name: "temporary".into(),
                    visible: true,
                    anchor: crate::debug::Anchor::World,
                    transform: crate::debug::GroupTransform::identity(),
                    lifetime: Lifetime::Seconds(1.0),
                    expires_tick: None,
                    expires_at_ms: Some(2100),
                    revision: 2,
                }],
                rev: 8,
                ..Default::default()
            }),
        )
        .unwrap();
        session.debug = Arc::new(patched);
        session.display_cache = None;
        let patched_view = display_debug_scene(&mut session, 1100);
        assert!(patched_view.groups.contains_key("temporary"));
        assert!(Arc::ptr_eq(&patched_view, &session.debug));

        let arena = rocketsim::Arena::new(rocketsim::GameMode::TheVoid);
        let mut frame = VisFrame::new(arena.get_arena_state(), 1.0 / 120.0);
        frame.debug_tick = 1;
        session.frame = Some(Arc::new(frame));
        let first_tick = display_debug_scene(&mut session, 1100);
        assert!(Arc::ptr_eq(&first_tick, &session.debug));
        Arc::make_mut(session.frame.as_mut().unwrap()).debug_tick = 2;
        let next_tick = display_debug_scene(&mut session, 1100);
        assert!(Arc::ptr_eq(&next_tick, &session.debug));

        let mut frame_group = DebugGroup::new(
            "frame",
            DrawList::new(),
            GroupOptions::default().lifetime(Lifetime::Frame),
        )
        .unwrap();
        frame_group.expires_tick = Some(4);
        let canonical = Arc::make_mut(&mut session.debug);
        canonical
            .groups
            .insert("frame".into(), Arc::new(frame_group));
        canonical.revision += 1;
        let before_frame = display_debug_scene(&mut session, 1100);
        assert!(!before_frame.groups.contains_key("frame"));
        Arc::make_mut(session.frame.as_mut().unwrap()).debug_tick = 3;
        assert!(
            display_debug_scene(&mut session, 1100)
                .groups
                .contains_key("frame")
        );
        Arc::make_mut(session.frame.as_mut().unwrap()).debug_tick = 4;
        assert!(
            !display_debug_scene(&mut session, 1100)
                .groups
                .contains_key("frame")
        );
        Arc::make_mut(session.frame.as_mut().unwrap()).debug_tick = 3;
        assert!(
            display_debug_scene(&mut session, 1100)
                .groups
                .contains_key("frame")
        );
        assert!(
            !display_debug_scene(&mut session, 2200)
                .groups
                .contains_key("temporary")
        );
        assert!(
            display_debug_scene(&mut session, 1100)
                .groups
                .contains_key("temporary")
        );
    }

    #[test]
    fn listener_stores_latest_frame_and_scene() {
        let listener = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        let mut stream = TcpStream::connect(listener.local_addr()).unwrap();
        let arena = rocketsim::Arena::new(rocketsim::GameMode::TheVoid);
        let mut frame = VisFrame::new(arena.get_arena_state(), 1.0 / 120.0);
        frame.scene = Some(SceneMetadata::empty(rocketsim::GameMode::TheVoid));
        stream
            .write_all(&encode_sim(&SimMsg::Frame(frame)))
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            let snapshot = listener.snapshot();
            if let Some(frame) = &snapshot.frame {
                assert!(snapshot.scene.is_some());
                assert!(frame.scene.is_none());
                assert_eq!(snapshot.received, 1);
                break;
            }
            assert!(Instant::now() < deadline, "frame was not received");
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    #[test]
    fn controls_keep_action_pulses_and_reject_stale_generations() {
        let listener = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        let mut stream = TcpStream::connect(listener.local_addr()).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        let generation = loop {
            let snapshot = listener.snapshot();
            if matches!(snapshot.status, ConnStatus::Connected { .. }) {
                break snapshot.generation;
            }
            assert!(Instant::now() < deadline, "connection was not accepted");
            std::thread::sleep(Duration::from_millis(2));
        };
        assert!(listener.send_controls_for_generation(
            generation,
            ControlMsg::new(0, rocketsim::CarControls::default(), 1, 1),
        ));
        assert!(listener.send_controls_for_generation(
            generation,
            ControlMsg::new(0, rocketsim::CarControls::default(), 4, 2),
        ));
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let mut bytes = [0u8; 1024];
        let count = stream.read(&mut bytes).unwrap();
        let (message, _) = crate::proto::decode_control(&bytes[..count])
            .unwrap()
            .unwrap();
        assert_eq!(message.generation, generation);
        assert_eq!(message.buttons, 5);
        drop(stream);
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            let snapshot = listener.snapshot();
            if snapshot.generation > generation && matches!(snapshot.status, ConnStatus::Waiting) {
                assert!(snapshot.frame.is_none());
                assert!(snapshot.scene.is_none());
                assert!(!listener.send_controls_for_generation(
                    generation,
                    ControlMsg::new(0, rocketsim::CarControls::default(), 7, 3),
                ));
                break;
            }
            assert!(Instant::now() < deadline, "disconnect was not observed");
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    #[test]
    fn competing_sim_connection_is_rejected() {
        let listener = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        let owner = TcpStream::connect(listener.local_addr()).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while !matches!(listener.snapshot().status, ConnStatus::Connected { .. }) {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        let mut extra = TcpStream::connect(listener.local_addr()).unwrap();
        extra
            .set_read_timeout(Some(Duration::from_secs(1)))
            .unwrap();
        let mut byte = [0u8; 1];
        assert!(matches!(extra.read(&mut byte), Ok(0) | Err(_)));
        assert!(matches!(
            listener.snapshot().status,
            ConnStatus::Connected { .. }
        ));
        drop(owner);
    }

    #[test]
    fn shutdown_with_stalled_peer_is_bounded() {
        let mut listener = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        let mut peer = TcpStream::connect(listener.local_addr()).unwrap();
        peer.write_all(&[0x20, 0x00]).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        let start = Instant::now();
        listener.shutdown();
        assert!(start.elapsed() < Duration::from_secs(1));
    }
}
