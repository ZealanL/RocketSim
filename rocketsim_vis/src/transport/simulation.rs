//! Send simulation frames to a detached viewer through [`NetVis`].
//!
//! [`NetVis`] implements [`rocketsim::Vis`]. It copies the latest update into
//! shared storage. A background thread handles network writes and retries.
//! The simulation does not wait for those writes. Intermediate frames can
//! be lost when a viewer is slow or disconnected.
//!
//! Prefer [`crate::ArenaVisExt::try_set_net_vis`] for normal applications.
//! It validates the address, reserves the viewer, and supplies arena metadata.
//! Construct [`NetVis`] directly only for custom visualizer integration.
//!
//! Frames and viewer controls share one connection. Use
//! [`crate::transport::viewer::ViewerListener`] to implement a viewer.

use std::{
    io::Read,
    net::{SocketAddr, TcpStream},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU8, Ordering},
    },
    thread::JoinHandle,
    time::Duration,
};

use rocketsim::{ArenaState, Vis};

use crate::{
    debug::{DebugRenderer, DebugScene},
    proto::{
        ControlMsg, DebugFull, DebugSyncMsg, SceneMetadata, SimMsg, VisFrame, decode_control,
        encode_sim,
    },
    sync::{SentDebug, mark_debug_sent, next_debug_sync},
    transport::OutgoingBuffer,
};

/// [`rocketsim::Vis`] implementation that streams `(ArenaState, dt)` to a
/// viewer process listening on `addr` (e.g. `"127.0.0.1:39201"`).
///
/// Construct with [`NetVis::connect`], or attach it to an arena with
/// [`crate::ArenaVisExt::set_net_vis_enabled`]. The sender retries after a
/// disconnect. It sends only the newest pending frame. Dropping this value
/// stops the sender thread.
pub struct NetVis {
    slot: Arc<Mutex<Option<VisFrame>>>,
    controls: Arc<Mutex<Option<(ControlMsg, std::time::Instant)>>>,
    actions: Arc<AtomicU8>,

    scene: Arc<Mutex<Option<SceneMetadata>>>,
    retained: DebugRenderer,
    debug_tick: Arc<std::sync::atomic::AtomicU64>,
    address: String,
    connected: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    sender: Option<JoinHandle<()>>,
    pub(crate) lease: Option<crate::launcher::ViewerLease>,
}

impl NetVis {
    /// Start a background sender without starting a viewer process.
    ///
    /// Supply a numeric `IP:port` address. The worker retries an absent viewer
    /// or invalid address instead of returning an error. This raw constructor
    /// does not reserve an address or populate scene metadata. Prefer
    /// [`crate::ArenaVisExt::try_set_net_vis`] for validated arena attachment.
    ///
    /// # Panics
    ///
    /// Panic if the operating system cannot create the sender thread.
    pub fn connect(addr: &str) -> Self {
        test_trace!("sender: creating connection worker for {addr}");
        let slot = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let controls = Arc::new(Mutex::new(None));
        let actions = Arc::new(AtomicU8::new(0));
        let connected = Arc::new(AtomicBool::new(false));

        let scene = Arc::new(Mutex::new(None));
        let debug_tick = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let retained = DebugRenderer::new_shared(debug_tick.clone());
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
                retained: retained.clone(),
            };
            std::thread::spawn(move || sender_loop(&addr, shared))
        };
        Self {
            slot,
            controls,
            actions,

            scene,
            retained,
            debug_tick,
            address,
            connected,
            stop,
            sender: Some(sender),
            lease: None,
        }
    }

    /// Returns the configured viewer socket address.
    pub fn address(&self) -> &str {
        &self.address
    }
    /// Returns whether the sender currently holds a viewer connection.
    pub fn connected(&self) -> bool {
        self.connected.load(Ordering::Acquire)
    }
    /// Returns the latest received control message if it arrived within the
    /// last 250 milliseconds. This does not consume the message.
    pub fn latest_controls(&self) -> Option<ControlMsg> {
        self.controls
            .lock()
            .ok()
            .and_then(|g| *g)
            .and_then(|(msg, at)| (at.elapsed() <= Duration::from_millis(250)).then_some(msg))
    }
    /// Return and clear accumulated action requests.
    ///
    /// Bit 0 requests reset, bit 1 requests a dribble, and bit 2 requests a
    /// ball launch. Return `0` if no action is pending. This does not perform
    /// the requested actions.
    pub fn take_actions(&self) -> u8 {
        self.actions.swap(0, Ordering::AcqRel)
    }

    /// Cloneable retained-scene handle. Accepted updates coalesce in bounded
    /// pending tables even while disconnected or paused; the sender thread
    /// serializes off the sim tick. Returns a handle even when disconnected.
    pub fn debug_renderer(&self) -> DebugRenderer {
        self.retained.clone()
    }
    /// Store valid scene metadata for the first frame of a connection.
    ///
    /// Ignore invalid metadata. Set this before sending the first frame.
    /// Changes after metadata was sent reach the viewer only on a subsequent
    /// connection. This does not modify the arena or check its game mode.
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
        test_trace!("sender {}: stop requested", self.address);
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.sender.take() {
            test_trace!("sender {}: joining worker", self.address);
            let _ = handle.join();
            test_trace!("sender {}: worker joined", self.address);
        }
    }
}

impl Vis for NetVis {
    fn update(&mut self, arena_state: &ArenaState, dt: f32) {
        // Tick first (atomic, never blocks): drives retained TTL expiry.
        // Retained tables are never touched here, so persistent drawings are
        // never cloned into ticks.
        let debug_tick = self
            .debug_tick
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed)
            .wrapping_add(1);
        // Latest-only overwrite via try_lock: drop the tick rather than
        // stall the simulation when the sender holds the slot.
        if let Ok(mut slot) = self.slot.try_lock() {
            let mut frame = VisFrame::new(arena_state.clone(), dt);
            frame.debug_tick = debug_tick;

            *slot = Some(frame);
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
    retained: DebugRenderer,
}

fn sender_loop(addr: &str, shared: SenderState) {
    let mut backoff = Duration::from_millis(500);
    let max_backoff = Duration::from_secs(5);
    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return;
        }
        test_trace!("sender {addr}: connecting");
        match connect_bounded(addr) {
            Ok(stream) => {
                test_trace!(
                    "sender {addr}: connected; local address {:?}",
                    stream.local_addr()
                );
                backoff = Duration::from_millis(500);
                let _ = stream.set_nodelay(true);
                shared.connected.store(true, Ordering::Release);
                pump(stream, &shared);
                test_trace!("sender {addr}: pump returned");
                shared.connected.store(false, Ordering::Release);
                if let Ok(mut c) = shared.controls.lock() {
                    *c = None;
                }
                shared.actions.store(0, Ordering::Release);
                // pump returned = connection lost; reconnect with backoff.
            }
            Err(error) => {
                test_trace!("sender {addr}: connect failed: {error}; retry in {backoff:?}");
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

/// Sends latest frames plus reliable retained-debug deltas, and receives
/// controls on the same accepted stream.
///
/// Frames stay latest-only (overwritten slot); debug deltas are reliable
/// per-group deltas split into `16 KiB` chunks interleaved with frames so a
/// large mesh upload never starves physics or controls. Serialization
/// happens here (off the sim tick). A reconnect resets `SentDebug` and
/// opens with a `Full`, so old deletes are never replayed.
fn pump(mut stream: TcpStream, shared: &SenderState) {
    test_trace!("sender: setting stream nonblocking");
    if let Err(error) = stream.set_nonblocking(true) {
        test_trace!("sender: nonblocking setup failed: {error}");
        return;
    }
    test_trace!("sender: setting write timeout");
    let _ = stream.set_write_timeout(Some(Duration::from_millis(250)));
    test_trace!("sender: stream configured; starting I/O");
    let peer = stream
        .peer_addr()
        .map(|a| a.to_string())
        .unwrap_or_default();
    let mut input = Vec::new();
    let mut chunk = [0u8; 2048];
    let mut metadata_sent = false;
    let mut queued_scene = false;
    let mut output = OutgoingBuffer::default();
    let mut sent = SentDebug::default();
    let mut debug_queue: std::collections::VecDeque<Vec<u8>> = std::collections::VecDeque::new();
    // When no debug transfer is known-sent yet but pending is empty, mark
    // the empty Full as sent so we do not spin generating `None` forever.
    let mut empty_full_marked = false;
    let mut last_was_debug = false;
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
                test_trace!("sender {peer}: received {n} control bytes");
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
        if !output.is_empty() {
            std::thread::sleep(Duration::from_millis(1));
            continue;
        }
        // Fill the debug chunk queue when empty (serialization off tick).
        if debug_queue.is_empty() {
            let pending = shared.retained.snapshot();
            if !sent.full_sent
                && !empty_full_marked
                && pending.groups.is_empty()
                && pending.meshes.is_empty()
                && pending.textures.is_empty()
            {
                sent.full_sent = true;
                empty_full_marked = true;
            } else if let Some(sync) = next_debug_sync(&pending, &sent) {
                let encoded = match crate::proto::encode_debug_sync(&sync) {
                    Ok(encoded) => encoded,
                    Err(delta_error) if matches!(&sync, DebugSyncMsg::Delta(_)) => {
                        eprintln!(
                            "rocketsim_vis: debug delta encoding failed; retrying as full scene: {delta_error}"
                        );
                        let full = full_debug_sync(&pending);
                        match crate::proto::encode_debug_sync(&full) {
                            Ok(encoded) => encoded,
                            Err(error) => {
                                eprintln!(
                                    "rocketsim_vis: full debug scene could not be encoded; reconnecting without acknowledging it: {error}"
                                );
                                return;
                            }
                        }
                    }
                    Err(error) => {
                        eprintln!(
                            "rocketsim_vis: debug scene could not be encoded; reconnecting without acknowledging it: {error}"
                        );
                        return;
                    }
                };
                let transfer = sent.transfer_id.wrapping_add(1).max(1);
                let chunks = crate::proto::split_debug_sync(&encoded, transfer);
                if chunks.is_empty() {
                    eprintln!(
                        "rocketsim_vis: encoded debug scene produced no chunks; reconnecting without acknowledging it"
                    );
                    return;
                }
                for msg in chunks {
                    debug_queue.push_back(encode_sim(&msg));
                }
                mark_debug_sent(&mut sent, &pending);
            }
        }
        let frame_pending = shared.slot.lock().map(|s| s.is_some()).unwrap_or(false);
        // Alternate only when both kinds wait. After debug, send physics; after
        // physics, send debug. This keeps both streams moving under steady load.
        let send_debug_next = !debug_queue.is_empty() && (!frame_pending || !last_was_debug);
        if send_debug_next && let Some(bytes) = debug_queue.pop_front() {
            if bytes.len() > crate::proto::MAX_SIM_MSG_BYTES + 4 {
                return;
            }
            output.queue(bytes);
            last_was_debug = true;
            continue;
        }
        let frame = shared.slot.lock().map(|mut s| s.take()).unwrap_or(None);
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
                let bytes = encode_sim(&SimMsg::Frame(frame));
                if bytes.len() > crate::proto::MAX_SIM_MSG_BYTES + 4 {
                    return;
                }
                output.queue(bytes);
                last_was_debug = false;
            }
            None => {
                if let Some(bytes) = debug_queue.pop_front() {
                    if bytes.len() > crate::proto::MAX_SIM_MSG_BYTES + 4 {
                        return;
                    }
                    output.queue(bytes);
                    last_was_debug = true;
                } else {
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
        }
    }
}

fn full_debug_sync(scene: &DebugScene) -> DebugSyncMsg {
    DebugSyncMsg::Full(DebugFull {
        groups: scene
            .groups
            .values()
            .map(|group| (**group).clone())
            .collect(),
        meshes: scene.meshes.values().map(|mesh| (**mesh).clone()).collect(),
        textures: scene
            .textures
            .values()
            .map(|texture| (**texture).clone())
            .collect(),
        rev: scene.revision,
    })
}

#[cfg(test)]
mod tests {
    use std::{
        io::{Read, Write},
        net::TcpListener,
        time::Instant,
    };

    use super::*;
    use crate::{
        debug::{
            Color, DebugError, DepthMode, DrawList, GroupOptions, GroupTransform, MAX_DEBUG_BYTES,
            MAX_GROUPS, MAX_MESH_INDICES, MAX_MESH_VERTICES, MeshResource, TextureResource, Width,
        },
        proto::{
            ControlMsg, DebugSyncMsg, MAX_DEBUG_SYNC_BYTES, decode_sim, encode_control,
            encode_debug_sync,
        },
        transport::viewer::ViewerListener,
    };

    fn wait_for_tick(mut stream: TcpStream, target: u64, deadline: Instant) -> Option<u64> {
        stream
            .set_read_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        let mut input = Vec::new();
        let mut chunk = [0u8; 65536];
        let mut latest = None;
        while Instant::now() < deadline {
            match stream.read(&mut chunk) {
                Ok(0) => return latest,
                Ok(count) => {
                    input.extend_from_slice(&chunk[..count]);
                    while let Some((message, consumed)) = decode_sim(&input).expect("valid stream")
                    {
                        if let SimMsg::Frame(frame) = message {
                            latest = Some(frame.state.tick_count);
                            if latest == Some(target) {
                                return latest;
                            }
                        }
                        input.drain(..consumed);
                    }
                }
                Err(error)
                    if error.kind() == std::io::ErrorKind::TimedOut
                        || error.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(_) => return latest,
            }
        }
        latest
    }

    #[test]
    fn same_stream_controls_preserve_action_pulses() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let vis = NetVis::connect(&listener.local_addr().unwrap().to_string());
        let (mut stream, _) = listener.accept().unwrap();
        for (buttons, seq) in [(1, 1), (4, 2)] {
            stream
                .write_all(&encode_control(&ControlMsg::new(
                    0,
                    rocketsim::CarControls::default(),
                    buttons,
                    seq,
                )))
                .unwrap();
        }
        let deadline = Instant::now() + Duration::from_secs(2);
        while vis.latest_controls().is_none_or(|message| message.seq < 2) {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(vis.take_actions(), 5);
        assert_eq!(vis.take_actions(), 0);
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
    fn sender_keeps_the_latest_physics_frame() {
        use rocketsim::{Arena, GameMode, Vis};

        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let mut vis = NetVis::connect(&listener.local_addr().unwrap().to_string());
        let (tx, rx) = std::sync::mpsc::channel();
        let server = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            let _ = tx.send(wait_for_tick(
                stream,
                2,
                Instant::now() + Duration::from_secs(5),
            ));
        });
        let mut arena = Arena::new(GameMode::TheVoid);
        for _ in 0..2 {
            arena.step_tick();
            vis.update(&arena.get_arena_state(), 1.0 / 120.0);
        }
        assert_eq!(rx.recv_timeout(Duration::from_secs(6)).unwrap(), Some(2));
        drop(vis);
        server.join().unwrap();
    }

    #[test]
    fn large_scene_and_debug_chunks_interleave_with_live_frames() {
        use std::sync::{
            Arc,
            atomic::{AtomicBool, Ordering},
        };

        use rocketsim::Vis;

        let viewer = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        let mut vis = NetVis::connect(&viewer.local_addr().to_string());
        let mut scene = SceneMetadata::empty(rocketsim::GameMode::TheVoid);
        scene.meshes.push(crate::proto::MeshData {
            vertices: (0..6000).map(|index| [index as f32, 0.0, 0.0]).collect(),
            indices: Vec::new(),
        });
        vis.set_scene_metadata(scene);
        let arena = rocketsim::Arena::new(rocketsim::GameMode::TheVoid);
        let state = arena.get_arena_state();
        rocketsim::Vis::update(&mut vis, &state, 1.0 / 120.0);
        let renderer = vis.debug_renderer();

        let old_group_names: Vec<_> = (0..MAX_GROUPS)
            .map(|index| format!("old/group_{index:03}_padding_for_full_removal_test"))
            .collect();
        for name in &old_group_names {
            let mut draw = DrawList::new();
            draw.point(
                [1.0, 2.0, 3.0],
                Color::WHITE,
                Width::pixels(2.0),
                DepthMode::Test,
            );
            renderer
                .replace_group(name, draw, GroupOptions::default())
                .unwrap();
        }
        renderer
            .insert_texture(
                TextureResource::new("bulk", 1024, 1024, vec![0; 1024 * 1024 * 4]).unwrap(),
            )
            .unwrap();

        let rich_index_count = MAX_MESH_INDICES - MAX_MESH_INDICES % 3;
        let mut rich_mesh = MeshResource::new(
            "rich",
            vec![[0.0; 3]; MAX_MESH_VERTICES],
            vec![Color::WHITE; MAX_MESH_VERTICES],
            Some(vec![[0.0, 0.0, 1.0]; MAX_MESH_VERTICES]),
            Some(vec![[0.0, 0.0]; MAX_MESH_VERTICES]),
            vec![(MAX_MESH_VERTICES - 1) as u32; rich_index_count],
        )
        .unwrap();
        assert!(matches!(
            renderer.insert_mesh(rich_mesh.clone()),
            Err(DebugError::PayloadTooLarge { .. })
        ));
        assert!(!renderer.snapshot().meshes.contains_key("rich"));
        rich_mesh.indices.truncate(3);

        let first_deadline = Instant::now() + Duration::from_secs(10);
        let first_transfer = loop {
            let snapshot = viewer.snapshot();
            if snapshot.debug_transfers > 0
                && snapshot.debug.groups.len() == old_group_names.len()
                && snapshot.debug.textures.contains_key("bulk")
            {
                break snapshot;
            }
            assert!(
                Instant::now() < first_deadline,
                "initial retained scene did not arrive"
            );
            std::thread::sleep(Duration::from_millis(2));
        };

        let stop = Arc::new(AtomicBool::new(false));
        let producer = {
            let stop = stop.clone();
            let mut vis = vis;
            std::thread::spawn(move || {
                while !stop.load(Ordering::Acquire) {
                    vis.update(&state, 1.0 / 120.0);
                    std::hint::spin_loop();
                }
            })
        };
        let frame_deadline = Instant::now() + Duration::from_secs(3);
        let frames_before_completion = loop {
            let snapshot = viewer.snapshot();
            if snapshot.received > first_transfer.received {
                break snapshot.received;
            }
            assert!(
                Instant::now() < frame_deadline,
                "physics frames did not resume"
            );
            std::thread::sleep(Duration::from_millis(1));
        };

        renderer.clear_all();
        renderer.insert_mesh(rich_mesh).unwrap();
        renderer
            .insert_texture(
                TextureResource::new("large_tex", 1024, 1023, vec![0; 1024 * 1023 * 4]).unwrap(),
            )
            .unwrap();
        renderer
            .insert_texture(
                TextureResource::new("fill", 1024, 256, vec![0; 1024 * 256 * 4]).unwrap(),
            )
            .unwrap();
        let mut draw = DrawList::new();
        draw.instance(
            "rich",
            [0.0; 3],
            GroupTransform::identity().rotation,
            [1.0; 3],
            Color::WHITE,
            Some("large_tex".into()),
            DepthMode::Test,
        );
        renderer
            .replace_group("rich_instance", draw, GroupOptions::default())
            .unwrap();
        let pending = renderer.snapshot();
        assert!(pending.estimated_bytes() <= MAX_DEBUG_BYTES);
        let mut sent = SentDebug::default();
        mark_debug_sent(&mut sent, &first_transfer.debug);
        let delta = next_debug_sync(&pending, &sent).expect("retained changes need a delta");
        assert!(matches!(&delta, DebugSyncMsg::Delta(_)));
        assert!(
            encode_debug_sync(&delta).is_err(),
            "many removals must push the near-limit delta over the wire cap"
        );
        let encoded_full = encode_debug_sync(&full_debug_sync(&pending))
            .expect("accepted near-limit scene must encode as a full publication");
        assert!(encoded_full.len() <= MAX_DEBUG_SYNC_BYTES);
        assert!(encoded_full.len() > MAX_DEBUG_SYNC_BYTES - 12 * 1024);

        let deadline = Instant::now() + Duration::from_secs(20);
        let mut completed = None;
        while Instant::now() < deadline {
            let snapshot = viewer.snapshot();
            if snapshot.debug_transfers > first_transfer.debug_transfers
                && snapshot.debug.groups.contains_key("rich_instance")
                && snapshot.debug.meshes.contains_key("rich")
                && snapshot.debug.textures.contains_key("large_tex")
                && snapshot.debug.textures.contains_key("fill")
                && old_group_names
                    .iter()
                    .all(|name| !snapshot.debug.groups.contains_key(name))
            {
                completed = Some(snapshot);
                break;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        stop.store(true, Ordering::Release);
        producer.join().unwrap();
        let completed = completed.expect("near-limit retained transfer did not arrive");
        assert_eq!(
            completed
                .scene
                .as_ref()
                .map(|scene| scene.meshes[0].vertices.len()),
            Some(6000)
        );
        assert!(
            completed.received > frames_before_completion,
            "physics did not advance while retained chunks were in flight"
        );
    }

    #[test]
    fn retained_updates_arrive_while_physics_is_paused() {
        let viewer = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        let vis = NetVis::connect(&viewer.local_addr().to_string());
        let renderer = vis.debug_renderer();
        let mut draw = DrawList::new();
        draw.point([0.0; 3], Color::RED, Width::pixels(3.0), DepthMode::Test);
        renderer
            .replace_group("paused", draw, GroupOptions::default())
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(5);
        while !viewer.snapshot().debug.groups.contains_key("paused") {
            assert!(Instant::now() < deadline, "retained group did not arrive");
            std::thread::sleep(Duration::from_millis(2));
        }
        renderer.set_group_visible("paused", false).unwrap();
        while viewer
            .snapshot()
            .debug
            .get_group("paused")
            .is_none_or(|group| group.visible)
        {
            assert!(Instant::now() < deadline, "visibility patch did not arrive");
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    #[test]
    fn retained_diff_sends_mesh_once_then_transform_only() {
        let retained = DebugRenderer::new_detached();
        retained
            .insert_mesh(
                MeshResource::new(
                    "tri",
                    vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [0.0, 100.0, 0.0]],
                    vec![Color::WHITE; 3],
                    None,
                    None,
                    vec![0, 1, 2],
                )
                .unwrap(),
            )
            .unwrap();
        let mut draw = DrawList::new();
        draw.instance(
            "tri",
            [0.0; 3],
            GroupTransform::identity().rotation,
            [1.0; 3],
            Color::WHITE,
            None,
            DepthMode::Test,
        );
        retained
            .replace_group("instance", draw, GroupOptions::default())
            .unwrap();
        let mut sent = SentDebug::default();
        let pending = retained.snapshot();
        let first = next_debug_sync(&pending, &sent).expect("full publication");
        assert!(matches!(first, DebugSyncMsg::Full(_)));
        mark_debug_sent(&mut sent, &pending);
        retained
            .set_group_transform("instance", GroupTransform::from_position([50.0, 0.0, 0.0]))
            .unwrap();
        let second = next_debug_sync(&retained.snapshot(), &sent).expect("delta publication");
        let DebugSyncMsg::Delta(delta) = second else {
            panic!("expected a delta")
        };
        assert!(delta.upsert_groups.is_empty());
        assert_eq!(delta.group_patches.len(), 1);
        assert!(delta.upsert_meshes.is_empty());
        assert!(delta.removed_groups.is_empty());
    }

    #[test]
    fn retained_diff_coalesces_changes_and_keeps_removals() {
        let retained = DebugRenderer::new_detached();
        for name in ["a", "b"] {
            let mut draw = DrawList::new();
            draw.point([0.0; 3], Color::RED, Width::pixels(3.0), DepthMode::Test);
            retained
                .replace_group(name, draw, GroupOptions::default())
                .unwrap();
        }
        let mut sent = SentDebug::default();
        mark_debug_sent(&mut sent, &retained.snapshot());
        for x in [2.0, 3.0] {
            let mut draw = DrawList::new();
            draw.point(
                [x, 0.0, 0.0],
                Color::BLUE,
                Width::pixels(3.0),
                DepthMode::Test,
            );
            retained
                .replace_group("a", draw, GroupOptions::default())
                .unwrap();
        }
        assert!(retained.clear_group("b").unwrap());
        let result = next_debug_sync(&retained.snapshot(), &sent).expect("delta");
        let DebugSyncMsg::Delta(delta) = result else {
            panic!("expected a delta")
        };
        assert_eq!(delta.upsert_groups.len(), 1);
        assert_eq!(delta.upsert_groups[0].name, "a");
        assert_eq!(delta.removed_groups, vec!["b"]);
    }
}
