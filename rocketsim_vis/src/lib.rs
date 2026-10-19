//! Connect a RocketSim simulation to the separate `rocketsim-viewer` application.
//!
//! Send simulation frames, arena geometry, and debug drawings. Receive optional
//! driving controls and action requests through the same TCP connection. This
//! crate does not include graphics dependencies. Your application runs the
//! simulation. The viewer only displays its state and sends input requests.
//!
//! # Choose an API
//!
//! | Task | API |
//! | --- | --- |
//! | Attach a viewer to an arena | [`ArenaVisExt`] |
//! | Draw lines, labels, and reusable meshes | [`debug`] |
//! | Run a simulation near real time | [`pace::TickPacer`] |
//! | Start a viewer and control its process lifetime | [`launcher`] |
//! | Apply the drive example's live mutators | [`mutators::Mutators`] |
//! | Write a custom viewer or transport | [`net_client`], [`net_vis`], [`proto`] |
//! | Create a fixed scene for screenshot tests | [`photo`] |
//!
//! Use [`ArenaVisExt`] for normal simulation applications. The lower-level
//! transport types do not start a simulation or apply controls for you.
//!
//! # Send your first frames
//!
//! Install or build `rocketsim-viewer` separately. Put its executable on `PATH`,
//! or set `ROCKETSIM_VIEWER` to its executable path. Use matching versions of
//! `rocketsim`, `rocketsim_vis`, and `rocketsim_viewer`.
//!
//! Supply compatible collision meshes before you create a Soccar arena. This
//! crate does not include those assets. [`rocketsim::init_from_default`] reads
//! `./collision_meshes/` relative to the process's working directory.
//!
//! This program creates an arena and shows about two seconds of simulation.
//! It starts a viewer if none listens at [`DEFAULT_VIEWER_ADDR`].
//!
//! ```no_run
//! use rocketsim::{
//!     Arena, CarBodyConfig, CarControls, GameMode, Team, init_from_default,
//! };
//! use rocketsim_vis::{ArenaVisExt, pace::TickPacer};
//!
//! fn main() -> std::io::Result<()> {
//!     init_from_default(true)?;
//!     let mut arena = Arena::new(GameMode::Soccar);
//!     let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
//!     arena.reset_to_random_kickoff(Some(0));
//!     arena.try_set_vis_enabled(true)?;
//!     arena.set_car_controls(car, CarControls::default().with_throttle(1.0));
//!
//!     let mut pacer = TickPacer::at_120hz();
//!     let mut ticks = 0;
//!     while ticks < 240 {
//!         for _ in 0..pacer.wait().min(240 - ticks) {
//!             arena.step_tick();
//!             ticks += 1;
//!         }
//!     }
//!     arena.try_set_vis_enabled(false)?;
//!     Ok(())
//! }
//! ```
//!
//! Each [`rocketsim::Arena::step_tick`] advances the simulation by `1/120`
//! second. It does not wait for real time. Use [`pace::TickPacer`] for
//! interactive runs. Omit that wait for batch simulation. A slow viewer can
//! miss intermediate frames. It does not stop the simulation's network sender.
//!
//! # Connect to a viewer you start yourself
//!
//! Start `rocketsim-viewer 39202`, then attach to that numeric address.
//! [`try_set_net_vis`](ArenaVisExt::try_set_net_vis) does not start a process.
//! It starts a background connection attempt and returns before connection.
//!
//! ```no_run
//! use rocketsim::{Arena, GameMode};
//! use rocketsim_vis::ArenaVisExt;
//!
//! // TheVoid does not need collision meshes.
//! let mut arena = Arena::new(GameMode::TheVoid);
//! arena.try_set_net_vis(Some("127.0.0.1:39202"))?;
//! arena.step_tick();
//! arena.try_set_net_vis(None)?;
//! # Ok::<(), std::io::Error>(())
//! ```
//!
//! Each viewer accepts one arena. Use a separate process and port for each
//! arena. A second arena in the same process gets [`std::io::ErrorKind::AddrInUse`]
//! if it reserves an address that another arena uses. Host names are not resolved.
//!
//! # Apply viewer controls
//!
//! Your application must apply received inputs before each simulation tick.
//! Use neutral controls when input is absent or more than 250 ms old. Check
//! `car_index` before you use it. For several cars, also release the previous
//! car's controls when the viewer selects another car.
//!
//! This example lets the viewer drive one car. It also handles reset requests.
//! Select the car in the viewer and close its settings menu before driving.
//!
//! ```no_run
//! use rocketsim::{
//!     Arena, CarBodyConfig, CarControls, GameMode, Team, init_from_default,
//! };
//! use rocketsim_vis::{ArenaVisExt, pace::TickPacer};
//!
//! init_from_default(true)?;
//! let mut arena = Arena::new(GameMode::Soccar);
//! let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
//! arena.reset_to_random_kickoff(Some(0));
//! arena.try_set_vis_enabled(true)?;
//! let mut pacer = TickPacer::at_120hz();
//! for _ in 0..240 {
//!     for _ in 0..pacer.wait() {
//!         let message = arena.latest_vis_controls()
//!             .filter(|message| message.car_index as usize == car);
//!         let controls = message.map_or_else(CarControls::default, |m| m.controls);
//!         if arena.take_vis_actions() & 1 != 0 {
//!             arena.reset_to_random_kickoff(None);
//!         }
//!         arena.set_car_controls(car, controls);
//!         arena.step_tick();
//!     }
//! }
//! # Ok::<(), std::io::Error>(())
//! ```
//!
//! [`ArenaVisExt::take_vis_actions`] consumes pending action bits. Bit 0
//! requests reset, bit 1 requests a dribble, and bit 2 requests a ball launch.
//! Your application decides how to handle each request. Mutator flags are
//! separate level values in [`proto::ControlMsg`]. Apply them with
//! [`mutators::Mutators`] if your application supports them.
//!
//! # Draw debug geometry
//!
//! Get a [`debug::DebugRenderer`] after you attach the arena. Use
//! [`draw_frame`](debug::DebugRenderer::draw_frame) for marks that last until
//! the next displayed simulation update. They expire at the following update.
//! Use named groups for persistent marks.
//! The [`debug`] module has complete examples for groups and resources.
//!
//! This example runs without a viewer. Use `arena.debug_renderer()` instead
//! of a detached renderer to send the same drawing to an attached viewer.
//!
//! ```
//! use rocketsim_vis::debug::{Color, DebugRenderer};
//!
//! let renderer = DebugRenderer::new_detached();
//! let target = renderer.group("planner.target");
//! target.draw(|draw| {
//!     draw.line([0.0, 0.0, 0.0], [500.0, 0.0, 200.0])
//!         .color(Color::GREEN)
//!         .width_px(2.0);
//!     draw.text_2d([0.02, 0.05], "Target selected");
//! })?;
//! assert_eq!(renderer.snapshot().groups["planner.target"].items.len(), 2);
//! target.clear()?;
//! # Ok::<(), rocketsim_vis::debug::DebugError>(())
//! ```
//!
//! World positions and dimensions use Unreal units. There are 50 Unreal units
//! per meter. `+Z` points up. Blue defends the `-Y` goal. Screen positions use
//! normalized coordinates with the origin at the top-left corner.
//!
//! # Connection lifetime and safety
//!
//! The sender retries a lost connection in the background. It keeps the latest
//! simulation frame instead of a frame queue. Retained debug groups and
//! resources remain available for the next connection.
//!
//! Detaching releases the arena's viewer attachment. A viewer started by this
//! crate closes when its last attachment or [`ViewerHandle`] is released. A
//! viewer that you start yourself stays open. Keep a handle from
//! [`try_ensure_viewer_running`] if you need the process between arena runs.
//!
//! The viewer listens on loopback by default. The protocol has no
//! authentication or encryption. Use remote connections only on trusted
//! networks. This crate and the matching viewer use protocol version 1.

#![warn(missing_docs)]

pub mod debug;
pub mod launcher;
pub mod mutators;
pub mod net_client;
pub mod net_vis;
pub mod pace;
pub mod photo;
pub mod proto;

use std::{any::Any, io, net::SocketAddr};

pub use launcher::{ViewerHandle, ensure_viewer_running, try_ensure_viewer_running};
pub use net_vis::NetVis;
use rocketsim::{Arena, GameMode};

/// Default numeric address for a viewer started through [`ArenaVisExt`].
///
/// The viewer listens at `127.0.0.1:39201`. Use
/// [`ArenaVisExt::try_set_net_vis`] to select a different address.
pub const DEFAULT_VIEWER_ADDR: &str = "127.0.0.1:39201";

/// Return the lowercase name used in viewer commands.
///
/// ```
/// use rocketsim::GameMode;
/// use rocketsim_vis::game_mode_name;
///
/// assert_eq!(game_mode_name(GameMode::TheVoid), "thevoid");
/// ```
pub fn game_mode_name(mode: GameMode) -> &'static str {
    match mode {
        GameMode::Soccar => "soccar",
        GameMode::Hoops => "hoops",
        GameMode::Heatseeker => "heatseeker",
        GameMode::Snowday => "snowday",
        GameMode::Dropshot => "dropshot",
        GameMode::TheVoid => "thevoid",
    }
}

/// Parse a viewer game-mode name without case sensitivity.
///
/// Accept `soccar`, `hoops`, `heatseeker`, `snowday`, `dropshot`, and `thevoid`.
/// Also accept `void` for `TheVoid`. Return `None` for an unknown name. This
/// function does not remove whitespace.
///
/// ```
/// use rocketsim::GameMode;
/// use rocketsim_vis::parse_game_mode;
///
/// assert_eq!(parse_game_mode("Hoops"), Some(GameMode::Hoops));
/// assert_eq!(parse_game_mode("void"), Some(GameMode::TheVoid));
/// assert_eq!(parse_game_mode("unknown"), None);
/// ```
pub fn parse_game_mode(text: &str) -> Option<GameMode> {
    match text.to_lowercase().as_str() {
        "soccar" => Some(GameMode::Soccar),
        "hoops" => Some(GameMode::Hoops),
        "heatseeker" => Some(GameMode::Heatseeker),
        "snowday" => Some(GameMode::Snowday),
        "dropshot" => Some(GameMode::Dropshot),
        "thevoid" | "void" => Some(GameMode::TheVoid),
        _ => None,
    }
}

/// Attach a viewer to an [`Arena`] and receive optional input requests.
///
/// Import this trait to call its methods on an arena. Prefer the `try_set_*`
/// methods to handle setup errors. Methods without `try_` write those errors
/// to standard error.
///
/// An attachment sends new state when the arena steps. It does not apply
/// controls, actions, or mutators. Your application must handle those inputs.
/// See the [crate guide](crate) for complete simulation and control examples.
pub trait ArenaVisExt {
    /// Attach the default viewer, or detach the current viewer.
    ///
    /// With `true`, start or reuse the viewer at [`DEFAULT_VIEWER_ADDR`]. If the
    /// arena already has a visualizer, keep that visualizer and its address.
    /// With `false`, remove the current visualizer and release its process lease.
    /// The background sender transmits frames when the arena steps.
    ///
    /// # Errors
    ///
    /// Return an error if another arena reserves the address, the executable
    /// cannot be found or started, or the viewer does not listen within five
    /// seconds. See [`launcher`] for executable discovery and process ownership.
    fn try_set_vis_enabled(&mut self, enabled: bool) -> io::Result<()>;

    /// Attach or detach a viewer and write setup errors to standard error.
    ///
    /// This is the convenience form of [`Self::try_set_vis_enabled`]. Use the
    /// fallible method when the application must recover from a setup error.
    fn set_vis_enabled(&mut self, enabled: bool) {
        if let Err(error) = self.try_set_vis_enabled(enabled) {
            eprintln!("rocketsim_vis: {error}");
        }
    }

    /// Attach to a numeric `IP:port` address, or detach with `None`.
    ///
    /// This method does not start a viewer or wait for a connection. The sender
    /// connects and retries in the background. It can accept debug updates
    /// before a viewer connects. Call [`Self::vis_connected`] to read its status.
    ///
    /// A new address replaces the current attachment. The same address leaves
    /// it unchanged. `None` removes the current visualizer. A remote address
    /// does not provide authentication or encryption.
    ///
    /// # Errors
    ///
    /// Return [`io::ErrorKind::InvalidInput`] for an address that is not a
    /// numeric socket address. Return [`io::ErrorKind::AddrInUse`] if another
    /// arena in this process reserves the address. Registry errors can also
    /// prevent attachment. A viewer that is absent is not a setup error.
    fn try_set_net_vis(&mut self, addr: Option<&str>) -> io::Result<()>;

    /// Attach without starting a viewer and write setup errors to standard error.
    ///
    /// This is the convenience form of [`Self::try_set_net_vis`].
    fn set_net_vis(&mut self, addr: Option<&str>) {
        if let Err(error) = self.try_set_net_vis(addr) {
            eprintln!("rocketsim_vis: {error}");
        }
    }

    /// Attach to `addr` without starting a viewer.
    ///
    /// Equivalent to `set_net_vis(Some(addr))`. Write setup errors to standard
    /// error. Use [`Self::try_set_net_vis`] to handle errors in the application.
    fn set_net_vis_enabled(&mut self, addr: &str) {
        self.set_net_vis(Some(addr));
    }

    /// Remove this arena's visualizer and release its viewer attachment.
    ///
    /// Equivalent to `set_net_vis(None)`. This does not close a viewer that
    /// the user started. See [`launcher::ViewerHandle`] for process ownership.
    fn set_net_vis_disabled(&mut self) {
        self.set_net_vis(None);
    }

    /// Return the latest control state from this arena's attached viewer.
    ///
    /// Return `None` when no input is available, the connection is lost, or
    /// the input is more than 250 ms old. The returned `car_index` identifies
    /// the requested target. Check it against [`Arena::num_cars`] before use.
    /// This method does not apply controls. Send [`rocketsim::CarControls::default`]
    /// when it returns `None` to release previous driving inputs.
    fn latest_vis_controls(&self) -> Option<proto::ControlMsg>;

    /// Consume pending action requests from the attached viewer.
    ///
    /// Bit 0 requests reset. Bit 1 requests a dribble. Bit 2 requests a ball
    /// launch. Later control messages do not erase pending requests. This
    /// method returns the accumulated bits once, then clears them. Repeated
    /// requests for one action can combine into one bit. Return `0` when no
    /// request is pending. The application must implement each action.
    fn take_vis_actions(&mut self) -> u8;

    /// Replace the small legacy drawing copied into subsequent frames.
    ///
    /// The drawing remains until it is replaced. Pass
    /// [`proto::DebugDraw::default`] to clear it. Ignore invalid drawings and
    /// calls without an attached [`NetVis`].
    ///
    /// Prefer [`debug::DebugRenderer::draw_frame`] for new frame drawings.
    /// Retained groups use [`Self::debug_renderer`] and are not copied into
    /// each simulation frame.
    fn set_vis_debug(&mut self, debug: proto::DebugDraw);

    /// Return a shared debug renderer for this arena's [`NetVis`] attachment.
    ///
    /// Return `None` if the arena has no [`NetVis`]. A viewer does not need to
    /// be connected yet. Accepted updates remain in bounded scene storage
    /// while disconnected or paused. The sender serializes them separately
    /// from simulation ticks.
    ///
    /// Cloned handles share the same scene. Detaching the arena stops its
    /// sender. Old handles still refer to the old scene and do not attach to
    /// a replacement connection. See [`debug`] for drawing examples.
    fn debug_renderer(&self) -> Option<debug::DebugRenderer>;

    /// Return whether the sender has a live viewer connection.
    ///
    /// This is a status value, not proof that a particular frame was displayed.
    /// The value can change immediately after this call. Return `false` when
    /// no [`NetVis`] is attached.
    fn vis_connected(&self) -> bool;
}

fn net_vis(arena: &Arena) -> Option<&NetVis> {
    arena
        .vis
        .as_ref()
        .and_then(|vis| (vis.as_ref() as &dyn Any).downcast_ref::<NetVis>())
}

fn attach(arena: &mut Arena, addr: SocketAddr, launch: bool) -> io::Result<()> {
    if net_vis(arena).is_some_and(|current| current.address().parse() == Ok(addr)) {
        return Ok(());
    }
    let metadata = proto::SceneMetadata::from_arena(arena);
    let lease = launcher::attach(addr, launch)?;
    let mut vis = NetVis::connect(&addr.to_string());
    vis.set_scene_metadata(metadata);
    vis.lease = Some(lease);
    arena.vis = Some(Box::new(vis));
    Ok(())
}

impl ArenaVisExt for Arena {
    fn try_set_vis_enabled(&mut self, enabled: bool) -> io::Result<()> {
        if !enabled {
            self.vis = None;
            return Ok(());
        }
        if self.is_vis_enabled() {
            return Ok(());
        }
        attach(
            self,
            DEFAULT_VIEWER_ADDR.parse().expect("default viewer address"),
            true,
        )
    }

    fn try_set_net_vis(&mut self, addr: Option<&str>) -> io::Result<()> {
        let Some(addr) = addr else {
            self.vis = None;
            return Ok(());
        };
        let addr = addr.parse().map_err(|error| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("invalid numeric viewer address: {error}"),
            )
        })?;
        attach(self, addr, false)
    }

    fn latest_vis_controls(&self) -> Option<proto::ControlMsg> {
        net_vis(self).and_then(NetVis::latest_controls)
    }

    fn take_vis_actions(&mut self) -> u8 {
        net_vis(self).map_or(0, NetVis::take_actions)
    }

    fn set_vis_debug(&mut self, debug: proto::DebugDraw) {
        if let Some(vis) = net_vis(self) {
            vis.set_debug(debug);
        }
    }

    fn debug_renderer(&self) -> Option<debug::DebugRenderer> {
        net_vis(self).map(NetVis::debug_renderer)
    }

    fn vis_connected(&self) -> bool {
        net_vis(self).is_some_and(NetVis::connected)
    }
}

#[cfg(test)]
mod tests {
    use std::{
        net::TcpListener,
        time::{Duration, Instant},
    };

    use super::*;
    use crate::{
        net_client::FrameClient,
        proto::{ControlMsg, DebugDraw, DebugText},
    };

    #[test]
    fn explicit_connections_retarget_without_launching_and_reject_duplicate_arenas() {
        let first = TcpListener::bind("127.0.0.1:0").unwrap();
        let second = TcpListener::bind("127.0.0.1:0").unwrap();
        let first_addr = first.local_addr().unwrap().to_string();
        let second_addr = second.local_addr().unwrap().to_string();
        let mut arena = Arena::new(GameMode::TheVoid);
        let mut other = Arena::new(GameMode::TheVoid);
        arena.try_set_net_vis(Some(&first_addr)).unwrap();
        assert_eq!(net_vis(&arena).unwrap().address(), first_addr);
        arena.try_set_vis_enabled(true).unwrap();
        assert_eq!(net_vis(&arena).unwrap().address(), first_addr);
        assert_eq!(
            other.try_set_net_vis(Some(&first_addr)).unwrap_err().kind(),
            io::ErrorKind::AddrInUse
        );
        other.set_vis_enabled(false);
        assert!(arena.is_vis_enabled());
        arena.try_set_net_vis(Some(&second_addr)).unwrap();
        assert_eq!(net_vis(&arena).unwrap().address(), second_addr);
        other.try_set_net_vis(Some(&first_addr)).unwrap();
        arena.set_vis_enabled(false);
        assert!(other.is_vis_enabled());
    }

    fn wait_for_debug(
        viewer: &FrameClient,
        deadline: Instant,
        mut pred: impl FnMut(&crate::debug::DebugScene) -> bool,
    ) {
        loop {
            if pred(&viewer.debug_scene()) {
                return;
            }
            assert!(
                Instant::now() < deadline,
                "retained debug condition never met"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    fn point_list(x: f32) -> crate::debug::DrawList {
        use crate::debug::{Color, DepthMode, Width};
        let mut list = crate::debug::DrawList::new();
        list.point(
            [x, 0.0, 0.0],
            Color::RED,
            Width::pixels(3.0),
            DepthMode::Test,
        );
        list
    }

    #[test]
    fn retained_delete_survives_skipped_physics_frames() {
        use crate::debug::GroupOptions;
        let viewer = FrameClient::try_connect("127.0.0.1:0").unwrap();
        let addr = viewer.local_addr().to_string();
        let mut arena = Arena::new(GameMode::TheVoid);
        arena.try_set_net_vis(Some(&addr)).unwrap();
        let renderer = arena.debug_renderer().expect("attached");
        // Publish several groups, then drive many rapid ticks (frame slot
        // overwrites/drops) while deleting everything.
        for i in 0..5 {
            renderer
                .replace_group(
                    &format!("g{i}"),
                    point_list(i as f32),
                    GroupOptions::default(),
                )
                .unwrap();
        }
        let deadline = Instant::now() + Duration::from_secs(10);
        // Wait for the first Full so the sender's `sent` set is populated;
        // later deletes then travel as reliable deltas, not dropped frames.
        loop {
            arena.step_tick();
            if viewer.debug_scene().group_count() >= 5 {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "initial debug Full never arrived"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
        for i in 0..5 {
            renderer.clear_group(&format!("g{i}")).unwrap();
        }
        // Burst ticks: latest-only frames drop, debug deletes must not.
        for _ in 0..30 {
            arena.step_tick();
        }
        wait_for_debug(&viewer, Instant::now() + Duration::from_secs(10), |s| {
            s.is_empty()
        });
        // Physics still flows independently.
        assert!(viewer.latest().is_some());
    }

    #[test]
    fn retained_updates_arrive_while_sim_paused() {
        use crate::debug::GroupOptions;
        let viewer = FrameClient::try_connect("127.0.0.1:0").unwrap();
        let addr = viewer.local_addr().to_string();
        let mut arena = Arena::new(GameMode::TheVoid);
        arena.try_set_net_vis(Some(&addr)).unwrap();
        let renderer = arena.debug_renderer().expect("attached");
        // No `step_tick` at all: the sender thread still syncs accepted
        // retained updates (paused sim).
        renderer
            .replace_group("paused", point_list(1.0), GroupOptions::default())
            .unwrap();
        wait_for_debug(&viewer, Instant::now() + Duration::from_secs(10), |s| {
            s.get_group("paused").is_some()
        });
        // Visibility-only update also arrives while paused (no TTL restart).
        renderer.set_group_visible("paused", false).unwrap();
        wait_for_debug(&viewer, Instant::now() + Duration::from_secs(10), |s| {
            s.get_group("paused").is_some_and(|g| !g.visible)
        });
    }

    #[test]
    fn reconnect_resyncs_clean_without_replaying_old_deletes() {
        use crate::debug::GroupOptions;
        let viewer = FrameClient::try_connect("127.0.0.1:0").unwrap();
        let addr = viewer.local_addr().to_string();
        let mut arena = Arena::new(GameMode::TheVoid);
        arena.try_set_net_vis(Some(&addr)).unwrap();
        let renderer = arena.debug_renderer().expect("attached");
        renderer
            .replace_group("stale", point_list(0.0), GroupOptions::default())
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            arena.step_tick();
            if viewer.debug_scene().get_group("stale").is_some() {
                break;
            }
            assert!(Instant::now() < deadline, "stale group never arrived");
            std::thread::sleep(Duration::from_millis(2));
        }
        let gen0 = viewer.generation();
        // Detach: TCP closes, receiver resets to a clean empty scene.
        arena.set_net_vis(None);
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if viewer.generation() > gen0 && viewer.debug_scene().is_empty() {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "receiver never reset on disconnect"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
        // Reattach with only the new group: Full resync carries `fresh`,
        // never replays the old `stale` delete tombstone.
        arena.try_set_net_vis(Some(&addr)).unwrap();
        let renderer = arena.debug_renderer().expect("reattached");
        renderer
            .replace_group("fresh", point_list(9.0), GroupOptions::default())
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            arena.step_tick();
            let scene = viewer.debug_scene();
            if scene.get_group("fresh").is_some() {
                assert!(
                    scene.get_group("stale").is_none(),
                    "stale group must not return"
                );
                break;
            }
            assert!(
                Instant::now() < deadline,
                "fresh group never arrived after reconnect"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    #[test]
    fn retained_rejections_are_typed_and_bounded() {
        use crate::debug::{DebugError, GroupOptions, TextureResource};
        let viewer = FrameClient::try_connect("127.0.0.1:0").unwrap();
        let addr = viewer.local_addr().to_string();
        let mut arena = Arena::new(GameMode::TheVoid);
        arena.try_set_net_vis(Some(&addr)).unwrap();
        let renderer = arena.debug_renderer().expect("attached");
        // Rejected submissions return explicit errors (never silent).
        assert!(matches!(
            renderer.replace_group("bad name!", point_list(0.0), GroupOptions::default()),
            Err(DebugError::InvalidName(_))
        ));
        assert!(matches!(
            TextureResource::new("tex", 0, 16, vec![]),
            Err(DebugError::InvalidTextureDim { .. })
        ));
        // Backpressure bound: oversized texture bytes are rejected before
        // ever reaching the network queue.
        let huge = TextureResource::new("huge", 1024, 1024, vec![0u8; 1024 * 1024 * 4]);
        assert!(huge.is_ok(), "max-size texture is accepted");
        let too_huge = TextureResource::new("too", 1025, 16, vec![0u8; 1025 * 16 * 4]);
        assert!(matches!(
            too_huge,
            Err(DebugError::InvalidTextureDim { .. })
        ));
    }

    #[test]
    fn arena_receives_controls_actions_and_sends_debug_without_gui() {
        let viewer = FrameClient::try_connect("127.0.0.1:0").unwrap();
        let addr = viewer.local_addr().to_string();
        let mut arena = Arena::new(GameMode::TheVoid);
        arena.try_set_net_vis(Some(&addr)).unwrap();
        arena.set_vis_debug(DebugDraw {
            text: vec![DebugText {
                text: "test".into(),
                position: [0.5, 0.5],
                color: [1.0; 4],
            }],
            ..Default::default()
        });
        let deadline = Instant::now() + Duration::from_secs(3);
        loop {
            arena.step_tick();
            if viewer
                .latest()
                .is_some_and(|frame| !frame.debug.text.is_empty())
            {
                break;
            }
            assert!(Instant::now() < deadline, "debug frame did not arrive");
            std::thread::sleep(Duration::from_millis(2));
        }
        assert!(viewer.scene().is_some());
        viewer.send_controls(ControlMsg::new(0, rocketsim::CarControls::default(), 5, 1));
        loop {
            if arena.latest_vis_controls().is_some() {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "controls did not reach the arena"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(arena.take_vis_actions(), 5);
        assert_eq!(arena.take_vis_actions(), 0);
        arena.set_vis_enabled(false);
        assert!(!arena.vis_connected());
    }
}
