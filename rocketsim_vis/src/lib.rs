//! Stream a RocketSim arena to the detached `rocketsim-viewer` process.
//!
//! Frames, scene geometry, debug drawing, and controls use one versioned TCP
//! connection. This crate does not depend on the GUI stack.
//!
//! Install or build `rocketsim-viewer` before calling
//! [`ArenaVisExt::set_vis_enabled`]. Use [`ArenaVisExt::set_net_vis`] to connect
//! to a viewer that you start separately. Addresses must be numeric `IP:port`
//! socket addresses. Each viewer accepts one arena.
//!
//! ```no_run
//! use rocketsim::{Arena, CarBodyConfig, GameMode, Team, init_from_default};
//! use rocketsim_vis::ArenaVisExt;
//!
//! init_from_default(true).unwrap();
//! let mut arena = Arena::new(GameMode::Soccar);
//! arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
//! arena.reset_to_random_kickoff(None);
//! arena.try_set_vis_enabled(true)?;
//! for _ in 0..240 {
//!     arena.step_tick();
//!     std::thread::sleep(std::time::Duration::from_millis(8));
//! }
//! arena.set_vis_enabled(false);
//! # Ok::<(), std::io::Error>(())
//! ```

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

/// Default endpoint for the detached viewer.
pub const DEFAULT_VIEWER_ADDR: &str = "127.0.0.1:39201";

/// Return the canonical lowercase game-mode name.
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

/// Parse a game-mode name without case sensitivity. `void` is an alias for `thevoid`.
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

/// Attach the viewer to an arena and receive optional driving inputs.
///
/// These methods never apply controls to the simulation. The simulation
/// decides whether to apply controls, actions, and mutators.
pub trait ArenaVisExt {
    /// Enable or disable the default viewer. Return startup errors to the caller.
    /// Calling this with the current enabled state does nothing.
    fn try_set_vis_enabled(&mut self, enabled: bool) -> io::Result<()>;

    /// Enable or disable the default viewer. Startup errors are written to stderr.
    fn set_vis_enabled(&mut self, enabled: bool) {
        if let Err(error) = self.try_set_vis_enabled(enabled) {
            eprintln!("rocketsim_vis: {error}");
        }
    }

    /// Connect to an explicit numeric address, or detach with `None`.
    /// This method never launches a process. A new address replaces the old connection.
    /// A second arena in this process cannot reserve the same viewer address.
    fn try_set_net_vis(&mut self, addr: Option<&str>) -> io::Result<()>;

    /// Connect without launching a process. Address errors are written to stderr.
    fn set_net_vis(&mut self, addr: Option<&str>) {
        if let Err(error) = self.try_set_net_vis(addr) {
            eprintln!("rocketsim_vis: {error}");
        }
    }

    /// Connect to the viewer at `addr` without launching it.
    fn set_net_vis_enabled(&mut self, addr: &str) {
        self.set_net_vis(Some(addr));
    }

    /// Detach this arena from its viewer.
    fn set_net_vis_disabled(&mut self) {
        self.set_net_vis(None);
    }

    /// Return the latest inputs, or `None` on disconnect or after 250 ms without inputs.
    fn latest_vis_controls(&self) -> Option<proto::ControlMsg>;

    /// Consume pending actions: bit 0 resets play, bit 1 dribbles, and bit 2 launches.
    /// Actions remain pending across later analog updates until consumed.
    fn take_vis_actions(&mut self) -> u8;

    /// Replace the debug drawing included in subsequent frames.
    ///
    /// Legacy per-tick frame drawings (small, latest-only). Retained scene
    /// drawings use [`debug_renderer`](Self::debug_renderer) instead and are
    /// never cloned into ticks.
    fn set_vis_debug(&mut self, debug: proto::DebugDraw);

    /// Cloneable retained-scene handle, or `None` when no viewer is attached.
    ///
    /// Accepted updates coalesce in bounded tables even while disconnected
    /// or while the sim is paused; the sender thread serializes off the sim
    /// tick. Short API locks only; the network path stays nonblocking.
    fn debug_renderer(&self) -> Option<debug::DebugRenderer>;

    /// Return whether the sender currently has a viewer connection.
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
