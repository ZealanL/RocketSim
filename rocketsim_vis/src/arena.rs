//! Attach viewer transport and arena metadata to a RocketSim arena.
//!
//! Import [`ArenaVisExt`] to attach or detach a viewer. This module does not
//! launch a process by itself. Use the crate-level helpers to start a viewer.

use std::{any::Any, io, net::SocketAddr};

use rocketsim::Arena;

use crate::{NetVis, proto::SceneMetadata};

/// Attach a viewer to an [`Arena`] and read its optional input requests.
///
/// The simulation must apply controls, actions, and mutators. This trait does
/// not apply viewer input. Use the fallible methods to handle setup errors.
pub trait ArenaVisExt {
    /// Attach the default viewer or detach the current viewer.
    ///
    /// With `true`, start or reuse the viewer at [`crate::DEFAULT_VIEWER_ADDR`].
    /// With `false`, remove the visualizer and release its process lease.
    ///
    /// # Errors
    ///
    /// Return an error if the address is in use, the viewer cannot start, or
    /// the viewer does not listen within five seconds.
    fn try_set_vis_enabled(&mut self, enabled: bool) -> io::Result<()>;

    /// Attach or detach a viewer and write setup errors to standard error.
    fn set_vis_enabled(&mut self, enabled: bool) {
        if let Err(error) = self.try_set_vis_enabled(enabled) {
            eprintln!("rocketsim_vis: {error}");
        }
    }

    /// Attach to a numeric socket address or detach with `None`.
    ///
    /// This method does not start a viewer or wait for a connection. The sender
    /// retries in the background. A remote address has no authentication or
    /// encryption.
    ///
    /// # Errors
    ///
    /// Return `InvalidInput` for a non-numeric address and `AddrInUse` if
    /// another arena in this process has reserved the address.
    fn try_set_net_vis(&mut self, addr: Option<&str>) -> io::Result<()>;

    /// Attach without starting a viewer and write setup errors to standard error.
    fn set_net_vis(&mut self, addr: Option<&str>) {
        if let Err(error) = self.try_set_net_vis(addr) {
            eprintln!("rocketsim_vis: {error}");
        }
    }

    /// Attach to `addr` without starting a viewer.
    fn set_net_vis_enabled(&mut self, addr: &str) {
        self.set_net_vis(Some(addr));
    }

    /// Remove this arena visualizer.
    fn set_net_vis_disabled(&mut self) {
        self.set_net_vis(None);
    }

    /// Return fresh controls from the attached viewer.
    ///
    /// Return `None` when input is absent, the connection is lost, or input is
    /// more than 250 milliseconds old. Check `car_index` before use. Apply
    /// neutral controls when this returns `None`.
    fn latest_vis_controls(&self) -> Option<crate::proto::ControlMsg>;

    /// Consume pending action requests from the viewer.
    ///
    /// Bit 0 requests reset, bit 1 requests a dribble, and bit 2 requests a
    /// ball launch. Return `0` when no request is pending. The application
    /// must perform each action.
    fn take_vis_actions(&mut self) -> u8;

    /// Return a shared retained-scene renderer for this arena.
    ///
    /// Return `None` when no [`NetVis`] is attached. Updates remain available
    /// while disconnected or paused. Detaching stops the sender. Cloned handles
    /// continue to refer to the detached scene.
    fn debug_renderer(&self) -> Option<crate::debug::DebugRenderer>;

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
    let metadata = SceneMetadata::from_arena(arena);
    let lease = crate::launcher::attach(addr, launch)?;
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
            crate::DEFAULT_VIEWER_ADDR
                .parse()
                .expect("default viewer address"),
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

    fn latest_vis_controls(&self) -> Option<crate::proto::ControlMsg> {
        net_vis(self).and_then(NetVis::latest_controls)
    }

    fn take_vis_actions(&mut self) -> u8 {
        net_vis(self).map_or(0, NetVis::take_actions)
    }

    fn debug_renderer(&self) -> Option<crate::debug::DebugRenderer> {
        net_vis(self).map(NetVis::debug_renderer)
    }

    fn vis_connected(&self) -> bool {
        net_vis(self).is_some_and(NetVis::connected)
    }
}

impl SceneMetadata {
    /// Copy collision meshes and the configured ball radius from an arena.
    ///
    /// Convert collision vertices to Unreal units. This operation can allocate
    /// a large buffer. Arena attachment calls it once during setup.
    ///
    /// # Panics
    ///
    /// Panic if loaded collision meshes fail [`Self::validate`].
    pub fn from_arena(arena: &Arena) -> Self {
        let game_mode = arena.get_arena_state().game_mode();
        let meshes = rocketsim::get_arena_collision_mesh_files(game_mode)
            .into_iter()
            .map(|mesh| crate::proto::MeshData {
                vertices: mesh
                    .get_vertices()
                    .iter()
                    .map(|vertex| (vertex * rocketsim::consts::BT_TO_UU).to_array())
                    .collect(),
                indices: mesh
                    .get_indices()
                    .iter()
                    .map(|&index| u32::try_from(index).unwrap_or(u32::MAX))
                    .collect(),
            })
            .collect();
        let metadata = Self {
            game_mode,
            ball_radius: arena.mutator_config().ball_radius,
            meshes,
        };
        assert!(
            metadata.validate(),
            "loaded arena meshes contain invalid data"
        );
        metadata
    }
}

#[cfg(test)]
mod tests {
    use std::{
        net::TcpListener,
        time::{Duration, Instant},
    };

    use rocketsim::{Arena, GameMode};

    use super::*;
    use crate::{
        debug::{DebugError, DebugScene, GroupOptions, TextureResource},
        transport::viewer::ViewerListener,
    };

    #[test]
    fn explicit_connections_retarget_without_launching_and_reject_duplicate_arenas() {
        test_trace!("explicit connections start");
        let first = TcpListener::bind("127.0.0.1:0").unwrap();
        let second = TcpListener::bind("127.0.0.1:0").unwrap();
        let first_addr = first.local_addr().unwrap().to_string();
        let second_addr = second.local_addr().unwrap().to_string();
        test_trace!("create arena");
        let mut arena = Arena::new(GameMode::TheVoid);
        test_trace!("arena created");
        test_trace!("create second arena");
        let mut other = Arena::new(GameMode::TheVoid);
        test_trace!("second arena created");
        test_trace!("attach first address");
        arena.try_set_net_vis(Some(&first_addr)).unwrap();
        test_trace!("attached first address");
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
        viewer: &ViewerListener,
        deadline: Instant,
        mut predicate: impl FnMut(&DebugScene) -> bool,
    ) {
        loop {
            let snapshot = viewer.snapshot();
            if predicate(&snapshot.debug) {
                return;
            }
            assert!(
                Instant::now() < deadline,
                "retained scene condition not met"
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
    fn retained_deletions_survive_skipped_physics_frames() {
        test_trace!("retained deletions start");
        let viewer = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        test_trace!("create arena");
        let mut arena = Arena::new(GameMode::TheVoid);
        test_trace!("arena created");
        test_trace!("attach viewer");
        arena
            .try_set_net_vis(Some(&viewer.local_addr().to_string()))
            .unwrap();
        test_trace!("viewer attached");
        let renderer = arena.debug_renderer().expect("renderer attached");
        for index in 0..5 {
            renderer
                .replace_group(
                    &format!("g{index}"),
                    point_list(index as f32),
                    GroupOptions::default(),
                )
                .unwrap();
        }
        let deadline = Instant::now() + Duration::from_secs(10);
        test_trace!("wait for initial scene");
        loop {
            arena.step_tick();
            if viewer.snapshot().debug.group_count() >= 5 {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "initial full scene did not arrive"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
        test_trace!("initial scene received");
        for index in 0..5 {
            renderer.clear_group(&format!("g{index}")).unwrap();
        }
        for _ in 0..30 {
            arena.step_tick();
        }
        test_trace!("wait for deletions");
        wait_for_debug(
            &viewer,
            Instant::now() + Duration::from_secs(10),
            DebugScene::is_empty,
        );
        test_trace!("deletions received");
        assert!(viewer.snapshot().frame.is_some());
        test_trace!("test body complete; dropping local values");
    }

    #[test]
    fn retained_updates_arrive_while_simulation_is_paused() {
        test_trace!("paused updates start");
        let viewer = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        test_trace!("create arena");
        let mut arena = Arena::new(GameMode::TheVoid);
        test_trace!("arena created");
        test_trace!("attach viewer");
        arena
            .try_set_net_vis(Some(&viewer.local_addr().to_string()))
            .unwrap();
        test_trace!("viewer attached");
        let renderer = arena.debug_renderer().expect("renderer attached");
        renderer
            .replace_group("paused", point_list(1.0), GroupOptions::default())
            .unwrap();
        test_trace!("wait for paused group");
        wait_for_debug(&viewer, Instant::now() + Duration::from_secs(10), |scene| {
            scene.get_group("paused").is_some()
        });
        test_trace!("paused group received");
        renderer.set_group_visible("paused", false).unwrap();
        test_trace!("wait for visibility update");
        wait_for_debug(&viewer, Instant::now() + Duration::from_secs(10), |scene| {
            scene
                .get_group("paused")
                .is_some_and(|group| !group.visible)
        });
        test_trace!("visibility update received");
        assert!(viewer.snapshot().frame.is_none());
        test_trace!("test body complete; dropping local values");
    }

    #[test]
    fn reconnect_sends_clean_full_scene_without_old_deletes() {
        test_trace!("reconnect start");
        let viewer = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        let addr = viewer.local_addr().to_string();
        test_trace!("create arena");
        let mut arena = Arena::new(GameMode::TheVoid);
        test_trace!("arena created");
        test_trace!("attach viewer");
        arena.try_set_net_vis(Some(&addr)).unwrap();
        test_trace!("viewer attached");
        arena
            .debug_renderer()
            .unwrap()
            .replace_group("stale", point_list(0.0), GroupOptions::default())
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        test_trace!("wait for first scene");
        loop {
            arena.step_tick();
            if viewer.snapshot().debug.get_group("stale").is_some() {
                break;
            }
            assert!(Instant::now() < deadline, "first scene did not arrive");
            std::thread::sleep(Duration::from_millis(2));
        }
        test_trace!("first scene received");
        let generation = viewer.snapshot().generation;
        test_trace!("detach viewer");
        arena.set_net_vis(None);
        test_trace!("viewer detached");
        test_trace!("wait for reset");
        let deadline = Instant::now() + Duration::from_secs(5);
        while viewer.snapshot().generation <= generation {
            assert!(
                Instant::now() < deadline,
                "viewer did not reset on disconnect"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
        test_trace!("viewer reset");
        test_trace!("reattach viewer");
        arena.try_set_net_vis(Some(&addr)).unwrap();
        test_trace!("viewer reattached");
        arena
            .debug_renderer()
            .unwrap()
            .replace_group("fresh", point_list(9.0), GroupOptions::default())
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        test_trace!("wait for fresh scene");
        loop {
            arena.step_tick();
            let snapshot = viewer.snapshot();
            if snapshot.debug.get_group("fresh").is_some() {
                assert!(snapshot.debug.get_group("stale").is_none());
                break;
            }
            assert!(Instant::now() < deadline, "fresh full scene did not arrive");
            std::thread::sleep(Duration::from_millis(2));
        }
        test_trace!("fresh scene received; dropping local values");
    }

    #[test]
    fn retained_rejections_are_typed_and_bounded() {
        test_trace!("retained rejections start");
        let viewer = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        test_trace!("create arena");
        let mut arena = Arena::new(GameMode::TheVoid);
        test_trace!("arena created");
        test_trace!("attach viewer");
        arena
            .try_set_net_vis(Some(&viewer.local_addr().to_string()))
            .unwrap();
        test_trace!("viewer attached");
        let renderer = arena.debug_renderer().expect("renderer attached");
        assert!(matches!(
            renderer.replace_group("bad name!", point_list(0.0), GroupOptions::default()),
            Err(DebugError::InvalidName(_))
        ));
        assert!(matches!(
            TextureResource::new("tex", 0, 16, vec![]),
            Err(DebugError::InvalidTextureDim { .. })
        ));
        let huge = TextureResource::new("huge", 1024, 1024, vec![0u8; 1024 * 1024 * 4]);
        assert!(huge.is_ok());
        let too_huge = TextureResource::new("too", 1025, 16, vec![0u8; 1025 * 16 * 4]);
        assert!(matches!(
            too_huge,
            Err(DebugError::InvalidTextureDim { .. })
        ));
    }

    #[test]
    fn arena_receives_controls_and_accumulates_action_pulses() {
        test_trace!("controls start");
        let viewer = ViewerListener::try_bind("127.0.0.1:0").unwrap();
        test_trace!("create arena");
        let mut arena = Arena::new(GameMode::TheVoid);
        test_trace!("arena created");
        test_trace!("attach viewer");
        arena
            .try_set_net_vis(Some(&viewer.local_addr().to_string()))
            .unwrap();
        test_trace!("viewer attached");
        let deadline = Instant::now() + Duration::from_secs(3);
        test_trace!("wait for first frame");
        while viewer.snapshot().frame.is_none() {
            arena.step_tick();
            assert!(Instant::now() < deadline, "frame did not arrive");
            std::thread::sleep(Duration::from_millis(2));
        }
        test_trace!("first frame received");
        test_trace!("send controls");
        assert!(viewer.send_controls(crate::proto::ControlMsg::new(
            0,
            rocketsim::CarControls::default(),
            1,
            1,
        )));
        assert!(viewer.send_controls(crate::proto::ControlMsg::new(
            0,
            rocketsim::CarControls::default(),
            4,
            2,
        )));
        test_trace!("wait for controls");
        loop {
            if arena
                .latest_vis_controls()
                .is_some_and(|message| message.seq == 2)
            {
                break;
            }
            assert!(Instant::now() < deadline, "controls did not reach arena");
            std::thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(arena.take_vis_actions(), 5);
        test_trace!("controls received");
        assert_eq!(arena.take_vis_actions(), 0);
        test_trace!("detach viewer");
        arena.set_vis_enabled(false);
        test_trace!("viewer detached");
        assert!(!arena.vis_connected());
        test_trace!("test body complete; dropping local values");
    }
}
