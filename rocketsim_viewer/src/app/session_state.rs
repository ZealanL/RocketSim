use std::sync::Arc;

use rocketsim_vis::{
    game_mode_name,
    proto::{SceneMetadata, VisFrame},
};

use super::{camera::CameraState, startup::metadata_changed};
use crate::geometry::mesh::ArenaMesh;

pub(super) struct SessionState {
    generation: Option<u64>,
    pub(super) mesh: Arc<ArenaMesh>,
    pub(super) soup_verts: usize,
    pub(super) welded_verts: usize,
    pub(super) n_indices: usize,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct SceneUpdate {
    pub(super) generation_changed: bool,
    pub(super) mode_changed: bool,
    pub(super) rebuild_arena: bool,
}

impl SessionState {
    pub(super) fn new(mesh: Arc<ArenaMesh>) -> Self {
        let mut state = Self {
            generation: None,
            mesh,
            soup_verts: 0,
            welded_verts: 0,
            n_indices: 0,
        };
        state.update_mesh_stats();
        state
    }

    fn observe_generation(&mut self, generation: u64) -> bool {
        let changed = self
            .generation
            .is_some_and(|previous| previous != generation);
        self.generation = Some(generation);
        changed
    }

    fn update_mesh_stats(&mut self) {
        self.soup_verts = self.mesh.soup_verts;
        self.welded_verts = self.mesh.welded_verts();
        self.n_indices = self.mesh.index_count();
    }

    pub(super) fn update_scene(
        &mut self,
        generation: u64,
        frame: Option<&VisFrame>,
        scene_metadata: &mut Option<Arc<SceneMetadata>>,
        camera: &mut CameraState,
    ) -> SceneUpdate {
        let generation_changed = self.observe_generation(generation);
        if matches!(
            (frame, scene_metadata.as_ref()),
            (Some(frame), Some(metadata)) if frame.state.game_mode() != metadata.game_mode
        ) {
            *scene_metadata = None;
        }

        if generation_changed {
            camera.reset_session();
            self.mesh = Arc::new(ArenaMesh::placeholder(self.mesh.mode));
            self.update_mesh_stats();
        }

        let metadata_changed = scene_metadata
            .as_ref()
            .is_some_and(|metadata| metadata_changed(camera.last_metadata.as_ref(), metadata));
        if let Some(metadata) = scene_metadata.as_ref().filter(|_| metadata_changed) {
            self.mesh = Arc::new(ArenaMesh::from_meshes(metadata.game_mode, &metadata.meshes));
            self.update_mesh_stats();
            camera.bounds_for_scene = camera
                .bounds
                .get_for_meshes(metadata.game_mode, &metadata.meshes);
            camera.last_metadata = Some(metadata.clone());
        }

        let mut mode_changed = false;
        if let Some(frame) = frame
            && frame.state.game_mode() != self.mesh.mode
        {
            let mode = frame.state.game_mode();
            self.mesh = Arc::new(
                scene_metadata
                    .as_ref()
                    .filter(|metadata| metadata.game_mode == mode)
                    .map(|metadata| ArenaMesh::from_meshes(mode, &metadata.meshes))
                    .unwrap_or_else(|| ArenaMesh::placeholder(mode)),
            );
            self.update_mesh_stats();
            if let Some(metadata) = scene_metadata
                .as_ref()
                .filter(|metadata| metadata.game_mode == mode)
            {
                camera.bounds_for_scene = camera.bounds.get_for_meshes(mode, &metadata.meshes);
                camera.last_metadata = Some(metadata.clone());
            } else {
                camera.bounds_for_scene = None;
                camera.last_metadata = None;
            }
            mode_changed = true;
            eprintln!(
                "rocketsim-viewer: game mode switched to {}",
                game_mode_name(mode)
            );
        }

        SceneUpdate {
            generation_changed,
            mode_changed,
            rebuild_arena: generation_changed || metadata_changed || mode_changed,
        }
    }
}

#[cfg(test)]
mod tests {
    use rocketsim::GameMode;

    use super::*;

    #[test]
    fn generation_changes_reset_for_reconnects_and_rewinds() {
        let mesh = Arc::new(ArenaMesh::placeholder(GameMode::Soccar));
        let mut session = SessionState::new(mesh);
        assert!(!session.observe_generation(7));
        assert!(!session.observe_generation(7));
        assert!(session.observe_generation(8));
        assert!(session.observe_generation(7));
    }
}
