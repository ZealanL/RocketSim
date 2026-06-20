pub mod backend;
pub mod camera;
mod ribbon_emitter;
pub mod vis_asset_loader;
mod vis_inst;

use rocketsim::{Arena, GameMode, get_arena_collision_mesh_files};
pub use vis_inst::*;

pub trait ArenaVisExt {
    fn set_vis_enabled(&mut self, vis_enabled: bool);
}

impl ArenaVisExt for Arena {
    fn set_vis_enabled(&mut self, vis_enabled: bool) {
        match (vis_enabled, self.get_vis_enabled()) {
            (true, false) => {
                let game_mode = self.game_mode();
                let mesh_game_mode = match game_mode {
                    GameMode::Heatseeker | GameMode::Snowday => GameMode::Soccar,
                    _ => game_mode,
                };

                let game_mode_mesh_files = get_arena_collision_mesh_files(mesh_game_mode);
                self.set_vis(Some(Box::new(VisInst::new(
                    game_mode,
                    game_mode_mesh_files.as_slice(),
                ))));
            }
            (false, true) => {
                let _ = self.take_vis();
            }
            _ => {}
        }
    }
}
