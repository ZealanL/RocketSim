pub mod backend;
pub mod camera;
mod ribbon_emitter;
pub mod vis_asset_loader;
mod vis_inst;

use rocketsim::Arena;
pub use vis_inst::*;

pub trait ArenaVisExt {
    fn set_vis_enabled(&mut self, vis_enabled: bool);
}

impl ArenaVisExt for Arena {
    fn set_vis_enabled(&mut self, vis_enabled: bool) {
        match (vis_enabled, self.is_vis_enabled()) {
            (true, false) => {
                let game_mode = self.game_mode();
                self.vis = Some(Box::new(VisInst::new(game_mode)));
            }
            (false, true) => {
                self.vis = None;
            }
            _ => {}
        }
    }
}
