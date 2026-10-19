//! CPU-side mesh and geometry construction.

pub mod frustum;
pub mod lines;
pub mod mesh;
pub mod obj;
pub mod pads;

#[cfg(test)]
pub(crate) fn init_collision_meshes() {
    use std::{path::Path, sync::OnceLock};

    static INIT: OnceLock<()> = OnceLock::new();
    INIT.get_or_init(|| {
        if rocketsim::is_initialized() {
            return;
        }
        if let Some(dir) = std::env::var_os("ROCKETSIM_MESHES_DIR") {
            rocketsim::init(dir, true).unwrap();
            return;
        }
        for candidate in [
            "./collision_meshes/",
            "../collision_meshes/",
            "../../collision_meshes/",
        ] {
            if Path::new(candidate).join("soccar").exists() {
                rocketsim::init(candidate, true).unwrap();
                return;
            }
        }
        rocketsim::init_from_default(true).unwrap();
    });
}
