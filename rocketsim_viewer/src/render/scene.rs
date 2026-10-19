//! Owned GL resources for one viewer scene.

use std::collections::HashMap;

use rocketsim::GameMode;

use crate::{
    effects::dropshot::DropshotTilesGl,
    geometry::{
        mesh::ArenaMesh,
        obj::ObjMesh,
        pads::{load_pad_meshes, variant_radius},
    },
    render::{
        arena::ArenaGl,
        debug::DebugGl,
        lines::LinesGl,
        main::{InstancedModelGl, MainInstProgram, MainProgram, ModelGl},
        pads::PadsGl,
        textures::{PngImage, TextureGl},
    },
};

/// GL resources for a mode. Destroy this object on its GL context thread.
pub struct SceneGl {
    /// Mode uploaded to this scene.
    pub mode: GameMode,
    /// Arena renderer.
    pub arena: ArenaGl,
    /// Textured model program.
    pub prog_main: MainProgram,
    /// Instanced actor program.
    pub prog_cars: MainInstProgram,
    /// Boost-pad renderer.
    pub pads: PadsGl,
    /// Dynamic Dropshot tile renderer.
    pub dropshot: DropshotTilesGl,
    /// Pad culling radii.
    pub pad_radii: [f32; 4],
    /// Debug-line renderer.
    pub lines: LinesGl,
    /// Retained debug scene renderer and GPU cache.
    pub debug: DebugGl,
    /// Uploaded static models.
    pub models: HashMap<String, ModelGl>,
    /// Uploaded instanced actor models.
    pub car_models: HashMap<String, InstancedModelGl>,
    /// Uploaded textures.
    pub textures: HashMap<String, TextureGl>,
}

impl SceneGl {
    /// Creates a complete scene from prebuilt arena geometry.
    pub fn new(gl: &glow::Context, mesh: &ArenaMesh) -> Self {
        use glow::HasContext;
        unsafe {
            eprintln!(
                "rocketsim-viewer: GPU {} / {}",
                gl.get_parameter_string(glow::RENDERER),
                gl.get_parameter_string(glow::VERSION)
            );
        }
        let pad_meshes = load_pad_meshes();
        let pad_radii = [
            variant_radius(&pad_meshes[0]),
            variant_radius(&pad_meshes[1]),
            variant_radius(&pad_meshes[2]),
            variant_radius(&pad_meshes[3]),
        ];
        let pads = PadsGl::new(gl, &pad_meshes);
        let mut models = HashMap::new();
        models.insert(
            crate::render::main::BALL_MODEL.to_string(),
            ModelGl::new(gl, &ObjMesh::load(include_str!("../models/ball.obj"))),
        );
        models.insert(
            crate::render::main::PUCK_MODEL.to_string(),
            ModelGl::new(gl, &ObjMesh::puck()),
        );
        let mut textures = HashMap::new();
        for (name, bytes) in [
            (
                crate::render::main::BALL_TEXTURE,
                include_bytes!("../textures/ball.png").as_slice(),
            ),
            (
                "car_blue",
                include_bytes!("../textures/car_blue.png").as_slice(),
            ),
            (
                "car_orange",
                include_bytes!("../textures/car_orange.png").as_slice(),
            ),
        ] {
            textures.insert(
                name.to_string(),
                TextureGl::new(gl, &PngImage::decode(bytes)),
            );
        }
        let mut car_models = HashMap::new();
        for (name, obj) in [
            ("car_octane", include_str!("../models/car_octane.obj")),
            ("car_dominus", include_str!("../models/car_dominus.obj")),
            ("car_breakout", include_str!("../models/car_breakout.obj")),
            ("car_merc", include_str!("../models/car_merc.obj")),
        ] {
            car_models.insert(
                name.to_string(),
                InstancedModelGl::new(gl, &ObjMesh::load(obj)),
            );
        }
        Self {
            mode: mesh.mode,
            arena: ArenaGl::new(gl, mesh),
            prog_main: MainProgram::new(gl),
            prog_cars: MainInstProgram::new(gl),
            pads,
            dropshot: DropshotTilesGl::new(gl),
            pad_radii,
            lines: LinesGl::new(gl),
            debug: DebugGl::new(gl),
            models,
            car_models,
            textures,
        }
    }

    /// Replaces arena buffers and retains all other scene resources.
    pub fn rebuild_arena(&mut self, gl: &glow::Context, mesh: &ArenaMesh) {
        self.arena.destroy(gl);
        self.arena = ArenaGl::new(gl, mesh);
        self.mode = mesh.mode;
    }

    /// Deletes all owned GL resources. Call once, before dropping, on the GL thread.
    pub fn destroy(&self, gl: &glow::Context) {
        self.arena.destroy(gl);
        self.prog_main.destroy(gl);
        self.prog_cars.destroy(gl);
        self.pads.destroy(gl);
        self.dropshot.destroy(gl);
        self.lines.destroy(gl);
        self.debug.destroy(gl);
        for model in self.models.values() {
            model.destroy(gl);
        }
        for model in self.car_models.values() {
            model.destroy(gl);
        }
        for texture in self.textures.values() {
            texture.destroy(gl);
        }
    }
}
