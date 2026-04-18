use std::f32::consts::PI;

use glam::{EulerRot, Mat3, Vec3};

use crate::{
    GameMode, consts,
    sim::collision_mesh_file::CollisionMeshFile,
    vis::backend::{Model, ModelSet, Texture, TextureSet},
};

pub fn load_models(game_mode: GameMode, arena_meshes: &[CollisionMeshFile]) -> ModelSet {
    // Make arena model, combining the meshes and the AABB planes
    let arena_model = {
        let arena_meshes_model = Model::from_arena_meshes(arena_meshes);
        let big_plane_model = Model::load_obj(include_str!("models/big_plane.obj"));
        let arena_aabb = consts::arena::get_aabb(game_mode);

        Model::concat(&[
            arena_meshes_model,
            big_plane_model.transform(
                // Floor
                Vec3::new(0.0, 0.0, arena_aabb.min.z),
                Mat3::from_euler(EulerRot::ZYX, 0.0, 0.0, 0.0),
            ),
            big_plane_model.transform(
                // Ceiling
                Vec3::new(0.0, 0.0, arena_aabb.max.z),
                Mat3::from_euler(EulerRot::ZYX, 0.0, 0.0, PI),
            ),
            big_plane_model.transform(
                // Left wall
                Vec3::new(arena_aabb.min.x, 0.0, arena_aabb.center().z),
                Mat3::from_euler(EulerRot::ZYX, 0.0, PI / 2.0, 0.0),
            ),
            big_plane_model.transform(
                // Right wall
                Vec3::new(arena_aabb.max.x, 0.0, arena_aabb.center().z),
                Mat3::from_euler(EulerRot::ZYX, 0.0, -PI / 2.0, 0.0),
            ),
        ])
    };

    let model_pairs = [
        ("ball", Model::load_obj(include_str!("models/ball.obj"))),
        (
            "car_octane",
            Model::load_obj(include_str!("models/car_octane.obj")),
        ),
        (
            "car_dominus",
            Model::load_obj(include_str!("models/car_dominus.obj")),
        ),
        (
            "car_breakout",
            Model::load_obj(include_str!("models/car_breakout.obj")),
        ),
        (
            "car_merc",
            Model::load_obj(include_str!("models/car_merc.obj")),
        ),
        (
            "pad_big_on", // TODO: Rename these model files, the names are confusing
            Model::load_obj(include_str!("models/boost_pad_big_on.obj")),
        ),
        (
            "pad_big_off",
            Model::load_obj(include_str!("models/boost_pad_big_off.obj")),
        ),
        (
            "pad_small_on",
            Model::load_obj(include_str!("models/boost_pad_small_off.obj")),
        ),
        (
            "pad_small_off",
            Model::load_obj(include_str!("models/boost_pad_small_on.obj")),
        ),
        ("arena", arena_model),
    ];
    ModelSet::new(&model_pairs)
}

pub fn load_textures() -> TextureSet {
    let texture_pairs = [
        (
            "ball",
            Texture::load_png(include_bytes!("textures/ball.png")),
        ),
        (
            "car_blue",
            Texture::load_png(include_bytes!("textures/car_blue.png")),
        ),
        (
            "car_orange",
            Texture::load_png(include_bytes!("textures/car_orange.png")),
        ),
        (
            "white",
            Texture::load_png(include_bytes!("textures/white.png")),
        ),
        (
            "gray",
            Texture::load_png(include_bytes!("textures/gray.png")),
        ),
        (
            "boost_pad",
            Texture::load_png(include_bytes!("textures/boost_pad.png")),
        ),
        (
            "boost_glow",
            Texture::load_png(include_bytes!("textures/boost_glow.png")),
        ),
    ];
    TextureSet::new(&texture_pairs)
}
