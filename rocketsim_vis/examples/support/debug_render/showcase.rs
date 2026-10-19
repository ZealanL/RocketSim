use glam::{Mat3A, Vec2, Vec3A};
use rocketsim_vis::debug::{
    Color, DebugRenderer, DepthMode, DrawList, TextureResource, Width, palette,
};

use super::meshes::{
    insert, normal_or_z, planet_mesh, procedural_mesh, submit_mesh, torus_knot_mesh,
};

pub fn showcase(
    renderer: &DebugRenderer,
    mesh_submit_ms: &mut f64,
) -> Result<(), Box<dyn std::error::Error>> {
    submit_mesh(
        renderer,
        planet_mesh("planet_mesh", palette::TEAM_BLUE)?,
        mesh_submit_ms,
    )?;
    submit_mesh(
        renderer,
        planet_mesh("moon_mesh", palette::ORANGE)?,
        mesh_submit_ms,
    )?;
    submit_mesh(renderer, torus_knot_mesh()?, mesh_submit_ms)?;

    let grid_size = 41usize;
    let mut grid_positions = Vec::with_capacity(grid_size * grid_size);
    let mut grid_colors = Vec::with_capacity(grid_size * grid_size);
    let mut grid_normals = Vec::with_capacity(grid_size * grid_size);
    let mut grid_uvs = Vec::with_capacity(grid_size * grid_size);
    let mut grid_indices = Vec::with_capacity((grid_size - 1) * (grid_size - 1) * 6);
    for y in 0..grid_size {
        for x in 0..grid_size {
            let u = x as f32 / (grid_size - 1) as f32;
            let v = y as f32 / (grid_size - 1) as f32;
            let height = (u * std::f32::consts::TAU * 3.0).sin()
                * (v * std::f32::consts::TAU * 2.0).cos()
                * 65.0;
            grid_positions.push(Vec3A::new(
                -1450.0 + u * 2900.0,
                -880.0 + v * 1760.0,
                100.0 + height,
            ));
            let heat = ((height + 65.0) / 130.0).clamp(0.0, 1.0);
            grid_colors.push(Color::rgba(
                0.12 + 0.8 * heat,
                0.24 + 0.5 * (1.0 - heat),
                0.9 - 0.55 * heat,
                0.9,
            ));
            grid_normals.push(normal_or_z(Vec3A::new(
                -0.15 * (u * std::f32::consts::TAU * 3.0).cos(),
                0.12 * (v * std::f32::consts::TAU * 2.0).sin(),
                1.0,
            )));
            grid_uvs.push(Vec2::new(u, v));
            if x + 1 < grid_size && y + 1 < grid_size {
                let a = (y * grid_size + x) as u32;
                let b = a + 1;
                let c = a + grid_size as u32;
                let d = c + 1;
                grid_indices.extend([a, b, c, b, d, c]);
            }
        }
    }
    renderer.insert_mesh(procedural_mesh(
        "heat_surface",
        grid_positions,
        grid_colors,
        grid_normals,
        grid_uvs,
        grid_indices,
    )?)?;
    let mut texels = Vec::with_capacity(64 * 64 * 4);
    for y in 0..64 {
        for x in 0..64 {
            let wave = (x as f32 * 0.19).sin() * (y as f32 * 0.16).cos() * 0.5 + 0.5;
            let checker = if (x / 8 + y / 8) % 2 == 0 { 0.78 } else { 0.34 };
            texels.extend([
                (35.0 + 180.0 * wave * checker) as u8,
                (60.0 + 145.0 * (1.0 - wave) * checker) as u8,
                (120.0 + 120.0 * wave) as u8,
                255,
            ]);
        }
    }
    renderer.insert_texture(TextureResource::new("heatmap_tex", 64, 64, texels)?)?;

    let mut scene = DrawList::new();
    scene.instance(
        "heat_surface",
        [0.0, 0.0, 0.0],
        Mat3A::IDENTITY.to_cols_array_2d(),
        [1.0; 3],
        Color::WHITE,
        Some("heatmap_tex".into()),
        DepthMode::Test,
    );
    scene.sphere_solid(
        [0.0, 0.0, 520.0],
        220.0,
        palette::CYAN.with_alpha(0.78),
        DepthMode::Test,
    );
    scene.sphere_lines(
        [0.0, 0.0, 520.0],
        245.0,
        palette::WHITE,
        Width::pixels(1.5),
        DepthMode::Always,
    );
    scene.box_solid(
        [-420.0, -200.0, 20.0],
        [-180.0, 40.0, 260.0],
        palette::ORANGE.with_alpha(0.65),
        DepthMode::Xray,
    );
    scene.box_edges(
        [-420.0, -200.0, 20.0],
        [-180.0, 40.0, 260.0],
        palette::ORANGE,
        Width::pixels(3.0),
        DepthMode::Test,
    );
    let spiral: Vec<[f32; 3]> = (0..1800)
        .map(|i| {
            let t = i as f32 * 0.045;
            let r = 500.0 + 140.0 * (t / 9.0).sin();
            [r * t.cos(), r * t.sin(), 80.0 + i as f32 * 0.55]
        })
        .collect();
    scene.polyline(
        spiral,
        palette::MAGENTA,
        Width::pixels(2.0),
        DepthMode::Test,
        false,
    );
    let mut rng = 0x12345678u32;
    for i in 0..1400 {
        rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
        let x = (rng as i32 as f32 / i32::MAX as f32) * 2100.0;
        rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
        let y = (rng as i32 as f32 / i32::MAX as f32) * 1200.0;
        scene.point(
            [x, y, 20.0 + (i % 9) as f32 * 16.0],
            palette::rainbow(i as f32 / 1400.0),
            Width::pixels(3.0),
            DepthMode::Test,
        );
    }
    scene.label_world(
        "RETAINED DEBUG SCENE",
        [0.0, -1250.0, 40.0],
        26.0,
        rocketsim_vis::debug::TextAlign::Center,
        Color::WHITE,
        None,
    );
    scene.label_screen(
        "DEBUG RENDER  /  retained resources",
        [0.5, 0.04],
        20.0,
        rocketsim_vis::debug::TextAlign::Center,
        Color::WHITE,
        Some(Color::BLACK.with_alpha(0.65)),
    );
    scene.axes(
        [0.0, 0.0, 4.0],
        240.0,
        Width::pixels(3.0),
        DepthMode::Always,
    );
    insert(renderer, "showcase", scene)?;

    let mut orbiters = DrawList::new();
    orbiters.instance(
        "planet_mesh",
        [1050.0, 0.0, 390.0],
        Mat3A::IDENTITY.to_cols_array_2d(),
        [1.0; 3],
        Color::WHITE,
        None,
        DepthMode::Test,
    );
    orbiters.instance(
        "moon_mesh",
        [1320.0, 0.0, 470.0],
        Mat3A::IDENTITY.to_cols_array_2d(),
        [0.48; 3],
        Color::WHITE,
        None,
        DepthMode::Test,
    );
    insert(renderer, "orbiters", orbiters)?;

    let mut knot = DrawList::new();
    knot.instance(
        "knot_mesh",
        [0.0, 0.0, 0.0],
        Mat3A::IDENTITY.to_cols_array_2d(),
        [1.0; 3],
        Color::WHITE,
        None,
        DepthMode::Test,
    );
    insert(renderer, "knot", knot)
}
