//! Retained debug-render API demo. Use `--drive` to explore a Soccar arena.
//!
//! Example: `cargo run --locked -p rocketsim_vis --example debug_render -- --addr 127.0.0.1:34254 --seconds 20 --mode showcase`

#[path = "support/drive.rs"]
mod drive_controls;
mod support;

use std::{
    collections::HashSet,
    fs,
    net::SocketAddr,
    path::PathBuf,
    time::{Duration, Instant},
};

use clap::{Parser, ValueEnum};
use drive_controls::Driver;
use glam::{Mat3A, Vec2, Vec3A, Vec3Swizzles};
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, GameMode, Team};
use rocketsim_vis::{
    ArenaVisExt,
    debug::{
        Color, DebugRenderer, DepthMode, DrawList, GroupOptions, MeshResource, TextureResource,
        Width, palette,
    },
    pace::TickPacer,
    photo::init_meshes,
};

#[derive(Clone, Copy, Debug, ValueEnum)]
enum Mode {
    Wire,
    Solid,
    Showcase,
    Stress,
}

#[derive(Parser)]
#[command(name = "debug_render", version)]
struct Cli {
    /// Connect to an existing viewer. Optional with --drive.
    #[arg(long, required_unless_present = "drive")]
    addr: Option<SocketAddr>,
    /// Stop after this many seconds. Default: 15, or no limit with --drive.
    #[arg(long, value_parser = parse_duration)]
    seconds: Option<Duration>,
    /// Add a drivable Octane and use a Soccar arena. Requires collision meshes.
    #[arg(long)]
    drive: bool,
    #[arg(long, value_enum, default_value_t = Mode::Showcase)]
    mode: Mode,
    #[arg(long, default_value_t = 10000)]
    count: usize,
    #[arg(long)]
    obj: Option<PathBuf>,
    #[arg(long = "static")]
    static_scene: bool,
    #[arg(long)]
    original_placement: bool,
    #[arg(long)]
    benchmark_output: Option<PathBuf>,
}

impl Cli {
    fn run_duration(&self) -> Option<Duration> {
        self.seconds
            .or_else(|| (!self.drive).then(|| Duration::from_secs(15)))
    }
}

fn parse_duration(text: &str) -> Result<Duration, String> {
    let seconds = text
        .parse::<f64>()
        .map_err(|_| "--seconds must be positive and finite".to_owned())?;
    Duration::try_from_secs_f64(seconds)
        .ok()
        .filter(|duration| !duration.is_zero())
        .ok_or_else(|| "--seconds must be positive, finite, and within range".to_owned())
}

#[derive(Clone, Debug)]
struct ObjFace {
    corners: Vec<Corner>,
    color: Color,
}
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
struct Corner {
    v: usize,
    vt: Option<usize>,
    vn: Option<usize>,
}
#[derive(Clone, Debug)]
struct ObjData {
    positions: Vec<Vec3A>,
    uvs: Vec<Vec2>,
    normals: Vec<Vec3A>,
    faces: Vec<ObjFace>,
}

fn obj_index(s: &str, len: usize) -> Result<usize, String> {
    let i: i64 = s.parse().map_err(|_| format!("invalid OBJ index '{s}'"))?;
    let resolved = if i > 0 {
        i - 1
    } else if i < 0 {
        len as i64 + i
    } else {
        return Err("OBJ indices cannot be zero".into());
    };
    if resolved < 0 || resolved as usize >= len {
        return Err(format!("OBJ index {i} is out of range for {len} values"));
    }
    Ok(resolved as usize)
}

fn parse_obj(src: &str, original_placement: bool) -> Result<ObjData, String> {
    let mut out = ObjData {
        positions: vec![],
        uvs: vec![],
        normals: vec![],
        faces: vec![],
    };
    let mut color = Color::WHITE;
    for (line_no, line) in src.lines().enumerate() {
        let mut fields = line.split_whitespace();
        let Some(tag) = fields.next() else { continue };
        let vals: Vec<_> = fields.collect();
        let err = |e: String| format!("OBJ line {}: {e}", line_no + 1);
        match tag {
            "v" => {
                if vals.len() < 3 {
                    return Err(err("vertex needs 3 coordinates".into()));
                }
                let x = parse_f32(vals[0]).map_err(err)?;
                let y = parse_f32(vals[1]).map_err(err)?;
                let z = parse_f32(vals[2]).map_err(err)?;
                let mut position = Vec3A::new(x, y, z).zxy() * 70.0;
                if original_placement {
                    position.x -= 3500.0;
                }
                out.positions.push(position);
            }
            "vt" => {
                if vals.len() < 2 {
                    return Err(err("texture coordinate needs 2 values".into()));
                }
                out.uvs.push(Vec2::new(
                    parse_f32(vals[0]).map_err(err)?,
                    parse_f32(vals[1]).map_err(err)?,
                ));
            }
            "vn" => {
                if vals.len() < 3 {
                    return Err(err("normal needs 3 values".into()));
                }
                out.normals.push(Vec3A::new(
                    parse_f32(vals[0]).map_err(err)?,
                    parse_f32(vals[1]).map_err(err)?,
                    parse_f32(vals[2]).map_err(err)?,
                ));
            }
            "o" | "g" => {
                if let Some(name) = vals.first() {
                    color = color_from_name(name);
                }
            }
            "f" => {
                if vals.len() < 3 {
                    return Err(err("face needs at least 3 corners".into()));
                }
                let mut corners = Vec::with_capacity(vals.len());
                for item in vals {
                    let mut parts = item.split('/');
                    let v =
                        obj_index(parts.next().unwrap_or(""), out.positions.len()).map_err(err)?;
                    let vt = parts
                        .next()
                        .filter(|s| !s.is_empty())
                        .map(|s| obj_index(s, out.uvs.len()))
                        .transpose()
                        .map_err(err)?;
                    let vn = parts
                        .next()
                        .filter(|s| !s.is_empty())
                        .map(|s| obj_index(s, out.normals.len()))
                        .transpose()
                        .map_err(err)?;
                    if parts.next().is_some() {
                        return Err(err("invalid face corner".into()));
                    }
                    corners.push(Corner { v, vt, vn });
                }
                out.faces.push(ObjFace { corners, color });
            }
            _ => {}
        }
    }
    if out.positions.is_empty() || out.faces.is_empty() {
        return Err("OBJ has no vertices or faces".into());
    }
    if !original_placement {
        let mut min = Vec3A::splat(f32::INFINITY);
        let mut max = Vec3A::splat(f32::NEG_INFINITY);
        for &position in &out.positions {
            min = min.min(position);
            max = max.max(position);
        }
        let center = (min + max) * 0.5;
        let origin = Vec3A::new(center.x, center.y, min.z);
        for position in &mut out.positions {
            *position -= origin;
        }
    }
    Ok(out)
}
fn parse_f32(s: &str) -> Result<f32, String> {
    let n: f32 = s.parse().map_err(|_| format!("invalid number '{s}'"))?;
    if n.is_finite() {
        Ok(n)
    } else {
        Err("non-finite number".into())
    }
}
fn color_from_name(name: &str) -> Color {
    let Some((_, hex)) = name.rsplit_once('_') else {
        return Color::WHITE;
    };
    if hex.len() != 6 {
        return Color::WHITE;
    }
    let Ok(rgb) = u32::from_str_radix(hex, 16) else {
        return Color::WHITE;
    };
    Color::rgb(
        ((rgb >> 16) & 255) as f32 / 255.0,
        ((rgb >> 8) & 255) as f32 / 255.0,
        (rgb & 255) as f32 / 255.0,
    )
}

fn make_mesh(data: &ObjData) -> Result<MeshResource, Box<dyn std::error::Error>> {
    let mut positions = Vec::new();
    let mut colors = Vec::new();
    let mut normals = Vec::new();
    let mut uvs = Vec::new();
    let mut indices = Vec::new();
    let mut remap = std::collections::HashMap::<(Corner, [u8; 4]), u32>::new();
    for face in &data.faces {
        let mut face_ids = Vec::with_capacity(face.corners.len());
        let a = data.positions[face.corners[0].v];
        let b = data.positions[face.corners[1].v];
        let c = data.positions[face.corners[2].v];
        let face_normal = normal_or_z((b - a).cross(c - a));
        for c in &face.corners {
            let key = (*c, face.color.to_array().map(|v| (v * 255.0).round() as u8));
            let id = *remap.entry(key).or_insert_with(|| {
                let id = positions.len() as u32;
                positions.push(data.positions[c.v].to_array());
                colors.push(face.color);
                normals.push(
                    c.vn.map(|n| normal_or_z(data.normals[n].zxy()))
                        .unwrap_or(face_normal)
                        .to_array(),
                );
                uvs.push(c.vt.map(|t| data.uvs[t]).unwrap_or(Vec2::ZERO).to_array());
                id
            });
            face_ids.push(id);
        }
        for i in 1..face_ids.len() - 1 {
            indices.extend([face_ids[0], face_ids[i], face_ids[i + 1]]);
        }
    }
    Ok(MeshResource::new(
        "imported_obj",
        positions,
        colors,
        Some(normals),
        Some(uvs),
        indices,
    )?)
}
// Keep the importer's existing cutoff for degenerate normals.
fn normal_or_z(normal: Vec3A) -> Vec3A {
    let length = normal.length();
    if length > 1e-6 {
        normal / length
    } else {
        Vec3A::Z
    }
}

fn insert(
    renderer: &DebugRenderer,
    name: &str,
    list: DrawList,
) -> Result<(), Box<dyn std::error::Error>> {
    renderer.replace_group(name, list, GroupOptions::default())?;
    Ok(())
}

fn rotation_z(angle: f32) -> [[f32; 3]; 3] {
    Mat3A::from_rotation_z(angle).transpose().to_cols_array_2d()
}

fn procedural_mesh(
    id: &str,
    positions: Vec<Vec3A>,
    colors: Vec<Color>,
    normals: Vec<Vec3A>,
    uvs: Vec<Vec2>,
    indices: Vec<u32>,
) -> Result<MeshResource, Box<dyn std::error::Error>> {
    Ok(MeshResource::new(
        id,
        positions.into_iter().map(|v| v.to_array()).collect(),
        colors,
        Some(normals.into_iter().map(|v| v.to_array()).collect()),
        Some(uvs.into_iter().map(|v| v.to_array()).collect()),
        indices,
    )?)
}

fn submit_mesh(
    renderer: &DebugRenderer,
    mesh: MeshResource,
    elapsed_ms: &mut f64,
) -> Result<(), Box<dyn std::error::Error>> {
    let start = Instant::now();
    renderer.insert_mesh(mesh)?;
    *elapsed_ms += start.elapsed().as_secs_f64() * 1000.0;
    Ok(())
}

fn planet_mesh(id: &str, color: Color) -> Result<MeshResource, Box<dyn std::error::Error>> {
    let t = (1.0 + 5.0_f32.sqrt()) * 0.5;
    let raw = [
        [-1.0, t, 0.0],
        [1.0, t, 0.0],
        [-1.0, -t, 0.0],
        [1.0, -t, 0.0],
        [0.0, -1.0, t],
        [0.0, 1.0, t],
        [0.0, -1.0, -t],
        [0.0, 1.0, -t],
        [t, 0.0, -1.0],
        [t, 0.0, 1.0],
        [-t, 0.0, -1.0],
        [-t, 0.0, 1.0],
    ];
    let faces: [[u32; 3]; 20] = [
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    let positions: Vec<_> = raw
        .into_iter()
        .map(|v| normal_or_z(Vec3A::from_array(v)) * 105.0)
        .collect();
    let normals: Vec<_> = positions.iter().map(|&p| normal_or_z(p)).collect();
    let colors = normals
        .iter()
        .map(|normal| {
            let light = (0.55 + 0.45 * normal.z.max(0.0)).clamp(0.0, 1.0);
            Color::rgba(color.r * light, color.g * light, color.b * light, 1.0)
        })
        .collect();
    let uvs = positions
        .iter()
        .map(|p| p.truncate() / 210.0 + Vec2::splat(0.5))
        .collect();
    let indices = faces.into_iter().flatten().collect();
    procedural_mesh(id, positions, colors, normals, uvs, indices)
}

fn torus_knot_mesh() -> Result<MeshResource, Box<dyn std::error::Error>> {
    let around = 96usize;
    let sides = 12usize;
    let mut positions = Vec::with_capacity(around * sides);
    let mut colors = Vec::with_capacity(around * sides);
    let mut normals = Vec::with_capacity(around * sides);
    let mut uvs = Vec::with_capacity(around * sides);
    let mut indices = Vec::with_capacity(around * sides * 6);
    for i in 0..around {
        let u = i as f32 / around as f32 * std::f32::consts::TAU;
        let (sin_2u, cos_2u) = (2.0 * u).sin_cos();
        let (sin_3u, cos_3u) = (3.0 * u).sin_cos();
        let center = Vec3A::new(
            520.0 * cos_2u + 130.0 * cos_3u,
            520.0 * sin_2u - 130.0 * sin_3u,
            560.0 + 150.0 * sin_3u,
        );
        let tangent = normal_or_z(Vec3A::new(
            -1040.0 * sin_2u - 390.0 * sin_3u,
            1040.0 * cos_2u - 390.0 * cos_3u,
            450.0 * cos_3u,
        ));
        let side = normal_or_z(Vec3A::Z.cross(tangent));
        let up = normal_or_z(tangent.cross(side));
        for j in 0..sides {
            let v = j as f32 / sides as f32 * std::f32::consts::TAU;
            let (sin_v, cos_v) = v.sin_cos();
            let n = normal_or_z(side * cos_v + up * sin_v);
            positions.push(center + n * 52.0);
            normals.push(n);
            colors.push(palette::rainbow(i as f32 / around as f32));
            uvs.push(Vec2::new(i as f32 / around as f32, j as f32 / sides as f32));
            let a = (i * sides + j) as u32;
            let b = (i * sides + (j + 1) % sides) as u32;
            let c = (((i + 1) % around) * sides + j) as u32;
            let d = (((i + 1) % around) * sides + (j + 1) % sides) as u32;
            indices.extend([a, b, c, b, d, c]);
        }
    }
    procedural_mesh("knot_mesh", positions, colors, normals, uvs, indices)
}

fn showcase(
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
        DepthMode::Always,
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
        DepthMode::Always,
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
            DepthMode::Always,
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    if cli.count > rocketsim_vis::debug::MAX_TOTAL_ITEMS {
        return Err(format!(
            "--count cannot exceed {}",
            rocketsim_vis::debug::MAX_TOTAL_ITEMS
        )
        .into());
    }
    let _viewer = if cli.addr.is_none() {
        support::prepare_viewer()?
    } else {
        None
    };
    let mode = if cli.drive {
        init_meshes();
        GameMode::Soccar
    } else {
        GameMode::TheVoid
    };
    let mut arena = Arena::new_with_config(ArenaConfig::new(mode).with_rng_seed(0));
    let mut driver = cli.drive.then(|| {
        let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        arena.reset_to_random_kickoff(None);
        Driver::new(car)
    });
    if let Some(addr) = cli.addr {
        arena.try_set_net_vis(Some(&addr.to_string()))?;
    } else {
        arena.try_set_vis_enabled(true)?;
    }
    if cli.drive {
        println!("Close settings with Escape. Press 1 to follow and drive the Octane.");
        println!("Press F for free camera. Use W/A/S/D and the mouse to move.");
        println!("Reset, dribble, and launch use Backspace, F2, and F4 by default.");
        println!("Debug geometry has no collision. Press Ctrl+C in this terminal to stop.");
    }
    let renderer = arena
        .debug_renderer()
        .ok_or("debug renderer unavailable after attach")?;
    let build_start = Instant::now();
    let mut mesh_submit_ms = 0.0;
    match cli.mode {
        Mode::Showcase => showcase(&renderer, &mut mesh_submit_ms)?,
        Mode::Wire | Mode::Solid => {
            if let Some(path) = &cli.obj {
                let text = fs::read_to_string(path)?;
                let data = parse_obj(&text, cli.original_placement)?;
                if matches!(cli.mode, Mode::Solid) {
                    let mesh = make_mesh(&data)?;
                    let bytes = mesh.estimated_bytes();
                    submit_mesh(&renderer, mesh, &mut mesh_submit_ms)?;
                    let mut list = DrawList::new();
                    list.instance(
                        "imported_obj",
                        [0.0, 0.0, 0.0],
                        Mat3A::IDENTITY.to_cols_array_2d(),
                        [1.0; 3],
                        Color::WHITE,
                        None,
                        DepthMode::Test,
                    );
                    insert(&renderer, "obj_solid", list)?;
                    eprintln!(
                        "OBJ solid: {} positions, {} faces, resource {:.2} MiB",
                        data.positions.len(),
                        data.faces.len(),
                        bytes as f64 / 1048576.0
                    );
                } else {
                    let mut edges = std::collections::BTreeMap::<
                        [u8; 4],
                        (Color, HashSet<(usize, usize)>),
                    >::new();
                    for face in &data.faces {
                        let key = face.color.to_array().map(|v| (v * 255.0).round() as u8);
                        let entry = edges
                            .entry(key)
                            .or_insert_with(|| (face.color, HashSet::new()));
                        for i in 0..face.corners.len() {
                            let a = face.corners[i].v;
                            let b = face.corners[(i + 1) % face.corners.len()].v;
                            entry.1.insert((a.min(b), a.max(b)));
                        }
                    }
                    let mut group_id = 0usize;
                    for (_, (color, edges)) in edges {
                        let mut sorted_edges: Vec<_> = edges.into_iter().collect();
                        sorted_edges.sort_unstable();
                        let mut list = DrawList::with_capacity(4096);
                        for (a, b) in sorted_edges {
                            if list.len() == 4096 {
                                insert(&renderer, &format!("obj_wire_{group_id}"), list)?;
                                group_id += 1;
                                list = DrawList::with_capacity(4096);
                            }
                            list.line(
                                data.positions[a].to_array(),
                                data.positions[b].to_array(),
                                color,
                                Width::pixels(1.2),
                                DepthMode::Always,
                            );
                        }
                        if !list.is_empty() {
                            insert(&renderer, &format!("obj_wire_{group_id}"), list)?;
                            group_id += 1;
                        }
                    }
                    eprintln!(
                        "OBJ wire: {} positions, {} polygon boundaries",
                        data.positions.len(),
                        data.faces.len()
                    );
                }
                let mut guides = DrawList::new();
                guides.grid(
                    [0.0, 0.0, 2.0],
                    1600.0,
                    400.0,
                    Color::rgba(0.45, 0.52, 0.62, 0.5),
                    Width::pixels(1.0),
                    DepthMode::Test,
                );
                guides.label_screen(
                    "OBJ DEBUG RENDER",
                    [0.5, 0.04],
                    20.0,
                    rocketsim_vis::debug::TextAlign::Center,
                    Color::WHITE,
                    Some(Color::BLACK.with_alpha(0.65)),
                );
                insert(&renderer, "obj_guides", guides)?;
            } else {
                return Err("--obj PATH is required for wire and solid modes".into());
            }
        }
        Mode::Stress => {
            let n = cli.count;
            let item_budget = n.saturating_sub(1);
            let point_count = item_budget / 2;
            let instance_count = item_budget - point_count;
            for start in (0..point_count).step_by(4096) {
                let mut batch = DrawList::with_capacity((point_count - start).min(4096));
                for i in start..(start + 4096).min(point_count) {
                    let a = i as f32 * 0.017;
                    batch.point(
                        [
                            a.cos() * 1800.0,
                            a.sin() * 1000.0,
                            80.0 + (i % 200) as f32 * 4.0,
                        ],
                        palette::rainbow(i as f32 / n.max(1) as f32),
                        Width::pixels(2.0),
                        DepthMode::Always,
                    );
                }
                insert(&renderer, &format!("stress_points_{start}"), batch)?;
            }
            let mesh = MeshResource::new(
                "stress_marker",
                vec![[-20.0, -20.0, 0.0], [20.0, -20.0, 0.0], [0.0, 20.0, 36.0]],
                vec![palette::CYAN; 3],
                Some(vec![[0.0, 0.0, 1.0]; 3]),
                None,
                vec![0, 1, 2],
            )?;
            submit_mesh(&renderer, mesh, &mut mesh_submit_ms)?;
            for start in (0..instance_count).step_by(4096) {
                let mut instances = DrawList::with_capacity((instance_count - start).min(4096));
                for i in start..(start + 4096).min(instance_count) {
                    let a = i as f32 * 0.041;
                    instances.instance(
                        "stress_marker",
                        [
                            a.cos() * 2400.0,
                            a.sin() * 1450.0,
                            80.0 + (i % 80) as f32 * 8.0,
                        ],
                        Mat3A::IDENTITY.to_cols_array_2d(),
                        [1.0; 3],
                        palette::rainbow(i as f32 / n.max(1) as f32),
                        None,
                        DepthMode::Test,
                    );
                }
                insert(&renderer, &format!("stress_instances_{start}"), instances)?;
            }
            let mut paths = DrawList::new();
            let line_points: Vec<_> = (0..n.min(4096))
                .map(|i| {
                    let x = -2200.0 + i as f32 * 4400.0 / n.max(1) as f32;
                    [x, -1250.0, 20.0 + (i % 2) as f32 * 40.0]
                })
                .collect();
            if n > 0 && line_points.len() >= 2 {
                paths.polyline(
                    line_points,
                    palette::LIME,
                    Width::pixels(1.0),
                    DepthMode::Always,
                    false,
                );
            }
            insert(&renderer, "stress_lines", paths)?;
        }
    }
    let build_ms = build_start.elapsed().as_secs_f64() * 1000.0;
    let scene = renderer.snapshot();
    let scene_items: usize = scene.groups.values().map(|g| g.items.len()).sum();
    let estimated_bytes = scene
        .meshes
        .values()
        .map(|mesh| mesh.estimated_bytes())
        .sum::<usize>()
        + scene
            .textures
            .values()
            .map(|texture| texture.estimated_bytes())
            .sum::<usize>()
        + scene
            .groups
            .values()
            .map(|g| g.estimated_bytes())
            .sum::<usize>();
    let mesh_bytes: usize = scene
        .meshes
        .values()
        .map(|mesh| mesh.estimated_bytes())
        .sum();
    let texture_bytes: usize = scene
        .textures
        .values()
        .map(|texture| texture.estimated_bytes())
        .sum();
    let mut animated_groups: Vec<String> = match cli.mode {
        Mode::Showcase => vec!["orbiters".into(), "knot".into()],
        Mode::Solid => vec!["obj_solid".into()],
        Mode::Wire => scene
            .groups
            .keys()
            .filter(|name| name.starts_with("obj_wire_"))
            .cloned()
            .collect(),
        Mode::Stress => scene
            .groups
            .keys()
            .filter(|name| name.starts_with("stress_instances_"))
            .cloned()
            .collect(),
    };
    animated_groups.sort();
    eprintln!("READY {} groups, {scene_items} items", scene.groups.len());
    let start = Instant::now();
    let deadline = cli
        .run_duration()
        .map(|duration| start.checked_add(duration).ok_or("--seconds is too large"))
        .transpose()?;
    let mut pacer = TickPacer::at_120hz();
    let mut frames = 0u64;
    let mut update_latencies_ms = Vec::new();
    let mut update_batches = 0u64;
    let mut last_update = Instant::now();
    let mut angle = 0.0f32;
    while deadline.is_none_or(|deadline| Instant::now() < deadline) {
        for _ in 0..pacer.wait() {
            if deadline.is_some_and(|deadline| Instant::now() >= deadline) {
                break;
            }
            if let Some(driver) = &mut driver {
                driver.tick(&mut arena);
            } else {
                arena.edit_ball_state(|ball| {
                    ball.phys.pos = Vec3A::new(0.0, 0.0, 900.0);
                    ball.phys.vel = Vec3A::ZERO;
                });
                arena.step_tick();
            }
            frames += 1;
        }
        if !cli.static_scene
            && last_update.elapsed() >= Duration::from_millis(33)
            && !animated_groups.is_empty()
        {
            angle += 0.035;
            let rotation = rotation_z(angle);
            let transform = rocketsim_vis::debug::GroupTransform {
                position: [0.0; 3],
                rotation,
                scale: [1.0; 3],
            };
            let t = Instant::now();
            for name in &animated_groups {
                renderer.set_group_transform(name, transform.clone())?;
            }
            update_latencies_ms.push(t.elapsed().as_secs_f64() * 1000.0);
            update_batches += 1;
            last_update = Instant::now();
        }
    }
    let run_ms = start.elapsed().as_secs_f64() * 1000.0;
    update_latencies_ms.sort_by(f64::total_cmp);
    let percentile = |p: f64| -> f64 {
        if update_latencies_ms.is_empty() {
            0.0
        } else {
            update_latencies_ms[((update_latencies_ms.len() - 1) as f64 * p).ceil() as usize]
        }
    };
    let update_mean_ms = if update_latencies_ms.is_empty() {
        0.0
    } else {
        update_latencies_ms.iter().sum::<f64>() / update_latencies_ms.len() as f64
    };
    let summary = format!(
        "{{\"mode\":\"{:?}\",\"frames\":{},\"groups\":{},\"items\":{},\"mesh_bytes\":{},\"texture_bytes\":{},\"resource_bytes\":{},\"build_ms\":{:.3},\"mesh_submit_ms\":{:.3},\"run_ms\":{:.3},\"update_batches\":{},\"update_mean_ms\":{:.3},\"update_p50_ms\":{:.3},\"update_p95_ms\":{:.3},\"update_max_ms\":{:.3},\"static\":{},\"drive\":{}}}",
        cli.mode,
        frames,
        scene.groups.len(),
        scene_items,
        mesh_bytes,
        texture_bytes,
        estimated_bytes,
        build_ms,
        mesh_submit_ms,
        run_ms,
        update_batches,
        update_mean_ms,
        percentile(0.50),
        percentile(0.95),
        update_latencies_ms.last().copied().unwrap_or(0.0),
        cli.static_scene,
        cli.drive
    );
    println!("SUMMARY {summary}");
    if let Some(path) = cli.benchmark_output {
        fs::write(path, format!("{summary}\n"))?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn debug_only_defaults_still_require_a_viewer_and_stop_after_15_seconds() {
        assert!(Cli::try_parse_from(["debug_render"]).is_err());
        let cli = Cli::try_parse_from(["debug_render", "--addr", "127.0.0.1:34254"]).unwrap();
        assert!(!cli.drive);
        assert_eq!(cli.run_duration(), Some(Duration::from_secs(15)));
    }

    #[test]
    fn drive_can_launch_a_viewer_and_runs_until_stopped() {
        let cli = Cli::try_parse_from(["debug_render", "--drive"]).unwrap();
        assert!(cli.drive);
        assert!(cli.addr.is_none());
        assert!(cli.run_duration().is_none());
    }

    #[test]
    fn drive_accepts_an_existing_viewer_and_a_time_limit() {
        let cli = Cli::try_parse_from([
            "debug_render",
            "--drive",
            "--addr",
            "127.0.0.1:34254",
            "--seconds",
            "1.5",
            "--static",
        ])
        .unwrap();
        assert!(cli.addr.is_some());
        assert!(cli.static_scene);
        assert_eq!(cli.run_duration(), Some(Duration::from_millis(1500)));
    }

    #[test]
    fn invalid_durations_return_errors() {
        for seconds in ["0", "-1", "NaN", "inf", "1e100", "invalid"] {
            assert!(parse_duration(seconds).is_err(), "{seconds}");
        }
    }

    #[test]
    fn parses_mixed_obj_faces_indices_and_color() {
        let src = "o trim_12aBcD\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvn 0 0 1\nf 1/1/1 2/2/1 3//1 4/1/1\n";
        let obj = parse_obj(src, false).unwrap();
        assert_eq!(obj.faces.len(), 1);
        assert_eq!(obj.faces[0].corners.len(), 4);
        assert_eq!(
            obj.faces[0].color,
            Color::rgb(
                0x12 as f32 / 255.0,
                0xab as f32 / 255.0,
                0xcd as f32 / 255.0
            )
        );
        let mesh = make_mesh(&obj).unwrap();
        assert_eq!(mesh.indices.len(), 6);
        assert_eq!(mesh.positions.len(), 4);
    }
    #[test]
    fn handles_negative_indices_and_original_coordinate_transform() {
        let obj = parse_obj("v 1 2 3\nv 2 2 3\nv 1 3 3\nf -3 -2 -1\n", true).unwrap();
        assert_eq!(obj.positions[0], Vec3A::new(-3290.0, 70.0, 140.0));
        assert!(parse_obj("v 0 0 0\nf 0 1 1\n", false).is_err());
    }

    #[test]
    fn recenters_default_obj_and_preserves_each_object_color() {
        let obj = parse_obj(
            "o red_FF0000\nv 10 0 0\nv 11 0 0\nv 10 1 0\nf 1 2 3\no blue_0000FF\nf 1 3 2\n",
            false,
        )
        .unwrap();
        assert_eq!(obj.faces[0].color, Color::rgb(1.0, 0.0, 0.0));
        assert_eq!(obj.faces[1].color, Color::rgb(0.0, 0.0, 1.0));
        assert!((obj.positions.iter().map(|p| p.x).sum::<f32>() / 3.0).abs() < 0.001);
        assert_eq!(
            obj.positions
                .iter()
                .map(|p| p.z)
                .fold(f32::INFINITY, f32::min),
            0.0
        );
    }

    #[test]
    fn normal_fallback_preserves_degenerate_cutoff() {
        assert_eq!(normal_or_z(Vec3A::ZERO), Vec3A::Z);
        assert_eq!(normal_or_z(Vec3A::X * 0.5e-6), Vec3A::Z);
        assert_eq!(normal_or_z(Vec3A::X * 2.0e-6), Vec3A::X);
        assert!(
            normal_or_z(Vec3A::new(0.0, 3.0, 4.0)).abs_diff_eq(Vec3A::new(0.0, 0.6, 0.8), 1e-6)
        );
    }

    #[test]
    fn rotation_stays_row_major_and_turns_x_toward_y() {
        let rows = rotation_z(std::f32::consts::FRAC_PI_2);
        assert!(Vec3A::from_array(rows[0]).abs_diff_eq(Vec3A::NEG_Y, 1e-6));
        let rotation = Mat3A::from_cols_array_2d(&rows).transpose();
        assert!((rotation * Vec3A::X).abs_diff_eq(Vec3A::Y, 1e-6));
    }

    #[test]
    fn imported_and_computed_normals_use_the_same_axis_mapping() {
        let vertices = "v 0 0 0\nv 1 0 0\nv 0 1 0\n";
        for faces in ["f 1 2 3\n", "vn 0 0 1\nf 1//1 2//1 3//1\n"] {
            let obj = parse_obj(&format!("{vertices}{faces}"), false).unwrap();
            let mesh = make_mesh(&obj).unwrap();
            for normal in mesh.normals.unwrap() {
                assert_eq!(Vec3A::from_array(normal), Vec3A::X);
            }
        }
    }

    #[test]
    fn procedural_meshes_keep_their_topology_and_unit_normals() {
        for (mesh, vertices, indices) in [
            (planet_mesh("planet", Color::WHITE).unwrap(), 12, 60),
            (torus_knot_mesh().unwrap(), 96 * 12, 96 * 12 * 6),
        ] {
            mesh.validate().unwrap();
            assert_eq!(mesh.positions.len(), vertices);
            assert_eq!(mesh.indices.len(), indices);
            for normal in mesh.normals.unwrap() {
                assert!((Vec3A::from_array(normal).length() - 1.0).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn malformed_obj_indices_and_numbers_return_errors() {
        for source in [
            "v 0 0 0\nf 1 2 3\n",
            "v 0 0 0\nv NaN 0 0\nf 1 1 1\n",
            "v 0 0 0\nf 1/99 1 1\n",
            "v 0 0 0\nf -2 -1 -1\n",
        ] {
            assert!(
                parse_obj(source, false).is_err(),
                "accepted invalid OBJ: {source}"
            );
        }
    }
}
