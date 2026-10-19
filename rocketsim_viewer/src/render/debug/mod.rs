//! GPU cache for the retained debug scene. Geometry and texture assets are
//! uploaded only when their public revisions change.
use std::{collections::HashMap, sync::Arc, time::Instant};

use glow::HasContext;
use rocketsim::ArenaState;
use rocketsim_vis::debug::{DebugItem, DebugScene, DepthMode, MeshResource};

use crate::render::gl_util::{bytes_of, compile_program, uniform_location};

use self::{
    order::{InstanceRun, OrderedInstance},
    prepare::{append_instance, depth_key, depth_mode, pack_segment},
    resources::{delete_mesh, upload_mesh, upload_ordered_instances},
    transform::instance_matrix,
};

mod cache;
mod draw;
mod lines;
pub mod order;
pub mod overlay;
mod prepare;
mod resources;
pub mod transform;

pub use cache::{BuiltinMesh, BuiltinTexture};
pub use lines::{DEBUG_LINE_FRAG_SRC, DEBUG_LINE_VERT_SRC, LineBatch, LineProgram, PackedLine};
pub use overlay::OverlayCache;
pub use transform::resolve_group_matrix;

pub const DEBUG_MESH_VERT_SRC: &str = include_str!("../../shaders/debug_mesh_vert.glsl");
pub const DEBUG_MESH_FRAG_SRC: &str = include_str!("../../shaders/debug_mesh_frag.glsl");

#[derive(Clone, Copy, Debug, Default, serde::Serialize)]
pub struct DebugStats {
    pub mesh_uploads: u64,
    pub texture_uploads: u64,
    pub upload_bytes: u64,
    pub draw_calls: u64,
    pub instances: u64,
    pub primitives: u64,
    pub culled: u64,
    pub line_uploads: u64,
    pub segments: u64,
    pub points: u64,
    pub cpu_ms: f64,
    pub gpu_ms: Option<f64>,
}

struct GpuMesh {
    vao: glow::VertexArray,
    buffers: [glow::Buffer; 4],
    index: glow::Buffer,
    count: i32,
    revision: u64,
    builtin: bool,
    transparent: bool,
    center: glam::Vec3A,
}

#[derive(Clone, Copy)]
struct InstanceData {
    transform: glam::Mat4,
    tint: [f32; 4],
    source_index: usize,
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
struct Material {
    mesh: String,
    texture: Option<String>,
}

struct PreparedGroup {
    group: Arc<rocketsim_vis::debug::DebugGroup>,
    transform: glam::Mat4,
}

struct Batch {
    mesh: String,
    texture: Option<String>,
    depth: DepthMode,
    buffer: glow::Buffer,
    count: i32,
    tint_opaque: bool,
    instances: Vec<InstanceData>,
}
struct SegmentBatch {
    gpu: LineBatch,
    depth: DepthMode,
    points: u64,
}

struct GpuTexture {
    handle: glow::Texture,
    revision: u64,
    builtin: bool,
    transparent: bool,
}

/// Retained GPU resource cache. Call `destroy` on the context thread.
pub struct DebugGl {
    program: glow::Program,
    view: glow::UniformLocation,
    proj: glow::UniformLocation,
    sampler: glow::UniformLocation,
    textured: glow::UniformLocation,
    group: glow::UniformLocation,
    alpha: glow::UniformLocation,
    meshes: HashMap<String, GpuMesh>,
    textures: HashMap<String, GpuTexture>,
    resource_sync: cache::ResourceSyncIdentity,
    batches: HashMap<String, Vec<Batch>>,
    reusable_instance_buffers: Vec<glow::Buffer>,
    geometry_revisions: HashMap<String, u64>,
    line_batches: HashMap<String, Vec<SegmentBatch>>,
    reusable_line_batches: Vec<LineBatch>,
    line_program: LineProgram,
    profiling: bool,
    timers: Option<[glow::Query; 3]>,
    timer_pending: bool,
    measure_frame: bool,
    view_matrix: glam::Mat4,
    projection_matrix: glam::Mat4,
    viewport: [f32; 2],
    prepared_groups: Vec<PreparedGroup>,
    group_scratch: Vec<PreparedGroup>,
    ordered_buffer: glow::Buffer,
    ordered_capacity: usize,
    ordered_data: Vec<f32>,
    ordered_scratch: Vec<f32>,
    ordered_runs: Vec<InstanceRun>,
    ordered_materials: Vec<Material>,
    ordered_material_ids: HashMap<Material, usize>,
    ordered_instances: Vec<OrderedInstance>,
    ordered_revision: Option<u64>,
    ordered_view: glam::Mat4,
    ordered_projection: glam::Mat4,
    ordered_dirty: bool,
    stats: DebugStats,
}

impl DebugGl {
    pub fn new(gl: &glow::Context) -> Self {
        let program = compile_program(
            gl,
            "retained debug mesh",
            DEBUG_MESH_VERT_SRC,
            DEBUG_MESH_FRAG_SRC,
        );
        let view = uniform_location(gl, program, "retained debug mesh", "u_view");
        let proj = uniform_location(gl, program, "retained debug mesh", "u_proj");
        let sampler = uniform_location(gl, program, "retained debug mesh", "u_texture");
        let textured = uniform_location(gl, program, "retained debug mesh", "u_textured");
        let group = uniform_location(gl, program, "retained debug mesh", "u_group");
        let alpha = uniform_location(gl, program, "retained debug mesh", "u_alpha");
        Self {
            program,
            view,
            proj,
            sampler,
            textured,
            group,
            alpha,
            meshes: HashMap::new(),
            textures: HashMap::new(),
            resource_sync: cache::ResourceSyncIdentity::default(),
            batches: HashMap::new(),
            reusable_instance_buffers: Vec::new(),
            geometry_revisions: HashMap::new(),
            line_batches: HashMap::new(),
            reusable_line_batches: Vec::new(),
            line_program: LineProgram::new(gl),
            profiling: false,
            timers: None,
            timer_pending: false,
            measure_frame: false,
            view_matrix: glam::Mat4::IDENTITY,
            projection_matrix: glam::Mat4::IDENTITY,
            viewport: [1.0; 2],
            prepared_groups: Vec::new(),
            group_scratch: Vec::new(),
            ordered_buffer: unsafe { gl.create_buffer().expect("debug ordered instance buffer") },
            ordered_capacity: 0,
            ordered_data: Vec::new(),
            ordered_scratch: Vec::new(),
            ordered_runs: Vec::new(),
            ordered_materials: Vec::new(),
            ordered_material_ids: HashMap::new(),
            ordered_instances: Vec::new(),
            ordered_revision: None,
            ordered_view: glam::Mat4::IDENTITY,
            ordered_projection: glam::Mat4::IDENTITY,
            ordered_dirty: true,
            stats: DebugStats::default(),
        }
    }

    pub fn set_profiling(&mut self, enabled: bool) {
        self.profiling = enabled;
    }

    /// Prepare cached geometry before the debug draw phases. Call each phase once.
    pub fn begin_frame(
        &mut self,
        gl: &glow::Context,
        scene: &DebugScene,
        state: Option<&ArenaState>,
        view: &glam::Mat4,
        proj: &glam::Mat4,
        viewport: [f32; 2],
    ) {
        let cpu_started = Instant::now();
        self.stats.draw_calls = 0;
        self.stats.instances = 0;
        self.stats.primitives = 0;
        self.stats.segments = 0;
        self.stats.points = 0;
        self.stats.culled = 0;
        let mut geometry_changed = false;
        for group in scene.groups.values().filter(|g| g.visible) {
            let batches = self.batches.entry(group.name.clone()).or_default();
            if self.geometry_revisions.get(&group.name) != Some(&group.geometry_revision) {
                geometry_changed = true;
                self.geometry_revisions
                    .insert(group.name.clone(), group.geometry_revision);
                if let Some(old) = self.line_batches.remove(&group.name) {
                    self.reusable_line_batches
                        .extend(old.into_iter().map(|batch| batch.gpu));
                }
                let mut segments: HashMap<u8, Vec<PackedLine>> = HashMap::new();
                self.reusable_instance_buffers
                    .extend(batches.drain(..).map(|batch| batch.buffer));
                let mut grouped: HashMap<(String, Option<String>, u8, bool), Vec<InstanceData>> =
                    HashMap::new();
                for (idx, item) in group.items.iter().enumerate() {
                    match item {
                        DebugItem::Instance(i) => {
                            let mat = instance_matrix(i);
                            let key = (
                                i.mesh.clone(),
                                i.texture.clone(),
                                depth_key(i.depth),
                                i.tint.a >= 1.0,
                            );
                            grouped.entry(key).or_default().push(InstanceData {
                                transform: mat,
                                tint: i.tint.to_array(),
                                source_index: idx,
                            });
                        }
                        DebugItem::Mesh(m) => {
                            let id = format!("@inline:{}:{idx}", group.name);
                            if let Some(old) = self.meshes.remove(&id) {
                                unsafe { delete_mesh(gl, old) };
                            }
                            let res = inline_resource(&id, m);
                            let gpu = unsafe { upload_mesh(gl, &res) };
                            self.stats.mesh_uploads += 1;
                            self.stats.upload_bytes +=
                                res.positions.len() as u64 * 48 + res.indices.len() as u64 * 4;
                            self.meshes.insert(id.clone(), gpu);
                            let key = (id, m.texture.clone(), depth_key(m.depth), true);
                            grouped.entry(key).or_default().push(InstanceData {
                                transform: glam::Mat4::IDENTITY,
                                tint: [1.0; 4],
                                source_index: idx,
                            });
                        }
                        DebugItem::Line(line) => {
                            segments
                                .entry(depth_key(line.depth))
                                .or_default()
                                .push(pack_segment(
                                    line.start,
                                    line.end,
                                    line.color.to_array(),
                                    line.width,
                                    false,
                                ));
                        }
                        DebugItem::Polyline(path) => {
                            let dest = segments.entry(depth_key(path.depth)).or_default();
                            for ends in path.points.windows(2) {
                                dest.push(pack_segment(
                                    ends[0],
                                    ends[1],
                                    path.color.to_array(),
                                    path.width,
                                    false,
                                ));
                            }
                            if path.closed && path.points.len() > 2 {
                                dest.push(pack_segment(
                                    *path.points.last().unwrap(),
                                    path.points[0],
                                    path.color.to_array(),
                                    path.width,
                                    false,
                                ));
                            }
                        }
                        DebugItem::Point(point) => {
                            segments
                                .entry(depth_key(point.depth))
                                .or_default()
                                .push(pack_segment(
                                    point.pos,
                                    point.pos,
                                    point.color.to_array(),
                                    point.size,
                                    true,
                                ));
                        }
                        _ => {}
                    }
                }
                let mut line_batches = Vec::new();
                for (depth, data) in segments {
                    let points = data.iter().filter(|line| line.point).count() as u64;
                    let (gpu, upload_bytes) =
                        if let Some(mut gpu) = self.reusable_line_batches.pop() {
                            let upload_bytes = gpu.update(gl, &data);
                            (gpu, upload_bytes)
                        } else {
                            let gpu = LineBatch::new(gl, &data);
                            let upload_bytes = gpu.upload_bytes;
                            (gpu, upload_bytes)
                        };
                    self.stats.line_uploads += 1;
                    self.stats.upload_bytes += upload_bytes as u64;
                    line_batches.push(SegmentBatch {
                        gpu,
                        depth: depth_mode(depth),
                        points,
                    });
                }

                self.line_batches.insert(group.name.clone(), line_batches);
                let mut grouped: Vec<_> = grouped.into_iter().collect();
                grouped.sort_unstable_by_key(|(_, instances)| instances[0].source_index);
                for ((mesh, texture, depth, tint_opaque), instances) in grouped {
                    let mut data = Vec::with_capacity(instances.len() * 20);
                    for instance in &instances {
                        append_instance(&mut data, instance.transform, instance.tint);
                    }
                    let buffer = self
                        .reusable_instance_buffers
                        .pop()
                        .unwrap_or_else(|| unsafe {
                            gl.create_buffer().expect("debug instance buffer")
                        });
                    unsafe {
                        gl.bind_buffer(glow::ARRAY_BUFFER, Some(buffer));
                        gl.buffer_data_u8_slice(
                            glow::ARRAY_BUFFER,
                            bytes_of(&data),
                            glow::DYNAMIC_DRAW,
                        );
                    }
                    let count = (data.len() / 20) as i32;
                    batches.push(Batch {
                        mesh,
                        texture,
                        depth: depth_mode(depth),
                        count,
                        buffer,
                        tint_opaque,
                        instances,
                    });
                    self.stats.upload_bytes += (data.len() * 4) as u64;
                }
            }
        }
        let stale_groups: Vec<String> = self
            .batches
            .keys()
            .filter(|name| !scene.groups.contains_key(*name))
            .cloned()
            .collect();
        for name in stale_groups {
            geometry_changed = true;
            self.geometry_revisions.remove(&name);
            if let Some(batches) = self.line_batches.remove(&name) {
                self.reusable_line_batches
                    .extend(batches.into_iter().map(|batch| batch.gpu));
            }
            if let Some(batches) = self.batches.remove(&name) {
                self.reusable_instance_buffers
                    .extend(batches.into_iter().map(|batch| batch.buffer));
            }
        }
        if geometry_changed {
            let live_inline: std::collections::HashSet<String> = scene
                .groups
                .values()
                .flat_map(|g| {
                    g.items
                        .iter()
                        .enumerate()
                        .filter(|(_, item)| matches!(item, DebugItem::Mesh(_)))
                        .map(|(i, _)| format!("@inline:{}:{i}", g.name))
                })
                .collect();
            let removed_inline: Vec<String> = self
                .meshes
                .keys()
                .filter(|id| id.starts_with("@inline:") && !live_inline.contains(*id))
                .cloned()
                .collect();
            for id in removed_inline {
                if let Some(mesh) = self.meshes.remove(&id) {
                    unsafe { delete_mesh(gl, mesh) };
                }
            }
        }
        self.view_matrix = *view;
        self.projection_matrix = *proj;
        self.viewport = viewport;
        self.group_scratch.clear();
        for group in scene.groups.values().filter(|group| group.visible) {
            if let Some(transform) = resolve_group_matrix(group, state) {
                self.group_scratch.push(PreparedGroup {
                    group: group.clone(),
                    transform,
                });
            }
        }
        let placements_changed = self.group_scratch.len() != self.prepared_groups.len()
            || self
                .group_scratch
                .iter()
                .zip(&self.prepared_groups)
                .any(|(next, previous)| {
                    next.group.name != previous.group.name || next.transform != previous.transform
                });
        std::mem::swap(&mut self.prepared_groups, &mut self.group_scratch);
        if geometry_changed
            || placements_changed
            || self.ordered_dirty
            || self.ordered_revision != Some(scene.revision)
            || self.ordered_view != *view
            || self.ordered_projection != *proj
        {
            if self.prepare_ordered_instances() && !self.ordered_data.is_empty() {
                self.stats.upload_bytes += unsafe {
                    upload_ordered_instances(
                        gl,
                        self.ordered_buffer,
                        &mut self.ordered_capacity,
                        &self.ordered_data,
                    ) as u64
                };
            }
            self.ordered_revision = Some(scene.revision);
            self.ordered_view = *view;
            self.ordered_projection = *proj;
            self.ordered_dirty = false;
        }
        self.prepare_timers(gl);
        self.stats.cpu_ms = cpu_started.elapsed().as_secs_f64() * 1000.0;
    }

    fn batch_is_transparent(&self, batch: &Batch) -> bool {
        !batch.tint_opaque
            || self
                .meshes
                .get(&batch.mesh)
                .is_some_and(|mesh| mesh.transparent)
            || batch
                .texture
                .as_ref()
                .and_then(|id| self.textures.get(id))
                .is_some_and(|texture| texture.transparent)
    }

    fn prepare_timers(&mut self, gl: &glow::Context) {
        self.measure_frame = false;
        if !self.profiling {
            return;
        }
        unsafe {
            let timers = *self.timers.get_or_insert_with(|| {
                std::array::from_fn(|_| gl.create_query().expect("debug timer query"))
            });
            if self.timer_pending
                && timers.iter().all(|&timer| {
                    gl.get_query_parameter_u32(timer, glow::QUERY_RESULT_AVAILABLE) != 0
                })
            {
                self.stats.gpu_ms = Some(
                    timers
                        .iter()
                        .map(|&timer| {
                            gl.get_query_parameter_u32(timer, glow::QUERY_RESULT) as f64
                                / 1_000_000.0
                        })
                        .sum(),
                );
                self.timer_pending = false;
            }
            self.measure_frame = !self.timer_pending;
        }
    }

    fn start_timer(&self, gl: &glow::Context, index: usize) {
        if self.measure_frame {
            unsafe {
                gl.begin_query(glow::TIME_ELAPSED, self.timers.unwrap()[index]);
            }
        }
    }

    fn stop_timer(&self, gl: &glow::Context) {
        if self.measure_frame {
            unsafe {
                gl.end_query(glow::TIME_ELAPSED);
            }
        }
    }

    pub fn end_frame(&mut self, gl: &glow::Context) {
        if self.measure_frame {
            self.timer_pending = true;
        }
        unsafe {
            gl.depth_mask(true);
            gl.depth_func(glow::LEQUAL);
            gl.enable(glow::DEPTH_TEST);
            gl.disable(glow::BLEND);
            gl.bind_vertex_array(None);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            gl.bind_texture(glow::TEXTURE_2D, None);
            gl.use_program(None);
        }
    }
    pub fn stats(&self) -> DebugStats {
        self.stats
    }

    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            for mesh in self.meshes.values() {
                gl.delete_vertex_array(mesh.vao);
                for buffer in mesh.buffers {
                    gl.delete_buffer(buffer);
                }
                gl.delete_buffer(mesh.index);
            }
            for texture in self.textures.values() {
                gl.delete_texture(texture.handle);
            }
            for batches in self.batches.values() {
                for batch in batches {
                    gl.delete_buffer(batch.buffer);
                }
            }
            for batches in self.line_batches.values() {
                for batch in batches {
                    batch.gpu.destroy(gl);
                }
            }
            for batch in &self.reusable_line_batches {
                batch.destroy(gl);
            }
            for buffer in &self.reusable_instance_buffers {
                gl.delete_buffer(*buffer);
            }
            self.line_program.destroy(gl);
            if let Some(timers) = self.timers {
                for timer in timers {
                    gl.delete_query(timer);
                }
            }
            gl.delete_buffer(self.ordered_buffer);
            gl.delete_program(self.program);
        }
    }
}

fn inline_resource(id: &str, m: &rocketsim_vis::debug::InlineMeshItem) -> MeshResource {
    MeshResource {
        id: id.into(),
        positions: m.positions.clone(),
        colors: m.colors.clone(),
        normals: m.normals.clone(),
        uvs: m.uvs.clone(),
        indices: m.indices.clone(),
        revision: 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::render::debug::{prepare::pack_segment, resources::recompute_normals};
    use rocketsim_vis::debug::Width;

    #[test]
    fn generated_normals_keep_degenerate_and_unused_vertex_fallbacks() {
        let mesh = MeshResource::new(
            "normals",
            vec![[0.0; 3], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [2.0; 3]],
            vec![rocketsim_vis::debug::Color::WHITE; 4],
            None,
            None,
            vec![0, 1, 2, 0, 0, 0],
        )
        .unwrap();
        assert_eq!(
            recompute_normals(&mesh),
            vec![
                glam::Vec3::X.to_array(),
                glam::Vec3::X.to_array(),
                glam::Vec3::X.to_array(),
                glam::Vec3::Z.to_array()
            ]
        );
    }

    #[test]
    fn segment_packing_preserves_width_units_and_point_flags() {
        let pixel = pack_segment([0.0; 3], [1.0; 3], [1.0; 4], Width::Pixels(3.0), false);
        let world = pack_segment([0.0; 3], [1.0; 3], [1.0; 4], Width::World(3.0), true);
        assert_eq!((pixel.width, pixel.pixels, pixel.point), (3.0, true, false));
        assert_eq!((world.width, world.pixels, world.point), (3.0, false, true));
    }
}
