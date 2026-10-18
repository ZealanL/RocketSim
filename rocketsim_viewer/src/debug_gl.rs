//! GPU cache for the retained debug scene. Geometry and texture assets are
//! uploaded only when their public revisions change.
use std::{collections::HashMap, sync::Arc, time::Instant};

use glow::HasContext;
use rocketsim::ArenaState;
use rocketsim_vis::debug::{
    Anchor, DebugItem, DebugScene, DepthMode, MeshResource, TextureResource, Width,
};

use crate::{
    debug_lines_gl::{LineBatch, LineProgram, PackedLine},
    debug_order::{DrawLayer, InstanceRun, OrderedInstance, contiguous_runs, sort_back_to_front},
    gl_util::{bytes_of, compile_program, uniform_location},
};

pub const DEBUG_MESH_VERT_SRC: &str = include_str!("shaders/debug_mesh_vert.glsl");
pub const DEBUG_MESH_FRAG_SRC: &str = include_str!("shaders/debug_mesh_frag.glsl");

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
    batches: HashMap<String, Vec<Batch>>,
    geometry_revisions: HashMap<String, u64>,
    line_batches: HashMap<String, Vec<SegmentBatch>>,
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
    ordered_runs: Vec<InstanceRun>,
    ordered_materials: Vec<Material>,
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
            batches: HashMap::new(),
            geometry_revisions: HashMap::new(),
            line_batches: HashMap::new(),
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
            ordered_runs: Vec::new(),
            ordered_materials: Vec::new(),
            ordered_revision: None,
            ordered_view: glam::Mat4::IDENTITY,
            ordered_projection: glam::Mat4::IDENTITY,
            ordered_dirty: true,
            stats: DebugStats::default(),
        }
    }

    /// Synchronize only changed asset revisions. The scene snapshot is immutable.
    pub fn sync(&mut self, gl: &glow::Context, scene: &Arc<DebugScene>) {
        for (id, mesh) in &scene.meshes {
            if self
                .meshes
                .get(id)
                .is_some_and(|gpu| gpu.revision == mesh.revision)
            {
                continue;
            }
            if let Some(old) = self.meshes.remove(id) {
                unsafe {
                    delete_mesh(gl, old);
                }
            }
            self.ordered_dirty = true;
            let gpu = unsafe { upload_mesh(gl, mesh) };
            self.stats.mesh_uploads += 1;
            self.stats.upload_bytes +=
                mesh.positions.len() as u64 * 48 + mesh.indices.len() as u64 * 4;
            self.meshes.insert(id.clone(), gpu);
        }
        let stale_meshes: Vec<String> = self
            .meshes
            .keys()
            .filter(|id| !id.starts_with("@inline:") && !scene.meshes.contains_key(*id))
            .cloned()
            .collect();
        for id in stale_meshes {
            self.ordered_dirty = true;
            if let Some(mesh) = self.meshes.remove(&id) {
                unsafe {
                    delete_mesh(gl, mesh);
                }
            }
        }
        for (id, tex) in &scene.textures {
            if self
                .textures
                .get(id)
                .is_some_and(|gpu| gpu.revision == tex.revision)
            {
                continue;
            }
            if let Some(old) = self.textures.remove(id) {
                unsafe {
                    gl.delete_texture(old.handle);
                }
            }
            self.ordered_dirty = true;
            let gpu = unsafe { upload_texture(gl, tex) };
            self.stats.texture_uploads += 1;
            self.stats.upload_bytes += tex.rgba.len() as u64;
            self.textures.insert(id.clone(), gpu);
        }
        let stale: Vec<String> = self
            .textures
            .keys()
            .filter(|id| !scene.textures.contains_key(*id))
            .cloned()
            .collect();
        for id in stale {
            self.ordered_dirty = true;
            if let Some(tex) = self.textures.remove(&id) {
                unsafe {
                    gl.delete_texture(tex.handle);
                }
            }
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
                    for batch in old {
                        batch.gpu.destroy(gl);
                    }
                }
                let mut segments: HashMap<u8, Vec<PackedLine>> = HashMap::new();
                unsafe {
                    for b in batches.drain(..) {
                        gl.delete_buffer(b.buffer);
                    }
                }
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
                    let gpu = LineBatch::new(gl, &data);
                    self.stats.line_uploads += 1;
                    self.stats.upload_bytes += gpu.upload_bytes as u64;
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
                    let buffer = unsafe { gl.create_buffer().expect("debug instance buffer") };
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
                for batch in batches {
                    batch.gpu.destroy(gl);
                }
            }
            if let Some(batches) = self.batches.remove(&name) {
                unsafe {
                    for b in batches {
                        gl.delete_buffer(b.buffer);
                    }
                }
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
            self.prepare_ordered_instances(gl);
            self.ordered_revision = Some(scene.revision);
            self.ordered_view = *view;
            self.ordered_projection = *proj;
            self.ordered_dirty = false;
        }
        self.prepare_timers(gl);
        self.stats.cpu_ms = cpu_started.elapsed().as_secs_f64() * 1000.0;
    }

    /// Draw a complete debug scene when no other world passes need to be interleaved.
    pub fn draw(
        &mut self,
        gl: &glow::Context,
        scene: &DebugScene,
        state: Option<&ArenaState>,
        view: &glam::Mat4,
        proj: &glam::Mat4,
        viewport: [f32; 2],
    ) {
        self.begin_frame(gl, scene, state, view, proj, viewport);
        self.draw_opaque(gl);
        self.draw_transparent(gl);
        self.draw_overlay(gl);
        self.end_frame(gl);
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

    fn prepare_ordered_instances(&mut self, gl: &glow::Context) {
        let mut materials = Vec::new();
        let mut material_ids = HashMap::new();
        let mut instances = Vec::new();
        for (group_index, prepared) in self.prepared_groups.iter().enumerate() {
            for batch in self.batches.get(&prepared.group.name).into_iter().flatten() {
                let layers: &[DrawLayer] = match batch.depth {
                    DepthMode::Test if self.batch_is_transparent(batch) => {
                        &[DrawLayer::Transparent]
                    }
                    DepthMode::Test => continue,
                    DepthMode::Xray => &[DrawLayer::XrayHidden, DrawLayer::XrayVisible],
                    DepthMode::Always => &[DrawLayer::Overlay],
                };
                let Some(mesh) = self.meshes.get(&batch.mesh) else {
                    continue;
                };
                let material = Material {
                    mesh: batch.mesh.clone(),
                    texture: batch.texture.clone(),
                };
                let material_index = *material_ids.entry(material.clone()).or_insert_with(|| {
                    let index = materials.len();
                    materials.push(material);
                    index
                });
                for instance in &batch.instances {
                    let transform = prepared.transform * instance.transform;
                    let center = transform.transform_point3a(mesh.center);
                    let forward_sign = if self.projection_matrix.z_axis.w != 0.0 {
                        self.projection_matrix.z_axis.w.signum()
                    } else {
                        self.projection_matrix.z_axis.z.signum()
                    };
                    let view_depth = self.view_matrix.transform_point3a(center).z * forward_sign;
                    for &layer in layers {
                        instances.push(OrderedInstance {
                            material: material_index,
                            layer,
                            view_depth,
                            source_order: (group_index, instance.source_index),
                            transform,
                            tint: instance.tint,
                        });
                    }
                }
            }
        }
        sort_back_to_front(&mut instances);
        self.ordered_runs = contiguous_runs(&instances);
        self.ordered_materials = materials;
        let mut data = Vec::with_capacity(instances.len() * 20);
        for instance in instances {
            append_instance(&mut data, instance.transform, instance.tint);
        }
        if data != self.ordered_data {
            if !data.is_empty() {
                let bytes = bytes_of(&data);
                unsafe {
                    gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.ordered_buffer));
                    if bytes.len() > self.ordered_capacity {
                        self.ordered_capacity = bytes.len().next_power_of_two();
                        gl.buffer_data_size(
                            glow::ARRAY_BUFFER,
                            self.ordered_capacity as i32,
                            glow::DYNAMIC_DRAW,
                        );
                    }
                    gl.buffer_sub_data_u8_slice(glow::ARRAY_BUFFER, 0, bytes);
                    gl.bind_buffer(glow::ARRAY_BUFFER, None);
                }
                self.stats.upload_bytes += bytes.len() as u64;
            }
            self.ordered_data = data;
        }
    }

    /// Write all opaque debug depth before any world transparency.
    pub fn draw_opaque(&mut self, gl: &glow::Context) {
        let started = Instant::now();
        self.start_timer(gl, 0);
        configure_depth(gl, DepthMode::Test, false, true);
        self.bind_mesh_program(gl, &glam::Mat4::IDENTITY, 1.0);
        for prepared in &self.prepared_groups {
            let Some(batches) = self.batches.get(&prepared.group.name) else {
                continue;
            };
            unsafe {
                gl.uniform_matrix_4_f32_slice(
                    Some(&self.group),
                    false,
                    &prepared.transform.to_cols_array(),
                );
            }
            for batch in batches {
                if batch.depth != DepthMode::Test || self.batch_is_transparent(batch) {
                    continue;
                }
                let Some(mesh) = self.meshes.get(&batch.mesh) else {
                    continue;
                };
                let texture = batch.texture.as_ref().and_then(|id| self.textures.get(id));
                bind_debug_texture(gl, texture, &self.textured);
                draw_instances(gl, mesh, batch.buffer, 0, batch.count);
                self.stats.draw_calls += 1;
                self.stats.instances += batch.count as u64;
                self.stats.primitives += mesh.count as u64 / 3 * batch.count as u64;
            }
        }
        self.stop_timer(gl);
        self.stats.cpu_ms += started.elapsed().as_secs_f64() * 1000.0;
    }

    /// Blend depth-tested debug drawing after the complete opaque world.
    pub fn draw_transparent(&mut self, gl: &glow::Context) {
        let started = Instant::now();
        self.start_timer(gl, 1);
        configure_depth(gl, DepthMode::Test, false, false);
        self.draw_ordered_layer(gl, DrawLayer::Transparent, 1.0);
        self.draw_lines(gl, DepthMode::Test, false);
        self.stop_timer(gl);
        self.stats.cpu_ms += started.elapsed().as_secs_f64() * 1000.0;
    }

    /// Draw X-ray and always-visible items after world transparency and effects.
    pub fn draw_overlay(&mut self, gl: &glow::Context) {
        let started = Instant::now();
        self.start_timer(gl, 2);
        for (layer, depth, hidden, alpha) in [
            (DrawLayer::XrayHidden, DepthMode::Xray, true, 0.22),
            (DrawLayer::XrayVisible, DepthMode::Xray, false, 1.0),
            (DrawLayer::Overlay, DepthMode::Always, false, 1.0),
        ] {
            configure_depth(gl, depth, hidden, false);
            self.draw_ordered_layer(gl, layer, alpha);
            self.draw_lines(gl, depth, hidden);
        }
        self.stop_timer(gl);
        self.stats.cpu_ms += started.elapsed().as_secs_f64() * 1000.0;
    }

    fn bind_mesh_program(&self, gl: &glow::Context, transform: &glam::Mat4, alpha: f32) {
        unsafe {
            gl.use_program(Some(self.program));
            gl.uniform_matrix_4_f32_slice(
                Some(&self.view),
                false,
                &self.view_matrix.to_cols_array(),
            );
            gl.uniform_matrix_4_f32_slice(
                Some(&self.proj),
                false,
                &self.projection_matrix.to_cols_array(),
            );
            gl.uniform_matrix_4_f32_slice(Some(&self.group), false, &transform.to_cols_array());
            gl.uniform_1_i32(Some(&self.sampler), 0);
            gl.uniform_1_f32(Some(&self.alpha), alpha);
        }
    }

    fn draw_ordered_layer(&mut self, gl: &glow::Context, layer: DrawLayer, alpha: f32) {
        self.bind_mesh_program(gl, &glam::Mat4::IDENTITY, alpha);
        for run in self.ordered_runs.iter().filter(|run| run.layer == layer) {
            let material = &self.ordered_materials[run.material];
            let Some(mesh) = self.meshes.get(&material.mesh) else {
                continue;
            };
            let texture = material
                .texture
                .as_ref()
                .and_then(|id| self.textures.get(id));
            bind_debug_texture(gl, texture, &self.textured);
            draw_instances(gl, mesh, self.ordered_buffer, run.first, run.count as i32);
            self.stats.draw_calls += 1;
            if layer != DrawLayer::XrayHidden {
                self.stats.instances += run.count as u64;
                self.stats.primitives += mesh.count as u64 / 3 * run.count as u64;
            }
        }
    }

    fn draw_lines(&mut self, gl: &glow::Context, depth: DepthMode, hidden: bool) {
        for prepared in &self.prepared_groups {
            for batch in self
                .line_batches
                .get(&prepared.group.name)
                .into_iter()
                .flatten()
                .filter(|batch| batch.depth == depth)
            {
                self.line_program.draw(
                    gl,
                    &batch.gpu,
                    &prepared.transform,
                    &self.view_matrix,
                    &self.projection_matrix,
                    self.viewport,
                    if hidden { 0.22 } else { 1.0 },
                );
                self.stats.draw_calls += 1;
                if !hidden {
                    self.stats.points += batch.points;
                    self.stats.segments += batch.gpu.count as u64 - batch.points;
                }
            }
        }
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

fn append_instance(data: &mut Vec<f32>, transform: glam::Mat4, tint: [f32; 4]) {
    data.extend_from_slice(&transform.to_cols_array());
    data.extend_from_slice(&tint);
}

fn configure_depth(gl: &glow::Context, depth: DepthMode, hidden: bool, write: bool) {
    unsafe {
        gl.disable(glow::CULL_FACE);
        gl.disable(glow::SCISSOR_TEST);
        if depth == DepthMode::Always {
            gl.disable(glow::DEPTH_TEST);
        } else {
            gl.enable(glow::DEPTH_TEST);
        }
        gl.depth_func(if hidden { glow::GREATER } else { glow::LEQUAL });
        gl.depth_mask(write);
        if write {
            gl.disable(glow::BLEND);
        } else {
            gl.enable(glow::BLEND);
            gl.blend_func_separate(
                glow::SRC_ALPHA,
                glow::ONE_MINUS_SRC_ALPHA,
                glow::ONE,
                glow::ONE_MINUS_SRC_ALPHA,
            );
        }
    }
}

fn bind_debug_texture(
    gl: &glow::Context,
    texture: Option<&GpuTexture>,
    textured: &glow::UniformLocation,
) {
    unsafe {
        gl.active_texture(glow::TEXTURE0);
        gl.bind_texture(glow::TEXTURE_2D, texture.map(|texture| texture.handle));
        gl.uniform_1_i32(Some(textured), i32::from(texture.is_some()));
    }
}

fn draw_instances(
    gl: &glow::Context,
    mesh: &GpuMesh,
    buffer: glow::Buffer,
    first: usize,
    count: i32,
) {
    if count == 0 {
        return;
    }
    let offset = (first * 80) as i32;
    unsafe {
        gl.bind_vertex_array(Some(mesh.vao));
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(buffer));
        for column in 0..4 {
            let location = 4 + column;
            gl.enable_vertex_attrib_array(location);
            gl.vertex_attrib_pointer_f32(
                location,
                4,
                glow::FLOAT,
                false,
                80,
                offset + (column * 16) as i32,
            );
            gl.vertex_attrib_divisor(location, 1);
        }
        gl.enable_vertex_attrib_array(8);
        gl.vertex_attrib_pointer_f32(8, 4, glow::FLOAT, false, 80, offset + 64);
        gl.vertex_attrib_divisor(8, 1);
        gl.draw_elements_instanced(glow::TRIANGLES, mesh.count, glow::UNSIGNED_INT, 0, count);
    }
}

fn mesh_center(mesh: &MeshResource) -> glam::Vec3A {
    let mut min = glam::Vec3A::splat(f32::INFINITY);
    let mut max = glam::Vec3A::splat(f32::NEG_INFINITY);
    for position in &mesh.positions {
        let position = glam::Vec3A::from_array(*position);
        min = min.min(position);
        max = max.max(position);
    }
    (min + max) * 0.5
}
fn pack_segment(
    start: [f32; 3],
    end: [f32; 3],
    color: [f32; 4],
    width: Width,
    point: bool,
) -> PackedLine {
    let (width, pixels) = match width {
        Width::Pixels(w) => (w, true),
        Width::World(w) => (w, false),
    };
    PackedLine {
        start,
        end,
        color,
        width,
        pixels,
        point,
    }
}

/// Resolve an anchor from the displayed snapshot. Missing targets hide the group.
pub fn resolve_group_matrix(
    group: &rocketsim_vis::debug::DebugGroup,
    state: Option<&ArenaState>,
) -> Option<glam::Mat4> {
    Some(anchor_matrix(&group.anchor, state)? * group_matrix(group))
}

fn depth_key(d: DepthMode) -> u8 {
    match d {
        DepthMode::Test => 0,
        DepthMode::Always => 1,
        DepthMode::Xray => 2,
    }
}
fn depth_mode(d: u8) -> DepthMode {
    match d {
        0 => DepthMode::Test,
        1 => DepthMode::Always,
        _ => DepthMode::Xray,
    }
}
fn rotation(r: &[[f32; 3]; 3]) -> glam::Mat3A {
    glam::Mat3A::from_cols_array_2d(r).transpose()
}
fn instance_matrix(i: &rocketsim_vis::debug::MeshInstanceItem) -> glam::Mat4 {
    glam::Mat4::from_scale_rotation_translation(
        glam::Vec3::from(i.scale),
        glam::Quat::from_mat3a(&rotation(&i.rotation)),
        glam::Vec3::from(i.position),
    )
}
fn group_matrix(g: &rocketsim_vis::debug::DebugGroup) -> glam::Mat4 {
    glam::Mat4::from_scale_rotation_translation(
        glam::Vec3::from(g.transform.scale),
        glam::Quat::from_mat3a(&rotation(&g.transform.rotation)),
        glam::Vec3::from(g.transform.position),
    )
}
fn anchor_matrix(a: &Anchor, s: Option<&ArenaState>) -> Option<glam::Mat4> {
    let (p, r) = match a {
        Anchor::World => (glam::Vec3A::ZERO, glam::Mat3A::IDENTITY),
        Anchor::Ball { offset: _ } => {
            let s = s?;
            (s.ball.phys.pos, s.ball.phys.rot_mat)
        }
        Anchor::Car { car_id, offset: _ } => {
            let s = s?;
            let (_, state) = s.cars.iter().find(|(info, _)| info.idx as u32 == *car_id)?;
            (state.phys.pos, state.phys.rot_mat)
        }
    };
    let off = match a {
        Anchor::World => glam::Vec3A::ZERO,
        Anchor::Ball { offset } | Anchor::Car { offset, .. } => glam::Vec3A::from_array(*offset),
    };
    Some(glam::Mat4::from_rotation_translation(
        glam::Quat::from_mat3a(&r),
        (p + r * off).into(),
    ))
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

unsafe fn upload_mesh(gl: &glow::Context, mesh: &MeshResource) -> GpuMesh {
    let vao = unsafe { gl.create_vertex_array().expect("debug mesh VAO") };
    let mut buffers = Vec::with_capacity(3);
    unsafe {
        gl.bind_vertex_array(Some(vao));
        let pos = gl.create_buffer().expect("debug positions");
        buffers.push(pos);
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(pos));
        gl.buffer_data_u8_slice(
            glow::ARRAY_BUFFER,
            bytes_of(&mesh.positions),
            glow::STATIC_DRAW,
        );
        gl.enable_vertex_attrib_array(0);
        gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, 12, 0);
        let normals = mesh
            .normals
            .clone()
            .unwrap_or_else(|| recompute_normals(mesh));
        let normal = gl.create_buffer().expect("debug normals");
        buffers.push(normal);
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(normal));
        gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(&normals), glow::STATIC_DRAW);
        gl.enable_vertex_attrib_array(1);
        gl.vertex_attrib_pointer_f32(1, 3, glow::FLOAT, false, 12, 0);
        let colors: Vec<[f32; 4]> = mesh.colors.iter().map(|c| c.to_array()).collect();
        let color = gl.create_buffer().expect("debug colors");
        buffers.push(color);
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(color));
        gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(&colors), glow::STATIC_DRAW);
        gl.enable_vertex_attrib_array(2);
        gl.vertex_attrib_pointer_f32(2, 4, glow::FLOAT, false, 16, 0);
        let uv = mesh
            .uvs
            .clone()
            .unwrap_or_else(|| vec![[0.0; 2]; mesh.positions.len()]);
        let uvbuf = gl.create_buffer().expect("debug UVs");
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(uvbuf));
        gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(&uv), glow::STATIC_DRAW);
        gl.enable_vertex_attrib_array(3);
        gl.vertex_attrib_pointer_f32(3, 2, glow::FLOAT, false, 8, 0);
        let index = gl.create_buffer().expect("debug indices");
        gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(index));
        gl.buffer_data_u8_slice(
            glow::ELEMENT_ARRAY_BUFFER,
            bytes_of(&mesh.indices),
            glow::STATIC_DRAW,
        );
        gl.bind_vertex_array(None);
        GpuMesh {
            vao,
            buffers: [buffers[0], buffers[1], buffers[2], uvbuf],
            index,
            count: mesh.indices.len() as i32,
            revision: mesh.revision,
            transparent: mesh.colors.iter().any(|c| c.a < 1.0),
            center: mesh_center(mesh),
        }
    }
}
fn recompute_normals(mesh: &MeshResource) -> Vec<[f32; 3]> {
    let mut normals = vec![glam::Vec3A::ZERO; mesh.positions.len()];
    for tri in mesh.indices.as_chunks::<3>().0 {
        let a = glam::Vec3A::from_array(mesh.positions[tri[0] as usize]);
        let b = glam::Vec3A::from_array(mesh.positions[tri[1] as usize]);
        let c = glam::Vec3A::from_array(mesh.positions[tri[2] as usize]);
        let normal = (b - a).cross(c - a);
        for &index in tri {
            normals[index as usize] += normal;
        }
    }
    normals
        .into_iter()
        .map(|normal| normal.normalize_or(glam::Vec3A::Z).to_array())
        .collect()
}
unsafe fn upload_texture(gl: &glow::Context, tex: &TextureResource) -> GpuTexture {
    unsafe {
        let handle = gl.create_texture().expect("debug texture");
        gl.bind_texture(glow::TEXTURE_2D, Some(handle));
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_MIN_FILTER,
            glow::LINEAR as i32,
        );
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_MAG_FILTER,
            glow::LINEAR as i32,
        );
        gl.pixel_store_i32(glow::UNPACK_ALIGNMENT, 1);
        gl.tex_image_2d(
            glow::TEXTURE_2D,
            0,
            glow::RGBA8 as i32,
            tex.width as i32,
            tex.height as i32,
            0,
            glow::RGBA,
            glow::UNSIGNED_BYTE,
            glow::PixelUnpackData::Slice(Some(&tex.rgba)),
        );
        GpuTexture {
            handle,
            revision: tex.revision,
            transparent: tex
                .rgba
                .as_chunks::<4>()
                .0
                .iter()
                .any(|pixel| pixel[3] < 255),
        }
    }
}
unsafe fn delete_mesh(gl: &glow::Context, mesh: GpuMesh) {
    unsafe {
        gl.delete_vertex_array(mesh.vao);
        for b in mesh.buffers {
            gl.delete_buffer(b);
        }
        gl.delete_buffer(mesh.index);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn composed_group_and_instance_transform_uses_local_then_group() {
        let instance = rocketsim_vis::debug::MeshInstanceItem {
            mesh: "mesh".into(),
            position: [2.0, 0.0, 0.0],
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            scale: [1.0; 3],
            tint: rocketsim_vis::debug::Color::WHITE,
            texture: None,
            depth: DepthMode::Test,
        };
        let mut group =
            rocketsim_vis::debug::DebugGroup::new("g", Default::default(), Default::default())
                .unwrap();
        group.transform.position = [0.0, 3.0, 0.0];
        group.transform.scale = [2.0; 3];
        let p =
            (group_matrix(&group) * instance_matrix(&instance)).transform_point3(glam::Vec3::ZERO);
        assert_eq!(p, glam::Vec3::new(4.0, 3.0, 0.0));
    }

    #[test]
    fn depth_batches_have_stable_keys() {
        assert_eq!(depth_key(DepthMode::Test), 0);
        assert_eq!(depth_key(DepthMode::Always), 1);
        assert_eq!(depth_mode(depth_key(DepthMode::Xray)), DepthMode::Xray);
    }

    #[test]
    fn missing_anchor_hides_group_and_world_anchor_needs_no_frame() {
        assert!(anchor_matrix(&Anchor::car(123456), None).is_none());
        assert_eq!(
            anchor_matrix(&Anchor::World, None),
            Some(glam::Mat4::IDENTITY)
        );
    }

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
    fn ball_anchor_keeps_its_rotation_and_local_offset() {
        let mut state = ArenaState::new_empty(rocketsim::GameMode::TheVoid);
        state.ball.phys.pos = glam::Vec3A::new(10.0, 20.0, 30.0);
        state.ball.phys.rot_mat = glam::Mat3A::from_rotation_z(std::f32::consts::FRAC_PI_2);
        let transform = anchor_matrix(
            &Anchor::Ball {
                offset: [2.0, 0.0, 0.0],
            },
            Some(&state),
        )
        .unwrap();
        assert!(
            transform
                .transform_point3(glam::Vec3::X)
                .abs_diff_eq(glam::Vec3::new(10.0, 23.0, 30.0), 1e-5)
        );
    }

    #[test]
    fn segment_packing_preserves_explicit_width_units() {
        let pixel = pack_segment([0.0; 3], [1.0; 3], [1.0; 4], Width::Pixels(3.0), false);
        let world = pack_segment([0.0; 3], [1.0; 3], [1.0; 4], Width::World(3.0), true);
        assert!(pixel.pixels && !pixel.point);
        assert!(!world.pixels && world.point);
    }
}
