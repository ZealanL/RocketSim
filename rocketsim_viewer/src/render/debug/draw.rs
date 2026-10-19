use std::time::Instant;

use glow::HasContext;
use rocketsim_vis::debug::DepthMode;

use super::{DebugGl, GpuMesh, GpuTexture, order::DrawLayer};

impl DebugGl {
    /// Draw a complete debug scene when no other world passes need to be interleaved.
    pub fn draw(
        &mut self,
        gl: &glow::Context,
        scene: &rocketsim_vis::debug::DebugScene,
        state: Option<&rocketsim::ArenaState>,
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
