use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

use glam::{Mat3, Vec3};
use glow::HasContext;
use rocketsim::GameMode;
use rocketsim_vis::{debug::DebugScene, proto::VisFrame};

use crate::{
    effects::dropshot::DropshotTileMesh,
    geometry::{frustum::frustum_planes, lines::Segment, mesh::ArenaMesh, pads::build_instances},
    render::{
        debug::DebugStats,
        main::{
            BALL_MODEL, BALL_TEXTURE, PUCK_MODEL, car_model_name, car_texture_name,
            group_car_draws, model_matrix,
        },
        scene::SceneGl,
        view,
    },
};

pub struct PreparedRenderFrame {
    pub scene: Arc<Mutex<Option<SceneGl>>>,
    pub mesh: Arc<ArenaMesh>,
    pub frame: Option<Arc<VisFrame>>,
    pub debug: Arc<DebugScene>,
    pub pending_segments: Arc<Mutex<Vec<Segment>>>,
    pub dropshot_mesh: Arc<DropshotTileMesh>,
    pub camera_position: glam::Vec3A,
    pub camera_direction: glam::Vec3A,
    pub camera_fov: f32,
    pub enclosed: bool,
    pub rebuild_arena: bool,
    pub generation_changed: bool,
    pub show_ball: bool,
    pub show_pads: bool,
    pub ball_radius_scale: f32,
    pub profiling: bool,
    pub measure_gpu: bool,
    pub debug_info: Arc<Mutex<DebugStats>>,
    pub last_draw_ms: Arc<Mutex<f64>>,
    pub gpu_ms: Arc<Mutex<f64>>,
    pub pads_info: Arc<Mutex<(usize, usize)>>,
    pub lines_info: Arc<Mutex<(usize, usize)>>,
}

pub fn paint_callback(rect: egui::Rect, prepared: PreparedRenderFrame) -> egui::PaintCallback {
    egui::PaintCallback {
        rect,
        callback: Arc::new(egui_glow::CallbackFn::new(move |info, painter| {
            let gl = painter.gl();
            let ppp = info.pixels_per_point;
            let w_px = (info.viewport.width() * ppp).round() as i32;
            let h_px = (info.viewport.height() * ppp).round() as i32;
            if w_px <= 1 || h_px <= 1 {
                return;
            }
            let x_px = (info.viewport.min.x * ppp).round() as i32;
            let y_px = (info.screen_size_px[1] as f32 - info.viewport.max.y * ppp).round() as i32;
            let aspect = w_px as f32 / h_px as f32;
            let (view_mat, proj_mat) = view::view_proj(
                Vec3::from(prepared.camera_position),
                Vec3::from(prepared.camera_position + prepared.camera_direction),
                prepared.camera_fov,
                aspect,
            );
            let t0 = Instant::now();
            let mut guard = prepared.scene.lock().expect("viewer scene poisoned");
            if guard.is_none() {
                *guard = Some(SceneGl::new(gl, &prepared.mesh));
            } else if prepared.rebuild_arena
                && let Some(scene) = guard.as_mut()
            {
                scene.rebuild_arena(gl, &prepared.mesh);
                if prepared.generation_changed {
                    scene.debug.destroy(gl);
                    scene.debug = crate::render::debug::DebugGl::new(gl);
                }
            }
            let scene: &mut SceneGl = guard.as_mut().expect("viewer scene built");
            scene.arena.draw(
                gl,
                (x_px, y_px, w_px, h_px),
                &glam::Mat4::IDENTITY,
                &view_mat,
                &proj_mat,
                prepared.enclosed,
            );
            scene.dropshot.upload_shared(gl, &prepared.dropshot_mesh);
            scene.dropshot.draw(gl, &view_mat, &proj_mat);
            scene.prog_main.set_view_proj(gl, &view_mat, &proj_mat);
            if let Some(frame) = &prepared.frame {
                if prepared.show_ball {
                    let ball = &frame.state.ball.phys;
                    let ball_model = if frame.state.game_mode() == GameMode::Snowday {
                        PUCK_MODEL
                    } else {
                        BALL_MODEL
                    };
                    let ball_model = scene
                        .models
                        .get(ball_model)
                        .or_else(|| scene.models.get(BALL_MODEL))
                        .expect("scene has a ball or puck model");
                    scene.prog_main.draw(
                        gl,
                        ball_model,
                        &scene.textures[BALL_TEXTURE],
                        &(model_matrix(Vec3::from(ball.pos), Mat3::from(ball.rot_mat))
                            * glam::Mat4::from_scale(
                                if frame.state.game_mode() == GameMode::Snowday {
                                    Vec3::new(
                                        prepared.ball_radius_scale,
                                        prepared.ball_radius_scale,
                                        1.0,
                                    )
                                } else {
                                    Vec3::splat(prepared.ball_radius_scale)
                                },
                            )),
                    );
                }
                let mut placements = Vec::new();
                for (info, state) in &frame.state.cars {
                    if state.is_demoed {
                        continue;
                    }
                    placements.push((
                        car_model_name(&info.config),
                        car_texture_name(info.team),
                        model_matrix(Vec3::from(state.phys.pos), Mat3::from(state.phys.rot_mat)),
                    ));
                }
                scene.prog_cars.set_view_proj(gl, &view_mat, &proj_mat);
                for group in group_car_draws(&placements) {
                    let model = scene
                        .car_models
                        .get_mut(group.model)
                        .expect("bundled car model");
                    model.upload(gl, &group.matrices);
                    scene
                        .prog_cars
                        .draw_group(gl, model, &scene.textures[group.texture]);
                }
                scene.prog_cars.unbind(gl);
            }
            scene.prog_main.unbind(gl);
            scene.debug.sync(gl, &prepared.debug);
            scene.debug.set_profiling(prepared.profiling);
            scene.debug.begin_frame(
                gl,
                &prepared.debug,
                prepared.frame.as_deref().map(|frame| &frame.state),
                &view_mat,
                &proj_mat,
                [w_px as f32, h_px as f32],
            );
            scene.debug.draw_opaque(gl);
            let planes = frustum_planes(proj_mat * view_mat);
            let empty: [Vec<glam::Mat4>; 4] = Default::default();
            let (instances, culled) = match &prepared.frame {
                Some(frame) if prepared.show_pads => {
                    build_instances(&frame.state.boost_pads, &planes, &scene.pad_radii)
                }
                _ => (empty, 0),
            };
            scene.pads.upload(gl, &instances);
            scene.pads.draw(gl, &view_mat, &proj_mat);
            scene
                .arena
                .draw_net(gl, &glam::Mat4::IDENTITY, &view_mat, &proj_mat);
            scene.debug.draw_transparent(gl);
            if let Ok(mut info) = prepared.pads_info.lock() {
                *info = (instances.iter().map(|values| values.len()).sum(), culled);
            }
            let segments = std::mem::take(
                &mut *prepared
                    .pending_segments
                    .lock()
                    .expect("viewer segments poisoned"),
            );
            scene.lines.upload(
                gl,
                &segments,
                prepared.camera_position,
                &view_mat,
                proj_mat.y_axis.y,
                h_px as f32,
            );
            scene.lines.draw(gl, &view_mat, &proj_mat);
            if let Ok(mut info) = prepared.lines_info.lock() {
                *info = (scene.lines.drawn(), scene.lines.truncated);
            }
            scene.debug.draw_overlay(gl);
            scene.debug.end_frame(gl);
            if let Ok(mut info) = prepared.debug_info.lock() {
                *info = scene.debug.stats();
            }
            if prepared.measure_gpu {
                let started = Instant::now();
                unsafe {
                    gl.finish();
                }
                if let Ok(mut gpu_ms) = prepared.gpu_ms.lock() {
                    *gpu_ms = started.elapsed().as_secs_f64() * 1000.0;
                }
            }
            drop(guard);
            if let Ok(mut draw_ms) = prepared.last_draw_ms.lock() {
                *draw_ms = t0.elapsed().as_secs_f64() * 1000.0;
            }
        })),
    }
}
