use glam::{Mat3A, Vec3A};
use std::sync::{Arc, RwLock};

#[derive(Debug, Clone)]
pub struct VisRenderModelObj {
    pub model_name: String,
    pub texture_name: Option<String>,
    pub pipeline_name: Option<String>,

    pub model_pos: Vec3A,
    pub model_rot_mat: Mat3A,
}

#[derive(Debug, Clone)]
pub struct VisRenderState {
    pub shut_down: bool,

    pub camera_pos: Vec3A,
    pub camera_look_target: Vec3A,
    pub camera_fov_deg: f32,

    pub render_objects: Vec<VisRenderModelObj>,
}

impl VisRenderState {
    pub fn add_model_obj(
        &mut self,
        model_name: &str,
        texture_name: Option<&str>,
        pipeline_name: Option<&str>,
        pos: Vec3A,
        rot_mat: Mat3A,
    ) {
        self.render_objects.push(VisRenderModelObj {
            model_name: model_name.to_string(),
            texture_name: texture_name.map(|s| s.to_string()),
            pipeline_name: pipeline_name.map(|s| s.to_string()),
            model_pos: pos,
            model_rot_mat: rot_mat,
        })
    }
}

impl Default for VisRenderState {
    fn default() -> Self {
        Self {
            shut_down: false,

            camera_pos: Vec3A::ZERO,
            camera_look_target: Vec3A::ZERO,
            camera_fov_deg: 60.0,

            render_objects: Vec::new(),
        }
    }
}

pub type SharedVisRenderState = Arc<RwLock<VisRenderState>>;
