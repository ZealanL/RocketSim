use std::sync::{Arc, RwLock};

use glam::{Mat3A, Vec3A};

use crate::backend::{Color, Elem2D};

/// One point of a 3D line strip, in Unreal units (uu).
#[derive(Debug, Copy, Clone)]
pub struct VisRenderLinePoint {
    /// Position in uu.
    pub pos: Vec3A,
    /// Point color.
    pub color: Color,
    /// Line width in screen-space units.
    pub width: f32,
}

/// A line strip: consecutive points are connected.
pub type VisRenderLine = Vec<VisRenderLinePoint>;

/// One model draw: a named model + texture + shader pipeline.
#[derive(Debug, Clone)]
pub struct VisRenderModelObj {
    /// Key into [`ModelSet`](crate::backend::ModelSet), e.g. `"ball"`, `"arena"`.
    pub model_name: String,
    /// Key into [`TextureSet`](crate::backend::TextureSet), e.g. `"car_blue"`.
    pub texture_name: Option<String>,
    /// Shader pipeline name (`"main"` default, `"arena"` for the stadium).
    pub pipeline_name: Option<String>,

    /// Position in uu.
    pub model_pos: Vec3A,
    /// Rotation/scale matrix.
    pub model_rot_mat: Mat3A,
}

/// Overlay text style for the debug panel.
#[derive(Debug, Copy, Clone)]
pub enum InfoLineType {
    /// Bright white line.
    Normal,
    /// Muted gray line (hints, controls).
    Dim,
}

/// Per-tick render snapshot shared with the renderer thread.
///
/// [`crate::VisInst`] rebuilds this every tick. To draw something custom,
/// implement [`rocketsim::Vis`] and push into `objects`, `lines`,
/// `info_text_lines`, or `shapes_2d`.
#[derive(Debug, Clone)]
pub struct VisRenderState {
    /// Currently unused; the window lifetime is tied to the `Vis` instance.
    pub shut_down: bool,

    /// Camera position in uu.
    pub camera_pos: Vec3A,
    /// Point the camera looks at in uu.
    pub camera_look_target: Vec3A,
    /// Vertical field of view in degrees.
    pub camera_fov_deg: f32,

    /// 3D models to draw.
    pub objects: Vec<VisRenderModelObj>,
    /// 3D line strips to draw.
    pub lines: Vec<VisRenderLine>,
    /// Overlay text lines in the debug panel.
    pub info_text_lines: Vec<(InfoLineType, String)>,
    /// 2D HUD shapes (boost meter, labels).
    pub shapes_2d: Vec<Elem2D>,
}

impl VisRenderState {
    /// Queues a model draw. Unknown names panic later in
    /// [`ModelSet::get_model_draw_range`](crate::backend::ModelSet::get_model_draw_range).
    pub fn add_model_obj(
        &mut self,
        model_name: &str,
        texture_name: Option<&str>,
        pipeline_name: Option<&str>,
        pos: Vec3A,
        rot_mat: Mat3A,
    ) {
        self.objects.push(VisRenderModelObj {
            model_name: model_name.to_string(),
            texture_name: texture_name.map(|s| s.to_string()),
            pipeline_name: pipeline_name.map(|s| s.to_string()),
            model_pos: pos,
            model_rot_mat: rot_mat,
        })
    }

    /// Queues a two-point line segment in uu.
    pub fn add_line_simple(&mut self, pos_a: Vec3A, pos_b: Vec3A, color: Color, width: f32) {
        self.lines.push(vec![
            VisRenderLinePoint {
                pos: pos_a,
                color,
                width,
            },
            VisRenderLinePoint {
                pos: pos_b,
                color,
                width,
            },
        ]);
    }
}

impl Default for VisRenderState {
    fn default() -> Self {
        Self {
            shut_down: false,

            camera_pos: Vec3A::ZERO,
            camera_look_target: Vec3A::ZERO,
            camera_fov_deg: 60.0,

            objects: Vec::new(),
            lines: Vec::new(),
            info_text_lines: Vec::new(),
            shapes_2d: Vec::new(),
        }
    }
}

pub type SharedVisRenderState = Arc<RwLock<VisRenderState>>;
