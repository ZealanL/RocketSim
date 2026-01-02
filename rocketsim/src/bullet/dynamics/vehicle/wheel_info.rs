use glam::{Affine3A, Vec3A};

#[derive(Clone, Copy)]
pub struct WheelInfoConstructionInfo {
    pub chassis_connection_cs: Vec3A,
    pub wheel_direction_cs: Vec3A,
    pub wheel_axle_cs: Vec3A,
    pub suspension_rest_length: f32,
    pub wheel_radius: f32,
}

#[derive(Default)]
pub struct RaycastInfo {
    pub contact_normal_ws: Vec3A,
    pub contact_point_ws: Vec3A,
    pub suspension_length: f32,
    pub hard_point_ws: Vec3A,
    pub wheel_direction_ws: Vec3A,
    pub wheel_axle_ws: Vec3A,
    pub is_in_contact: bool,
    pub ground_object: Option<usize>,
}

pub struct WheelInfo {
    pub raycast_info: RaycastInfo,
    pub world_trans: Affine3A,
    pub chassis_connection_point_cs: Vec3A,
    pub wheel_direction_cs: Vec3A,
    pub wheel_axle_cs: Vec3A,
    pub suspension_rest_length_1: f32,
    pub wheels_radius: f32,
    pub engine_force: f32,
    pub brake: f32,
    pub clipped_inv_contact_dot_suspension: f32,
    pub suspension_relative_velocity: f32,
    pub wheels_suspension_force: f32,
}

impl WheelInfo {
    pub fn new(ci: WheelInfoConstructionInfo) -> Self {
        Self {
            suspension_rest_length_1: ci.suspension_rest_length,
            wheels_radius: ci.wheel_radius,
            chassis_connection_point_cs: ci.chassis_connection_cs,
            wheel_direction_cs: ci.wheel_direction_cs,
            wheel_axle_cs: ci.wheel_axle_cs,
            engine_force: 0.0,
            brake: 0.0,
            raycast_info: RaycastInfo::default(),
            world_trans: Affine3A::IDENTITY,
            clipped_inv_contact_dot_suspension: 0.0,
            suspension_relative_velocity: 0.0,
            wheels_suspension_force: 0.0,
        }
    }
}
