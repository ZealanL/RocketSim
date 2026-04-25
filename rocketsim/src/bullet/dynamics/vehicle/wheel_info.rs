use glam::Vec3A;

pub struct RaycastInfo {
    pub contact_normal_ws: Vec3A,
    pub contact_point_ws: Vec3A,
    pub suspension_length: f32,
    pub hard_point_ws: Vec3A,
    pub wheel_direction_ws: Vec3A,
    pub wheel_axle_ws: Vec3A,
    pub is_in_contact: bool,
    pub ground_obj: Option<usize>,
}

impl RaycastInfo {
    const DEFAULT: Self = Self {
        contact_normal_ws: Vec3A::ZERO,
        contact_point_ws: Vec3A::ZERO,
        suspension_length: 0.0,
        hard_point_ws: Vec3A::ZERO,
        wheel_direction_ws: Vec3A::ZERO,
        wheel_axle_ws: Vec3A::ZERO,
        is_in_contact: false,
        ground_obj: None,
    };
}

pub struct WheelInfo {
    pub raycast_info: RaycastInfo,
    pub axle_dir: Vec3A,
    pub chassis_connection_point_cs: Vec3A,
    pub suspension_rest_length_1: f32,
    pub wheels_radius: f32,
    pub engine_force: f32,
    pub brake: f32,
    pub clipped_inv_contact_dot_suspension: f32,
    pub suspension_relative_vel: f32,
    pub wheels_suspension_force: f32,
}

impl WheelInfo {
    pub const DEFAULT: Self = Self {
        chassis_connection_point_cs: Vec3A::ZERO,
        suspension_rest_length_1: 0.0,
        wheels_radius: 0.0,
        engine_force: 0.0,
        brake: 0.0,
        raycast_info: RaycastInfo::DEFAULT,
        axle_dir: Vec3A::ZERO,
        clipped_inv_contact_dot_suspension: 0.0,
        suspension_relative_vel: 0.0,
        wheels_suspension_force: 0.0,
    };
}
