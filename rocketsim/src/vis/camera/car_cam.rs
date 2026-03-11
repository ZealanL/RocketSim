use crate::shared::rsmath;
use crate::vis::camera::CameraConfig;
use crate::{ArenaState, CarState};
use glam::{Vec2, Vec3A};

pub struct CarCam {
    pub car_idx: usize,
    pub face_ball: bool,
}

impl CarCam {
    pub fn new(car_idx: usize) -> Self {
        Self {
            car_idx,
            face_ball: true,
        }
    }

    fn calc_car_cam_dir_2d(
        &self,
        car_state: &CarState,
        _cam_config: &CameraConfig,
        prev_dir: Vec2,
    ) -> Vec2 {
        let car_vel_2d = car_state.vel.truncate();
        let car_forward_2d = car_state.get_forward_dir().truncate().normalize_or_zero();

        // TODO: Move these to the camera config

        const LERP_MAX_VEL: f32 = 500.0;
        const LERP_SCALE_MIN: f32 = 0.3;
        let lerp_speed = (car_vel_2d.length() / LERP_MAX_VEL).clamp(LERP_SCALE_MIN, 1.0);

        const VEL_INFLUENCE_MAX_VEL: f32 = 500.0;
        let vel_influence = (car_vel_2d.length() / VEL_INFLUENCE_MAX_VEL).clamp(0.0, 1.0);

        let new_dir = car_forward_2d.lerp(car_vel_2d.normalize_or_zero(), vel_influence);

        prev_dir.lerp(new_dir, lerp_speed).normalize_or_zero()
    }

    pub fn update_car_cam_pos_dir(
        &self,
        arena_state: &ArenaState,
        cam_config: &CameraConfig,
        pos: &mut Vec3A,
        dir: &mut Vec3A,
    ) {
        let car_state = &arena_state.car_states[self.car_idx];
        let ball_state = &arena_state.ball_state;

        let car_ball_dir = (ball_state.pos - car_state.pos).normalize_or_zero();

        let up_tilt_scale = if self.face_ball {
            let delta_z = ball_state.pos.z - car_state.pos.z;

            let cam_height_range =
                cam_config.car_cam.tilt_ball_max_height - cam_config.car_cam.tilt_ball_min_height;
            assert!(cam_height_range > 0.0);
            let a = ((delta_z - cam_config.car_cam.tilt_ball_min_height) / cam_height_range)
                .clamp(0.0, 1.0);
            let b = car_ball_dir.z.clamp(0.0, 1.0);
            f32::min(a, b)
        } else {
            0.0
        }
        .max(cam_config.car_cam.tilt_min_height_scale)
        .powf(cam_config.car_cam.tilt_exponent);
        let dist_scale = 1.0 - (up_tilt_scale * cam_config.car_cam.tilt_dist_portion);

        let base_pos =
            car_state.pos + Vec3A::new(0.0, 0.0, cam_config.car_cam.height * (1.0 - up_tilt_scale));

        let flat_dir = if self.face_ball {
            (ball_state.pos - base_pos).truncate().normalize_or_zero()
        } else {
            self.calc_car_cam_dir_2d(car_state, cam_config, dir.truncate().normalize_or_zero())
        }
        .extend(0.0)
        .to_vec3a();

        *pos =
            base_pos - (rsmath::to_2d(*dir).normalize_or_zero() * cam_config.car_cam.distance * dist_scale);
        *dir = if self.face_ball {
            // Recalculate once more to make sure it's exactly correct
            (ball_state.pos - *pos).normalize_or_zero()
        } else {
            dir.lerp(flat_dir, 0.1)
        };
    }
}
