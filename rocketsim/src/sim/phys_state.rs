use std::fmt::Display;

use glam::{Mat3A, Vec3A};

/// Default is not implemented for this struct,
/// because the initial start height of the ball/car is different.
/// The correct values are set in `BallState::default()` and `CarState::default()`
#[derive(Clone, Copy, Debug)]
pub struct PhysState {
    pub pos: Vec3A,
    pub rot_mat: Mat3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
}

impl PhysState {
    /// Flip Y axis (aka rotate 180 degrees around Z axis)
    pub fn flip_y(mut self) -> Self {
        const INVERT_SCALE: Vec3A = Vec3A::new(-1.0, -1.0, 1.0);

        self.pos *= INVERT_SCALE;
        self.vel *= INVERT_SCALE;
        self.ang_vel *= INVERT_SCALE;

        for i in 0..3 {
            *self.rot_mat.col_mut(i) *= INVERT_SCALE;
        }

        self
    }

    /// Mirror along X axis (Reflection across the YZ plane)
    pub fn mirror_x(mut self) -> Self {
        const FLIP_SCALES: Vec3A = Vec3A::new(-1.0, 1.0, 1.0);

        self.pos *= FLIP_SCALES;
        self.vel *= FLIP_SCALES;

        // Thanks Rolv, JPK, and Kaiyo!
        self.rot_mat.x_axis *= FLIP_SCALES;
        self.rot_mat.y_axis *= -FLIP_SCALES;
        self.rot_mat.z_axis *= FLIP_SCALES;

        self.ang_vel *= -FLIP_SCALES;

        self
    }

    /// Mirror along Y axis (Reflection across the XZ plane)
    pub fn mirror_y(mut self) -> Self {
        const FLIP_SCALES: Vec3A = Vec3A::new(1.0, -1.0, 1.0);

        self.pos *= FLIP_SCALES;
        self.vel *= FLIP_SCALES;

        self.rot_mat.x_axis *= FLIP_SCALES;
        self.rot_mat.y_axis *= -FLIP_SCALES;
        self.rot_mat.z_axis *= FLIP_SCALES;

        self.ang_vel *= -FLIP_SCALES;

        self
    }

    pub const fn get_forward_dir(&self) -> Vec3A {
        self.rot_mat.x_axis
    }

    pub const fn get_right_dir(&self) -> Vec3A {
        self.rot_mat.y_axis
    }

    pub const fn get_up_dir(&self) -> Vec3A {
        self.rot_mat.z_axis
    }
}

impl Display for PhysState {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str("PhysState {")?;
        f.write_fmt(format_args!("\n\tpos: {}", self.pos))?;
        f.write_fmt(format_args!("\n\trot_mat: {}", self.rot_mat))?;
        f.write_fmt(format_args!("\n\tvel: {}", self.vel))?;
        f.write_fmt(format_args!("\n\tang_vel: {}", self.ang_vel))?;
        f.write_str("}")
    }
}
