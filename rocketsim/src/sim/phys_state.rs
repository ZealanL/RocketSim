use std::fmt::Display;

use glam::{Mat3A, Vec3A};

/// Rigid-body pose + velocity shared by balls and cars.
///
/// All values are in Unreal units (uu): `pos`/`vel` in uu and uu/s,
/// `ang_vel` in rad/s, `rot_mat` columns are forward (`x`), right (`y`),
/// up (`z`).
///
/// There is deliberately no `Default`: use [`crate::BallState::DEFAULT`] or
/// [`crate::CarState::DEFAULT`] so the spawn height is correct.
#[derive(Clone, Copy, Debug)]
pub struct PhysState {
    pub pos: Vec3A,
    pub rot_mat: Mat3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
}

impl PhysState {
    /// Flip across the field center (rotate 180° about Z).
    ///
    /// Used to mirror Blue spawns to Orange and vice versa.
    #[must_use]
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

    /// Mirror across the YZ plane (`x -> -x`), including rotation/ang-vel.
    #[must_use]
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

    /// Mirror across the XZ plane (`y -> -y`), including rotation/ang-vel.
    #[must_use]
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

    /// Car/ball forward axis (`rot_mat.x_axis`).
    #[must_use]
    pub const fn get_forward_dir(&self) -> Vec3A {
        self.rot_mat.x_axis
    }

    /// Car/ball right axis (`rot_mat.y_axis`).
    #[must_use]
    pub const fn get_right_dir(&self) -> Vec3A {
        self.rot_mat.y_axis
    }

    /// Car/ball up axis (`rot_mat.z_axis`).
    #[must_use]
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
