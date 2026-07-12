use std::{
    f32::consts::{FRAC_PI_2, PI, TAU},
    ops::Sub,
};

use glam::Vec3A;

#[derive(Clone, Copy, Debug, Default)]
pub struct Angle {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

impl From<Vec3A> for Angle {
    fn from(forward: Vec3A) -> Self {
        let (yaw, pitch);

        if forward.y.abs() > f32::EPSILON || forward.x.abs() > f32::EPSILON {
            yaw = forward.y.atan2(forward.x);

            let dist_2d = forward.with_z(0.0).length();
            pitch = -(-forward.z).atan2(dist_2d);
        } else {
            yaw = 0.0;
            pitch = if forward.z > f32::EPSILON {
                FRAC_PI_2
            } else if forward.z < -f32::EPSILON {
                -FRAC_PI_2
            } else {
                0.0
            }
        }

        Self {
            yaw,
            pitch,
            roll: 0.0,
        }
    }
}

impl Sub for Angle {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            yaw: self.yaw - rhs.yaw,
            pitch: self.pitch - rhs.pitch,
            roll: self.roll - rhs.roll,
        }
    }
}

impl Angle {
    pub fn normalize_fix(&mut self) {
        self.yaw = (self.yaw + PI).rem_euclid(TAU) - PI;
        self.pitch = (self.pitch + FRAC_PI_2).rem_euclid(PI) - FRAC_PI_2;
        self.roll = (self.roll + PI).rem_euclid(TAU) - PI;
    }

    pub fn get_forward_vec(self) -> Vec3A {
        let (sp, cp) = (-self.pitch).sin_cos();
        let (sy, cy) = self.yaw.sin_cos();

        Vec3A::new(cp * cy, cp * sy, -sp)
    }

    /// See: <https://unrealarchive.org/wikis/unreal-wiki/Rotator.html>
    ///
    /// You can determine the rounding from measuring the resulting vector directions from conversions
    pub fn round_ue3(self) -> Self {
        const TO_INTS: f32 = (1 << 15) as f32 / PI;
        const BACK_TO_RADIANS: f32 = (1.0 / TO_INTS) * 4.0;
        const ROUNDING_MASK: i32 = 0x4000 - 1;

        let r_yaw = ((self.yaw * TO_INTS) as i32 >> 2) & ROUNDING_MASK;
        let r_pitch = ((self.pitch * TO_INTS) as i32 >> 2) & ROUNDING_MASK;
        debug_assert!(self.roll.abs() < f32::EPSILON);

        Self {
            yaw: r_yaw as f32 * BACK_TO_RADIANS,
            pitch: r_pitch as f32 * BACK_TO_RADIANS,
            roll: 0.0,
        }
    }
}
