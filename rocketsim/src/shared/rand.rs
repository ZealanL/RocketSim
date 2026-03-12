#![allow(dead_code)]

use std::f32::consts::PI;
use glam::{Mat3A, Quat, Vec3A};

/// Returns a random number from 0.0 to 1.0
pub fn rand_frac() -> f32 {
    fastrand::f32()
}

pub fn rand_f32(min: f32, max: f32) -> f32 {
    rand_frac() * (max - min) + min
}

pub fn rand_vec(min: Vec3A, max: Vec3A) -> Vec3A {
    assert!(min.cmple(max).all(), "Minimum vector {min} must be <= maximum vector {max}");
    Vec3A::new(
        rand_f32(min[0], max[0]),
        rand_f32(min[1], max[1]),
        rand_f32(min[2], max[2]),
    )
}

/// Returns a random Vec3A with each component uniformly distributed from -1.0 to 1.0
pub fn rand_unit_cube_vec() -> Vec3A {
    Vec3A::new(
        rand_f32(-1.0, 1.0),
        rand_f32(-1.0, 1.0),
        rand_f32(-1.0, 1.0),
    )
}

/// NOTE: Uniformly distributed over all possible directions (without cube bias)
pub fn rand_norm_vec() -> Vec3A {
    loop { // TODO: This will fail about 50% of the time, is generating guassian numbers better?
        let v = rand_unit_cube_vec();
        const EPS: f32 = 1e-3;
        let sq_len = v.length_squared();
        if (EPS.powi(2)..1.0).contains(&sq_len) {
            return v / sq_len.sqrt();
        }
    }
}

/// Returns a random quaternion uniformly distributed over all possible rotations
///
/// Uses a cool algorithm invented by https://history.siggraph.org/person/ken-shoemake
/// (See: https://www.sciencedirect.com/science/chapter/edited-volume/abs/pii/B9780080507552500361)
pub fn rand_quat() -> Quat {
    let u1 = rand_frac();
    let (s1, c1) = (2.0 * PI * rand_frac()).sin_cos();
    let (s2, c2) = (2.0 * PI * rand_frac()).sin_cos();

    let r1 = (1.0 - u1).sqrt();
    let r2 = u1.sqrt();

    Quat::from_xyzw(r1 * s1, r1 * c1, r2 * s2, r2 * c2)
}

pub fn rand_rot_mat() -> Mat3A {
    Mat3A::from_quat(rand_quat())
}

/// Unformly-random rotation matrix where `up = [0, 0, 1]`
pub fn rand_rot_mat_upright() -> Mat3A {
    Mat3A::from_rotation_z(rand_f32(0.0, 2.0 * PI))
}

macro_rules! make_rand_int_fn {
    ($($name:ident: $t:ident),*) => {
        $(
            pub fn $name(min: $t, max_exclusive: $t) -> $t {
                fastrand::$t(min..max_exclusive)
            }
        )*
    };
}
make_rand_int_fn! {
    rand_u8: u8,
    rand_u16: u16,
    rand_u32: u32,
    rand_u64: u64,

    rand_i8: i8,
    rand_i16: i16,
    rand_i32: i32,
    rand_i64: i64,

    rand_usize: usize
}