//! Car-to-car time-of-impact poses for the demolition gate.
//!
//! Both hitboxes sweep from one tick before to one tick after their
//! start-of-tick poses. The sweep advances toward the allowed penetration.
//! Without a hit, the caller keeps the start-of-tick poses.
//!
//! Without `box-triangle-gjk`, distance queries use SAT and analytic closest
//! features. With that feature, distance queries use GJK/EPA. Both paths use
//! the same motion, margins, advancement rules, and output interpolation.
//!
//! Without `native-math`, orientation math uses `glam` and platform functions.
//! With `native-math`, orientation math uses the engine scalar functions.
//! Values are in Bullet units during the sweep.

mod scalar;

use glam::{Affine3A, Mat3A, Quat, Vec3A};

use self::scalar::{angular_velocity, dot, length, slerp};
use crate::{
    CarBodyConfig, CarState,
    bullet::{
        collision::narrowphase::{CarToiBoxShape, box_sweep_closest_points},
        linear_math::{
            game_get_rotation_scalar, game_integrate_orientation_basis_columns,
            game_quat_normalize_scalar, game_set_rotation_scalar_columns,
        },
    },
    consts::UU_TO_BT,
};

/// Extra overlap that the sweep allows, in Bullet units.
const ALLOWED_PENETRATION: f32 = 0.04;

/// Smallest distance that the loop treats as a hit, in Bullet units.
const MIN_DISTANCE: f32 = 0.001;

/// Relative motion value that cannot produce a new fraction.
const FLOAT_EPSILON: f32 = 1.192_092_9e-7;

/// Maximum number of fraction updates.
const MAX_ITERATIONS: u32 = 64;

/// Smallest squared quaternion length that keeps the pose.
const MIN_QUAT_LEN_SQ: f32 = 1.0e-8;

/// One box pose for the sweep: basis columns and origin in Bullet units.
#[derive(Clone, Copy)]
struct CarToiTransform {
    fwd: Vec3A,
    right: Vec3A,
    up: Vec3A,
    origin: Vec3A,
}

impl CarToiTransform {
    /// Return the world transform of this pose.
    fn affine(self) -> Affine3A {
        Affine3A {
            matrix3: Mat3A::from_cols(self.fwd, self.right, self.up),
            translation: self.origin,
        }
    }

    /// Move this pose by a constant linear and angular velocity over `dt`.
    fn integrate(self, lin: Vec3A, ang: Vec3A, dt: f32) -> Self {
        let mut out = self;
        out.origin = Vec3A::new(
            dt * lin.x + self.origin.x,
            dt * lin.y + self.origin.y,
            dt * lin.z + self.origin.z,
        );

        let mut q_mirror = Quat::IDENTITY;
        game_integrate_orientation_basis_columns(
            &mut out.fwd,
            &mut out.right,
            &mut out.up,
            &mut q_mirror,
            ang,
            dt,
            None,
        );
        out
    }
}

/// Build the start-of-tick pose of one car.
///
/// The car rotation matrix is the source value. Convert it through one
/// quaternion before the sweep.
fn start_transform(state: &CarState) -> CarToiTransform {
    let rot_mat = state.phys.rot_mat;
    let columns = game_set_rotation_scalar_columns(game_get_rotation_scalar(
        rot_mat.x_axis,
        rot_mat.y_axis,
        rot_mat.z_axis,
    ));

    CarToiTransform {
        fwd: columns.x_axis,
        right: columns.y_axis,
        up: columns.z_axis,
        origin: Vec3A::new(
            state.phys.pos.x * UU_TO_BT,
            state.phys.pos.y * UU_TO_BT,
            state.phys.pos.z * UU_TO_BT,
        ),
    }
}

/// Return the poses one tick before and one tick after the start of the tick.
fn sweep_endpoints(
    state: &CarState,
    vel: Vec3A,
    tick_dt: f32,
) -> (CarToiTransform, CarToiTransform) {
    let start = start_transform(state);
    let lin = Vec3A::new(vel.x * UU_TO_BT, vel.y * UU_TO_BT, vel.z * UU_TO_BT);
    let ang = state.phys.ang_vel;

    (
        start.integrate(
            Vec3A::new(-lin.x, -lin.y, -lin.z),
            Vec3A::new(-ang.x, -ang.y, -ang.z),
            tick_dt,
        ),
        start.integrate(lin, ang, tick_dt),
    )
}

/// Move a car pose to the centre of its hitbox.
fn hitbox_transform(transform: CarToiTransform, config: &CarBodyConfig) -> CarToiTransform {
    let offset = Vec3A::new(
        config.hitbox_pos_offset.x * UU_TO_BT,
        config.hitbox_pos_offset.y * UU_TO_BT,
        config.hitbox_pos_offset.z * UU_TO_BT,
    );

    let mut out = transform;
    out.origin = transform.affine().transform_point3a(offset);
    out
}

/// Return the hitbox half extents that the sweep uses, in Bullet units.
///
/// Recover the published size by subtracting 2.5 uu from the configured size.
/// Scale its half extents to Bullet units, then add the 0.025 BT margin.
/// Convert through uu to keep the operation order.
/// `CarBodyConfig` does not store a separate published size.
fn hitbox_half_extents_bt(config: &CarBodyConfig) -> Vec3A {
    let size = config.hitbox_size;
    let published = Vec3A::new(size.x - 2.5, size.y - 2.5, size.z - 2.5);

    let half_with_margin = Vec3A::new(
        (published.x * 0.02) * 0.5 + 0.025,
        (published.y * 0.02) * 0.5 + 0.025,
        (published.z * 0.02) * 0.5 + 0.025,
    );

    let size_uu = Vec3A::new(
        (half_with_margin.x * 50.0) * 2.0,
        (half_with_margin.y * 50.0) * 2.0,
        (half_with_margin.z * 50.0) * 2.0,
    );

    Vec3A::new(
        (size_uu.x * 0.5) * UU_TO_BT,
        (size_uu.y * 0.5) * UU_TO_BT,
        (size_uu.z * 0.5) * UU_TO_BT,
    )
}

/// Return the interpolated origin of the two sweep endpoints.
fn interpolated_origin(from: CarToiTransform, to: CarToiTransform, fraction: f32) -> Vec3A {
    Vec3A::new(
        (to.origin.x * 50.0 - from.origin.x * 50.0) * fraction + from.origin.x * 50.0,
        (to.origin.y * 50.0 - from.origin.y * 50.0) * fraction + from.origin.y * 50.0,
        (to.origin.z * 50.0 - from.origin.z * 50.0) * fraction + from.origin.z * 50.0,
    )
}

/// Return the interpolated rotation of the two sweep endpoints.
fn interpolated_rotation(from: CarToiTransform, to: CarToiTransform, fraction: f32) -> Quat {
    game_quat_normalize_scalar(
        slerp(
            game_get_rotation_scalar(from.fwd, from.right, from.up),
            game_get_rotation_scalar(to.fwd, to.right, to.up),
            fraction,
        ),
        MIN_QUAT_LEN_SQ,
    )
}

/// One impact pose with position in uu and quaternion rotation.
struct ImpactPose {
    pos: Vec3A,
    rot: Quat,
}

/// Return both impact poses, or `None` when the sweep finds no hit.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn time_of_impact_poses(
    a: &CarState,
    a_config: &CarBodyConfig,
    a_vel: Vec3A,
    b: &CarState,
    b_config: &CarBodyConfig,
    b_vel: Vec3A,
    tick_dt: f32,
) -> Option<[ImpactPose; 2]> {
    let box_a = CarToiBoxShape::new(hitbox_half_extents_bt(a_config));
    let box_b = CarToiBoxShape::new(hitbox_half_extents_bt(b_config));

    let (from_a, to_a) = sweep_endpoints(a, a_vel, tick_dt);
    let (from_b, to_b) = sweep_endpoints(b, b_vel, tick_dt);

    let ca0 = hitbox_transform(from_a, a_config);
    let ca1 = hitbox_transform(to_a, a_config);
    let cb0 = hitbox_transform(from_b, b_config);
    let cb1 = hitbox_transform(to_b, b_config);

    let lin_a = ca1.origin - ca0.origin;
    let lin_b = cb1.origin - cb0.origin;

    let ang_a = angular_velocity(ca0.fwd, ca0.right, ca0.up, ca1.fwd, ca1.right, ca1.up);
    let ang_b = angular_velocity(cb0.fwd, cb0.right, cb0.up, cb1.fwd, cb1.right, cb1.up);

    let max_ang =
        length(ang_a) * box_a.angular_motion_disc() + length(ang_b) * box_b.angular_motion_disc();
    let rel = lin_b - lin_a;

    if length(rel) + max_ang == 0.0 {
        return None;
    }

    let mut contact = box_sweep_closest_points(&box_a, &ca0.affine(), &box_b, &cb0.affine())?;

    let mut distance = contact.distance + ALLOWED_PENETRATION;
    if dot(rel, contact.normal) + max_ang <= FLOAT_EPSILON {
        return None;
    }

    let mut lambda = 0.0f32;
    let mut fraction = 0.0f32;
    let mut iter = 0u32;

    while distance > MIN_DISTANCE {
        let projection = dot(rel, contact.normal) + max_ang;
        if projection <= FLOAT_EPSILON {
            return None;
        }

        fraction = lambda + distance / projection;
        if fraction > 1.0 || fraction < 0.0 || fraction <= lambda {
            return None;
        }

        let ia = ca0.integrate(lin_a, ang_a, fraction);
        let ib = cb0.integrate(lin_b, ang_b, fraction);

        contact = box_sweep_closest_points(&box_a, &ia.affine(), &box_b, &ib.affine())?;

        distance = contact.distance + ALLOWED_PENETRATION;
        iter += 1;
        if iter > MAX_ITERATIONS {
            return None;
        }

        lambda = fraction;
    }

    if !(fraction < 1.0) {
        return None;
    }

    Some([
        ImpactPose {
            pos: interpolated_origin(from_a, to_a, fraction),
            rot: interpolated_rotation(from_a, to_a, fraction),
        },
        ImpactPose {
            pos: interpolated_origin(from_b, to_b, fraction),
            rot: interpolated_rotation(from_b, to_b, fraction),
        },
    ])
}

/// Replace both poses with the time-of-impact poses used by the demolition
/// cone.
///
/// Both hitboxes sweep from one tick before to one tick after their
/// start-of-tick poses, with the given start velocities. On failure, both
/// poses keep their start-of-tick values.
pub(super) fn get_time_of_impact_poses(
    a: &mut CarState,
    a_config: &CarBodyConfig,
    a_vel: Vec3A,
    b: &mut CarState,
    b_config: &CarBodyConfig,
    b_vel: Vec3A,
    tick_time: f32,
) {
    let Some([pose_a, pose_b]) =
        time_of_impact_poses(a, a_config, a_vel, b, b_config, b_vel, tick_time)
    else {
        return;
    };

    a.phys.pos = pose_a.pos;
    a.phys.rot_mat = game_set_rotation_scalar_columns(pose_a.rot);
    b.phys.pos = pose_b.pos;
    b.phys.rot_mat = game_set_rotation_scalar_columns(pose_b.rot);
}
