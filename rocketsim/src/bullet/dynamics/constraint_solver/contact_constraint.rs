use glam::Vec3A;

use super::contact_solver_info;
use crate::bullet::dynamics::rigid_body::RigidBody;

pub fn resolve_single_collision(
    body1: &RigidBody,
    body2: &RigidBody,
    contact_pos_world: Vec3A,
    contact_normal_on_b: Vec3A,
    time_step: f32,
    distance: f32,
) -> f32 {
    let rel_pos1 = contact_pos_world - body1.get_world_trans().translation;
    let rel_pos2 = contact_pos_world - body2.get_world_trans().translation;

    let vel1 = body1.get_vel_in_local_point(rel_pos1);
    let vel2 = body2.get_vel_in_local_point(rel_pos2);
    let vel = vel1 - vel2;
    let rel_vel = contact_normal_on_b.dot(vel);

    let positional_error = contact_solver_info::WHEEL_PUSHBACK_ERP * -distance / time_step;
    let vel_error = -rel_vel;
    let denom0 = body1.compute_impulse_denominator(contact_pos_world, contact_normal_on_b);
    let denom1 = body2.compute_impulse_denominator(contact_pos_world, contact_normal_on_b);
    let jac_diag_ab_inv = 1.0 / (denom0 + denom1);

    let penetration_impulse = positional_error * jac_diag_ab_inv;
    let vel_impulse = vel_error * jac_diag_ab_inv;

    let normal_impulse = penetration_impulse + vel_impulse;
    normal_impulse.max(0.0)
}

pub fn resolve_single_bilateral_fake_ground(body1: &RigidBody, pos: Vec3A, normal: Vec3A) -> f32 {
    const CONTACT_DAMPING: f32 = -0.2;

    debug_assert!(normal.is_normalized());
    let body1_comt = body1.get_world_trans();

    let rel_pos1 = pos - body1_comt.translation;

    let vel1 = body1.get_vel_in_local_point(rel_pos1);

    // The fake ground body has zero position offset, inverse inertia, and
    // inverse mass by construction, so its Jacobian terms are exactly zero
    // and only add `+ 0.0` twice to a strictly positive diagonal (the chassis
    // has non-zero mass). Skip them instead of computing them per wheel.
    let a_j = body1_comt
        .matrix3
        .mul_transpose_vec3a(rel_pos1.cross(normal));
    let jac_diag_ab = body1.inv_mass + (body1.inv_inertia_local * a_j).dot(a_j);
    let jac_diag_ab_inv = 1.0 / jac_diag_ab;
    let rel_vel = normal.dot(vel1);

    CONTACT_DAMPING * rel_vel * jac_diag_ab_inv
}
