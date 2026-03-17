use glam::{Mat3A, Vec3A};

pub struct JacbobianBody<'a> {
    pub world: &'a Mat3A,
    pub rel_pos: Vec3A,
    pub inertia_inv: Vec3A,
    pub mass_inv: f32,
}

pub fn get_jacobian_diagonal(
    body_a: &JacbobianBody<'_>,
    body_b: &JacbobianBody<'_>,
    joint_axis: Vec3A,
) -> f32 {
    let a_j = body_a
        .world
        .mul_transpose_vec3a(body_a.rel_pos.cross(joint_axis));
    let b_j = body_b
        .world
        .mul_transpose_vec3a(body_b.rel_pos.cross(-joint_axis));
    let min_v_jt_0 = body_a.inertia_inv * a_j;
    let min_v_jt_1 = body_b.inertia_inv * b_j;

    body_a.mass_inv + min_v_jt_0.dot(a_j) + body_b.mass_inv + min_v_jt_1.dot(b_j)
}
