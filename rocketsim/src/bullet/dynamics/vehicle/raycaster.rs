use glam::Vec3A;

use crate::bullet::{
    collision::dispatch::quad_ray_callbacks::{
        ClosestQuadRayResultCallback, QuadRayResultCallback,
    },
    dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
};

#[derive(Clone, Copy)]
pub struct VehicleRaycasterResult<'a> {
    pub hit_point_in_world: Vec3A,
    pub hit_normal_in_world: Vec3A,
    pub rigid_body_idx: usize,
    pub rigid_body: &'a RigidBody,
}

#[derive(Clone, Copy)]
pub struct VehicleRaycaster {
    added_filter_mask: u8,
    min_hit_dist_sq: f32,
}

impl VehicleRaycaster {
    pub const fn new(added_filter_mask: u8, min_hit_dist_sq: f32) -> Self {
        Self {
            added_filter_mask,
            min_hit_dist_sq,
        }
    }

    pub fn cast_rays<'a>(
        &self,
        collision_world: &'a DiscreteDynamicsWorld,
        from: &[Vec3A; 4],
        to: &[Vec3A; 4],
        ignore_obj: &RigidBody,
    ) -> [Option<VehicleRaycasterResult<'a>>; 4] {
        let mut ray_callback = ClosestQuadRayResultCallback::new(from, to, Some(ignore_obj));
        ray_callback.base.collision_filter_group |= self.added_filter_mask;
        ray_callback.base.min_hit_dist_sq = self.min_hit_dist_sq;
        collision_world.ray_test(from, to, &mut ray_callback);

        let mut results = [None; 4];

        for (i, result) in results.iter_mut().enumerate() {
            if ray_callback.has_hit(i)
                && let Some(co_idx) = ray_callback.base.collision_obj_idx[i]
            {
                let rb = &collision_world.bodies()[co_idx];
                if rb.has_contact_response() {
                    *result = Some(VehicleRaycasterResult {
                        rigid_body_idx: co_idx,
                        rigid_body: rb,
                        hit_point_in_world: ray_callback.hit_point_world[i],
                        hit_normal_in_world: ray_callback.hit_normal_world[i].normalize_or_zero(),
                    });
                }
            }
        }

        results
    }
}
