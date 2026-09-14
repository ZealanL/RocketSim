use glam::Vec3A;

use crate::{
    GameMode, WheelRaycastMode,
    bullet::{
        collision::dispatch::quad_ray_callbacks::{
            ClosestQuadRayResultCallback, QuadRayResultCallback,
        },
        dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
    },
    consts::{UU_TO_BT, arena::get_aabb},
};

#[derive(Clone, Copy)]
pub struct VehicleRaycasterResult<'a> {
    pub hit_point_in_world: Vec3A,
    pub hit_normal_in_world: Vec3A,
    pub rigid_body_idx: usize,
    pub rigid_body: &'a RigidBody,
}

/// Squared minimum distance from a wheel hard point to an accepted ray
/// hit, BT^2. Live target closest-hit acceptance field. Rejects grazes
/// off the contact patch (e.g. the extra wheel-on-ball hit in
/// `cb_turtle_land` i39) during closest-hit selection, so a rejected near
/// hit never hides a farther valid candidate on the same ray.
pub const WHEEL_RAY_MIN_HIT_DIST_SQ: f32 = 0.116684;

pub struct VehicleRaycaster {
    added_filter_mask: u8,
    game_mode: GameMode,
    mode: WheelRaycastMode,
}

impl VehicleRaycaster {
    pub const fn new(
        added_filter_mask: u8,
        game_mode: GameMode,
        mode: WheelRaycastMode,
    ) -> Self {
        Self {
            added_filter_mask,
            game_mode,
            mode,
        }
    }

    /// Analytic arena-plane suspension ray: the reference
    /// `SuspensionCollisionGrid` unmarked-cell path applied to every wheel
    /// ray. The ray only sees the floor, ceiling, and side-wall planes; the
    /// arena trimesh and dynamic bodies are ignored.
    fn cast_planes<'a>(
        &self,
        collision_world: &'a DiscreteDynamicsWorld,
        from: &[Vec3A; 4],
        to: &[Vec3A; 4],
    ) -> [Option<VehicleRaycasterResult<'a>>; 4] {
        let aabb = get_aabb(self.game_mode);
        let extent_x = aabb.max.x * UU_TO_BT;
        let extent_y = aabb.max.y * UU_TO_BT;
        let height = aabb.max.z * UU_TO_BT;
        let is_hoops = self.game_mode == GameMode::Hoops;

        // The stand-in for the reference `defaultWorldCollisionRB`
        // (`_worldCollisionRBs[0]`): any static body with contact response
        // identifies the hit as world contact and serves as the pushback
        // resolve target.
        let Some(co_idx) = collision_world
            .bodies()
            .iter()
            .position(|b| b.is_static_obj() && b.has_contact_response())
        else {
            return [None; 4];
        };
        let body = &collision_world.bodies()[co_idx];

        let mut results = [None; 4];
        for (i, (&start, &end)) in from.iter().zip(to.iter()).enumerate() {
            let delta = end - start;
            let dist = delta.length();
            if dist == 0.0 {
                continue;
            }
            let dir = delta / dist;

            let mut dist_to_plane = f32::MAX;
            let mut normal = Vec3A::ZERO;
            if end.z <= 0.0 || end.z >= height {
                if dir.z < 0.0 {
                    dist_to_plane = (5.96e-8 - start.z) / dir.z;
                    normal = Vec3A::Z;
                } else {
                    dist_to_plane = (height - start.z) / dir.z;
                    normal = Vec3A::NEG_Z;
                }
            } else {
                if dir.x.signum() == start.x.signum() {
                    dist_to_plane = (start.x.abs() - extent_x).abs() / dir.x.abs();
                    normal = Vec3A::new(-end.x.signum(), 0.0, 0.0);
                }
                if is_hoops && dir.y.signum() == start.y.signum() {
                    dist_to_plane = (start.y.abs() - extent_y).abs() / dir.y.abs();
                    normal = Vec3A::new(0.0, -end.y.signum(), 0.0);
                }
            }

            if dist_to_plane < dist {
                results[i] = Some(VehicleRaycasterResult {
                    hit_point_in_world: start + dir * dist_to_plane,
                    hit_normal_in_world: normal,
                    rigid_body_idx: co_idx,
                    rigid_body: body,
                });
            }
        }
        results
    }

    pub fn cast_rays<'a>(
        &self,
        collision_world: &'a DiscreteDynamicsWorld,
        from: &[Vec3A; 4],
        to: &[Vec3A; 4],
        ignore_obj: &RigidBody,
    ) -> [Option<VehicleRaycasterResult<'a>>; 4] {
        if self.mode == WheelRaycastMode::ArenaPlanes {
            return self.cast_planes(collision_world, from, to);
        }

        let mut ray_callback = ClosestQuadRayResultCallback::new(from, to, Some(ignore_obj));
        ray_callback.base.collision_filter_group |= self.added_filter_mask;
        ray_callback.base.min_hit_dist_sq = WHEEL_RAY_MIN_HIT_DIST_SQ;
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
