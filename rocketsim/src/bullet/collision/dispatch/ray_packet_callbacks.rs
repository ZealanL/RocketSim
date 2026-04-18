use glam::{Vec3A, Vec4};

use crate::bullet::{
    collision::{
        broadphase::{BroadphaseAabbCallback, BroadphaseProxy, CollisionFilterGroups},
        dispatch::collision_world::CollisionWorld,
        shapes::{triangle_callback::ProcessRayPacketTriangle, triangle_shape::TriangleShape},
    },
    dynamics::rigid_body::RigidBody,
    linear_math::interpolate_3,
};

pub struct LocalRayResult {
    collision_obj_idx: usize,
    hit_normal_world: Vec3A,
    hit_fraction: f32,
}

#[derive(Clone, Copy)]
pub struct RayResultCallbackBase {
    pub closest_hit_fraction: Vec4,
    pub collision_obj_idx: [Option<usize>; 4],
    pub ignore_obj_world_idx: Option<usize>,
    pub collision_filter_group: u8,
    pub collision_filter_mask: u8,
}

impl Default for RayResultCallbackBase {
    fn default() -> Self {
        Self {
            closest_hit_fraction: Vec4::ONE,
            collision_obj_idx: [None; 4],
            collision_filter_group: CollisionFilterGroups::Default as u8,
            collision_filter_mask: CollisionFilterGroups::All as u8,
            ignore_obj_world_idx: None,
        }
    }
}

pub trait RayResultCallback {
    fn get_base(&self) -> &RayResultCallbackBase;
    fn has_hit(&self, ray_idx: usize) -> bool {
        self.get_base().collision_obj_idx[ray_idx].is_some()
    }
    fn needs_collision(&self, proxy0: &BroadphaseProxy) -> bool {
        let base = self.get_base();
        if base.ignore_obj_world_idx == Some(proxy0.client_obj_idx) {
            return false;
        }

        proxy0.collision_filter_group & base.collision_filter_mask != 0
            && base.collision_filter_group & proxy0.collision_filter_mask != 0
    }
    fn add_single_result(&mut self, ray_result: LocalRayResult, ray_idx: usize);
}

pub struct ClosestRayResultCallback<'a> {
    pub base: RayResultCallbackBase,
    ray_from_world: &'a [Vec3A; 4],
    ray_to_world: &'a [Vec3A; 4],
    pub hit_normal_world: [Vec3A; 4],
    pub hit_point_world: [Vec3A; 4],
}

impl<'a> ClosestRayResultCallback<'a> {
    pub fn new(
        ray_from_world: &'a [Vec3A; 4],
        ray_to_world: &'a [Vec3A; 4],
        ignore_obj: &RigidBody,
    ) -> Self {
        Self {
            base: RayResultCallbackBase {
                ignore_obj_world_idx: Some(ignore_obj.world_array_idx),
                ..Default::default()
            },
            ray_from_world,
            ray_to_world,
            hit_normal_world: [Vec3A::ZERO; 4],
            hit_point_world: [Vec3A::ZERO; 4],
        }
    }
}

impl RayResultCallback for ClosestRayResultCallback<'_> {
    #[inline]
    fn get_base(&self) -> &RayResultCallbackBase {
        &self.base
    }

    fn add_single_result(&mut self, ray_result: LocalRayResult, ray_idx: usize) {
        if ray_result.hit_fraction > self.base.closest_hit_fraction[ray_idx] {
            return;
        }

        self.base.closest_hit_fraction[ray_idx] = ray_result.hit_fraction;
        self.hit_normal_world[ray_idx] = ray_result.hit_normal_world;

        self.base.collision_obj_idx[ray_idx] = Some(ray_result.collision_obj_idx);
        self.hit_point_world[ray_idx] = interpolate_3(
            self.ray_from_world[ray_idx],
            self.ray_to_world[ray_idx],
            ray_result.hit_fraction,
        );
    }
}

pub struct QuadRayCallback<'a, T: RayResultCallback> {
    ray_from_world: &'a [Vec3A; 4],
    ray_to_world: &'a [Vec3A; 4],
    world: &'a CollisionWorld,
    result_callback: &'a mut T,
}

impl<'a, T: RayResultCallback> QuadRayCallback<'a, T> {
    pub const fn new(
        ray_from_world: &'a [Vec3A; 4],
        ray_to_world: &'a [Vec3A; 4],
        world: &'a CollisionWorld,
        result_callback: &'a mut T,
    ) -> Self {
        Self {
            ray_from_world,
            ray_to_world,
            world,
            result_callback,
        }
    }
}

impl<T: RayResultCallback> BroadphaseAabbCallback for QuadRayCallback<'_, T> {
    fn process(&mut self, proxy: &BroadphaseProxy) -> bool {
        let obj_idx = proxy.client_obj_idx;
        let rb = &self.world.collision_objs[proxy.client_obj_idx];
        let handle_idx = rb.get_broadphase_handle().unwrap();
        let handle = &self.world.broadphase_pair_cache.handles[handle_idx];

        if self.result_callback.needs_collision(handle) {
            CollisionWorld::quad_ray_test(
                self.ray_from_world,
                self.ray_to_world,
                rb,
                obj_idx,
                self.result_callback,
            );
        }

        true
    }
}

pub struct BridgeTriangleRaycastPacketCallback<'a, T: RayResultCallback> {
    pub to: &'a [Vec3A; 4],
    pub from: &'a [Vec3A; 4],
    pub hit_fraction: Vec4,
    pub collision_obj: &'a RigidBody,
    pub collision_obj_idx: usize,
    pub result_callback: &'a mut T,
}

impl<T: RayResultCallback> BridgeTriangleRaycastPacketCallback<'_, T> {
    fn internal_report_hit(&mut self, hit_normal_local: Vec3A, hit_fraction: f32, ray_idx: usize) {
        let hit_normal_world = self.collision_obj.get_world_trans().matrix3 * hit_normal_local;

        let ray_result = LocalRayResult {
            hit_fraction,
            hit_normal_world,
            collision_obj_idx: self.collision_obj_idx,
        };
        self.result_callback.add_single_result(ray_result, ray_idx);
    }

    pub fn report_hit(&mut self, hit_normal_local: Vec3A, hit_fraction: f32, ray_idx: usize) {
        if hit_fraction >= self.hit_fraction[ray_idx] {
            return;
        }

        self.internal_report_hit(hit_normal_local, hit_fraction, ray_idx);
    }

    fn process_triangle(&mut self, triangle: &TriangleShape, lambda_max: &mut f32, ray_idx: usize) {
        const EDGE_TOLERANCE: f32 = -0.0001;

        let dist = triangle.points[0].dot(triangle.normal);
        let dist_a = triangle.normal.dot(self.from[ray_idx]) - dist;
        let dist_b = triangle.normal.dot(self.to[ray_idx]) - dist;
        if dist_a * dist_b >= 0.0 {
            return; // same sign
        }

        let proj_length = dist_a - dist_b;
        let distance = dist_a / proj_length;
        if distance >= self.hit_fraction[ray_idx] {
            *lambda_max = self.hit_fraction[ray_idx];
            return;
        }

        let point = self.from[ray_idx].lerp(self.to[ray_idx], distance);
        let v0p = triangle.points[0] - point;
        let v1p = triangle.points[1] - point;
        let cp0 = v0p.cross(v1p);
        if cp0.dot(triangle.normal) < EDGE_TOLERANCE {
            return;
        }

        let v2p = triangle.points[2] - point;
        let cp1 = v1p.cross(v2p);
        if cp1.dot(triangle.normal) < EDGE_TOLERANCE {
            return;
        }

        let cp2 = v2p.cross(v0p);
        if cp2.dot(triangle.normal) < EDGE_TOLERANCE {
            return;
        }

        *lambda_max = distance;
        if dist_a <= 0.0 {
            self.internal_report_hit(-triangle.normal, distance, ray_idx);
        } else {
            self.internal_report_hit(triangle.normal, distance, ray_idx);
        }
    }
}

impl<T: RayResultCallback> ProcessRayPacketTriangle for BridgeTriangleRaycastPacketCallback<'_, T> {
    fn process_packet_node(
        &mut self,
        triangle: &TriangleShape,
        active_mask: u8,
        lambda_max: &mut Vec4,
    ) {
        for i in 0..4 {
            if (active_mask & (1 << i)) != 0 {
                self.process_triangle(triangle, &mut lambda_max[i], i);
            }
        }
    }
}
