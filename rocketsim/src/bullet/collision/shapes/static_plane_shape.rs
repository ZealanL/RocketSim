use glam::{Affine3A, Vec3A};

use crate::{
    bullet::collision::dispatch::quad_ray_callbacks::{
        BridgeTriQuadRayCallback, QuadRayResultCallback,
    },
    bullet::collision::shapes::{
        triangle_callback::ProcessQuadRayTriangle, triangle_shape::TriangleShape,
    },
    bullet::linear_math::bullet_normalize,
    shared::{Aabb, QuadRayInfo},
};

pub struct StaticPlaneShape {
    plane_normal: Vec3A,
    is_single_axis: bool,
    single_axis_idx: usize,
    single_axis_backwards: bool,
    pub aabb_ident_cache: Aabb,
    pub aabb_cache: Aabb,
    #[cfg(debug_assertions)]
    pub aabb_cache_trans: Affine3A,
}

impl StaticPlaneShape {
    pub fn new(world_trans: Affine3A, plane_normal: Vec3A) -> Self {
        let plane_normal = bullet_normalize(plane_normal);
        debug_assert!(plane_normal.is_normalized());

        let [x, y, z]: [bool; 3] = plane_normal.abs().cmpge(Vec3A::splat(f32::EPSILON)).into();

        let (is_single_axis, single_axis_idx, single_axis_backwards) =
            if u8::from(x) + u8::from(y) + u8::from(z) == 1 {
                let axis = plane_normal.abs().max_position();

                (true, axis, plane_normal[axis].is_sign_negative())
            } else {
                (false, 0, false)
            };

        let mut plane = Self {
            plane_normal,
            is_single_axis,
            single_axis_idx,
            single_axis_backwards,
            aabb_ident_cache: Aabb::ZERO,
            aabb_cache: Aabb::ZERO,
            #[cfg(debug_assertions)]
            aabb_cache_trans: world_trans,
        };

        plane.aabb_ident_cache = plane.get_aabb(&Affine3A::IDENTITY);
        plane.aabb_cache = plane.get_aabb(&world_trans);

        plane
    }

    pub fn get_aabb(&self, t: &Affine3A) -> Aabb {
        let mut min = Vec3A::MIN;
        let mut max = Vec3A::MAX;

        if self.is_single_axis {
            const PLANE_CONSTANT_OFFSET: f32 = 0.2;

            min[self.single_axis_idx] = t.translation[self.single_axis_idx] - PLANE_CONSTANT_OFFSET;
            max[self.single_axis_idx] = t.translation[self.single_axis_idx] + PLANE_CONSTANT_OFFSET;

            (if self.single_axis_backwards {
                &mut max
            } else {
                &mut min
            })[self.single_axis_idx] = if self.single_axis_backwards {
                f32::MAX
            } else {
                f32::MIN
            };
        }

        Aabb { min, max }
    }

    pub const fn get_plane_normal(&self) -> Vec3A {
        self.plane_normal
    }

    pub fn perform_quad_raycast<T: QuadRayResultCallback>(
        &self,
        result_callback: &mut BridgeTriQuadRayCallback<T>,
        ray_info: &QuadRayInfo,
    ) {
        let plane = &self.aabb_ident_cache;
        if !ray_info.aabb.intersects(plane) {
            return;
        }

        // btStaticPlaneShape::processAllTriangles generates two finite
        // triangles covering the query AABB, then routes them through
        // btTriangleRaycastCallback. Reproduce that sequence instead of
        // intersecting the infinite plane directly; the arithmetic affects
        // the final hit point by a few ulps and feeds wheel friction.
        let mut lambda_max = result_callback.hit_fraction;

        // The C++ vehicle raycaster submits each wheel ray independently, so
        // btStaticPlaneShape receives that single segment's AABB. Build the
        // two covering triangles per lane rather than using the union AABB of
        // the packet (which would change the generated radius).
        for i in 0..4 {
            let ray_min = ray_info.ray_sources[i].min(ray_info.ray_targets[i]);
            let ray_max = ray_info.ray_sources[i].max(ray_info.ray_targets[i]);
            let half_extents = 0.5 * (ray_max - ray_min);
            let radius = half_extents.length();
            let center = 0.5 * (ray_max + ray_min);

            let projected_center = center - self.plane_normal.dot(center) * self.plane_normal;
            let (tangent_0, tangent_1) = if self.plane_normal.z.abs() > 0.7071067811865475 {
                let a = self.plane_normal.y * self.plane_normal.y
                    + self.plane_normal.z * self.plane_normal.z;
                let k = a.sqrt().recip();
                let p = Vec3A::new(0.0, -self.plane_normal.z * k, self.plane_normal.y * k);
                let q = Vec3A::new(a * k, -self.plane_normal.x * p.z, self.plane_normal.x * p.y);
                (p, q)
            } else {
                let a = self.plane_normal.x * self.plane_normal.x
                    + self.plane_normal.y * self.plane_normal.y;
                let k = a.sqrt().recip();
                let p = Vec3A::new(-self.plane_normal.y * k, self.plane_normal.x * k, 0.0);
                let q = Vec3A::new(-self.plane_normal.z * p.y, self.plane_normal.z * p.x, a * k);
                (p, q)
            };

            let triangles = [
                [
                    projected_center + tangent_0 * radius + tangent_1 * radius,
                    projected_center + tangent_0 * radius - tangent_1 * radius,
                    projected_center - tangent_0 * radius - tangent_1 * radius,
                ],
                [
                    projected_center - tangent_0 * radius - tangent_1 * radius,
                    projected_center - tangent_0 * radius + tangent_1 * radius,
                    projected_center + tangent_0 * radius + tangent_1 * radius,
                ],
            ];

            for points in triangles {
                // process_triangle recomputes Bullet's unnormalized normal;
                // avoid the unused cached-normal normalization here.
                let triangle = TriangleShape {
                    points,
                    normal: Vec3A::ZERO,
                    normal_length: 0.0,
                };
                result_callback.process_node(&triangle, 1 << i, &mut lambda_max);
            }
        }
    }
}
