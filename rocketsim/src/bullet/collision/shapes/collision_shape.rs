use std::sync::Arc;

use glam::{Affine3A, Vec3A};

use super::{
    bvh_triangle_mesh_shape::BvhTriangleMeshShape,
    compound_shape::CompoundShape,
    convex_hull_shape::ConvexHullShape,
    sphere_shape::{SPHERE_RADIUS_MARGIN, SphereShape},
    static_plane_shape::StaticPlaneShape,
    triangle_shape::TriangleShape,
};
use crate::{
    bullet::collision::dispatch::quad_ray_callbacks::{
        BridgeTriQuadRayCallback, QuadRayResultCallback,
    },
    shared::{Aabb, QuadRayInfo},
};

pub enum CollisionShapes {
    Compound(CompoundShape),
    Sphere(SphereShape),
    ConvexHull(ConvexHullShape),
    StaticPlane(StaticPlaneShape),
    TriangleMesh(Arc<BvhTriangleMeshShape>),
    Triangle(TriangleShape),
}

#[cfg(debug_assertions)]
fn fast_compare_trans(a: &Affine3A, b: &Affine3A) -> bool {
    a.translation == b.translation
        && a.matrix3.x_axis == b.matrix3.x_axis
        && a.matrix3.y_axis == b.matrix3.y_axis
}

impl CollisionShapes {
    pub fn get_aabb(&self, t: &Affine3A) -> Aabb {
        match self {
            Self::Sphere(shape) => shape.get_aabb(t),
            Self::Compound(shape) => shape.get_aabb(t),
            Self::ConvexHull(shape) => shape.get_aabb(t),
            Self::StaticPlane(shape) => {
                #[cfg(debug_assertions)]
                debug_assert!(fast_compare_trans(t, &shape.aabb_cache_trans));
                shape.aabb_cache
            }
            Self::TriangleMesh(shape) => {
                #[cfg(debug_assertions)]
                debug_assert!(fast_compare_trans(t, &Affine3A::IDENTITY));
                shape.aabb_ident_cache
            }
            Self::Triangle(shape) => shape.aabb(),
        }
    }

    fn get_bounding_sphere(&self) -> (Vec3A, f32) {
        let aabb = match self {
            Self::Sphere(sphere) => {
                return (Vec3A::ZERO, sphere.get_radius() + SPHERE_RADIUS_MARGIN);
            }
            Self::Compound(compound) => compound.get_ident_aabb(),
            Self::StaticPlane(shape) => &shape.aabb_ident_cache,
            Self::TriangleMesh(shape) => &shape.aabb_ident_cache,
            Self::ConvexHull(shape) => shape.get_ident_aabb(),
            Self::Triangle(shape) => {
                let aabb = shape.aabb();
                let center = (aabb.min + aabb.max) * 0.5;
                let radius = (aabb.max - aabb.min).length() * 0.5;
                return (center, radius);
            }
        };

        let center = (aabb.min + aabb.max) * 0.5;
        let radius = (aabb.max - aabb.min).length() * 0.5;

        (center, radius)
    }

    fn get_angular_motion_disc(&self) -> f32 {
        let (center, disc) = self.get_bounding_sphere();
        disc + center.length()
    }

    pub fn get_contact_breaking_threshold(&self, default_contact_threshold: f32) -> f32 {
        self.get_angular_motion_disc() * default_contact_threshold
    }

    pub const fn get_margin(&self) -> f32 {
        match self {
            Self::Sphere(shape) => shape.get_margin(),
            Self::Compound(shape) => shape.get_margin(),
            Self::ConvexHull(shape) => shape.get_margin(),
            Self::StaticPlane(_) | Self::TriangleMesh(_) | Self::Triangle(_) => 0.0,
        }
    }

    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        match self {
            Self::Sphere(shape) => shape.local_get_supporting_vertex(vec),
            Self::Compound(shape) => shape.child_shape.local_get_supporting_vertex(vec),
            Self::ConvexHull(shape) => shape.local_get_supporting_vertex(vec),
            Self::Triangle(shape) => shape.local_get_supporting_vertex(vec),
            _ => unreachable!(),
        }
    }

    pub fn local_get_support_vertex_without_margin(&self, vec: Vec3A) -> Vec3A {
        match self {
            Self::Sphere(_) => Vec3A::ZERO,
            Self::Compound(shape) => shape
                .child_shape
                .local_get_supporting_vertex_without_margin(vec),
            Self::ConvexHull(shape) => shape.local_get_supporting_vertex_without_margin(vec),
            Self::Triangle(shape) => shape.local_get_supporting_vertex_without_margin(vec),
            _ => unreachable!(),
        }
    }

    pub fn perform_quad_raycast<T: QuadRayResultCallback>(
        &self,
        result_callback: &mut BridgeTriQuadRayCallback<T>,
        ray_info: &mut QuadRayInfo,
    ) {
        match self {
            Self::Compound(compound) => {
                compound.perform_quad_raycast(result_callback, ray_info);
            }
            Self::Sphere(sphere) => {
                sphere.perform_quad_raycast(result_callback, ray_info);
            }
            Self::StaticPlane(plane) => {
                plane.perform_quad_raycast(result_callback, ray_info);
            }
            Self::TriangleMesh(mesh) => {
                mesh.perform_quad_raycast(result_callback, ray_info);
            }
            Self::ConvexHull(hull) => {
                hull.perform_quad_raycast(result_callback, ray_info);
            }
            Self::Triangle(_) => unreachable!(),
        }
    }
}
