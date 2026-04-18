use std::sync::Arc;

use glam::{Affine3A, Vec3A};

use super::{
    bvh_triangle_mesh_shape::BvhTriangleMeshShape, compound_shape::CompoundShape,
    sphere_shape::SphereShape, static_plane_shape::StaticPlaneShape,
};
use crate::{
    bullet::collision::{
        dispatch::ray_packet_callbacks::{BridgeTriangleRaycastPacketCallback, RayResultCallback},
        shapes::{convex_hull_shape::ConvexHullShape, sphere_shape::SPHERE_RADIUS_MARGIN},
    },
    shared::{Aabb, RayPacketInfo},
};

pub enum CollisionShapes {
    Compound(CompoundShape),
    Sphere(SphereShape),
    ConvexHull(ConvexHullShape),
    StaticPlane(StaticPlaneShape),
    TriangleMesh(Arc<BvhTriangleMeshShape>),
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
        }
    }

    fn get_bounding_sphere(&self) -> (Vec3A, f32) {
        match self {
            Self::Sphere(sphere) => (Vec3A::ZERO, sphere.get_radius() + SPHERE_RADIUS_MARGIN),
            Self::Compound(compound) => {
                let aabb = compound.get_ident_aabb();
                let center = (aabb.min + aabb.max) * 0.5;
                let radius = (aabb.max - aabb.min).length() * 0.5;

                (center, radius)
            }
            Self::StaticPlane(shape) => {
                let aabb = &shape.aabb_ident_cache;
                let center = (aabb.min + aabb.max) * 0.5;
                let radius = (aabb.max - aabb.min).length() * 0.5;

                (center, radius)
            }
            Self::TriangleMesh(shape) => {
                let aabb = &shape.aabb_ident_cache;
                let center = (aabb.min + aabb.max) * 0.5;
                let radius = (aabb.max - aabb.min).length() * 0.5;

                (center, radius)
            }
            Self::ConvexHull(shape) => {
                let aabb = shape.get_ident_aabb();
                let center = (aabb.min + aabb.max) * 0.5;
                let radius = (aabb.max - aabb.min).length() * 0.5;

                (center, radius)
            }
        }
    }

    fn get_angular_motion_disc(&self) -> f32 {
        let (center, disc) = self.get_bounding_sphere();
        disc + center.length()
    }

    pub fn get_contact_breaking_threshold(&self, default_contact_threshold: f32) -> f32 {
        self.get_angular_motion_disc() * default_contact_threshold
    }

    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        match self {
            Self::Sphere(shape) => shape.local_get_supporting_vertex(vec),
            Self::Compound(shape) => shape.child_shape.local_get_supporting_vertex(vec),
            Self::ConvexHull(shape) => shape.local_get_supporting_vertex(vec),
            _ => todo!(),
        }
    }

    pub fn perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastPacketCallback<T>,
        ray_info: &mut RayPacketInfo,
    ) {
        match self {
            Self::Compound(compound) => {
                compound.perform_raycast(result_callback, ray_info);
            }
            Self::Sphere(sphere) => {
                sphere.perform_raycast(result_callback, ray_info);
            }
            Self::StaticPlane(plane) => {
                plane.perform_raycast(result_callback, ray_info);
            }
            Self::TriangleMesh(mesh) => {
                mesh.perform_raycast(result_callback, ray_info);
            }
            Self::ConvexHull(hull) => {
                hull.perform_raycast(result_callback, ray_info);
            }
        }
    }
}
