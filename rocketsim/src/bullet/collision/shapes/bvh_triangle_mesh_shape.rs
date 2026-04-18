use glam::Affine3A;

use super::{
    triangle_callback::ProcessTriangle, triangle_info_map::TriangleInfoMap,
    triangle_mesh::TriangleMesh, triangle_mesh_shape::TriangleMeshShape,
};
use crate::{
    bullet::collision::{
        dispatch::{internal_edge_utility::generate_internal_edge_info, tri_bvh_util::*},
        shapes::{optimized_bvh::create_bvh, triangle_callback::ProcessRayPacketTriangle},
    },
    shared::{Aabb, RayPacketInfo, bvh::Tree},
};

pub struct BvhTriangleMeshShape {
    bvh: Tree,
    mesh_interface: TriangleMesh,
    triangle_info_map: TriangleInfoMap,
    pub aabb_ident_cache: Aabb,
}

impl BvhTriangleMeshShape {
    pub fn new(mesh_interface: TriangleMesh) -> Self {
        let triangle_mesh_shape = TriangleMeshShape::new(&mesh_interface);

        // pre-calculate the aabb
        let trans = Affine3A::IDENTITY;
        let aabb_ident_cache = triangle_mesh_shape.get_aabb(&trans);

        let bvh = create_bvh(&mesh_interface, triangle_mesh_shape.local_aabb);
        let triangle_info_map = generate_internal_edge_info(&bvh, &mesh_interface);

        Self {
            bvh,
            mesh_interface,
            triangle_info_map,
            aabb_ident_cache,
        }
    }

    pub const fn get_triangle_info_map(&self) -> &TriangleInfoMap {
        &self.triangle_info_map
    }

    pub const fn get_mesh_interface(&self) -> &TriangleMesh {
        &self.mesh_interface
    }

    pub fn process_all_triangles<T: ProcessTriangle>(&self, callback: &mut T, aabb: &Aabb) {
        let mut my_node_callback = NodeOverlapCallback::new(self.get_mesh_interface(), callback);
        self.bvh
            .report_aabb_overlapping_node(&mut my_node_callback, aabb);
    }

    pub fn check_overlap_with(&self, aabb: &Aabb) -> bool {
        self.bvh.check_overlap_with(aabb)
    }

    pub fn perform_raycast<T: ProcessRayPacketTriangle>(
        &self,
        callback: &mut T,
        ray_info: &mut RayPacketInfo,
    ) {
        let mut my_node_callback =
            RayPacketNodeOverlapCallback::new(self.get_mesh_interface(), callback);
        self.bvh
            .report_ray_packet_overlapping_node(&mut my_node_callback, ray_info);
    }
}
