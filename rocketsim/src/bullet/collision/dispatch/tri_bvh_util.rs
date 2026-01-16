use crate::bullet::collision::shapes::{
    triangle_callback::{ProcessRayPacketTriangle, ProcessTriangle},
    triangle_mesh::TriangleMesh,
    triangle_shape::TriangleShape,
};
use crate::shared::Aabb;
use crate::shared::bvh::*;
use glam::Vec4;

pub struct NodeOverlapCallback<'a, T: ProcessTriangle> {
    tris: &'a [TriangleShape],
    aabbs: &'a [Aabb],
    callback: &'a mut T,
}

impl<'a, T: ProcessTriangle> NodeOverlapCallback<'a, T> {
    pub fn new(mesh_interface: &'a TriangleMesh, callback: &'a mut T) -> Self {
        let (tris, aabbs) = mesh_interface.get_tris_aabbs();

        Self {
            tris,
            aabbs,
            callback,
        }
    }
}

impl<T: ProcessTriangle> ProcessNode for NodeOverlapCallback<'_, T> {
    fn process_node(&mut self, node_triangle_idx: usize) {
        self.callback.process_triangle(
            &self.tris[node_triangle_idx],
            &self.aabbs[node_triangle_idx],
            node_triangle_idx,
        );
    }
}

pub struct RayPacketNodeOverlapCallback<'a, T: ProcessRayPacketTriangle> {
    tris: &'a [TriangleShape],
    callback: &'a mut T,
}

impl<'a, T: ProcessRayPacketTriangle> RayPacketNodeOverlapCallback<'a, T> {
    pub fn new(mesh_interface: &'a TriangleMesh, callback: &'a mut T) -> Self {
        let (tris, _) = mesh_interface.get_tris_aabbs();

        Self { tris, callback }
    }
}

impl<T: ProcessRayPacketTriangle> ProcessRayPacketNode for RayPacketNodeOverlapCallback<'_, T> {
    fn process_packet_node(&mut self, triangle_idx: usize, active_mask: u8, lambda_max: &mut Vec4) {
        self.callback
            .process_packet_node(&self.tris[triangle_idx], active_mask, lambda_max);
    }
}
