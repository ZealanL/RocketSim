use glam::Vec3A;

use super::triangle_mesh::TriangleMesh;
use crate::shared::{
    Aabb,
    bvh::{BvhNodeType, Node, Tree},
};

fn update_triangle_aabb(mut aabb: Aabb) -> Aabb {
    const MIN_AABB_DIMENSION: f32 = 0.002;
    const MIN_AABB_HALF_DIMENSION: f32 = MIN_AABB_DIMENSION / 2.0;

    let diff = (aabb.max - aabb.min).cmplt(const { Vec3A::splat(MIN_AABB_DIMENSION) });

    if diff.any() {
        let [x, y, z] = diff.into();

        if x {
            aabb.max.x += MIN_AABB_HALF_DIMENSION;
            aabb.min.x -= MIN_AABB_HALF_DIMENSION;
        }

        if y {
            aabb.max.y += MIN_AABB_HALF_DIMENSION;
            aabb.min.y -= MIN_AABB_HALF_DIMENSION;
        }

        if z {
            aabb.max.z += MIN_AABB_HALF_DIMENSION;
            aabb.min.z -= MIN_AABB_HALF_DIMENSION;
        }
    }

    aabb
}

pub fn create_bvh(triangles: &TriangleMesh, aabb: Aabb) -> Tree {
    let mut leaf_nodes: Vec<_> = triangles
        .get_tris_aabbs()
        .1
        .iter()
        .copied()
        .map(update_triangle_aabb)
        .enumerate()
        .map(|(triangle_idx, aabb)| Node {
            aabb,
            node_type: BvhNodeType::Leaf {
                leaf_idx: triangle_idx,
            },
        })
        .collect();

    let num_leaf_nodes = leaf_nodes.len();

    let mut bvh = Tree::new(aabb, num_leaf_nodes);
    bvh.build_tree(&mut leaf_nodes, 0, num_leaf_nodes);

    bvh
}
