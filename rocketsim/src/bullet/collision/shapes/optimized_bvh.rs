use glam::Vec3A;

use super::{triangle_mesh::TriangleMesh, triangle_shape::TriangleShape};
use crate::shared::{
    Aabb,
    bvh::{BvhNodeType, Node, Tree},
};

struct BvhQuantization {
    min: Vec3A,
    max: Vec3A,
    scale: Vec3A,
}

impl BvhQuantization {
    fn new(aabb: Aabb) -> Self {
        const MARGIN: f32 = 1.0;
        const QUANTIZED_RANGE: f32 = 65533.0;

        let margin = Vec3A::splat(MARGIN);
        let mut result = Self {
            min: aabb.min - margin,
            max: aabb.max + margin,
            scale: Vec3A::ZERO,
        };
        result.update_scale(QUANTIZED_RANGE);

        let quantized_min = result.quantize_point(result.min, false);
        let unquantized_min = result.unquantize_point(quantized_min);
        result.min = result.min.min(unquantized_min - margin);
        result.update_scale(QUANTIZED_RANGE);

        let quantized_max = result.quantize_point(result.max, true);
        let unquantized_max = result.unquantize_point(quantized_max);
        result.max = result.max.max(unquantized_max + margin);
        result.update_scale(QUANTIZED_RANGE);

        result
    }

    fn update_scale(&mut self, quantized_range: f32) {
        self.scale = Vec3A::splat(quantized_range) / (self.max - self.min);
    }

    fn quantize_point(&self, point: Vec3A, is_max: bool) -> [u16; 3] {
        let value = (point - self.min) * self.scale;
        let mut result = [0; 3];
        for axis in 0..3 {
            result[axis] = if is_max {
                ((value[axis] + 1.0) as u16) | 1
            } else {
                (value[axis] as u16) & 0xfffe
            };
        }
        result
    }

    fn unquantize_point(&self, point: [u16; 3]) -> Vec3A {
        Vec3A::new(
            f32::from(point[0]) / self.scale.x,
            f32::from(point[1]) / self.scale.y,
            f32::from(point[2]) / self.scale.z,
        ) + self.min
    }

    fn quantize_aabb(&self, aabb: Aabb) -> Aabb {
        Aabb {
            min: self.unquantize_point(self.quantize_point(aabb.min, false)),
            max: self.unquantize_point(self.quantize_point(aabb.max, true)),
        }
    }
}

fn update_triangle_aabb(triangle: &TriangleShape) -> Aabb {
    const MIN_AABB_DIMENSION: f32 = 0.002;
    const MIN_AABB_HALF_DIMENSION: f32 = MIN_AABB_DIMENSION / 2.0;

    let mut aabb = triangle.aabb();
    let diff = (aabb.max - aabb.min).cmplt(Vec3A::splat(MIN_AABB_DIMENSION));
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
    let quantization = BvhQuantization::new(aabb);
    let mut leaf_nodes: Vec<_> = triangles
        .get_tris()
        .iter()
        .map(update_triangle_aabb)
        .map(|triangle_aabb| quantization.quantize_aabb(triangle_aabb))
        .enumerate()
        .map(|(triangle_idx, aabb)| Node {
            aabb,
            node_type: BvhNodeType::Leaf {
                leaf_idx: triangle_idx,
            },
        })
        .collect();

    let tree_aabb = leaf_nodes
        .iter()
        .skip(1)
        .fold(leaf_nodes[0].aabb, |bounds, leaf| {
            bounds.combine(&leaf.aabb)
        });
    Tree::build_bullet(tree_aabb, &mut leaf_nodes)
}
