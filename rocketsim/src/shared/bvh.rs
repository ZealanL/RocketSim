use std::iter::repeat_n;
use std::mem;

use glam::{Vec3A, Vec4};

use crate::bullet::linear_math::{aabb_util_2::intersect_ray_aabb_packet, ray_packet::RayInfo};
use crate::shared::Aabb;

pub trait ProcessNode {
    fn process_node(&mut self, leaf_idx: usize);
}

/// Just adds all the leaf indices to a Vec<usize>
#[derive(Debug, Clone)]
pub struct SimpleNodeProcessor {
    pub leaf_indices: Vec<usize>,
}
impl SimpleNodeProcessor {
    pub fn new() -> Self {
        Self {
            leaf_indices: Vec::new(),
        }
    }
}
impl ProcessNode for SimpleNodeProcessor {
    fn process_node(&mut self, leaf_idx: usize) {
        self.leaf_indices.push(leaf_idx);
    }
}

pub trait ProcessRayNode {
    fn process_node(&mut self, leaf_idx: usize, active_mask: u8, lambda_max: &mut Vec4);
}

#[derive(Debug, Default, Clone)]
pub struct Tree {
    pub aabb: Aabb,
    pub cur_node_idx: usize,
    pub nodes: Box<[Node]>,
}

impl Tree {
    const SAH_BINS: usize = 4;

    pub fn new(aabb: Aabb, num_leaf_nodes: usize) -> Self {
        Self {
            aabb,
            cur_node_idx: 0,
            nodes: repeat_n(Node::DEFAULT, 2 * num_leaf_nodes).collect(),
        }
    }

    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    fn calc_sah_split(leaf_nodes: &mut [Node], start_idx: usize, end_idx: usize) -> usize {
        let count = end_idx - start_idx;
        debug_assert!(count >= 2);

        if count == 2 {
            return start_idx + 1;
        }

        // Compute centroid bounds
        let (cmin, cmax) = leaf_nodes[start_idx..end_idx]
            .iter()
            .map(|leaf| leaf.aabb.center())
            .fold(
                (Vec3A::splat(f32::INFINITY), Vec3A::splat(f32::NEG_INFINITY)),
                |(min_acc, max_acc), c| (min_acc.min(c), max_acc.max(c)),
            );

        let extents = (cmax - cmin).to_array();

        let mut best_axis = None;
        let mut best_cost = f32::INFINITY;
        let mut best_bin = 0;

        for (axis, extent) in extents.into_iter().enumerate() {
            if extent <= f32::EPSILON {
                continue;
            }

            let mut bin_counts = [0usize; Self::SAH_BINS];
            let mut bin_bounds = [Aabb::ZERO; Self::SAH_BINS];

            // Fill bins
            let scale = (Self::SAH_BINS - 1) as f32 / extent;
            for leaf in &leaf_nodes[start_idx..end_idx] {
                let c = leaf.aabb.center();
                let idx = ((c[axis] - cmin[axis]) * scale).clamp(0.0, (Self::SAH_BINS - 1) as f32)
                    as usize;
                if bin_counts[idx] == 0 {
                    bin_bounds[idx] = leaf.aabb;
                } else {
                    bin_bounds[idx] += leaf.aabb;
                }
                bin_counts[idx] += 1;
            }

            // Prefix areas/counts
            let mut prefix_area = [0.0; Self::SAH_BINS];
            let mut prefix_count = [0usize; Self::SAH_BINS];
            let mut running_aabb = Aabb::ZERO;
            let mut running_count = 0usize;
            for i in 0..Self::SAH_BINS {
                if bin_counts[i] > 0 {
                    running_aabb += bin_bounds[i];
                    running_count += bin_counts[i];
                }

                prefix_area[i] = running_aabb.area();
                prefix_count[i] = running_count;
            }

            // Suffix areas/counts
            let mut suffix_area = [0.0; Self::SAH_BINS];
            let mut suffix_count = [0usize; Self::SAH_BINS];
            running_aabb = Aabb::ZERO;
            running_count = 0;
            for i in (0..Self::SAH_BINS).rev() {
                if bin_counts[i] > 0 {
                    running_aabb += bin_bounds[i];
                    running_count += bin_counts[i];
                }

                suffix_area[i] = running_aabb.area();
                suffix_count[i] = running_count;
            }

            // Evaluate splits between bins
            for (i, (nl, nr)) in prefix_count
                .into_iter()
                .zip(suffix_count.into_iter().skip(1))
                .enumerate()
            {
                if nl == 0 || nr == 0 {
                    continue;
                }

                let cost = prefix_area[i] * nl as f32 + suffix_area[i + 1] * nr as f32;
                if cost < best_cost {
                    best_cost = cost;
                    best_axis = Some(axis);
                    best_bin = i;
                }
            }
        }

        let axis = best_axis.unwrap();

        // Partition by chosen axis/bin
        let split_value =
            cmin[axis] + extents[axis] * ((best_bin + 1) as f32) / (Self::SAH_BINS as f32);
        let mut mid = start_idx;
        for i in start_idx..end_idx {
            let c = leaf_nodes[i].aabb.center();
            if c[axis] <= split_value {
                if i != mid {
                    Self::swap_leaf_nodes(leaf_nodes, i, mid);
                }
                mid += 1;
            }
        }

        mid
    }

    fn swap_leaf_nodes(leaf_nodes: &mut [Node], i: usize, split_idx: usize) {
        debug_assert_ne!(i, split_idx);
        let [a, b] = unsafe { leaf_nodes.get_disjoint_unchecked_mut([split_idx, i]) };
        mem::swap(a, b);
    }

    pub fn build_tree(&mut self, leaf_nodes: &mut [Node], start_idx: usize, end_idx: usize) {
        let num_indices = end_idx - start_idx;
        let cur_idx = self.cur_node_idx;

        debug_assert!(num_indices > 0);

        if num_indices == 1 {
            self.nodes[self.cur_node_idx] = leaf_nodes[start_idx];
            self.cur_node_idx += 1;
            return;
        }

        let split_idx = Self::calc_sah_split(leaf_nodes, start_idx, end_idx);

        let internal_node_idx = self.cur_node_idx;

        {
            let node = &mut self.nodes[internal_node_idx];
            node.aabb.min = self.aabb.max;
            node.aabb.max = self.aabb.min;

            for leaf in &leaf_nodes[start_idx..end_idx] {
                node.aabb += leaf.aabb;
            }
        }

        self.cur_node_idx += 1;

        self.build_tree(leaf_nodes, start_idx, split_idx);
        self.build_tree(leaf_nodes, split_idx, end_idx);

        let escape_idx = self.cur_node_idx - cur_idx;
        self.nodes[internal_node_idx].node_type = BvhNodeType::Branch { escape_idx };
    }

    fn walk_stackless_tree_find_overlap(
        &self,
        aabb: &Aabb,
        start_node_idx: usize,
        end_node_idx: usize,
    ) -> bool {
        let mut cur_idx = start_node_idx;
        while cur_idx < end_node_idx {
            let root_node = &self.nodes[cur_idx];
            let aabb_overlap = aabb.intersects(&root_node.aabb);

            match root_node.node_type {
                BvhNodeType::Leaf { leaf_idx: _ } => {
                    if aabb_overlap {
                        return true;
                    }

                    cur_idx += 1;
                }
                BvhNodeType::Branch { escape_idx } => {
                    cur_idx += if aabb_overlap { 1 } else { escape_idx };
                }
            }
        }

        false
    }

    pub fn check_overlap_with(&self, aabb: &Aabb) -> bool {
        aabb.intersects(&self.aabb)
            && self.walk_stackless_tree_find_overlap(aabb, 0, self.cur_node_idx)
    }

    fn walk_stackless_tree<T: ProcessNode>(
        &self,
        node_callback: &mut T,
        aabb: &Aabb,
        start_node_idx: usize,
        end_node_idx: usize,
    ) {
        let mut cur_idx = start_node_idx;
        while cur_idx < end_node_idx {
            let root_node = &self.nodes[cur_idx];
            let aabb_overlap = aabb.intersects(&root_node.aabb);

            match root_node.node_type {
                BvhNodeType::Leaf { leaf_idx } => {
                    if aabb_overlap {
                        node_callback.process_node(leaf_idx);
                    }

                    cur_idx += 1;
                }
                BvhNodeType::Branch { escape_idx } => {
                    cur_idx += if aabb_overlap { 1 } else { escape_idx };
                }
            }
        }
    }

    pub fn report_aabb_overlapping_node<T: ProcessNode>(&self, node_callback: &mut T, aabb: &Aabb) {
        if aabb.intersects(&self.aabb) {
            self.walk_stackless_tree(node_callback, aabb, 0, self.cur_node_idx);
        }
    }

    fn walk_stackless_tree_against_ray_packet<T: ProcessRayNode>(
        &self,
        node_callback: &mut T,
        ray_info: &mut RayInfo,
        origins: &[Vec4; 3],
        inv_dir: &[Vec4; 3],
        start_node_idx: usize,
        end_node_idx: usize,
    ) {
        let mut cur_idx = start_node_idx;
        while cur_idx < end_node_idx {
            let root_node = &self.nodes[cur_idx];
            let overlap = ray_info.aabb.intersects(&root_node.aabb);

            match root_node.node_type {
                BvhNodeType::Leaf { leaf_idx } => {
                    if overlap {
                        let mask = intersect_ray_aabb_packet(
                            origins,
                            inv_dir,
                            &root_node.aabb,
                            ray_info.lambda_max,
                        );

                        if mask != 0 {
                            node_callback.process_node(leaf_idx, mask, &mut ray_info.lambda_max);
                        }
                    }
                    cur_idx += 1;
                }
                BvhNodeType::Branch { escape_idx } => {
                    cur_idx += if overlap { 1 } else { escape_idx };
                }
            }
        }
    }

    pub fn report_ray_packet_overlapping_node<T: ProcessRayNode>(
        &self,
        node_callback: &mut T,
        ray_info: &mut RayInfo,
    ) {
        if !ray_info.aabb.intersects(&self.aabb) {
            return;
        }

        let (origins, inv_dirs) = ray_info.calc_pos_dir();
        self.walk_stackless_tree_against_ray_packet(
            node_callback,
            ray_info,
            &origins,
            &inv_dirs,
            0,
            self.cur_node_idx,
        );
    }
}

#[derive(Debug, Clone, Copy)]
pub enum BvhNodeType {
    Leaf { leaf_idx: usize },
    Branch { escape_idx: usize },
}

#[derive(Debug, Clone, Copy)]
pub struct Node {
    pub aabb: Aabb,
    pub node_type: BvhNodeType,
}

impl Node {
    pub const DEFAULT: Self = Self {
        aabb: Aabb::ZERO,
        node_type: BvhNodeType::Leaf { leaf_idx: 0 },
    };
}
