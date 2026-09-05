use std::{iter::repeat_n, mem};

use glam::{Vec3A, Vec4};

use super::{Aabb, QuadRayInfo};

pub trait ProcessNode {
    fn process_node(&mut self, leaf_idx: usize);
}

pub trait ProcessQuadRayNode {
    fn process_node(&mut self, leaf_idx: usize, active_mask: u8, lambda_max: &mut Vec4);
}

#[derive(Debug, Default, Clone)]
pub struct Tree {
    pub aabb: Aabb,
    wide_nodes: Box<[WideNode]>,
    leaves: Box<[WideLeaf]>,
}

impl Tree {
    const SAH_BINS: usize = 4;
    const TRAVERSAL_STACK_SIZE: usize = 128;

    pub fn build(aabb: Aabb, leaf_nodes: &mut [Node]) -> Self {
        Self::from_binary(BinaryTree::build(aabb, leaf_nodes), leaf_nodes.len())
    }

    pub(crate) fn build_bullet(aabb: Aabb, leaf_nodes: &mut [Node]) -> Self {
        Self::from_binary(BinaryTree::build_bullet(aabb, leaf_nodes), leaf_nodes.len())
    }

    fn from_binary(binary: BinaryTree, num_leaves: usize) -> Self {
        let aabb = binary.aabb;
        let mut wide_nodes = Vec::new();
        let mut leaves = Vec::with_capacity(num_leaves);
        let max_wide_depth = binary.build_wide_node(0, &mut wide_nodes, &mut leaves);
        assert!(max_wide_depth * 3 < Self::TRAVERSAL_STACK_SIZE);

        Self {
            aabb,
            wide_nodes: wide_nodes.into_boxed_slice(),
            leaves: leaves.into_boxed_slice(),
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

    fn calc_bullet_split(leaf_nodes: &mut [Node], start_idx: usize, end_idx: usize) -> usize {
        let count = end_idx - start_idx;
        debug_assert!(count >= 2);

        let mut mean = Vec3A::ZERO;
        for leaf in &leaf_nodes[start_idx..end_idx] {
            mean += leaf.aabb.center();
        }
        mean *= 1.0 / count as f32;

        let mut variance = Vec3A::ZERO;
        for leaf in &leaf_nodes[start_idx..end_idx] {
            let difference = leaf.aabb.center() - mean;
            variance += difference * difference;
        }
        variance *= 1.0 / (count as f32 - 1.0);

        let axis = if variance.x < variance.y {
            if variance.y < variance.z { 2 } else { 1 }
        } else if variance.x < variance.z {
            2
        } else {
            0
        };
        let split_value = mean[axis];

        let mut split_idx = start_idx;
        for i in start_idx..end_idx {
            if leaf_nodes[i].aabb.center()[axis] > split_value {
                if i != split_idx {
                    Self::swap_leaf_nodes(leaf_nodes, i, split_idx);
                }
                split_idx += 1;
            }
        }

        let balanced_range = count / 3;
        if split_idx <= start_idx + balanced_range || split_idx >= end_idx - 1 - balanced_range {
            split_idx = start_idx + (count >> 1);
        }

        debug_assert!(split_idx != start_idx && split_idx != end_idx);
        split_idx
    }

    fn swap_leaf_nodes(leaf_nodes: &mut [Node], i: usize, split_idx: usize) {
        debug_assert_ne!(i, split_idx);
        let [a, b] = unsafe { leaf_nodes.get_disjoint_unchecked_mut([split_idx, i]) };
        mem::swap(a, b);
    }

    pub fn check_overlap_with(&self, aabb: &Aabb) -> bool {
        if !aabb.intersects(&self.aabb) {
            return false;
        }

        let mut stack = [0usize; Self::TRAVERSAL_STACK_SIZE];
        let mut stack_len = 1;
        while stack_len != 0 {
            stack_len -= 1;
            let node = &self.wide_nodes[stack[stack_len]];
            let mask = node.intersection_mask(aabb);
            for lane in 0..node.child_count as usize {
                if mask & (1 << lane) == 0 {
                    continue;
                }
                if let Some(child_idx) = node.children[lane].leaf_idx() {
                    let _ = child_idx;
                    return true;
                }
                stack[stack_len] = node.children[lane].branch_idx();
                stack_len += 1;
            }
        }
        false
    }

    pub fn report_aabb_overlapping_node<T: ProcessNode>(&self, node_callback: &mut T, aabb: &Aabb) {
        if !aabb.intersects(&self.aabb) {
            return;
        }

        // Only the pushed prefix is read, so leave the rest uninitialized.
        use std::mem::MaybeUninit;
        let mut stack = [MaybeUninit::<WideChild>::uninit(); Self::TRAVERSAL_STACK_SIZE];
        stack[0].write(WideChild::branch(0));
        let mut stack_len = 1;
        while stack_len != 0 {
            stack_len -= 1;
            // SAFETY: only indices below `stack_len + 1` are read, each written before bump.
            let work = unsafe { stack[stack_len].assume_init() };
            if let Some(storage_idx) = work.leaf_idx() {
                node_callback.process_node(self.leaves[storage_idx].leaf_idx);
                continue;
            }

            let node = &self.wide_nodes[work.branch_idx()];
            let mask = node.intersection_mask(aabb);
            for lane in (0..node.child_count as usize).rev() {
                if mask & (1 << lane) != 0 {
                    std::hint::cold_path();
                    debug_assert!(stack_len < Self::TRAVERSAL_STACK_SIZE);
                    stack[stack_len].write(node.children[lane]);
                    stack_len += 1;
                }
            }
        }
    }

    pub fn report_quad_ray_overlapping_node<T: ProcessQuadRayNode>(
        &self,
        node_callback: &mut T,
        ray_info: &mut QuadRayInfo,
    ) {
        if !ray_info.aabb.intersects(&self.aabb) {
            return;
        }

        let (origins, inv_dirs) = ray_info.calc_pos_dir();
        let mut stack = [WideChild::default(); Self::TRAVERSAL_STACK_SIZE];
        stack[0] = WideChild::branch(0);
        let mut stack_len = 1;
        while stack_len != 0 {
            stack_len -= 1;
            let work = stack[stack_len];
            if let Some(storage_idx) = work.leaf_idx() {
                let leaf = self.leaves[storage_idx];
                let mask = QuadRayInfo::intersect_quad_ray_aabb(
                    &origins,
                    &inv_dirs,
                    &leaf.bounds,
                    ray_info.lambda_max,
                );
                if mask != 0 {
                    node_callback.process_node(leaf.leaf_idx, mask, &mut ray_info.lambda_max);
                }
                continue;
            }

            let node = &self.wide_nodes[work.branch_idx()];
            let mask = node.intersection_mask(&ray_info.aabb);
            for lane in (0..node.child_count as usize).rev() {
                if mask & (1 << lane) != 0 {
                    std::hint::cold_path();
                    stack[stack_len] = node.children[lane];
                    stack_len += 1;
                }
            }
        }
    }
}

struct BinaryTree {
    aabb: Aabb,
    cur_node_idx: usize,
    nodes: Box<[Node]>,
}

impl BinaryTree {
    fn build(aabb: Aabb, leaf_nodes: &mut [Node]) -> Self {
        assert!(!leaf_nodes.is_empty());
        let mut tree = Self::new(aabb, leaf_nodes.len());
        tree.build_subtree(leaf_nodes, 0, leaf_nodes.len(), Tree::calc_sah_split);
        tree
    }

    fn build_bullet(aabb: Aabb, leaf_nodes: &mut [Node]) -> Self {
        assert!(!leaf_nodes.is_empty());
        let mut tree = Self::new(aabb, leaf_nodes.len());
        tree.build_subtree(leaf_nodes, 0, leaf_nodes.len(), Tree::calc_bullet_split);
        tree
    }

    fn new(aabb: Aabb, num_leaves: usize) -> Self {
        Self {
            aabb,
            cur_node_idx: 0,
            nodes: repeat_n(Node::DEFAULT, 2 * num_leaves).collect(),
        }
    }

    fn build_subtree(
        &mut self,
        leaf_nodes: &mut [Node],
        start_idx: usize,
        end_idx: usize,
        split_fn: fn(&mut [Node], usize, usize) -> usize,
    ) {
        let num_indices = end_idx - start_idx;
        let cur_idx = self.cur_node_idx;

        if num_indices == 1 {
            self.nodes[self.cur_node_idx] = leaf_nodes[start_idx];
            self.cur_node_idx += 1;
            return;
        }

        let split_idx = split_fn(leaf_nodes, start_idx, end_idx);
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
        self.build_subtree(leaf_nodes, start_idx, split_idx, split_fn);
        self.build_subtree(leaf_nodes, split_idx, end_idx, split_fn);

        self.nodes[internal_node_idx].node_type = BvhNodeType::Branch {
            escape_idx: self.cur_node_idx - cur_idx,
        };
    }

    fn children(&self, node_idx: usize) -> [usize; 2] {
        let left_idx = node_idx + 1;
        let right_idx = match self.nodes[left_idx].node_type {
            BvhNodeType::Leaf { .. } => left_idx + 1,
            BvhNodeType::Branch { escape_idx } => left_idx + escape_idx,
        };
        [left_idx, right_idx]
    }

    fn build_wide_node(
        &self,
        binary_root: usize,
        wide_nodes: &mut Vec<WideNode>,
        leaves: &mut Vec<WideLeaf>,
    ) -> usize {
        let wide_idx = wide_nodes.len();
        wide_nodes.push(WideNode::default());

        let mut frontier = vec![binary_root];
        while frontier.len() < 4 {
            let Some((slot, _)) = frontier
                .iter()
                .enumerate()
                .filter_map(|(slot, &idx)| match self.nodes[idx].node_type {
                    BvhNodeType::Leaf { .. } => None,
                    BvhNodeType::Branch { escape_idx } => Some((slot, escape_idx)),
                })
                .max_by_key(|&(_, subtree_size)| subtree_size)
            else {
                break;
            };

            let [left, right] = self.children(frontier[slot]);
            frontier.splice(slot..=slot, [left, right]);
        }

        let mut wide = WideNode::with_child_count(frontier.len() as u8);
        let mut max_child_depth = 0;
        for (lane, binary_idx) in frontier.into_iter().enumerate() {
            let node = self.nodes[binary_idx];
            wide.set_bounds(lane, node.aabb);
            wide.children[lane] = match node.node_type {
                BvhNodeType::Leaf { leaf_idx } => {
                    let storage_idx = leaves.len();
                    leaves.push(WideLeaf {
                        bounds: node.aabb,
                        leaf_idx,
                    });
                    WideChild::leaf(storage_idx)
                }
                BvhNodeType::Branch { .. } => {
                    let child_idx = wide_nodes.len();
                    max_child_depth =
                        max_child_depth.max(self.build_wide_node(binary_idx, wide_nodes, leaves));
                    WideChild::branch(child_idx)
                }
            };
        }
        wide_nodes[wide_idx] = wide;
        max_child_depth + 1
    }
}

#[derive(Debug, Clone, Copy)]
struct WideLeaf {
    bounds: Aabb,
    leaf_idx: usize,
}

#[derive(Debug, Default, Clone, Copy)]
struct WideNode {
    min_x: Vec4,
    min_y: Vec4,
    min_z: Vec4,
    max_x: Vec4,
    max_y: Vec4,
    max_z: Vec4,
    children: [WideChild; 4],
    child_count: u8,
    // Valid-child mask: (1 << child_count) - 1.
    valid_mask: u32,
}

impl WideNode {
    fn with_child_count(child_count: u8) -> Self {
        debug_assert!((1..=4).contains(&child_count));
        Self {
            child_count,
            valid_mask: (1u32 << child_count) - 1,
            ..Default::default()
        }
    }

    fn set_bounds(&mut self, lane: usize, aabb: Aabb) {
        self.min_x[lane] = aabb.min.x;
        self.min_y[lane] = aabb.min.y;
        self.min_z[lane] = aabb.min.z;
        self.max_x[lane] = aabb.max.x;
        self.max_y[lane] = aabb.max.y;
        self.max_z[lane] = aabb.max.z;
    }

    fn intersection_mask(&self, aabb: &Aabb) -> u32 {
        let overlap = self.min_x.cmple(Vec4::splat(aabb.max.x))
            & self.max_x.cmpge(Vec4::splat(aabb.min.x))
            & self.min_y.cmple(Vec4::splat(aabb.max.y))
            & self.max_y.cmpge(Vec4::splat(aabb.min.y))
            & self.min_z.cmple(Vec4::splat(aabb.max.z))
            & self.max_z.cmpge(Vec4::splat(aabb.min.z));
        overlap.bitmask() & self.valid_mask
    }
}

#[derive(Debug, Default, Clone, Copy)]
struct WideChild(usize);

impl WideChild {
    const LEAF_BIT: usize = 1 << (usize::BITS - 1);

    const fn leaf(leaf_idx: usize) -> Self {
        assert!(leaf_idx < Self::LEAF_BIT);
        Self(Self::LEAF_BIT | leaf_idx)
    }

    const fn branch(branch_idx: usize) -> Self {
        Self(branch_idx)
    }

    const fn leaf_idx(self) -> Option<usize> {
        if self.0 & Self::LEAF_BIT != 0 {
            Some(self.0 & !Self::LEAF_BIT)
        } else {
            None
        }
    }

    const fn branch_idx(self) -> usize {
        self.0
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

#[cfg(test)]
mod tests {
    use std::mem::size_of;

    use glam::Vec3A;

    use super::{Aabb, BvhNodeType, Node, ProcessNode, Tree};

    #[derive(Default)]
    struct Collector(Vec<usize>);

    impl ProcessNode for Collector {
        fn process_node(&mut self, leaf_idx: usize) {
            self.0.push(leaf_idx);
        }
    }

    #[test]
    fn tree_stores_only_runtime_bvh_data() {
        assert_eq!(size_of::<Tree>(), 64);
    }

    #[test]
    fn wide_traversal_handles_single_leaf() {
        let bounds = Aabb::new(Vec3A::ZERO, Vec3A::ONE);
        let mut leaves = [Node {
            aabb: bounds,
            node_type: BvhNodeType::Leaf { leaf_idx: 7 },
        }];
        let tree = Tree::build(bounds, &mut leaves);

        let mut actual = Collector::default();
        tree.report_aabb_overlapping_node(&mut actual, &bounds);
        assert_eq!(actual.0, [7]);
        assert!(tree.check_overlap_with(&bounds));
    }

    #[test]
    fn wide_traversal_matches_brute_force_in_binary_order() {
        let mut leaves: Vec<_> = (0..257)
            .map(|idx| {
                let x = ((idx * 37) % 101) as f32 - 50.0;
                let y = ((idx * 61) % 97) as f32 - 48.0;
                let z = ((idx * 17) % 43) as f32 - 21.0;
                let min = Vec3A::new(x, y, z);
                Node {
                    aabb: Aabb::new(min, min + Vec3A::splat(1.5)),
                    node_type: BvhNodeType::Leaf { leaf_idx: idx },
                }
            })
            .collect();
        let tree_aabb = leaves
            .iter()
            .skip(1)
            .fold(leaves[0].aabb, |bounds, leaf| bounds.combine(&leaf.aabb));
        let tree = Tree::build(tree_aabb, &mut leaves);

        for query_idx in 0..100 {
            let center = Vec3A::new(
                ((query_idx * 29) % 113) as f32 - 56.0,
                ((query_idx * 47) % 109) as f32 - 54.0,
                ((query_idx * 13) % 53) as f32 - 26.0,
            );
            let query = Aabb::new(center - 8.0, center + 8.0);
            let expected: Vec<_> = leaves
                .iter()
                .filter_map(|node| match node.node_type {
                    BvhNodeType::Leaf { leaf_idx } if query.intersects(&node.aabb) => {
                        Some(leaf_idx)
                    }
                    _ => None,
                })
                .collect();
            let mut actual = Collector::default();
            tree.report_aabb_overlapping_node(&mut actual, &query);
            assert_eq!(actual.0, expected);
            assert_eq!(tree.check_overlap_with(&query), !expected.is_empty());
        }
    }
}
