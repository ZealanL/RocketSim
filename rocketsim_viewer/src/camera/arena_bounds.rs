//! Camera containment: raycasts against the arena collision hull.
//!
//! BVH below (`Tree` and friends) is copied from
//! `rocketsim/src/shared/bvh.rs` (same repo, MIT) so the viewer traces the
//! exact hull the sim collides with. The Bullet-specific split and the
//! unused AABB queries (`check_overlap_with`,
//! `report_aabb_overlapping_node`) are dropped; the SAH build and the
//! quad-ray traversal are verbatim. `Aabb`/`QuadRayInfo` are reused from
//! `rocketsim::shared` (no copy).
//!
//! [`ArenaBounds`] wraps one game mode's collision tris (world vertices in
//! uu, i.e. Bullet units times [`BT_TO_UU`](rocketsim::consts::BT_TO_UU))
//! plus procedural quads for the sim's ANALYTIC planes: the mesh alone has
//! holes (probed: no surface at all on the Soccar ±X walls between z≈400
//! and z≈1400 — physics covers those spans with infinite
//! `StaticPlaneShape`s, see `Arena::setup_arena_collision_shapes`). So the
//! trace set mirrors physics exactly: floor cap (all modes; the hull is
//! open at the bottom), ceiling cap (all modes), X-wall quads (all but
//! Dropshot), Y-wall quads (Hoops only). Dropshot's floor stays whole over
//! tile holes (broken tiles never open camera holes). Visual backdrop
//! planes (`big_plane` aprons) are deliberately NOT included — they are
//! scenery, not walls. `TheVoid` has no hull and builds empty (every query
//! misses; cameras stay unconstrained).
//!
//! Per-frame cost is 1–2 short segment casts (single-digit µs); the BVH
//! builds once per mode and is shared via [`ArenaBoundsCache`].

use std::{iter::repeat_n, mem, sync::Arc};

use glam::{Vec3A, Vec4};
use rocketsim::{
    GameMode,
    consts::{BT_TO_UU, arena::get_aabb},
    shared::{Aabb, QuadRayInfo},
};
use rocketsim_vis::proto::MeshData;
use rustc_hash::FxHashMap;

// ---------------------------------------------------------------------------
// Copied BVH (rocketsim/src/shared/bvh.rs, bullet split removed).
// ---------------------------------------------------------------------------

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

    fn swap_leaf_nodes(leaf_nodes: &mut [Node], i: usize, split_idx: usize) {
        debug_assert_ne!(i, split_idx);
        let [a, b] = unsafe { leaf_nodes.get_disjoint_unchecked_mut([split_idx, i]) };
        mem::swap(a, b);
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
struct WideChild(u32);

impl WideChild {
    const LEAF_BIT: u32 = 1 << (u32::BITS - 1);

    const fn leaf(leaf_idx: usize) -> Self {
        assert!(leaf_idx < Self::LEAF_BIT as usize);
        Self(Self::LEAF_BIT | leaf_idx as u32)
    }

    const fn branch(branch_idx: usize) -> Self {
        assert!(branch_idx < Self::LEAF_BIT as usize);
        Self(branch_idx as u32)
    }

    const fn leaf_idx(self) -> Option<usize> {
        if self.0 & Self::LEAF_BIT != 0 {
            Some((self.0 & !Self::LEAF_BIT) as usize)
        } else {
            None
        }
    }

    const fn branch_idx(self) -> usize {
        self.0 as usize
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

// ---------------------------------------------------------------------------
// Arena containment queries.
// ---------------------------------------------------------------------------

/// Boom backoff for follow views (uu): the camera stops this far inside
/// the wall. RL-flavored: the follow boom is long, so it needs margin.
pub const FOLLOW_BACKOFF: f32 = 60.0;
/// Wall margin for freecam slides (uu): small, so tight interior shots
/// still frame.
pub const FREECAM_BACKOFF: f32 = 15.0;

/// Closest wall hit along a segment: fraction `t` of `from → to`, the hit
/// point, and the geometric normal flipped to face the incoming ray.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegHit {
    pub t: f32,
    pub point: Vec3A,
    pub normal: Vec3A,
}

/// Double-sided segment-vs-triangle test (`t` in `[0, 1]`). Returns the
/// fraction plus the geometric normal facing the ray.
fn ray_tri(from: Vec3A, dir: Vec3A, tri: &[Vec3A; 3]) -> Option<(f32, Vec3A)> {
    let e1 = tri[1] - tri[0];
    let e2 = tri[2] - tri[0];
    let p = dir.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-8 {
        return None;
    }
    let inv = 1.0 / det;
    let s = from - tri[0];
    let u = s.dot(p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = dir.dot(q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(q) * inv;
    if !(0.0..=1.0).contains(&t) {
        return None;
    }
    let mut n = e1.cross(e2).normalize_or_zero();
    if n.dot(dir) > 0.0 {
        n = -n;
    }
    Some((t, n))
}

/// Leaf AABB with a minimum thickness per axis (mirrors Bullet's
/// `update_triangle_aabb`: zero-thickness slabs from flat floor tris still
/// traverse).
fn tri_aabb(tri: &[Vec3A; 3]) -> Aabb {
    const MIN_DIM: f32 = 0.002;
    const HALF: f32 = MIN_DIM / 2.0;
    let mut aabb = Aabb::new(
        tri[0].min(tri[1]).min(tri[2]),
        tri[0].max(tri[1]).max(tri[2]),
    );
    let diff = (aabb.max - aabb.min).cmplt(Vec3A::splat(MIN_DIM));
    let [x, y, z] = diff.into();
    if x {
        aabb.max.x += HALF;
        aabb.min.x -= HALF;
    }
    if y {
        aabb.max.y += HALF;
        aabb.min.y -= HALF;
    }
    if z {
        aabb.max.z += HALF;
        aabb.min.z -= HALF;
    }
    aabb
}

struct Closest<'a> {
    tris: &'a [[Vec3A; 3]],
    from: Vec3A,
    dir: Vec3A,
    best_t: f32,
    best_n: Vec3A,
}

impl ProcessQuadRayNode for Closest<'_> {
    fn process_node(&mut self, leaf_idx: usize, active_mask: u8, lambda_max: &mut Vec4) {
        // All 4 packet lanes carry the same segment, so lane 0 decides.
        if active_mask & 1 == 0 {
            return;
        }
        if let Some((t, n)) = ray_tri(self.from, self.dir, &self.tris[leaf_idx])
            && t < self.best_t
        {
            self.best_t = t;
            self.best_n = n;
            *lambda_max = Vec4::splat(t);
        }
    }
}

struct Counter<'a> {
    tris: &'a [[Vec3A; 3]],
    from: Vec3A,
    dir: Vec3A,
    hits: Vec<f32>,
}

impl ProcessQuadRayNode for Counter<'_> {
    fn process_node(&mut self, leaf_idx: usize, active_mask: u8, _lambda_max: &mut Vec4) {
        // Parity needs EVERY crossing: never shrink `lambda_max`.
        if active_mask & 1 == 0 {
            return;
        }
        if let Some((t, _)) = ray_tri(self.from, self.dir, &self.tris[leaf_idx])
            && t > 1e-6
            && t < 1.0 - 1e-6
        {
            self.hits.push(t);
        }
    }
}

/// Clusters sorted crossing fractions: coincident/double-covered surfaces
/// (hull curve over an analytic quad, lid over the ceiling cap) count once.
/// `eps_t` is the fraction equivalent of ~5uu of ray travel.
fn clustered_parity(hits: &mut [f32], eps_t: f32) -> bool {
    hits.sort_by(|a, b| a.total_cmp(b));
    let mut count = 0;
    let mut last = f32::NEG_INFINITY;
    for &t in hits.iter() {
        if t - last > eps_t {
            count += 1;
            last = t;
        }
    }
    count % 2 == 1
}

/// Traceable arena hull for one game mode: collision tris in uu plus
/// procedural quads for the sim's analytic planes (floor, ceiling, side
/// walls — see module docs).
pub struct ArenaBounds {
    mode: GameMode,
    tris: Vec<[Vec3A; 3]>,
    tree: Option<Tree>,
}

/// Pushes a quad (two double-sided tris) spanning the given corners.
fn push_quad(tris: &mut Vec<[Vec3A; 3]>, a: Vec3A, b: Vec3A, c: Vec3A, d: Vec3A) {
    tris.push([a, b, c]);
    tris.push([a, c, d]);
}

impl ArenaBounds {
    /// Builds from the loaded `.cmf` files (needs `rocketsim::init` first —
    /// the viewer calls `photo::init_meshes` at startup). `Heatseeker` and
    /// `Snowday` share Soccar's meshes (same mapping as the sim).
    ///
    /// # Panics
    ///
    /// Panics if RocketSim was not initialized.
    pub fn build(mode: GameMode) -> Self {
        let mut tris = Vec::new();
        for mesh in rocketsim::get_arena_collision_mesh_files(mode) {
            let verts = mesh.get_vertices();
            let (triangles, _) = mesh.get_indices().as_chunks::<3>();
            for &[a, b, c] in triangles {
                tris.push([
                    verts[a] * BT_TO_UU,
                    verts[b] * BT_TO_UU,
                    verts[c] * BT_TO_UU,
                ]);
            }
        }
        Self::add_analytic_planes(mode, &mut tris);
        Self::from_tris(mode, tris)
    }

    /// Builds a hull from streamed world-space geometry. This path does not
    /// require RocketSim mesh initialization.
    pub fn from_meshes(mode: GameMode, meshes: &[MeshData]) -> Self {
        let mut tris = Vec::new();
        for mesh in meshes {
            let (triangles, _) = mesh.indices.as_chunks::<3>();
            for tri in triangles {
                if tri.iter().any(|&i| i as usize >= mesh.vertices.len()) {
                    continue;
                }
                let point = |i: u32| {
                    let v = mesh.vertices[i as usize];
                    Vec3A::new(v[0], v[1], v[2])
                };
                tris.push([point(tri[0]), point(tri[1]), point(tri[2])]);
            }
        }
        Self::add_analytic_planes(mode, &mut tris);
        Self::from_tris(mode, tris)
    }

    fn add_analytic_planes(mode: GameMode, tris: &mut Vec<[Vec3A; 3]>) {
        // Analytic planes as finite quads (mirrors
        // `Arena::setup_arena_collision_shapes`; the mesh alone leaves the
        // flat spans open). Floor doubles as the parity closer; Dropshot's
        // stays whole over tile holes.
        if !tris.is_empty() {
            let aabb = get_aabb(mode);
            let (x0, x1, y0, y1, z0, z1) = (
                aabb.min.x, aabb.max.x, aabb.min.y, aabb.max.y, aabb.min.z, aabb.max.z,
            );
            push_quad(
                tris,
                Vec3A::new(x0, y0, z0),
                Vec3A::new(x1, y0, z0),
                Vec3A::new(x1, y1, z0),
                Vec3A::new(x0, y1, z0),
            );
            push_quad(
                tris,
                Vec3A::new(x0, y0, z1),
                Vec3A::new(x1, y0, z1),
                Vec3A::new(x1, y1, z1),
                Vec3A::new(x0, y1, z1),
            );
            if mode != GameMode::Dropshot {
                for &x in &[x0, x1] {
                    push_quad(
                        tris,
                        Vec3A::new(x, y0, z0),
                        Vec3A::new(x, y1, z0),
                        Vec3A::new(x, y1, z1),
                        Vec3A::new(x, y0, z1),
                    );
                }
            }
            if mode == GameMode::Hoops {
                for &y in &[y0, y1] {
                    push_quad(
                        tris,
                        Vec3A::new(x0, y, z0),
                        Vec3A::new(x1, y, z0),
                        Vec3A::new(x1, y, z1),
                        Vec3A::new(x0, y, z1),
                    );
                }
            }
        }
    }

    /// Builds from explicit tris (unit tests; no init needed).
    pub fn from_tris(mode: GameMode, tris: Vec<[Vec3A; 3]>) -> Self {
        let tree = if tris.is_empty() {
            None
        } else {
            let mut leaves: Vec<Node> = tris
                .iter()
                .enumerate()
                .map(|(i, tri)| Node {
                    aabb: tri_aabb(tri),
                    node_type: BvhNodeType::Leaf { leaf_idx: i },
                })
                .collect();
            let root = leaves
                .iter()
                .skip(1)
                .fold(leaves[0].aabb, |bounds, leaf| bounds.combine(&leaf.aabb));
            Some(Tree::build(root, &mut leaves))
        };
        Self { mode, tris, tree }
    }

    /// Game mode this hull was built for.
    pub fn mode(&self) -> GameMode {
        self.mode
    }

    /// Traceable triangle count (hull + floor cap).
    pub fn tri_count(&self) -> usize {
        self.tris.len()
    }

    /// True when there is no hull (`TheVoid`): every query misses.
    pub fn is_empty(&self) -> bool {
        self.tris.is_empty()
    }

    /// Closest wall hit along `from → to`, or `None` (empty hull included).
    pub fn segment_hit(&self, from: Vec3A, to: Vec3A) -> Option<SegHit> {
        let tree = self.tree.as_ref()?;
        let dir = to - from;
        if dir.length_squared() < 1e-12 {
            return None;
        }
        let froms = [from; 4];
        let tos = [to; 4];
        let mut ray_info = QuadRayInfo::new(&froms, &tos);
        let mut cb = Closest {
            tris: &self.tris,
            from,
            dir,
            best_t: f32::INFINITY,
            best_n: Vec3A::Z,
        };
        tree.report_quad_ray_overlapping_node(&mut cb, &mut ray_info);
        if cb.best_t.is_finite() {
            Some(SegHit {
                t: cb.best_t,
                point: from + dir * cb.best_t,
                normal: cb.best_n,
            })
        } else {
            None
        }
    }

    /// Pulls a follow boom inside: walks `focus → desired` and stops
    /// `backoff` uu before the first wall. No hit (or empty hull) returns
    /// `desired` unchanged. `focus` (car + height) is always inside, so no
    /// inside/outside test is needed.
    pub fn clamp_ray(&self, focus: Vec3A, desired: Vec3A, backoff: f32) -> Vec3A {
        let seg = desired - focus;
        let len = seg.length();
        if len < 1e-6 {
            return desired;
        }
        match self.segment_hit(focus, desired) {
            None => desired,
            Some(hit) => {
                let t = (hit.t - backoff / len).clamp(0.0, 1.0);
                focus + seg * t
            }
        }
    }

    /// Pulls a follow-camera pose inside the hull (boom shortening): clamps
    /// `pos` along `focus → pos` with [`FOLLOW_BACKOFF`], re-aiming
    /// ball-cam at the ball from the pulled-in pose (car-cam keeps `dir`).
    /// Returns the pose unchanged when already inside. Both follow call
    /// sites (manual + director) share this so the behavior can't drift.
    pub fn contain_follow(
        &self,
        focus: Vec3A,
        ball_pos: Vec3A,
        face_ball: bool,
        pos: Vec3A,
        dir: Vec3A,
    ) -> (Vec3A, Vec3A) {
        let clamped = self.clamp_ray(focus, pos, FOLLOW_BACKOFF);
        if clamped == pos {
            return (pos, dir);
        }
        let dir = if face_ball {
            (ball_pos - clamped).normalize_or_zero()
        } else {
            dir
        };
        (clamped, dir)
    }

    /// Slides a freecam step along walls: advances to `backoff` before the
    /// first wall, projects the remainder onto the wall plane, and takes
    /// what fits. Callers only invoke this when `prev` is inside (see
    /// [`contains`](Self::contains)): moves starting outside pass through
    /// untouched so a camera staged outside can still fly in.
    pub fn slide_move(&self, prev: Vec3A, desired: Vec3A, backoff: f32) -> Vec3A {
        if self.tree.is_none() {
            return desired;
        }
        let seg = desired - prev;
        let len = seg.length();
        if len < 1e-6 {
            return prev;
        }
        let Some(hit) = self.segment_hit(prev, desired) else {
            return desired;
        };
        let t = (hit.t - backoff / len).max(0.0);
        let p1 = prev + seg * t;
        let rem = desired - hit.point;
        let slide = rem - hit.normal * rem.dot(hit.normal);
        if slide.length_squared() < 1e-12 {
            return p1;
        }
        let target2 = p1 + slide;
        match self.segment_hit(p1, target2) {
            None => target2,
            Some(h2) => {
                let t2 = (h2.t - backoff / slide.length()).clamp(0.0, 1.0);
                p1 + slide * t2
            }
        }
    }

    /// Inside test via +X parity (odd crossings = inside). Points outside
    /// the whole-hull AABB short-circuit to `false`; empty hulls report
    /// `true` (unconstrained modes allow everything).
    pub fn contains(&self, p: Vec3A) -> bool {
        let Some(tree) = self.tree.as_ref() else {
            return true;
        };
        if !(p.cmple(tree.aabb.max).all() && p.cmpge(tree.aabb.min).all()) {
            return false;
        }
        let to = Vec3A::new(tree.aabb.max.x + 10_000.0, p.y, p.z);
        let dir = to - p;
        let len = dir.length();
        if len < 1e-6 {
            return true;
        }
        let froms = [p; 4];
        let tos = [to; 4];
        let mut ray_info = QuadRayInfo::new(&froms, &tos);
        let mut cb = Counter {
            tris: &self.tris,
            from: p,
            dir,
            hits: Vec::new(),
        };
        tree.report_quad_ray_overlapping_node(&mut cb, &mut ray_info);
        clustered_parity(&mut cb.hits, 5.0 / len)
    }
}

/// Lazily built hulls, one per game mode (Soccar-first; other modes build
/// on first sight). `TheVoid` maps to `None` (no hull). Shared via `Arc`
/// so cameras only hold a cheap handle per frame.
#[derive(Default)]
pub struct ArenaBoundsCache {
    map: FxHashMap<GameMode, Arc<ArenaBounds>>,
    mesh_keys: FxHashMap<GameMode, u64>,
}

impl ArenaBoundsCache {
    /// Empty cache; hulls build on first lookup.
    pub fn new() -> Self {
        Self::default()
    }

    /// Hull for `mode` (`None` = `TheVoid`, unconstrained).
    ///
    /// # Panics
    ///
    /// Panics if RocketSim was not initialized (same as
    /// [`ArenaBounds::build`]).
    pub fn reset(&mut self) {
        self.map.clear();
        self.mesh_keys.clear();
    }

    pub fn get_for_meshes(
        &mut self,
        mode: GameMode,
        meshes: &[MeshData],
    ) -> Option<Arc<ArenaBounds>> {
        if mode == GameMode::TheVoid {
            return None;
        }
        let mut key = 0xcbf29ce484222325u64;
        for mesh in meshes {
            key = (key ^ mesh.vertices.len() as u64).wrapping_mul(0x100000001b3);
            key = (key ^ mesh.indices.len() as u64).wrapping_mul(0x100000001b3);
            for v in &mesh.vertices {
                for x in v {
                    key = (key ^ u64::from(x.to_bits())).wrapping_mul(0x100000001b3);
                }
            }
            for i in &mesh.indices {
                key = (key ^ u64::from(*i)).wrapping_mul(0x100000001b3);
            }
        }
        if self.mesh_keys.get(&mode) == Some(&key)
            && let Some(bounds) = self.map.get(&mode)
        {
            return Some(bounds.clone());
        }
        let bounds = Arc::new(ArenaBounds::from_meshes(mode, meshes));
        self.mesh_keys.insert(mode, key);
        self.map.insert(mode, bounds.clone());
        Some(bounds)
    }

    pub fn get(&mut self, mode: GameMode) -> Option<Arc<ArenaBounds>> {
        if mode == GameMode::TheVoid {
            return None;
        }
        if let Some(b) = self.map.get(&mode) {
            return Some(b.clone());
        }
        let b = Arc::new(ArenaBounds::build(mode));
        self.mesh_keys.remove(&mode);
        self.map.insert(mode, b.clone());
        Some(b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn soccar() -> ArenaBounds {
        crate::photo::init_meshes();
        ArenaBounds::build(GameMode::Soccar)
    }

    #[test]
    fn streamed_mesh_bounds_and_cache_do_not_require_mesh_init() {
        let mesh = MeshData {
            vertices: vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [0.0, 100.0, 0.0]],
            indices: vec![0, 1, 2],
        };
        let bounds = ArenaBounds::from_meshes(GameMode::Soccar, std::slice::from_ref(&mesh));
        assert_eq!(bounds.tri_count(), 9);
        assert!(
            bounds
                .segment_hit(Vec3A::new(25.0, 25.0, 10.0), Vec3A::new(25.0, 25.0, -10.0))
                .is_some()
        );

        let mut cache = ArenaBoundsCache::new();
        let first = cache
            .get_for_meshes(GameMode::Soccar, std::slice::from_ref(&mesh))
            .unwrap();
        let same = cache
            .get_for_meshes(GameMode::Soccar, std::slice::from_ref(&mesh))
            .unwrap();
        assert!(Arc::ptr_eq(&first, &same));

        let changed = MeshData {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [100.0, 0.0, 0.0],
                [0.0, 100.0, 0.0],
                [100.0, 100.0, 0.0],
            ],
            indices: vec![0, 1, 2, 1, 3, 2],
        };
        let rebuilt = cache.get_for_meshes(GameMode::Soccar, &[changed]).unwrap();
        assert!(!Arc::ptr_eq(&first, &rebuilt));
        assert_eq!(rebuilt.tri_count(), 10);
        cache.reset();
        let reset = cache
            .get_for_meshes(GameMode::Soccar, std::slice::from_ref(&mesh))
            .unwrap();
        assert!(!Arc::ptr_eq(&first, &reset));
    }

    #[test]
    fn soccar_hull_size() {
        // 8020 collision tris + 8 analytic-quad tris (floor, ceiling,
        // 2 X-walls; re-pin on hull change).
        let b = soccar();
        assert_eq!(b.tri_count(), 8028, "soccar hull changed size?");
        assert!(!b.is_empty());
    }

    #[test]
    fn side_wall_blocks_exit() {
        let b = soccar();
        // x=3000 stays clear of the goal mouth (|x| < 893).
        let focus = Vec3A::new(3000.0, 0.0, 117.0);
        let out = Vec3A::new(8000.0, 0.0, 500.0);
        let hit = b.segment_hit(focus, out).expect("must hit +X wall");
        // Analytic X-wall plane at exactly 4096 (the mesh alone is open
        // here — see module docs).
        assert!(
            (4090.0..4100.0).contains(&hit.point.x),
            "wall hit misplaced: {hit:?}"
        );
        let clamped = b.clamp_ray(focus, out, FOLLOW_BACKOFF);
        assert!(clamped.x < hit.point.x && clamped.x > 3900.0);
        assert!((hit.point.x - clamped.x - FOLLOW_BACKOFF).abs() < 2.0);
    }

    #[test]
    fn goal_recess_passes_through_mouth() {
        let b = soccar();
        // Down the middle the ray enters the goal box (no wall at 5120).
        let focus = Vec3A::new(0.0, 4000.0, 300.0);
        let out = Vec3A::new(0.0, 8000.0, 300.0);
        let hit = b.segment_hit(focus, out).expect("must hit goal back");
        assert!(
            hit.point.y > 5800.0,
            "should reach the goal back, not the line: {hit:?}"
        );
    }

    #[test]
    fn floor_cap_blocks_and_contains_needs_it() {
        let b = soccar();
        let hit = b
            .segment_hit(Vec3A::new(0.0, 0.0, 500.0), Vec3A::new(0.0, 0.0, -500.0))
            .expect("must hit the floor cap");
        assert!(hit.point.z.abs() < 2.0, "floor hit misplaced: {hit:?}");
        assert!(b.contains(Vec3A::new(0.0, 0.0, 500.0)));
        assert!(!b.contains(Vec3A::new(0.0, -7000.0, 5000.0)));
    }

    /// Closed-box tris for parity/slide tests (a lone wall is not a
    /// volume — `contains` is only meaningful on closed hulls).
    fn box_tris(lo: Vec3A, hi: Vec3A) -> Vec<[Vec3A; 3]> {
        let mut tris = Vec::new();
        let (x0, y0, z0, x1, y1, z1) = (lo.x, lo.y, lo.z, hi.x, hi.y, hi.z);
        // Floor/ceiling (normal ±Z), X-walls, Y-walls.
        for (a, b, c, d) in [
            (
                Vec3A::new(x0, y0, z0),
                Vec3A::new(x1, y0, z0),
                Vec3A::new(x1, y1, z0),
                Vec3A::new(x0, y1, z0),
            ),
            (
                Vec3A::new(x0, y0, z1),
                Vec3A::new(x1, y0, z1),
                Vec3A::new(x1, y1, z1),
                Vec3A::new(x0, y1, z1),
            ),
            (
                Vec3A::new(x0, y0, z0),
                Vec3A::new(x0, y1, z0),
                Vec3A::new(x0, y1, z1),
                Vec3A::new(x0, y0, z1),
            ),
            (
                Vec3A::new(x1, y0, z0),
                Vec3A::new(x1, y1, z0),
                Vec3A::new(x1, y1, z1),
                Vec3A::new(x1, y0, z1),
            ),
            (
                Vec3A::new(x0, y0, z0),
                Vec3A::new(x1, y0, z0),
                Vec3A::new(x1, y0, z1),
                Vec3A::new(x0, y0, z1),
            ),
            (
                Vec3A::new(x0, y1, z0),
                Vec3A::new(x1, y1, z0),
                Vec3A::new(x1, y1, z1),
                Vec3A::new(x0, y1, z1),
            ),
        ] {
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
        tris
    }

    #[test]
    fn slide_runs_along_wall() {
        // Closed [-100, 100]² box, 200 tall: inside/outside parity plus a
        // diagonal push into the +X face (X stops, Y slides through).
        let b = ArenaBounds::from_tris(
            GameMode::Soccar,
            box_tris(
                Vec3A::new(-100.0, -100.0, 0.0),
                Vec3A::new(100.0, 100.0, 200.0),
            ),
        );
        assert!(b.contains(Vec3A::new(-50.0, 0.0, 100.0)));
        assert!(!b.contains(Vec3A::new(150.0, 0.0, 100.0)));
        let slid = b.slide_move(
            Vec3A::new(-50.0, 0.0, 100.0),
            Vec3A::new(150.0, 80.0, 100.0),
            FREECAM_BACKOFF,
        );
        assert!(
            (slid.x - (100.0 - FREECAM_BACKOFF)).abs() < 2.0,
            "slid={slid:?}"
        );
        // Post-hit remainder (y 60→80) survives projected onto the wall.
        assert!((slid.y - 74.4).abs() < 1.0, "slid={slid:?}");
        assert!((slid.z - 100.0).abs() < 1e-3, "slid={slid:?}");
    }

    #[test]
    fn void_is_unconstrained() {
        crate::photo::init_meshes();
        let b = ArenaBounds::build(GameMode::TheVoid);
        assert!(b.is_empty());
        let want = Vec3A::new(1.0, 2.0, 3.0);
        assert_eq!(b.clamp_ray(Vec3A::ZERO, want, FOLLOW_BACKOFF), want);
        assert!(b.segment_hit(Vec3A::ZERO, want).is_none());
        assert!(b.contains(want));
    }

    #[test]
    fn degenerate_segments_miss() {
        let b = soccar();
        assert!(b.segment_hit(Vec3A::ZERO, Vec3A::ZERO).is_none());
        assert_eq!(
            b.clamp_ray(Vec3A::ZERO, Vec3A::ZERO, FOLLOW_BACKOFF),
            Vec3A::ZERO
        );
        assert_eq!(
            b.slide_move(Vec3A::ZERO, Vec3A::ZERO, FREECAM_BACKOFF),
            Vec3A::ZERO
        );
    }
}
