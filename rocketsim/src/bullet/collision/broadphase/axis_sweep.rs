use glam::Vec3A;

use super::{BroadphasePair, BroadphaseProxy, overlapping_pair_cache::OverlappingPairCache};
use crate::bullet::{
    collision::{
        dispatch::collision_dispatcher::CollisionDispatcher,
        narrowphase::persistent_manifold::ContactAddedCallback,
    },
    dynamics::rigid_body::RigidBody,
};

/// Symmetric world AABB half extent in BT
pub(super) const AXIS_SWEEP_WORLD_HALF_EXTENT: f32 = f32::from_bits(0x4623_d6f6);
const AXIS_SWEEP_HANDLE_MASK: u16 = 0xfffe;
pub(super) const AXIS_SWEEP_SENTINEL: u16 = 0xffff;
pub(super) const AXIS_SWEEP_MAX_HANDLES: u16 = 0x4000;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Edge {
    pos: u16,
    handle: u16,
}

impl Edge {
    #[inline]
    const fn is_max(self) -> bool {
        self.pos & 1 != 0
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct HandleEdges {
    min: [u16; 3],
    max: [u16; 3],
}

/// Axis sweep broadphase.
///
/// Handle 0 is the sentinel. Live handles start at 1.
#[derive(Clone)]
pub(super) struct AxisSweep3 {
    handles: Vec<BroadphaseProxy>,
    handle_edges: Vec<HandleEdges>,
    edges: [Vec<Edge>; 3],
    num_handles: u16,
    free_list: Vec<u16>,
    next_new_handle: u16,
    world_aabb_min: Vec3A,
    quantize: Vec3A,
    pair_cache: OverlappingPairCache,
}

impl AxisSweep3 {
    pub(super) fn new() -> Self {
        let world_aabb_min = Vec3A::splat(-AXIS_SWEEP_WORLD_HALF_EXTENT);
        let world_aabb_max = Vec3A::splat(AXIS_SWEEP_WORLD_HALF_EXTENT);
        let quantize =
            Vec3A::splat(AXIS_SWEEP_SENTINEL as f32 / (world_aabb_max - world_aabb_min).x);

        let mut handles = Vec::with_capacity(64);
        handles.push(BroadphaseProxy::default());

        let mut handle_edges = Vec::with_capacity(64);
        // Handle 0 starts with the sentinel edges at indices 0 and 1.
        handle_edges.push(HandleEdges {
            min: [0; 3],
            max: [1; 3],
        });

        let sentinel_edge = Edge {
            pos: AXIS_SWEEP_SENTINEL,
            handle: 0,
        };
        let edges = [
            vec![Edge { pos: 0, handle: 0 }, sentinel_edge],
            vec![Edge { pos: 0, handle: 0 }, sentinel_edge],
            vec![Edge { pos: 0, handle: 0 }, sentinel_edge],
        ];

        Self {
            handles,
            handle_edges,
            edges,
            num_handles: 0,
            free_list: Vec::with_capacity(8),
            next_new_handle: 1,
            world_aabb_min,
            quantize,
            pair_cache: OverlappingPairCache::default(),
        }
    }

    #[inline]
    pub(super) fn handles(&self) -> &[BroadphaseProxy] {
        &self.handles
    }

    #[inline]
    pub(super) fn proxy(&self, index: usize) -> &BroadphaseProxy {
        &self.handles[index]
    }

    #[inline]
    pub(super) fn proxy_mut(&mut self, index: usize) -> &mut BroadphaseProxy {
        &mut self.handles[index]
    }

    #[inline]
    pub(super) fn retained_pairs(&self) -> &[BroadphasePair] {
        self.pair_cache.retained_pairs()
    }

    /// Add a handle and return its 1-based handle index.
    ///
    /// 1. Quantize
    /// 2. Append the two edges just inside the max sentinel
    /// 3. Sort axes 0 and 1 down without overlap events
    /// 4. Sort axis 2 down with overlap events.
    pub(super) fn add_handle(
        &mut self,
        mut proxy: BroadphaseProxy,
        dispatcher: &mut CollisionDispatcher,
    ) -> usize {
        let handle = self.alloc_handle();
        let handle_index = handle as usize;

        if handle_index >= self.handles.len() {
            self.handles
                .resize(handle_index + 1, BroadphaseProxy::default());
            self.handle_edges
                .resize(handle_index + 1, HandleEdges::default());
        }

        proxy.unique_id = handle as u32;
        let aabb = proxy.aabb;
        self.handles[handle_index] = proxy;
        self.handle_edges[handle_index] = HandleEdges::default();

        let quantized_min = self.quantize_point(aabb.min, 0);
        let quantized_max = self.quantize_point(aabb.max, 1);
        let limit = self.num_handles * 2;

        for axis in 0..3 {
            self.handle_edges[0].max[axis] = self.handle_edges[0].max[axis].wrapping_add(2);

            let needed = limit as usize + 2;
            if self.edges[axis].len() < needed {
                self.edges[axis].resize(
                    needed,
                    Edge {
                        pos: AXIS_SWEEP_SENTINEL,
                        handle: 0,
                    },
                );
            }

            // `limit - 1` is the old max sentinel. Move it one slot up and
            // insert the new min/max edges just before it.
            self.edges[axis][limit as usize + 1] = self.edges[axis][limit as usize - 1];
            self.edges[axis][limit as usize - 1] = Edge {
                pos: quantized_min[axis],
                handle,
            };
            self.edges[axis][limit as usize] = Edge {
                pos: quantized_max[axis],
                handle,
            };

            self.handle_edges[handle_index].min[axis] = limit - 1;
            self.handle_edges[handle_index].max[axis] = limit;
        }

        self.sort_added_handle::<0, false>(handle_index, dispatcher);
        self.sort_added_handle::<1, false>(handle_index, dispatcher);
        self.sort_added_handle::<2, true>(handle_index, dispatcher);

        handle_index
    }

    pub(super) fn set_aabb(
        &mut self,
        handle: usize,
        aabb: crate::shared::Aabb,
        dispatcher: &mut CollisionDispatcher,
    ) {
        self.handles[handle].aabb = aabb;

        let quantized_min = self.quantize_point(aabb.min, 0);
        let quantized_max = self.quantize_point(aabb.max, 1);

        self.update_axis::<0>(handle, quantized_min[0], quantized_max[0], dispatcher);
        self.update_axis::<1>(handle, quantized_min[1], quantized_max[1], dispatcher);
        self.update_axis::<2>(handle, quantized_min[2], quantized_max[2], dispatcher);
    }

    fn sort_added_handle<const AXIS: usize, const UPDATE_OVERLAPS: bool>(
        &mut self,
        handle: usize,
        dispatcher: &mut CollisionDispatcher,
    ) {
        let min_edge = self.handle_edges[handle].min[AXIS];
        self.sort_min_down::<AXIS, UPDATE_OVERLAPS>(min_edge);
        let max_edge = self.handle_edges[handle].max[AXIS];
        self.sort_max_down::<AXIS, UPDATE_OVERLAPS>(max_edge, dispatcher);
    }

    fn update_axis<const AXIS: usize>(
        &mut self,
        handle: usize,
        quantized_min: u16,
        quantized_max: u16,
        dispatcher: &mut CollisionDispatcher,
    ) {
        let min_edge = self.handle_edges[handle].min[AXIS];
        let max_edge = self.handle_edges[handle].max[AXIS];
        let dmin = i32::from(quantized_min) - i32::from(self.edges[AXIS][min_edge as usize].pos);
        let dmax = i32::from(quantized_max) - i32::from(self.edges[AXIS][max_edge as usize].pos);

        if dmin == 0 && dmax == 0 {
            return;
        }
        self.edges[AXIS][min_edge as usize].pos = quantized_min;
        self.edges[AXIS][max_edge as usize].pos = quantized_max;

        // Expand first (add overlaps),
        // then shrink (remove overlaps)
        if dmin < 0 {
            self.sort_min_down::<AXIS, true>(min_edge);
        }
        if dmax > 0 {
            self.sort_max_up::<AXIS, true>(max_edge);
        }
        if dmin > 0 {
            self.sort_min_up::<AXIS, true>(min_edge, dispatcher);
        }
        if dmax < 0 {
            self.sort_max_down::<AXIS, true>(max_edge, dispatcher);
        }
    }

    pub(super) fn process_all_overlapping_pairs<T>(
        &mut self,
        collision_objs: &[RigidBody],
        dispatcher: &mut CollisionDispatcher,
        contact_added_callback: &mut T,
    ) where
        T: ContactAddedCallback,
    {
        self.pair_cache.process_all_overlapping_pairs(
            collision_objs,
            dispatcher,
            &self.handles,
            contact_added_callback,
        );
    }

    fn alloc_handle(&mut self) -> u16 {
        let handle = if let Some(handle) = self.free_list.pop() {
            handle
        } else {
            let handle = self.next_new_handle;
            assert!(
                handle <= AXIS_SWEEP_MAX_HANDLES,
                "axis sweep handle capacity exceeded: {} > {}",
                handle,
                AXIS_SWEEP_MAX_HANDLES
            );
            self.next_new_handle += 1;
            handle
        };

        self.num_handles += 1;
        handle
    }

    fn quantize_point(&self, point: Vec3A, is_max: u16) -> [u16; 3] {
        let scaled = (point - self.world_aabb_min) * self.quantize;
        let clamped = Vec3A::select(
            scaled.cmpge(Vec3A::ZERO),
            scaled.min(Vec3A::splat(AXIS_SWEEP_SENTINEL as f32)),
            Vec3A::ZERO,
        );

        let quantized = unsafe {
            [
                clamped.x.to_int_unchecked::<i32>() as u16,
                clamped.y.to_int_unchecked::<i32>() as u16,
                clamped.z.to_int_unchecked::<i32>() as u16,
            ]
        };
        quantized.map(|value| (value & AXIS_SWEEP_HANDLE_MASK) | is_max)
    }

    /// Compare the sorted edge indices, not the quantized positions. Touching counts as overlap.
    fn test_overlap_2d(&self, handle_a: u16, handle_b: u16, axis: usize) -> bool {
        let axis1 = (1usize << axis) & 3;
        let axis2 = (1usize << axis1) & 3;
        let a = &self.handle_edges[handle_a as usize];
        let b = &self.handle_edges[handle_b as usize];

        !(a.max[axis1] < b.min[axis1]
            || b.max[axis1] < a.min[axis1]
            || a.max[axis2] < b.min[axis2]
            || b.max[axis2] < a.min[axis2])
    }

    fn add_pair(&mut self, handle_a: u16, handle_b: u16) {
        let a = &self.handles[handle_a as usize];
        let b = &self.handles[handle_b as usize];
        self.pair_cache
            .add_overlapping_pair(a, handle_a as usize, b, handle_b as usize);
    }

    fn remove_pair(&mut self, handle_a: u16, handle_b: u16, dispatcher: &mut CollisionDispatcher) {
        let a = self.handles[handle_a as usize];
        let b = self.handles[handle_b as usize];
        let (id0, id1) = (a.unique_id.min(b.unique_id), a.unique_id.max(b.unique_id));

        let handles = &self.handles;
        if let Some(pair) = self.pair_cache.remove_overlapping_pair(handles, id0, id1) {
            dispatcher.release_manifold(
                handles[pair.proxy0].client_obj_idx as usize,
                handles[pair.proxy1].client_obj_idx as usize,
            );
        }
    }

    /// A min edge moving down only adds overlaps.
    fn sort_min_down<const AXIS: usize, const UPDATE_OVERLAPS: bool>(&mut self, edge: u16) {
        let axis = AXIS;
        let mut index = edge as usize;
        let moving_handle = self.edges[axis][index].handle;

        while index > 0 && self.edges[axis][index].pos < self.edges[axis][index - 1].pos {
            let previous = self.edges[axis][index - 1];
            let previous_handle = previous.handle;

            if previous.is_max() {
                if UPDATE_OVERLAPS && self.test_overlap_2d(moving_handle, previous_handle, axis) {
                    self.add_pair(moving_handle, previous_handle);
                }
                self.handle_edges[previous_handle as usize].max[axis] =
                    self.handle_edges[previous_handle as usize].max[axis].wrapping_add(1);
            } else {
                self.handle_edges[previous_handle as usize].min[axis] =
                    self.handle_edges[previous_handle as usize].min[axis].wrapping_add(1);
            }

            self.handle_edges[moving_handle as usize].min[axis] =
                self.handle_edges[moving_handle as usize].min[axis].wrapping_sub(1);

            self.edges[axis].swap(index, index - 1);
            index -= 1;
        }
    }

    /// A max edge moving up only adds overlaps.
    fn sort_max_up<const AXIS: usize, const UPDATE_OVERLAPS: bool>(&mut self, edge: u16) {
        let axis = AXIS;
        let mut index = edge as usize;
        let moving_handle = self.edges[axis][index].handle;
        let len = self.edges[axis].len();

        while index + 1 < len
            && self.edges[axis][index + 1].handle != 0
            && self.edges[axis][index].pos >= self.edges[axis][index + 1].pos
        {
            let next = self.edges[axis][index + 1];
            let next_handle = next.handle;

            if !next.is_max() {
                if UPDATE_OVERLAPS && self.test_overlap_2d(moving_handle, next_handle, axis) {
                    self.add_pair(moving_handle, next_handle);
                }
                self.handle_edges[next_handle as usize].min[axis] =
                    self.handle_edges[next_handle as usize].min[axis].wrapping_sub(1);
            } else {
                self.handle_edges[next_handle as usize].max[axis] =
                    self.handle_edges[next_handle as usize].max[axis].wrapping_sub(1);
            }

            self.handle_edges[moving_handle as usize].max[axis] =
                self.handle_edges[moving_handle as usize].max[axis].wrapping_add(1);

            self.edges[axis].swap(index, index + 1);
            index += 1;
        }
    }

    /// A min edge moving up only removes overlaps.
    fn sort_min_up<const AXIS: usize, const UPDATE_OVERLAPS: bool>(
        &mut self,
        edge: u16,
        dispatcher: &mut CollisionDispatcher,
    ) {
        let axis = AXIS;
        let mut index = edge as usize;
        let moving_handle = self.edges[axis][index].handle;
        let len = self.edges[axis].len();

        while index + 1 < len
            && self.edges[axis][index + 1].handle != 0
            && self.edges[axis][index].pos >= self.edges[axis][index + 1].pos
        {
            let next = self.edges[axis][index + 1];
            let next_handle = next.handle;

            if next.is_max() {
                if UPDATE_OVERLAPS && self.test_overlap_2d(moving_handle, next_handle, axis) {
                    self.remove_pair(moving_handle, next_handle, dispatcher);
                }
                self.handle_edges[next_handle as usize].max[axis] =
                    self.handle_edges[next_handle as usize].max[axis].wrapping_sub(1);
            } else {
                self.handle_edges[next_handle as usize].min[axis] =
                    self.handle_edges[next_handle as usize].min[axis].wrapping_sub(1);
            }

            self.handle_edges[moving_handle as usize].min[axis] =
                self.handle_edges[moving_handle as usize].min[axis].wrapping_add(1);

            self.edges[axis].swap(index, index + 1);
            index += 1;
        }
    }

    /// A max edge moving down only removes overlaps.
    fn sort_max_down<const AXIS: usize, const UPDATE_OVERLAPS: bool>(
        &mut self,
        edge: u16,
        dispatcher: &mut CollisionDispatcher,
    ) {
        let axis = AXIS;
        let mut index = edge as usize;
        let moving_handle = self.edges[axis][index].handle;

        while index > 0 && self.edges[axis][index].pos < self.edges[axis][index - 1].pos {
            let previous = self.edges[axis][index - 1];
            let previous_handle = previous.handle;

            if !previous.is_max() {
                if UPDATE_OVERLAPS && self.test_overlap_2d(moving_handle, previous_handle, axis) {
                    self.remove_pair(moving_handle, previous_handle, dispatcher);
                }
                self.handle_edges[previous_handle as usize].min[axis] =
                    self.handle_edges[previous_handle as usize].min[axis].wrapping_add(1);
            } else {
                self.handle_edges[previous_handle as usize].max[axis] =
                    self.handle_edges[previous_handle as usize].max[axis].wrapping_add(1);
            }

            self.handle_edges[moving_handle as usize].max[axis] =
                self.handle_edges[moving_handle as usize].max[axis].wrapping_sub(1);

            self.edges[axis].swap(index, index - 1);
            index -= 1;
        }
    }
}
