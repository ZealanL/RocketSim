use std::ops::Deref;

use arrayvec::ArrayVec;
use glam::{USizeVec3, Vec3A};

use super::{
    broadphase_proxy::BroadphaseProxy, overlapping_pair_cache::HashedOverlappingPairCache,
};
use crate::bullet::{
    collision::{
        broadphase::broadphase_proxy::BroadphaseAabbCallback,
        dispatch::collision_dispatcher::CollisionDispatcher,
        narrowphase::persistent_manifold::ContactAddedCallback,
        shapes::collision_shape::CollisionShapes,
    },
    dynamics::rigid_body::RigidBody,
};
use crate::shared::Aabb;

pub struct GridBroadphaseProxy {
    broadphase_proxy: BroadphaseProxy,
    cell_idx: usize,
    indices: USizeVec3,
}

impl Deref for GridBroadphaseProxy {
    type Target = BroadphaseProxy;

    fn deref(&self) -> &Self::Target {
        &self.broadphase_proxy
    }
}

struct GridCell {
    dyn_handles: Vec<usize>,
    static_handles: Vec<usize>,
}

impl GridCell {
    fn new(initial_size: usize) -> Self {
        Self {
            dyn_handles: Vec::with_capacity(initial_size),
            static_handles: Vec::new(),
        }
    }
}

impl GridCell {
    fn remove_static(&mut self, proxy_idx: usize) {
        if let Some(pos) = self
            .static_handles
            .iter()
            .copied()
            .position(|x| x == proxy_idx)
        {
            self.static_handles.remove(pos);
        }
    }

    fn remove_dyn(&mut self, proxy_idx: usize) {
        if let Some(pos) = self
            .dyn_handles
            .iter()
            .copied()
            .position(|x| x == proxy_idx)
        {
            self.dyn_handles.remove(pos);
        }
    }
}

struct CellGrid {
    max_pos: Vec3A,
    min_pos: Vec3A,
    cell_size: f32,
    cell_size_sq: f32,
    num_cells: USizeVec3,
    num_dyn_proxies: u32,
    cells: Box<[GridCell]>,
}

impl CellGrid {
    fn get_cell_indices(&self, pos: Vec3A) -> USizeVec3 {
        let cell_idx_f = (pos - self.min_pos) / self.cell_size;
        cell_idx_f
            .as_usizevec3()
            .clamp(USizeVec3::ZERO, self.num_cells - USizeVec3::ONE)
    }

    fn get_cell_idx(&self, pos: Vec3A) -> usize {
        self.cell_indices_to_idx(self.get_cell_indices(pos))
    }

    const fn cell_indices_to_idx(&self, indices: USizeVec3) -> usize {
        indices.x * self.num_cells.y * self.num_cells.z + indices.y * self.num_cells.z + indices.z
    }

    fn get_cell_min_pos(&self, indices: USizeVec3) -> Vec3A {
        self.min_pos + indices.as_vec3a() * self.cell_size
    }

    fn get_cell(&mut self, indices: USizeVec3) -> &mut GridCell {
        let idx = self.cell_indices_to_idx(indices);
        &mut self.cells[idx]
    }

    fn update_cells_static<const ADD: bool>(
        &mut self,
        proxy: &GridBroadphaseProxy,
        col_obj: &RigidBody,
        proxy_idx: usize,
    ) {
        // TODO: "Fix dumb massive value aabb bug"
        let aabb_max = proxy.aabb.max.min(self.max_pos);

        let min = self.get_cell_indices(proxy.aabb.min);
        let max = self.get_cell_indices(aabb_max);

        let tri_mesh_shape = match col_obj.get_collision_shape() {
            CollisionShapes::TriangleMesh(mesh) => Some(mesh.as_ref()),
            _ => None,
        };

        let mut cells = ArrayVec::<usize, 27>::new();

        for i in min.x..=max.x {
            for j in min.y..=max.y {
                for k in min.z..=max.z {
                    debug_assert!(cells.is_empty());
                    if ADD && let Some(mesh_interface) = tri_mesh_shape {
                        let cell_min = self.get_cell_min_pos(USizeVec3::new(i, j, k));
                        let cell_aabb =
                            Aabb::new(cell_min, cell_min + Vec3A::splat(self.cell_size));

                        if !mesh_interface.check_overlap_with(&cell_aabb) {
                            continue;
                        }
                    }

                    for i1 in 0..=2 {
                        for j1 in 0..=2 {
                            for k1 in 0..=2 {
                                let mut cell = USizeVec3::new(i + i1, j + j1, k + k1);
                                if cell.cmpeq(USizeVec3::ZERO).any() {
                                    continue;
                                }

                                cell -= USizeVec3::ONE;

                                if cell.cmpge(self.num_cells).any() {
                                    continue;
                                }

                                cells.push(self.cell_indices_to_idx(cell));
                            }
                        }
                    }

                    for &i in &cells {
                        if ADD {
                            let mut already_exists = false;
                            for &static_handle in &self.cells[i].static_handles {
                                // check if static_handle and proxy are the same
                                if static_handle == proxy_idx {
                                    already_exists = true;
                                    break;
                                }
                            }

                            if !already_exists {
                                self.cells[i].static_handles.push(proxy_idx);
                            }
                        } else {
                            self.cells[i].remove_static(proxy_idx);
                        }
                    }

                    cells.clear();
                }
            }
        }
    }

    fn update_cells_dynamic<const ADD: bool>(&mut self, proxy_idx: usize, indices: USizeVec3) {
        let min = USizeVec3::ONE.max(indices) - USizeVec3::ONE;
        let max = (self.num_cells - USizeVec3::ONE).min(indices + USizeVec3::ONE);

        for i in min.x..=max.x {
            for j in min.y..=max.y {
                for k in min.z..=max.z {
                    let cell = self.get_cell(USizeVec3::new(i, j, k));

                    if ADD {
                        cell.dyn_handles.push(proxy_idx);
                    } else {
                        cell.remove_dyn(proxy_idx);
                    }
                }
            }
        }
    }
}

pub struct GridBroadphase {
    cell_grid: CellGrid,
    min_dyn_handle_idx: usize,
    pub handles: Vec<GridBroadphaseProxy>,
    pair_cache: HashedOverlappingPairCache,
}

impl GridBroadphase {
    pub fn new(
        min_pos: Vec3A,
        max_pos: Vec3A,
        cell_size: f32,
        initial_handles_size: usize,
        pair_cache: HashedOverlappingPairCache,
    ) -> Self {
        debug_assert!(min_pos.cmple(max_pos).all(), "Invalid min/max pos");

        let range = max_pos - min_pos;
        let num_cells = (range / cell_size)
            .ceil()
            .as_usizevec3()
            .max(USizeVec3::ONE);
        let total_cells = num_cells.element_product();
        let cells = (0..total_cells)
            .map(|_| GridCell::new(initial_handles_size))
            .collect();

        Self {
            min_dyn_handle_idx: 0,
            cell_grid: CellGrid {
                max_pos,
                min_pos,
                cell_size,
                cell_size_sq: cell_size * cell_size,
                num_cells,
                num_dyn_proxies: 0,
                cells,
            },
            handles: Vec::with_capacity(32),
            pair_cache,
        }
    }

    pub fn set_aabb(&mut self, col_obj: &RigidBody, proxy_idx: usize, aabb: Aabb) {
        let sbp = &mut self.handles[proxy_idx];

        if sbp.aabb.min != aabb.min || sbp.aabb.max != aabb.max {
            if sbp.is_static {
                self.cell_grid
                    .update_cells_static::<false>(sbp, col_obj, proxy_idx);

                sbp.broadphase_proxy.aabb = aabb;

                self.cell_grid
                    .update_cells_static::<true>(sbp, col_obj, proxy_idx);
            } else {
                let old_idx = sbp.cell_idx;
                sbp.broadphase_proxy.aabb = aabb;

                let new_indices = self.cell_grid.get_cell_indices(aabb.min);
                let new_idx = self.cell_grid.cell_indices_to_idx(new_indices);
                self.handles[proxy_idx].cell_idx = new_idx;

                if new_idx != old_idx && self.cell_grid.num_dyn_proxies > 1 {
                    self.cell_grid
                        .update_cells_dynamic::<false>(proxy_idx, new_indices);
                    self.handles[proxy_idx].indices = new_indices;
                    self.cell_grid
                        .update_cells_dynamic::<true>(proxy_idx, new_indices);
                }
            }
        }
    }

    pub fn create_proxy(
        &mut self,
        aabb: Aabb,
        co: &RigidBody,
        collision_filter_group: u8,
        collision_filter_mask: u8,
    ) -> usize {
        debug_assert!(aabb.min.cmple(aabb.max).all());

        let is_static = co.is_static_object();
        let world_idx = co.world_array_idx;

        let new_handle_idx = self.handles.len();
        let indices = self.cell_grid.get_cell_indices(aabb.min);
        let cell_idx = self.cell_grid.cell_indices_to_idx(indices);

        let new_handle = GridBroadphaseProxy {
            broadphase_proxy: BroadphaseProxy {
                aabb,
                client_object_idx: world_idx,
                collision_filter_group,
                collision_filter_mask,
                is_static,
                unique_id: u32::try_from(new_handle_idx).unwrap() + 2,
            },
            cell_idx,
            indices,
        };

        if is_static {
            if self.cell_grid.num_dyn_proxies == 0 {
                self.min_dyn_handle_idx = new_handle_idx + 1;
            }

            self.cell_grid
                .update_cells_static::<true>(&new_handle, co, new_handle_idx);
        } else {
            debug_assert!(aabb.min.distance_squared(aabb.max) <= self.cell_grid.cell_size_sq);

            self.cell_grid
                .update_cells_dynamic::<true>(new_handle_idx, indices);
            self.cell_grid.num_dyn_proxies += 1;
        }

        self.handles.push(new_handle);
        new_handle_idx
    }

    pub fn calculate_overlapping_pairs(&mut self) {
        debug_assert!(self.pair_cache.is_empty());
        if self.cell_grid.num_dyn_proxies == 0 {
            return;
        }

        for (i, proxy) in self
            .handles
            .iter()
            .enumerate()
            .skip(self.min_dyn_handle_idx)
            .filter(|(_, proxy)| !proxy.is_static)
        {
            let cell = &self.cell_grid.cells[proxy.cell_idx];
            for &other_proxy_idx in &cell.static_handles {
                let other_proxy = &self.handles[other_proxy_idx];

                if proxy.aabb.intersects(&other_proxy.aabb)
                    && !self.pair_cache.contains_pair(proxy, other_proxy)
                {
                    self.pair_cache
                        .add_overlapping_pair(proxy, i, other_proxy, other_proxy_idx);
                }
            }

            if self.cell_grid.num_dyn_proxies > 1 && !cell.dyn_handles.is_empty() {
                for &other_proxy_idx in &cell.dyn_handles {
                    if i == other_proxy_idx {
                        continue;
                    }

                    let other_proxy = &self.handles[other_proxy_idx];
                    if proxy.aabb.intersects(&other_proxy.aabb)
                        && !self.pair_cache.contains_pair(proxy, other_proxy)
                    {
                        self.pair_cache.add_overlapping_pair(
                            proxy,
                            i,
                            other_proxy,
                            other_proxy_idx,
                        );
                    }
                }
            }
        }
    }

    pub fn process_all_overlapping_pairs<T: ContactAddedCallback>(
        &mut self,
        collision_objects: &[RigidBody],
        dispatcher: &mut CollisionDispatcher,
        contact_added_callback: &mut T,
    ) {
        self.pair_cache.process_all_overlapping_pairs(
            collision_objects,
            dispatcher,
            &self.handles,
            contact_added_callback,
        );
    }

    pub fn ray_test<T: BroadphaseAabbCallback>(
        &self,
        ray_from: &[Vec3A; 4],
        ray_to: &[Vec3A; 4],
        ray_callback: &mut T,
    ) {
        debug_assert!(ray_from[0].distance_squared(ray_to[0]) < self.cell_grid.cell_size_sq);
        debug_assert!(ray_from[1].distance_squared(ray_to[1]) < self.cell_grid.cell_size_sq);
        debug_assert!(ray_from[2].distance_squared(ray_to[2]) < self.cell_grid.cell_size_sq);
        debug_assert!(ray_from[3].distance_squared(ray_to[3]) < self.cell_grid.cell_size_sq);
        let cell = &self.cell_grid.cells[self.cell_grid.get_cell_idx(ray_from[0])];

        for &other_proxy_idx in cell.static_handles.iter().chain(&cell.dyn_handles) {
            let other_proxy = &self.handles[other_proxy_idx];
            ray_callback.process(other_proxy);
        }
    }
}
