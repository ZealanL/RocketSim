use glam::{IVec3, USizeVec3, Vec3A};

use super::{broadphase_proxy::BroadphaseProxy, overlapping_pair_cache::OverlappingPairCache};
use crate::{
    bullet::{
        collision::{
            broadphase::{CollisionFilterGroups, broadphase_proxy::BroadphaseAabbCallback},
            dispatch::collision_dispatcher::CollisionDispatcher,
            narrowphase::persistent_manifold::ContactAddedCallback,
            shapes::collision_shape::CollisionShapes,
        },
        dynamics::rigid_body::RigidBody,
        linear_math::AffineExt,
    },
    shared::Aabb,
};

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
    cells: Box<[GridCell]>,
}

impl CellGrid {
    fn get_cell_indices(&self, pos: Vec3A) -> USizeVec3 {
        let cell_idx_f = (pos - self.min_pos) / self.cell_size;
        unsafe {
            IVec3 {
                x: cell_idx_f.x.to_int_unchecked::<i32>(),
                y: cell_idx_f.y.to_int_unchecked::<i32>(),
                z: cell_idx_f.z.to_int_unchecked::<i32>(),
            }
        }
        .max(IVec3::ZERO)
        .as_usizevec3()
        .min(self.num_cells - USizeVec3::ONE)
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

    fn update_cells_static(
        &mut self,
        proxy: &BroadphaseProxy,
        col_obj: &RigidBody,
        proxy_idx: usize,
    ) {
        let min = self.get_cell_indices(proxy.aabb.min.max(self.min_pos));
        let max = self.get_cell_indices(proxy.aabb.max.min(self.max_pos));

        let tri_mesh_shape = match col_obj.get_collision_shape() {
            CollisionShapes::TriangleMesh(mesh) => Some(mesh.as_ref()),
            _ => None,
        };
        // Mesh BVHs hold local triangles. Goal components carry a body
        // translation, so test world grid cells in mesh-local space.
        let world_to_local = col_obj.get_world_trans().transpose();

        for i in min.x..=max.x {
            for j in min.y..=max.y {
                for k in min.z..=max.z {
                    if let Some(mesh_interface) = tri_mesh_shape {
                        let cell_min = self.get_cell_min_pos(USizeVec3::new(i, j, k));
                        let cell_aabb =
                            Aabb::new(cell_min, cell_min + Vec3A::splat(self.cell_size));
                        let local_cell = cell_aabb.transform(&world_to_local, 0.0);

                        if !mesh_interface.check_overlap_with(&local_cell) {
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

                                let i = self.cell_indices_to_idx(cell);
                                if self.cells[i].static_handles.contains(&proxy_idx) {
                                    continue;
                                }

                                self.cells[i].static_handles.push(proxy_idx);
                            }
                        }
                    }
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
    pub handles: Vec<BroadphaseProxy>,
    pair_cache: OverlappingPairCache,
}

impl GridBroadphase {
    pub fn new(
        min_pos: Vec3A,
        max_pos: Vec3A,
        cell_size: f32,
        initial_handles_size: usize,
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
                cells,
            },
            handles: Vec::with_capacity(32),
            pair_cache: OverlappingPairCache::default(),
        }
    }

    #[cfg(test)]
    pub(crate) const fn num_cells(&self) -> USizeVec3 {
        self.cell_grid.num_cells
    }

    pub fn set_aabb(&mut self, col_obj: &RigidBody, proxy_idx: usize, aabb: Aabb) {
        let sbp = &mut self.handles[proxy_idx];
        sbp.aabb = aabb;

        if (sbp.collision_filter_group & CollisionFilterGroups::Static) != 0 {
            self.cell_grid.update_cells_static(sbp, col_obj, proxy_idx);
        } else {
            let old_idx = sbp.cell_idx as usize;
            let new_indices = self.cell_grid.get_cell_indices(aabb.min);
            let new_idx = self.cell_grid.cell_indices_to_idx(new_indices);

            self.handles[proxy_idx].cell_idx = u32::try_from(new_idx).unwrap();
            if new_idx != old_idx {
                let [x, y, z] = self.handles[proxy_idx].indices;
                self.cell_grid.update_cells_dynamic::<false>(
                    proxy_idx,
                    USizeVec3::new(x as usize, y as usize, z as usize),
                );
                self.handles[proxy_idx].indices = new_indices.as_uvec3().to_array();
                self.cell_grid
                    .update_cells_dynamic::<true>(proxy_idx, new_indices);
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

        let is_static = (collision_filter_group & CollisionFilterGroups::Static) != 0;
        let new_handle_idx = self.handles.len();
        let indices = self.cell_grid.get_cell_indices(aabb.min);
        let cell_idx = self.cell_grid.cell_indices_to_idx(indices);

        let new_handle = BroadphaseProxy {
            aabb,
            client_obj_idx: u32::try_from(co.world_array_idx).unwrap(),
            collision_filter_group,
            collision_filter_mask,
            unique_id: u32::try_from(new_handle_idx).unwrap(),
            cell_idx: u32::try_from(cell_idx).unwrap(),
            indices: indices.as_uvec3().to_array(),
        };

        if is_static {
            self.min_dyn_handle_idx = new_handle_idx + 1;
            self.cell_grid
                .update_cells_static(&new_handle, co, new_handle_idx);
        } else {
            debug_assert!(
                aabb.min.distance_squared(aabb.max) <= self.cell_grid.cell_size_sq,
                "Dynamic objects must fit within a single cell - ({} > {})",
                aabb.min.distance_squared(aabb.max),
                self.cell_grid.cell_size_sq
            );

            self.cell_grid
                .update_cells_dynamic::<true>(new_handle_idx, indices);
        }

        self.handles.push(new_handle);
        new_handle_idx
    }

    pub fn calculate_overlapping_pairs(&mut self) {
        debug_assert!(self.pair_cache.is_empty());
        for (i, proxy) in self
            .handles
            .iter()
            .enumerate()
            .skip(self.min_dyn_handle_idx)
            .filter(|(_, proxy)| {
                (proxy.collision_filter_group & CollisionFilterGroups::Static) == 0
            })
        {
            let cell = &self.cell_grid.cells[proxy.cell_idx as usize];
            for &other_proxy_idx in &cell.static_handles {
                let other_proxy = &self.handles[other_proxy_idx];

                if proxy.aabb.intersects(&other_proxy.aabb) {
                    self.pair_cache
                        .add_overlapping_pair(proxy, i, other_proxy, other_proxy_idx);
                }
            }

            for &other_proxy_idx in &cell.dyn_handles {
                if i >= other_proxy_idx {
                    continue;
                }

                let other_proxy = &self.handles[other_proxy_idx];
                if proxy.aabb.intersects(&other_proxy.aabb) {
                    self.pair_cache
                        .add_overlapping_pair(proxy, i, other_proxy, other_proxy_idx);
                }
            }
        }
    }

    pub fn process_all_overlapping_pairs<T: ContactAddedCallback>(
        &mut self,
        collision_objs: &[RigidBody],
        dispatcher: &mut CollisionDispatcher,
        contact_added_callback: &mut T,
    ) {
        self.pair_cache.process_all_overlapping_pairs(
            collision_objs,
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
        let ray_aabb = ray_from.iter().zip(ray_to).skip(1).fold(
            Aabb::new(ray_from[0].min(ray_to[0]), ray_from[0].max(ray_to[0])),
            |bounds, (from, to)| bounds.combine(&Aabb::new(from.min(*to), from.max(*to))),
        );

        for &other_proxy_idx in cell.static_handles.iter().chain(&cell.dyn_handles) {
            let other_proxy = &self.handles[other_proxy_idx];
            if ray_aabb.intersects(&other_proxy.aabb) {
                ray_callback.process(other_proxy);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::{USizeVec3, Vec3A};

    use super::GridBroadphase;

    #[test]
    fn arena_sized_cell_disables_grid_partitioning() {
        let min_pos = Vec3A::new(-5600.0, -6000.0, 0.0);
        let max_pos = Vec3A::new(5600.0, 6000.0, 2200.0);
        let cell_size = (max_pos - min_pos).max_element();
        let broadphase = GridBroadphase::new(min_pos, max_pos, cell_size, 1);

        assert_eq!(broadphase.num_cells(), USizeVec3::ONE);
    }
}
