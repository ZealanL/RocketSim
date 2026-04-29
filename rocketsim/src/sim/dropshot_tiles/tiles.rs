use glam::{Vec3, Vec3A};

use crate::{Team, consts::dropshot};

pub const TILE_POSITIONS: [Vec3A; dropshot::NUM_TILES_PER_TEAM] = get_tile_positions();
pub const TILE_NEIGHBORS_1: [Varr<7>; dropshot::NUM_TILES_PER_TEAM] = get_tile_neighbors::<1, _>();
pub const TILE_NEIGHBORS_2: [Varr<19>; dropshot::NUM_TILES_PER_TEAM] = get_tile_neighbors::<2, _>();

const fn get_tile_positions() -> [Vec3A; dropshot::NUM_TILES_PER_TEAM] {
    let mut tile_positions = [Vec3A::ZERO; dropshot::NUM_TILES_PER_TEAM];

    let mut cur_idx = 0;
    let mut y = dropshot::TILE_OFFSET_Y;

    let mut i = 0;
    while i < dropshot::NUM_TILE_ROWS {
        let num_tiles = dropshot::TILES_IN_FIRST_ROW - i;
        let row_size_x = dropshot::TILE_WIDTH_X * num_tiles as f32;
        let row_start_x = -(row_size_x / 2.0) + (dropshot::TILE_WIDTH_X / 2.0);

        let mut j = 0;
        while j < num_tiles {
            let x = row_start_x + dropshot::TILE_WIDTH_X * j as f32;
            tile_positions[cur_idx] = Vec3A::new(x, y, 0.0);
            cur_idx += 1;
            j += 1;
        }

        y += dropshot::ROW_OFFSET_Y;
        i += 1;
    }

    assert!(
        cur_idx == dropshot::NUM_TILES_PER_TEAM,
        "Failed to reach tile amount, make sure tile info is correct"
    );

    tile_positions
}

#[derive(Clone, Copy)]
pub struct Varr<const N: usize> {
    arr: [usize; N],
    len: usize,
}

impl<const N: usize> Varr<N> {
    const fn new() -> Self {
        Self {
            arr: [0; N],
            len: 0,
        }
    }

    const fn push(&mut self, value: usize) {
        if self.len >= N {
            panic!("Varr overflow");
        }

        self.arr[self.len] = value;
        self.len += 1;
    }

    pub fn as_slice(&self) -> &[usize] {
        &self.arr[..self.len]
    }
}

const fn get_tile_pos(team: Team, index: usize) -> Vec3 {
    let [x, y, z] = TILE_POSITIONS[index].to_array();
    let team = (team as i8 * 2 - 1) as f32;
    Vec3::new(x * team, y * team, z * team)
}

const fn get_tile_neighbors<const RADIUS: u8, const MAX_NEIGHBORS: usize>()
-> [Varr<MAX_NEIGHBORS>; dropshot::NUM_TILES_PER_TEAM] {
    let neighbor_max_radius = dropshot::TILE_WIDTH_X * 1.2 * RADIUS as f32;
    let neighbor_max_radius_sq = neighbor_max_radius * neighbor_max_radius;

    let mut neighbors = [Varr::new(); dropshot::NUM_TILES_PER_TEAM];

    let mut i = 0;
    while i < dropshot::NUM_TILES_PER_TEAM {
        let pos = get_tile_pos(Team::Blue, i);

        let mut j = 0;
        while j < dropshot::NUM_TILES_PER_TEAM {
            let other_pos = get_tile_pos(Team::Blue, j);
            let diff = Vec3 {
                x: pos.x - other_pos.x,
                y: pos.y - other_pos.y,
                z: pos.z - other_pos.z,
            };

            let dist_sq = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
            if dist_sq < neighbor_max_radius_sq {
                neighbors[i].push(j);
            }

            j += 1;
        }

        i += 1;
    }

    neighbors
}
