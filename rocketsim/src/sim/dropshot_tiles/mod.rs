mod state;
mod tiles;

use std::array::from_fn;

use arrayvec::ArrayVec;
use glam::Vec3A;
pub use state::{TileDamageState, TileStates};

use crate::{
    Team,
    bullet::collision::shapes::convex_hull_shape::ConvexHullShape,
    consts::{UU_TO_BT, dropshot},
};

pub(crate) struct TileNeighbors {
    tile_neighbors_1: [ArrayVec<usize, 9>; dropshot::NUM_TILES_PER_TEAM],
    tile_neighbors_2: [ArrayVec<usize, 20>; dropshot::NUM_TILES_PER_TEAM],
}

impl Default for TileNeighbors {
    fn default() -> Self {
        let (tile_neighbors_1, tile_neighbors_2) = Self::get_tile_neighbors();

        Self {
            tile_neighbors_1,
            tile_neighbors_2,
        }
    }
}

impl TileNeighbors {
    #[must_use]
    pub fn get_tile_pos(team: Team, index: usize) -> Vec3A {
        tiles::TILE_POSITIONS[index] * f32::from(team as i8 * 2 - 1)
    }

    fn get_tile_neighbors() -> (
        [ArrayVec<usize, 9>; dropshot::NUM_TILES_PER_TEAM],
        [ArrayVec<usize, 20>; dropshot::NUM_TILES_PER_TEAM],
    ) {
        const NEIGHBOR_MAX_RADIUS: f32 = dropshot::TILE_WIDTH_X * 1.2;

        let mut tile_neighbors_1 = from_fn(|_| ArrayVec::new());
        let mut tile_neighbors_2 = from_fn(|_| ArrayVec::new());

        for i in 0..dropshot::NUM_TILES_PER_TEAM {
            let pos = Self::get_tile_pos(Team::Blue, i);

            let neightbor_map_1 = &mut tile_neighbors_1[i];
            let neightbor_map_2 = &mut tile_neighbors_2[i];

            for j in 0..dropshot::NUM_TILES_PER_TEAM {
                let other_pos = Self::get_tile_pos(Team::Blue, j);
                let dist = pos.distance(other_pos);

                if dist < NEIGHBOR_MAX_RADIUS {
                    neightbor_map_1.push(j);
                }

                if dist < NEIGHBOR_MAX_RADIUS * 2.0 {
                    neightbor_map_2.push(j);
                }
            }
        }

        (tile_neighbors_1, tile_neighbors_2)
    }

    #[must_use]
    pub fn get_neighbor_indices_1(&self, start_idx: usize) -> &[usize] {
        &self.tile_neighbors_1[start_idx]
    }

    #[must_use]
    pub fn get_neighbor_indices_2(&self, start_idx: usize) -> &[usize] {
        &self.tile_neighbors_2[start_idx]
    }

    pub fn make_tile_shapes() -> impl Iterator<Item = ConvexHullShape> {
        Team::ALL.into_iter().flat_map(|team| {
            (0..dropshot::NUM_TILES_PER_TEAM).map(move |i| {
                let pos = Self::get_tile_pos(team, i);

                let mut points = Vec::with_capacity(6);
                for j in 0..6 {
                    const CLAMP_Y: f32 = dropshot::TILE_OFFSET_Y * UU_TO_BT;

                    let mut vert = pos * UU_TO_BT + dropshot::TILE_HEXAGON_VERTS_BT[j];

                    // Clamp vert from crossing middle part at x=0
                    vert.y = match team {
                        Team::Blue => vert.y.min(-CLAMP_Y),
                        Team::Orange => vert.y.max(CLAMP_Y),
                    };

                    points.push(vert);
                }

                ConvexHullShape::new(points.into_boxed_slice())
            })
        })
    }
}
