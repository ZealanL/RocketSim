mod state;
mod tiles;

use glam::Vec3A;
pub use state::{TileDamageState, TileStates};

use crate::{
    Team,
    bullet::collision::shapes::convex_hull_shape::ConvexHullShape,
    consts::{UU_TO_BT, dropshot},
};

#[must_use]
pub(crate) fn get_tile_pos(team: Team, index: usize) -> Vec3A {
    tiles::TILE_POSITIONS[index] * f32::from(team as i8 * 2 - 1)
}

#[must_use]
pub(crate) fn get_neighbor_indices_1(start_idx: usize) -> &'static [usize] {
    tiles::TILE_NEIGHBORS_1[start_idx].as_slice()
}

#[must_use]
pub(crate) fn get_neighbor_indices_2(start_idx: usize) -> &'static [usize] {
    tiles::TILE_NEIGHBORS_2[start_idx].as_slice()
}

pub(crate) fn make_tile_shapes() -> impl Iterator<Item = ConvexHullShape> {
    Team::ALL.into_iter().flat_map(|team| {
        (0..dropshot::NUM_TILES_PER_TEAM).map(move |i| {
            let pos = get_tile_pos(team, i);

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
