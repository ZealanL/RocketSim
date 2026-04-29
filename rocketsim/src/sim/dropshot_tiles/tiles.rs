use glam::Vec3A;

use crate::consts::dropshot;

pub const TILE_POSITIONS: [Vec3A; dropshot::NUM_TILES_PER_TEAM] = get_tile_positions();

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
