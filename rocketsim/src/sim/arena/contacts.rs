use glam::Vec3A;

use super::Arena;

/// Combine per-world-object contact normal sums into the world-contact normal.
///
/// Each object sum is normalized. The object normals are added. The total is
/// normalized. An object whose sum is near zero is skipped. Returns `None`
/// when no object sum or the total is usable.
fn combine_world_contact_normal_sums(sums: impl IntoIterator<Item = Vec3A>) -> Option<Vec3A> {
    const MIN_LEN_SQ: f32 = 1e-8;
    let mut total = Vec3A::ZERO;
    for sum in sums {
        let len_sq = sum.length_squared();
        if len_sq >= MIN_LEN_SQ {
            total += sum / len_sq.sqrt();
        }
    }
    let len_sq = total.length_squared();
    (len_sq >= MIN_LEN_SQ).then(|| total / len_sq.sqrt())
}

impl Arena {
    /// Build the game's car world-contact normal for this tick.
    ///
    /// Per world object, sum the contact normals and normalize the sum. Then
    /// sum the object normals and normalize the result. A car whose sums are
    /// all unusable keeps its last contact normal. No object count limit
    /// applies.
    pub(super) fn update_world_contact_normal(&mut self) {
        let car_count = self.cars.len();
        for car_idx in 0..car_count {
            let combined = combine_world_contact_normal_sums(
                self.contact_tracker
                    .world_contact_normal_groups()
                    .iter()
                    .filter(|group| group.car_idx == car_idx)
                    .map(|group| group.sum),
            );
            if let Some(normal) = combined {
                self.cars[car_idx].state.cache.world_contact_normal = Some(normal);
            }
        }
    }
}
