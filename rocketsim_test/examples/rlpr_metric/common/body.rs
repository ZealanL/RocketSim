use glam::Vec3A;
use rocketsim::consts::BT_TO_UU;
use rocketsim_test::rlpr::{cpp_records::RecordingInfo, tick_record::TickRecord};

use super::rows::tick_car_count;

/// Number of known body presets (Octane, Dominus, Plank, Breakout, Hybrid,
/// Merc, Psyclops).
pub const NUM_BODY_PRESETS: usize = 7;

/// Preset display names in preset-index order.
pub const BODY_PRESET_NAMES: [&str; NUM_BODY_PRESETS] = [
    "Octane", "Dominus", "Plank", "Breakout", "Hybrid", "Merc", "Psyclops",
];

/// Largest header-vs-preset mismatch that still counts as a match, in uu.
///
/// The header stores f32 bounds in BT; scaling by [`BT_TO_UU`] leaves about
/// 4e-4 uu of rounding on the recorded captures. The closest presets
/// (Octane and Psyclops sizes) differ by 0.134 uu, so 0.01 separates every
/// known preset with wide margin on both sides.
pub const BODY_MATCH_TOL_UU: f32 = 0.01;

/// One preset's hitbox bounds in uu: full size plus center offset.
#[derive(Clone, Copy, Debug)]
pub struct HitboxBounds {
    pub size: Vec3A,
    pub offset: Vec3A,
}

/// Match recording header hitbox bounds to one known body preset index.
///
/// Scales the `RecordingInfo` min/max from BT to uu, then compares full size
/// and center offset against every entry of `bounds`. Unknown bounds are an
/// error: the metric must fail rather than score a guessed body. The header
/// samples the first recorded car only, so it cannot see a mixed roster.
///
/// `bounds` is supplied per backend so the v2 and v3 sims are matched against
/// their own preset tables.
pub fn body_preset_from_info(
    info: &RecordingInfo,
    bounds: &[HitboxBounds; NUM_BODY_PRESETS],
) -> Result<usize, String> {
    let min: Vec3A = info.hitbox_rel_min_bt.into();
    let max: Vec3A = info.hitbox_rel_max_bt.into();
    let size_uu = (max - min) * BT_TO_UU;
    let offset_uu = (max + min) * 0.5 * BT_TO_UU;
    for (index, preset) in bounds.iter().enumerate() {
        let size_err = (size_uu - preset.size).abs().max_element();
        let offset_err = (offset_uu - preset.offset).abs().max_element();
        if size_err <= BODY_MATCH_TOL_UU && offset_err <= BODY_MATCH_TOL_UU {
            return Ok(index);
        }
    }
    Err(format!(
        "unknown car body (size {size_uu:?} uu, offset {offset_uu:?} uu): no preset matches within {BODY_MATCH_TOL_UU} uu"
    ))
}

/// Largest settled-height mismatch that still counts as a body match, in uu.
///
/// A car resting level on the ground settles at a height fixed by its wheel
/// geometry, and the sim reproduces RL's settled height to under 0.001 uu on
/// every bundled capture. The closest separable presets (Octane and Dominus)
/// differ by 0.041 uu, so 0.02 sits between the measurement error and the
/// smallest real gap. Octane and Hybrid share their wheels and settle
/// identically, so they cannot be told apart this way at all.
pub const SETTLED_HEIGHT_TOL_UU: f32 = 0.02;

/// Minimum speed for a tick to count toward a car's settled height, in uu/s.
const SETTLED_MAX_SPEED_UU_S: f32 = 1.0;

/// Minimum car up-axis z for a tick to count, so tilted and flipped cars are
/// excluded from the settled-height sample.
const SETTLED_MIN_UP_Z: f32 = 0.999;

/// Settled chassis height of one recorded car, in uu.
///
/// The median height over every tick where the car is on the ground, level,
/// and at rest. The median, not the mean: a handful of frames per recording
/// are grounded and level but not actually settled, such as the first tick
/// after a goal reset, and they drag a mean far enough to break matching.
/// The bulk of the sample is exact rather than statistical — on all four
/// bundled captures the interquartile spread is under 0.001 uu, because a
/// level rest height is a geometric constant of the body. `None` when the
/// car is never seen settled.
pub fn car_settled_height(ticks: &[TickRecord], car_idx: usize) -> Option<f32> {
    let mut heights: Vec<f32> = ticks
        .iter()
        .filter_map(|tick| {
            let car = tick.car_records.get(car_idx)?;
            if !car.is_on_ground {
                return None;
            }
            if Vec3A::from(car.phys.rot.column(2)).z < SETTLED_MIN_UP_Z {
                return None;
            }
            if Vec3A::from(car.phys.lin_vel).length() > SETTLED_MAX_SPEED_UU_S {
                return None;
            }
            Some(car.phys.pos.z)
        })
        .collect();
    if heights.is_empty() {
        return None;
    }
    heights.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    Some(heights[heights.len() / 2])
}

/// Match one car's settled height to a body preset.
///
/// Returns the preset whose reference height is within
/// [`SETTLED_HEIGHT_TOL_UU`].
///
/// Several presets can match at once only because they share wheels: Octane
/// and Hybrid have identical suspension and settle at the same height, and
/// nothing in a recording distinguishes their hitboxes. `preferred` is the
/// header's preset, which resolves that tie whenever the header body's own
/// wheels match this car. Otherwise the lowest-index candidate wins, so an
/// unresolvable Hybrid is reported as Octane rather than silently dropped.
///
/// Returns an error when nothing matches: a settled height that fits no
/// preset means either the body is custom or the engine no longer reproduces
/// RL's rest geometry, and the metric must fail rather than score a guess.
pub fn body_from_settled_height(
    height: f32,
    reference: &[f32; NUM_BODY_PRESETS],
    preferred: Option<usize>,
) -> Result<usize, String> {
    let candidates: Vec<usize> = (0..NUM_BODY_PRESETS)
        .filter(|&i| (height - reference[i]).abs() <= SETTLED_HEIGHT_TOL_UU)
        .collect();
    match candidates.first() {
        None => Err(format!(
            "settled height {height:.4} uu matches no preset within {SETTLED_HEIGHT_TOL_UU} uu (reference {reference:?})"
        )),
        Some(&first) => Ok(preferred
            .filter(|p| candidates.contains(p))
            .unwrap_or(first)),
    }
}

/// Body preset for every recorded car, one entry per car slot.
///
/// The recording header only carries the first car's hitbox, so a mixed
/// roster is invisible to [`body_preset_from_info`]. Each car's settled
/// resting height fingerprints its wheels, which is what detects the rest of
/// a mixed roster: the bundled `london_vs_nexto_1v1` capture is Plank against
/// Octane and the header alone only ever described the Plank car.
///
/// `reference` is each preset's settled height from the sim and `header_body`
/// is the header's preset, used to break the Octane/Hybrid wheel tie and as
/// the fallback for a car never seen settled. A car the settled height
/// contradicts the header is still resolved from its own wheels: the header
/// is an exact hitbox reading but only ever sampled car 0.
pub fn bodies_from_ticks(
    ticks: &[TickRecord],
    header_body: usize,
    reference: &[f32; NUM_BODY_PRESETS],
) -> Result<Vec<usize>, String> {
    let num_cars = ticks.first().map_or(0, tick_car_count);
    (0..num_cars)
        .map(|car_idx| match car_settled_height(ticks, car_idx) {
            Some(height) => body_from_settled_height(height, reference, Some(header_body))
                .map_err(|err| format!("car {car_idx}: {err}")),
            None => Ok(header_body),
        })
        .collect()
}

/// Car slots that use the header body because no settled height was found.
///
/// The header hitbox describes car 0 only. A non-empty result means the
/// roster is not measured for those slots and the caller must not present it
/// as a per-car measurement.
pub fn header_fallback_slots(ticks: &[TickRecord]) -> Vec<usize> {
    let num_cars = ticks.first().map_or(0, tick_car_count);
    (0..num_cars)
        .filter(|&car_idx| car_settled_height(ticks, car_idx).is_none())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::test_support::*;

    #[test]
    fn header_fallback_slots_reports_never_settled_cars() {
        let mut tick = multi_tick(0, 0.0, 2, 0b11);
        tick.car_records[0].is_on_ground = true;
        tick.car_records[0].phys.lin_vel = vec(0.0, 0.0, 0.0);
        tick.car_records[1].is_on_ground = false;
        assert_eq!(header_fallback_slots(&[tick]), vec![1]);
    }
}
