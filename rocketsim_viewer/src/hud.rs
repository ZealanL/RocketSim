//! Boost meter helpers: pure math for the viewer HUD.
//!
//! The viewer draws the bar; this module only maps raw boost to plain
//! numbers and text. No glam, no egui here, so tests stay fast.
//! Raw boost is [`CarState::boost`](rocketsim::CarState::boost)
//! (`0..100`; out-of-range input clamps, NaN maps to empty).
//!
//! ```rust
//! use rocketsim_viewer::hud::{boost_fraction, boost_label, is_boost_low};
//!
//! assert_eq!(boost_fraction(50.0), 0.5);
//! assert_eq!(boost_label(87.0), "87");
//! assert!(is_boost_low(10.0));
//! assert!(!is_boost_low(25.0));
//! ```

use rocketsim::{CarInfo, CarState};

/// Full tank (matches the sim max).
pub const MAX_BOOST: f32 = 100.0;
/// Warn when clamped boost is below this.
pub const LOW_BOOST_THRESHOLD: f32 = 25.0;

/// Clamp raw boost to `0..=MAX_BOOST`. NaN gives 0 (empty, warns).
pub fn clamp_boost(raw: f32) -> f32 {
    if raw.is_nan() || raw <= 0.0 {
        0.0
    } else if raw >= MAX_BOOST {
        MAX_BOOST
    } else {
        raw
    }
}

/// Boost as `0..1` for `egui::ProgressBar`. NaN gives 0.
pub fn boost_fraction(raw: f32) -> f32 {
    clamp_boost(raw) / MAX_BOOST
}

/// Boost as `0..100` (rounded). NaN gives 0.
pub fn boost_percent(raw: f32) -> u32 {
    (boost_fraction(raw) * MAX_BOOST).round() as u32
}

/// Short label: digits only, e.g. `"87"`. Viewer adds the `"Boost "` prefix.
pub fn boost_label(raw: f32) -> String {
    boost_percent(raw).to_string()
}

/// True when clamped boost is below [`LOW_BOOST_THRESHOLD`].
/// NaN counts as low (it maps to 0).
pub fn is_boost_low(raw: f32) -> bool {
    clamp_boost(raw) < LOW_BOOST_THRESHOLD
}

/// Raw boost for the watched car, or `None` when the meter must hide.
/// `None` means: no follow view, bad index, or demoed car.
pub fn spectated_boost(cars: &[(CarInfo, CarState)], spectated: Option<u32>) -> Option<f32> {
    let idx = spectated? as usize;
    let (_, state) = cars.get(idx)?;
    if state.is_demoed {
        return None;
    }
    Some(state.boost)
}

/// All meter values in one call (viewer draws from this).
#[derive(Debug, Clone, PartialEq)]
pub struct BoostMeter {
    /// `0..1` for the gauge arc.
    pub fraction: f32,
    /// `0..100` for text.
    pub percent: u32,
    /// Digits only, e.g. `"87"`.
    pub label: String,
    /// True below 25.
    pub low: bool,
}

/// Build a [`BoostMeter`] from raw boost.
pub fn boost_meter(raw: f32) -> BoostMeter {
    BoostMeter {
        fraction: boost_fraction(raw),
        percent: boost_percent(raw),
        label: boost_label(raw),
        low: is_boost_low(raw),
    }
}

/// Gauge sweep angle (rad) for the HUD boost arc: `0` empty, `TAU` full.
/// The viewer draws this clockwise from the top of the dial.
pub fn boost_sweep(raw: f32) -> f32 {
    boost_fraction(raw) * std::f32::consts::TAU
}

#[cfg(test)]
mod tests {
    use super::*;

    fn car(boost: f32, demoed: bool) -> (CarInfo, CarState) {
        let mut state = CarState::DEFAULT;
        state.boost = boost;
        state.is_demoed = demoed;
        (CarInfo::default(), state)
    }

    #[test]
    fn clamp_edges() {
        assert_eq!(clamp_boost(0.0), 0.0);
        assert_eq!(clamp_boost(50.0), 50.0);
        assert_eq!(clamp_boost(100.0), 100.0);
        // Out of range clamps.
        assert_eq!(clamp_boost(-5.0), 0.0);
        assert_eq!(clamp_boost(-0.0), 0.0);
        assert_eq!(clamp_boost(150.0), 100.0);
        assert_eq!(clamp_boost(f32::INFINITY), 100.0);
        assert_eq!(clamp_boost(f32::NEG_INFINITY), 0.0);
        // Bad input maps to empty.
        assert_eq!(clamp_boost(f32::NAN), 0.0);
    }

    #[test]
    fn fraction_edges() {
        assert_eq!(boost_fraction(0.0), 0.0);
        assert_eq!(boost_fraction(50.0), 0.5);
        assert_eq!(boost_fraction(100.0), 1.0);
        assert_eq!(boost_fraction(-5.0), 0.0);
        assert_eq!(boost_fraction(150.0), 1.0);
        assert_eq!(boost_fraction(f32::NAN), 0.0);
        assert_eq!(boost_fraction(f32::INFINITY), 1.0);
        assert_eq!(boost_fraction(f32::NEG_INFINITY), 0.0);
    }

    #[test]
    fn percent_and_label_edges() {
        assert_eq!(boost_percent(0.0), 0);
        assert_eq!(boost_percent(87.0), 87);
        assert_eq!(boost_percent(87.4), 87);
        assert_eq!(boost_percent(87.5), 88);
        assert_eq!(boost_percent(100.0), 100);
        assert_eq!(boost_percent(-5.0), 0);
        assert_eq!(boost_percent(150.0), 100);
        assert_eq!(boost_percent(f32::NAN), 0);
        assert_eq!(boost_percent(f32::INFINITY), 100);
        assert_eq!(boost_label(87.0), "87");
        assert_eq!(boost_label(100.0), "100");
        assert_eq!(boost_label(f32::NAN), "0");
        assert_eq!(boost_label(-3.0), "0");
        assert_eq!(boost_label(120.0), "100");
    }

    #[test]
    fn low_flag_edges() {
        assert!(is_boost_low(0.0));
        assert!(is_boost_low(10.0));
        assert!(is_boost_low(24.9));
        assert!(!is_boost_low(25.0));
        assert!(!is_boost_low(100.0));
        // Clamped first: over-full is not low, negative is low.
        assert!(!is_boost_low(150.0));
        assert!(is_boost_low(-5.0));
        // NaN maps to 0, so it warns.
        assert!(is_boost_low(f32::NAN));
    }

    #[test]
    fn spectated_lookup_hides_when_needed() {
        let cars = vec![car(87.0, false), car(10.0, true)];
        // Live car passes raw boost through.
        assert_eq!(spectated_boost(&cars, Some(0)), Some(87.0));
        // Demoed car hides the meter.
        assert_eq!(spectated_boost(&cars, Some(1)), None);
        // No follow view hides the meter.
        assert_eq!(spectated_boost(&cars, None), None);
        // Bad index hides the meter.
        assert_eq!(spectated_boost(&cars, Some(99)), None);
        assert_eq!(spectated_boost(&[], Some(0)), None);
    }

    #[test]
    fn meter_bundles_helpers() {
        let m = boost_meter(87.0);
        assert_eq!(m.fraction, boost_fraction(87.0));
        assert_eq!(m.percent, 87);
        assert_eq!(m.label, "87");
        assert!(!m.low);
        let low = boost_meter(f32::NAN);
        assert_eq!(low.fraction, 0.0);
        assert_eq!(low.percent, 0);
        assert_eq!(low.label, "0");
        assert!(low.low);
    }

    #[test]
    fn sweep_edges() {
        use std::f32::consts::TAU;
        assert_eq!(boost_sweep(0.0), 0.0);
        assert_eq!(boost_sweep(100.0), TAU);
        assert!((boost_sweep(50.0) - TAU / 2.0).abs() < 1e-6);
        assert_eq!(boost_sweep(f32::NAN), 0.0);
        assert_eq!(boost_sweep(150.0), TAU);
    }
}
