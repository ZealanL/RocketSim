//! Boost meter values for the viewer HUD.

use rocketsim::{CarInfo, CarState};

/// Raw boost for the followed car. Hide the meter without a live car.
pub(crate) fn spectated_boost(cars: &[(CarInfo, CarState)], followed: Option<u32>) -> Option<f32> {
    let (_, state) = cars.get(followed? as usize)?;
    (!state.is_demoed).then_some(state.boost)
}

pub(crate) struct BoostMeter {
    pub(crate) fraction: f32,
    pub(crate) label: String,
    pub(crate) low: bool,
}

pub(crate) fn boost_meter(raw: f32) -> BoostMeter {
    let boost = if raw.is_nan() {
        0.0
    } else {
        raw.clamp(0.0, 100.0)
    };
    BoostMeter {
        fraction: boost / 100.0,
        label: (boost.round() as u32).to_string(),
        low: boost < 25.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn meter_clamps_invalid_values_and_rounds_only_the_label() {
        for (raw, fraction, label, low) in [
            (f32::NAN, 0.0, "0", true),
            (f32::NEG_INFINITY, 0.0, "0", true),
            (-5.0, 0.0, "0", true),
            (24.9, 0.249, "25", true),
            (25.0, 0.25, "25", false),
            (87.4, 0.874, "87", false),
            (87.5, 0.875, "88", false),
            (150.0, 1.0, "100", false),
            (f32::INFINITY, 1.0, "100", false),
        ] {
            let meter = boost_meter(raw);
            assert!((meter.fraction - fraction).abs() < 1e-6, "{raw}");
            assert_eq!((meter.label.as_str(), meter.low), (label, low), "{raw}");
        }
    }

    #[test]
    fn meter_is_hidden_without_a_live_followed_car() {
        let mut state = CarState::DEFAULT;
        state.boost = 87.0;
        let mut cars = vec![(CarInfo::default(), state)];
        assert_eq!(spectated_boost(&cars, Some(0)), Some(87.0));
        assert_eq!(spectated_boost(&cars, None), None);
        assert_eq!(spectated_boost(&cars, Some(1)), None);
        cars[0].1.is_demoed = true;
        assert_eq!(spectated_boost(&cars, Some(0)), None);
        assert_eq!(spectated_boost(&[], Some(0)), None);
    }
}
