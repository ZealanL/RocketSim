//! Spectate-only input gate (pure, no IO).
//!
//! [`gate`] sits between the per-frame input build in `rocketsim-viewer.rs` and
//! the control-channel send: spectate-only mode (or a non-driving frame)
//! forces neutral inputs plus zeroed buttons, so held keys and gamepad
//! blends never reach the sim. Frames still send every tick, so the channel
//! stays live; `driven_car` tracking is display only and untouched.

use rocketsim::CarControls;

/// Gate driver inputs for one frame.
///
/// Returns neutral plus zeroed buttons when `spectate_only` is set or the
/// frame is not driving (no follow view, or window unfocused); otherwise
/// passes `controls`/`buttons` through unchanged. `CarControls` is `Copy`,
/// neutral is [`CarControls::default`].
pub fn gate(
    spectate_only: bool,
    driving: bool,
    controls: CarControls,
    buttons: u8,
) -> (CarControls, u8) {
    if spectate_only || !driving {
        (CarControls::default(), 0)
    } else {
        (controls, buttons)
    }
}

/// One-line overlay `Input:` text for spectate vs driving states.
///
/// Spectate-only always reports spectating (even when the frame would
/// otherwise drive); driving reports the live car; anything else reports
/// the follow-a-car hint. The caller picks the unfocused / free-camera /
/// no-sim lines; this helper covers only the spectate-vs-drive wording.
pub fn status_text(spectate_only: bool, driving: bool, driven_car: u32) -> String {
    if spectate_only {
        format!("Input: spectating car {driven_car} — inputs off")
    } else if driving {
        format!("Input: driving car {driven_car} · live")
    } else {
        "Input: watching — follow a car (1-8) to drive it".to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Full keyboard + gamepad blend: axes off neutral, level inputs set,
    /// every button bit set.
    fn hot_input() -> (CarControls, u8) {
        let controls = CarControls {
            throttle: 1.0,
            steer: -0.5,
            pitch: 0.25,
            yaw: -0.5,
            roll: 1.0,
            jump: true,
            boost: true,
            handbrake: true,
        };
        (controls, 0b111)
    }

    #[test]
    fn driving_passes_through() {
        let (controls, buttons) = hot_input();
        let (gated, gated_buttons) = gate(false, true, controls, buttons);
        assert_eq!(gated, controls);
        assert_eq!(gated_buttons, buttons);
    }

    #[test]
    fn not_driving_sends_neutral() {
        let (controls, _) = hot_input();
        let (gated, gated_buttons) = gate(false, false, controls, 0b111);
        assert_eq!(gated, CarControls::default());
        assert_eq!(gated_buttons, 0);
    }

    #[test]
    fn spectate_only_beats_driving() {
        // Follow view + focused + keys held still sends neutral + zero.
        let (controls, _) = hot_input();
        let (gated, gated_buttons) = gate(true, true, controls, 0b111);
        assert_eq!(gated, CarControls::default());
        assert_eq!(gated_buttons, 0);
    }

    #[test]
    fn spectate_only_without_driving_stays_neutral() {
        let (controls, _) = hot_input();
        let (gated, gated_buttons) = gate(true, false, controls, 0b111);
        assert_eq!(gated, CarControls::default());
        assert_eq!(gated_buttons, 0);
    }

    #[test]
    fn blended_gamepad_input_still_gated() {
        // Gamepad blend lands on top of viewer keys; spectate-only must
        // zero the blend too (analog + level + wire buttons).
        let blended = CarControls {
            throttle: 0.8,
            steer: 0.6,
            pitch: -0.3,
            yaw: 0.6,
            roll: -1.0,
            jump: true,
            boost: true,
            handbrake: false,
        };
        assert_ne!(blended, CarControls::default());
        let (gated, gated_buttons) = gate(true, true, blended, 0b101);
        assert_eq!(gated, CarControls::default());
        assert_eq!(gated_buttons, 0);
    }

    #[test]
    fn status_text_wording() {
        let spectating = status_text(true, true, 2);
        assert!(spectating.contains("spectating car 2"), "{spectating}");
        assert!(spectating.contains("off"), "{spectating}");

        let driving = status_text(false, true, 2);
        assert!(driving.contains("driving car 2"), "{driving}");

        let watching = status_text(false, false, 0);
        assert!(watching.contains("watching"), "{watching}");
        assert_ne!(spectating, driving);
    }
}
