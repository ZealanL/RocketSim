//! Spectate-only input gate (pure, no IO).
//!
//! [`gate`] clears inputs when spectate-only mode is set or driving is blocked.
//! Spectate-only messages use an invalid car index. Simulations must ignore
//! that index instead of replacing policy controls with neutral controls.
//! Messages still send every tick. Camera selection does not change.

use rocketsim::CarControls;

/// Gate driver inputs for one frame.
///
/// Spectate-only mode returns `u32::MAX` as the car index and clears inputs.
/// A blocked driving frame keeps the car index but clears inputs. This
/// releases held controls when the window loses focus or a menu opens.
pub fn gate(
    spectate_only: bool,
    driving: bool,
    car_index: u32,
    controls: CarControls,
    buttons: u8,
) -> (u32, CarControls, u8) {
    if spectate_only {
        (u32::MAX, CarControls::default(), 0)
    } else if !driving {
        (car_index, CarControls::default(), 0)
    } else {
        (car_index, controls, buttons)
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
    fn input_gate_releases_manual_controls_without_overriding_spectated_bots() {
        use rocketsim_vis::proto::{ControlMsg, decode_control, encode_control};

        let (controls, buttons) = hot_input();
        for (spectate_only, driving, input, expected_index, expected_controls, expected_buttons) in [
            (false, true, controls, 1, controls, buttons),
            (
                false,
                true,
                CarControls::default(),
                1,
                CarControls::default(),
                buttons,
            ),
            (false, false, controls, 1, CarControls::default(), 0),
            (true, true, controls, u32::MAX, CarControls::default(), 0),
            (true, false, controls, u32::MAX, CarControls::default(), 0),
        ] {
            let (index, gated_controls, gated_buttons) =
                gate(spectate_only, driving, 1, input, buttons);
            let bytes = encode_control(&ControlMsg::new(index, gated_controls, gated_buttons, 1));
            let (message, _) = decode_control(&bytes).unwrap().unwrap();
            assert_eq!(message.car_index, expected_index);
            assert_eq!(message.controls, expected_controls);
            assert_eq!(message.buttons, expected_buttons);

            // Apply the message with the same bounds check as a policy client.
            let mut policy_controls = [controls; 2];
            if let Some(car) = policy_controls.get_mut(message.car_index as usize) {
                *car = message.controls;
            }
            assert_eq!(policy_controls[0], controls);
            assert_eq!(
                policy_controls[1],
                if spectate_only {
                    controls
                } else {
                    expected_controls
                }
            );
        }
    }
}
