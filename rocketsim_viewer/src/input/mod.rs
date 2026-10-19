//! Device-independent actions, input evaluation, and capture.

mod actions;
mod capture;
mod evaluator;
pub mod legacy;
pub mod sources;
pub(crate) mod spectate;

pub use actions::{ACTIONS, Action, ActionMap};
pub use capture::Capture;
pub use evaluator::{ActionValue, Evaluator, InputSnapshot};
pub use sources::{Axis2, InputSource};

#[cfg(test)]
mod tests {
    use super::legacy::Bindings;
    use super::sources::{Key, MouseButton, PadAxis, PadButton};
    use super::*;

    fn all_sources() -> Vec<InputSource> {
        let mut sources: Vec<InputSource> =
            Key::ALL.iter().copied().map(InputSource::Key).collect();
        sources.extend(
            [
                MouseButton::Primary,
                MouseButton::Secondary,
                MouseButton::Middle,
                MouseButton::Extra1,
                MouseButton::Extra2,
            ]
            .into_iter()
            .map(InputSource::Mouse),
        );
        for axis in [Axis2::X, Axis2::Y] {
            for sign in [-1, 1] {
                sources.push(InputSource::MouseAxis { axis, sign });
                sources.push(InputSource::Wheel { axis, sign });
            }
        }
        for button in [
            PadButton::South,
            PadButton::East,
            PadButton::North,
            PadButton::West,
            PadButton::C,
            PadButton::Z,
            PadButton::LeftTrigger,
            PadButton::LeftTrigger2,
            PadButton::RightTrigger,
            PadButton::RightTrigger2,
            PadButton::Select,
            PadButton::Start,
            PadButton::Mode,
            PadButton::LeftThumb,
            PadButton::RightThumb,
            PadButton::DPadUp,
            PadButton::DPadDown,
            PadButton::DPadLeft,
            PadButton::DPadRight,
        ] {
            sources.push(InputSource::PadButton(button));
        }
        for axis in [
            PadAxis::LeftStickX,
            PadAxis::LeftStickY,
            PadAxis::LeftZ,
            PadAxis::RightStickX,
            PadAxis::RightStickY,
            PadAxis::RightZ,
            PadAxis::DPadX,
            PadAxis::DPadY,
        ] {
            for sign in [-1, 1] {
                sources.push(InputSource::PadAxis { axis, sign });
            }
        }
        for code in [0, 1, 42, 0x30000, 0x30001, u32::MAX] {
            for sign in [-1, 1] {
                sources.push(InputSource::PadRaw { code, sign });
            }
        }
        sources
    }

    #[test]
    fn action_maps_round_trip_multiple_sources_and_sparse_slots() {
        let cases = [
            (Action::Jump, 0, InputSource::Key(Key::Space)),
            (Action::Director, 1, InputSource::Mouse(MouseButton::Extra2)),
            (
                Action::MenuAccept,
                1,
                InputSource::PadAxis {
                    axis: PadAxis::LeftStickY,
                    sign: -1,
                },
            ),
            (
                Action::FreeDollyForward,
                0,
                InputSource::Wheel {
                    axis: Axis2::Y,
                    sign: 1,
                },
            ),
            (
                Action::Boost,
                1,
                InputSource::PadRaw {
                    code: 0x30000,
                    sign: 1,
                },
            ),
        ];
        let mut map = ActionMap::unbound();
        for (action, slot, source) in cases {
            map.set(action, slot, source);
        }
        let encoded = toml::to_string(&map).unwrap();
        let decoded: ActionMap = toml::from_str(&encoded).unwrap();
        assert_eq!(decoded, map);
        assert_eq!(
            decoded.sources(Action::MenuToggle),
            &[InputSource::Unbound; 2]
        );
        assert_eq!(decoded.sources(Action::Jump)[1], InputSource::Unbound);
        assert_eq!(
            toml::from_str::<ActionMap>("bindings = {}").unwrap(),
            ActionMap::unbound()
        );
    }

    #[test]
    fn action_map_revisions_track_distinct_versions() {
        let mut map = ActionMap::unbound();
        let source = InputSource::Key(Key::A);
        let initial_revision = map.revision();
        map.set(Action::Jump, 0, InputSource::Unbound);
        assert_eq!(map.revision(), initial_revision);
        map.set(Action::Jump, 0, source);
        assert_ne!(map.revision(), initial_revision);
        let bound_revision = map.revision();
        map.set(Action::Jump, 0, source);
        assert_eq!(map.revision(), bound_revision);

        let mut clone = map.clone();
        assert_eq!(clone.revision(), map.revision());
        clone.set(Action::Boost, 0, source);
        assert_ne!(clone.revision(), map.revision());

        let encoded = toml::to_string(&map).unwrap();
        let decoded: ActionMap = toml::from_str(&encoded).unwrap();
        assert_eq!(decoded, map);
        assert_ne!(decoded.revision(), map.revision());
    }

    #[test]
    fn evaluate_into_clears_and_reuses_the_output_map() {
        let source = InputSource::Key(Key::A);
        let mut map = ActionMap::unbound();
        map.set(Action::Jump, 0, source);
        let mut snapshot = InputSnapshot::new();
        snapshot.set_down(source, true);
        let mut evaluator = Evaluator::default();
        let mut values = std::collections::HashMap::from([(
            Action::MenuBack,
            ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        )]);

        evaluator.evaluate_into(&map, &snapshot, 0.0, &mut values);
        assert_eq!(values.len(), ACTIONS.len());
        assert!(values[&Action::Jump].just_pressed);
        assert!(!values[&Action::MenuBack].pressed);

        snapshot.set_down(source, false);
        evaluator.evaluate_into(&map, &snapshot, 0.0, &mut values);
        assert!(values[&Action::Jump].just_released);
        assert_eq!(values.len(), ACTIONS.len());
    }

    #[test]
    fn default_map_matches_legacy_migration() {
        assert_eq!(
            ActionMap::from_legacy(&Bindings::default()),
            ActionMap::default()
        );
        assert_eq!(
            ActionMap::default().sources(Action::Jump)[0],
            InputSource::Mouse(MouseButton::Primary)
        );
    }

    #[test]
    fn evaluator_edges_focus_and_rebinding() {
        let source = InputSource::Key(Key::A);
        let mut map = ActionMap::unbound();
        map.set(Action::Jump, 0, source);
        let mut snapshot = InputSnapshot::new();
        let mut evaluator = Evaluator::default();

        snapshot.set_down(source, true);
        assert!(evaluator.evaluate(&map, &snapshot, 0.0)[&Action::Jump].just_pressed);
        snapshot.focused = false;
        assert!(evaluator.evaluate(&map, &snapshot, 0.0)[&Action::Jump].just_released);
        snapshot.focused = true;
        assert!(!evaluator.evaluate(&map, &snapshot, 0.0)[&Action::Jump].pressed);
        snapshot.set_down(source, false);
        evaluator.evaluate(&map, &snapshot, 0.0);
        snapshot.set_down(source, true);
        assert!(evaluator.evaluate(&map, &snapshot, 0.0)[&Action::Jump].just_pressed);

        map.set(Action::Jump, 0, InputSource::Unbound);
        map.set(Action::Boost, 0, source);
        let value = evaluator.evaluate(&map, &snapshot, 0.0)[&Action::Boost];
        assert!(!value.just_pressed && !value.pressed);
        snapshot.set_down(source, false);
        evaluator.evaluate(&map, &snapshot, 0.0);
        snapshot.set_down(source, true);
        assert!(evaluator.evaluate(&map, &snapshot, 0.0)[&Action::Boost].just_pressed);
    }

    #[test]
    fn capture_accepts_every_concrete_source() {
        for source in all_sources() {
            let magnitude = match source {
                InputSource::PadAxis { .. } | InputSource::PadRaw { .. } => 0.75,
                InputSource::MouseAxis { .. } => 6.0,
                InputSource::Wheel { .. } => 1.0,
                _ => 0.75,
            };
            let mut snapshot = InputSnapshot::new();
            snapshot.set(source, magnitude);
            assert_eq!(
                Capture::begin(&InputSnapshot::new()).poll(&snapshot),
                Some(source)
            );
        }
    }

    #[test]
    fn capture_waits_for_release_and_filters_drift() {
        let held = InputSource::Mouse(MouseButton::Primary);
        let axis = InputSource::PadRaw { code: 42, sign: -1 };
        let escape = InputSource::Key(Key::Escape);
        let mut snapshot = InputSnapshot::new();
        snapshot.set_down(held, true);
        let mut capture = Capture::begin(&snapshot);
        assert_eq!(capture.poll(&snapshot), None);
        snapshot.set_down(held, false);
        snapshot.set(axis, 0.49);
        assert_eq!(capture.poll(&snapshot), None);
        snapshot.set_down(escape, true);
        assert_eq!(capture.poll(&snapshot), Some(escape));
    }

    #[test]
    fn capture_focus_loss_keeps_the_release_gate() {
        let held = InputSource::Mouse(MouseButton::Primary);
        let next = InputSource::Key(Key::X);
        let mut snapshot = InputSnapshot::new();
        snapshot.set_down(held, true);
        let mut capture = Capture::begin(&snapshot);
        snapshot.focused = false;
        assert_eq!(capture.poll(&snapshot), None);
        snapshot.focused = true;
        assert_eq!(capture.poll(&snapshot), None);
        snapshot.set_down(held, false);
        assert_eq!(capture.poll(&snapshot), None);
        snapshot.set_down(next, true);
        assert_eq!(capture.poll(&snapshot), Some(next));
    }

    #[test]
    fn capture_prefers_physical_modifiers_and_strongest_axis() {
        for (generic, physicals) in [
            (Key::Shift, [Key::ShiftLeft, Key::ShiftRight]),
            (Key::Ctrl, [Key::ControlLeft, Key::ControlRight]),
            (Key::Alt, [Key::AltLeft, Key::AltRight]),
        ] {
            for physical in physicals {
                let mut snapshot = InputSnapshot::new();
                snapshot.set_down(InputSource::Key(generic), true);
                snapshot.set_down(InputSource::Key(physical), true);
                assert_eq!(
                    Capture::begin(&InputSnapshot::new()).poll(&snapshot),
                    Some(InputSource::Key(physical))
                );
            }
        }
        let x = InputSource::PadAxis {
            axis: PadAxis::LeftStickX,
            sign: 1,
        };
        let y = InputSource::PadAxis {
            axis: PadAxis::LeftStickY,
            sign: 1,
        };
        let mut snapshot = InputSnapshot::new();
        snapshot.set(x, 0.7);
        snapshot.set(y, 0.95);
        assert_eq!(
            Capture::begin(&InputSnapshot::new()).poll(&snapshot),
            Some(y)
        );
        snapshot.set(x, 0.8);
        snapshot.set(y, 0.8);
        assert_eq!(
            Capture::begin(&InputSnapshot::new()).poll(&snapshot),
            Some(x)
        );
    }

    #[test]
    fn physical_modifier_bindings_are_independent() {
        for (left, right) in [
            (Key::ShiftLeft, Key::ShiftRight),
            (Key::ControlLeft, Key::ControlRight),
            (Key::AltLeft, Key::AltRight),
        ] {
            let mut map = ActionMap::unbound();
            map.set(Action::MenuAccept, 0, InputSource::Key(left));
            let mut snapshot = InputSnapshot::new();
            let mut evaluator = Evaluator::default();
            snapshot.set_down(InputSource::Key(right), true);
            assert!(!evaluator.evaluate(&map, &snapshot, 0.0)[&Action::MenuAccept].pressed);
            snapshot.set_down(InputSource::Key(right), false);
            snapshot.set_down(InputSource::Key(left), true);
            assert!(evaluator.evaluate(&map, &snapshot, 0.0)[&Action::MenuAccept].pressed);
        }
    }

    #[test]
    fn signed_sources_reject_invalid_signs() {
        for sign in [-2, 0, 2] {
            let text =
                format!("type = \"PadAxis\"\nvalue = {{ axis = \"LeftStickX\", sign = {sign} }}");
            assert!(toml::from_str::<InputSource>(&text).is_err());
        }
        for sign in [-1, 1] {
            let text =
                format!("type = \"PadAxis\"\nvalue = {{ axis = \"LeftStickX\", sign = {sign} }}");
            assert!(toml::from_str::<InputSource>(&text).is_ok());
        }
    }

    #[test]
    fn deadzone_is_pad_only_and_applies_per_source() {
        let key = InputSource::Key(Key::K);
        let axis = InputSource::PadAxis {
            axis: PadAxis::LeftStickX,
            sign: 1,
        };
        let mouse = InputSource::MouseAxis {
            axis: Axis2::X,
            sign: 1,
        };
        let mut map = ActionMap::unbound();
        map.set(Action::YawLeft, 0, key);
        map.set(Action::YawLeft, 1, axis);
        map.set(Action::LookRight, 0, mouse);
        let mut snapshot = InputSnapshot::new();
        snapshot.set(key, 0.6);
        snapshot.set(axis, 0.2);
        snapshot.set(mouse, 12.0);
        let mut evaluator = Evaluator::default();
        assert_eq!(
            evaluator.source_value(&snapshot, axis, 0.1),
            (0.2 - 0.1) / 0.9
        );
        assert_eq!(
            evaluator.evaluate(&map, &snapshot, 0.3)[&Action::YawLeft].value,
            0.6
        );
        assert_eq!(
            evaluator.evaluate(&map, &snapshot, 0.3)[&Action::LookRight].value,
            12.0
        );
        snapshot.focused = false;
        assert_eq!(evaluator.source_value(&snapshot, mouse, 0.0), 0.0);
    }

    #[test]
    fn legacy_pad_fields_migrate_to_controller_slots() {
        let mut legacy = Bindings::default();
        legacy.pad.camera.follow = [
            PadButton::South,
            PadButton::East,
            PadButton::North,
            PadButton::West,
            PadButton::C,
            PadButton::Z,
            PadButton::Start,
            PadButton::Mode,
        ];
        legacy.pad.camera.director = PadButton::LeftThumb;
        legacy.pad.camera.autocam = PadButton::RightThumb;
        legacy.pad.camera.free = PadButton::DPadUp;
        legacy.pad.camera.cycle = PadButton::DPadDown;
        legacy.pad.actions.kickoff_reset = PadButton::Select;
        legacy.pad.actions.dribble = PadButton::DPadLeft;
        legacy.pad.actions.launch = PadButton::DPadRight;
        let map = ActionMap::from_legacy(&legacy);
        for (index, action) in ACTIONS[16..24].iter().copied().enumerate() {
            assert_eq!(
                map.sources(action)[1],
                InputSource::PadButton(legacy.pad.camera.follow[index])
            );
        }
        assert_eq!(
            map.sources(Action::PitchForward)[0],
            InputSource::Key(legacy.car.throttle_fwd)
        );
        assert_eq!(map.sources(Action::AirRoll), map.sources(Action::Handbrake));
        for (action, button) in [
            (Action::Director, PadButton::LeftThumb),
            (Action::AutoCam, PadButton::RightThumb),
            (Action::FreeCam, PadButton::DPadUp),
            (Action::NextCam, PadButton::DPadDown),
            (Action::KickoffReset, PadButton::Select),
            (Action::Dribble, PadButton::DPadLeft),
            (Action::Launch, PadButton::DPadRight),
        ] {
            assert_eq!(map.sources(action)[1], InputSource::PadButton(button));
        }
    }
}
