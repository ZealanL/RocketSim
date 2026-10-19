use std::collections::HashMap;

use crate::{
    input::sources::{Key, MouseButton, PadAxis, PadButton, key_down},
    input::{Action, ActionMap, ActionValue, Axis2, Evaluator, InputSnapshot, InputSource},
};
use rocketsim::CarControls;

pub(super) struct InputState {
    pub(super) evaluator: Evaluator,
    pub(super) last_focused: bool,
    pub(super) cursor_grabbed: bool,
    pub(super) action_values: HashMap<Action, ActionValue>,
    pub(super) gilrs: Option<gilrs::Gilrs>,
    pub(super) pad_buttons: HashMap<PadButton, f32>,
    pub(super) raw_pad: HashMap<u32, f32>,
    pub(super) active_pad: Option<gilrs::GamepadId>,
    pub(super) pad_selection_manual: bool,
    pub(super) suppress_pad_on_next_frame: bool,
}

impl InputState {
    pub(super) fn new() -> Self {
        Self {
            evaluator: Evaluator::default(),
            last_focused: false,
            cursor_grabbed: false,
            action_values: HashMap::new(),
            gilrs: gilrs::Gilrs::new().ok(),
            pad_buttons: HashMap::new(),
            raw_pad: HashMap::new(),
            active_pad: None,
            pad_selection_manual: false,
            suppress_pad_on_next_frame: false,
        }
    }

    pub(super) fn poll_gamepads(&mut self) -> PadFramePulses {
        let mut events = Vec::new();
        let mut device_changed = false;
        if let Some(gilrs) = self.gilrs.as_mut() {
            while let Some(event) = gilrs.next_event() {
                events.push(event);
            }
            let previous = self.active_pad;
            let connected_event = events.iter().find_map(|event| {
                matches!(event.event, gilrs::EventType::Connected).then_some(event.id)
            });
            let disconnected_event = events.iter().find_map(|event| {
                matches!(event.event, gilrs::EventType::Disconnected).then_some(event.id)
            });
            let connected: Vec<_> = gilrs
                .gamepads()
                .filter(|(_, pad)| pad.is_connected())
                .map(|(id, _)| id)
                .collect();
            let stable_fallback = super::session::choose_active_pad(
                self.active_pad,
                connected_event,
                disconnected_event,
                &connected,
            );
            let mut ranked: Vec<_> = gilrs
                .gamepads()
                .filter(|(_, pad)| pad.is_connected())
                .map(|(id, pad)| (pad_priority(pad), id))
                .collect();
            ranked.sort_by_key(|(priority, _)| *priority);
            let best = ranked.last().map(|(_, id)| *id).or(stable_fallback);
            let current_connected = self
                .active_pad
                .is_some_and(|id| gilrs.gamepad(id).is_connected());
            let current_priority = self.active_pad.map(|id| pad_priority(gilrs.gamepad(id)));
            if !current_connected {
                self.active_pad = best;
                self.pad_selection_manual = false;
            } else if !self.pad_selection_manual
                && let (Some(best), Some(best_priority), Some(current_priority)) = (
                    best,
                    ranked.last().map(|(priority, _)| *priority),
                    current_priority,
                )
                && best_priority > current_priority
            {
                self.active_pad = Some(best);
            }
            device_changed = previous != self.active_pad;
            if device_changed {
                self.raw_pad.clear();
                self.pad_buttons.clear();
                self.suppress_pad_on_next_frame = true;
            }
        }
        let suppress_pulses = device_changed || self.suppress_pad_on_next_frame;
        let mut pulses = PadFramePulses::default();
        for event in &events {
            if Some(event.id) != self.active_pad {
                continue;
            }
            match event.event {
                gilrs::EventType::ButtonPressed(button, code) => {
                    update_pad_buttons(&mut self.pad_buttons, event.event);
                    update_raw_pad(&mut self.raw_pad, event.event);
                    if !suppress_pulses {
                        pulses.record_press(button, code.into_u32());
                    }
                }
                gilrs::EventType::ButtonReleased(_, _)
                | gilrs::EventType::ButtonRepeated(_, _)
                | gilrs::EventType::ButtonChanged(_, _, _)
                | gilrs::EventType::AxisChanged(_, _, _) => {
                    update_pad_buttons(&mut self.pad_buttons, event.event);
                    update_raw_pad(&mut self.raw_pad, event.event);
                }
                _ => {}
            }
        }
        pulses
    }

    pub(super) fn suppress_until_release(&mut self, snapshot: &InputSnapshot) {
        self.evaluator.suppress_until_release(snapshot);
    }

    pub(super) fn update_focus(&mut self, focused: bool, snapshot: &InputSnapshot) {
        if focused && !self.last_focused {
            self.suppress_until_release(snapshot);
        }
        self.last_focused = focused;
    }

    pub(super) fn snapshot(
        &self,
        ctx: &egui::Context,
        pulses: &PadFramePulses,
        focused: bool,
    ) -> InputSnapshot {
        action_input_snapshot(
            ctx,
            self.gilrs.as_ref(),
            self.active_pad,
            &self.raw_pad,
            pulses,
            &self.pad_buttons,
            focused,
        )
    }
}

pub(super) fn to_gilrs_axis(a: PadAxis) -> gilrs::Axis {
    use gilrs::Axis as G;
    match a {
        PadAxis::LeftStickX => G::LeftStickX,
        PadAxis::LeftStickY => G::LeftStickY,
        PadAxis::LeftZ => G::LeftZ,
        PadAxis::RightStickX => G::RightStickX,
        PadAxis::RightStickY => G::RightStickY,
        PadAxis::RightZ => G::RightZ,
        PadAxis::DPadX => G::DPadX,
        PadAxis::DPadY => G::DPadY,
        PadAxis::Unknown => G::Unknown,
    }
}

pub(super) fn controller_priority(
    connected: bool,
    stick_axes: usize,
    face_buttons: usize,
    mapping: gilrs::MappingSource,
) -> i32 {
    if !connected {
        return i32::MIN;
    }
    let mapping_bonus = match mapping {
        gilrs::MappingSource::SdlMappings => 20,
        gilrs::MappingSource::Driver => 10,
        gilrs::MappingSource::None => 0,
    };
    stick_axes as i32 * 20 + face_buttons as i32 * 5 + mapping_bonus
}

pub(super) fn pad_priority(pad: gilrs::Gamepad<'_>) -> i32 {
    let stick_axes = [
        gilrs::Axis::LeftStickX,
        gilrs::Axis::LeftStickY,
        gilrs::Axis::RightStickX,
        gilrs::Axis::RightStickY,
    ]
    .into_iter()
    .filter(|axis| pad.axis_data(*axis).is_some())
    .count();
    let face_buttons = [
        gilrs::Button::South,
        gilrs::Button::East,
        gilrs::Button::North,
        gilrs::Button::West,
    ]
    .into_iter()
    .filter(|button| pad.button_data(*button).is_some())
    .count();
    controller_priority(
        pad.is_connected(),
        stick_axes,
        face_buttons,
        pad.mapping_source(),
    )
}

pub(super) fn can_drive(spectated: bool, focused: bool, rebinding: bool, menu_open: bool) -> bool {
    spectated && focused && !rebinding && !menu_open
}

#[allow(clippy::too_many_arguments)]
pub(super) fn look_delta(
    map: &ActionMap,
    evaluator: &Evaluator,
    snapshot: &InputSnapshot,
    negative: Action,
    positive: Action,
    dt: f32,
    mouse_sensitivity: f32,
    deadzone: f32,
    mouse_busy: bool,
) -> f32 {
    let delta_for = |action| {
        map.sources(action)
            .iter()
            .copied()
            .filter(|source| {
                !mouse_busy
                    || !matches!(
                        source,
                        InputSource::MouseAxis { .. } | InputSource::Wheel { .. }
                    )
            })
            .fold(0.0, |sum, source| {
                let value = evaluator.source_value(snapshot, source, deadzone).max(0.0);
                sum + match source {
                    InputSource::MouseAxis { .. } => value * mouse_sensitivity,
                    InputSource::Wheel { .. } => value * mouse_sensitivity * 8.0,
                    _ => value.min(1.0) * 2.5 * dt,
                }
            })
    };
    delta_for(negative) - delta_for(positive)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn freecam_dolly_delta(
    map: &ActionMap,
    evaluator: &Evaluator,
    snapshot: &InputSnapshot,
    dt: f32,
    wheel_scale: f32,
    speed: f32,
    deadzone: f32,
    mouse_busy: bool,
) -> f32 {
    let action_delta = |action| {
        map.sources(action)
            .iter()
            .copied()
            .filter(|source| !mouse_busy || !matches!(source, InputSource::Wheel { .. }))
            .map(|source| {
                let value = evaluator.source_value(snapshot, source, deadzone).max(0.0);
                if matches!(source, InputSource::Wheel { .. }) {
                    value * wheel_scale
                } else {
                    value.min(1.0) * speed * dt
                }
            })
            .sum::<f32>()
    };
    action_delta(Action::FreeDollyForward) - action_delta(Action::FreeDollyBack)
}

pub(super) fn mouse_motion_delta(events: &[egui::Event], pointer_delta: egui::Vec2) -> egui::Vec2 {
    let mut saw_raw_motion = false;
    let raw_delta = events
        .iter()
        .filter_map(|event| match event {
            egui::Event::MouseMoved(delta) => {
                saw_raw_motion = true;
                Some(*delta)
            }
            _ => None,
        })
        .fold(egui::Vec2::ZERO, |sum, delta| sum + delta);
    if events
        .iter()
        .any(|event| matches!(event, egui::Event::PointerMoved(_)))
    {
        pointer_delta
    } else if saw_raw_motion {
        raw_delta
    } else {
        pointer_delta
    }
}

#[derive(Debug, Default)]
pub(super) struct PadFramePulses {
    pub(super) buttons: HashMap<PadButton, f32>,
    pub(super) raw: HashMap<u32, f32>,
}

impl PadFramePulses {
    pub(super) fn record_press(&mut self, button: gilrs::Button, code: u32) {
        self.raw
            .entry(code)
            .and_modify(|value| *value = value.max(1.0))
            .or_insert(1.0);
        let button = from_gilrs_button(button);
        if button != PadButton::Unknown {
            self.buttons
                .entry(button)
                .and_modify(|value| *value = value.max(1.0))
                .or_insert(1.0);
        }
    }
}

pub(super) fn update_raw_button_value(raw: &mut HashMap<u32, f32>, code: u32, value: f32) {
    raw.insert(code, value);
}

pub(super) fn pad_frame_value(current: f32, pulse: f32) -> f32 {
    current.max(pulse)
}

pub(super) fn from_gilrs_button(button: gilrs::Button) -> PadButton {
    use gilrs::Button as G;
    match button {
        G::South => PadButton::South,
        G::East => PadButton::East,
        G::North => PadButton::North,
        G::West => PadButton::West,
        G::C => PadButton::C,
        G::Z => PadButton::Z,
        G::LeftTrigger => PadButton::LeftTrigger,
        G::LeftTrigger2 => PadButton::LeftTrigger2,
        G::RightTrigger => PadButton::RightTrigger,
        G::RightTrigger2 => PadButton::RightTrigger2,
        G::Select => PadButton::Select,
        G::Start => PadButton::Start,
        G::Mode => PadButton::Mode,
        G::LeftThumb => PadButton::LeftThumb,
        G::RightThumb => PadButton::RightThumb,
        G::DPadUp => PadButton::DPadUp,
        G::DPadDown => PadButton::DPadDown,
        G::DPadLeft => PadButton::DPadLeft,
        G::DPadRight => PadButton::DPadRight,
        G::Unknown => PadButton::Unknown,
    }
}

pub(super) fn update_pad_button_state(
    buttons: &mut HashMap<PadButton, f32>,
    button: gilrs::Button,
    value: f32,
) {
    let button = from_gilrs_button(button);
    if button != PadButton::Unknown {
        buttons.insert(button, value);
    }
}

pub(super) fn update_pad_buttons(buttons: &mut HashMap<PadButton, f32>, event: gilrs::EventType) {
    match event {
        gilrs::EventType::ButtonPressed(button, _) => {
            update_pad_button_state(buttons, button, 1.0);
        }
        gilrs::EventType::ButtonReleased(button, _) => {
            update_pad_button_state(buttons, button, 0.0);
        }
        gilrs::EventType::ButtonChanged(button, value, _) => {
            update_pad_button_state(buttons, button, value);
        }
        _ => {}
    }
}

pub(super) fn update_raw_pad(raw: &mut HashMap<u32, f32>, event: gilrs::EventType) {
    match event {
        gilrs::EventType::ButtonPressed(_, code) | gilrs::EventType::ButtonRepeated(_, code) => {
            update_raw_button_value(raw, code.into_u32(), 1.0);
        }
        gilrs::EventType::ButtonReleased(_, code) => {
            update_raw_button_value(raw, code.into_u32(), 0.0);
        }
        gilrs::EventType::ButtonChanged(_, value, code)
        | gilrs::EventType::AxisChanged(_, value, code) => {
            update_raw_button_value(raw, code.into_u32(), value);
        }
        _ => {}
    }
}

pub(super) fn action_input_snapshot(
    ctx: &egui::Context,
    gilrs: Option<&gilrs::Gilrs>,
    active_pad: Option<gilrs::GamepadId>,
    raw: &HashMap<u32, f32>,
    pulses: &PadFramePulses,
    pad_buttons: &HashMap<PadButton, f32>,
    focused: bool,
) -> InputSnapshot {
    let mut snapshot = InputSnapshot::new();
    snapshot.focused = focused;
    ctx.input(|i| {
        for key in Key::ALL {
            let source = InputSource::Key(*key);
            snapshot.set_down(
                source,
                key_down(i, *key) || key.to_egui().is_some_and(|k| i.key_pressed(k)),
            );
        }
        for (button, pointer) in [
            (MouseButton::Primary, egui::PointerButton::Primary),
            (MouseButton::Secondary, egui::PointerButton::Secondary),
            (MouseButton::Middle, egui::PointerButton::Middle),
            (MouseButton::Extra1, egui::PointerButton::Extra1),
            (MouseButton::Extra2, egui::PointerButton::Extra2),
        ] {
            snapshot.set_down(
                InputSource::Mouse(button),
                i.pointer.button_down(pointer) || i.pointer.button_pressed(pointer),
            );
        }
        let pointer_delta = i.pointer.delta();
        let delta = mouse_motion_delta(&i.events, pointer_delta);

        for axis in [Axis2::X, Axis2::Y] {
            let movement = if axis == Axis2::X { delta.x } else { delta.y };
            let wheel = if axis == Axis2::X {
                i.smooth_scroll_delta.x
            } else {
                i.smooth_scroll_delta.y
            };
            for sign in [-1, 1] {
                snapshot.set(
                    InputSource::MouseAxis { axis, sign },
                    (movement * sign as f32).max(0.0),
                );
                snapshot.set(
                    InputSource::Wheel { axis, sign },
                    (wheel * sign as f32).max(0.0),
                );
            }
        }
    });
    if let (Some(gilrs), Some(id)) = (gilrs, active_pad) {
        let pad = gilrs.gamepad(id);
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
            let value = pad_buttons.get(&button).copied().unwrap_or(0.0);
            let value = pad_frame_value(value, pulses.buttons.get(&button).copied().unwrap_or(0.0));
            snapshot.set(InputSource::PadButton(button), value);
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
            let value = pad
                .axis_data(to_gilrs_axis(axis))
                .map_or(0.0, |data| data.value());
            for sign in [-1, 1] {
                snapshot.set(
                    InputSource::PadAxis { axis, sign },
                    (value * sign as f32).max(0.0),
                );
            }
        }
    }
    for (&code, &value) in raw {
        let value = pad_frame_value(value, pulses.raw.get(&code).copied().unwrap_or(0.0));
        for sign in [-1, 1] {
            snapshot.set(
                InputSource::PadRaw { code, sign },
                (value * sign as f32).max(0.0),
            );
        }
    }
    for (&code, &value) in &pulses.raw {
        if !raw.contains_key(&code) {
            for sign in [-1, 1] {
                snapshot.set(
                    InputSource::PadRaw { code, sign },
                    (value * sign as f32).max(0.0),
                );
            }
        }
    }
    snapshot
}

#[allow(clippy::too_many_arguments)]
pub(super) fn action_controls(
    values: &HashMap<Action, crate::input::ActionValue>,
    map: &ActionMap,
    evaluator: &Evaluator,
    snapshot: &InputSnapshot,
    deadzone: f32,
    enabled: bool,
    pointer_busy: bool,
    reset_requested: bool,
) -> (CarControls, u8) {
    let value = |a| values.get(&a).map_or(0.0, |v| v.value);
    let pressed = |a| values.get(&a).is_some_and(|v| v.pressed);
    let just_pressed = |a| values.get(&a).is_some_and(|v| v.just_pressed);
    let mut controls = CarControls::default();
    let mut buttons = 0;
    if enabled {
        controls.throttle =
            (value(Action::ThrottleForward) - value(Action::ThrottleBack)).clamp(-1.0, 1.0);
        controls.steer = (value(Action::SteerLeft) - value(Action::SteerRight)).clamp(-1.0, 1.0);
        controls.pitch = (value(Action::PitchBack) - value(Action::PitchForward)).clamp(-1.0, 1.0);
        controls.yaw = (value(Action::YawLeft) - value(Action::YawRight)).clamp(-1.0, 1.0);
        controls.roll = (value(Action::RollLeft) - value(Action::RollRight)).clamp(-1.0, 1.0);
        controls.handbrake = pressed(Action::Handbrake);
        let non_mouse_pressed = |action| {
            map.sources(action).iter().copied().any(|source| {
                !matches!(
                    source,
                    InputSource::Mouse(_)
                        | InputSource::MouseAxis { .. }
                        | InputSource::Wheel { .. }
                ) && evaluator.source_value(snapshot, source, deadzone) >= 0.5
            })
        };
        controls.jump = pressed(Action::Jump) && (!pointer_busy || non_mouse_pressed(Action::Jump));
        controls.boost =
            pressed(Action::Boost) && (!pointer_busy || non_mouse_pressed(Action::Boost));
        if pressed(Action::AirRoll) {
            controls.roll += controls.yaw;
            controls.yaw = 0.0;
        }
        if just_pressed(Action::KickoffReset) || reset_requested {
            buttons |= 1;
        }
        if just_pressed(Action::Dribble) {
            buttons |= 2;
        }
        if just_pressed(Action::Launch) {
            buttons |= 4;
        }
    }
    (controls, buttons)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::Capture;

    #[test]
    fn focus_and_rebinding_wait_for_held_input_to_release() {
        let mut map = ActionMap::default();
        let source = InputSource::Key(Key::Q);
        map.set(Action::MenuToggle, 0, source);
        map.set(Action::MenuToggle, 1, InputSource::Unbound);
        let mut input = InputState::new();
        let mut snapshot = InputSnapshot::new();
        snapshot.focused = true;
        snapshot.set_down(source, true);
        assert!(input.evaluator.evaluate(&map, &snapshot, 0.1)[&Action::MenuToggle].just_pressed);

        snapshot.focused = false;
        input.evaluator.evaluate(&map, &snapshot, 0.1);
        snapshot.focused = true;
        input.suppress_until_release(&snapshot);
        assert!(!input.evaluator.evaluate(&map, &snapshot, 0.1)[&Action::MenuToggle].pressed);

        let mut capture = Capture::begin(&snapshot);
        assert_eq!(capture.poll(&snapshot), None);
        snapshot.set_down(source, false);
        assert_eq!(capture.poll(&snapshot), None);
        snapshot.set_down(source, true);
        assert_eq!(capture.poll(&snapshot), Some(source));

        input.evaluator.evaluate(&map, &snapshot, 0.1);
        snapshot.set_down(source, false);
        input.evaluator.evaluate(&map, &snapshot, 0.1);
        snapshot.set_down(source, true);
        assert!(input.evaluator.evaluate(&map, &snapshot, 0.1)[&Action::MenuToggle].just_pressed);
    }

    #[test]
    fn mapped_stick_and_facepad_ranks_above_a_system_control_device() {
        let system_control = controller_priority(true, 0, 1, gilrs::MappingSource::Driver);
        let xbox = controller_priority(true, 4, 4, gilrs::MappingSource::SdlMappings);
        assert!(xbox > system_control);
        assert_eq!(
            controller_priority(false, 4, 4, gilrs::MappingSource::SdlMappings),
            i32::MIN
        );
    }

    #[test]
    fn mapped_dpad_buttons_keep_independent_event_state_and_release() {
        let mut buttons = HashMap::new();
        update_pad_button_state(&mut buttons, gilrs::Button::DPadUp, 1.0);
        assert_eq!(buttons.get(&PadButton::DPadUp), Some(&1.0));
        assert_eq!(buttons.get(&PadButton::DPadDown), None);

        update_pad_button_state(&mut buttons, gilrs::Button::DPadUp, 0.0);
        assert_eq!(buttons.get(&PadButton::DPadUp), Some(&0.0));
        assert_eq!(buttons.get(&PadButton::DPadDown), None);
        update_pad_button_state(&mut buttons, gilrs::Button::LeftTrigger, 0.37);
        assert_eq!(buttons.get(&PadButton::LeftTrigger), Some(&0.37));
        update_pad_button_state(&mut buttons, gilrs::Button::DPadUp, 0.0);
        assert_eq!(buttons.get(&PadButton::DPadUp), Some(&0.0));
        assert_eq!(buttons.get(&PadButton::LeftTrigger), Some(&0.37));
    }

    #[test]
    fn pad_press_release_pulses_survive_one_frame_for_mapped_and_raw_inputs() {
        let code = 196_608;
        let mut pulses = PadFramePulses::default();
        let mut raw = HashMap::new();
        let mut buttons = HashMap::new();
        update_pad_button_state(&mut buttons, gilrs::Button::South, 1.0);
        pulses.record_press(gilrs::Button::South, code);
        update_pad_button_state(&mut buttons, gilrs::Button::South, 0.0);
        update_raw_button_value(&mut raw, code, 0.0);

        assert_eq!(buttons.get(&PadButton::South), Some(&0.0));
        assert_eq!(
            pad_frame_value(
                buttons.get(&PadButton::South).copied().unwrap_or(0.0),
                pulses.buttons[&PadButton::South]
            ),
            1.0
        );
        assert_eq!(pad_frame_value(raw[&code], pulses.raw[&code]), 1.0);

        let next_frame = PadFramePulses::default();
        assert_eq!(
            pad_frame_value(
                raw[&code],
                next_frame.raw.get(&code).copied().unwrap_or(0.0)
            ),
            0.0
        );
        assert!(next_frame.buttons.is_empty());

        let mut unknown_pulses = PadFramePulses::default();
        unknown_pulses.record_press(gilrs::Button::Unknown, code);
        assert!(!unknown_pulses.buttons.contains_key(&PadButton::Unknown));
        assert_eq!(unknown_pulses.raw[&code], 1.0);
        update_raw_button_value(&mut raw, code, 0.3);
        assert_eq!(pad_frame_value(raw[&code], 0.0), 0.3);
    }

    #[test]
    fn action_controls_preserve_analog_values_and_gate_all_side_effects() {
        let mut values = HashMap::new();
        values.insert(
            Action::ThrottleForward,
            crate::input::ActionValue {
                value: 0.4,
                pressed: false,
                just_pressed: false,
                just_released: false,
            },
        );
        values.insert(
            Action::SteerLeft,
            crate::input::ActionValue {
                value: 0.7,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        values.insert(
            Action::Launch,
            crate::input::ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        let map = ActionMap::default();
        let evaluator = Evaluator::default();
        let snapshot = InputSnapshot::new();
        let (controls, buttons) = action_controls(
            &values, &map, &evaluator, &snapshot, 0.1, true, false, false,
        );
        assert!((controls.throttle - 0.4).abs() < 1e-6);
        assert!((controls.steer - 0.7).abs() < 1e-6);
        assert_eq!(buttons, 0b100);
        let (neutral, no_buttons) = action_controls(
            &values, &map, &evaluator, &snapshot, 0.1, false, false, false,
        );
        assert_eq!(neutral, CarControls::default());
        assert_eq!(no_buttons, 0);

        let value = |value, pressed| crate::input::ActionValue {
            value,
            pressed,
            just_pressed: false,
            just_released: false,
        };
        values.clear();
        values.insert(Action::YawLeft, value(0.4, true));
        values.insert(Action::RollLeft, value(0.3, true));
        values.insert(Action::AirRoll, value(1.0, true));
        let (air_roll, _) = action_controls(
            &values, &map, &evaluator, &snapshot, 0.1, true, false, false,
        );
        assert_eq!(air_roll.yaw, 0.0);
        assert!((air_roll.roll - 0.7).abs() < 1e-6);
        assert!(!air_roll.handbrake);

        for (spectated, focused, rebinding, menu_open) in [
            (false, true, false, false),
            (true, false, false, false),
            (true, true, true, false),
            (true, true, false, true),
        ] {
            assert!(!can_drive(spectated, focused, rebinding, menu_open));
        }
        assert!(can_drive(true, true, false, false));
    }

    #[test]
    fn freecam_look_signs_and_mouse_motion_ignore_frame_time() {
        let mut map = ActionMap::default();
        for action in [
            Action::LookLeft,
            Action::LookRight,
            Action::LookUp,
            Action::LookDown,
        ] {
            map.set(action, 0, InputSource::Unbound);
            map.set(action, 1, InputSource::Unbound);
        }
        map.set(
            Action::LookLeft,
            0,
            InputSource::MouseAxis {
                axis: Axis2::X,
                sign: 1,
            },
        );
        map.set(
            Action::LookDown,
            0,
            InputSource::MouseAxis {
                axis: Axis2::Y,
                sign: 1,
            },
        );
        let mut snapshot = InputSnapshot::new();
        snapshot.focused = true;
        snapshot.set(
            InputSource::MouseAxis {
                axis: Axis2::X,
                sign: 1,
            },
            10.0,
        );
        snapshot.set(
            InputSource::MouseAxis {
                axis: Axis2::Y,
                sign: 1,
            },
            10.0,
        );
        let mut evaluator = Evaluator::default();
        evaluator.evaluate(&map, &snapshot, 0.1);
        let yaw_30fps = look_delta(
            &map,
            &evaluator,
            &snapshot,
            Action::LookLeft,
            Action::LookRight,
            1.0 / 30.0,
            0.005,
            0.1,
            false,
        );
        let yaw_144fps = look_delta(
            &map,
            &evaluator,
            &snapshot,
            Action::LookLeft,
            Action::LookRight,
            1.0 / 144.0,
            0.005,
            0.1,
            false,
        );
        let pitch = look_delta(
            &map,
            &evaluator,
            &snapshot,
            Action::LookUp,
            Action::LookDown,
            1.0 / 60.0,
            0.005,
            0.1,
            false,
        );
        assert!((yaw_30fps - 0.05).abs() < 1e-6);
        assert_eq!(yaw_30fps, yaw_144fps);
        assert!((pitch + 0.05).abs() < 1e-6);
    }

    #[test]
    fn pointer_delta_wins_when_backend_emits_absolute_mouse_events() {
        let events = [
            egui::Event::MouseMoved(egui::vec2(800.0, 360.0)),
            egui::Event::PointerMoved(egui::pos2(800.0, 360.0)),
        ];
        assert_eq!(
            mouse_motion_delta(&events, egui::vec2(160.0, 0.0)),
            egui::vec2(160.0, 0.0)
        );
        let zero_raw_with_pointer = [
            egui::Event::MouseMoved(egui::Vec2::ZERO),
            egui::Event::PointerMoved(egui::pos2(900.0, 360.0)),
        ];
        assert_eq!(
            mouse_motion_delta(&zero_raw_with_pointer, egui::vec2(100.0, 0.0)),
            egui::vec2(100.0, 0.0)
        );
        let relative_raw_only = [egui::Event::MouseMoved(egui::vec2(4.0, -3.0))];
        assert_eq!(
            mouse_motion_delta(&relative_raw_only, egui::Vec2::ZERO),
            egui::vec2(4.0, -3.0)
        );
        let zero_raw_only = [egui::Event::MouseMoved(egui::Vec2::ZERO)];
        assert_eq!(
            mouse_motion_delta(&zero_raw_only, egui::Vec2::ZERO),
            egui::Vec2::ZERO
        );

        let ctx = egui::Context::default();
        ctx.begin_pass(egui::RawInput {
            events: vec![egui::Event::PointerMoved(egui::pos2(100.0, 80.0))],
            ..Default::default()
        });
        let mut output = ctx.end_pass();
        output.textures_delta.clear();
        ctx.begin_pass(egui::RawInput {
            events: vec![egui::Event::PointerMoved(egui::pos2(117.0, 91.0))],
            ..Default::default()
        });
        let delta = ctx.input(|input| mouse_motion_delta(&input.events, input.pointer.delta()));
        output = ctx.end_pass();
        output.textures_delta.clear();
        assert_eq!(delta, egui::vec2(17.0, 11.0));
    }

    #[test]
    fn pointer_hover_blocks_mouse_look_but_not_controller_look() {
        let mut map = ActionMap::default();
        for action in [Action::LookLeft, Action::LookRight] {
            map.set(action, 0, InputSource::Unbound);
            map.set(action, 1, InputSource::Unbound);
        }
        let mouse = InputSource::MouseAxis {
            axis: Axis2::X,
            sign: 1,
        };
        let stick = InputSource::PadAxis {
            axis: PadAxis::RightStickX,
            sign: 1,
        };
        map.set(Action::LookLeft, 0, mouse);
        map.set(Action::LookRight, 0, stick);
        let mut snapshot = InputSnapshot::new();
        snapshot.focused = true;
        snapshot.set(mouse, 12.0);
        snapshot.set(stick, 1.0);
        let mut evaluator = Evaluator::default();
        evaluator.evaluate(&map, &snapshot, 0.1);
        let delta = look_delta(
            &map,
            &evaluator,
            &snapshot,
            Action::LookLeft,
            Action::LookRight,
            1.0 / 60.0,
            0.005,
            0.1,
            true,
        );
        assert!((delta + 2.5 / 60.0).abs() < 1e-6);
    }

    #[test]
    fn freecam_dolly_wheel_delta_is_not_scaled_by_frame_time() {
        let mut map = ActionMap::default();
        let forward = InputSource::Wheel {
            axis: Axis2::Y,
            sign: 1,
        };
        map.set(Action::FreeDollyForward, 0, forward);
        map.set(Action::FreeDollyForward, 1, InputSource::Unbound);
        map.set(Action::FreeDollyBack, 0, InputSource::Unbound);
        map.set(Action::FreeDollyBack, 1, InputSource::Unbound);
        let mut snapshot = InputSnapshot::new();
        snapshot.focused = true;
        snapshot.set(forward, 2.0);
        let mut evaluator = Evaluator::default();
        evaluator.evaluate(&map, &snapshot, 0.1);
        let slow = freecam_dolly_delta(
            &map,
            &evaluator,
            &snapshot,
            1.0 / 30.0,
            48.0,
            220.0,
            0.1,
            false,
        );
        let fast = freecam_dolly_delta(
            &map,
            &evaluator,
            &snapshot,
            1.0 / 144.0,
            48.0,
            220.0,
            0.1,
            false,
        );
        assert_eq!(slow, 96.0);
        assert_eq!(slow, fast);
    }
}
