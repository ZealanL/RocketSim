use super::{ViewerUiState, input_state::InputState};
use crate::{
    input::{Action, ActionMap, Axis2, InputSnapshot, InputSource},
    settings::ViewerSettings,
};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(super) struct CaptureUpdate {
    pub(super) completed: bool,
    pub(super) cancelled: bool,
}

pub(super) struct CaptureContext<'a> {
    pub(super) ui: &'a mut ViewerUiState,
    pub(super) settings: &'a mut ViewerSettings,
    pub(super) input: &'a mut InputState,
    pub(super) action_map: &'a ActionMap,
    pub(super) snapshot: &'a InputSnapshot,
    pub(super) cancel_hover: bool,
    pub(super) cancel_click: bool,
    pub(super) dt: f32,
}

pub(super) fn update_capture(context: CaptureContext<'_>) -> CaptureUpdate {
    let CaptureContext {
        ui,
        settings,
        input,
        action_map,
        snapshot,
        cancel_hover,
        cancel_click,
        dt,
    } = context;
    let mut update = CaptureUpdate::default();
    let Some((action, slot)) = ui.rebind_action else {
        return update;
    };

    if !ui.menu_open {
        ui.rebind_action = None;
        ui.rebind_capture = None;
        input.suppress_until_release(snapshot);
        update.cancelled = true;
        return update;
    }

    let back_held = action_map
        .sources(Action::MenuBack)
        .iter()
        .copied()
        .find(|source| *source != InputSource::Unbound && snapshot.value(*source).abs() >= 0.5);
    if ui
        .capture_start_back
        .is_some_and(|source| snapshot.value(source).abs() < 0.5)
    {
        ui.capture_start_back = None;
    }
    let cancel_source = back_held.filter(|source| Some(*source) != ui.capture_start_back);
    let bind_source = if let Some(source) = cancel_source {
        ui.capture_pending_back = Some(source);
        ui.capture_cancel_hold += dt;
        if ui.capture_cancel_hold >= 0.75 {
            ui.rebind_action = None;
            ui.rebind_capture = None;
            ui.capture_cancel_hold = 0.0;
            ui.capture_pending_back = None;
            input.suppress_until_release(snapshot);
            update.cancelled = true;
            return update;
        }
        None
    } else if ui.capture_pending_back.is_some() {
        ui.capture_cancel_hold = 0.0;
        ui.capture_pending_back.take()
    } else {
        ui.capture_cancel_hold = 0.0;
        None
    };

    if cancel_click || cancel_source.is_some() {
        return update;
    }
    let Some(capture) = ui.rebind_capture.as_mut() else {
        return update;
    };

    let mut capture_snapshot = snapshot.clone();
    if cancel_hover {
        for axis in [Axis2::X, Axis2::Y] {
            for sign in [-1, 1] {
                capture_snapshot.set(InputSource::MouseAxis { axis, sign }, 0.0);
            }
        }
    }
    if let Some(source) = capture.poll(&capture_snapshot).or(bind_source) {
        settings
            .action_bindings
            .get_or_insert_with(|| action_map.clone())
            .set(action, slot, source);
        settings.mark_dirty();
        ui.binding_warning = None;
        ui.rebind_action = None;
        ui.rebind_capture = None;
        input.suppress_until_release(snapshot);
        update.completed = true;
    }
    update
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        input::sources::Key,
        input::{Capture, InputSnapshot},
    };

    #[test]
    fn short_back_press_rebinds_but_a_held_press_cancels() {
        let back = InputSource::Key(Key::Q);
        let mut action_map = ActionMap::default();
        action_map.set(Action::MenuBack, 0, back);

        let mut settings = ViewerSettings::default();
        let mut input = InputState::new();
        let mut ui = ViewerUiState::new(true);
        ui.rebind_action = Some((Action::Jump, 0));
        let mut snapshot = InputSnapshot::new();
        ui.rebind_capture = Some(Capture::begin(&snapshot));
        snapshot.set_down(back, true);
        let pressed = update_capture(CaptureContext {
            ui: &mut ui,
            settings: &mut settings,
            input: &mut input,
            action_map: &action_map,
            snapshot: &snapshot,
            cancel_hover: false,
            cancel_click: false,
            dt: 0.2,
        });
        assert!(!pressed.cancelled);
        snapshot.set_down(back, false);
        let released = update_capture(CaptureContext {
            ui: &mut ui,
            settings: &mut settings,
            input: &mut input,
            action_map: &action_map,
            snapshot: &snapshot,
            cancel_hover: false,
            cancel_click: false,
            dt: 0.01,
        });
        assert!(released.completed);
        assert_eq!(settings.action_map().sources(Action::Jump)[0], back);

        let mut settings = ViewerSettings::default();
        let mut input = InputState::new();
        let mut ui = ViewerUiState::new(true);
        ui.rebind_action = Some((Action::Jump, 0));
        let mut snapshot = InputSnapshot::new();
        ui.rebind_capture = Some(Capture::begin(&snapshot));
        snapshot.set_down(back, true);
        let first = update_capture(CaptureContext {
            ui: &mut ui,
            settings: &mut settings,
            input: &mut input,
            action_map: &action_map,
            snapshot: &snapshot,
            cancel_hover: false,
            cancel_click: false,
            dt: 0.4,
        });
        assert!(!first.cancelled);
        let held = update_capture(CaptureContext {
            ui: &mut ui,
            settings: &mut settings,
            input: &mut input,
            action_map: &action_map,
            snapshot: &snapshot,
            cancel_hover: false,
            cancel_click: false,
            dt: 0.36,
        });
        assert!(held.cancelled);
        assert_eq!(ui.rebind_action, None);
        assert!(settings.action_bindings.is_none());
    }
}
