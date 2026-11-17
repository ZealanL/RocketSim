//! On-screen buttons declared by the simulation for custom actions.

use egui::Context;

use crate::{
    input::{CustomActionRegistry, InputSource},
    settings::ViewerSettings,
};

/// Draw the Actions window and return the declaration indices that were clicked.
///
/// The window appears only when the setting is on and at least one action has
/// an on-screen button. A click maps to the same `custom_presses` bit as a
/// keybind press. An action with no keybind still works here.
pub(crate) fn show_action_buttons(
    ctx: &Context,
    settings: &ViewerSettings,
    registry: &CustomActionRegistry,
) -> Vec<usize> {
    if !settings.show_action_buttons {
        return Vec::new();
    }
    let targets: Vec<_> = registry.button_actions().collect();
    if targets.is_empty() {
        return Vec::new();
    }
    let mut pressed = Vec::new();
    // Let `horizontal_wrapped` use the full window width so buttons fill the
    // window and reflow when the user resizes it. `default_width` only sets the
    // initial size; the window stays resizable. Do not wrap each button in
    // `push_id`: that gives the button a child ui whose max rect is only the
    // space left on the current row, so egui squeezes the button instead of
    // moving it to a new row. Buttons use position-based auto ids, so duplicate
    // labels stay distinct. Cap the list height and scroll for long lists.
    egui::Window::new("Actions")
        .default_width(360.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .max_height(360.0)
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        for (index, action) in targets {
                            let label = action.button_label.as_deref().unwrap_or(&action.name);
                            let response = ui.button(label);
                            if response.clicked() {
                                pressed.push(index);
                            }
                            let bound = settings
                                .custom_action_map()
                                .sources(&action.name)
                                .iter()
                                .copied()
                                .find(|source| *source != InputSource::Unbound)
                                .map(InputSource::name);
                            response.on_hover_ui(|ui| {
                                ui.label(format!("Action: {}", action.name));
                                match bound {
                                    Some(name) => {
                                        ui.label(format!("Bound to {name}"));
                                    }
                                    None => {
                                        ui.label("No keybind (button only)");
                                    }
                                }
                            });
                        }
                    });
                });
        });
    pressed
}
