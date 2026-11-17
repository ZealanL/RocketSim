//! Snapshot and edge evaluation for action maps.

use std::collections::{HashMap, HashSet};

use super::{
    actions::{ACTIONS, Action, ActionMap},
    custom::{CustomActionMap, CustomActionRegistry},
    sources::InputSource,
};

#[derive(Debug, Clone, Default)]
pub struct InputSnapshot {
    pub(super) values: HashMap<InputSource, f32>,
    pub focused: bool,
}
impl InputSnapshot {
    pub fn new() -> Self {
        Self {
            values: HashMap::new(),
            focused: true,
        }
    }
    pub fn set(&mut self, s: InputSource, v: f32) {
        self.values.insert(s, if v.is_finite() { v } else { 0.0 });
    }
    pub fn set_down(&mut self, s: InputSource, b: bool) {
        self.set(s, if b { 1.0 } else { 0.0 });
    }
    pub fn value(&self, s: InputSource) -> f32 {
        if self.focused {
            self.values.get(&s).copied().unwrap_or(0.0)
        } else {
            0.0
        }
    }
    pub fn clear(&mut self) {
        self.values.clear();
    }
}
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ActionValue {
    pub value: f32,
    pub pressed: bool,
    pub just_pressed: bool,
    pub just_released: bool,
}
#[derive(Debug)]
pub struct Evaluator {
    previous: [bool; ACTIONS.len()],
    previous_map_revision: Option<u64>,
    previous_custom: HashMap<String, bool>,
    previous_custom_revision: Option<(u64, u64)>,
    suppressed: HashSet<InputSource>,
    was_focused: bool,
    was_focused_custom: bool,
}
impl Default for Evaluator {
    fn default() -> Self {
        Self {
            previous: [false; ACTIONS.len()],
            previous_map_revision: None,
            previous_custom: HashMap::new(),
            previous_custom_revision: None,
            suppressed: HashSet::new(),
            was_focused: true,
            was_focused_custom: true,
        }
    }
}
impl Evaluator {
    pub fn suppress_until_release(&mut self, s: &InputSnapshot) {
        if s.focused {
            self.suppressed
                .extend(s.values.iter().filter_map(|(source, value)| {
                    if value.max(0.0) > source_release_threshold(*source) {
                        Some(*source)
                    } else {
                        None
                    }
                }));
        }
    }
    pub fn source_value(
        &self,
        snapshot: &InputSnapshot,
        source: InputSource,
        deadzone: f32,
    ) -> f32 {
        if source == InputSource::Unbound || !snapshot.focused || self.suppressed.contains(&source)
        {
            return 0.0;
        }
        let mut value = snapshot.value(source).max(0.0);
        if !matches!(
            source,
            InputSource::MouseAxis { .. } | InputSource::Wheel { .. }
        ) {
            value = value.clamp(0.0, 1.0);
        }
        if matches!(
            source,
            InputSource::PadAxis { .. } | InputSource::PadRaw { .. }
        ) {
            let dz = deadzone.clamp(0.0, 0.99);
            value = if value <= dz {
                0.0
            } else {
                ((value - dz) / (1.0 - dz)).clamp(0.0, 1.0)
            };
        }
        value
    }
    pub fn evaluate(
        &mut self,
        map: &ActionMap,
        snapshot: &InputSnapshot,
        deadzone: f32,
    ) -> HashMap<Action, ActionValue> {
        let mut values = HashMap::with_capacity(ACTIONS.len());
        self.evaluate_into(map, snapshot, deadzone, &mut values);
        values
    }

    /// Evaluate actions into a caller-owned map to reuse its allocation.
    pub fn evaluate_into(
        &mut self,
        m: &ActionMap,
        s: &InputSnapshot,
        d: f32,
        out: &mut HashMap<Action, ActionValue>,
    ) {
        let regain = s.focused && !self.was_focused;
        let map_changed = self
            .previous_map_revision
            .is_some_and(|revision| revision != m.revision());
        if regain || map_changed {
            self.suppress_until_release(s);
        }
        self.previous_map_revision = Some(m.revision());
        if s.focused {
            self.suppressed
                .retain(|source| s.value(*source).max(0.0) > source_release_threshold(*source));
        }
        out.clear();
        for a in ACTIONS {
            let v = self.eval_sources(m.sources(a), s, d);
            let down = v.abs() >= 0.5;
            let previous = &mut self.previous[a as usize];
            let old = *previous;
            *previous = down;
            let jp = down && !old && !regain;
            let jr = !down && old;
            out.insert(
                a,
                ActionValue {
                    value: v,
                    pressed: down,
                    just_pressed: jp,
                    just_released: jr,
                },
            );
        }
        self.was_focused = s.focused;
    }

    fn eval_sources(&self, sources: &[InputSource; 2], s: &InputSnapshot, d: f32) -> f32 {
        let mut v: f32 = 0.0;
        for &src in sources {
            if src == InputSource::Unbound || self.suppressed.contains(&src) {
                continue;
            }
            let x = self.source_value(s, src, d);
            if x.abs() > v.abs() {
                v = x;
            }
        }
        v
    }

    /// Evaluate simulation-defined custom actions into a caller-owned map.
    ///
    /// Edge state is keyed by action name. A declaration or binding change
    /// clears old edges so a held key does not appear as a fresh press.
    pub fn evaluate_custom_into(
        &mut self,
        registry: &CustomActionRegistry,
        map: &CustomActionMap,
        s: &InputSnapshot,
        d: f32,
        out: &mut HashMap<String, ActionValue>,
    ) {
        let revision = (registry.rev, map.revision());
        let changed = self
            .previous_custom_revision
            .is_some_and(|previous| previous != revision);
        if changed {
            self.previous_custom.clear();
        }
        self.previous_custom_revision = Some(revision);
        let regain = s.focused && !self.was_focused_custom;
        self.was_focused_custom = s.focused;
        out.clear();
        for action in &registry.actions {
            let v = self.eval_sources(map.sources(&action.name), s, d);
            let down = v.abs() >= 0.5;
            let old = self
                .previous_custom
                .get(&action.name)
                .copied()
                .unwrap_or(false);
            self.previous_custom.insert(action.name.clone(), down);
            out.insert(
                action.name.clone(),
                ActionValue {
                    value: v,
                    pressed: down,
                    just_pressed: down && !old && !regain,
                    just_released: !down && old,
                },
            );
        }
    }
}
fn source_release_threshold(source: InputSource) -> f32 {
    if matches!(
        source,
        InputSource::PadAxis { .. }
            | InputSource::PadRaw { .. }
            | InputSource::MouseAxis { .. }
            | InputSource::Wheel { .. }
    ) {
        0.1
    } else {
        0.5
    }
}

#[cfg(test)]
mod tests {
    use rocketsim_vis::proto::{CustomActionDecl, CustomActionDef};

    use super::*;
    use crate::input::sources::Key;

    #[test]
    fn custom_actions_follow_press_and_release_edges() {
        let decl = CustomActionDecl {
            rev: 1,
            actions: vec![CustomActionDef {
                name: "ping".into(),
                default_keys: vec!["F5".into()],
                button: None,
            }],
        };
        let registry = CustomActionRegistry::from_decl(&decl);
        let mut map = CustomActionMap::unbound();
        map.merge_defaults(&registry);
        let source = InputSource::Key(Key::F5);
        let mut evaluator = Evaluator::default();
        let mut snapshot = InputSnapshot::new();
        let mut out = HashMap::new();

        evaluator.evaluate_custom_into(&registry, &map, &snapshot, 0.0, &mut out);
        assert!(!out["ping"].pressed);

        snapshot.set_down(source, true);
        evaluator.evaluate_custom_into(&registry, &map, &snapshot, 0.0, &mut out);
        assert!(out["ping"].just_pressed);

        evaluator.evaluate_custom_into(&registry, &map, &snapshot, 0.0, &mut out);
        assert!(!out["ping"].just_pressed);
        assert!(out["ping"].pressed);

        snapshot.set_down(source, false);
        evaluator.evaluate_custom_into(&registry, &map, &snapshot, 0.0, &mut out);
        assert!(out["ping"].just_released);
    }
}
