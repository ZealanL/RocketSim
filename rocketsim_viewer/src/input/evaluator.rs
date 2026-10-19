//! Snapshot and edge evaluation for action maps.

use std::collections::{HashMap, HashSet};

use super::{
    actions::{ACTIONS, Action, ActionMap},
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
    suppressed: HashSet<InputSource>,
    was_focused: bool,
}
impl Default for Evaluator {
    fn default() -> Self {
        Self {
            previous: [false; ACTIONS.len()],
            previous_map_revision: None,
            suppressed: HashSet::new(),
            was_focused: true,
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
            let mut v: f32 = 0.0;
            for &src in m.sources(a) {
                if src == InputSource::Unbound || self.suppressed.contains(&src) {
                    continue;
                }
                let x = self.source_value(s, src, d);
                if x.abs() > v.abs() {
                    v = x;
                }
            }
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
