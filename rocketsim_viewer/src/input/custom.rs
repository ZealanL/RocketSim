//! Simulation-defined custom actions and their persisted bindings.
//!
//! A [`CustomActionRegistry`] is built from the declaration sent by the
//! simulation. A [`CustomActionMap`] holds the viewer's bindings for those
//! names. Defaults fill missing entries; user rebinds survive later
//! declarations and viewer restarts.

use std::{
    collections::BTreeMap,
    sync::atomic::{AtomicU64, Ordering},
};

use rocketsim_vis::proto::CustomActionDecl;
use serde::{Deserialize, Serialize};

use super::sources::{InputSource, Key};

/// One custom action declared by the simulation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CustomAction {
    /// Stable name from the simulation. Used as the settings key.
    pub name: String,
    /// Parsed default keyboard binds. Unknown key names are dropped.
    pub default_sources: Vec<InputSource>,
}

/// Ordered custom actions from the current simulation declaration.
#[derive(Clone, Debug, Default)]
pub struct CustomActionRegistry {
    /// Declaration revision, used to detect replacements.
    pub rev: u64,
    /// Actions in declaration order. Indices are used on the wire.
    pub actions: Vec<CustomAction>,
}

impl CustomActionRegistry {
    /// Build a registry from a wire declaration, parsing default key names.
    pub fn from_decl(decl: &CustomActionDecl) -> Self {
        let actions = decl
            .actions
            .iter()
            .map(|action| CustomAction {
                name: action.name.clone(),
                default_sources: action
                    .default_keys
                    .iter()
                    .filter_map(|key| Key::from_name(key).map(InputSource::Key))
                    .collect(),
            })
            .collect();
        Self {
            rev: decl.rev,
            actions,
        }
    }

    /// Return the action at `index`, if present.
    pub fn get(&self, index: usize) -> Option<&CustomAction> {
        self.actions.get(index)
    }

    /// Return the index of the action named `name`, if present.
    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.actions.iter().position(|action| action.name == name)
    }

    /// Return `true` when no actions are declared.
    pub fn is_empty(&self) -> bool {
        self.actions.is_empty()
    }
}

static NEXT_CUSTOM_REVISION: AtomicU64 = AtomicU64::new(1);

fn next_revision() -> u64 {
    NEXT_CUSTOM_REVISION
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |revision| {
            revision.checked_add(1)
        })
        .unwrap_or(1)
}

const UNBOUND: [InputSource; 2] = [InputSource::Unbound; 2];

/// Persisted bindings for simulation-defined custom actions.
///
/// Keys are action names, so rebinding an action keeps its identity across
/// declaration replacements and viewer restarts.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CustomActionMap {
    #[serde(default)]
    bindings: BTreeMap<String, [InputSource; 2]>,
    #[serde(skip, default = "next_revision")]
    revision: u64,
}

impl PartialEq for CustomActionMap {
    fn eq(&self, other: &Self) -> bool {
        self.bindings == other.bindings
    }
}

impl CustomActionMap {
    /// Return an empty binding map.
    pub fn unbound() -> Self {
        Self {
            bindings: BTreeMap::new(),
            revision: next_revision(),
        }
    }

    pub(super) fn revision(&self) -> u64 {
        self.revision
    }

    /// Return the two binding slots for `name`, or two unbound slots.
    pub fn sources(&self, name: &str) -> &[InputSource; 2] {
        self.bindings.get(name).unwrap_or(&UNBOUND)
    }

    /// Set one binding slot for `name`.
    pub fn set(&mut self, name: impl Into<String>, slot: usize, source: InputSource) {
        if slot >= 2 {
            return;
        }
        let name = name.into();
        let current = *self.sources(&name);
        if current[slot] == source {
            return;
        }
        let mut next = current;
        next[slot] = source;
        self.bindings.insert(name, next);
        self.revision = next_revision();
    }

    /// Add default bindings for declared actions that have no saved entry.
    ///
    /// Return `true` when at least one entry was added. Existing entries are
    /// never overwritten, so a user rebind is not lost when the simulation
    /// replaces its declaration.
    pub fn merge_defaults(&mut self, registry: &CustomActionRegistry) -> bool {
        let mut added = false;
        for action in &registry.actions {
            if self.bindings.contains_key(&action.name) {
                continue;
            }
            let mut slots = UNBOUND;
            for (slot, source) in action.default_sources.iter().take(2).enumerate() {
                slots[slot] = *source;
            }
            if slots != UNBOUND {
                added = true;
            }
            self.bindings.insert(action.name.clone(), slots);
        }
        if added {
            self.revision = next_revision();
        }
        added
    }

    /// Return binding conflicts among declared custom actions.
    pub fn conflicts(
        &self,
        registry: &CustomActionRegistry,
    ) -> Vec<(String, usize, String, usize)> {
        let mut out = Vec::new();
        let names: Vec<&str> = registry
            .actions
            .iter()
            .map(|action| action.name.as_str())
            .filter(|name| self.bindings.contains_key(*name))
            .collect();
        for (i, left) in names.iter().enumerate() {
            for slot in 0..2 {
                let source = self.sources(left)[slot];
                if source == InputSource::Unbound {
                    continue;
                }
                for (j, right) in names.iter().enumerate().skip(i) {
                    for other_slot in 0..2 {
                        if (j > i || other_slot > slot) && source == self.sources(right)[other_slot]
                        {
                            out.push(((*left).to_string(), slot, (*right).to_string(), other_slot));
                        }
                    }
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use rocketsim_vis::proto::{CustomActionDecl, CustomActionDef};

    use super::*;

    fn declaration() -> CustomActionDecl {
        CustomActionDecl {
            rev: 1,
            actions: vec![
                CustomActionDef {
                    name: "show_route".into(),
                    default_keys: vec!["F5".into(), "Space".into()],
                },
                CustomActionDef {
                    name: "ping".into(),
                    default_keys: vec!["V".into(), "NotAKey".into()],
                },
            ],
        }
    }

    #[test]
    fn defaults_fill_missing_entries_and_keep_user_rebinds() {
        let registry = CustomActionRegistry::from_decl(&declaration());
        assert_eq!(
            registry.actions[1].default_sources,
            vec![InputSource::Key(Key::V)]
        );

        let mut map = CustomActionMap::unbound();
        assert!(map.merge_defaults(&registry));
        assert_eq!(map.sources("show_route")[0], InputSource::Key(Key::F5));
        assert_eq!(map.sources("show_route")[1], InputSource::Key(Key::Space));
        assert!(!map.merge_defaults(&registry));

        map.set("show_route", 0, InputSource::Key(Key::V));
        let mut changed = declaration();
        changed.rev = 2;
        changed.actions[0].default_keys = vec!["B".into()];
        let next = CustomActionRegistry::from_decl(&changed);
        assert!(!map.merge_defaults(&next));
        assert_eq!(map.sources("show_route")[0], InputSource::Key(Key::V));
    }

    #[test]
    fn conflicts_report_shared_custom_bindings() {
        let mut registry = CustomActionRegistry::from_decl(&declaration());
        registry.actions.push(CustomAction {
            name: "duplicate".into(),
            default_sources: vec![InputSource::Key(Key::F5)],
        });
        let mut map = CustomActionMap::unbound();
        map.merge_defaults(&registry);
        let conflicts = map.conflicts(&registry);
        assert!(
            conflicts
                .iter()
                .any(|(name, _, other, _)| name == "show_route" && other == "duplicate")
        );
    }
}
