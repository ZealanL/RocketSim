//! Viewer settings: TOML persistence.
//!
//! File: `settings.toml` under `rocketsim-vis/` in the OS config dir
//! (`./settings.toml` fallback). Loading never fails (missing/corrupt
//! files give defaults); every field has a default so old files parse.

use std::{
    io,
    path::{Path, PathBuf},
    time::Instant,
};

use serde::{Deserialize, Serialize};

use crate::{
    camera::CameraConfig,
    input::{Action, ActionMap, InputSource, legacy::Bindings},
};

/// Device view for the universal action binding table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BindingDevice {
    #[default]
    All,
    KeyboardMouse,
    Controller,
}

impl BindingDevice {
    /// Return the original action slots visible in this device view.
    pub fn visible_slots(self, sources: [InputSource; 2]) -> Vec<usize> {
        sources
            .iter()
            .enumerate()
            .filter_map(|(slot, source)| {
                let visible = match self {
                    Self::All => true,
                    Self::KeyboardMouse => matches!(
                        source,
                        InputSource::Key(_)
                            | InputSource::Mouse(_)
                            | InputSource::MouseAxis { .. }
                            | InputSource::Wheel { .. }
                            | InputSource::Unbound
                    ),
                    Self::Controller => matches!(
                        source,
                        InputSource::PadButton(_)
                            | InputSource::PadAxis { .. }
                            | InputSource::PadRaw { .. }
                            | InputSource::Unbound
                    ),
                };
                visible.then_some(slot)
            })
            .collect()
    }

    /// Reset only slots shown in this device view. Hidden-device bindings stay unchanged.
    pub fn reset_action(self, map: &mut ActionMap, action: Action) {
        let defaults = ActionMap::default();
        let visible = self.visible_slots(*map.sources(action));
        if self == Self::All {
            for slot in visible {
                map.set(action, slot, defaults.sources(action)[slot]);
            }
            return;
        }

        for &slot in &visible {
            map.set(action, slot, InputSource::Unbound);
        }
        let mut assigned = [false; 2];
        for (default_slot, source) in defaults.sources(action).iter().copied().enumerate() {
            let matches_device = match self {
                Self::All => true,
                Self::KeyboardMouse => matches!(
                    source,
                    InputSource::Key(_)
                        | InputSource::Mouse(_)
                        | InputSource::MouseAxis { .. }
                        | InputSource::Wheel { .. }
                ),
                Self::Controller => matches!(
                    source,
                    InputSource::PadButton(_)
                        | InputSource::PadAxis { .. }
                        | InputSource::PadRaw { .. }
                ),
            };
            if !matches_device || source == InputSource::Unbound {
                continue;
            }
            let target = if visible.contains(&default_slot) && !assigned[default_slot] {
                Some(default_slot)
            } else {
                visible.iter().copied().find(|slot| !assigned[*slot])
            };
            if let Some(target) = target {
                map.set(action, target, source);
                assigned[target] = true;
            }
        }
    }
}

/// Frame-rate limiter (Hz, 0 = uncapped). Default 60.
pub const DEFAULT_FPS_LIMIT: u32 = 60;
/// UI scale multiplier. Default 1.0.
pub const DEFAULT_UI_SCALE: f32 = 1.0;
/// MSAA samples (relaunch applies). Default 4.
pub const DEFAULT_MSAA: u16 = 4;
/// Debounce between settings saves (s).
const SAVE_DEBOUNCE_SECS: f32 = 1.0;

/// Persistent viewer settings (TOML-serializable, all fields defaulted).
///
/// Shape: `[camera]` tuning (FOV, follow distance/height/angle, stiffness,
/// transition) plus overlay flags (`show_trail`, `show_boost`,
/// `show_ground_line`, `show_pads`), `fps_limit_hz` (0 = uncapped),
/// `ui_scale`, `vsync`/`msaa` (apply on relaunch), and `[bindings]`
/// key/mouse/gamepad tables (all defaulted, old files still parse).
/// Loading never fails (missing/corrupt files give defaults); every field
/// has a default so old files missing new keys still parse.
///
/// ```rust
/// use rocketsim_viewer::settings::ViewerSettings;
///
/// let mut s = ViewerSettings::default();
/// s.show_boost = false;
/// s.fps_limit_hz = 0; // uncapped
/// s.mark_dirty();
/// s.reset_camera(); // back to RL defaults, stays dirty until saved
///
/// // File shape: plain TOML, unknown keys ignored on load.
/// let text = toml::to_string(&s).unwrap();
/// let back: ViewerSettings = toml::from_str(&text).unwrap();
/// assert!(!back.show_boost);
/// assert_eq!(back.fps_limit_hz, 0);
/// ```
#[derive(Debug, Clone, Serialize)]
pub struct ViewerSettings {
    /// Follow-camera + FOV tuning (RL defaults).
    pub camera: CameraConfig,
    /// Ball trail ribbon.
    pub show_trail: bool,
    /// Per-car boost flames.
    pub show_boost: bool,
    /// Ball-to-ground drop line.
    pub show_ground_line: bool,
    /// Boost pads (instanced draws).
    pub show_pads: bool,
    /// Frame-rate cap in Hz (0 = uncapped).
    pub fps_limit_hz: u32,
    /// UI scale multiplier.
    pub ui_scale: f32,
    /// Vsync (applies on relaunch).
    pub vsync: bool,
    /// MSAA samples (applies on relaunch).
    pub msaa: u16,
    /// Legacy table for settings-file compatibility. Runtime actions use `action_bindings`.
    /// Direct edits to legacy action fields do not update that map. Call `set_legacy_bindings` to refresh it.
    pub bindings: Bindings,
    /// Action bindings. This table takes precedence over legacy bindings.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action_bindings: Option<crate::input::ActionMap>,
    /// Spectate-only mode: watch the followed car, send neutral inputs.
    pub spectate_only: bool,
    /// Drive-demo mutators, sent as ControlMsg mutator bits (all off = RL
    /// defaults). Only options the sim can apply live are listed here.
    pub mut_unlimited_boost: bool,
    pub mut_recharge: bool,
    pub mut_flips: bool,
    pub mut_double_jumps: bool,
    /// Last save time (not persisted).
    #[serde(skip)]
    last_save: Option<Instant>,
    /// Unsaved changes pending (not persisted).
    #[serde(skip)]
    dirty: bool,
    #[serde(skip)]
    explicit_path: Option<PathBuf>,
    #[serde(skip)]
    fallback_action_bindings: ActionMap,
}

#[derive(Deserialize)]
#[serde(default)]
struct ViewerSettingsData {
    camera: CameraConfig,
    show_trail: bool,
    show_boost: bool,
    show_ground_line: bool,
    show_pads: bool,
    fps_limit_hz: u32,
    ui_scale: f32,
    vsync: bool,
    msaa: u16,
    bindings: Bindings,
    action_bindings: Option<ActionMap>,
    spectate_only: bool,
    mut_unlimited_boost: bool,
    mut_recharge: bool,
    mut_flips: bool,
    mut_double_jumps: bool,
}

impl Default for ViewerSettingsData {
    fn default() -> Self {
        let settings = ViewerSettings::default();
        Self {
            camera: settings.camera,
            show_trail: settings.show_trail,
            show_boost: settings.show_boost,
            show_ground_line: settings.show_ground_line,
            show_pads: settings.show_pads,
            fps_limit_hz: settings.fps_limit_hz,
            ui_scale: settings.ui_scale,
            vsync: settings.vsync,
            msaa: settings.msaa,
            bindings: settings.bindings,
            action_bindings: settings.action_bindings,
            spectate_only: settings.spectate_only,
            mut_unlimited_boost: settings.mut_unlimited_boost,
            mut_recharge: settings.mut_recharge,
            mut_flips: settings.mut_flips,
            mut_double_jumps: settings.mut_double_jumps,
        }
    }
}

impl<'de> Deserialize<'de> for ViewerSettings {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let data = ViewerSettingsData::deserialize(deserializer)?;
        let fallback_action_bindings = ActionMap::from_legacy(&data.bindings);
        let action_bindings = Some(
            data.action_bindings
                .unwrap_or_else(|| fallback_action_bindings.clone()),
        );
        Ok(Self {
            camera: data.camera,
            show_trail: data.show_trail,
            show_boost: data.show_boost,
            show_ground_line: data.show_ground_line,
            show_pads: data.show_pads,
            fps_limit_hz: data.fps_limit_hz,
            ui_scale: data.ui_scale,
            vsync: data.vsync,
            msaa: data.msaa,
            bindings: data.bindings,
            action_bindings,
            spectate_only: data.spectate_only,
            mut_unlimited_boost: data.mut_unlimited_boost,
            mut_recharge: data.mut_recharge,
            mut_flips: data.mut_flips,
            mut_double_jumps: data.mut_double_jumps,
            last_save: None,
            dirty: false,
            explicit_path: None,
            fallback_action_bindings,
        })
    }
}

impl Default for ViewerSettings {
    fn default() -> Self {
        Self {
            camera: CameraConfig::default(),
            show_trail: true,
            show_boost: true,
            show_ground_line: true,
            show_pads: true,
            fps_limit_hz: DEFAULT_FPS_LIMIT,
            ui_scale: DEFAULT_UI_SCALE,
            vsync: true,
            msaa: DEFAULT_MSAA,
            bindings: Bindings::default(),
            action_bindings: None,
            spectate_only: false,
            mut_unlimited_boost: false,
            mut_recharge: false,
            mut_flips: false,
            mut_double_jumps: false,
            last_save: None,
            dirty: false,
            explicit_path: None,
            fallback_action_bindings: ActionMap::default(),
        }
    }
}

impl ViewerSettings {
    /// Return the canonical action map without cloning it.
    pub fn action_map(&self) -> &ActionMap {
        self.action_bindings
            .as_ref()
            .unwrap_or(&self.fallback_action_bindings)
    }

    /// Return a clone of the canonical action map.
    pub fn input_bindings(&self) -> ActionMap {
        self.action_map().clone()
    }

    /// Replace legacy bindings and migrate them to the runtime action map.
    pub fn set_legacy_bindings(&mut self, bindings: Bindings) {
        self.bindings = bindings;
        let action_map = ActionMap::from_legacy(&self.bindings);
        self.fallback_action_bindings = action_map.clone();
        self.action_bindings = Some(action_map);
        self.mark_dirty();
    }

    /// Settings file path (`rocketsim-vis/settings.toml` in the OS config
    /// dir, `./settings.toml` fallback).
    pub fn path() -> PathBuf {
        let dir = directories_dir().unwrap_or_else(|| PathBuf::from("."));
        dir.join("settings.toml")
    }

    /// Loads from disk, or defaults (with a stderr note) when missing or
    /// corrupt. Unknown fields are ignored; missing fields use defaults.
    pub fn load() -> Self {
        Self::load_from_path(&Self::path())
    }

    pub fn load_from_path(path: &Path) -> Self {
        match std::fs::read_to_string(path) {
            Ok(text) => match toml::from_str::<Self>(&text) {
                Ok(mut settings) => {
                    settings.explicit_path = Some(path.to_path_buf());
                    settings.validate();
                    settings
                }
                Err(e) => {
                    eprintln!(
                        "rocketsim_vis: corrupt {} ({e}), using defaults",
                        path.display()
                    );
                    preserve_corrupt(path);
                    Self {
                        explicit_path: Some(path.to_path_buf()),
                        ..Self::default()
                    }
                }
            },
            Err(_) => Self {
                explicit_path: Some(path.to_path_buf()),
                ..Self::default()
            },
        }
    }

    pub fn save(&self) -> io::Result<()> {
        let default_path;
        let path = match self.explicit_path.as_deref() {
            Some(path) => path,
            None => {
                default_path = Self::path();
                &default_path
            }
        };
        self.save_to_path(path)
    }

    pub fn save_to_path(&self, path: &Path) -> io::Result<()> {
        let text =
            toml::to_string(self).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        atomic_write(path, text.as_bytes())
    }

    fn validate(&mut self) {
        fn finite_clamp(value: &mut f32, fallback: f32, low: f32, high: f32) {
            *value = if value.is_finite() {
                value.clamp(low, high)
            } else {
                fallback
            };
        }
        let default = CameraConfig::default();
        finite_clamp(
            &mut self.camera.fov_degrees,
            default.fov_degrees,
            30.0,
            170.0,
        );
        let pos = self.camera.birds_eye_pos;
        let mut x = pos.x;
        let mut y = pos.y;
        let mut z = pos.z;
        finite_clamp(&mut x, default.birds_eye_pos.x, -20000.0, 20000.0);
        finite_clamp(&mut y, default.birds_eye_pos.y, -20000.0, 20000.0);
        finite_clamp(&mut z, default.birds_eye_pos.z, -20000.0, 20000.0);
        self.camera.birds_eye_pos = glam::Vec3A::new(x, y, z);
        let cam = &mut self.camera.car_cam;
        finite_clamp(&mut cam.distance, default.car_cam.distance, 50.0, 1000.0);
        finite_clamp(&mut cam.height, default.car_cam.height, 0.0, 500.0);
        finite_clamp(&mut cam.angle_deg, default.car_cam.angle_deg, -45.0, 45.0);
        finite_clamp(&mut cam.stiffness, default.car_cam.stiffness, 0.0, 1.0);
        finite_clamp(
            &mut cam.transition_speed,
            default.car_cam.transition_speed,
            0.1,
            5.0,
        );
        finite_clamp(&mut self.ui_scale, DEFAULT_UI_SCALE, 0.5, 2.0);
        if !matches!(self.msaa, 0 | 2 | 4 | 8) {
            self.msaa = DEFAULT_MSAA;
        }
    }

    /// Marks changed (call after any mutation); flushed by
    /// [`save_if_due`](Self::save_if_due) once per second.
    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    /// Saves at most once per second after changes.
    pub fn save_if_due(&mut self) {
        if !self.dirty {
            return;
        }
        let due = match self.last_save {
            None => true,
            Some(t) => t.elapsed().as_secs_f32() >= SAVE_DEBOUNCE_SECS,
        };
        if due {
            if let Err(e) = self.save() {
                eprintln!("rocketsim_vis: cannot save settings ({e})");
                self.last_save = Some(Instant::now());
            } else {
                self.last_save = Some(Instant::now());
                self.dirty = false;
            }
        }
    }

    pub fn flush(&mut self) -> io::Result<()> {
        if self.dirty {
            self.save()?;
            self.dirty = false;
            self.last_save = Some(Instant::now());
        }
        Ok(())
    }

    /// Resets camera tuning to RL defaults (preset button).
    pub fn reset_camera(&mut self) {
        self.camera = CameraConfig::default();
        self.mark_dirty();
    }

    /// Resets everything to defaults.
    pub fn reset_all(&mut self) {
        let explicit_path = self.explicit_path.take();
        *self = Self {
            explicit_path,
            ..Self::default()
        };
        self.mark_dirty();
    }

    /// Packs the mutator toggles into a ControlMsg mutator bitmask (all
    /// off = RL defaults = zero).
    pub fn mutators_mask(&self) -> u8 {
        use rocketsim_vis::proto::{
            MUT_BOOST_RECHARGE, MUT_UNLIMITED_BOOST, MUT_UNLIMITED_DOUBLE_JUMPS,
            MUT_UNLIMITED_FLIPS,
        };
        let mut mask = 0u8;
        if self.mut_unlimited_boost {
            mask |= MUT_UNLIMITED_BOOST;
        }
        if self.mut_recharge {
            mask |= MUT_BOOST_RECHARGE;
        }
        if self.mut_flips {
            mask |= MUT_UNLIMITED_FLIPS;
        }
        if self.mut_double_jumps {
            mask |= MUT_UNLIMITED_DOUBLE_JUMPS;
        }
        mask
    }
}

fn atomic_write(path: &Path, bytes: &[u8]) -> io::Result<()> {
    use std::io::Write;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT_TEMP_ID: AtomicU64 = AtomicU64::new(0);

    let (temp, mut file) = (0..100)
        .find_map(|_| {
            let id = NEXT_TEMP_ID.fetch_add(1, Ordering::Relaxed);
            let temp = path.with_extension(format!("tmp-{}-{id}", std::process::id()));
            match std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&temp)
            {
                Ok(file) => Some(Ok((temp, file))),
                Err(error) if error.kind() == io::ErrorKind::AlreadyExists => None,
                Err(error) => Some(Err(error)),
            }
        })
        .unwrap_or_else(|| {
            Err(io::Error::new(
                io::ErrorKind::AlreadyExists,
                "could not allocate a unique settings temporary file",
            ))
        })?;

    let result = (|| {
        file.write_all(bytes)?;
        file.flush()?;
        file.sync_all()?;
        std::fs::rename(&temp, path)
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&temp);
    }
    result
}

fn preserve_corrupt(path: &Path) {
    if !path.exists() {
        return;
    }
    let backup = path.with_extension("toml.corrupt");
    if !backup.exists() {
        let _ = std::fs::copy(path, backup);
    }
}

/// OS config dir + `rocketsim-vis`, or `None` when no home resolves.
fn directories_dir() -> Option<PathBuf> {
    #[cfg(target_os = "macos")]
    let base =
        std::env::var_os("HOME").map(|h| PathBuf::from(h).join("Library/Application Support"))?;
    #[cfg(target_os = "windows")]
    let base = std::env::var_os("APPDATA").map(PathBuf::from)?;
    #[cfg(not(any(target_os = "macos", target_os = "windows")))]
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("rocketsim-vis"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::sources::{InputBinding, Key, MouseButton, PadButton};

    #[test]
    fn binding_device_views_keep_original_slots_and_hide_other_devices() {
        use crate::input::{
            InputSource,
            sources::{Key, MouseButton, PadButton},
        };

        let sources = [
            InputSource::Key(Key::W),
            InputSource::PadButton(PadButton::South),
        ];
        assert_eq!(BindingDevice::All.visible_slots(sources), vec![0, 1]);
        assert_eq!(BindingDevice::KeyboardMouse.visible_slots(sources), vec![0]);
        assert_eq!(BindingDevice::Controller.visible_slots(sources), vec![1]);

        let unbound = [
            InputSource::Unbound,
            InputSource::Mouse(MouseButton::Primary),
        ];
        assert_eq!(BindingDevice::Controller.visible_slots(unbound), vec![0]);
    }

    #[test]
    fn device_reset_preserves_hidden_assignment() {
        use crate::input::{
            InputSource,
            sources::{Key, PadButton},
        };

        let mut map = ActionMap::default();
        map.set(Action::Jump, 0, InputSource::PadButton(PadButton::West));
        map.set(Action::Jump, 1, InputSource::Key(Key::Q));
        BindingDevice::KeyboardMouse.reset_action(&mut map, Action::Jump);
        assert_eq!(
            map.sources(Action::Jump)[0],
            InputSource::PadButton(PadButton::West)
        );
        assert_eq!(
            map.sources(Action::Jump)[1],
            ActionMap::default().sources(Action::Jump)[0]
        );
    }

    #[test]
    fn device_reset_keeps_empty_visible_slot_unbound() {
        use crate::input::{InputSource, sources::Key};

        let mut map = ActionMap::default();
        map.set(Action::Jump, 0, InputSource::Key(Key::K));
        map.set(Action::Jump, 1, InputSource::Unbound);
        BindingDevice::KeyboardMouse.reset_action(&mut map, Action::Jump);
        let defaults = ActionMap::default();
        assert_eq!(
            map.sources(Action::Jump)[0],
            defaults.sources(Action::Jump)[0]
        );
        assert_eq!(map.sources(Action::Jump)[1], InputSource::Unbound);
    }

    #[test]
    fn device_reset_assigns_one_default_for_two_visible_slots() {
        use crate::input::{InputSource, sources::PadButton};

        let mut map = ActionMap::default();
        map.set(Action::Jump, 0, InputSource::PadButton(PadButton::West));
        map.set(Action::Jump, 1, InputSource::PadButton(PadButton::East));
        BindingDevice::Controller.reset_action(&mut map, Action::Jump);
        let defaults = ActionMap::default();
        assert_eq!(map.sources(Action::Jump)[0], InputSource::Unbound);
        assert_eq!(
            map.sources(Action::Jump)[1],
            defaults.sources(Action::Jump)[1]
        );
    }

    #[test]
    fn round_trip() {
        let mut s = ViewerSettings::default();
        s.camera.car_cam.distance = 300.0;
        s.show_boost = false;
        let text = toml::to_string(&s).unwrap();
        let back: ViewerSettings = toml::from_str(&text).unwrap();
        assert_eq!(back.camera.car_cam.distance, 300.0);
        assert!(!back.show_boost);
        assert!(back.show_trail);
    }

    #[test]
    fn old_file_still_parses() {
        // Sparse old file (only camera keys): everything else defaults on.
        // Unknown keys are ignored, missing keys use serde defaults.
        let text = r#"
            unknown_legacy_key = true
            [camera]
            fov_degrees = 100.0
            [camera.car_cam]
            distance = 300.0
        "#;
        let back: ViewerSettings = toml::from_str(text).unwrap();
        assert_eq!(back.camera.fov_degrees, 100.0);
        assert_eq!(back.camera.car_cam.distance, 300.0);
        assert_eq!(back.camera.car_cam.stiffness, 0.45);
        assert!(back.show_trail && back.show_pads);
        assert_eq!(back.fps_limit_hz, DEFAULT_FPS_LIMIT);
    }

    #[test]
    fn bindings_default_and_old_file() {
        // Defaults copy today's hardcoded mapping.
        let s = ViewerSettings::default();
        assert_eq!(s.bindings.car.throttle_fwd, Key::W);
        assert_eq!(
            s.bindings.car.jump,
            InputBinding::Mouse(MouseButton::Primary)
        );
        assert_eq!(s.bindings.actions.kickoff_reset, Key::Backspace);
        assert_eq!(s.bindings.camera.free, Key::F);
        assert_eq!(s.bindings.ui.menu_toggle, Key::Escape);
        assert_eq!(s.bindings.pad.jump, PadButton::South);

        // Old file without `[bindings]`: bindings use defaults.
        let text = r#"
            show_boost = false
            [camera]
            fov_degrees = 100.0
        "#;
        let back: ViewerSettings = toml::from_str(text).unwrap();
        assert!(!back.show_boost);
        assert_eq!(back.bindings, Bindings::default());

        // Partial `[bindings]` table: known keys apply, rest default.
        let text = r#"
            [bindings.car]
            throttle_fwd = "ArrowUp"
            [bindings.camera]
            free = "G"
        "#;
        let back: ViewerSettings = toml::from_str(text).unwrap();
        assert_eq!(back.bindings.car.throttle_fwd, Key::ArrowUp);
        assert_eq!(back.bindings.car.throttle_back, Key::S);
        assert_eq!(back.bindings.camera.free, Key::G);
        assert_eq!(back.bindings.camera.cycle, Key::C);
    }

    #[test]
    fn setting_legacy_bindings_updates_the_canonical_map() {
        let mut settings = ViewerSettings::default();
        let mut bindings = Bindings::default();
        bindings.car.throttle_fwd = Key::ArrowUp;
        settings.set_legacy_bindings(bindings);
        assert!(settings.dirty);
        assert_eq!(
            settings.action_map().sources(Action::ThrottleForward)[0],
            InputSource::Key(Key::ArrowUp)
        );

        settings.bindings.car.throttle_fwd = Key::B;
        assert_eq!(
            settings.action_map().sources(Action::ThrottleForward)[0],
            InputSource::Key(Key::ArrowUp)
        );
    }

    #[test]
    fn universal_rebinds_survive_save_and_reload() {
        use crate::input::{Action, Axis2, InputSource};

        let path = test_path("actions");
        let mut settings = ViewerSettings::load_from_path(&path);
        let mut bindings = settings.input_bindings();
        for (action, source) in [
            (Action::Jump, InputSource::Key(Key::Escape)),
            (Action::Director, InputSource::Mouse(MouseButton::Extra2)),
            (
                Action::MenuAccept,
                InputSource::PadAxis {
                    axis: crate::input::sources::PadAxis::LeftStickY,
                    sign: -1,
                },
            ),
            (
                Action::FreeUp,
                InputSource::Wheel {
                    axis: Axis2::Y,
                    sign: 1,
                },
            ),
            (
                Action::Boost,
                InputSource::PadRaw {
                    code: 0x30000,
                    sign: 1,
                },
            ),
        ] {
            bindings.set(action, 1, source);
        }
        bindings.set(Action::ThrottleBack, 0, InputSource::Unbound);
        settings.action_bindings = Some(bindings.clone());
        settings.mark_dirty();
        settings.flush().unwrap();
        let loaded = ViewerSettings::load_from_path(&path);
        assert_eq!(loaded.action_map(), &bindings);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn old_bindings_migrate_until_action_table_is_present() {
        use crate::input::{Action, ActionMap, InputSource};

        let path = test_path("legacy-actions");
        std::fs::write(
            &path,
            "[bindings.car]\nthrottle_fwd = \"ArrowUp\"\n[bindings.pad.camera]\ndirector = \"LeftThumb\"\n",
        )
        .unwrap();
        let mut settings = ViewerSettings::load_from_path(&path);
        assert_eq!(
            settings.action_bindings,
            Some(ActionMap::from_legacy(&settings.bindings))
        );
        let migrated = settings.action_map() as *const ActionMap;
        assert_eq!(settings.action_map() as *const ActionMap, migrated);
        assert_eq!(
            settings.action_map().sources(Action::ThrottleForward)[0],
            InputSource::Key(Key::ArrowUp)
        );
        assert_eq!(
            settings.action_map().sources(Action::Director)[1],
            InputSource::PadButton(PadButton::LeftThumb)
        );

        settings.action_bindings = Some(ActionMap::default());
        let configured = settings.action_bindings.as_ref().unwrap() as *const ActionMap;
        assert_eq!(settings.action_map() as *const ActionMap, configured);
        assert_eq!(
            settings.action_map().sources(Action::ThrottleForward)[0],
            InputSource::Key(Key::W)
        );
        settings.reset_all();
        assert!(settings.action_bindings.is_none());
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn camera_config_serde() {
        let cfg = CameraConfig {
            fov_degrees: 100.0,
            ..CameraConfig::default()
        };
        let text = toml::to_string(&cfg).unwrap();
        let back: CameraConfig = toml::from_str(&text).unwrap();
        assert_eq!(back.fov_degrees, 100.0);
        assert_eq!(back.car_cam.stiffness, 0.45);
    }

    fn test_path(label: &str) -> PathBuf {
        std::env::temp_dir().join(format!(
            "rocketsim-settings-{label}-{}-{}.toml",
            std::process::id(),
            Instant::now().elapsed().as_nanos()
        ))
    }

    #[test]
    fn explicit_path_flush_replaces_atomically() {
        let path = test_path("flush");
        std::fs::write(&path, "old contents").unwrap();
        let mut settings = ViewerSettings::load_from_path(&path);
        settings.show_boost = false;
        settings.mark_dirty();
        settings.flush().unwrap();
        assert!(!ViewerSettings::load_from_path(&path).show_boost);
        assert!(!settings.dirty);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn failed_save_keeps_dirty_and_retries_after_debounce() {
        let path = test_path("retry");
        let blocker = path.with_extension("blocker");
        std::fs::write(&blocker, "not a directory").unwrap();
        let nested = blocker.join("settings.toml");
        let mut settings = ViewerSettings::load_from_path(&nested);
        settings.mark_dirty();
        assert!(settings.flush().is_err());
        assert!(settings.dirty, "failed flush must retain pending changes");
        settings.save_if_due();
        assert!(settings.dirty);
        settings.explicit_path = Some(path.clone());
        settings.save_if_due();
        assert!(
            settings.dirty,
            "failed writes must wait for the retry debounce"
        );
        settings.last_save = Some(Instant::now() - std::time::Duration::from_secs(2));
        settings.save_if_due();
        assert!(!settings.dirty);
        let _ = std::fs::remove_file(blocker);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn corrupt_settings_are_preserved_before_recovery() {
        let path = test_path("corrupt");
        let corrupt = b"not valid TOML = [";
        std::fs::write(&path, corrupt).unwrap();
        let settings = ViewerSettings::load_from_path(&path);
        assert_eq!(settings.fps_limit_hz, DEFAULT_FPS_LIMIT);
        let backup = path.with_extension("toml.corrupt");
        assert_eq!(std::fs::read(&backup).unwrap(), corrupt);
        let _ = std::fs::remove_file(path);
        let _ = std::fs::remove_file(backup);
    }

    #[test]
    fn loaded_values_are_finite_clamped_and_supported() {
        let path = test_path("validate");
        std::fs::write(&path, "ui_scale = 99.0\nmsaa = 3\n[camera]\nfov_degrees = inf\n[camera.car_cam]\ndistance = 5000.0\nstiffness = nan\n").unwrap();
        let loaded = ViewerSettings::load_from_path(&path);
        assert_eq!(loaded.camera.fov_degrees, 110.0);
        assert_eq!(loaded.camera.car_cam.distance, 1000.0);
        assert_eq!(loaded.camera.car_cam.stiffness, 0.45);
        assert_eq!(loaded.ui_scale, 2.0);
        assert_eq!(loaded.msaa, DEFAULT_MSAA);
        let _ = std::fs::remove_file(path);
    }
}
