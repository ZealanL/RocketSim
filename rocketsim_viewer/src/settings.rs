//! Viewer settings: TOML persistence.
//!
//! File: `settings.toml` under `rocketsim-vis/` in the OS config dir
//! (`./settings.toml` fallback). Loading never fails (missing/corrupt
//! files give defaults); every field has a default so old files parse.

use std::{path::PathBuf, time::Instant};

use serde::{Deserialize, Serialize};

use crate::{bindings::Bindings, camera::CameraConfig};

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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ViewerSettings {
    /// Follow-camera + FOV tuning (RL defaults).
    #[serde(default)]
    pub camera: CameraConfig,
    /// Ball trail ribbon.
    #[serde(default = "on")]
    pub show_trail: bool,
    /// Per-car boost flames.
    #[serde(default = "on")]
    pub show_boost: bool,
    /// Ball-to-ground drop line.
    #[serde(default = "on")]
    pub show_ground_line: bool,
    /// Boost pads (instanced draws).
    #[serde(default = "on")]
    pub show_pads: bool,
    /// Frame-rate cap in Hz (0 = uncapped).
    #[serde(default = "default_fps")]
    pub fps_limit_hz: u32,
    /// UI scale multiplier.
    #[serde(default = "default_ui_scale")]
    pub ui_scale: f32,
    /// Vsync (applies on relaunch).
    #[serde(default = "on")]
    pub vsync: bool,
    /// MSAA samples (applies on relaunch).
    #[serde(default = "default_msaa")]
    pub msaa: u16,
    /// Key/mouse/gamepad bindings (all defaulted, old files parse).
    #[serde(default)]
    pub bindings: Bindings,
    /// Spectate-only mode: watch the followed car, send neutral inputs.
    #[serde(default)]
    pub spectate_only: bool,
    /// Drive-demo mutators, sent as ControlMsg mutator bits (all off = RL
    /// defaults). Only options the sim can apply live are listed here.
    #[serde(default)]
    pub mut_unlimited_boost: bool,
    #[serde(default)]
    pub mut_recharge: bool,
    #[serde(default)]
    pub mut_flips: bool,
    #[serde(default)]
    pub mut_double_jumps: bool,
    /// Last save time (not persisted).
    #[serde(skip)]
    last_save: Option<Instant>,
    /// Unsaved changes pending (not persisted).
    #[serde(skip)]
    dirty: bool,
}

fn on() -> bool {
    true
}

fn default_fps() -> u32 {
    DEFAULT_FPS_LIMIT
}

fn default_ui_scale() -> f32 {
    DEFAULT_UI_SCALE
}

fn default_msaa() -> u16 {
    DEFAULT_MSAA
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
            spectate_only: false,
            mut_unlimited_boost: false,
            mut_recharge: false,
            mut_flips: false,
            mut_double_jumps: false,
            last_save: None,
            dirty: false,
        }
    }
}

impl ViewerSettings {
    /// Settings file path (`rocketsim-vis/settings.toml` in the OS config
    /// dir, `./settings.toml` fallback).
    pub fn path() -> PathBuf {
        let dir = directories_dir().unwrap_or_else(|| PathBuf::from("."));
        dir.join("settings.toml")
    }

    /// Loads from disk, or defaults (with a stderr note) when missing or
    /// corrupt. Unknown fields are ignored; missing fields use defaults.
    pub fn load() -> Self {
        let path = Self::path();
        match std::fs::read_to_string(&path) {
            Ok(text) => match toml::from_str::<Self>(&text) {
                Ok(settings) => settings,
                Err(e) => {
                    eprintln!(
                        "rocketsim_vis: corrupt {} ({e}), using defaults",
                        path.display()
                    );
                    Self::default()
                }
            },
            Err(_) => Self::default(),
        }
    }

    /// Writes to disk, creating parent dirs. Errors go to stderr (never fatal).
    pub fn save(&self) {
        let path = Self::path();
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        match toml::to_string(self) {
            Ok(text) => {
                if let Err(e) = std::fs::write(&path, text) {
                    eprintln!("rocketsim_vis: cannot save {} ({e})", path.display());
                }
            }
            Err(e) => eprintln!("rocketsim_vis: cannot serialize settings ({e})"),
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
            self.save();
            self.last_save = Some(Instant::now());
            self.dirty = false;
        }
    }

    /// Resets camera tuning to RL defaults (preset button).
    pub fn reset_camera(&mut self) {
        self.camera = CameraConfig::default();
        self.mark_dirty();
    }

    /// Resets everything to defaults.
    pub fn reset_all(&mut self) {
        *self = Self::default();
        self.mark_dirty();
    }

    /// Packs the mutator toggles into a ControlMsg mutator bitmask (all
    /// off = RL defaults = zero).
    pub fn mutators_mask(&self) -> u8 {
        use crate::proto::{
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
    use crate::bindings::{Key, MouseButton, PadButton};

    #[test]
    fn defaults_match_spec() {
        let s = ViewerSettings::default();
        assert_eq!(s.camera.fov_degrees, 110.0);
        assert_eq!(s.camera.car_cam.distance, 270.0);
        assert!(s.show_trail && s.show_boost && s.show_ground_line && s.show_pads);
        assert_eq!(s.fps_limit_hz, 60);
        assert_eq!(s.ui_scale, 1.0);
        assert!(s.vsync);
        assert_eq!(s.msaa, 4);
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
        assert_eq!(s.bindings.car.jump, MouseButton::Primary);
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
    fn camera_config_serde() {
        // CameraConfig must survive TOML (it lives inside settings).
        let cfg = CameraConfig {
            fov_degrees: 100.0,
            ..CameraConfig::default()
        };
        let text = toml::to_string(&cfg).unwrap();
        let back: CameraConfig = toml::from_str(&text).unwrap();
        assert_eq!(back.fov_degrees, 100.0);
        assert_eq!(back.car_cam.stiffness, 0.45);
    }
}
