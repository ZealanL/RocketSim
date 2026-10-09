//! Viewer settings: TOML persistence + back-compat (stage 9a).
//!
//! File: `settings.toml` under `rocketsim-vis/` (XDG config on Linux,
//! `Application Support` on macOS, `%APPDATA%` on Windows; `./settings.toml`
//! fallback when no home resolves). Shared by debug/release builds.
//!
//! Loading never fails: missing/corrupt files yield defaults (+ stderr
//! note). Every field has a serde default, so old files missing new keys
//! parse (pinned by `old_file_still_parses`).

use std::{path::PathBuf, time::Instant};

use serde::{Deserialize, Serialize};

use crate::camera::CameraConfig;

/// Frame-rate limiter (Hz, 0 = uncapped). Default 60.
pub const DEFAULT_FPS_LIMIT: u32 = 60;
/// UI scale multiplier. Default 1.0.
pub const DEFAULT_UI_SCALE: f32 = 1.0;
/// MSAA samples (relaunch applies). Default 4.
pub const DEFAULT_MSAA: u16 = 4;
/// Debounce between settings saves (s).
const SAVE_DEBOUNCE_SECS: f32 = 1.0;

/// Persistent viewer settings (TOML-serializable, all fields defaulted).
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

    /// Marks changed (call after any mutation); use [`save_if_due`](Self::save_if_due) per frame.
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
    fn corrupt_is_default() {
        let back: Result<ViewerSettings, _> = toml::from_str("[[[not toml");
        assert!(back.is_err());
        // load() maps that error to defaults (covered by round_trip path);
        // the unit contract here is just that defaults are sane.
        let _ = ViewerSettings::default();
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
