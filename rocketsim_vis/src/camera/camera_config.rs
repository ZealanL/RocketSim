//! Camera tuning (stage 8a): RL-authentic follow values.
//!
//! `distance`/`height` place the camera boom behind the car on the
//! horizontal bearing; `angle_deg` is the car-cam look pitch; `fov_degrees`
//! is **horizontal** FOV (RL convention, converted at projection time).
//! `stiffness` drives speed zoom-out and car-cam yaw lag (ball-cam yaw is
//! always rigid); `transition_speed` drives the toggle swing time.

use glam::Vec3A;
use serde::{Deserialize, Serialize};

/// RL default boom length (uu).
pub const D_DISTANCE: f32 = 270.0;
/// RL default focus height (uu).
pub const D_HEIGHT: f32 = 100.0;
/// RL default car-cam pitch (deg).
pub const D_ANGLE_DEG: f32 = -3.0;
/// RL default stiffness.
pub const D_STIFFNESS: f32 = 0.45;
/// RL default toggle rate.
pub const D_TRANSITION: f32 = 1.2;
/// RL default horizontal FOV (deg).
pub const D_FOV: f32 = 110.0;
/// RL default birds-eye position (uu).
pub const D_BIRDS_EYE: Vec3A = Vec3A::new(-3000.0, 0.0, 1500.0);

/// Tuning for the follow-car camera.
#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
pub struct CarCameraConfig {
    /// Boom length behind the car in uu (before speed zoom). Default: 270.
    pub distance: f32,
    /// Focus height above the car in uu. Default: 100.
    pub height: f32,
    /// Car-cam look pitch in degrees (negative = down). Default: -3.0.
    ///
    /// Ball-cam looks exactly at the ball; this only shapes car-cam.
    pub angle_deg: f32,
    /// Zoom-out + yaw-lag stiffness (0-1). Higher = tighter. Default: 0.45.
    pub stiffness: f32,
    /// Ball-cam toggle swing rate (1-2). Higher = snappier. Default: 1.2.
    pub transition_speed: f32,
}

impl Default for CarCameraConfig {
    fn default() -> Self {
        Self {
            distance: 270.0,
            height: 100.0,
            angle_deg: -3.0,
            stiffness: 0.45,
            transition_speed: 1.2,
        }
    }
}

/// Top-level camera settings.
#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraConfig {
    /// Horizontal field of view in degrees. Default: 110.
    pub fov_degrees: f32,
    /// Fixed birds-eye camera position in uu. Default: (-3000, 0, 1500).
    pub birds_eye_pos: Vec3A,
    /// Follow-car tuning.
    pub car_cam: CarCameraConfig,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            fov_degrees: 110.0,
            birds_eye_pos: Vec3A::new(-3000.0, 0.0, 1500.0),
            car_cam: CarCameraConfig::default(),
        }
    }
}

/// Named camera preset (one-click tuning; values persist like sliders).
///
/// The pro presets are transcribed from published pro settings (see each
/// variant's docs). Only fields our follow camera models get applied: boom
/// (`distance`/`height`/`angle_deg`), horizontal `fov`, `stiffness`, and
/// `transition_speed`. Publish-only fields — swivel speed, camera shake,
/// ball-cam mode, sensitivities — have no viewer equivalent and are ignored
/// (their published values are noted in the docs for traceability, never
/// invented).
///
/// `Close`/`Far` are legacy generic presets kept so the existing viewer menu
/// keeps compiling; prefer the pro presets for anything user-facing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraPreset {
    /// RL defaults: 270/100/-3°/110°, stiffness 0.45, transition 1.2.
    Default,
    /// Legacy generic tighter boom: 200/80/-5°/100°.
    ///
    /// Leaves stiffness/transition untouched. Kept for menu/API compat.
    Close,
    /// Legacy generic longer boom: 350/130/-1°/110°.
    ///
    /// Leaves stiffness/transition untouched. Kept for menu/API compat.
    /// (FOV was 120.0, which RL itself caps at 110 — clamped to 110.)
    Far,
    /// ApparentlyJack: 270/100/-3.0°/110°, stiffness 0.35, transition 1.80.
    ///
    /// Source: Liquipedia "List of player camera settings" (ApparentlyJack
    /// row, updated 2025-11-10). Older snapshots differ (BLAST.tv news
    /// 2025-04-29 lists stiffness 0.40 / transition 1.70; gamingcfg 2024
    /// lists angle -5.0) — the newest Liquipedia row is used. Swivel 2.20,
    /// shake off, ball-cam toggle are not applied.
    ApparentlyJack,
    /// BeastMode: 270/90/-4.0°/109°, stiffness 0.45, transition 1.20.
    ///
    /// Sources agree: Liquipedia "List of player camera settings"
    /// (BeastMode row, updated 2025-09-14), BLAST.tv player profile, and
    /// gamingcfg 2024 all list the same boom/FOV/stiffness/transition.
    /// Swivel 7.00, shake off, ball-cam toggle are not applied.
    BeastMode,
    /// Squishy (SquishyMuffinz): 270/90/-5.0°/110°, stiffness 0.35,
    /// transition 1.80.
    ///
    /// Source: Liquipedia "SquishyMuffinz" camera-settings row. Note:
    /// gamer-guides "SquishyMuffinz Settings (2026)" lists a different
    /// snapshot (260/100/-4.0°, transition 1.30) — the Liquipedia row is
    /// used. Swivel 7.00, shake off, ball-cam toggle are not applied.
    Squishy,
    /// Vatira: 270/90/-5.0°/110°, stiffness 0.35, transition 1.50.
    ///
    /// Source: BLAST.tv "Vatira Rocket League controls and camera settings"
    /// (2025-06-25). Note: the BLAST.tv player profile snapshot lists
    /// swivel 8.5 / transition 1.7 instead of 7.10 / 1.50 — the dated news
    /// article is used. Swivel, shake off, ball-cam toggle are not applied.
    Vatira,
    /// zen (Team Vitality): 270/100/-3.0°/110°, stiffness 0.35,
    /// transition 1.40.
    ///
    /// Sources: Liquipedia "zen" (updated 2026-02-12) and gamer-guides
    /// "zen Settings" (2025-08-03). Note: BLAST.tv "zen Rocket League
    /// settings" (2025-06-21) and several 2024/2025 roundups list
    /// transition 1.00 instead of 1.40 — the newest Liquipedia value is
    /// used. Swivel 4.00, shake off, ball-cam toggle are not applied.
    Zen,
}

impl CameraPreset {
    /// Every preset in viewer-menu order: RL default first, then pros
    /// alphabetically, then legacy generics last. Viewer menus should
    /// iterate this instead of hardcoding names.
    pub const ALL: &'static [CameraPreset] = &[
        CameraPreset::Default,
        CameraPreset::ApparentlyJack,
        CameraPreset::BeastMode,
        CameraPreset::Squishy,
        CameraPreset::Vatira,
        CameraPreset::Zen,
        CameraPreset::Close,
        CameraPreset::Far,
    ];

    /// Short button label for viewer menus.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Default => "Default",
            Self::ApparentlyJack => "ApparentlyJack",
            Self::BeastMode => "BeastMode",
            Self::Squishy => "Squishy",
            Self::Vatira => "Vatira",
            Self::Zen => "Zen",
            Self::Close => "Close",
            Self::Far => "Far",
        }
    }

    /// Boom tuning: `(distance_uu, height_uu, angle_deg)`.
    pub const fn boom(self) -> (f32, f32, f32) {
        match self {
            Self::Default => (D_DISTANCE, D_HEIGHT, D_ANGLE_DEG),
            Self::Close => (200.0, 80.0, -5.0),
            Self::Far => (350.0, 130.0, -1.0),
            Self::ApparentlyJack => (270.0, 100.0, -3.0),
            Self::BeastMode => (270.0, 90.0, -4.0),
            Self::Squishy => (270.0, 90.0, -5.0),
            Self::Vatira => (270.0, 90.0, -5.0),
            Self::Zen => (270.0, 100.0, -3.0),
        }
    }

    /// Horizontal FOV in degrees (RL convention).
    pub const fn fov(self) -> f32 {
        match self {
            Self::Default => D_FOV,
            Self::Close => 100.0,
            Self::Far => 110.0,
            Self::ApparentlyJack => 110.0,
            Self::BeastMode => 109.0,
            Self::Squishy => 110.0,
            Self::Vatira => 110.0,
            Self::Zen => 110.0,
        }
    }

    /// Stiffness to apply, or `None` to leave the current value untouched
    /// (legacy presets predate stiffness support).
    pub const fn stiffness(self) -> Option<f32> {
        match self {
            Self::Default => Some(D_STIFFNESS),
            Self::Close | Self::Far => None,
            Self::ApparentlyJack => Some(0.35),
            Self::BeastMode => Some(0.45),
            Self::Squishy => Some(0.35),
            Self::Vatira => Some(0.35),
            Self::Zen => Some(0.35),
        }
    }

    /// Transition speed to apply, or `None` to leave the current value
    /// untouched (legacy presets predate transition support).
    pub const fn transition_speed(self) -> Option<f32> {
        match self {
            Self::Default => Some(D_TRANSITION),
            Self::Close | Self::Far => None,
            Self::ApparentlyJack => Some(1.80),
            Self::BeastMode => Some(1.20),
            Self::Squishy => Some(1.80),
            Self::Vatira => Some(1.50),
            Self::Zen => Some(1.40),
        }
    }

    /// Applies the preset to a follow config (boom + FOV, plus stiffness /
    /// transition for presets that define them; the rest is untouched).
    pub fn apply(self, cfg: &mut CarCameraConfig, fov: &mut f32) {
        let (distance, height, angle_deg) = self.boom();
        cfg.distance = distance;
        cfg.height = height;
        cfg.angle_deg = angle_deg;
        *fov = self.fov();
        if let Some(stiffness) = self.stiffness() {
            cfg.stiffness = stiffness;
        }
        if let Some(transition_speed) = self.transition_speed() {
            cfg.transition_speed = transition_speed;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rl_defaults() {
        let cfg = CameraConfig::default();
        assert_eq!(cfg.fov_degrees, 110.0);
        assert_eq!(cfg.birds_eye_pos, Vec3A::new(-3000.0, 0.0, 1500.0));
        assert_eq!(cfg.car_cam.distance, 270.0);
        assert_eq!(cfg.car_cam.height, 100.0);
        assert_eq!(cfg.car_cam.angle_deg, -3.0);
        assert_eq!(cfg.car_cam.stiffness, 0.45);
        assert_eq!(cfg.car_cam.transition_speed, 1.2);
    }

    #[test]
    fn presets_apply_exactly() {
        // (preset, distance, height, angle_deg, fov).
        let boom_fov = [
            (CameraPreset::Default, (270.0, 100.0, -3.0, 110.0)),
            (CameraPreset::Close, (200.0, 80.0, -5.0, 100.0)),
            (CameraPreset::Far, (350.0, 130.0, -1.0, 110.0)),
            (CameraPreset::ApparentlyJack, (270.0, 100.0, -3.0, 110.0)),
            (CameraPreset::BeastMode, (270.0, 90.0, -4.0, 109.0)),
            (CameraPreset::Squishy, (270.0, 90.0, -5.0, 110.0)),
            (CameraPreset::Vatira, (270.0, 90.0, -5.0, 110.0)),
            (CameraPreset::Zen, (270.0, 100.0, -3.0, 110.0)),
        ];
        for (preset, (distance, height, angle_deg, fov)) in boom_fov {
            let mut cfg = CameraConfig::default();
            preset.apply(&mut cfg.car_cam, &mut cfg.fov_degrees);
            assert_eq!(
                (
                    cfg.car_cam.distance,
                    cfg.car_cam.height,
                    cfg.car_cam.angle_deg,
                    cfg.fov_degrees
                ),
                (distance, height, angle_deg, fov),
                "{preset:?}"
            );
            // Accessors agree with `apply`.
            assert_eq!(preset.boom(), (distance, height, angle_deg));
            assert_eq!(preset.fov(), fov);
        }

        // Stiffness / transition: Default + pro presets set researched
        // values; legacy presets leave the current values untouched.
        let feel = [
            (CameraPreset::Default, Some((0.45, 1.2))),
            (CameraPreset::Close, None),
            (CameraPreset::Far, None),
            (CameraPreset::ApparentlyJack, Some((0.35, 1.80))),
            (CameraPreset::BeastMode, Some((0.45, 1.20))),
            (CameraPreset::Squishy, Some((0.35, 1.80))),
            (CameraPreset::Vatira, Some((0.35, 1.50))),
            (CameraPreset::Zen, Some((0.35, 1.40))),
        ];
        for (preset, expected) in feel {
            // Sentinel values prove legacy presets don't touch the fields.
            let mut cfg = CameraConfig::default();
            cfg.car_cam.stiffness = 0.77;
            cfg.car_cam.transition_speed = 1.91;
            preset.apply(&mut cfg.car_cam, &mut cfg.fov_degrees);
            match expected {
                Some((stiffness, transition_speed)) => {
                    assert_eq!(cfg.car_cam.stiffness, stiffness, "{preset:?}");
                    assert_eq!(cfg.car_cam.transition_speed, transition_speed, "{preset:?}");
                }
                None => {
                    assert_eq!(cfg.car_cam.stiffness, 0.77, "{preset:?}");
                    assert_eq!(cfg.car_cam.transition_speed, 1.91, "{preset:?}");
                }
            }
            assert_eq!(preset.stiffness(), expected.map(|(s, _)| s));
            assert_eq!(preset.transition_speed(), expected.map(|(_, t)| t));
        }

        // Default restores everything.
        let mut cfg = CameraConfig::default();
        CameraPreset::Zen.apply(&mut cfg.car_cam, &mut cfg.fov_degrees);
        CameraPreset::Default.apply(&mut cfg.car_cam, &mut cfg.fov_degrees);
        assert_eq!(cfg, CameraConfig::default());
    }

    #[test]
    fn preset_ranges_valid() {
        // Engine/pro ranges: distance 100-400, height 40-200,
        // angle -15..=5, FOV 100-115, stiffness 0-1, transition 1-2.
        for &preset in CameraPreset::ALL {
            let mut cfg = CameraConfig::default();
            preset.apply(&mut cfg.car_cam, &mut cfg.fov_degrees);
            assert!(
                (100.0..=400.0).contains(&cfg.car_cam.distance),
                "{preset:?} distance {}",
                cfg.car_cam.distance
            );
            assert!(
                (40.0..=200.0).contains(&cfg.car_cam.height),
                "{preset:?} height {}",
                cfg.car_cam.height
            );
            assert!(
                (-15.0..=5.0).contains(&cfg.car_cam.angle_deg),
                "{preset:?} angle {}",
                cfg.car_cam.angle_deg
            );
            assert!(
                (100.0..=115.0).contains(&cfg.fov_degrees),
                "{preset:?} fov {}",
                cfg.fov_degrees
            );
            assert!(
                (0.0..=1.0).contains(&cfg.car_cam.stiffness),
                "{preset:?} stiffness {}",
                cfg.car_cam.stiffness
            );
            assert!(
                (1.0..=2.0).contains(&cfg.car_cam.transition_speed),
                "{preset:?} transition {}",
                cfg.car_cam.transition_speed
            );
        }
    }

    #[test]
    fn preset_menu_table_covers_all_variants() {
        // Every variant appears exactly once; Default leads the menu.
        assert_eq!(CameraPreset::ALL.len(), 8);
        assert_eq!(CameraPreset::ALL[0], CameraPreset::Default);
        let mut names: Vec<&str> = CameraPreset::ALL.iter().map(|p| p.name()).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), CameraPreset::ALL.len());
        for preset in CameraPreset::ALL {
            assert!(!preset.name().is_empty(), "{preset:?}");
        }
    }
}
