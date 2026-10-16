//! Free camera: a position plus a look direction.
//!
//! Mouse looks ([`FreeCam::look`]), WASD flies on the view-relative frame
//! ([`FreeCam::fly`]), Space/Shift move in world Z, wheel dollies
//! ([`FreeCam::dolly`]). Pose ([`FreeCam::pos_dir`]) derives from
//! `pos`/`yaw`/`pitch` each call, with a floor clamp and a position box.
//! No hidden time state: huge or non-finite `dt` only shrinks one step.
//! No wall-clock, no RNG.
//!
//! `yaw` is the bearing from +X toward +Y (+Y is `+90°`); `pitch` is `+`
//! looking up, clamped to `±MAX_PITCH_RAD`. Screen-right is
//! `(sin yaw, −cos yaw, 0)` (level form, valid at all pitches).

use std::f32::consts::{FRAC_PI_2, PI};

use glam::Vec3A;

use crate::camera::{ArenaBounds, FREECAM_BACKOFF, clamp_dt};

/// Default position (uu): south stands, high up, framing most of Soccar.
pub const DEFAULT_POS: Vec3A = Vec3A::new(0.0, -6062.0, 3600.0);
/// Default look yaw (rad): due north (+Y), toward field center.
pub const DEFAULT_YAW_RAD: f32 = FRAC_PI_2;
/// Default look pitch (rad): down at the center circle from the stands.
pub const DEFAULT_PITCH_RAD: f32 = -30.0 * PI / 180.0;
/// Look-pitch clamp (rad): just short of straight up/down.
pub const MAX_PITCH_RAD: f32 = 89.0 * PI / 180.0;
/// Fly speed (uu/s) at full (normalized) input.
pub const MOVE_SPEED: f32 = 3000.0;
/// Position clamp half-extent in X (uu): Soccar/Dropshot + margin.
pub const POS_X_LIM: f32 = 7000.0;
/// Position clamp half-extent in Y (uu): Soccar + margin.
pub const POS_Y_LIM: f32 = 7000.0;
/// Highest the camera may fly (uu).
pub const POS_Z_MAX: f32 = 8000.0;
/// Camera never goes below this height (uu; same as the follow view).
pub const CAM_MIN_Z: f32 = 30.0;

/// FPS-style free camera: `pos` + look `(yaw, pitch)`.
///
/// `yaw = 0` looks along +X; `yaw = +90°` looks along +Y. `pitch = 0` is
/// level, `+` looks up. See the module docs for the full scheme.
///
/// ```rust
/// use rocketsim_viewer::camera::FreeCam;
///
/// let mut cam = FreeCam::new();
/// cam.look(0.1, 0.05); // mouse deltas, already scaled by sensitivity
/// cam.fly(1.0, 0.0, 0.0, 1.0 / 60.0); // forward one tick
/// cam.dolly(100.0); // wheel step along the view
/// let (pos, dir) = cam.pos_dir();
/// assert!((dir.length() - 1.0).abs() < 1e-4);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct FreeCam {
    pos: Vec3A,
    yaw: f32,
    pitch: f32,
}

impl FreeCam {
    /// Default overview: south stands, looking north-down at center.
    pub fn new() -> Self {
        Self {
            pos: DEFAULT_POS,
            yaw: DEFAULT_YAW_RAD,
            pitch: DEFAULT_PITCH_RAD,
        }
    }

    /// Builds a pose directly; yaw wraps to `[−π, π]`, pitch clamps to
    /// `±MAX_PITCH_RAD`, position clamps to the box, non-finite inputs fall
    /// back to defaults.
    pub fn with_pose(pos: Vec3A, yaw: f32, pitch: f32) -> Self {
        let mut cam = Self::new();
        cam.pos = clamp_pos(pos);
        cam.yaw = wrap_yaw(sanitize(yaw, DEFAULT_YAW_RAD));
        cam.pitch = sanitize(pitch, DEFAULT_PITCH_RAD).clamp(-MAX_PITCH_RAD, MAX_PITCH_RAD);
        cam
    }

    /// Restores the default overview pose.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Sets the position directly (box-clamped); used after containment
    /// slides a [`fly`](Self::fly)/[`dolly`](Self::dolly) step.
    pub fn set_pos(&mut self, pos: Vec3A) {
        self.pos = clamp_pos(pos);
    }

    /// Current camera position (uu).
    pub fn pos(&self) -> Vec3A {
        self.pos
    }

    /// Current look yaw (rad, `[−π, π]`).
    pub fn yaw(&self) -> f32 {
        self.yaw
    }

    /// Current look pitch (rad, `+` = up).
    pub fn pitch(&self) -> f32 {
        self.pitch
    }

    /// Rotates the view by `d_yaw`/`d_pitch` radians (caller scales mouse
    /// pixels; the viewer passes `-dx*s` for yaw and `-dy*s` for pitch).
    /// Yaw wraps, pitch clamps; non-finite deltas are ignored.
    pub fn look(&mut self, d_yaw: f32, d_pitch: f32) {
        if d_yaw.is_finite() {
            self.yaw = wrap_yaw(self.yaw + d_yaw);
        }
        if d_pitch.is_finite() {
            self.pitch = (self.pitch + d_pitch).clamp(-MAX_PITCH_RAD, MAX_PITCH_RAD);
        }
    }

    /// Flies view-relative: `forward` (W) along the LEVEL view bearing
    /// (never changes height — pitch is ignored), `right` (D)
    /// screen-right, `up` (Space/Shift) in world Z. Only Space/Shift move
    /// vertically. Inputs clamp to `[-1, 1]` and normalize when longer
    /// than 1; `dt` clamps to `[0, 1/30]`.
    pub fn fly(&mut self, forward: f32, right: f32, up: f32, dt: f32) {
        let dt = clamp_dt(dt);
        let f = sanitize(forward, 0.0).clamp(-1.0, 1.0);
        let r = sanitize(right, 0.0).clamp(-1.0, 1.0);
        let u = sanitize(up, 0.0).clamp(-1.0, 1.0);
        let (fwd, rgt) = frame(self.yaw, self.pitch);
        // Level flight: drop the pitch component so W/D never climb.
        let fwd_level = Vec3A::new(fwd.x, fwd.y, 0.0)
            .try_normalize()
            .unwrap_or(Vec3A::new(self.yaw.cos(), self.yaw.sin(), 0.0));
        let mut v = fwd_level * f + rgt * r + Vec3A::Z * u;
        let speed = v.length();
        if dt <= 0.0 || speed == 0.0 {
            self.pos = clamp_pos(self.pos);
            return;
        }
        if speed > 1.0 {
            v /= speed;
        }
        self.pos = clamp_pos(self.pos + v * MOVE_SPEED * dt);
    }

    /// Dollies `delta` uu along the view direction (caller scales wheel
    /// steps); position clamps to the box. Non-finite input is ignored.
    pub fn dolly(&mut self, delta: f32) {
        if delta.is_finite() && delta != 0.0 {
            let (fwd, _) = frame(self.yaw, self.pitch);
            self.pos = clamp_pos(self.pos + fwd * delta);
        }
    }

    /// [`fly`](Self::fly) plus wall sliding: when `bounds` is present and
    /// the pre-move position is inside the hull, the step slides along
    /// walls instead of passing through. Moves starting outside pass
    /// through untouched, so a camera staged outside (e.g. the default
    /// stands overview) can still fly in; once inside it stays inside.
    pub fn fly_bounded(
        &mut self,
        forward: f32,
        right: f32,
        up: f32,
        dt: f32,
        bounds: Option<&ArenaBounds>,
    ) {
        let prev = self.pos;
        self.fly(forward, right, up, dt);
        if let Some(b) = bounds
            && b.contains(prev)
        {
            self.pos = clamp_pos(b.slide_move(prev, self.pos, FREECAM_BACKOFF));
        }
    }

    /// [`dolly`](Self::dolly) plus wall sliding (same contract as
    /// [`fly_bounded`](Self::fly_bounded)).
    pub fn dolly_bounded(&mut self, delta: f32, bounds: Option<&ArenaBounds>) {
        let prev = self.pos;
        self.dolly(delta);
        if let Some(b) = bounds
            && b.contains(prev)
        {
            self.pos = clamp_pos(b.slide_move(prev, self.pos, FREECAM_BACKOFF));
        }
    }

    /// Derives `(position, direction)` from the current state. Direction is
    /// unit by construction; position carries the box clamp.
    pub fn pos_dir(&self) -> (Vec3A, Vec3A) {
        let (fwd, _) = frame(self.yaw, self.pitch);
        (clamp_pos(self.pos), fwd)
    }
}

impl Default for FreeCam {
    fn default() -> Self {
        Self::new()
    }
}

/// View-relative frame: unit look direction + unit screen-right (level
/// form, well-defined at all pitches).
fn frame(yaw: f32, pitch: f32) -> (Vec3A, Vec3A) {
    let cp = pitch.cos();
    let fwd = Vec3A::new(cp * yaw.cos(), cp * yaw.sin(), pitch.sin());
    let rgt = Vec3A::new(yaw.sin(), -yaw.cos(), 0.0);
    (fwd, rgt)
}

fn sanitize(v: f32, fallback: f32) -> f32 {
    if v.is_finite() { v } else { fallback }
}

fn wrap_yaw(yaw: f32) -> f32 {
    let two_pi = 2.0 * PI;
    let w = yaw.rem_euclid(two_pi);
    if w > PI { w - two_pi } else { w }
}

fn clamp_pos(p: Vec3A) -> Vec3A {
    Vec3A::new(
        sanitize(p.x, 0.0).clamp(-POS_X_LIM, POS_X_LIM),
        sanitize(p.y, 0.0).clamp(-POS_Y_LIM, POS_Y_LIM),
        sanitize(p.z, DEFAULT_POS.z).clamp(CAM_MIN_Z, POS_Z_MAX),
    )
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_4;

    use super::*;

    fn approx(a: Vec3A, b: Vec3A, eps: f32) -> bool {
        (a - b).length() <= eps
    }

    #[test]
    fn look_math_exact_level() {
        // Yaw 0 looks +X; yaw +90° looks +Y.
        let cam = FreeCam::with_pose(Vec3A::new(100.0, 200.0, 300.0), 0.0, 0.0);
        let (pos, dir) = cam.pos_dir();
        assert!(
            approx(pos, Vec3A::new(100.0, 200.0, 300.0), 1e-3),
            "{pos:?}"
        );
        assert!(approx(dir, Vec3A::new(1.0, 0.0, 0.0), 1e-5), "{dir:?}");

        let cam = FreeCam::with_pose(Vec3A::new(100.0, 200.0, 300.0), FRAC_PI_2, 0.0);
        let (_, dir) = cam.pos_dir();
        assert!(approx(dir, Vec3A::new(0.0, 1.0, 0.0), 1e-5), "{dir:?}");

        // Pitch +45° looks up-forward.
        let cam = FreeCam::with_pose(Vec3A::ZERO, 0.0, FRAC_PI_2 / 2.0);
        let (_, dir) = cam.pos_dir();
        let c = (FRAC_PI_2 / 2.0).cos();
        let s = (FRAC_PI_2 / 2.0).sin();
        assert!(approx(dir, Vec3A::new(c, 0.0, s), 1e-5), "{dir:?}");
        assert!((dir.length() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn look_signs_match_mouse() {
        // Facing +X: a yaw DECREASE (what the viewer sends for a rightward
        // mouse: -dx) swings the view toward -Y, which is screen-right.
        let mut cam = FreeCam::with_pose(Vec3A::ZERO, 0.0, 0.0);
        cam.look(-0.1, 0.0);
        let (_, dir) = cam.pos_dir();
        assert!(dir.y < 0.0, "mouse-right must look screen-right: {dir:?}");

        // A pitch INCREASE (viewer sends -dy, so mouse-up is +) looks up.
        cam = FreeCam::with_pose(Vec3A::ZERO, 0.0, 0.0);
        cam.look(0.0, 0.1);
        let (_, dir) = cam.pos_dir();
        assert!(dir.z > 0.0, "mouse-up must look up: {dir:?}");
    }

    #[test]
    fn default_overview_is_sane() {
        let cam = FreeCam::new();
        assert_eq!(cam.pos(), DEFAULT_POS);
        assert!((cam.yaw() - DEFAULT_YAW_RAD).abs() < 1e-6);
        let (pos, dir) = cam.pos_dir();
        // South stands, above the field, looking north-down at center.
        assert!(pos.y < 0.0);
        assert!(pos.z > 1000.0);
        assert!(dir.y > 0.0 && dir.z < 0.0);
        assert!((dir.length() - 1.0).abs() < 1e-5);
        assert!(pos.is_finite() && dir.is_finite());
    }

    #[test]
    fn yaw_wraps_and_pitch_clamps() {
        let mut cam = FreeCam::new();
        cam.look(100.0 * PI, 0.0);
        assert!(cam.yaw() >= -PI && cam.yaw() <= PI);

        cam.look(0.0, 10.0);
        assert!((cam.pitch() - MAX_PITCH_RAD).abs() < 1e-6);
        cam.look(0.0, -10.0);
        assert!((cam.pitch() + MAX_PITCH_RAD).abs() < 1e-6);

        // Non-finite deltas are ignored, never poison the state.
        let before = cam;
        cam.look(f32::NAN, f32::INFINITY);
        cam.fly(f32::NAN, f32::NAN, f32::NAN, f32::NAN);
        cam.dolly(f32::NAN);
        assert_eq!(cam.yaw(), before.yaw());
        assert_eq!(cam.pitch(), before.pitch());
        let (pos, dir) = cam.pos_dir();
        assert!(pos.is_finite() && dir.is_finite());
        assert!((dir.length() - 1.0).abs() < 1e-4);
    }

    #[test]
    fn fly_moves_view_relative() {
        // Facing +Y: W (+forward) pushes +Y, D (+right) pushes +X (the
        // screen-right side), Space (+up) rises.
        let mut cam = FreeCam::with_pose(Vec3A::ZERO, FRAC_PI_2, 0.0);
        let p0 = cam.pos();
        cam.fly(1.0, 0.0, 0.0, 1.0 / 60.0);
        assert!(((cam.pos() - p0) - Vec3A::new(0.0, MOVE_SPEED / 60.0, 0.0)).length() < 1e-2);

        cam = FreeCam::with_pose(Vec3A::ZERO, FRAC_PI_2, 0.0);
        cam.fly(0.0, 1.0, 0.0, 1.0 / 60.0);
        assert!(((cam.pos() - p0) - Vec3A::new(MOVE_SPEED / 60.0, 0.0, 0.0)).length() < 1e-2);

        cam = FreeCam::with_pose(Vec3A::ZERO, FRAC_PI_2, 0.0);
        cam.fly(0.0, 0.0, 1.0, 1.0 / 60.0);
        assert!(((cam.pos() - p0) - Vec3A::new(0.0, 0.0, MOVE_SPEED / 60.0)).length() < 1e-2);

        // Pitched down 45°: W/D still fly level (no height change);
        // only Space/Shift move vertically.
        cam = FreeCam::with_pose(Vec3A::new(0.0, 0.0, 500.0), FRAC_PI_2, -FRAC_PI_4);
        let p0 = cam.pos();
        cam.fly(1.0, 1.0, 0.0, 1.0 / 60.0);
        assert!((cam.pos().z - p0.z).abs() < 1e-2);
        assert!((cam.pos().y - p0.y) > 0.0 && (cam.pos().x - p0.x) > 0.0);

        // Combined inputs move at the same unit speed (normalized).
        cam = FreeCam::new();
        let p0 = cam.pos();
        cam.fly(1.0, 1.0, 1.0, 1.0 / 30.0);
        assert!(((cam.pos() - p0).length() - MOVE_SPEED / 30.0).abs() < 1e-2);
    }

    #[test]
    fn dolly_tracks_view_and_clamps() {
        // (0,0,0) builds at the floor clamp (z=30); dolly runs +X level.
        let mut cam = FreeCam::with_pose(Vec3A::ZERO, 0.0, 0.0);
        cam.dolly(500.0);
        assert!((cam.pos() - Vec3A::new(500.0, 0.0, CAM_MIN_Z)).length() < 1e-2);

        cam.dolly(1e9);
        assert_eq!(cam.pos().x, POS_X_LIM);

        // Construction clamps too, including NaN -> fallback.
        let cam = FreeCam::with_pose(Vec3A::new(1e9, -1e9, -50.0), f32::NAN, f32::INFINITY);
        assert_eq!(cam.pos().x, POS_X_LIM);
        assert_eq!(cam.pos().y, -POS_Y_LIM);
        assert_eq!(cam.pos().z, CAM_MIN_Z);
        assert!((cam.yaw() - DEFAULT_YAW_RAD).abs() < 1e-6);
        assert!((cam.pitch() - DEFAULT_PITCH_RAD).abs() < 1e-6);

        // Pitched straight down at the floor: flying never goes under.
        let mut cam = FreeCam::with_pose(Vec3A::new(0.0, 0.0, 100.0), 0.0, -MAX_PITCH_RAD);
        for _ in 0..60 {
            cam.fly(1.0, 0.0, 0.0, 1.0 / 30.0);
        }
        assert!(cam.pos().z >= CAM_MIN_Z);
        let (pos, dir) = cam.pos_dir();
        assert!(pos.is_finite());
        assert!((dir.length() - 1.0).abs() < 1e-4);
    }

    #[test]
    fn huge_dt_stays_finite_and_bounded() {
        let mut cam = FreeCam::new();
        let p0 = cam.pos();
        cam.fly(1.0, 1.0, 1.0, 10.0);
        // One clamped step only: travel <= MOVE_SPEED * MAX_DT (+fp slack).
        assert!((cam.pos() - p0).length() <= MOVE_SPEED * (1.0 / 30.0) + 1e-2);
        let (pos, dir) = cam.pos_dir();
        assert!(pos.is_finite() && dir.is_finite());
        assert!((dir.length() - 1.0).abs() < 1e-4);
    }

    #[test]
    fn bounded_fly_stops_at_wall_but_enters_from_outside() {
        use rocketsim::GameMode;

        use crate::camera::{ArenaBounds, FREECAM_BACKOFF};
        crate::photo::init_meshes();
        let bounds = ArenaBounds::build(GameMode::Soccar);

        // Inside, facing +X into the wall: the step stops one backoff
        // short instead of passing through.
        let mut cam = FreeCam::with_pose(Vec3A::new(3800.0, 0.0, 500.0), 0.0, 0.0);
        for _ in 0..120 {
            cam.fly_bounded(1.0, 0.0, 0.0, 1.0 / 30.0, Some(&bounds));
        }
        assert!(
            (cam.pos().x - (4096.0 - FREECAM_BACKOFF)).abs() < 30.0,
            "freecam should rest against the wall: {:?}",
            cam.pos()
        );
        assert!(cam.pos().x < 4096.0);

        // Outside (default stands overview), diving at the roof: entry is
        // allowed — containment only blocks exits, never entries.
        let mut cam = FreeCam::new();
        assert!(!bounds.contains(cam.pos()));
        let before = cam.pos();
        for _ in 0..120 {
            cam.fly_bounded(0.0, 0.0, -1.0, 1.0 / 30.0, Some(&bounds));
        }
        assert!(
            cam.pos().z < before.z - 100.0,
            "outside camera must be free to descend: {:?}",
            cam.pos()
        );
    }
}
