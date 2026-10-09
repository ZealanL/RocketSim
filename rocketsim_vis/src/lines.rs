//! Line segments, ribbon trails, and line config (stage 7a).
//!
//! Ports the old `RibbonEmitter` (rate emission, advection, expiry,
//! oldest-fade) with headless-testable output: [`RibbonEmitter::segments`]
//! returns line segments directly instead of pushing into a render state.
//! Configs (ball trail, boost) are the old values verbatim.

use std::collections::VecDeque;

use glam::{FloatExt, Vec3A};

/// Linear RGBA color, components 0-1 (old `backend::Color` subset).
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Color {
    /// Red channel (0-1).
    pub r: f32,
    /// Green channel (0-1).
    pub g: f32,
    /// Blue channel (0-1).
    pub b: f32,
    /// Alpha channel (0-1, 0 = transparent).
    pub a: f32,
}

impl Color {
    /// Opaque white.
    pub const WHITE: Self = Self::new_rgb(1.0, 1.0, 1.0);

    /// Creates a color from 0-1 RGBA components.
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Creates an opaque color from 0-1 RGB components.
    pub const fn new_rgb(r: f32, g: f32, b: f32) -> Self {
        Self::new(r, g, b, 1.0)
    }

    /// Copies the color with a new alpha value.
    pub const fn with_alpha(&self, alpha: f32) -> Self {
        Self::new(self.r, self.g, self.b, alpha)
    }

    /// Linearly interpolates to `other` by `t` (0-1).
    pub fn lerp(self, other: Self, t: f32) -> Self {
        let it = 1.0 - t;
        Self::new(
            self.r * it + other.r * t,
            self.g * it + other.g * t,
            self.b * it + other.b * t,
            self.a * it + other.a * t,
        )
    }

    /// Array form for GL upload.
    pub fn to_array(self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }
}

/// One line segment: endpoints with per-end color and width.
#[derive(Debug, Clone)]
pub struct Segment {
    /// Start point (world uu).
    pub a: Vec3A,
    /// End point (world uu).
    pub b: Vec3A,
    /// Start color.
    pub color_a: Color,
    /// End color.
    pub color_b: Color,
    /// Start width (uu).
    pub width_a: f32,
    /// End width (uu).
    pub width_b: f32,
}

impl Segment {
    /// Flat segment (same color/width both ends), like old `add_line_simple`.
    pub fn flat(a: Vec3A, b: Vec3A, color: Color, width: f32) -> Self {
        Self {
            a,
            b,
            color_a: color,
            color_b: color,
            width_a: width,
            width_b: width,
        }
    }

    /// Fully transparent (blending no-op at every pixel — safe to skip at
    /// upload with bit-exact output, whatever the widths).
    pub fn is_invisible(&self) -> bool {
        self.color_a.a <= 0.001 && self.color_b.a <= 0.001
    }
}

/// Ball-to-ground drop line, as the old vis drew it (white 0.15, width 2).
pub fn ground_line(ball_pos: Vec3A, floor_z: f32) -> Segment {
    Segment::flat(
        ball_pos,
        Vec3A::new(ball_pos.x, ball_pos.y, floor_z),
        Color::WHITE.with_alpha(0.15),
        2.0,
    )
}

/// One expanded quad corner: world position plus pre-interpolated color
/// and width (the GPU only transforms and varies them through).
#[derive(Debug, Clone, Copy)]
pub struct LineCorner {
    /// Camera-facing quad corner (world uu).
    pub world: Vec3A,
    /// Interpolated color at this corner.
    pub color: Color,
    /// Interpolated width at this corner (uu, for the frag discard test).
    pub width: f32,
}

/// Corner (side, along) signs in fixed order: (-,-), (+,-), (-,+), (+,+).
pub const CORNERS: [(f32, f32); 4] = [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)];

/// Size margin on expansion (superset guards float mismatch; ~1% overdraw).
const EXPAND_MARGIN: f32 = 1.1;

/// Expands a segment into its 4 camera-facing quad corners (world space),
/// replicating the old vertex-shader math on CPU: axis interpolation,
/// view-perp offset, 1px minimum width, end-cap pads.
///
/// `eye` is the camera world position, `view` the view matrix (for
/// camera-plane distance), `proj11` is `proj[1][1]`, `screen_h` the
/// viewport height in pixels. Degenerate segments yield a zero-area quad
/// (invisible, matching the old NaN-clipped behavior).
pub fn expand_segment(
    seg: &Segment,
    eye: Vec3A,
    view: &glam::Mat4,
    proj11: f32,
    screen_h: f32,
) -> [LineCorner; 4] {
    let axis_len2 = (seg.b - seg.a).length_squared();
    if axis_len2 < 1e-12 {
        let c = LineCorner {
            world: seg.a,
            color: seg.color_a,
            width: seg.width_a,
        };
        return [c; 4];
    }
    let line_dir = (seg.b - seg.a) / axis_len2.sqrt();
    let mut out = [LineCorner {
        world: seg.a,
        color: seg.color_a,
        width: seg.width_a,
    }; 4];
    for (i, (side, along)) in CORNERS.iter().enumerate() {
        let frac = (along + 1.0) / 2.0;
        let axis = seg.a.lerp(seg.b, frac);
        let w = seg.width_a + (seg.width_b - seg.width_a) * frac;
        let view_pos = view.transform_point3a(axis);
        let dist = view_pos.z.abs();
        let minw = (2.0 * dist) / (proj11 * screen_h);
        let cw = w.max(minw);
        let view_dir = axis - eye;
        let view_dir = if view_dir.length_squared() > 1e-12 {
            view_dir.normalize()
        } else {
            line_dir
        };
        let mut perp = line_dir.cross(view_dir);
        perp = if perp.length_squared() > 1e-6 {
            perp.normalize()
        } else {
            Vec3A::Z
        };
        let pad = line_dir * (*along * cw);
        let world = axis + pad + perp * (*side * (cw + minw) * 0.5);
        // Margin about the axis (superset, never clips).
        let world = axis + (world - axis) * EXPAND_MARGIN;
        out[i] = LineCorner {
            world,
            color: seg.color_a.lerp(seg.color_b, frac),
            width: w,
        };
    }
    out
}

#[derive(Debug, Copy, Clone)]
struct RibbonPoint {
    pos: Vec3A,
    vel: Vec3A,
    time_alive: f32,
    lifetime: f32,
    connected: bool,
}

impl RibbonPoint {
    fn is_alive(&self) -> bool {
        self.time_alive < self.lifetime
    }

    fn time_left(&self) -> f32 {
        (self.lifetime - self.time_alive).max(0.0)
    }

    fn time_left_frac(&self) -> f32 {
        self.time_left() / self.lifetime
    }
}

/// Emission/visual config for a ribbon trail (old values verbatim).
#[derive(Debug, Copy, Clone)]
pub struct RibbonConfig {
    /// Points per second.
    pub emit_rate: f32,
    /// Seconds each point lives.
    pub emit_lifetime: f32,
    /// Head color.
    pub start_color: Color,
    /// Tail color.
    pub end_color: Color,
    /// Head width (uu).
    pub start_width: f32,
    /// Tail width (uu).
    pub end_width: f32,
    /// Width fade curve.
    pub width_exponent: f32,
    /// Color fade curve.
    pub color_exponent: f32,
}

impl RibbonConfig {
    /// Ball trail (old vis values): white 0.3→0 over 1s, width 30→0.
    pub const fn ball_trail() -> Self {
        Self {
            emit_rate: 30.0,
            emit_lifetime: 1.0,
            start_color: Color::new(1.0, 1.0, 1.0, 0.3),
            end_color: Color::new(1.0, 1.0, 1.0, 0.0),
            start_width: 30.0,
            end_width: 0.0,
            width_exponent: 0.5,
            color_exponent: 1.0,
        }
    }

    /// Boost flames (old vis values): yellow-white→orange over 0.5s.
    pub const fn boost() -> Self {
        Self {
            emit_rate: 30.0,
            emit_lifetime: 0.5,
            start_color: Color::new(1.0, 1.0, 0.5, 1.0),
            end_color: Color::new(1.0, 0.4, 0.2, 0.0),
            start_width: 20.0,
            end_width: 0.0,
            width_exponent: 0.7,
            color_exponent: 1.0,
        }
    }
}

/// Point-trail emitter (old `RibbonEmitter`, render-state free).
#[derive(Debug, Clone)]
pub struct RibbonEmitter {
    config: RibbonConfig,
    points: VecDeque<RibbonPoint>,
    time_since_emit: f32,
}

impl RibbonEmitter {
    /// Creates an emitter with `config` and no points.
    pub fn new(config: RibbonConfig) -> Self {
        Self {
            config,
            points: VecDeque::new(),
            time_since_emit: 0.0,
        }
    }

    /// Advances time; emits while `can_emit` at the configured rate.
    pub fn update(&mut self, can_emit: bool, emit_pos: Vec3A, emit_vel: Vec3A, dt: f32) {
        if self.time_since_emit < (1.0 / self.config.emit_rate) {
            self.time_since_emit += dt;
        } else if can_emit {
            self.time_since_emit = 0.0;
            self.points.push_front(RibbonPoint {
                pos: emit_pos,
                vel: emit_vel,
                time_alive: 0.0,
                lifetime: self.config.emit_lifetime,
                connected: true,
            });
        }

        if !can_emit && let Some(first) = self.points.front_mut() {
            // Emission stopped: disconnect the head so the trail detaches.
            first.connected = false;
        }

        for point in self.points.iter_mut() {
            point.pos += point.vel * dt;
            point.time_alive += dt;
        }

        while let Some(last) = self.points.back() {
            if !last.is_alive() {
                self.points.pop_back();
            } else {
                break;
            }
        }
    }

    /// Live point count (diagnostics/tests).
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Whether no points are alive.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Line segments from the head (`emit_pos`) through all live points.
    /// Oldest point fades toward the head; disconnected heads emit nothing.
    pub fn segments(&self, emit_pos: Vec3A) -> Vec<Segment> {
        let mut segs = Vec::new();
        if self.points.is_empty() {
            return segs;
        }
        let mut prev_pos = emit_pos;
        let mut prev_color = self.config.start_color;
        let mut prev_width = self.config.start_width;
        for (i, point) in self.points.iter().enumerate() {
            let is_oldest = i == self.points.len() - 1;
            let frac = point.time_left_frac();
            let cur_pos = if is_oldest {
                prev_pos.lerp(point.pos, frac)
            } else {
                point.pos
            };
            let cur_color = self.config.end_color.lerp(
                self.config.start_color,
                frac.powf(self.config.color_exponent),
            );
            let cur_width = if point.connected && !is_oldest {
                f32::lerp(
                    self.config.end_width,
                    self.config.start_width,
                    frac.powf(self.config.width_exponent),
                )
            } else {
                0.0
            };
            if point.connected {
                segs.push(Segment {
                    a: prev_pos,
                    b: cur_pos,
                    color_a: prev_color,
                    color_b: cur_color,
                    width_a: prev_width,
                    width_b: cur_width,
                });
            }
            prev_pos = cur_pos;
            prev_color = cur_color;
            prev_width = cur_width;
        }
        segs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn color_lerp() {
        let c = Color::new(0.0, 0.0, 0.0, 0.0).lerp(Color::new(1.0, 1.0, 1.0, 1.0), 0.5);
        assert_eq!(c, Color::new(0.5, 0.5, 0.5, 0.5));
        assert_eq!(Color::WHITE.with_alpha(0.15).a, 0.15);
    }

    #[test]
    fn ground_line_shape() {
        let seg = ground_line(Vec3A::new(10.0, 20.0, 93.0), 0.0);
        assert_eq!(seg.a, Vec3A::new(10.0, 20.0, 93.0));
        assert_eq!(seg.b, Vec3A::new(10.0, 20.0, 0.0));
        assert_eq!(seg.width_a, 2.0);
        assert!((seg.color_a.a - 0.15).abs() < 1e-6);
    }

    #[test]
    fn emission_rate_and_expiry() {
        // Effective rate is ~24/s (emit every 5th tick at 120Hz — same
        // off-by-one the old emitter had); 2s run reaches expiry steady
        // state (~24 alive with a 1s lifetime).
        let mut ribbon = RibbonEmitter::new(RibbonConfig::ball_trail());
        for _ in 0..240 {
            ribbon.update(true, Vec3A::ZERO, Vec3A::ZERO, 1.0 / 120.0);
        }
        assert!((20..=28).contains(&ribbon.len()), "len={}", ribbon.len());
        // Another lifetime with no emission: all expire.
        for _ in 0..150 {
            ribbon.update(false, Vec3A::ZERO, Vec3A::ZERO, 1.0 / 120.0);
        }
        assert_eq!(ribbon.len(), 0);
        assert!(ribbon.segments(Vec3A::ZERO).is_empty());
    }

    #[test]
    fn segments_follow_points() {
        let mut ribbon = RibbonEmitter::new(RibbonConfig::ball_trail());
        // Emit one point ahead, then read segments at the head.
        for _ in 0..10 {
            ribbon.update(true, Vec3A::new(100.0, 0.0, 0.0), Vec3A::ZERO, 1.0 / 120.0);
        }
        assert!(!ribbon.is_empty());
        let segs = ribbon.segments(Vec3A::ZERO);
        assert_eq!(segs.len(), ribbon.len(), "one segment per live point");
        assert_eq!(segs[0].a, Vec3A::ZERO, "head starts at emit pos");
        assert_eq!(segs[0].color_a, RibbonConfig::ball_trail().start_color);
    }

    #[test]
    fn stopped_emission_detaches() {
        let mut ribbon = RibbonEmitter::new(RibbonConfig::ball_trail());
        for _ in 0..60 {
            ribbon.update(true, Vec3A::ZERO, Vec3A::ZERO, 1.0 / 120.0);
        }
        let live = ribbon.len();
        assert!(!ribbon.is_empty());
        ribbon.update(false, Vec3A::ZERO, Vec3A::ZERO, 1.0 / 120.0);
        // Head disconnected → one fewer segment than points.
        assert_eq!(ribbon.segments(Vec3A::ZERO).len(), live - 1);
    }

    #[test]
    fn boost_config_values() {
        let c = RibbonConfig::boost();
        assert_eq!((c.emit_rate, c.emit_lifetime), (30.0, 0.5));
        assert_eq!((c.start_width, c.end_width), (20.0, 0.0));
    }

    fn test_view() -> (Vec3A, glam::Mat4, f32, f32) {
        let eye = Vec3A::new(0.0, -5000.0, 1000.0);
        let (view, proj) = crate::view::view_proj(
            glam::Vec3::new(eye.x, eye.y, eye.z),
            glam::Vec3::ZERO,
            90.0,
            16.0 / 9.0,
        );
        (eye, view, proj.y_axis.y, 720.0)
    }

    #[test]
    fn expand_covers_endpoints() {
        let seg = Segment::flat(
            Vec3A::new(0.0, 0.0, 100.0),
            Vec3A::new(500.0, 0.0, 100.0),
            Color::WHITE,
            10.0,
        );
        let (eye, view, proj11, h) = test_view();
        let corners = expand_segment(&seg, eye, &view, proj11, h);
        // Bounding box of corners must contain both endpoints (superset).
        let (mut mnx, mut mxx, mut mny, mut mxy) = (f32::MAX, f32::MIN, f32::MAX, f32::MIN);
        for c in corners {
            mnx = mnx.min(c.world.x);
            mxx = mxx.max(c.world.x);
            mny = mny.min(c.world.y);
            mxy = mxy.max(c.world.y);
        }
        assert!(mnx <= 0.0 && mxx >= 500.0, "x-range {mnx}..{mxx}");
        assert!(mny <= 0.0 && mxy >= 0.0, "y-range {mny}..{mxy}");
        // Colors interpolate head->tail across the along axis.
        assert_eq!(corners[0].color, seg.color_a);
    }

    #[test]
    fn expand_degenerate_is_zero_area() {
        let seg = Segment::flat(
            Vec3A::new(1.0, 2.0, 3.0),
            Vec3A::new(1.0, 2.0, 3.0),
            Color::WHITE,
            5.0,
        );
        let (eye, view, proj11, h) = test_view();
        let corners = expand_segment(&seg, eye, &view, proj11, h);
        for c in corners {
            assert_eq!(c.world, seg.a);
        }
    }

    #[test]
    fn expand_respects_min_width() {
        // Hairline far away: corners must still span the 1px minimum.
        let seg = Segment::flat(
            Vec3A::new(0.0, 4000.0, 0.0),
            Vec3A::new(100.0, 4000.0, 0.0),
            Color::WHITE,
            0.1,
        );
        let (eye, view, proj11, h) = test_view();
        let corners = expand_segment(&seg, eye, &view, proj11, h);
        let spread = corners
            .iter()
            .map(|c| (c.world - seg.a).length())
            .fold(0.0_f32, f32::max);
        assert!(spread > 1.0, "min-width collapsed: {spread}");
    }

    #[test]
    fn expand_is_symmetric_about_axis() {
        // +side/-side corner midpoints land back on the axis (margin scales
        // about the axis, so the quad stays centered).
        let seg = Segment::flat(
            Vec3A::new(-200.0, 100.0, 50.0),
            Vec3A::new(300.0, -100.0, 50.0),
            Color::WHITE,
            8.0,
        );
        let (eye, view, proj11, h) = test_view();
        let corners = expand_segment(&seg, eye, &view, proj11, h);
        for along in [0, 2] {
            // Pair midpoints sit on the axis line (end-cap pads shift along
            // it; margin scales symmetrically about it).
            let mid = (corners[along].world + corners[along + 1].world) * 0.5;
            let frac = along as f32 / 2.0;
            let axis = seg.a.lerp(seg.b, frac);
            let dir = (seg.b - seg.a).normalize();
            assert!(
                (mid - axis).cross(dir).length() < 1e-3,
                "quad off-axis: {mid:?} vs {axis:?}"
            );
        }
    }
}
