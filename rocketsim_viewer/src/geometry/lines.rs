//! Line segments, ribbon trails, and line config.
//!
//! [`RibbonEmitter`] emits trail ribbons with testable output:
//! [`RibbonEmitter::segments`] returns segments directly. Ball/boost
//! configs keep their tuned values.

use std::collections::VecDeque;

use glam::{FloatExt, Vec3A};

/// Linear RGBA color, components 0-1.
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
    /// Use a ribbon with shared edges instead of round end caps.
    /// Adjacent ribbon segments join when their endpoint positions match.
    pub ribbon: bool,
}

impl Segment {
    /// Flat segment (same color/width both ends).
    pub fn flat(a: Vec3A, b: Vec3A, color: Color, width: f32) -> Self {
        Self {
            a,
            b,
            color_a: color,
            color_b: color,
            width_a: width,
            width_b: width,
            ribbon: false,
        }
    }

    /// Fully transparent (safe to skip at upload; output is bit-exact).
    pub fn is_invisible(&self) -> bool {
        self.color_a.a <= 0.001 && self.color_b.a <= 0.001
    }
}

/// Ball-to-ground drop line (white 0.15, width 2).
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

/// Expands a segment into 4 camera-facing quad corners (world space),
/// matching the vertex-shader math on CPU: axis interpolation,
/// view-perp offset, 1px minimum width, end-cap pads.
///
/// `eye` is the camera pos, `view` the view matrix, `proj11` is
/// `proj[1][1]`, `screen_h` the viewport height in pixels. Degenerate
/// segments give a zero-area quad (invisible — degenerate input clips away).
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

fn ribbon_offset(
    point: Vec3A,
    incoming: Vec3A,
    outgoing: Vec3A,
    width: f32,
    eye: Vec3A,
    view: &glam::Mat4,
    pixel_scale: f32,
) -> Vec3A {
    let fallback = Vec3A::from(view.row(0).truncate());
    let normal = |direction: Vec3A| {
        direction
            .cross(point - eye)
            .try_normalize()
            .unwrap_or(fallback)
    };
    let before = normal(incoming);
    let after = normal(outgoing);
    let miter = (before + after).try_normalize().unwrap_or(after);
    // Limit the miter at sharp turns. Both sides still use the same edge.
    let scale = miter.dot(after).abs().max(0.25);
    let min_width = view.transform_point3a(point).z.abs() * pixel_scale;
    let half_width = if width > 0.0 {
        width.max(min_width) * 0.5
    } else {
        0.0
    };
    miter * (half_width / scale)
}

fn expand_ribbon(
    segment: &Segment,
    previous: Option<&Segment>,
    next: Option<&Segment>,
    eye: Vec3A,
    view: &glam::Mat4,
    proj11: f32,
    screen_h: f32,
) -> [LineCorner; 4] {
    let direction = (segment.b - segment.a).normalize_or_zero();
    let incoming = previous.map_or(direction, |segment| {
        (segment.b - segment.a).normalize_or_zero()
    });
    let outgoing = next.map_or(direction, |segment| {
        (segment.b - segment.a).normalize_or_zero()
    });
    let pixel_scale = 2.0 / (proj11 * screen_h);
    let start = ribbon_offset(
        segment.a,
        incoming,
        direction,
        segment.width_a,
        eye,
        view,
        pixel_scale,
    );
    let end = ribbon_offset(
        segment.b,
        direction,
        outgoing,
        segment.width_b,
        eye,
        view,
        pixel_scale,
    );
    std::array::from_fn(|index| {
        let (side, along) = CORNERS[index];
        let (point, offset, color, width) = if along < 0.0 {
            (segment.a, start, segment.color_a, segment.width_a)
        } else {
            (segment.b, end, segment.color_b, segment.width_b)
        };
        LineCorner {
            world: point + offset * side,
            color,
            width,
        }
    })
}

/// Number of packed floats per line corner.
pub const LINE_VERTEX_FLOATS: usize = 15;

/// Reusable CPU storage for expanded line quads.
#[derive(Default)]
pub struct LineBuffer {
    /// Packed positions, endpoints, colors, widths, and ribbon flags for GPU upload.
    pub vertices: Vec<f32>,
    /// Number of visible segments included in the buffer.
    pub drawn: usize,
    /// Number of visible segments excluded by the limit.
    pub truncated: usize,
}

impl LineBuffer {
    /// Expand visible segments. Keep the allocation for the next frame.
    /// Ribbon segments share edges at matching adjacent endpoints.
    pub fn prepare(
        &mut self,
        segments: &[Segment],
        eye: Vec3A,
        view: &glam::Mat4,
        proj11: f32,
        screen_h: f32,
        limit: usize,
    ) {
        self.vertices.clear();
        self.drawn = 0;
        self.truncated = 0;
        self.vertices
            .reserve(segments.len().min(limit) * 4 * LINE_VERTEX_FLOATS);
        for (index, segment) in segments.iter().enumerate() {
            if segment.is_invisible() {
                continue;
            }
            if self.drawn == limit {
                self.truncated += 1;
                continue;
            }
            let start = segment.a.to_array();
            let end = segment.b.to_array();
            let corners = if segment.ribbon {
                let previous = index
                    .checked_sub(1)
                    .and_then(|index| segments.get(index))
                    .filter(|previous| previous.ribbon && previous.b == segment.a);
                let next = segments
                    .get(index + 1)
                    .filter(|next| next.ribbon && segment.b == next.a);
                expand_ribbon(segment, previous, next, eye, view, proj11, screen_h)
            } else {
                expand_segment(segment, eye, view, proj11, screen_h)
            };
            for corner in corners {
                self.vertices.extend_from_slice(&corner.world.to_array());
                self.vertices.extend_from_slice(&start);
                self.vertices.extend_from_slice(&end);
                self.vertices.extend_from_slice(&corner.color.to_array());
                self.vertices.push(corner.width);
                self.vertices.push(if segment.ribbon { 1.0 } else { 0.0 });
            }
            self.drawn += 1;
        }
    }
}

/// Fast `frac.powf(exp)` for the exponents the ribbon configs use:
/// 1.0 is the identity (exact for finite inputs), 0.5 is `sqrt` (within
/// 1 ulp of `powf`; ribbon outputs feed the overlay lines only, never the
/// static scene). Anything else falls back to `powf`.
fn pow_shape(frac: f32, exp: f32) -> f32 {
    if exp == 1.0 {
        frac
    } else if exp == 0.5 {
        frac.sqrt()
    } else {
        frac.powf(exp)
    }
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

/// Emission/visual config for a ribbon trail (tuned values).
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
    /// Ball trail: white 0.3→0 over 1s, width 30→0.
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

    /// Boost flames: yellow-white→orange over 0.5s.
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

/// Point-trail emitter (render-state free).
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

    /// Removes all points and resets the emission timer.
    pub fn reset(&mut self) {
        self.points.clear();
        self.time_since_emit = 0.0;
    }

    /// Removes all points without changing the emission timer.
    pub fn clear(&mut self) {
        self.points.clear();
    }

    /// Live point count (diagnostics/tests).
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Whether no points are alive.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Ribbon segments from the head (`emit_pos`) through the live points.
    /// Repeated positions share one cross-section. Disconnected points start a new strip.
    pub fn segments(&self, emit_pos: Vec3A) -> Vec<Segment> {
        let mut segs: Vec<Segment> = Vec::with_capacity(self.points.len());
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
                pow_shape(frac, self.config.color_exponent),
            );
            let cur_width = if point.connected && !is_oldest {
                f32::lerp(
                    self.config.end_width,
                    self.config.start_width,
                    pow_shape(frac, self.config.width_exponent),
                )
            } else {
                0.0
            };
            if point.connected {
                // Repeated snapshots can put several samples at the same position.
                // Keep the head attributes until the centerline moves again.
                if cur_pos == prev_pos {
                    if let Some(last) = segs.last_mut() {
                        last.color_b = cur_color;
                        last.width_b = cur_width;
                        prev_color = cur_color;
                        prev_width = cur_width;
                    }
                    continue;
                }
                segs.push(Segment {
                    a: prev_pos,
                    b: cur_pos,
                    color_a: prev_color,
                    color_b: cur_color,
                    width_a: prev_width,
                    width_b: cur_width,
                    ribbon: true,
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
    fn pow_shape_matches() {
        // 1.0 is the identity (exact), 0.5 is sqrt, anything else is powf.
        for frac in [0.0, 0.25, 0.5, 0.75, 1.0] {
            assert_eq!(pow_shape(frac, 1.0), frac);
            assert_eq!(pow_shape(frac, 0.5), frac.sqrt());
            assert_eq!(pow_shape(frac, 0.7), frac.powf(0.7));
        }
    }

    #[test]
    fn emission_rate_and_expiry() {
        // Effective rate is ~24/s (emit every 5th tick at 120Hz, hence
        // ~24/s rather than 25/s); 2s run reaches expiry steady
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
    fn segments_and_detach() {
        let mut ribbon = RibbonEmitter::new(RibbonConfig::ball_trail());
        // Emit moving points, then read segments at the head.
        for tick in 0..10 {
            ribbon.update(
                true,
                Vec3A::new(100.0 + tick as f32, 0.0, 0.0),
                Vec3A::ZERO,
                1.0 / 120.0,
            );
        }
        assert!(!ribbon.is_empty());
        let segs = ribbon.segments(Vec3A::ZERO);
        assert_eq!(segs.len(), ribbon.len(), "one segment per live point");
        assert_eq!(segs[0].a, Vec3A::ZERO, "head starts at emit pos");
        assert_eq!(segs[0].color_a, RibbonConfig::ball_trail().start_color);
        // Stop emission: the head detaches (one fewer segment than points).
        let live = ribbon.len();
        ribbon.update(false, Vec3A::ZERO, Vec3A::ZERO, 1.0 / 120.0);
        assert_eq!(ribbon.segments(Vec3A::ZERO).len(), live - 1);

        ribbon.reset();
        for _ in 0..120 {
            ribbon.update(true, Vec3A::ZERO, Vec3A::ZERO, 1.0 / 120.0);
        }
        assert!(ribbon.segments(Vec3A::ZERO).is_empty());
        let head = Vec3A::new(100.0, 0.0, 0.0);
        let segments = ribbon.segments(head);
        assert_eq!(
            segments.len(),
            1,
            "repeated samples must share one cross-section"
        );
        assert_eq!(segments[0].a, head);
        assert_eq!(segments[0].b, Vec3A::ZERO);
        assert_eq!(
            segments[0].width_b, 0.0,
            "the collapsed tail must still taper"
        );
    }

    fn test_view() -> (Vec3A, glam::Mat4, f32, f32) {
        let eye = Vec3A::new(0.0, -5000.0, 1000.0);
        let (view, proj) = crate::render::view::view_proj(
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
    fn ribbon_packing_shares_edges_through_turns_without_caps_or_width_steps() {
        let (eye, view, proj11, height) = test_view();
        let joint = Vec3A::new(0.0, 0.0, 100.0);
        let incoming = Vec3A::X;
        for outgoing in [Vec3A::X, Vec3A::Z, -Vec3A::X, (joint - eye).normalize()] {
            let mut segments = [
                Segment::flat(joint - incoming * 100.0, joint, Color::WHITE, 30.0),
                Segment::flat(joint, joint + outgoing * 100.0, Color::WHITE, 20.0),
            ];
            segments[0].width_b = 20.0;
            segments[0].color_b = Color::WHITE.with_alpha(0.2);
            segments[1].color_a = segments[0].color_b;
            segments[1].width_b = 0.0;
            for segment in &mut segments {
                segment.ribbon = true;
            }
            let mut packed = LineBuffer::default();
            packed.prepare(&segments, eye, &view, proj11, height, 2);
            assert_eq!(packed.drawn, 2);
            for side in 0..2 {
                let end = (2 + side) * LINE_VERTEX_FLOATS;
                let start = (4 + side) * LINE_VERTEX_FLOATS;
                assert_eq!(
                    &packed.vertices[end..end + 3],
                    &packed.vertices[start..start + 3]
                );
                assert_eq!(
                    &packed.vertices[end + 9..end + 14],
                    &packed.vertices[start + 9..start + 14]
                );
                let offset = Vec3A::from_slice(&packed.vertices[end..end + 3]) - joint;
                assert!(offset.is_finite() && offset.length() <= 40.001);
            }
            for vertex in packed.vertices.as_chunks::<LINE_VERTEX_FLOATS>().0 {
                assert_eq!(vertex[14], 1.0, "ribbons must not use the capsule shader");
            }
            let first = Vec3A::from_slice(&packed.vertices[..3]);
            assert!(
                (first - segments[0].a).dot(incoming).abs() < 1e-4,
                "no round cap at the head"
            );
        }
    }

    #[test]
    fn line_packing_handles_degenerate_segments_clipping_and_reuse() {
        let (eye, view, proj11, h) = test_view();
        // Degenerate segment: zero-area quad (degenerate input clips away).
        let seg = Segment::flat(
            Vec3A::new(1.0, 2.0, 3.0),
            Vec3A::new(1.0, 2.0, 3.0),
            Color::WHITE,
            5.0,
        );
        for c in expand_segment(&seg, eye, &view, proj11, h) {
            assert_eq!(c.world, seg.a);
        }
        // Hairline far away: corners still span the 1px minimum.
        let seg = Segment::flat(
            Vec3A::new(0.0, 4000.0, 0.0),
            Vec3A::new(100.0, 4000.0, 0.0),
            Color::WHITE,
            0.1,
        );
        let corners = expand_segment(&seg, eye, &view, proj11, h);
        let spread = corners
            .iter()
            .map(|c| (c.world - seg.a).length())
            .fold(0.0_f32, f32::max);
        assert!(spread > 1.0, "min-width collapsed: {spread}");
        // +/-side corner midpoints land back on the axis (quad centered).
        let seg = Segment::flat(
            Vec3A::new(-200.0, 100.0, 50.0),
            Vec3A::new(300.0, -100.0, 50.0),
            Color::WHITE,
            8.0,
        );
        let corners = expand_segment(&seg, eye, &view, proj11, h);
        let invisible = Segment::flat(seg.a, seg.b, Color::WHITE.with_alpha(0.0), 8.0);
        let segments = [invisible, seg.clone(), seg.clone()];
        let mut packed = LineBuffer::default();
        packed.prepare(&segments, eye, &view, proj11, h, 1);
        assert_eq!((packed.drawn, packed.truncated), (1, 1));
        assert_eq!(packed.vertices.len(), 4 * LINE_VERTEX_FLOATS);
        assert_eq!(&packed.vertices[..3], &corners[0].world.to_array());
        assert_eq!(&packed.vertices[3..6], &seg.a.to_array());
        assert_eq!(&packed.vertices[6..9], &seg.b.to_array());
        assert_eq!(&packed.vertices[9..13], &seg.color_a.to_array());
        assert_eq!(packed.vertices[13], seg.width_a);
        let allocation = packed.vertices.as_ptr();
        packed.prepare(&segments, eye, &view, proj11, h, 1);
        assert_eq!(packed.vertices.as_ptr(), allocation);
        packed.prepare(&segments, eye, &view, proj11, h, 0);
        assert_eq!((packed.drawn, packed.truncated), (0, 2));
        assert!(packed.vertices.is_empty());
        for along in [0, 2] {
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
