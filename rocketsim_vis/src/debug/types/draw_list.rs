use glam::Vec3A;

use super::{
    error::{DebugError, DebugResult},
    items::{
        DebugItem, InlineMeshItem, LabelItem, LabelPos, LineItem, MeshInstanceItem, PointItem,
        PolylineItem, RectBorder, RectItem, TextAlign,
    },
    limits::MAX_ITEMS_PER_GROUP,
    style::Color,
    transform::{DepthMode, Width},
};

// ---------------------------------------------------------------------------
// DrawList + builder helpers
// ---------------------------------------------------------------------------

/// An ordered list of drawing items for one group.
///
/// Builder methods append items and return the list for chaining. They do not
/// validate input. Call [`validate`](Self::validate) before use; submission
/// also validates the list.
///
/// # Example
///
/// ```
/// use rocketsim_vis::debug::{Color, DebugItem, DepthMode, DrawList, Width};
///
/// let mut draw = DrawList::new();
/// draw.line([0.0, 0.0, 0.0], [100.0, 0.0, 0.0],
///     Color::RED, Width::pixels(2.0), DepthMode::Test);
/// assert_eq!(draw.len(), 1);
/// assert!(matches!(draw.items[0], DebugItem::Line(_)));
/// ```
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DrawList {
    /// Items in draw order before renderer batching.
    pub items: Vec<DebugItem>,
}

impl DrawList {
    /// Create an empty draw list.
    #[must_use]
    pub fn new() -> Self {
        Self { items: Vec::new() }
    }

    /// Create an empty list with capacity for at most [`MAX_ITEMS_PER_GROUP`] items.
    ///
    /// This limits the initial allocation, not the number of later pushes.
    /// Submission rejects lists that exceed the item limit.
    #[must_use]
    pub fn with_capacity(n: usize) -> Self {
        Self {
            items: Vec::with_capacity(n.min(MAX_ITEMS_PER_GROUP)),
        }
    }

    /// Return the number of items.
    #[must_use]
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// Return `true` if the list has no items.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Clear all items.
    pub fn clear(&mut self) {
        self.items.clear();
    }

    /// Push a prebuilt item.
    pub fn push(&mut self, item: DebugItem) -> &mut Self {
        self.items.push(item);
        self
    }

    /// Append a line between two group-local points in Unreal units.
    pub fn line(
        &mut self,
        start: [f32; 3],
        end: [f32; 3],
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        self.items.push(DebugItem::Line(LineItem {
            start,
            end,
            color,
            width,
            depth,
        }));
        self
    }

    /// Append a connected line strip. Set `closed` to join the last point to the first.
    pub fn polyline(
        &mut self,
        points: Vec<[f32; 3]>,
        color: Color,
        width: Width,
        depth: DepthMode,
        closed: bool,
    ) -> &mut Self {
        self.items.push(DebugItem::Polyline(PolylineItem {
            points,
            color,
            width,
            depth,
            closed,
        }));
        self
    }

    /// Append a square point sprite at a group-local position in Unreal units.
    pub fn point(
        &mut self,
        pos: [f32; 3],
        color: Color,
        size: Width,
        depth: DepthMode,
    ) -> &mut Self {
        self.items.push(DebugItem::Point(PointItem {
            pos,
            color,
            size,
            depth,
        }));
        self
    }

    /// Append an inline indexed triangle mesh in group-local coordinates.
    ///
    /// Supply one color per vertex. Optional normals and UV coordinates must
    /// match the vertex count. Indices form triangles and must be in range.
    /// `texture` names an existing reusable texture. See [`InlineMeshItem`]
    /// for validation rules. Prefer a [`MeshResource`](crate::debug::MeshResource)
    /// and [`Self::instance`] when several items share the same geometry.
    #[allow(clippy::too_many_arguments)]
    pub fn mesh(
        &mut self,
        positions: Vec<[f32; 3]>,
        colors: Vec<Color>,
        normals: Option<Vec<[f32; 3]>>,
        uvs: Option<Vec<[f32; 2]>>,
        indices: Vec<u32>,
        texture: Option<String>,
        depth: DepthMode,
    ) -> &mut Self {
        self.items.push(DebugItem::Mesh(InlineMeshItem {
            positions,
            colors,
            normals,
            uvs,
            indices,
            texture,
            depth,
        }));
        self
    }

    /// Append an instance of an existing reusable mesh.
    ///
    /// Apply scale, then row-major rotation, then translation in Unreal
    /// units. `tint` multiplies vertex colors. `texture` optionally names a
    /// reusable texture; the mesh resource does not store a texture itself.
    /// See the [module example](crate::debug) for efficient shared geometry.
    #[allow(clippy::too_many_arguments)]
    pub fn instance(
        &mut self,
        mesh: impl Into<String>,
        position: [f32; 3],
        rotation: [[f32; 3]; 3],
        scale: [f32; 3],
        tint: Color,
        texture: Option<String>,
        depth: DepthMode,
    ) -> &mut Self {
        self.items.push(DebugItem::Instance(MeshInstanceItem {
            mesh: mesh.into(),
            position,
            rotation,
            scale,
            tint,
            texture,
            depth,
        }));
        self
    }

    /// Append a screen label at viewport fractions, with a top-left origin.
    ///
    /// `size` uses viewer logical points. See [`LabelItem`] for size limits
    /// and [`LabelPos::Screen`] for coordinates outside the viewport.
    pub fn label_screen(
        &mut self,
        text: impl Into<String>,
        pos: [f32; 2],
        size: f32,
        align: TextAlign,
        color: Color,
        bg: Option<Color>,
    ) -> &mut Self {
        self.items.push(DebugItem::Label(LabelItem {
            text: text.into(),
            pos: LabelPos::Screen(pos),
            size,
            align,
            color,
            bg,
        }));
        self
    }

    /// Append a projected label at a group-local position in Unreal units.
    ///
    /// `size` uses viewer logical points. The label remains an overlay and
    /// is not hidden by world geometry. See [`LabelItem`].
    pub fn label_world(
        &mut self,
        text: impl Into<String>,
        pos: [f32; 3],
        size: f32,
        align: TextAlign,
        color: Color,
        bg: Option<Color>,
    ) -> &mut Self {
        self.items.push(DebugItem::Label(LabelItem {
            text: text.into(),
            pos: LabelPos::World(pos),
            size,
            align,
            color,
            bg,
        }));
        self
    }

    /// Append a screen rectangle using viewport fractions.
    ///
    /// The viewer clips the rectangle to the viewport. `texture` must name
    /// an existing resource. See [`RectItem`] and [`RectBorder`] for details.
    pub fn rect(
        &mut self,
        min: [f32; 2],
        max: [f32; 2],
        fill: Color,
        border: Option<RectBorder>,
        texture: Option<String>,
    ) -> &mut Self {
        self.items.push(DebugItem::Rect(RectItem {
            min,
            max,
            fill,
            border,
            texture,
        }));
        self
    }

    // -- shape helpers (at least box / sphere / arrows / grid) -------------

    /// Append 12 line items for an axis-aligned box.
    ///
    /// `center` and per-axis half-extents `half` use group-local Unreal units.
    pub fn box_edges(
        &mut self,
        center: [f32; 3],
        half: [f32; 3],
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        let c = center;
        let h = half;
        let v = [
            [c[0] - h[0], c[1] - h[1], c[2] - h[2]],
            [c[0] + h[0], c[1] - h[1], c[2] - h[2]],
            [c[0] + h[0], c[1] + h[1], c[2] - h[2]],
            [c[0] - h[0], c[1] + h[1], c[2] - h[2]],
            [c[0] - h[0], c[1] - h[1], c[2] + h[2]],
            [c[0] + h[0], c[1] - h[1], c[2] + h[2]],
            [c[0] + h[0], c[1] + h[1], c[2] + h[2]],
            [c[0] - h[0], c[1] + h[1], c[2] + h[2]],
        ];
        for &(a, b) in &[
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ] {
            self.line(v[a], v[b], color, width, depth);
        }
        self
    }

    /// Append one inline mesh for an axis-aligned box.
    ///
    /// `center` and per-axis half-extents `half` use group-local Unreal units.
    /// The mesh has 8 vertices, 12 triangles, and one color for all vertices.
    pub fn box_solid(
        &mut self,
        center: [f32; 3],
        half: [f32; 3],
        color: Color,
        depth: DepthMode,
    ) -> &mut Self {
        let c = center;
        let h = half;
        let positions = vec![
            [c[0] - h[0], c[1] - h[1], c[2] - h[2]],
            [c[0] + h[0], c[1] - h[1], c[2] - h[2]],
            [c[0] + h[0], c[1] + h[1], c[2] - h[2]],
            [c[0] - h[0], c[1] + h[1], c[2] - h[2]],
            [c[0] - h[0], c[1] - h[1], c[2] + h[2]],
            [c[0] + h[0], c[1] - h[1], c[2] + h[2]],
            [c[0] + h[0], c[1] + h[1], c[2] + h[2]],
            [c[0] - h[0], c[1] + h[1], c[2] + h[2]],
        ];
        let indices = vec![
            0, 1, 2, 0, 2, 3, // -z
            4, 6, 5, 4, 7, 6, // +z
            0, 4, 5, 0, 5, 1, // -y
            2, 6, 7, 2, 7, 3, // +y
            0, 3, 7, 0, 7, 4, // -x
            1, 5, 6, 1, 6, 2, // +x
        ];
        self.mesh(positions, vec![color; 8], None, None, indices, None, depth)
    }

    /// Append a closed circle polyline in the plane of `axis_u` and `axis_v`.
    ///
    /// Use perpendicular unit axes for a circular result. The helper does
    /// not normalize them. `center` and `radius` use group-local Unreal units.
    /// Restrict the segment count to `8..=128` before generating points.
    #[allow(clippy::too_many_arguments)]
    pub fn circle(
        &mut self,
        center: [f32; 3],
        radius: f32,
        axis_u: [f32; 3],
        axis_v: [f32; 3],
        segments: usize,
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        let n = segments.clamp(8, 128);
        let center = Vec3A::from_array(center);
        let axis_u = Vec3A::from_array(axis_u);
        let axis_v = Vec3A::from_array(axis_v);
        let mut points = Vec::with_capacity(n);
        for i in 0..n {
            let t = (i as f32) / (n as f32) * std::f32::consts::TAU;
            let (s, c) = t.sin_cos();
            points.push((center + (axis_u * c + axis_v * s) * radius).to_array());
        }
        self.polyline(points, color, width, depth, true)
    }

    /// Wireframe sphere (3 great circles).
    pub fn sphere_lines(
        &mut self,
        center: [f32; 3],
        radius: f32,
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        self.circle(
            center,
            radius,
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            32,
            color,
            width,
            depth,
        );
        self.circle(
            center,
            radius,
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            32,
            color,
            width,
            depth,
        );
        self.circle(
            center,
            radius,
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            32,
            color,
            width,
            depth,
        );
        self
    }

    /// Append one solid sphere mesh at a group-local center, in Unreal units.
    ///
    /// Use 12 rings and 24 segments: 325 vertices and 576 triangles. Include
    /// one color for all vertices, with no explicit normals or UV coordinates.
    pub fn sphere_solid(
        &mut self,
        center: [f32; 3],
        radius: f32,
        color: Color,
        depth: DepthMode,
    ) -> &mut Self {
        let rings = 12usize;
        let segments = 24usize;
        let mut positions = Vec::with_capacity((rings + 1) * (segments + 1));
        for r in 0..=rings {
            let theta = (r as f32) / (rings as f32) * std::f32::consts::PI;
            let (st, ct) = theta.sin_cos();
            for s in 0..=segments {
                let phi = (s as f32) / (segments as f32) * std::f32::consts::TAU;
                let (sp, cp) = phi.sin_cos();
                positions.push([
                    center[0] + radius * st * cp,
                    center[1] + radius * st * sp,
                    center[2] + radius * ct,
                ]);
            }
        }
        let mut indices = Vec::with_capacity(rings * segments * 6);
        let row = segments + 1;
        for r in 0..rings {
            for s in 0..segments {
                let a = (r * row + s) as u32;
                let b = (r * row + s + 1) as u32;
                let c = ((r + 1) * row + s) as u32;
                let d = ((r + 1) * row + s + 1) as u32;
                indices.extend_from_slice(&[a, c, b, b, c, d]);
            }
        }
        let colors = vec![color; positions.len()];
        self.mesh(positions, colors, None, None, indices, None, depth)
    }

    /// Append an arrow shaft and wireframe head in group-local Unreal units.
    ///
    /// Add one shaft and eight head edges if the direction is finite and at
    /// least `0.001` units long. Otherwise, append only the shaft. Submission
    /// still rejects non-finite endpoints. Head length is 20 percent of shaft
    /// length, restricted to `5..=40` units.
    pub fn arrow(
        &mut self,
        start: [f32; 3],
        end: [f32; 3],
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        self.line(start, end, color, width, depth);
        let start_vec = Vec3A::from_array(start);
        let end_vec = Vec3A::from_array(end);
        let dir = end_vec - start_vec;
        let len = dir.length();
        if !len.is_finite() || len < 1e-3 {
            return self;
        }
        let d = dir / len;
        let head_len = (len * 0.2).clamp(5.0, 40.0);
        let head_r = head_len * 0.4;
        // Pick a stable perpendicular.
        let up = if d.z.abs() < 0.9 { Vec3A::Z } else { Vec3A::Y };
        let u = d.cross(up);
        let u = u / u.length().max(1e-6);
        let v = d.cross(u);
        let base = end_vec - d * head_len;
        let corners = [
            (base + (u + v) * head_r).to_array(),
            (base + (u - v) * head_r).to_array(),
            (base + (-u - v) * head_r).to_array(),
            (base + (-u + v) * head_r).to_array(),
        ];
        for c in &corners {
            self.line(*c, end, color, width, depth);
        }
        for i in 0..4 {
            self.line(corners[i], corners[(i + 1) % 4], color, width, depth);
        }
        self
    }

    /// Append an XY grid at `origin[2]`. Lines span `origin ± extent` on X and Y. The step count is capped at 128; the helper appends two lines per step slot, including the zero slot.
    pub fn grid(
        &mut self,
        origin: [f32; 3],
        extent: f32,
        step: f32,
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        if !(extent.is_finite() && step.is_finite() && extent > 0.0 && step > 0.0) {
            return self;
        }
        let n = ((extent / step).floor() as usize).min(128);
        for i in 0..=n {
            let o = -extent + (i as f32) * step;
            self.line(
                [origin[0] + o, origin[1] - extent, origin[2]],
                [origin[0] + o, origin[1] + extent, origin[2]],
                color,
                width,
                depth,
            );
            self.line(
                [origin[0] - extent, origin[1] + o, origin[2]],
                [origin[0] + extent, origin[1] + o, origin[2]],
                color,
                width,
                depth,
            );
        }
        self
    }

    /// RGB axes triad (X red, Y green, Z blue).
    pub fn axes(
        &mut self,
        origin: [f32; 3],
        len: f32,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        self.line(
            origin,
            [origin[0] + len, origin[1], origin[2]],
            Color::RED,
            width,
            depth,
        );
        self.line(
            origin,
            [origin[0], origin[1] + len, origin[2]],
            Color::GREEN,
            width,
            depth,
        );
        self.line(
            origin,
            [origin[0], origin[1], origin[2] + len],
            Color::BLUE,
            width,
            depth,
        );
        self
    }

    /// Validate all items plus count caps for `group`.
    pub fn validate(&self, group: &str) -> DebugResult<()> {
        if self.items.len() > MAX_ITEMS_PER_GROUP {
            return Err(DebugError::TooManyItems {
                group: group.to_string(),
                count: self.items.len(),
                max: MAX_ITEMS_PER_GROUP,
            });
        }
        for item in &self.items {
            item.validate()?;
        }
        Ok(())
    }

    /// Rough byte estimate.
    pub fn estimated_bytes(&self) -> usize {
        self.items.iter().map(DebugItem::estimated_bytes).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arrow_has_orthogonal_head_and_skips_degenerate_head() {
        let mut list = DrawList::new();
        list.arrow(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 100.0],
            Color::RED,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        assert_eq!(list.items.len(), 9);
        let DebugItem::Line(first_corner_edge) = &list.items[1] else {
            panic!("expected arrow head line");
        };
        let corner = Vec3A::from_array(first_corner_edge.start);
        let radial = corner - Vec3A::new(0.0, 0.0, 80.0);
        assert!(radial.dot(Vec3A::Z).abs() < 1e-5);
        assert!((radial.length() - 20.0 * 0.4 * 2.0_f32.sqrt()).abs() < 1e-5);

        let mut degenerate = DrawList::new();
        degenerate.arrow(
            [1.0, 2.0, 3.0],
            [1.0, 2.0, 3.0],
            Color::RED,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        assert_eq!(degenerate.items.len(), 1);
    }

    #[test]
    fn grid_lies_on_xy_plane_at_origin_z_with_requested_extents() {
        let origin = [10.0, 20.0, 30.0];
        let extent = 4.0;
        let mut list = DrawList::new();
        list.grid(
            origin,
            extent,
            2.0,
            Color::RED,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        assert_eq!(list.items.len(), 6);
        for item in &list.items {
            let DebugItem::Line(line) = item else {
                panic!("expected grid line");
            };
            assert_eq!(line.start[2], origin[2]);
            assert_eq!(line.end[2], origin[2]);
            assert!(
                line.start
                    .iter()
                    .chain(line.end.iter())
                    .all(|v| v.is_finite())
            );
            for (axis, &center) in origin.iter().take(2).enumerate() {
                assert!(line.start[axis] >= center - extent);
                assert!(line.start[axis] <= center + extent);
                assert!(line.end[axis] >= center - extent);
                assert!(line.end[axis] <= center + extent);
            }
        }
        let DebugItem::Line(first) = &list.items[0] else {
            panic!("expected grid line");
        };
        assert_eq!(
            first.start,
            [origin[0] - extent, origin[1] - extent, origin[2]]
        );
        assert_eq!(
            first.end,
            [origin[0] - extent, origin[1] + extent, origin[2]]
        );
    }

    #[test]
    fn grid_preserves_step_caps_and_ignores_invalid_dimensions() {
        let mut list = DrawList::new();
        list.grid(
            [0.0; 3],
            0.0,
            1.0,
            Color::WHITE,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        list.grid(
            [0.0; 3],
            1.0,
            f32::NAN,
            Color::WHITE,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        assert!(list.is_empty());
        list.grid(
            [0.0; 3],
            1000.0,
            1.0,
            Color::WHITE,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        assert_eq!(list.items.len(), 258);
    }

    #[test]
    fn circle_points_follow_the_supplied_plane() {
        let mut list = DrawList::new();
        list.circle(
            [1.0, 2.0, 3.0],
            2.0,
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            8,
            Color::RED,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        let DebugItem::Polyline(circle) = &list.items[0] else {
            panic!("expected circle polyline");
        };
        assert_eq!(circle.points.len(), 8);
        assert!(circle.points.iter().all(|point| point[1] == 2.0));
        assert_eq!(circle.points[0], [3.0, 2.0, 3.0]);
    }
}
