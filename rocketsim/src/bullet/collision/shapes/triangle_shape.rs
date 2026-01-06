use glam::Vec3A;

pub struct ContactInfo {
    pub result_normal: Vec3A,
    pub contact_point: Vec3A,
    pub depth: f32,
}

/// A triangle made from 3 points.
#[derive(Clone, Copy, Debug, Default)]
pub struct TriangleShape {
    pub points: [Vec3A; 3],
    /// `edges` = \[`p1 - p0`, `p2 - p1`, `p0 - p2`]
    pub edges: [Vec3A; 3],
    pub normal: Vec3A,
    pub normal_length: f32,
}

impl TriangleShape {
    #[inline]
    /// Create a new triangle from 3 points.
    pub fn new(points: [Vec3A; 3]) -> Self {
        let edges = [
            points[1] - points[0],
            points[2] - points[1],
            points[0] - points[2],
        ];

        let (normal, normal_length) = edges[0].cross(-edges[2]).normalize_and_length();

        Self {
            points,
            edges,
            normal,
            normal_length,
        }
    }

    #[inline]
    /// Create a new triangle from an iterator that must be of 3 points
    pub fn from_points_iter(mut iter: impl Iterator<Item = Vec3A>) -> Self {
        Self::new([
            iter.next().unwrap(),
            iter.next().unwrap(),
            iter.next().unwrap(),
        ])
    }

    /// Check if a point projected onto the same place as the triangle
    /// is within the bounds of it.
    pub fn face_contains(&self, n: Vec3A, obj_to_points: &[Vec3A; 3]) -> bool {
        let c0 = self.edges[0].cross(obj_to_points[0]);
        if c0.dot(n) < 0. {
            return false;
        }

        let c1 = self.edges[1].cross(obj_to_points[1]);
        if c1.dot(n) < 0. {
            return false;
        }

        let c2 = self.edges[2].cross(obj_to_points[2]);
        c2.dot(n) >= 0.
    }

    /// Instead of using bullet's method,
    /// we use the method described here which is much faster:
    /// <https://stackoverflow.com/a/74395029/10930209>
    pub fn closest_point(&self, obj_to_points: &[Vec3A; 3]) -> Vec3A {
        let ab = self.edges[0];
        let ac = -self.edges[2];

        let d1 = ab.dot(obj_to_points[0]);
        let d2 = ac.dot(obj_to_points[0]);
        if d1 <= 0. && d2 <= 0. {
            return self.points[0];
        }

        let d3 = ab.dot(obj_to_points[1]);
        let d4 = ac.dot(obj_to_points[1]);
        if d3 >= 0. && d4 <= d3 {
            return self.points[1];
        }

        let d5 = ab.dot(obj_to_points[2]);
        let d6 = ac.dot(obj_to_points[2]);
        if d6 >= 0. && d5 <= d6 {
            return self.points[2];
        }

        let vc = d1 * d4 - d3 * d2;
        if vc <= 0. && d1 >= 0. && d3 <= 0. {
            let v = d1 / (d1 - d3);
            return self.points[0] + v * ab;
        }

        let vb = d5 * d2 - d1 * d6;
        if vb <= 0. && d2 >= 0. && d6 <= 0. {
            let v = d2 / (d2 - d6);
            return self.points[0] + v * ac;
        }

        let va = d3 * d6 - d5 * d4;
        if va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0. {
            let v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return self.points[1] + v * self.edges[1];
        }

        let denom = 1. / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;
        self.points[0] + v * ab + w * ac
    }

    /// Check if a sphere intersects the triangle.
    pub fn intersect_sphere(
        &self,
        obj_center: Vec3A,
        radius: f32,
        threshold: f32,
    ) -> Option<ContactInfo> {
        let mut triangle_normal = self.normal;
        let obj_to_center = obj_center - self.points[0];
        let mut distance_from_plane = obj_to_center.dot(triangle_normal);

        if distance_from_plane < 0. {
            distance_from_plane *= -1.0;
            triangle_normal *= -1.0;
        }

        let radius_with_threshold = radius + threshold;
        if distance_from_plane >= radius_with_threshold {
            return None;
        }

        let obj_to_points = [
            obj_to_center,
            obj_center - self.points[1],
            obj_center - self.points[2],
        ];

        let contact_point = if self.face_contains(triangle_normal, &obj_to_points) {
            obj_center - triangle_normal * distance_from_plane
        } else {
            let closest_point = self.closest_point(&obj_to_points);
            let distance_sqr = (closest_point - obj_center).length_squared();

            if distance_sqr < radius_with_threshold * radius_with_threshold {
                closest_point
            } else {
                return None;
            }
        };

        let contact_to_center = obj_center - contact_point;
        let distance_sqr = contact_to_center.length_squared();

        if distance_sqr >= radius_with_threshold * radius_with_threshold {
            return None;
        }

        let (result_normal, depth) = if distance_sqr > f32::EPSILON {
            let distance = distance_sqr.sqrt();
            (contact_to_center / distance, -(radius - distance))
        } else {
            (triangle_normal, -radius)
        };

        Some(ContactInfo {
            result_normal,
            contact_point,
            depth,
        })
    }

    #[allow(unused)]
    pub fn get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        let dots = Vec3A::new(
            self.points[0].dot(vec),
            self.points[1].dot(vec),
            self.points[2].dot(vec),
        );

        self.points[dots.max_position()]
    }
}
