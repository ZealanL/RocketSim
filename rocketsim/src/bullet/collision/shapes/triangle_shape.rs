use glam::{Vec3, Vec3A};

use crate::shared::Aabb;

pub struct ContactInfo {
    pub result_normal: Vec3A,
    pub contact_point: Vec3A,
    pub depth: f32,
}

#[cfg(test)]
fn segment_sqr_distance(from: Vec3A, to: Vec3A, p: Vec3A, nearest: &mut Vec3A) -> f32 {
    let mut diff = p - from;
    let v = to - from;
    let mut t = v.dot(diff);

    if t > 0. {
        let dot_vv = v.dot(v);
        if t < dot_vv {
            t /= dot_vv;
            diff -= t * v;
        } else {
            t = 1.;
            diff -= v;
        }
    } else {
        t = 0.;
    }

    *nearest = from + t * v;
    diff.dot(diff)
}

/// Closest point on a triangle, matching Bullet's fast Voronoi-region
/// implementation used by SphereTriangleDetector.
fn closest_point_triangle(p: Vec3A, a: Vec3A, b: Vec3A, c: Vec3A) -> Vec3A {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + v * ab;
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let v = d2 / (d2 - d6);
        return a + v * ac;
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + v * (c - b);
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a + v * ab + w * ac
}

/// A triangle made from 3 points.
#[derive(Clone, Copy, Debug, Default)]
pub struct TriangleShape {
    pub points: [Vec3A; 3],

    pub normal: Vec3A,
    pub normal_length: f32,
}


impl TriangleShape {
    pub fn edge(&self, index: usize) -> Vec3A {
        match index {
            0 => self.points[1] - self.points[0],
            1 => self.points[2] - self.points[1],
            2 => self.points[0] - self.points[2],
            _ => unreachable!(),
        }
    }

    #[inline]
    pub fn aabb(&self) -> Aabb {
        Aabb {
            min: self.points[0].min(self.points[1]).min(self.points[2]),
            max: self.points[0].max(self.points[1]).max(self.points[2]),
        }
    }

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
        let c0 = self.edge(0).cross(obj_to_points[0]);
        if c0.dot(n) < 0. {
            return false;
        }

        let c1 = self.edge(1).cross(obj_to_points[1]);
        if c1.dot(n) < 0. {
            return false;
        }

        let c2 = self.edge(2).cross(obj_to_points[2]);
        c2.dot(n) >= 0.
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
            let contact_capsule_radius_sqr = radius_with_threshold * radius_with_threshold;
            let contact_point =
                closest_point_triangle(obj_center, self.points[0], self.points[1], self.points[2]);
            let min_distance_sqr = contact_point.distance_squared(obj_center);
            if min_distance_sqr < contact_capsule_radius_sqr {
                contact_point
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
            let inverse_distance = 1.0 / distance;
            (contact_to_center * inverse_distance, -(radius - distance))
        } else {
            (triangle_normal, -radius)
        };

        Some(ContactInfo {
            result_normal,
            contact_point,
            depth,
        })
    }

    pub fn local_get_supporting_vertex_without_margin(&self, vec: Vec3A) -> Vec3A {
        let dots = Vec3::new(
            vec.dot(self.points[0]),
            vec.dot(self.points[1]),
            vec.dot(self.points[2]),
        );

        self.points[dots.max_position()]
    }

    #[inline]
    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        self.local_get_supporting_vertex_without_margin(vec)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_sqr_distance_matches_bullet_clamping() {
        let from = Vec3A::new(1.0, 2.0, 3.0);
        let to = Vec3A::new(5.0, 2.0, 3.0);
        let mut nearest = Vec3A::ZERO;

        let distance_sqr = segment_sqr_distance(from, to, Vec3A::new(3.0, 5.0, 3.0), &mut nearest);
        assert_eq!(nearest, Vec3A::new(3.0, 2.0, 3.0));
        assert_eq!(distance_sqr, 9.0);

        let distance_sqr = segment_sqr_distance(from, to, Vec3A::new(0.0, 5.0, 3.0), &mut nearest);
        assert_eq!(nearest, from);
        assert_eq!(distance_sqr, 10.0);

        let distance_sqr = segment_sqr_distance(from, to, Vec3A::new(6.0, 5.0, 3.0), &mut nearest);
        assert_eq!(nearest, to);
        assert_eq!(distance_sqr, 10.0);
    }

    #[test]
    fn sphere_edge_fallback_keeps_first_equal_minimum() {
        let triangle = TriangleShape::new([
            Vec3A::new(0.0, 0.0, 0.0),
            Vec3A::new(2.0, 0.0, 0.0),
            Vec3A::new(0.0, 2.0, 0.0),
        ]);
        let sphere_center = Vec3A::new(-0.5, -0.5, 0.25);
        let mut nearest = Vec3A::ZERO;
        let edge_zero_distance = segment_sqr_distance(
            triangle.points[0],
            triangle.points[1],
            sphere_center,
            &mut nearest,
        );
        let edge_two_distance = segment_sqr_distance(
            triangle.points[2],
            triangle.points[0],
            sphere_center,
            &mut nearest,
        );
        assert_eq!(edge_zero_distance, edge_two_distance);

        let contact = triangle
            .intersect_sphere(sphere_center, 1.0, 0.0)
            .expect("sphere should intersect an edge capsule");
        assert_eq!(contact.contact_point, triangle.points[0]);
    }
}
