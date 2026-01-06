use glam::{Mat3A, Vec3A};

pub struct Obb {
    pub center: Vec3A,
    pub axis: Mat3A,
    pub extent: Vec3A,
}

impl Obb {
    pub const fn new(center: Vec3A, axis: Mat3A, extent: Vec3A) -> Self {
        Self {
            center,
            axis,
            extent,
        }
    }
}
