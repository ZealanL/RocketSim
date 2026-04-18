use glam::Vec3A;

pub struct Face {
    indices: Vec<usize>,
    place: [f32; 4],
}

pub struct ConvexPolyhedron {
    vertices: Vec<Vec3A>,
    faces: Vec<Face>,
    unique_edges: Vec<Vec3A>,
    local_center: Vec3A,
    extents: Vec3A,
    radius: f32,
    c: Vec3A,
    e: Vec3A,
}
