//! Prints soup-vs-welded vertex budgets per game mode.
//!
//! Run with: `cargo run -p rocketsim_viewer --bin mesh_stats`

use rocketsim::GameMode;
use rocketsim_viewer::geometry::mesh::ArenaMesh;
#[path = "../../examples/support/photo.rs"]
mod photo;
use photo::init_meshes;

fn main() {
    init_meshes();
    for mode in [
        GameMode::Soccar,
        GameMode::Hoops,
        GameMode::Heatseeker,
        GameMode::Snowday,
        GameMode::Dropshot,
        GameMode::TheVoid,
    ] {
        let mesh = ArenaMesh::build(mode);
        let ratio = if mesh.welded_verts() == 0 {
            0.0
        } else {
            mesh.soup_verts as f64 / mesh.welded_verts() as f64
        };
        // Shell extents catch mode meshes that dwarf (or miss) the play
        // volume the cameras assume.
        let (mut mn, mut mx) = (glam::Vec3::splat(f32::MAX), glam::Vec3::splat(f32::MIN));
        for v in mesh.shell.verts.iter().chain(mesh.lid.verts.iter()) {
            mn = mn.min(*v);
            mx = mx.max(*v);
        }
        println!(
            "{mode:?}: soup={} welded={} indices={} shell={} lid={} net={} field={} ({ratio:.2}x) extent=[{:.0},{:.0},{:.0}]..[{:.0},{:.0},{:.0}]",
            mesh.soup_verts,
            mesh.welded_verts(),
            mesh.index_count(),
            mesh.shell.verts.len(),
            mesh.lid.verts.len(),
            mesh.net.verts.len(),
            mesh.field.verts.len(),
            mn.x,
            mn.y,
            mn.z,
            mx.x,
            mx.y,
            mx.z,
        );
    }
}
