//! Prints soup-vs-welded vertex budgets per game mode.
//!
//! Run with: `cargo run -p rocketsim_vis --bin mesh_stats`

use rocketsim::GameMode;
use rocketsim_vis::{mesh::ArenaMesh, photo::init_meshes};

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
        println!(
            "{mode:?}: soup={} welded={} indices={} shell={} lid={} field={} ({ratio:.2}x)",
            mesh.soup_verts,
            mesh.welded_verts(),
            mesh.index_count(),
            mesh.shell.verts.len(),
            mesh.lid.verts.len(),
            mesh.field.verts.len(),
        );
    }
}
