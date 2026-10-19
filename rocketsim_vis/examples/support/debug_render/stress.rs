use glam::Mat3A;
use rocketsim_vis::debug::{DebugRenderer, DepthMode, DrawList, MeshResource, Width, palette};

use super::meshes::{insert, submit_mesh};

pub fn render(
    renderer: &DebugRenderer,
    n: usize,
    mesh_submit_ms: &mut f64,
) -> Result<(), Box<dyn std::error::Error>> {
    let item_budget = n.saturating_sub(1);
    let point_count = item_budget / 2;
    let instance_count = item_budget - point_count;
    for start in (0..point_count).step_by(4096) {
        let mut batch = DrawList::with_capacity((point_count - start).min(4096));
        for i in start..(start + 4096).min(point_count) {
            let a = i as f32 * 0.017;
            batch.point(
                [
                    a.cos() * 1800.0,
                    a.sin() * 1000.0,
                    80.0 + (i % 200) as f32 * 4.0,
                ],
                palette::rainbow(i as f32 / n.max(1) as f32),
                Width::pixels(2.0),
                DepthMode::Always,
            );
        }
        insert(renderer, &format!("stress_points_{start}"), batch)?;
    }
    let mesh = MeshResource::new(
        "stress_marker",
        vec![[-20.0, -20.0, 0.0], [20.0, -20.0, 0.0], [0.0, 20.0, 36.0]],
        vec![palette::CYAN; 3],
        Some(vec![[0.0, 0.0, 1.0]; 3]),
        None,
        vec![0, 1, 2],
    )?;
    submit_mesh(renderer, mesh, mesh_submit_ms)?;
    for start in (0..instance_count).step_by(4096) {
        let mut instances = DrawList::with_capacity((instance_count - start).min(4096));
        for i in start..(start + 4096).min(instance_count) {
            let a = i as f32 * 0.041;
            instances.instance(
                "stress_marker",
                [
                    a.cos() * 2400.0,
                    a.sin() * 1450.0,
                    80.0 + (i % 80) as f32 * 8.0,
                ],
                Mat3A::IDENTITY.to_cols_array_2d(),
                [1.0; 3],
                palette::rainbow(i as f32 / n.max(1) as f32),
                None,
                DepthMode::Test,
            );
        }
        insert(renderer, &format!("stress_instances_{start}"), instances)?;
    }
    let mut paths = DrawList::new();
    let line_points: Vec<_> = (0..n.min(4096))
        .map(|i| {
            let x = -2200.0 + i as f32 * 4400.0 / n.max(1) as f32;
            [x, -1250.0, 20.0 + (i % 2) as f32 * 40.0]
        })
        .collect();
    if n > 0 && line_points.len() >= 2 {
        paths.polyline(
            line_points,
            palette::LIME,
            Width::pixels(1.0),
            DepthMode::Always,
            false,
        );
    }
    insert(renderer, "stress_lines", paths)?;
    Ok(())
}
