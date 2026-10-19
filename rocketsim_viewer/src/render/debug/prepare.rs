use rocketsim_vis::debug::{DepthMode, Width};

use super::{
    DebugGl, Material,
    order::{DrawLayer, OrderedInstance, contiguous_runs_into},
};

impl DebugGl {
    pub(super) fn prepare_ordered_instances(&mut self) -> bool {
        self.ordered_materials.clear();
        self.ordered_material_ids.clear();
        self.ordered_instances.clear();
        for (group_index, prepared) in self.prepared_groups.iter().enumerate() {
            for batch in self.batches.get(&prepared.group.name).into_iter().flatten() {
                let layers: &[DrawLayer] = match batch.depth {
                    DepthMode::Test if self.batch_is_transparent(batch) => {
                        &[DrawLayer::Transparent]
                    }
                    DepthMode::Test => continue,
                    DepthMode::Xray => &[DrawLayer::XrayHidden, DrawLayer::XrayVisible],
                    DepthMode::Always => &[DrawLayer::Overlay],
                };
                let Some(mesh) = self.meshes.get(&batch.mesh) else {
                    continue;
                };
                let material = Material {
                    mesh: batch.mesh.clone(),
                    texture: batch.texture.clone(),
                };
                let material_index = if let Some(&index) = self.ordered_material_ids.get(&material)
                {
                    index
                } else {
                    let index = self.ordered_materials.len();
                    self.ordered_materials.push(material.clone());
                    self.ordered_material_ids.insert(material, index);
                    index
                };
                for instance in &batch.instances {
                    let transform = prepared.transform * instance.transform;
                    let center = transform.transform_point3a(mesh.center);
                    let forward_sign = if self.projection_matrix.z_axis.w != 0.0 {
                        self.projection_matrix.z_axis.w.signum()
                    } else {
                        self.projection_matrix.z_axis.z.signum()
                    };
                    let view_depth = self.view_matrix.transform_point3a(center).z * forward_sign;
                    for &layer in layers {
                        self.ordered_instances.push(OrderedInstance {
                            material: material_index,
                            layer,
                            view_depth,
                            source_order: (group_index, instance.source_index),
                            transform,
                            tint: instance.tint,
                        });
                    }
                }
            }
        }
        super::order::sort_back_to_front(&mut self.ordered_instances);
        contiguous_runs_into(&self.ordered_instances, &mut self.ordered_runs);

        self.ordered_scratch.clear();
        for instance in &self.ordered_instances {
            append_instance(&mut self.ordered_scratch, instance.transform, instance.tint);
        }
        if self.ordered_scratch != self.ordered_data {
            std::mem::swap(&mut self.ordered_data, &mut self.ordered_scratch);
            true
        } else {
            false
        }
    }
}

pub(super) fn depth_key(depth: DepthMode) -> u8 {
    match depth {
        DepthMode::Test => 0,
        DepthMode::Always => 1,
        DepthMode::Xray => 2,
    }
}

pub(super) fn depth_mode(depth: u8) -> DepthMode {
    match depth {
        0 => DepthMode::Test,
        1 => DepthMode::Always,
        _ => DepthMode::Xray,
    }
}

pub(super) fn append_instance(data: &mut Vec<f32>, transform: glam::Mat4, tint: [f32; 4]) {
    data.extend_from_slice(&transform.to_cols_array());
    data.extend_from_slice(&tint);
}

pub(super) fn pack_segment(
    start: [f32; 3],
    end: [f32; 3],
    color: [f32; 4],
    width: Width,
    point: bool,
) -> super::lines::PackedLine {
    let (width, pixels) = match width {
        Width::Pixels(width) => (width, true),
        Width::World(width) => (width, false),
    };
    super::lines::PackedLine {
        start,
        end,
        color,
        width,
        pixels,
        point,
    }
}
