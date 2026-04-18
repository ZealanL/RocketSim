use ahash::AHashMap;
use glam::{Vec3, Vec3A};
use miniquad::{BufferId, BufferSource, BufferType, BufferUsage, RenderingBackend};

use crate::{shared::rsmath, vis::backend::Model};

#[derive(Debug, Clone)]
pub struct ModelSet {
    /// Maps each model name to the start offset and length in the buffers
    index_map: AHashMap<String, (usize, usize)>,
    /// Contains the data from all models concatenated
    concat_model: Model,
}

impl ModelSet {
    pub fn new(models: &[(&str, Model)]) -> Self {
        let mut index_map = AHashMap::new();
        let mut concat_model = Model::empty();
        for (name_str, model) in models {
            let start_idx = concat_model.num_verts();
            concat_model.verts.append(&mut model.verts.clone());
            concat_model.vert_uvs.append(&mut model.vert_uvs.clone());
            let end_idx = concat_model.num_verts();

            index_map.insert(name_str.to_string(), (start_idx, end_idx));
        }

        Self {
            index_map,
            concat_model,
        }
    }

    pub fn calc_normals(&self) -> Vec<Vec3> {
        let mut results = Vec::with_capacity(self.concat_model.verts.len());

        for tri_verts in self.concat_model.verts.chunks(3) {
            let tri_verts = [
                tri_verts[0].to_vec3a(),
                tri_verts[1].to_vec3a(),
                tri_verts[2].to_vec3a(),
            ];
            let tri_normal = rsmath::try_calc_tri_normal(&tri_verts).unwrap_or(Vec3A::ZERO);

            for _ in 0..3 {
                results.push(tri_normal.to_vec3());
            }
        }

        results
    }

    pub fn build_mq_vertex_buffers(&self, ctx: &mut Box<dyn RenderingBackend>) -> Vec<BufferId> {
        let normals = self.calc_normals();
        vec![
            ctx.new_buffer(
                BufferType::VertexBuffer,
                BufferUsage::Immutable,
                BufferSource::slice(&self.concat_model.verts),
            ),
            ctx.new_buffer(
                BufferType::VertexBuffer,
                BufferUsage::Immutable,
                BufferSource::slice(&normals),
            ),
            ctx.new_buffer(
                BufferType::VertexBuffer,
                BufferUsage::Immutable,
                BufferSource::slice(&self.concat_model.vert_uvs),
            ),
        ]
    }

    pub fn get_model_draw_range(&self, model_name: &str) -> (usize, usize) {
        self.index_map[model_name]
    }

    pub fn num_verts(&self) -> usize {
        self.concat_model.num_verts()
    }
}
