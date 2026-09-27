use miniquad::{
    RawId, RenderingBackend, TextureAccess, TextureFormat, TextureId, TextureParams, TextureSource,
};
use rustc_hash::FxHashMap;

use crate::backend::Texture;

/// All textures uploaded to the GPU, addressed by name.
///
/// Names are the keys passed to [`TextureSet::new`], e.g. `"ball"`,
/// `"car_blue"`, `"boost_pad"`.
#[derive(Debug, Clone)]
pub struct TextureSet {
    map: FxHashMap<String, (Texture, usize)>,
}

impl TextureSet {
    /// Indexes `textures` by name in insertion order.
    pub fn new(textures: &[(&str, Texture)]) -> Self {
        let mut map = FxHashMap::default();
        for (name_str, texture) in textures {
            let idx = map.len();
            map.insert(name_str.to_string(), (texture.clone(), idx));
        }

        Self { map }
    }

    pub fn build_mq_textures(&self, ctx: &mut Box<dyn RenderingBackend>) -> Vec<TextureId> {
        let mut results: Vec<TextureId> = Vec::new();
        results.resize(
            self.num_textures(),
            TextureId::from_raw_id(RawId::OpenGl(0)),
        );

        for (texture, texture_idx) in self.map.values() {
            results[*texture_idx] = ctx.new_texture(
                TextureAccess::Static,
                TextureSource::Bytes(&texture.image_bytes),
                TextureParams {
                    width: texture.width,
                    height: texture.height,
                    format: TextureFormat::RGBA8,
                    ..Default::default()
                },
            );
        }

        results
    }

    /// Number of textures in the set.
    pub fn num_textures(&self) -> usize {
        self.map.len()
    }

    /// GPU slot for `name`. Panics on unknown names.
    pub fn get_texture_idx(&self, name: &str) -> usize {
        self.map[name].1
    }
}
