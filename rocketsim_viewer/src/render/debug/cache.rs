use std::{
    collections::HashMap,
    sync::{Arc, Weak},
};

use glow::HasContext;
use rocketsim_vis::debug::{Color, DebugItem, DebugScene, MeshResource, TextureResource};

use super::{
    DebugGl,
    resources::{delete_mesh, upload_mesh, upload_texture},
};

pub use rocketsim_vis::debug::{BuiltinMesh, BuiltinTexture};

pub(super) const BUILTIN_REVISION: u64 = u64::MAX;

#[derive(Default)]
pub(super) struct ResourceSyncIdentity {
    scene: Option<Weak<DebugScene>>,
    revision: Option<u64>,
    visible_items: HashMap<String, Weak<Vec<DebugItem>>>,
    meshes: HashMap<String, Weak<MeshResource>>,
    textures: HashMap<String, Weak<TextureResource>>,
}

impl ResourceSyncIdentity {
    fn matches(&self, scene: &Arc<DebugScene>) -> bool {
        if self.revision == Some(scene.revision)
            && self
                .scene
                .as_ref()
                .and_then(Weak::upgrade)
                .is_some_and(|cached| Arc::ptr_eq(&cached, scene))
        {
            return true;
        }

        let mut visible_count = 0;
        for (name, group) in &scene.groups {
            if !group.visible {
                continue;
            }
            visible_count += 1;
            if !self
                .visible_items
                .get(name)
                .is_some_and(|cached| same_allocation(cached, &group.items))
            {
                return false;
            }
        }
        visible_count == self.visible_items.len()
            && weak_map_matches(&self.meshes, &scene.meshes)
            && weak_map_matches(&self.textures, &scene.textures)
    }

    fn remember_root(&mut self, scene: &Arc<DebugScene>) {
        self.scene = Some(Arc::downgrade(scene));
        self.revision = Some(scene.revision);
    }

    fn record(&mut self, scene: &Arc<DebugScene>) {
        self.scene = Some(Arc::downgrade(scene));
        self.revision = Some(scene.revision);
        self.visible_items.clear();
        self.meshes.clear();
        self.textures.clear();
        for (name, group) in &scene.groups {
            if group.visible {
                self.visible_items
                    .insert(name.clone(), Arc::downgrade(&group.items));
            }
        }
        for (id, mesh) in &scene.meshes {
            self.meshes.insert(id.clone(), Arc::downgrade(mesh));
        }
        for (id, texture) in &scene.textures {
            self.textures.insert(id.clone(), Arc::downgrade(texture));
        }
    }
}

fn same_allocation<T>(cached: &Weak<T>, current: &Arc<T>) -> bool {
    cached.strong_count() > 0 && cached.as_ptr() == Arc::as_ptr(current)
}

fn weak_map_matches<T>(
    cached: &HashMap<String, Weak<T>>,
    current: &std::collections::BTreeMap<String, Arc<T>>,
) -> bool {
    cached.len() == current.len()
        && current.iter().all(|(id, resource)| {
            cached
                .get(id)
                .is_some_and(|cached| same_allocation(cached, resource))
        })
}

fn builtin_mesh(id: &str) -> Option<MeshResource> {
    let builtin = [
        BuiltinMesh::Ball,
        BuiltinMesh::Puck,
        BuiltinMesh::CarOctane,
        BuiltinMesh::CarDominus,
        BuiltinMesh::CarBreakout,
        BuiltinMesh::CarMerc,
    ]
    .into_iter()
    .find(|builtin| builtin.id() == id)?;
    let obj = match builtin {
        BuiltinMesh::Ball => {
            crate::geometry::obj::ObjMesh::load(include_str!("../../models/ball.obj"))
        }
        BuiltinMesh::Puck => crate::geometry::obj::ObjMesh::puck(),
        BuiltinMesh::CarOctane => {
            crate::geometry::obj::ObjMesh::load(include_str!("../../models/car_octane.obj"))
        }
        BuiltinMesh::CarDominus => {
            crate::geometry::obj::ObjMesh::load(include_str!("../../models/car_dominus.obj"))
        }
        BuiltinMesh::CarBreakout => {
            crate::geometry::obj::ObjMesh::load(include_str!("../../models/car_breakout.obj"))
        }
        BuiltinMesh::CarMerc => {
            crate::geometry::obj::ObjMesh::load(include_str!("../../models/car_merc.obj"))
        }
    };
    Some(MeshResource {
        id: id.to_owned(),
        positions: obj
            .verts
            .iter()
            .map(|position| position.to_array())
            .collect(),
        colors: vec![Color::WHITE; obj.verts.len()],
        normals: Some(obj.normals.iter().map(|normal| normal.to_array()).collect()),
        uvs: Some(obj.uvs.iter().map(|uv| uv.to_array()).collect()),
        indices: obj.indices,
        revision: BUILTIN_REVISION,
    })
}

pub(super) fn builtin_texture(id: &str) -> Option<TextureResource> {
    let builtin = [
        BuiltinTexture::Ball,
        BuiltinTexture::CarBlue,
        BuiltinTexture::CarOrange,
    ]
    .into_iter()
    .find(|builtin| builtin.id() == id)?;
    let bytes = match builtin {
        BuiltinTexture::Ball => include_bytes!("../../textures/ball.png").as_slice(),
        BuiltinTexture::CarBlue => include_bytes!("../../textures/car_blue.png").as_slice(),
        BuiltinTexture::CarOrange => include_bytes!("../../textures/car_orange.png").as_slice(),
    };
    let image = crate::render::textures::PngImage::decode(bytes);
    Some(TextureResource {
        id: builtin.id().to_owned(),
        width: image.width,
        height: image.height,
        rgba: image.bytes,
        revision: BUILTIN_REVISION,
    })
}

impl DebugGl {
    /// Upload referenced assets whose revisions changed.
    pub fn sync(&mut self, gl: &glow::Context, scene: &Arc<DebugScene>) {
        if self.resource_sync.matches(scene) {
            self.resource_sync.remember_root(scene);
            return;
        }

        for group in scene.groups.values().filter(|group| group.visible) {
            for item in group.items.iter() {
                match item {
                    DebugItem::Instance(instance) => {
                        self.sync_mesh_reference(gl, scene, &instance.mesh);
                        if let Some(texture) = &instance.texture {
                            self.sync_texture_reference(gl, scene, texture);
                        }
                    }
                    DebugItem::Mesh(mesh) => {
                        if let Some(texture) = &mesh.texture {
                            self.sync_texture_reference(gl, scene, texture);
                        }
                    }
                    _ => {}
                }
            }
        }

        let stale_meshes: Vec<String> = self
            .meshes
            .keys()
            .filter(|id| {
                !id.starts_with("@inline:")
                    && !id.starts_with("@builtin/")
                    && !scene.meshes.contains_key(*id)
            })
            .cloned()
            .collect();
        for id in stale_meshes {
            self.ordered_dirty = true;
            if let Some(mesh) = self.meshes.remove(&id) {
                unsafe { delete_mesh(gl, mesh) };
            }
        }

        let stale_textures: Vec<String> = self
            .textures
            .keys()
            .filter(|id| !id.starts_with("@builtin/") && !scene.textures.contains_key(*id))
            .cloned()
            .collect();
        for id in stale_textures {
            self.ordered_dirty = true;
            if let Some(texture) = self.textures.remove(&id) {
                unsafe { gl.delete_texture(texture.handle) };
            }
        }
        self.resource_sync.record(scene);
    }

    fn sync_mesh_reference(&mut self, gl: &glow::Context, scene: &DebugScene, id: &str) {
        if let Some(mesh) = scene.meshes.get(id) {
            self.sync_mesh(gl, id, mesh, false);
        } else if !self.meshes.get(id).is_some_and(|gpu| gpu.builtin)
            && let Some(mesh) = builtin_mesh(id)
        {
            self.sync_mesh(gl, id, &mesh, true);
        }
    }

    fn sync_texture_reference(&mut self, gl: &glow::Context, scene: &DebugScene, id: &str) {
        if let Some(texture) = scene.textures.get(id) {
            self.sync_texture(gl, id, texture, false);
        } else if !self.textures.get(id).is_some_and(|gpu| gpu.builtin)
            && let Some(texture) = builtin_texture(id)
        {
            self.sync_texture(gl, id, &texture, true);
        }
    }

    fn sync_mesh(&mut self, gl: &glow::Context, id: &str, mesh: &MeshResource, builtin: bool) {
        if self
            .meshes
            .get(id)
            .is_some_and(|gpu| gpu.revision == mesh.revision && gpu.builtin == builtin)
        {
            return;
        }
        if let Some(old) = self.meshes.remove(id) {
            unsafe { delete_mesh(gl, old) };
        }
        self.ordered_dirty = true;
        let mut gpu = unsafe { upload_mesh(gl, mesh) };
        gpu.builtin = builtin;
        self.stats.mesh_uploads += 1;
        self.stats.upload_bytes += mesh.positions.len() as u64 * 48 + mesh.indices.len() as u64 * 4;
        self.meshes.insert(id.to_owned(), gpu);
    }

    fn sync_texture(
        &mut self,
        gl: &glow::Context,
        id: &str,
        texture: &TextureResource,
        builtin: bool,
    ) {
        if self
            .textures
            .get(id)
            .is_some_and(|gpu| gpu.revision == texture.revision && gpu.builtin == builtin)
        {
            return;
        }
        if let Some(old) = self.textures.remove(id) {
            unsafe { gl.delete_texture(old.handle) };
        }
        self.ordered_dirty = true;
        let mut gpu = unsafe { upload_texture(gl, texture) };
        gpu.builtin = builtin;
        self.stats.texture_uploads += 1;
        self.stats.upload_bytes += texture.rgba.len() as u64;
        self.textures.insert(id.to_owned(), gpu);
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use rocketsim_vis::debug::{DebugGroup, DrawList, GroupOptions};

    fn mesh_resource() -> Arc<MeshResource> {
        Arc::new(MeshResource {
            id: "mesh".into(),
            positions: Vec::new(),
            colors: Vec::new(),
            normals: None,
            uvs: None,
            indices: Vec::new(),
            revision: 4,
        })
    }

    fn texture() -> Arc<TextureResource> {
        Arc::new(TextureResource {
            id: "texture".into(),
            width: 1,
            height: 1,
            rgba: vec![255; 4],
            revision: 5,
        })
    }

    fn scene(
        revision: u64,
        group: Option<Arc<DebugGroup>>,
        mesh: Option<Arc<MeshResource>>,
        texture: Option<Arc<TextureResource>>,
    ) -> Arc<DebugScene> {
        Arc::new(DebugScene {
            groups: group
                .map(|group| [(group.name.clone(), group)].into())
                .unwrap_or_default(),
            meshes: mesh
                .map(|mesh| [(mesh.id.clone(), mesh)].into())
                .unwrap_or_default(),
            textures: texture
                .map(|texture| [(texture.id.clone(), texture)].into())
                .unwrap_or_default(),
            revision,
        })
    }

    #[test]
    fn sync_identity_skips_shared_snapshots_and_detects_filtered_or_replaced_data() {
        let group =
            Arc::new(DebugGroup::new("group", DrawList::new(), GroupOptions::default()).unwrap());
        let mesh = mesh_resource();
        let texture = texture();
        let first = scene(
            8,
            Some(group.clone()),
            Some(mesh.clone()),
            Some(texture.clone()),
        );
        let mut cache = ResourceSyncIdentity::default();
        cache.record(&first);
        assert!(cache.matches(&first));

        let next_snapshot = scene(
            8,
            Some(group.clone()),
            Some(mesh.clone()),
            Some(texture.clone()),
        );
        assert!(cache.matches(&next_snapshot));
        assert!(cache.matches(&scene(
            9,
            Some(group.clone()),
            Some(mesh.clone()),
            Some(texture.clone()),
        )));
        assert!(!cache.matches(&scene(8, None, Some(mesh.clone()), Some(texture.clone()),)));

        let mut hidden = (*group).clone();
        hidden.visible = false;
        let filtered = scene(
            8,
            Some(Arc::new(hidden)),
            next_snapshot.meshes.values().next().cloned(),
            next_snapshot.textures.values().next().cloned(),
        );
        assert!(!cache.matches(&filtered));
        cache.record(&filtered);
        assert!(cache.matches(&filtered));

        let replacement = scene(
            8,
            filtered.groups.values().next().cloned(),
            Some(mesh_resource()),
            filtered.textures.values().next().cloned(),
        );
        assert!(!cache.matches(&replacement));
        assert!(!cache.matches(&scene(9, None, None, None)));
        assert!(!cache.matches(&scene(8, None, None, None)));
    }
}
