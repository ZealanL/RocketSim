//! Retained debug rendering: typed scene, builder helpers, and a cloneable
//! [`DebugRenderer`] handle.
//!
//! Use [`DebugRenderer::draw_frame`] for simple drawing with default styles.
//! Use [`GroupHandle::draw`] for persistent drawing. Obtain a
//! [`DebugRenderer`] from [`crate::ArenaVisExt::debug_renderer`] after
//! attach. The renderer publishes named groups plus reusable mesh/texture
//! assets. The transport syncs retained state separately from overwritten
//! physics frames: mesh bytes travel once, later transform-only updates
//! send only small group records.
//!
//! Transform order (renderer contract): item local, then instance
//! transform, then group transform (scale, then rotation, then
//! translation), then anchor offset, then the live anchor frame
//! (`World` identity, else ball/car rigid transform from the newest frame).

pub mod canvas;
pub mod renderer;
pub mod scene;
pub mod types;

pub use canvas::Canvas;
pub use renderer::{DebugRenderer, GroupHandle, ItemHandle, MeshHandle, TextureHandle};
pub use scene::{DebugGroup, DebugScene, GroupOptions, MeshResource, TextureResource};
pub use types::{
    Anchor, Color, DebugError, DebugItem, DebugResult, DepthMode, DrawList, GroupTransform,
    InlineMeshItem, LabelItem, LabelPos, Lifetime, LineItem, MAX_DEBUG_BYTES, MAX_GROUPS,
    MAX_ITEMS_PER_GROUP, MAX_MESH_INDICES, MAX_MESH_VERTICES, MAX_MESHES, MAX_NAME_LEN,
    MAX_POLYLINE_POINTS, MAX_TEXT_LEN, MAX_TEXTURE_BYTES, MAX_TEXTURE_DIM, MAX_TEXTURES,
    MAX_TOTAL_ITEMS, MeshInstanceItem, PointItem, PolylineItem, RectBorder, RectItem, TextAlign,
    Width, palette, validate_name,
};
