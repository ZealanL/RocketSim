//! Debug drawing types split into private implementation modules.
//!
//! Use the items re-exported here through `rocketsim_vis::debug`.

mod draw_list;
mod error;
mod items;
mod limits;
mod style;
mod transform;

pub use draw_list::DrawList;
pub use error::{DebugError, DebugResult};
pub use items::{
    DebugItem, InlineMeshItem, LabelItem, LabelPos, LineItem, MeshInstanceItem, PointItem,
    PolylineItem, RectBorder, RectItem, TextAlign,
};
pub use limits::{
    MAX_DEBUG_BYTES, MAX_GROUPS, MAX_ITEMS_PER_GROUP, MAX_MESH_INDICES, MAX_MESH_VERTICES,
    MAX_MESHES, MAX_NAME_LEN, MAX_POLYLINE_POINTS, MAX_TEXT_LEN, MAX_TEXTURE_BYTES,
    MAX_TEXTURE_DIM, MAX_TEXTURES, MAX_TOTAL_ITEMS, validate_name,
};
pub use style::{Color, palette};
pub use transform::{Anchor, DepthMode, GroupTransform, Lifetime, Width};
