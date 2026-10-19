//! Retained debug rendering for drawings and reusable assets.
//!
//! # Draw one frame
//!
//! Use [`DebugRenderer::draw_frame`] for temporary marks. It replaces the
//! reserved group. It remains through the next simulation update and expires
//! at the following update.
//!
//! ```
//! use rocketsim_vis::debug::{Color, DebugRenderer};
//!
//! let renderer = DebugRenderer::new_detached();
//! let revision = renderer.draw_frame(|draw| {
//!     draw.line([0.0, 0.0, 0.0], [100.0, 0.0, 0.0])
//!         .color(Color::RED);
//! }).unwrap();
//! assert!(revision > 0);
//! ```
//!
//! # Keep and update a group
//!
//! A named group persists by default. Change its visibility, transform, or
//! anchor without replacing its items. These updates preserve its lifetime.
//!
//! ```
//! use rocketsim_vis::debug::{Anchor, DebugRenderer, GroupTransform};
//!
//! let renderer = DebugRenderer::new_detached();
//! let group = renderer.group("marker");
//! group.draw(|draw| { draw.point([0.0, 0.0, 0.0]); }).unwrap();
//! group.set_visible(false).unwrap();
//! group.set_transform(GroupTransform::from_position([10.0, 0.0, 0.0])).unwrap();
//! group.set_anchor(Anchor::World).unwrap();
//! assert_eq!(group.name(), "marker");
//! ```
//!
//! # Reuse mesh and texture resources
//!
//! Insert resources before you submit items that refer to them. A resource
//! cannot be removed while a group uses it. Keep resource bytes in the scene
//! when you update only group transforms. This avoids repeated mesh uploads.
//!
//! Obtain a renderer from [`crate::ArenaVisExt::debug_renderer`] after attach,
//! or use [`DebugRenderer::new_detached`] for local work without a viewer or
//! network connection. Detached renderer ticks advance only through
//! [`DebugRenderer::advance_tick`]. Handles own a renderer clone and remain
//! usable while the renderer exists. Item handles become stale after geometry
//! changes. Replace the group or request a new item handle in that case.
//!
//! # Coordinate systems and limits
//!
//! Three-dimensional positions use group-local Unreal units. Screen positions
//! use viewport fractions. Transform order is item local, instance, group,
//! anchor offset, then live anchor frame. Apply scale, then rotation, then
//! translation at each stage. [`GroupTransform`] stores rotation matrix rows.
//!
//! Use [`DepthMode::Test`] when opaque world geometry must hide a drawing.
//! [`DepthMode::Always`] draws over the world. Labels and screen rectangles
//! are overlays and do not use depth testing.
//!
//! Scene limits reject oversized submissions with [`DebugError`]. See the
//! `MAX_*` constants for item, resource, and byte limits. Store shared geometry
//! in resources and update only transforms when geometry does not change.
//!
//! # Draw several instances with one resource
//!
//! This example runs without a viewer. It inserts one mesh and one texture,
//! then uses them in 32 instances. Moving the group does not replace either
//! resource. Remove references before you remove a resource.
//!
//! ```
//! use std::sync::Arc;
//! use glam::Mat3A;
//! use rocketsim_vis::debug::{
//!     Color, DebugError, DebugRenderer, DepthMode, DrawList, GroupOptions,
//!     GroupTransform, MeshResource, TextureResource,
//! };
//!
//! let renderer = DebugRenderer::new_detached();
//! renderer.insert_texture(TextureResource::new(
//!     "marker/color", 1, 1, vec![255, 180, 40, 255],
//! )?)?;
//! renderer.insert_mesh(MeshResource::new(
//!     "marker/triangle",
//!     vec![[0.0, 0.0, 0.0], [80.0, 0.0, 0.0], [0.0, 80.0, 0.0]],
//!     vec![Color::WHITE; 3],
//!     None,
//!     Some(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]),
//!     vec![0, 1, 2],
//! )?)?;
//! let mut items = DrawList::new();
//! for index in 0..32 {
//!     items.instance(
//!         "marker/triangle", [index as f32 * 100.0, 0.0, 100.0],
//!         Mat3A::IDENTITY.to_cols_array_2d(), [1.0; 3], Color::WHITE,
//!         Some("marker/color".into()), DepthMode::Test,
//!     );
//! }
//! renderer.replace_group("markers", items, GroupOptions::new())?;
//! let before = renderer.snapshot();
//! renderer.set_group_transform(
//!     "markers", GroupTransform::from_position([0.0, 500.0, 0.0]),
//! )?;
//! let after = renderer.snapshot();
//! assert_eq!(after.groups["markers"].items.len(), 32);
//! assert!(Arc::ptr_eq(
//!     &before.meshes["marker/triangle"], &after.meshes["marker/triangle"],
//! ));
//! assert!(matches!(renderer.remove_mesh("marker/triangle"),
//!     Err(DebugError::ResourceInUse(_))));
//! renderer.clear_groups();
//! assert!(renderer.remove_mesh("marker/triangle")?);
//! assert!(renderer.remove_texture("marker/color")?);
//! # Ok::<(), DebugError>(())
//! ```

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
