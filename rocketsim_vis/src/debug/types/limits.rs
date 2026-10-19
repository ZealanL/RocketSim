use super::error::{DebugError, DebugResult};

/// Maximum retained groups per scene.
pub const MAX_GROUPS: usize = 256;
/// Maximum items in one group.
pub const MAX_ITEMS_PER_GROUP: usize = 4096;
/// Maximum total items across a scene snapshot.
pub const MAX_TOTAL_ITEMS: usize = 65_536;
/// Maximum points in one polyline.
pub const MAX_POLYLINE_POINTS: usize = 4096;
/// Maximum vertices in one inline mesh or mesh resource.
pub const MAX_MESH_VERTICES: usize = 65_536;
/// Maximum indices in one inline mesh or mesh resource (must be `% 3 == 0`).
pub const MAX_MESH_INDICES: usize = 262_144;
/// Maximum reusable meshes per scene.
pub const MAX_MESHES: usize = 128;
/// Maximum reusable textures per scene.
pub const MAX_TEXTURES: usize = 64;
/// Maximum texture width or height in pixels.
pub const MAX_TEXTURE_DIM: u32 = 1024;
/// Maximum decoded RGBA bytes for one texture (`1024*1024*4`).
pub const MAX_TEXTURE_BYTES: usize = 4 * 1024 * 1024;
/// Maximum UTF-8 bytes for one label.
pub const MAX_TEXT_LEN: usize = 1024;
/// Maximum bytes for a group / mesh / texture name or id.
pub const MAX_NAME_LEN: usize = 64;
/// Maximum estimated bytes for retained groups and resources in one scene.
pub const MAX_DEBUG_BYTES: usize = 8 * 1024 * 1024;

/// Validate a group name or resource identifier.
///
/// Accept 1 through [`MAX_NAME_LEN`] UTF-8 bytes. Each slash-separated segment
/// must be nonempty, must not be `.` or `..`, and must not contain whitespace
/// or control characters. Other Unicode characters are accepted.
///
/// # Errors
///
/// Return [`DebugError::InvalidName`] with the supplied name if a rule fails.
///
/// ```
/// use rocketsim_vis::debug::validate_name;
///
/// assert!(validate_name("planner/target").is_ok());
/// assert!(validate_name("planner/../target").is_err());
/// assert!(validate_name("target name").is_err());
/// ```
pub fn validate_name(name: &str) -> DebugResult<()> {
    let len = name.len();
    if len == 0 || len > MAX_NAME_LEN {
        return Err(DebugError::InvalidName(name.to_string()));
    }
    if name.split('/').any(|segment| {
        segment.is_empty()
            || segment == "."
            || segment == ".."
            || segment.chars().any(|c| c.is_control() || c.is_whitespace())
    }) {
        return Err(DebugError::InvalidName(name.to_string()));
    }
    Ok(())
}

pub(super) fn finite3(v: &[f32; 3], what: &'static str) -> DebugResult<()> {
    if v.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(DebugError::InvalidNumber(what))
    }
}

pub(super) fn finite2(v: &[f32; 2], what: &'static str) -> DebugResult<()> {
    if v.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(DebugError::InvalidNumber(what))
    }
}
