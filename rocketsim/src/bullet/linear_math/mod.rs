mod affine_ext;
mod game_rotation;
mod obb;
mod quat_ext;
mod transform_util;
mod util;

pub(super) use affine_ext::AffineExt;
pub(super) use game_rotation::bullet_quat_from_basis_columns;
pub(crate) use game_rotation::{
    game_get_rotation_scalar, game_integrate_orientation_basis_columns, game_quat_normalize_scalar,
    game_set_rotation_scalar_columns,
};
pub(super) use obb::Obb;
pub(super) use quat_ext::QuatExt;
pub(super) use transform_util::{integrate_trans, integrate_trans_no_rot};
pub(super) use util::{interpolate_3, max_dot, plane_space_1};
