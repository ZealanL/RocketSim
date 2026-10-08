/// What kind of physics body something is.
///
/// Reported by [`crate::RaycastHitInfo::user_info`] (raycasts),
/// [`crate::CarState::wheels_with_contact`] (wheel contact), and used
/// internally to route collision pairs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum UserInfoType {
    /// Static arena geometry (walls, floor, ceiling, goal meshes).
    #[default]
    None,
    /// A car chassis (wheels don't get their own bodies).
    Car,
    /// The ball (or Snowday puck).
    Ball,
    /// A Dropshot tile (only present in Dropshot mode).
    DropshotTile,
}
