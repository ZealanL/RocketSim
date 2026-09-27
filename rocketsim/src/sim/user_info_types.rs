#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum UserInfoType {
    #[default]
    None,
    Car,
    Ball,
    DropshotTile,
}
