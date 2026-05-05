use glam::Vec3A;

#[derive(Clone, Copy, Debug, Default)]
pub struct BoostPadConfig {
    pub pos: Vec3A,
    pub is_big: bool,
}
