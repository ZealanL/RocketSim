use std::{
    f32::consts::TAU,
    iter::repeat_n,
    ops::{BitAnd, BitOrAssign, Deref, DerefMut},
};

pub enum TriInfoFlag {
    V0V1Convex = 1,
    V1V2Convex = 1 << 1,
    V2V0Convex = 1 << 2,
    V0V1SwapNormalB = 1 << 3,
    V1V2SwapNormalB = 1 << 4,
    V2V0SwapNormalB = 1 << 5,
}

impl BitOrAssign<TriInfoFlag> for u8 {
    fn bitor_assign(&mut self, rhs: TriInfoFlag) {
        *self |= rhs as u8;
    }
}

impl BitAnd<TriInfoFlag> for u8 {
    type Output = u8;

    fn bitand(self, rhs: TriInfoFlag) -> Self::Output {
        self & (rhs as u8)
    }
}

#[derive(Clone, Copy)]
pub struct TriangleInfo {
    pub flags: u8,
    pub edge_v0_v1_angle: f32,
    pub edge_v1_v2_angle: f32,
    pub edge_v2_v0_angle: f32,
}

impl Default for TriangleInfo {
    fn default() -> Self {
        Self {
            flags: 0,
            edge_v0_v1_angle: TAU,
            edge_v1_v2_angle: TAU,
            edge_v2_v0_angle: TAU,
        }
    }
}

#[derive(Clone)]
pub struct TriangleInfoMap(Box<[TriangleInfo]>);

impl TriangleInfoMap {
    pub const CONVEX_EPSILON: f32 = 0.0;
    pub const PLANAR_EPSILON: f32 = 0.0001;
    pub const EQUAL_VERTEX_THRESHOLD: f32 = 0.0001 * 0.0001;
    pub const EDGE_DISTANCE_THRESHOLD: f32 = 0.1;
    pub const MAX_EDGE_ANGLE_THRESHOLD: f32 = TAU;

    pub fn new(map_size: usize) -> Self {
        Self(repeat_n(TriangleInfo::default(), map_size).collect())
    }
}

impl Deref for TriangleInfoMap {
    type Target = [TriangleInfo];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for TriangleInfoMap {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
