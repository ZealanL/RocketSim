use glam::Vec3A;

#[derive(Clone, Copy, Debug, Default)]

pub struct BallHitInfo {
    /// Position of the hit relative to the ball's position
    pub relative_pos_on_ball: Vec3A,
    /// World position of the ball when the hit occured
    pub ball_pos: Vec3A,
    /// Extra velocity added to base collision velocity
    pub extra_hit_vel: Vec3A,
    /// Arena tick count when the hit occured
    pub tick_count_when_hit: u64,
    /// Arena tick count when the last extra ball-car hit impulse was applied
    ///
    /// This is needed since the extra ball-car hit impulse cannot be applied on two consecutive ticks
    pub tick_count_when_extra_impulse_applied: u64,
}
