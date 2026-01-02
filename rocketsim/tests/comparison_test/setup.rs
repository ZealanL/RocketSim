use crate::comparison_test::ControlSeq;
use glam::{Mat3A, Vec3A};
use rocketsim::{BallState, CarControls, CarState, Team};
use rocketsim_rs::consts;

#[derive(Debug, Clone)]
pub struct CarSetup {
    pub team: Team,
    pub control_seq: ControlSeq,

    pub pos: Vec3A,
    pub rot_mat: Mat3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
    pub on_ground: bool,

    pub boost: f32,
}

impl CarSetup {
    pub const fn new(team: Team, pos: Vec3A) -> Self {
        Self {
            team,
            control_seq: ControlSeq::new(),
            pos,
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
            boost: consts::BOOST_MAX,
            on_ground: false,
        }
    }

    pub fn with_control_seq(mut self, control_seq: ControlSeq) -> Self {
        self.control_seq = control_seq;
        self
    }

    pub const fn with_rot(mut self, rot: Mat3A) -> Self {
        self.rot_mat = rot;
        self
    }

    pub const fn with_vel(mut self, vel: Vec3A) -> Self {
        self.vel = vel;
        self
    }

    pub const fn with_ang_vel(mut self, ang_vel: Vec3A) -> Self {
        self.ang_vel = ang_vel;
        self
    }

    pub const fn with_boost(mut self, boost: f32) -> Self {
        self.boost = boost;
        self
    }

    pub const fn with_on_ground(mut self, on_ground: bool) -> Self {
        self.on_ground = on_ground;
        self
    }

    pub fn make_initial_car_state(&self) -> CarState {
        let mut result = CarState::DEFAULT;
        result.phys.pos = self.pos;
        result.phys.rot_mat = self.rot_mat;
        result.phys.vel = self.vel;
        result.phys.ang_vel = self.ang_vel;

        result.controls = self.control_seq.get_controls_at_tick(0);
        result.boost = self.boost;
        result
    }
}

#[derive(Debug, Clone, Copy)]
pub struct BallSetup {
    pub pos: Vec3A,
    pub rot_mat: Mat3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
}

impl BallSetup {
    pub const fn new(pos: Vec3A) -> Self {
        Self {
            pos,
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        }
    }

    pub const fn with_rot(mut self, rot: Mat3A) -> Self {
        self.rot_mat = rot;
        self
    }

    pub const fn with_vel(mut self, vel: Vec3A) -> Self {
        self.vel = vel;
        self
    }

    pub const fn with_ang_vel(mut self, ang_vel: Vec3A) -> Self {
        self.ang_vel = ang_vel;
        self
    }

    pub const fn make_ball_state(&self) -> BallState {
        let mut result = BallState::DEFAULT;
        result.phys.pos = self.pos;
        result.phys.rot_mat = self.rot_mat;
        result.phys.vel = self.vel;
        result.phys.ang_vel = self.ang_vel;
        result
    }
}
