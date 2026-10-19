use glam::Vec3A;
use rocketsim_vis::proto::VisFrame;

use crate::{
    effects::fx::CarFx,
    geometry::lines::{RibbonConfig, RibbonEmitter, Segment, ground_line},
    settings::ViewerSettings,
};

use super::session;

pub(super) struct EffectState {
    ball_ribbon: RibbonEmitter,
    boost_ribbons: Vec<RibbonEmitter>,
    car_fx: CarFx,
    last_tick: Option<u64>,
    last_ball_pos: Option<Vec3A>,
    last_car_positions: Vec<Vec3A>,
    last_fx_segments: Vec<Segment>,
}

pub(super) struct EffectUpdate {
    pub(super) segments: Vec<Segment>,
    pub(super) rewound: bool,
}

impl Default for EffectState {
    fn default() -> Self {
        Self {
            ball_ribbon: RibbonEmitter::new(RibbonConfig::ball_trail()),
            boost_ribbons: Vec::new(),
            car_fx: CarFx::default(),
            last_tick: None,
            last_ball_pos: None,
            last_car_positions: Vec::new(),
            last_fx_segments: Vec::new(),
        }
    }
}

impl EffectState {
    pub(super) fn reset_session(&mut self) {
        *self = Self::default();
    }

    pub(super) fn update(&mut self, frame: &VisFrame, settings: &ViewerSettings) -> EffectUpdate {
        let state = &frame.state;
        let tick_now = state.tick_count;
        let advance = session::tick_advance(self.last_tick, tick_now);
        let ball_pos = state.ball.phys.pos;
        let ball_teleport = self
            .last_ball_pos
            .is_some_and(|previous| previous.distance(ball_pos) > 1200.0);
        if advance.reset || ball_teleport {
            self.ball_ribbon = RibbonEmitter::new(RibbonConfig::ball_trail());
            self.boost_ribbons.clear();
            self.car_fx = CarFx::default();
            self.last_fx_segments.clear();
        }
        if advance.reset {
            self.last_car_positions.clear();
        }
        self.last_tick = Some(tick_now);
        self.last_ball_pos = Some(ball_pos);
        for _ in 0..advance.steps {
            self.ball_ribbon
                .update(true, ball_pos, Vec3A::ZERO, frame.dt);
        }
        let mut segments = if settings.show_trail {
            self.ball_ribbon.segments(ball_pos)
        } else {
            Vec::new()
        };
        let floor_z = rocketsim::consts::arena::get_aabb(state.game_mode()).min.z;
        if settings.show_ground_line {
            segments.push(ground_line(ball_pos, floor_z));
        }
        if advance.steps > 0 {
            for _ in 0..advance.steps {
                self.last_fx_segments = self.car_fx.update(state, frame.dt);
            }
        }
        segments.extend(self.last_fx_segments.iter().cloned());
        for (index, (_, car)) in state.cars.iter().enumerate() {
            while self.boost_ribbons.len() <= index {
                self.boost_ribbons
                    .push(RibbonEmitter::new(RibbonConfig::boost()));
            }
            let car_pos = car.phys.pos;
            if self
                .last_car_positions
                .get(index)
                .is_some_and(|previous| previous.distance(car_pos) > 1200.0)
            {
                self.boost_ribbons[index] = RibbonEmitter::new(RibbonConfig::boost());
            }
            let forward = car.phys.get_forward_dir();
            let emit_pos = car_pos - forward * 60.0;
            let emit_vel = -forward * 300.0;
            for _ in 0..advance.steps {
                self.boost_ribbons[index].update(car.is_boosting, emit_pos, emit_vel, frame.dt);
            }
            if settings.show_boost {
                segments.extend(self.boost_ribbons[index].segments(emit_pos));
            }
        }
        self.last_car_positions = state.cars.iter().map(|(_, car)| car.phys.pos).collect();
        EffectUpdate {
            segments,
            rewound: advance.reset,
        }
    }
}
