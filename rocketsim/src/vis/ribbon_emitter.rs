#![allow(dead_code)]

use std::collections::VecDeque;

use glam::{FloatExt, Vec3A};

use crate::vis::backend::{Color, VisRenderLinePoint, VisRenderState};

#[derive(Debug, Copy, Clone)]
struct RibbonPoint {
    pos: Vec3A,
    vel: Vec3A,
    time_alive: f32,
    lifetime: f32,
    connected: bool,
}

impl RibbonPoint {
    fn is_alive(&self) -> bool {
        self.time_alive < self.lifetime
    }

    fn time_left(&self) -> f32 {
        (self.lifetime - self.time_alive).max(0.0)
    }

    fn time_left_frac(&self) -> f32 {
        self.time_left() / self.lifetime
    }
}

#[derive(Debug, Copy, Clone)]
pub struct RibbonConfig {
    pub emit_rate: f32,
    pub emit_lifetime: f32,

    pub start_color: Color,
    pub end_color: Color,

    pub start_width: f32,
    pub end_width: f32,

    pub width_exponent: f32,
    pub color_exponent: f32,
}

impl Default for RibbonConfig {
    fn default() -> Self {
        Self {
            emit_rate: 30.0,
            emit_lifetime: 1.0,

            start_color: Color::WHITE,
            end_color: Color::WHITE,

            start_width: 30.0,
            end_width: 0.0,

            width_exponent: 1.0,
            color_exponent: 1.0,
        }
    }
}

/// Based on: https://github.com/ZealanL/RocketSimVis/blob/main/src/ribbon.py
#[derive(Debug, Clone)]
pub struct RibbonEmitter {
    config: RibbonConfig,
    points: VecDeque<RibbonPoint>,
    time_since_emit: f32,
}

impl RibbonEmitter {
    pub fn new(config: RibbonConfig) -> Self {
        Self {
            config,
            points: VecDeque::new(),
            time_since_emit: 0.0,
        }
    }

    pub fn update(&mut self, can_emit: bool, emit_pos: Vec3A, emit_vel: Vec3A, delta_time: f32) {
        if self.time_since_emit < (1.0 / self.config.emit_rate) {
            self.time_since_emit += delta_time;
        } else if can_emit {
            self.time_since_emit = 0.0;
            let new_point = RibbonPoint {
                pos: emit_pos,
                vel: emit_vel,
                time_alive: 0.0,
                lifetime: self.config.emit_lifetime,
                connected: true,
            };
            self.points.push_front(new_point);
        }

        if !can_emit {
            // Disconnect first point
            if let Some(first_point) = self.points.front_mut() {
                first_point.connected = false;
            }
        }

        // Update points
        for point in self.points.iter_mut() {
            point.pos += point.vel * delta_time;
            point.time_alive += delta_time;
        }

        // Remove dead points
        while let Some(last_point) = self.points.back() {
            if !last_point.is_alive() {
                self.points.pop_back();
            } else {
                break;
            }
        }
    }

    pub fn reset(&mut self) {
        *self = RibbonEmitter::new(self.config);
    }

    pub fn render(&mut self, render_state: &mut VisRenderState, emit_pos: Vec3A) {
        if self.points.is_empty() {
            return;
        }

        let mut prev_pos = emit_pos;
        let mut prev_color = self.config.start_color;
        let mut prev_width = self.config.start_width;
        for (i, point) in self.points.iter().enumerate() {
            let is_oldest = i == (self.points.len() - 1);

            let lifetime_frac = point.time_left_frac();
            let cur_pos = if is_oldest {
                Vec3A::lerp(prev_pos, point.pos, lifetime_frac)
            } else {
                point.pos
            };
            let cur_color = Color::lerp(
                self.config.end_color,
                self.config.start_color,
                lifetime_frac.powf(self.config.color_exponent),
            );

            let cur_width = if point.connected && !is_oldest {
                f32::lerp(
                    self.config.end_width,
                    self.config.start_width,
                    lifetime_frac.powf(self.config.width_exponent),
                )
            } else {
                0.0
            };

            if point.connected {
                render_state.lines.push(vec![
                    VisRenderLinePoint {
                        pos: prev_pos,
                        width: prev_width,
                        color: prev_color,
                    },
                    VisRenderLinePoint {
                        pos: cur_pos,
                        width: cur_width,
                        color: cur_color,
                    },
                ]);
            }

            prev_pos = cur_pos;
            prev_color = cur_color;
            prev_width = cur_width;
        }
    }
}
