use glam::Vec3A;
use std::collections::VecDeque;

#[derive(Debug, Copy, Clone)]
struct RibbonPoint {
    pos: Vec3A,
    vel: Vec3A,
    time_active: f32,
    connected: bool,
}

/// Ref: https://github.com/ZealanL/RocketSimVis/blob/main/src/ribbon.py
#[derive(Debug, Clone)]
pub struct RibbonEmitter {
    points: VecDeque<RibbonPoint>,
    time_since_emit: f32,
}

impl RibbonEmitter {
    pub fn new() -> Self {
        Self {
            points: VecDeque::new(),
            time_since_emit: 0.0,
        }
    }

    pub fn update(
        &mut self,
        can_emit: bool,
        emit_delay: f32,
        emit_pos: Vec3A,
        emit_vel: Vec3A,
        lifetime: f32,
        delta_time: f32,
    ) {
        if self.time_since_emit < emit_delay {
            self.time_since_emit += delta_time;

            // Disconnect first point
            if let Some(first_point) = self.points.front_mut() {
                first_point.connected = false;
            }
        } else if can_emit {
            self.time_since_emit = 0.0;
            let new_point = RibbonPoint {
                pos: emit_pos,
                vel: emit_vel,
                time_active: 0.0,
                connected: true,
            };
            self.points.push_front(new_point);
        }

        // Update points
        for point in self.points.iter_mut() {
            point.pos += point.vel * delta_time;
            point.time_active += delta_time;
        }

        // Remove dead points
        while let Some(last_point) = self.points.back() {
            if last_point.time_active > lifetime {
                self.points.pop_back();
            } else {
                break;
            }
        }
    }

    pub fn reset(&mut self) {
        *self = RibbonEmitter::new();
    }
}
