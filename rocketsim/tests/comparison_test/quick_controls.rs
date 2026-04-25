use rocketsim::CarControls;

pub const fn quick_drive(throttle: f32, steer: f32, boost: bool, handbrake: bool) -> CarControls {
    CarControls {
        throttle,
        steer,
        boost,
        handbrake,

        pitch: 0.0,
        yaw: 0.0,
        roll: 0.0,
        jump: false,
    }
}

pub const fn quick_air(pitch: f32, yaw: f32, roll: f32, jump: bool, boost: bool) -> CarControls {
    CarControls {
        throttle: 0.0,
        steer: 0.0,

        pitch,
        yaw,
        roll,
        jump,
        boost,

        handbrake: false,
    }
}
