use rocketsim_flat as flat;

impl From<flat::GameMode> for crate::GameMode {
    fn from(value: flat::GameMode) -> Self {
        match value {
            flat::GameMode::Soccar => Self::Soccar,
            flat::GameMode::Hoops => Self::Hoops,
            flat::GameMode::Heatseeker => Self::Heatseeker,
            flat::GameMode::Snowday => Self::Snowday,
            flat::GameMode::Dropshot => Self::Dropshot,
            flat::GameMode::TheVoid => Self::TheVoid,
        }
    }
}

impl From<crate::GameMode> for flat::GameMode {
    fn from(value: crate::GameMode) -> Self {
        match value {
            crate::GameMode::Soccar => Self::Soccar,
            crate::GameMode::Hoops => Self::Hoops,
            crate::GameMode::Heatseeker => Self::Heatseeker,
            crate::GameMode::Snowday => Self::Snowday,
            crate::GameMode::Dropshot => Self::Dropshot,
            crate::GameMode::TheVoid => Self::TheVoid,
        }
    }
}

impl From<flat::Team> for crate::Team {
    fn from(value: flat::Team) -> Self {
        match value {
            flat::Team::Blue => Self::Blue,
            flat::Team::Orange => Self::Orange,
        }
    }
}

impl From<crate::Team> for flat::Team {
    fn from(value: crate::Team) -> Self {
        match value {
            crate::Team::Blue => Self::Blue,
            crate::Team::Orange => Self::Orange,
        }
    }
}

impl From<crate::TileState> for flat::TileState {
    fn from(value: crate::TileState) -> Self {
        match value {
            crate::TileState::Full => Self::Full,
            crate::TileState::Damaged => Self::Damaged,
            crate::TileState::Broken => Self::Broken,
        }
    }
}

impl From<flat::TileState> for crate::TileState {
    fn from(value: flat::TileState) -> Self {
        match value {
            flat::TileState::Full => Self::Full,
            flat::TileState::Damaged => Self::Damaged,
            flat::TileState::Broken => Self::Broken,
        }
    }
}

impl From<&flat::DropshotTile> for crate::DropshotTile {
    fn from(value: &flat::DropshotTile) -> Self {
        Self {
            pos: value.pos.into(),
            state: value.state.into(),
        }
    }
}

impl From<&crate::DropshotTile> for flat::DropshotTile {
    fn from(value: &crate::DropshotTile) -> Self {
        Self {
            pos: value.pos.into(),
            state: value.state.into(),
        }
    }
}

impl From<flat::DropshotInfo> for crate::DropshotInfo {
    fn from(value: flat::DropshotInfo) -> Self {
        Self {
            charge_level: value.charge_level,
            accumulated_hit_force: value.accumulated_hit_force,
            y_target_dir: value.y_target_dir,
            has_damaged: value.has_damaged,
            last_damage_tick: value.last_damage_tick,
        }
    }
}

impl From<crate::DropshotInfo> for flat::DropshotInfo {
    fn from(value: crate::DropshotInfo) -> Self {
        Self {
            charge_level: value.charge_level,
            accumulated_hit_force: value.accumulated_hit_force,
            y_target_dir: value.y_target_dir,
            has_damaged: value.has_damaged,
            last_damage_tick: value.last_damage_tick,
        }
    }
}

impl From<flat::BoostPadConfig> for crate::BoostPadConfig {
    fn from(value: flat::BoostPadConfig) -> Self {
        Self {
            pos: value.pos.into(),
            is_big: value.is_big,
        }
    }
}

impl From<crate::BoostPadConfig> for flat::BoostPadConfig {
    fn from(value: crate::BoostPadConfig) -> Self {
        Self {
            pos: value.pos.into(),
            is_big: value.is_big,
        }
    }
}

impl From<flat::BoostPadState> for crate::BoostPadState {
    fn from(_value: flat::BoostPadState) -> Self {
        Self {
            gave_boost_tick_count: None,
        }
    }
}

impl From<crate::BoostPadState> for flat::BoostPadState {
    fn from(value: crate::BoostPadState) -> Self {
        Self {
            is_active: value.gave_boost_tick_count.is_none(),
            cooldown: 0.0,
            cur_locked_car: 0,
            prev_locked_car_idx: 0,
        }
    }
}

impl From<&flat::BoostPadInfo> for crate::BoostPadInfo {
    fn from(value: &flat::BoostPadInfo) -> Self {
        Self {
            config: value.config.into(),
            state: value.state.into(),
        }
    }
}

impl From<&crate::BoostPadInfo> for flat::BoostPadInfo {
    fn from(value: &crate::BoostPadInfo) -> Self {
        Self {
            config: value.config.into(),
            state: value.state.into(),
        }
    }
}

impl From<flat::WheelPairConfig> for crate::WheelPairConfig {
    fn from(value: flat::WheelPairConfig) -> Self {
        Self {
            wheel_radius: value.wheel_radius,
            suspension_rest_length: value.suspension_rest_length,
            connection_point_offset: value.connection_point_offset.into(),
        }
    }
}

impl From<crate::WheelPairConfig> for flat::WheelPairConfig {
    fn from(value: crate::WheelPairConfig) -> Self {
        Self {
            wheel_radius: value.wheel_radius,
            suspension_rest_length: value.suspension_rest_length,
            connection_point_offset: value.connection_point_offset.into(),
        }
    }
}

impl From<flat::CarConfig> for crate::CarBodyConfig {
    fn from(value: flat::CarConfig) -> Self {
        Self {
            hitbox_size: value.hitbox_size.into(),
            hitbox_pos_offset: value.hitbox_pos_offset.into(),
            front_wheels: value.front_wheels.into(),
            back_wheels: value.back_wheels.into(),
            three_wheels: value.three_wheels,
            dodge_deadzone: value.dodge_deadzone,
        }
    }
}

impl From<crate::CarBodyConfig> for flat::CarConfig {
    fn from(value: crate::CarBodyConfig) -> Self {
        Self {
            hitbox_size: value.hitbox_size.into(),
            hitbox_pos_offset: value.hitbox_pos_offset.into(),
            front_wheels: value.front_wheels.into(),
            back_wheels: value.back_wheels.into(),
            three_wheels: value.three_wheels,
            dodge_deadzone: value.dodge_deadzone,
        }
    }
}

impl From<flat::PhysState> for crate::PhysState {
    fn from(value: flat::PhysState) -> Self {
        Self {
            pos: value.pos.into(),
            rot_mat: value.rot_mat.into(),
            vel: value.vel.into(),
            ang_vel: value.ang_vel.into(),
        }
    }
}

impl From<crate::PhysState> for flat::PhysState {
    fn from(value: crate::PhysState) -> Self {
        Self {
            pos: value.pos.into(),
            rot_mat: value.rot_mat.into(),
            vel: value.vel.into(),
            ang_vel: value.ang_vel.into(),
        }
    }
}

impl From<&flat::CarContact> for crate::CarContact {
    fn from(value: &flat::CarContact) -> Self {
        Self {
            other_car_idx: value.other_car_idx,
            cooldown_timer: value.cooldown_timer,
        }
    }
}

impl From<crate::CarContact> for Box<flat::CarContact> {
    fn from(value: crate::CarContact) -> Self {
        let mut new = Self::default();
        new.other_car_idx = value.other_car_idx;
        new.cooldown_timer = value.cooldown_timer;

        new
    }
}

impl From<flat::CarControls> for crate::CarControls {
    fn from(value: flat::CarControls) -> Self {
        Self {
            throttle: value.throttle,
            steer: value.steer,
            pitch: value.pitch,
            yaw: value.yaw,
            roll: value.roll,
            jump: value.jump,
            boost: value.boost,
            handbrake: value.handbrake,
        }
    }
}

impl From<crate::CarControls> for flat::CarControls {
    fn from(value: crate::CarControls) -> Self {
        Self {
            throttle: value.throttle,
            steer: value.steer,
            pitch: value.pitch,
            yaw: value.yaw,
            roll: value.roll,
            jump: value.jump,
            boost: value.boost,
            handbrake: value.handbrake,
        }
    }
}

impl From<&flat::BallHitInfo> for crate::BallHitInfo {
    fn from(value: &flat::BallHitInfo) -> Self {
        Self {
            relative_pos_on_ball: value.relative_pos_on_ball.into(),
            ball_pos: value.ball_pos.into(),
            extra_hit_vel: value.extra_hit_vel.into(),
            tick_count_when_hit: value.tick_count_when_hit,
            tick_count_when_extra_impulse_applied: value.tick_count_when_extra_impulse_applied,
        }
    }
}

impl From<crate::BallHitInfo> for Box<flat::BallHitInfo> {
    fn from(value: crate::BallHitInfo) -> Self {
        let mut new = Self::default();
        new.relative_pos_on_ball = value.relative_pos_on_ball.into();
        new.ball_pos = value.ball_pos.into();
        new.extra_hit_vel = value.extra_hit_vel.into();
        new.tick_count_when_hit = value.tick_count_when_hit;
        new.tick_count_when_extra_impulse_applied = value.tick_count_when_extra_impulse_applied;

        new
    }
}

impl From<&flat::CarState> for crate::CarState {
    fn from(value: &flat::CarState) -> Self {
        Self {
            phys: value.physics.into(),
            is_on_ground: value.is_on_ground,
            wheels_with_contact: value.wheels_with_contact.into(),
            has_jumped: value.has_jumped,
            has_double_jumped: value.has_double_jumped,
            has_flipped: value.has_flipped,
            flip_rel_torque: value.flip_rel_torque.into(),
            jump_time: value.jump_time,
            flip_time: value.flip_time,
            is_flipping: value.is_flipping,
            is_jumping: value.is_jumping,
            air_time: value.air_time,
            air_time_since_jump: value.air_time_since_jump,
            boost: value.boost,
            time_since_boosted: value.time_since_boosted,
            is_boosting: value.is_boosting,
            boosting_time: value.boosting_time,
            is_supersonic: value.is_supersonic,
            supersonic_time: value.supersonic_time,
            handbrake_val: value.handbrake_val,
            is_auto_flipping: value.is_auto_flipping,
            auto_flip_timer: value.auto_flip_timer,
            auto_flip_torque_scale: value.auto_flip_torque_scale,
            world_contact_normal: value.world_contact_normal.map(Into::into),
            car_contact: value.car_contact.as_deref().map(Into::into),
            is_demoed: value.is_demoed,
            demo_respawn_timer: value.demo_respawn_timer,
            ball_hit_info: value.ball_hit_info.as_deref().map(Into::into),
            controls: value.last_controls.into(),
            prev_controls: value.last_controls.into(),
        }
    }
}

impl From<crate::CarState> for Box<flat::CarState> {
    fn from(value: crate::CarState) -> Self {
        let mut new = Self::default();
        new.physics = value.phys.into();
        new.is_on_ground = value.is_on_ground;
        new.wheels_with_contact = value.wheels_with_contact.into();
        new.has_jumped = value.has_jumped;
        new.has_double_jumped = value.has_double_jumped;
        new.has_flipped = value.has_flipped;
        new.flip_rel_torque = value.flip_rel_torque.into();
        new.jump_time = value.jump_time;
        new.flip_time = value.flip_time;
        new.is_flipping = value.is_flipping;
        new.is_jumping = value.is_jumping;
        new.air_time = value.air_time;
        new.air_time_since_jump = value.air_time_since_jump;
        new.boost = value.boost;
        new.time_since_boosted = value.time_since_boosted;
        new.is_boosting = value.is_boosting;
        new.boosting_time = value.boosting_time;
        new.is_supersonic = value.is_supersonic;
        new.supersonic_time = value.supersonic_time;
        new.handbrake_val = value.handbrake_val;
        new.is_auto_flipping = value.is_auto_flipping;
        new.auto_flip_timer = value.auto_flip_timer;
        new.auto_flip_torque_scale = value.auto_flip_torque_scale;
        new.world_contact_normal = value.world_contact_normal.map(Into::into);
        new.car_contact = value.car_contact.map(Into::into);
        new.is_demoed = value.is_demoed;
        new.demo_respawn_timer = value.demo_respawn_timer;
        new.ball_hit_info = value.ball_hit_info.map(Into::into);
        new.last_controls = value.prev_controls.into();

        new
    }
}

impl From<&flat::CarInfo> for crate::CarInfo {
    fn from(value: &flat::CarInfo) -> Self {
        Self {
            id: value.id,
            team: value.team.into(),
            state: value.state.as_ref().into(),
            config: value.config.into(),
        }
    }
}

impl From<&crate::CarInfo> for flat::CarInfo {
    fn from(value: &crate::CarInfo) -> Self {
        Self {
            id: value.id,
            team: value.team.into(),
            state: value.state.into(),
            config: value.config.into(),
        }
    }
}

impl From<flat::HeatseekerInfo> for crate::HeatseekerInfo {
    fn from(value: flat::HeatseekerInfo) -> Self {
        Self {
            y_target_dir: value.y_target_dir,
            cur_target_speed: value.cur_target_speed,
            time_since_hit: value.time_since_hit,
        }
    }
}

impl From<crate::HeatseekerInfo> for flat::HeatseekerInfo {
    fn from(value: crate::HeatseekerInfo) -> Self {
        Self {
            y_target_dir: value.y_target_dir,
            cur_target_speed: value.cur_target_speed,
            time_since_hit: value.time_since_hit,
        }
    }
}

impl From<flat::BallState> for crate::BallState {
    fn from(value: flat::BallState) -> Self {
        Self {
            phys: value.physics.into(),
            hs_info: value.hs_info.into(),
            ds_info: value.ds_info.into(),
        }
    }
}

impl From<crate::BallState> for flat::BallState {
    fn from(value: crate::BallState) -> Self {
        Self {
            physics: value.phys.into(),
            hs_info: value.hs_info.into(),
            ds_info: value.ds_info.into(),
        }
    }
}

impl From<&flat::GameState> for crate::GameState {
    fn from(value: &flat::GameState) -> Self {
        Self {
            TICK_RATE: value.TICK_RATE,
            tick_count: value.tick_count,
            game_mode: value.game_mode.into(),
            cars: value
                .cars
                .as_ref()
                .map(|cars| cars.iter().map(Into::into).collect()),
            ball: value.ball.into(),
            pads: value
                .pads
                .as_ref()
                .map(|pads| pads.iter().map(Into::into).collect()),
            tiles: value.tiles.as_deref().map(|tiles| {
                [
                    tiles.blue_tiles.iter().map(Into::into).collect(),
                    tiles.orange_tiles.iter().map(Into::into).collect(),
                ]
            }),
        }
    }
}

impl From<&crate::GameState> for Box<flat::GameState> {
    fn from(value: &crate::GameState) -> Self {
        let mut new = Self::default();
        new.TICK_RATE = value.TICK_RATE;
        new.tick_count = value.tick_count;
        new.game_mode = value.game_mode.into();
        new.cars = value
            .cars
            .as_ref()
            .map(|cars| cars.iter().map(Into::into).collect());
        new.ball = value.ball.into();
        new.pads = value
            .pads
            .as_ref()
            .map(|pads| pads.iter().map(Into::into).collect());
        new.tiles = value.tiles.as_ref().map(|[blue_tiles, orange_tiles]| {
            let mut new = Box::<flat::DropshotTilesByTeam>::default();
            new.blue_tiles = blue_tiles.iter().map(Into::into).collect();
            new.orange_tiles = orange_tiles.iter().map(Into::into).collect();

            new
        });

        new
    }
}
