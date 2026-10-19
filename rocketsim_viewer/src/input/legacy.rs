//! Legacy binding tables kept for settings and external compatibility.

use serde::{Deserialize, Serialize};

use super::{
    actions::{ACTIONS, Action, ActionMap},
    sources::{InputBinding, InputSource, Key, MouseButton, PadAnalog, PadAxis, PadButton},
};

/// Car driving inputs (viewer keyboard + mouse).
///
/// Defaults copy today's mapping: `W`/`S` throttle, `A`/`D` steer,
/// `Q`/`E` roll, `Shift` handbrake, left-click jump, right-click
/// boost. Steer sign stays as probed: left sends +1.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct CarBindings {
    #[serde(default = "default_throttle_fwd")]
    pub throttle_fwd: Key,
    #[serde(default = "default_throttle_back")]
    pub throttle_back: Key,
    #[serde(default = "default_steer_left")]
    pub steer_left: Key,
    #[serde(default = "default_steer_right")]
    pub steer_right: Key,
    #[serde(default = "default_roll_left")]
    pub roll_left: Key,
    #[serde(default = "default_roll_right")]
    pub roll_right: Key,
    #[serde(default = "default_handbrake")]
    pub handbrake: Key,
    #[serde(default = "default_jump")]
    pub jump: InputBinding,
    #[serde(default = "default_boost")]
    pub boost: InputBinding,
}

fn default_throttle_fwd() -> Key {
    Key::W
}

fn default_throttle_back() -> Key {
    Key::S
}

fn default_steer_left() -> Key {
    Key::A
}

fn default_steer_right() -> Key {
    Key::D
}

fn default_roll_left() -> Key {
    Key::Q
}

fn default_roll_right() -> Key {
    Key::E
}

fn default_handbrake() -> Key {
    Key::Shift
}

fn default_jump() -> InputBinding {
    InputBinding::Mouse(MouseButton::Primary)
}

fn default_boost() -> InputBinding {
    InputBinding::Mouse(MouseButton::Secondary)
}

impl Default for CarBindings {
    fn default() -> Self {
        Self {
            throttle_fwd: default_throttle_fwd(),
            throttle_back: default_throttle_back(),
            steer_left: default_steer_left(),
            steer_right: default_steer_right(),
            roll_left: default_roll_left(),
            roll_right: default_roll_right(),
            handbrake: default_handbrake(),
            jump: default_jump(),
            boost: default_boost(),
        }
    }
}

/// One-shot sim actions (viewer keys, sent as button bits).
///
/// Defaults copy today's mapping: `Backspace` kickoff reset,
/// `F2` dribble, `F4` launch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActionBindings {
    #[serde(default = "default_kickoff_reset")]
    pub kickoff_reset: Key,
    #[serde(default = "default_dribble")]
    pub dribble: Key,
    #[serde(default = "default_launch")]
    pub launch: Key,
}

fn default_kickoff_reset() -> Key {
    Key::Backspace
}

fn default_dribble() -> Key {
    Key::F2
}

fn default_launch() -> Key {
    Key::F4
}

impl Default for ActionBindings {
    fn default() -> Self {
        Self {
            kickoff_reset: default_kickoff_reset(),
            dribble: default_dribble(),
            launch: default_launch(),
        }
    }
}

/// Camera shortcut keys.
///
/// Defaults copy today's mapping: `1`-`8` follow, `9` director,
/// `0` auto-cam, `F` free camera, `C` next car, `Space` ball cam.
/// Free-camera flight reuses the car keys (`WASD` fly, `Space` up,
/// `Shift` down), so it needs no extra fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct CameraBindings {
    #[serde(default = "default_follow")]
    pub follow: [Key; 8],
    #[serde(default = "default_director")]
    pub director: Key,
    #[serde(default = "default_autocam")]
    pub autocam: Key,
    #[serde(default = "default_free")]
    pub free: Key,
    #[serde(default = "default_cycle")]
    pub cycle: Key,
    #[serde(default = "default_ball_cam")]
    pub ball_cam: Key,
}

fn default_follow() -> [Key; 8] {
    [
        Key::Num1,
        Key::Num2,
        Key::Num3,
        Key::Num4,
        Key::Num5,
        Key::Num6,
        Key::Num7,
        Key::Num8,
    ]
}

fn default_director() -> Key {
    Key::Num9
}

fn default_autocam() -> Key {
    Key::Num0
}

fn default_free() -> Key {
    Key::F
}

fn default_cycle() -> Key {
    Key::C
}

fn default_ball_cam() -> Key {
    Key::Space
}

impl Default for CameraBindings {
    fn default() -> Self {
        Self {
            follow: default_follow(),
            director: default_director(),
            autocam: default_autocam(),
            free: default_free(),
            cycle: default_cycle(),
            ball_cam: default_ball_cam(),
        }
    }
}

/// Viewer UI keys. Default copies today: `Escape` toggles the menu.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct UiBindings {
    #[serde(default = "default_menu_toggle")]
    pub menu_toggle: Key,
    /// Gamepad menu toggle (PS Options / Xbox Menu / Switch +).
    #[serde(default = "default_menu_toggle_pad")]
    pub menu_toggle_pad: PadButton,
    /// Gamepad menu select (PS Cross / Xbox A).
    #[serde(default = "default_ui_activate")]
    pub ui_activate: PadButton,
    /// Gamepad menu back (PS Circle / Xbox B).
    #[serde(default = "default_ui_back")]
    pub ui_back: PadButton,
}

fn default_menu_toggle() -> Key {
    Key::Escape
}

fn default_menu_toggle_pad() -> PadButton {
    PadButton::Start
}

fn default_ui_activate() -> PadButton {
    PadButton::South
}

fn default_ui_back() -> PadButton {
    PadButton::East
}

impl Default for UiBindings {
    fn default() -> Self {
        Self {
            menu_toggle: default_menu_toggle(),
            menu_toggle_pad: default_menu_toggle_pad(),
            ui_activate: default_ui_activate(),
            ui_back: default_ui_back(),
        }
    }
}

/// Gamepad mapping for the drive demo.
///
/// Defaults copy today's `gilrs` mapping: left stick steers and
/// pitches, right trigger drives, left trigger brakes, bumpers
/// roll, `South` jumps, `East` boosts, `West` handbrakes. Right stick
/// looks (camera swivel, viewer-side). Trigger fields read the analog
/// value (0 to 1); the rest read pressed. An axis-bound throttle reads
/// its positive half, an axis-bound brake its negative half, so one
/// stick can do push-to-drive / pull-to-brake.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PadBindings {
    #[serde(default = "default_pad_jump")]
    pub jump: PadButton,
    #[serde(default = "default_pad_boost")]
    pub boost: PadButton,
    #[serde(default = "default_pad_handbrake")]
    pub handbrake: PadButton,
    #[serde(default = "default_pad_roll_left")]
    pub roll_left: PadButton,
    #[serde(default = "default_pad_roll_right")]
    pub roll_right: PadButton,
    #[serde(default = "default_pad_throttle")]
    pub throttle_trigger: PadAnalog,
    #[serde(default = "default_pad_brake")]
    pub brake_trigger: PadAnalog,
    #[serde(default = "default_pad_steer_axis")]
    pub steer_axis: PadAxis,
    #[serde(default = "default_pad_pitch_axis")]
    pub pitch_axis: PadAxis,
    /// Right-stick look, left/right (viewer camera swivel).
    #[serde(default = "default_pad_look_x")]
    pub look_x: PadAxis,
    /// Right-stick look, up/down (viewer camera swivel).
    #[serde(default = "default_pad_look_y")]
    pub look_y: PadAxis,
    /// Ball-cam toggle (viewer follow view; RL uses Triangle/Y).
    #[serde(default = "default_pad_ball_cam")]
    pub ball_cam: PadButton,
    /// Action buttons (viewer folds them into the buttons bitmask).
    /// Unbound by default: bind them below to drive purely by pad.
    #[serde(default)]
    pub actions: PadActionBindings,
    /// Camera shortcuts (viewer-side). Unbound by default.
    #[serde(default)]
    pub camera: PadCameraBindings,
    /// Stick deadzone, 0..0.5 (clamped at use; default 0.1). Covers
    /// drive sticks and right-stick look.
    #[serde(default = "default_pad_deadzone")]
    pub stick_deadzone: f32,
}

fn default_pad_jump() -> PadButton {
    PadButton::South
}

fn default_pad_boost() -> PadButton {
    PadButton::East
}

fn default_pad_handbrake() -> PadButton {
    PadButton::West
}

fn default_pad_roll_left() -> PadButton {
    PadButton::LeftTrigger
}

fn default_pad_roll_right() -> PadButton {
    PadButton::RightTrigger
}

fn default_pad_throttle() -> PadAnalog {
    PadAnalog::Button(PadButton::RightTrigger2)
}

fn default_pad_brake() -> PadAnalog {
    PadAnalog::Button(PadButton::LeftTrigger2)
}

fn default_pad_steer_axis() -> PadAxis {
    PadAxis::LeftStickX
}

fn default_pad_pitch_axis() -> PadAxis {
    PadAxis::LeftStickY
}

fn default_pad_look_x() -> PadAxis {
    PadAxis::RightStickX
}

fn default_pad_look_y() -> PadAxis {
    PadAxis::RightStickY
}

fn default_pad_ball_cam() -> PadButton {
    PadButton::North
}

fn default_pad_deadzone() -> f32 {
    0.1
}

impl Default for PadBindings {
    fn default() -> Self {
        Self {
            jump: default_pad_jump(),
            boost: default_pad_boost(),
            handbrake: default_pad_handbrake(),
            roll_left: default_pad_roll_left(),
            roll_right: default_pad_roll_right(),
            throttle_trigger: default_pad_throttle(),
            brake_trigger: default_pad_brake(),
            steer_axis: default_pad_steer_axis(),
            pitch_axis: default_pad_pitch_axis(),
            look_x: default_pad_look_x(),
            look_y: default_pad_look_y(),
            ball_cam: default_pad_ball_cam(),
            stick_deadzone: default_pad_deadzone(),
            actions: PadActionBindings::default(),
            camera: PadCameraBindings::default(),
        }
    }
}

/// Pad action buttons, folded into the control bitmask viewer-side.
/// Every slot binds like the rest; `Unknown` means unbound (capture
/// never assigns it, and it never fires). Bind any of them to drive
/// the whole demo without touching keyboard or mouse.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct PadActionBindings {
    #[serde(default)]
    pub kickoff_reset: PadButton,
    #[serde(default)]
    pub dribble: PadButton,
    #[serde(default)]
    pub launch: PadButton,
}

/// Pad camera shortcuts, viewer-side. `Unknown` = unbound, as above.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct PadCameraBindings {
    #[serde(default)]
    pub follow: [PadButton; 8],
    #[serde(default)]
    pub director: PadButton,
    #[serde(default)]
    pub autocam: PadButton,
    #[serde(default)]
    pub free: PadButton,
    #[serde(default)]
    pub cycle: PadButton,
}

/// All binding tables. Stored as `[bindings]` in `settings.toml`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Bindings {
    #[serde(default)]
    pub car: CarBindings,
    #[serde(default)]
    pub actions: ActionBindings,
    #[serde(default)]
    pub camera: CameraBindings,
    #[serde(default)]
    pub ui: UiBindings,
    #[serde(default)]
    pub pad: PadBindings,
}

impl Bindings {
    pub fn conflicts(&self) -> Vec<String> {
        let mut entries: Vec<(String, InputBinding)> = vec![
            (
                "car.throttle_fwd".into(),
                InputBinding::Key(self.car.throttle_fwd),
            ),
            (
                "car.throttle_back".into(),
                InputBinding::Key(self.car.throttle_back),
            ),
            (
                "car.steer_left".into(),
                InputBinding::Key(self.car.steer_left),
            ),
            (
                "car.steer_right".into(),
                InputBinding::Key(self.car.steer_right),
            ),
            (
                "car.roll_left".into(),
                InputBinding::Key(self.car.roll_left),
            ),
            (
                "car.roll_right".into(),
                InputBinding::Key(self.car.roll_right),
            ),
            (
                "car.handbrake".into(),
                InputBinding::Key(self.car.handbrake),
            ),
            ("car.jump".into(), self.car.jump),
            ("car.boost".into(), self.car.boost),
            (
                "actions.kickoff_reset".into(),
                InputBinding::Key(self.actions.kickoff_reset),
            ),
            (
                "actions.dribble".into(),
                InputBinding::Key(self.actions.dribble),
            ),
            (
                "actions.launch".into(),
                InputBinding::Key(self.actions.launch),
            ),
            (
                "camera.director".into(),
                InputBinding::Key(self.camera.director),
            ),
            (
                "camera.autocam".into(),
                InputBinding::Key(self.camera.autocam),
            ),
            ("camera.free".into(), InputBinding::Key(self.camera.free)),
            ("camera.cycle".into(), InputBinding::Key(self.camera.cycle)),
            (
                "camera.ball_cam".into(),
                InputBinding::Key(self.camera.ball_cam),
            ),
            (
                "ui.menu_toggle".into(),
                InputBinding::Key(self.ui.menu_toggle),
            ),
        ];
        for (i, key) in self.camera.follow.iter().enumerate() {
            entries.push((format!("camera.follow{}", i + 1), InputBinding::Key(*key)));
        }
        let mut conflicts = Vec::new();
        for i in 0..entries.len() {
            for j in i + 1..entries.len() {
                if entries[i].1 != InputBinding::Unknown && entries[i].1 == entries[j].1 {
                    conflicts.push(format!("{} conflicts with {}", entries[i].0, entries[j].0));
                }
            }
        }
        conflicts
    }
}

impl ActionMap {
    pub fn from_legacy(b: &Bindings) -> Self {
        let mut m = Self::default();
        let mut put = |a, slot, s| m.set(a, slot, s);
        for (a, k) in [
            (Action::ThrottleForward, b.car.throttle_fwd),
            (Action::ThrottleBack, b.car.throttle_back),
            (Action::SteerLeft, b.car.steer_left),
            (Action::SteerRight, b.car.steer_right),
            (Action::RollLeft, b.car.roll_left),
            (Action::RollRight, b.car.roll_right),
            (Action::Handbrake, b.car.handbrake),
            (Action::AirRoll, b.car.handbrake),
            (Action::PitchForward, b.car.throttle_fwd),
            (Action::PitchBack, b.car.throttle_back),
            (Action::YawLeft, b.car.steer_left),
            (Action::YawRight, b.car.steer_right),
            (Action::KickoffReset, b.actions.kickoff_reset),
            (Action::Dribble, b.actions.dribble),
            (Action::Launch, b.actions.launch),
            (Action::Director, b.camera.director),
            (Action::AutoCam, b.camera.autocam),
            (Action::FreeCam, b.camera.free),
            (Action::NextCam, b.camera.cycle),
            (Action::BallCam, b.camera.ball_cam),
            (Action::MenuToggle, b.ui.menu_toggle),
        ] {
            put(a, 0, InputSource::Key(k));
        }
        for (i, k) in b.camera.follow.iter().enumerate() {
            put(ACTIONS[16 + i], 0, InputSource::Key(*k));
        }
        for (a, x) in [(Action::Jump, b.car.jump), (Action::Boost, b.car.boost)] {
            put(a, 0, from_old(x));
        }
        for (a, p) in [
            (Action::Jump, b.pad.jump),
            (Action::Boost, b.pad.boost),
            (Action::Handbrake, b.pad.handbrake),
            (Action::AirRoll, b.pad.handbrake),
            (Action::RollLeft, b.pad.roll_left),
            (Action::RollRight, b.pad.roll_right),
            (Action::BallCam, b.pad.ball_cam),
            (Action::MenuToggle, b.ui.menu_toggle_pad),
            (Action::MenuAccept, b.ui.ui_activate),
            (Action::MenuBack, b.ui.ui_back),
        ] {
            put(a, 1, pad_button(p));
        }
        for (a, p) in [
            (Action::ThrottleForward, b.pad.throttle_trigger),
            (Action::ThrottleBack, b.pad.brake_trigger),
        ] {
            put(a, 1, analog(p));
        }
        for (a, ax, sgn) in [
            (Action::SteerLeft, b.pad.steer_axis, -1),
            (Action::SteerRight, b.pad.steer_axis, 1),
            (Action::YawLeft, b.pad.steer_axis, -1),
            (Action::YawRight, b.pad.steer_axis, 1),
            (Action::PitchForward, b.pad.pitch_axis, 1),
            (Action::PitchBack, b.pad.pitch_axis, -1),
            (Action::LookLeft, b.pad.look_x, -1),
            (Action::LookRight, b.pad.look_x, 1),
            (Action::LookUp, b.pad.look_y, 1),
            (Action::LookDown, b.pad.look_y, -1),
            (Action::SwivelLeft, b.pad.look_x, -1),
            (Action::SwivelRight, b.pad.look_x, 1),
            (Action::SwivelUp, b.pad.look_y, 1),
            (Action::SwivelDown, b.pad.look_y, -1),
        ] {
            put(
                a,
                1,
                if ax == PadAxis::Unknown {
                    InputSource::Unbound
                } else {
                    InputSource::PadAxis {
                        axis: ax,
                        sign: sgn,
                    }
                },
            );
        }
        for (i, p) in b.pad.camera.follow.iter().enumerate() {
            if *p != PadButton::Unknown {
                put(ACTIONS[16 + i], 1, InputSource::PadButton(*p));
            }
        }
        for (a, p) in [
            (Action::KickoffReset, b.pad.actions.kickoff_reset),
            (Action::Dribble, b.pad.actions.dribble),
            (Action::Launch, b.pad.actions.launch),
            (Action::Director, b.pad.camera.director),
            (Action::AutoCam, b.pad.camera.autocam),
            (Action::FreeCam, b.pad.camera.free),
            (Action::NextCam, b.pad.camera.cycle),
        ] {
            if p != PadButton::Unknown {
                put(a, 1, InputSource::PadButton(p));
            }
        }
        m
    }
}

fn from_old(b: InputBinding) -> InputSource {
    match b {
        InputBinding::Key(k) => InputSource::Key(k),
        InputBinding::Mouse(b) => InputSource::Mouse(b),
        InputBinding::Unknown => InputSource::Unbound,
    }
}
fn pad_button(button: PadButton) -> InputSource {
    if button == PadButton::Unknown {
        InputSource::Unbound
    } else {
        InputSource::PadButton(button)
    }
}
fn analog(a: PadAnalog) -> InputSource {
    match a {
        PadAnalog::Button(b) => pad_button(b),
        PadAnalog::Axis(PadAxis::Unknown) | PadAnalog::NegAxis(PadAxis::Unknown) => {
            InputSource::Unbound
        }
        PadAnalog::Axis(a) => InputSource::PadAxis { axis: a, sign: 1 },
        PadAnalog::NegAxis(a) => InputSource::PadAxis { axis: a, sign: -1 },
        PadAnalog::Raw(c) => InputSource::PadRaw { code: c, sign: 1 },
        PadAnalog::RawNeg(c) => InputSource::PadRaw { code: c, sign: -1 },
    }
}

#[cfg(test)]
mod tests {
    use super::{super::sources::clamp_deadzone, *};

    #[test]
    fn legacy_defaults_sparse_tables_and_roundtrip() {
        let defaults = Bindings::default();
        assert_eq!(defaults.car.throttle_fwd, Key::W);
        assert_eq!(defaults.car.jump, InputBinding::Mouse(MouseButton::Primary));
        assert_eq!(defaults.camera.follow[0], Key::Num1);
        assert_eq!(defaults.pad.jump, PadButton::South);
        assert_eq!(defaults.pad.stick_deadzone, 0.1);
        let encoded = toml::to_string(&defaults).unwrap();
        assert_eq!(toml::from_str::<Bindings>(&encoded).unwrap(), defaults);

        let sparse: Bindings = toml::from_str(
            "unknown_future = true\n[car]\nthrottle_fwd = \"ArrowUp\"\njump = \"Space\"\n[pad]\nthrottle_trigger = \"-LeftStickY\"",
        ).unwrap();
        assert_eq!(sparse.car.throttle_fwd, Key::ArrowUp);
        assert_eq!(sparse.car.throttle_back, Key::S);
        assert_eq!(sparse.car.jump, InputBinding::Key(Key::Space));
        assert_eq!(
            sparse.pad.throttle_trigger,
            PadAnalog::NegAxis(PadAxis::LeftStickY)
        );
        assert_eq!(sparse.camera.ball_cam, Key::Space);
    }

    #[test]
    fn pad_analog_toml_scenarios() {
        let cases = [
            ("RightTrigger2", PadAnalog::Button(PadButton::RightTrigger2)),
            ("Unknown", PadAnalog::Button(PadButton::Unknown)),
            ("LeftStickY", PadAnalog::Axis(PadAxis::LeftStickY)),
            ("-LeftStickY", PadAnalog::NegAxis(PadAxis::LeftStickY)),
            ("+2", PadAnalog::Raw(2)),
            ("-5", PadAnalog::RawNeg(5)),
        ];
        for (text, expected) in cases {
            let parsed = toml::from_str::<PadBindings>(&format!("throttle_trigger = {text:?}"))
                .unwrap()
                .throttle_trigger;
            assert_eq!(parsed, expected, "{text}");
            assert_eq!(PadAnalog::from_name(&parsed.name()), Some(expected));
            let encoded = toml::to_string(&PadBindings {
                throttle_trigger: parsed,
                ..PadBindings::default()
            })
            .unwrap();
            let decoded: PadBindings = toml::from_str(&encoded).unwrap();
            assert_eq!(decoded.throttle_trigger, expected);
        }
        assert!(toml::from_str::<PadBindings>("throttle_trigger = \"not-a-source\"").is_err());
        assert_eq!(PadAnalog::Button(PadButton::South).half_sign(), 0);
        assert_eq!(PadAnalog::Axis(PadAxis::LeftStickX).half_sign(), 1);
        assert_eq!(PadAnalog::NegAxis(PadAxis::LeftStickX).half_sign(), -1);
        assert_eq!(PadAnalog::RawNeg(2).half_sign(), -1);
    }

    #[test]
    fn key_names_and_egui_mapping_roundtrip() {
        assert_eq!(Key::ALL.len(), egui::Key::ALL.len() + 3);
        for key in Key::ALL {
            assert_eq!(Key::from_name(key.name()), Some(*key), "{key:?}");
            match key.to_egui() {
                Some(mapped) => assert_eq!(Key::from_egui(mapped), Some(*key), "{key:?}"),
                None => assert!(matches!(key, Key::Shift | Key::Ctrl | Key::Alt)),
            }
        }
        for key in egui::Key::ALL {
            assert!(Key::from_egui(*key).is_some(), "{key:?}");
        }
        assert_eq!(Key::from_name("not-a-key"), None);
    }

    #[test]
    fn legacy_conflicts_report_duplicates_but_ignore_unknown() {
        let mut bindings = Bindings::default();
        bindings.actions.dribble = bindings.car.throttle_fwd;
        assert_eq!(
            bindings.conflicts(),
            vec!["car.throttle_fwd conflicts with actions.dribble"]
        );
        bindings.car.jump = InputBinding::Unknown;
        assert!(
            !bindings
                .conflicts()
                .iter()
                .any(|conflict| conflict.contains("car.jump"))
        );
    }

    #[test]
    fn deadzone_clamps_invalid_values_and_defaults_sparse_fields() {
        for (value, expected) in [
            (0.1, 0.1),
            (-1.0, 0.0),
            (0.9, 0.5),
            (f32::NAN, 0.1),
            (f32::INFINITY, 0.1),
        ] {
            assert_eq!(clamp_deadzone(value), expected);
        }
        assert_eq!(
            toml::from_str::<PadBindings>("").unwrap().stick_deadzone,
            0.1
        );
        assert_eq!(
            toml::from_str::<PadBindings>("stick_deadzone = 0.2")
                .unwrap()
                .stick_deadzone,
            0.2
        );
    }
}
