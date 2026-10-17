//! Unified action bindings and device-independent input evaluation.
use crate::bindings::{Bindings, InputBinding, Key, PadAnalog, PadAxis, PadButton};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Axis2 {
    X,
    Y,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "type", content = "value", try_from = "InputSourceWire")]
pub enum InputSource {
    Key(Key),
    Mouse(crate::bindings::MouseButton),
    MouseAxis { axis: Axis2, sign: i8 },
    Wheel { axis: Axis2, sign: i8 },
    PadButton(PadButton),
    PadAxis { axis: PadAxis, sign: i8 },
    PadRaw { code: u32, sign: i8 },
    Unbound,
}
#[derive(Deserialize)]
#[serde(tag = "type", content = "value")]
enum InputSourceWire {
    Key(Key),
    Mouse(crate::bindings::MouseButton),
    MouseAxis { axis: Axis2, sign: i8 },
    Wheel { axis: Axis2, sign: i8 },
    PadButton(PadButton),
    PadAxis { axis: PadAxis, sign: i8 },
    PadRaw { code: u32, sign: i8 },
    Unbound,
}
impl TryFrom<InputSourceWire> for InputSource {
    type Error = &'static str;
    fn try_from(w: InputSourceWire) -> Result<Self, Self::Error> {
        Ok(match w {
            InputSourceWire::Key(x) => Self::Key(x),
            InputSourceWire::Mouse(x) => Self::Mouse(x),
            InputSourceWire::PadButton(x) => Self::PadButton(x),
            InputSourceWire::Unbound => Self::Unbound,
            InputSourceWire::MouseAxis { axis, sign } => Self::MouseAxis {
                axis,
                sign: valid_sign(sign)?,
            },
            InputSourceWire::Wheel { axis, sign } => Self::Wheel {
                axis,
                sign: valid_sign(sign)?,
            },
            InputSourceWire::PadAxis { axis, sign } => Self::PadAxis {
                axis,
                sign: valid_sign(sign)?,
            },
            InputSourceWire::PadRaw { code, sign } => Self::PadRaw {
                code,
                sign: valid_sign(sign)?,
            },
        })
    }
}
fn valid_sign(sign: i8) -> Result<i8, &'static str> {
    if sign == 1 || sign == -1 {
        Ok(sign)
    } else {
        Err("signed input must use sign -1 or 1")
    }
}
impl InputSource {
    pub fn name(self) -> String {
        match self {
            Self::Key(k) => k.name().into(),
            Self::Mouse(b) => format!("Mouse {b:?}"),
            Self::MouseAxis { axis, sign } => {
                format!("Mouse {axis:?} {}", if sign < 0 { "−" } else { "+" })
            }
            Self::Wheel { axis, sign } => {
                format!("Wheel {axis:?} {}", if sign < 0 { "−" } else { "+" })
            }
            Self::PadButton(b) => format!("Pad {b:?}"),
            Self::PadAxis { axis, sign } => {
                format!("Pad {axis:?} {}", if sign < 0 { "−" } else { "+" })
            }
            Self::PadRaw { code, sign } => {
                format!("Pad raw {}{code}", if sign < 0 { "−" } else { "+" })
            }
            Self::Unbound => "Unbound".into(),
        }
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[repr(usize)]
pub enum Action {
    ThrottleForward,
    ThrottleBack,
    SteerLeft,
    SteerRight,
    PitchForward,
    PitchBack,
    YawLeft,
    YawRight,
    RollLeft,
    RollRight,
    Jump,
    Boost,
    Handbrake,
    KickoffReset,
    Dribble,
    Launch,
    Follow1,
    Follow2,
    Follow3,
    Follow4,
    Follow5,
    Follow6,
    Follow7,
    Follow8,
    Director,
    AutoCam,
    FreeCam,
    WideCam,
    NextCam,
    BallCam,
    FreeForward,
    FreeBack,
    FreeLeft,
    FreeRight,
    FreeUp,
    FreeDown,
    LookLeft,
    LookRight,
    LookUp,
    LookDown,
    FreeFast,
    FreeReset,
    MenuToggle,
    MenuAccept,
    MenuBack,
    MenuUp,
    MenuDown,
    MenuLeft,
    MenuRight,
    FreeDollyForward,
    FreeDollyBack,
    AirRoll,
    SwivelLeft,
    SwivelRight,
    SwivelUp,
    SwivelDown,
}
pub const ACTIONS: [Action; 56] = [
    Action::ThrottleForward,
    Action::ThrottleBack,
    Action::SteerLeft,
    Action::SteerRight,
    Action::PitchForward,
    Action::PitchBack,
    Action::YawLeft,
    Action::YawRight,
    Action::RollLeft,
    Action::RollRight,
    Action::Jump,
    Action::Boost,
    Action::Handbrake,
    Action::KickoffReset,
    Action::Dribble,
    Action::Launch,
    Action::Follow1,
    Action::Follow2,
    Action::Follow3,
    Action::Follow4,
    Action::Follow5,
    Action::Follow6,
    Action::Follow7,
    Action::Follow8,
    Action::Director,
    Action::AutoCam,
    Action::FreeCam,
    Action::WideCam,
    Action::NextCam,
    Action::BallCam,
    Action::FreeForward,
    Action::FreeBack,
    Action::FreeLeft,
    Action::FreeRight,
    Action::FreeUp,
    Action::FreeDown,
    Action::LookLeft,
    Action::LookRight,
    Action::LookUp,
    Action::LookDown,
    Action::FreeFast,
    Action::FreeReset,
    Action::MenuToggle,
    Action::MenuAccept,
    Action::MenuBack,
    Action::MenuUp,
    Action::MenuDown,
    Action::MenuLeft,
    Action::MenuRight,
    Action::FreeDollyForward,
    Action::FreeDollyBack,
    Action::AirRoll,
    Action::SwivelLeft,
    Action::SwivelRight,
    Action::SwivelUp,
    Action::SwivelDown,
];
impl Action {
    pub fn label(self) -> &'static str {
        ACTION_NAMES[self as usize]
    }
    pub fn group(self) -> &'static str {
        match self {
            Self::ThrottleForward
            | Self::ThrottleBack
            | Self::SteerLeft
            | Self::SteerRight
            | Self::PitchForward
            | Self::PitchBack
            | Self::YawLeft
            | Self::YawRight
            | Self::RollLeft
            | Self::RollRight
            | Self::Jump
            | Self::Boost
            | Self::Handbrake
            | Self::AirRoll => "Driving",
            Self::KickoffReset | Self::Dribble | Self::Launch => "Simulation",
            Self::Follow1
            | Self::Follow2
            | Self::Follow3
            | Self::Follow4
            | Self::Follow5
            | Self::Follow6
            | Self::Follow7
            | Self::Follow8
            | Self::Director
            | Self::AutoCam
            | Self::FreeCam
            | Self::WideCam
            | Self::NextCam
            | Self::BallCam
            | Self::SwivelLeft
            | Self::SwivelRight
            | Self::SwivelUp
            | Self::SwivelDown => "Camera",
            Self::FreeForward
            | Self::FreeBack
            | Self::FreeLeft
            | Self::FreeRight
            | Self::FreeUp
            | Self::FreeDown
            | Self::LookLeft
            | Self::LookRight
            | Self::LookUp
            | Self::LookDown
            | Self::FreeFast
            | Self::FreeReset
            | Self::FreeDollyForward
            | Self::FreeDollyBack => "Free camera",
            Self::MenuToggle
            | Self::MenuAccept
            | Self::MenuBack
            | Self::MenuUp
            | Self::MenuDown
            | Self::MenuLeft
            | Self::MenuRight => "Menu",
        }
    }
}
const ACTION_NAMES: [&str; 56] = [
    "Throttle forward",
    "Throttle back",
    "Steer left",
    "Steer right",
    "Pitch forward",
    "Pitch back",
    "Yaw left",
    "Yaw right",
    "Roll left",
    "Roll right",
    "Jump",
    "Boost",
    "Handbrake",
    "Kickoff reset",
    "Dribble",
    "Launch",
    "Follow 1",
    "Follow 2",
    "Follow 3",
    "Follow 4",
    "Follow 5",
    "Follow 6",
    "Follow 7",
    "Follow 8",
    "Director camera",
    "Auto camera",
    "Free camera",
    "Wide camera",
    "Next camera",
    "Ball camera",
    "Move forward",
    "Move back",
    "Move left",
    "Move right",
    "Move up",
    "Move down",
    "Look left",
    "Look right",
    "Look up",
    "Look down",
    "Move fast",
    "Reset free camera",
    "Toggle menu",
    "Accept",
    "Back",
    "Menu up",
    "Menu down",
    "Menu left",
    "Menu right",
    "Dolly forward",
    "Dolly back",
    "Air roll",
    "Swivel left",
    "Swivel right",
    "Swivel up",
    "Swivel down",
];
/// Sparse action-keyed bindings. Missing actions deserialize as unbound.
/// [`ActionMap::default`] provides the viewer defaults; use [`ActionMap::unbound`] for a blank map.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ActionMap {
    #[serde(default)]
    bindings: BTreeMap<Action, [InputSource; 2]>,
}
impl ActionMap {
    pub fn unbound() -> Self {
        Self {
            bindings: BTreeMap::new(),
        }
    }
}
impl Default for ActionMap {
    fn default() -> Self {
        let mut m = Self::unbound();

        for (a, k) in [
            (Action::ThrottleForward, Key::W),
            (Action::ThrottleBack, Key::S),
            (Action::SteerLeft, Key::A),
            (Action::SteerRight, Key::D),
            (Action::RollLeft, Key::Q),
            (Action::RollRight, Key::E),
            (Action::Handbrake, Key::Shift),
            (Action::AirRoll, Key::Shift),
            (Action::PitchForward, Key::W),
            (Action::PitchBack, Key::S),
            (Action::YawLeft, Key::A),
            (Action::YawRight, Key::D),
            (Action::KickoffReset, Key::Backspace),
            (Action::Dribble, Key::F2),
            (Action::Launch, Key::F4),
            (Action::Director, Key::Num9),
            (Action::AutoCam, Key::Num0),
            (Action::FreeCam, Key::F),
            (Action::NextCam, Key::C),
            (Action::BallCam, Key::Space),
            (Action::MenuToggle, Key::Escape),
            (Action::MenuAccept, Key::Enter),
            (Action::MenuBack, Key::Backspace),
            (Action::MenuUp, Key::ArrowUp),
            (Action::MenuDown, Key::ArrowDown),
            (Action::MenuLeft, Key::ArrowLeft),
            (Action::MenuRight, Key::ArrowRight),
            (Action::FreeFast, Key::Ctrl),
            (Action::FreeReset, Key::R),
            (Action::FreeForward, Key::W),
            (Action::FreeBack, Key::S),
            (Action::FreeLeft, Key::A),
            (Action::FreeRight, Key::D),
            (Action::FreeUp, Key::Space),
            (Action::FreeDown, Key::Shift),
        ] {
            m.set(a, 0, InputSource::Key(k));
        }
        for (i, a) in [
            Action::Follow1,
            Action::Follow2,
            Action::Follow3,
            Action::Follow4,
            Action::Follow5,
            Action::Follow6,
            Action::Follow7,
            Action::Follow8,
        ]
        .into_iter()
        .enumerate()
        {
            m.set(
                a,
                0,
                InputSource::Key(
                    [
                        Key::Num1,
                        Key::Num2,
                        Key::Num3,
                        Key::Num4,
                        Key::Num5,
                        Key::Num6,
                        Key::Num7,
                        Key::Num8,
                    ][i],
                ),
            );
        }
        for (a, b) in [
            (Action::Jump, PadButton::South),
            (Action::Boost, PadButton::East),
            (Action::Handbrake, PadButton::West),
            (Action::RollLeft, PadButton::LeftTrigger),
            (Action::RollRight, PadButton::RightTrigger),
            (Action::BallCam, PadButton::North),
            (Action::MenuToggle, PadButton::Start),
            (Action::MenuAccept, PadButton::South),
            (Action::MenuBack, PadButton::East),
            (Action::MenuUp, PadButton::DPadUp),
            (Action::MenuDown, PadButton::DPadDown),
            (Action::MenuLeft, PadButton::DPadLeft),
            (Action::MenuRight, PadButton::DPadRight),
            (Action::AirRoll, PadButton::West),
        ] {
            m.set(a, 1, InputSource::PadButton(b));
        }
        m.set(
            Action::FreeDollyForward,
            0,
            InputSource::Wheel {
                axis: Axis2::Y,
                sign: 1,
            },
        );
        m.set(
            Action::FreeDollyBack,
            0,
            InputSource::Wheel {
                axis: Axis2::Y,
                sign: -1,
            },
        );
        m.set(
            Action::Jump,
            0,
            InputSource::Mouse(crate::bindings::MouseButton::Primary),
        );
        m.set(
            Action::Boost,
            0,
            InputSource::Mouse(crate::bindings::MouseButton::Secondary),
        );
        for (action, axis, sign) in [
            (Action::LookLeft, Axis2::X, -1),
            (Action::LookRight, Axis2::X, 1),
            (Action::LookUp, Axis2::Y, -1),
            (Action::LookDown, Axis2::Y, 1),
        ] {
            m.set(action, 0, InputSource::MouseAxis { axis, sign });
        }
        for (action, button) in [
            (Action::FreeUp, PadButton::RightTrigger2),
            (Action::FreeDown, PadButton::LeftTrigger2),
            (Action::FreeFast, PadButton::LeftThumb),
            (Action::FreeReset, PadButton::RightThumb),
        ] {
            m.set(action, 1, InputSource::PadButton(button));
        }
        m.set(
            Action::ThrottleForward,
            1,
            InputSource::PadButton(PadButton::RightTrigger2),
        );
        m.set(
            Action::ThrottleBack,
            1,
            InputSource::PadButton(PadButton::LeftTrigger2),
        );
        for (a, ax, sgn) in [
            (Action::SteerLeft, PadAxis::LeftStickX, -1),
            (Action::SteerRight, PadAxis::LeftStickX, 1),
            (Action::YawLeft, PadAxis::LeftStickX, -1),
            (Action::YawRight, PadAxis::LeftStickX, 1),
            (Action::PitchForward, PadAxis::LeftStickY, 1),
            (Action::PitchBack, PadAxis::LeftStickY, -1),
            (Action::LookLeft, PadAxis::RightStickX, -1),
            (Action::LookRight, PadAxis::RightStickX, 1),
            (Action::LookUp, PadAxis::RightStickY, 1),
            (Action::LookDown, PadAxis::RightStickY, -1),
            (Action::SwivelLeft, PadAxis::RightStickX, -1),
            (Action::SwivelRight, PadAxis::RightStickX, 1),
            (Action::SwivelUp, PadAxis::RightStickY, 1),
            (Action::SwivelDown, PadAxis::RightStickY, -1),
            (Action::FreeLeft, PadAxis::LeftStickX, -1),
            (Action::FreeRight, PadAxis::LeftStickX, 1),
            (Action::FreeForward, PadAxis::LeftStickY, 1),
            (Action::FreeBack, PadAxis::LeftStickY, -1),
        ] {
            m.set(
                a,
                1,
                InputSource::PadAxis {
                    axis: ax,
                    sign: sgn,
                },
            );
        }
        m
    }
}
impl ActionMap {
    pub fn sources(&self, a: Action) -> &[InputSource; 2] {
        self.bindings.get(&a).unwrap_or(&UNBOUND)
    }
    pub fn set(&mut self, a: Action, slot: usize, source: InputSource) {
        if slot < 2 {
            self.bindings.entry(a).or_insert([InputSource::Unbound; 2])[slot] = source;
        }
    }
    pub fn conflicts(&self) -> Vec<(Action, usize, Action, usize)> {
        let mut out = Vec::new();
        for (i, a) in ACTIONS.iter().enumerate() {
            for s in 0..2 {
                let x = self.sources(*a)[s];
                if x == InputSource::Unbound {
                    continue;
                }
                for b in ACTIONS.iter().skip(i) {
                    for t in 0..2 {
                        let j = *b as usize;
                        if (j > i || t > s) && x == self.sources(*b)[t] {
                            out.push((*a, s, *b, t));
                        }
                    }
                }
            }
        }
        out
    }
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
const UNBOUND: [InputSource; 2] = [InputSource::Unbound; 2];
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
#[derive(Debug, Clone, Default)]
pub struct InputSnapshot {
    values: HashMap<InputSource, f32>,
    pub focused: bool,
}
impl InputSnapshot {
    pub fn new() -> Self {
        Self {
            values: HashMap::new(),
            focused: true,
        }
    }
    pub fn set(&mut self, s: InputSource, v: f32) {
        self.values.insert(s, if v.is_finite() { v } else { 0.0 });
    }
    pub fn set_down(&mut self, s: InputSource, b: bool) {
        self.set(s, if b { 1.0 } else { 0.0 });
    }
    pub fn value(&self, s: InputSource) -> f32 {
        if self.focused {
            self.values.get(&s).copied().unwrap_or(0.0)
        } else {
            0.0
        }
    }
    pub fn clear(&mut self) {
        self.values.clear();
    }
}
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ActionValue {
    pub value: f32,
    pub pressed: bool,
    pub just_pressed: bool,
    pub just_released: bool,
}
#[derive(Debug)]
pub struct Evaluator {
    previous: HashMap<Action, bool>,
    previous_map: Option<ActionMap>,
    suppressed: HashSet<InputSource>,
    was_focused: bool,
}
impl Default for Evaluator {
    fn default() -> Self {
        Self {
            previous: HashMap::new(),
            previous_map: None,
            suppressed: HashSet::new(),
            was_focused: true,
        }
    }
}
impl Evaluator {
    pub fn suppress_until_release(&mut self, s: &InputSnapshot) {
        if s.focused {
            self.suppressed
                .extend(s.values.iter().filter_map(|(source, value)| {
                    if value.max(0.0) > source_release_threshold(*source) {
                        Some(*source)
                    } else {
                        None
                    }
                }));
        }
    }
    pub fn source_value(
        &self,
        snapshot: &InputSnapshot,
        source: InputSource,
        deadzone: f32,
    ) -> f32 {
        if source == InputSource::Unbound || !snapshot.focused || self.suppressed.contains(&source)
        {
            return 0.0;
        }
        let mut value = snapshot.value(source).max(0.0);
        if !matches!(
            source,
            InputSource::MouseAxis { .. } | InputSource::Wheel { .. }
        ) {
            value = value.clamp(0.0, 1.0);
        }
        if matches!(
            source,
            InputSource::PadAxis { .. } | InputSource::PadRaw { .. }
        ) {
            let dz = deadzone.clamp(0.0, 0.99);
            value = if value <= dz {
                0.0
            } else {
                ((value - dz) / (1.0 - dz)).clamp(0.0, 1.0)
            };
        }
        value
    }
    pub fn evaluate(
        &mut self,
        m: &ActionMap,
        s: &InputSnapshot,
        d: f32,
    ) -> HashMap<Action, ActionValue> {
        let regain = s.focused && !self.was_focused;
        let map_changed = self
            .previous_map
            .as_ref()
            .is_some_and(|previous| previous != m);
        if regain || map_changed {
            self.suppress_until_release(s);
        }
        self.previous_map = Some(m.clone());
        if s.focused {
            self.suppressed
                .retain(|source| s.value(*source).max(0.0) > source_release_threshold(*source));
        }
        let mut out = HashMap::new();
        for a in ACTIONS {
            let mut v: f32 = 0.0;
            for &src in m.sources(a) {
                if src == InputSource::Unbound || self.suppressed.contains(&src) {
                    continue;
                }
                let x = self.source_value(s, src, d);
                if x.abs() > v.abs() {
                    v = x;
                }
            }
            let down = v.abs() >= 0.5;
            let old = self.previous.insert(a, down).unwrap_or(false);
            let jp = down && !old && !regain;
            let jr = !down && old;
            out.insert(
                a,
                ActionValue {
                    value: v,
                    pressed: down,
                    just_pressed: jp,
                    just_released: jr,
                },
            );
        }
        self.was_focused = s.focused;
        out
    }
}
fn source_release_threshold(source: InputSource) -> f32 {
    if matches!(
        source,
        InputSource::PadAxis { .. }
            | InputSource::PadRaw { .. }
            | InputSource::MouseAxis { .. }
            | InputSource::Wheel { .. }
    ) {
        0.1
    } else {
        0.5
    }
}
fn capture_priority(source: InputSource) -> u8 {
    match source {
        InputSource::Key(
            Key::ShiftLeft
            | Key::ShiftRight
            | Key::ControlLeft
            | Key::ControlRight
            | Key::AltLeft
            | Key::AltRight,
        ) => 0,
        InputSource::Key(Key::Shift | Key::Ctrl | Key::Alt) => 5,
        InputSource::Key(_) | InputSource::Mouse(_) | InputSource::PadButton(_) => 1,
        InputSource::PadAxis { .. } => 2,
        InputSource::MouseAxis { .. } | InputSource::Wheel { .. } => 3,
        InputSource::PadRaw { .. } => 4,
        InputSource::Unbound => 6,
    }
}
fn capture_score(source: InputSource, value: f32) -> f32 {
    if matches!(
        source,
        InputSource::PadAxis { .. } | InputSource::PadRaw { .. }
    ) {
        value.clamp(0.0, 1.0)
    } else {
        value
    }
}
#[derive(Debug, Default)]
pub struct Capture {
    waiting_release: HashSet<InputSource>,
    armed: bool,
}
impl Capture {
    pub fn begin(snapshot: &InputSnapshot) -> Self {
        Self {
            waiting_release: snapshot
                .values
                .iter()
                .filter_map(|(s, v)| if *v >= 0.5 { Some(*s) } else { None })
                .collect(),
            armed: false,
        }
    }
    pub fn poll(&mut self, s: &InputSnapshot) -> Option<InputSource> {
        if !s.focused {
            return None;
        }
        self.waiting_release.retain(|x| s.value(*x) >= 0.5);
        if !self.waiting_release.is_empty() {
            return None;
        }
        self.armed = true;
        let mut candidates: Vec<_> = s
            .values
            .iter()
            .filter_map(|(source, value)| {
                let threshold = match source {
                    InputSource::PadAxis { .. } | InputSource::PadRaw { .. } => 0.6,
                    InputSource::MouseAxis { .. } => 6.0,
                    InputSource::Wheel { .. } => 1.0,
                    _ => 0.5,
                };
                if *value >= threshold {
                    Some((
                        *source,
                        capture_priority(*source),
                        capture_score(*source, *value),
                    ))
                } else {
                    None
                }
            })
            .collect();
        candidates.sort_by(
            |(source_a, priority_a, score_a), (source_b, priority_b, score_b)| {
                priority_a
                    .cmp(priority_b)
                    .then_with(|| score_b.total_cmp(score_a))
                    .then_with(|| source_a.name().cmp(&source_b.name()))
            },
        );
        candidates.first().map(|(source, _, _)| *source)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn action_count() {
        assert_eq!(ACTIONS.len(), 56);
    }
    #[test]
    fn stable_roundtrip() {
        let mut m = ActionMap::default();
        m.set(Action::Jump, 1, InputSource::Key(Key::Space));
        let t = toml::to_string(&m).unwrap();
        assert!(t.contains("Jump"));
        assert_eq!(toml::from_str::<ActionMap>(&t).unwrap(), m);
    }
    #[test]
    fn evaluator_edges_and_focus() {
        let mut m = ActionMap::default();
        let k = InputSource::Key(Key::A);
        m.set(Action::Jump, 0, k);
        let mut s = InputSnapshot::new();
        s.set_down(k, true);
        let mut e = Evaluator::default();
        assert!(e.evaluate(&m, &s, 0.0)[&Action::Jump].just_pressed);
        s.focused = false;
        assert!(!e.evaluate(&m, &s, 0.0)[&Action::Jump].pressed);
        s.focused = true;
        assert!(!e.evaluate(&m, &s, 0.0)[&Action::Jump].just_pressed);
    }
    fn all_sources() -> Vec<InputSource> {
        let mut sources: Vec<InputSource> =
            Key::ALL.iter().copied().map(InputSource::Key).collect();
        for button in [
            crate::bindings::MouseButton::Primary,
            crate::bindings::MouseButton::Secondary,
            crate::bindings::MouseButton::Middle,
            crate::bindings::MouseButton::Extra1,
            crate::bindings::MouseButton::Extra2,
        ] {
            sources.push(InputSource::Mouse(button));
        }
        for axis in [Axis2::X, Axis2::Y] {
            for sign in [-1, 1] {
                sources.push(InputSource::MouseAxis { axis, sign });
                sources.push(InputSource::Wheel { axis, sign });
            }
        }
        for button in [
            PadButton::South,
            PadButton::East,
            PadButton::North,
            PadButton::West,
            PadButton::C,
            PadButton::Z,
            PadButton::LeftTrigger,
            PadButton::LeftTrigger2,
            PadButton::RightTrigger,
            PadButton::RightTrigger2,
            PadButton::Select,
            PadButton::Start,
            PadButton::Mode,
            PadButton::LeftThumb,
            PadButton::RightThumb,
            PadButton::DPadUp,
            PadButton::DPadDown,
            PadButton::DPadLeft,
            PadButton::DPadRight,
        ] {
            sources.push(InputSource::PadButton(button));
        }
        for axis in [
            PadAxis::LeftStickX,
            PadAxis::LeftStickY,
            PadAxis::LeftZ,
            PadAxis::RightStickX,
            PadAxis::RightStickY,
            PadAxis::RightZ,
            PadAxis::DPadX,
            PadAxis::DPadY,
        ] {
            for sign in [-1, 1] {
                sources.push(InputSource::PadAxis { axis, sign });
            }
        }
        for code in [0, 1, 42, 0x30000, 0x30001, u32::MAX] {
            for sign in [-1, 1] {
                sources.push(InputSource::PadRaw { code, sign });
            }
        }
        sources
    }
    #[test]
    fn capture_accepts_every_concrete_source() {
        for source in all_sources() {
            let magnitude = match source {
                InputSource::PadAxis { .. } | InputSource::PadRaw { .. } => 0.75,
                InputSource::MouseAxis { .. } => 6.0,
                InputSource::Wheel { .. } => 1.0,
                _ => 0.75,
            };
            let mut snapshot = InputSnapshot::new();
            snapshot.set(source, magnitude);
            assert_eq!(
                Capture::begin(&InputSnapshot::new()).poll(&snapshot),
                Some(source),
                "{source:?}"
            );
        }
    }
    #[test]
    fn sparse_maps_leave_omitted_actions_unbound() {
        let map: ActionMap = toml::from_str("bindings = {}").unwrap();
        assert_eq!(map, ActionMap::unbound());
        assert_eq!(map.sources(Action::MenuToggle), &[InputSource::Unbound; 2]);
    }
    #[test]
    fn exhaustive_sources_actions_and_slots_roundtrip_and_evaluate_edges() {
        let sources = all_sources();
        for action in ACTIONS {
            for source in sources.iter().copied() {
                for slot in 0..2 {
                    let mut map = ActionMap::unbound();
                    map.set(action, slot, source);
                    assert_eq!(map.sources(action)[1 - slot], InputSource::Unbound);
                    let encoded = toml::to_string(&map).unwrap();
                    let decoded: ActionMap = toml::from_str(&encoded).unwrap();
                    assert_eq!(decoded, map, "{action:?} / {source:?} / slot {slot}");
                    let mut snapshot = InputSnapshot::new();
                    snapshot.set(source, 0.75);
                    let mut eval = Evaluator::default();
                    let values = eval.evaluate(&decoded, &snapshot, 0.1);
                    let value = values[&action];
                    let expected = if matches!(
                        source,
                        InputSource::PadAxis { .. } | InputSource::PadRaw { .. }
                    ) {
                        (0.75f32 - 0.1) / (1.0 - 0.1)
                    } else {
                        0.75
                    };
                    assert!(
                        (value.value - expected).abs() < 0.00001,
                        "{action:?} / {source:?}: {} != {expected}",
                        value.value
                    );
                    assert!(value.pressed && value.just_pressed);
                    assert!(
                        ACTIONS
                            .iter()
                            .filter(|other| **other != action)
                            .all(|other| !values[other].pressed)
                    );
                    assert!(!eval.evaluate(&decoded, &snapshot, 0.1)[&action].just_pressed);
                    snapshot.set(source, 0.0);
                    let released = eval.evaluate(&decoded, &snapshot, 0.1)[&action];
                    assert!(
                        !released.pressed && released.just_released,
                        "{action:?} / {source:?}"
                    );
                }
            }
        }
    }
    #[test]
    fn capture_waits_for_release_then_accepts_escape_and_rejects_drift() {
        let initial = InputSource::Mouse(crate::bindings::MouseButton::Primary);
        let candidate = InputSource::PadRaw { code: 42, sign: -1 };
        let escape = InputSource::Key(Key::Escape);
        let mut s = InputSnapshot::new();
        s.set_down(initial, true);
        let mut c = Capture::begin(&s);
        assert_eq!(c.poll(&s), None);
        s.set_down(initial, false);
        s.set(candidate, 0.6);
        assert_eq!(c.poll(&s), Some(candidate));
        let mut c = Capture::begin(&InputSnapshot::new());
        let mut drift = InputSnapshot::new();
        drift.set(candidate, 0.49);
        assert_eq!(c.poll(&drift), None);
        drift.set_down(escape, true);
        assert_eq!(c.poll(&drift), Some(escape));
    }
    #[test]
    fn capture_focus_loss_preserves_release_gate() {
        let held = InputSource::Mouse(crate::bindings::MouseButton::Primary);
        let next = InputSource::Key(Key::X);
        let mut snapshot = InputSnapshot::new();
        snapshot.set_down(held, true);
        let mut capture = Capture::begin(&snapshot);
        snapshot.focused = false;
        assert_eq!(capture.poll(&snapshot), None);
        snapshot.focused = true;
        assert_eq!(capture.poll(&snapshot), None);
        snapshot.set_down(held, false);
        assert_eq!(capture.poll(&snapshot), None);
        snapshot.set_down(next, true);
        assert_eq!(capture.poll(&snapshot), Some(next));
    }
    #[test]
    fn capture_prefers_physical_modifier_to_generic_alias() {
        for (generic, physicals) in [
            (Key::Shift, [Key::ShiftLeft, Key::ShiftRight]),
            (Key::Ctrl, [Key::ControlLeft, Key::ControlRight]),
            (Key::Alt, [Key::AltLeft, Key::AltRight]),
        ] {
            for physical in physicals {
                let mut snapshot = InputSnapshot::new();
                snapshot.set_down(InputSource::Key(generic), true);
                snapshot.set_down(InputSource::Key(physical), true);
                assert_eq!(
                    Capture::begin(&InputSnapshot::new()).poll(&snapshot),
                    Some(InputSource::Key(physical))
                );
            }
        }
    }
    #[test]
    fn physical_modifier_bindings_are_independent() {
        for (left, right) in [
            (Key::ShiftLeft, Key::ShiftRight),
            (Key::ControlLeft, Key::ControlRight),
            (Key::AltLeft, Key::AltRight),
        ] {
            let mut map = ActionMap::unbound();
            map.set(Action::MenuAccept, 0, InputSource::Key(left));
            let mut snapshot = InputSnapshot::new();
            snapshot.set_down(InputSource::Key(right), true);
            let mut evaluator = Evaluator::default();
            assert!(!evaluator.evaluate(&map, &snapshot, 0.0)[&Action::MenuAccept].pressed);
            snapshot.set_down(InputSource::Key(right), false);
            snapshot.set_down(InputSource::Key(left), true);
            assert!(evaluator.evaluate(&map, &snapshot, 0.0)[&Action::MenuAccept].pressed);
        }
    }
    #[test]
    fn capture_chooses_strongest_axis_then_stable_name_tie() {
        let x = InputSource::PadAxis {
            axis: PadAxis::LeftStickX,
            sign: 1,
        };
        let y = InputSource::PadAxis {
            axis: PadAxis::LeftStickY,
            sign: 1,
        };
        let mut snapshot = InputSnapshot::new();
        snapshot.set(x, 0.7);
        snapshot.set(y, 0.95);
        assert_eq!(
            Capture::begin(&InputSnapshot::new()).poll(&snapshot),
            Some(y)
        );
        snapshot.set(x, 0.8);
        snapshot.set(y, 0.8);
        assert_eq!(
            Capture::begin(&InputSnapshot::new()).poll(&snapshot),
            Some(x)
        );
    }
    #[test]
    fn focus_regain_suppresses_held_input_until_release() {
        let src = InputSource::Key(Key::A);
        let mut map = ActionMap::default();
        map.set(Action::Jump, 0, src);
        let mut s = InputSnapshot::new();
        let mut e = Evaluator::default();
        s.set_down(src, true);
        assert!(e.evaluate(&map, &s, 0.0)[&Action::Jump].pressed);
        s.focused = false;
        e.evaluate(&map, &s, 0.0);
        s.focused = true;
        assert!(!e.evaluate(&map, &s, 0.0)[&Action::Jump].pressed);
        s.set_down(src, false);
        e.evaluate(&map, &s, 0.0);
        s.set_down(src, true);
        assert!(e.evaluate(&map, &s, 0.0)[&Action::Jump].just_pressed);
    }
    #[test]
    fn signed_sources_reject_invalid_deserialized_signs() {
        let text = r#"type = "PadAxis"
value = { axis = "LeftStickX", sign = 0 }"#;
        assert!(toml::from_str::<InputSource>(text).is_err());
        assert!(valid_sign(-1).is_ok() && valid_sign(1).is_ok());
        assert!(valid_sign(0).is_err() && valid_sign(2).is_err());
    }
    #[test]
    fn deadzone_is_pad_only_and_is_applied_per_source() {
        let mut map = ActionMap::default();
        let key = InputSource::Key(Key::K);
        let axis = InputSource::PadAxis {
            axis: PadAxis::LeftStickX,
            sign: 1,
        };
        map.set(Action::YawLeft, 0, key);
        map.set(Action::YawLeft, 1, axis);
        let mut snapshot = InputSnapshot::new();
        snapshot.set(key, 0.6);
        snapshot.set(axis, 0.2);
        snapshot.set(axis, -0.75);
        assert_eq!(Evaluator::default().source_value(&snapshot, axis, 0.1), 0.0);
        snapshot.set(axis, 0.2);
        assert_eq!(
            Evaluator::default().evaluate(&map, &snapshot, 0.3)[&Action::YawLeft].value,
            0.6
        );
        let mouse = InputSource::MouseAxis {
            axis: Axis2::X,
            sign: 1,
        };
        map.set(Action::LookRight, 0, mouse);
        snapshot.set(mouse, 12.0);
        assert_eq!(
            Evaluator::default().evaluate(&map, &snapshot, 0.5)[&Action::LookRight].value,
            12.0
        );
        let mut evaluator = Evaluator::default();
        snapshot.focused = false;
        assert_eq!(evaluator.source_value(&snapshot, mouse, 0.0), 0.0);
        snapshot.focused = true;
        evaluator.suppress_until_release(&snapshot);
        assert_eq!(evaluator.source_value(&snapshot, mouse, 0.0), 0.0);
    }
    #[test]
    fn legacy_pad_camera_and_action_fields_migrate() {
        let mut legacy = Bindings::default();
        legacy.pad.camera.follow = [
            PadButton::South,
            PadButton::East,
            PadButton::North,
            PadButton::West,
            PadButton::C,
            PadButton::Z,
            PadButton::Start,
            PadButton::Mode,
        ];
        legacy.pad.camera.director = PadButton::LeftThumb;
        legacy.pad.camera.autocam = PadButton::RightThumb;
        legacy.pad.camera.free = PadButton::DPadUp;
        legacy.pad.camera.cycle = PadButton::DPadDown;
        legacy.pad.actions.kickoff_reset = PadButton::Select;
        legacy.pad.actions.dribble = PadButton::DPadLeft;
        legacy.pad.actions.launch = PadButton::DPadRight;
        let map = ActionMap::from_legacy(&legacy);
        for i in 0..8 {
            assert_eq!(
                map.sources(ACTIONS[16 + i])[1],
                InputSource::PadButton(legacy.pad.camera.follow[i])
            );
        }
        assert_eq!(
            map.sources(Action::PitchForward)[0],
            InputSource::Key(legacy.car.throttle_fwd)
        );
        assert_eq!(
            map.sources(Action::YawLeft)[0],
            InputSource::Key(legacy.car.steer_left)
        );
        assert_eq!(
            map.sources(Action::AirRoll)[0],
            InputSource::Key(legacy.car.handbrake)
        );
        assert_eq!(
            map.sources(Action::AirRoll)[1],
            InputSource::PadButton(legacy.pad.handbrake)
        );
        assert_eq!(
            map.sources(Action::YawRight)[1],
            InputSource::PadAxis {
                axis: legacy.pad.steer_axis,
                sign: 1
            }
        );
        for (action, button) in [
            (Action::Director, PadButton::LeftThumb),
            (Action::AutoCam, PadButton::RightThumb),
            (Action::FreeCam, PadButton::DPadUp),
            (Action::NextCam, PadButton::DPadDown),
            (Action::KickoffReset, PadButton::Select),
            (Action::Dribble, PadButton::DPadLeft),
            (Action::Launch, PadButton::DPadRight),
        ] {
            assert_eq!(map.sources(action)[1], InputSource::PadButton(button));
        }
    }
    #[test]
    fn defaults_cover_keyboard_mouse_and_controller_freecam() {
        let map = ActionMap::default();
        assert_eq!(ActionMap::from_legacy(&Bindings::default()), map);
        assert_eq!(
            map.sources(Action::Jump)[0],
            InputSource::Mouse(crate::bindings::MouseButton::Primary)
        );
        assert_eq!(
            map.sources(Action::Boost)[0],
            InputSource::Mouse(crate::bindings::MouseButton::Secondary)
        );
        assert_eq!(
            map.sources(Action::FreeUp)[1],
            InputSource::PadButton(PadButton::RightTrigger2)
        );
        assert_eq!(
            map.sources(Action::FreeDown)[1],
            InputSource::PadButton(PadButton::LeftTrigger2)
        );
        assert_eq!(
            map.sources(Action::MenuUp)[1],
            InputSource::PadButton(PadButton::DPadUp)
        );
        assert_eq!(
            map.sources(Action::MenuRight)[1],
            InputSource::PadButton(PadButton::DPadRight)
        );
        assert_eq!(
            map.sources(Action::PitchForward)[0],
            InputSource::Key(Key::W)
        );
        assert_eq!(map.sources(Action::YawLeft)[0], InputSource::Key(Key::A));
        assert_eq!(map.sources(Action::AirRoll), map.sources(Action::Handbrake));
        for (action, axis, sign) in [
            (Action::SwivelLeft, PadAxis::RightStickX, -1),
            (Action::SwivelRight, PadAxis::RightStickX, 1),
            (Action::SwivelUp, PadAxis::RightStickY, 1),
            (Action::SwivelDown, PadAxis::RightStickY, -1),
        ] {
            assert_eq!(map.sources(action)[0], InputSource::Unbound);
            assert_eq!(map.sources(action)[1], InputSource::PadAxis { axis, sign });
        }
        assert_eq!(Action::SwivelLeft.group(), "Camera");
        assert_eq!(
            map.sources(Action::FreeDollyForward)[0],
            InputSource::Wheel {
                axis: Axis2::Y,
                sign: 1
            }
        );
        assert_eq!(
            map.sources(Action::FreeDollyBack)[0],
            InputSource::Wheel {
                axis: Axis2::Y,
                sign: -1
            }
        );
        assert_eq!(Action::FreeDollyForward.group(), "Free camera");
        assert_eq!(Action::AirRoll.group(), "Driving");
        assert_eq!(Action::MenuToggle.group(), "Menu");
        assert_eq!(Action::FreeReset.group(), "Free camera");
    }
    #[test]
    fn map_changes_do_not_create_a_press_edge_for_held_source() {
        let src = InputSource::Key(Key::A);
        let mut map = ActionMap::default();
        map.set(Action::Jump, 0, src);
        let mut s = InputSnapshot::new();
        s.set_down(src, true);
        let mut e = Evaluator::default();
        e.evaluate(&map, &s, 0.0);
        map.set(Action::Jump, 0, InputSource::Unbound);
        map.set(Action::Boost, 0, src);
        let value = e.evaluate(&map, &s, 0.0)[&Action::Boost];
        assert!(!value.just_pressed && !value.pressed);
        s.set_down(src, false);
        e.evaluate(&map, &s, 0.0);
        s.set_down(src, true);
        assert!(e.evaluate(&map, &s, 0.0)[&Action::Boost].just_pressed);
    }
}
