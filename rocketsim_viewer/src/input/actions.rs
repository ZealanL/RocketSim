//! Stable action identifiers, defaults, and action-map storage.

use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    sync::atomic::{AtomicU64, Ordering},
};

use super::sources::{Axis2, InputSource, Key, MouseButton, PadAxis, PadButton};

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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionMap {
    #[serde(default)]
    bindings: BTreeMap<Action, [InputSource; 2]>,
    // Evaluator reads this revision instead of cloning the map each frame.
    #[serde(skip, default = "next_revision")]
    revision: u64,
}

static NEXT_REVISION: AtomicU64 = AtomicU64::new(1);

fn next_revision() -> u64 {
    NEXT_REVISION
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |revision| {
            revision.checked_add(1)
        })
        .expect("action map revision counter exhausted")
}

impl PartialEq for ActionMap {
    fn eq(&self, other: &Self) -> bool {
        self.bindings == other.bindings
    }
}

const UNBOUND: [InputSource; 2] = [InputSource::Unbound; 2];

impl ActionMap {
    pub fn unbound() -> Self {
        Self {
            bindings: BTreeMap::new(),
            revision: next_revision(),
        }
    }

    pub(super) fn revision(&self) -> u64 {
        self.revision
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
        m.set(Action::Jump, 0, InputSource::Mouse(MouseButton::Primary));
        m.set(Action::Boost, 0, InputSource::Mouse(MouseButton::Secondary));
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
        if slot < 2 && self.sources(a)[slot] != source {
            self.bindings.entry(a).or_insert([InputSource::Unbound; 2])[slot] = source;
            self.revision = next_revision();
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
}
