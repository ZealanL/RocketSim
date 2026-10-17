//! Configurable input bindings for the viewer and the drive demo.
//!
//! Each table maps a game action to a key, a mouse button, or a
//! gamepad input. All tables use plain TOML strings, so users can
//! edit `settings.toml` by hand. Every field has a default, so old
//! files without a `[bindings]` section still parse.
//!
//! Defaults copy today's hardcoded mapping exactly:
//! viewer driving (`WASD` + mouse, `Q`/`E` roll, `Shift` handbrake),
//! action keys (`Backspace` reset, `F2` dribble, `F4` launch),
//! camera keys (`1`-`8` follow, `9` director, `0` auto, `F` free,
//! `C` cycle, `Space` ball cam), `Escape` menu, and the drive demo
//! gamepad mapping (sticks, triggers, face buttons).
//!
//! `Key` is a small local enum, not `egui::Key`: the stock `egui`
//! build has no `serde` support, and `Shift` is a modifier state in
//! `egui`, not a key. Convert at the call site with
//! [`Key::to_egui`] / [`Key::from_egui`], or read state directly
//! with [`key_down`] / [`key_pressed`]. Pad enums mirror `gilrs`
//! 0.11 names (`Button::*`, `Axis::*`) without depending on `gilrs`,
//! so this module builds in the library; the drive example maps
//! them onto real `gilrs` values.

use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// Keyboard input, stored as a short TOML string (`"W"`, `"Space"`).
///
/// Covers today's defaults plus common rebind targets (arrows,
/// function keys, modifiers). Modifier variants (`Shift`, `Ctrl`,
/// `Alt`) read the `egui` modifier state, since `egui` has no key
/// event for them; see [`key_down`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Key {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
    O,
    P,
    Q,
    R,
    S,
    T,
    U,
    V,
    W,
    X,
    Y,
    Z,
    Num0,
    Num1,
    Num2,
    Num3,
    Num4,
    Num5,
    Num6,
    Num7,
    Num8,
    Num9,
    F1,
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F9,
    F10,
    F11,
    F12,
    Space,
    Tab,
    Enter,
    Escape,
    Backspace,
    ArrowUp,
    ArrowDown,
    ArrowLeft,
    ArrowRight,
    Insert,
    Delete,
    Home,
    End,
    PageUp,
    PageDown,
    Copy,
    Cut,
    Paste,
    Colon,
    Comma,
    Backslash,
    Slash,
    Pipe,
    Questionmark,
    Exclamationmark,
    OpenBracket,
    CloseBracket,
    OpenCurlyBracket,
    CloseCurlyBracket,
    Backtick,
    Minus,
    Period,
    Plus,
    Equals,
    Semicolon,
    Quote,
    F13,
    F14,
    F15,
    F16,
    F17,
    F18,
    F19,
    F20,
    F21,
    F22,
    F23,
    F24,
    F25,
    F26,
    F27,
    F28,
    F29,
    F30,
    F31,
    F32,
    F33,
    F34,
    F35,
    BrowserBack,
    BrowserForward,
    BrowserRefresh,
    BrowserSearch,
    BrowserHome,
    BrowserFavorites,
    BrowserStop,
    MediaPlayPause,
    MediaTrackNext,
    MediaTrackPrevious,
    MediaStop,
    AudioVolumeMute,
    AudioVolumeDown,
    AudioVolumeUp,
    LaunchMail,
    LaunchApp1,
    LaunchApp2,
    ShiftLeft,
    ShiftRight,
    ControlLeft,
    ControlRight,
    AltLeft,
    AltRight,
    SuperLeft,
    SuperRight,
    IntlBackslash,
    CapsLock,
    NumLock,
    ScrollLock,
    PrintScreen,
    Pause,
    Menu,
    Fn,
    Eject,
    Help,
    Power,
    Sleep,
    Clear,
    /// Either Shift key (modifier state, no press edge).
    Shift,
    /// Control key (modifier state, no press edge).
    Ctrl,
    /// Alt key (modifier state, no press edge).
    Alt,
}

impl Key {
    /// Every bindable key (used by rebind capture loops).
    pub const ALL: &'static [Key] = &[
        Key::A,
        Key::B,
        Key::C,
        Key::D,
        Key::E,
        Key::F,
        Key::G,
        Key::H,
        Key::I,
        Key::J,
        Key::K,
        Key::L,
        Key::M,
        Key::N,
        Key::O,
        Key::P,
        Key::Q,
        Key::R,
        Key::S,
        Key::T,
        Key::U,
        Key::V,
        Key::W,
        Key::X,
        Key::Y,
        Key::Z,
        Key::Num0,
        Key::Num1,
        Key::Num2,
        Key::Num3,
        Key::Num4,
        Key::Num5,
        Key::Num6,
        Key::Num7,
        Key::Num8,
        Key::Num9,
        Key::F1,
        Key::F2,
        Key::F3,
        Key::F4,
        Key::F5,
        Key::F6,
        Key::F7,
        Key::F8,
        Key::F9,
        Key::F10,
        Key::F11,
        Key::F12,
        Key::Space,
        Key::Tab,
        Key::Enter,
        Key::Escape,
        Key::Backspace,
        Key::ArrowUp,
        Key::ArrowDown,
        Key::ArrowLeft,
        Key::ArrowRight,
        Key::Insert,
        Key::Delete,
        Key::Home,
        Key::End,
        Key::PageUp,
        Key::PageDown,
        Key::Copy,
        Key::Cut,
        Key::Paste,
        Key::Colon,
        Key::Comma,
        Key::Backslash,
        Key::Slash,
        Key::Pipe,
        Key::Questionmark,
        Key::Exclamationmark,
        Key::OpenBracket,
        Key::CloseBracket,
        Key::OpenCurlyBracket,
        Key::CloseCurlyBracket,
        Key::Backtick,
        Key::Minus,
        Key::Period,
        Key::Plus,
        Key::Equals,
        Key::Semicolon,
        Key::Quote,
        Key::F13,
        Key::F14,
        Key::F15,
        Key::F16,
        Key::F17,
        Key::F18,
        Key::F19,
        Key::F20,
        Key::F21,
        Key::F22,
        Key::F23,
        Key::F24,
        Key::F25,
        Key::F26,
        Key::F27,
        Key::F28,
        Key::F29,
        Key::F30,
        Key::F31,
        Key::F32,
        Key::F33,
        Key::F34,
        Key::F35,
        Key::BrowserBack,
        Key::BrowserForward,
        Key::BrowserRefresh,
        Key::BrowserSearch,
        Key::BrowserHome,
        Key::BrowserFavorites,
        Key::BrowserStop,
        Key::MediaPlayPause,
        Key::MediaTrackNext,
        Key::MediaTrackPrevious,
        Key::MediaStop,
        Key::AudioVolumeMute,
        Key::AudioVolumeDown,
        Key::AudioVolumeUp,
        Key::LaunchMail,
        Key::LaunchApp1,
        Key::LaunchApp2,
        Key::ShiftLeft,
        Key::ShiftRight,
        Key::ControlLeft,
        Key::ControlRight,
        Key::AltLeft,
        Key::AltRight,
        Key::SuperLeft,
        Key::SuperRight,
        Key::IntlBackslash,
        Key::CapsLock,
        Key::NumLock,
        Key::ScrollLock,
        Key::PrintScreen,
        Key::Pause,
        Key::Menu,
        Key::Fn,
        Key::Eject,
        Key::Help,
        Key::Power,
        Key::Sleep,
        Key::Clear,
        Key::Shift,
        Key::Ctrl,
        Key::Alt,
    ];

    /// Short display name; matches the TOML string.
    pub fn name(self) -> &'static str {
        match self {
            Key::A => "A",
            Key::B => "B",
            Key::C => "C",
            Key::D => "D",
            Key::E => "E",
            Key::F => "F",
            Key::G => "G",
            Key::H => "H",
            Key::I => "I",
            Key::J => "J",
            Key::K => "K",
            Key::L => "L",
            Key::M => "M",
            Key::N => "N",
            Key::O => "O",
            Key::P => "P",
            Key::Q => "Q",
            Key::R => "R",
            Key::S => "S",
            Key::T => "T",
            Key::U => "U",
            Key::V => "V",
            Key::W => "W",
            Key::X => "X",
            Key::Y => "Y",
            Key::Z => "Z",
            Key::Num0 => "Num0",
            Key::Num1 => "Num1",
            Key::Num2 => "Num2",
            Key::Num3 => "Num3",
            Key::Num4 => "Num4",
            Key::Num5 => "Num5",
            Key::Num6 => "Num6",
            Key::Num7 => "Num7",
            Key::Num8 => "Num8",
            Key::Num9 => "Num9",
            Key::F1 => "F1",
            Key::F2 => "F2",
            Key::F3 => "F3",
            Key::F4 => "F4",
            Key::F5 => "F5",
            Key::F6 => "F6",
            Key::F7 => "F7",
            Key::F8 => "F8",
            Key::F9 => "F9",
            Key::F10 => "F10",
            Key::F11 => "F11",
            Key::F12 => "F12",
            Key::Space => "Space",
            Key::Tab => "Tab",
            Key::Enter => "Enter",
            Key::Escape => "Escape",
            Key::Backspace => "Backspace",
            Key::ArrowUp => "ArrowUp",
            Key::ArrowDown => "ArrowDown",
            Key::ArrowLeft => "ArrowLeft",
            Key::ArrowRight => "ArrowRight",
            Key::Shift => "Shift",
            Key::Ctrl => "Ctrl",
            Key::Insert => "Insert",
            Key::Delete => "Delete",
            Key::Home => "Home",
            Key::End => "End",
            Key::PageUp => "PageUp",
            Key::PageDown => "PageDown",
            Key::Copy => "Copy",
            Key::Cut => "Cut",
            Key::Paste => "Paste",
            Key::Colon => "Colon",
            Key::Comma => "Comma",
            Key::Backslash => "Backslash",
            Key::Slash => "Slash",
            Key::Pipe => "Pipe",
            Key::Questionmark => "Questionmark",
            Key::Exclamationmark => "Exclamationmark",
            Key::OpenBracket => "OpenBracket",
            Key::CloseBracket => "CloseBracket",
            Key::OpenCurlyBracket => "OpenCurlyBracket",
            Key::CloseCurlyBracket => "CloseCurlyBracket",
            Key::Backtick => "Backtick",
            Key::Minus => "Minus",
            Key::Period => "Period",
            Key::Plus => "Plus",
            Key::Equals => "Equals",
            Key::Semicolon => "Semicolon",
            Key::Quote => "Quote",
            Key::F13 => "F13",
            Key::F14 => "F14",
            Key::F15 => "F15",
            Key::F16 => "F16",
            Key::F17 => "F17",
            Key::F18 => "F18",
            Key::F19 => "F19",
            Key::F20 => "F20",
            Key::F21 => "F21",
            Key::F22 => "F22",
            Key::F23 => "F23",
            Key::F24 => "F24",
            Key::F25 => "F25",
            Key::F26 => "F26",
            Key::F27 => "F27",
            Key::F28 => "F28",
            Key::F29 => "F29",
            Key::F30 => "F30",
            Key::F31 => "F31",
            Key::F32 => "F32",
            Key::F33 => "F33",
            Key::F34 => "F34",
            Key::F35 => "F35",
            Key::BrowserBack => "BrowserBack",
            Key::BrowserForward => "BrowserForward",
            Key::BrowserRefresh => "BrowserRefresh",
            Key::BrowserSearch => "BrowserSearch",
            Key::BrowserHome => "BrowserHome",
            Key::BrowserFavorites => "BrowserFavorites",
            Key::BrowserStop => "BrowserStop",
            Key::MediaPlayPause => "MediaPlayPause",
            Key::MediaTrackNext => "MediaTrackNext",
            Key::MediaTrackPrevious => "MediaTrackPrevious",
            Key::MediaStop => "MediaStop",
            Key::AudioVolumeMute => "AudioVolumeMute",
            Key::AudioVolumeDown => "AudioVolumeDown",
            Key::AudioVolumeUp => "AudioVolumeUp",
            Key::LaunchMail => "LaunchMail",
            Key::LaunchApp1 => "LaunchApp1",
            Key::LaunchApp2 => "LaunchApp2",
            Key::ShiftLeft => "ShiftLeft",
            Key::ShiftRight => "ShiftRight",
            Key::ControlLeft => "ControlLeft",
            Key::ControlRight => "ControlRight",
            Key::AltLeft => "AltLeft",
            Key::AltRight => "AltRight",
            Key::SuperLeft => "SuperLeft",
            Key::SuperRight => "SuperRight",
            Key::IntlBackslash => "IntlBackslash",
            Key::CapsLock => "CapsLock",
            Key::NumLock => "NumLock",
            Key::ScrollLock => "ScrollLock",
            Key::PrintScreen => "PrintScreen",
            Key::Pause => "Pause",
            Key::Menu => "Menu",
            Key::Fn => "Fn",
            Key::Eject => "Eject",
            Key::Help => "Help",
            Key::Power => "Power",
            Key::Sleep => "Sleep",
            Key::Clear => "Clear",
            Key::Alt => "Alt",
        }
    }

    /// Parses a [`name`](Self::name) string; `None` when unknown.
    pub fn from_name(name: &str) -> Option<Key> {
        Some(match name {
            "A" => Key::A,
            "B" => Key::B,
            "C" => Key::C,
            "D" => Key::D,
            "E" => Key::E,
            "F" => Key::F,
            "G" => Key::G,
            "H" => Key::H,
            "I" => Key::I,
            "J" => Key::J,
            "K" => Key::K,
            "L" => Key::L,
            "M" => Key::M,
            "N" => Key::N,
            "O" => Key::O,
            "P" => Key::P,
            "Q" => Key::Q,
            "R" => Key::R,
            "S" => Key::S,
            "T" => Key::T,
            "U" => Key::U,
            "V" => Key::V,
            "W" => Key::W,
            "X" => Key::X,
            "Y" => Key::Y,
            "Z" => Key::Z,
            "Num0" => Key::Num0,
            "Num1" => Key::Num1,
            "Num2" => Key::Num2,
            "Num3" => Key::Num3,
            "Num4" => Key::Num4,
            "Num5" => Key::Num5,
            "Num6" => Key::Num6,
            "Num7" => Key::Num7,
            "Num8" => Key::Num8,
            "Num9" => Key::Num9,
            "F1" => Key::F1,
            "F2" => Key::F2,
            "F3" => Key::F3,
            "F4" => Key::F4,
            "F5" => Key::F5,
            "F6" => Key::F6,
            "F7" => Key::F7,
            "F8" => Key::F8,
            "F9" => Key::F9,
            "F10" => Key::F10,
            "F11" => Key::F11,
            "F12" => Key::F12,
            "Space" => Key::Space,
            "Tab" => Key::Tab,
            "Enter" => Key::Enter,
            "Escape" => Key::Escape,
            "Backspace" => Key::Backspace,
            "ArrowUp" => Key::ArrowUp,
            "ArrowDown" => Key::ArrowDown,
            "ArrowLeft" => Key::ArrowLeft,
            "ArrowRight" => Key::ArrowRight,
            "Shift" => Key::Shift,
            "Ctrl" => Key::Ctrl,
            "Insert" => Key::Insert,
            "Delete" => Key::Delete,
            "Home" => Key::Home,
            "End" => Key::End,
            "PageUp" => Key::PageUp,
            "PageDown" => Key::PageDown,
            "Copy" => Key::Copy,
            "Cut" => Key::Cut,
            "Paste" => Key::Paste,
            "Colon" => Key::Colon,
            "Comma" => Key::Comma,
            "Backslash" => Key::Backslash,
            "Slash" => Key::Slash,
            "Pipe" => Key::Pipe,
            "Questionmark" => Key::Questionmark,
            "Exclamationmark" => Key::Exclamationmark,
            "OpenBracket" => Key::OpenBracket,
            "CloseBracket" => Key::CloseBracket,
            "OpenCurlyBracket" => Key::OpenCurlyBracket,
            "CloseCurlyBracket" => Key::CloseCurlyBracket,
            "Backtick" => Key::Backtick,
            "Minus" => Key::Minus,
            "Period" => Key::Period,
            "Plus" => Key::Plus,
            "Equals" => Key::Equals,
            "Semicolon" => Key::Semicolon,
            "Quote" => Key::Quote,
            "F13" => Key::F13,
            "F14" => Key::F14,
            "F15" => Key::F15,
            "F16" => Key::F16,
            "F17" => Key::F17,
            "F18" => Key::F18,
            "F19" => Key::F19,
            "F20" => Key::F20,
            "F21" => Key::F21,
            "F22" => Key::F22,
            "F23" => Key::F23,
            "F24" => Key::F24,
            "F25" => Key::F25,
            "F26" => Key::F26,
            "F27" => Key::F27,
            "F28" => Key::F28,
            "F29" => Key::F29,
            "F30" => Key::F30,
            "F31" => Key::F31,
            "F32" => Key::F32,
            "F33" => Key::F33,
            "F34" => Key::F34,
            "F35" => Key::F35,
            "BrowserBack" => Key::BrowserBack,
            "BrowserForward" => Key::BrowserForward,
            "BrowserRefresh" => Key::BrowserRefresh,
            "BrowserSearch" => Key::BrowserSearch,
            "BrowserHome" => Key::BrowserHome,
            "BrowserFavorites" => Key::BrowserFavorites,
            "BrowserStop" => Key::BrowserStop,
            "MediaPlayPause" => Key::MediaPlayPause,
            "MediaTrackNext" => Key::MediaTrackNext,
            "MediaTrackPrevious" => Key::MediaTrackPrevious,
            "MediaStop" => Key::MediaStop,
            "AudioVolumeMute" => Key::AudioVolumeMute,
            "AudioVolumeDown" => Key::AudioVolumeDown,
            "AudioVolumeUp" => Key::AudioVolumeUp,
            "LaunchMail" => Key::LaunchMail,
            "LaunchApp1" => Key::LaunchApp1,
            "LaunchApp2" => Key::LaunchApp2,
            "ShiftLeft" => Key::ShiftLeft,
            "ShiftRight" => Key::ShiftRight,
            "ControlLeft" => Key::ControlLeft,
            "ControlRight" => Key::ControlRight,
            "AltLeft" => Key::AltLeft,
            "AltRight" => Key::AltRight,
            "SuperLeft" => Key::SuperLeft,
            "SuperRight" => Key::SuperRight,
            "IntlBackslash" => Key::IntlBackslash,
            "CapsLock" => Key::CapsLock,
            "NumLock" => Key::NumLock,
            "ScrollLock" => Key::ScrollLock,
            "PrintScreen" => Key::PrintScreen,
            "Pause" => Key::Pause,
            "Menu" => Key::Menu,
            "Fn" => Key::Fn,
            "Eject" => Key::Eject,
            "Help" => Key::Help,
            "Power" => Key::Power,
            "Sleep" => Key::Sleep,
            "Clear" => Key::Clear,
            "Alt" => Key::Alt,
            _ => return None,
        })
    }

    /// Maps onto `egui`; `None` for modifiers (no `egui` key exists).
    pub fn to_egui(self) -> Option<egui::Key> {
        Some(match self {
            Key::A => egui::Key::A,
            Key::B => egui::Key::B,
            Key::C => egui::Key::C,
            Key::D => egui::Key::D,
            Key::E => egui::Key::E,
            Key::F => egui::Key::F,
            Key::G => egui::Key::G,
            Key::H => egui::Key::H,
            Key::I => egui::Key::I,
            Key::J => egui::Key::J,
            Key::K => egui::Key::K,
            Key::L => egui::Key::L,
            Key::M => egui::Key::M,
            Key::N => egui::Key::N,
            Key::O => egui::Key::O,
            Key::P => egui::Key::P,
            Key::Q => egui::Key::Q,
            Key::R => egui::Key::R,
            Key::S => egui::Key::S,
            Key::T => egui::Key::T,
            Key::U => egui::Key::U,
            Key::V => egui::Key::V,
            Key::W => egui::Key::W,
            Key::X => egui::Key::X,
            Key::Y => egui::Key::Y,
            Key::Z => egui::Key::Z,
            Key::Num0 => egui::Key::Num0,
            Key::Num1 => egui::Key::Num1,
            Key::Num2 => egui::Key::Num2,
            Key::Num3 => egui::Key::Num3,
            Key::Num4 => egui::Key::Num4,
            Key::Num5 => egui::Key::Num5,
            Key::Num6 => egui::Key::Num6,
            Key::Num7 => egui::Key::Num7,
            Key::Num8 => egui::Key::Num8,
            Key::Num9 => egui::Key::Num9,
            Key::F1 => egui::Key::F1,
            Key::F2 => egui::Key::F2,
            Key::F3 => egui::Key::F3,
            Key::F4 => egui::Key::F4,
            Key::F5 => egui::Key::F5,
            Key::F6 => egui::Key::F6,
            Key::F7 => egui::Key::F7,
            Key::F8 => egui::Key::F8,
            Key::F9 => egui::Key::F9,
            Key::F10 => egui::Key::F10,
            Key::F11 => egui::Key::F11,
            Key::F12 => egui::Key::F12,
            Key::Space => egui::Key::Space,
            Key::Tab => egui::Key::Tab,
            Key::Enter => egui::Key::Enter,
            Key::Escape => egui::Key::Escape,
            Key::Backspace => egui::Key::Backspace,
            Key::ArrowUp => egui::Key::ArrowUp,
            Key::ArrowDown => egui::Key::ArrowDown,
            Key::ArrowLeft => egui::Key::ArrowLeft,
            Key::ArrowRight => egui::Key::ArrowRight,
            Key::Insert => egui::Key::Insert,
            Key::Delete => egui::Key::Delete,
            Key::Home => egui::Key::Home,
            Key::End => egui::Key::End,
            Key::PageUp => egui::Key::PageUp,
            Key::PageDown => egui::Key::PageDown,
            Key::Copy => egui::Key::Copy,
            Key::Cut => egui::Key::Cut,
            Key::Paste => egui::Key::Paste,
            Key::Colon => egui::Key::Colon,
            Key::Comma => egui::Key::Comma,
            Key::Backslash => egui::Key::Backslash,
            Key::Slash => egui::Key::Slash,
            Key::Pipe => egui::Key::Pipe,
            Key::Questionmark => egui::Key::Questionmark,
            Key::Exclamationmark => egui::Key::Exclamationmark,
            Key::OpenBracket => egui::Key::OpenBracket,
            Key::CloseBracket => egui::Key::CloseBracket,
            Key::OpenCurlyBracket => egui::Key::OpenCurlyBracket,
            Key::CloseCurlyBracket => egui::Key::CloseCurlyBracket,
            Key::Backtick => egui::Key::Backtick,
            Key::Minus => egui::Key::Minus,
            Key::Period => egui::Key::Period,
            Key::Plus => egui::Key::Plus,
            Key::Equals => egui::Key::Equals,
            Key::Semicolon => egui::Key::Semicolon,
            Key::Quote => egui::Key::Quote,
            Key::F13 => egui::Key::F13,
            Key::F14 => egui::Key::F14,
            Key::F15 => egui::Key::F15,
            Key::F16 => egui::Key::F16,
            Key::F17 => egui::Key::F17,
            Key::F18 => egui::Key::F18,
            Key::F19 => egui::Key::F19,
            Key::F20 => egui::Key::F20,
            Key::F21 => egui::Key::F21,
            Key::F22 => egui::Key::F22,
            Key::F23 => egui::Key::F23,
            Key::F24 => egui::Key::F24,
            Key::F25 => egui::Key::F25,
            Key::F26 => egui::Key::F26,
            Key::F27 => egui::Key::F27,
            Key::F28 => egui::Key::F28,
            Key::F29 => egui::Key::F29,
            Key::F30 => egui::Key::F30,
            Key::F31 => egui::Key::F31,
            Key::F32 => egui::Key::F32,
            Key::F33 => egui::Key::F33,
            Key::F34 => egui::Key::F34,
            Key::F35 => egui::Key::F35,
            Key::BrowserBack => egui::Key::BrowserBack,
            Key::BrowserForward => egui::Key::BrowserForward,
            Key::BrowserRefresh => egui::Key::BrowserRefresh,
            Key::BrowserSearch => egui::Key::BrowserSearch,
            Key::BrowserHome => egui::Key::BrowserHome,
            Key::BrowserFavorites => egui::Key::BrowserFavorites,
            Key::BrowserStop => egui::Key::BrowserStop,
            Key::MediaPlayPause => egui::Key::MediaPlayPause,
            Key::MediaTrackNext => egui::Key::MediaTrackNext,
            Key::MediaTrackPrevious => egui::Key::MediaTrackPrevious,
            Key::MediaStop => egui::Key::MediaStop,
            Key::AudioVolumeMute => egui::Key::AudioVolumeMute,
            Key::AudioVolumeDown => egui::Key::AudioVolumeDown,
            Key::AudioVolumeUp => egui::Key::AudioVolumeUp,
            Key::LaunchMail => egui::Key::LaunchMail,
            Key::LaunchApp1 => egui::Key::LaunchApp1,
            Key::LaunchApp2 => egui::Key::LaunchApp2,
            Key::ShiftLeft => egui::Key::ShiftLeft,
            Key::ShiftRight => egui::Key::ShiftRight,
            Key::ControlLeft => egui::Key::ControlLeft,
            Key::ControlRight => egui::Key::ControlRight,
            Key::AltLeft => egui::Key::AltLeft,
            Key::AltRight => egui::Key::AltRight,
            Key::SuperLeft => egui::Key::SuperLeft,
            Key::SuperRight => egui::Key::SuperRight,
            Key::IntlBackslash => egui::Key::IntlBackslash,
            Key::CapsLock => egui::Key::CapsLock,
            Key::NumLock => egui::Key::NumLock,
            Key::ScrollLock => egui::Key::ScrollLock,
            Key::PrintScreen => egui::Key::PrintScreen,
            Key::Pause => egui::Key::Pause,
            Key::Menu => egui::Key::Menu,
            Key::Fn => egui::Key::Fn,
            Key::Eject => egui::Key::Eject,
            Key::Help => egui::Key::Help,
            Key::Power => egui::Key::Power,
            Key::Sleep => egui::Key::Sleep,
            Key::Clear => egui::Key::Clear,
            Key::Shift | Key::Ctrl | Key::Alt => return None,
        })
    }

    /// Maps from `egui`; `None` for keys outside the bindable set.
    pub fn from_egui(key: egui::Key) -> Option<Key> {
        Some(match key {
            egui::Key::A => Key::A,
            egui::Key::B => Key::B,
            egui::Key::C => Key::C,
            egui::Key::D => Key::D,
            egui::Key::E => Key::E,
            egui::Key::F => Key::F,
            egui::Key::G => Key::G,
            egui::Key::H => Key::H,
            egui::Key::I => Key::I,
            egui::Key::J => Key::J,
            egui::Key::K => Key::K,
            egui::Key::L => Key::L,
            egui::Key::M => Key::M,
            egui::Key::N => Key::N,
            egui::Key::O => Key::O,
            egui::Key::P => Key::P,
            egui::Key::Q => Key::Q,
            egui::Key::R => Key::R,
            egui::Key::S => Key::S,
            egui::Key::T => Key::T,
            egui::Key::U => Key::U,
            egui::Key::V => Key::V,
            egui::Key::W => Key::W,
            egui::Key::X => Key::X,
            egui::Key::Y => Key::Y,
            egui::Key::Z => Key::Z,
            egui::Key::Num0 => Key::Num0,
            egui::Key::Num1 => Key::Num1,
            egui::Key::Num2 => Key::Num2,
            egui::Key::Num3 => Key::Num3,
            egui::Key::Num4 => Key::Num4,
            egui::Key::Num5 => Key::Num5,
            egui::Key::Num6 => Key::Num6,
            egui::Key::Num7 => Key::Num7,
            egui::Key::Num8 => Key::Num8,
            egui::Key::Num9 => Key::Num9,
            egui::Key::F1 => Key::F1,
            egui::Key::F2 => Key::F2,
            egui::Key::F3 => Key::F3,
            egui::Key::F4 => Key::F4,
            egui::Key::F5 => Key::F5,
            egui::Key::F6 => Key::F6,
            egui::Key::F7 => Key::F7,
            egui::Key::F8 => Key::F8,
            egui::Key::F9 => Key::F9,
            egui::Key::F10 => Key::F10,
            egui::Key::F11 => Key::F11,
            egui::Key::F12 => Key::F12,
            egui::Key::Space => Key::Space,
            egui::Key::Tab => Key::Tab,
            egui::Key::Enter => Key::Enter,
            egui::Key::Escape => Key::Escape,
            egui::Key::Backspace => Key::Backspace,
            egui::Key::ArrowUp => Key::ArrowUp,
            egui::Key::ArrowDown => Key::ArrowDown,
            egui::Key::ArrowLeft => Key::ArrowLeft,
            egui::Key::ArrowRight => Key::ArrowRight,
            egui::Key::Insert => Key::Insert,
            egui::Key::Delete => Key::Delete,
            egui::Key::Home => Key::Home,
            egui::Key::End => Key::End,
            egui::Key::PageUp => Key::PageUp,
            egui::Key::PageDown => Key::PageDown,
            egui::Key::Copy => Key::Copy,
            egui::Key::Cut => Key::Cut,
            egui::Key::Paste => Key::Paste,
            egui::Key::Colon => Key::Colon,
            egui::Key::Comma => Key::Comma,
            egui::Key::Backslash => Key::Backslash,
            egui::Key::Slash => Key::Slash,
            egui::Key::Pipe => Key::Pipe,
            egui::Key::Questionmark => Key::Questionmark,
            egui::Key::Exclamationmark => Key::Exclamationmark,
            egui::Key::OpenBracket => Key::OpenBracket,
            egui::Key::CloseBracket => Key::CloseBracket,
            egui::Key::OpenCurlyBracket => Key::OpenCurlyBracket,
            egui::Key::CloseCurlyBracket => Key::CloseCurlyBracket,
            egui::Key::Backtick => Key::Backtick,
            egui::Key::Minus => Key::Minus,
            egui::Key::Period => Key::Period,
            egui::Key::Plus => Key::Plus,
            egui::Key::Equals => Key::Equals,
            egui::Key::Semicolon => Key::Semicolon,
            egui::Key::Quote => Key::Quote,
            egui::Key::F13 => Key::F13,
            egui::Key::F14 => Key::F14,
            egui::Key::F15 => Key::F15,
            egui::Key::F16 => Key::F16,
            egui::Key::F17 => Key::F17,
            egui::Key::F18 => Key::F18,
            egui::Key::F19 => Key::F19,
            egui::Key::F20 => Key::F20,
            egui::Key::F21 => Key::F21,
            egui::Key::F22 => Key::F22,
            egui::Key::F23 => Key::F23,
            egui::Key::F24 => Key::F24,
            egui::Key::F25 => Key::F25,
            egui::Key::F26 => Key::F26,
            egui::Key::F27 => Key::F27,
            egui::Key::F28 => Key::F28,
            egui::Key::F29 => Key::F29,
            egui::Key::F30 => Key::F30,
            egui::Key::F31 => Key::F31,
            egui::Key::F32 => Key::F32,
            egui::Key::F33 => Key::F33,
            egui::Key::F34 => Key::F34,
            egui::Key::F35 => Key::F35,
            egui::Key::BrowserBack => Key::BrowserBack,
            egui::Key::BrowserForward => Key::BrowserForward,
            egui::Key::BrowserRefresh => Key::BrowserRefresh,
            egui::Key::BrowserSearch => Key::BrowserSearch,
            egui::Key::BrowserHome => Key::BrowserHome,
            egui::Key::BrowserFavorites => Key::BrowserFavorites,
            egui::Key::BrowserStop => Key::BrowserStop,
            egui::Key::MediaPlayPause => Key::MediaPlayPause,
            egui::Key::MediaTrackNext => Key::MediaTrackNext,
            egui::Key::MediaTrackPrevious => Key::MediaTrackPrevious,
            egui::Key::MediaStop => Key::MediaStop,
            egui::Key::AudioVolumeMute => Key::AudioVolumeMute,
            egui::Key::AudioVolumeDown => Key::AudioVolumeDown,
            egui::Key::AudioVolumeUp => Key::AudioVolumeUp,
            egui::Key::LaunchMail => Key::LaunchMail,
            egui::Key::LaunchApp1 => Key::LaunchApp1,
            egui::Key::LaunchApp2 => Key::LaunchApp2,
            egui::Key::ShiftLeft => Key::ShiftLeft,
            egui::Key::ShiftRight => Key::ShiftRight,
            egui::Key::ControlLeft => Key::ControlLeft,
            egui::Key::ControlRight => Key::ControlRight,
            egui::Key::AltLeft => Key::AltLeft,
            egui::Key::AltRight => Key::AltRight,
            egui::Key::SuperLeft => Key::SuperLeft,
            egui::Key::SuperRight => Key::SuperRight,
            egui::Key::IntlBackslash => Key::IntlBackslash,
            egui::Key::CapsLock => Key::CapsLock,
            egui::Key::NumLock => Key::NumLock,
            egui::Key::ScrollLock => Key::ScrollLock,
            egui::Key::PrintScreen => Key::PrintScreen,
            egui::Key::Pause => Key::Pause,
            egui::Key::Menu => Key::Menu,
            egui::Key::Fn => Key::Fn,
            egui::Key::Eject => Key::Eject,
            egui::Key::Help => Key::Help,
            egui::Key::Power => Key::Power,
            egui::Key::Sleep => Key::Sleep,
            egui::Key::Clear => Key::Clear,
        })
    }
}

/// Level state for a binding: modifiers read the modifier flags.
pub fn key_down(input: &egui::InputState, key: Key) -> bool {
    if let Some(mapped) = key.to_egui() {
        return input.key_down(mapped);
    }
    // No `egui::Key` for modifiers: use the modifier state instead.
    match key {
        Key::Shift => input.modifiers.shift,
        Key::Ctrl => input.modifiers.ctrl,
        Key::Alt => input.modifiers.alt,
        _ => false,
    }
}

/// Press edge for a binding. Modifiers have no press edge in `egui`,
/// so they always read `false` here; use normal keys for buttons.
pub fn key_pressed(input: &egui::InputState, key: Key) -> bool {
    key.to_egui()
        .is_some_and(|mapped| input.key_pressed(mapped))
}

/// Mouse button, stored as a TOML string (`"Primary"`, ...).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MouseButton {
    Primary,
    Secondary,
    Middle,
    Extra1,
    Extra2,
}

/// Level state for a mouse-button binding.
pub fn mouse_down(input: &egui::InputState, button: MouseButton) -> bool {
    match button {
        MouseButton::Primary => input.pointer.primary_down(),
        MouseButton::Secondary => input.pointer.secondary_down(),
        MouseButton::Middle => input.pointer.middle_down(),
        MouseButton::Extra1 => input.pointer.button_down(egui::PointerButton::Extra1),
        MouseButton::Extra2 => input.pointer.button_down(egui::PointerButton::Extra2),
    }
}

/// Keyboard or mouse input stored as a compatible TOML string.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InputBinding {
    Key(Key),
    Mouse(MouseButton),
    Unknown,
}

impl Serialize for InputBinding {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(&self.name())
    }
}
impl<'de> Deserialize<'de> for InputBinding {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let name = String::deserialize(deserializer)?;
        if let Some(key) = Key::from_name(&name) {
            return Ok(Self::Key(key));
        }
        match name.as_str() {
            "Primary" => Ok(Self::Mouse(MouseButton::Primary)),
            "Secondary" => Ok(Self::Mouse(MouseButton::Secondary)),
            "Middle" => Ok(Self::Mouse(MouseButton::Middle)),
            "Extra1" => Ok(Self::Mouse(MouseButton::Extra1)),
            "Extra2" => Ok(Self::Mouse(MouseButton::Extra2)),
            "Unknown" => Ok(Self::Unknown),
            _ => Err(serde::de::Error::custom(format!(
                "unknown input binding: {name}"
            ))),
        }
    }
}
impl InputBinding {
    pub fn name(self) -> String {
        match self {
            Self::Key(k) => k.name().to_owned(),
            Self::Mouse(m) => format!("{m:?}"),
            Self::Unknown => "Unknown".to_owned(),
        }
    }
}
pub fn input_down(input: &egui::InputState, binding: InputBinding) -> bool {
    match binding {
        InputBinding::Key(k) => key_down(input, k),
        InputBinding::Mouse(m) => mouse_down(input, m),
        InputBinding::Unknown => false,
    }
}

/// Gamepad button, stored as a TOML string.
///
/// Names match `gilrs` 0.11 `Button` exactly, but this enum has no
/// `gilrs` dependency; the drive example maps each variant over.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum PadButton {
    South,
    East,
    North,
    West,
    C,
    Z,
    LeftTrigger,
    LeftTrigger2,
    RightTrigger,
    RightTrigger2,
    Select,
    Start,
    Mode,
    LeftThumb,
    RightThumb,
    DPadUp,
    DPadDown,
    DPadLeft,
    DPadRight,
    #[default]
    Unknown,
}

impl PadButton {
    /// Parse a TOML/UI name (`South`, ...). Matches [`Debug`](std::fmt::Debug).
    pub fn from_name(name: &str) -> Option<PadButton> {
        Some(match name {
            "South" => PadButton::South,
            "East" => PadButton::East,
            "North" => PadButton::North,
            "West" => PadButton::West,
            "C" => PadButton::C,
            "Z" => PadButton::Z,
            "LeftTrigger" => PadButton::LeftTrigger,
            "LeftTrigger2" => PadButton::LeftTrigger2,
            "RightTrigger" => PadButton::RightTrigger,
            "RightTrigger2" => PadButton::RightTrigger2,
            "Select" => PadButton::Select,
            "Start" => PadButton::Start,
            "Mode" => PadButton::Mode,
            "LeftThumb" => PadButton::LeftThumb,
            "RightThumb" => PadButton::RightThumb,
            "DPadUp" => PadButton::DPadUp,
            "DPadDown" => PadButton::DPadDown,
            "DPadLeft" => PadButton::DPadLeft,
            "DPadRight" => PadButton::DPadRight,
            "Unknown" => PadButton::Unknown,
            _ => return None,
        })
    }
}

/// Gamepad axis, stored as a TOML string.
///
/// Names match `gilrs` 0.11 `Axis` exactly, without the dependency.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PadAxis {
    LeftStickX,
    LeftStickY,
    LeftZ,
    RightStickX,
    RightStickY,
    RightZ,
    DPadX,
    DPadY,
    Unknown,
}

impl PadAxis {
    /// Parse a TOML/UI name (`LeftStickY`, ...). Matches [`Debug`](std::fmt::Debug).
    pub fn from_name(name: &str) -> Option<PadAxis> {
        Some(match name {
            "LeftStickX" => PadAxis::LeftStickX,
            "LeftStickY" => PadAxis::LeftStickY,
            "LeftZ" => PadAxis::LeftZ,
            "RightStickX" => PadAxis::RightStickX,
            "RightStickY" => PadAxis::RightStickY,
            "RightZ" => PadAxis::RightZ,
            "DPadX" => PadAxis::DPadX,
            "DPadY" => PadAxis::DPadY,
            "Unknown" => PadAxis::Unknown,
            _ => return None,
        })
    }
}

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

/// One analog drive input: either a trigger-style button (reads its
/// 0..1 analog value) or a stick axis half. Axis halves are explicit so
/// any stick direction binds either way round: `Axis` is the positive
/// half, `NegAxis` the negative half (both read as 0..1 magnitudes).
/// Raw ev codes cover pads whose axes gilrs leaves unmapped (their
/// pulls otherwise show up as `Unknown`): `Raw`/`RawNeg` are the
/// positive/negative halves of one, captured live in the menu.
///
/// Stored as one plain string: `"RightTrigger2"`, `"LeftStickY"`,
/// `"-LeftStickY"`, `"+2"`, `"-2"`. Older files with bare trigger or
/// stick names still parse (only `"Unknown"` exists in both name
/// tables, and it binds as a button).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PadAnalog {
    Button(PadButton),
    Axis(PadAxis),
    NegAxis(PadAxis),
    Raw(u32),
    RawNeg(u32),
}

impl PadAnalog {
    /// Short TOML/UI name (`RightTrigger2`, `-LeftStickY`, `+2`).
    pub fn name(self) -> String {
        match self {
            PadAnalog::Button(b) => format!("{b:?}"),
            PadAnalog::Axis(a) => format!("{a:?}"),
            PadAnalog::NegAxis(a) => format!("-{a:?}"),
            PadAnalog::Raw(c) => format!("+{c}"),
            PadAnalog::RawNeg(c) => format!("-{c}"),
        }
    }

    /// Parse [`name`](Self::name) output (and every older shape).
    pub fn from_name(text: &str) -> Option<PadAnalog> {
        if let Some(rest) = text.strip_prefix('-') {
            // Negative half: axis name or raw ev code.
            if let Ok(code) = rest.parse::<u32>() {
                return Some(PadAnalog::RawNeg(code));
            }
            return PadAxis::from_name(rest).map(PadAnalog::NegAxis);
        }
        if let Some(rest) = text.strip_prefix('+') {
            // Raw ev code, positive half.
            return rest.parse::<u32>().ok().map(PadAnalog::Raw);
        }
        // Legacy shapes: button names first (so `"Unknown"` binds as a
        // button, as before), then axis names.
        if let Some(b) = PadButton::from_name(text) {
            return Some(PadAnalog::Button(b));
        }
        PadAxis::from_name(text).map(PadAnalog::Axis)
    }

    /// Which half an axis-flavored source reads: +1 positive, −1
    /// negative, 0 button (no half). Each half already reads as a 0..1
    /// magnitude, so throttle and brake use it directly.
    pub fn half_sign(self) -> i8 {
        match self {
            PadAnalog::Button(_) => 0,
            PadAnalog::Axis(_) | PadAnalog::Raw(_) => 1,
            PadAnalog::NegAxis(_) | PadAnalog::RawNeg(_) => -1,
        }
    }
}

impl serde::Serialize for PadAnalog {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&self.name())
    }
}

impl<'de> serde::Deserialize<'de> for PadAnalog {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let text = String::deserialize(d)?;
        PadAnalog::from_name(&text)
            .ok_or_else(|| serde::de::Error::custom(format!("bad analog binding: {text:?}")))
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

/// Clamped deadzone actually used (guards hand-edited TOML).
pub fn clamp_deadzone(dz: f32) -> f32 {
    if dz.is_finite() {
        dz.clamp(0.0, 0.5)
    } else {
        0.1
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_today() {
        // Exact copy of the hardcoded mapping (viewer + drive demo).
        let b = Bindings::default();
        assert_eq!(b.car.throttle_fwd, Key::W);
        assert_eq!(b.car.throttle_back, Key::S);
        assert_eq!(b.car.steer_left, Key::A);
        assert_eq!(b.car.steer_right, Key::D);
        assert_eq!(b.car.roll_left, Key::Q);
        assert_eq!(b.car.roll_right, Key::E);
        assert_eq!(b.car.handbrake, Key::Shift);
        assert_eq!(b.car.jump, InputBinding::Mouse(MouseButton::Primary));
        assert_eq!(b.car.boost, InputBinding::Mouse(MouseButton::Secondary));
        assert_eq!(b.actions.kickoff_reset, Key::Backspace);
        assert_eq!(b.actions.dribble, Key::F2);
        assert_eq!(b.actions.launch, Key::F4);
        assert_eq!(
            b.camera.follow,
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
        );
        assert_eq!(b.camera.director, Key::Num9);
        assert_eq!(b.camera.autocam, Key::Num0);
        assert_eq!(b.camera.free, Key::F);
        assert_eq!(b.camera.cycle, Key::C);
        assert_eq!(b.camera.ball_cam, Key::Space);
        assert_eq!(b.ui.menu_toggle, Key::Escape);
        assert_eq!(b.pad.jump, PadButton::South);
        assert_eq!(b.pad.boost, PadButton::East);
        assert_eq!(b.pad.handbrake, PadButton::West);
        assert_eq!(b.pad.roll_left, PadButton::LeftTrigger);
        assert_eq!(b.pad.roll_right, PadButton::RightTrigger);
        assert_eq!(
            b.pad.throttle_trigger,
            PadAnalog::Button(PadButton::RightTrigger2)
        );
        assert_eq!(
            b.pad.brake_trigger,
            PadAnalog::Button(PadButton::LeftTrigger2)
        );
        assert_eq!(b.pad.steer_axis, PadAxis::LeftStickX);
        assert_eq!(b.pad.pitch_axis, PadAxis::LeftStickY);
        assert_eq!(b.pad.look_x, PadAxis::RightStickX);
        assert_eq!(b.pad.look_y, PadAxis::RightStickY);
        assert_eq!(b.pad.ball_cam, PadButton::North);
        assert_eq!(b.pad.stick_deadzone, 0.1);
        // Action/camera pad slots start unbound.
        assert_eq!(b.pad.actions.kickoff_reset, PadButton::Unknown);
        assert_eq!(b.pad.camera.follow, [PadButton::Unknown; 8]);
        assert_eq!(b.pad.camera.director, PadButton::Unknown);
        assert_eq!(b.ui.menu_toggle_pad, PadButton::Start);
        assert_eq!(b.ui.ui_activate, PadButton::South);
        assert_eq!(b.ui.ui_back, PadButton::East);
    }

    #[test]
    fn pad_analog_old_strings_still_parse_as_buttons() {
        // Pre-analog files stored bare trigger names: untagged parsing
        // reads them back as the Button variant (same behavior).
        let back: Bindings = toml::from_str(
            "[pad]\nthrottle_trigger = \"RightTrigger2\"\nbrake_trigger = \"LeftTrigger2\"",
        )
        .unwrap();
        assert_eq!(
            back.pad.throttle_trigger,
            PadAnalog::Button(PadButton::RightTrigger2)
        );
        assert_eq!(
            back.pad.brake_trigger,
            PadAnalog::Button(PadButton::LeftTrigger2)
        );

        // Axis strings parse as the Axis variant and round-trip.
        let back: Bindings = toml::from_str("[pad]\nthrottle_trigger = \"LeftStickY\"").unwrap();
        assert_eq!(
            back.pad.throttle_trigger,
            PadAnalog::Axis(PadAxis::LeftStickY)
        );
        let text = toml::to_string(&back.pad).unwrap();
        assert!(text.contains("throttle_trigger = \"LeftStickY\""));
        let back2: PadBindings = toml::from_str(&text).unwrap();
        assert_eq!(back.pad, back2);
    }

    #[test]
    fn pad_analog_signed_and_raw_shapes() {
        // `-Name` binds the negative half; `+N`/`-N` bind raw ev codes.
        let neg: Bindings = toml::from_str("[pad]\nthrottle_trigger = \"-LeftStickY\"").unwrap();
        assert_eq!(
            neg.pad.throttle_trigger,
            PadAnalog::NegAxis(PadAxis::LeftStickY)
        );
        let raw: Bindings = toml::from_str("[pad]\nbrake_trigger = \"+2\"").unwrap();
        assert_eq!(raw.pad.brake_trigger, PadAnalog::Raw(2));
        let raw_neg: Bindings = toml::from_str("[pad]\nbrake_trigger = \"-5\"").unwrap();
        assert_eq!(raw_neg.pad.brake_trigger, PadAnalog::RawNeg(5));
        // Names round-trip exactly (what the menu shows is what parses).
        for src in [
            PadAnalog::Button(PadButton::South),
            PadAnalog::Axis(PadAxis::LeftStickY),
            PadAnalog::NegAxis(PadAxis::LeftStickY),
            PadAnalog::Raw(2),
            PadAnalog::RawNeg(5),
        ] {
            assert_eq!(PadAnalog::from_name(src.name().as_str()), Some(src));
            // Through a real table, like settings.toml.
            let pad = PadBindings {
                throttle_trigger: src,
                ..PadBindings::default()
            };
            let text = toml::to_string(&pad).unwrap();
            assert!(text.contains(&format!("throttle_trigger = {:?}", src.name())));
            let back: PadBindings = toml::from_str(&text).unwrap();
            assert_eq!(back.throttle_trigger, src);
        }
        // Half signs: buttons have none, axes read their half.
        assert_eq!(PadAnalog::Button(PadButton::South).half_sign(), 0);
        assert_eq!(PadAnalog::Axis(PadAxis::LeftStickX).half_sign(), 1);
        assert_eq!(PadAnalog::NegAxis(PadAxis::LeftStickX).half_sign(), -1);
        assert_eq!(PadAnalog::Raw(2).half_sign(), 1);
        assert_eq!(PadAnalog::RawNeg(2).half_sign(), -1);
        // Garbage is rejected (falls back at the settings level).
        assert!(toml::from_str::<PadAnalog>("\"Nope\"").is_err());
    }

    #[test]
    fn deadzone_clamps() {
        assert_eq!(clamp_deadzone(0.1), 0.1);
        assert_eq!(clamp_deadzone(-1.0), 0.0);
        assert_eq!(clamp_deadzone(0.9), 0.5);
        assert_eq!(clamp_deadzone(f32::NAN), 0.1);
        assert_eq!(clamp_deadzone(f32::INFINITY), 0.1);
        // Sparse files keep the default.
        let back: PadBindings = toml::from_str("").unwrap();
        assert_eq!(back.stick_deadzone, 0.1);
        let back: PadBindings = toml::from_str("stick_deadzone = 0.2").unwrap();
        assert_eq!(back.stick_deadzone, 0.2);
    }

    #[test]
    fn defaults_round_trip_through_toml() {
        let b = Bindings::default();
        let text = toml::to_string(&b).unwrap();
        // Keys persist as short readable strings.
        assert!(text.contains("throttle_fwd = \"W\""));
        assert!(text.contains("handbrake = \"Shift\""));
        assert!(text.contains("jump = \"Primary\""));
        assert!(text.contains("kickoff_reset = \"Backspace\""));
        assert!(text.contains("menu_toggle = \"Escape\""));
        let back: Bindings = toml::from_str(&text).unwrap();
        assert_eq!(b, back);
    }

    #[test]
    fn sparse_and_unknown_input_falls_back_to_defaults() {
        // Old file with no `[bindings]` section: all defaults.
        let back: Bindings = toml::from_str("").unwrap();
        assert_eq!(back, Bindings::default());

        // Partial table with unknown keys: known keys apply, the
        // rest keeps defaults, unknown keys are ignored.
        let text = r#"
            unknown_future_key = true
            [car]
            throttle_fwd = "ArrowUp"
            unknown_key = 1
            [camera]
            unknown_key = 1
        "#;
        let back: Bindings = toml::from_str(text).unwrap();
        assert_eq!(back.car.throttle_fwd, Key::ArrowUp);
        assert_eq!(back.car.throttle_back, Key::S);
        assert_eq!(back.car.jump, InputBinding::Mouse(MouseButton::Primary));
        assert_eq!(back.camera.ball_cam, Key::Space);
        assert_eq!(back.ui.menu_toggle, Key::Escape);
        assert_eq!(back.pad.jump, PadButton::South);
    }

    #[test]
    fn default_keys_have_no_duplicates() {
        // Two actions on one key would fire together, so every
        // default key must be unique across all tables.
        let b = Bindings::default();
        let keys = [
            b.car.throttle_fwd,
            b.car.throttle_back,
            b.car.steer_left,
            b.car.steer_right,
            b.car.roll_left,
            b.car.roll_right,
            b.car.handbrake,
            b.actions.kickoff_reset,
            b.actions.dribble,
            b.actions.launch,
            b.camera.director,
            b.camera.autocam,
            b.camera.free,
            b.camera.cycle,
            b.camera.ball_cam,
            b.ui.menu_toggle,
        ];
        let mut all: Vec<Key> = keys.to_vec();
        all.extend(b.camera.follow);
        for (i, a) in all.iter().enumerate() {
            for other in &all[i + 1..] {
                assert_ne!(a, other, "duplicate default key binding: {a:?}");
            }
        }
        // Mouse pair and pad buttons must differ too.
        assert_ne!(b.car.jump, b.car.boost);
        let pad = [
            b.pad.jump,
            b.pad.boost,
            b.pad.handbrake,
            b.pad.roll_left,
            b.pad.roll_right,
        ];
        for (i, a) in pad.iter().enumerate() {
            for other in &pad[i + 1..] {
                assert_ne!(a, other, "duplicate default pad binding: {a:?}");
            }
        }
        // Analog sources default to distinct triggers.
        assert_ne!(b.pad.throttle_trigger, b.pad.brake_trigger);
        assert_ne!(b.pad.steer_axis, b.pad.pitch_axis);
    }

    #[test]
    fn old_mouse_strings_and_keyboard_car_bindings_parse() {
        let old: Bindings =
            toml::from_str("[car]\njump = \"Primary\"\nboost = \"Secondary\"").unwrap();
        assert_eq!(old.car.jump, InputBinding::Mouse(MouseButton::Primary));
        assert_eq!(old.car.boost, InputBinding::Mouse(MouseButton::Secondary));
        let keyboard: Bindings = toml::from_str("[car]\njump = \"Space\"").unwrap();
        assert_eq!(keyboard.car.jump, InputBinding::Key(Key::Space));
        assert_eq!(keyboard.car.jump.name(), "Space");
    }

    #[test]
    fn conflicts_report_cross_context_duplicates_but_ignore_unknown() {
        let mut bindings = Bindings::default();
        bindings.actions.dribble = bindings.car.throttle_fwd;
        let conflicts = bindings.conflicts();
        assert_eq!(
            conflicts,
            vec!["car.throttle_fwd conflicts with actions.dribble"]
        );
        bindings.car.jump = InputBinding::Unknown;
        assert!(!bindings.conflicts().iter().any(|c| c.contains("car.jump")));
    }

    #[test]
    fn names_cover_all_keys() {
        // Display names match TOML strings and parse back.
        assert_eq!(Key::ALL.len(), egui::Key::ALL.len() + 3);
        for key in Key::ALL {
            let back = Key::from_name(key.name());
            assert_eq!(back, Some(*key), "name round trip for {key:?}");
        }
        assert_eq!(Key::from_name("Nope"), None);
    }

    #[test]
    fn egui_conversion_round_trips() {
        // Every egui key survives the round trip. Legacy collapsed modifier aliases remain available.
        for key in Key::ALL {
            match key.to_egui() {
                Some(mapped) => assert_eq!(
                    Key::from_egui(mapped),
                    Some(*key),
                    "egui round trip for {key:?}"
                ),
                None => assert!(
                    matches!(key, Key::Shift | Key::Ctrl | Key::Alt),
                    "only modifiers lack an egui key: {key:?}"
                ),
            }
        }
        assert_eq!(Key::from_egui(egui::Key::W), Some(Key::W));
        assert_eq!(Key::from_egui(egui::Key::Space), Some(Key::Space));
        assert_eq!(Key::from_egui(egui::Key::Escape), Some(Key::Escape));
        for key in egui::Key::ALL {
            assert!(Key::from_egui(*key).is_some(), "missing egui key {key:?}");
        }
        assert_eq!(Key::from_egui(egui::Key::Colon), Some(Key::Colon));
        assert_eq!(Key::from_egui(egui::Key::F20), Some(Key::F20));
    }
}
