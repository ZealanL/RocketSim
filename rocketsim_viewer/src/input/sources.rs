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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Axis2 {
    X,
    Y,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "type", content = "value", try_from = "InputSourceWire")]
pub enum InputSource {
    Key(Key),
    Mouse(MouseButton),
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
    Mouse(MouseButton),
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

/// Clamped deadzone actually used (guards hand-edited TOML).
pub fn clamp_deadzone(dz: f32) -> f32 {
    if dz.is_finite() {
        dz.clamp(0.0, 0.5)
    } else {
        0.1
    }
}
