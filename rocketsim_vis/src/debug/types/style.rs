use super::error::{DebugError, DebugResult};

/// Typed RGBA color, components `0..=1` (linear, alpha forward).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Color {
    /// Red component in linear 0..=1 RGBA.
    pub r: f32,
    /// Green component in linear 0..=1 RGBA.
    pub g: f32,
    /// Blue component in linear 0..=1 RGBA.
    pub b: f32,
    /// Alpha component in 0..=1; 0 is clear and 1 is opaque.
    pub a: f32,
}

impl Color {
    /// Opaque white, with all RGB components 1.0.
    pub const WHITE: Self = Self {
        r: 1.0,
        g: 1.0,
        b: 1.0,
        a: 1.0,
    };
    /// Opaque black.
    pub const BLACK: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    };
    /// Black with alpha 0.0.
    pub const TRANSPARENT: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 0.0,
    };
    /// Opaque red.
    pub const RED: Self = Self {
        r: 1.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    };
    /// Opaque green.
    pub const GREEN: Self = Self {
        r: 0.0,
        g: 1.0,
        b: 0.0,
        a: 1.0,
    };
    /// Opaque blue with components (0, 0.5, 1).
    pub const BLUE: Self = Self {
        r: 0.0,
        g: 0.5,
        b: 1.0,
        a: 1.0,
    };
    /// Opaque yellow.
    pub const YELLOW: Self = Self {
        r: 1.0,
        g: 1.0,
        b: 0.0,
        a: 1.0,
    };
    /// Opaque cyan.
    pub const CYAN: Self = Self {
        r: 0.0,
        g: 1.0,
        b: 1.0,
        a: 1.0,
    };
    /// Opaque magenta.
    pub const MAGENTA: Self = Self {
        r: 1.0,
        g: 0.0,
        b: 1.0,
        a: 1.0,
    };
    /// Opaque orange with components (1, 0.5, 0).
    pub const ORANGE: Self = Self {
        r: 1.0,
        g: 0.5,
        b: 0.0,
        a: 1.0,
    };

    /// Create an RGBA color without validation.
    ///
    /// Supply finite components in `0..=1`. Submission checks them with
    /// [`Self::validate`]. An alpha of `0.0` is transparent; `1.0` is opaque.
    #[must_use]
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Create an RGB color with alpha `1.0`, without validation.
    #[must_use]
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1.0 }
    }

    /// Create an RGBA color without validation. Equivalent to [`Self::new`].
    #[must_use]
    pub const fn rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Return this color with the supplied alpha, without validation.
    #[must_use]
    pub const fn with_alpha(self, a: f32) -> Self {
        Self { a, ..self }
    }

    /// Check finite `0..=1` on all components.
    pub fn validate(&self) -> DebugResult<()> {
        if [self.r, self.g, self.b, self.a]
            .iter()
            .all(|x| x.is_finite() && (0.0..=1.0).contains(x))
        {
            Ok(())
        } else {
            Err(DebugError::InvalidColor)
        }
    }

    /// Return components in red, green, blue, alpha order.
    #[must_use]
    pub const fn to_array(self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }

    /// Create a color from red, green, blue, alpha order, without validation.
    #[must_use]
    pub const fn from_array(v: [f32; 4]) -> Self {
        Self {
            r: v[0],
            g: v[1],
            b: v[2],
            a: v[3],
        }
    }
}

impl Default for Color {
    fn default() -> Self {
        Self::WHITE
    }
}

/// Common colors and allocation-free palette helpers.
pub mod palette {
    use super::Color;

    /// Opaque white, with all RGB components 1.0.
    pub const WHITE: Color = Color::WHITE;
    /// Opaque black.
    pub const BLACK: Color = Color::BLACK;
    /// Opaque red.
    pub const RED: Color = Color::RED;
    /// Opaque green.
    pub const GREEN: Color = Color::GREEN;
    /// Opaque blue with components (0, 0.5, 1).
    pub const BLUE: Color = Color::BLUE;
    /// Opaque yellow.
    pub const YELLOW: Color = Color::YELLOW;
    /// Opaque cyan.
    pub const CYAN: Color = Color::CYAN;
    /// Opaque magenta.
    pub const MAGENTA: Color = Color::MAGENTA;
    /// Opaque orange with components (1, 0.5, 0).
    pub const ORANGE: Color = Color::ORANGE;
    /// Opaque purple with components (0.6, 0.2, 0.9).
    pub const PURPLE: Color = Color::rgb(0.6, 0.2, 0.9);
    /// Opaque lime with components (0.6, 1, 0.2).
    pub const LIME: Color = Color::rgb(0.6, 1.0, 0.2);
    /// Team blue color with components (0.15, 0.45, 1).
    pub const TEAM_BLUE: Color = Color::rgb(0.15, 0.45, 1.0);
    /// Team orange color with components (1, 0.55, 0.1).
    pub const TEAM_ORANGE: Color = Color::rgb(1.0, 0.55, 0.1);
    /// Opaque white ball color.
    pub const BALL_WHITE: Color = Color::WHITE;
    /// White at 25 percent alpha.
    pub const GHOST: Color = Color::rgba(1.0, 1.0, 1.0, 0.25);

    /// Return [`TEAM_BLUE`] if `blue` is `true`, or [`TEAM_ORANGE`] otherwise.
    #[must_use]
    pub const fn team(blue: bool) -> Color {
        if blue { TEAM_BLUE } else { TEAM_ORANGE }
    }

    /// Return `c` with the supplied alpha, without validation.
    #[must_use]
    pub const fn with_alpha(c: Color, a: f32) -> Color {
        c.with_alpha(a)
    }

    /// Return an opaque rainbow color for a finite hue in turns.
    ///
    /// The hue repeats at each integer. Saturation and value are both `1.0`.
    #[must_use]
    pub fn rainbow(t: f32) -> Color {
        let h = (t.fract() + 1.0).fract() * 6.0;
        let c = 1.0;
        let x = c * (1.0 - ((h % 2.0) - 1.0).abs());
        let (r, g, b) = match h as u32 {
            0 => (c, x, 0.0),
            1 => (x, c, 0.0),
            2 => (0.0, c, x),
            3 => (0.0, x, c),
            4 => (x, 0.0, c),
            _ => (c, 0.0, x),
        };
        Color::rgb(r, g, b)
    }
}
