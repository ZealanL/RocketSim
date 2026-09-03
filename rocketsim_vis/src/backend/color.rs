#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Color {
    pub const ALPHA: Color = Color::new(0.0, 0.0, 0.0, 0.0);
    pub const BLACK: Color = Color::new_rgb(0.0, 0.0, 0.0);
    pub const WHITE: Color = Color::new_rgb(1.0, 1.0, 1.0);

    pub const RED: Color = Color::new_rgb(1.0, 0.0, 0.0);
    pub const ORANGE: Color = Color::new_rgb(1.0, 0.4, 0.0);
    pub const YELLOW: Color = Color::new_rgb(1.0, 1.0, 0.0);
    pub const GREEN: Color = Color::new_rgb(0.0, 1.0, 0.0);
    pub const AQUA: Color = Color::new_rgb(0.0, 1.0, 1.0);
    pub const BLUE: Color = Color::new_rgb(0.0, 0.3, 1.0);
    pub const PURPLE: Color = Color::new_rgb(0.5, 0.0, 1.0);
    pub const MAGENTA: Color = Color::new_rgb(1.0, 0.0, 1.0);

    /////////////////////

    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    pub const fn new_rgb(r: f32, g: f32, b: f32) -> Self {
        Self::new(r, g, b, 1.0)
    }

    pub const fn to_array(self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }

    pub const fn is_valid(self) -> bool {
        let vals = self.to_array();
        let mut i = 0;
        while i < vals.len() {
            let val = vals[i];
            if val < 0.0 || val > 1.0 {
                return false;
            }

            i += 1;
        }
        true
    }

    pub const fn with_alpha(&self, alpha: f32) -> Self {
        Self::new(self.r, self.g, self.b, alpha)
    }

    pub const fn lerp(self, other: Self, t: f32) -> Self {
        let it = 1.0 - t;
        Self::new(
            self.r * it + other.r * t,
            self.g * it + other.g * t,
            self.b * it + other.b * t,
            self.a * it + other.a * t,
        )
    }

    pub(in crate::backend) fn to_egui(self) -> egui::Color32 {
        egui::Color32::from_rgba_unmultiplied(
            (self.r.clamp(0.0, 1.0) * 255.0).round() as u8,
            (self.g.clamp(0.0, 1.0) * 255.0).round() as u8,
            (self.b.clamp(0.0, 1.0) * 255.0).round() as u8,
            (self.a.clamp(0.0, 1.0) * 255.0).round() as u8,
        )
    }
}

impl std::ops::Mul<f32> for Color {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        self.with_alpha(self.a * rhs)
    }
}
