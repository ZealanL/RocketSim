#[derive(Clone, Copy, Debug)]
struct LinearPiece {
    pub base_x: f32,
    pub base_y: f32,
    pub max_x: f32,
    pub max_y: f32,
    pub x_diff: f32,
    pub y_diff: f32,
}

/// Continuous piecewise-linear `(x, y)` mapping (e.g. steer vs speed).
///
/// Lookup clamps: inputs at/below the first `x` return the first `y`, inputs
/// above the last `x` return the last `y`. Used throughout `consts::curves`.
#[derive(Clone, Copy, Debug)]
pub struct LinearPieceCurve<const N: usize> {
    curve: [LinearPiece; N],
}

impl<const N: usize> LinearPieceCurve<N> {
    /// Builds a curve from ascending `(x, y)` control points.
    ///
    /// `x` values should ascend; behavior is linear interpolation between
    /// neighbors (no validation in `const` context).
    pub const fn new(value_mappings: [(f32, f32); N]) -> Self {
        let mut curve = [LinearPiece {
            base_x: 0.0,
            base_y: 0.0,
            max_x: 0.0,
            max_y: 0.0,
            x_diff: 0.0,
            y_diff: 0.0,
        }; N];

        curve[0].max_x = value_mappings[0].0;
        curve[0].max_y = value_mappings[0].1;

        let mut i = 1;
        while i < N {
            let prev = &value_mappings[i - 1];
            let this = &value_mappings[i];

            curve[i].base_x = prev.0;
            curve[i].base_y = prev.1;
            curve[i].max_x = this.0;
            curve[i].max_y = this.1;
            curve[i].x_diff = this.0 - prev.0;
            curve[i].y_diff = this.1 - prev.1;

            i += 1;
        }

        Self { curve }
    }

    /// Samples the curve at `input` (clamped at both ends, see type docs).
    ///
    /// # Arguments
    ///
    /// * `input` - The input to the curve
    pub fn get_output(&self, input: f32) -> f32 {
        debug_assert!(N != 0);

        let first_val_pair = self.curve[0];
        if input <= first_val_pair.max_x {
            return first_val_pair.max_y;
        }

        let Some(pair) = self.curve.iter().skip(1).find(|pair| pair.max_x > input) else {
            return self.curve[N - 1].max_y;
        };

        let interp_frac = (input - pair.base_x) / pair.x_diff;
        pair.y_diff * interp_frac + pair.base_y
    }
}
