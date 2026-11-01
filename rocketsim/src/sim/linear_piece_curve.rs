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

impl<const N: usize> PartialEq for LinearPieceCurve<N> {
    fn eq(&self, other: &Self) -> bool {
        let mut i = 0;
        while i < N {
            if self.curve[i].max_x != other.curve[i].max_x
                || self.curve[i].max_y != other.curve[i].max_y
            {
                return false;
            }
            i += 1;
        }
        true
    }
}

impl<const N: usize> LinearPieceCurve<N> {
    /// Builds a curve from ascending `(x, y)` control points.
    ///
    /// `x` values should ascend. Repeated final points are supported. Behavior
    /// is linear interpolation between neighbors (no validation in `const` context).
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
    #[inline]
    pub fn get_output(&self, input: f32) -> f32 {
        debug_assert!(N != 0);

        // A single-point curve is constant: both clamp ends return its `y`.
        if N == 1 {
            return self.curve[0].max_y;
        }

        let first_val_pair = self.curve[0];
        if input <= first_val_pair.max_x {
            return first_val_pair.max_y;
        }

        // Manual index loop over `1..N`: the same strict-`>` first-match
        // selection as the former `skip(1).find(...)`, but without iterator
        // adapters. `N` is const per monomorphization, so this unrolls into
        // check-free indexed loads.
        let mut i = 1usize;
        while i < N {
            if self.curve[i].max_x > input {
                break;
            }
            i += 1;
        }
        if i == N {
            return self.curve[N - 1].max_y;
        }

        let pair = self.curve[i];
        let interp_frac = (input - pair.base_x) / pair.x_diff;
        pair.y_diff * interp_frac + pair.base_y
    }
}

#[cfg(feature = "serde")]
impl<const N: usize> serde::Serialize for LinearPieceCurve<N>
where
    [(f32, f32); N]: serde::Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let points = self.curve.map(|piece| (piece.max_x, piece.max_y));
        serde::Serialize::serialize(&points, serializer)
    }
}

#[cfg(feature = "serde")]
impl<'de, const N: usize> serde::Deserialize<'de> for LinearPieceCurve<N>
where
    [(f32, f32); N]: serde::Deserialize<'de>,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let points = <[(f32, f32); N] as serde::Deserialize>::deserialize(deserializer)?;
        Ok(Self::new(points))
    }
}
