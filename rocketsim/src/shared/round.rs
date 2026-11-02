//! Inline rounding helpers for physics quantization.
//!
//! The helpers use the magic-constant method. They do not call a library
//! function. The result is the same as [`f32::round_ties_even`].

use glam::Vec3A;

/// The smallest `f32` whose adjacent binade has an integer unit in the last
/// place. This is `2^23`.
const MAGIC: f32 = 8_388_608.0;

/// Rounds to the nearest integer. Rounds halfway cases to the nearest even
/// integer.
///
/// Finite results match [`f32::round_ties_even`] bit for bit. Signed zeros
/// stay signed. Values at or above `2^23` in magnitude are already integral.
/// NaN and infinities stay unchanged. NaN payload bits can differ from the
/// standard function.
///
/// The add and subtract use the active floating-point rounding mode. The
/// default mode is round to nearest even. The target must not use extended
/// precision for `f32` values.
#[inline]
#[must_use]
pub(crate) fn round_ties_even(x: f32) -> f32 {
    if x.abs() < MAGIC {
        // A signed value keeps the addition in a binade with a unit in the
        // last place.
        let magic = MAGIC.copysign(x);
        let rounded = (x + magic) - magic;
        if rounded == 0.0 {
            // The add and subtract pair lose the sign of a zero result.
            0.0f32.copysign(x)
        } else {
            rounded
        }
    } else {
        // Finite values are integral. NaN and infinities stay unchanged.
        x
    }
}

/// Applies [`round_ties_even`] to each element of a vector.
///
/// The vector path handles bounded values. Other values use the scalar
/// helper. The three elements are independent. Their order does not change
/// the result.
#[inline]
#[must_use]
pub(crate) fn round_ties_even_vec3a(v: Vec3A) -> Vec3A {
    let limit = Vec3A::splat(MAGIC);
    if v.abs().cmplt(limit).all() {
        let magic = limit.copysign(v);
        let rounded = (v + magic) - magic;
        Vec3A::select(rounded.cmpeq(Vec3A::ZERO), rounded.copysign(v), rounded)
    } else {
        Vec3A::new(
            round_ties_even(v.x),
            round_ties_even(v.y),
            round_ties_even(v.z),
        )
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3A;

    use super::{round_ties_even, round_ties_even_vec3a};

    fn assert_scalar_matches_std(x: f32) {
        let actual = round_ties_even(x);
        let expected = x.round_ties_even();
        if x.is_nan() {
            assert!(actual.is_nan());
            assert!(expected.is_nan());
        } else {
            assert_eq!(
                actual.to_bits(),
                expected.to_bits(),
                "input bits {:#010x}",
                x.to_bits()
            );
        }
    }

    fn test_values() -> Vec<f32> {
        const MAGIC: f32 = 8_388_608.0;
        let mut values = vec![
            0.0,
            -0.0,
            0.499_999_97,
            -0.499_999_97,
            0.5,
            -0.5,
            0.500_000_06,
            -0.500_000_06,
            1.5,
            -1.5,
            2.5,
            -2.5,
            f32::MIN_POSITIVE,
            -f32::MIN_POSITIVE,
            f32::from_bits(1),
            -f32::from_bits(1),
            4_194_303.5,
            -4_194_303.5,
            4_194_304.5,
            -4_194_304.5,
            MAGIC - 1.0,
            -(MAGIC - 1.0),
            8_388_607.5,
            -8_388_607.5,
            MAGIC,
            -MAGIC,
            MAGIC + 2.0,
            -MAGIC - 2.0,
            16_777_215.0,
            -16_777_215.0,
            16_777_216.0,
            -16_777_216.0,
            2_147_483_648.0,
            -2_147_483_648.0,
            f32::MAX,
            f32::MIN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::NAN,
            -f32::NAN,
        ];

        let mut rng = fastrand::Rng::with_seed(0x5EED_2026);
        for _ in 0..20_000 {
            values.push(f32::from_bits(rng.u32(..)));
        }

        values
    }

    #[test]
    fn scalar_matches_std_on_tables_and_bit_patterns() {
        for x in test_values() {
            assert_scalar_matches_std(x);
        }
    }

    #[test]
    fn vector_matches_scalar_on_tables_and_bit_patterns() {
        let values = test_values();
        let mut rng = fastrand::Rng::with_seed(0x5EED_2027);

        for _ in 0..4_000 {
            let input = Vec3A::from_array([
                values[rng.usize(..values.len())],
                values[rng.usize(..values.len())],
                values[rng.usize(..values.len())],
            ]);
            let actual = round_ties_even_vec3a(input).to_array();
            let expected = input.to_array();
            for lane in 0..3 {
                let expected = expected[lane].round_ties_even();
                if expected.is_nan() {
                    assert!(actual[lane].is_nan());
                } else {
                    assert_eq!(
                        actual[lane].to_bits(),
                        expected.to_bits(),
                        "lane {lane}, input bits {:#010x}",
                        input.to_array()[lane].to_bits()
                    );
                }
            }
        }
    }
}
