//! Misc. operations on points and vectors in 2d.

use nalgebra::{vector, Unit, Vector2};

/// Compute a perpendicular vector by rotating the given vector 90 degrees
/// counterclockwise.
///
/// #Example
///
///     use {
///         compgeo::operations::perp_vec2d,
///         nalgebra::{vector, Unit, Vector2},
///         approx::assert_relative_eq,
///     };
///
///     let original = vector![1.0, 1.0];
///     let normal = perp_vec2d(&original);
///
///     assert_relative_eq!(normal, vector![-1.0, 1.0]);
///     assert_relative_eq!(normal.norm(), original.norm());
///
pub fn perp_vec2d(vector: &Vector2<f32>) -> Vector2<f32> {
    vector![-vector.y, vector.x]
}

/// Compute a perpendicular vector by rotating the given vector 90 degrees
/// counterclockwise.
///
/// This function is specialized for unit vectors to avoid renormalization.
///
/// # Example
///
///     use {
///         compgeo::operations::perp_unit2d,
///         nalgebra::{vector, Unit, Vector2},
///         approx::assert_relative_eq,
///     };
///
///     let original = Unit::new_normalize(vector![1.0, 1.0]);
///     let normal = perp_unit2d(&original);
///
///     assert_relative_eq!(normal, Unit::new_normalize(vector![-1.0, 1.0]));
///     assert_relative_eq!(normal.norm(), 1.0);
///
pub fn perp_unit2d(vector: &Unit<Vector2<f32>>) -> Unit<Vector2<f32>> {
    Unit::new_unchecked(vector![-vector.y, vector.x])
}
