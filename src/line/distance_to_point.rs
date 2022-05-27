/// Types which implement this trait can compute their distance from an
/// arbitrary point.
pub trait DistanceToPoint {
    /// Compute the L2 Norm distance from this object to an arbitrary point.
    fn distance_to_point(&self, point: &::nalgebra::Point2<f32>) -> f32;

    /// Compute the squared L2 Norm distance from this object to an arbritrary
    /// point.
    fn distance_to_point_squared(&self, point: &::nalgebra::Point2<f32>)
        -> f32;
}

impl DistanceToPoint for nalgebra::Point2<f32> {
    /// The distance between two points is just `|a - b|`.
    ///
    /// Some implementations use negative values to indicate direction.
    /// Therefore, it's important to compare absolute values when checking
    /// distances between multiple different implementations.
    fn distance_to_point(&self, point: &nalgebra::Point2<f32>) -> f32 {
        (point - self).norm()
    }

    /// The squared distance between two points is just `|a - b| * |a - b|`.
    ///
    /// Some implementations use negative values to indicate direction.
    /// Therefore, it's important to compare absolute values when checking
    /// distances between multiple different implementations.
    fn distance_to_point_squared(&self, point: &nalgebra::Point2<f32>) -> f32 {
        (point - self).norm_squared()
    }
}
