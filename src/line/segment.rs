use {crate::line::DistanceToPoint, nalgebra::Point2};

/// A line-segment defined by a start point and an end point.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Segment {
    pub start: Point2<f32>,
    pub end: Point2<f32>,
}

impl Segment {
    /// Create a new line segment defined by a start and end point.
    ///
    /// # Example
    ///
    /// ```none
    /// +----------------------------------------------------+
    /// |    ^                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |    +                                               |
    /// |    | start(1, 3)    end(7, 3)                      |
    /// |    +  +-----------------+                          |
    /// |    |                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |x---+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+> |
    /// |    |                                               |
    /// |  y |                                               |
    /// +----------------------------------------------------+
    /// ```
    ///     use ::{
    ///         nalgebra::{point, Point2},
    ///         compgeo::line::Segment,
    ///     };
    ///
    ///     let segment = Segment::new(point![1.0, 3.0], point![7.0, 3.0]);
    ///
    pub fn new(start: Point2<f32>, end: Point2<f32>) -> Self {
        Self { start, end }
    }

    /// Get the closest point on this segment to an arbitrarily provided point.
    ///
    /// # Example 1 - Between Endpoints
    ///
    /// The first case to consider is when the point lies 'between' the two
    /// line endpoints.
    ///
    /// ```none
    /// +----------------------------------------------------+
    /// |    ^                                               |
    /// |  7 +                                               |
    /// |    |                end(6, 6)                      |
    /// |  6 +                 +                             |
    /// |    |               -/                              |
    /// |  5 +              /                                |
    /// |    |            -/                                 |
    /// |  4 +           + ClosestPoint(4, 4)                |
    /// |    |         -/                                    |
    /// |  3 +        /     + P(5, 3)                        |
    /// |    |      -/                                       |
    /// |  2 +     +                                         |
    /// |    |   start(2,2)                                  |
    /// |  1 +                                               |
    /// |    |                                               |
    /// |x---+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+> |
    /// |    |  1  2  3  4  5  6  7  8  9  10 11 12 13 14    |
    /// |  y |                                               |
    /// +----------------------------------------------------+
    /// ```
    ///     use ::{
    ///         compgeo::line::Segment,
    ///         nalgebra::{point, Point2},
    ///         approx::assert_relative_eq,
    ///     };
    ///
    ///     let segment = Segment::new(point![2.0, 2.0], point![6.0, 6.0]);
    ///     assert_relative_eq!(
    ///         segment.closest_point(&point![5.0, 3.0]),
    ///         point![4.0, 4.0],
    ///     );
    ///
    /// # Example 2 - Before The Start Point
    ///
    /// ```none
    /// +----------------------------------------------------+
    /// |    ^                                               |
    /// |  7 +                                               |
    /// |    |                end(6, 6)                      |
    /// |  6 +                 +                             |
    /// |    |               -/                              |
    /// |  5 +              /                                |
    /// |    |            -/                                 |
    /// |  4 +           /                                   |
    /// |    |         -/                                    |
    /// |  3 +        /                                      |
    /// |    |      -/                                       |
    /// |  2 +     +                                         |
    /// |    |   start(2,2)                                  |
    /// |  1 +                                               |
    /// |    |     +P(2, 0.5)                                |
    /// |x---+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+> |
    /// |    |  1  2  3  4  5  6  7  8  9  10 11 12 13 14    |
    /// |  y |                                               |
    /// +----------------------------------------------------+
    /// ```
    ///     use ::{
    ///         compgeo::line::Segment,
    ///         nalgebra::{point, Point2},
    ///         approx::assert_relative_eq,
    ///     };
    ///
    ///     let segment = Segment::new(point![2.0, 2.0], point![6.0, 6.0]);
    ///     assert_relative_eq!(
    ///         segment.closest_point(&point![2.0, 0.5]),
    ///         point![2.0, 2.0],
    ///     );
    ///
    /// # Example 3 - After The End Point
    ///
    /// ```none
    /// +----------------------------------------------------+
    /// |    ^                                               |
    /// |  7 +                             +P(10, 7)         |
    /// |    |                end(6, 6)                      |
    /// |  6 +                 +                             |
    /// |    |               -/                              |
    /// |  5 +              /                                |
    /// |    |            -/                                 |
    /// |  4 +           /                                   |
    /// |    |         -/                                    |
    /// |  3 +        /                                      |
    /// |    |      -/                                       |
    /// |  2 +     +                                         |
    /// |    |   start(2,2)                                  |
    /// |  1 +                                               |
    /// |    |                                               |
    /// |x---+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+> |
    /// |    |  1  2  3  4  5  6  7  8  9  10 11 12 13 14    |
    /// |  y |                                               |
    /// +----------------------------------------------------+
    /// ```
    ///     use ::{
    ///         compgeo::line::Segment,
    ///         nalgebra::{point, Point2},
    ///         approx::assert_relative_eq,
    ///     };
    ///
    ///     let segment = Segment::new(point![2.0, 2.0], point![6.0, 6.0]);
    ///     assert_relative_eq!(
    ///         segment.closest_point(&point![10.0, 7.0]),
    ///         point![6.0, 6.0],
    ///     );
    ///
    pub fn closest_point(&self, point: &Point2<f32>) -> Point2<f32> {
        let direction = self.end - self.start;
        let w = point - self.start;

        let c1 = w.dot(&direction);
        if c1 <= 0.0 {
            // This only occurs if the point is *before* the start point.
            return self.start;
        }

        let c2 = direction.norm_squared();
        if c2 <= c1 {
            // this only occurs if the point is *after* the end point.
            return self.end;
        }

        let b = c1 / c2;
        self.start + b * direction
    }

    /// The distance between the start and end points.
    pub fn length(&self) -> f32 {
        (self.start - self.end).norm()
    }

    /// The squared distance between the start and end points.
    ///
    /// Note: this is faster to compute than the [`Segment::length`] because
    ///       there's no `sqrt` operation.
    pub fn length_squared(&self) -> f32 {
        (self.start - self.end).norm_squared()
    }
}

impl DistanceToPoint for Segment {
    /// Compute the distance from the nearby point to this line segment.
    fn distance_to_point(&self, point: &nalgebra::Point2<f32>) -> f32 {
        (point - self.closest_point(point)).norm()
    }

    /// Compute the squared distance between the line segment and a point.
    ///
    /// See [`Segment::distance_to_point`] for a detailed explanation.
    fn distance_to_point_squared(&self, point: &nalgebra::Point2<f32>) -> f32 {
        (point - self.closest_point(point)).norm_squared()
    }
}
