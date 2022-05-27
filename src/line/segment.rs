use crate::line::Ray;
use nalgebra::{Point2, Unit};

/// A line-segment defined by a start point and an end point.
#[derive(Debug, Copy, Clone)]
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
    ///
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
}

impl From<Segment> for Ray {
    /// Create a Ray which starts at the segment's `start` point and points
    /// towards the `end` point.
    ///
    /// # Example
    ///
    /// Turn a segment like:
    ///
    /// ```none
    /// +----------------------------------------------------+
    /// |    ^                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |    +                                  -+           |
    /// |    |                               --/ end(12,6)   |
    /// |    +                           ---/                |
    /// |    |                        --/                    |
    /// |    +                     --/                       |
    /// |    |                 ---/                          |
    /// |    +              --/                              |
    /// |    |           --/                                 |
    /// |    +       ---/                                    |
    /// |    |    --/                                        |
    /// |    +  +/ start(1, 1)                               |
    /// |    |                                               |
    /// |x---+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+> |
    /// |    |                                               |
    /// |  y |                                               |
    /// +----------------------------------------------------+
    /// ```
    ///
    /// Into a ray like:
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
    /// |    |                                               |
    /// |    +                                               |
    /// |    |          direction(0.910366, 0.413803)        |
    /// |    +        ->                                     |
    /// |    |    ---/                                       |
    /// |    +  +/                                           |
    /// |    |   origin(1, 1)                                |
    /// |x---+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+> |
    /// |    |                                               |
    /// |  y |                                               |
    /// +----------------------------------------------------+
    /// ```
    ///     use ::{
    ///         compgeo::line::{Segment, Ray},
    ///         nalgebra::{Point2, point, vector, Unit},
    ///         approx::assert_relative_eq,
    ///     };
    ///
    ///     let segment = Segment::new(point![1.0, 1.0], point![12.0, 6.0]);
    ///     let ray: Ray = segment.into();
    ///
    ///     assert_relative_eq!(ray.origin, point![1.0, 1.0]);
    ///     assert_relative_eq!(
    ///         ray.direction,
    ///         Unit::new_unchecked(vector![0.9103665, 0.41380295])
    ///     );
    ///
    fn from(segment: Segment) -> Self {
        let direction = Unit::new_normalize(segment.end - segment.start);
        let origin = segment.start;
        Ray::new(origin, direction)
    }
}
