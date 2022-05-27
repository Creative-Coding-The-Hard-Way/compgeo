use {
    crate::line::{DistanceToPoint, Segment},
    nalgebra::{Point2, Unit, Vector2},
};

/// A Ray is a half-line which begins at an origin point.
///
/// The points on a Ray are defined parametrically with the following equation:
///
/// ```math
/// Ray(t) = P + t*V
/// ```
///
/// Where `P` is the origin point for the Ray and `V` is the direction the
/// Ray points.
///
#[derive(Debug, Copy, Clone)]
pub struct Ray {
    /// The Ray's origin in 2d space.
    pub origin: Point2<f32>,

    /// The direction the Ray points.
    pub direction: Unit<Vector2<f32>>,
}

impl Ray {
    /// Create a new Ray with the given origin and direction.
    ///
    /// A Ray is defined by the parametric equation:
    /// ```math
    /// Ray(t) = origin + t*direction
    /// ```
    ///
    /// # Example
    ///
    /// Create a ray which looks like
    ///
    /// ```none
    /// +----------------------------------------------------+
    /// |    ^                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |    +              >                                |
    /// |    |            -/  Ray Direction - Unit(1, 1)     |
    /// |    +           /                                   |
    /// |    |         -/                                    |
    /// |    +        /                                      |
    /// |    |      -/                                       |
    /// |    +     /                                         |
    /// |    |   -/                                          |
    /// |    +  + Ray Origin (1, 1)                          |
    /// |    |                                               |
    /// |x---+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+> |
    /// |    |                                               |
    /// |  y |                                               |
    /// +----------------------------------------------------+
    /// ```
    ///     use ::{
    ///         compgeo::line::Ray,
    ///         nalgebra::{Point2, Unit, vector, point},
    ///     };
    ///
    ///     let ray = Ray::new(
    ///         point![1.0, 1.0],
    ///         Unit::new_normalize(vector![1.0, 1.0])
    ///     );
    ///
    pub fn new(origin: Point2<f32>, direction: Unit<Vector2<f32>>) -> Self {
        Self { origin, direction }
    }

    /// Create a line segment from this Ray with a given length.
    pub fn as_segment(&self, length: f32) -> Segment {
        Segment::new(self.origin, self.origin + self.direction.scale(length))
    }
}

impl DistanceToPoint for Ray {
    /// Compute the signed distance from the ray to a point in space.
    ///
    /// A positive value means that the point is 'in front' of the ray's origin
    /// based on the direction vector. A negative value means that the point is
    /// 'behind' the ray's origin based on the direction vector.
    ///
    /// # Example 1 - Behind
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
    /// |    +     ^                                         |
    /// |    |     |                                         |
    /// |    +     |  Ray at (2, 2) with direction (0, 1)    |
    /// |    |     |                                         |
    /// |    +     +                                         |
    /// |    |                                               |
    /// |    +  + Point at (1, 1)                            |
    /// |    |                                               |
    /// |x---+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+> |
    /// |    |                                               |
    /// |  y |                                               |
    /// +----------------------------------------------------+
    /// ```
    ///     use ::{
    ///         compgeo::line::{DistanceToPoint, Ray},
    ///         nalgebra::{Point2, Unit, vector, point},
    ///         approx::assert_relative_eq,
    ///     };
    ///
    ///     let ray = Ray::new(
    ///         point![2.0, 2.0],
    ///         Unit::new_normalize(vector![0.0, 1.0])
    ///     );
    ///     let point = point![1.0, 1.0];
    ///
    ///     // distance is negative because the point is "behind" the ray's
    ///     // origin relative to the direction vector
    ///     assert_relative_eq!(
    ///         ray.distance_to_point(&point),
    ///         -(ray.origin - point).norm()
    ///     );
    ///
    /// # Example 2 - In Front
    ///
    /// ```none
    /// +----------------------------------------------------+
    /// |    ^                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |    +  + Point at (1, 5)                            |
    /// |    |                                               |
    /// |    +     ^                                         |
    /// |    |     |                                         |
    /// |    +     |  Ray at (2, 2) with direction (0, 1)    |
    /// |    |     |                                         |
    /// |    +     +                                         |
    /// |    |                                               |
    /// |    +                                               |
    /// |    |                                               |
    /// |x---+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+> |
    /// |    |                                               |
    /// |  y |                                               |
    /// +----------------------------------------------------+
    /// ```
    ///     use ::{
    ///         compgeo::line::{DistanceToPoint, Ray},
    ///         nalgebra::{Point2, Unit, vector, point},
    ///         approx::assert_relative_eq,
    ///     };
    ///
    ///     let ray = Ray::new(
    ///         point![2.0, 2.0],
    ///         Unit::new_normalize(vector![0.0, 1.0])
    ///     );
    ///     let point = point![1.0, 5.0];
    ///
    ///     assert_relative_eq!(
    ///         ray.distance_to_point(&point),
    ///         1.0,
    ///     );
    ///
    ///
    fn distance_to_point(&self, point: &Point2<f32>) -> f32 {
        let w = point - self.origin;
        let projection = w.dot(&self.direction);
        if projection <= 0.0 {
            // The projection can only be below 0 when the point is *behind*
            // the origin (relative to the direction vector)
            -w.norm()
        } else {
            (w - self.direction.scale(projection)).norm()
        }
    }

    /// Compute the squared distance to the given point.
    ///
    /// The semantics are identical to [`DistanceToPoint::distance_to_point`].
    fn distance_to_point_squared(&self, point: &nalgebra::Point2<f32>) -> f32 {
        let w = point - self.origin;
        let projection = w.dot(&self.direction);
        if projection <= 0.0 {
            // The projection can only be below 0 when the point is *behind*
            // the origin (relative to the direction vector)
            -w.norm_squared()
        } else {
            (w - self.direction.scale(projection)).norm_squared()
        }
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
