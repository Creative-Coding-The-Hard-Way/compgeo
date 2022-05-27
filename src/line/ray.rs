use crate::{line::Line, operations::perp_unit2d};
use nalgebra::{Point2, Unit, Vector2};

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
    ///         compgeo::line::Ray,
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
    ///         compgeo::line::Ray,
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
    pub fn distance_to_point(&self, point: &Point2<f32>) -> f32 {
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
}

impl From<Ray> for Line {
    /// Build an infinite line based on this ray's direction and position.
    ///
    /// # Example
    ///
    ///     use ::{
    ///         compgeo::line::{Line, Ray},
    ///         nalgebra::{Point2, Unit, vector, point},
    ///         approx::assert_relative_eq,
    ///     };
    ///
    ///     let ray = Ray::new(
    ///         point![1.0, 1.0],
    ///         Unit::new_normalize(vector![1.0, 1.0])
    ///     );
    ///     let line: Line = ray.into();
    ///
    ///     assert_relative_eq!(
    ///         line.normal,
    ///         Unit::new_normalize(vector![-1.0, 1.0])
    ///     );
    ///     assert_relative_eq!(line.c, 0.0);
    ///
    fn from(ray: Ray) -> Self {
        let normal = perp_unit2d(&ray.direction);
        let c = -Line::new(normal, 0.0).distance_to_point(&ray.origin);
        Line::new(normal, c)
    }
}
