use {
    crate::{
        line::{DistanceToPoint, Ray},
        operations::perp_unit2d,
    },
    nalgebra::{Point2, Unit, Vector2},
};

/// A line in 2-dimensions which extends infinitely in either direction.
///
/// A line is defined by its normal and offset along the normal to create
/// the following implicit construction:
///
/// ```math
/// f(x, y) = ax + by + c = 0
/// ```
///
/// Where the vector `(a, b)` is the normal vector to the line and `c` is the
/// offset along the normal.
///
#[derive(Debug, Copy, Clone)]
pub struct Line {
    /// The normal vector for the line - the vector which is perpendicular to
    /// the line.
    pub normal: Unit<Vector2<f32>>,

    /// The constant offset used in the implicit line equation.
    pub c: f32,
}

impl Line {
    /// Create a new line from the given normal vector and offset.
    ///
    /// A line is defined by the equation:
    ///
    /// ```math
    /// f(x, y) = normal.x * x + normal.y * y + c == 0
    /// ```
    ///
    /// Where `normal` and `c` are the arguments to this constructor.
    ///
    /// # Example
    ///
    ///     use ::{
    ///         compgeo::line::Line,
    ///         nalgebra::{Point2, Unit, vector, point},
    ///     };
    ///
    ///     let line = Line::new(Unit::new_normalize(vector![0.0, 1.0]), 0.0);
    ///
    pub fn new(normal: Unit<Vector2<f32>>, c: f32) -> Self {
        Self { normal, c }
    }
}

impl DistanceToPoint for Line {
    /// Compute the distance from the point to the line. The output is signed
    /// and can therefore be used to tell if the given point is 'above' or
    /// 'below' the line based on the normal vector.
    ///
    /// # Example
    ///
    ///     use ::{
    ///         compgeo::line::{DistanceToPoint, Line},
    ///         nalgebra::{Point2, Unit, vector, point},
    ///         approx::assert_relative_eq,
    ///     };
    ///
    ///     let line = Line::new(Unit::new_normalize(vector![0.0, 1.0]), 0.0);
    ///     let distance = line.distance_to_point(&point![3.0, 2.3]);
    ///
    ///     assert_relative_eq!(distance, 2.3);
    ///
    ///     let line = Line::new(
    ///         Unit::new_normalize(vector![1.0, 1.0]),
    ///         -2.0
    ///     );
    ///     let distance = line.distance_to_point(&point![1.0, 1.0]);
    ///
    ///     assert_relative_eq!(distance, -0.58578646);
    ///
    fn distance_to_point(&self, point: &Point2<f32>) -> f32 {
        // This equation works because the normal vector is always of unit
        // length
        (self.normal.x * point.x) + (self.normal.y * point.y) + self.c
    }

    /// The squared distance from the line to a point.
    ///
    /// This can be useful if other algorithms are using squared distance for
    /// calculations.
    ///
    /// Note: this actually a little more expensive to compute than
    ///       [`Line::distance_to_point`].
    fn distance_to_point_squared(&self, point: &Point2<f32>) -> f32 {
        let distance = self.distance_to_point(point);
        distance * distance
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
