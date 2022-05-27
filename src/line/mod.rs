//! Structs and algorithms for Lines in two dimensions.

mod distance_to_point;
mod infinite;
mod ray;
mod segment;

pub mod intersection;

pub use self::{
    distance_to_point::DistanceToPoint, infinite::Line, ray::Ray,
    segment::Segment,
};
