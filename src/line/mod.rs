//! Structs and algorithms for Lines in two dimensions.

mod infinite;
mod ray;
mod segment;

pub use self::{infinite::Line, ray::Ray, segment::Segment};
