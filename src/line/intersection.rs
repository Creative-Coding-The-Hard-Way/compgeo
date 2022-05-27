//! Functions and types for calculating the intersections between lines.

use crate::{line::Segment, operations::perp_vec2d};

/// This type represents the intersection between two line segments.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SegmentIntersection {
    /// The objects have no intersection.
    None,

    /// The objects intersect at a point.
    Point(nalgebra::Point2<f32>),

    /// The segments overlap in a segment,
    Overlap(Segment),
}

/// Compute the intersection between two line segments.
pub fn intersect_segments(a: &Segment, b: &Segment) -> SegmentIntersection {
    let dir_a = a.end - a.start;
    let dir_b = b.end - b.start;

    // check for degenerate cases: parallel lines, segments which have zero
    // length
    if dir_a.dot(&perp_vec2d(&dir_b)) <= f32::EPSILON {
        // First, check if the segments are degenerate
        let sqr_len_a = a.length_squared();
        let sqr_len_b = b.length_squared();

        if sqr_len_a == 0.0 && sqr_len_b == 0.0 {
            // both segments are just points
            if a.start == b.start {
                return SegmentIntersection::Point(a.start);
            } else {
                return SegmentIntersection::None;
            }
        }

        if sqr_len_a == 0.0 {
            // just segment a is a point
            // TODO: IMPLEMENT THIS
        }

        // The lines are parallel, or so close to it as to be unable to
        // tell.
        return SegmentIntersection::None;
    }

    // let du = u.norm_squared();
    // let dv = v.norm_squared();
    // if du == 0 && dv == 0

    SegmentIntersection::None
}

#[cfg(test)]
mod test {
    use {
        crate::line::{
            intersection::{intersect_segments, SegmentIntersection},
            Segment,
        },
        nalgebra::point,
    };

    #[test]
    pub fn segments_should_not_intersect_when_parallel_and_non_overlapping() {
        let s1 = Segment::new(point![0.0, 0.0], point![1.0, 0.0]);
        let s2 = Segment::new(point![0.0, 1.0], point![2.0, 1.0]);
        assert!(intersect_segments(&s1, &s2) == SegmentIntersection::None);
    }

    #[test]
    pub fn segments_should_not_intersect_when_theyre_just_separate_points() {
        let s1 = Segment::new(point![0.0, 0.0], point![0.0, 0.0]);
        let s2 = Segment::new(point![1.0, 0.0], point![1.0, 0.0]);
        assert!(intersect_segments(&s1, &s2) == SegmentIntersection::None);
    }

    #[test]
    pub fn segments_should_intersect_when_theyre_identical_separate_points() {
        let s1 = Segment::new(point![1.0, 0.0], point![1.0, 0.0]);
        let s2 = Segment::new(point![1.0, 0.0], point![1.0, 0.0]);
        assert!(
            intersect_segments(&s1, &s2)
                == SegmentIntersection::Point(point![1.0, 0.0])
        );
    }
}
