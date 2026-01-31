//! Geometry primitives for the slicer.
//!
//! This module provides the fundamental geometric types used throughout the slicing pipeline:
//! - [`Point`] and [`Point3`] - 2D and 3D points with integer coordinates (scaled)
//! - [`PointF`] and [`Point3F`] - 2D and 3D points with floating-point coordinates (unscaled)
//! - [`Line`] - Line segment between two points
//! - [`Polygon`] - Closed polygon (boundary)
//! - [`Polyline`] - Open polyline (path)
//! - [`ExPolygon`] - Polygon with holes (exterior + interior contours)
//! - [`BoundingBox`] and [`BoundingBox3`] - Axis-aligned bounding boxes
//!
//! ## Coordinate System
//!
//! The slicer uses scaled integer coordinates internally to avoid floating-point precision issues.
//! Coordinates are scaled by `SCALING_FACTOR` (1,000,000), so 1 unit = 1 nanometer.
//!
//! - Use `scale()` / `scaled()` to convert from mm to internal units
//! - Use `unscale()` / `unscaled()` to convert from internal units to mm

pub mod aabb_tree;
mod bounding_box;
pub mod elephant_foot;
mod expolygon;
mod line;
mod point;
mod polygon;
mod polyline;
pub mod simplify;
mod transform;

pub use bounding_box::{BoundingBox, BoundingBox3, BoundingBox3F, BoundingBoxF};
pub use elephant_foot::{
    calculate_compensation, compensate_expolygon, compensate_expolygons, compensate_polygon,
    elephant_foot_spacing, scaled_elephant_foot_spacing, ElephantFootCompensator,
    ElephantFootConfig,
};
pub use expolygon::{ExPolygon, ExPolygons};
pub use line::{Line, Lines};
pub use point::{Point, Point3, Point3F, PointF, Points, Points3};
pub use polygon::{Polygon, Polygons};
pub use polyline::{Polyline, Polylines};
pub use simplify::{
    douglas_peucker, douglas_peucker_polygon, douglas_peucker_polyline, remove_collinear_points,
    remove_duplicate_points, simplify_comprehensive, simplify_polygon,
    simplify_polygon_comprehensive, simplify_polygons, simplify_polyline,
    simplify_polyline_comprehensive, simplify_polylines, simplify_resolution, SimplifyConfig,
    COLLINEARITY_THRESHOLD, MESHFIX_MAXIMUM_DEVIATION, MESHFIX_MAXIMUM_EXTRUSION_AREA_DEVIATION,
    MESHFIX_MAXIMUM_RESOLUTION, MINIMUM_SEGMENT_LENGTH,
};
pub use transform::{Transform2D, Transform3D};

// Re-export AABB tree types
pub use aabb_tree::{
    closest_point_on_triangle, ray_box_intersect, ray_triangle_intersect, AABBClosestPointResult,
    AABBNode, AABBTree, IndexedTriangleSet, RayHit, Vec3, AABB3,
};

use crate::{Coord, CoordF};

/// Calculate the cross product of two 2D vectors (returns a scalar).
/// This is useful for determining the orientation of three points.
#[inline]
pub fn cross2(v1: Point, v2: Point) -> i128 {
    v1.x as i128 * v2.y as i128 - v1.y as i128 * v2.x as i128
}

/// Calculate the cross product of two 2D vectors (floating-point version).
#[inline]
pub fn cross2f(v1: PointF, v2: PointF) -> CoordF {
    v1.x * v2.y - v1.y * v2.x
}

/// Calculate the dot product of two 2D vectors.
#[inline]
pub fn dot2(v1: Point, v2: Point) -> i128 {
    v1.x as i128 * v2.x as i128 + v1.y as i128 * v2.y as i128
}

/// Calculate the dot product of two 2D vectors (floating-point version).
#[inline]
pub fn dot2f(v1: PointF, v2: PointF) -> CoordF {
    v1.x * v2.x + v1.y * v2.y
}

/// Calculate the perpendicular vector (rotate 90 degrees counter-clockwise).
#[inline]
pub fn perp(v: Point) -> Point {
    Point::new(-v.y, v.x)
}

/// Calculate the perpendicular vector (floating-point version).
#[inline]
pub fn perpf(v: PointF) -> PointF {
    PointF::new(-v.y, v.x)
}

/// Calculate the angle between two vectors (in radians).
pub fn angle_between(v1: PointF, v2: PointF) -> CoordF {
    let dot = dot2f(v1, v2);
    let cross = cross2f(v1, v2);
    cross.atan2(dot)
}

/// Linear interpolation between two points.
#[inline]
pub fn lerp(a: Point, b: Point, t: CoordF) -> Point {
    Point::new(
        (a.x as CoordF + (b.x - a.x) as CoordF * t).round() as Coord,
        (a.y as CoordF + (b.y - a.y) as CoordF * t).round() as Coord,
    )
}

/// Linear interpolation between two points (floating-point version).
#[inline]
pub fn lerpf(a: PointF, b: PointF, t: CoordF) -> PointF {
    PointF::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

/// Check if a value is approximately equal to another within epsilon.
#[inline]
pub fn approx_eq(a: CoordF, b: CoordF, epsilon: CoordF) -> bool {
    (a - b).abs() < epsilon
}

/// Check if two points are approximately equal.
#[inline]
pub fn points_approx_eq(a: PointF, b: PointF, epsilon: CoordF) -> bool {
    approx_eq(a.x, b.x, epsilon) && approx_eq(a.y, b.y, epsilon)
}

/// Orientation of three points.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientation {
    /// Counter-clockwise (left turn)
    CounterClockwise,
    /// Clockwise (right turn)
    Clockwise,
    /// Collinear (no turn)
    Collinear,
}

/// Determine the orientation of three points.
pub fn orientation(p1: Point, p2: Point, p3: Point) -> Orientation {
    let cross = cross2(p2 - p1, p3 - p2);
    if cross > 0 {
        Orientation::CounterClockwise
    } else if cross < 0 {
        Orientation::Clockwise
    } else {
        Orientation::Collinear
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cross2() {
        let v1 = Point::new(1, 0);
        let v2 = Point::new(0, 1);
        assert_eq!(cross2(v1, v2), 1); // Counter-clockwise

        let v3 = Point::new(0, -1);
        assert_eq!(cross2(v1, v3), -1); // Clockwise
    }

    #[test]
    fn test_perp() {
        let v = Point::new(1, 0);
        let p = perp(v);
        assert_eq!(p.x, 0);
        assert_eq!(p.y, 1);
    }

    #[test]
    fn test_orientation() {
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1, 0);
        let p3 = Point::new(1, 1);
        assert_eq!(orientation(p1, p2, p3), Orientation::CounterClockwise);

        let p4 = Point::new(1, -1);
        assert_eq!(orientation(p1, p2, p4), Orientation::Clockwise);

        let p5 = Point::new(2, 0);
        assert_eq!(orientation(p1, p2, p5), Orientation::Collinear);
    }

    #[test]
    fn test_lerp() {
        let a = Point::new(0, 0);
        let b = Point::new(100, 100);
        let mid = lerp(a, b, 0.5);
        assert_eq!(mid.x, 50);
        assert_eq!(mid.y, 50);
    }
}
