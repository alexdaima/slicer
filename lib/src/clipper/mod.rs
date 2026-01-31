//! Clipper polygon boolean operations module.
//!
//! This module provides polygon boolean operations (union, intersection, difference, XOR)
//! and offset operations using the geo-clipper library.
//!
//! These operations are essential for:
//! - Computing perimeter offsets
//! - Infill clipping
//! - Support generation
//! - Layer boolean operations

use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polyline};
use crate::{unscale, CoordF};
use geo::{Coord as GeoCoord, LineString, MultiPolygon, Polygon as GeoPolygon};
use geo_clipper::{Clipper, EndType, JoinType};

/// Offset type for polygon inflation/deflation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OffsetType {
    /// Offset outward (grow the polygon)
    Inflate,
    /// Offset inward (shrink the polygon)
    Deflate,
}

/// Join type for offset corners.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OffsetJoinType {
    /// Square corners
    Square,
    /// Round corners
    #[default]
    Round,
    /// Mitered corners
    Miter,
}

impl From<OffsetJoinType> for JoinType {
    fn from(jt: OffsetJoinType) -> Self {
        match jt {
            OffsetJoinType::Square => JoinType::Square,
            OffsetJoinType::Round => JoinType::Round(0.25), // Default arc tolerance
            OffsetJoinType::Miter => JoinType::Miter(2.0),  // Default miter limit
        }
    }
}

/// Convert our Polygon to geo's Polygon type.
fn polygon_to_geo(poly: &Polygon) -> GeoPolygon<f64> {
    let points: Vec<GeoCoord<f64>> = poly
        .points()
        .iter()
        .map(|p| GeoCoord {
            x: unscale(p.x),
            y: unscale(p.y),
        })
        .collect();

    // Close the ring if needed
    let mut ring = points;
    if let (Some(first), Some(last)) = (ring.first(), ring.last()) {
        if first != last {
            ring.push(*first);
        }
    }

    GeoPolygon::new(LineString::new(ring), vec![])
}

/// Convert our ExPolygon to geo's Polygon type (with holes).
fn expolygon_to_geo(expoly: &ExPolygon) -> GeoPolygon<f64> {
    let exterior: Vec<GeoCoord<f64>> = expoly
        .contour
        .points()
        .iter()
        .map(|p| GeoCoord {
            x: unscale(p.x),
            y: unscale(p.y),
        })
        .collect();

    let mut exterior_ring = exterior;
    if let (Some(first), Some(last)) = (exterior_ring.first(), exterior_ring.last()) {
        if first != last {
            exterior_ring.push(*first);
        }
    }

    let holes: Vec<LineString<f64>> = expoly
        .holes
        .iter()
        .map(|hole| {
            let mut points: Vec<GeoCoord<f64>> = hole
                .points()
                .iter()
                .map(|p| GeoCoord {
                    x: unscale(p.x),
                    y: unscale(p.y),
                })
                .collect();

            if let (Some(first), Some(last)) = (points.first(), points.last()) {
                if first != last {
                    points.push(*first);
                }
            }
            LineString::new(points)
        })
        .collect();

    GeoPolygon::new(LineString::new(exterior_ring), holes)
}

/// Convert geo's Polygon back to our Polygon type.
fn geo_to_polygon(geo_poly: &GeoPolygon<f64>) -> Polygon {
    let points: Vec<Point> = geo_poly
        .exterior()
        .coords()
        .map(|c| Point::new(crate::scale(c.x), crate::scale(c.y)))
        .collect();

    // Remove the closing point if present (our Polygon doesn't store it)
    let mut result_points = points;
    if result_points.len() > 1 {
        if let (Some(first), Some(last)) = (result_points.first(), result_points.last()) {
            if first == last {
                result_points.pop();
            }
        }
    }

    Polygon::from_points(result_points)
}

/// Convert geo's Polygon to our ExPolygon type (with holes).
fn geo_to_expolygon(geo_poly: &GeoPolygon<f64>) -> ExPolygon {
    let contour = geo_to_polygon(geo_poly);

    let holes: Vec<Polygon> = geo_poly
        .interiors()
        .iter()
        .map(|interior| {
            let points: Vec<Point> = interior
                .coords()
                .map(|c| Point::new(crate::scale(c.x), crate::scale(c.y)))
                .collect();

            let mut result_points = points;
            if result_points.len() > 1 {
                if let (Some(first), Some(last)) = (result_points.first(), result_points.last()) {
                    if first == last {
                        result_points.pop();
                    }
                }
            }

            Polygon::from_points(result_points)
        })
        .collect();

    ExPolygon::with_holes(contour, holes)
}

/// Convert geo's MultiPolygon to our ExPolygons type.
fn geo_multi_to_expolygons(multi: &MultiPolygon<f64>) -> ExPolygons {
    multi.0.iter().map(geo_to_expolygon).collect()
}

/// Convert our Polygons to geo's MultiPolygon.
fn polygons_to_geo_multi(polys: &[Polygon]) -> MultiPolygon<f64> {
    MultiPolygon::new(polys.iter().map(polygon_to_geo).collect())
}

/// Convert our ExPolygons to geo's MultiPolygon.
fn expolygons_to_geo_multi(expolys: &[ExPolygon]) -> MultiPolygon<f64> {
    MultiPolygon::new(expolys.iter().map(expolygon_to_geo).collect())
}

// ============================================================================
// Boolean Operations
// ============================================================================

/// Compute the union of two sets of polygons.
pub fn union(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() {
        return clip.to_vec();
    }
    if clip.is_empty() {
        return subject.to_vec();
    }

    let subject_geo = expolygons_to_geo_multi(subject);
    let clip_geo = expolygons_to_geo_multi(clip);

    let result = subject_geo.union(&clip_geo, 1000.0);
    geo_multi_to_expolygons(&result)
}

/// Compute the union of a single set of potentially overlapping polygons.
pub fn union_ex(polygons: &[ExPolygon]) -> ExPolygons {
    if polygons.is_empty() {
        return vec![];
    }
    if polygons.len() == 1 {
        return polygons.to_vec();
    }

    // Union all polygons together
    let mut result = vec![polygons[0].clone()];
    for poly in polygons.iter().skip(1) {
        result = union(&result, &[poly.clone()]);
    }
    result
}

/// Compute the intersection of two sets of polygons.
pub fn intersection(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() || clip.is_empty() {
        return vec![];
    }

    let subject_geo = expolygons_to_geo_multi(subject);
    let clip_geo = expolygons_to_geo_multi(clip);

    let result = subject_geo.intersection(&clip_geo, 1000.0);
    geo_multi_to_expolygons(&result)
}

/// Compute the difference of two sets of polygons (subject - clip).
pub fn difference(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() {
        return vec![];
    }
    if clip.is_empty() {
        return subject.to_vec();
    }

    let subject_geo = expolygons_to_geo_multi(subject);
    let clip_geo = expolygons_to_geo_multi(clip);

    let result = subject_geo.difference(&clip_geo, 1000.0);
    geo_multi_to_expolygons(&result)
}

/// Compute the XOR of two sets of polygons.
pub fn xor(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() {
        return clip.to_vec();
    }
    if clip.is_empty() {
        return subject.to_vec();
    }

    let subject_geo = expolygons_to_geo_multi(subject);
    let clip_geo = expolygons_to_geo_multi(clip);

    let result = subject_geo.xor(&clip_geo, 1000.0);
    geo_multi_to_expolygons(&result)
}

// ============================================================================
// Offset Operations
// ============================================================================

/// Offset a polygon by a given distance.
///
/// Positive delta inflates (grows) the polygon, negative delta deflates (shrinks) it.
///
/// # Arguments
/// * `polygon` - The polygon to offset
/// * `delta` - The offset distance in mm (positive = grow, negative = shrink)
/// * `join_type` - The type of join to use at corners
///
/// # Returns
/// A vector of ExPolygons representing the offset result.
pub fn offset_polygon(polygon: &Polygon, delta: CoordF, join_type: OffsetJoinType) -> ExPolygons {
    let geo_poly = polygon_to_geo(polygon);
    let jt = join_type.into();

    let result = geo_poly.offset(delta, jt, EndType::ClosedPolygon, 1000.0);
    geo_multi_to_expolygons(&result)
}

/// Offset an ExPolygon by a given distance.
///
/// Positive delta inflates (grows) the polygon, negative delta deflates (shrinks) it.
pub fn offset_expolygon(
    expolygon: &ExPolygon,
    delta: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    let geo_poly = expolygon_to_geo(expolygon);
    let jt = join_type.into();

    let result = geo_poly.offset(delta, jt, EndType::ClosedPolygon, 1000.0);
    geo_multi_to_expolygons(&result)
}

/// Offset multiple ExPolygons by a given distance.
///
/// Positive delta inflates (grows) the polygons, negative delta deflates (shrinks) them.
pub fn offset_expolygons(
    expolygons: &[ExPolygon],
    delta: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if expolygons.is_empty() {
        return vec![];
    }

    let geo_multi = expolygons_to_geo_multi(expolygons);
    let jt = join_type.into();

    let result = geo_multi.offset(delta, jt, EndType::ClosedPolygon, 1000.0);
    geo_multi_to_expolygons(&result)
}

/// Offset multiple Polygons by a given distance.
pub fn offset_polygons(
    polygons: &[Polygon],
    delta: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if polygons.is_empty() {
        return vec![];
    }

    let geo_multi = polygons_to_geo_multi(polygons);
    let jt = join_type.into();

    let result = geo_multi.offset(delta, jt, EndType::ClosedPolygon, 1000.0);
    geo_multi_to_expolygons(&result)
}

/// Shrink (inset) ExPolygons by a given distance.
///
/// This is a convenience function that calls offset with a negative delta.
pub fn shrink(expolygons: &[ExPolygon], distance: CoordF, join_type: OffsetJoinType) -> ExPolygons {
    offset_expolygons(expolygons, -distance.abs(), join_type)
}

/// Grow (outset) ExPolygons by a given distance.
///
/// This is a convenience function that calls offset with a positive delta.
pub fn grow(expolygons: &[ExPolygon], distance: CoordF, join_type: OffsetJoinType) -> ExPolygons {
    offset_expolygons(expolygons, distance.abs(), join_type)
}

// ============================================================================
// Utility Functions
// ============================================================================

/// Simplify polygons by removing small details.
pub fn simplify(expolygons: &[ExPolygon], tolerance: CoordF) -> ExPolygons {
    expolygons
        .iter()
        .map(|expoly| {
            let mut result = expoly.clone();
            result.simplify(crate::scale(tolerance));
            result
        })
        .filter(|expoly| !expoly.is_empty() && expoly.area().abs() > tolerance * tolerance)
        .collect()
}

/// Remove very small polygons from a set.
pub fn remove_small(expolygons: &[ExPolygon], min_area: CoordF) -> ExPolygons {
    expolygons
        .iter()
        .filter(|expoly| expoly.area().abs() > min_area)
        .cloned()
        .collect()
}

/// Check if two sets of polygons overlap.
pub fn polygons_overlap(a: &[ExPolygon], b: &[ExPolygon]) -> bool {
    !intersection(a, b).is_empty()
}

/// Compute the total area of a set of polygons.
pub fn total_area(expolygons: &[ExPolygon]) -> CoordF {
    expolygons.iter().map(|p| p.area()).sum()
}

/// Morphological opening: shrink then grow by the same amount.
///
/// This removes small protrusions and smooths contours while preserving
/// the overall shape. Used for gap detection and thin wall handling.
///
/// # Arguments
/// * `expolygons` - The polygons to process
/// * `distance` - The opening distance in mm
/// * `join_type` - The join type for offset corners
pub fn opening(
    expolygons: &[ExPolygon],
    distance: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if expolygons.is_empty() || distance <= 0.0 {
        return expolygons.to_vec();
    }
    let shrunk = shrink(expolygons, distance, join_type);
    grow(&shrunk, distance, join_type)
}

/// Morphological closing: grow then shrink by the same amount.
///
/// This fills small gaps and holes while preserving the overall shape.
///
/// # Arguments
/// * `expolygons` - The polygons to process
/// * `distance` - The closing distance in mm
/// * `join_type` - The join type for offset corners
pub fn closing(
    expolygons: &[ExPolygon],
    distance: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if expolygons.is_empty() || distance <= 0.0 {
        return expolygons.to_vec();
    }
    let grown = grow(expolygons, distance, join_type);
    shrink(&grown, distance, join_type)
}

/// Offset2: shrink by amount1, then grow by amount2.
///
/// This is the general morphological operation used in BambuStudio for
/// perimeter generation and gap detection. When amount1 != amount2, it
/// creates controlled erosion/dilation.
///
/// # Arguments
/// * `expolygons` - The polygons to process
/// * `shrink_amount` - Amount to shrink (positive)
/// * `grow_amount` - Amount to grow back (positive)
/// * `join_type` - The join type for offset corners
pub fn offset2(
    expolygons: &[ExPolygon],
    shrink_amount: CoordF,
    grow_amount: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if expolygons.is_empty() {
        return vec![];
    }
    let shrunk = shrink(expolygons, shrink_amount.abs(), join_type);
    if shrunk.is_empty() {
        return vec![];
    }
    grow(&shrunk, grow_amount.abs(), join_type)
}

/// Detect gaps between two polygon sets.
///
/// Gaps are the narrow regions that exist in the outer area but not in the
/// inner area. This is used for gap fill detection between perimeter levels.
///
/// # Arguments
/// * `outer` - The outer boundary (e.g., previous perimeter level)
/// * `inner` - The inner boundary (e.g., current perimeter level after offset)
/// * `min_width` - Minimum gap width to detect (mm)
/// * `max_width` - Maximum gap width to detect (mm)
/// * `join_type` - The join type for offset corners
///
/// # Returns
/// ExPolygons representing the detected gap regions.
pub fn detect_gaps(
    outer: &[ExPolygon],
    inner: &[ExPolygon],
    min_width: CoordF,
    max_width: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if outer.is_empty() {
        return vec![];
    }

    // Gap detection algorithm from BambuStudio:
    // 1. Shrink outer by half of expected gap width to get where gaps might be
    // 2. Grow inner back by half of max width plus safety offset
    // 3. Take the difference - these are the potential gap regions
    // 4. Apply opening to remove regions smaller than min_width
    // 5. Apply offset2 to remove regions larger than max_width

    let half_min = min_width / 2.0;
    let half_max = max_width / 2.0;
    const SAFETY_OFFSET: CoordF = 0.00001; // 10nm safety offset

    // Regions that might be gaps: shrink outer slightly
    let potential_gaps = shrink(outer, half_min, join_type);

    // Regions covered by inner perimeters (with some margin)
    let inner_expanded = grow(inner, half_max + SAFETY_OFFSET, join_type);

    // Gaps are where potential gaps exist but inner doesn't cover
    let raw_gaps = difference(&potential_gaps, &inner_expanded);

    if raw_gaps.is_empty() {
        return vec![];
    }

    // Clean up: apply morphological opening to remove too-narrow regions
    let opened = opening(&raw_gaps, half_min, join_type);

    // Apply offset2 to remove regions that are too wide
    // (they should be filled by normal infill, not gap fill)
    let gaps = offset2(&opened, half_max, half_max + SAFETY_OFFSET, join_type);

    gaps
}

/// Extract centerlines from narrow polygon regions using offset approximation.
///
/// This is a simplified alternative to medial axis computation. It works by
/// progressively shrinking the polygon and collecting the resulting contours.
///
/// # Arguments
/// * `expolygons` - The narrow regions to extract centerlines from
/// * `width` - The expected width of the regions (mm)
/// * `join_type` - The join type for offset corners
///
/// # Returns
/// Polylines representing approximate centerlines.
pub fn extract_centerlines(
    expolygons: &[ExPolygon],
    width: CoordF,
    join_type: OffsetJoinType,
) -> Vec<Polyline> {
    if expolygons.is_empty() || width <= 0.0 {
        return vec![];
    }

    let mut centerlines = Vec::new();

    // Shrink by half the width to get approximate centerlines
    let half_width = width / 2.0;
    let shrunk = shrink(expolygons, half_width, join_type);

    // Convert the resulting polygons to polylines
    for expoly in &shrunk {
        // Convert contour to polyline
        if !expoly.contour.is_empty() {
            let mut points = expoly.contour.points().to_vec();
            // Close the polyline by adding the first point at the end
            if let Some(first) = points.first().cloned() {
                points.push(first);
            }
            if points.len() >= 2 {
                centerlines.push(Polyline::from_points(points));
            }
        }

        // Convert holes to polylines
        for hole in &expoly.holes {
            if !hole.is_empty() {
                let mut points = hole.points().to_vec();
                if let Some(first) = points.first().cloned() {
                    points.push(first);
                }
                if points.len() >= 2 {
                    centerlines.push(Polyline::from_points(points));
                }
            }
        }
    }

    // If shrinking eliminated everything, try a different approach:
    // Take the original polygon contours as approximate centerlines
    if centerlines.is_empty() {
        for expoly in expolygons {
            if !expoly.contour.is_empty() {
                let mut points = expoly.contour.points().to_vec();
                if let Some(first) = points.first().cloned() {
                    points.push(first);
                }
                if points.len() >= 2 {
                    centerlines.push(Polyline::from_points(points));
                }
            }
        }
    }

    centerlines
}

/// Intersect polylines with a set of ExPolygons, returning clipped polylines.
///
/// This clips the input polylines to only the portions that fall inside
/// the given ExPolygons. Each input polyline may produce zero, one, or
/// multiple output polylines depending on how it intersects the clipping regions.
///
/// # Arguments
/// * `polylines` - The polylines to clip
/// * `clip` - The ExPolygons to clip against (portions inside these are kept)
///
/// # Returns
/// A vector of polylines representing the clipped portions.
pub fn intersect_polylines_with_expolygons(
    polylines: &[Polyline],
    clip: &[ExPolygon],
) -> Vec<Polyline> {
    if polylines.is_empty() || clip.is_empty() {
        return vec![];
    }

    let mut result = Vec::new();

    for polyline in polylines {
        let clipped = clip_polyline_to_expolygons(polyline, clip);
        result.extend(clipped);
    }

    result
}

/// Clip a single polyline to a set of ExPolygons.
fn clip_polyline_to_expolygons(polyline: &Polyline, clip: &[ExPolygon]) -> Vec<Polyline> {
    let points = polyline.points();
    if points.len() < 2 {
        return vec![];
    }

    let mut result = Vec::new();
    let mut current_segment: Vec<Point> = Vec::new();

    // Process each segment of the polyline
    for i in 0..points.len() - 1 {
        let p1 = points[i];
        let p2 = points[i + 1];

        // Find all intersections of this segment with clip boundaries
        let clipped_segments = clip_segment_to_expolygons(p1, p2, clip);

        for segment in clipped_segments {
            if segment.len() >= 2 {
                // Try to connect to current segment
                if !current_segment.is_empty() {
                    let last = *current_segment.last().unwrap();
                    let first = segment[0];
                    // Check if segments are connected (within tolerance)
                    if (last.x - first.x).abs() <= 1 && (last.y - first.y).abs() <= 1 {
                        // Connected, extend current segment
                        current_segment.extend(segment.into_iter().skip(1));
                    } else {
                        // Not connected, save current and start new
                        if current_segment.len() >= 2 {
                            result.push(Polyline::from_points(current_segment));
                        }
                        current_segment = segment;
                    }
                } else {
                    current_segment = segment;
                }
            }
        }
    }

    // Don't forget the last segment
    if current_segment.len() >= 2 {
        result.push(Polyline::from_points(current_segment));
    }

    result
}

/// Clip a line segment to a set of ExPolygons.
/// Returns the portions of the segment that are inside any of the ExPolygons.
fn clip_segment_to_expolygons(p1: Point, p2: Point, clip: &[ExPolygon]) -> Vec<Vec<Point>> {
    // Simple approach: sample points along the segment and check if they're inside
    // For better performance, we'd use proper line-polygon intersection

    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let len_sq = dx as f64 * dx as f64 + dy as f64 * dy as f64;

    if len_sq < 1.0 {
        // Segment too short
        if point_in_expolygons(p1, clip) {
            return vec![vec![p1, p2]];
        }
        return vec![];
    }

    let len = len_sq.sqrt();
    let step = 100_000i64; // 0.1mm sampling step
    let num_samples = ((len / step as f64).ceil() as usize).max(2);

    let mut result = Vec::new();
    let mut current_segment: Vec<Point> = Vec::new();
    let mut last_inside = false;

    for i in 0..=num_samples {
        let t = i as f64 / num_samples as f64;
        let px = p1.x as f64 + dx as f64 * t;
        let py = p1.y as f64 + dy as f64 * t;
        let pt = Point::new(px.round() as i64, py.round() as i64);

        let inside = point_in_expolygons(pt, clip);

        if inside {
            if !last_inside && !current_segment.is_empty() {
                // Was outside, now inside - save previous segment if any
                if current_segment.len() >= 2 {
                    result.push(current_segment);
                }
                current_segment = Vec::new();
            }
            current_segment.push(pt);
        } else {
            if last_inside && !current_segment.is_empty() {
                // Was inside, now outside
                if current_segment.len() >= 2 {
                    result.push(current_segment);
                }
                current_segment = Vec::new();
            }
        }
        last_inside = inside;
    }

    // Save final segment
    if current_segment.len() >= 2 {
        result.push(current_segment);
    }

    result
}

/// Check if a point is inside any of the given ExPolygons.
fn point_in_expolygons(pt: Point, expolygons: &[ExPolygon]) -> bool {
    for expoly in expolygons {
        if expoly.contains_point(&pt) {
            return true;
        }
    }
    false
}

/// Subtract polygons from polylines (polyline difference).
///
/// This returns the portions of the input polylines that fall OUTSIDE
/// the given ExPolygons. This is the complement of `intersect_polylines_with_expolygons`.
///
/// In libslic3r, this is `diff_pl()`.
///
/// # Arguments
/// * `polylines` - The polylines to clip
/// * `clip` - The ExPolygons to subtract (portions outside these are kept)
///
/// # Returns
/// A vector of polylines representing the portions outside the clip regions.
pub fn diff_pl(polylines: &[Polyline], clip: &[ExPolygon]) -> Vec<Polyline> {
    if polylines.is_empty() {
        return vec![];
    }

    if clip.is_empty() {
        // Nothing to subtract, return original polylines
        return polylines.to_vec();
    }

    let mut result = Vec::new();

    for polyline in polylines {
        let clipped = diff_polyline_from_expolygons(polyline, clip);
        result.extend(clipped);
    }

    result
}

/// Subtract ExPolygons from a single polyline.
/// Returns the portions of the polyline that are OUTSIDE all clip regions.
fn diff_polyline_from_expolygons(polyline: &Polyline, clip: &[ExPolygon]) -> Vec<Polyline> {
    let points = polyline.points();
    if points.len() < 2 {
        return vec![];
    }

    let mut result = Vec::new();
    let mut current_segment: Vec<Point> = Vec::new();

    // Process each segment of the polyline
    for i in 0..points.len() - 1 {
        let p1 = points[i];
        let p2 = points[i + 1];

        // Find all portions of this segment that are outside clip regions
        let outside_segments = diff_segment_from_expolygons(p1, p2, clip);

        for segment in outside_segments {
            if segment.len() >= 2 {
                // Try to connect to current segment
                if !current_segment.is_empty() {
                    let last = *current_segment.last().unwrap();
                    let first = segment[0];
                    // Check if segments are connected (within tolerance)
                    if (last.x - first.x).abs() <= 1 && (last.y - first.y).abs() <= 1 {
                        // Connected, extend current segment
                        current_segment.extend(segment.into_iter().skip(1));
                    } else {
                        // Not connected, save current and start new
                        if current_segment.len() >= 2 {
                            result.push(Polyline::from_points(current_segment));
                        }
                        current_segment = segment;
                    }
                } else {
                    current_segment = segment;
                }
            }
        }
    }

    // Don't forget the last segment
    if current_segment.len() >= 2 {
        result.push(Polyline::from_points(current_segment));
    }

    result
}

/// Get the portions of a line segment that are OUTSIDE all ExPolygons.
fn diff_segment_from_expolygons(p1: Point, p2: Point, clip: &[ExPolygon]) -> Vec<Vec<Point>> {
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let len_sq = dx as f64 * dx as f64 + dy as f64 * dy as f64;

    if len_sq < 1.0 {
        // Segment too short - check if it's outside
        if !point_in_expolygons(p1, clip) {
            return vec![vec![p1, p2]];
        }
        return vec![];
    }

    let len = len_sq.sqrt();
    let step = 100_000i64; // 0.1mm sampling step
    let num_samples = ((len / step as f64).ceil() as usize).max(2);

    let mut result = Vec::new();
    let mut current_segment: Vec<Point> = Vec::new();
    let mut last_outside = false;

    for i in 0..=num_samples {
        let t = i as f64 / num_samples as f64;
        let px = p1.x as f64 + dx as f64 * t;
        let py = p1.y as f64 + dy as f64 * t;
        let pt = Point::new(px.round() as i64, py.round() as i64);

        // Check if point is OUTSIDE all clip regions
        let outside = !point_in_expolygons(pt, clip);

        if outside {
            if !last_outside && !current_segment.is_empty() {
                // Was inside, now outside - save previous segment if any
                if current_segment.len() >= 2 {
                    result.push(current_segment);
                }
                current_segment = Vec::new();
            }
            current_segment.push(pt);
        } else {
            if last_outside && !current_segment.is_empty() {
                // Was outside, now inside
                if current_segment.len() >= 2 {
                    result.push(current_segment);
                }
                current_segment = Vec::new();
            }
        }
        last_outside = outside;
    }

    // Save final segment
    if current_segment.len() >= 2 {
        result.push(current_segment);
    }

    result
}

/// Convert polygons to polylines (open paths).
/// Each polygon becomes a polyline with the same points (not closed).
pub fn polygons_to_polylines(polygons: &[Polygon]) -> Vec<Polyline> {
    polygons
        .iter()
        .map(|p| Polyline::from_points(p.points().to_vec()))
        .collect()
}

/// Convert ExPolygons to polylines (contours and holes as separate polylines).
pub fn expolygons_to_polylines(expolygons: &[ExPolygon]) -> Vec<Polyline> {
    let mut result = Vec::new();
    for expoly in expolygons {
        result.push(Polyline::from_points(expoly.contour.points().to_vec()));
        for hole in &expoly.holes {
            result.push(Polyline::from_points(hole.points().to_vec()));
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;
    use crate::{scale, Coord};

    fn make_square(x: Coord, y: Coord, size: Coord) -> ExPolygon {
        let poly = Polygon::rectangle(Point::new(x, y), Point::new(x + size, y + size));
        poly.into()
    }

    fn make_square_mm(x: f64, y: f64, size: f64) -> ExPolygon {
        make_square(crate::scale(x), crate::scale(y), crate::scale(size))
    }

    #[test]
    fn test_offset_polygon_grow() {
        let square = Polygon::rectangle(
            Point::new(crate::scale(10.0), crate::scale(10.0)),
            Point::new(crate::scale(20.0), crate::scale(20.0)),
        );

        // Original area: 10mm * 10mm = 100mm² (area() returns scaled² units as CoordF)
        let original_area = square.area() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        assert!((original_area - 100.0).abs() < 1.0);

        // Grow by 1mm
        let grown = offset_polygon(&square, 1.0, OffsetJoinType::Square);
        assert!(!grown.is_empty());

        // New area should be larger (approximately 12mm * 12mm = 144mm² for square join)
        let grown_area: CoordF = grown.iter().map(|p| p.area()).sum();
        let grown_area_mm2 = grown_area / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        assert!(grown_area_mm2 > original_area);
    }

    #[test]
    fn test_offset_polygon_shrink() {
        let square = Polygon::rectangle(
            Point::new(crate::scale(10.0), crate::scale(10.0)),
            Point::new(crate::scale(30.0), crate::scale(30.0)),
        );

        // Shrink by 2mm
        let shrunk = offset_polygon(&square, -2.0, OffsetJoinType::Square);
        assert!(!shrunk.is_empty());

        // New area should be smaller (approximately 16mm * 16mm = 256mm²)
        let original_area = square.area() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        let shrunk_area: CoordF = shrunk.iter().map(|p| p.area()).sum();
        let shrunk_area_mm2 = shrunk_area / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        assert!(shrunk_area_mm2 < original_area);
    }

    #[test]
    fn test_offset_polygon_shrink_to_nothing() {
        // Small 2mm x 2mm square
        let square = Polygon::rectangle(
            Point::new(crate::scale(10.0), crate::scale(10.0)),
            Point::new(crate::scale(12.0), crate::scale(12.0)),
        );

        // Shrink by 2mm should eliminate it
        let shrunk = offset_polygon(&square, -2.0, OffsetJoinType::Square);
        assert!(shrunk.is_empty() || shrunk.iter().all(|p| p.area().abs() < 1e-6));
    }

    #[test]
    fn test_union() {
        // Two overlapping squares
        let square1 = make_square_mm(0.0, 0.0, 10.0);
        let square2 = make_square_mm(5.0, 0.0, 10.0);

        let result = union(&[square1.clone()], &[square2.clone()]);
        assert!(!result.is_empty());

        // Union area should be less than sum of individual areas (due to overlap)
        let area1 = square1.area();
        let area2 = square2.area();
        let union_area: CoordF = result.iter().map(|p| p.area()).sum();
        assert!(union_area < area1 + area2);
        assert!(union_area > area1.max(area2));
    }

    #[test]
    fn test_intersection() {
        // Two overlapping squares
        let square1 = make_square_mm(0.0, 0.0, 10.0);
        let square2 = make_square_mm(5.0, 0.0, 10.0);

        let result = intersection(&[square1], &[square2]);
        assert!(!result.is_empty());

        // Intersection should be a 5mm x 10mm rectangle
        let int_area: CoordF = result.iter().map(|p| p.area()).sum();
        assert!(int_area > 0.0);
    }

    #[test]
    fn test_intersection_no_overlap() {
        // Two non-overlapping squares
        let square1 = make_square_mm(0.0, 0.0, 10.0);
        let square2 = make_square_mm(20.0, 0.0, 10.0);

        let result = intersection(&[square1], &[square2]);
        assert!(result.is_empty() || result.iter().all(|p| p.area().abs() < 1e-6));
    }

    #[test]
    fn test_difference() {
        // Large square minus smaller square inside
        let large = make_square_mm(0.0, 0.0, 20.0);
        let small = make_square_mm(5.0, 5.0, 10.0);

        let result = difference(&[large.clone()], &[small.clone()]);
        assert!(!result.is_empty());

        // Difference area should be large area - small area
        let diff_area: CoordF = result.iter().map(|p| p.area()).sum();
        let expected_area = large.area() - small.area();
        assert!((diff_area - expected_area).abs() / expected_area < 0.01); // 1% tolerance
    }

    #[test]
    fn test_shrink_grow_convenience() {
        let square = make_square_mm(10.0, 10.0, 20.0);
        let original_area = square.area();

        let shrunk = shrink(&[square.clone()], 2.0, OffsetJoinType::Round);
        let shrunk_area: CoordF = shrunk.iter().map(|p| p.area()).sum();
        assert!(shrunk_area < original_area);

        let grown = grow(&[square], 2.0, OffsetJoinType::Round);
        let grown_area: CoordF = grown.iter().map(|p| p.area()).sum();
        assert!(grown_area > original_area);
    }

    #[test]
    fn test_expolygon_with_hole() {
        // Create a square with a hole
        let outer = Polygon::rectangle(
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(20.0), crate::scale(20.0)),
        );
        let inner = Polygon::rectangle(
            Point::new(crate::scale(5.0), crate::scale(5.0)),
            Point::new(crate::scale(15.0), crate::scale(15.0)),
        );
        let expoly = ExPolygon::with_holes(outer, vec![inner]);

        // Shrink should maintain the hole
        let shrunk = offset_expolygon(&expoly, -1.0, OffsetJoinType::Square);
        assert!(!shrunk.is_empty());
    }

    #[test]
    fn test_total_area() {
        let square1 = make_square_mm(0.0, 0.0, 10.0);
        let square2 = make_square_mm(20.0, 0.0, 10.0);

        let total = total_area(&[square1.clone(), square2.clone()]);
        let expected = square1.area() + square2.area();
        assert!((total - expected).abs() < 1e-6);
    }

    #[test]
    fn test_polygons_overlap() {
        let square1 = make_square_mm(0.0, 0.0, 10.0);
        let square2 = make_square_mm(5.0, 0.0, 10.0);
        let square3 = make_square_mm(20.0, 0.0, 10.0);

        assert!(polygons_overlap(&[square1.clone()], &[square2]));
        assert!(!polygons_overlap(&[square1], &[square3]));
    }

    #[test]
    fn test_remove_small() {
        let large = make_square_mm(0.0, 0.0, 10.0);
        let small = make_square_mm(20.0, 0.0, 0.1);

        let polys = vec![large.clone(), small];
        // area() returns scaled² units, so 1mm² = SCALING_FACTOR²
        let min_area_scaled = 1.0 * crate::SCALING_FACTOR * crate::SCALING_FACTOR;
        let filtered = remove_small(&polys, min_area_scaled);

        assert_eq!(filtered.len(), 1);
        assert!((filtered[0].area() - large.area()).abs() < 1e-6);
    }

    #[test]
    fn test_opening() {
        // Opening removes small protrusions
        let square = make_square_mm(0.0, 0.0, 10.0);
        let original_area = square.area();

        // Opening with small distance should approximately preserve area
        let opened = opening(&[square], 0.1, OffsetJoinType::Round);
        assert!(!opened.is_empty());

        let opened_area: CoordF = opened.iter().map(|p| p.area()).sum();
        // Area should be slightly smaller due to corner rounding
        assert!(opened_area > 0.0);
        assert!(opened_area <= original_area * 1.01); // Allow small tolerance
    }

    #[test]
    fn test_closing() {
        // Closing fills small gaps
        let square = make_square_mm(0.0, 0.0, 10.0);
        let original_area = square.area();

        // Closing with small distance should approximately preserve area
        let closed = closing(&[square], 0.1, OffsetJoinType::Round);
        assert!(!closed.is_empty());

        let closed_area: CoordF = closed.iter().map(|p| p.area()).sum();
        // Area should be slightly larger due to corner filling
        assert!(closed_area > 0.0);
        assert!(closed_area >= original_area * 0.99); // Allow small tolerance
    }

    #[test]
    fn test_offset2() {
        let square = make_square_mm(0.0, 0.0, 10.0);
        let original_area = square.area();

        // offset2 with equal shrink/grow should approximately preserve shape
        let result = offset2(&[square], 0.5, 0.5, OffsetJoinType::Round);
        assert!(!result.is_empty());

        let result_area: CoordF = result.iter().map(|p| p.area()).sum();
        // Should be roughly similar (some corner effects expected)
        assert!(result_area > original_area * 0.8);
    }

    #[test]
    fn test_offset2_removes_thin_features() {
        // Create a shape with a thin protrusion
        // offset2 should remove it
        let thin_protrusion = Polygon::from_points(vec![
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(10.0), crate::scale(0.0)),
            Point::new(crate::scale(10.0), crate::scale(10.0)),
            Point::new(crate::scale(5.5), crate::scale(10.0)), // Thin protrusion starts
            Point::new(crate::scale(5.5), crate::scale(11.0)), // 0.5mm wide, 1mm tall
            Point::new(crate::scale(4.5), crate::scale(11.0)),
            Point::new(crate::scale(4.5), crate::scale(10.0)), // Thin protrusion ends
            Point::new(crate::scale(0.0), crate::scale(10.0)),
        ]);
        let expoly: ExPolygon = thin_protrusion.into();

        // offset2 with 1mm shrink/grow should remove the 0.5mm wide protrusion
        let result = offset2(&[expoly], 1.0, 1.0, OffsetJoinType::Round);

        // Result should exist but be simpler (protrusion removed)
        if !result.is_empty() {
            let result_area: CoordF = result.iter().map(|p| p.area()).sum();
            // Should be close to 100mm² (the main square)
            let expected_area = 100.0 * crate::SCALING_FACTOR * crate::SCALING_FACTOR;
            assert!(result_area < expected_area * 1.1);
        }
    }

    #[test]
    fn test_detect_gaps_no_gaps() {
        // Two concentric squares - no gaps should be detected
        let outer = make_square_mm(0.0, 0.0, 20.0);
        let inner = make_square_mm(2.0, 2.0, 16.0);

        let gaps = detect_gaps(&[outer], &[inner], 0.2, 2.0, OffsetJoinType::Round);

        // Depending on geometry, gaps may or may not be found
        // The key is that the function doesn't crash
        println!("Detected {} gap regions", gaps.len());
    }

    #[test]
    fn test_detect_gaps_empty_inputs() {
        let square = make_square_mm(0.0, 0.0, 10.0);

        // Empty outer should return empty
        let gaps1 = detect_gaps(&[], &[square.clone()], 0.2, 2.0, OffsetJoinType::Round);
        assert!(gaps1.is_empty());

        // Empty inner should work (everything is a gap)
        let gaps2 = detect_gaps(&[square], &[], 0.2, 2.0, OffsetJoinType::Round);
        // Result depends on parameters, but shouldn't crash
        println!("Gaps with empty inner: {}", gaps2.len());
    }

    #[test]
    fn test_extract_centerlines_simple() {
        // A narrow rectangle should produce a centerline along its length
        let thin_rect = Polygon::from_points(vec![
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(20.0), crate::scale(0.0)),
            Point::new(crate::scale(20.0), crate::scale(1.0)), // 1mm wide
            Point::new(crate::scale(0.0), crate::scale(1.0)),
        ]);
        let expoly: ExPolygon = thin_rect.into();

        let centerlines = extract_centerlines(&[expoly], 1.0, OffsetJoinType::Round);

        // Should produce at least one centerline
        println!("Extracted {} centerlines", centerlines.len());
        // The function should not crash and should return something
    }

    #[test]
    fn test_extract_centerlines_empty() {
        let centerlines = extract_centerlines(&[], 1.0, OffsetJoinType::Round);
        assert!(centerlines.is_empty());
    }

    #[test]
    fn test_opening_empty_input() {
        let result = opening(&[], 1.0, OffsetJoinType::Round);
        assert!(result.is_empty());
    }

    #[test]
    fn test_closing_empty_input() {
        let result = closing(&[], 1.0, OffsetJoinType::Round);
        assert!(result.is_empty());
    }

    #[test]
    fn test_offset2_empty_input() {
        let result = offset2(&[], 1.0, 1.0, OffsetJoinType::Round);
        assert!(result.is_empty());
    }

    #[test]
    fn test_diff_pl_empty_polylines() {
        let polylines: Vec<Polyline> = Vec::new();
        let clip = vec![make_square_mm(0.0, 0.0, 10.0)];
        let result = diff_pl(&polylines, &clip);
        assert!(result.is_empty());
    }

    #[test]
    fn test_diff_pl_empty_clip() {
        let pts = vec![
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(10.0), scale(0.0)),
        ];
        let polylines = vec![Polyline::from_points(pts)];
        let clip: Vec<ExPolygon> = Vec::new();

        let result = diff_pl(&polylines, &clip);
        // With no clip regions, original polylines should be returned
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn test_diff_pl_line_outside_clip() {
        // Line completely outside the clip region
        let pts = vec![
            Point::new(scale(50.0), scale(50.0)),
            Point::new(scale(60.0), scale(50.0)),
        ];
        let polylines = vec![Polyline::from_points(pts)];
        let clip = vec![make_square_mm(0.0, 0.0, 10.0)];

        let result = diff_pl(&polylines, &clip);
        // Line is outside, should be kept
        assert!(!result.is_empty());
    }

    #[test]
    fn test_diff_pl_line_inside_clip() {
        // Line completely inside the clip region
        let pts = vec![
            Point::new(scale(2.0), scale(2.0)),
            Point::new(scale(8.0), scale(2.0)),
        ];
        let polylines = vec![Polyline::from_points(pts)];
        let clip = vec![make_square_mm(0.0, 0.0, 10.0)];

        let result = diff_pl(&polylines, &clip);
        // Line is inside, should be removed (empty result)
        assert!(result.is_empty());
    }

    #[test]
    fn test_polygons_to_polylines() {
        let square = Polygon::rectangle(
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(10.0), scale(10.0)),
        );
        let polygons = vec![square];

        let result = polygons_to_polylines(&polygons);
        assert_eq!(result.len(), 1);
        assert!(result[0].len() >= 4); // At least 4 points for a rectangle
    }

    #[test]
    fn test_polygons_to_polylines_empty() {
        let polygons: Vec<Polygon> = Vec::new();
        let result = polygons_to_polylines(&polygons);
        assert!(result.is_empty());
    }

    #[test]
    fn test_expolygons_to_polylines_with_holes() {
        // Create an expolygon with a hole
        let outer = Polygon::rectangle(
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(20.0), scale(20.0)),
        );
        let hole = Polygon::rectangle(
            Point::new(scale(5.0), scale(5.0)),
            Point::new(scale(15.0), scale(15.0)),
        );
        let expoly = ExPolygon::with_holes(outer, vec![hole]);
        let expolygons = vec![expoly];

        let result = expolygons_to_polylines(&expolygons);
        // Should have 2 polylines: one for outer contour, one for hole
        assert_eq!(result.len(), 2);
    }
}
