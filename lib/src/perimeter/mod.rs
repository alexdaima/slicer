//! Perimeter generation module.
//!
//! This module handles the generation of perimeters (walls) for each layer,
//! including support for multi-wall generation using polygon offset operations.
//!
//! # Overview
//!
//! Perimeters are the outlines that define the shape of each layer. They are
//! generated from the slice contours by offsetting inward multiple times:
//!
//! - Outer perimeters (external walls) - printed first or last depending on settings
//! - Inner perimeters (internal walls) - printed between outer and infill
//!
//! # Algorithm
//!
//! 1. Start with the slice ExPolygons
//! 2. Offset inward by half the extrusion width to get the outer perimeter centerline
//! 3. Offset inward by the full extrusion width for each subsequent perimeter
//! 4. Continue until the desired number of perimeters is reached or the polygon disappears
//!
//! # Variable-Width Perimeters (Arachne)
//!
//! The `arachne` submodule provides variable-width perimeter generation that
//! adapts the extrusion width based on local geometry. This improves print
//! quality for thin walls and narrow features.
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/PerimeterGenerator.cpp`
//! - `src/libslic3r/Arachne/` (variable-width perimeters)

pub mod arachne;
pub mod fuzzy_skin;

use crate::clipper::{difference, extract_centerlines, grow, offset2, shrink, OffsetJoinType};
use crate::flow::Flow;
use crate::geometry::{ExPolygon, ExPolygons, Polygon, Polyline};
use crate::CoordF;

/// Inset overlap tolerance for gap detection (same as BambuStudio).
const INSET_OVERLAP_TOLERANCE: CoordF = 0.15;

/// Configuration for perimeter generation.
#[derive(Debug, Clone)]
pub struct PerimeterConfig {
    /// Number of perimeter loops to generate.
    pub perimeter_count: usize,

    /// Extrusion width for perimeters (mm).
    pub perimeter_extrusion_width: CoordF,

    /// Extrusion width for external (outer) perimeters (mm).
    pub external_perimeter_extrusion_width: CoordF,

    /// Spacing between internal perimeter centerlines (mm).
    /// This is typically less than width to account for overlap.
    /// Calculated as: width - height × (1 - π/4)
    pub perimeter_spacing: CoordF,

    /// Spacing for external perimeters (mm).
    pub external_perimeter_spacing: CoordF,

    /// Spacing between external and first internal perimeter (mm).
    /// This is the average of external and internal spacing.
    pub external_to_internal_spacing: CoordF,

    /// Layer height (mm) - needed for spacing calculations.
    pub layer_height: CoordF,

    /// Whether to print external perimeters first (outside-in).
    pub external_perimeters_first: bool,

    /// Minimum area for a perimeter loop to be kept (mm²).
    pub min_perimeter_area: CoordF,

    /// Gap fill threshold - fill gaps smaller than this (mm).
    pub gap_fill_threshold: CoordF,

    /// Whether to detect and handle thin walls.
    pub thin_walls: bool,

    /// Join type for perimeter offset corners.
    pub join_type: OffsetJoinType,
}

impl Default for PerimeterConfig {
    fn default() -> Self {
        Self::new(0.45, 0.45, 0.2, 3)
    }
}

impl PerimeterConfig {
    /// Create a new perimeter configuration with proper spacing calculations.
    ///
    /// # Arguments
    /// * `perimeter_width` - Width of internal perimeters (mm)
    /// * `external_width` - Width of external perimeters (mm)
    /// * `layer_height` - Layer height (mm)
    /// * `perimeter_count` - Number of perimeters to generate
    pub fn new(
        perimeter_width: CoordF,
        external_width: CoordF,
        layer_height: CoordF,
        perimeter_count: usize,
    ) -> Self {
        // Calculate spacing using the Flow module's formula:
        // spacing = width - height × (1 - π/4)
        let perimeter_spacing =
            Flow::rounded_rectangle_extrusion_spacing(perimeter_width, layer_height)
                .unwrap_or(perimeter_width * 0.9);

        let external_perimeter_spacing =
            Flow::rounded_rectangle_extrusion_spacing(external_width, layer_height)
                .unwrap_or(external_width * 0.9);

        // Spacing between external and first internal perimeter
        // BambuStudio uses: 0.5 × (ext_spacing + internal_spacing)
        let external_to_internal_spacing = 0.5 * (external_perimeter_spacing + perimeter_spacing);

        Self {
            perimeter_count,
            perimeter_extrusion_width: perimeter_width,
            external_perimeter_extrusion_width: external_width,
            perimeter_spacing,
            external_perimeter_spacing,
            external_to_internal_spacing,
            layer_height,
            external_perimeters_first: false,
            min_perimeter_area: 0.01, // 0.01 mm²
            gap_fill_threshold: 0.0,  // Disabled by default
            thin_walls: false,        // Disabled by default
            join_type: OffsetJoinType::Miter,
        }
    }

    /// Create a configuration with custom spacing values.
    pub fn with_spacing(mut self, internal_spacing: CoordF, external_spacing: CoordF) -> Self {
        self.perimeter_spacing = internal_spacing;
        self.external_perimeter_spacing = external_spacing;
        self.external_to_internal_spacing = 0.5 * (external_spacing + internal_spacing);
        self
    }
}

/// A single perimeter loop with associated metadata.
#[derive(Debug, Clone)]
pub struct PerimeterLoop {
    /// The polygon representing the perimeter centerline.
    pub polygon: Polygon,

    /// Whether this is an external (outer) perimeter.
    pub is_external: bool,

    /// Whether this is a contour (outer boundary) or hole (inner boundary).
    pub is_contour: bool,

    /// The perimeter index (0 = outermost, increasing inward).
    pub perimeter_index: usize,

    /// Extrusion width for this perimeter (mm).
    pub extrusion_width: CoordF,

    /// Depth/nesting level (for ordering).
    pub depth: usize,
}

impl PerimeterLoop {
    /// Create a new perimeter loop.
    pub fn new(
        polygon: Polygon,
        is_external: bool,
        is_contour: bool,
        perimeter_index: usize,
        extrusion_width: CoordF,
    ) -> Self {
        Self {
            polygon,
            is_external,
            is_contour,
            perimeter_index,
            extrusion_width,
            depth: 0,
        }
    }

    /// Get the perimeter length in mm.
    pub fn length_mm(&self) -> CoordF {
        // perimeter() already returns unscaled CoordF in mm
        self.polygon.perimeter() / crate::SCALING_FACTOR
    }

    /// Convert to a polyline (open path) by splitting at the best seam point.
    pub fn to_polyline(&self) -> Polyline {
        self.polygon.to_polyline()
    }

    /// Convert to a closed polyline.
    pub fn to_closed_polyline(&self) -> Polyline {
        self.polygon.to_closed_polyline()
    }
}

/// Result of perimeter generation for a single region.
#[derive(Debug, Clone, Default)]
pub struct PerimeterResult {
    /// Generated perimeter loops, ordered for printing.
    pub perimeters: Vec<PerimeterLoop>,

    /// The area remaining after all perimeters (for infill).
    pub infill_area: ExPolygons,

    /// Thin fill paths (areas too narrow for perimeters).
    pub thin_fills: Vec<Polyline>,

    /// Gap fill paths (small gaps between perimeters).
    pub gap_fills: Vec<Polyline>,
}

impl PerimeterResult {
    /// Create a new empty result.
    pub fn new() -> Self {
        Self::default()
    }

    /// Check if any perimeters were generated.
    pub fn has_perimeters(&self) -> bool {
        !self.perimeters.is_empty()
    }

    /// Get the total number of perimeter loops.
    pub fn perimeter_count(&self) -> usize {
        self.perimeters.len()
    }

    /// Get external perimeters only.
    pub fn external_perimeters(&self) -> impl Iterator<Item = &PerimeterLoop> {
        self.perimeters.iter().filter(|p| p.is_external)
    }

    /// Get internal perimeters only.
    pub fn internal_perimeters(&self) -> impl Iterator<Item = &PerimeterLoop> {
        self.perimeters.iter().filter(|p| !p.is_external)
    }

    /// Get perimeters ordered for outside-in printing.
    pub fn ordered_outside_in(&self) -> Vec<&PerimeterLoop> {
        let mut result: Vec<_> = self.perimeters.iter().collect();
        result.sort_by_key(|p| p.perimeter_index);
        result
    }

    /// Get perimeters ordered for inside-out printing.
    pub fn ordered_inside_out(&self) -> Vec<&PerimeterLoop> {
        let mut result: Vec<_> = self.perimeters.iter().collect();
        result.sort_by_key(|p| std::cmp::Reverse(p.perimeter_index));
        result
    }

    /// Get the total perimeter length in mm.
    pub fn total_length_mm(&self) -> CoordF {
        self.perimeters.iter().map(|p| p.length_mm()).sum()
    }
}

/// Perimeter generator.
///
/// Generates perimeter loops from slice ExPolygons using polygon offset operations.
#[derive(Debug, Clone)]
pub struct PerimeterGenerator {
    config: PerimeterConfig,
}

impl PerimeterGenerator {
    /// Create a new perimeter generator with the given configuration.
    pub fn new(config: PerimeterConfig) -> Self {
        Self { config }
    }

    /// Create a perimeter generator with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(PerimeterConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &PerimeterConfig {
        &self.config
    }

    /// Get mutable access to the configuration.
    pub fn config_mut(&mut self) -> &mut PerimeterConfig {
        &mut self.config
    }

    /// Generate perimeters for the given slice ExPolygons.
    ///
    /// # Arguments
    /// * `slices` - The slice ExPolygons to generate perimeters for
    ///
    /// # Returns
    /// A PerimeterResult containing the generated perimeters and infill area.
    pub fn generate(&self, slices: &[ExPolygon]) -> PerimeterResult {
        let mut result = PerimeterResult::new();

        if slices.is_empty() || self.config.perimeter_count == 0 {
            result.infill_area = slices.to_vec();
            return result;
        }

        // Track all perimeters by level for ordering
        let mut perimeter_levels: Vec<Vec<PerimeterLoop>> =
            vec![Vec::new(); self.config.perimeter_count];

        // Track accumulated gaps for gap fill
        let mut accumulated_gaps: ExPolygons = Vec::new();

        // Current working area (starts as the full slice)
        // Sort slices deterministically by bounding box for reproducible results
        let mut current_area = slices.to_vec();
        Self::sort_expolygons_deterministic(&mut current_area);

        // Whether to detect gaps (enabled when gap_fill_threshold > 0)
        let detect_gap_fill = self.config.gap_fill_threshold > 0.0;

        // Generate each perimeter level
        for perimeter_idx in 0..self.config.perimeter_count {
            let is_external = perimeter_idx == 0;

            // Calculate the extrusion width for this perimeter
            let extrusion_width = if is_external {
                self.config.external_perimeter_extrusion_width
            } else {
                self.config.perimeter_extrusion_width
            };

            // Calculate the offset distance for this perimeter
            // BambuStudio uses different calculations for each perimeter level:
            // - External (i=0): offset by half external width
            // - First internal (i=1): offset by external_to_internal_spacing
            // - Subsequent internal (i>1): offset by perimeter_spacing
            let offset_distance = if perimeter_idx == 0 {
                // External perimeter: offset by half width to get centerline at edge
                self.config.external_perimeter_extrusion_width / 2.0
            } else if perimeter_idx == 1 {
                // First internal perimeter: use spacing between external and internal
                self.config.external_to_internal_spacing
            } else {
                // Subsequent internal perimeters: use internal spacing
                self.config.perimeter_spacing
            };

            // Calculate spacing for gap detection
            let min_spacing = if is_external {
                self.config.external_perimeter_spacing
            } else {
                self.config.perimeter_spacing
            };

            // Use offset2 for internal perimeters to enable proper gap detection
            // This removes narrow protrusions that would cause gaps
            let perimeter_area = if perimeter_idx == 0 || !detect_gap_fill {
                // External perimeter or gap fill disabled: simple shrink
                shrink(&current_area, offset_distance, self.config.join_type)
            } else {
                // Internal perimeters with gap fill: use offset2 to clean up
                // shrink by (distance + min_spacing/2 - epsilon), then grow by (min_spacing/2 - epsilon)
                let shrink_amount = offset_distance + min_spacing / 2.0 - 0.001;
                let grow_amount = min_spacing / 2.0 - 0.001;
                offset2(
                    &current_area,
                    shrink_amount,
                    grow_amount,
                    self.config.join_type,
                )
            };

            // Detect gaps between previous and current perimeter levels
            if detect_gap_fill && perimeter_idx > 0 && !perimeter_area.is_empty() {
                // Gap detection: find regions in current_area that are too narrow
                // to fit a perimeter but wide enough for gap fill
                //
                // BambuStudio algorithm:
                // gaps = diff(offset(last, -0.5*distance), offset(offsets, 0.5*distance + safety))
                let gap_outer = shrink(&current_area, 0.5 * offset_distance, self.config.join_type);
                let gap_inner = grow(
                    &perimeter_area,
                    0.5 * offset_distance + 0.001,
                    self.config.join_type,
                );
                let gaps = difference(&gap_outer, &gap_inner);

                if !gaps.is_empty() {
                    accumulated_gaps.extend(gaps);
                }
            }

            if perimeter_area.is_empty() {
                // No more room for perimeters
                break;
            }

            // Extract perimeter loops from the offset result
            for expoly in &perimeter_area {
                // Add contour as a perimeter
                if !expoly.contour.is_empty() && self.is_valid_perimeter(&expoly.contour) {
                    let loop_item = PerimeterLoop::new(
                        expoly.contour.clone(),
                        is_external,
                        true, // is_contour
                        perimeter_idx,
                        extrusion_width,
                    );
                    perimeter_levels[perimeter_idx].push(loop_item);
                }

                // Add holes as perimeters (they're inner boundaries)
                // Sort holes deterministically
                let mut sorted_holes = expoly.holes.clone();
                sorted_holes.sort_by(|a, b| Self::compare_polygons_deterministic(a, b));

                for hole in &sorted_holes {
                    if !hole.is_empty() && self.is_valid_perimeter(hole) {
                        let loop_item = PerimeterLoop::new(
                            hole.clone(),
                            is_external,
                            false, // is_contour (it's a hole)
                            perimeter_idx,
                            extrusion_width,
                        );
                        perimeter_levels[perimeter_idx].push(loop_item);
                    }
                }
            }

            // Update current area for next iteration
            current_area = perimeter_area;
        }

        // The remaining area after all perimeters is the infill area
        // We need to shrink once more by half the inner perimeter width
        let inner_offset = self.config.perimeter_extrusion_width / 2.0;
        result.infill_area = shrink(&current_area, inner_offset, self.config.join_type);

        // Process accumulated gaps into gap fills
        if detect_gap_fill && !accumulated_gaps.is_empty() {
            result.gap_fills = self.process_gaps(&accumulated_gaps);
        }

        // Order perimeters for printing
        // Default is inside-out (external perimeters last) unless configured otherwise
        // Sort each level deterministically before adding
        if self.config.external_perimeters_first {
            // Outside-in: start from perimeter 0 (external)
            for mut level in perimeter_levels {
                Self::sort_perimeter_loops_deterministic(&mut level);
                result.perimeters.extend(level);
            }
        } else {
            // Inside-out: start from innermost perimeter
            for mut level in perimeter_levels.into_iter().rev() {
                Self::sort_perimeter_loops_deterministic(&mut level);
                result.perimeters.extend(level);
            }
        }

        result
    }

    /// Generate perimeters for a single ExPolygon.
    pub fn generate_single(&self, expoly: &ExPolygon) -> PerimeterResult {
        self.generate(&[expoly.clone()][..])
    }

    /// Process detected gaps into gap fill polylines.
    ///
    /// This implements BambuStudio's gap fill algorithm:
    /// 1. Collapse gaps using morphological opening
    /// 2. Remove regions that are too small or too large
    /// 3. Extract centerlines using medial axis approximation
    fn process_gaps(&self, gaps: &ExPolygons) -> Vec<Polyline> {
        use crate::clipper::opening;

        if gaps.is_empty() {
            return vec![];
        }

        let perimeter_width = self.config.perimeter_extrusion_width;
        let perimeter_spacing = perimeter_width; // Approximation

        // BambuStudio gap fill parameters
        let min_width = 0.2 * perimeter_width * (1.0 - INSET_OVERLAP_TOLERANCE);
        let max_width = 2.0 * perimeter_spacing;

        // Collapse gaps: opening removes regions narrower than min_width
        let opened = opening(gaps, min_width / 2.0, self.config.join_type);

        if opened.is_empty() {
            return vec![];
        }

        // Remove regions that are too wide (they should be infill, not gap fill)
        // offset2 with -max/2, +max/2 removes regions wider than max_width
        let collapsed = offset2(
            &opened,
            max_width / 2.0,
            max_width / 2.0 + 0.00001, // Small safety offset
            self.config.join_type,
        );

        // Keep only the difference (the narrow parts)
        let gap_regions = difference(&opened, &collapsed);

        if gap_regions.is_empty() {
            // All gaps were too wide, use the opened result directly
            // but only if within threshold
            let filtered: ExPolygons = opened
                .into_iter()
                .filter(|g| {
                    // Rough width estimate: area / perimeter * 2
                    let perim = g.contour.perimeter();
                    if perim > 0.0 {
                        let width_estimate = (g.area().abs() * 2.0) / perim;
                        width_estimate <= max_width && width_estimate >= min_width
                    } else {
                        false
                    }
                })
                .collect();

            if filtered.is_empty() {
                return vec![];
            }

            return extract_centerlines(&filtered, perimeter_width, self.config.join_type);
        }

        // Extract centerlines from gap regions
        // Use an average expected width for centerline extraction
        let avg_width = (min_width + max_width) / 2.0;
        let mut centerlines = extract_centerlines(&gap_regions, avg_width, self.config.join_type);

        // Filter out very short gap fills
        let min_length = self.config.gap_fill_threshold;
        centerlines.retain(|pl| pl.length() >= crate::scale(min_length) as f64);

        centerlines
    }

    /// Sort ExPolygons deterministically by bounding box (min X, then min Y, then area).
    fn sort_expolygons_deterministic(expolys: &mut [ExPolygon]) {
        expolys.sort_by(|a, b| {
            let bb_a = a.bounding_box();
            let bb_b = b.bounding_box();

            // Compare by min X first
            match bb_a.min.x.cmp(&bb_b.min.x) {
                std::cmp::Ordering::Equal => {}
                other => return other,
            }

            // Then by min Y
            match bb_a.min.y.cmp(&bb_b.min.y) {
                std::cmp::Ordering::Equal => {}
                other => return other,
            }

            // Finally by area (larger first for stability)
            let area_a = a.area().abs();
            let area_b = b.area().abs();
            area_b
                .partial_cmp(&area_a)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    /// Compare two polygons deterministically for sorting.
    fn compare_polygons_deterministic(a: &Polygon, b: &Polygon) -> std::cmp::Ordering {
        let bb_a = a.bounding_box();
        let bb_b = b.bounding_box();

        // Compare by min X first
        match bb_a.min.x.cmp(&bb_b.min.x) {
            std::cmp::Ordering::Equal => {}
            other => return other,
        }

        // Then by min Y
        match bb_a.min.y.cmp(&bb_b.min.y) {
            std::cmp::Ordering::Equal => {}
            other => return other,
        }

        // Finally by area
        let area_a = a.area().abs();
        let area_b = b.area().abs();
        area_b
            .partial_cmp(&area_a)
            .unwrap_or(std::cmp::Ordering::Equal)
    }

    /// Sort perimeter loops deterministically by bounding box.
    fn sort_perimeter_loops_deterministic(loops: &mut [PerimeterLoop]) {
        loops.sort_by(|a, b| Self::compare_polygons_deterministic(&a.polygon, &b.polygon));
    }

    /// Check if a polygon is valid for use as a perimeter.
    fn is_valid_perimeter(&self, polygon: &Polygon) -> bool {
        if polygon.len() < 3 {
            return false;
        }

        // Check minimum area
        // area() returns scaled area (scaled_units²), convert to mm²
        let area_mm2 = polygon.area() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        if area_mm2.abs() < self.config.min_perimeter_area {
            return false;
        }

        true
    }

    /// Calculate the inset distance for getting to the infill boundary.
    ///
    /// This is the total distance from the slice edge to the inner infill boundary.
    pub fn total_inset_distance(&self) -> CoordF {
        if self.config.perimeter_count == 0 {
            return 0.0;
        }

        // First perimeter: half external width
        let mut total = self.config.external_perimeter_extrusion_width / 2.0;

        // Inner perimeters: full width each
        if self.config.perimeter_count > 1 {
            total +=
                self.config.perimeter_extrusion_width * (self.config.perimeter_count - 1) as CoordF;
        }

        // Half width on the inside
        total += self.config.perimeter_extrusion_width / 2.0;

        total
    }
}

impl Default for PerimeterGenerator {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Compute the infill area for given slices and perimeter configuration.
///
/// This is a convenience function that generates perimeters and returns only the infill area.
pub fn compute_infill_area(slices: &[ExPolygon], config: &PerimeterConfig) -> ExPolygons {
    let generator = PerimeterGenerator::new(config.clone());
    let result = generator.generate(slices);
    result.infill_area
}

/// Generate perimeters with default configuration.
pub fn generate_perimeters(slices: &[ExPolygon]) -> PerimeterResult {
    PerimeterGenerator::with_defaults().generate(slices)
}

/// Generate perimeters with custom parameters.
pub fn generate_perimeters_with(
    slices: &[ExPolygon],
    perimeter_count: usize,
    extrusion_width: CoordF,
) -> PerimeterResult {
    let config = PerimeterConfig {
        perimeter_count,
        perimeter_extrusion_width: extrusion_width,
        external_perimeter_extrusion_width: extrusion_width,
        ..Default::default()
    };
    PerimeterGenerator::new(config).generate(slices)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    fn make_square_mm(x: f64, y: f64, size: f64) -> ExPolygon {
        let poly = Polygon::rectangle(
            Point::new(crate::scale(x), crate::scale(y)),
            Point::new(crate::scale(x + size), crate::scale(y + size)),
        );
        poly.into()
    }

    fn make_square_with_hole_mm(
        x: f64,
        y: f64,
        outer_size: f64,
        hole_offset: f64,
        hole_size: f64,
    ) -> ExPolygon {
        let outer = Polygon::rectangle(
            Point::new(crate::scale(x), crate::scale(y)),
            Point::new(crate::scale(x + outer_size), crate::scale(y + outer_size)),
        );
        let inner = Polygon::rectangle(
            Point::new(crate::scale(x + hole_offset), crate::scale(y + hole_offset)),
            Point::new(
                crate::scale(x + hole_offset + hole_size),
                crate::scale(y + hole_offset + hole_size),
            ),
        );
        ExPolygon::with_holes(outer, vec![inner])
    }

    #[test]
    fn test_perimeter_config_default() {
        let config = PerimeterConfig::default();
        assert_eq!(config.perimeter_count, 3);
        assert!((config.perimeter_extrusion_width - 0.45).abs() < 1e-6);
        // Check that spacing is properly calculated (less than width)
        assert!(config.perimeter_spacing < config.perimeter_extrusion_width);
        assert!(config.perimeter_spacing > 0.0);
    }

    #[test]
    fn test_perimeter_config_spacing() {
        // Test that spacing is calculated correctly using Flow formula
        let config = PerimeterConfig::new(0.45, 0.45, 0.2, 3);

        // spacing = width - height × (1 - π/4) ≈ 0.45 - 0.2 × 0.2146 ≈ 0.407
        let expected_spacing = 0.45 - 0.2 * (1.0 - 0.25 * std::f64::consts::PI);
        assert!(
            (config.perimeter_spacing - expected_spacing).abs() < 1e-6,
            "Expected spacing ~{:.4}, got {:.4}",
            expected_spacing,
            config.perimeter_spacing
        );

        // External to internal spacing should be average
        let expected_ext_to_int =
            0.5 * (config.external_perimeter_spacing + config.perimeter_spacing);
        assert!(
            (config.external_to_internal_spacing - expected_ext_to_int).abs() < 1e-6,
            "Expected ext_to_int spacing ~{:.4}, got {:.4}",
            expected_ext_to_int,
            config.external_to_internal_spacing
        );
    }

    #[test]
    fn test_generator_simple_square() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = PerimeterConfig {
            perimeter_count: 2,
            perimeter_extrusion_width: 0.5,
            external_perimeter_extrusion_width: 0.5,
            ..Default::default()
        };

        let generator = PerimeterGenerator::new(config);
        let result = generator.generate(&[square]);

        // Should have perimeters
        assert!(result.has_perimeters());

        // Should have external perimeters
        assert!(result.external_perimeters().count() > 0);

        // Should have infill area
        assert!(!result.infill_area.is_empty());

        println!("Generated {} perimeters", result.perimeter_count());
        println!("Total perimeter length: {:.2} mm", result.total_length_mm());
    }

    #[test]
    fn test_generator_with_hole() {
        let expoly = make_square_with_hole_mm(0.0, 0.0, 30.0, 10.0, 10.0);
        let config = PerimeterConfig {
            perimeter_count: 3,
            perimeter_extrusion_width: 0.45,
            external_perimeter_extrusion_width: 0.45,
            ..Default::default()
        };

        let generator = PerimeterGenerator::new(config);
        let result = generator.generate(&[expoly]);

        // Should have perimeters for both outer contour and hole
        assert!(result.has_perimeters());
        assert!(result.perimeter_count() > 3); // Multiple perimeters for contour + hole

        // Should have infill area
        assert!(!result.infill_area.is_empty());
    }

    #[test]
    fn test_generator_too_small() {
        // 1mm x 1mm square with 0.5mm perimeter width and 3 perimeters
        // Total inset would be about 1.5mm, so this should shrink to nothing
        let tiny = make_square_mm(0.0, 0.0, 1.0);
        let config = PerimeterConfig {
            perimeter_count: 3,
            perimeter_extrusion_width: 0.5,
            external_perimeter_extrusion_width: 0.5,
            ..Default::default()
        };

        let generator = PerimeterGenerator::new(config);
        let result = generator.generate(&[tiny]);

        // Should still generate some perimeters (at least the first one)
        // but infill area will likely be empty
        println!(
            "Tiny square: {} perimeters, {} infill areas",
            result.perimeter_count(),
            result.infill_area.len()
        );
    }

    #[test]
    fn test_generator_zero_perimeters() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = PerimeterConfig {
            perimeter_count: 0,
            ..Default::default()
        };

        let generator = PerimeterGenerator::new(config);
        let result = generator.generate(&[square.clone()]);

        // No perimeters
        assert!(!result.has_perimeters());

        // Infill area should be the original slice
        assert!(!result.infill_area.is_empty());
    }

    #[test]
    fn test_ordering_outside_in() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = PerimeterConfig {
            perimeter_count: 3,
            external_perimeters_first: true,
            ..Default::default()
        };

        let generator = PerimeterGenerator::new(config);
        let result = generator.generate(&[square]);

        let ordered = result.ordered_outside_in();
        assert!(!ordered.is_empty());

        // First should be external (index 0)
        assert!(ordered[0].is_external);
        assert_eq!(ordered[0].perimeter_index, 0);
    }

    #[test]
    fn test_ordering_inside_out() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = PerimeterConfig {
            perimeter_count: 3,
            external_perimeters_first: false,
            ..Default::default()
        };

        let generator = PerimeterGenerator::new(config);
        let result = generator.generate(&[square]);

        let ordered = result.ordered_inside_out();
        assert!(!ordered.is_empty());

        // First should be innermost (highest index)
        // Last should be external
        assert!(ordered.last().unwrap().is_external);
    }

    #[test]
    fn test_total_inset_distance() {
        let config = PerimeterConfig {
            perimeter_count: 3,
            perimeter_extrusion_width: 0.5,
            external_perimeter_extrusion_width: 0.5,
            ..Default::default()
        };

        let generator = PerimeterGenerator::new(config);
        let inset = generator.total_inset_distance();

        // Expected: 0.25 (half external) + 0.5 + 0.5 (2 inner) + 0.25 (half inner) = 1.5
        assert!((inset - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_convenience_functions() {
        let square = make_square_mm(0.0, 0.0, 20.0);

        // Test generate_perimeters
        let result = generate_perimeters(&[square.clone()]);
        assert!(result.has_perimeters());

        // Test generate_perimeters_with
        let result2 = generate_perimeters_with(&[square.clone()], 2, 0.4);
        assert!(result2.has_perimeters());

        // Test compute_infill_area
        let config = PerimeterConfig::default();
        let infill = compute_infill_area(&[square], &config);
        assert!(!infill.is_empty());
    }

    #[test]
    fn test_perimeter_loop_to_polyline() {
        let square = Polygon::rectangle(
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(10.0), crate::scale(10.0)),
        );

        let loop_item = PerimeterLoop::new(square.clone(), true, true, 0, 0.45);

        let polyline = loop_item.to_polyline();
        assert!(!polyline.is_empty());
        assert_eq!(polyline.len(), square.len());

        let closed = loop_item.to_closed_polyline();
        assert!(!closed.is_empty());
        assert_eq!(closed.len(), square.len() + 1); // Closed has extra point
    }

    #[test]
    fn test_multiple_regions() {
        // Two separate squares
        let square1 = make_square_mm(0.0, 0.0, 15.0);
        let square2 = make_square_mm(20.0, 0.0, 15.0);

        let generator = PerimeterGenerator::with_defaults();
        let result = generator.generate(&[square1, square2]);

        // Should have perimeters for both regions
        assert!(result.perimeter_count() >= 6); // At least 3 per region

        // Should have infill areas for both
        assert!(result.infill_area.len() >= 2);
    }

    #[test]
    fn test_gap_fill_detection_disabled_by_default() {
        // When gap_fill_threshold is 0, gap fills should not be generated
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = PerimeterConfig {
            perimeter_count: 3,
            perimeter_extrusion_width: 0.45,
            external_perimeter_extrusion_width: 0.45,
            gap_fill_threshold: 0.0, // Disabled
            ..Default::default()
        };

        let generator = PerimeterGenerator::new(config);
        let result = generator.generate(&[square]);

        // Gap fills should be empty when disabled
        assert!(result.gap_fills.is_empty());
    }

    #[test]
    fn test_gap_fill_detection_enabled() {
        // Create a narrow region that should generate gap fills
        // A thin rectangle that's too narrow for normal infill but suitable for gap fill
        let thin_rect = Polygon::from_points(vec![
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(20.0), crate::scale(0.0)),
            Point::new(crate::scale(20.0), crate::scale(1.5)), // 1.5mm tall - narrow
            Point::new(crate::scale(0.0), crate::scale(1.5)),
        ]);
        let expoly: ExPolygon = thin_rect.into();

        let config = PerimeterConfig {
            perimeter_count: 2,
            perimeter_extrusion_width: 0.45,
            external_perimeter_extrusion_width: 0.45,
            gap_fill_threshold: 0.2, // Enable gap fill
            ..Default::default()
        };

        let generator = PerimeterGenerator::new(config);
        let result = generator.generate(&[expoly]);

        // Should generate perimeters
        assert!(result.has_perimeters());
        println!(
            "Generated {} perimeters, {} gap fills",
            result.perimeter_count(),
            result.gap_fills.len()
        );
    }

    #[test]
    fn test_gap_fill_with_complex_shape() {
        // Create an L-shape that will have gaps at the corner
        let l_shape = Polygon::from_points(vec![
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(10.0), crate::scale(0.0)),
            Point::new(crate::scale(10.0), crate::scale(5.0)),
            Point::new(crate::scale(5.0), crate::scale(5.0)),
            Point::new(crate::scale(5.0), crate::scale(10.0)),
            Point::new(crate::scale(0.0), crate::scale(10.0)),
        ]);
        let expoly: ExPolygon = l_shape.into();

        let config = PerimeterConfig {
            perimeter_count: 3,
            perimeter_extrusion_width: 0.45,
            external_perimeter_extrusion_width: 0.45,
            gap_fill_threshold: 0.3,
            ..Default::default()
        };

        let generator = PerimeterGenerator::new(config);
        let result = generator.generate(&[expoly]);

        assert!(result.has_perimeters());
        // L-shapes often create gaps at corners during perimeter generation
        println!(
            "L-shape: {} perimeters, {} gap fills, {} infill regions",
            result.perimeter_count(),
            result.gap_fills.len(),
            result.infill_area.len()
        );
    }

    #[test]
    fn test_thin_fills_empty_by_default() {
        // Thin fills require thin_walls detection which is separate from gap fill
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = PerimeterConfig::default();

        let generator = PerimeterGenerator::new(config);
        let result = generator.generate(&[square]);

        // Thin fills aren't populated yet (requires medial axis implementation)
        assert!(result.thin_fills.is_empty());
    }
}
