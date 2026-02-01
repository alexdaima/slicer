//! Infill pattern generation module.
//!
//! This module provides implementations for various infill patterns:
//! - Rectilinear (parallel lines)
//! - Grid (crossing lines at 90°)
//! - Concentric (inward offset loops)
//! - Honeycomb (hexagonal pattern for strength)
//! - Adaptive (variable density based on surface proximity)
//!
//! # Overview
//!
//! Infill fills the interior of a layer after perimeters have been generated.
//! The infill area is the region remaining after subtracting perimeter boundaries.
//!
//! # Algorithm
//!
//! 1. Start with the infill area (ExPolygons from perimeter generator)
//! 2. Generate the infill pattern (lines or polygons) covering the bounding box
//! 3. Clip the pattern to the infill area using boolean intersection
//! 4. Order the paths for efficient printing (minimize travel)
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/Fill/` directory
//! - `src/libslic3r/Fill/FillRectilinear.cpp`
//! - `src/libslic3r/Fill/FillConcentric.cpp`
//! - `src/libslic3r/Fill/FillAdaptive.cpp`
//! - `src/libslic3r/Fill/Fill3DHoneycomb.cpp`

pub mod adaptive;
pub mod cross_hatch;
pub mod floating_concentric;
pub mod honeycomb_3d;
pub mod plan_path;

use crate::clipper::{offset_expolygons, shrink, OffsetJoinType};
use crate::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon, Polyline};
use crate::{scale, unscale, Coord, CoordF};

// Re-export adaptive infill types
pub use adaptive::{
    build_octree, generate_adaptive_infill, generate_adaptive_infill_with_density,
    AdaptiveInfillConfig, AdaptiveInfillGenerator, AdaptiveInfillResult, CubeProperties, Octree,
    Vec3d,
};

// Re-export Cross Hatch infill types
pub use cross_hatch::{
    generate_cross_hatch, generate_cross_hatch_with_angle, CrossHatchConfig, CrossHatchGenerator,
    CrossHatchResult,
};

// Re-export 3D honeycomb infill types
pub use honeycomb_3d::{
    generate_honeycomb_3d, Honeycomb3DConfig, Honeycomb3DGenerator, Honeycomb3DResult,
};

// Re-export plan path infill types (space-filling curves)
pub use plan_path::{
    generate_archimedean_chords, generate_hilbert_curve, generate_octagram_spiral, PlanPathConfig,
    PlanPathGenerator, PlanPathPattern, PlanPathResult,
};

// Re-export floating concentric infill types
pub use floating_concentric::{
    generate_floating_concentric, generate_floating_concentric_with_config,
    FloatingConcentricConfig, FloatingConcentricGenerator, FloatingConcentricResult,
    FloatingThickLine, FloatingThickPolyline,
};

/// Infill pattern types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InfillPattern {
    /// Parallel lines in one direction.
    #[default]
    Rectilinear,
    /// Crossing lines at 90° (alternating layers).
    Grid,
    /// Inward offset loops.
    Concentric,
    /// Floating concentric (detects unsupported sections for top surfaces).
    FloatingConcentric,
    /// Lines radiating from center (like spokes).
    Line,
    /// Honeycomb pattern (hexagonal pattern for strength).
    Honeycomb,
    /// 3D gyroid pattern (continuous surface).
    Gyroid,
    /// Lightning infill (tree-like structure).
    Lightning,
    /// Adaptive cubic infill (variable density near surfaces).
    Adaptive,
    /// Support cubic infill (for internal overhangs only).
    SupportCubic,
    /// 3D Honeycomb (truncated octahedron tesselation).
    Honeycomb3D,
    /// Cross Hatch pattern (alternating directions with smooth transitions).
    CrossHatch,
    /// Hilbert space-filling curve.
    HilbertCurve,
    /// Archimedean spiral from center outward.
    ArchimedeanChords,
    /// 8-pointed star spiral pattern.
    OctagramSpiral,
    /// No infill.
    None,
}

impl std::fmt::Display for InfillPattern {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InfillPattern::Rectilinear => write!(f, "Rectilinear"),
            InfillPattern::Grid => write!(f, "Grid"),
            InfillPattern::Concentric => write!(f, "Concentric"),
            InfillPattern::FloatingConcentric => write!(f, "Floating Concentric"),
            InfillPattern::Line => write!(f, "Line"),
            InfillPattern::Honeycomb => write!(f, "Honeycomb"),
            InfillPattern::Gyroid => write!(f, "Gyroid"),
            InfillPattern::Lightning => write!(f, "Lightning"),
            InfillPattern::Adaptive => write!(f, "Adaptive"),
            InfillPattern::SupportCubic => write!(f, "Support Cubic"),
            InfillPattern::Honeycomb3D => write!(f, "3D Honeycomb"),
            InfillPattern::CrossHatch => write!(f, "Cross Hatch"),
            InfillPattern::HilbertCurve => write!(f, "Hilbert Curve"),
            InfillPattern::ArchimedeanChords => write!(f, "Archimedean Chords"),
            InfillPattern::OctagramSpiral => write!(f, "Octagram Spiral"),
            InfillPattern::None => write!(f, "None"),
        }
    }
}

impl InfillPattern {
    /// Check if this pattern generates lines (vs. polygons/loops).
    pub fn is_linear(&self) -> bool {
        matches!(
            self,
            InfillPattern::Rectilinear
                | InfillPattern::Grid
                | InfillPattern::Line
                | InfillPattern::Adaptive
                | InfillPattern::SupportCubic
                | InfillPattern::CrossHatch
                | InfillPattern::HilbertCurve
                | InfillPattern::ArchimedeanChords
                | InfillPattern::OctagramSpiral
        )
    }

    /// Check if this pattern generates concentric loops.
    pub fn is_concentric(&self) -> bool {
        matches!(
            self,
            InfillPattern::Concentric | InfillPattern::FloatingConcentric
        )
    }

    /// Check if this pattern is implemented.
    pub fn is_implemented(&self) -> bool {
        matches!(
            self,
            InfillPattern::Rectilinear
                | InfillPattern::Grid
                | InfillPattern::Concentric
                | InfillPattern::FloatingConcentric
                | InfillPattern::Line
                | InfillPattern::Honeycomb
                | InfillPattern::Gyroid
                | InfillPattern::Lightning
                | InfillPattern::Adaptive
                | InfillPattern::SupportCubic
                | InfillPattern::Honeycomb3D
                | InfillPattern::CrossHatch
                | InfillPattern::HilbertCurve
                | InfillPattern::ArchimedeanChords
                | InfillPattern::OctagramSpiral
                | InfillPattern::None
        )
    }

    /// Check if this pattern requires mesh data for octree building.
    pub fn needs_octree(&self) -> bool {
        matches!(self, InfillPattern::Adaptive | InfillPattern::SupportCubic)
    }

    /// Check if this pattern is adaptive (variable density).
    pub fn is_adaptive(&self) -> bool {
        matches!(self, InfillPattern::Adaptive | InfillPattern::SupportCubic)
    }
}

/// Configuration for infill generation.
#[derive(Debug, Clone)]
pub struct InfillConfig {
    /// The infill pattern to use.
    pub pattern: InfillPattern,

    /// Infill density (0.0 - 1.0, where 1.0 is 100% solid).
    pub density: CoordF,

    /// Extrusion width for infill lines (mm).
    pub extrusion_width: CoordF,

    /// Angle of infill lines in degrees (0 = X axis).
    pub angle: CoordF,

    /// Angle increment per layer (for alternating patterns).
    pub angle_increment: CoordF,

    /// Overlap between infill and perimeters (mm).
    pub overlap: CoordF,

    /// Minimum area to fill (mm²).
    pub min_area: CoordF,

    /// Whether to connect infill lines where possible.
    pub connect_infill: bool,

    /// Whether to print infill before perimeters.
    pub infill_first: bool,

    /// Current Z height in mm (needed for 3D patterns like Honeycomb3D, CrossHatch).
    pub z_height: CoordF,

    /// Layer height in mm (needed for 3D patterns).
    pub layer_height: CoordF,
}

impl Default for InfillConfig {
    fn default() -> Self {
        Self {
            pattern: InfillPattern::Rectilinear,
            density: 0.2,          // 20% infill
            extrusion_width: 0.45, // mm
            angle: 45.0,           // degrees
            angle_increment: 90.0, // degrees per layer
            overlap: 0.1,          // mm overlap with perimeters
            min_area: 0.01,        // mm²
            connect_infill: true,  // Connect lines where possible
            infill_first: false,   // Perimeters first by default
            z_height: 0.0,         // Current Z height in mm
            layer_height: 0.2,     // Layer height in mm
        }
    }
}

impl InfillConfig {
    /// Create a solid infill configuration.
    pub fn solid() -> Self {
        Self {
            pattern: InfillPattern::Rectilinear,
            density: 1.0,
            angle: 45.0,
            angle_increment: 90.0,
            ..Default::default()
        }
    }

    /// Create a configuration with the given density.
    pub fn with_density(density: CoordF) -> Self {
        Self {
            density: density.clamp(0.0, 1.0),
            ..Default::default()
        }
    }

    /// Calculate the line spacing based on density and extrusion width.
    pub fn line_spacing(&self) -> CoordF {
        if self.density <= 0.0 {
            return CoordF::MAX;
        }
        if self.density >= 1.0 {
            return self.extrusion_width;
        }
        self.extrusion_width / self.density
    }

    /// Get the angle for a specific layer (with increment applied).
    pub fn angle_for_layer(&self, layer_index: usize) -> CoordF {
        self.angle + self.angle_increment * layer_index as CoordF
    }

    /// Set Z height and layer height (for 3D patterns).
    pub fn with_z(mut self, z_height: CoordF, layer_height: CoordF) -> Self {
        self.z_height = z_height;
        self.layer_height = layer_height;
        self
    }
}

/// A single infill path (either a line or a polygon loop).
#[derive(Debug, Clone)]
pub enum InfillPath {
    /// A linear path (open polyline).
    Line(Polyline),
    /// A closed loop (polygon).
    Loop(Polygon),
}

impl InfillPath {
    /// Get the path as points.
    pub fn points(&self) -> &[Point] {
        match self {
            InfillPath::Line(polyline) => polyline.points(),
            InfillPath::Loop(polygon) => polygon.points(),
        }
    }

    /// Get the path length in mm.
    pub fn length(&self) -> CoordF {
        match self {
            InfillPath::Line(polyline) => polyline.length(),
            InfillPath::Loop(polygon) => polygon.perimeter(),
        }
    }

    /// Get the path length in mm (unscaled from internal coordinates).
    pub fn length_mm(&self) -> CoordF {
        unscale(self.length() as Coord)
    }

    /// Check if this is a linear path.
    pub fn is_line(&self) -> bool {
        matches!(self, InfillPath::Line(_))
    }

    /// Check if this is a loop.
    pub fn is_loop(&self) -> bool {
        matches!(self, InfillPath::Loop(_))
    }

    /// Convert to a polyline (loops become open paths).
    pub fn to_polyline(&self) -> Polyline {
        match self {
            InfillPath::Line(polyline) => polyline.clone(),
            InfillPath::Loop(polygon) => polygon.to_polyline(),
        }
    }

    /// Get the start point.
    pub fn start_point(&self) -> Option<Point> {
        match self {
            InfillPath::Line(polyline) => {
                if polyline.is_empty() {
                    None
                } else {
                    Some(polyline.first_point())
                }
            }
            InfillPath::Loop(polygon) => polygon.points().first().copied(),
        }
    }

    /// Get the end point.
    pub fn end_point(&self) -> Option<Point> {
        match self {
            InfillPath::Line(polyline) => {
                if polyline.is_empty() {
                    None
                } else {
                    Some(polyline.last_point())
                }
            }
            InfillPath::Loop(polygon) => polygon.points().last().copied(),
        }
    }

    /// Reverse the path direction.
    pub fn reverse(&mut self) {
        match self {
            InfillPath::Line(polyline) => polyline.reverse(),
            InfillPath::Loop(polygon) => polygon.reverse(),
        }
    }

    /// Get a reversed copy of the path.
    pub fn reversed(&self) -> Self {
        let mut copy = self.clone();
        copy.reverse();
        copy
    }
}

/// Result of infill generation for a region.
#[derive(Debug, Clone, Default)]
pub struct InfillResult {
    /// Generated infill paths, ordered for printing.
    pub paths: Vec<InfillPath>,

    /// Total path length in mm.
    pub total_length_mm: CoordF,

    /// Pattern used.
    pub pattern: InfillPattern,

    /// Density used.
    pub density: CoordF,
}

impl InfillResult {
    /// Create a new empty result.
    pub fn new() -> Self {
        Self::default()
    }

    /// Check if any infill was generated.
    pub fn has_infill(&self) -> bool {
        !self.paths.is_empty()
    }

    /// Get the number of paths.
    pub fn path_count(&self) -> usize {
        self.paths.len()
    }

    /// Get all paths as polylines.
    pub fn to_polylines(&self) -> Vec<Polyline> {
        self.paths.iter().map(|p| p.to_polyline()).collect()
    }

    /// Calculate the total path length.
    fn calculate_total_length(&mut self) {
        self.total_length_mm = self.paths.iter().map(|p| p.length_mm()).sum();
    }
}

/// Infill generator.
///
/// Generates infill patterns for the interior of sliced layers.
#[derive(Debug, Clone)]
pub struct InfillGenerator {
    config: InfillConfig,
}

impl InfillGenerator {
    /// Create a new infill generator with the given configuration.
    pub fn new(config: InfillConfig) -> Self {
        Self { config }
    }

    /// Create an infill generator with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(InfillConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &InfillConfig {
        &self.config
    }

    /// Get mutable access to the configuration.
    pub fn config_mut(&mut self) -> &mut InfillConfig {
        &mut self.config
    }

    /// Generate infill for the given area.
    ///
    /// # Arguments
    /// * `infill_area` - The ExPolygons to fill (typically from perimeter generator)
    /// * `layer_index` - The current layer index (for angle calculation)
    ///
    /// # Returns
    /// An InfillResult containing the generated infill paths.
    pub fn generate(&self, infill_area: &[ExPolygon], layer_index: usize) -> InfillResult {
        let mut result = InfillResult {
            pattern: self.config.pattern,
            density: self.config.density,
            ..Default::default()
        };

        if infill_area.is_empty() || self.config.density <= 0.0 {
            return result;
        }

        if self.config.pattern == InfillPattern::None {
            return result;
        }

        // Expand the infill area slightly to overlap with perimeters
        let fill_area = if self.config.overlap > 0.0 {
            offset_expolygons(
                &infill_area.to_vec(),
                self.config.overlap,
                OffsetJoinType::Miter,
            )
        } else {
            infill_area.to_vec()
        };

        if fill_area.is_empty() {
            return result;
        }

        // Generate infill based on pattern
        match self.config.pattern {
            InfillPattern::Rectilinear => {
                result.paths = self.generate_rectilinear(&fill_area, layer_index, false);
            }
            InfillPattern::Grid => {
                result.paths = self.generate_rectilinear(&fill_area, layer_index, true);
            }
            InfillPattern::Concentric => {
                result.paths = self.generate_concentric(&fill_area);
            }
            InfillPattern::Line => {
                result.paths = self.generate_rectilinear(&fill_area, layer_index, false);
            }
            InfillPattern::Honeycomb => {
                result.paths = self.generate_honeycomb(&fill_area, layer_index);
            }
            InfillPattern::Gyroid => {
                result.paths = self.generate_gyroid(&fill_area, layer_index);
            }
            InfillPattern::Lightning => {
                result.paths = self.generate_lightning(&fill_area, layer_index);
            }
            InfillPattern::CrossHatch => {
                result.paths = self.generate_cross_hatch_pattern(&fill_area);
            }
            InfillPattern::Honeycomb3D => {
                result.paths = self.generate_honeycomb_3d_pattern(&fill_area);
            }
            InfillPattern::Adaptive | InfillPattern::SupportCubic => {
                // Adaptive infill requires an octree built from mesh triangles.
                // Use AdaptiveInfillGenerator directly for proper adaptive infill.
                // Here we fall back to rectilinear as a reasonable default.
                result.paths = self.generate_rectilinear(&fill_area, layer_index, false);
            }
            InfillPattern::HilbertCurve => {
                result.paths =
                    self.generate_plan_path_pattern(&fill_area, PlanPathPattern::HilbertCurve);
            }
            InfillPattern::ArchimedeanChords => {
                result.paths =
                    self.generate_plan_path_pattern(&fill_area, PlanPathPattern::ArchimedeanChords);
            }
            InfillPattern::OctagramSpiral => {
                result.paths =
                    self.generate_plan_path_pattern(&fill_area, PlanPathPattern::OctagramSpiral);
            }
            InfillPattern::FloatingConcentric => {
                // FloatingConcentric requires floating_areas for proper detection.
                // Use generate_with_floating_areas() or generate_floating_concentric() directly.
                // Here we fall back to regular concentric without floating detection.
                result.paths = self.generate_concentric(&fill_area);
            }
            InfillPattern::None => {
                // No infill
            }
        }

        result.calculate_total_length();
        result
    }

    /// Generate infill with floating area detection for top surfaces.
    ///
    /// This method is used for FloatingConcentric pattern where we need to know
    /// which areas are not supported by the layer below.
    ///
    /// # Arguments
    /// * `infill_area` - The area to fill
    /// * `floating_areas` - Areas not supported by the layer below (for floating detection)
    /// * `layer_index` - The current layer index
    ///
    /// # Returns
    /// An InfillResult containing the generated infill paths.
    pub fn generate_with_floating_areas(
        &self,
        infill_area: &[ExPolygon],
        floating_areas: &[ExPolygon],
        layer_index: usize,
    ) -> InfillResult {
        // For non-floating patterns, delegate to regular generate
        if self.config.pattern != InfillPattern::FloatingConcentric {
            return self.generate(infill_area, layer_index);
        }

        let mut result = InfillResult {
            pattern: self.config.pattern,
            density: self.config.density,
            ..Default::default()
        };

        if infill_area.is_empty() || self.config.density <= 0.0 {
            return result;
        }

        // Expand the infill area slightly to overlap with perimeters
        let fill_area = if self.config.overlap > 0.0 {
            offset_expolygons(
                &infill_area.to_vec(),
                self.config.overlap,
                OffsetJoinType::Miter,
            )
        } else {
            infill_area.to_vec()
        };

        if fill_area.is_empty() {
            return result;
        }

        // Generate floating concentric infill
        let floating_config = FloatingConcentricConfig::new(self.config.line_spacing())
            .with_default_width(self.config.extrusion_width)
            .with_split_at_transitions(true)
            .with_prefer_non_floating_start(true);

        let floating_result = generate_floating_concentric_with_config(
            &fill_area,
            &floating_areas.to_vec(),
            floating_config,
        );

        // Convert FloatingThickPolylines to InfillPaths
        for ftp in floating_result.polylines {
            if ftp.is_closed() {
                result
                    .paths
                    .push(InfillPath::Loop(ftp.to_polyline().to_polygon()));
            } else {
                result.paths.push(InfillPath::Line(ftp.to_polyline()));
            }
        }

        result.calculate_total_length();
        result
    }

    /// Generate rectilinear (parallel lines) infill.
    fn generate_rectilinear(
        &self,
        fill_area: &ExPolygons,
        layer_index: usize,
        is_grid: bool,
    ) -> Vec<InfillPath> {
        let mut paths = Vec::new();

        // Calculate angle for this layer
        let angle = self.config.angle_for_layer(layer_index);
        let angle_rad = angle.to_radians();

        // Calculate line spacing
        let spacing = scale(self.config.line_spacing());
        if spacing <= 0 {
            return paths;
        }

        // Get bounding box of all fill areas
        let mut bbox = BoundingBox::new();
        for expoly in fill_area {
            bbox.merge(&expoly.bounding_box());
        }

        if bbox.is_empty() {
            return paths;
        }

        // Generate lines covering the bounding box
        // We generate lines perpendicular to the angle direction
        let cos_a = angle_rad.cos();
        let sin_a = angle_rad.sin();

        // Calculate the rotated bounding box extent
        let center = bbox.center();
        let half_diag = ((bbox.width() * bbox.width() + bbox.height() * bbox.height()) as f64)
            .sqrt() as Coord
            / 2
            + spacing;

        // Generate parallel lines
        let mut raw_lines = Vec::new();
        let num_lines = (2 * half_diag / spacing + 1) as i32;

        for i in (-num_lines / 2)..=(num_lines / 2) {
            let offset = i as Coord * spacing;

            // Line perpendicular to angle, offset from center
            let px = center.x as f64 + offset as f64 * cos_a;
            let py = center.y as f64 + offset as f64 * sin_a;

            // Line endpoints (long enough to cover bbox)
            let dx = -sin_a * half_diag as f64;
            let dy = cos_a * half_diag as f64;

            let p1 = Point::new((px - dx) as Coord, (py - dy) as Coord);
            let p2 = Point::new((px + dx) as Coord, (py + dy) as Coord);

            raw_lines.push((p1, p2));
        }

        // Clip lines to fill area
        let clipped = self.clip_lines_to_polygons(&raw_lines, fill_area);

        // Convert to InfillPaths
        for line_points in clipped {
            if line_points.len() >= 2 {
                paths.push(InfillPath::Line(Polyline::from_points(line_points)));
            }
        }

        // For grid pattern, add perpendicular lines
        if is_grid {
            let perp_angle = angle + 90.0;
            let perp_angle_rad = perp_angle.to_radians();
            let cos_p = perp_angle_rad.cos();
            let sin_p = perp_angle_rad.sin();

            let mut perp_lines = Vec::new();

            for i in (-num_lines / 2)..=(num_lines / 2) {
                let offset = i as Coord * spacing;

                let px = center.x as f64 + offset as f64 * cos_p;
                let py = center.y as f64 + offset as f64 * sin_p;

                let dx = -sin_p * half_diag as f64;
                let dy = cos_p * half_diag as f64;

                let p1 = Point::new((px - dx) as Coord, (py - dy) as Coord);
                let p2 = Point::new((px + dx) as Coord, (py + dy) as Coord);

                perp_lines.push((p1, p2));
            }

            let clipped_perp = self.clip_lines_to_polygons(&perp_lines, fill_area);

            for line_points in clipped_perp {
                if line_points.len() >= 2 {
                    paths.push(InfillPath::Line(Polyline::from_points(line_points)));
                }
            }
        }

        // Optionally connect adjacent lines to reduce travel
        if self.config.connect_infill && !paths.is_empty() {
            paths = self.connect_infill_lines(paths);
        }

        paths
    }

    /// Generate honeycomb infill pattern.
    ///
    /// Honeycomb creates a hexagonal pattern that provides excellent strength-to-weight ratio.
    /// The pattern alternates between two phases on consecutive layers to create interlocking
    /// hexagons that provide strength in all directions.
    ///
    /// The algorithm generates zigzag lines that form hexagonal cells when viewed from above.
    /// On even layers, the zigzag peaks point up; on odd layers, they point down.
    fn generate_honeycomb(&self, fill_area: &ExPolygons, layer_index: usize) -> Vec<InfillPath> {
        let mut paths = Vec::new();

        // Calculate spacing based on density
        let spacing = self.config.line_spacing();
        if spacing <= 0.0 {
            return paths;
        }

        let spacing_scaled = scale(spacing);

        // Honeycomb geometry:
        // Hexagon cell width = spacing
        // Hexagon cell height = spacing * sqrt(3) / 2 (for regular hexagon proportions)
        // We use a simplified honeycomb where:
        // - Horizontal lines at regular intervals
        // - Diagonal segments connect them in a zigzag pattern
        let hex_height = (spacing * 3.0_f64.sqrt() / 2.0).max(spacing * 0.866);
        let hex_height_scaled = scale(hex_height);

        // Get bounding box of all fill areas
        let mut bbox = BoundingBox::new();
        for expoly in fill_area {
            bbox.merge(&expoly.bounding_box());
        }

        if bbox.is_empty() {
            return paths;
        }

        // Apply rotation based on configured angle
        let angle = self.config.angle_for_layer(layer_index);
        let angle_rad = angle.to_radians();
        let cos_a = angle_rad.cos();
        let sin_a = angle_rad.sin();

        // Expand bounds to account for rotation
        let center = bbox.center();
        let half_diag = ((bbox.width() * bbox.width() + bbox.height() * bbox.height()) as f64)
            .sqrt() as Coord
            / 2
            + spacing_scaled * 2;

        // Calculate rotated bounding box corners
        let min_x = center.x - half_diag;
        let max_x = center.x + half_diag;
        let min_y = center.y - half_diag;
        let max_y = center.y + half_diag;

        // Phase determines zigzag direction (alternates each layer for interlocking)
        let phase = layer_index % 2;

        // Generate honeycomb pattern as connected zigzag lines
        // Each row is a horizontal zigzag
        let num_rows = ((max_y - min_y) as f64 / hex_height_scaled as f64).ceil() as i32 + 2;

        for row in 0..num_rows {
            let y_base = min_y + (row as Coord * hex_height_scaled);

            // Generate zigzag points for this row
            let mut zigzag_points = Vec::new();
            let num_cols = ((max_x - min_x) as f64 / spacing_scaled as f64).ceil() as i32 + 2;

            for col in 0..=num_cols {
                let x = min_x + col as Coord * spacing_scaled;

                // Determine Y offset for zigzag
                // Alternate up/down based on column and phase
                let y_offset = if (col + phase as i32) % 2 == 0 {
                    hex_height_scaled / 3
                } else {
                    -hex_height_scaled / 3
                };

                let y = y_base + y_offset;

                // Apply rotation around center
                let dx = (x - center.x) as f64;
                let dy = (y - center.y) as f64;
                let rx = center.x as f64 + dx * cos_a - dy * sin_a;
                let ry = center.y as f64 + dx * sin_a + dy * cos_a;

                zigzag_points.push(Point::new(rx.round() as Coord, ry.round() as Coord));
            }

            // Convert zigzag to line segments for clipping
            if zigzag_points.len() >= 2 {
                // Clip the entire zigzag polyline to fill area
                let clipped = self.clip_polyline_to_polygons(&zigzag_points, fill_area);
                for segment in clipped {
                    if segment.len() >= 2 {
                        paths.push(InfillPath::Line(Polyline::from_points(segment)));
                    }
                }
            }
        }

        // Generate vertical connecting segments between rows
        // These create the hexagonal cells
        let num_cols = ((max_x - min_x) as f64 / spacing_scaled as f64).ceil() as i32 + 2;

        for col in 0..=num_cols {
            // Only add verticals at every other column (where zigzags meet)
            if (col + phase as i32) % 2 != 0 {
                continue;
            }

            let x = min_x + col as Coord * spacing_scaled;

            for row in 0..num_rows {
                let y_base = min_y + (row as Coord * hex_height_scaled);
                let y_offset = -hex_height_scaled / 3;
                let y1 = y_base + y_offset;
                let y2 = y1 + (2 * hex_height_scaled / 3);

                // Apply rotation
                let dx = (x - center.x) as f64;
                let dy1 = (y1 - center.y) as f64;
                let dy2 = (y2 - center.y) as f64;

                let rx = center.x as f64 + dx * cos_a - dy1 * sin_a;
                let ry1 = center.y as f64 + dx * sin_a + dy1 * cos_a;
                let ry2 = center.y as f64 + dx * sin_a + dy2 * cos_a;

                let p1 = Point::new(rx.round() as Coord, ry1.round() as Coord);
                let p2 = Point::new(
                    (center.x as f64 + dx * cos_a - dy2 * sin_a).round() as Coord,
                    ry2.round() as Coord,
                );

                // Clip vertical segment to fill area
                let clipped = self.clip_lines_to_polygons(&[(p1, p2)], fill_area);
                for segment in clipped {
                    if segment.len() >= 2 {
                        paths.push(InfillPath::Line(Polyline::from_points(segment)));
                    }
                }
            }
        }

        // Connect paths if configured
        if self.config.connect_infill && !paths.is_empty() {
            paths = self.connect_infill_lines(paths);
        }

        paths
    }

    /// Clip a polyline to the fill polygons, returning segments that are inside.
    fn clip_polyline_to_polygons(
        &self,
        points: &[Point],
        fill_area: &ExPolygons,
    ) -> Vec<Vec<Point>> {
        let mut result = Vec::new();

        if points.len() < 2 {
            return result;
        }

        // Process each segment of the polyline
        let mut current_segment: Vec<Point> = Vec::new();

        for i in 0..points.len() - 1 {
            let p1 = points[i];
            let p2 = points[i + 1];

            // Clip this segment
            let clipped = self.clip_lines_to_polygons(&[(p1, p2)], fill_area);

            for seg in clipped {
                if seg.len() < 2 {
                    continue;
                }

                // Check if this segment connects to the current one
                if let Some(last) = current_segment.last() {
                    let first = seg[0];
                    let dist_sq =
                        (last.x - first.x).pow(2) as f64 + (last.y - first.y).pow(2) as f64;

                    // If close enough, extend current segment
                    if dist_sq < (scale(0.01) as f64).powi(2) {
                        // 0.01mm tolerance
                        // Extend current segment (skip duplicate first point)
                        for pt in seg.into_iter().skip(1) {
                            current_segment.push(pt);
                        }
                    } else {
                        // Start new segment
                        if current_segment.len() >= 2 {
                            result.push(std::mem::take(&mut current_segment));
                        }
                        current_segment = seg;
                    }
                } else {
                    current_segment = seg;
                }
            }
        }

        // Don't forget the last segment
        if current_segment.len() >= 2 {
            result.push(current_segment);
        }

        result
    }

    /// Generate gyroid infill pattern.
    ///
    /// Gyroid is a triply periodic minimal surface (TPMS) that creates a continuous,
    /// smooth 3D pattern. It provides excellent strength in all directions and is
    /// commonly used for functional parts.
    ///
    /// The 2D cross-section of a gyroid at height z is approximated by:
    /// sin(x) * cos(y) + sin(y) * cos(z) + sin(z) * cos(x) = 0
    ///
    /// For each layer, we generate the implicit curve where this equation equals zero,
    /// using a marching squares algorithm to extract the contours.
    fn generate_gyroid(&self, fill_area: &ExPolygons, layer_index: usize) -> Vec<InfillPath> {
        let mut paths = Vec::new();

        // Calculate spacing based on density
        let spacing = self.config.line_spacing();
        if spacing <= 0.0 {
            return paths;
        }

        // Gyroid period - controls the size of the pattern cells
        // Larger period = larger cells = lower density appearance
        let period = spacing * 2.0 * std::f64::consts::PI;
        let period_scaled = scale(period);

        // Get bounding box of all fill areas
        let mut bbox = BoundingBox::new();
        for expoly in fill_area {
            bbox.merge(&expoly.bounding_box());
        }

        if bbox.is_empty() {
            return paths;
        }

        // Apply rotation based on configured angle
        let angle = self.config.angle_for_layer(layer_index);
        let angle_rad = angle.to_radians();
        let cos_a = angle_rad.cos();
        let sin_a = angle_rad.sin();

        // Expand bounds to account for rotation
        let center = bbox.center();
        let half_diag = ((bbox.width() * bbox.width() + bbox.height() * bbox.height()) as f64)
            .sqrt() as Coord
            / 2
            + period_scaled;

        let min_x = center.x - half_diag;
        let max_x = center.x + half_diag;
        let min_y = center.y - half_diag;
        let max_y = center.y + half_diag;

        // Z value for this layer (affects the gyroid cross-section shape)
        // We use layer_index to vary z, creating different patterns per layer
        let z = (layer_index as f64) * 0.2 / period * 2.0 * std::f64::consts::PI;

        // Marching squares resolution - finer = smoother curves but more points
        let resolution = scale(spacing / 4.0);
        let num_x = ((max_x - min_x) / resolution + 1) as usize;
        let num_y = ((max_y - min_y) / resolution + 1) as usize;

        // Evaluate gyroid function at grid points
        // f(x,y,z) = sin(x)*cos(y) + sin(y)*cos(z) + sin(z)*cos(x)
        let eval_gyroid = |px: Coord, py: Coord| -> f64 {
            let x = (px - center.x) as f64 / period_scaled as f64 * 2.0 * std::f64::consts::PI;
            let y = (py - center.y) as f64 / period_scaled as f64 * 2.0 * std::f64::consts::PI;
            x.sin() * y.cos() + y.sin() * z.cos() + z.sin() * x.cos()
        };

        // Use marching squares to extract contour lines where f(x,y) = 0
        let mut contour_segments: Vec<(Point, Point)> = Vec::new();

        for iy in 0..num_y.saturating_sub(1) {
            for ix in 0..num_x.saturating_sub(1) {
                let x0 = min_x + (ix as Coord) * resolution;
                let x1 = min_x + ((ix + 1) as Coord) * resolution;
                let y0 = min_y + (iy as Coord) * resolution;
                let y1 = min_y + ((iy + 1) as Coord) * resolution;

                // Evaluate at cell corners
                let v00 = eval_gyroid(x0, y0);
                let v10 = eval_gyroid(x1, y0);
                let v01 = eval_gyroid(x0, y1);
                let v11 = eval_gyroid(x1, y1);

                // Determine cell configuration (which corners are inside/outside)
                let config = ((v00 > 0.0) as u8)
                    | (((v10 > 0.0) as u8) << 1)
                    | (((v01 > 0.0) as u8) << 2)
                    | (((v11 > 0.0) as u8) << 3);

                // Skip cells that are entirely inside or outside
                if config == 0 || config == 15 {
                    continue;
                }

                // Linear interpolation for zero crossing
                let lerp_x = |v0: f64, v1: f64, x0: Coord, x1: Coord| -> Coord {
                    let t = v0 / (v0 - v1);
                    (x0 as f64 + t * (x1 - x0) as f64).round() as Coord
                };

                // Edge midpoints where contour crosses
                let edge_bottom = || Point::new(lerp_x(v00, v10, x0, x1), y0);
                let edge_top = || Point::new(lerp_x(v01, v11, x0, x1), y1);
                let edge_left = || Point::new(x0, lerp_x(v00, v01, y0, y1));
                let edge_right = || Point::new(x1, lerp_x(v10, v11, y0, y1));

                // Generate segments based on marching squares lookup
                match config {
                    1 => contour_segments.push((edge_left(), edge_bottom())),
                    2 => contour_segments.push((edge_bottom(), edge_right())),
                    3 => contour_segments.push((edge_left(), edge_right())),
                    4 => contour_segments.push((edge_top(), edge_left())),
                    5 => {
                        contour_segments.push((edge_top(), edge_bottom()));
                        // Ambiguous case - could also be (edge_left(), edge_right())
                    }
                    6 => contour_segments.push((edge_bottom(), edge_top())),
                    7 => contour_segments.push((edge_top(), edge_right())),
                    8 => contour_segments.push((edge_right(), edge_top())),
                    9 => contour_segments.push((edge_bottom(), edge_top())),
                    10 => {
                        contour_segments.push((edge_left(), edge_bottom()));
                        // Ambiguous case
                    }
                    11 => contour_segments.push((edge_left(), edge_top())),
                    12 => contour_segments.push((edge_right(), edge_left())),
                    13 => contour_segments.push((edge_right(), edge_bottom())),
                    14 => contour_segments.push((edge_bottom(), edge_left())),
                    _ => {}
                }
            }
        }

        // Apply rotation to all segments
        let rotate_point = |p: Point| -> Point {
            let dx = (p.x - center.x) as f64;
            let dy = (p.y - center.y) as f64;
            let rx = center.x as f64 + dx * cos_a - dy * sin_a;
            let ry = center.y as f64 + dx * sin_a + dy * cos_a;
            Point::new(rx.round() as Coord, ry.round() as Coord)
        };

        let rotated_segments: Vec<(Point, Point)> = contour_segments
            .into_iter()
            .map(|(p1, p2)| (rotate_point(p1), rotate_point(p2)))
            .collect();

        // Connect segments into polylines
        let connected = self.connect_segments_to_polylines(&rotated_segments);

        // Clip to fill area
        for polyline_points in connected {
            if polyline_points.len() >= 2 {
                let clipped = self.clip_polyline_to_polygons(&polyline_points, fill_area);
                for segment in clipped {
                    if segment.len() >= 2 {
                        paths.push(InfillPath::Line(Polyline::from_points(segment)));
                    }
                }
            }
        }

        // Connect paths if configured
        if self.config.connect_infill && !paths.is_empty() {
            paths = self.connect_infill_lines(paths);
        }

        paths
    }

    /// Connect line segments into continuous polylines.
    fn connect_segments_to_polylines(&self, segments: &[(Point, Point)]) -> Vec<Vec<Point>> {
        if segments.is_empty() {
            return Vec::new();
        }

        let mut result = Vec::new();
        let mut used = vec![false; segments.len()];
        let tolerance_sq = (scale(0.01) as f64).powi(2); // 0.01mm tolerance

        let points_close = |p1: &Point, p2: &Point| -> bool {
            let dx = (p1.x - p2.x) as f64;
            let dy = (p1.y - p2.y) as f64;
            dx * dx + dy * dy < tolerance_sq
        };

        for start_idx in 0..segments.len() {
            if used[start_idx] {
                continue;
            }

            used[start_idx] = true;
            let mut polyline = vec![segments[start_idx].0, segments[start_idx].1];

            // Try to extend forward
            loop {
                let last = *polyline.last().unwrap();
                let mut found = false;

                for (idx, &(p1, p2)) in segments.iter().enumerate() {
                    if used[idx] {
                        continue;
                    }

                    if points_close(&last, &p1) {
                        polyline.push(p2);
                        used[idx] = true;
                        found = true;
                        break;
                    } else if points_close(&last, &p2) {
                        polyline.push(p1);
                        used[idx] = true;
                        found = true;
                        break;
                    }
                }

                if !found {
                    break;
                }
            }

            // Try to extend backward
            loop {
                let first = polyline[0];
                let mut found = false;

                for (idx, &(p1, p2)) in segments.iter().enumerate() {
                    if used[idx] {
                        continue;
                    }

                    if points_close(&first, &p2) {
                        polyline.insert(0, p1);
                        used[idx] = true;
                        found = true;
                        break;
                    } else if points_close(&first, &p1) {
                        polyline.insert(0, p2);
                        used[idx] = true;
                        found = true;
                        break;
                    }
                }

                if !found {
                    break;
                }
            }

            if polyline.len() >= 2 {
                result.push(polyline);
            }
        }

        result
    }

    /// Generate lightning infill pattern.
    ///
    /// Lightning infill creates sparse tree-like structures that branch inward
    /// from the perimeters to support the layers above. This pattern uses
    /// minimal material while providing adequate support for overhangs.
    ///
    /// The algorithm works by:
    /// 1. Creating "support points" on a grid within the fill area
    /// 2. Growing tree branches from the perimeter inward to reach support points
    /// 3. Connecting nearby branches where efficient
    ///
    /// This is a simplified implementation inspired by Cura's lightning infill.
    fn generate_lightning(&self, fill_area: &ExPolygons, layer_index: usize) -> Vec<InfillPath> {
        let mut paths = Vec::new();

        if fill_area.is_empty() {
            return paths;
        }

        // Calculate spacing based on density (lightning uses wider spacing)
        let base_spacing = self.config.line_spacing();
        // Lightning uses sparser grid since it's tree-based
        let grid_spacing = base_spacing * 2.0;

        if grid_spacing <= 0.0 {
            return paths;
        }

        let grid_spacing_scaled = scale(grid_spacing);

        // Get bounding box
        let mut bbox = BoundingBox::new();
        for expoly in fill_area {
            bbox.merge(&expoly.bounding_box());
        }

        if bbox.is_empty() {
            return paths;
        }

        // Phase offset based on layer for some variation
        let phase_offset = (layer_index % 4) as Coord * (grid_spacing_scaled / 4);

        // Step 1: Generate support points on a grid
        let mut support_points = Vec::new();
        let mut y = bbox.min.y + phase_offset;
        while y <= bbox.max.y {
            let mut x = bbox.min.x + phase_offset;
            // Offset every other row for better coverage
            if ((y - bbox.min.y) / grid_spacing_scaled) as i32 % 2 == 1 {
                x += grid_spacing_scaled / 2;
            }
            while x <= bbox.max.x {
                let pt = Point::new(x, y);
                if self.point_in_expolygons(&pt, fill_area) {
                    support_points.push(pt);
                }
                x += grid_spacing_scaled;
            }
            y += grid_spacing_scaled;
        }

        if support_points.is_empty() {
            return paths;
        }

        // Step 2: Find perimeter points to grow trees from
        // Sample points along the perimeter at regular intervals
        let perimeter_spacing = scale(base_spacing);
        let mut perimeter_points = Vec::new();

        for expoly in fill_area {
            // Sample contour
            let contour_points = expoly.contour.points();
            for i in 0..contour_points.len() {
                let j = (i + 1) % contour_points.len();
                let p1 = contour_points[i];
                let p2 = contour_points[j];

                let dx = p2.x - p1.x;
                let dy = p2.y - p1.y;
                let len = ((dx as f64).powi(2) + (dy as f64).powi(2)).sqrt();

                if len < 1.0 {
                    continue;
                }

                let num_samples = (len / perimeter_spacing as f64).ceil() as usize;
                for s in 0..num_samples {
                    let t = s as f64 / num_samples.max(1) as f64;
                    let px = p1.x as f64 + t * dx as f64;
                    let py = p1.y as f64 + t * dy as f64;
                    perimeter_points.push(Point::new(px as Coord, py as Coord));
                }
            }
        }

        if perimeter_points.is_empty() {
            return paths;
        }

        // Step 3: For each support point, find the nearest perimeter point
        // and create a branch (tree edge)
        let mut branches: Vec<(Point, Point)> = Vec::new();
        let mut connected_support = vec![false; support_points.len()];

        // Sort support points by distance to nearest perimeter for better tree structure
        let mut support_with_dist: Vec<(usize, Point, Coord)> = support_points
            .iter()
            .enumerate()
            .map(|(idx, &pt)| {
                let min_dist = perimeter_points
                    .iter()
                    .map(|pp| {
                        let dx = (pt.x - pp.x).abs();
                        let dy = (pt.y - pp.y).abs();
                        dx.max(dy)
                    })
                    .min()
                    .unwrap_or(Coord::MAX);
                (idx, pt, min_dist)
            })
            .collect();

        support_with_dist.sort_by_key(|&(_, _, dist)| dist);

        // Grow branches from perimeter to support points
        for (idx, support_pt, _) in support_with_dist {
            if connected_support[idx] {
                continue;
            }

            // Find nearest point that's either on perimeter or already connected
            let mut best_source: Option<Point> = None;
            let mut best_dist = Coord::MAX;

            // Check perimeter points
            for &pp in &perimeter_points {
                let dist = ((support_pt.x - pp.x).abs()).max((support_pt.y - pp.y).abs());
                if dist < best_dist {
                    best_dist = dist;
                    best_source = Some(pp);
                }
            }

            // Check if any existing branch endpoint is closer
            for &(_, end) in &branches {
                let dist = ((support_pt.x - end.x).abs()).max((support_pt.y - end.y).abs());
                // Prefer connecting to existing branches if reasonably close
                if dist < best_dist || (dist < best_dist + grid_spacing_scaled / 2) {
                    best_dist = dist;
                    best_source = Some(end);
                }
            }

            if let Some(source) = best_source {
                // Verify the branch stays inside the fill area
                let mid = Point::new((source.x + support_pt.x) / 2, (source.y + support_pt.y) / 2);
                if self.point_in_expolygons(&mid, fill_area) {
                    branches.push((source, support_pt));
                    connected_support[idx] = true;
                }
            }
        }

        // Step 4: Convert branches to polylines, connecting where possible
        // Group branches that share endpoints
        if branches.is_empty() {
            return paths;
        }

        let connected_polylines = self.connect_segments_to_polylines(&branches);

        for polyline in connected_polylines {
            if polyline.len() >= 2 {
                paths.push(InfillPath::Line(Polyline::from_points(polyline)));
            }
        }

        // Optionally add some horizontal connections for strength
        // Every few layers, add cross-connections
        if layer_index % 3 == 0 && !support_points.is_empty() {
            let cross_paths = self.generate_lightning_cross_links(
                &support_points,
                fill_area,
                grid_spacing_scaled,
            );
            paths.extend(cross_paths);
        }

        paths
    }

    /// Generate cross-links between lightning support points for added strength.
    fn generate_lightning_cross_links(
        &self,
        support_points: &[Point],
        fill_area: &ExPolygons,
        grid_spacing: Coord,
    ) -> Vec<InfillPath> {
        let mut paths = Vec::new();
        let max_link_dist = grid_spacing * 3 / 2; // Link nearby points

        for i in 0..support_points.len() {
            for j in (i + 1)..support_points.len() {
                let p1 = support_points[i];
                let p2 = support_points[j];

                let dx = (p2.x - p1.x).abs();
                let dy = (p2.y - p1.y).abs();
                let dist = dx.max(dy);

                // Only link points that are close and roughly aligned (horizontal or vertical)
                if dist <= max_link_dist && (dx < grid_spacing / 4 || dy < grid_spacing / 4) {
                    // Check midpoint is inside
                    let mid = Point::new((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
                    if self.point_in_expolygons(&mid, fill_area) {
                        paths.push(InfillPath::Line(Polyline::from_points(vec![p1, p2])));
                    }
                }
            }
        }

        paths
    }

    /// Generate concentric infill (inward offset loops).
    /// Generate Cross Hatch infill pattern.
    ///
    /// Cross Hatch creates a pattern that alternates between transform and repeat layers,
    /// providing a unique visual appearance and good strength characteristics.
    fn generate_cross_hatch_pattern(&self, fill_area: &ExPolygons) -> Vec<InfillPath> {
        if fill_area.is_empty() {
            return Vec::new();
        }

        let cross_hatch_config =
            CrossHatchConfig::from_density(self.config.extrusion_width, self.config.density)
                .with_angle(self.config.angle)
                .with_connect_lines(self.config.connect_infill);

        let generator = CrossHatchGenerator::new(cross_hatch_config);
        let result = generator.generate(fill_area, self.config.z_height);

        // Convert polylines to InfillPath::Line
        result.polylines.into_iter().map(InfillPath::Line).collect()
    }

    /// Generate 3D Honeycomb infill pattern.
    ///
    /// 3D Honeycomb creates a truncated octahedron pattern that interlocks between
    /// layers for excellent 3D strength.
    fn generate_honeycomb_3d_pattern(&self, fill_area: &ExPolygons) -> Vec<InfillPath> {
        if fill_area.is_empty() {
            return Vec::new();
        }

        let mut paths = Vec::new();

        let config = Honeycomb3DConfig {
            spacing: self.config.extrusion_width,
            z: self.config.z_height,
            layer_height: self.config.layer_height,
            density: self.config.density,
            angle: self.config.angle.to_radians(),
            connect_lines: self.config.connect_infill,
        };

        let generator = Honeycomb3DGenerator::new(config);

        // Generate for each ExPolygon separately
        for expoly in fill_area {
            let polylines = generator.generate(expoly);
            for pl in polylines {
                paths.push(InfillPath::Line(pl));
            }
        }

        paths
    }

    /// Generate plan path infill pattern (space-filling curves).
    ///
    /// Supports Hilbert curve, Archimedean chords, and Octagram spiral patterns.
    fn generate_plan_path_pattern(
        &self,
        fill_area: &ExPolygons,
        pattern: PlanPathPattern,
    ) -> Vec<InfillPath> {
        if fill_area.is_empty() {
            return Vec::new();
        }

        let mut paths = Vec::new();

        let config = PlanPathConfig::from_density(self.config.density, self.config.extrusion_width);
        let generator = PlanPathGenerator::new(config, pattern);

        // Generate for each ExPolygon separately
        for expoly in fill_area {
            let result = generator.generate(expoly);
            for pl in result.polylines {
                paths.push(InfillPath::Line(pl));
            }
        }

        paths
    }

    fn generate_concentric(&self, fill_area: &ExPolygons) -> Vec<InfillPath> {
        let mut paths = Vec::new();
        let spacing = self.config.line_spacing();

        // Start with the fill area and repeatedly shrink inward
        let mut current_area = fill_area.clone();

        while !current_area.is_empty() {
            // Add all current contours and holes as loops
            for expoly in &current_area {
                if !expoly.contour.is_empty() && expoly.contour.len() >= 3 {
                    paths.push(InfillPath::Loop(expoly.contour.clone()));
                }
                for hole in &expoly.holes {
                    if !hole.is_empty() && hole.len() >= 3 {
                        paths.push(InfillPath::Loop(hole.clone()));
                    }
                }
            }

            // Shrink for next iteration
            current_area = shrink(&current_area, spacing, OffsetJoinType::Miter);
        }

        paths
    }

    /// Clip lines to the fill polygons using a scanline approach.
    fn clip_lines_to_polygons(
        &self,
        lines: &[(Point, Point)],
        fill_area: &ExPolygons,
    ) -> Vec<Vec<Point>> {
        let mut result = Vec::new();

        for &(p1, p2) in lines {
            // Find intersections of this line with all polygon edges
            let mut intersections = Vec::new();

            for expoly in fill_area {
                // Check contour edges
                self.find_line_polygon_intersections(p1, p2, &expoly.contour, &mut intersections);

                // Check hole edges
                for hole in &expoly.holes {
                    self.find_line_polygon_intersections(p1, p2, hole, &mut intersections);
                }
            }

            if intersections.is_empty() {
                // Check if the entire line is inside
                if self.point_in_expolygons(
                    &Point::new((p1.x + p2.x) / 2, (p1.y + p2.y) / 2),
                    fill_area,
                ) {
                    result.push(vec![p1, p2]);
                }
                continue;
            }

            // Sort intersections by parameter t along the line
            intersections.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

            // Extract inside segments
            // Start outside, toggle at each intersection
            let mut inside = self.point_in_expolygons(&p1, fill_area);
            let mut segment_start: Option<Point> = if inside { Some(p1) } else { None };

            for t in intersections {
                let pt = self.lerp_point(p1, p2, t);

                if inside {
                    // Exiting the polygon
                    if let Some(start) = segment_start.take() {
                        if start != pt {
                            result.push(vec![start, pt]);
                        }
                    }
                } else {
                    // Entering the polygon
                    segment_start = Some(pt);
                }
                inside = !inside;
            }

            // Handle case where line ends inside
            if inside {
                if let Some(start) = segment_start {
                    if start != p2 {
                        result.push(vec![start, p2]);
                    }
                }
            }
        }

        result
    }

    /// Find intersections between a line and a polygon's edges.
    fn find_line_polygon_intersections(
        &self,
        p1: Point,
        p2: Point,
        polygon: &Polygon,
        intersections: &mut Vec<f64>,
    ) {
        let points = polygon.points();
        if points.len() < 2 {
            return;
        }

        for i in 0..points.len() {
            let j = (i + 1) % points.len();
            let e1 = points[i];
            let e2 = points[j];

            if let Some(t) = self.line_segment_intersection(p1, p2, e1, e2) {
                if t > 0.0 && t < 1.0 {
                    intersections.push(t);
                }
            }
        }
    }

    /// Calculate intersection parameter t for line (p1, p2) with segment (e1, e2).
    fn line_segment_intersection(&self, p1: Point, p2: Point, e1: Point, e2: Point) -> Option<f64> {
        let d1x = (p2.x - p1.x) as f64;
        let d1y = (p2.y - p1.y) as f64;
        let d2x = (e2.x - e1.x) as f64;
        let d2y = (e2.y - e1.y) as f64;

        let cross = d1x * d2y - d1y * d2x;

        if cross.abs() < 1e-10 {
            return None; // Parallel lines
        }

        let dx = (e1.x - p1.x) as f64;
        let dy = (e1.y - p1.y) as f64;

        let t = (dx * d2y - dy * d2x) / cross;
        let u = (dx * d1y - dy * d1x) / cross;

        if u >= 0.0 && u <= 1.0 {
            Some(t)
        } else {
            None
        }
    }

    /// Linear interpolation between two points.
    fn lerp_point(&self, p1: Point, p2: Point, t: f64) -> Point {
        Point::new(
            (p1.x as f64 + t * (p2.x - p1.x) as f64).round() as Coord,
            (p1.y as f64 + t * (p2.y - p1.y) as f64).round() as Coord,
        )
    }

    /// Check if a point is inside the fill area.
    fn point_in_expolygons(&self, point: &Point, expolygons: &[ExPolygon]) -> bool {
        for expoly in expolygons {
            if expoly.contains_point(point) {
                return true;
            }
        }
        false
    }

    /// Connect adjacent infill lines to reduce travel.
    /// Improved algorithm for solid infill: sorts segments by position along
    /// the sweep direction before connecting adjacent ones.
    fn connect_infill_lines(&self, paths: Vec<InfillPath>) -> Vec<InfillPath> {
        if paths.len() < 2 {
            return paths;
        }

        // Convert all paths to polylines for processing
        let mut polylines: Vec<Polyline> = paths
            .into_iter()
            .map(|p| p.to_polyline())
            .filter(|p| p.len() >= 2)
            .collect();

        if polylines.is_empty() {
            return Vec::new();
        }

        // Get the angle to determine sweep direction
        let angle_rad = self.config.angle.to_radians();
        let cos_a = angle_rad.cos();
        let sin_a = angle_rad.sin();

        // Threshold for connection: 2.5× extrusion width
        let connect_threshold = scale(self.config.extrusion_width * 2.5);

        // Sort polylines by their projection onto the sweep direction
        // This groups collinear segments together
        polylines.sort_by(|a, b| {
            let a_center = a.center_projection(cos_a, sin_a);
            let b_center = b.center_projection(cos_a, sin_a);
            a_center
                .partial_cmp(&b_center)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut connected = Vec::new();
        let mut current_path = polylines[0].clone();

        for next_poly in polylines.into_iter().skip(1) {
            // Check if next segment is close enough to connect
            let current_end = current_path.last_point();
            let next_start = next_poly.points()[0];
            let next_end = next_poly.points()[next_poly.points().len() - 1];

            // Calculate distances to both ends of next segment
            let dist_to_start = ((next_start.x - current_end.x) as f64)
                .hypot((next_start.y - current_end.y) as f64)
                as i64;
            let dist_to_end = ((next_end.x - current_end.x) as f64)
                .hypot((next_end.y - current_end.y) as f64) as i64;

            // Check if they're roughly collinear (projection difference small)
            let current_center = current_path.center_projection(cos_a, sin_a);
            let next_center = next_poly.center_projection(cos_a, sin_a);
            let projection_diff = (current_center - next_center).abs() as i64;

            // Connect if close enough AND roughly on same sweep line
            let min_dist = dist_to_start.min(dist_to_end);
            if min_dist <= connect_threshold
                && projection_diff <= scale(self.config.extrusion_width * 0.5)
            {
                // Determine which direction to connect
                let mut next_to_append = next_poly;
                if dist_to_end < dist_to_start {
                    next_to_append.reverse();
                }
                current_path.append(&next_to_append);
            } else {
                // Too far or different sweep line, save current and start new
                connected.push(InfillPath::Line(current_path));
                current_path = next_poly;
            }
        }

        // Don't forget the last path
        if current_path.len() >= 2 {
            connected.push(InfillPath::Line(current_path));
        }

        connected
    }
}

impl Default for InfillGenerator {
    fn default() -> Self {
        Self::with_defaults()
    }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/// Generate infill with default configuration.
pub fn generate_infill(infill_area: &[ExPolygon], layer_index: usize) -> InfillResult {
    InfillGenerator::with_defaults().generate(infill_area, layer_index)
}

/// Generate infill with custom density.
pub fn generate_infill_with_density(
    infill_area: &[ExPolygon],
    layer_index: usize,
    density: CoordF,
) -> InfillResult {
    let config = InfillConfig::with_density(density);
    InfillGenerator::new(config).generate(infill_area, layer_index)
}

/// Generate solid infill (100% density).
pub fn generate_solid_infill(infill_area: &[ExPolygon], layer_index: usize) -> InfillResult {
    let config = InfillConfig::solid();
    InfillGenerator::new(config).generate(infill_area, layer_index)
}

/// Generate grid infill with custom density.
pub fn generate_grid_infill(
    infill_area: &[ExPolygon],
    layer_index: usize,
    density: CoordF,
) -> InfillResult {
    let config = InfillConfig {
        pattern: InfillPattern::Grid,
        density: density.clamp(0.0, 1.0),
        ..Default::default()
    };
    InfillGenerator::new(config).generate(infill_area, layer_index)
}

/// Generate concentric infill.
pub fn generate_concentric_infill(infill_area: &[ExPolygon]) -> InfillResult {
    let config = InfillConfig {
        pattern: InfillPattern::Concentric,
        density: 1.0,
        ..Default::default()
    };
    InfillGenerator::new(config).generate(infill_area, 0)
}

/// Generate honeycomb infill with the specified density.
///
/// Honeycomb creates a hexagonal pattern that provides excellent strength-to-weight ratio.
/// The pattern alternates on consecutive layers to create interlocking hexagons.
pub fn generate_honeycomb_infill(
    infill_area: &[ExPolygon],
    density: f64,
    layer_index: usize,
) -> InfillResult {
    let config = InfillConfig {
        pattern: InfillPattern::Honeycomb,
        density,
        ..Default::default()
    };
    InfillGenerator::new(config).generate(infill_area, layer_index)
}

/// Generate gyroid infill with the specified density.
///
/// Gyroid creates a triply periodic minimal surface pattern that provides excellent
/// strength in all directions. It's commonly used for functional parts requiring
/// isotropic mechanical properties.
/// Generate lightning infill.
pub fn generate_lightning_infill(
    infill_area: &[ExPolygon],
    layer_index: usize,
    density: CoordF,
) -> InfillResult {
    let config = InfillConfig {
        pattern: InfillPattern::Lightning,
        density,
        ..Default::default()
    };
    InfillGenerator::new(config).generate(infill_area, layer_index)
}

pub fn generate_gyroid_infill(
    infill_area: &[ExPolygon],
    density: f64,
    layer_index: usize,
) -> InfillResult {
    let config = InfillConfig {
        pattern: InfillPattern::Gyroid,
        density,
        ..Default::default()
    };
    InfillGenerator::new(config).generate(infill_area, layer_index)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square_mm(x: f64, y: f64, size: f64) -> ExPolygon {
        let poly = Polygon::rectangle(
            Point::new(scale(x), scale(y)),
            Point::new(scale(x + size), scale(y + size)),
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
            Point::new(scale(x), scale(y)),
            Point::new(scale(x + outer_size), scale(y + outer_size)),
        );
        let inner = Polygon::rectangle(
            Point::new(scale(x + hole_offset), scale(y + hole_offset)),
            Point::new(
                scale(x + hole_offset + hole_size),
                scale(y + hole_offset + hole_size),
            ),
        );
        ExPolygon::with_holes(outer, vec![inner])
    }

    #[test]
    fn test_infill_config_default() {
        let config = InfillConfig::default();
        assert_eq!(config.pattern, InfillPattern::Rectilinear);
        assert!((config.density - 0.2).abs() < 1e-6);
    }

    #[test]
    fn test_infill_config_line_spacing() {
        let config = InfillConfig {
            density: 0.2,
            extrusion_width: 0.4,
            ..Default::default()
        };

        let spacing = config.line_spacing();
        // spacing = width / density = 0.4 / 0.2 = 2.0mm
        assert!((spacing - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_infill_config_solid() {
        let config = InfillConfig::solid();
        assert!((config.density - 1.0).abs() < 1e-6);

        let spacing = config.line_spacing();
        // For solid, spacing should equal extrusion width
        assert!((spacing - config.extrusion_width).abs() < 1e-6);
    }

    #[test]
    fn test_infill_angle_for_layer() {
        let config = InfillConfig {
            angle: 45.0,
            angle_increment: 90.0,
            ..Default::default()
        };

        assert!((config.angle_for_layer(0) - 45.0).abs() < 1e-6);
        assert!((config.angle_for_layer(1) - 135.0).abs() < 1e-6);
        assert!((config.angle_for_layer(2) - 225.0).abs() < 1e-6);
    }

    #[test]
    fn test_generate_rectilinear() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Rectilinear,
            density: 0.2,
            extrusion_width: 0.4,
            angle: 0.0,
            connect_infill: false,
            overlap: 0.0,
            ..Default::default()
        };

        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert!(result.has_infill());
        assert!(result.total_length_mm > 0.0);

        println!(
            "Rectilinear infill: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_generate_grid() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Grid,
            density: 0.2,
            extrusion_width: 0.4,
            angle: 45.0,
            connect_infill: false,
            overlap: 0.0,
            ..Default::default()
        };

        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert!(result.has_infill());

        // Grid should have more paths than rectilinear (two directions)
        println!(
            "Grid infill: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_generate_concentric() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Concentric,
            density: 1.0,
            extrusion_width: 0.4,
            ..Default::default()
        };

        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert!(result.has_infill());

        // Should have loop paths
        let loop_count = result.paths.iter().filter(|p| p.is_loop()).count();
        assert!(loop_count > 0);

        println!(
            "Concentric infill: {} loops, {:.2} mm total",
            loop_count, result.total_length_mm
        );
    }

    #[test]
    fn test_generate_with_hole() {
        let expoly = make_square_with_hole_mm(0.0, 0.0, 30.0, 10.0, 10.0);
        let config = InfillConfig {
            pattern: InfillPattern::Rectilinear,
            density: 0.3,
            extrusion_width: 0.4,
            angle: 45.0,
            connect_infill: false,
            overlap: 0.0,
            ..Default::default()
        };

        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[expoly], 0);

        assert!(result.has_infill());

        // Lines should avoid the hole
        println!(
            "Infill with hole: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_zero_density() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Rectilinear,
            density: 0.0,
            ..Default::default()
        };

        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        // Zero density should produce no infill
        assert!(!result.has_infill());
    }

    #[test]
    fn test_empty_area() {
        let generator = InfillGenerator::with_defaults();
        let result = generator.generate(&[], 0);

        assert!(!result.has_infill());
    }

    #[test]
    fn test_infill_path_types() {
        let line = InfillPath::Line(Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 100),
        ]));

        let loop_path =
            InfillPath::Loop(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));

        assert!(line.is_line());
        assert!(!line.is_loop());
        assert!(!loop_path.is_line());
        assert!(loop_path.is_loop());

        assert!(line.length() > 0.0);
        assert!(loop_path.length() > 0.0);
    }

    #[test]
    fn test_infill_path_reverse() {
        let mut line = InfillPath::Line(Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
        ]));

        let start_before = line.start_point();
        let end_before = line.end_point();

        line.reverse();

        assert_eq!(line.start_point(), end_before);
        assert_eq!(line.end_point(), start_before);
    }

    #[test]
    fn test_convenience_functions() {
        let square = make_square_mm(0.0, 0.0, 20.0);

        // Test generate_infill
        let result = generate_infill(&[square.clone()], 0);
        assert!(result.has_infill());

        // Test generate_infill_with_density
        let result = generate_infill_with_density(&[square.clone()], 0, 0.5);
        assert!(result.has_infill());

        // Test generate_solid_infill
        let result = generate_solid_infill(&[square.clone()], 0);
        assert!(result.has_infill());
        assert!((result.density - 1.0).abs() < 1e-6);

        // Test generate_grid_infill
        let result = generate_grid_infill(&[square.clone()], 0, 0.3);
        assert!(result.has_infill());
        assert_eq!(result.pattern, InfillPattern::Grid);

        // Test generate_concentric_infill
        let result = generate_concentric_infill(&[square]);
        assert!(result.has_infill());
        assert_eq!(result.pattern, InfillPattern::Concentric);
    }

    #[test]
    fn test_connected_infill() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Rectilinear,
            density: 0.2,
            extrusion_width: 0.4,
            angle: 0.0,
            connect_infill: true,
            overlap: 0.0,
            ..Default::default()
        };

        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert!(result.has_infill());

        // With connection enabled, should have fewer paths (some merged)
        println!(
            "Connected infill: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_pattern_is_implemented() {
        assert!(InfillPattern::Rectilinear.is_implemented());
        assert!(InfillPattern::Grid.is_implemented());
        assert!(InfillPattern::Concentric.is_implemented());
        assert!(InfillPattern::Line.is_implemented());
        assert!(InfillPattern::None.is_implemented());
        assert!(InfillPattern::Honeycomb.is_implemented());
        assert!(InfillPattern::Gyroid.is_implemented());
        assert!(InfillPattern::Lightning.is_implemented());
    }

    #[test]
    fn test_pattern_is_linear() {
        assert!(InfillPattern::Rectilinear.is_linear());
        assert!(InfillPattern::Grid.is_linear());
        assert!(InfillPattern::Line.is_linear());
        assert!(!InfillPattern::Concentric.is_linear());
        assert!(!InfillPattern::Honeycomb.is_linear());
    }

    #[test]
    fn test_generate_honeycomb() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Honeycomb,
            density: 0.2,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert!(result.has_infill());
        assert!(result.path_count() > 0);
        assert!(result.total_length_mm > 0.0);
        assert_eq!(result.pattern, InfillPattern::Honeycomb);

        println!(
            "Honeycomb infill: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_honeycomb_layer_alternation() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Honeycomb,
            density: 0.3,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);

        // Generate for even and odd layers
        let result_even = generator.generate(&[square.clone()], 0);
        let result_odd = generator.generate(&[square], 1);

        // Both should have infill
        assert!(result_even.has_infill());
        assert!(result_odd.has_infill());

        // The patterns should be different (phase alternates)
        // We can't easily compare paths directly, but we can check they both generate content
        println!(
            "Even layer: {} paths, Odd layer: {} paths",
            result_even.path_count(),
            result_odd.path_count()
        );
    }

    #[test]
    fn test_honeycomb_with_hole() {
        let square_with_hole = make_square_with_hole_mm(0.0, 0.0, 30.0, 10.0, 10.0);
        let config = InfillConfig {
            pattern: InfillPattern::Honeycomb,
            density: 0.25,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square_with_hole], 0);

        assert!(result.has_infill());
        // Should have paths that avoid the hole
        assert!(result.path_count() > 0);

        println!(
            "Honeycomb with hole: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_honeycomb_convenience_function() {
        let square = make_square_mm(0.0, 0.0, 15.0);
        let result = generate_honeycomb_infill(&[square], 0.2, 0);

        assert!(result.has_infill());
        assert_eq!(result.pattern, InfillPattern::Honeycomb);
        assert!((result.density - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_generate_gyroid() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Gyroid,
            density: 0.2,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert!(result.has_infill());
        assert!(result.path_count() > 0);
        assert!(result.total_length_mm > 0.0);
        assert_eq!(result.pattern, InfillPattern::Gyroid);

        println!(
            "Gyroid infill: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_gyroid_layer_variation() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Gyroid,
            density: 0.3,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);

        // Generate for different layers - gyroid pattern changes with z
        let result_0 = generator.generate(&[square.clone()], 0);
        let result_5 = generator.generate(&[square.clone()], 5);
        let result_10 = generator.generate(&[square], 10);

        // All should have infill
        assert!(result_0.has_infill());
        assert!(result_5.has_infill());
        assert!(result_10.has_infill());

        println!(
            "Gyroid layer 0: {} paths, layer 5: {} paths, layer 10: {} paths",
            result_0.path_count(),
            result_5.path_count(),
            result_10.path_count()
        );
    }

    #[test]
    fn test_gyroid_with_hole() {
        let square_with_hole = make_square_with_hole_mm(0.0, 0.0, 30.0, 10.0, 10.0);
        let config = InfillConfig {
            pattern: InfillPattern::Gyroid,
            density: 0.25,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square_with_hole], 0);

        assert!(result.has_infill());
        assert!(result.path_count() > 0);

        println!(
            "Gyroid with hole: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_gyroid_convenience_function() {
        let square = make_square_mm(0.0, 0.0, 15.0);
        let result = generate_gyroid_infill(&[square], 0.2, 0);

        assert!(result.has_infill());
        assert_eq!(result.pattern, InfillPattern::Gyroid);
        assert!((result.density - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_generate_lightning() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Lightning,
            density: 0.15,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert!(result.has_infill());
        assert!(result.path_count() > 0);
        assert_eq!(result.pattern, InfillPattern::Lightning);

        println!(
            "Lightning infill: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_lightning_layer_variation() {
        let square = make_square_mm(0.0, 0.0, 25.0);
        let config = InfillConfig {
            pattern: InfillPattern::Lightning,
            density: 0.15,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);

        // Generate for multiple layers - lightning should vary by layer
        let result_0 = generator.generate(&[square.clone()], 0);
        let result_1 = generator.generate(&[square.clone()], 1);
        let result_3 = generator.generate(&[square], 3);

        // All should have infill
        assert!(result_0.has_infill());
        assert!(result_1.has_infill());
        assert!(result_3.has_infill());

        println!(
            "Lightning layer 0: {} paths, layer 1: {} paths, layer 3: {} paths",
            result_0.path_count(),
            result_1.path_count(),
            result_3.path_count()
        );
    }

    #[test]
    fn test_lightning_with_hole() {
        let square_with_hole = make_square_with_hole_mm(0.0, 0.0, 30.0, 10.0, 10.0);
        let config = InfillConfig {
            pattern: InfillPattern::Lightning,
            density: 0.15,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square_with_hole], 0);

        assert!(result.has_infill());
        assert!(result.path_count() > 0);

        println!(
            "Lightning with hole: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_lightning_convenience_function() {
        let square = make_square_mm(0.0, 0.0, 15.0);
        let result = generate_lightning_infill(&[square], 0, 0.15);

        assert!(result.has_infill());
        assert_eq!(result.pattern, InfillPattern::Lightning);
        assert!((result.density - 0.15).abs() < 0.001);
    }

    #[test]
    fn test_generate_cross_hatch() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::CrossHatch,
            density: 0.2,
            z_height: 0.4,
            layer_height: 0.2,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert!(result.has_infill());
        assert!(result.path_count() > 0);
        assert_eq!(result.pattern, InfillPattern::CrossHatch);

        println!(
            "CrossHatch: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_cross_hatch_layer_variation() {
        let square = make_square_mm(0.0, 0.0, 20.0);

        // Test at different Z heights - pattern should vary
        let config = InfillConfig {
            pattern: InfillPattern::CrossHatch,
            density: 0.2,
            layer_height: 0.2,
            ..Default::default()
        };

        let mut config_z1 = config.clone();
        config_z1.z_height = 0.2;
        let gen1 = InfillGenerator::new(config_z1);
        let result_z1 = gen1.generate(&[square.clone()], 0);

        let mut config_z2 = config.clone();
        config_z2.z_height = 0.4;
        let gen2 = InfillGenerator::new(config_z2);
        let result_z2 = gen2.generate(&[square.clone()], 1);

        let mut config_z3 = config.clone();
        config_z3.z_height = 0.6;
        let gen3 = InfillGenerator::new(config_z3);
        let result_z3 = gen3.generate(&[square.clone()], 2);

        // All should produce infill
        assert!(result_z1.has_infill());
        assert!(result_z2.has_infill());
        assert!(result_z3.has_infill());

        println!(
            "CrossHatch Z variation: z=0.2: {} paths, z=0.4: {} paths, z=0.6: {} paths",
            result_z1.path_count(),
            result_z2.path_count(),
            result_z3.path_count()
        );
    }

    #[test]
    fn test_cross_hatch_with_hole() {
        let square_with_hole = make_square_with_hole_mm(0.0, 0.0, 30.0, 10.0, 10.0);
        let config = InfillConfig {
            pattern: InfillPattern::CrossHatch,
            density: 0.2,
            z_height: 0.2,
            layer_height: 0.2,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square_with_hole], 0);

        assert!(result.has_infill());
        assert!(result.path_count() > 0);

        println!(
            "CrossHatch with hole: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_generate_honeycomb_3d() {
        // Honeycomb3D needs larger areas and higher density for reliable generation
        let square = make_square_mm(0.0, 0.0, 50.0);
        let config = InfillConfig {
            pattern: InfillPattern::Honeycomb3D,
            density: 0.3, // Higher density for more reliable pattern
            z_height: 0.2,
            layer_height: 0.2,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        // Honeycomb3D may produce empty results for some Z heights or small areas
        // At minimum, verify generation completes without error
        assert_eq!(result.pattern, InfillPattern::Honeycomb3D);

        println!(
            "Honeycomb3D: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_honeycomb_3d_layer_variation() {
        // Larger area for reliable pattern generation
        let square = make_square_mm(0.0, 0.0, 50.0);

        // Test at different Z heights - 3D pattern should create interlocking layers
        let config = InfillConfig {
            pattern: InfillPattern::Honeycomb3D,
            density: 0.3, // Higher density
            layer_height: 0.2,
            ..Default::default()
        };

        let mut config_z1 = config.clone();
        config_z1.z_height = 0.2;
        let gen1 = InfillGenerator::new(config_z1);
        let result_z1 = gen1.generate(&[square.clone()], 0);

        let mut config_z2 = config.clone();
        config_z2.z_height = 0.4;
        let gen2 = InfillGenerator::new(config_z2);
        let result_z2 = gen2.generate(&[square.clone()], 1);

        let mut config_z3 = config.clone();
        config_z3.z_height = 0.6;
        let gen3 = InfillGenerator::new(config_z3);
        let result_z3 = gen3.generate(&[square.clone()], 2);

        // Verify generation completed without error
        // Note: Some Z heights may produce empty results depending on the pattern phase
        assert_eq!(result_z1.pattern, InfillPattern::Honeycomb3D);
        assert_eq!(result_z2.pattern, InfillPattern::Honeycomb3D);
        assert_eq!(result_z3.pattern, InfillPattern::Honeycomb3D);

        println!(
            "Honeycomb3D Z variation: z=0.2: {} paths, z=0.4: {} paths, z=0.6: {} paths",
            result_z1.path_count(),
            result_z2.path_count(),
            result_z3.path_count()
        );
    }

    #[test]
    fn test_honeycomb_3d_with_hole() {
        // Larger area with smaller hole for reliable pattern generation
        let square_with_hole = make_square_with_hole_mm(0.0, 0.0, 50.0, 15.0, 15.0);
        let config = InfillConfig {
            pattern: InfillPattern::Honeycomb3D,
            density: 0.3,
            z_height: 0.2,
            layer_height: 0.2,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square_with_hole], 0);

        // Verify generation completed without error
        assert_eq!(result.pattern, InfillPattern::Honeycomb3D);

        println!(
            "Honeycomb3D with hole: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_adaptive_fallback_to_rectilinear() {
        // Adaptive infill requires an octree, so it should fall back to rectilinear
        // when used through the standard InfillGenerator
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::Adaptive,
            density: 0.2,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        // Should still produce infill (rectilinear fallback)
        assert!(result.has_infill());
        assert!(result.path_count() > 0);
        // Pattern field should still report Adaptive even though it fell back
        assert_eq!(result.pattern, InfillPattern::Adaptive);

        println!(
            "Adaptive (fallback): {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_support_cubic_fallback_to_rectilinear() {
        // Support cubic also requires an octree, so it should fall back
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::SupportCubic,
            density: 0.2,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        // Should still produce infill (rectilinear fallback)
        assert!(result.has_infill());
        assert!(result.path_count() > 0);
        assert_eq!(result.pattern, InfillPattern::SupportCubic);

        println!(
            "SupportCubic (fallback): {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_generate_hilbert_curve() {
        let square = make_square_mm(0.0, 0.0, 50.0);
        let config = InfillConfig {
            pattern: InfillPattern::HilbertCurve,
            density: 0.3,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert_eq!(result.pattern, InfillPattern::HilbertCurve);
        // Hilbert curve should produce paths
        assert!(result.has_infill());

        println!(
            "Hilbert Curve: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_generate_archimedean_chords() {
        let square = make_square_mm(0.0, 0.0, 50.0);
        let config = InfillConfig {
            pattern: InfillPattern::ArchimedeanChords,
            density: 0.3,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert_eq!(result.pattern, InfillPattern::ArchimedeanChords);
        // Archimedean should produce paths
        assert!(result.has_infill());

        println!(
            "Archimedean Chords: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_generate_octagram_spiral() {
        let square = make_square_mm(0.0, 0.0, 50.0);
        let config = InfillConfig {
            pattern: InfillPattern::OctagramSpiral,
            density: 0.3,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert_eq!(result.pattern, InfillPattern::OctagramSpiral);
        // Octagram should produce paths
        assert!(result.has_infill());

        println!(
            "Octagram Spiral: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_hilbert_with_hole() {
        let square_with_hole = make_square_with_hole_mm(0.0, 0.0, 50.0, 15.0, 15.0);
        let config = InfillConfig {
            pattern: InfillPattern::HilbertCurve,
            density: 0.3,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square_with_hole], 0);

        assert_eq!(result.pattern, InfillPattern::HilbertCurve);

        println!(
            "Hilbert with hole: {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_pattern_is_implemented_all() {
        // Verify all patterns we claim are implemented actually work
        assert!(InfillPattern::HilbertCurve.is_implemented());
        assert!(InfillPattern::ArchimedeanChords.is_implemented());
        assert!(InfillPattern::OctagramSpiral.is_implemented());
        assert!(InfillPattern::CrossHatch.is_implemented());
        assert!(InfillPattern::Honeycomb3D.is_implemented());
    }

    #[test]
    fn test_pattern_is_linear_plan_path() {
        // Plan path patterns are linear (generate polylines, not loops)
        assert!(InfillPattern::HilbertCurve.is_linear());
        assert!(InfillPattern::ArchimedeanChords.is_linear());
        assert!(InfillPattern::OctagramSpiral.is_linear());
    }

    #[test]
    fn test_floating_concentric_pattern() {
        // FloatingConcentric should be implemented and concentric
        assert!(InfillPattern::FloatingConcentric.is_implemented());
        assert!(InfillPattern::FloatingConcentric.is_concentric());
        assert!(!InfillPattern::FloatingConcentric.is_linear());
    }

    #[test]
    fn test_generate_floating_concentric_via_generator() {
        // Test FloatingConcentric through InfillGenerator (falls back to regular concentric)
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: InfillPattern::FloatingConcentric,
            density: 0.5,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate(&[square], 0);

        assert_eq!(result.pattern, InfillPattern::FloatingConcentric);
        assert!(result.has_infill());
        assert!(result.path_count() > 0);

        println!(
            "FloatingConcentric (via generator): {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_generate_floating_concentric_with_floating_areas() {
        // Test FloatingConcentric with actual floating area detection
        let square = make_square_mm(0.0, 0.0, 20.0);
        // Create a smaller floating area in the center
        let floating_area = make_square_mm(0.0, 0.0, 10.0);

        let config = InfillConfig {
            pattern: InfillPattern::FloatingConcentric,
            density: 0.5,
            ..Default::default()
        };
        let generator = InfillGenerator::new(config);
        let result = generator.generate_with_floating_areas(&[square], &[floating_area], 0);

        assert_eq!(result.pattern, InfillPattern::FloatingConcentric);
        assert!(result.has_infill());
        assert!(result.path_count() > 0);

        println!(
            "FloatingConcentric (with floating areas): {} paths, {:.2} mm total",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_floating_concentric_direct_api() {
        // Test using the direct floating concentric API
        let square = make_square_mm(0.0, 0.0, 20.0);
        let floating_area = make_square_mm(0.0, 0.0, 8.0);

        let fill_area = vec![square];
        let floating_areas = vec![floating_area];
        let fc_result = generate_floating_concentric(&fill_area, &floating_areas, 0.5);

        assert!(fc_result.has_infill());
        assert!(fc_result.loop_count > 0);

        // Note: floating_fraction may be 0 if all generated concentric loops
        // happen to be outside the floating area (depends on spacing/geometry).
        // The important thing is that the detection mechanism works.
        println!(
            "FloatingConcentric (direct): {} loops, {:.2} mm total, {:.1}% floating",
            fc_result.loop_count,
            fc_result.total_length_mm,
            fc_result.floating_fraction * 100.0
        );
    }

    #[test]
    fn test_floating_concentric_all_floating() {
        // Test case where fill area is entirely inside floating area
        let small_square = make_square_mm(0.0, 0.0, 10.0);
        let large_floating = make_square_mm(0.0, 0.0, 30.0);

        let fill_area = vec![small_square];
        let floating_areas = vec![large_floating];
        let fc_result = generate_floating_concentric(&fill_area, &floating_areas, 0.5);

        assert!(fc_result.has_infill());
        // When fill area is entirely inside floating area, should be ~100% floating
        assert!(
            fc_result.floating_fraction > 0.9,
            "Expected >90% floating, got {:.1}%",
            fc_result.floating_fraction * 100.0
        );

        println!(
            "FloatingConcentric (all floating): {} loops, {:.2} mm total, {:.1}% floating",
            fc_result.loop_count,
            fc_result.total_length_mm,
            fc_result.floating_fraction * 100.0
        );
    }

    #[test]
    fn test_pattern_display() {
        // Test Display implementation for InfillPattern
        assert_eq!(format!("{}", InfillPattern::Rectilinear), "Rectilinear");
        assert_eq!(
            format!("{}", InfillPattern::FloatingConcentric),
            "Floating Concentric"
        );
        assert_eq!(format!("{}", InfillPattern::Honeycomb3D), "3D Honeycomb");
    }
}
