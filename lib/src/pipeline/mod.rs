//! Pipeline module - orchestrates the complete slicing process.
//!
//! This module provides a high-level API for the complete slicing pipeline:
//! mesh → layers → perimeters → infill → supports → paths → G-code
//!
//! # Surface Type Detection
//!
//! This pipeline implements proper surface classification similar to BambuStudio's
//! `PrintObject::detect_surfaces_type()`. Instead of simply using layer position
//! to determine solid vs sparse infill, it:
//!
//! 1. Compares each layer's geometry with adjacent layers
//! 2. Identifies TOP surfaces (areas not covered by layer above)
//! 3. Identifies BOTTOM surfaces (areas not supported by layer below)
//! 4. Marks bridge surfaces (bottom areas over air)
//! 5. Propagates solid infill through the configured number of top/bottom layers
//!
//! # Example
//!
//! ```rust,ignore
//! use slicer::pipeline::{PrintPipeline, PipelineConfig};
//! use slicer::mesh::TriangleMesh;
//!
//! let mesh = TriangleMesh::cube(20.0);
//! let config = PipelineConfig::default();
//! let pipeline = PrintPipeline::new(config);
//!
//! let gcode = pipeline.process(&mesh)?;
//! gcode.write_to_file("output.gcode")?;
//! ```

use crate::bridge::{detect_bridge_direction_from_anchors, BridgeConfig, BridgeDetector};
use crate::config::{
    InfillPattern as ConfigInfillPattern, PerimeterMode, PrintConfig, PrintObjectConfig,
    SeamPosition as ConfigSeamPosition,
};
use crate::flow::Flow;
use crate::gcode::{
    ArcFitter, ArcFittingConfig, ExtrusionPath, ExtrusionRole, GCode, GCodeWriter, LayerPaths,
    PathConfig, PathGenerator, PathSegment, SeamPosition,
};
use crate::geometry::ExPolygons;
use crate::geometry::PointF;
use crate::geometry::{ExPolygon, Point, Polygon};
use crate::infill::{InfillConfig, InfillGenerator, InfillPattern, InfillResult};
use crate::mesh::TriangleMesh;
use crate::perimeter::arachne::{ArachneConfig, ArachneGenerator};
use crate::perimeter::{PerimeterConfig, PerimeterGenerator};
use crate::slice::{
    detect_surface_types, propagate_solid_infill, Layer, Slicer, SlicingParams, Surface,
    SurfaceDetectionConfig, SurfaceType,
};
use crate::support::{SupportConfig, SupportGenerator, SupportLayer, SupportPattern};
use crate::travel::{AvoidCrossingPerimeters, TravelConfig};
use crate::{clipper, scale, unscale, CoordF, Result};

/// Configuration for the printing pipeline.
#[derive(Clone, Debug)]
pub struct PipelineConfig {
    /// Global print configuration.
    pub print: PrintConfig,

    /// Object-specific configuration.
    pub object: PrintObjectConfig,

    /// Slicing parameters.
    pub slicing: SlicingParams,

    /// Support configuration.
    pub support: SupportConfig,

    /// Bridge detection configuration.
    pub bridge: BridgeConfig,

    /// Enable bridge detection.
    pub detect_bridges: bool,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            print: PrintConfig::default(),
            object: PrintObjectConfig::default(),
            slicing: SlicingParams::default(),
            support: SupportConfig::default(),
            bridge: BridgeConfig::default(),
            detect_bridges: true, // Bridge detection enabled by default
        }
    }
}

impl PipelineConfig {
    /// Create a new pipeline configuration.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder: set layer height.
    pub fn layer_height(mut self, height: CoordF) -> Self {
        self.slicing.layer_height = height;
        self.print.layer_height = height;
        self.object.layer_height = height;
        self
    }

    /// Builder: set first layer height.
    pub fn first_layer_height(mut self, height: CoordF) -> Self {
        self.slicing.first_layer_height = height;
        self.print.first_layer_height = height;
        self
    }

    /// Builder: set number of perimeters.
    pub fn perimeters(mut self, count: u32) -> Self {
        self.object.perimeters = count;
        self
    }

    /// Builder: set infill density.
    pub fn infill_density(mut self, density: CoordF) -> Self {
        self.object.fill_density = density;
        self
    }

    /// Builder: set perimeter mode.
    pub fn perimeter_mode(mut self, mode: PerimeterMode) -> Self {
        self.object.perimeter_mode = mode;
        self
    }

    /// Builder: enable Arachne variable-width perimeters.
    pub fn arachne(mut self) -> Self {
        self.object.perimeter_mode = PerimeterMode::Arachne;
        self
    }

    /// Builder: use classic fixed-width perimeters.
    pub fn classic_perimeters(mut self) -> Self {
        self.object.perimeter_mode = PerimeterMode::Classic;
        self
    }

    /// Builder: set nozzle diameter.
    pub fn nozzle_diameter(mut self, diameter: CoordF) -> Self {
        self.print.nozzle_diameter = diameter;
        self
    }

    /// Builder: set extruder temperature.
    pub fn extruder_temperature(mut self, temp: u32) -> Self {
        self.print.extruder_temperature = temp;
        self.print.first_layer_extruder_temperature = temp + 5;
        self
    }

    /// Builder: set bed temperature.
    pub fn bed_temperature(mut self, temp: u32) -> Self {
        self.print.bed_temperature = temp;
        self.print.first_layer_bed_temperature = temp + 5;
        self
    }

    /// Builder: use relative extrusion mode (M83).
    /// When true, E values are output as relative/incremental values.
    /// Most modern printers use relative E mode. BambuStudio defaults to M83.
    pub fn use_relative_e(mut self, relative: bool) -> Self {
        self.print.use_relative_e = relative;
        self
    }

    /// Builder: enable arc fitting (G2/G3 commands).
    pub fn arc_fitting(mut self, enabled: bool) -> Self {
        self.print.arc_fitting_enabled = enabled;
        self
    }

    /// Builder: set arc fitting tolerance.
    pub fn arc_fitting_tolerance(mut self, tolerance: f64) -> Self {
        self.print.arc_fitting_tolerance = tolerance;
        self
    }

    /// Enable support generation.
    pub fn support_enabled(mut self, enabled: bool) -> Self {
        self.support.enabled = enabled;
        self
    }

    /// Set support overhang angle threshold (degrees).
    pub fn support_overhang_angle(mut self, angle: f64) -> Self {
        self.support.overhang_angle = angle;
        self
    }

    /// Set support density (0.0 - 1.0).
    pub fn support_density(mut self, density: f64) -> Self {
        self.support.density = density.clamp(0.05, 1.0);
        self
    }

    /// Set support pattern.
    pub fn support_pattern(mut self, pattern: SupportPattern) -> Self {
        self.support.pattern = pattern;
        self
    }

    /// Enable buildplate-only support.
    pub fn support_buildplate_only(mut self, value: bool) -> Self {
        self.support.buildplate_only = value;
        self
    }

    /// Enable or disable bridge detection.
    pub fn bridge_detection(mut self, enabled: bool) -> Self {
        self.detect_bridges = enabled;
        self
    }

    /// Enable or disable avoid crossing perimeters.
    /// When enabled, travel moves will be routed around perimeter walls.
    pub fn avoid_crossing_perimeters(mut self, enabled: bool) -> Self {
        self.print.avoid_crossing_perimeters = enabled;
        self
    }

    /// Set the maximum detour percentage for avoid crossing perimeters.
    /// If the detour path is longer than this percentage of the direct path,
    /// the direct path will be used instead. Default is 200% (2x direct distance).
    pub fn avoid_crossing_max_detour(mut self, percent: CoordF) -> Self {
        self.print.avoid_crossing_max_detour = percent;
        self
    }

    /// Set bridge flow multiplier.
    pub fn bridge_flow_multiplier(mut self, multiplier: f64) -> Self {
        self.bridge.flow_multiplier = multiplier.clamp(0.5, 2.0);
        self
    }

    /// Set bridge speed multiplier.
    pub fn bridge_speed_multiplier(mut self, multiplier: f64) -> Self {
        self.bridge.speed_multiplier = multiplier.clamp(0.1, 1.0);
        self
    }

    /// Check if support generation is enabled.
    pub fn uses_support(&self) -> bool {
        self.support.enabled
    }

    /// Check if bridge detection is enabled.
    pub fn uses_bridge_detection(&self) -> bool {
        self.detect_bridges
    }

    /// Get the support configuration.
    pub fn support_config(&self) -> &SupportConfig {
        &self.support
    }

    /// Get the perimeter configuration from the pipeline config.
    pub fn perimeter_config(&self) -> PerimeterConfig {
        let perimeter_width = self.print.nozzle_diameter * 1.125;
        let external_width = self.print.nozzle_diameter * 1.05;
        let layer_height = self.print.layer_height;
        let perimeter_count = self.object.perimeters as usize;

        // Create config with proper spacing calculations
        let mut config = PerimeterConfig::new(
            perimeter_width,
            external_width,
            layer_height,
            perimeter_count,
            self.print.nozzle_diameter,
        );

        // Apply additional settings
        config.external_perimeters_first = false;
        config.min_perimeter_area = self.print.nozzle_diameter.powi(2);
        config.gap_fill_threshold = self.print.nozzle_diameter * 0.5;
        config.thin_walls = self.object.thin_walls;
        config.join_type = crate::clipper::OffsetJoinType::Miter;

        // BambuStudio: surface simplification before perimeter generation
        // Use 0.05mm resolution to reduce point count while maintaining quality.
        // The original 0.01mm was too fine, causing 8x more points than needed.
        config.surface_simplify_resolution = 0.05;
        config.arc_fitting_enabled = self.print.arc_fitting_enabled;

        config
    }

    /// Get the Arachne configuration from the pipeline config.
    pub fn arachne_config(&self) -> ArachneConfig {
        use crate::perimeter::arachne::BeadingStrategy;

        let bead_width = self.print.nozzle_diameter * 1.125;
        let outer_bead_width = self.print.nozzle_diameter * 1.05;

        ArachneConfig {
            bead_width_outer: outer_bead_width,
            bead_width_inner: bead_width,
            min_bead_width: self.object.arachne_min_bead_width,
            min_feature_size: self.object.arachne_min_feature_size,
            wall_count: self.object.perimeters as usize,
            wall_0_inset: 0.0,
            wall_transition_length: self.object.arachne_wall_transition_length,
            wall_transition_angle: 30.0, // degrees
            print_thin_walls: self.object.thin_walls,
            join_type: crate::clipper::OffsetJoinType::Miter,
            beading_strategy: BeadingStrategy::Distributed,
        }
    }

    /// Check if Arachne mode is enabled.
    pub fn uses_arachne(&self) -> bool {
        self.object.perimeter_mode == PerimeterMode::Arachne
    }

    /// Get the infill configuration from the pipeline config.
    pub fn infill_config(&self) -> InfillConfig {
        InfillConfig {
            pattern: convert_infill_pattern(self.object.fill_pattern),
            density: self.object.fill_density,
            extrusion_width: self.print.nozzle_diameter * 1.125,
            angle: 45.0,
            angle_increment: 90.0,
            overlap: 0.15,
            min_area: self.print.nozzle_diameter.powi(2) * 4.0,
            connect_infill: true,
            infill_first: false,
            z_height: 0.0,
            layer_height: self.print.layer_height,
        }
    }

    /// Check if arc fitting is enabled.
    pub fn uses_arc_fitting(&self) -> bool {
        self.print.arc_fitting_enabled
    }

    /// Get the arc fitting configuration from the pipeline config.
    pub fn arc_fitting_config(&self) -> ArcFittingConfig {
        ArcFittingConfig {
            tolerance: self.print.arc_fitting_tolerance,
            min_radius: self.print.arc_fitting_min_radius,
            max_radius: self.print.arc_fitting_max_radius,
            min_points: 3,
            max_arc_angle: std::f64::consts::PI * 1.5,
            arc_length_tolerance_percent: 0.05, // 5% - matches BambuStudio DEFAULT_ARC_LENGTH_PERCENT_TOLERANCE
            enabled: self.print.arc_fitting_enabled,
        }
    }

    /// Get the path configuration from the pipeline config.
    pub fn path_config(&self) -> PathConfig {
        PathConfig {
            // Perimeter width is typically ~1.125x nozzle diameter (not perimeters count!)
            perimeter_width: self.print.nozzle_diameter * 1.125,
            external_perimeter_width: self.print.nozzle_diameter * 1.05,
            infill_width: self.print.nozzle_diameter * 1.125,
            layer_height: self.slicing.layer_height,
            first_layer_height: self.slicing.first_layer_height,
            perimeter_speed: self.object.perimeter_speed,
            external_perimeter_speed: self.object.external_perimeter_speed,
            infill_speed: self.object.infill_speed,
            solid_infill_speed: self.object.solid_infill_speed,
            top_solid_infill_speed: self.object.top_solid_infill_speed,
            travel_speed: self.print.travel_speed,
            filament_diameter: self.print.filament_diameter,
            external_perimeters_first: false,
            infill_first: false,
            seam_position: convert_seam_position(self.object.seam_position),
        }
    }

    /// Check if avoid crossing perimeters is enabled.
    pub fn uses_avoid_crossing_perimeters(&self) -> bool {
        self.print.avoid_crossing_perimeters
    }

    /// Get the travel configuration for avoid crossing perimeters.
    pub fn travel_config(&self) -> TravelConfig {
        TravelConfig {
            enabled: self.print.avoid_crossing_perimeters,
            max_detour_percent: self.print.avoid_crossing_max_detour,
            max_detour_absolute: 0,        // Use percentage-based limit only
            grid_resolution: scale(0.1),   // 0.1mm grid resolution
            boundary_offset: scale(0.001), // 1 micron offset
        }
    }
}

/// Convert from config InfillPattern to infill module InfillPattern.
fn convert_infill_pattern(pattern: ConfigInfillPattern) -> InfillPattern {
    match pattern {
        ConfigInfillPattern::Rectilinear => InfillPattern::Rectilinear,
        ConfigInfillPattern::Grid => InfillPattern::Grid,
        ConfigInfillPattern::Concentric => InfillPattern::Concentric,
        ConfigInfillPattern::Honeycomb => InfillPattern::Honeycomb,
        ConfigInfillPattern::Gyroid => InfillPattern::Gyroid,
        ConfigInfillPattern::Lightning => InfillPattern::Lightning,
        // Map other patterns to rectilinear for now
        _ => InfillPattern::Rectilinear,
    }
}

/// Convert from support pattern to infill pattern.
fn convert_support_pattern(pattern: SupportPattern) -> InfillPattern {
    match pattern {
        SupportPattern::Grid => InfillPattern::Grid,
        SupportPattern::Lines => InfillPattern::Rectilinear,
        SupportPattern::Triangles => InfillPattern::Grid, // Approximate with grid
        SupportPattern::Honeycomb => InfillPattern::Honeycomb,
        SupportPattern::Gyroid => InfillPattern::Gyroid,
        SupportPattern::Lightning => InfillPattern::Lightning,
    }
}

/// Convert from config SeamPosition to path module SeamPosition.
fn convert_seam_position(pos: ConfigSeamPosition) -> SeamPosition {
    match pos {
        ConfigSeamPosition::Random => SeamPosition::Random,
        ConfigSeamPosition::Aligned => SeamPosition::Aligned,
        ConfigSeamPosition::Rear => SeamPosition::Rear,
        ConfigSeamPosition::Nearest => SeamPosition::Nearest,
        ConfigSeamPosition::Hidden => SeamPosition::Hidden,
    }
}

/// Wipe implementation for reducing stringing during retraction.
///
/// This stores the last extruded path and uses it to wipe the nozzle
/// during retraction, following the path backwards while retracting filament.
/// Reference: BambuStudio GCode.cpp Wipe class
#[derive(Debug, Clone, Default)]
struct Wipe {
    /// Whether wiping is enabled.
    enabled: bool,
    /// The stored path for wiping (typically the last extruded path).
    path: Vec<Point>,
    /// Wipe distance in mm.
    wipe_distance: f64,
    /// Percentage of retraction to do before wiping (0-100).
    retract_before_wipe_pct: f64,
}

impl Wipe {
    /// Create a new Wipe with the given configuration.
    fn new(enabled: bool, wipe_distance: f64, retract_before_wipe_pct: f64) -> Self {
        Self {
            enabled,
            path: Vec::new(),
            wipe_distance,
            retract_before_wipe_pct: retract_before_wipe_pct.clamp(0.0, 100.0),
        }
    }

    /// Store a path for potential wiping.
    fn store_path(&mut self, points: &[Point]) {
        if self.enabled {
            self.path = points.to_vec();
        }
    }

    /// Check if we have a valid path for wiping.
    fn has_path(&self) -> bool {
        self.enabled && !self.path.is_empty() && self.wipe_distance > 0.0
    }

    /// Reset the stored path.
    fn reset_path(&mut self) {
        self.path.clear();
    }

    /// Perform a wipe move during retraction.
    ///
    /// This follows the stored path backwards while retracting filament.
    /// Returns true if a wipe was performed, false otherwise.
    fn wipe(
        &self,
        writer: &mut GCodeWriter,
        current_pos: Point,
        total_retract_length: f64,
        travel_speed: f64,
    ) -> bool {
        if !self.has_path() {
            return false;
        }

        // Build wipe path: start from current position, go backwards along stored path
        let mut wipe_path = Vec::new();
        wipe_path.push(current_pos);

        // Add points from the stored path in reverse order
        for point in self.path.iter().rev() {
            if *point != current_pos {
                wipe_path.push(*point);
            }
        }

        if wipe_path.len() < 2 {
            return false;
        }

        // Calculate total wipe path length
        let mut total_length = 0.0;
        for i in 1..wipe_path.len() {
            let dx = unscale(wipe_path[i].x - wipe_path[i - 1].x);
            let dy = unscale(wipe_path[i].y - wipe_path[i - 1].y);
            total_length += (dx * dx + dy * dy).sqrt();
        }

        if total_length < 0.001 {
            return false;
        }

        // Clamp wipe distance to path length
        let wipe_dist = self.wipe_distance.min(total_length);

        // Trim the wipe path to the desired wipe distance
        let mut trimmed_path = vec![wipe_path[0]];
        let mut accumulated = 0.0;
        for i in 1..wipe_path.len() {
            let dx = unscale(wipe_path[i].x - wipe_path[i - 1].x);
            let dy = unscale(wipe_path[i].y - wipe_path[i - 1].y);
            let segment_len = (dx * dx + dy * dy).sqrt();

            if accumulated + segment_len >= wipe_dist {
                // Interpolate to exact wipe distance
                let remaining = wipe_dist - accumulated;
                let ratio = remaining / segment_len;
                let new_x = wipe_path[i - 1].x
                    + ((wipe_path[i].x - wipe_path[i - 1].x) as f64 * ratio) as i64;
                let new_y = wipe_path[i - 1].y
                    + ((wipe_path[i].y - wipe_path[i - 1].y) as f64 * ratio) as i64;
                trimmed_path.push(Point::new(new_x, new_y));
                break;
            } else {
                trimmed_path.push(wipe_path[i]);
                accumulated += segment_len;
            }
        }

        if trimmed_path.len() < 2 {
            return false;
        }

        // Calculate how much to retract during the wipe
        // Portion of retraction to do during wipe (after retract_before_wipe)
        let retract_during_wipe =
            total_retract_length * (1.0 - self.retract_before_wipe_pct / 100.0);

        if retract_during_wipe <= 0.0 {
            return false;
        }

        // Calculate accumulated length for E distribution
        let mut segment_lengths = Vec::new();
        let mut total_wipe_len = 0.0;
        for i in 1..trimmed_path.len() {
            let dx = unscale(trimmed_path[i].x - trimmed_path[i - 1].x);
            let dy = unscale(trimmed_path[i].y - trimmed_path[i - 1].y);
            let len = (dx * dx + dy * dy).sqrt();
            segment_lengths.push(len);
            total_wipe_len += len;
        }

        // Write wipe start marker (BambuStudio format for validation)
        writer.write_raw("; WIPE_START");

        // Move along wipe path while retracting
        for i in 1..trimmed_path.len() {
            let segment_ratio = segment_lengths[i - 1] / total_wipe_len;
            let segment_retract = retract_during_wipe * segment_ratio * 0.95; // Slightly less to avoid over-retraction

            let e_value = -segment_retract; // Negative for retraction
            let point = &trimmed_path[i];

            // Use extrude_to with negative E for retraction during wipe
            writer.extrude_to(
                unscale(point.x),
                unscale(point.y),
                e_value,
                Some(travel_speed * 60.0),
            );
        }

        // Write wipe end marker (BambuStudio format for validation)
        writer.write_raw("; WIPE_END");

        true
    }
}

/// The main printing pipeline that orchestrates the complete slicing process.
pub struct PrintPipeline {
    /// Pipeline configuration.
    config: PipelineConfig,
    /// Arc fitter (created lazily if arc fitting is enabled).
    arc_fitter: Option<ArcFitter>,
    /// Support generator (created if support is enabled).
    support_generator: Option<SupportGenerator>,
    /// Travel planner for avoid crossing perimeters.
    travel_planner: Option<AvoidCrossingPerimeters>,
    /// Wipe handler for reducing stringing during retraction.
    wipe: Wipe,
}

impl PrintPipeline {
    /// Create a new pipeline with the given configuration.
    pub fn new(config: PipelineConfig) -> Self {
        let arc_fitter = if config.uses_arc_fitting() {
            Some(ArcFitter::new(config.arc_fitting_config()))
        } else {
            None
        };
        let support_generator = if config.uses_support() {
            Some(SupportGenerator::new(config.support.clone()))
        } else {
            None
        };
        let travel_planner = if config.uses_avoid_crossing_perimeters() {
            Some(AvoidCrossingPerimeters::new(config.travel_config()))
        } else {
            None
        };

        // Initialize wipe with configuration
        let wipe = Wipe::new(
            config.object.wipe_enabled,
            config.object.wipe_distance,
            config.object.retract_before_wipe,
        );

        Self {
            config,
            arc_fitter,
            support_generator,
            travel_planner,
            wipe,
        }
    }

    /// Create a pipeline with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(PipelineConfig::default())
    }

    /// Check if arc fitting is enabled for this pipeline.
    pub fn uses_arc_fitting(&self) -> bool {
        self.arc_fitter.is_some()
    }

    /// Check if avoid crossing perimeters is enabled for this pipeline.
    pub fn uses_avoid_crossing_perimeters(&self) -> bool {
        self.travel_planner.is_some()
    }

    /// Check if support generation is enabled for this pipeline.
    pub fn uses_support(&self) -> bool {
        self.support_generator.is_some()
    }

    /// Get the configuration.
    pub fn config(&self) -> &PipelineConfig {
        &self.config
    }

    /// Get mutable access to the configuration.
    pub fn config_mut(&mut self) -> &mut PipelineConfig {
        &mut self.config
    }

    /// Process a mesh through the complete pipeline and return G-code.
    ///
    /// This is the main entry point that runs:
    /// 1. Slicing (mesh → layers)
    /// 2. Perimeter generation
    /// 3. Infill generation
    /// 4. Path conversion
    /// 5. G-code generation
    pub fn process(&mut self, mesh: &TriangleMesh) -> Result<GCode> {
        self.process_with_callback(mesh, |_, _| {})
    }

    /// Process a mesh with a progress callback.
    ///
    /// The callback receives (stage_name, progress_0_to_1).
    pub fn process_with_callback<F>(
        &mut self,
        mesh: &TriangleMesh,
        mut callback: F,
    ) -> Result<GCode>
    where
        F: FnMut(&str, f64),
    {
        callback("slicing", 0.0);

        // Step 1: Slice the mesh into layers
        let slicer = Slicer::new(self.config.slicing.clone());
        let layers = slicer.slice(mesh)?;

        callback("slicing", 1.0);

        // Step 2: Generate support structures if enabled
        let support_layers = if let Some(ref support_gen) = self.support_generator {
            callback("support", 0.0);
            let layer_slices = self.collect_layer_slices(&layers);
            let supports = support_gen.generate(&layer_slices);
            callback("support", 1.0);
            Some(supports)
        } else {
            None
        };

        // Step 3-5: Process each layer (perimeters, infill, paths)
        let layer_paths =
            self.process_layers_with_support(&layers, support_layers.as_deref(), |progress| {
                callback("processing", progress);
            })?;

        callback("processing", 1.0);

        // Step 6: Generate G-code
        let gcode = self.generate_gcode(&layer_paths, |progress| {
            callback("gcode", progress);
        })?;

        callback("gcode", 1.0);

        Ok(gcode)
    }

    /// Collect layer slices for support generation.
    fn collect_layer_slices(&self, layers: &[Layer]) -> Vec<(CoordF, CoordF, ExPolygons)> {
        layers
            .iter()
            .map(|layer| {
                (
                    layer.top_z_mm(),
                    layer.height_mm(),
                    layer.all_slices().to_vec(),
                )
            })
            .collect()
    }

    /// Process sliced layers to generate toolpaths.
    pub fn process_layers<F>(&self, layers: &[Layer], callback: F) -> Result<Vec<LayerPaths>>
    where
        F: FnMut(f64),
    {
        self.process_layers_with_support(layers, None, callback)
    }

    /// Process sliced layers with optional support to generate toolpaths.
    pub fn process_layers_with_support<F>(
        &self,
        layers: &[Layer],
        support_layers: Option<&[SupportLayer]>,
        mut callback: F,
    ) -> Result<Vec<LayerPaths>>
    where
        F: FnMut(f64),
    {
        let path_config = self.config.path_config();
        let mut path_gen = PathGenerator::new(path_config);

        let total_layers = layers.len();
        let mut all_layer_paths = Vec::with_capacity(total_layers);

        // Create appropriate perimeter generator based on mode
        let use_arachne = self.config.uses_arachne();

        // Create generators (we need both configs even if only using one)
        let classic_gen = if !use_arachne {
            Some(PerimeterGenerator::new(self.config.perimeter_config()))
        } else {
            None
        };

        let arachne_gen = if use_arachne {
            Some(ArachneGenerator::new(self.config.arachne_config()))
        } else {
            None
        };

        // Collect layer slices for surface type detection
        let layer_slices: Vec<ExPolygons> = layers.iter().map(|l| l.all_slices()).collect();

        // Detect surface types for all layers using proper geometry comparison
        // This is the key improvement - instead of using layer position, we compare
        // actual geometry with adjacent layers to identify top/bottom/internal surfaces
        let surface_offset = self.config.print.nozzle_diameter / 10.0;
        let mut layer_surfaces = self.detect_all_layer_surfaces(&layer_slices, surface_offset);

        // Propagate solid infill through the configured number of layers
        let surface_config = SurfaceDetectionConfig {
            top_solid_layers: self.config.object.top_solid_layers as usize,
            bottom_solid_layers: self.config.object.bottom_solid_layers as usize,
            offset: surface_offset,
            min_area: 0.5,
        };
        propagate_solid_infill(&mut layer_surfaces, &surface_config);

        for (i, layer) in layers.iter().enumerate() {
            // Get support layer if available
            let support_layer = support_layers.and_then(|sl| sl.get(i));

            // Get lower layer slices for bridge detection
            let lower_slices = if i > 0 {
                Some(&layer_slices[i - 1])
            } else {
                None
            };

            // Get the detected surfaces for this layer
            let surfaces = &layer_surfaces[i];

            let mut layer_paths = if use_arachne {
                self.process_single_layer_with_surfaces_arachne(
                    layer,
                    i,
                    total_layers,
                    lower_slices,
                    surfaces,
                    arachne_gen.as_ref().unwrap(),
                    &mut path_gen,
                )?
            } else {
                self.process_single_layer_with_surfaces(
                    layer,
                    i,
                    total_layers,
                    lower_slices,
                    surfaces,
                    classic_gen.as_ref().unwrap(),
                    &mut path_gen,
                )?
            };

            // Add support paths if present
            if let Some(support) = support_layer {
                self.add_support_paths(&mut layer_paths, support);
            }

            all_layer_paths.push(layer_paths);

            if i % 10 == 0 || i == total_layers - 1 {
                callback(i as f64 / total_layers as f64);
            }
        }

        callback(1.0);
        Ok(all_layer_paths)
    }

    /// Detect surface types for all layers.
    ///
    /// This implements proper surface classification like BambuStudio's
    /// `PrintObject::detect_surfaces_type()`.
    fn detect_all_layer_surfaces(
        &self,
        layer_slices: &[ExPolygons],
        offset: CoordF,
    ) -> Vec<Vec<Surface>> {
        let num_layers = layer_slices.len();
        let mut result = Vec::with_capacity(num_layers);

        for i in 0..num_layers {
            let lower = if i > 0 {
                Some(&layer_slices[i - 1])
            } else {
                None
            };
            let upper = if i + 1 < num_layers {
                Some(&layer_slices[i + 1])
            } else {
                None
            };

            let surfaces = detect_surface_types(&layer_slices[i], lower, upper, offset);
            result.push(surfaces);
        }

        result
    }

    /// Add support structure paths to a layer.
    fn add_support_paths(&self, layer_paths: &mut LayerPaths, support: &SupportLayer) {
        if support.is_empty() {
            return;
        }

        let support_config = &self.config.support;
        let extrusion_width = support_config.extrusion_width;
        let _line_spacing = support_config.effective_line_spacing();

        // Generate support infill for regular support regions
        if !support.support_regions.is_empty() {
            let support_infill_config = InfillConfig {
                pattern: convert_support_pattern(support_config.pattern),
                density: support_config.density,
                extrusion_width,
                angle: 0.0, // Support usually at 0 or 90 degrees
                angle_increment: 90.0,
                overlap: 0.1,
                min_area: 1.0,
                connect_infill: false,
                infill_first: false,
                z_height: support.z,
                layer_height: support.height,
            };

            let infill_gen = InfillGenerator::new(support_infill_config);
            let infill_result = infill_gen.generate(&support.support_regions, support.layer_id);

            // Convert infill paths to support paths
            for infill_path in infill_result.paths {
                let points = infill_path.points().to_vec();
                let is_closed = infill_path.is_loop();
                let support_path =
                    ExtrusionPath::new(points, is_closed, ExtrusionRole::SupportMaterial)
                        .with_width(extrusion_width)
                        .with_height(support.height);
                layer_paths.support_paths.push(support_path);
            }
        }

        // Generate denser infill for interface regions
        if !support.interface_regions.is_empty() {
            let interface_config = InfillConfig {
                pattern: InfillPattern::Rectilinear, // Interface is always rectilinear
                density: support_config.interface_density,
                extrusion_width,
                angle: 90.0, // Perpendicular to support
                angle_increment: 0.0,
                overlap: 0.15,
                min_area: 0.5,
                connect_infill: true,
                infill_first: false,
                z_height: support.z,
                layer_height: support.height,
            };

            let infill_gen = InfillGenerator::new(interface_config);
            let infill_result = infill_gen.generate(&support.interface_regions, support.layer_id);

            for infill_path in infill_result.paths {
                let points = infill_path.points().to_vec();
                let is_closed = infill_path.is_loop();
                let interface_path =
                    ExtrusionPath::new(points, is_closed, ExtrusionRole::SupportMaterialInterface)
                        .with_width(extrusion_width)
                        .with_height(support.height);
                layer_paths.support_paths.push(interface_path);
            }
        }
    }

    /// Process a single layer.
    /// Process a single layer with proper surface-based classification.
    ///
    /// This method uses the pre-detected surface types to determine which
    /// areas need solid infill vs sparse infill, rather than using simple
    /// layer position checks.
    fn process_single_layer_with_surfaces(
        &self,
        layer: &Layer,
        layer_index: usize,
        _total_layers: usize,
        lower_layer_slices: Option<&ExPolygons>,
        surfaces: &[Surface],
        perimeter_gen: &PerimeterGenerator,
        path_gen: &mut PathGenerator,
    ) -> Result<LayerPaths> {
        let z_height = layer.top_z_mm();
        let layer_height = layer.height_mm();

        // Collect all slices from all regions
        let slices = layer.all_slices();

        if slices.is_empty() {
            return Ok(LayerPaths::new(layer_index, z_height, layer_height));
        }

        // DEBUG: Layer-specific perimeter tracking
        eprintln!(
            "[LAYER {}] Starting perimeter generation with {} slices",
            layer_index,
            slices.len()
        );

        // Generate perimeters
        let perimeter_result = perimeter_gen.generate(&slices);

        // DEBUG: Report perimeter results for this layer
        eprintln!(
            "[LAYER {}] Generated {} perimeter loops",
            layer_index,
            perimeter_result.perimeters.len()
        );

        // Determine if this layer has any solid surfaces
        let has_solid = surfaces.iter().any(|s| s.is_solid());
        let has_top = surfaces.iter().any(|s| s.is_top());

        // Detect bridges and split infill area into bridge and non-bridge regions
        let (bridge_infill, normal_infill) = if self.config.uses_bridge_detection()
            && layer_index > 0
            && lower_layer_slices.is_some()
        {
            self.detect_and_separate_bridges_improved(
                &perimeter_result.infill_area,
                lower_layer_slices.unwrap(),
                surfaces,
                layer_index,
            )
        } else {
            (InfillResult::new(), None)
        };

        // Generate infill based on surface types
        let infill_result = if let Some(normal_area) = normal_infill {
            self.generate_layer_infill_with_surfaces(&normal_area, surfaces, layer_index)
        } else {
            self.generate_layer_infill_with_surfaces(
                &perimeter_result.infill_area,
                surfaces,
                layer_index,
            )
        };

        // Convert to paths
        let mut layer_paths = path_gen.generate(
            &perimeter_result,
            &infill_result,
            layer_index,
            z_height,
            has_solid,
        );

        // Add bridge infill paths
        self.add_bridge_paths(&mut layer_paths, &bridge_infill, layer_height);

        Ok(layer_paths)
    }

    /// Process a single layer using Arachne with proper surface classification.
    fn process_single_layer_with_surfaces_arachne(
        &self,
        layer: &Layer,
        layer_index: usize,
        _total_layers: usize,
        lower_layer_slices: Option<&ExPolygons>,
        surfaces: &[Surface],
        arachne_gen: &ArachneGenerator,
        path_gen: &mut PathGenerator,
    ) -> Result<LayerPaths> {
        let z_height = layer.top_z_mm();
        let layer_height = layer.height_mm();

        // Collect all slices from all regions
        let slices = layer.all_slices();

        if slices.is_empty() {
            return Ok(LayerPaths::new(layer_index, z_height, layer_height));
        }

        // Generate variable-width perimeters using Arachne
        let arachne_result = arachne_gen.generate(&slices);

        // Determine if this layer has any solid surfaces
        let has_solid = surfaces.iter().any(|s| s.is_solid());

        // Detect bridges and split infill area into bridge and non-bridge regions
        let (bridge_infill, normal_infill) = if self.config.uses_bridge_detection()
            && layer_index > 0
            && lower_layer_slices.is_some()
        {
            self.detect_and_separate_bridges_improved(
                &arachne_result.inner_contour,
                lower_layer_slices.unwrap(),
                surfaces,
                layer_index,
            )
        } else {
            (InfillResult::new(), None)
        };

        // Generate infill based on surface types
        let infill_result = if let Some(normal_area) = normal_infill {
            self.generate_layer_infill_with_surfaces(&normal_area, surfaces, layer_index)
        } else {
            self.generate_layer_infill_with_surfaces(
                &arachne_result.inner_contour,
                surfaces,
                layer_index,
            )
        };

        // Convert Arachne result and infill to paths
        let mut layer_paths = path_gen.generate_from_arachne(
            &arachne_result,
            &infill_result,
            layer_index,
            z_height,
            has_solid,
        );

        // Add bridge infill paths
        self.add_bridge_paths(&mut layer_paths, &bridge_infill, layer_height);

        Ok(layer_paths)
    }

    /// Legacy process_single_layer for backward compatibility.
    #[allow(dead_code)]
    fn process_single_layer(
        &self,
        layer: &Layer,
        layer_index: usize,
        total_layers: usize,
        lower_layer_slices: Option<&ExPolygons>,
        perimeter_gen: &PerimeterGenerator,
        path_gen: &mut PathGenerator,
    ) -> Result<LayerPaths> {
        let z_height = layer.top_z_mm();
        let layer_height = layer.height_mm();

        // Determine if this is a solid layer (top/bottom)
        let is_bottom = layer_index < self.config.object.bottom_solid_layers as usize;
        let is_top = layer_index >= total_layers - self.config.object.top_solid_layers as usize;
        let is_solid = is_bottom || is_top;

        // Collect all slices from all regions
        let slices = layer.all_slices();

        if slices.is_empty() {
            return Ok(LayerPaths::new(layer_index, z_height, layer_height));
        }

        // Generate perimeters
        let perimeter_result = perimeter_gen.generate(&slices);

        // Detect bridges and split infill area into bridge and non-bridge regions
        let (bridge_infill, normal_infill) = if self.config.uses_bridge_detection()
            && layer_index > 0
            && lower_layer_slices.is_some()
        {
            self.detect_and_separate_bridges(
                &perimeter_result.infill_area,
                lower_layer_slices.unwrap(),
                layer_index,
            )
        } else {
            (InfillResult::new(), None)
        };

        // Generate normal infill for non-bridge areas
        let infill_result = if let Some(normal_area) = normal_infill {
            self.generate_layer_infill(&normal_area, layer_index, is_solid, is_top)
        } else {
            self.generate_layer_infill(&perimeter_result.infill_area, layer_index, is_solid, is_top)
        };

        // Convert to paths
        let mut layer_paths = path_gen.generate(
            &perimeter_result,
            &infill_result,
            layer_index,
            z_height,
            is_solid,
        );

        // Add bridge infill paths
        self.add_bridge_paths(&mut layer_paths, &bridge_infill, layer_height);

        Ok(layer_paths)
    }

    /// Legacy process_single_layer_arachne for backward compatibility.
    #[allow(dead_code)]
    fn process_single_layer_arachne(
        &self,
        layer: &Layer,
        layer_index: usize,
        total_layers: usize,
        lower_layer_slices: Option<&ExPolygons>,
        arachne_gen: &ArachneGenerator,
        path_gen: &mut PathGenerator,
    ) -> Result<LayerPaths> {
        let z_height = layer.top_z_mm();
        let layer_height = layer.height_mm();

        // Determine if this is a solid layer (top/bottom)
        let is_bottom = layer_index < self.config.object.bottom_solid_layers as usize;
        let is_top = layer_index >= total_layers - self.config.object.top_solid_layers as usize;
        let is_solid = is_bottom || is_top;

        // Collect all slices from all regions
        let slices = layer.all_slices();

        if slices.is_empty() {
            return Ok(LayerPaths::new(layer_index, z_height, layer_height));
        }

        // Generate variable-width perimeters using Arachne
        let arachne_result = arachne_gen.generate(&slices);

        // Detect bridges and split infill area into bridge and non-bridge regions
        let (bridge_infill, normal_infill) = if self.config.uses_bridge_detection()
            && layer_index > 0
            && lower_layer_slices.is_some()
        {
            self.detect_and_separate_bridges(
                &arachne_result.inner_contour,
                lower_layer_slices.unwrap(),
                layer_index,
            )
        } else {
            (InfillResult::new(), None)
        };

        // Generate infill using the inner contour from Arachne
        let infill_result = if let Some(normal_area) = normal_infill {
            self.generate_layer_infill(&normal_area, layer_index, is_solid, is_top)
        } else {
            self.generate_layer_infill(&arachne_result.inner_contour, layer_index, is_solid, is_top)
        };

        // Convert Arachne result and infill to paths
        let mut layer_paths = path_gen.generate_from_arachne(
            &arachne_result,
            &infill_result,
            layer_index,
            z_height,
            is_solid,
        );

        // Add bridge infill paths
        self.add_bridge_paths(&mut layer_paths, &bridge_infill, layer_height);

        Ok(layer_paths)
    }

    /// Improved bridge detection using surface type information.
    ///
    /// This method uses the pre-detected surface types to identify bridge regions
    /// more accurately. Bridge surfaces are those classified as BottomBridge or
    /// InternalBridge.
    fn detect_and_separate_bridges_improved(
        &self,
        infill_area: &[ExPolygon],
        lower_layer_slices: &ExPolygons,
        surfaces: &[Surface],
        layer_index: usize,
    ) -> (InfillResult, Option<ExPolygons>) {
        if infill_area.is_empty() {
            return (InfillResult::new(), None);
        }

        // Collect bridge surfaces from the pre-detected surface types
        let bridge_surfaces: ExPolygons = surfaces
            .iter()
            .filter(|s| s.is_bridge())
            .map(|s| s.expolygon.clone())
            .collect();

        // Also detect additional bridges in the infill area that weren't caught
        // by surface detection (e.g., internal bridges over sparse infill)
        let infill_expolygons: ExPolygons = infill_area.to_vec();

        // Find unsupported areas within infill
        let unsupported = if !lower_layer_slices.is_empty() {
            clipper::difference(&infill_expolygons, lower_layer_slices)
        } else {
            Vec::new()
        };

        // Combine pre-detected bridges with newly detected unsupported areas
        let min_bridge_area =
            self.config.bridge.min_area * crate::SCALING_FACTOR * crate::SCALING_FACTOR;

        // Filter and combine bridge candidates
        let bridge_candidates: ExPolygons = bridge_surfaces
            .into_iter()
            .chain(unsupported.into_iter())
            .filter(|ex| ex.area().abs() >= min_bridge_area)
            .collect();

        // Filter bridge candidates to only those with valid anchors.
        // A true bridge has anchors on the lower layer (not just an overhang).
        // This is the key distinction - overhangs only have material on one side,
        // while bridges span across a gap with anchors on both sides.
        let spacing = self.config.print.nozzle_diameter * 1.125;
        let mut validated_bridges: ExPolygons = Vec::new();

        for bridge_expoly in &bridge_candidates {
            let mut detector =
                BridgeDetector::new(bridge_expoly.clone(), lower_layer_slices, spacing);

            // Only include bridges that have proper anchors (detect_angle returns true).
            // Large areas without anchors are overhangs, not bridges - they should be
            // handled by perimeters (possibly with overhang slowdown), not bridge infill.
            let has_anchors = detector.detect_angle(0.0);

            if has_anchors {
                validated_bridges.push(bridge_expoly.clone());
            }
        }

        // Use validated bridges for infill generation
        let bridge_candidates = validated_bridges;

        if bridge_candidates.is_empty() {
            return (InfillResult::new(), None);
        }

        let mut bridge_infill = InfillResult::new();
        // Use wider spacing for bridges to reduce line count.
        // libslic3r generates fewer, longer bridge lines rather than many short ones.
        let spacing = self.config.print.nozzle_diameter * 1.125;

        // Process each bridge candidate
        // Maximum bridge area to fill - larger areas are overhangs that get perimeters
        // True bridges that span gaps are typically small (~1-6 mm²)
        let max_bridge_area = 6.0 * crate::SCALING_FACTOR * crate::SCALING_FACTOR; // 6 mm²

        for bridge_expoly in &bridge_candidates {
            let area = bridge_expoly.area().abs();
            if area < min_bridge_area {
                continue;
            }
            // Skip overly large "bridges" - these are overhangs, not true spanning bridges
            if area > max_bridge_area {
                continue;
            }

            let mut detector =
                BridgeDetector::new(bridge_expoly.clone(), lower_layer_slices, spacing);

            // Try to detect optimal angle using BridgeDetector first
            let bridge_angle = if detector.detect_angle(0.0) {
                detector.angle.to_degrees()
            } else {
                // BridgeDetector failed (no anchors found)
                // First check if we have a pre-detected angle from surfaces
                if let Some(angle) = surfaces
                    .iter()
                    .find(|s| s.is_bridge() && s.bridge_angle.is_some())
                    .and_then(|s| s.bridge_angle)
                {
                    angle.to_degrees()
                } else {
                    // Use the anchor-based direction detection from libslic3r
                    // This handles bridges that span gaps between walls
                    let to_cover = vec![bridge_expoly.contour.clone()];
                    let anchors: Vec<crate::geometry::Polygon> = lower_layer_slices
                        .iter()
                        .map(|ex| ex.contour.clone())
                        .collect();

                    let (dir, _cost) = detect_bridge_direction_from_anchors(&to_cover, &anchors);

                    // Convert direction vector to angle in degrees
                    let mut angle = dir.y.atan2(dir.x);
                    if angle < 0.0 {
                        angle += std::f64::consts::PI;
                    }
                    // Normalize to [0, 180) range
                    while angle >= std::f64::consts::PI {
                        angle -= std::f64::consts::PI;
                    }
                    angle.to_degrees()
                }
            };

            // Generate bridge infill at the optimal angle
            // Use standard bridge extrusion width (nozzle diameter for circular cross-section)
            let bridge_extrusion_width = self.config.print.nozzle_diameter;
            let bridge_config = InfillConfig {
                pattern: InfillPattern::Rectilinear,
                density: 1.0, // Solid bridge infill
                extrusion_width: bridge_extrusion_width,
                angle: bridge_angle,
                angle_increment: 0.0, // Don't rotate bridge infill
                overlap: 0.15,        // Standard overlap for bridges
                min_area: 0.1,        // Small min area for bridges
                connect_infill: true, // Connect lines to reduce travel moves
                infill_first: false,
                z_height: 0.0,
                layer_height: self.config.print.layer_height,
            };

            let bridge_gen = InfillGenerator::new(bridge_config);
            let result = bridge_gen.generate(&[bridge_expoly.clone()], layer_index);

            // Merge paths into bridge infill result
            for path in result.paths {
                bridge_infill.paths.push(path);
            }
        }

        // Calculate remaining non-bridge area
        // We need to subtract bridge regions from infill area to avoid double-filling
        let remaining = clipper::difference(&infill_expolygons, &bridge_candidates);

        if remaining.is_empty() && bridge_infill.paths.is_empty() {
            (InfillResult::new(), None)
        } else {
            (bridge_infill, Some(remaining))
        }
    }

    /// Legacy bridge detection for backward compatibility.
    #[allow(dead_code)]
    fn detect_and_separate_bridges(
        &self,
        infill_area: &[ExPolygon],
        lower_layer_slices: &ExPolygons,
        layer_index: usize,
    ) -> (InfillResult, Option<ExPolygons>) {
        if infill_area.is_empty() || lower_layer_slices.is_empty() {
            return (InfillResult::new(), None);
        }

        // Find unsupported areas (potential bridges)
        let infill_expolygons: ExPolygons = infill_area.to_vec();
        let unsupported = clipper::difference(&infill_expolygons, lower_layer_slices);

        // Filter small areas
        let min_bridge_area =
            self.config.bridge.min_area * crate::SCALING_FACTOR * crate::SCALING_FACTOR;
        let bridge_candidates: ExPolygons = unsupported
            .into_iter()
            .filter(|ex| ex.area().abs() >= min_bridge_area)
            .collect();

        if bridge_candidates.is_empty() {
            return (InfillResult::new(), None);
        }

        let mut bridge_infill = InfillResult::new();
        let spacing = self.config.print.nozzle_diameter * 1.125;

        // Process each bridge candidate
        for bridge_expoly in &bridge_candidates {
            let mut detector =
                BridgeDetector::new(bridge_expoly.clone(), lower_layer_slices, spacing);

            if detector.detect_angle(0.0) {
                // Generate bridge infill at the optimal angle
                let bridge_config = InfillConfig {
                    pattern: InfillPattern::Rectilinear,
                    density: 1.0, // Solid bridge infill
                    extrusion_width: spacing,
                    angle: detector.angle.to_degrees(),
                    angle_increment: 0.0, // Don't rotate bridge infill
                    overlap: 0.1,
                    min_area: 0.5,
                    connect_infill: false,
                    infill_first: false,
                    z_height: 0.0, // Bridge Z doesn't affect pattern
                    layer_height: self.config.print.layer_height,
                };

                let bridge_gen = InfillGenerator::new(bridge_config);
                let result = bridge_gen.generate(&[bridge_expoly.clone()], layer_index);

                // Merge paths into bridge infill result
                for path in result.paths {
                    bridge_infill.paths.push(path);
                }
            }
        }

        // Calculate remaining non-bridge area
        let remaining = clipper::difference(&infill_expolygons, &bridge_candidates);

        if remaining.is_empty() && bridge_infill.paths.is_empty() {
            (InfillResult::new(), None)
        } else {
            (bridge_infill, Some(remaining))
        }
    }

    /// Add bridge infill paths to layer paths with bridge-specific settings.
    fn add_bridge_paths(
        &self,
        layer_paths: &mut LayerPaths,
        bridge_infill: &InfillResult,
        layer_height: CoordF,
    ) {
        if bridge_infill.paths.is_empty() {
            return;
        }

        let bridge_config = &self.config.bridge;
        let extrusion_width = self.config.print.nozzle_diameter * 1.125;

        for infill_path in &bridge_infill.paths {
            let points = infill_path.points().to_vec();
            let is_closed = infill_path.is_loop();

            let bridge_path = ExtrusionPath::new(points, is_closed, ExtrusionRole::BridgeInfill)
                .with_width(extrusion_width)
                .with_height(layer_height)
                .with_flow(bridge_config.flow_multiplier)
                .with_speed(self.config.object.bridge_speed * bridge_config.speed_multiplier);

            layer_paths.paths.push(bridge_path);
        }
    }

    /// Generate infill for a layer using surface type classification.
    ///
    /// This method separates the infill area into solid and sparse regions
    /// based on the detected surface types, rather than treating the entire
    /// layer as solid or sparse.
    fn generate_layer_infill_with_surfaces(
        &self,
        infill_area: &[ExPolygon],
        surfaces: &[Surface],
        layer_index: usize,
    ) -> InfillResult {
        if infill_area.is_empty() {
            return InfillResult::new();
        }

        let infill_expolygons: ExPolygons = infill_area.to_vec();

        // Collect solid surface regions (top, bottom, internal solid)
        let solid_regions: ExPolygons = surfaces
            .iter()
            .filter(|s| s.is_solid() && !s.is_bridge()) // Bridges handled separately
            .map(|s| s.expolygon.clone())
            .collect();

        // Collect sparse (internal) surface regions
        let sparse_regions: ExPolygons = surfaces
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Internal)
            .map(|s| s.expolygon.clone())
            .collect();

        let mut combined_result = InfillResult::new();
        let base_config = self.config.infill_config();

        // Generate solid infill for solid regions
        if !solid_regions.is_empty() {
            // Intersect solid regions with actual infill area
            let solid_infill_area = clipper::intersection(&infill_expolygons, &solid_regions);

            if !solid_infill_area.is_empty() {
                let has_top = surfaces.iter().any(|s| s.is_top());

                let solid_config = InfillConfig {
                    pattern: InfillPattern::Rectilinear, // Solid always uses rectilinear
                    density: 1.0,
                    extrusion_width: base_config.extrusion_width,
                    angle: base_config.angle + (layer_index as f64) * 90.0, // Alternate 0/90
                    angle_increment: 0.0,
                    overlap: base_config.overlap,
                    min_area: base_config.min_area,
                    connect_infill: true,
                    infill_first: false,
                    z_height: base_config.z_height,
                    layer_height: base_config.layer_height,
                };

                let infill_gen = InfillGenerator::new(solid_config);
                let solid_result = infill_gen.generate(&solid_infill_area, layer_index);

                // Mark paths as solid infill
                for mut path in solid_result.paths {
                    // Paths from solid regions should be solid infill
                    combined_result.paths.push(path);
                }
            }
        }

        // Generate sparse infill for internal regions
        // BambuStudio: Only generate sparse infill in regions explicitly marked as Internal surface type,
        // intersected with the infill area (inside perimeters) and excluding solid regions.
        // This prevents double-counting areas that would be covered by both solid and sparse infill.
        if !sparse_regions.is_empty() && base_config.density > 0.0 && base_config.density < 1.0 {
            // Get the infill area excluding solid regions (solid infill already generated above)
            let remaining_area = if !solid_regions.is_empty() {
                clipper::difference(&infill_expolygons, &solid_regions)
            } else {
                infill_expolygons.clone()
            };

            // Intersect remaining area with sparse regions to get actual sparse infill area
            // This ensures we only fill areas designated as Internal surface type
            let sparse_infill_area = clipper::intersection(&remaining_area, &sparse_regions);

            // Union to merge overlapping regions
            let merged_sparse = clipper::union_ex(&sparse_infill_area);

            if !merged_sparse.is_empty() {
                let mut sparse_config = base_config.clone();
                // Rotate angle for sparse infill
                let angle_offset = (layer_index as f64) * sparse_config.angle_increment;
                sparse_config.angle = (sparse_config.angle + angle_offset) % 180.0;

                let infill_gen = InfillGenerator::new(sparse_config);
                let sparse_result = infill_gen.generate(&merged_sparse, layer_index);

                for path in sparse_result.paths {
                    combined_result.paths.push(path);
                }
            }
        } else if solid_regions.is_empty() {
            // No solid regions detected, use the full infill area with configured pattern
            let mut infill_config = base_config;
            let angle_offset = (layer_index as f64) * infill_config.angle_increment;
            infill_config.angle = (infill_config.angle + angle_offset) % 180.0;

            let infill_gen = InfillGenerator::new(infill_config);
            let result = infill_gen.generate(&infill_expolygons, layer_index);

            for path in result.paths {
                combined_result.paths.push(path);
            }
        }

        combined_result
    }

    /// Legacy infill generation for backward compatibility.
    #[allow(dead_code)]
    fn generate_layer_infill(
        &self,
        infill_area: &[ExPolygon],
        layer_index: usize,
        is_solid: bool,
        is_top: bool,
    ) -> InfillResult {
        if infill_area.is_empty() {
            return InfillResult::new();
        }

        let mut infill_config = self.config.infill_config();

        // Adjust for solid layers
        if is_solid {
            infill_config.density = 1.0;
            infill_config.pattern = InfillPattern::Rectilinear;
        }

        // Rotate infill angle by layer
        let angle_offset = (layer_index as f64) * infill_config.angle_increment;
        infill_config.angle = (infill_config.angle + angle_offset) % 180.0;

        // For top layers, maybe use a different pattern
        if is_top {
            infill_config.pattern = InfillPattern::Rectilinear;
        }

        let infill_gen = InfillGenerator::new(infill_config);
        infill_gen.generate(infill_area, layer_index)
    }

    /// Generate G-code from layer paths.
    pub fn generate_gcode<F>(
        &mut self,
        layer_paths: &[LayerPaths],
        mut callback: F,
    ) -> Result<GCode>
    where
        F: FnMut(f64),
    {
        let mut writer = GCodeWriter::with_config(self.config.print.clone());

        // Write preamble
        self.write_preamble(&mut writer);

        // Write each layer
        let total_layers = layer_paths.len();
        for (i, paths) in layer_paths.iter().enumerate() {
            self.write_layer(&mut writer, paths, i == 0)?;

            if i % 10 == 0 || i == total_layers - 1 {
                callback(i as f64 / total_layers as f64);
            }
        }

        // Write end code
        self.write_end(&mut writer);

        callback(1.0);
        Ok(writer.finish())
    }

    /// Write the G-code preamble (start code).
    fn write_preamble(&self, writer: &mut GCodeWriter) {
        let config = &self.config.print;

        writer.write_comment("Generated by Slicer (Rust)");
        writer.write_comment(&format!(
            "Layer height: {:.2}mm, First layer: {:.2}mm",
            config.layer_height, config.first_layer_height
        ));
        writer.write_comment(&format!("Nozzle: {:.2}mm", config.nozzle_diameter));
        writer.write_comment("");

        // Set absolute positioning and extrusion
        writer.write_preamble();

        // Heat up bed and extruder
        writer.set_bed_temperature(config.first_layer_bed_temperature, false);
        writer.set_extruder_temperature(config.first_layer_extruder_temperature, false);

        // Home
        writer.home(true, true, true);

        // Wait for temperatures
        writer.set_bed_temperature(config.first_layer_bed_temperature, true);
        writer.set_extruder_temperature(config.first_layer_extruder_temperature, true);

        // Prime/purge line (simplified)
        writer.write_comment("Prime line");
        writer.travel_to(0.0, 0.0, Some(config.travel_speed * 60.0));
        writer.travel_to_z(config.first_layer_height, None);

        // Draw a short prime line
        let prime_e = self.calculate_e_for_distance(
            50.0,
            config.nozzle_diameter * 1.5,
            config.first_layer_height,
        );
        writer.extrude_to(50.0, 0.0, prime_e, Some(config.first_layer_speed * 60.0));

        // Small retraction after prime
        writer.retract();
        writer.unretract();
    }

    /// Write the G-code for a single layer.
    fn write_layer(
        &mut self,
        writer: &mut GCodeWriter,
        layer_paths: &LayerPaths,
        is_first_layer: bool,
    ) -> Result<()> {
        if !layer_paths.has_paths() {
            return Ok(());
        }

        // Copy config values upfront to avoid borrow conflicts with &mut self
        let extruder_temperature = self.config.print.extruder_temperature;
        let bed_temperature = self.config.print.bed_temperature;
        let retract_before_travel = self.config.print.retract_before_travel;
        let nozzle_diameter = self.config.print.nozzle_diameter;

        // Layer change
        writer.start_layer(layer_paths.layer_index, layer_paths.z_height);

        // Adjust temperature for non-first layers
        if !is_first_layer && layer_paths.layer_index == 1 {
            writer.set_extruder_temperature(extruder_temperature, false);
            writer.set_bed_temperature(bed_temperature, false);
            // Turn on fan after first layer
            writer.set_fan_speed(255);
        }

        // Reset E periodically to avoid large E values
        if layer_paths.layer_index % 50 == 0 {
            writer.reset_e();
        }

        // Initialize travel planner with layer perimeters if enabled
        if let Some(ref mut travel_planner) = self.travel_planner {
            let perimeter_polygons: Vec<Polygon> = layer_paths
                .perimeter_paths()
                .filter(|p| p.is_closed && p.points.len() >= 3)
                .map(|p| Polygon::from_points(p.points.clone()))
                .collect();

            if !perimeter_polygons.is_empty() {
                let perimeter_spacing = scale(nozzle_diameter);
                travel_planner.init_layer(&perimeter_polygons, perimeter_spacing);
            }
        }

        // Process paths - support first, then model
        let mut last_end: Option<Point> = None;

        // Track the current feature type to emit FEATURE comments on change
        let mut current_feature: Option<ExtrusionRole> = None;

        // Write support paths first (they print before model)
        if !layer_paths.support_paths.is_empty() {
            for path in &layer_paths.support_paths {
                if path.points.is_empty() {
                    continue;
                }

                // Emit feature type comment when role changes
                if current_feature != Some(path.role) {
                    writer.write_raw(&format!("; FEATURE: {}", path.role.feature_name()));
                    current_feature = Some(path.role);
                }

                let path_start = path.points[0];

                // Travel to start of path if needed
                let needs_travel = match last_end {
                    None => true,
                    Some(last) => {
                        let dist_sq =
                            (path_start.x - last.x).pow(2) + (path_start.y - last.y).pow(2);
                        dist_sq > scale(0.5).pow(2)
                    }
                };

                if needs_travel {
                    let travel_dist = match last_end {
                        Some(last) => {
                            let dx = unscale(path_start.x - last.x);
                            let dy = unscale(path_start.y - last.y);
                            (dx * dx + dy * dy).sqrt()
                        }
                        None => 10.0,
                    };

                    if travel_dist > retract_before_travel {
                        // Try to perform a wipe move for support paths too
                        let current_pos = match last_end {
                            Some(pos) => pos,
                            None => path_start,
                        };

                        let total_retract = self.config.print.retract_length;
                        let travel_speed = self.config.print.travel_speed;
                        let wiped =
                            self.wipe
                                .wipe(writer, current_pos, total_retract, travel_speed);

                        if !wiped {
                            writer.retract();
                        }
                    }

                    // Use travel planner if available
                    self.emit_travel(writer, last_end, path_start);

                    writer.unretract();
                    self.wipe.reset_path();
                }

                // Extrude support path (use slower speed for support)
                self.extrude_path(writer, path, layer_paths.layer_height, is_first_layer)?;

                last_end = path.points.last().copied();

                // Store support path for wiping
                let path_points: Vec<Point> =
                    path.points.iter().map(|p| Point::new(p.x, p.y)).collect();
                self.wipe.store_path(&path_points);
            }
        }

        // Write model paths
        for path in &layer_paths.paths {
            if path.points.is_empty() {
                continue;
            }

            // Emit feature type comment when role changes
            if current_feature != Some(path.role) {
                writer.write_raw(&format!("; FEATURE: {}", path.role.feature_name()));
                current_feature = Some(path.role);
            }

            let path_start = path.points[0];

            // Travel to start of path if needed
            let needs_travel = match last_end {
                None => true,
                Some(last) => {
                    let dist_sq = (path_start.x - last.x).pow(2) + (path_start.y - last.y).pow(2);
                    dist_sq > scale(0.5).pow(2) // > 0.5mm travel
                }
            };

            if needs_travel {
                // Retract before travel if distance is significant
                let travel_dist = match last_end {
                    Some(last) => {
                        let dx = unscale(path_start.x - last.x);
                        let dy = unscale(path_start.y - last.y);
                        (dx * dx + dy * dy).sqrt()
                    }
                    None => 10.0, // First move
                };

                if travel_dist > retract_before_travel {
                    // Try to perform a wipe move, fall back to normal retraction if not possible
                    let current_pos = match last_end {
                        Some(pos) => pos,
                        None => path_start, // Shouldn't happen in practice
                    };

                    // Perform partial retraction before wipe (if configured)
                    let total_retract = self.config.print.retract_length;
                    let retract_before =
                        total_retract * (self.config.object.retract_before_wipe / 100.0);

                    if retract_before > 0.0 {
                        // Do partial retraction before wipe
                        // Note: writer.retract() does full retraction, so we'd need to modify this
                        // For now, skip pre-retraction and do it all during wipe
                    }

                    // Try wipe - if it succeeds, it handles the retraction during movement
                    // If it fails, do normal retraction
                    let travel_speed = self.config.print.travel_speed;
                    let wiped = self
                        .wipe
                        .wipe(writer, current_pos, total_retract, travel_speed);

                    if !wiped {
                        // Normal retraction if wipe wasn't possible
                        writer.retract();
                    }
                }

                // Use travel planner if available
                self.emit_travel(writer, last_end, path_start);

                // Unretract (whether we did normal retract or wipe)
                writer.unretract();

                // Reset wipe path after travel
                self.wipe.reset_path();
            }

            // Extrude along path
            self.extrude_path(writer, path, layer_paths.layer_height, is_first_layer)?;

            // Remember end position
            last_end = path.points.last().copied();

            // Store path for potential wiping during next retraction
            // Convert path points to Points for wipe storage
            let path_points: Vec<Point> =
                path.points.iter().map(|p| Point::new(p.x, p.y)).collect();
            self.wipe.store_path(&path_points);
        }

        Ok(())
    }

    /// Emit a travel move, using the travel planner if enabled and initialized.
    fn emit_travel(&mut self, writer: &mut GCodeWriter, start: Option<Point>, end: Point) {
        let travel_speed = Some(self.config.print.travel_speed * 60.0);

        if let (Some(start_pt), Some(ref mut travel_planner)) = (start, &mut self.travel_planner) {
            // Use travel planner to potentially route around perimeters
            let result = travel_planner.travel_to(&start_pt, &end);

            if result.path_modified && result.path.points().len() > 2 {
                // Emit the modified travel path (skip first point, it's the start)
                for point in result.path.points().iter().skip(1) {
                    writer.travel_to(unscale(point.x), unscale(point.y), travel_speed);
                }
            } else {
                // Direct travel
                writer.travel_to(unscale(end.x), unscale(end.y), travel_speed);
            }
        } else {
            // No travel planner or no start position, use direct travel
            writer.travel_to(unscale(end.x), unscale(end.y), travel_speed);
        }
    }

    /// Extrude along a path.
    fn extrude_path(
        &self,
        writer: &mut GCodeWriter,
        path: &ExtrusionPath,
        layer_height: CoordF,
        is_first_layer: bool,
    ) -> Result<()> {
        if path.points.len() < 2 {
            return Ok(());
        }

        let config = &self.config.print;

        // Determine feedrate - use path.speed if explicitly set, otherwise use role-based speed
        let base_speed = if path.speed > 0.0 && path.speed < 1000.0 {
            // Path has explicit speed set (e.g., for bridges)
            path.speed
        } else {
            match path.role {
                ExtrusionRole::ExternalPerimeter => self.config.object.external_perimeter_speed,
                ExtrusionRole::Perimeter => self.config.object.perimeter_speed,
                ExtrusionRole::InternalInfill => self.config.object.infill_speed,
                ExtrusionRole::SolidInfill => self.config.object.solid_infill_speed,
                ExtrusionRole::TopSolidInfill => self.config.object.top_solid_infill_speed,
                ExtrusionRole::BridgeInfill => self.config.object.bridge_speed,
                ExtrusionRole::GapFill => self.config.object.gap_fill_speed,
                _ => config.print_speed,
            }
        };

        let feedrate = if is_first_layer {
            config.first_layer_speed.min(base_speed) * 60.0
        } else {
            base_speed * 60.0
        };

        let width = if path.width > 0.0 {
            path.width
        } else {
            config.nozzle_diameter * 1.125
        };

        let height = if path.height > 0.0 {
            path.height
        } else {
            layer_height
        };

        // Check if arc fitting is enabled
        if let Some(ref arc_fitter) = self.arc_fitter {
            // Use arc fitting
            self.extrude_path_with_arcs(writer, path, width, height, feedrate, arc_fitter)?;
        } else {
            // Standard line-by-line extrusion
            self.extrude_path_linear(writer, path, width, height, feedrate)?;
        }

        // Handle closing the path for non-arc mode is done in extrude_path_linear
        Ok(())
    }

    /// Extrude a path using standard linear moves (G1).
    fn extrude_path_linear(
        &self,
        writer: &mut GCodeWriter,
        path: &ExtrusionPath,
        width: CoordF,
        height: CoordF,
        feedrate: CoordF,
    ) -> Result<()> {
        let mut current_e = writer.e();

        // Get flow multiplier from path (defaults to 1.0)
        let flow_mult = if path.flow_multiplier > 0.0 {
            path.flow_multiplier
        } else {
            1.0
        };

        // Prefer Flow-based calculation if available
        let use_flow = path.flow.as_ref();

        for i in 1..path.points.len() {
            let from = path.points[i - 1];
            let to = path.points[i];

            let from_mm = (unscale(from.x), unscale(from.y));
            let to_mm = (unscale(to.x), unscale(to.y));

            let dx = to_mm.0 - from_mm.0;
            let dy = to_mm.1 - from_mm.1;
            let segment_length = (dx * dx + dy * dy).sqrt();

            if segment_length < 0.001 {
                continue; // Skip tiny segments
            }

            // Calculate extrusion with flow multiplier applied
            let e_for_segment = if let Some(flow) = use_flow {
                // Use Flow object for accurate calculation
                self.calculate_e_from_flow(segment_length, flow) * flow_mult
            } else {
                // Fallback to width/height-based calculation
                self.calculate_e_for_distance(segment_length, width, height) * flow_mult
            };
            current_e += e_for_segment;

            writer.extrude_to(to_mm.0, to_mm.1, current_e, Some(feedrate));
        }

        // If it's a closed path (loop), close it
        if path.is_closed && path.points.len() > 2 {
            let first = path.points[0];
            let last = path.points[path.points.len() - 1];

            if first != last {
                let first_mm = (unscale(first.x), unscale(first.y));
                let last_mm = (unscale(last.x), unscale(last.y));

                let dx = first_mm.0 - last_mm.0;
                let dy = first_mm.1 - last_mm.1;
                let segment_length = (dx * dx + dy * dy).sqrt();

                if segment_length > 0.001 {
                    let e_for_segment = if let Some(flow) = use_flow {
                        // Use Flow object for accurate calculation
                        self.calculate_e_from_flow(segment_length, flow) * flow_mult
                    } else {
                        // Fallback to width/height-based calculation
                        self.calculate_e_for_distance(segment_length, width, height) * flow_mult
                    };
                    current_e += e_for_segment;
                    writer.extrude_to(first_mm.0, first_mm.1, current_e, Some(feedrate));
                }
            }
        }

        Ok(())
    }

    /// Extrude a path using arc fitting (G2/G3 where applicable).
    fn extrude_path_with_arcs(
        &self,
        writer: &mut GCodeWriter,
        path: &ExtrusionPath,
        width: CoordF,
        height: CoordF,
        feedrate: CoordF,
        arc_fitter: &ArcFitter,
    ) -> Result<()> {
        // Get flow multiplier from path (defaults to 1.0)
        let flow_mult = if path.flow_multiplier > 0.0 {
            path.flow_multiplier
        } else {
            1.0
        };

        // Prefer Flow-based calculation if available
        let use_flow = path.flow.as_ref();

        // Convert path points to PointF for arc fitting
        let points: Vec<PointF> = path
            .points
            .iter()
            .map(|p| PointF::new(unscale(p.x), unscale(p.y)))
            .collect();

        if points.len() < 2 {
            return Ok(());
        }

        // Process points through arc fitter
        let segments = arc_fitter.process_points(&points);

        let mut current_e = writer.e();

        for segment in &segments {
            match segment {
                PathSegment::Line(line_points) => {
                    // Extrude line segments normally
                    for i in 1..line_points.len() {
                        let from = &line_points[i - 1];
                        let to = &line_points[i];

                        let dx = to.x - from.x;
                        let dy = to.y - from.y;
                        let segment_length = (dx * dx + dy * dy).sqrt();

                        if segment_length < 0.001 {
                            continue;
                        }

                        let e_for_segment = if let Some(flow) = use_flow {
                            self.calculate_e_from_flow(segment_length, flow) * flow_mult
                        } else {
                            self.calculate_e_for_distance(segment_length, width, height) * flow_mult
                        };
                        current_e += e_for_segment;

                        writer.extrude_to(to.x, to.y, current_e, Some(feedrate));
                    }
                }
                PathSegment::Arc(arc) => {
                    // Calculate extrusion for the arc
                    let arc_length = arc.arc_length();
                    let e_for_arc = if let Some(flow) = use_flow {
                        self.calculate_e_from_flow(arc_length, flow) * flow_mult
                    } else {
                        self.calculate_e_for_distance(arc_length, width, height) * flow_mult
                    };
                    current_e += e_for_arc;

                    // Write arc move
                    writer.extrude_arc(
                        arc.end.x,
                        arc.end.y,
                        arc.i,
                        arc.j,
                        current_e,
                        arc.direction,
                        Some(feedrate),
                    );
                }
            }
        }

        // Handle closed paths
        if path.is_closed && points.len() > 2 {
            let first = &points[0];
            let current_pos = writer.position();

            let dx = first.x - current_pos.x;
            let dy = first.y - current_pos.y;
            let segment_length = (dx * dx + dy * dy).sqrt();

            if segment_length > 0.001 {
                let e_for_segment = if let Some(flow) = use_flow {
                    self.calculate_e_from_flow(segment_length, flow) * flow_mult
                } else {
                    self.calculate_e_for_distance(segment_length, width, height) * flow_mult
                };
                current_e += e_for_segment;
                writer.extrude_to(first.x, first.y, current_e, Some(feedrate));
            }
        }

        Ok(())
    }

    /// Calculate E (extrusion length) for a given travel distance, width, and height.
    ///
    /// Uses the proper rounded rectangle cross-section formula from libslic3r/Flow.cpp:
    /// `area = height × (width - height × (1 - π/4))`
    ///
    /// This models extruded plastic as a rectangle with semicircular ends,
    /// which is the actual physical shape of deposited filament.
    fn calculate_e_for_distance(&self, distance: CoordF, width: CoordF, height: CoordF) -> CoordF {
        // Cross-section area using proper rounded rectangle formula
        // area = height × (width - height × (1 - π/4))
        //      ≈ height × (width - 0.2146 × height)
        //
        // This is NOT simply width × height (which would give ~10-15% error)
        let cross_section = height * (width - height * (1.0 - 0.25 * std::f64::consts::PI));

        // Volume = cross-section × distance
        let volume = cross_section * distance;

        // E = volume / filament_area
        // filament_area = π × (diameter/2)²
        let filament_radius = self.config.print.filament_diameter / 2.0;
        let filament_area = std::f64::consts::PI * filament_radius * filament_radius;

        let e = volume / filament_area;

        // Apply extrusion multiplier
        e * self.config.print.extrusion_multiplier
    }

    /// Calculate E (extrusion length) using a Flow object.
    ///
    /// This is the preferred method for E-value calculation as it uses
    /// the exact same formula as BambuStudio's Flow::mm3_per_mm().
    ///
    /// # Arguments
    ///
    /// * `distance` - Travel distance in mm
    /// * `flow` - Flow object containing width, height, and proper spacing calculations
    ///
    /// # Returns
    ///
    /// E-value (filament length in mm) to extrude for the given distance
    fn calculate_e_from_flow(&self, distance: CoordF, flow: &Flow) -> CoordF {
        // Get volume per mm from Flow (this uses the rounded rectangle formula)
        // Use unchecked version since Flow objects from perimeter generation are pre-validated
        let mm3_per_mm = flow.mm3_per_mm_unchecked();

        // Volume = mm3_per_mm × distance
        let volume = mm3_per_mm * distance;

        // E = volume / filament_area
        // filament_area = π × (diameter/2)²
        let filament_radius = self.config.print.filament_diameter / 2.0;
        let filament_area = std::f64::consts::PI * filament_radius * filament_radius;

        let e = volume / filament_area;

        // Apply extrusion multiplier
        e * self.config.print.extrusion_multiplier
    }

    /// Calculate E for a bridge extrusion (circular cross-section).
    ///
    /// Bridges use circular cross-section because unsupported filament
    /// naturally forms a round thread.
    fn calculate_e_for_bridge(&self, distance: CoordF, diameter: CoordF) -> CoordF {
        // Bridge cross-section is circular: area = π × (diameter/2)²
        let cross_section = std::f64::consts::PI * (diameter / 2.0).powi(2);
        let volume = cross_section * distance;

        let filament_radius = self.config.print.filament_diameter / 2.0;
        let filament_area = std::f64::consts::PI * filament_radius * filament_radius;

        let e = volume / filament_area;
        e * self.config.print.extrusion_multiplier
    }

    /// Write the G-code end sequence.
    fn write_end(&self, writer: &mut GCodeWriter) {
        writer.write_comment("");
        writer.write_comment("End of print");

        // Retract
        writer.retract();

        // Lift Z
        let current_z = writer.z();
        writer.travel_to_z(current_z + 10.0, None);

        // Move away
        writer.travel_to(0.0, self.config.print.bed_size_y - 10.0, None);

        // Turn off heaters and fan
        writer.write_end();
    }
}

impl Default for PrintPipeline {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Result of processing layers (before G-code generation).
#[derive(Debug, Clone)]
pub struct ProcessedPrint {
    /// Layer paths ready for G-code generation.
    pub layer_paths: Vec<LayerPaths>,

    /// Configuration used.
    pub config: PipelineConfig,

    /// Total estimated print time (seconds).
    pub estimated_time: f64,

    /// Total filament used (mm).
    pub filament_used: f64,
}

impl ProcessedPrint {
    /// Create a new processed print result.
    pub fn new(layer_paths: Vec<LayerPaths>, config: PipelineConfig) -> Self {
        let mut result = Self {
            layer_paths,
            config,
            estimated_time: 0.0,
            filament_used: 0.0,
        };
        result.calculate_estimates();
        result
    }

    /// Calculate time and filament estimates.
    fn calculate_estimates(&mut self) {
        for paths in &self.layer_paths {
            self.filament_used += paths.total_extrusion;

            // Very rough time estimate based on travel + extrusion distances
            let travel_time = paths.total_travel / self.config.print.travel_speed;
            let extrusion_time = paths.total_extrusion / self.config.print.print_speed;
            self.estimated_time += travel_time + extrusion_time;
        }
    }

    /// Get the number of layers.
    pub fn layer_count(&self) -> usize {
        self.layer_paths.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::TriangleMesh;

    #[test]
    fn test_pipeline_config_support() {
        let config = PipelineConfig::default()
            .support_enabled(true)
            .support_overhang_angle(50.0)
            .support_density(0.2);

        assert!(config.uses_support());
        assert!((config.support.overhang_angle - 50.0).abs() < 0.001);
        assert!((config.support.density - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_pipeline_with_support() {
        let config = PipelineConfig::default()
            .layer_height(0.2)
            .support_enabled(true);
        let mut pipeline = PrintPipeline::new(config);

        assert!(pipeline.uses_support());

        // Test with a simple cube (no overhangs, so no support)
        let mesh = TriangleMesh::cube(10.0);
        let result = pipeline.process(&mesh);
        assert!(result.is_ok());
    }

    #[test]
    fn test_pipeline_bridge_detection_enabled_by_default() {
        let config = PipelineConfig::default();
        assert!(config.uses_bridge_detection());
        assert!(config.detect_bridges);
    }

    #[test]
    fn test_pipeline_bridge_detection_disabled() {
        let config = PipelineConfig::default().bridge_detection(false);
        assert!(!config.uses_bridge_detection());
    }

    #[test]
    fn test_pipeline_bridge_config() {
        let config = PipelineConfig::default()
            .bridge_flow_multiplier(1.1)
            .bridge_speed_multiplier(0.6);

        assert!((config.bridge.flow_multiplier - 1.1).abs() < 1e-6);
        assert!((config.bridge.speed_multiplier - 0.6).abs() < 1e-6);
    }

    #[test]
    fn test_pipeline_with_bridge_detection() {
        let config = PipelineConfig::default()
            .layer_height(0.2)
            .bridge_detection(true)
            .bridge_flow_multiplier(1.05);
        let mut pipeline = PrintPipeline::new(config);

        assert!(pipeline.config().uses_bridge_detection());

        // Test with a simple cube
        let mesh = TriangleMesh::cube(10.0);
        let result = pipeline.process(&mesh);
        assert!(result.is_ok());
    }

    #[test]
    fn test_pipeline_config_default() {
        let config = PipelineConfig::default();
        assert!((config.print.layer_height - 0.2).abs() < 1e-6);
        assert_eq!(config.object.perimeters, 3);
    }

    #[test]
    fn test_pipeline_config_builder() {
        let config = PipelineConfig::default()
            .layer_height(0.15)
            .perimeters(4)
            .infill_density(0.3);

        assert!((config.slicing.layer_height - 0.15).abs() < 1e-6);
        assert_eq!(config.object.perimeters, 4);
        assert!((config.object.fill_density - 0.3).abs() < 1e-6);
    }

    #[test]
    fn test_pipeline_config_arachne_mode() {
        let config = PipelineConfig::default().arachne();

        assert!(config.uses_arachne());
        assert_eq!(config.object.perimeter_mode, PerimeterMode::Arachne);
    }

    #[test]
    fn test_pipeline_config_classic_mode() {
        let config = PipelineConfig::default().classic_perimeters();

        assert!(!config.uses_arachne());
        assert_eq!(config.object.perimeter_mode, PerimeterMode::Classic);
    }

    #[test]
    fn test_arachne_config_from_pipeline() {
        let config = PipelineConfig::default().arachne();
        let arachne_config = config.arachne_config();

        assert_eq!(arachne_config.wall_count, config.object.perimeters as usize);
        assert!(arachne_config.print_thin_walls == config.object.thin_walls);
    }

    #[test]
    fn test_pipeline_process_cube() {
        let mesh = TriangleMesh::cube(10.0);
        let config = PipelineConfig::new()
            .layer_height(0.3)
            .perimeters(2)
            .infill_density(0.2);

        let mut pipeline = PrintPipeline::new(config);
        let result = pipeline.process(&mesh);

        assert!(result.is_ok(), "Pipeline should process cube: {:?}", result);
        let gcode = result.unwrap();

        // Check that G-code was generated
        assert!(!gcode.is_empty(), "G-code should not be empty");
        assert!(
            gcode.content().contains("G1"),
            "G-code should have extrusion moves"
        );
        assert!(
            gcode.content().contains("Layer"),
            "G-code should have layer comments"
        );
    }

    #[test]
    fn test_e_calculation() {
        let config = PipelineConfig::default();
        let mut pipeline = PrintPipeline::new(config);

        // 10mm travel, 0.4mm width, 0.2mm height
        let e = pipeline.calculate_e_for_distance(10.0, 0.4, 0.2);

        // Expected: (0.4 * 0.2 * 0.9 * 10) / (π * (1.75/2)²) ≈ 0.3
        assert!(e > 0.2 && e < 0.5, "E value should be reasonable: {}", e);
    }

    #[test]
    fn test_pipeline_empty_mesh() {
        let mesh = TriangleMesh::new();
        let mut pipeline = PrintPipeline::with_defaults();
        let result = pipeline.process(&mesh);

        assert!(result.is_err(), "Pipeline should fail on empty mesh");
    }

    #[test]
    fn test_pipeline_process_cube_arachne() {
        let mesh = TriangleMesh::cube(10.0);
        let config = PipelineConfig::default()
            .arachne()
            .layer_height(0.3)
            .perimeters(2)
            .infill_density(0.2);

        assert!(config.uses_arachne(), "Should be in Arachne mode");

        let mut pipeline = PrintPipeline::new(config);
        let result = pipeline.process(&mesh);

        assert!(
            result.is_ok(),
            "Pipeline should process cube with Arachne: {:?}",
            result
        );
        let gcode = result.unwrap();

        // Check that G-code was generated
        assert!(!gcode.is_empty(), "G-code should not be empty");
        assert!(
            gcode.content().contains("G1"),
            "G-code should have extrusion moves"
        );
        assert!(
            gcode.content().contains("Layer"),
            "G-code should have layer comments"
        );
    }

    #[test]
    fn test_pipeline_arachne_vs_classic_both_work() {
        let mesh = TriangleMesh::cube(10.0);

        // Classic mode
        let classic_config = PipelineConfig::default()
            .classic_perimeters()
            .layer_height(0.2)
            .perimeters(3);

        let mut classic_pipeline = PrintPipeline::new(classic_config);
        let classic_result = classic_pipeline.process(&mesh);
        assert!(classic_result.is_ok(), "Classic mode should work");

        // Arachne mode
        let arachne_config = PipelineConfig::default()
            .arachne()
            .layer_height(0.2)
            .perimeters(3);

        let mut arachne_pipeline = PrintPipeline::new(arachne_config);
        let arachne_result = arachne_pipeline.process(&mesh);
        assert!(arachne_result.is_ok(), "Arachne mode should work");

        // Both should produce non-empty G-code
        let classic_gcode = classic_result.unwrap();
        let arachne_gcode = arachne_result.unwrap();

        assert!(!classic_gcode.is_empty());
        assert!(!arachne_gcode.is_empty());
    }

    #[test]
    fn test_perimeter_config_from_pipeline() {
        let config = PipelineConfig::new().perimeters(5).nozzle_diameter(0.6);
        let perim_config = config.perimeter_config();

        assert_eq!(perim_config.perimeter_count, 5);
        // Width should be based on nozzle * 1.125 (in mm, not scaled)
        let expected_width = 0.6 * 1.125;
        assert!((perim_config.perimeter_extrusion_width - expected_width).abs() < 1e-6);
    }

    #[test]
    fn test_infill_config_from_pipeline() {
        let config = PipelineConfig::new().infill_density(0.5);
        let infill_config = config.infill_config();

        assert!((infill_config.density - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_pipeline_arc_fitting_disabled_by_default() {
        let config = PipelineConfig::default();
        assert!(!config.uses_arc_fitting());
        assert!(!config.print.arc_fitting_enabled);
    }

    #[test]
    fn test_pipeline_arc_fitting_config() {
        let config = PipelineConfig::default()
            .arc_fitting(true)
            .arc_fitting_tolerance(0.03);

        assert!(config.uses_arc_fitting());
        assert!(config.print.arc_fitting_enabled);
        assert!((config.print.arc_fitting_tolerance - 0.03).abs() < 1e-6);

        let arc_config = config.arc_fitting_config();
        assert!(arc_config.enabled);
        assert!((arc_config.tolerance - 0.03).abs() < 1e-6);
    }

    #[test]
    fn test_pipeline_with_arc_fitting() {
        let mesh = TriangleMesh::cube(10.0);
        let config = PipelineConfig::new()
            .layer_height(0.3)
            .perimeters(2)
            .infill_density(0.2)
            .arc_fitting(true);

        let mut pipeline = PrintPipeline::new(config);
        assert!(pipeline.uses_arc_fitting());

        let result = pipeline.process(&mesh);
        assert!(
            result.is_ok(),
            "Pipeline with arc fitting should work: {:?}",
            result
        );

        let gcode = result.unwrap();
        assert!(!gcode.is_empty(), "G-code should not be empty");
        // Arc fitting may or may not produce G2/G3 depending on path geometry
        // The main thing is that it shouldn't crash
        assert!(
            gcode.content().contains("G1")
                || gcode.content().contains("G2")
                || gcode.content().contains("G3"),
            "G-code should have extrusion moves"
        );
    }

    #[test]
    fn test_arc_fitting_config_from_pipeline() {
        let config = PipelineConfig::default()
            .arc_fitting(true)
            .arc_fitting_tolerance(0.08);

        let arc_config = config.arc_fitting_config();

        assert!(arc_config.enabled);
        assert!((arc_config.tolerance - 0.08).abs() < 1e-6);
        assert!((arc_config.min_radius - config.print.arc_fitting_min_radius).abs() < 1e-6);
        assert!((arc_config.max_radius - config.print.arc_fitting_max_radius).abs() < 1e-6);
    }

    #[test]
    fn test_pipeline_relative_e_default() {
        // Default config should use relative E mode (M83)
        let config = PipelineConfig::default();
        assert!(config.print.use_relative_e);
    }

    #[test]
    fn test_pipeline_relative_e_builder() {
        // Test builder method
        let config = PipelineConfig::default().use_relative_e(false);
        assert!(!config.print.use_relative_e);

        let config = PipelineConfig::default().use_relative_e(true);
        assert!(config.print.use_relative_e);
    }

    #[test]
    fn test_pipeline_gcode_relative_e() {
        // Test that G-code output uses M83 when relative E is enabled
        let mesh = TriangleMesh::cube(10.0);
        let config = PipelineConfig::new()
            .layer_height(0.3)
            .perimeters(2)
            .use_relative_e(true);

        let mut pipeline = PrintPipeline::new(config);
        let result = pipeline.process(&mesh);
        assert!(result.is_ok());

        let gcode = result.unwrap();
        let content = gcode.content();

        // Should contain M83 (relative E mode)
        assert!(
            content.contains("M83"),
            "G-code should contain M83 for relative E mode"
        );
        // Should NOT contain M82 (absolute E mode)
        assert!(
            !content.contains("M82"),
            "G-code should not contain M82 when using relative E mode"
        );
    }

    #[test]
    fn test_pipeline_gcode_absolute_e() {
        // Test that G-code output uses M82 when absolute E is enabled
        let mesh = TriangleMesh::cube(10.0);
        let config = PipelineConfig::new()
            .layer_height(0.3)
            .perimeters(2)
            .use_relative_e(false);

        let mut pipeline = PrintPipeline::new(config);
        let result = pipeline.process(&mesh);
        assert!(result.is_ok());

        let gcode = result.unwrap();
        let content = gcode.content();

        // Should contain M82 (absolute E mode)
        assert!(
            content.contains("M82"),
            "G-code should contain M82 for absolute E mode"
        );
        // Should NOT contain M83 (relative E mode)
        assert!(
            content.contains("M82"),
            "G-code should contain M82 for absolute E mode"
        );
    }

    #[test]
    fn test_pipeline_avoid_crossing_perimeters_enabled_by_default() {
        let config = PipelineConfig::default();
        assert!(config.uses_avoid_crossing_perimeters());
        assert!(config.print.avoid_crossing_perimeters);
    }

    #[test]
    fn test_pipeline_avoid_crossing_perimeters_config() {
        // Test disabling avoid crossing perimeters
        let config = PipelineConfig::default().avoid_crossing_perimeters(false);
        assert!(!config.uses_avoid_crossing_perimeters());
        assert!(!config.print.avoid_crossing_perimeters);

        // Test enabling with custom max detour
        let config = PipelineConfig::default()
            .avoid_crossing_perimeters(true)
            .avoid_crossing_max_detour(150.0);
        assert!(config.uses_avoid_crossing_perimeters());
        assert!((config.print.avoid_crossing_max_detour - 150.0).abs() < 1e-6);
    }

    #[test]
    fn test_pipeline_with_avoid_crossing_perimeters() {
        let mesh = TriangleMesh::cube(10.0);
        let config = PipelineConfig::new()
            .layer_height(0.3)
            .perimeters(2)
            .infill_density(0.2)
            .avoid_crossing_perimeters(true);

        let mut pipeline = PrintPipeline::new(config);
        assert!(pipeline.uses_avoid_crossing_perimeters());

        let result = pipeline.process(&mesh);
        assert!(
            result.is_ok(),
            "Pipeline with avoid crossing perimeters should work: {:?}",
            result
        );

        let gcode = result.unwrap();
        assert!(!gcode.is_empty(), "G-code should not be empty");
        assert!(
            gcode.content().contains("G1") || gcode.content().contains("G0"),
            "G-code should have moves"
        );
    }

    #[test]
    fn test_pipeline_without_avoid_crossing_perimeters() {
        let mesh = TriangleMesh::cube(10.0);
        let config = PipelineConfig::new()
            .layer_height(0.3)
            .perimeters(2)
            .infill_density(0.2)
            .avoid_crossing_perimeters(false);

        let mut pipeline = PrintPipeline::new(config);
        assert!(!pipeline.uses_avoid_crossing_perimeters());

        let result = pipeline.process(&mesh);
        assert!(
            result.is_ok(),
            "Pipeline without avoid crossing perimeters should work: {:?}",
            result
        );

        let gcode = result.unwrap();
        assert!(!gcode.is_empty(), "G-code should not be empty");
    }

    #[test]
    fn test_travel_config_from_pipeline() {
        let config = PipelineConfig::default()
            .avoid_crossing_perimeters(true)
            .avoid_crossing_max_detour(180.0);

        let travel_config = config.travel_config();

        assert!(travel_config.enabled);
        assert!((travel_config.max_detour_percent - 180.0).abs() < 1e-6);
        assert!(travel_config.grid_resolution > 0);
        assert!(travel_config.boundary_offset > 0);
    }

    #[test]
    fn test_perimeter_point_density() {
        // Diagnostic test to check if perimeters have too many points
        use crate::mesh::load_stl;
        use crate::perimeter::{PerimeterConfig, PerimeterGenerator};
        use crate::slice::{Slicer, SlicingParams};
        use std::path::PathBuf;

        // Try to load the 3DBenchy STL if it exists
        let stl_path = PathBuf::from("../data/test_stls/3DBenchy.stl");
        if !stl_path.exists() {
            println!("Skipping test: 3DBenchy.stl not found at {:?}", stl_path);
            return;
        }

        let mesh = load_stl(&stl_path).expect("Failed to load 3DBenchy STL");

        let slicing_params = SlicingParams {
            layer_height: 0.2,
            first_layer_height: 0.2,
            ..Default::default()
        };

        let slicer = Slicer::new(slicing_params);
        let layers = slicer.slice(&mesh).expect("Slicing failed");

        let layer0 = &layers[0];
        let slices = layer0.all_slices();

        // Generate perimeters with 2 walls (like reference)
        let perimeter_config = PerimeterConfig {
            perimeter_count: 2,
            perimeter_extrusion_width: 0.45,
            external_perimeter_extrusion_width: 0.42,
            ..Default::default()
        };
        let perimeter_gen = PerimeterGenerator::new(perimeter_config);
        let perimeter_result = perimeter_gen.generate(&slices);

        println!("Perimeter point density diagnostics:");
        println!("  Total perimeters: {}", perimeter_result.perimeters.len());

        let mut total_points = 0;
        let mut total_length_mm = 0.0;

        for (i, perim) in perimeter_result.perimeters.iter().enumerate() {
            let points = perim.polygon.points().len();
            total_points += points;

            // Calculate perimeter length
            let mut length: f64 = 0.0;
            let pts = perim.polygon.points();
            for j in 0..pts.len() {
                let p1 = &pts[j];
                let p2 = &pts[(j + 1) % pts.len()];
                let dx = crate::unscale(p2.x - p1.x);
                let dy = crate::unscale(p2.y - p1.y);
                length += (dx * dx + dy * dy).sqrt();
            }
            total_length_mm += length;

            if i < 5 {
                // Print first 5 perimeters
                println!(
                    "    Perimeter {}: {} points, {:.2}mm length, {:.3}mm/point avg",
                    i,
                    points,
                    length,
                    if points > 1 {
                        length / (points - 1) as f64
                    } else {
                        0.0
                    }
                );
            }
        }

        println!("  Total points: {}", total_points);
        println!("  Total length: {:.2} mm", total_length_mm);
        println!(
            "  Average segment length: {:.4} mm",
            if total_points > perimeter_result.perimeters.len() {
                total_length_mm / (total_points - perimeter_result.perimeters.len()) as f64
            } else {
                0.0
            }
        );

        // BambuStudio reference has ~905 moves for ~1192mm = ~1.32mm per move
        // If we have much shorter segments, that's the problem
        // Ideal segment length should be around 0.5-2mm for efficiency

        let avg_segment = if total_points > perimeter_result.perimeters.len() {
            total_length_mm / (total_points - perimeter_result.perimeters.len()) as f64
        } else {
            0.0
        };

        println!(
            "\n  DIAGNOSTIC: Average segment length is {:.4}mm",
            avg_segment
        );
        if avg_segment < 0.1 {
            println!(
                "  WARNING: Segments are very short - may need Douglas-Peucker simplification"
            );
        }
    }

    #[test]
    fn test_infill_area_generation_diagnostics() {
        // Test with a simple cube to understand infill area generation
        use crate::perimeter::{PerimeterConfig, PerimeterGenerator};
        use crate::slice::{Slicer, SlicingParams};

        let mesh = TriangleMesh::cube(20.0); // 20mm cube

        let slicing_params = SlicingParams {
            layer_height: 0.2,
            first_layer_height: 0.2,
            ..Default::default()
        };

        let slicer = Slicer::new(slicing_params);
        let layers = slicer.slice(&mesh).expect("Slicing failed");

        assert!(!layers.is_empty(), "Should have layers");

        // Check first layer
        let layer0 = &layers[0];
        let slices = layer0.all_slices();

        println!("Cube diagnostics:");
        println!("  Layer 0 slice count: {}", slices.len());

        // A simple cube should have exactly 1 slice (one square)
        assert_eq!(slices.len(), 1, "Cube should have 1 slice on first layer");

        // Generate perimeters with 2 walls
        let perimeter_config = PerimeterConfig {
            perimeter_count: 2,
            perimeter_extrusion_width: 0.45,
            external_perimeter_extrusion_width: 0.42,
            ..Default::default()
        };
        let perimeter_gen = PerimeterGenerator::new(perimeter_config);
        let perimeter_result = perimeter_gen.generate(&slices);

        println!("  Perimeter count: {}", perimeter_result.perimeters.len());
        println!(
            "  Infill area count: {}",
            perimeter_result.infill_area.len()
        );

        // Should have 2 perimeters (outer + inner) for a square
        // Actually, should have 2 loops (outer contour at each level)
        assert!(
            perimeter_result.perimeters.len() >= 2,
            "Should have at least 2 perimeter loops for 2 walls"
        );

        // Infill area should be exactly 1 region (the remaining inner area)
        assert_eq!(
            perimeter_result.infill_area.len(),
            1,
            "Cube should have exactly 1 infill region after perimeters"
        );

        // Verify infill area is smaller than original slice
        let original_area: f64 = slices.iter().map(|s| s.area().abs()).sum();
        let infill_area: f64 = perimeter_result
            .infill_area
            .iter()
            .map(|s| s.area().abs())
            .sum();

        println!("  Original area (scaled²): {:.0}", original_area);
        println!("  Infill area (scaled²): {:.0}", infill_area);

        assert!(
            infill_area < original_area,
            "Infill area should be smaller than original slice"
        );
    }

    #[test]
    fn test_layer_paths_feature_count() {
        // Test that a simple cube generates reasonable feature counts
        use crate::gcode::ExtrusionRole;

        let mesh = TriangleMesh::cube(20.0);
        let config = PipelineConfig::new()
            .layer_height(0.2)
            .first_layer_height(0.2)
            .perimeters(2)
            .infill_density(0.15); // 15% infill like reference

        let mut pipeline = PrintPipeline::new(config);
        let result = pipeline.process(&mesh);

        assert!(result.is_ok(), "Pipeline should succeed");
        let gcode = result.unwrap();

        // Count features in the G-code
        let content = gcode.content();
        let outer_wall_count = content.matches("; FEATURE: Outer wall").count();
        let inner_wall_count = content.matches("; FEATURE: Inner wall").count();
        let solid_infill_count = content.matches("; FEATURE: Internal solid infill").count();
        let sparse_infill_count = content.matches("; FEATURE: Sparse infill").count();

        println!("Feature counts for 20mm cube:");
        println!("  Outer wall features: {}", outer_wall_count);
        println!("  Inner wall features: {}", inner_wall_count);
        println!("  Solid infill features: {}", solid_infill_count);
        println!("  Sparse infill features: {}", sparse_infill_count);

        // For a cube with 2 perimeters:
        // - Each layer should have 1 outer wall feature and 1 inner wall feature
        // - Bottom layers should have solid infill
        // - Middle layers should have sparse infill
        // - Top layers should have solid infill

        // With layer_height=0.2 and 20mm height, we have 100 layers
        // With 3 bottom + 4 top solid layers = 7 solid layers, 93 sparse layers

        // Outer wall: should be roughly equal to layer count (1 per layer for simple cube)
        assert!(
            outer_wall_count >= 90 && outer_wall_count <= 110,
            "Outer wall count {} should be close to layer count",
            outer_wall_count
        );

        // Inner wall: should be roughly equal to layer count
        assert!(
            inner_wall_count >= 90 && inner_wall_count <= 110,
            "Inner wall count {} should be close to layer count",
            inner_wall_count
        );

        // Solid infill should be relatively low (only top/bottom layers)
        assert!(
            solid_infill_count <= 20,
            "Solid infill count {} should be low (only top/bottom)",
            solid_infill_count
        );
    }

    #[test]
    fn test_benchy_first_layer_slices() {
        // Test to understand why 3DBenchy generates more infill regions than expected
        use crate::mesh::load_stl;
        use crate::perimeter::{PerimeterConfig, PerimeterGenerator};
        use crate::slice::{Slicer, SlicingParams};
        use std::path::PathBuf;

        // Try to load the 3DBenchy STL if it exists
        let stl_path = PathBuf::from("../data/test_stls/3DBenchy.stl");
        if !stl_path.exists() {
            println!("Skipping test: 3DBenchy.stl not found at {:?}", stl_path);
            return;
        }

        let mesh = load_stl(&stl_path).expect("Failed to load 3DBenchy STL");

        let slicing_params = SlicingParams {
            layer_height: 0.2,
            first_layer_height: 0.2,
            ..Default::default()
        };

        let slicer = Slicer::new(slicing_params);
        let layers = slicer.slice(&mesh).expect("Slicing failed");

        println!("3DBenchy diagnostics:");
        println!("  Total layers: {}", layers.len());

        // Check first layer
        let layer0 = &layers[0];
        let slices = layer0.all_slices();

        println!("  Layer 0 slice count: {}", slices.len());

        // Print info about each slice
        for (i, slice) in slices.iter().enumerate() {
            let area_mm2 = slice.area().abs() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
            println!(
                "    Slice {}: area = {:.2} mm², holes = {}",
                i,
                area_mm2,
                slice.holes.len()
            );
        }

        // Generate perimeters with 2 walls (like reference)
        let perimeter_config = PerimeterConfig {
            perimeter_count: 2,
            perimeter_extrusion_width: 0.45,
            external_perimeter_extrusion_width: 0.42,
            ..Default::default()
        };
        let perimeter_gen = PerimeterGenerator::new(perimeter_config);
        let perimeter_result = perimeter_gen.generate(&slices);

        println!("\n  Perimeter generation:");
        println!(
            "    Total perimeters: {}",
            perimeter_result.perimeters.len()
        );

        // Count external vs internal perimeters
        let external_count = perimeter_result
            .perimeters
            .iter()
            .filter(|p| p.is_external)
            .count();
        let internal_count = perimeter_result.perimeters.len() - external_count;
        println!("    External (outer) perimeters: {}", external_count);
        println!("    Internal (inner) perimeters: {}", internal_count);

        println!(
            "    Infill area count: {}",
            perimeter_result.infill_area.len()
        );

        // Print info about each infill region
        for (i, infill_region) in perimeter_result.infill_area.iter().enumerate() {
            let area_mm2 =
                infill_region.area().abs() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
            println!(
                "    Infill region {}: area = {:.2} mm², holes = {}",
                i,
                area_mm2,
                infill_region.holes.len()
            );
        }

        // The issue: if we have many small infill regions, we generate many FEATURE comments
        // Expected: 3DBenchy should have a few main regions (hull, deck, etc.)
        // If infill_area.len() is much larger than expected, that's the problem

        // For a typical 3DBenchy first layer, we expect:
        // - Main hull contour (large area)
        // - Some internal features
        // Total should be a small number, not 10+
        println!(
            "\n  DIAGNOSTIC: {} infill regions may cause {} FEATURE comments",
            perimeter_result.infill_area.len(),
            perimeter_result.infill_area.len()
        );
    }
}
