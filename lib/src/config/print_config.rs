//! Print configuration types.
//!
//! This module provides the main configuration types for controlling
//! the slicing and printing process, mirroring BambuStudio's PrintConfig.

use crate::CoordF;
use serde::{Deserialize, Serialize};
use std::fmt;

/// Main print configuration containing global print settings.
///
/// This encompasses settings that affect the entire print, such as
/// bed size, general print speeds, and global parameters.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PrintConfig {
    // === Bed Configuration ===
    /// Bed size X (mm).
    pub bed_size_x: CoordF,
    /// Bed size Y (mm).
    pub bed_size_y: CoordF,
    /// Print origin X offset (mm).
    pub print_origin_x: CoordF,
    /// Print origin Y offset (mm).
    pub print_origin_y: CoordF,

    // === Layer Heights ===
    /// Default layer height (mm).
    pub layer_height: CoordF,
    /// First layer height (mm).
    pub first_layer_height: CoordF,

    // === Speeds (mm/s) ===
    /// Default print speed.
    pub print_speed: CoordF,
    /// Travel move speed.
    pub travel_speed: CoordF,
    /// First layer speed.
    pub first_layer_speed: CoordF,

    // === Temperatures ===
    /// Extruder temperature (°C).
    pub extruder_temperature: u32,
    /// First layer extruder temperature (°C).
    pub first_layer_extruder_temperature: u32,
    /// Bed temperature (°C).
    pub bed_temperature: u32,
    /// First layer bed temperature (°C).
    pub first_layer_bed_temperature: u32,

    // === Retraction ===
    /// Retraction length (mm).
    pub retract_length: CoordF,
    /// Retraction speed (mm/s).
    pub retract_speed: CoordF,
    /// Z lift on retraction (mm).
    pub retract_lift: CoordF,
    /// Minimum travel distance before retraction (mm).
    pub retract_before_travel: CoordF,

    // === Extrusion ===
    /// Nozzle diameter (mm).
    pub nozzle_diameter: CoordF,
    /// Filament diameter (mm).
    pub filament_diameter: CoordF,
    /// Extrusion multiplier (flow rate adjustment).
    pub extrusion_multiplier: CoordF,

    // === Skirt/Brim ===
    /// Number of skirt loops.
    pub skirt_loops: u32,
    /// Skirt distance from object (mm).
    pub skirt_distance: CoordF,
    /// Skirt minimum length (mm).
    pub skirt_min_length: CoordF,
    /// Brim width (mm), 0 = no brim.
    pub brim_width: CoordF,

    // === Support ===
    /// Enable support structures.
    pub support_enabled: bool,
    /// Support type (normal, tree).
    pub support_type: SupportType,
    /// Support overhang threshold angle (degrees).
    pub support_threshold_angle: CoordF,
    /// Support density (0.0 - 1.0).
    pub support_density: CoordF,

    // === Misc ===
    /// Enable spiral/vase mode.
    pub spiral_vase: bool,
    /// G-code flavor.
    pub gcode_flavor: GCodeFlavor,
    /// Resolution for G-code output (mm).
    pub resolution: CoordF,

    // === Extrusion Mode ===
    /// Use relative extrusion mode (M83).
    /// When true, E values are relative (incremental) rather than absolute.
    /// Most modern printers and slicers use relative E mode (M83).
    /// BambuStudio uses relative E mode by default.
    pub use_relative_e: bool,

    // === Arc Fitting ===
    /// Enable arc fitting (G2/G3 commands).
    /// When enabled, sequences of line segments that form arcs will be
    /// converted to G2/G3 arc commands, reducing file size and improving
    /// print quality on firmware that supports arc moves.
    pub arc_fitting_enabled: bool,
    /// Arc fitting tolerance (mm).
    /// Maximum deviation allowed when fitting arcs to line segments.
    /// Smaller values produce more accurate arcs but fewer arc segments.
    pub arc_fitting_tolerance: CoordF,
    /// Minimum arc radius (mm).
    /// Arcs with smaller radii will be kept as line segments.
    pub arc_fitting_min_radius: CoordF,
    /// Maximum arc radius (mm).
    /// Very large radii are essentially straight lines and will be kept as line segments.
    pub arc_fitting_max_radius: CoordF,

    // === Travel Optimization ===
    /// Enable avoid crossing perimeters.
    /// When enabled, travel moves will be routed around perimeter walls
    /// to avoid leaving marks on the printed surface.
    pub avoid_crossing_perimeters: bool,
    /// Maximum detour percentage for avoid crossing perimeters.
    /// If the detour path is longer than this percentage of the direct path,
    /// the direct path will be used instead. Default is 200% (2x direct distance).
    pub avoid_crossing_max_detour: CoordF,
}

impl PrintConfig {
    /// Create a new PrintConfig with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder method: set layer height.
    pub fn layer_height(mut self, height: CoordF) -> Self {
        self.layer_height = height;
        self
    }

    /// Builder method: set first layer height.
    pub fn first_layer_height(mut self, height: CoordF) -> Self {
        self.first_layer_height = height;
        self
    }

    /// Builder method: set nozzle diameter.
    pub fn nozzle_diameter(mut self, diameter: CoordF) -> Self {
        self.nozzle_diameter = diameter;
        self
    }

    /// Builder method: set print speed.
    pub fn print_speed(mut self, speed: CoordF) -> Self {
        self.print_speed = speed;
        self
    }

    /// Builder method: enable/disable support.
    pub fn support(mut self, enabled: bool) -> Self {
        self.support_enabled = enabled;
        self
    }

    /// Builder method: set support type.
    pub fn support_type(mut self, support_type: SupportType) -> Self {
        self.support_type = support_type;
        self
    }

    /// Builder method: set brim width.
    pub fn brim_width(mut self, width: CoordF) -> Self {
        self.brim_width = width;
        self
    }

    /// Validate the configuration.
    pub fn validate(&self) -> Result<(), String> {
        if self.layer_height <= 0.0 {
            return Err("Layer height must be positive".into());
        }
        if self.first_layer_height <= 0.0 {
            return Err("First layer height must be positive".into());
        }
        if self.nozzle_diameter <= 0.0 {
            return Err("Nozzle diameter must be positive".into());
        }
        if self.filament_diameter <= 0.0 {
            return Err("Filament diameter must be positive".into());
        }
        if self.extrusion_multiplier <= 0.0 {
            return Err("Extrusion multiplier must be positive".into());
        }
        Ok(())
    }
}

impl PrintConfig {
    /// Builder: set relative extrusion mode (M83).
    pub fn use_relative_e(mut self, relative: bool) -> Self {
        self.use_relative_e = relative;
        self
    }

    pub fn arc_fitting(mut self, enabled: bool) -> Self {
        self.arc_fitting_enabled = enabled;
        self
    }

    pub fn arc_fitting_tolerance(mut self, tolerance: CoordF) -> Self {
        self.arc_fitting_tolerance = tolerance;
        self
    }

    pub fn arc_fitting_radius(mut self, min: CoordF, max: CoordF) -> Self {
        self.arc_fitting_min_radius = min;
        self.arc_fitting_max_radius = max;
        self
    }

    /// Enable or disable avoid crossing perimeters.
    pub fn avoid_crossing_perimeters(mut self, enabled: bool) -> Self {
        self.avoid_crossing_perimeters = enabled;
        self
    }

    /// Set the maximum detour percentage for avoid crossing perimeters.
    pub fn avoid_crossing_max_detour(mut self, percent: CoordF) -> Self {
        self.avoid_crossing_max_detour = percent;
        self
    }
}

impl Default for PrintConfig {
    fn default() -> Self {
        Self {
            // Bed
            bed_size_x: 256.0,
            bed_size_y: 256.0,
            print_origin_x: 0.0,
            print_origin_y: 0.0,

            // Layer heights
            layer_height: 0.2,
            first_layer_height: 0.2,

            // Speeds
            print_speed: 50.0,
            travel_speed: 150.0,
            first_layer_speed: 20.0,

            // Temperatures
            extruder_temperature: 210,
            first_layer_extruder_temperature: 215,
            bed_temperature: 60,
            first_layer_bed_temperature: 65,

            // Retraction
            retract_length: 1.0,
            retract_speed: 30.0,
            retract_lift: 0.0,
            retract_before_travel: 2.0,

            // Extrusion
            nozzle_diameter: 0.4,
            filament_diameter: 1.75,
            extrusion_multiplier: 1.0,

            // Skirt/Brim
            skirt_loops: 1,
            skirt_distance: 6.0,
            skirt_min_length: 0.0,
            brim_width: 0.0,

            // Support
            support_enabled: false,
            support_type: SupportType::Normal,
            support_threshold_angle: 45.0,
            support_density: 0.15,

            // Misc
            spiral_vase: false,
            gcode_flavor: GCodeFlavor::Marlin,
            resolution: 0.0125,

            // Extrusion Mode
            use_relative_e: true, // Match BambuStudio default (M83)

            // Arc Fitting
            arc_fitting_enabled: false, // Disabled by default for compatibility
            arc_fitting_tolerance: 0.05, // 50 microns
            arc_fitting_min_radius: 0.5, // 0.5mm minimum
            arc_fitting_max_radius: 1000.0, // 1m maximum

            // Travel Optimization
            avoid_crossing_perimeters: true, // Enabled by default (matches BambuStudio)
            avoid_crossing_max_detour: 200.0, // 200% = 2x direct distance
        }
    }
}

impl fmt::Display for PrintConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "PrintConfig(layer={:.2}mm, nozzle={:.2}mm, speed={:.0}mm/s)",
            self.layer_height, self.nozzle_diameter, self.print_speed
        )
    }
}

/// Configuration for a specific print object.
///
/// These settings can be applied per-object to override global settings.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PrintObjectConfig {
    /// Layer height for this object (mm).
    pub layer_height: CoordF,

    /// Number of perimeters/shells.
    pub perimeters: u32,

    /// Number of solid top layers.
    pub top_solid_layers: u32,

    /// Number of solid bottom layers.
    pub bottom_solid_layers: u32,

    /// Infill density (0.0 - 1.0).
    pub fill_density: CoordF,

    /// Infill pattern.
    pub fill_pattern: InfillPattern,

    /// Perimeter speed (mm/s).
    pub perimeter_speed: CoordF,

    /// External perimeter speed (mm/s).
    pub external_perimeter_speed: CoordF,

    /// Infill speed (mm/s).
    pub infill_speed: CoordF,

    /// Solid infill speed (mm/s).
    pub solid_infill_speed: CoordF,

    /// Top solid infill speed (mm/s).
    pub top_solid_infill_speed: CoordF,

    /// Bridge speed (mm/s).
    pub bridge_speed: CoordF,

    /// Gap fill speed (mm/s).
    pub gap_fill_speed: CoordF,

    /// Enable thin walls detection.
    pub thin_walls: bool,

    /// Enable gap fill.
    pub gap_fill: bool,

    /// Detect bridging perimeters.
    pub overhangs: bool,

    /// Slice closing radius (mm).
    pub slice_closing_radius: CoordF,

    /// XY size compensation (mm).
    pub xy_size_compensation: CoordF,

    /// Elephant foot compensation (mm).
    pub elephant_foot_compensation: CoordF,

    /// Seam position preference.
    pub seam_position: SeamPosition,

    /// Enable fuzzy skin.
    pub fuzzy_skin: bool,

    /// Fuzzy skin thickness (mm).
    pub fuzzy_skin_thickness: CoordF,

    /// Fuzzy skin point distance (mm).
    pub fuzzy_skin_point_distance: CoordF,

    /// Enable wipe during retraction to reduce stringing.
    pub wipe_enabled: bool,

    /// Wipe distance (mm) - how far to move while wiping.
    pub wipe_distance: CoordF,

    /// Retract before wipe percentage (0-100).
    /// How much of the retraction happens before vs during the wipe move.
    pub retract_before_wipe: CoordF,

    /// Perimeter generation mode (Classic or Arachne).
    pub perimeter_mode: PerimeterMode,

    /// Minimum bead width for Arachne mode (mm).
    /// Walls thinner than this will not be printed.
    pub arachne_min_bead_width: CoordF,

    /// Minimum feature size for Arachne mode (mm).
    /// Features smaller than this will be ignored.
    pub arachne_min_feature_size: CoordF,

    /// Wall transition length for Arachne mode (mm).
    /// Length over which wall count transitions occur.
    pub arachne_wall_transition_length: CoordF,
}

impl PrintObjectConfig {
    /// Create a new PrintObjectConfig with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder method: set number of perimeters.
    pub fn perimeters(mut self, count: u32) -> Self {
        self.perimeters = count;
        self
    }

    /// Builder method: set infill density.
    pub fn fill_density(mut self, density: CoordF) -> Self {
        self.fill_density = density;
        self
    }

    /// Builder method: set infill pattern.
    pub fn fill_pattern(mut self, pattern: InfillPattern) -> Self {
        self.fill_pattern = pattern;
        self
    }

    /// Builder method: set seam position.
    pub fn seam_position(mut self, position: SeamPosition) -> Self {
        self.seam_position = position;
        self
    }

    /// Builder method: set perimeter mode.
    pub fn perimeter_mode(mut self, mode: PerimeterMode) -> Self {
        self.perimeter_mode = mode;
        self
    }

    /// Builder method: enable Arachne mode.
    pub fn arachne(mut self) -> Self {
        self.perimeter_mode = PerimeterMode::Arachne;
        self
    }

    /// Builder method: set Arachne minimum bead width.
    pub fn arachne_min_bead_width(mut self, width: CoordF) -> Self {
        self.arachne_min_bead_width = width;
        self
    }

    /// Builder method: set Arachne minimum feature size.
    pub fn arachne_min_feature_size(mut self, size: CoordF) -> Self {
        self.arachne_min_feature_size = size;
        self
    }
}

impl Default for PrintObjectConfig {
    fn default() -> Self {
        Self {
            layer_height: 0.2,
            perimeters: 3,
            top_solid_layers: 4,
            bottom_solid_layers: 3,
            fill_density: 0.2,
            fill_pattern: InfillPattern::Grid,
            perimeter_speed: 45.0,
            external_perimeter_speed: 25.0,
            infill_speed: 80.0,
            solid_infill_speed: 40.0,
            top_solid_infill_speed: 30.0,
            bridge_speed: 25.0,
            gap_fill_speed: 20.0,
            thin_walls: true,
            gap_fill: true,
            overhangs: true,
            slice_closing_radius: 0.049,
            xy_size_compensation: 0.0,
            elephant_foot_compensation: 0.0,
            seam_position: SeamPosition::Aligned,
            fuzzy_skin: false,
            fuzzy_skin_thickness: 0.3,
            fuzzy_skin_point_distance: 0.8,
            wipe_enabled: true, // Enable wipe by default (matching BambuStudio)
            wipe_distance: 2.0, // 2mm wipe distance
            retract_before_wipe: 0.0, // 0% - do all retraction during wipe
            perimeter_mode: PerimeterMode::Classic,
            arachne_min_bead_width: 0.1,
            arachne_min_feature_size: 0.1,
            arachne_wall_transition_length: 0.4,
        }
    }
}

impl fmt::Display for PrintObjectConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "PrintObjectConfig(perimeters={}, infill={:.0}%, pattern={:?})",
            self.perimeters,
            self.fill_density * 100.0,
            self.fill_pattern
        )
    }
}

/// Support structure type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SupportType {
    /// Normal/classic support structures.
    #[default]
    Normal,
    /// Tree-style support structures.
    Tree,
    /// Hybrid support (tree + normal).
    Hybrid,
}

/// G-code flavor/dialect.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum GCodeFlavor {
    /// Marlin firmware (most common).
    #[default]
    Marlin,
    /// RepRap firmware.
    RepRap,
    /// Klipper firmware.
    Klipper,
    /// Smoothieware firmware.
    Smoothie,
    /// Sailfish firmware (MakerBot).
    Sailfish,
    /// Mach3/LinuxCNC.
    Mach3,
    /// No extrusion (for CNC/laser).
    NoExtrusion,
}

/// Infill pattern type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum InfillPattern {
    /// Rectilinear lines.
    Rectilinear,
    /// Grid pattern (crossing lines).
    #[default]
    Grid,
    /// Triangular pattern.
    Triangles,
    /// Cubic pattern.
    Cubic,
    /// Honeycomb pattern.
    Honeycomb,
    /// 3D Honeycomb pattern.
    Honeycomb3D,
    /// Gyroid pattern.
    Gyroid,
    /// Concentric pattern.
    Concentric,
    /// Cross-hatch pattern.
    CrossHatch,
    /// Lightning infill.
    Lightning,
    /// Adaptive cubic infill.
    AdaptiveCubic,
}

/// Seam position preference.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SeamPosition {
    /// Random seam position.
    Random,
    /// Nearest to previous layer's seam.
    #[default]
    Aligned,
    /// Rear of the model.
    Rear,
    /// Nearest corner.
    Nearest,
    /// Hidden in corners when possible.
    Hidden,
}

/// Perimeter generation mode.
///
/// Controls how perimeters (walls) are generated for each layer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PerimeterMode {
    /// Classic fixed-width perimeters using polygon offset.
    /// Each perimeter has a constant extrusion width.
    #[default]
    Classic,

    /// Arachne variable-width perimeters.
    /// Adapts extrusion width based on local geometry to better fill
    /// thin walls and narrow features. This produces higher quality
    /// prints for models with thin features.
    Arachne,
}

impl PerimeterMode {
    /// Returns true if this mode uses variable-width extrusion.
    pub fn is_variable_width(&self) -> bool {
        matches!(self, PerimeterMode::Arachne)
    }

    /// Returns the display name for this mode.
    pub fn name(&self) -> &'static str {
        match self {
            PerimeterMode::Classic => "Classic",
            PerimeterMode::Arachne => "Arachne",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_print_config_default() {
        let config = PrintConfig::default();
        assert!((config.layer_height - 0.2).abs() < 1e-6);
        assert!((config.nozzle_diameter - 0.4).abs() < 1e-6);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_print_config_builder() {
        let config = PrintConfig::new()
            .layer_height(0.15)
            .nozzle_diameter(0.6)
            .print_speed(60.0)
            .support(true);

        assert!((config.layer_height - 0.15).abs() < 1e-6);
        assert!((config.nozzle_diameter - 0.6).abs() < 1e-6);
        assert!((config.print_speed - 60.0).abs() < 1e-6);
        assert!(config.support_enabled);
    }

    #[test]
    fn test_print_config_validation() {
        let mut config = PrintConfig::default();
        assert!(config.validate().is_ok());

        config.layer_height = 0.0;
        assert!(config.validate().is_err());

        config.layer_height = 0.2;
        config.nozzle_diameter = -1.0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_print_object_config_default() {
        let config = PrintObjectConfig::default();
        assert_eq!(config.perimeters, 3);
        assert!((config.fill_density - 0.2).abs() < 1e-6);
        assert_eq!(config.fill_pattern, InfillPattern::Grid);
    }

    #[test]
    fn test_print_object_config_builder() {
        let config = PrintObjectConfig::new()
            .perimeters(4)
            .fill_density(0.5)
            .fill_pattern(InfillPattern::Gyroid);

        assert_eq!(config.perimeters, 4);
        assert!((config.fill_density - 0.5).abs() < 1e-6);
        assert_eq!(config.fill_pattern, InfillPattern::Gyroid);
    }

    #[test]
    fn test_infill_pattern_default() {
        assert_eq!(InfillPattern::default(), InfillPattern::Grid);
    }

    #[test]
    fn test_support_type_default() {
        assert_eq!(SupportType::default(), SupportType::Normal);
    }

    #[test]
    fn test_gcode_flavor_default() {
        assert_eq!(GCodeFlavor::default(), GCodeFlavor::Marlin);
    }

    #[test]
    fn test_seam_position_default() {
        assert_eq!(SeamPosition::default(), SeamPosition::Aligned);
    }

    #[test]
    fn test_perimeter_mode_default() {
        assert_eq!(PerimeterMode::default(), PerimeterMode::Classic);
    }

    #[test]
    fn test_perimeter_mode_is_variable_width() {
        assert!(!PerimeterMode::Classic.is_variable_width());
        assert!(PerimeterMode::Arachne.is_variable_width());
    }

    #[test]
    fn test_print_object_config_arachne() {
        let config = PrintObjectConfig::new()
            .perimeter_mode(PerimeterMode::Arachne)
            .arachne_min_bead_width(0.15);

        assert_eq!(config.perimeter_mode, PerimeterMode::Arachne);
        assert!((config.arachne_min_bead_width - 0.15).abs() < 1e-6);
    }

    #[test]
    fn test_print_object_config_arachne_builder() {
        let config = PrintObjectConfig::new().arachne();
        assert_eq!(config.perimeter_mode, PerimeterMode::Arachne);
    }
}
