//! # Slicer
//!
//! A Rust rewrite of the BambuStudio core slicing algorithm.
//!
//! This library provides a complete 3D printing slicing pipeline:
//! - STL mesh loading and processing
//! - Layer slicing with configurable layer heights
//! - Perimeter generation (including Arachne variable-width)
//! - Infill pattern generation
//! - Support structure generation
//! - G-code generation
//!
//! ## Example
//!
//! ```rust,ignore
//! use slicer::{Mesh, SlicingConfig, Slicer};
//!
//! let mesh = Mesh::from_stl("model.stl")?;
//! let config = SlicingConfig::default();
//! let slicer = Slicer::new(config);
//! let gcode = slicer.slice(&mesh)?;
//! gcode.write_to_file("output.gcode")?;
//! ```

// Core modules
pub mod adhesion;
pub mod bridge;
pub mod clipper;
pub mod config;
pub mod edge_grid;
pub mod flow;
pub mod gcode;
pub mod geometry;
pub mod infill;
pub mod mcp;
pub mod mesh;
pub mod perimeter;
pub mod pipeline;
pub mod print;
pub mod profiles;
pub mod slice;
pub mod support;
pub mod travel;

// Re-export commonly used types
pub use config::{PerimeterMode, PrintConfig, PrintObjectConfig, PrintRegionConfig};
pub use flow::{
    support_material_1st_layer_flow, support_material_flow, support_material_interface_flow,
    support_transition_flow, Flow, FlowError, FlowResult, FlowRole, BRIDGE_EXTRA_SPACING,
};
pub use gcode::{
    ExtrusionPath, ExtrusionRole, GCode, GCodeWriter, LayerPaths, PathConfig, PathGenerator,
    SeamPosition,
};

// Re-export adhesion types
pub use adhesion::{
    BrimConfig, BrimGenerator, BrimResult, BrimType, RaftConfig, RaftGenerator, RaftLayer,
    RaftLayerType, RaftResult, SkirtConfig, SkirtGenerator, SkirtResult,
};

// Re-export cooling types
pub use gcode::cooling::{CoolingBuffer, CoolingConfig, CoolingMove, CoolingResult};

// Re-export G-code validation types
pub use gcode::validation::{
    validate_gcode_files, validate_gcode_files_with_config, FeatureStats, FeatureType,
    IssueCategory, IssueSeverity, LayerValidation, ReportFormat, ScoreBreakdown, ValidationConfig,
    ValidationIssue, ValidationReport, ValidationSummary,
};

// Re-export G-code comparison types
pub use gcode::compare::{
    compare_gcode, compare_gcode_files, ComparisonConfig, ComparisonResult, GCodeComparator,
    GCodeMove, LayerComparison, LayerInfo as GCodeLayerInfo, MoveComparison, ParsedGCode,
};

// Re-export pressure equalizer types
pub use gcode::pressure_equalizer::{
    PressureEqualizer, PressureEqualizerConfig, PressureEqualizerStats,
};

// Re-export ironing types
pub use gcode::ironing::{
    generate_ironing, should_iron_layer, IroningConfig, IroningGenerator, IroningPath,
    IroningResult, IroningType,
};

// Re-export spiral vase types
pub use gcode::spiral_vase::{SpiralPoint, SpiralVase, SpiralVaseConfig};

// Re-export seam placer types
pub use gcode::seam_placer::{
    create_seam_placer, place_seam, EnforcedBlockedSeamPoint, LayerOutline, LayerSeams, Perimeter,
    PerimeterOutline, Point3f, SeamCandidate, SeamPlacer, SeamPlacerConfig, SeamPlacerStats,
    SeamPositionMode,
};

// Re-export wipe tower types
pub use gcode::wipe_tower::{
    align_ceil, align_floor, align_round, is_valid_gcode, BedShape, BlockDepthInfo, BoxCoordinates,
    Extrusion as WipeTowerExtrusion, FilamentParameters, GCodeFlavor as WipeTowerGCodeFlavor,
    LimitFlow, NozzleChangeResult, ToolChangeInfo, ToolChangeResult, Vec2f, WipeShape, WipeTower,
    WipeTowerBlock, WipeTowerConfig, WipeTowerLayerInfo, WipeTowerWriter,
};

// Re-export tool ordering types
pub use gcode::tool_ordering::{
    calculate_flush_volume, find_optimal_ordering_exhaustive, generate_all_orderings,
    optimize_extruder_sequence, CustomGCodeItem, CustomGCodeType, ExtrusionRoleType,
    FilamentChangeMode, FilamentChangeStats, FilamentMapMode, FlushMatrix, LayerTools,
    ToolOrdering, ToolOrderingConfig, WipingExtrusions,
};

// Re-export multi-material coordination types
pub use gcode::multi_material::{
    MultiMaterialConfig, MultiMaterialCoordinator, MultiMaterialLayer, MultiMaterialPlan,
    ToolChange, WipeTowerBounds,
};

// Re-export profiles types
pub use profiles::{
    // Slice configuration
    AdhesionSettings,
    AdvancedSettings,
    // Printer profile components
    BedConfig,
    BedTemperature,
    BuildVolume,
    Clearance,
    CoolingConfig as PrinterCoolingConfig,
    EnclosureConfig,
    EnvironmentSettings,
    ExtruderConfig,
    FilamentCooling,
    FilamentProfile,
    FlowSettings,
    GCodeConfig,
    GCodeSettings as SliceGCodeSettings,
    InfillSettings,
    LayerLimits,
    MachineLimits,
    MultiMaterialSettings,
    NozzleSpec,
    NozzleTemperature,
    PerimeterSettings,
    PhysicalProperties,
    PrinterFeatures,
    PrinterProfile,
    ProfileError,
    ProfileMetadata,
    ProfileRegistry,
    ProfileResult,
    // Common components
    ProfileSource,
    QualitySettings,
    RetractionConfig as ProfileRetractionConfig,
    SliceConfig,
    SpecialProperties,
    SpeedSettings,
    SupportSettings as SliceSupportSettings,
    // Filament profile components
    TemperatureSettings,
};

// Re-export elephant foot compensation
pub use geometry::elephant_foot::{ElephantFootCompensator, ElephantFootConfig};
pub use geometry::{BoundingBox, BoundingBox3, ExPolygon, Line, Point, Point3, Polygon, Polyline};

// Re-export path simplification
pub use geometry::simplify::{
    douglas_peucker, douglas_peucker_polygon, douglas_peucker_polyline, simplify_comprehensive,
    simplify_polygon, simplify_polygon_comprehensive, simplify_polygons, simplify_polyline,
    simplify_polyline_comprehensive, simplify_polylines, SimplifyConfig, MESHFIX_MAXIMUM_DEVIATION,
    MESHFIX_MAXIMUM_RESOLUTION,
};

// Re-export AABB tree types
pub use geometry::{
    closest_point_on_triangle, ray_box_intersect, ray_triangle_intersect, AABBClosestPointResult,
    AABBNode, AABBTree, IndexedTriangleSet, RayHit, Vec3, AABB3,
};
pub use mesh::{Triangle, TriangleMesh};
pub use print::{Print, PrintObject};
pub use slice::{Layer, Slicer, SlicingParams};

// Re-export adaptive layer heights
pub use slice::{
    compute_adaptive_heights, compute_adaptive_heights_with_quality, AdaptiveHeightsConfig,
    AdaptiveLayerHeight, AdaptiveSlicing, FaceZ, SlopeErrorMetric,
};

// Re-export clipper operations
pub use clipper::{
    difference, grow, intersection, offset_expolygon, offset_expolygons, offset_polygon,
    offset_polygons, shrink, union, union_ex, xor, OffsetJoinType, OffsetType,
};

// Re-export perimeter generation
pub use perimeter::{
    compute_infill_area, generate_perimeters, generate_perimeters_with, PerimeterConfig,
    PerimeterGenerator, PerimeterLoop, PerimeterResult,
};

// Re-export fuzzy skin
pub use perimeter::fuzzy_skin::{
    apply_fuzzy_skin_extrusion, apply_fuzzy_skin_polygon, fuzzy_extrusion_line,
    fuzzy_extrusion_line_params, fuzzy_polygon, fuzzy_polygon_params, fuzzy_polyline,
    should_fuzzify, FuzzySkinConfig,
};

// Re-export Arachne variable-width perimeter generation
pub use perimeter::arachne::{
    generate_arachne_walls, generate_arachne_walls_with_width, ArachneConfig, ArachneGenerator,
    ArachneResult, BeadingCalculator, BeadingResult, BeadingStrategy, ExtrusionJunction,
    ExtrusionLine, VariableWidthLines,
};

// Re-export pipeline
pub use pipeline::{PipelineConfig, PrintPipeline, ProcessedPrint};

// Re-export infill generation
pub use infill::{
    generate_concentric_infill, generate_grid_infill, generate_gyroid_infill,
    generate_honeycomb_infill, generate_infill, generate_infill_with_density,
    generate_solid_infill, InfillConfig, InfillGenerator, InfillPath, InfillPattern, InfillResult,
};

// Re-export adaptive infill
pub use infill::{
    build_octree, generate_adaptive_infill, generate_adaptive_infill_with_density,
    AdaptiveInfillConfig, AdaptiveInfillGenerator, AdaptiveInfillResult, CubeProperties, Octree,
    Vec3d,
};

// Re-export 3D honeycomb infill
pub use infill::{
    generate_honeycomb_3d, Honeycomb3DConfig, Honeycomb3DGenerator, Honeycomb3DResult,
};

// Re-export Cross Hatch infill
pub use infill::{
    generate_cross_hatch, generate_cross_hatch_with_angle, CrossHatchConfig, CrossHatchGenerator,
    CrossHatchResult,
};

// Re-export plan path infill (space-filling curves)
pub use infill::{
    generate_archimedean_chords, generate_hilbert_curve, generate_octagram_spiral, PlanPathConfig,
    PlanPathGenerator, PlanPathPattern, PlanPathResult,
};

// Re-export floating concentric infill
pub use infill::{
    generate_floating_concentric, generate_floating_concentric_with_config,
    FloatingConcentricConfig, FloatingConcentricGenerator, FloatingConcentricResult,
    FloatingThickLine, FloatingThickPolyline,
};

// Re-export support generation
pub use support::{
    sample_overhang_points, SupportConfig, SupportGenerator, SupportLayer, SupportPattern,
    SupportType, TreeBranch, TreeSupportGenerator,
};

// Re-export tree support 3D types
pub use support::tree_model_volumes::{
    find_nearest_safe_position, is_safe_position, point_inside_polygons, AvoidanceType,
    RadiusLayerKey, RadiusLayerPolygonCache, TreeModelVolumes, TreeModelVolumesConfig,
    COLLISION_RESOLUTION, EXPONENTIAL_FACTOR, EXPONENTIAL_THRESHOLD,
};
pub use support::tree_support_3d::{
    LayerSupportElements, LineInformation, LineInformations, SupportElements, TreeSupport3D,
    TreeSupport3DConfig, TreeSupport3DResult,
};
pub use support::tree_support_settings::{
    AreaIncreaseSettings, AvoidanceTypeCompact, InterfacePreference, LineStatus, ParentIndices,
    SupportElement, SupportElementState, SupportElementStateBits, TreeSupportMeshGroupSettings,
    TreeSupportSettings, TREE_CIRCLE_RESOLUTION,
};

// Re-export bridge detection
pub use bridge::{
    detect_bridges, detect_bridging_direction, detect_internal_bridges, generate_bridge_infill,
    Bridge, BridgeConfig, BridgeDetector, InternalBridgeConfig, InternalBridgeDetector,
};

// Re-export edge grid
pub use edge_grid::{ClosestPointResult, Contour, EdgeGrid, Intersection};

// Re-export travel planning
pub use travel::{AvoidCrossingPerimeters, TravelConfig, TravelResult};

/// Coordinate type used throughout the slicer.
/// Using i64 for integer coordinates (scaled by SCALING_FACTOR) to avoid floating-point issues.
pub type Coord = i64;

/// Floating-point coordinate type for unscaled values.
pub type CoordF = f64;

/// Scaling factor: coordinates are stored as integers scaled by this factor.
/// 1 unit = 1 nanometer, so 1mm = 1_000_000 units.
/// This matches BambuStudio/PrusaSlicer's internal scaling.
pub const SCALING_FACTOR: f64 = 1_000_000.0;

/// Scale a floating-point coordinate to integer.
#[inline]
pub fn scale(v: CoordF) -> Coord {
    (v * SCALING_FACTOR).round() as Coord
}

/// Unscale an integer coordinate to floating-point.
#[inline]
pub fn unscale(v: Coord) -> CoordF {
    v as CoordF / SCALING_FACTOR
}

/// Scale a floating-point coordinate to integer (same as scale, for compatibility).
#[inline]
pub fn scaled(v: CoordF) -> Coord {
    scale(v)
}

/// Unscale an integer coordinate to floating-point (same as unscale, for compatibility).
#[inline]
pub fn unscaled(v: Coord) -> CoordF {
    unscale(v)
}

/// Result type used throughout the slicer.
pub type Result<T> = std::result::Result<T, Error>;

/// Error type for slicer operations.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Mesh error: {0}")]
    Mesh(String),

    #[error("Slicing error: {0}")]
    Slicing(String),

    #[error("G-code error: {0}")]
    GCode(String),

    #[error("Configuration error: {0}")]
    Config(String),

    #[error("Invalid geometry: {0}")]
    Geometry(String),

    #[error("Cancelled")]
    Cancelled,
}

/// Version information
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scaling() {
        // 1mm should scale to 1_000_000
        assert_eq!(scale(1.0), 1_000_000);

        // And back
        assert!((unscale(1_000_000) - 1.0).abs() < 1e-10);

        // Test sub-millimeter precision
        assert_eq!(scale(0.001), 1_000); // 1 micron
        assert_eq!(scale(0.0001), 100); // 100 nanometers
    }
}
