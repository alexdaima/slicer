//! G-code generation module.
//!
//! This module provides types and functions for generating G-code from
//! sliced layers, mirroring BambuStudio's GCode and GCodeWriter classes.

pub mod arc_fitting;
pub mod compare;
pub mod cooling;
mod generator;
pub mod ironing;
pub mod multi_material;
mod path;
pub mod pressure_equalizer;
pub mod retract_crossing;
pub mod seam_placer;
pub mod spiral_vase;
pub mod tool_ordering;
pub mod validation;
pub mod wipe_tower;
mod writer;

pub use arc_fitting::{
    fit_arcs, fit_arcs_to_points, ArcDirection, ArcFitter, ArcFittingConfig, ArcFittingStats,
    FittedArc, PathSegment,
};
pub use compare::{
    compare_gcode, compare_gcode_files, ComparisonConfig, ComparisonResult, ExtrusionMode,
    ExtrusionTracker, GCodeComparator, GCodeMove, LayerComparison, LayerInfo, MoveComparison,
    ParsedGCode,
};
pub use cooling::{
    estimate_layer_time, CoolingBuffer, CoolingConfig, CoolingMove, CoolingResult,
    PerExtruderAdjustments,
};
pub use generator::{GCode, GCodeStats};
pub use ironing::{
    generate_ironing, should_iron_layer, IroningConfig, IroningGenerator, IroningPath,
    IroningResult, IroningType,
};
pub use multi_material::{
    MultiMaterialConfig, MultiMaterialCoordinator, MultiMaterialLayer, MultiMaterialPlan,
    ToolChange, WipeTowerBounds,
};
pub use path::{
    generate_paths, generate_solid_paths, ExtrusionPath, ExtrusionRole, LayerPaths, PathConfig,
    PathGenerator, SeamPosition,
};
pub use pressure_equalizer::{PressureEqualizer, PressureEqualizerConfig, PressureEqualizerStats};
pub use retract_crossing::{RetractCrossingConfig, RetractDecision, RetractWhenCrossingPerimeters};
pub use seam_placer::{
    create_seam_placer, place_seam, EnforcedBlockedSeamPoint, LayerOutline, LayerSeams, Perimeter,
    PerimeterOutline, Point3f, SeamCandidate, SeamPlacer, SeamPlacerConfig, SeamPlacerStats,
    SeamPositionMode,
};
pub use spiral_vase::{
    extract_vase_perimeter, is_vase_mode_compatible, SpiralLayer, SpiralPoint, SpiralResult,
    SpiralVase, SpiralVaseConfig,
};
pub use tool_ordering::{
    calculate_flush_volume, find_optimal_ordering_exhaustive, generate_all_orderings,
    optimize_extruder_sequence, CustomGCodeItem, CustomGCodeType, ExtrusionRoleType,
    FilamentChangeMode, FilamentChangeStats, FilamentMapMode, FlushMatrix, LayerTools,
    ToolOrdering, ToolOrderingConfig, WipingExtrusions,
};
pub use validation::{
    validate_gcode_files, validate_gcode_files_with_config, FeatureStats, FeatureType,
    IssueCategory, IssueSeverity, LayerValidation, ReportFormat, ScoreBreakdown, ValidationConfig,
    ValidationIssue, ValidationReport, ValidationSummary,
};
pub use wipe_tower::{
    align_ceil, align_floor, align_round, is_valid_gcode, BedShape, BlockDepthInfo, BoxCoordinates,
    Extrusion, FilamentParameters, GCodeFlavor as WipeTowerGCodeFlavor, LimitFlow,
    NozzleChangeResult, ToolChangeInfo, ToolChangeResult, Vec2f, WipeShape, WipeTower,
    WipeTowerBlock, WipeTowerConfig, WipeTowerLayerInfo, WipeTowerWriter,
};
pub use writer::GCodeWriter;

/// G-code command types.
#[derive(Clone, Debug, PartialEq)]
pub enum GCodeCommand {
    /// G0 - Rapid move (travel)
    RapidMove {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        f: Option<f64>,
    },
    /// G1 - Linear move (extrusion)
    LinearMove {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        e: Option<f64>,
        f: Option<f64>,
    },
    /// G2 - Clockwise arc
    ArcCW {
        x: f64,
        y: f64,
        i: f64,
        j: f64,
        e: Option<f64>,
        f: Option<f64>,
    },
    /// G3 - Counter-clockwise arc
    ArcCCW {
        x: f64,
        y: f64,
        i: f64,
        j: f64,
        e: Option<f64>,
        f: Option<f64>,
    },
    /// G28 - Home
    Home { x: bool, y: bool, z: bool },
    /// G90 - Absolute positioning
    AbsolutePositioning,
    /// G91 - Relative positioning
    RelativePositioning,
    /// G92 - Set position
    SetPosition {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        e: Option<f64>,
    },
    /// M82 - Absolute extrusion
    AbsoluteExtrusion,
    /// M83 - Relative extrusion
    RelativeExtrusion,
    /// M104 - Set extruder temperature (no wait)
    SetExtruderTemp { s: u32 },
    /// M109 - Set extruder temperature and wait
    SetExtruderTempWait { s: u32 },
    /// M140 - Set bed temperature (no wait)
    SetBedTemp { s: u32 },
    /// M190 - Set bed temperature and wait
    SetBedTempWait { s: u32 },
    /// M106 - Set fan speed
    SetFanSpeed { s: u32 },
    /// M107 - Fan off
    FanOff,
    /// Comment
    Comment(String),
    /// Raw G-code line
    Raw(String),
}

impl GCodeCommand {
    /// Convert the command to a G-code string.
    pub fn to_gcode(&self) -> String {
        match self {
            GCodeCommand::RapidMove { x, y, z, f } => {
                let mut cmd = String::from("G0");
                if let Some(v) = x {
                    cmd.push_str(&format!(" X{:.3}", v));
                }
                if let Some(v) = y {
                    cmd.push_str(&format!(" Y{:.3}", v));
                }
                if let Some(v) = z {
                    cmd.push_str(&format!(" Z{:.3}", v));
                }
                if let Some(v) = f {
                    cmd.push_str(&format!(" F{:.0}", v));
                }
                cmd
            }
            GCodeCommand::LinearMove { x, y, z, e, f } => {
                let mut cmd = String::from("G1");
                if let Some(v) = x {
                    cmd.push_str(&format!(" X{:.3}", v));
                }
                if let Some(v) = y {
                    cmd.push_str(&format!(" Y{:.3}", v));
                }
                if let Some(v) = z {
                    cmd.push_str(&format!(" Z{:.3}", v));
                }
                if let Some(v) = e {
                    cmd.push_str(&format!(" E{:.5}", v));
                }
                if let Some(v) = f {
                    cmd.push_str(&format!(" F{:.0}", v));
                }
                cmd
            }
            GCodeCommand::ArcCW { x, y, i, j, e, f } => {
                let mut cmd = format!("G2 X{:.3} Y{:.3} I{:.3} J{:.3}", x, y, i, j);
                if let Some(v) = e {
                    cmd.push_str(&format!(" E{:.5}", v));
                }
                if let Some(v) = f {
                    cmd.push_str(&format!(" F{:.0}", v));
                }
                cmd
            }
            GCodeCommand::ArcCCW { x, y, i, j, e, f } => {
                let mut cmd = format!("G3 X{:.3} Y{:.3} I{:.3} J{:.3}", x, y, i, j);
                if let Some(v) = e {
                    cmd.push_str(&format!(" E{:.5}", v));
                }
                if let Some(v) = f {
                    cmd.push_str(&format!(" F{:.0}", v));
                }
                cmd
            }
            GCodeCommand::Home { x, y, z } => {
                let mut cmd = String::from("G28");
                if *x {
                    cmd.push_str(" X");
                }
                if *y {
                    cmd.push_str(" Y");
                }
                if *z {
                    cmd.push_str(" Z");
                }
                cmd
            }
            GCodeCommand::AbsolutePositioning => "G90".to_string(),
            GCodeCommand::RelativePositioning => "G91".to_string(),
            GCodeCommand::SetPosition { x, y, z, e } => {
                let mut cmd = String::from("G92");
                if let Some(v) = x {
                    cmd.push_str(&format!(" X{:.3}", v));
                }
                if let Some(v) = y {
                    cmd.push_str(&format!(" Y{:.3}", v));
                }
                if let Some(v) = z {
                    cmd.push_str(&format!(" Z{:.3}", v));
                }
                if let Some(v) = e {
                    cmd.push_str(&format!(" E{:.5}", v));
                }
                cmd
            }
            GCodeCommand::AbsoluteExtrusion => "M82".to_string(),
            GCodeCommand::RelativeExtrusion => "M83".to_string(),
            GCodeCommand::SetExtruderTemp { s } => format!("M104 S{}", s),
            GCodeCommand::SetExtruderTempWait { s } => format!("M109 S{}", s),
            GCodeCommand::SetBedTemp { s } => format!("M140 S{}", s),
            GCodeCommand::SetBedTempWait { s } => format!("M190 S{}", s),
            GCodeCommand::SetFanSpeed { s } => format!("M106 S{}", s),
            GCodeCommand::FanOff => "M107".to_string(),
            GCodeCommand::Comment(text) => format!("; {}", text),
            GCodeCommand::Raw(line) => line.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rapid_move() {
        let cmd = GCodeCommand::RapidMove {
            x: Some(10.0),
            y: Some(20.0),
            z: None,
            f: Some(3000.0),
        };
        assert_eq!(cmd.to_gcode(), "G0 X10.000 Y20.000 F3000");
    }

    #[test]
    fn test_linear_move() {
        let cmd = GCodeCommand::LinearMove {
            x: Some(10.0),
            y: Some(20.0),
            z: None,
            e: Some(1.5),
            f: Some(1200.0),
        };
        assert_eq!(cmd.to_gcode(), "G1 X10.000 Y20.000 E1.50000 F1200");
    }

    #[test]
    fn test_temperature_commands() {
        assert_eq!(
            GCodeCommand::SetExtruderTemp { s: 200 }.to_gcode(),
            "M104 S200"
        );
        assert_eq!(
            GCodeCommand::SetExtruderTempWait { s: 210 }.to_gcode(),
            "M109 S210"
        );
        assert_eq!(GCodeCommand::SetBedTemp { s: 60 }.to_gcode(), "M140 S60");
        assert_eq!(
            GCodeCommand::SetBedTempWait { s: 65 }.to_gcode(),
            "M190 S65"
        );
    }

    #[test]
    fn test_fan_commands() {
        assert_eq!(GCodeCommand::SetFanSpeed { s: 255 }.to_gcode(), "M106 S255");
        assert_eq!(GCodeCommand::FanOff.to_gcode(), "M107");
    }

    #[test]
    fn test_comment() {
        let cmd = GCodeCommand::Comment("Layer 1".to_string());
        assert_eq!(cmd.to_gcode(), "; Layer 1");
    }
}
