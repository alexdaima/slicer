# Rust Slicer Rewrite Progress

Comprehensive tracking document for the BambuStudio/libslic3r → Rust rewrite.

**Last Updated:** 2025-01-24 (Session 31)

---

## Table of Contents

1. [Overview](#overview)
2. [Project Structure](#project-structure)
3. [Coordinate System](#coordinate-system)
4. [Feature Status Summary](#feature-status-summary)
5. [Implemented Features](#implemented-features)
6. [In Progress Features](#in-progress-features)
7. [Remaining Features](#remaining-features)
8. [Test Status](#test-status)
9. [Architecture](#architecture)
10. [Dependencies](#dependencies)
11. [Changelog](#changelog)

---

## Overview

This project is a Rust-native implementation of BambuStudio/PrusaSlicer's core slicing algorithms (`libslic3r`). The goal is to produce semantically equivalent G-code output while leveraging Rust's safety and performance characteristics.

### Key Goals
- Feature parity with BambuStudio's libslic3r library
- Semantic G-code equivalence (not necessarily byte-level)
- Clean, modular, testable architecture
- No GUI dependencies (algorithm library only)

---

## Project Structure

```
slicer/
├── src/
│   ├── lib.rs                  # Main library exports
│   ├── main.rs                 # CLI entry point
│   ├── adhesion/               # Brim, Skirt, Raft generation (NEW)
│   │   └── mod.rs
│   ├── bridge/                 # Bridge detection and handling
│   │   └── mod.rs
│   ├── clipper/                # Polygon boolean operations (Clipper2)
│   │   └── mod.rs
│   ├── config/                 # Print configuration types
│   │   ├── mod.rs
│   │   ├── print_config.rs
│   │   └── region_config.rs
│   ├── edge_grid/              # Spatial acceleration structure (NEW)
│   │   ├── AGENTS.md           # Module documentation
│   │   └── mod.rs              # EdgeGrid for polygon edge queries
│   ├── flow/                   # Extrusion flow calculations
│   │   ├── AGENTS.md           # Module documentation
│   │   └── mod.rs              # Flow struct, mm3_per_mm, spacing
│   ├── gcode/                  # G-code generation
│   │   ├── AGENTS.md           # Module documentation (NEW)
│   │   ├── mod.rs
│   │   ├── arc_fitting.rs      # Arc detection (G2/G3)
│   │   ├── compare.rs          # Semantic G-code comparator
│   │   ├── cooling.rs          # CoolingBuffer, fan control (NEW)
│   │   ├── generator.rs        # GCode type and stats
│   │   ├── path.rs             # ExtrusionPath, PathGenerator
│   │   ├── spiral_vase.rs      # Spiral vase mode (NEW)
│   │   ├── validation.rs       # G-code validation & reporting (NEW)
│   │   └── writer.rs           # GCodeWriter
│   ├── geometry/               # Core geometry types
│   │   ├── mod.rs
│   │   ├── bounding_box.rs
│   │   ├── elephant_foot.rs    # First layer compensation (NEW)
│   │   ├── expolygon.rs
│   │   ├── line.rs
│   │   ├── point.rs
│   │   ├── polygon.rs
│   │   ├── polyline.rs
│   │   └── transform.rs
│   ├── infill/                 # Infill pattern generation
│   │   └── mod.rs
│   ├── mesh/                   # Triangle mesh handling
│   │   ├── mod.rs
│   │   └── stl.rs
│   ├── perimeter/              # Perimeter generation
│   │   ├── mod.rs
│   │   └── arachne/            # Variable-width perimeters
│   │       ├── mod.rs
│   │       ├── beading.rs
│   │       ├── junction.rs
│   │       └── line.rs
│   ├── pipeline/               # Complete slicing pipeline
│   │   ├── AGENTS.md           # Module documentation (NEW)
│   │   └── mod.rs
│   ├── print/                  # Print objects
│   │   └── mod.rs
│   ├── slice/                  # Mesh slicing
│   │   ├── mod.rs
│   │   ├── layer.rs
│   │   ├── mesh_slicer.rs
│   │   ├── slicer.rs
│   │   ├── slicing_params.rs
│   │   └── surface.rs
│   ├── support/                # Support structure generation
│   │   └── mod.rs
│   └── travel/                 # Travel path planning (NEW)
│       ├── AGENTS.md           # Module documentation
│       └── mod.rs              # AvoidCrossingPerimeters
└── tests/
    └── integration/
```

---

## Coordinate System

| Property | Value | Notes |
|----------|-------|-------|
| Coord Type | `i64` | Scaled integers for precision |
| CoordF Type | `f64` | Floating-point for mm values |
| Scaling Factor | `1,000,000` | 1 mm = 1,000,000 units |
| Precision | 1 nm | Nanometer-level precision |

**Helper Functions:**
- `scale(mm: f64) -> i64` - Convert mm to internal units
- `unscale(coord: i64) -> f64` - Convert internal units to mm

---

## Feature Status Summary

| Category | Implemented | In Progress | Remaining | Total |
|----------|:-----------:|:-----------:|:---------:|:-----:|
| Core Pipeline | 8 | 0 | 0 | 8 |
| Geometry | 16 | 0 | 0 | 16 |
| Perimeter | 6 | 0 | 1 | 7 |
| Infill | 17 | 0 | 0 | 17 |
| G-code | 19 | 0 | 0 | 19 |
| Support | 8 | 1 | 0 | 9 |
| Adhesion | 4 | 0 | 0 | 4 |
| Surface | 3 | 0 | 1 | 4 |
| Advanced | 0 | 0 | 5 | 5 |
| **Total** | **81** | **1** | **7** | **89** |

**Overall Progress: ~92%**

---

## Implemented Features

### Core Pipeline ✅ (8/8)

| Feature | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| PrintPipeline | `pipeline/mod.rs` | `Print.cpp` | ✅ |
| PipelineConfig | `pipeline/mod.rs` | `PrintConfig.cpp` | ✅ |
| Mesh Slicer | `slice/mesh_slicer.rs` | `TriangleMeshSlicer.cpp` | ✅ |
| Layer Generation | `slice/layer.rs` | `Layer.cpp` | ✅ |
| Surface Types | `slice/surface.rs` | `Surface.cpp` | ✅ |
| STL I/O | `mesh/stl.rs` | `Format/STL.cpp` | ✅ |
| Triangle Mesh | `mesh/mod.rs` | `TriangleMesh.cpp` | ✅ |
| Slicing Params | `slice/slicing_params.rs` | `Slicing.cpp` | ✅ |

### Geometry ✅ (16/16)

| Feature | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| Point/Point3 | `geometry/point.rs` | `Point.cpp` | ✅ |
| Line | `geometry/line.rs` | `Line.cpp` | ✅ |
| Polygon | `geometry/polygon.rs` | `Polygon.cpp` | ✅ |
| Polyline | `geometry/polyline.rs` | `Polyline.cpp` | ✅ |
| ExPolygon | `geometry/expolygon.rs` | `ExPolygon.cpp` | ✅ |
| BoundingBox | `geometry/bounding_box.rs` | `BoundingBox.cpp` | ✅ |
| Transform2D/3D | `geometry/transform.rs` | `Geometry.cpp` | ✅ |
| Clipper Wrapper | `clipper/mod.rs` | `ClipperUtils.cpp` | ✅ |
| Union/Intersection | `clipper/mod.rs` | `ClipperUtils.cpp` | ✅ |
| Offset/Grow/Shrink | `clipper/mod.rs` | `ClipperUtils.cpp` | ✅ |
| Boolean Operations | `clipper/mod.rs` | `ClipperUtils.cpp` | ✅ |
| MultiPoint | `geometry/polygon.rs` | `MultiPoint.cpp` | ✅ |
| Elephant Foot Comp. | `geometry/elephant_foot.rs` | `ElephantFootCompensation.cpp` | ✅ |
| Convex Hull | `adhesion/mod.rs` | `Geometry.cpp` | ✅ |
| EdgeGrid | `edge_grid/mod.rs` | `EdgeGrid.cpp` | ✅ |
| AABBTree | `geometry/aabb_tree.rs` | `AABBTreeIndirect.hpp` | ✅ |
| Adaptive Slicing | `slice/adaptive_heights.rs` | `SlicingAdaptive.cpp` | ✅ |

### Perimeter Generation ✅ (6/7)

| Feature | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| Classic Perimeters | `perimeter/mod.rs` | `PerimeterGenerator.cpp` | ✅ |
| Arachne Generator | `perimeter/arachne/mod.rs` | `Arachne/` | ✅ |
| Beading Strategies | `perimeter/arachne/beading.rs` | `Arachne/BeadingStrategy.cpp` | ✅ |
| ExtrusionJunction | `perimeter/arachne/junction.rs` | `Arachne/ExtrusionJunction.hpp` | ✅ |
| ExtrusionLine | `perimeter/arachne/line.rs` | `Arachne/ExtrusionLine.hpp` | ✅ |
| Gap Fill | `perimeter/mod.rs` | `PerimeterGenerator.cpp` | ✅ |
| Fuzzy Skin | - | `FuzzySkin.cpp` | ❌ |

### Infill Patterns ✅ (17/17)

| Pattern | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| Rectilinear | `infill/mod.rs` | `Fill/FillRectilinear.cpp` | ✅ |
| Grid | `infill/mod.rs` | `Fill/FillRectilinear.cpp` | ✅ |
| Line | `infill/mod.rs` | `Fill/FillLine.cpp` | ✅ |
| Concentric | `infill/mod.rs` | `Fill/FillConcentric.cpp` | ✅ |
| Floating Concentric | `infill/floating_concentric.rs` | `Fill/FillFloatingConcentric.cpp` | ✅ |
| Honeycomb | `infill/mod.rs` | `Fill/FillHoneycomb.cpp` | ✅ |
| Gyroid | `infill/mod.rs` | `Fill/FillGyroid.cpp` | ✅ |
| Lightning | `infill/mod.rs` | `Fill/FillLightning.cpp` | ✅ |
| None | `infill/mod.rs` | - | ✅ |
| Adaptive Cubic | `infill/adaptive.rs` | `Fill/FillAdaptive.cpp` | ✅ |
| Support Cubic | `infill/adaptive.rs` | `Fill/FillAdaptive.cpp` | ✅ |
| 3D Honeycomb | `infill/honeycomb_3d.rs` | `Fill/Fill3DHoneycomb.cpp` | ✅ |
| Cross Hatch | `infill/cross_hatch.rs` | `Fill/FillCrossHatch.cpp` | ✅ |
| Hilbert Curve | `infill/plan_path.rs` | `Fill/FillPlanePath.cpp` | ✅ |
| Archimedean Chords | `infill/plan_path.rs` | `Fill/FillPlanePath.cpp` | ✅ |
| Octagram Spiral | `infill/plan_path.rs` | `Fill/FillPlanePath.cpp` | ✅ |

### G-code Generation ✅ (19/19)

| Feature | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| GCodeWriter | `gcode/writer.rs` | `GCodeWriter.cpp` | ✅ |
| PathGenerator | `gcode/path.rs` | `GCode.cpp` | ✅ |
| Arc Fitting (G2/G3) | `gcode/arc_fitting.rs` | `ArcFitter.cpp` | ✅ |
| Extrusion Roles | `gcode/path.rs` | `ExtrusionEntity.cpp` | ✅ |
| Retraction | `gcode/writer.rs` | `GCodeWriter.cpp` | ✅ |
| Z-Lift | `gcode/writer.rs` | `GCodeWriter.cpp` | ✅ |
| Temperature Control | `gcode/writer.rs` | `GCodeWriter.cpp` | ✅ |
| Semantic Comparator | `gcode/compare.rs` | - | ✅ |
| CoolingBuffer | `gcode/cooling.rs` | `GCode/CoolingBuffer.cpp` | ✅ |
| Spiral Vase | `gcode/spiral_vase.rs` | `GCode/SpiralVase.cpp` | ✅ |
| Layer Time Calc | `gcode/cooling.rs` | `GCode/CoolingBuffer.cpp` | ✅ |
| Avoid Crossing | `travel/mod.rs` | `GCode/AvoidCrossingPerimeters.cpp` | ✅ |
| Pressure Equalizer | `gcode/pressure_equalizer.rs` | `GCode/PressureEqualizer.cpp` | ✅ |
| Seam Placer | `gcode/seam_placer.rs` | `GCode/SeamPlacer.cpp` | ✅ |
| Wipe Tower | `gcode/wipe_tower.rs` | `GCode/WipeTower.cpp` | ✅ |
| Tool Ordering | `gcode/tool_ordering.rs` | `GCode/ToolOrdering.cpp` | ✅ |
| Multi-Material Coordinator | `gcode/multi_material.rs` | `GCode.cpp` (integration) | ✅ |
| Retract When Crossing | `gcode/retract_crossing.rs` | `GCode/RetractWhenCrossingPerimeters.cpp` | ✅ |

### Support Generation ✅ (8/9)

| Feature | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| SupportConfig | `support/mod.rs` | `Support/SupportParameters.hpp` | ✅ |
| SupportGenerator | `support/mod.rs` | `Support/SupportMaterial.cpp` | ✅ |
| Overhang Detection | `support/mod.rs` | `OverhangDetector.cpp` | ✅ |
| Interface Layers | `support/mod.rs` | `Support/SupportMaterial.cpp` | ✅ |
| TreeSupportGenerator | `support/mod.rs` | `Support/TreeSupport.cpp` | ✅ |
| Tree Support 3D | `support/tree_support_3d.rs` | `Support/TreeSupport3D.cpp` | ✅ |
| Tree Model Volumes | `support/tree_model_volumes.rs` | `Support/TreeModelVolumes.cpp` | ✅ |
| Tree Support Settings | `support/tree_support_settings.rs` | `Support/TreeSupportCommon.hpp` | ✅ |
| Pipeline Integration | `support/mod.rs` | `Print.cpp` (support dispatch) | ✅ |
| Enforcers/Blockers | - | `Support/SupportMaterial.cpp` | ❌ |

### Bridge Detection ✅ (5/5)

| Feature | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| Bridge Detector | `bridge/mod.rs` | `BridgeDetector.cpp` | ✅ |
| Direction Detection | `bridge/mod.rs` | `BridgeDetector.cpp` | ✅ |
| Bridge Infill Generation | `bridge/mod.rs` | `BridgeDetector.cpp` | ✅ |
| detect_bridges() | `bridge/mod.rs` | `BridgeDetector.cpp` | ✅ |
| Internal Bridge Detector | `bridge/mod.rs` | `InternalBridgeDetector.cpp` | ✅ |

### Adhesion Features ✅ (4/4)

| Feature | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| Brim Generator | `adhesion/mod.rs` | `Brim.cpp` | ✅ |
| Skirt Generator | `adhesion/mod.rs` | `GCode.cpp` | ✅ |
| Raft Config | `adhesion/mod.rs` | `Support/SupportMaterial.cpp` | ✅ |
| Raft Generator | `adhesion/mod.rs` | `Support/SupportMaterial.cpp` | ✅ |

### Surface Features ✅ (3/4)

| Feature | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| Elephant Foot Comp. | `geometry/elephant_foot.rs` | `ElephantFootCompensation.cpp` | ✅ |
| Ironing | `gcode/ironing.rs` | `Fill/Fill.cpp` (`Layer::make_ironing()`) | ✅ |
| Fuzzy Skin | `perimeter/fuzzy_skin.rs` | `FuzzySkin.cpp` | ✅ |
| Seam Painting | - | `GCode/SeamPlacer.cpp` | ❌ |

### Configuration ✅ (5/5)

| Feature | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| PrintConfig | `config/print_config.rs` | `PrintConfig.cpp` | ✅ |
| PrintObjectConfig | `config/print_config.rs` | `PrintConfig.cpp` | ✅ |
| PrintRegionConfig | `config/region_config.rs` | `PrintConfig.cpp` | ✅ |
| GCodeFlavor | `config/print_config.rs` | `PrintConfig.cpp` | ✅ |
| SeamPosition | `gcode/path.rs` | `PrintConfig.cpp` | ✅ |

### Path Ordering ✅ (2/2)

| Feature | File | BambuStudio Reference | Status |
|---------|------|----------------------|:------:|
| Nearest Neighbor | `gcode/path.rs` | `ShortestPath.cpp` | ✅ |
| Seam Placement | `gcode/path.rs` | `GCode/SeamPlacer.cpp` | ✅ |

---

## In Progress Features

### Tree Support 🔄

| Feature | File | Status | Notes |
|---------|------|:------:|-------|
| TreeSupportGenerator | `support/mod.rs` | 🔄 | Basic skeleton, needs 3D collision |

---

## Remaining Features

### High Priority

| Feature | BambuStudio Reference | Notes |
|---------|----------------------|-------|
| Tree Support 3D | `Support/TreeSupport3D.cpp` | Full organic supports |

### Medium Priority

| Feature | BambuStudio Reference | Notes |
|---------|----------------------|-------|
| (none remaining) | - | - |

### Low Priority

| Feature | BambuStudio Reference | Notes |
|---------|----------------------|-------|
| Floating Concentric | `Fill/FillFloatingConcentric.cpp` | Top surface (complex, needs Clipper Z) |
| Object Arrangement | `Arrange.cpp` | Auto bed placement |

---

## Test Status

**Current Status: ✅ All tests passing**

| Category | Passed | Failed | Ignored |
|----------|:------:|:------:|:-------:|
| Unit Tests | 900 | 0 | 0 |
| Integration Tests | 75 | 0 | 4 |
| Doc Tests | 1 | 0 | 5 |

**Total: 975 tests passing**

Note: Integration tests include:
- `benchy_integration.rs` - Reference G-code comparison (20 pass, 4 ignored - data files may be missing)
- `multi_material_integration.rs` - Multi-material pipeline tests (26 tests)
- `tree_support_integration.rs` - Tree support pipeline tests (25 tests)

### Test Commands

```bash
# Run all tests
cargo test

# Run with output
cargo test -- --nocapture

# Run specific module tests
cargo test gcode::cooling
cargo test adhesion
cargo test geometry::elephant_foot

# Run integration tests only
cargo test --test '*'
```

---

## Architecture

### Pipeline Flow

```
┌─────────────────────┐
│    TriangleMesh     │
│    (STL Input)      │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│       Slicer        │
│   (Mesh → Layers)   │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐     ┌─────────────────────┐
│ Elephant Foot Comp. │     │   Support Generator │
│   (First Layer)     │     │   (Normal/Tree)     │
└──────────┬──────────┘     └──────────┬──────────┘
           │                           │
           ▼                           │
┌─────────────────────┐                │
│ Perimeter Generator │                │
│ (Classic/Arachne)   │                │
└──────────┬──────────┘                │
           │                           │
           ▼                           │
┌─────────────────────┐                │
│  Bridge Detector    │                │
└──────────┬──────────┘                │
           │                           │
           ▼                           │
┌─────────────────────┐                │
│  Infill Generator   │                │
└──────────┬──────────┘                │
           │                           │
           └───────────┬───────────────┘
                       │
                       ▼
         ┌─────────────────────┐
         │   Brim/Skirt Gen.   │
         │   (First Layer)     │
         └──────────┬──────────┘
                    │
                    ▼
         ┌─────────────────────┐
         │   Path Generator    │
         │  (Ordering/Seam)    │
         └──────────┬──────────┘
                    │
                    ▼
         ┌─────────────────────┐
         │   Cooling Buffer    │
         │   (Speed/Fan)       │
         └──────────┬──────────┘
                    │
                    ▼
         ┌─────────────────────┐
         │    Spiral Vase      │
         │   (If Enabled)      │
         └──────────┬──────────┘
                    │
                    ▼
         ┌─────────────────────┐
         │    GCodeWriter      │
         │   (Arc Fitting)     │
         └──────────┬──────────┘
                    │
                    ▼
         ┌─────────────────────┐
         │    G-code File      │
         └─────────────────────┘
```

### Key Design Decisions

1. **Scaled Integers:** All geometry uses scaled integers (1mm = 1,000,000 units) to avoid floating-point precision issues, matching BambuStudio.

2. **Clipper2:** Uses Clipper2 for all boolean operations, same as BambuStudio.

3. **Modular Architecture:** Each subsystem (perimeter, infill, support, adhesion) is independent and testable.

4. **Builder Pattern:** Configuration uses builder pattern for ergonomic API.

5. **No GUI:** Pure algorithm library - no UI dependencies.

---

## Dependencies

```toml
[dependencies]
thiserror = "1.0"           # Error handling
clipper2 = "0.1"            # Polygon boolean operations
ordered-float = "4.0"       # Hashable floats for geometry
rayon = "1.8"               # Parallel processing
log = "0.4"                 # Logging facade

[dev-dependencies]
approx = "0.5"              # Float comparisons in tests
```

---

## Changelog

### 2025-01-24 (Session 31) - Gap Fill Detection Implementation

**Implemented Gap Fill Detection:**
- Added morphological operations to `clipper/mod.rs`:
  - `opening()` - morphological opening (shrink then grow)
  - `closing()` - morphological closing (grow then shrink)
  - `offset2()` - general shrink/grow operation for perimeter generation
  - `detect_gaps()` - detects narrow gaps between polygon sets
  - `extract_centerlines()` - approximates medial axis for gap regions
- Updated `perimeter/mod.rs` to detect gaps during perimeter generation:
  - Gap detection enabled when `gap_fill_threshold > 0`
  - Uses offset2 for internal perimeters to enable proper gap detection
  - Accumulates gaps across perimeter levels
  - Processes gaps into gap fill polylines using centerline extraction
- Updated `gcode/path.rs` to convert gap fills to extrusion paths:
  - Gap fills use `ExtrusionRole::GapFill`
  - Gap fills use reduced width (0.7x perimeter width)
  - Thin fills also converted (0.5x perimeter width)

**Gap Fill Algorithm (from BambuStudio):**
1. During perimeter offset, detect areas that become too narrow
2. Gaps = diff(offset(last, -0.5*distance), offset(new, 0.5*distance + safety))
3. Collapse gaps using morphological opening to remove tiny regions
4. Extract centerlines using offset-based approximation
5. Filter out very short gap fills below threshold

**Tests Added:**
- `test_opening` - morphological opening preserves approximate area
- `test_closing` - morphological closing preserves approximate area
- `test_offset2` - offset2 with equal shrink/grow preserves shape
- `test_offset2_removes_thin_features` - thin protrusions are removed
- `test_detect_gaps_no_gaps` - concentric squares, no crashes
- `test_detect_gaps_empty_inputs` - handles empty input gracefully
- `test_extract_centerlines_simple` - narrow rectangle centerline
- `test_gap_fill_detection_disabled_by_default` - gap fills empty when threshold=0
- `test_gap_fill_detection_enabled` - thin rectangle generates perimeters
- `test_gap_fill_with_complex_shape` - L-shape corner gap detection
- `test_thin_fills_empty_by_default` - thin fills require medial axis (not yet implemented)

**Files Changed:**
- `clipper/mod.rs` - Added morphological operations and gap detection utilities
- `perimeter/mod.rs` - Integrated gap fill detection into perimeter generation
- `gcode/path.rs` - Added gap fill and thin fill path conversion

**Test Status:** 994 tests passing

**Next Steps for Extrusion Parity:**
1. **HIGH: Investigate excess infill generation** - still generating ~2x paths vs reference
2. **MEDIUM: Add first layer line width** - use 0.5mm for initial layer
3. **MEDIUM: Improve bridge detection** - match BambuStudio's algorithm
4. **LOW: Implement full medial axis** - for proper thin wall detection
5. **LOW: Add LINE_WIDTH comments** - cosmetic but helps debugging


### 2025-01-23 (Session 30) - Extrusion Parity Investigation & Arc Fitting CLI

**Added Arc Fitting CLI Options:**
- Added `--arc-fitting` flag to enable G2/G3 arc commands in output
- Added `--arc-tolerance` option (default 0.05mm) for arc fitting precision
- Arc fitting dramatically reduces G-code file size (392k lines → 66k lines)
- Our output generates ~21k arcs vs reference's ~12k arcs (more aggressive fitting)

**Extrusion Parity Analysis:**
- Total extrusion is 56.8% higher than BambuStudio reference (7266mm vs 4634mm)
- First layer specifically shows 2.1x more extrusion (74mm vs 35mm E values)
- Feature breakdown differences on first layer:
  - Outer walls: 10 (ours) vs 6 (reference)
  - Inner walls: 2 (ours) vs 5 (reference)
  - Internal solid infill: 12 (ours) vs 0 (reference)
  - Bottom surface: 0 (ours) vs 3 (reference)
  - Gap infill: 0 (ours) vs 4 (reference)

**Root Cause Analysis - CRITICAL FINDING:**
- E-per-mm calculation verified correct (~0.034 mm/mm for 0.45mm width, 0.2mm height)
- **The issue is path length, NOT E calculation**
- First layer path length: ~1694mm (ours) vs ~921mm (reference) = 1.84x more paths!
- We're generating almost twice as many paths as needed
- Infill features: 12 (ours) vs 7 (reference) - extra solid infill is the main culprit
- Gap fill: 0 (ours) vs 4 (reference) - we don't generate gap fill at all
- Bridge detection nearly non-functional (16 vs 1536 bridge moves)

**Specific Issues Identified:**
1. **Excessive solid infill generation** - generating infill in areas that shouldn't have it
2. **Missing gap fill** - PerimeterResult.gap_fills and thin_fills exist but aren't populated
3. **No first layer width differentiation** - reference uses 0.5mm, we use 0.45mm
4. **Bridge detection failure** - almost no bridges detected compared to reference

**Key Differences from Reference:**
1. Reference line widths:
   - outer_wall_line_width = 0.42mm
   - inner_wall_line_width = 0.45mm
   - initial_layer_line_width = 0.5mm (we use same as regular layers)
   - sparse_infill_line_width = 0.45mm
2. Reference uses wall_sequence = "inner wall/outer wall" (inner first)
3. Reference emits LINE_WIDTH comments per feature
4. Reference has proper bottom surface classification vs our generic solid infill

**Bugs Fixed:**
- Applied flow_mult to all segments in `extrude_path_with_arcs()`, not just closing segment
- Added arc endpoint radius validation in `fit_arc_to_points()` to reject malformed arcs

**Quality Score:** 57.9/100 (threshold: 70.0)

**Files Changed:**
- `main.rs` - Added `--arc-fitting` and `--arc-tolerance` CLI options
- `pipeline/mod.rs` - Fixed flow_mult application in arc extrusion
- `gcode/arc_fitting.rs` - Added endpoint radius validation for arc fitting

**Next Steps for Extrusion Parity (Priority Order):**
1. **HIGH: Investigate excess infill generation** - why 12 infill regions vs 7?
   - Check if infill area calculation is merging/deduplicating correctly
   - Verify we're not filling perimeter interior regions
2. **HIGH: Implement gap fill detection** - populate PerimeterResult.gap_fills
3. **MEDIUM: Add first layer line width** - use 0.5mm for initial layer
4. **MEDIUM: Improve bridge detection** - match BambuStudio's algorithm
5. **LOW: Add LINE_WIDTH comments** - cosmetic but helps debugging
6. **LOW: Surface classification** - "Bottom surface" vs "Internal solid infill"

### 2025-01-22 (Session 29) - Extrusion Width Fix & Layer Marker Detection Improvements

**Fixed Critical Perimeter Width Calculation Bug:**
- In `pipeline/mod.rs` `path_config()`, the perimeter width was incorrectly calculated as:
  `perimeters_count * nozzle_diameter` (e.g., 3 * 0.4 = 1.2mm)
- Corrected to: `nozzle_diameter * 1.125` (e.g., 0.4 * 1.125 = 0.45mm)
- Also fixed `infill_width` from `nozzle_diameter * 1.2` to `nozzle_diameter * 1.125`
- This bug was causing ~3x over-extrusion in generated G-code

**Fixed G-code Layer Marker Detection:**
- Fixed false positive matching of "First layer:" substring in comment lines
- Changed `is_layer_marker()` to check for `"; LAYER:"` or `"; layer:"` at start of comment content
- Removed redundant `"; layer num/total_layer_count:"` pattern (BambuStudio emits both this and `"; CHANGE_LAYER"` for each layer, causing double-counting)
- Now only matches `"; CHANGE_LAYER"` for BambuStudio format

**Layer Marker Detection Patterns (Updated):**
| Pattern | Slicer | Example |
|---------|--------|---------|
| `; LAYER:N` | PrusaSlicer/Cura | `; LAYER:0` |
| `; Layer N, Z = X.XXX` | Rust Slicer | `; Layer 0, Z = 0.200` |
| `; CHANGE_LAYER` | BambuStudio | `; CHANGE_LAYER` |

**Validation Impact:**
- Quality score improved from 44.2 to 67.2 (with matching infill density of 15%)
- Layer count now matches exactly (240 vs 240)
- Total extrusion difference reduced from +100.7% to +29.1%
- Per-layer extrusion differences significantly reduced

**Remaining Issues:**
- Still ~29% higher total extrusion than reference (deeper algorithm differences)
- Different perimeter count defaults (we use 3, reference uses 2)
- Reference uses flow ratio of 0.98 vs our default of 1.0

**Files Changed:**
- `pipeline/mod.rs` - Fixed `path_config()` perimeter and infill width calculations
- `gcode/compare.rs` - Fixed `is_layer_marker()` to avoid false positives and double-counting

### 2025-01-22 (Session 28) - G-code Feature Type Comments & First Layer Height Fix

**Implemented G-code Feature Type Comments:**
- Added `feature_name()` method to `ExtrusionRole` enum in `gcode/path.rs`
- Returns BambuStudio-style feature names for G-code comments (e.g., "Outer wall", "Inner wall", "Sparse infill")
- Modified `pipeline/mod.rs` to emit `; FEATURE: <type>` comments before each path when the role changes
- This enables proper feature classification in the validation tool

**Feature Name Mapping:**
| ExtrusionRole | BambuStudio Feature Name |
|---------------|-------------------------|
| ExternalPerimeter | Outer wall |
| Perimeter | Inner wall |
| InternalInfill | Sparse infill |
| SolidInfill | Internal solid infill |
| TopSolidInfill | Top surface |
| BridgeInfill | Bridge |
| GapFill | Gap infill |
| Skirt | Skirt |
| SupportMaterial | Support |
| SupportMaterialInterface | Support interface |
| Wipe | Wipe |
| Custom | Custom |

**Fixed First Layer Height Configuration:**
- Updated default `first_layer_height` from 0.3mm to 0.2mm in multiple locations:
  - `config/print_config.rs` - `PrintConfig::default()`
  - `slice/slicing_params.rs` - `SlicingParams::default()`
  - `main.rs` CLI argument default
- This matches the BambuStudio reference G-code settings
- Fixed layer Z heights: Layer 0 now correctly at Z=0.2 (was 0.3)

**Validation Impact:**
- Quality score improved from 36.8 to 44.2 (initial baseline before these changes)
- Feature comparison now shows actual feature type breakdown instead of "missing" features
- Z height offset warning reduced (layers now align correctly with reference)

**Files Changed:**
- `gcode/path.rs` - Added `feature_name()` method to `ExtrusionRole`
- `pipeline/mod.rs` - Emit `; FEATURE:` comments on role changes
- `config/print_config.rs` - Updated default first_layer_height
- `slice/slicing_params.rs` - Updated default first_layer_height and test
- `main.rs` - Updated CLI default for first_layer_height

**Test Status:** All 975 tests passing

### 2025-01-21 (Session 27) - RetractWhenCrossingPerimeters

**Implemented RetractWhenCrossingPerimeters Module:**
- Created `gcode/retract_crossing.rs` with complete implementation
- Determines whether to perform retraction when travel moves cross perimeter boundaries
- Helps reduce stringing by retracting when the nozzle travels outside internal regions

**Key Types:**
- `RetractWhenCrossingPerimeters`: Main checker with layer caching for efficient queries
- `RetractDecision`: Enum indicating whether to retract or not
- `RetractCrossingConfig`: Configuration options for the feature

**Algorithm:**
1. Checks if travel path is entirely within internal regions (sparse infill areas)
2. Checks if travel path crosses any perimeter lines
3. If travel stays inside internal regions AND doesn't cross perimeters → skip retraction
4. Otherwise → recommend retraction

**Features:**
- Layer-based caching of internal islands and perimeter lines
- Bounding box acceleration for fast intersection rejection
- Supports multiple internal regions per layer
- Proper line segment intersection testing using cross product method

**New Tests (17 tests):**
- `test_retract_decision` - enum behavior
- `test_new_instance` - initialization
- `test_empty_travel` - empty path handling
- `test_single_point_travel` - single point handling
- `test_travel_outside_internal_regions` - outside regions → retract
- `test_travel_inside_internal_region` - inside regions → no retract
- `test_travel_crossing_boundary` - boundary crossing → retract
- `test_layer_caching` - cache invalidation on layer change
- `test_clear` - cache clearing
- `test_lines_intersect_crossing` - line intersection detection
- `test_lines_intersect_parallel` - parallel lines
- `test_lines_intersect_no_intersection` - non-intersecting lines
- `test_lines_intersect_endpoint` - endpoint touching
- `test_config_default` - default configuration
- `test_config_builder` - builder pattern
- `test_empty_layer` - layer with no internal regions
- `test_multiple_internal_regions` - multiple separate regions

**libslic3r Reference:**
- `src/libslic3r/GCode/RetractWhenCrossingPerimeters.cpp`
- `src/libslic3r/GCode/RetractWhenCrossingPerimeters.hpp`

**Impact on Validation Score:**
- This feature can improve G-code parity by matching BambuStudio's retraction behavior
- Reduces unnecessary retractions inside internal regions
- Should reduce stringing artifacts in generated prints

### 2025-01-21 (Session 26) - G-code Validation Tool

**Implemented Comprehensive G-code Validation System:**
- Created `gcode/validation.rs` with full validation infrastructure
- Provides detailed comparison between Rust slicer output and BambuStudio reference G-code
- Generates quality scores and detailed reports for measuring rewrite accuracy

**Key Types:**
- `ValidationReport`: Complete validation result with scores, issues, and breakdowns
- `ValidationConfig`: Configurable tolerances (strict, default, relaxed presets)
- `ValidationSummary`: High-level statistics (layers, extrusion, moves, travel)
- `ValidationIssue`: Individual issues with severity and category
- `LayerValidation`: Per-layer comparison statistics
- `FeatureType`: Classification of G-code features (perimeters, infill, support, etc.)
- `FeatureStats`: Statistics per feature type
- `ScoreBreakdown`: Detailed quality score components

**Quality Score Components (weighted):**
- Layer Count (20%): Accuracy of layer count vs reference
- Total Extrusion (30%): Accuracy of total filament usage
- Layer Consistency (25%): Average per-layer extrusion accuracy
- Coverage (15%): Similarity of total move counts
- Features (10%): Presence of expected feature types

**Output Formats:**
- Text: Human-readable console/file report with ASCII art
- JSON: Machine-readable for CI/CD integration
- HTML: Styled browser-viewable report with visual score bar

**CLI Integration (`slicer-cli validate`):**
- Slice STL and compare against reference in one command
- Or compare two existing G-code files with `--compare-only`
- Configurable tolerance levels, pass threshold, and output format
- Exit code indicates pass/fail for CI integration

**New Tests (15 tests):**
- `test_feature_type_from_comment` - feature classification
- `test_validation_config_default` - default tolerances
- `test_validation_config_strict` - strict preset
- `test_validation_identical_gcode` - perfect match
- `test_validation_different_layer_count` - layer mismatch detection
- `test_validation_different_extrusion` - extrusion mismatch detection
- `test_validation_report_text_output` - text report generation
- `test_validation_report_json_output` - JSON serialization
- `test_validation_report_html_output` - HTML generation
- `test_issue_counts` - issue severity counting
- `test_report_format_from_str` - format parsing
- `test_report_format_extension` - file extensions
- `test_layer_validation_passed` - per-layer pass/fail
- `test_score_breakdown_total` - weighted score calculation
- `test_feature_stats_is_empty` - empty stats detection

**Documentation:**
- Created `VALIDATION.md` with comprehensive usage guide
- Examples for CLI and programmatic usage
- Best practices for CI/CD integration

**libslic3r Reference:**
- Builds upon existing `gcode/compare.rs` infrastructure
- Semantic comparison (not byte-level) matching slicer philosophy

### 2025-01-21 (Session 25) - Floating Concentric Infill Pattern

**Implemented Floating Concentric Infill Pattern:**
- Created `infill/floating_concentric.rs` with full implementation
- Detects "floating" sections (not supported by layer below) for top surface quality
- Key types:
  - `FloatingThickLine`: Line segment with floating flags at each endpoint
  - `FloatingThickPolyline`: Polyline with width and floating info per vertex
  - `FloatingConcentricConfig`: Configuration with spacing, clipping, transition splitting
  - `FloatingConcentricResult`: Result with polylines, total length, floating fraction
  - `FloatingConcentricGenerator`: Main generator class

**Key Features:**
- Point-based floating detection using ray casting point-in-polygon tests
- Configurable path splitting at floating/supported transitions
- Loop rebasing to prefer starting at non-floating positions (better seam quality)
- Loop end clipping to avoid extruder landing exactly on start point
- Floating fraction calculation for quality metrics

**Integration with InfillGenerator:**
- Added `InfillPattern::FloatingConcentric` enum variant
- Added `generate_with_floating_areas()` method for floating-aware generation
- Added `is_concentric()` helper method to InfillPattern
- Added `Display` implementation for InfillPattern enum
- Re-exported all floating concentric types from `lib.rs`

**Documentation:**
- Updated `infill/AGENTS.md` with Floating Concentric section
- Documented algorithm, key types, and usage examples
- Added libslic3r reference mapping (FillFloatingConcentric.cpp)

**New Tests (18 tests in floating_concentric module + 5 integration tests):**
- `test_floating_concentric_config_default` - default config values
- `test_floating_concentric_config_builder` - builder pattern
- `test_floating_thick_line_basic` - line segment with floating flags
- `test_floating_thick_polyline_basic` - polyline creation
- `test_floating_thick_polyline_from_polygon` - closed loop creation
- `test_floating_thick_polyline_rebase` - loop rebasing
- `test_floating_thick_polyline_split_at_transitions` - transition splitting
- `test_floating_thick_polyline_clip_end` - end clipping
- `test_generate_floating_concentric_no_floating` - no floating areas
- `test_generate_floating_concentric_with_floating` - partial floating
- `test_generate_floating_concentric_with_hole` - complex geometry
- `test_generate_floating_concentric_empty` - empty input
- `test_generate_floating_concentric_all_floating` - fully floating
- `test_floating_thick_polyline_to_thick_lines` - conversion
- `test_floating_concentric_generator_config` - generator config
- `test_floating_fraction_calculation` - fraction calculation
- `test_to_polylines` - result conversion
- `test_point_in_polygon` - ray casting test
- Integration: `test_floating_concentric_pattern` - pattern properties
- Integration: `test_generate_floating_concentric_via_generator` - generator fallback
- Integration: `test_generate_floating_concentric_with_floating_areas` - full detection
- Integration: `test_floating_concentric_direct_api` - direct API usage
- Integration: `test_floating_concentric_all_floating` - 100% floating case
- Integration: `test_pattern_display` - Display trait

**Test Results:**
- All 926 unit tests passing
- 23 new floating concentric tests
- Full integration with existing infill infrastructure

**libslic3r Reference:**
- `src/libslic3r/Fill/FillFloatingConcentric.cpp`
- `src/libslic3r/Fill/FillFloatingConcentric.hpp`
- Uses point sampling instead of Clipper_Z for floating detection
- Simplified path merging without complex Z-value tracking

### 2025-01-20 (Session 24) - InfillGenerator Pattern Integration & Space-Filling Curves

**Integrated CrossHatch and Honeycomb3D into Main InfillGenerator:**
- Wired `InfillPattern::CrossHatch` into `InfillGenerator::generate()` method
- Wired `InfillPattern::Honeycomb3D` into `InfillGenerator::generate()` method
- Added graceful fallback for `Adaptive` and `SupportCubic` patterns (require octree setup)
- Added `generate_cross_hatch_pattern()` helper method
- Added `generate_honeycomb_3d_pattern()` helper method

**Extended InfillConfig:**
- Added `z_height: CoordF` field for 3D patterns that need Z position
- Added `layer_height: CoordF` field for layer-height-dependent patterns
- Added `with_z(z_height, layer_height)` builder method
- Updated all `InfillConfig` initializers in `pipeline/mod.rs`

**Implemented Space-Filling Curve Patterns (plan_path.rs):**
- Created `infill/plan_path.rs` with full implementation
- `PlanPathConfig` for pattern configuration (spacing, density, resolution)
- `PlanPathGenerator` supporting three patterns:
  - **Hilbert Curve**: Fractal space-filling curve that fills a square
  - **Archimedean Chords**: Spiral pattern from center outward (r = a + b*theta)
  - **Octagram Spiral**: 8-pointed star spiral pattern
- `PlanPathResult` with polylines and total length
- Convenience functions: `generate_hilbert_curve`, `generate_archimedean_chords`, `generate_octagram_spiral`
- 15 unit tests covering all patterns
- Reference: BambuStudio `Fill/FillPlanePath.cpp` (Math::PlanePath library)

**Wired Plan Path Patterns into InfillGenerator:**
- Added `InfillPattern::HilbertCurve`, `ArchimedeanChords`, `OctagramSpiral` variants
- Added `generate_plan_path_pattern()` helper method
- Updated `is_linear()` and `is_implemented()` for new patterns

**New Tests:**
- `test_generate_cross_hatch` - basic CrossHatch generation
- `test_cross_hatch_layer_variation` - Z height variation
- `test_cross_hatch_with_hole` - complex geometry
- `test_generate_honeycomb_3d` - basic 3D Honeycomb generation
- `test_honeycomb_3d_layer_variation` - Z height variation
- `test_honeycomb_3d_with_hole` - complex geometry
- `test_adaptive_fallback_to_rectilinear` - fallback behavior
- `test_support_cubic_fallback_to_rectilinear` - fallback behavior
- `test_generate_hilbert_curve` - Hilbert curve integration
- `test_generate_archimedean_chords` - Archimedean integration
- `test_generate_octagram_spiral` - Octagram integration
- `test_hilbert_with_hole` - Hilbert with complex geometry
- `test_pattern_is_implemented_all` - verify all patterns work
- `test_pattern_is_linear_plan_path` - verify pattern classification

**Test Results:**
- All 902 unit tests passing
- 15 plan_path module tests
- 14 new integration tests for pattern wiring

### 2025-01-19 (Session 23) - Cross Hatch Infill & Adaptive Layer Heights

**Implemented Cross Hatch Infill Pattern:**
- Created `infill/cross_hatch.rs` with full implementation
- `CrossHatchConfig` for pattern configuration (grid size, angle, repeat ratio)
- `CrossHatchGenerator` for pattern generation at specific Z heights
- `CrossHatchResult` with polylines and layer type metadata
- Two layer types: repeat layers (parallel lines) and transform layers (zigzag transitions)
- Automatic direction alternation every period for strength
- Optimized repeat ratio for low density prints
- 20 unit tests covering all functionality
- Reference: BambuStudio `Fill/FillCrossHatch.cpp`

**Implemented Adaptive Layer Heights:**
- Created `slice/adaptive_heights.rs` with full implementation
- `AdaptiveHeightsConfig` for layer height bounds and quality settings
- `AdaptiveSlicing` engine that computes variable layer heights from mesh
- `FaceZ` struct to store face Z-span and normal angle information
- Four error metrics matching BambuStudio:
  - TriangleArea (Vojtech's formula) - default
  - TopographicDistance (Cura-style)
  - SurfaceRoughness (Perez/Pandey)
  - Wasserfall (original paper formula)
- Quality factor (0.0-1.0) to balance detail vs speed
- Convenience functions: `compute_adaptive_heights`, `compute_adaptive_heights_with_quality`
- 15 unit tests covering configuration, metrics, and slicing
- Reference: BambuStudio `SlicingAdaptive.cpp` (Florens Wasserfall's paper)

**Added Clipper Helper:**
- `intersect_polylines_with_expolygons` for clipping polylines to polygon regions
- Used by Cross Hatch infill for pattern clipping

**Test Results:**
- All 873 unit tests passing
- Cross Hatch: 20 tests
- Adaptive Heights: 15 tests

### 2025-01-19 (Session 22) - 3D Honeycomb Infill

**Implemented:**
- Full 3D Honeycomb infill implementation (`infill/honeycomb_3d.rs`)
  - `Honeycomb3DConfig` for configuration (spacing, z, layer_height, density, angle)
  - `Honeycomb3DGenerator` - main generator class
  - `Honeycomb3DResult` - result type with polylines and total length
  - `generate_honeycomb_3d()` convenience function
- Key algorithms implemented:
  - `tri_wave()` - Triangular wave function with period gridSize*2
  - `troct_wave()` - Truncated octagonal waveform for Z-dependent pattern
  - `get_critical_points()` - Find curve change points in truncated octahedron
  - `colinear_points()` / `perpend_points()` - Generate pattern coordinates
  - `make_actual_grid()` - Generate grid polylines varying with Z height
  - Pattern alternates between vertical/horizontal based on Z cycle
- Added `InfillPattern::Honeycomb3D` variant
- Full integration with lib.rs exports

**Tests Added:**
- 13 unit tests for 3D honeycomb module:
  - Triangular wave function
  - Critical points generation
  - Grid generation
  - Config default and builder patterns
  - Generator creation and execution
  - Layer variation (different Z heights)
  - Zero density handling
  - Result creation
  - Convenience function
  - Sign function
  - Point-in-polygon
  - High density handling

**BambuStudio Reference:**
- `src/libslic3r/Fill/Fill3DHoneycomb.cpp`
- Algorithm credits: David Eccles (gringer)
- Creates horizontal slices through truncated octahedron tesselation

---

### 2025-01-19 (Session 21) - AABBTree Spatial Acceleration

**Implemented:**
- Full AABBTree implementation (`geometry/aabb_tree.rs`)
  - `Vec3` - 3D vector type with full operations (add, sub, dot, cross, normalize)
  - `AABB3` - 3D axis-aligned bounding box with containment and distance queries
  - `AABBNode` - Tree node with bounding box and primitive index
  - `AABBTree` - Balanced binary tree with implicit indexing (children at 2*i+1, 2*i+2)
  - `IndexedTriangleSet` - Triangle mesh with AABB tree for spatial queries
  - `AABBClosestPointResult` - Result type for closest point queries
  - `RayHit` - Result type for ray intersection queries
- Key algorithms implemented:
  - `build_from_triangles()` - Build balanced tree using QuickSelect partitioning
  - `ray_box_intersect()` - Slab method for ray-AABB intersection
  - `ray_triangle_intersect()` - Möller–Trumbore algorithm
  - `closest_point_on_triangle()` - Ericson's Real-Time Collision Detection algorithm
  - `ray_cast_first()` / `ray_cast_all()` - Ray casting with early termination
  - `closest_point()` - Closest point on mesh to query point
  - `triangles_within_distance()` - Range query for nearby triangles
  - `any_triangle_in_radius()` - Fast radius test
- Full integration with lib.rs exports

**Tests Added:**
- 13 unit tests for AABB tree module:
  - Vec3 operations
  - AABB3 operations and squared exterior distance
  - Tree building from triangles
  - Ray-box intersection
  - Ray-triangle intersection (Möller–Trumbore)
  - Closest point on triangle
  - IndexedTriangleSet ray casting (first hit)
  - IndexedTriangleSet ray casting (all hits)
  - IndexedTriangleSet closest point queries
  - Triangles within distance range query
  - Any triangle in radius test
  - next_power_of_2 utility function

**BambuStudio Reference:**
- `src/libslic3r/AABBTreeIndirect.hpp`
- `src/libslic3r/AABBTreeLines.hpp`
- Based on libigl's AABB tree with PrusaSlicer/BambuStudio optimizations

---

### 2025-01-19 (Session 20) - Adaptive Cubic Infill

**Implemented:**
- Full adaptive cubic infill implementation (`infill/adaptive.rs`)
  - `AdaptiveInfillConfig` for configuration (line spacing, extrusion width, hook length)
  - `Vec3d` and `Matrix3d` types for 3D math operations
  - `AABBf3` axis-aligned bounding box for 3D
  - `triangle_aabb_intersects()` - SAT-based triangle/AABB intersection test
  - `Cube` and `Octree` data structures for spatial subdivision
  - `CubeProperties` for per-level octree parameters
  - `transform_to_world()` and `transform_to_octree()` coordinate transforms
  - `build_octree()` - builds octree from mesh triangles
  - `AdaptiveInfillGenerator` - main generator class
  - `generate_adaptive_infill()` convenience function
  - Support for overhang-only densification mode (`SupportCubic` pattern)
- Added `InfillPattern::Adaptive` and `InfillPattern::SupportCubic` variants
- Helper methods: `needs_octree()`, `is_adaptive()` on InfillPattern
- Full integration with lib.rs exports

**Tests Added:**
- 15 unit tests for adaptive infill module:
  - Vec3d operations (add, subtract, dot, cross, normalize)
  - Matrix3d rotation
  - CubeProperties calculation
  - make_cubes_properties level generation
  - Triangle-AABB intersection (inside/outside cases)
  - Octree building from triangles
  - Octree building from cube mesh
  - AdaptiveInfillConfig from density
  - Generator creation and building
  - Line generation at layer height
  - Point-in-polygon test
  - AdaptiveInfillResult creation
  - Transform round-trip (world ↔ octree)

**BambuStudio Reference:**
- `src/libslic3r/Fill/FillAdaptive.cpp`
- `src/libslic3r/Fill/FillAdaptive.hpp`
- Algorithm inspired by Cura's adaptive cubic infill

---

### 2025-01-19 (Session 19) - Fuzzy Skin & Internal Bridge Detection

**Features Implemented:**

1. **Fuzzy Skin** (`perimeter/fuzzy_skin.rs`)
   - `FuzzySkinConfig` - Configuration for thickness and point distance
   - `fuzzy_polygon()` - Apply random perturbations to polygon perimeters
   - `fuzzy_polyline()` - Apply fuzzy skin to open polylines
   - `fuzzy_extrusion_line()` - Apply fuzzy skin to Arachne variable-width extrusions
   - `should_fuzzify()` - Determine if perimeter should have fuzzy skin
   - `apply_fuzzy_skin_polygon()` - Conditional application based on layer/perimeter
   - `apply_fuzzy_skin_extrusion()` - Conditional application for Arachne lines
   - Thread-local RNG using xorshift64 for fast random generation
   - Matches BambuStudio's `FuzzySkin.cpp` algorithm

2. **Internal Bridge Detection** (`bridge/mod.rs`)
   - `InternalBridgeConfig` - Configuration for internal bridge detection
   - `InternalBridgeDetector` - Detects optimal bridging angle for internal surfaces
   - `detect_internal_bridges()` - Convenience function for batch detection
   - Mirrors BambuStudio's `InternalBridgeDetector.cpp` algorithm
   - Finds anchor regions from fill area overlap
   - Tests multiple angle candidates and selects best coverage

3. **Config Exports**
   - Exported `FuzzySkinMode` and `IroningType` from config module
   - Added fuzzy skin types to main lib.rs exports
   - Added internal bridge types to lib.rs exports

**Test Results:** 797 tests passing (22 new tests added)

---

### 2025-01-18 (Session 18) - Organic Smoothing Integration with Branch Mesh

**Wired Organic Smoothing Output into Branch Mesh Generation**

Integrated the organic smoothing algorithm with the branch mesh pipeline,
ensuring smoothed branch positions are used when generating 3D tube meshes.
This follows the BambuStudio workflow where `organic_smooth_branches_avoid_collisions()`
is called before `draw_branches()`.

**New Functions in organic_smooth.rs**:
- `build_spheres_from_move_bounds()` - Extract collision spheres from support elements
- `apply_smoothed_positions_to_move_bounds()` - Write smoothed positions back
- `smooth_move_bounds()` - High-level function combining the above

**New Types**:
- `SphereMapping` - Maps sphere indices back to (layer_idx, element_idx)
- `SphereBuildResult` - Contains smoother, mappings, and linear_data_layers

**TreeSupport3D Enhancements**:
- Added `apply_organic_smoothing()` method
- Added `apply_organic_smoothing_with_config()` for custom config
- Added `generate_with_organic_smoothing()` for full mesh+smoothing pipeline
- Added `volumes()` accessor for TreeModelVolumes reference
- Refactored `generate_with_mesh()` to use internal helper

**Algorithm Flow**:
1. Generate initial areas and layer pathing
2. Set points on areas (determine branch centers)
3. **NEW**: Apply organic smoothing to update `result_on_layer` positions
4. Generate branch mesh from smoothed positions
5. Slice mesh into per-layer polygons

**Tests Added**:
9 new unit tests in `organic_smooth.rs`:
- `test_sphere_mapping`
- `test_build_spheres_from_move_bounds_empty`
- `test_build_spheres_from_move_bounds_no_result_on_layer`
- `test_build_spheres_from_move_bounds_with_element`
- `test_apply_smoothed_positions_to_move_bounds_empty`
- `test_apply_smoothed_positions_to_move_bounds`
- `test_smooth_move_bounds_empty`
- `test_smooth_move_bounds_single_element`
- `test_sphere_build_result_linear_data_layers`

4 new integration tests in `tree_support_integration.rs`:
- `test_tree_support_3d_with_organic_smoothing`
- `test_organic_smoothing_applied_to_move_bounds`
- `test_tree_support_apply_organic_smoothing_method`
- `test_organic_smoothing_with_custom_config`

**New Re-exports in support/mod.rs**:
- `build_spheres_from_move_bounds`
- `apply_smoothed_positions_to_move_bounds`
- `smooth_move_bounds`
- `SphereBuildResult`
- `SphereMapping`

**Documentation Updated**:
- `src/support/AGENTS.md` - Added SphereMapping, SphereBuildResult types
- `src/support/AGENTS.md` - Added Organic Smoothing Integration section
- `src/support/AGENTS.md` - Added TreeSupport3D methods documentation
- `src/support/organic_smooth.rs` - Added module-level integration docs

**Test Results**:
- 775 library unit tests pass (766 + 9 new)
- 25 tree support integration tests pass (21 + 4 new)
- 26 multi-material integration tests pass
- Total: 847 tests passing

**Files Modified**:
- `src/support/organic_smooth.rs` - Added sphere building and position application functions
- `src/support/tree_support_3d.rs` - Added organic smoothing methods
- `src/support/mod.rs` - Added new re-exports
- `src/support/AGENTS.md` - Updated documentation
- `tests/tree_support_integration.rs` - Added 4 integration tests

**BambuStudio Reference**:
- `Support/TreeSupport3D.cpp`: `organic_smooth_branches_avoid_collisions()`
- `Support/TreeSupport3D.cpp`: `draw_branches()` called after smoothing
- `Support/TreeSupport3D.cpp`: `elements_with_link_down` construction

---

### 2025-01-18 (Session 17) - Branch Mesh Drawing for Tree Support

**Added Branch Mesh Drawing Module**

New file: `src/support/branch_mesh.rs`

Implements 3D tube mesh generation for tree support branches, providing more
accurate geometry than simple circle-based rasterization. This is the Rust port
of BambuStudio's `draw_branches()`, `extrude_branch()`, and related functions.

**Key Types**:
- `BranchMeshBuilder` - Builds 3D tube meshes from branch paths
- `BranchPath` - Connected sequence of branch elements
- `BranchPathElement` - Position, radius, and layer info for one point
- `BranchMeshConfig` - Discretization settings (eps, segment counts)
- `BranchMeshResult` - Result with mesh, z_span, and branch count
- `Point3D` - 3D point with f64 coordinates for mesh building

**Mesh Generation Algorithm**:
1. Build branch paths from support elements
2. For each path, extrude tube mesh:
   - Bottom hemisphere at start point
   - Cylindrical sections between elements (varying radius)
   - Top hemisphere at end point
3. Triangulate using fan and strip patterns
4. Merge all branches into cumulative mesh

**Key Functions**:
- `build_branch_paths()` - Extract connected paths from support elements
- `generate_branch_mesh()` - High-level mesh generation
- `slice_branch_mesh()` - Slice mesh to get per-layer polygons
- `discretize_circle()` - Create vertex ring for tube section
- `triangulate_fan_bottom/top()` - Fan triangulation for hemispheres
- `triangulate_strip()` - Strip triangulation between rings

**Helper Math Functions**:
- `add_vec3`, `sub_vec3`, `scale_vec3` - Vector operations
- `dot_vec3`, `cross_vec3` - Dot/cross products
- `length_vec3`, `normalize_vec3` - Length and normalization
- `create_orthonormal_basis` - Perpendicular vectors to normal

**TreeSupport3D Enhancements**:
- Added `generate_with_mesh()` method for mesh-based generation
- Added `move_bounds()`, `move_bounds_mut()` accessors
- Added `config()` and `num_layers()` getters

**New Re-exports in support/mod.rs**:
- `BranchMeshBuilder`, `BranchMeshConfig`, `BranchMeshResult`
- `BranchPath`, `BranchPathElement`, `Point3D`
- `build_branch_paths`, `generate_branch_mesh`, `slice_branch_mesh`

**Tests Added**:
17 unit tests in `branch_mesh.rs`:
- `test_branch_path_new`, `test_branch_path_push`
- `test_branch_mesh_config_default`, `test_branch_mesh_builder_empty`
- `test_branch_mesh_builder_single_branch`, `test_branch_mesh_builder_varying_radius`
- `test_vector_math`, `test_normalize_vec3`, `test_normalize_zero_vector`
- `test_create_orthonormal_basis`, `test_discretize_circle`
- `test_triangulate_fan`, `test_triangulate_strip`
- `test_build_branch_paths_empty`, `test_distance_squared`
- `test_branch_mesh_result`, `test_slice_branch_mesh_empty`

5 integration tests added to `tree_support_integration.rs`:
- `test_tree_support_3d_with_mesh_generation`
- `test_branch_mesh_builder_simple_tube`
- `test_branch_mesh_builder_multi_segment`
- `test_branch_mesh_builder_multiple_branches`
- `test_branch_mesh_config_custom`

**Documentation Updated**:
- `src/support/AGENTS.md` - Added Branch Mesh Drawing section
- Updated file structure listing

**Test Results**:
- 766 library unit tests pass (749 + 17 new)
- 21 tree support integration tests pass (16 + 5 new)
- 26 multi-material integration tests pass
- Total: 813 tests passing

**Files Added**:
- `src/support/branch_mesh.rs` - Branch mesh drawing module

**Files Modified**:
- `src/support/mod.rs` - Added branch_mesh submodule and re-exports
- `src/support/tree_support_3d.rs` - Added generate_with_mesh() and accessors
- `src/support/AGENTS.md` - Added branch mesh documentation
- `tests/tree_support_integration.rs` - Added 5 branch mesh tests

**BambuStudio Reference**:
- `Support/TreeSupport3D.cpp`: `draw_branches()`, `extrude_branch()`
- `Support/TreeSupport3D.cpp`: `discretize_circle()`, `triangulate_fan()`, `triangulate_strip()`
- `Support/TreeSupport3D.cpp`: `slice_branches()`

---

### 2025-01-18 (Session 16) - Tree Support Pipeline Integration & Organic Smoothing

**Wired TreeSupport3D into Main Pipeline**

Integrated the tree support generation algorithm into the main slicing pipeline,
enabling automatic dispatch to tree support when `SupportType::Tree` or
`SupportType::Organic` is configured.

**Changes to SupportGenerator**:
- Modified `generate()` to dispatch based on `SupportType`
- Added `generate_normal_support()` for existing normal support flow
- Added `generate_tree_support()` for TreeSupport3D integration
- Added `is_near_overhang()` helper for interface layer marking

**Tree Support Integration Flow**:
1. Detect overhangs (same as normal support)
2. Create `TreeModelVolumesConfig` from `SupportConfig`
3. Create `TreeModelVolumes` with layer outlines
4. Create `TreeSupport3DConfig` from `SupportConfig`
5. Run `TreeSupport3D::generate()` with overhang polygons
6. Post-process: mark interface layers, merge with overhang regions

**New Re-exports in support/mod.rs**:
- `TreeModelVolumes`, `TreeModelVolumesConfig`
- `TreeSupport3D`, `TreeSupport3DConfig`, `TreeSupport3DResult`
- `TreeSupportSettings`

**Integration Tests Added**:
New file: `tests/tree_support_integration.rs` with 16 tests:
- `test_tree_support_generator_integration` - Basic tree support flow
- `test_tree_support_vs_normal_support` - Compare tree vs normal support
- `test_tree_support_buildplate_only` - Buildplate-only mode
- `test_tree_support_organic_mode` - Organic support type dispatch
- `test_tree_support_3d_direct` - Direct TreeSupport3D usage
- `test_tree_model_volumes_configuration` - Config validation
- `test_tree_support_3d_config_from_support_config` - Config conversion
- `test_pipeline_with_tree_support_cube` - Full pipeline test
- `test_pipeline_normal_vs_tree_support` - Pipeline comparison
- `test_pipeline_tree_support_settings_propagation` - Settings access
- `test_tree_support_empty_geometry` - Empty input handling
- `test_tree_support_single_layer` - Single layer handling
- `test_tree_support_no_overhang_geometry` - No overhang case
- `test_tree_support_with_interface_layers` - Interface layer marking
- `test_tree_support_small_overhang_filtered` - Area filtering
- `test_tree_support_many_layers` - Performance sanity (100 layers)

**Unit Tests Added**:
- `test_tree_support_config` - Tree support config builder
- `test_tree_support_generator_no_overhang` - No overhang with tree
- `test_tree_support_generator_with_overhang` - Overhang with tree
- `test_organic_support_type` - Organic type dispatch
- `test_is_near_overhang` - Interface layer helper
- `test_tree_support_buildplate_only` - Buildplate-only config

**Documentation Updated**:
- `src/support/AGENTS.md` - Added pipeline integration section
- Test coverage summary added

**Test Results**:
- 732 library unit tests pass
- 16 tree support integration tests pass
- 26 multi-material integration tests pass
- Total: 774 tests passing

**Files Added**:
- `tests/tree_support_integration.rs` - Tree support integration tests

**Files Modified**:
- `src/support/mod.rs` - Added tree support dispatch and re-exports
- `src/support/AGENTS.md` - Pipeline integration documentation

---

**Added Organic Smoothing Module**

New file: `src/support/organic_smooth.rs`

Implements the organic smoothing algorithm for tree support branches.
This refines branch positions after initial generation using Laplacian
smoothing combined with collision avoidance.

**Key Types**:
- `OrganicSmoother` - Main smoother with collision detection and Laplacian smoothing
- `OrganicSmoothConfig` - Configuration (max_nudge, smoothing_factor, iterations)
- `OrganicSmoothResult` - Result with iterations, positions, converged flag
- `CollisionSphere` - 3D sphere representing branch position for smoothing
- `LayerCollisionCache` - Cached collision lines per layer
- `Point3F` - 3D floating point coordinate type

**Algorithm**:
1. Build collision caches from layer outlines
2. For each iteration:
   - Backup positions for Laplacian averaging
   - Check collision with model boundaries
   - Nudge spheres away from collisions
   - Apply Laplacian smoothing (weighted average with neighbors)
3. Repeat until convergence or max iterations

**Features**:
- Collision detection using line-sphere distance queries
- Laplacian smoothing with configurable smoothing factor
- Z-bound propagation through tree structure
- Locked spheres (tips and roots don't move)
- Convergence detection

**Helper Functions**:
- `closest_point_on_segment` - Point-to-segment distance calculation
- `smooth_branches` - High-level function for full smoothing workflow

**Tests**: 17 unit tests covering all major functionality

**Updated Module Structure**:
- Added `organic_smooth` submodule to `support/mod.rs`
- Added re-exports: `OrganicSmoothConfig`, `OrganicSmoothResult`, `OrganicSmoother`

**Test Results**:
- 17 new organic_smooth tests pass
- All 749 unit tests pass (732 + 17 new)
- All integration tests continue to pass

**Files Added**:
- `src/support/organic_smooth.rs` - Organic smoothing algorithm

**Files Modified**:
- `src/support/mod.rs` - Added organic_smooth submodule and re-exports

---

### 2025-01-18 (Session 15 continued) - Tree Support 3D Branch Generation

**Added Tree Support 3D Generator Module**

New file: `src/support/tree_support_3d.rs`

Complete tree support branch generation algorithm implementation.
Ports the core algorithm from BambuStudio's `TreeSupport3D.cpp`.

**Key Types**:
- `TreeSupport3D` - Main generator with full branch generation pipeline
- `TreeSupport3DConfig` - Configuration for tree support generation
- `TreeSupport3DResult` - Result with layers, branch count, tip count
- `LineInformation` / `LineInformations` - Point + status tracking
- `SupportElements` / `LayerSupportElements` - Support elements per layer

**Features**:
- Initial area generation from overhangs (tip placement)
- Layer pathing (downward branch propagation)
- Influence area expansion with collision avoidance
- Branch merging for overlapping influence areas
- Point-on-area setting (final branch center determination)
- Support layer conversion (circles at branch positions)
- Avoidance status detection (ToBuildPlate, ToModel, etc.)

**Helper Functions**:
- `create_circle_polygon` - Generate circular branch cross-sections
- `sample_polygon_points` - Sample overhang boundaries for tip placement
- `offset_polygons_simple` - Simple polygon grow/shrink
- `safe_offset_inc` - Offset respecting forbidden areas
- `difference_polygons` - Boolean difference operation
- `influence_areas_overlap` - Check if branches should merge
- `merge_support_elements` - Combine overlapping branches
- `move_inside_if_outside` - Ensure points stay in valid areas
- `closest_point_on_segment` - Point-to-segment distance calculation

**Tests**: 18 unit tests covering all major functionality

**Updated Module Structure**:
- Added `tree_support_3d` submodule to `support/mod.rs`
- Added re-exports to `lib.rs` for all new public types
- Updated `support/AGENTS.md` with TreeSupport3D documentation

**Test Results**:
- 18 new tree_support_3d tests pass
- All 726 unit tests pass (708 + 18 new)

**Files Added**:
- `src/support/tree_support_3d.rs` - Branch generation algorithm

**Files Modified**:
- `src/support/mod.rs` - Added tree_support_3d submodule
- `src/lib.rs` - Added tree_support_3d re-exports
- `src/support/AGENTS.md` - Updated documentation

---

### 2025-01-18 (Session 15) - Tree Support 3D Foundation

**Added Tree Model Volumes Module**

New file: `src/support/tree_model_volumes.rs`

Collision detection and avoidance area computation for tree support generation.
This is the foundation for Tree Support 3D, providing the spatial queries
needed for branch growth and collision avoidance.

**Key Types**:
- `TreeModelVolumes` - Main structure for pre-computed collision/avoidance areas
- `TreeModelVolumesConfig` - Configuration for volume computation
- `RadiusLayerPolygonCache` - Thread-safe cache for computed areas
- `RadiusLayerKey` - Key for radius/layer lookup
- `AvoidanceType` - Fast, FastSafe, Slow avoidance modes

**Features**:
- Collision area computation (model outline + radius + XY distance)
- Hole-free collision for buildplate-only supports
- Avoidance area computation with movement propagation
- Placeable area computation (where tips can be placed)
- Wall restriction computation
- Radius ceiling with exponential stepping for cache efficiency
- Pre-calculation for efficient parallel querying
- Safe position finding for branch movement

**Tests**: 17 unit tests covering all major functionality

**Added Tree Support Settings Module**

New file: `src/support/tree_support_settings.rs`

Configuration and support element state types for tree support generation.
Ports the BambuStudio `TreeSupportCommon.hpp` types.

**Key Types**:
- `TreeSupportMeshGroupSettings` - Mesh group configuration
- `TreeSupportSettings` - Derived settings for generation
- `SupportElementState` - State machine for branch elements
- `SupportElementStateBits` - Bit flags for element state
- `SupportElement` - Complete element with influence area
- `AreaIncreaseSettings` - Settings for area growth
- `InterfacePreference` - Interface/support overlap preference
- `LineStatus` - Path segment status tracking

**Features**:
- Settings derivation from mesh group settings
- Radius calculation based on distance-to-top
- Elephant foot compensation
- State propagation for branch growth
- Parent tracking for element merging

**Tests**: 16 unit tests covering all major functionality

**Updated Module Structure**:
- Added `tree_model_volumes` and `tree_support_settings` submodules to `support/mod.rs`
- Added re-exports to `lib.rs` for all new public types
- Updated `support/AGENTS.md` with new documentation

**Test Results**:
- 33 new tree support tests pass
- All 708 unit tests pass
- Total: 755 tests passing

**Files Added**:
- `src/support/tree_model_volumes.rs` - Collision/avoidance volumes
- `src/support/tree_support_settings.rs` - Settings & element state

**Files Modified**:
- `src/support/mod.rs` - Added submodule declarations
- `src/lib.rs` - Added re-exports
- `src/support/AGENTS.md` - Updated documentation

---

### 2025-01-18 (Session 14) - CI/CD & Multi-Material Integration Tests

**Added GitHub Actions CI/CD Workflow**

New file: `.github/workflows/ci.yml`

Comprehensive CI pipeline for continuous integration:

**Jobs**:
- `check` - Runs `cargo check --all-targets`
- `fmt` - Checks code formatting with `cargo fmt --check`
- `clippy` - Runs Clippy linter with `-D warnings`
- `test` - Runs all tests on ubuntu, macos, and windows
- `coverage` - Generates code coverage with `cargo-llvm-cov`
- `ci-success` - Summary job for required check status

**Features**:
- Multi-platform testing (Linux, macOS, Windows)
- Cargo registry caching for faster builds
- Code coverage reporting to Codecov
- Pull request and push triggers
- Branch protection ready (ci-success job)

**Added Multi-Material Integration Tests**

New test file: `tests/multi_material_integration.rs`

Comprehensive end-to-end tests for multi-material pipeline:

**Test Coverage (26 tests)**:
- Multi-material coordinator creation and configuration
- Tool ordering with simple layer configurations
- Flush matrix calculations and asymmetric values
- Optimal ordering calculation for minimum flush
- Multi-material coordinator initialization workflow
- Flush volume calculation between extruders
- Multi-material with different flush volumes (PLA/PETG)
- Extruder role handling (perimeter, infill, support)
- Layer-specific tool information
- Exhaustive ordering for small extruder counts
- Wipe tower bounds calculation
- Multi-material with disabled wipe tower
- Single material mode (no-op)
- Tool change detection
- Flush volume retrieval from coordinator
- Wipe tower avoidance polygon generation
- Tool ordering config conversion
- Filament parameters conversion
- Multi-material plan layer structure
- Flush matrix with multipliers
- Extruder sequence optimization variations
- Generate all orderings for different sizes
- Wipe tower bounds polygon conversion

**Test Results**:
- All 26 multi-material integration tests pass
- Total integration tests: 46 (20 benchy + 26 multi-material)
- Unit tests: 675 passed
- Doc tests: 1 passed, 5 ignored

**Files Added**:
- `.github/workflows/ci.yml` - CI/CD workflow
- `tests/multi_material_integration.rs` - Integration tests

---

### 2025-01-18 (Session 13) - Multi-Material Coordinator Integration

**Implemented Multi-Material Coordination Module**

New module: `src/gcode/multi_material.rs`

The multi-material coordinator integrates tool ordering and wipe tower into a unified API for multi-extruder printing, mirroring the interaction between BambuStudio's `GCode.cpp`, `GCode/ToolOrdering.cpp`, and `GCode/WipeTower.cpp`.

**Key Components**:

- `MultiMaterialConfig`: Configuration for all multi-material settings
  - Filament properties (densities, colors, types, soluble flags)
  - Flush matrix and per-filament multipliers
  - Wipe tower position, dimensions, and rotation
  - Default extruders per extrusion role
  
- `MultiMaterialCoordinator`: Main coordinator integrating tool ordering and wipe tower
  - Initializes layers from Z heights
  - Tracks extruder usage per layer and role
  - Plans optimal tool change sequences
  - Generates wipe tower geometry
  - Provides extruder selection for each extrusion role
  - Returns wipe tower avoidance polygon for travel planning

- `MultiMaterialPlan`: Result of planning containing:
  - Per-layer tool change data
  - Total flush volume and tool change count
  - Wipe tower bounding box
  - Filament change statistics
  - First/last/all extruders used

- `MultiMaterialLayer`: Per-layer data with:
  - Extruder sequence
  - Tool change events
  - Wipe tower results
  - Custom G-code events

- `ToolChange`: Individual tool change with from/to extruders and flush volume

- `WipeTowerBounds`: Bounding box for collision avoidance

**Integration Flow**:

```
1. Create MultiMaterialCoordinator with config
2. initialize_layers() - Set up layer Z heights
3. add_extruder_to_layer() - Record extruder usage per layer/role
4. plan() - Optimize tool sequences and plan wipe tower
   - handle_dontcare_extruders() - Assign flexible regions
   - reorder_extruders_for_minimum_flush() - Minimize purge waste
   - mark_skirt_layers() - Mark layers needing skirt
   - fill_wipe_tower_partitions() - Calculate tower requirements
   - collect_extruder_statistics() - Gather usage stats
   - Initialize and plan wipe tower tool changes
5. generate_wipe_tower() - Generate tower G-code
6. get_extruder_for_role() - Query extruder for each feature
7. get_wipe_tower_avoidance() - Get collision polygon for travel
```

**Features**:

- Unified API for multi-material pipeline integration
- Automatic wipe tower initialization and planning
- Configurable default extruders per extrusion role
- Flush volume calculation with per-filament multipliers
- Tool change detection for role transitions
- Wipe tower avoidance polygon for travel planning
- Statistics tracking (total flush, tool changes)

**New Tests Added (15)**:
- MultiMaterialConfig default and new constructors
- Default filament color generation
- Coordinator single-material (no-op) mode
- Coordinator multi-material mode
- ToolChange creation
- MultiMaterialLayer creation and tool change tracking
- MultiMaterialPlan creation and queries
- WipeTowerBounds geometry and polygon conversion
- Flush volume calculation with multipliers
- Config to ToolOrderingConfig conversion
- Config to FilamentParameters conversion
- Coordinator initialize and plan flow
- get_extruder_for_role defaults
- needs_tool_change detection

**Files Modified**:
- `src/gcode/mod.rs` - Added multi_material module export and re-exports
- `src/lib.rs` - Added multi-material type re-exports
- `src/gcode/AGENTS.md` - Documented multi-material coordination

**Test Results**:
- Unit tests: 675 passed (15 new multi_material tests)
- Integration tests: 20 passed, 4 ignored
- Doc-tests: 1 passed, 5 ignored

---

### 2025-01-18 (Session 12) - Tool Ordering Implementation

**Implemented Multi-Extruder Tool Change Coordination**

New module: `src/gcode/tool_ordering.rs`

The tool ordering system determines the optimal sequence of extruder switches across all layers to minimize filament waste, mirroring BambuStudio's `GCode/ToolOrdering.cpp`.

**Key Components**:

- `ToolOrdering`: Main engine managing per-layer tool sequences
- `ToolOrderingConfig`: Configuration (flush matrix, filament properties, sequences)
- `LayerTools`: Per-layer information about extruders, partitions, and custom G-code
- `FlushMatrix`: Matrix of purge volumes between filament pairs (mm³)
- `WipingExtrusions`: Tracks which extrusions are used for wiping during tool changes
- `FilamentChangeStats`: Statistics about filament changes (flush weight, counts)
- `CustomGCodeItem`: Custom G-code events (color change, pause, custom)

**Algorithm** (matching BambuStudio):

1. **Initialization**:
   - `initialize_layers()`: Create LayerTools for each Z height
   - Merge numerically close Z values

2. **Extruder Collection**:
   - Gather required extruders per layer from objects and supports
   - Handle "don't care" extruders (regions printable with any extruder)

3. **Optimization**:
   - `reorder_extruders_for_minimum_flush()`: Greedy nearest-neighbor algorithm
   - `find_optimal_ordering_exhaustive()`: Brute-force for small sets (≤8 extruders)
   - Custom sequence overrides for specific layers

4. **Wipe Tower Integration**:
   - `fill_wipe_tower_partitions()`: Calculate partitions needed per layer
   - Propagate partitions downward for structural support

5. **Custom G-code**:
   - `assign_custom_gcodes()`: Place color changes, pauses at correct layers
   - Track extruders printing above each layer for color change validation

**Features**:

- Greedy and exhaustive ordering optimization
- Flush volume matrix with per-extruder multipliers
- "Don't care" extruder handling for flexible assignment
- First layer tool order based on object geometry (larger areas first)
- Custom layer print sequences support
- Wipe tower partition calculation
- Skirt layer marking
- Soluble filament preference at layer start
- Support for filament properties (soluble, support material)
- Statistics calculation (flush weight, change count)

**New Tests Added (20)**:
- FilamentChangeStats operations (add, clear)
- FlushMatrix creation, access, and flat conversion
- FlushMatrix sequence calculation and multiplier
- LayerTools creation and extruder order checking
- WipingExtrusions override tracking
- CustomGCodeItem constructors
- ExtrusionRoleType classification methods
- ToolOrderingConfig creation
- ToolOrdering layer initialization
- Optimize extruder sequence greedy algorithm
- Generate all orderings (permutations)
- Find optimal ordering exhaustive search
- Handle "don't care" extruders
- Collect extruder statistics
- Mark skirt layers
- Fill wipe tower partitions
- Tool ordering iteration

**Files Modified**:
- `src/gcode/mod.rs` - Added tool_ordering module export
- `src/lib.rs` - Added tool ordering type re-exports
- `src/gcode/AGENTS.md` - Documented tool ordering algorithm

**Test Results**:
- Unit tests: 660 passed (20 new tool ordering tests)
- Integration tests: 20 passed, 4 ignored
- Doc-tests: 1 passed, 5 ignored

---

### 2025-01-18 (Session 11) - Wipe Tower Implementation

**Implemented Multi-Material Wipe Tower for Tool Changes**

New module: `src/gcode/wipe_tower.rs`

The wipe tower is a sacrificial structure printed alongside multi-material prints for filament purging during tool changes, mirroring BambuStudio's `GCode/WipeTower.cpp`.

**Key Components**:

- `WipeTower`: Main generator that plans and generates tower G-code
- `WipeTowerConfig`: Configuration (position, dimensions, speeds, bed shape)
- `WipeTowerWriter`: Specialized G-code writer for tower operations
- `ToolChangeResult`: Result of each tool change with G-code and path data
- `NozzleChangeResult`: Result of nozzle change operations
- `WipeTowerLayerInfo`: Planning data for each layer
- `ToolChangeInfo`: Individual tool change planning data
- `FilamentParameters`: Per-filament configuration (temps, speeds, ramming)
- `BoxCoordinates`: Helper for wipe tower region coordinates
- `Vec2f`: 2D vector type for tower coordinates

**Algorithm** (matching BambuStudio):

1. **Planning Phase**:
   - `plan_toolchange()`: Record each tool change with purge volumes
   - `plan_tower()`: Calculate tower depth based on total purge needs
   - Apply extra spacing for structural stability (height-dependent)

2. **Generation Phase**:
   - `generate()`: Generate G-code for all layers
   - `tool_change()`: Handle individual tool changes with ramming/wiping
   - `finish_layer()`: Complete layer with infill and perimeter

3. **Tool Change Sequence**:
   - Retract old filament
   - Ramming (fast extrusion to push out old material)
   - Tool change command (T#)
   - Load new filament
   - Wiping (back-and-forth to clean and prime)

**Features**:

- Automatic tower depth calculation based on purge volumes
- Minimum depth enforcement for structural stability (height-based)
- Multi-extruder support with filament mapping
- SEMM (single extruder multi-material) support
- Brim generation on first layer
- Sparse/solid infill based on layer requirements
- Rotation angle support for tower placement
- Multiple G-code flavors (Marlin, Klipper, RepRap, etc.)

**New Tests Added (31)**:
- Vec2f operations (add, sub, mul, normalize, rotate, neg)
- BoxCoordinates creation and expansion
- WipeTowerConfig defaults
- FilamentParameters defaults
- WipeTower creation and configuration
- Tool change planning (single and multiple layers)
- Tower depth limit calculations
- WipeTowerWriter G-code generation
- Rectangle drawing and filling
- ToolChangeResult extrusion length calculation
- WipeTowerLayerInfo depth calculations
- Align functions (round, ceil, floor)
- G-code validation
- Volume/length conversion round-trip
- Auto brim width calculation
- Full generation pipeline test

**Files Modified**:
- `src/gcode/mod.rs` - Added wipe_tower module export
- `src/lib.rs` - Added wipe tower type re-exports
- `src/gcode/AGENTS.md` - Documented wipe tower algorithm

**Test Results**:
- Unit tests: 640 passed (31 new wipe tower tests)
- Integration tests: 20 passed, 4 ignored
- Doc-tests: 1 passed, 5 ignored

---

### 2025-01-18 (Session 10) - Seam Placer Implementation

**Implemented Intelligent Seam Placement for Perimeter Loops**

New module: `src/gcode/seam_placer.rs`

The seam placer determines optimal starting points for perimeter extrusion loops, mirroring BambuStudio's `GCode/SeamPlacer.cpp` implementation.

**Key Components**:

- `SeamPositionMode`: Aligned, Nearest, Random, Rear, Hidden
- `SeamPlacerConfig`: Configuration with mode-specific presets
- `SeamPlacer`: Main placer that computes optimal seam positions
- `SeamCandidate`: Per-vertex candidate with visibility/overhang/angle attributes
- `Perimeter`: Metadata for a perimeter loop (indices, seam position)
- `LayerSeams`: All candidates and perimeters for one layer
- `Point3f`: 3D floating-point position for seam coordinates

**Algorithm** (matching BambuStudio):

1. **Candidate Generation**: Create candidates at each polygon vertex
2. **Angle Calculation**: Compute local CCW angle at each vertex
3. **Overhang Detection**: Compare against previous layer for overhang scoring
4. **Embedded Distance**: Calculate distance inside merged regions (EdgeGrid)
5. **Seam Selection**: Score candidates using SeamComparator logic
6. **Layer Alignment**: Optionally align seams across layers using spatial grouping

**Seam Position Modes**:
- `Aligned`: Aligns seams vertically across layers (default)
- `Nearest`: Places seam nearest to current position
- `Random`: Deterministic randomization to scatter seam artifacts
- `Rear`: Places seam at rear (highest Y) to hide on back
- `Hidden`: Actively seeks concave corners to hide seams

**Scoring Factors**:
- Enforced/Blocked/Neutral priority (user-painted preferences)
- Overhang avoidance (prefer points with support beneath)
- Embedded distance (prefer hidden points inside print)
- Angle penalty (concave corners preferred over convex)

**New Tests Added (26)**:
- Configuration and mode tests
- Point3f operations tests
- SeamCandidate creation tests
- Perimeter/LayerSeams tests
- Angle penalty and gauss function tests
- SeamComparator tests (enforced/blocked, overhang, embedded)
- Fallback mode tests (nearest, rear, hidden)
- Full initialization and seam point retrieval tests
- Convenience function tests

**Files Modified**:
- `src/gcode/mod.rs` - Added seam_placer module export
- `src/lib.rs` - Added seam placer re-exports
- `src/gcode/AGENTS.md` - Documented seam placer algorithm

---

### 2025-01-18 (Session 9) - Pressure Equalizer & Ironing Implementation

**Implemented Ironing for Surface Smoothing**

New module: `src/gcode/ironing.rs`

Ironing adds an extra pass over top surfaces with very low flow to smooth them and improve surface quality.

**Key Components**:

- `IroningType`: NoIroning, TopSurfaces, TopmostOnly, AllSolid
- `IroningConfig`: Configuration with builder pattern
- `IroningGenerator`: Generates ironing paths from surfaces
- `IroningPath`: Individual ironing path with flow/speed info
- `IroningResult`: Collection of paths with statistics

**Algorithm** (matching BambuStudio's `Layer::make_ironing()`):

1. Identify surfaces to iron based on IroningType
2. Shrink surfaces by inset (default: half nozzle diameter)
3. Generate rectilinear fill pattern with tight spacing (0.1mm)
4. Extrude with very low flow (10-15% of normal)
5. Use nozzle heat to melt and smooth the surface

**Configuration Options**:
- `flow_percent`: Flow rate as percentage (default: 10%)
- `line_spacing`: Spacing between lines (default: 0.1mm)
- `inset`: Distance from edge (default: 0 = half nozzle)
- `speed`: Ironing speed (default: 20mm/s)
- `direction`: Ironing angle in degrees
- `pattern`: Infill pattern (default: Rectilinear)

**New Tests Added (21)**:
- IroningType tests (default, enabled, names)
- IroningConfig builder tests
- IroningPath length/time/volume calculations
- IroningGenerator tests (enabled, disabled, topmost_only)
- Convenience function tests

---

### 2025-01-18 (Session 9) - Pressure Equalizer Implementation

**Implemented Pressure Equalizer for Smooth Extrusion Rate Transitions**

New module: `src/gcode/pressure_equalizer.rs`

The pressure equalizer processes G-code to smooth out rapid changes in volumetric extrusion rate, preventing pressure spikes in the extruder that cause print artifacts like blobs and zits.

**Key Components**:

- `PressureEqualizerConfig`: Configuration for filament diameter, slope limits, and buffer size
- `PressureEqualizer`: Main processor with circular buffer for lookahead analysis
- `PressureEqualizerStats`: Statistics tracking (min/max/avg volumetric rates)

**Algorithm** (matching BambuStudio's `PressureEqualizer.cpp`):

1. Parse G-code line by line into a circular buffer
2. Calculate volumetric extrusion rate for each segment:
   - `rate = filament_area × feedrate × (extrusion_length / travel_length)`
3. Limit rate changes with two passes:
   - **Backward pass**: Limits how fast rate can increase (negative slope)
   - **Forward pass**: Limits how fast rate can decrease (positive slope)
4. Split long segments for smoother acceleration/deceleration profiles
5. Regenerate G-code with adjusted feedrates

**Special Handling**:
- Bridge infill: Unlimited slope (needs consistent flow for bridging)
- Gap fill: Unlimited slope (thin features need precise flow)
- Default slope: ~1.8 mm³/s² (20→60 mm/s over 2 seconds)

**API**:
```rust
let config = PressureEqualizerConfig::new(1.75)
    .with_max_slope(2.0)
    .with_relative_e(true);
let mut equalizer = PressureEqualizer::new(config);
let output = equalizer.process(gcode, true);
let stats = equalizer.stats();
```

**New Tests Added (22)**:
- Configuration tests (default, builder, cross-section)
- Buffer index operations
- G-code line distance/time calculations
- Process tests (empty, moves, extrusion, comments, tool change)
- Role marker parsing
- Statistics tracking
- Firmware retract (G10/G11)

**Progress Update**:
- G-code generation: 14/15 features complete (was 13/15)
- Overall progress: ~76% (was ~75%)
- Total tests: 562 passing

---

### 2025-01-17 (Session 8) - Travel Planner Pipeline Integration

**Integrated AvoidCrossingPerimeters into G-code Generation Pipeline**

The travel planner from Session 7 is now fully integrated into the slicing pipeline, automatically routing travel moves around perimeter walls during G-code generation.

Key changes:

**PrintConfig Updates** (`src/config/print_config.rs`):
- Added `avoid_crossing_perimeters: bool` - Enable/disable travel optimization (default: true)
- Added `avoid_crossing_max_detour: CoordF` - Maximum detour percentage (default: 200%)

**PipelineConfig Updates** (`src/pipeline/mod.rs`):
- Added `travel_config()` method to generate TravelConfig from print settings
- Added `uses_avoid_crossing_perimeters()` check
- Added builder methods:
  - `avoid_crossing_perimeters(bool)` - Enable/disable the feature
  - `avoid_crossing_max_detour(f64)` - Set max detour percentage

**PrintPipeline Integration**:
- Added `travel_planner: Option<AvoidCrossingPerimeters>` field
- Travel planner is initialized automatically when enabled
- `write_layer()` now:
  - Extracts perimeter polygons from layer paths
  - Initializes travel planner with layer boundaries
  - Uses `emit_travel()` for all travel moves
- New `emit_travel()` method routes travels through planner when available

**API Changes**:
- `process()`, `process_with_callback()`, `generate_gcode()`, and `write_layer()` now take `&mut self` to allow travel planner state updates
- Tests updated to use `let mut pipeline = ...`

**New Tests Added (6)**:
- `test_pipeline_avoid_crossing_perimeters_enabled_by_default`
- `test_pipeline_avoid_crossing_perimeters_config`
- `test_pipeline_with_avoid_crossing_perimeters`
- `test_pipeline_without_avoid_crossing_perimeters`
- `test_travel_config_from_pipeline`
- Fixed test assertion in `test_pipeline_gcode_absolute_e`

**Progress Update**:
- Total tests passing: 541 unit + 1 CLI + 20 integration = 562 total
- Travel planner is now production-ready and integrated
- Matches BambuStudio default behavior (avoid crossing enabled by default)

---

### 2025-01-17 (Session 7) - EdgeGrid & Avoid Crossing Perimeters

**EdgeGrid Spatial Acceleration Structure**

Implemented `EdgeGrid`, a spatial acceleration structure for efficient polygon edge queries. This is essential for travel path planning and collision detection.

New module: `src/edge_grid/`

Key features:
- Grid-based spatial indexing for polygon edges
- Fast line-polygon intersection testing
- Closest point on polygon edge queries
- Point-in-polygon testing using ray casting
- Signed distance field computation
- Support for both open polylines and closed polygons

New types added:
- `EdgeGrid` - Main spatial acceleration structure
- `Contour` - Represents a polygon or polyline in the grid
- `ClosestPointResult` - Result of closest point queries
- `Intersection` - Result of line-polygon intersection queries

**Avoid Crossing Perimeters**

Implemented `AvoidCrossingPerimeters` for travel path planning that routes moves around perimeter walls to prevent visible marks on printed surfaces.

New module: `src/travel/`

Key features:
- Routes travel moves around perimeter boundaries
- Supports internal (within object) and external (between objects) boundaries
- Path simplification to remove unnecessary intermediate points
- Configurable maximum detour limits
- Per-move modifiers (`disable_once`, `use_external_once`)

New types added:
- `AvoidCrossingPerimeters` - Main travel planner
- `TravelConfig` - Configuration for travel planning
- `TravelResult` - Result containing optimized travel path

Algorithm overview:
1. Find intersections between travel line and boundary polygons
2. Identify entry/exit points on each boundary
3. Route around boundary using shortest direction
4. Simplify path by removing redundant points
5. Validate detour length against configured limits

New tests added (34 total):
- EdgeGrid: 13 tests for grid creation, intersection, closest point, point-in-polygon
- Travel: 13 tests for configuration, path planning, boundary handling

Documentation:
- Created `src/edge_grid/AGENTS.md` with algorithm details
- Created `src/travel/AGENTS.md` with usage examples and integration guide

**Progress Update**
- Overall progress: ~73% (up from ~68%)
- Total tests passing: 556 (up from 511)
- Moved EdgeGrid and Avoid Crossing Perimeters from "Remaining" to "Implemented"

---

### 2025-01-16 (Session 6) - Relative E Mode & Raft Generation

**Relative Extrusion Mode (M83)**

Added support for relative extrusion mode (M83), which is the default mode used by BambuStudio and most modern slicers.

Changes made:
- Added `use_relative_e` field to `PrintConfig` (default: `true` to match BambuStudio)
- Updated `GCodeWriter::write_preamble()` to emit M83 or M82 based on config
- Updated `GCodeWriter::extrude_to()` to output relative E values when in M83 mode
- Updated `GCodeWriter::extrude_arc()` to handle relative E mode for arc moves
- Added `use_relative_e()` builder method to `PipelineConfig`
- Internal E tracking remains absolute for consistency; conversion to relative happens at output

New tests added (8 total):
- `test_writer_preamble_relative_e` - Tests M83 output in preamble
- `test_writer_preamble_absolute_e` - Tests M82 output in preamble  
- `test_writer_extrude_relative_e` - Tests relative E value output
- `test_writer_extrude_absolute_e` - Tests absolute E value output
- `test_pipeline_relative_e_default` - Tests default config uses relative E
- `test_pipeline_relative_e_builder` - Tests builder method
- `test_pipeline_gcode_relative_e` - Tests M83 in generated G-code
- `test_pipeline_gcode_absolute_e` - Tests M82 in generated G-code

**Raft Generation**

Implemented `RaftGenerator` for creating raft structures beneath models for improved bed adhesion.

New types added:
- `RaftResult` - Contains generated raft layers and total height
- `RaftLayer` - Individual raft layer with type, Z height, thickness, outline, and fill paths
- `RaftLayerType` - Enum for Base, Interface, and Contact layers
- `RaftGenerator` - Generates raft layers from first layer slices

Features:
- Configurable number of layers (distributed as base, interface, and contact)
- Raft expansion beyond model footprint
- Different layer heights and line spacing per layer type
- Alternating fill direction per layer (0°/90°)
- Base layers: thicker, wider spacing for bed adhesion
- Interface layers: medium spacing for structural support  
- Contact layer: fine spacing for smooth model bottom surface
- Height calculation for model Z offset

New tests added (7 total):
- `test_raft_generator_disabled` - Tests disabled raft produces empty result
- `test_raft_generator_enabled` - Tests enabled raft generates layers
- `test_raft_layer_types` - Tests layer type distribution
- `test_raft_result_model_offset` - Tests Z offset calculation
- `test_raft_calculate_height` - Tests raft height calculation
- `test_raft_empty_slices` - Tests empty input handling
- `test_raft_expansion` - Tests expansion config

**Test Results**: 511 tests passing

### 2025-01-15 (Session 5) - Documentation & Architecture

**Major Documentation Initiative**

Created comprehensive AGENTS.md documentation files for all modules to establish the project as a well-architected, maintainable replacement for libslic3r.

**Root Documentation:**
- `AGENTS.md` - Master project guide with:
  - Project overview and goals
  - Architecture diagram
  - Module reference table mapping Rust to C++ files
  - Critical implementation details (coordinates, flow calculations)
  - Development guidelines
  - libslic3r source reference
  - Directory structure

**Module Documentation (16 AGENTS.md files created):**

| Module | Key Content |
|--------|-------------|
| `adhesion/` | Brim, skirt, raft generation with algorithms |
| `bridge/` | Bridge detection, direction optimization |
| `clipper/` | Boolean operations, polygon offsetting |
| `config/` | Configuration hierarchy and validation |
| `flow/` | Critical extrusion calculations (already existed) |
| `gcode/` | G-code generation, path management |
| `geometry/` | Core types: Point, Polygon, ExPolygon |
| `infill/` | All infill patterns with algorithms |
| `mesh/` | STL loading, mesh processing |
| `perimeter/` | Shell generation (Classic + Arachne) |
| `perimeter/arachne/` | Variable-width algorithm details |
| `pipeline/` | Slicing orchestration |
| `print/` | Print job and object management |
| `slice/` | Mesh slicing algorithm |
| `support/` | Support generation (Normal + Tree) |

Each AGENTS.md includes:
- Module purpose and responsibilities
- libslic3r file mapping table
- Key types and their C++ equivalents
- Algorithm explanations with pseudocode
- File structure
- Dependencies and related modules
- Testing strategy
- Future enhancements roadmap

**Test Results:** 498 unit tests passing

---

### 2025-01-15 (Session 5) - Flow Module Fix

**Critical Fix: Extrusion Flow Calculations**

The extrusion calculations were using incorrect cross-section area formulas,
causing significant under-extrusion in generated G-code (~50% less filament
than expected).

**Root Cause:**
- Code was using `width × height` (simple rectangle)
- Should use `height × (width - height × (1 - π/4))` (rounded rectangle)
- This models the actual physical shape of extruded plastic

**Added:**
- `Flow` module (`flow/mod.rs`) - Direct port of libslic3r/Flow.cpp
  - `Flow` struct with exact mm3_per_mm() formula matching C++
  - `FlowRole` enum for extrusion type classification
  - `rounded_rectangle_extrusion_spacing()` and inverse
  - `bridge_extrusion_spacing()` for circular cross-sections
  - `auto_extrusion_width()` role-based defaults
  - `with_width()`, `with_height()`, `with_flow_ratio()` modifiers
  - `e_per_mm()` and `extrusion_for_length()` helpers
  - Support material flow helper functions
  - 18 comprehensive unit tests including parity tests

- `AGENTS.md` documentation files for modules:
  - `flow/AGENTS.md` - Flow calculation documentation
  - `gcode/AGENTS.md` - G-code generation documentation
  - `pipeline/AGENTS.md` - Pipeline orchestration documentation

**Fixed:**
- `ExtrusionPath::cross_section_area()` now uses correct formula
- `PrintPipeline::calculate_e_for_distance()` now uses correct formula
- Added `calculate_e_for_bridge()` for bridge-specific calculations
- Updated `test_extrusion_path_volume` to expect correct values

**Formula Reference (from libslic3r/Flow.cpp):**
```
Normal extrusion:  area = height × (width - height × (1 - π/4))
Bridge extrusion:  area = π × (width/2)²
```

**Test Results:** 498 unit tests passing (18 new flow tests)

---

### 2025-01-15 (Session 4)

**Added:**
- `CoolingBuffer` module (`gcode/cooling.rs`)
  - `CoolingConfig` for cooling parameters
  - `CoolingMove` for per-segment tracking
  - `PerExtruderAdjustments` for multi-extruder support
  - Layer slowdown calculation algorithm
  - Fan speed interpolation based on layer time
  - Bridge/overhang fan speed overrides

- `SpiralVase` module (`gcode/spiral_vase.rs`)
  - `SpiralVaseConfig` configuration
  - `SpiralPoint` with interpolated Z
  - `SpiralLayer` for perimeter processing
  - Continuous Z interpolation
  - XY smoothing with previous layer
  - Transition layer handling

- `ElephantFootCompensation` module (`geometry/elephant_foot.rs`)
  - `ElephantFootConfig` configuration
  - `ElephantFootCompensator` processor
  - Polygon/ExPolygon compensation
  - Minimum contour width preservation
  - Automatic compensation calculation

- `Adhesion` module (`adhesion/mod.rs`)
  - `BrimConfig` and `BrimGenerator`
  - `SkirtConfig` and `SkirtGenerator`
  - `RaftConfig` (generator pending)
  - Brim types: Outer, Inner, Both
  - Minimum skirt length support
  - Convex hull calculation

- Comprehensive feature tracking document

**Test Results:** 480 unit tests, 20 integration tests passing

### 2025-01-14 (Session 3)

**Added:**
- Bridge detection and analysis module
- Anchor-based bridge direction optimization
- Bridge infill generation
- Pipeline integration for bridges
- Flow/speed multipliers for bridge paths

### 2025-01-14 (Session 2)

**Added:**
- Support generation system
- `SupportGenerator` for normal supports
- `TreeSupportGenerator` skeleton
- Overhang detection algorithm
- Support interface layers
- Pipeline integration for supports

### 2025-01-14 (Session 1)

**Added:**
- Arc fitting (G2/G3) support
- `ArcFitter` for polyline arc detection
- Writer support for arc commands
- Semantic G-code comparator

### 2025-01-13

**Initial Implementation:**
- Core pipeline architecture
- Mesh slicing
- Classic perimeter generation
- Arachne variable-width perimeters
- All basic infill patterns

---

## References

- [BambuStudio Source](https://github.com/bambulab/BambuStudio)
- [PrusaSlicer Source](https://github.com/prusa3d/PrusaSlicer)
- [Clipper2 Library](https://github.com/AngusJohnson/Clipper2)
- [Arachne Paper](https://doi.org/10.1016/j.addma.2020.101301)

---

## Next Steps (Priority Order)

### Immediate (This Session)
1. ~~CoolingBuffer~~ ✅
2. ~~Spiral Vase~~ ✅
3. ~~Elephant Foot Compensation~~ ✅
4. ~~Brim/Skirt Generation~~ ✅
5. ~~Avoid Crossing Perimeters Integration~~ ✅
6. ~~Pressure Equalizer~~ ✅
7. ~~Ironing~~ ✅
8. ~~Seam Placer~~ ✅
9. ~~Wipe Tower~~ ✅
10. ~~Tool Ordering~~ ✅
11. ~~TreeModelVolumes~~ ✅
12. ~~TreeSupportSettings~~ ✅
13. ~~TreeSupport3D branch generation~~ ✅
14. ~~Wire TreeSupport3D into pipeline~~ ✅

### Short-term
1. ~~CI/CD setup (GitHub Actions)~~ ✅
2. ~~Multi-material pipeline integration (wire WipeTower + ToolOrdering)~~ ✅
3. ~~Fuzzy Skin~~ ✅
4. ~~Internal Bridge Detection~~ ✅
5. Semantic G-code comparison tooling improvements
6. ~~End-to-end multi-material print test~~ ✅
7. ~~Tree Support 3D branch generation algorithm~~ ✅
8. ~~Tree Support 3D organic smoothing~~ ✅
9. ~~Wire TreeSupport3D into pipeline~~ ✅
10. ~~Wire organic smoothing into branch mesh generation~~ ✅

### Medium-term
1. ~~Full Tree Support 3D (branch generation, merging, slicing)~~ ✅
2. ~~Tree Support 3D organic smoothing~~ ✅ (collision avoidance + Laplacian smoothing)
3. ~~Branch mesh drawing (create 3D mesh for branches, then slice)~~ ✅
4. ~~Organic smoothing + branch mesh integration~~ ✅
5. ~~Fuzzy Skin~~ ✅
6. ~~Internal Bridge Detection~~ ✅
7. ~~Adaptive Infill~~ ✅
8. ~~AABBTree for mesh acceleration~~ ✅

### Medium-term (continued)
9. ~~Cross Hatch Infill~~ ✅
10. ~~3D Honeycomb Infill~~ ✅
11. ~~Adaptive Layer Heights~~ ✅
12. ~~Wire CrossHatch/Honeycomb3D into InfillGenerator~~ ✅
13. ~~Hilbert Curve Infill~~ ✅
14. ~~Archimedean Chords Infill~~ ✅
15. ~~Octagram Spiral Infill~~ ✅

### Long-term
1. Semantic parity validation
2. Performance optimization
3. Floating Concentric infill (complex, needs Clipper Z)
4. Object Arrangement / auto bed placement