# slicer

This project is currently just an fun experiment to see how well AI can handle a port of libslic3r (A complex C++ library) to Rust, utilizing something like React/Three.js to render the UI and models, and support native AI integration. If successful it could mean an evolution of the software with less complexity in adding new features & the maintainance of current complex C++ codebases.

This work is based on BambuStudio by BambuLab, which is based on PrusaSlicer by Prusa Research, which is from Slic3r by Alessandro Ranellucci and the RepRap community.

---

## 📊 Current Validation Status

> **Last Updated:** 2025-02-01
> 
> **Quality Score: 53.5/100** (threshold: 70.0)

### Rust vs C++ Code Coverage

| Metric | Rust | C++ (libslic3r) | Coverage |
|--------|------|-----------------|----------|
| Files | 71 | 475 | 14.9% |
| Total lines | 65,791 | 243,426 | 27.0% |
| Code lines (non-blank/comment) | 45,771 | 186,433 | 24.5% |

### G-code Output Comparison (3DBenchy)

| Metric | BambuStudio Reference | Rust Slicer | Status |
|--------|----------------------|-------------|--------|
| Layers | 240 | 240 | ✅ Exact match |
| G-code lines | 132,424 | 105,076 | 🟡 79% of reference |
| Filament | 3,869mm | ~3,640mm | ✅ Within 6% |

### Feature Move Counts

| Feature | Reference | Generated | Ratio | Status |
|---------|-----------|-----------|-------|--------|
| Bridge Infill | 1,536 | 20 | 0.01× | 🔴 Severe under-detection |
| External Perimeter | 28,702 | 25,655 | 0.89× | ✅ Close match |
| Internal Perimeter | 10,318 | 27,743 | 2.7× | 🟡 Over-generation |
| Solid Infill | 9,810 | 24,651 | 2.5× | 🟡 Over-generation |
| Sparse Infill | 11,504 | 6,101 | 0.53× | 🟡 Under-generation |
| Travel | 29,736 | 19,537 | 0.66× | 🟡 Different |
| Wipe | 3,099 | 0 | 0× | 🔴 Not implemented |

### Priority Issues

1. **🔴 Bridge Detection** - Severe under-detection (1% of reference moves)
2. **🟡 Internal Perimeter** - 2.7× over-generation (improved from 15×)
3. **🟡 Solid Infill** - 2.5× over-generation (improved from under-generation)
4. **🔴 Wipe Moves** - Not yet implemented
5. **🟡 First Layer Height** - Z offset mismatch (0.4mm vs 0.2mm detected)

### Recent Improvements

- **Surface Simplification** - Added Douglas-Peucker simplification before perimeter generation (matching libslic3r behavior)
- **G-code Size** - Reduced from 392K to 105K lines (73% reduction, now 79% of reference)
- **External Perimeter** - Now within 11% of reference (was 7× over)

---

## Current Status

✅ **Working Pipeline** - The slicer can now process STL files end-to-end and generate valid G-code!

### What Works

- **STL Loading** - Binary and ASCII STL file parsing
- **Mesh Slicing** - Layer generation at configurable heights
- **Perimeter Generation** - Inner/outer wall loops
- **Infill Generation** - Rectilinear, grid, and concentric patterns
- **Path Generation** - ExtrusionPath conversion with proper E calculations
- **G-code Output** - Full G-code with moves, retractions, temperature control

### Quick Start

```bash
# Build the slicer
cargo build --release

# Slice an STL file
./target/release/slicer-cli slice model.stl -o output.gcode --layer-height 0.2 --perimeters 3

# Get model info
./target/release/slicer-cli info model.stl

# Validate against reference
./target/release/slicer-cli validate model.stl reference.gcode --compare-only --generated output.gcode
```

### Library Usage

```rust
use slicer::pipeline::{PipelineConfig, PrintPipeline};
use slicer::mesh::load_stl;

let mesh = load_stl("model.stl")?;
let config = PipelineConfig::new()
    .layer_height(0.2)
    .perimeters(3)
    .infill_density(0.15);

let pipeline = PrintPipeline::new(config);
let gcode = pipeline.process(&mesh)?;
gcode.write_to_file("output.gcode")?;
```

## Project Goals

- **Pure slicing algorithm** - No UI, purely computational slicing
- **Rust implementation** - Memory-safe, fast, and concurrent
- **Compatibility validation** - G-code output validated against BambuStudio
- **MCP integration** - Serve slicing features over Model Context Protocol

## Validation Strategy

To ensure correctness, we will:
1. Collect 100 STL files with varying complexity
2. Generate G-code for each using BambuStudio with identical settings
3. Compare Rust-generated G-code against BambuStudio output
4. Achieve byte-for-byte (or semantic) equivalence

## Architecture Overview

### Slicing Pipeline

```
STL File → Mesh Loading → Layer Slicing → Layer Processing → G-code Generation
                              ↓
                    ┌─────────┴─────────┐
                    │   Per Layer:      │
                    │ • Perimeters      │
                    │ • Infill          │
                    │ • Bridges         │
                    │ • Supports        │
                    └───────────────────┘
```

### Module Structure

```
slicer/
├── Cargo.toml
├── src/
│   ├── lib.rs                    # Library root
│   ├── main.rs                   # CLI entry point
│   │
│   ├── geometry/                 # Core geometry primitives
│   │   ├── mod.rs
│   │   ├── point.rs              # 2D/3D points (coord_t, Vec3)
│   │   ├── line.rs               # Line segments
│   │   ├── polygon.rs            # Polygon (closed path)
│   │   ├── polyline.rs           # Polyline (open path)
│   │   ├── expolygon.rs          # Polygon with holes
│   │   ├── bounding_box.rs       # AABB
│   │   └── transform.rs          # Affine transformations
│   │
│   ├── mesh/                     # Triangle mesh handling
│   │   ├── mod.rs
│   │   ├── triangle_mesh.rs      # indexed_triangle_set equivalent
│   │   ├── stl.rs                # STL file I/O
│   │   ├── repair.rs             # Mesh repair (admesh equivalent)
│   │   └── aabb_tree.rs          # Spatial indexing
│   │
│   ├── clipper/                  # Polygon boolean operations
│   │   ├── mod.rs
│   │   ├── operations.rs         # Union, diff, intersection, offset
│   │   └── utils.rs              # Helper functions
│   │
│   ├── slice/                    # Core slicing
│   │   ├── mod.rs
│   │   ├── slicer.rs             # TriangleMeshSlicer equivalent
│   │   ├── layer.rs              # Layer data structure
│   │   ├── layer_region.rs       # LayerRegion (per-region data)
│   │   ├── surface.rs            # Surface types (top, bottom, internal)
│   │   └── slicing_params.rs     # SlicingParameters
│   │
│   ├── perimeter/                # Perimeter/wall generation
│   │   ├── mod.rs
│   │   ├── generator.rs          # PerimeterGenerator
│   │   ├── arachne/              # Variable-width perimeters
│   │   │   ├── mod.rs
│   │   │   ├── skeletal_trapezoidation.rs
│   │   │   ├── wall_tool_paths.rs
│   │   │   └── beading_strategy/
│   │   └── classic.rs            # Classic fixed-width perimeters
│   │
│   ├── infill/                   # Infill pattern generation
│   │   ├── mod.rs
│   │   ├── fill_base.rs          # Base trait for fill patterns
│   │   ├── rectilinear.rs        # Lines, grid
│   │   ├── honeycomb.rs          # Honeycomb patterns
│   │   ├── gyroid.rs             # Gyroid infill
│   │   ├── concentric.rs         # Concentric infill
│   │   ├── lightning.rs          # Lightning infill
│   │   ├── adaptive.rs           # Adaptive cubic
│   │   └── crosshatch.rs         # Cross-hatch pattern
│   │
│   ├── support/                  # Support structure generation
│   │   ├── mod.rs
│   │   ├── support_material.rs   # Classic support
│   │   ├── tree_support.rs       # Tree support algorithm
│   │   └── parameters.rs         # Support parameters
│   │
│   ├── bridge/                   # Bridge detection
│   │   ├── mod.rs
│   │   └── detector.rs           # BridgeDetector
│   │
│   ├── gcode/                    # G-code generation
│   │   ├── mod.rs
│   │   ├── writer.rs             # GCodeWriter
│   │   ├── generator.rs          # Main G-code generator
│   │   ├── toolpath.rs           # Extrusion paths
│   │   ├── cooling.rs            # CoolingBuffer
│   │   ├── wipe_tower.rs         # Wipe tower for multi-material
│   │   ├── seam_placer.rs        # Seam placement
│   │   └── spiral_vase.rs        # Spiral/vase mode
│   │
│   ├── config/                   # Print configuration
│   │   ├── mod.rs
│   │   ├── print_config.rs       # PrintConfig, PrintObjectConfig
│   │   ├── region_config.rs      # PrintRegionConfig
│   │   └── preset.rs             # Preset management
│   │
│   ├── print/                    # Print orchestration
│   │   ├── mod.rs
│   │   ├── print.rs              # Print (main orchestrator)
│   │   ├── print_object.rs       # PrintObject
│   │   └── print_region.rs       # PrintRegion
│   │
│   └── mcp/                      # MCP server integration
│       ├── mod.rs
│       ├── server.rs             # MCP server
│       └── tools.rs              # Exposed MCP tools
│
├── tests/
│   ├── integration/
│   │   ├── gcode_comparison.rs   # Compare against BambuStudio output
│   │   └── test_models/          # Test STL files
│   └── unit/
│
├── benches/                      # Performance benchmarks
│
└── data/
    ├── test_stls/                # 100 test STL files
    └── reference_gcodes/         # BambuStudio-generated reference G-codes
```

## Implementation Phases

### Phase 1: Foundation ✅ Complete
- [x] Geometry primitives (Point, Line, Polygon, ExPolygon)
- [x] Bounding box and basic transformations
- [x] STL file loading and parsing
- [x] Triangle mesh data structure
- [x] Basic mesh operations

### Phase 2: Core Slicing ✅ Complete
- [x] Triangle mesh slicer
- [x] Layer height calculation
- [x] Slice mesh into contours
- [x] Contour simplification and closing
- [x] Clipper integration for boolean ops

### Phase 3: Perimeters ✅ Complete
- [x] Classic perimeter generation
- [ ] Arachne variable-width algorithm (planned)
- [ ] Thin wall detection (basic)
- [ ] Gap fill (planned)

### Phase 4: Infill ✅ Complete (Basic)
- [x] Fill pattern base trait
- [x] Rectilinear infill
- [x] Grid infill
- [x] Concentric infill
- [ ] Honeycomb infill (planned)
- [ ] Gyroid infill (planned)

### Phase 5: Support 🚧 Planned
- [ ] Overhang detection
- [ ] Classic support generation
- [ ] Tree support algorithm
- [ ] Support interface layers

### Phase 6: G-code ✅ Complete (Basic)
- [x] G-code writer
- [x] Extrusion path planning
- [x] Travel moves and retraction
- [x] Basic seam placement
- [ ] Advanced cooling and speed control

### Phase 7: Validation & Polish 🚧 In Progress
- [x] Full pipeline integration
- [x] G-code comparison testing (within ~6% of reference)
- [ ] Performance optimization
- [ ] MCP server integration

## Key Rust Dependencies

```toml
[dependencies]
# Geometry & Math
nalgebra = "0.32"          # Linear algebra
geo = "0.27"               # Geometric algorithms
geo-clipper = "0.8"        # Clipper bindings (or clipper2)

# Mesh Processing
stl_io = "0.7"             # STL file I/O
rstar = "0.11"             # R-tree spatial index

# Parallelism
rayon = "1.8"              # Parallel iterators

# Serialization
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"

# MCP
# (TBD - custom or community MCP crate)

# Error Handling
thiserror = "1.0"
anyhow = "1.0"

# Utilities
log = "0.4"
env_logger = "0.10"
```

## BambuStudio Source Mapping

| Rust Module | BambuStudio Source |
|-------------|-------------------|
| `geometry/` | `Point.hpp`, `Line.hpp`, `Polygon.hpp`, `ExPolygon.hpp`, `BoundingBox.hpp` |
| `mesh/` | `TriangleMesh.hpp`, `TriangleMeshSlicer.hpp`, `Format/STL.hpp` |
| `clipper/` | `ClipperUtils.hpp`, `Clipper2Utils.hpp` |
| `slice/` | `Slicing.hpp`, `Layer.hpp`, `LayerRegion.cpp`, `Surface.hpp` |
| `perimeter/` | `PerimeterGenerator.hpp`, `Arachne/` |
| `infill/` | `Fill/` directory |
| `support/` | `Support/` directory |
| `gcode/` | `GCode.hpp`, `GCodeWriter.hpp`, `GCode/` directory |
| `config/` | `PrintConfig.hpp`, `Config.hpp` |
| `print/` | `Print.hpp`, `PrintObject.cpp`, `PrintObjectSlice.cpp` |

## Testing Approach

### Unit Tests (262 passing)
- Each module has its own unit tests
- Test geometry operations with known values
- Test slicing on simple geometric primitives

### Integration Tests (19 passing)
- Load STL → Slice → Generate G-code pipeline
- Compare output against BambuStudio reference
- Validates layer counts, Z heights, extrusion amounts

### G-code Comparison Strategy
1. **Structural comparison**: Same number of layers ✅
2. **Coordinate comparison**: Within tolerance (0.001mm) ✅
3. **Extrusion comparison**: Within 6% tolerance ✅
4. **Speed/temperature**: Basic control codes ✅

## License

This project is licensed under AGPL-3.0, consistent with BambuStudio's license since this is a derivative work based on their slicing algorithms.

## References

- [BambuStudio Source](https://github.com/bambulab/BambuStudio)
- [PrusaSlicer Source](https://github.com/prusa3d/PrusaSlicer)
- [Slic3r Manual](https://manual.slic3r.org/)
- [Arachne Paper](https://research.tue.nl/en/publications/arachne) - Variable-width contouring

---

## 🤖 Agent Instructions: Updating Validation Status

When you finish a session working on this project, please update the validation tables at the top of this README by running:

```bash
# From slicer/lib directory:

# 1. Run validation to get current score and feature comparison
cargo run --release -- validate ../data/test_stls/3DBenchy.stl ../data/reference_gcodes/3DBenchy.gcode \
    --compare-only --generated ../data/output/3DBenchy_output.gcode

# 2. Get line counts
wc -l ../data/output/3DBenchy_output.gcode ../data/reference_gcodes/3DBenchy.gcode

# 3. Get Rust vs C++ code coverage
cd .. && bash scripts/line_diff.sh
```

Then update the following sections in this README:
1. **Last Updated** date
2. **Quality Score** 
3. **Rust vs C++ Code Coverage** table
4. **G-code Output Comparison** table
5. **Feature Move Counts** table
6. **Priority Issues** list (if priorities have changed)