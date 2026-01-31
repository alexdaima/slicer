# Rust Slicer - BambuStudio libslic3r Rewrite

## Project Overview

This project is a **complete Rust rewrite** of BambuStudio's `libslic3r` library - the core slicing engine that converts 3D models into G-code for FDM 3D printers.

### Goals

1. **100% Functional Parity**: Produce identical slicing results to BambuStudio/libslic3r
2. **Clean Architecture**: Modular, well-documented, easily maintainable codebase
3. **Type Safety**: Leverage Rust's type system to prevent runtime errors
4. **Performance**: Match or exceed C++ performance through safe concurrency
5. **No GUI Dependencies**: Pure algorithm library, usable in any context

### Non-Goals

- GUI implementation (that's a separate project)
- Byte-for-byte identical G-code (semantic equivalence is the target)
- SLA/resin printing support (FDM only for now)

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                           Input                                      │
│                     TriangleMesh (STL/3MF)                           │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        mesh/ Module                                  │
│              STL loading, mesh repair, transformations               │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        slice/ Module                                 │
│         Horizontal slicing → Layer polygons at each Z height         │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      perimeter/ Module                               │
│         Generate perimeter shells (Classic or Arachne mode)          │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        infill/ Module                                │
│              Generate infill patterns for interior regions           │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       support/ Module                                │
│          Generate support structures for overhangs                   │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        gcode/ Module                                 │
│         Convert toolpaths to G-code with proper extrusion            │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                           Output                                     │
│                      G-code file for printer                         │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Module Reference

Each module has its own `AGENTS.md` with detailed documentation. Here's the quick reference:

| Module | Purpose | libslic3r Equivalent |
|--------|---------|---------------------|
| `geometry/` | Core types: Point, Polygon, ExPolygon, Line | `Point.cpp`, `Polygon.cpp`, `ExPolygon.cpp`, `Line.cpp` |
| `geometry/simplify` | Path simplification (Douglas-Peucker, resolution-based) | `MultiPoint.cpp`, `Arachne/utils/ExtrusionLine.cpp` |
| `mesh/` | Triangle mesh loading and processing | `TriangleMesh.cpp`, STL format handlers |
| `slice/` | Mesh slicing into layers | `TriangleMeshSlicer.cpp`, `Slicing.cpp`, `Layer.cpp` |
| `perimeter/` | Perimeter/shell generation | `PerimeterGenerator.cpp` |
| `perimeter/arachne/` | Variable-width perimeters | `Arachne/` directory |
| `infill/` | Infill pattern generation | `Fill/` directory |
| `support/` | Support structure generation | `Support/` directory |
| `bridge/` | Bridge detection and handling | `BridgeDetector.cpp` |
| `flow/` | Extrusion flow calculations | `Flow.cpp` |
| `gcode/` | G-code generation and writing | `GCode.cpp`, `GCodeWriter.cpp`, `GCode/` directory |
| `adhesion/` | Brim, skirt, raft generation | `Brim.cpp` + Print methods |
| `config/` | Print configuration | `PrintConfig.cpp` |
| `clipper/` | Polygon boolean operations | `ClipperUtils.cpp`, `Clipper2Utils.cpp` |
| `pipeline/` | Orchestrates the full slicing process | `Print.cpp`, `PrintObject.cpp` |
| `print/` | Print and PrintObject types | `Print.hpp`, `PrintObject.cpp` |

---

## Critical Implementation Details

### Coordinate System

```rust
// Coordinates are scaled integers for numerical stability
pub type Coord = i64;      // Scaled coordinates (internal)
pub type CoordF = f64;     // Floating-point mm values (external)

pub const SCALING_FACTOR: f64 = 1_000_000.0;  // 1mm = 1,000,000 units

pub fn scale(mm: f64) -> Coord { (mm * SCALING_FACTOR).round() as Coord }
pub fn unscale(coord: Coord) -> f64 { coord as f64 / SCALING_FACTOR }
```

**Why**: Floating-point operations can accumulate errors. By using scaled integers internally (1 unit = 1 nanometer), we get exact geometric operations.

### Flow Calculations (CRITICAL)

The extrusion flow calculation must match libslic3r **exactly**. The cross-section of extruded plastic is a **rounded rectangle** (stadium shape), not a simple rectangle:

```
Normal extrusion:  area = height × (width - height × (1 - π/4))
Bridge extrusion:  area = π × (width/2)²
```

**WARNING**: Using `width × height` gives 10-15% error! Always use the `flow/` module.

### libslic3r Mapping Strategy

For each feature:

1. **Find the C++ source** in BambuStudio/src/libslic3r/
2. **Understand the algorithm** - read the code, comments, and any related papers
3. **Port to Rust** - maintain algorithmic fidelity, improve code structure
4. **Document the mapping** in the module's AGENTS.md
5. **Test for parity** - compare output against BambuStudio reference

---

## Development Guidelines

### Adding a New Feature

1. Create the module directory under `src/`
2. Create `AGENTS.md` documenting:
   - Purpose of the module
   - libslic3r file mapping
   - Key algorithms and formulas
   - Public API
3. Implement with tests
4. Update this root AGENTS.md
5. Update `lib.rs` exports

### Code Style

- **Explicit over implicit**: Prefer verbose, clear code over clever one-liners
- **Document formulas**: Any math from libslic3r should have comments explaining it
- **Test coverage**: Every public function should have unit tests
- **Error handling**: Use `Result<T, Error>` for fallible operations

### Testing Strategy

1. **Unit tests**: In each module's `mod.rs` or dedicated test files
2. **Integration tests**: In `tests/` directory, run full pipeline
3. **Parity tests**: Compare against BambuStudio reference G-code
4. **Benchmarks**: For performance-critical code

Run tests:
```bash
cargo test              # All tests
cargo test --lib        # Library tests only
cargo test flow::       # Tests matching pattern
```

---

## libslic3r Source Reference

The reference implementation is in `BambuStudio/src/libslic3r/`. Key files:

### Core Types
- `Point.hpp/cpp` - 2D/3D points
- `Line.hpp/cpp` - Line segments
- `Polygon.hpp/cpp` - Closed polygons
- `Polyline.hpp/cpp` - Open polylines
- `ExPolygon.hpp/cpp` - Polygon with holes
- `BoundingBox.hpp/cpp` - Axis-aligned bounding boxes

### Slicing Pipeline
- `TriangleMesh.hpp/cpp` - Mesh data structure
- `TriangleMeshSlicer.hpp/cpp` - Mesh → layer slices
- `Layer.hpp/cpp` - Layer data structure
- `Surface.hpp/cpp` - Classified surfaces (top, bottom, etc.)

### Toolpath Generation
- `PerimeterGenerator.hpp/cpp` - Perimeter shells
- `Fill/*.cpp` - Infill patterns
- `BridgeDetector.hpp/cpp` - Bridge detection
- `Support/*.cpp` - Support structures

### G-code
- `Flow.hpp/cpp` - **CRITICAL** - Extrusion calculations
- `GCode.hpp/cpp` - G-code generation orchestration
- `GCodeWriter.hpp/cpp` - G-code command writing
- `GCode/CoolingBuffer.cpp` - Layer time and fan control
- `GCode/SpiralVase.cpp` - Vase mode

### Configuration
- `PrintConfig.hpp/cpp` - All print settings
- `Print.hpp/cpp` - Print job management
- `PrintObject.cpp` - Per-object processing

---

## Current Status

See `REWRITE_PROGRESS.md` for detailed feature tracking.

### Completed ✅
- Core geometry types (Point, Polygon, ExPolygon, Line, Polyline)
- Mesh loading (STL)
- Basic slicing pipeline
- Perimeter generation (Classic + Arachne)
- Multiple infill patterns (Rectilinear, Grid, Honeycomb, Gyroid, Lightning, Concentric)
- Bridge detection
- Basic support generation
- G-code writing with arc fitting
- Flow calculations (corrected)
- Cooling buffer
- Spiral vase mode
- Brim and skirt

### In Progress 🔄
- Tree support improvements
- Full parity testing

### Remaining 📋
- Avoid crossing perimeters
- Pressure equalizer
- Seam painting
- Ironing
- Fuzzy skin
- Adaptive slicing
- Multi-material support
- Wipe tower

---

## Directory Structure

```
slicer/
├── AGENTS.md              # This file - project overview
├── REWRITE_PROGRESS.md    # Detailed progress tracking
├── README.md              # User-facing documentation
├── Cargo.toml             # Rust dependencies
├── src/
│   ├── lib.rs             # Library entry point and exports
│   ├── main.rs            # CLI application
│   ├── adhesion/          # Brim, skirt, raft
│   │   ├── AGENTS.md
│   │   └── mod.rs
│   ├── bridge/            # Bridge detection
│   │   ├── AGENTS.md
│   │   └── mod.rs
│   ├── clipper/           # Boolean operations
│   │   ├── AGENTS.md
│   │   └── mod.rs
│   ├── config/            # Configuration types
│   │   ├── AGENTS.md
│   │   ├── mod.rs
│   │   ├── print_config.rs
│   │   └── region_config.rs
│   ├── flow/              # Extrusion calculations
│   │   ├── AGENTS.md
│   │   └── mod.rs
│   ├── gcode/             # G-code generation
│   │   ├── AGENTS.md
│   │   ├── mod.rs
│   │   ├── arc_fitting.rs
│   │   ├── compare.rs
│   │   ├── cooling.rs
│   │   ├── generator.rs
│   │   ├── path.rs
│   │   ├── spiral_vase.rs
│   │   └── writer.rs
│   ├── geometry/          # Core geometry types
│   │   ├── AGENTS.md
│   │   ├── mod.rs
│   │   ├── bounding_box.rs
│   │   ├── elephant_foot.rs
│   │   ├── expolygon.rs
│   │   ├── line.rs
│   │   ├── point.rs
│   │   ├── polygon.rs
│   │   ├── polyline.rs
│   │   └── transform.rs
│   ├── infill/            # Infill patterns
│   │   ├── AGENTS.md
│   │   └── mod.rs
│   ├── mesh/              # Mesh handling
│   │   ├── AGENTS.md
│   │   ├── mod.rs
│   │   └── stl.rs
│   ├── perimeter/         # Perimeter generation
│   │   ├── AGENTS.md
│   │   ├── mod.rs
│   │   └── arachne/       # Variable-width
│   │       ├── AGENTS.md
│   │       ├── mod.rs
│   │       ├── beading.rs
│   │       ├── junction.rs
│   │       └── line.rs
│   ├── pipeline/          # Slicing orchestration
│   │   ├── AGENTS.md
│   │   └── mod.rs
│   ├── print/             # Print objects
│   │   ├── AGENTS.md
│   │   └── mod.rs
│   ├── slice/             # Mesh slicing
│   │   ├── AGENTS.md
│   │   ├── mod.rs
│   │   ├── layer.rs
│   │   ├── mesh_slicer.rs
│   │   ├── slicer.rs
│   │   ├── slicing_params.rs
│   │   └── surface.rs
│   └── support/           # Support generation
│       ├── AGENTS.md
│       └── mod.rs
└── tests/
    ├── README.md
    └── benchy_integration.rs
```

---

## Contributing

1. **Pick a feature** from the "Remaining" list above
2. **Study the C++ implementation** in BambuStudio
3. **Create a branch** with descriptive name
4. **Implement with tests** and AGENTS.md documentation
5. **Run parity tests** against reference output
6. **Submit PR** with clear description of changes

---

## Questions?

- Check the module-specific AGENTS.md files for detailed documentation
- Look at the C++ source in BambuStudio for reference
- Run tests to see expected behavior