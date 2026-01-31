//! Slicing module - converts meshes into layers.
//!
//! This module contains the core slicing functionality:
//! - [`Slicer`] - Main slicing engine
//! - [`Layer`] - Represents a single layer of the print
//! - [`SlicingParams`] - Configuration for the slicing process
//! - [`mesh_slicer`] - Low-level mesh-plane intersection algorithms
//! - [`adaptive_heights`] - Adaptive layer height computation

mod adaptive_heights;
mod layer;
mod mesh_slicer;
mod slicer;
mod slicing_params;
mod surface;

pub use adaptive_heights::{
    compute_adaptive_heights, compute_adaptive_heights_with_quality, AdaptiveHeightsConfig,
    AdaptiveLayerHeight, AdaptiveSlicing, FaceZ, SlopeErrorMetric,
};
pub use layer::{Layer, LayerRegion};
pub use mesh_slicer::{slice_mesh, slice_mesh_at_z};
pub use slicer::Slicer;
pub use slicing_params::SlicingParams;
pub use surface::{
    detect_all_surface_types, detect_surface_types, propagate_solid_infill, Surface,
    SurfaceCollection, SurfaceDetectionConfig, SurfaceType,
};
