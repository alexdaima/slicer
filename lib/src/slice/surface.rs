//! Surface types for layer regions.
//!
//! This module provides the Surface type representing classified regions
//! within a layer, mirroring BambuStudio's Surface class.
//!
//! # Surface Type Detection
//!
//! Surface types are detected by comparing the current layer's geometry with
//! adjacent layers:
//!
//! - **Top**: Areas of the current layer not covered by the layer above
//! - **Bottom**: Areas of the current layer not supported by the layer below
//! - **BottomBridge**: Bottom areas that span over air (no support below)
//! - **Internal**: Areas covered both above and below (get sparse infill)
//! - **InternalSolid**: Internal areas that need solid infill (near top/bottom)
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/Surface.hpp/cpp`
//! - `PrintObject::detect_surfaces_type()` in `PrintObject.cpp`

use crate::clipper::{difference, intersection, opening, union_ex, OffsetJoinType};
use crate::geometry::{ExPolygon, ExPolygons};
use crate::CoordF;
use serde::{Deserialize, Serialize};
use std::fmt;

/// Classification of a surface within a layer.
///
/// Surfaces are classified to determine how they should be filled:
/// - Top/bottom surfaces get solid infill
/// - Internal surfaces get sparse infill
/// - Bridge surfaces need special handling
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SurfaceType {
    /// Top surface (visible from above).
    Top,
    /// Bottom surface (visible from below, or first layer).
    Bottom,
    /// Bottom surface that bridges over air/support.
    BottomBridge,
    /// Internal solid surface (between top/bottom and infill).
    #[default]
    InternalSolid,
    /// Internal surface that will receive sparse infill.
    Internal,
    /// Internal bridge surface.
    InternalBridge,
    /// Internal void (empty space, no infill).
    InternalVoid,
}

impl SurfaceType {
    /// Check if this surface type is a top surface.
    #[inline]
    pub fn is_top(&self) -> bool {
        matches!(self, SurfaceType::Top)
    }

    /// Check if this surface type is a bottom surface.
    #[inline]
    pub fn is_bottom(&self) -> bool {
        matches!(self, SurfaceType::Bottom | SurfaceType::BottomBridge)
    }

    /// Check if this surface type is a bridge.
    #[inline]
    pub fn is_bridge(&self) -> bool {
        matches!(
            self,
            SurfaceType::BottomBridge | SurfaceType::InternalBridge
        )
    }

    /// Check if this surface type requires solid infill.
    #[inline]
    pub fn is_solid(&self) -> bool {
        matches!(
            self,
            SurfaceType::Top
                | SurfaceType::Bottom
                | SurfaceType::BottomBridge
                | SurfaceType::InternalSolid
                | SurfaceType::InternalBridge
        )
    }

    /// Check if this surface type is internal (not top or bottom).
    #[inline]
    pub fn is_internal(&self) -> bool {
        matches!(
            self,
            SurfaceType::Internal
                | SurfaceType::InternalSolid
                | SurfaceType::InternalBridge
                | SurfaceType::InternalVoid
        )
    }

    /// Check if this surface type is external (top or bottom).
    #[inline]
    pub fn is_external(&self) -> bool {
        !self.is_internal()
    }

    /// Get a human-readable name for this surface type.
    pub fn name(&self) -> &'static str {
        match self {
            SurfaceType::Top => "top",
            SurfaceType::Bottom => "bottom",
            SurfaceType::BottomBridge => "bottom bridge",
            SurfaceType::InternalSolid => "internal solid",
            SurfaceType::Internal => "internal",
            SurfaceType::InternalBridge => "internal bridge",
            SurfaceType::InternalVoid => "internal void",
        }
    }
}

impl fmt::Display for SurfaceType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.name())
    }
}

/// A surface is a classified region within a layer.
///
/// Each surface has a type (determining how it should be filled)
/// and geometry (the ExPolygon defining its shape).
#[derive(Clone, Default, Serialize, Deserialize)]
pub struct Surface {
    /// The geometry of this surface.
    pub expolygon: ExPolygon,

    /// The type/classification of this surface.
    pub surface_type: SurfaceType,

    /// Thickness of this surface (layer height), in mm.
    pub thickness: CoordF,

    /// Thickness of the layer below, in mm (for bridge calculations).
    pub thickness_layers: usize,

    /// Bridge angle in radians (for bridge surfaces).
    /// None if not a bridge or angle not yet determined.
    pub bridge_angle: Option<CoordF>,

    /// Extra perimeters needed for this surface.
    pub extra_perimeters: usize,
}

impl Surface {
    /// Create a new surface with the given geometry and type.
    pub fn new(expolygon: ExPolygon, surface_type: SurfaceType) -> Self {
        Self {
            expolygon,
            surface_type,
            thickness: 0.0,
            thickness_layers: 1,
            bridge_angle: None,
            extra_perimeters: 0,
        }
    }

    /// Create a new top surface.
    pub fn top(expolygon: ExPolygon) -> Self {
        Self::new(expolygon, SurfaceType::Top)
    }

    /// Create a new bottom surface.
    pub fn bottom(expolygon: ExPolygon) -> Self {
        Self::new(expolygon, SurfaceType::Bottom)
    }

    /// Create a new internal surface.
    pub fn internal(expolygon: ExPolygon) -> Self {
        Self::new(expolygon, SurfaceType::Internal)
    }

    /// Create a new internal solid surface.
    pub fn internal_solid(expolygon: ExPolygon) -> Self {
        Self::new(expolygon, SurfaceType::InternalSolid)
    }

    /// Create a new bridge surface.
    pub fn bridge(expolygon: ExPolygon, angle: Option<CoordF>) -> Self {
        Self {
            expolygon,
            surface_type: SurfaceType::BottomBridge,
            thickness: 0.0,
            thickness_layers: 1,
            bridge_angle: angle,
            extra_perimeters: 0,
        }
    }

    /// Check if this surface is empty (no geometry).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.expolygon.is_empty()
    }

    /// Get the area of this surface.
    #[inline]
    pub fn area(&self) -> CoordF {
        self.expolygon.area()
    }

    /// Check if this is a top surface.
    #[inline]
    pub fn is_top(&self) -> bool {
        self.surface_type.is_top()
    }

    /// Check if this is a bottom surface.
    #[inline]
    pub fn is_bottom(&self) -> bool {
        self.surface_type.is_bottom()
    }

    /// Check if this is a bridge surface.
    #[inline]
    pub fn is_bridge(&self) -> bool {
        self.surface_type.is_bridge()
    }

    /// Check if this is a solid surface.
    #[inline]
    pub fn is_solid(&self) -> bool {
        self.surface_type.is_solid()
    }

    /// Check if this is an internal surface.
    #[inline]
    pub fn is_internal(&self) -> bool {
        self.surface_type.is_internal()
    }

    /// Check if this is an external surface.
    #[inline]
    pub fn is_external(&self) -> bool {
        self.surface_type.is_external()
    }

    /// Set the surface type.
    pub fn set_type(&mut self, surface_type: SurfaceType) {
        self.surface_type = surface_type;
    }

    /// Set the bridge angle.
    pub fn set_bridge_angle(&mut self, angle: CoordF) {
        self.bridge_angle = Some(angle);
    }

    /// Set the thickness.
    pub fn set_thickness(&mut self, thickness: CoordF) {
        self.thickness = thickness;
    }

    /// Set the number of thickness layers.
    pub fn set_thickness_layers(&mut self, layers: usize) {
        self.thickness_layers = layers;
    }
}

impl fmt::Debug for Surface {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Surface({:?}, area={:.2}mm²)",
            self.surface_type,
            self.area()
        )
    }
}

impl fmt::Display for Surface {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} surface (area={:.2}mm²)",
            self.surface_type,
            self.area()
        )
    }
}

impl From<ExPolygon> for Surface {
    fn from(expolygon: ExPolygon) -> Self {
        Self::new(expolygon, SurfaceType::default())
    }
}

/// Type alias for a collection of surfaces.
pub type Surfaces = Vec<Surface>;

/// Collection of surfaces with utility methods.
#[derive(Clone, Default, Serialize, Deserialize)]
pub struct SurfaceCollection {
    /// The surfaces in this collection.
    pub surfaces: Vec<Surface>,
}

impl SurfaceCollection {
    /// Create a new empty surface collection.
    pub fn new() -> Self {
        Self {
            surfaces: Vec::new(),
        }
    }

    /// Create a surface collection from a vector of surfaces.
    pub fn from_surfaces(surfaces: Vec<Surface>) -> Self {
        Self { surfaces }
    }

    /// Check if the collection is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.surfaces.is_empty()
    }

    /// Get the number of surfaces.
    #[inline]
    pub fn len(&self) -> usize {
        self.surfaces.len()
    }

    /// Add a surface to the collection.
    pub fn push(&mut self, surface: Surface) {
        self.surfaces.push(surface);
    }

    /// Clear all surfaces.
    pub fn clear(&mut self) {
        self.surfaces.clear();
    }

    /// Get all surfaces of a specific type.
    pub fn filter_by_type(&self, surface_type: SurfaceType) -> Vec<&Surface> {
        self.surfaces
            .iter()
            .filter(|s| s.surface_type == surface_type)
            .collect()
    }

    /// Get all top surfaces.
    pub fn top_surfaces(&self) -> Vec<&Surface> {
        self.surfaces.iter().filter(|s| s.is_top()).collect()
    }

    /// Get all bottom surfaces.
    pub fn bottom_surfaces(&self) -> Vec<&Surface> {
        self.surfaces.iter().filter(|s| s.is_bottom()).collect()
    }

    /// Get all solid surfaces.
    pub fn solid_surfaces(&self) -> Vec<&Surface> {
        self.surfaces.iter().filter(|s| s.is_solid()).collect()
    }

    /// Get all bridge surfaces.
    pub fn bridge_surfaces(&self) -> Vec<&Surface> {
        self.surfaces.iter().filter(|s| s.is_bridge()).collect()
    }

    /// Get the total area of all surfaces.
    pub fn total_area(&self) -> CoordF {
        self.surfaces.iter().map(|s| s.area()).sum()
    }

    /// Check if any surface has the given type.
    pub fn has_type(&self, surface_type: SurfaceType) -> bool {
        self.surfaces.iter().any(|s| s.surface_type == surface_type)
    }
}

impl fmt::Display for SurfaceCollection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SurfaceCollection({} surfaces)", self.surfaces.len())
    }
}

/// Detect surface types for a layer by comparing with adjacent layers.
///
/// This implements the core surface classification algorithm from BambuStudio's
/// `PrintObject::detect_surfaces_type()`.
///
/// # Arguments
///
/// * `current_slices` - The ExPolygons of the current layer
/// * `lower_slices` - The ExPolygons of the layer below (if any)
/// * `upper_slices` - The ExPolygons of the layer above (if any)
/// * `offset` - Small offset for robust intersection (typically flow width / 10)
///
/// # Returns
///
/// A vector of Surfaces with appropriate types assigned.
///
/// # Algorithm
///
/// 1. Find TOP surfaces: areas of current layer not covered by upper layer
/// 2. Find BOTTOM surfaces: areas of current layer not supported by lower layer
/// 3. Find BOTTOM_BRIDGE surfaces: bottom areas completely over air
/// 4. Remaining areas are INTERNAL
pub fn detect_surface_types(
    current_slices: &ExPolygons,
    lower_slices: Option<&ExPolygons>,
    upper_slices: Option<&ExPolygons>,
    offset: CoordF,
) -> Vec<Surface> {
    if current_slices.is_empty() {
        return Vec::new();
    }

    let mut surfaces = Vec::new();

    // Find TOP surfaces: difference between current and upper
    let top_expolygons = if let Some(upper) = upper_slices {
        if !upper.is_empty() {
            // Top = areas not covered by the layer above
            let diff = difference(current_slices, upper);
            // Apply small opening to remove noise
            if offset > 0.0 {
                opening(&diff, offset, OffsetJoinType::Miter)
            } else {
                diff
            }
        } else {
            // No upper layer geometry - entire layer is top
            current_slices.clone()
        }
    } else {
        // No upper layer - entire layer is top
        current_slices.clone()
    };

    // Find BOTTOM surfaces: difference between current and lower
    let (bottom_expolygons, bottom_bridge_expolygons) = if let Some(lower) = lower_slices {
        if !lower.is_empty() {
            // Bottom = areas not supported by the layer below
            let diff = difference(current_slices, lower);
            let bottom = if offset > 0.0 {
                opening(&diff, offset, OffsetJoinType::Miter)
            } else {
                diff
            };
            // All bottom areas without support below are bridges
            (Vec::new(), bottom)
        } else {
            // Lower layer exists but is empty - this is a bridge over void
            (Vec::new(), current_slices.clone())
        }
    } else {
        // First layer - all is bottom (on build plate)
        (current_slices.clone(), Vec::new())
    };

    // Collect top and bottom polygons for computing internal areas
    let mut top_bottom_polygons: Vec<crate::geometry::Polygon> = Vec::new();

    // Add top surfaces
    for expoly in &top_expolygons {
        if expoly.area().abs() > 1.0 {
            // Filter tiny areas
            top_bottom_polygons.push(expoly.contour.clone());
            for hole in &expoly.holes {
                top_bottom_polygons.push(hole.clone());
            }
            surfaces.push(Surface::top(expoly.clone()));
        }
    }

    // Add bottom surfaces (on build plate)
    for expoly in &bottom_expolygons {
        if expoly.area().abs() > 1.0 {
            top_bottom_polygons.push(expoly.contour.clone());
            for hole in &expoly.holes {
                top_bottom_polygons.push(hole.clone());
            }
            surfaces.push(Surface::bottom(expoly.clone()));
        }
    }

    // Add bridge surfaces
    for expoly in &bottom_bridge_expolygons {
        if expoly.area().abs() > 1.0 {
            top_bottom_polygons.push(expoly.contour.clone());
            for hole in &expoly.holes {
                top_bottom_polygons.push(hole.clone());
            }
            surfaces.push(Surface::bridge(expoly.clone(), None));
        }
    }

    // Handle overlapping top and bottom (thin membranes)
    // If areas are both top and bottom, prefer bottom (allows bridge detection)
    if !top_expolygons.is_empty()
        && (!bottom_expolygons.is_empty() || !bottom_bridge_expolygons.is_empty())
    {
        let all_bottom: ExPolygons = bottom_expolygons
            .iter()
            .chain(bottom_bridge_expolygons.iter())
            .cloned()
            .collect();

        // Remove overlapping areas from top
        let top_only = difference(&top_expolygons, &all_bottom);

        // Rebuild surfaces with non-overlapping top
        surfaces.retain(|s| !s.is_top());
        for expoly in top_only {
            if expoly.area().abs() > 1.0 {
                surfaces.push(Surface::top(expoly));
            }
        }
    }

    // Find INTERNAL surfaces: areas that are neither top nor bottom
    // Convert surfaces to ExPolygons for difference operation
    let classified: ExPolygons = surfaces.iter().map(|s| s.expolygon.clone()).collect();

    let internal_expolygons = if !classified.is_empty() {
        difference(current_slices, &classified)
    } else {
        // No surfaces classified yet - if we have both upper and lower layers,
        // and both have geometry covering us, then the whole slice is internal
        if upper_slices.is_some() && lower_slices.is_some() {
            // Check if we're fully covered by both
            let upper = upper_slices.unwrap();
            let lower = lower_slices.unwrap();
            if !upper.is_empty() && !lower.is_empty() {
                // Both layers have geometry - classify remaining as internal
                current_slices.to_vec()
            } else {
                Vec::new()
            }
        } else {
            Vec::new()
        }
    };

    // Add internal surfaces
    for expoly in internal_expolygons {
        if expoly.area().abs() > 1.0 {
            surfaces.push(Surface::internal(expoly));
        }
    }

    surfaces
}

/// Detect surface types for multiple layers in parallel.
///
/// This processes all layers and assigns surface types based on
/// comparison with adjacent layers.
///
/// # Arguments
///
/// * `layer_slices` - Slice geometry for each layer
/// * `offset` - Small offset for robust intersection
///
/// # Returns
///
/// A vector of Surface vectors, one per layer.
pub fn detect_all_surface_types(layer_slices: &[ExPolygons], offset: CoordF) -> Vec<Vec<Surface>> {
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

/// Configuration for surface type detection.
#[derive(Debug, Clone)]
pub struct SurfaceDetectionConfig {
    /// Number of solid top layers.
    pub top_solid_layers: usize,
    /// Number of solid bottom layers.
    pub bottom_solid_layers: usize,
    /// Offset for robust intersection (typically flow width / 10).
    pub offset: CoordF,
    /// Minimum area for a surface to be kept (mm²).
    pub min_area: CoordF,
}

impl Default for SurfaceDetectionConfig {
    fn default() -> Self {
        Self {
            top_solid_layers: 3,
            bottom_solid_layers: 3,
            offset: 0.045, // ~0.45mm / 10
            min_area: 0.5, // 0.5 mm²
        }
    }
}

/// Propagate solid infill through layers.
///
/// This implements the "discover_vertical_shells" logic from BambuStudio.
/// Ensures that solid infill extends through the configured number of layers
/// below top surfaces and above bottom surfaces.
///
/// # Arguments
///
/// * `surfaces` - Mutable surface vectors for each layer
/// * `config` - Surface detection configuration
pub fn propagate_solid_infill(surfaces: &mut [Vec<Surface>], config: &SurfaceDetectionConfig) {
    let num_layers = surfaces.len();
    if num_layers == 0 {
        return;
    }

    // Propagate from top surfaces downward
    for layer_idx in 0..num_layers {
        // Find top surfaces at this layer
        let top_regions: ExPolygons = surfaces[layer_idx]
            .iter()
            .filter(|s| s.is_top())
            .map(|s| s.expolygon.clone())
            .collect();

        if top_regions.is_empty() {
            continue;
        }

        // Propagate solid infill to layers below (up to top_solid_layers - 1)
        for offset in 1..config.top_solid_layers {
            if layer_idx < offset {
                break;
            }
            let target_layer = layer_idx - offset;

            // Find internal surfaces that overlap with top regions
            for surface in &mut surfaces[target_layer] {
                if surface.surface_type == SurfaceType::Internal {
                    let overlap = intersection(&[surface.expolygon.clone()], &top_regions);
                    if !overlap.is_empty() {
                        // This internal surface should be solid
                        surface.surface_type = SurfaceType::InternalSolid;
                    }
                }
            }
        }
    }

    // Propagate from bottom surfaces upward
    for layer_idx in 0..num_layers {
        // Find bottom surfaces at this layer
        let bottom_regions: ExPolygons = surfaces[layer_idx]
            .iter()
            .filter(|s| s.is_bottom())
            .map(|s| s.expolygon.clone())
            .collect();

        if bottom_regions.is_empty() {
            continue;
        }

        // Propagate solid infill to layers above (up to bottom_solid_layers - 1)
        for offset in 1..config.bottom_solid_layers {
            let target_layer = layer_idx + offset;
            if target_layer >= num_layers {
                break;
            }

            // Find internal surfaces that overlap with bottom regions
            for surface in &mut surfaces[target_layer] {
                if surface.surface_type == SurfaceType::Internal {
                    let overlap = intersection(&[surface.expolygon.clone()], &bottom_regions);
                    if !overlap.is_empty() {
                        // This internal surface should be solid
                        surface.surface_type = SurfaceType::InternalSolid;
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};

    fn make_square_expolygon() -> ExPolygon {
        let poly = Polygon::rectangle(Point::new(0, 0), Point::new(1000000, 1000000));
        ExPolygon::new(poly)
    }

    #[test]
    fn test_surface_type_classification() {
        assert!(SurfaceType::Top.is_top());
        assert!(!SurfaceType::Top.is_bottom());
        assert!(SurfaceType::Top.is_solid());
        assert!(SurfaceType::Top.is_external());

        assert!(SurfaceType::Bottom.is_bottom());
        assert!(SurfaceType::Bottom.is_solid());
        assert!(SurfaceType::Bottom.is_external());

        assert!(SurfaceType::BottomBridge.is_bottom());
        assert!(SurfaceType::BottomBridge.is_bridge());
        assert!(SurfaceType::BottomBridge.is_solid());

        assert!(SurfaceType::Internal.is_internal());
        assert!(!SurfaceType::Internal.is_solid());

        assert!(SurfaceType::InternalSolid.is_internal());
        assert!(SurfaceType::InternalSolid.is_solid());

        assert!(SurfaceType::InternalBridge.is_bridge());
        assert!(SurfaceType::InternalBridge.is_solid());
    }

    #[test]
    fn test_surface_new() {
        let expoly = make_square_expolygon();
        let surface = Surface::new(expoly, SurfaceType::Top);

        assert!(surface.is_top());
        assert!(surface.is_solid());
        assert!(!surface.is_empty());
        assert!(surface.area() > 0.0);
    }

    #[test]
    fn test_surface_constructors() {
        let expoly = make_square_expolygon();

        let top = Surface::top(expoly.clone());
        assert!(top.is_top());

        let bottom = Surface::bottom(expoly.clone());
        assert!(bottom.is_bottom());

        let internal = Surface::internal(expoly.clone());
        assert!(internal.is_internal());
        assert!(!internal.is_solid());

        let bridge = Surface::bridge(expoly.clone(), Some(0.5));
        assert!(bridge.is_bridge());
        assert_eq!(bridge.bridge_angle, Some(0.5));
    }

    #[test]
    fn test_surface_setters() {
        let expoly = make_square_expolygon();
        let mut surface = Surface::new(expoly, SurfaceType::Internal);

        surface.set_type(SurfaceType::Top);
        assert!(surface.is_top());

        surface.set_bridge_angle(1.5);
        assert_eq!(surface.bridge_angle, Some(1.5));

        surface.set_thickness(0.2);
        assert!((surface.thickness - 0.2).abs() < 1e-6);

        surface.set_thickness_layers(3);
        assert_eq!(surface.thickness_layers, 3);
    }

    #[test]
    fn test_surface_collection() {
        let expoly = make_square_expolygon();

        let mut collection = SurfaceCollection::new();
        assert!(collection.is_empty());

        collection.push(Surface::top(expoly.clone()));
        collection.push(Surface::bottom(expoly.clone()));
        collection.push(Surface::internal(expoly.clone()));

        assert_eq!(collection.len(), 3);
        assert!(!collection.is_empty());

        assert_eq!(collection.top_surfaces().len(), 1);
        assert_eq!(collection.bottom_surfaces().len(), 1);
        assert_eq!(collection.solid_surfaces().len(), 2); // top + bottom

        assert!(collection.has_type(SurfaceType::Top));
        assert!(collection.has_type(SurfaceType::Bottom));
        assert!(collection.has_type(SurfaceType::Internal));
        assert!(!collection.has_type(SurfaceType::InternalBridge));
    }

    #[test]
    fn test_surface_collection_filter() {
        let expoly = make_square_expolygon();

        let mut collection = SurfaceCollection::new();
        collection.push(Surface::top(expoly.clone()));
        collection.push(Surface::top(expoly.clone()));
        collection.push(Surface::bottom(expoly.clone()));

        let tops = collection.filter_by_type(SurfaceType::Top);
        assert_eq!(tops.len(), 2);

        let bottoms = collection.filter_by_type(SurfaceType::Bottom);
        assert_eq!(bottoms.len(), 1);
    }

    #[test]
    fn test_surface_type_name() {
        assert_eq!(SurfaceType::Top.name(), "top");
        assert_eq!(SurfaceType::Bottom.name(), "bottom");
        assert_eq!(SurfaceType::BottomBridge.name(), "bottom bridge");
        assert_eq!(SurfaceType::InternalSolid.name(), "internal solid");
        assert_eq!(SurfaceType::Internal.name(), "internal");
        assert_eq!(SurfaceType::InternalBridge.name(), "internal bridge");
        assert_eq!(SurfaceType::InternalVoid.name(), "internal void");
    }

    #[test]
    fn test_detect_surface_types_first_layer() {
        let expoly = make_square_expolygon();
        let current_slices = vec![expoly];

        let surfaces = detect_surface_types(&current_slices, None, None, 0.01);

        // First layer with no layer below should be all bottom
        assert_eq!(surfaces.len(), 1);
        assert!(surfaces[0].is_bottom());
    }

    #[test]
    fn test_detect_surface_types_top_layer() {
        let expoly = make_square_expolygon();
        let current_slices = vec![expoly.clone()];
        let lower_slices = vec![expoly];

        let surfaces = detect_surface_types(&current_slices, Some(&lower_slices), None, 0.01);

        // Top layer with support below but nothing above should be top
        assert_eq!(surfaces.len(), 1);
        assert!(surfaces[0].is_top());
    }

    #[test]
    fn test_detect_surface_types_internal() {
        let expoly = make_square_expolygon();
        let current_slices = vec![expoly.clone()];
        let lower_slices = vec![expoly.clone()];
        let upper_slices = vec![expoly];

        let surfaces = detect_surface_types(
            &current_slices,
            Some(&lower_slices),
            Some(&upper_slices),
            0.01,
        );

        // Middle layer with same geometry above and below should be internal
        assert_eq!(surfaces.len(), 1);
        assert_eq!(surfaces[0].surface_type, SurfaceType::Internal);
    }
}
