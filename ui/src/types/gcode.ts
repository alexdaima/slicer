/**
 * G-code types for 3D printer toolpath visualization
 */

export type MoveType = 'extrude' | 'travel' | 'retract' | 'unretract' | 'dwell';

export interface Point3D {
  x: number;
  y: number;
  z: number;
}

export interface GCodeSegment {
  id: number;
  type: MoveType;
  start: Point3D;
  end: Point3D;
  extrusionWidth?: number;
  layerHeight?: number;
  speed: number;
  e: number;
  lineNumber: number;
  featureType?: FeatureType;
}

export type FeatureType = 
  | 'outer-wall'
  | 'inner-wall'
  | 'skin'
  | 'infill'
  | 'support'
  | 'skirt'
  | 'brim'
  | 'raft'
  | 'bridge'
  | 'travel'
  | 'custom';

export interface ParsedGCode {
  segments: GCodeSegment[];
  bounds: {
    min: Point3D;
    max: Point3D;
  };
  layers: Map<number, GCodeSegment[]>;
  totalLayers: number;
  maxSpeed: number;
  maxExtrusion: number;
}

export interface LayerInfo {
  z: number;
  segments: GCodeSegment[];
  bounds: {
    min: { x: number; y: number };
    max: { x: number; y: number };
  };
}

export const FEATURE_COLORS: Record<FeatureType, string> = {
  'outer-wall': '#FF6B6B',
  'inner-wall': '#4ECDC4',
  'skin': '#45B7D1',
  'infill': '#96CEB4',
  'support': '#FFEAA7',
  'skirt': '#DDA0DD',
  'brim': '#DDA0DD',
  'raft': '#98D8C8',
  'bridge': '#F7DC6F',
  'travel': '#808080',
  'custom': '#CCCCCC',
};

export const FEATURE_OPACITY: Record<FeatureType, number> = {
  'outer-wall': 1.0,
  'inner-wall': 0.9,
  'skin': 0.85,
  'infill': 0.7,
  'support': 0.6,
  'skirt': 0.8,
  'brim': 0.8,
  'raft': 0.75,
  'bridge': 0.9,
  'travel': 0.3,
  'custom': 0.8,
};
