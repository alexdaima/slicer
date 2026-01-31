/**
 * G-code parser - converts G-code text to renderable segments
 * Optimized for large files with chunked processing
 */

import type { GCodeSegment, MoveType, FeatureType, ParsedGCode, Point3D, LayerInfo } from '../types/gcode';

interface ParserState {
  x: number;
  y: number;
  z: number;
  e: number;
  f: number;
  isAbsolute: boolean;
  currentFeature: FeatureType;
  layerHeight: number;
  extrusionWidth: number;
}

export class GCodeParser {
  private state: ParserState;
  private segmentId = 0;

  constructor() {
    this.state = {
      x: 0,
      y: 0,
      z: 0,
      e: 0,
      f: 3000,
      isAbsolute: true,
      currentFeature: 'custom',
      layerHeight: 0.2,
      extrusionWidth: 0.4,
    };
  }

  parse(gcodeText: string): ParsedGCode {
    const lines = gcodeText.split(/\r?\n/);
    const segments: GCodeSegment[] = [];
    const layers = new Map<number, GCodeSegment[]>();
    
    let minX = Number.POSITIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let minZ = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    let maxZ = Number.NEGATIVE_INFINITY;
    let maxSpeed = 0;
    let maxExtrusion = 0;

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i].trim();
      if (!line) continue;
      
      // Handle comments that contain feature type info (before skipping)
      if (line.startsWith(';')) {
        // Handle feature type comments (Cura, PrusaSlicer, BambuStudio)
        if (line.includes(';TYPE:')) {
          const match = line.match(/;TYPE:(.+)/);
          if (match) {
            this.state.currentFeature = this.mapFeatureType(match[1].trim());
          }
        }
        // Handle feature comments from BambuStudio
        else if (line.includes('; FEATURE:')) {
          const match = line.match(/; FEATURE:(.+)/);
          if (match) {
            this.state.currentFeature = this.mapFeatureType(match[1].trim());
          }
        }
        continue;
      }

      const segment = this.parseLine(line, i + 1);
      if (segment) {
        segments.push(segment);

        // Update bounds
        minX = Math.min(minX, segment.start.x, segment.end.x);
        minY = Math.min(minY, segment.start.y, segment.end.y);
        minZ = Math.min(minZ, segment.start.z, segment.end.z);
        maxX = Math.max(maxX, segment.start.x, segment.end.x);
        maxY = Math.max(maxY, segment.start.y, segment.end.y);
        maxZ = Math.max(maxZ, segment.start.z, segment.end.z);
        maxSpeed = Math.max(maxSpeed, segment.speed);
        maxExtrusion = Math.max(maxExtrusion, segment.e);

        // Group by layer
        const layerZ = Math.round(segment.start.z * 1000) / 1000;
        if (!layers.has(layerZ)) {
          layers.set(layerZ, []);
        }
        const layerSegments = layers.get(layerZ);
        if (layerSegments) {
          layerSegments.push(segment);
        }
      }
    }

    // Sort layers
    const sortedLayers = new Map([...layers.entries()].sort((a, b) => a[0] - b[0]));

    return {
      segments,
      bounds: {
        min: { x: minX, y: minY, z: minZ },
        max: { x: maxX, y: maxY, z: maxZ },
      },
      layers: sortedLayers,
      totalLayers: sortedLayers.size,
      maxSpeed,
      maxExtrusion,
    };
  }

  parseChunk(lines: string[], startIndex: number): GCodeSegment[] {
    const segments: GCodeSegment[] = [];
    
    for (let i = 0; i < lines.length; i++) {
      const line = lines[i].trim();
      if (!line) continue;
      
      // Handle comments that contain feature type info (before skipping)
      if (line.startsWith(';')) {
        // Handle feature type comments (Cura, PrusaSlicer, BambuStudio)
        if (line.includes(';TYPE:')) {
          const match = line.match(/;TYPE:(.+)/);
          if (match) {
            this.state.currentFeature = this.mapFeatureType(match[1].trim());
          }
        }
        // Handle feature comments from BambuStudio
        else if (line.includes('; FEATURE:')) {
          const match = line.match(/; FEATURE:(.+)/);
          if (match) {
            this.state.currentFeature = this.mapFeatureType(match[1].trim());
          }
        }
        continue;
      }

      const segment = this.parseLine(line, startIndex + i + 1);
      if (segment) {
        segments.push(segment);
      }
    }
    
    return segments;
  }

  private parseLine(line: string, lineNumber: number): GCodeSegment | null {
    // Note: Comments (including feature type comments) are handled upstream in parse/parseChunk
    const command = this.extractCommand(line);
    if (!command) return null;

    const params = this.extractParams(line);
    
    // Only process G0/G1 moves
    if (command !== 'G0' && command !== 'G1') {
      return null;
    }

    const start: Point3D = { x: this.state.x, y: this.state.y, z: this.state.z };
    
    // Update position
    if (this.state.isAbsolute) {
      if (params.x !== undefined) this.state.x = params.x;
      if (params.y !== undefined) this.state.y = params.y;
      if (params.z !== undefined) this.state.z = params.z;
    } else {
      if (params.x !== undefined) this.state.x += params.x;
      if (params.y !== undefined) this.state.y += params.y;
      if (params.z !== undefined) this.state.z += params.z;
    }

    const end: Point3D = { x: this.state.x, y: this.state.y, z: this.state.z };

    // Update speed
    if (params.f !== undefined) {
      this.state.f = params.f;
    }

    // Determine move type
    let type: MoveType = 'travel';
    let eDelta = 0;
    
    if (params.e !== undefined) {
      eDelta = params.e - this.state.e;
      if (this.state.isAbsolute) {
        this.state.e = params.e;
      } else {
        this.state.e += params.e;
      }

      if (eDelta > 0.001) {
        type = 'extrude';
      } else if (eDelta < -0.001) {
        type = 'retract';
      } else if (Math.abs(eDelta) > 0.0001) {
        type = 'unretract';
      }
    }

    // Skip zero-length moves
    if (start.x === end.x && start.y === end.y && start.z === end.z) {
      return null;
    }

    this.segmentId++;

    return {
      id: this.segmentId,
      type,
      start,
      end,
      extrusionWidth: type === 'extrude' ? this.state.extrusionWidth : undefined,
      layerHeight: type === 'extrude' ? this.state.layerHeight : undefined,
      speed: this.state.f,
      e: Math.abs(eDelta),
      lineNumber,
      featureType: type === 'extrude' ? this.state.currentFeature : 'travel',
    };
  }

  private extractCommand(line: string): string | null {
    const match = line.match(/^([GM]\d+(?:\.\d+)?)/i);
    return match ? match[1].toUpperCase() : null;
  }

  private extractParams(line: string): Record<string, number> {
    const params: Record<string, number> = {};
    const regex = /([XYZEF])([-+]?[\d.]+(?:e[-+]?\d+)?)/gi;
    let match: RegExpExecArray | null;
    // biome-ignore lint/suspicious/noAssignInExpressions: regex iteration
    while ((match = regex.exec(line)) !== null) {
      const key = match[1].toLowerCase();
      const value = Number.parseFloat(match[2]);
      if (!Number.isNaN(value)) {
        params[key] = value;
      }
    }
    
    return params;
  }

  private mapFeatureType(type: string): FeatureType {
    const mapping: Record<string, FeatureType> = {
      // BambuStudio feature types
      'OUTER WALL': 'outer-wall',
      'INNER WALL': 'inner-wall',
      'OVERHANG WALL': 'inner-wall',
      'TOP SURFACE': 'skin',
      'BOTTOM SURFACE': 'skin',
      'INTERNAL SOLID INFILL': 'skin',
      'SPARSE INFILL': 'infill',
      'GAP INFILL': 'infill',
      'BRIDGE': 'bridge',
      'SUPPORT': 'support',
      'SKIRT': 'skirt',
      'BRIM': 'brim',
      'RAFT': 'raft',
      'CUSTOM': 'custom',
      'FLOATING VERTICAL SHELL': 'support',
      // Legacy/PrusaSlicer/Cura types
      'WALL-OUTER': 'outer-wall',
      'WALL-INNER': 'inner-wall',
      'WALL-INTERNAL': 'inner-wall',
      'SKIN': 'skin',
      'SKIN_TOP': 'skin',
      'SKIN_BOTTOM': 'skin',
      'FILL': 'infill',
      'INFILL': 'infill',
      'SUPPORT-INTERFACE': 'support',
      'SUPPORT-ROOF': 'support',
      'TRAVEL': 'travel',
    };
    
    return mapping[type.toUpperCase()] || 'custom';
  }

  reset(): void {
    this.segmentId = 0;
    this.state = {
      x: 0,
      y: 0,
      z: 0,
      e: 0,
      f: 3000,
      isAbsolute: true,
      currentFeature: 'custom',
      layerHeight: 0.2,
      extrusionWidth: 0.4,
    };
  }
}

// Chunked parser for large files
export async function parseGCodeChunks(
  gcodeText: string, 
  onProgress?: (progress: number) => void,
  chunkSize = 10000
): Promise<ParsedGCode> {
  const lines = gcodeText.split(/\r?\n/);
  const parser = new GCodeParser();
  const allSegments: GCodeSegment[] = [];
  const layers = new Map<number, GCodeSegment[]>();
  
  let minX = Number.POSITIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let minZ = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;
  let maxZ = Number.NEGATIVE_INFINITY;
  let maxSpeed = 0;
  let maxExtrusion = 0;

  for (let i = 0; i < lines.length; i += chunkSize) {
    const chunk = lines.slice(i, i + chunkSize);
    const segments = parser.parseChunk(chunk, i);
    
    allSegments.push(...segments);

    // Update bounds and layer info
    for (const segment of segments) {
      minX = Math.min(minX, segment.start.x, segment.end.x);
      minY = Math.min(minY, segment.start.y, segment.end.y);
      minZ = Math.min(minZ, segment.start.z, segment.end.z);
      maxX = Math.max(maxX, segment.start.x, segment.end.x);
      maxY = Math.max(maxY, segment.start.y, segment.end.y);
      maxZ = Math.max(maxZ, segment.start.z, segment.end.z);
      maxSpeed = Math.max(maxSpeed, segment.speed);
      maxExtrusion = Math.max(maxExtrusion, segment.e);

      const layerZ = Math.round(segment.start.z * 1000) / 1000;
      if (!layers.has(layerZ)) {
        layers.set(layerZ, []);
      }
      const layerSegments = layers.get(layerZ);
      if (layerSegments) {
        layerSegments.push(segment);
      }
    }

    if (onProgress) {
      onProgress(Math.min((i + chunkSize) / lines.length, 1));
    }

    // Yield to event loop
    if (i % (chunkSize * 5) === 0) {
      await new Promise(resolve => setTimeout(resolve, 0));
    }
  }

  const sortedLayers = new Map([...layers.entries()].sort((a, b) => a[0] - b[0]));

  return {
    segments: allSegments,
    bounds: {
      min: { x: minX === Number.POSITIVE_INFINITY ? 0 : minX, y: minY === Number.POSITIVE_INFINITY ? 0 : minY, z: minZ === Number.POSITIVE_INFINITY ? 0 : minZ },
      max: { x: maxX === Number.NEGATIVE_INFINITY ? 100 : maxX, y: maxY === Number.NEGATIVE_INFINITY ? 100 : maxY, z: maxZ === Number.NEGATIVE_INFINITY ? 100 : maxZ },
    },
    layers: sortedLayers,
    totalLayers: sortedLayers.size,
    maxSpeed,
    maxExtrusion,
  };
}
