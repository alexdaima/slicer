/**
 * React Three Fiber G-code renderer with tube/extrusion visualization
 * Renders G-code segments as 3D tubes like BambuStudio/OrcaSlicer
 */

import React, { useMemo } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls, Box } from '@react-three/drei';
import * as THREE from 'three';
import type { GCodeSegment, ParsedGCode, FeatureType } from '../types/gcode';
import { FEATURE_COLORS, FEATURE_OPACITY } from '../types/gcode';
import { Nozzle } from './Nozzle';

interface GCodeRendererProps {
  parsedGCode: ParsedGCode;
  visibleLayerRange?: [number, number];
  lineWidth?: number;
  showTravelMoves?: boolean;
  segmentLimit?: number;
  nozzlePosition?: [number, number, number];
  showNozzle?: boolean;
}

// Color cache to avoid repeated color parsing
const colorCache = new Map<string, THREE.Color>();

function getCachedColor(colorHex: string): THREE.Color {
  if (!colorCache.has(colorHex)) {
    colorCache.set(colorHex, new THREE.Color(colorHex));
  }
  const cachedColor = colorCache.get(colorHex);
  if (!cachedColor) {
    throw new Error(`Color ${colorHex} not found in cache`);
  }
  return cachedColor;
}

/**
 * Efficient instanced cylinder renderer for G-code segments
 * Uses THREE.InstancedMesh for maximum performance
 */
const GCodeTubes: React.FC<GCodeRendererProps> = ({
  parsedGCode,
  visibleLayerRange,
  showTravelMoves = false,
}) => {
  // Process segments into instanced data by feature
  const instancesByFeature = useMemo(() => {
    const instances = new Map<FeatureType, {
      matrices: THREE.Matrix4[];
      colors: THREE.Color[];
    }>();
    
    const up = new THREE.Vector3(0, 1, 0);
    const position = new THREE.Vector3();
    const scale = new THREE.Vector3();
    const quaternion = new THREEQuaternion();
    const matrix = new THREE.Matrix4();
    
    // Build a map of Z values to layer indices for faster lookup
    const layerZMap = new Map<number, number>();
    const layerZValues = Array.from(parsedGCode.layers.keys()).sort((a, b) => a - b);
    layerZValues.forEach((z, index) => layerZMap.set(z, index));

    for (const segment of parsedGCode.segments) {
      // Filter by layer range
      if (visibleLayerRange) {
        const [minLayer, maxLayer] = visibleLayerRange;
        const zRounded = Math.round(segment.start.z * 1000) / 1000;
        const layerIndex = layerZMap.get(zRounded) ?? -1;
        if (layerIndex < minLayer || layerIndex > maxLayer) continue;
      }

      // Only render extrusion moves (travel moves create ring artifacts at corners)
      if (segment.type !== 'extrude') continue;

      const feature = segment.featureType || 'custom';
      if (!instances.has(feature)) {
        instances.set(feature, { matrices: [], colors: [] });
      }

      const featureData = instances.get(feature);
      if (!featureData) continue;

      // Calculate segment properties
      const start = segment.start;
      const end = segment.end;
      
      // Midpoint
      position.set(
        (start.x + end.x) / 2,
        (start.y + end.y) / 2,
        (start.z + end.z) / 2
      );

      // Calculate length
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      const dz = end.z - start.z;
      const length = Math.sqrt(dx * dx + dy * dy + dz * dz);

      // Filter out very short segments to prevent overlapping geometry artifacts
      // Minimum length of 0.05mm prevents "ring" appearance from overlapping cylinders
      if (length < 0.05) continue;

      // Calculate orientation
      const direction = new THREE.Vector3(dx, dy, dz).normalize();
      quaternion.setFromUnitVectors(up, direction);

      // Scale: thin radius for outer-wall travel moves, reduced width for extrusions
      // Using 0.6 * extrusionWidth/2 to minimize overlapping at corners
      const isTravel = segment.type !== 'extrude';
      const radius = isTravel ? 0.03 : (segment.extrusionWidth || 0.4) * 0.3;
      scale.set(radius, length, radius);

      // Build matrix
      matrix.compose(position, quaternion, scale);
      featureData.matrices.push(matrix.clone());

      // Color
      const baseColor = getCachedColor(FEATURE_COLORS[feature]);
      featureData.colors.push(baseColor.clone());
    }

    return instances;
  }, [parsedGCode, visibleLayerRange]);

  // Build instanced meshes for each feature
  const instancedMeshes = useMemo(() => {
    const meshes: Array<{
      mesh: THREE.InstancedMesh;
      feature: FeatureType;
    }> = [];

    const cylinderGeo = new THREE.CylinderGeometry(1, 1, 1, 8, 1);

    for (const [feature, data] of instancesByFeature) {
      if (data.matrices.length === 0) continue;

      const opacity = FEATURE_OPACITY[feature];
      const material = new THREE.MeshStandardMaterial({
        color: 0xffffff,
        roughness: 0.4,
        metalness: 0.1,
        transparent: opacity < 1,
        opacity: opacity,
      });

      const instancedMesh = new THREE.InstancedMesh(
        cylinderGeo,
        material,
        data.matrices.length
      );

      // Set matrices and colors
      for (let i = 0; i < data.matrices.length; i++) {
        const matrix = data.matrices[i];
        const color = data.colors[i];
        if (matrix && color) {
          instancedMesh.setMatrixAt(i, matrix);
          instancedMesh.setColorAt(i, color);
        }
      }
      
      instancedMesh.instanceMatrix.needsUpdate = true;
      if (instancedMesh.instanceColor) {
        instancedMesh.instanceColor.needsUpdate = true;
      }

      meshes.push({ mesh: instancedMesh, feature });
    }

    return meshes;
  }, [instancesByFeature]);

  return (
    <group>
      {instancedMeshes.map((item, index) => (
        <primitive key={`${item.feature}-${index}`} object={item.mesh} />
      ))}
    </group>
  );
};

// Helper for quaternion calculation
const THREEQuaternion = THREE.Quaternion;

/**
 * Simplified line renderer for travels or low-detail mode
 */
const GCodeLineSegments: React.FC<GCodeRendererProps> = ({
  parsedGCode,
  visibleLayerRange,
  showTravelMoves = false,
  segmentLimit = 500000,
}) => {
  // Group segments by feature type for batch rendering
  const segmentsByFeature = useMemo(() => {
    const groups = new Map<FeatureType, GCodeSegment[]>();
    
    let segmentCount = 0;
    const layerZValues = Array.from(parsedGCode.layers.keys());
    
    for (const segment of parsedGCode.segments) {
      if (segmentCount >= segmentLimit) break;
      
      // Filter by layer range
      if (visibleLayerRange) {
        const [minLayer, maxLayer] = visibleLayerRange;
        const layerIndex = layerZValues.findIndex(z => Math.abs(z - segment.start.z) < 0.001);
        if (layerIndex < minLayer || layerIndex > maxLayer) continue;
      }

      // Skip non-extrusion moves (travel, retract, unretract) unless enabled
      if (segment.type !== 'extrude' && !showTravelMoves) continue;

      const feature = segment.featureType || 'custom';
      if (!groups.has(feature)) {
        groups.set(feature, []);
      }
      const featureSegments = groups.get(feature);
      if (featureSegments) {
        featureSegments.push(segment);
      }
      segmentCount++;
    }

    return groups;
  }, [parsedGCode, visibleLayerRange, showTravelMoves, segmentLimit]);

  // Create geometry for each feature group
  const lineGeometries = useMemo(() => {
    const geometries: Array<{ geometry: THREE.BufferGeometry; color: THREE.Color; count: number }> = [];

    for (const [feature, segments] of segmentsByFeature) {
      if (segments.length === 0) continue;

      const positions = new Float32Array(segments.length * 6);
      
      for (let i = 0; i < segments.length; i++) {
        const seg = segments[i];
        if (!seg) continue;
        const idx = i * 6;
        positions[idx] = seg.start.x;
        positions[idx + 1] = seg.start.y;
        positions[idx + 2] = seg.start.z;
        positions[idx + 3] = seg.end.x;
        positions[idx + 4] = seg.end.y;
        positions[idx + 5] = seg.end.z;
      }

      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      geometry.computeBoundingSphere();

      geometries.push({
        geometry,
        color: getCachedColor(FEATURE_COLORS[feature]),
        count: segments.length,
      });
    }

    return geometries;
  }, [segmentsByFeature]);

  return (
    <group>
      {lineGeometries.map((item, index) => (
        <lineSegments key={`${item.count}-${index}`} geometry={item.geometry}>
          <lineBasicMaterial color={item.color} linewidth={1} />
        </lineSegments>
      ))}
    </group>
  );
};

/**
 * Travel moves renderer (always uses lines)
 */
const TravelMoves: React.FC<GCodeRendererProps> = (props) => {
  const travelProps = {
    ...props,
    showTravelMoves: true,
  };
  
  return <GCodeLineSegments {...travelProps} segmentLimit={50000} />;
};

/**
 * Progressive LOD renderer
 */
const GCodeLOD: React.FC<GCodeRendererProps> = (props) => {
  const { camera } = useThree();
  const [detailLevel, setDetailLevel] = React.useState<'high' | 'medium' | 'low'>('medium');

  useFrame(() => {
    const distance = camera.position.distanceTo(new THREE.Vector3(
      (props.parsedGCode.bounds.min.x + props.parsedGCode.bounds.max.x) / 2,
      (props.parsedGCode.bounds.min.y + props.parsedGCode.bounds.max.y) / 2,
      (props.parsedGCode.bounds.min.z + props.parsedGCode.bounds.max.z) / 2
    ));

    const modelSize = Math.max(
      props.parsedGCode.bounds.max.x - props.parsedGCode.bounds.min.x,
      props.parsedGCode.bounds.max.y - props.parsedGCode.bounds.min.y,
      props.parsedGCode.bounds.max.z - props.parsedGCode.bounds.min.z
    );

    if (distance < modelSize * 0.5) {
      setDetailLevel('high');
    } else if (distance < modelSize * 1.5) {
      setDetailLevel('medium');
    } else {
      setDetailLevel('low');
    }
  });

  const segmentLimit = useMemo(() => {
    switch (detailLevel) {
      case 'high': return 300000;
      case 'medium': return 100000;
      case 'low': return 25000;
      default: return 100000;
    }
  }, [detailLevel]);

  return (
    <>
      <GCodeTubes {...props} />
      {props.showTravelMoves && <TravelMoves {...props} segmentLimit={10000} />}
    </>
  );
};

/**
 * Bounding box helper
 */
const BoundingBox: React.FC<{ parsedGCode: ParsedGCode }> = ({ parsedGCode }) => {
  const size = useMemo(() => [
    parsedGCode.bounds.max.x - parsedGCode.bounds.min.x,
    parsedGCode.bounds.max.y - parsedGCode.bounds.min.y,
    parsedGCode.bounds.max.z - parsedGCode.bounds.min.z,
  ], [parsedGCode]);

  const center = useMemo(() => [
    (parsedGCode.bounds.min.x + parsedGCode.bounds.max.x) / 2,
    (parsedGCode.bounds.min.y + parsedGCode.bounds.max.y) / 2,
    (parsedGCode.bounds.min.z + parsedGCode.bounds.max.z) / 2,
  ], [parsedGCode]);

  return (
    <Box args={size as [number, number, number]} position={center as [number, number, number]}>
      <meshBasicMaterial color="#444444" wireframe transparent opacity={0.3} />
    </Box>
  );
};

/**
 * Main G-code scene component
 */
interface GCodeSceneProps extends GCodeRendererProps {
  showBoundingBox?: boolean;
  renderMode?: 'tubes' | 'lines' | 'lod';
}

export const GCodeScene: React.FC<GCodeSceneProps> = ({
  parsedGCode,
  showBoundingBox = true,
  renderMode = 'tubes',
  visibleLayerRange,
  lineWidth,
  showTravelMoves,
  segmentLimit,
  nozzlePosition,
  showNozzle = false,
}) => {
  const center = useMemo(() => [
    (parsedGCode.bounds.min.x + parsedGCode.bounds.max.x) / 2,
    (parsedGCode.bounds.min.y + parsedGCode.bounds.max.y) / 2,
    (parsedGCode.bounds.min.z + parsedGCode.bounds.max.z) / 2,
  ], [parsedGCode]);

  const renderContent = useMemo(() => {
    const commonProps = {
      parsedGCode,
      visibleLayerRange,
      lineWidth,
      showTravelMoves,
      segmentLimit,
    };

    switch (renderMode) {
      case 'tubes':
        return (
          <>
            <GCodeTubes {...commonProps} />
            {showTravelMoves && <TravelMoves {...commonProps} segmentLimit={50000} />}
          </>
        );
      case 'lines':
        return <GCodeLineSegments {...commonProps} />;
      case 'lod':
        return <GCodeLOD {...commonProps} />;
      default:
        return (
          <>
            <GCodeTubes {...commonProps} />
            {showTravelMoves && <TravelMoves {...commonProps} segmentLimit={50000} />}
          </>
        );
    }
  }, [renderMode, parsedGCode, visibleLayerRange, lineWidth, showTravelMoves, segmentLimit]);

  return (
    <>
      <color attach="background" args={['#0f0f0f']} />
      
      <ambientLight intensity={0.4} />
      <directionalLight 
        position={[50, 50, 50]} 
        intensity={1.2}
        castShadow
        shadow-mapSize={[2048, 2048]}
      />
      <directionalLight 
        position={[-50, -50, 30]} 
        intensity={0.4}
      />
      <pointLight position={[0, 0, 50]} intensity={0.5} />
      
      <group rotation={[-Math.PI / 2, 0, 0]}>
        <group position={[-(center[0] ?? 0), -(center[1] ?? 0), -(center[2] ?? 0)]}>
          {renderContent}
          {showBoundingBox && <BoundingBox parsedGCode={parsedGCode} />}
          {showNozzle && nozzlePosition && (
            <Nozzle 
              position={[
                nozzlePosition[0],
                nozzlePosition[1],
                nozzlePosition[2]
              ]} 
              visible={showNozzle} 
            />
          )}
        </group>
      </group>
      
      <OrbitControls 
        makeDefault 
        enableDamping 
        dampingFactor={0.05}
        target={[0, 0, 0]}
      />
    </>
  );
};

/**
 * Loading indicator
 */
export const GCodeLoading: React.FC = () => {
  return (
    <div style={{
      position: 'absolute',
      top: '50%',
      left: '50%',
      transform: 'translate(-50%, -50%)',
      color: 'white',
      fontFamily: 'system-ui, sans-serif',
      fontSize: '1.5rem',
      textAlign: 'center',
    }}>
      <div>Loading G-code...</div>
      <div style={{ fontSize: '1rem', opacity: 0.7, marginTop: '0.5rem' }}>
        Parsing large files may take a moment
      </div>
    </div>
  );
};

/**
 * Main G-code viewer component with Canvas
 */
interface GCodeViewerProps extends GCodeSceneProps {
  className?: string;
  style?: React.CSSProperties;
}

export const GCodeViewer: React.FC<GCodeViewerProps> = ({
  className,
  style,
  ...props
}) => {
  const modelSize = useMemo(() => Math.max(
    props.parsedGCode.bounds.max.x - props.parsedGCode.bounds.min.x,
    props.parsedGCode.bounds.max.y - props.parsedGCode.bounds.min.y,
    props.parsedGCode.bounds.max.z - props.parsedGCode.bounds.min.z
  ), [props.parsedGCode]);

  return (
    <div 
      className={className} 
      style={{ 
        width: '100%', 
        height: '100%',
        position: 'absolute',
        top: 0,
        left: 0,
        ...style 
      }}
    >
      <Canvas
        camera={{
          position: [
            modelSize * 0.7,
            modelSize * 1.0,
            modelSize * 0.7,
          ],
          fov: 50,
          near: 0.1,
          far: 10000,
        }}
        gl={{ 
          antialias: true, 
          alpha: false,
        }}
        shadows
        dpr={[1, 2]}
        performance={{ min: 0.5 }}
      >
        <GCodeScene {...props} />
      </Canvas>
    </div>
  );
};

export default GCodeViewer;
