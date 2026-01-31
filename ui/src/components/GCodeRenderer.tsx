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

interface GCodeRendererProps {
  parsedGCode: ParsedGCode;
  visibleLayerRange?: [number, number];
  lineWidth?: number;
  showTravelMoves?: boolean;
  segmentLimit?: number;
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
 * Tube-based renderer - renders extrusions as 3D cylinders
 * Similar to BambuStudio/OrcaSlicer visualization
 */
const GCodeTubes: React.FC<GCodeRendererProps & { 
  showSeams?: boolean;
  tubeResolution?: number;
}> = ({
  parsedGCode,
  visibleLayerRange,
  showTravelMoves = false,
  segmentLimit = 100000,
  showSeams = true,
  tubeResolution = 6, // Number of segments around the cylinder (6 = hexagon, 8 = octagon)
}) => {
  // Group segments by feature type and prepare instanced data
  const instancesByFeature = useMemo(() => {
    const instances = new Map<FeatureType, {
      matrices: THREE.Matrix4[];
      colors: THREE.Color[];
    }>();
    
    let segmentCount = 0;
    const layerZValues = Array.from(parsedGCode.layers.keys());
    const up = new THREE.Vector3(0, 1, 0);
    const position = new THREE.Vector3();
    const scale = new THREE.Vector3();
    const quaternion = new THREEQuaternion();
    const matrix = new THREE.Matrix4();

    for (const segment of parsedGCode.segments) {
      if (segmentCount >= segmentLimit) break;

      // Filter by layer range
      if (visibleLayerRange) {
        const [minLayer, maxLayer] = visibleLayerRange;
        const layerIndex = layerZValues.findIndex(z => Math.abs(z - segment.start.z) < 0.001);
        if (layerIndex < minLayer || layerIndex > maxLayer) continue;
      }

      // Only render actual extrusion moves as tubes
      // Skip travels, retracts, and unretracts
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

      if (length < 0.001) continue;

      // Calculate orientation
      const direction = new THREE.Vector3(dx, dy, dz).normalize();
      quaternion.setFromUnitVectors(up, direction);

      // Scale: radius based on extrusion width, length based on segment length
      const radius = (segment.extrusionWidth || 0.4) / 2;
      scale.set(radius, length, radius);

      // Build matrix
      matrix.compose(position, quaternion, scale);
      featureData.matrices.push(matrix.clone());

      // Color with optional seam highlight
      const baseColor = getCachedColor(FEATURE_COLORS[feature]);
      const color = baseColor.clone();
      
      // Highlight retraction/unretraction points as seams
      if (showSeams && (segment.type === 'retract' || segment.type === 'unretract')) {
        color.multiplyScalar(0.5); // Darken for seams
      }
      
      featureData.colors.push(color);
      segmentCount++;
    }

    return instances;
  }, [parsedGCode, visibleLayerRange, segmentLimit, showSeams]);

  // Build instanced meshes for each feature
  const instancedMeshes = useMemo(() => {
    const meshes: Array<{
      geometry: THREE.CylinderGeometry;
      material: THREE.MeshStandardMaterial;
      count: number;
      matrices: THREE.Matrix4[];
      colors: THREE.Color[];
      feature: FeatureType;
    }> = [];

    for (const [feature, data] of instancesByFeature) {
      if (data.matrices.length === 0) continue;

      // Create geometry - cylinder oriented along Y axis (default)
      // CylinderGeometry(radiusTop, radiusBottom, height, radialSegments)
      const geometry = new THREE.CylinderGeometry(1, 1, 1, tubeResolution, 1);

      // Create material with appropriate opacity
      const opacity = FEATURE_OPACITY[feature];
      const material = new THREE.MeshStandardMaterial({
        color: 0xffffff, // Will use instance colors
        roughness: 0.4,
        metalness: 0.1,
        transparent: opacity < 1,
        opacity: opacity,
      });

      meshes.push({
        geometry,
        material,
        count: data.matrices.length,
        matrices: data.matrices,
        colors: data.colors,
        feature,
      });
    }

    return meshes;
  }, [instancesByFeature, tubeResolution]);

  return (
    <group>
      {instancedMeshes.map((meshData) => (
        <InstancedTubeMesh key={meshData.feature} {...meshData} />
      ))}
    </group>
  );
};

/**
 * Individual instanced mesh component
 * Updates matrices and colors in a useLayoutEffect
 */
interface InstancedTubeMeshProps {
  geometry: THREE.CylinderGeometry;
  material: THREE.MeshStandardMaterial;
  count: number;
  matrices: THREE.Matrix4[];
  colors: THREE.Color[];
  feature: FeatureType;
}

const InstancedTubeMesh: React.FC<InstancedTubeMeshProps> = ({
  geometry,
  material,
  count,
  matrices,
  colors,
}) => {
  const meshRef = React.useRef<THREE.InstancedMesh>(null);

  React.useLayoutEffect(() => {
    if (!meshRef.current) return;

    const mesh = meshRef.current;
    
    for (let i = 0; i < count; i++) {
      mesh.setMatrixAt(i, matrices[i]);
      mesh.setColorAt(i, colors[i]);
    }
    
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) {
      mesh.instanceColor.needsUpdate = true;
    }
  }, [count, matrices, colors]);

  return (
    <instancedMesh
      ref={meshRef}
      args={[geometry, material, count]}
      castShadow
      receiveShadow
    />
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

      const positions = new Float32Array(segments.length * 6); // 2 points * 3 coordinates
      
      for (let i = 0; i < segments.length; i++) {
        const seg = segments[i];
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
  // Filter to only travel moves
  const travelProps = {
    ...props,
    showTravelMoves: true,
  };
  
  return <GCodeLineSegments {...travelProps} segmentLimit={50000} />;
};

/**
 * Progressive LOD renderer - switches between tubes and lines based on camera distance
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

  // Adjust segment limit based on detail level
  const segmentLimit = useMemo(() => {
    switch (detailLevel) {
      case 'high': return 100000;
      case 'medium': return 50000;
      case 'low': return 25000;
      default: return 50000;
    }
  }, [detailLevel]);

  const tubeResolution = useMemo(() => {
    switch (detailLevel) {
      case 'high': return 8;
      case 'medium': return 6;
      case 'low': return 4;
      default: return 6;
    }
  }, [detailLevel]);

  return (
    <>
      <GCodeTubes {...props} segmentLimit={segmentLimit} tubeResolution={tubeResolution} />
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
  tubeResolution?: number;
}

export const GCodeScene: React.FC<GCodeSceneProps> = ({
  parsedGCode,
  showBoundingBox = true,
  renderMode = 'tubes',
  tubeResolution = 6,
  visibleLayerRange,
  lineWidth,
  showTravelMoves,
  segmentLimit,
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
            <GCodeTubes {...commonProps} tubeResolution={tubeResolution} />
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
            <GCodeTubes {...commonProps} tubeResolution={tubeResolution} />
            {showTravelMoves && <TravelMoves {...commonProps} segmentLimit={50000} />}
          </>
        );
    }
  }, [renderMode, parsedGCode, visibleLayerRange, lineWidth, showTravelMoves, segmentLimit, tubeResolution]);

  return (
    <>
      <color attach="background" args={['#1a1a1a']} />
      
      {/* Lighting setup for 3D tubes */}
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
      
      {/* Rotate -90° on X to make Z point up (Y in Three.js space) */}
      <group rotation={[-Math.PI / 2, 0, 0]}>
        <group position={[-center[0], -center[1], -center[2]]}>
          {renderContent}
          {showBoundingBox && <BoundingBox parsedGCode={parsedGCode} />}
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
            modelSize * 1.5,
            modelSize * 1.5,
            modelSize * 2,
          ],
          fov: 50,
          near: 0.1,
          far: 10000,
        }}
        gl={{ 
          antialias: true, 
          alpha: false,
          shadowMap: { enabled: true, type: THREE.PCFSoftShadowMap }
        }}
        dpr={[1, 2]}
        performance={{ min: 0.5 }}
      >
        <GCodeScene {...props} />
      </Canvas>
    </div>
  );
};

export default GCodeViewer;
