/**
 * Nozzle indicator component - renders an upside-down triangle to show current nozzle position
 */
import { useRef } from 'react';
import type React from 'react';
import * as THREE from 'three';
import { useFrame } from '@react-three/fiber';

interface NozzleProps {
  position: [number, number, number];
  visible: boolean;
}

export const Nozzle: React.FC<NozzleProps> = ({ position, visible }) => {
  if (!visible) return null;

  return (
    <group position={position}>
      {/* Main cone (nozzle) pointing down */}
      <mesh rotation={[-Math.PI / 2, 0, 0]}>
        <coneGeometry args={[2.5, 6, 3]} />
        <meshStandardMaterial 
          color="#ff6b35" 
          emissive="#ff6b35"
          emissiveIntensity={0.5}
          transparent
          opacity={0.8}
        />
      </mesh>
    </group>
  );
};
