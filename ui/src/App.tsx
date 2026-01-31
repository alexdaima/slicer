/**
 * Main App component for G-code viewer
 * Restructured with clear layout: sidebar, topbar, and dedicated canvas area
 */

import { type FC, useState, useEffect, useCallback } from 'react';
import { createRoot } from 'react-dom/client';
import { GCodeViewer } from './components/GCodeRenderer';
import { parseGCodeChunks } from './parser/gcodeParser';
import { FEATURE_COLORS, type ParsedGCode } from './types/gcode';

// G-code file path
const GCODE_URL = '/api/gcode/benchy';

const App: FC = () => {
  const [parsedGCode, setParsedGCode] = useState<ParsedGCode | null>(null);
  const [loading, setLoading] = useState(true);
  const [progress, setProgress] = useState(0);
  const [error, setError] = useState<string | null>(null);
  
  // View controls
  const [visibleLayerRange, setVisibleLayerRange] = useState<[number, number]>([0, 1000]);
  const [showTravelMoves, setShowTravelMoves] = useState(false);
  const [renderMode, setRenderMode] = useState<'tubes' | 'lines' | 'lod'>('tubes');

  // Load G-code on mount
  useEffect(() => {
    const loadGCode = async () => {
      try {
        setLoading(true);
        
        const response = await fetch(GCODE_URL);
        if (!response.ok) {
          throw new Error(`Failed to load G-code: ${response.status} ${response.statusText}`);
        }
        
        const gcodeText = await response.text();
        
        const parsed = await parseGCodeChunks(
          gcodeText,
          (p) => setProgress(p),
          10000
        );
        
        setParsedGCode(parsed);
        setVisibleLayerRange([0, parsed.totalLayers - 1]);
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Unknown error');
      } finally {
        setLoading(false);
      }
    };

    loadGCode();
  }, []);

  const handleLayerChange = useCallback((max: number) => {
    setVisibleLayerRange([0, max]);
  }, []);

  // Loading state
  if (loading) {
    return (
      <div className="loading-overlay">
        <h1>Loading G-code...</h1>
        <p>{Math.round(progress * 100)}% complete</p>
        <div className="progress-bar">
          <div 
            className="progress-fill" 
            style={{ width: `${Math.round(progress * 100)}%` }}
          />
        </div>
        <p style={{ fontSize: '0.8rem', opacity: 0.6, marginTop: '1rem' }}>
          Parsing 3DBenchy.gcode
        </p>
      </div>
    );
  }

  // Error state
  if (error) {
    return (
      <div className="loading-overlay">
        <h1 style={{ color: '#FF6B6B' }}>Error</h1>
        <p>{error}</p>
      </div>
    );
  }

  // Not loaded yet
  if (!parsedGCode) {
    return (
      <div className="loading-overlay">
        <h1>Preparing viewer...</h1>
      </div>
    );
  }

  // Main UI layout
  return (
    <div className="app-container">
      {/* Left Sidebar - Controls */}
      <aside className="sidebar">
        <div>
          <h2>Controls</h2>
          
          <div className="control-group">
            <label htmlFor="render-mode">Render Mode</label>
            <select 
              id="render-mode"
              value={renderMode} 
              onChange={(e) => setRenderMode(e.target.value as 'tubes' | 'lines' | 'lod')}
            >
              <option value="tubes">3D Tubes (BambuStudio)</option>
              <option value="lines">Lines (Fast)</option>
              <option value="lod">Auto LOD</option>
            </select>
          </div>

          <div className="control-group">
            <label htmlFor="travel-moves" style={{ display: 'flex', alignItems: 'center', cursor: 'pointer' }}>
              <input 
                id="travel-moves"
                type="checkbox" 
                checked={showTravelMoves} 
                onChange={(e) => setShowTravelMoves(e.target.checked)}
              />
              Show Travel Moves
            </label>
          </div>

          <div className="control-group">
            <label htmlFor="layer-range">
              Max Layer: {visibleLayerRange[1]} / {parsedGCode.totalLayers - 1}
            </label>
            <input
              id="layer-range"
              type="range"
              min={0}
              max={parsedGCode.totalLayers - 1}
              value={visibleLayerRange[1]}
              onChange={(e) => handleLayerChange(Number.parseInt(e.target.value))}
            />
          </div>
        </div>

        {/* Legend */}
        <div className="legend-section">
          <h3>Feature Types</h3>
          <div className="legend-grid">
            {Object.entries(FEATURE_COLORS).map(([feature, color]) => (
              <div key={feature} className="legend-item">
                <div 
                  className="legend-color" 
                  style={{ backgroundColor: color }}
                />
                <span>{feature.replace(/-/g, ' ')}</span>
              </div>
            ))}
          </div>
        </div>
      </aside>

      {/* Top Bar - Stats */}
      <header className="topbar">
        <div className="stats-group">
          <span><strong>Segments:</strong> {parsedGCode.segments.length.toLocaleString()}</span>
          <span><strong>Layers:</strong> {parsedGCode.totalLayers}</span>
        </div>
        <div className="bounds-display">
          <span>X: {parsedGCode.bounds.min.x.toFixed(1)} - {parsedGCode.bounds.max.x.toFixed(1)} mm</span>
          <span>Y: {parsedGCode.bounds.min.y.toFixed(1)} - {parsedGCode.bounds.max.y.toFixed(1)} mm</span>
          <span>Z: {parsedGCode.bounds.min.z.toFixed(1)} - {parsedGCode.bounds.max.z.toFixed(1)} mm</span>
        </div>
      </header>

      {/* Main 3D Viewer Area */}
      <main className="viewer-container">
        <GCodeViewer
          parsedGCode={parsedGCode}
          visibleLayerRange={visibleLayerRange}
          showTravelMoves={showTravelMoves}
          renderMode={renderMode}
        />
      </main>
    </div>
  );
};

// Mount React app
const container = document.getElementById('root');
if (container) {
  const root = createRoot(container);
  root.render(<App />);
}

export default App;
