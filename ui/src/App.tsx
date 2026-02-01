/**
 * Slicer UI with shadcn components
 */

import { type FC, useState, useEffect, useCallback } from 'react';
import { createRoot } from 'react-dom/client';
import { GCodeViewer } from './components/GCodeRenderer';
import { parseGCodeChunks } from './parser/gcodeParser';
import { FEATURE_COLORS, type ParsedGCode } from './types/gcode';
import { Button } from '@/components/ui/button';
import { AIChat } from '@/components/AIChat';
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from '@/components/ui/dropdown-menu';
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs';
import { ChevronDown } from 'lucide-react';

const GCODE_URL = '/api/gcode/benchy';

type ColorSchema = 'filament' | 'line-type' | 'speed' | 'layer-height' | 'line-width' | 'flow' | 'layer-time';

const COLOR_SCHEMAS: Record<ColorSchema, { name: string; description: string }> = {
  'filament': { name: 'Filament', description: 'Filament color' },
  'line-type': { name: 'Line Type', description: 'Outer wall, inner wall, infill' },
  'speed': { name: 'Speed', description: 'Print speed' },
  'layer-height': { name: 'Layer Height', description: 'Layer thickness' },
  'line-width': { name: 'Line Width', description: 'Extrusion width' },
  'flow': { name: 'Flow', description: 'Material flow' },
  'layer-time': { name: 'Layer Time', description: 'Time per layer' },
};

interface SliceOptions {
  printer: string;
  material: string;
  quality: {
    layer_height: number;
    first_layer_height: number;
    resolution: number;
  };
  speed: {
    print_speed: number;
    travel_speed: number;
    first_layer_speed: number;
    perimeter_speed: number;
    external_perimeter_speed: number;
    infill_speed: number;
  };
  perimeters: {
    count: number;
    top_solid_layers: number;
    bottom_solid_layers: number;
    mode: 'classic' | 'arachne';
    thin_walls: boolean;
  };
  infill: {
    density: number;
    pattern: string;
    angle: number;
  };
  support: {
    enabled: boolean;
    type: 'normal' | 'tree';
    threshold_angle: number;
    density: number;
  };
  adhesion: {
    type: 'skirt' | 'brim' | 'raft' | 'none';
    skirt_loops: number;
    brim_width: number;
  };
}

const App: FC = () => {
  const [parsedGCode, setParsedGCode] = useState<ParsedGCode | null>(null);
  const [loading, setLoading] = useState(true);
  const [progress, setProgress] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const [needsRerender, setNeedsRerender] = useState(false);
  const [aiInput, setAiInput] = useState('');
  
  const [colorSchema, setColorSchema] = useState<ColorSchema>('line-type');
  const [visibleLayerRange, setVisibleLayerRange] = useState<[number, number]>([0, 1000]);
  
  // Nozzle position state
  const [currentLayerIndex, setCurrentLayerIndex] = useState(0);
  const [currentSegmentIndex, setCurrentSegmentIndex] = useState(0);
  const [showNozzle, setShowNozzle] = useState(true);
  const [showGCodeCommand, setShowGCodeCommand] = useState(false);
  const [nozzlePosition, setNozzlePosition] = useState<[number, number, number]>([0, 0, 0]);
  
  const [sliceOptions, setSliceOptions] = useState<SliceOptions>({
    printer: 'bambu-lab-h2d',
    material: 'bambu-pla-basic',
    quality: {
      layer_height: 0.2,
      first_layer_height: 0.2,
      resolution: 0.0125,
    },
    speed: {
      print_speed: 100,
      travel_speed: 250,
      first_layer_speed: 30,
      perimeter_speed: 100,
      external_perimeter_speed: 50,
      infill_speed: 150,
    },
    perimeters: {
      count: 3,
      top_solid_layers: 4,
      bottom_solid_layers: 4,
      mode: 'classic',
      thin_walls: true,
    },
    infill: {
      density: 15,
      pattern: 'grid',
      angle: 45,
    },
    support: {
      enabled: false,
      type: 'normal',
      threshold_angle: 45,
      density: 0.15,
    },
    adhesion: {
      type: 'skirt',
      skirt_loops: 3,
      brim_width: 0,
    },
  });

  const handleSettingChange = useCallback((newOptions: SliceOptions) => {
    setSliceOptions(newOptions);
    setNeedsRerender(true);
  }, []);

  const handleAiSubmit = useCallback(() => {
    const input = aiInput.toLowerCase();
    let newOptions = { ...sliceOptions };
    
    if (input.includes('strong') || input.includes('strength')) {
      newOptions.perimeters.count = 4;
      newOptions.infill.density = 30;
    }
    if (input.includes('fast') || input.includes('quick')) {
      newOptions.speed.print_speed = 150;
      newOptions.quality.layer_height = 0.28;
    }
    if (input.includes('fine') || input.includes('quality')) {
      newOptions.quality.layer_height = 0.12;
      newOptions.perimeters.count = 3;
    }
    if (input.includes('support')) {
      newOptions.support.enabled = true;
    }
    
    setSliceOptions(newOptions);
    setNeedsRerender(true);
    setAiInput('');
  }, [aiInput, sliceOptions]);

  useEffect(() => {
    const loadGCode = async () => {
      try {
        setLoading(true);
        const response = await fetch(GCODE_URL);
        if (!response.ok) {
          throw new Error(`Failed to load G-code: ${response.status} ${response.statusText}`);
        }
        const gcodeText = await response.text();
        const parsed = await parseGCodeChunks(gcodeText, (p) => setProgress(p), 10000);
        setParsedGCode(parsed);
        setVisibleLayerRange([0, parsed.totalLayers - 1]);
        
        // Set sliders to max values by default
        const lastLayerIndex = parsed.totalLayers - 1;
        setCurrentLayerIndex(lastLayerIndex);
        
        // Get the last layer's segments and set to last segment
        const layerZValues = Array.from(parsed.layers.keys()).sort((a, b) => a - b);
        const lastLayerZ = layerZValues[lastLayerIndex];
        if (lastLayerZ !== undefined) {
          const lastLayerSegments = parsed.layers.get(lastLayerZ);
          if (lastLayerSegments && lastLayerSegments.length > 0) {
            setCurrentSegmentIndex(lastLayerSegments.length - 1);
          }
        }
        
        setNeedsRerender(false);
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Unknown error');
      } finally {
        setLoading(false);
      }
    };
    loadGCode();
  }, []);

  const handlePreview = useCallback(() => {
    setNeedsRerender(false);
  }, []);

  const handleLayerChange = useCallback((max: number) => {
    setVisibleLayerRange([0, max]);
    setCurrentLayerIndex(max);
  }, []);

  // Update nozzle position based on current layer and segment
  useEffect(() => {
    if (!parsedGCode) return;

    const layerZValues = Array.from(parsedGCode.layers.keys()).sort((a, b) => a - b);
    const currentLayerZ = layerZValues[currentLayerIndex];
    if (currentLayerZ === undefined) return;

    const layerSegments = parsedGCode.layers.get(currentLayerZ);
    if (!layerSegments || layerSegments.length === 0) return;

    const segmentIdx = Math.min(currentSegmentIndex, layerSegments.length - 1);
    const segment = layerSegments[segmentIdx];
    if (!segment) return;

    // Position nozzle at the end point of the current segment
    setNozzlePosition([segment.end.x, segment.end.y, segment.end.z]);
  }, [parsedGCode, currentLayerIndex, currentSegmentIndex]);

  const getCurrentSegmentInfo = useCallback(() => {
    if (!parsedGCode) return null;

    const layerZValues = Array.from(parsedGCode.layers.keys()).sort((a, b) => a - b);
    const currentLayerZ = layerZValues[currentLayerIndex];
    if (currentLayerZ === undefined) return null;

    const layerSegments = parsedGCode.layers.get(currentLayerZ);
    if (!layerSegments || layerSegments.length === 0) return null;

    const segmentIdx = Math.min(currentSegmentIndex, layerSegments.length - 1);
    const segment = layerSegments[segmentIdx];
    if (!segment) return null;

    return {
      segment,
      layerIndex: currentLayerIndex,
      layerZ: currentLayerZ,
      segmentIndex: segmentIdx,
      totalSegmentsInLayer: layerSegments.length,
    };
  }, [parsedGCode, currentLayerIndex, currentSegmentIndex]);

  // Get segments with context (2 before, current, 2 after) spanning across layers
  const getSegmentContext = useCallback(() => {
    if (!parsedGCode) return null;

    const layerZValues = Array.from(parsedGCode.layers.keys()).sort((a, b) => a - b);
    const currentLayerZ = layerZValues[currentLayerIndex];
    if (currentLayerZ === undefined) return null;

    const layerSegments = parsedGCode.layers.get(currentLayerZ);
    if (!layerSegments || layerSegments.length === 0) return null;

    const segmentIdx = Math.min(currentSegmentIndex, layerSegments.length - 1);
    const currentSegment = layerSegments[segmentIdx];
    if (!currentSegment) return null;
    
    // Find the current segment's index in the global segments array
    const globalIndex = parsedGCode.segments.findIndex(s => s.id === currentSegment.id);
    if (globalIndex === -1) return null;
    
    // Get segments in range [globalIndex - 2, globalIndex + 2] from the global array
    const contextSegments = [];
    for (let i = globalIndex - 2; i <= globalIndex + 2; i++) {
      if (i >= 0 && i < parsedGCode.segments.length) {
        contextSegments.push({
          segment: parsedGCode.segments[i],
          isCurrent: i === globalIndex,
        });
      }
    }

    return contextSegments;
  }, [parsedGCode, currentLayerIndex, currentSegmentIndex]);

  const handleLayerScroll = useCallback((layerIdx: number) => {
    setCurrentLayerIndex(layerIdx);
    setCurrentSegmentIndex(0); // Reset to start of layer
    setVisibleLayerRange([0, layerIdx]);
  }, []);

  const handleSegmentScroll = useCallback((segmentIdx: number) => {
    setCurrentSegmentIndex(segmentIdx);
  }, []);

  if (loading) {
    return (
      <div className="loading-screen">
        <div className="loading-content">
          <div className="loading-spinner" />
          <h1>Loading Slicer...</h1>
          <p>{Math.round(progress * 100)}%</p>
          <div className="progress-bar">
            <div className="progress-fill" style={{ width: `${Math.round(progress * 100)}%` }} />
          </div>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="loading-screen">
        <div className="loading-content">
          <h1 style={{ color: '#FF4444' }}>Error</h1>
          <p>{error}</p>
        </div>
      </div>
    );
  }

  return (
    <div className="slicer-app">
      {/* Top Toolbar with shadcn */}
      <header className="top-toolbar">
        <div className="toolbar-left">
          <div className="logo">
            <span className="logo-icon">◈</span>
            <span className="logo-text">SLICER</span>
          </div>
          
          <nav className="toolbar-links">
            <a href="https://docs.example.com" className="toolbar-link">Documentation</a>
            <a href="https://help.example.com" className="toolbar-link">Help</a>
            <a href="https://github.com" className="toolbar-link">GitHub</a>
          </nav>
        </div>

        <div className="toolbar-center">
          <span className="model-name">3DBenchy.stl</span>
        </div>

        <div className="toolbar-right">
          <DropdownMenu>
            <DropdownMenuTrigger asChild>
              <Button variant="ghost" size="sm" className="gap-0.5">
                File <ChevronDown className="h-2.5 w-2.5" />
              </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent align="end">
              <DropdownMenuItem>Import STL</DropdownMenuItem>
              <DropdownMenuItem>Open Project</DropdownMenuItem>
              <DropdownMenuItem>Save Project</DropdownMenuItem>
            </DropdownMenuContent>
          </DropdownMenu>
          
          <DropdownMenu>
            <DropdownMenuTrigger asChild>
              <Button variant="ghost" size="sm" className="gap-0.5">
                Export <ChevronDown className="h-2.5 w-2.5" />
              </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent align="end">
              <DropdownMenuItem>Export G-code</DropdownMenuItem>
              <DropdownMenuItem>Export 3MF</DropdownMenuItem>
              <DropdownMenuItem>Export Config</DropdownMenuItem>
            </DropdownMenuContent>
          </DropdownMenu>
          
          <Button 
            size="sm" 
            className={needsRerender ? 'animate-pulse' : ''}
            onClick={handlePreview}
          >
            Preview
            {needsRerender && <span className="ml-1 text-red-500">●</span>}
          </Button>
        </div>
      </header>

      {/* Main Content */}
      <div className="main-content">
        {/* Settings Panel with shadcn Tabs */}
        <aside className="settings-panel">
          <Tabs defaultValue="settings" className="w-full h-full flex flex-col">
            <TabsList>
              <TabsTrigger value="settings">Settings</TabsTrigger>
              <TabsTrigger value="ai">AI</TabsTrigger>
            </TabsList>
            
            <TabsContent value="settings">
              <div className="panel-content">
                <div className="panel-section">
                  <h3>Printer & Material</h3>
                  <div className="control-group">
                    <label>Printer</label>
                    <select 
                      value={sliceOptions.printer}
                      onChange={(e) => handleSettingChange({...sliceOptions, printer: (e.target as HTMLSelectElement).value})}
                    >
                      <option value="bambu-lab-h2d">Bambu Lab H2D</option>
                      <option value="bambu-lab-x1">Bambu Lab X1</option>
                      <option value="bambu-lab-p1p">Bambu Lab P1P</option>
                    </select>
                  </div>
                  <div className="control-group">
                    <label>Material</label>
                    <select 
                      value={sliceOptions.material}
                      onChange={(e) => handleSettingChange({...sliceOptions, material: (e.target as HTMLSelectElement).value})}
                    >
                      <option value="bambu-pla-basic">Bambu PLA Basic</option>
                      <option value="bambu-pla-matte">Bambu PLA Matte</option>
                      <option value="bambu-abs">Bambu ABS</option>
                      <option value="bambu-petg">Bambu PETG</option>
                    </select>
                  </div>
                </div>

                <div className="panel-section">
                  <h3>Quality</h3>
                  <div className="control-group">
                    <label>Layer Height: {sliceOptions.quality.layer_height}mm</label>
                    <input type="range" min="0.08" max="0.32" step="0.04" 
                      value={sliceOptions.quality.layer_height}
                      onChange={(e) => handleSettingChange({...sliceOptions, quality: {...sliceOptions.quality, layer_height: Number.parseFloat((e.target as HTMLInputElement).value)}})} />
                  </div>
                </div>

                <div className="panel-section">
                  <h3>Strength</h3>
                  <div className="control-group">
                    <label>Walls: {sliceOptions.perimeters.count}</label>
                    <input type="range" min="1" max="10" 
                      value={sliceOptions.perimeters.count}
                      onChange={(e) => handleSettingChange({...sliceOptions, perimeters: {...sliceOptions.perimeters, count: Number.parseInt((e.target as HTMLInputElement).value)}})} />
                  </div>
                  <div className="control-group">
                    <label>Infill: {sliceOptions.infill.density}%</label>
                    <input type="range" min="0" max="100" step="5" 
                      value={sliceOptions.infill.density}
                      onChange={(e) => handleSettingChange({...sliceOptions, infill: {...sliceOptions.infill, density: Number.parseInt((e.target as HTMLInputElement).value)}})} />
                  </div>
                </div>

                <div className="panel-section">
                  <h3>Support</h3>
                  <div className="control-group checkbox">
                    <label>
                      <input type="checkbox" checked={sliceOptions.support.enabled}
                        onChange={(e) => handleSettingChange({...sliceOptions, support: {...sliceOptions.support, enabled: (e.target as HTMLInputElement).checked}})} />
                      Enable Support
                    </label>
                  </div>
                </div>
              </div>
            </TabsContent>

            <TabsContent value="ai">
              <AIChat />
            </TabsContent>
          </Tabs>
        </aside>

        {/* Viewer */}
        <main className="viewer-wrapper">
          <div className="viewer-container">
            {parsedGCode && (
              <GCodeViewer 
                parsedGCode={parsedGCode} 
                visibleLayerRange={visibleLayerRange} 
                renderMode="tubes"
                nozzlePosition={nozzlePosition}
                showNozzle={showNozzle}
              />
            )}
            
            {/* Overlay Controls */}
            <div className="viewer-overlay">
              {/* Left Panel - View Controls */}
              <div className="viewer-left-panel">
                <div className="overlay-panel">
                  <label>View Type</label>
                  <select value={colorSchema} onChange={(e) => setColorSchema((e.target as HTMLSelectElement).value as ColorSchema)}>
                    {Object.entries(COLOR_SCHEMAS).map(([key, { name }]) => (
                      <option key={key} value={key}>{name}</option>
                    ))}
                  </select>
                </div>

                <div className="overlay-panel mt-3">
                  <label>Object Info</label>
                  <div className="info-grid">
                    <div className="info-item">
                      <span className="info-label">Layers</span>
                      <span className="info-value">{parsedGCode?.totalLayers || 0}</span>
                    </div>
                    <div className="info-item">
                      <span className="info-label">Segments</span>
                      <span className="info-value">{parsedGCode?.segments.length.toLocaleString() || 0}</span>
                    </div>
                    <div className="info-item">
                      <span className="info-label">Size (mm)</span>
                      <span className="info-value">
                        {parsedGCode ? `${(parsedGCode.bounds.max.x - parsedGCode.bounds.min.x).toFixed(1)} × ${(parsedGCode.bounds.max.y - parsedGCode.bounds.min.y).toFixed(1)} × ${(parsedGCode.bounds.max.z - parsedGCode.bounds.min.z).toFixed(1)}` : '-'}
                      </span>
                    </div>
                  </div>
                </div>

                <div className="overlay-panel mt-3">
                  <label className="checkbox-label">
                    <input 
                      type="checkbox" 
                      checked={showNozzle} 
                      onChange={(e) => setShowNozzle((e.target as HTMLInputElement).checked)}
                    />
                    Show Nozzle
                  </label>
                </div>
              </div>

              {/* Bottom Panel - Nozzle Position Controls */}
              {parsedGCode && (
                <div className="viewer-bottom-panel">
                  <div className="nozzle-controls">
                    <div className="nozzle-control-group">
                      <label>Layer: {currentLayerIndex + 1} / {parsedGCode.totalLayers}</label>
                      <input 
                        type="range" 
                        className="nozzle-slider"
                        min={0} 
                        max={parsedGCode.totalLayers - 1}
                        value={currentLayerIndex} 
                        onChange={(e) => handleLayerScroll(Number.parseInt((e.target as HTMLInputElement).value))}
                        style={{
                          background: `linear-gradient(to right, var(--accent-primary) 0%, var(--accent-primary) ${(currentLayerIndex / Math.max(1, parsedGCode.totalLayers - 1)) * 100}%, var(--bg-tertiary) ${(currentLayerIndex / Math.max(1, parsedGCode.totalLayers - 1)) * 100}%, var(--bg-tertiary) 100%)`
                        }}
                      />
                    </div>

                    <div className="nozzle-control-group">
                      <label>
                        Position: {currentSegmentIndex + 1} / {(() => {
                          const layerZValues = Array.from(parsedGCode.layers.keys()).sort((a, b) => a - b);
                          const currentLayerZ = layerZValues[currentLayerIndex];
                          const layerSegments = currentLayerZ !== undefined ? parsedGCode.layers.get(currentLayerZ) : null;
                          return layerSegments?.length || 0;
                        })()}
                      </label>
                      <input 
                        type="range" 
                        className="nozzle-slider"
                        min={0} 
                        max={(() => {
                          const layerZValues = Array.from(parsedGCode.layers.keys()).sort((a, b) => a - b);
                          const currentLayerZ = layerZValues[currentLayerIndex];
                          const layerSegments = currentLayerZ !== undefined ? parsedGCode.layers.get(currentLayerZ) : null;
                          return Math.max(0, (layerSegments?.length || 1) - 1);
                        })()}
                        value={currentSegmentIndex}
                        onChange={(e) => handleSegmentScroll(Number.parseInt((e.target as HTMLInputElement).value))}
                        style={{
                          background: (() => {
                            const layerZValues = Array.from(parsedGCode.layers.keys()).sort((a, b) => a - b);
                            const currentLayerZ = layerZValues[currentLayerIndex];
                            const layerSegments = currentLayerZ !== undefined ? parsedGCode.layers.get(currentLayerZ) : null;
                            const maxSegment = Math.max(0, (layerSegments?.length || 1) - 1);
                            const percentage = maxSegment > 0 ? (currentSegmentIndex / maxSegment) * 100 : 0;
                            return `linear-gradient(to right, var(--accent-primary) 0%, var(--accent-primary) ${percentage}%, var(--bg-tertiary) ${percentage}%, var(--bg-tertiary) 100%)`;
                          })()
                        }}
                      />
                    </div>

                    <Button 
                      variant="secondary" 
                      size="sm"
                      onClick={() => setShowGCodeCommand(!showGCodeCommand)}
                    >
                      {showGCodeCommand ? 'Hide' : 'Show'} G-Code
                    </Button>

                    {showGCodeCommand && (() => {
                      const contextSegments = getSegmentContext();
                      if (!contextSegments) return null;
                      return (
                        <div className="gcode-display">
                          <div className="gcode-lines">
                            {contextSegments.map(({ segment, isCurrent }) => {
                              const gcodeLine = `G1 X${segment.end.x.toFixed(3)} Y${segment.end.y.toFixed(3)} Z${segment.end.z.toFixed(3)}${segment.e > 0 ? ` E${segment.e.toFixed(4)}` : ''} F${(segment.speed * 60).toFixed(0)}`;
                              return (
                                <div 
                                  key={segment.id} 
                                  className={`gcode-line ${isCurrent ? 'gcode-line-current' : ''}`}
                                >
                                  <span className="gcode-line-number">{segment.lineNumber}</span>
                                  <span className="gcode-line-text">{gcodeLine}</span>
                                </div>
                              );
                            })}
                          </div>
                        </div>
                      );
                    })()}
                  </div>
                </div>
              )}
            </div>
          </div>
        </main>
      </div>
    </div>
  );
};

const container = document.getElementById('root');
if (container) {
  const root = createRoot(container);
  root.render(<App />);
}

export default App;
