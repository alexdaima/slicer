/**
 * Bun server for G-code viewer
 * Serves the React app and provides the G-code file API
 */

import index from './index.html';
import { POST as chatHandler } from './src/api/chat';

const GCODE_PATH = '../data/reference_gcodes/3DBenchy.gcode';

Bun.serve({
  routes: {
    '/': index,
    
    // API endpoint to serve the G-code file
    '/api/gcode/benchy': async () => {
      try {
        const file = Bun.file(GCODE_PATH);
        if (!(await file.exists())) {
          return new Response('G-code file not found', { status: 404 });
        }
        
        const content = await file.text();
        return new Response(content, {
          headers: {
            'Content-Type': 'text/plain',
            'Cache-Control': 'public, max-age=3600',
          },
        });
      } catch (error) {
        console.error('Error reading G-code file:', error);
        return new Response('Error reading G-code file', { status: 500 });
      }
    },
    
    // AI Chat API endpoint
    '/api/chat': {
      POST: chatHandler,
    },
  },
  
  development: {
    hmr: true,
    console: true,
  },
  
  port: 3000,
});

console.log('G-code Viewer server running at http://localhost:3000');
console.log('Loading G-code from:', GCODE_PATH);