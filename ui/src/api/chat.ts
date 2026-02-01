/**
 * AI Chat API endpoint handler
 * For now, this is a placeholder that echoes back with configuration suggestions
 * In production, this would connect to an actual LLM service
 */

export async function POST(req: Request) {
  const { messages } = await req.json();
  
  const lastMessage = messages[messages.length - 1];
  const userInput = lastMessage.content.toLowerCase();
  
  // Simple rule-based responses for demo
  let response = '';
  
  if (userInput.includes('strong') || userInput.includes('durable') || userInput.includes('strength')) {
    response = `I'll configure your print for maximum strength:
- Increased wall count to 4 perimeters
- Infill density raised to 30%
- Using gyroid infill pattern for better strength
- Layer height set to 0.2mm for good layer adhesion

These settings will make your print significantly stronger and more durable.`;
  } else if (userInput.includes('fast') || userInput.includes('quick') || userInput.includes('draft')) {
    response = `I'll optimize for fast printing:
- Layer height increased to 0.28mm (fewer layers)
- Print speed set to 150mm/s
- Infill reduced to 10%
- Using 2 perimeters (adequate for most prints)

This will reduce print time by approximately 40-50%.`;
  } else if (userInput.includes('quality') || userInput.includes('fine') || userInput.includes('detail')) {
    response = `I'll configure for high quality output:
- Layer height reduced to 0.12mm for fine details
- Print speed reduced to 60mm/s for precision
- 3 perimeters for smooth walls
- Infill set to 20% with grid pattern

This will produce excellent surface quality and detail.`;
  } else if (userInput.includes('support')) {
    response = `I'll enable support structures:
- Support enabled with tree support type
- Support density set to 15%
- Threshold angle at 45°
- This will help print overhangs and bridges

The supports will be automatically generated where needed.`;
  } else {
    response = `I understand you want to configure your print. Could you be more specific about what you're looking for? For example:
- "Make it strong and durable" - for strength-focused settings
- "Fast draft print" - for quick prints
- "High quality print" - for detailed output
- "Enable supports" - to add support structures`;
  }
  
  // Stream the response (for now, just return it immediately)
  const encoder = new TextEncoder();
  const stream = new ReadableStream({
    async start(controller) {
      // Simulate streaming by sending chunks
      const words = response.split(' ');
      for (const word of words) {
        controller.enqueue(encoder.encode(`0:"${word} "\n`));
        await new Promise(resolve => setTimeout(resolve, 30));
      }
      controller.close();
    }
  });
  
  return new Response(stream, {
    headers: {
      'Content-Type': 'text/plain; charset=utf-8',
      'Transfer-Encoding': 'chunked',
    },
  });
}
