// const http = require('http');
// const WebSocket = require('ws');

// // Create an HTTP server
// const server = http.createServer((req, res) => {
//   res.writeHead(200, { 'Content-Type': 'text/plain' });
//   res.end('WebSocket server is running\n');
// });

// Attach the WebSocket server to the HTTP server
// const wss = new WebSocket.Server({ server });

// Handle connection events
// wss.on('connection', (ws) => {
//   console.log("Client connected");

//   ws.on('message', (message) => {
//     console.log("Received message:", message);
    
//     // Relay message to all other connected clients except the sender
//     wss.clients.forEach(client => {
//       if (client !== ws && client.readyState === WebSocket.OPEN) {
//         client.send(message);
//       }
//     });
//   });

//   ws.on('close', () => {
//     console.log("Client disconnected");
//   });
// });

// // Start the HTTP server to listen on port 8080
// server.listen(8080, () => {
//   console.log("Signaling server is running on ws://localhost:8080");
// });


// signaling_server.js

const WebSocket = require('ws');
const wss = new WebSocket.Server({ port: 8080 });

console.log("Signaling server is running on ws://localhost:8080");

const clients = new Map();

wss.on('connection', (ws) => {
    console.log("Client connected");

    ws.on('message', (message) => {
        try {
            const parsedMessage = JSON.parse(message);

            if (parsedMessage.type === 'register') {
                clients.set(ws, parsedMessage.role);
                console.log(`Client registered as: ${parsedMessage.role}`);
                return;
            }

            console.log(`Received message: ${message}`);

            if (parsedMessage.type === 'command') {
                wss.clients.forEach(client => {
                    if (clients.get(client) === 'robot' && client.readyState === WebSocket.OPEN) {
                        client.send(message);
                    }
                });
            } else if (parsedMessage.type === 'state_update') {
                wss.clients.forEach(client => {
                    if (clients.get(client) === 'gui' && client.readyState === WebSocket.OPEN) {
                        client.send(message);
                    }
                });
            } else if (parsedMessage.type === 'video_stream') {
                wss.clients.forEach(client => {
                    if (client !== ws && client.readyState === WebSocket.OPEN) {
                        client.send(message);
                    }
                });
            } else {
                console.warn(`Unknown message type: ${parsedMessage.type}`);
            }
        } catch (error) {
            console.error(`Error processing message: ${error.message}`);
        }
    });

    ws.on('close', () => {
        console.log("Client disconnected");
        clients.delete(ws);
    });

    ws.on('error', (error) => {
        console.error("WebSocket error:", error);
    });
});

