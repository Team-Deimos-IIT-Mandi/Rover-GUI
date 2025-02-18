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

// When a client connects to the WebSocket server
wss.on('connection', (ws) => {
    console.log("Client connected");

    // Handle incoming messages from the rover and relay them to all connected clients
    ws.on('message', (message) => {
        console.log("Received message from a client");

        // Broadcast the received message to all other connected clients
        wss.clients.forEach(client => {
            if (client !== ws && client.readyState === WebSocket.OPEN) {
                client.send(message);
            }
        });
    });

    // Handle client disconnections
    ws.on('close', () => {
        console.log("Client disconnected");
    });

    // Handle errors
    ws.on('error', (error) => {
        console.error("WebSocket error:", error);
    });
});
