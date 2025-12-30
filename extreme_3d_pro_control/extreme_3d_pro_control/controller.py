#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
import asyncio
import websockets
import json
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from queue import Queue

# Complete HTML interface embedded in Python
HTML_CONTENT = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ROS2 Joystick Monitor</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { 
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
    background: linear-gradient(135deg, #0f172a 0%, #1e3a8a 50%, #0f172a 100%); 
    color: white; 
    height: 100vh; 
    overflow: hidden;
}

.container { 
    height: 100vh; 
    display: flex;
    flex-direction: column;
    padding: 8px; 
    gap: 8px;
}

header { 
    display: flex; 
    align-items: center; 
    justify-content: space-between; 
    padding: 6px 14px;
    background: rgba(30,41,59,0.8); 
    backdrop-filter: blur(10px); 
    border-radius: 10px; 
    height: 45px;
    flex-shrink: 0;
}

h1 { 
    font-size: 1.3rem; 
    font-weight: 700; 
}

.status { 
    display: inline-flex; 
    align-items: center; 
    gap: 6px; 
    padding: 5px 12px; 
    background: rgba(0,0,0,0.3); 
    border-radius: 16px; 
    font-size: 0.8rem;
}

.status-dot { 
    width: 9px; 
    height: 9px; 
    border-radius: 50%; 
    background: #ef4444; 
    animation: pulse 2s infinite;
}

.status-dot.connected { 
    background: #22c55e; 
    animation: none;
}

@keyframes pulse { 
    0%, 100% { opacity: 1; } 
    50% { opacity: 0.5; } 
}

.main-content {
    flex: 1;
    display: flex;
    flex-direction: column;
    gap: 8px;
    min-height: 0;
}

.top-section {
    display: grid; 
    grid-template-columns: 1fr 160px 1fr; 
    gap: 8px; 
    height: calc(100vh - 220px);
}

.card { 
    background: rgba(30,41,59,0.8); 
    backdrop-filter: blur(10px); 
    border: 1px solid rgba(71,85,105,0.5);
    border-radius: 10px; 
    padding: 12px; 
    display: flex; 
    flex-direction: column;
}

.card h2 { 
    font-size: 0.95rem; 
    margin-bottom: 10px; 
    display: flex; 
    align-items: center; 
    gap: 6px; 
    font-weight: 600;
}

/* Joystick */
.joystick-container { 
    flex: 1;
    display: flex;
    align-items: center;
    justify-content: center;
}

.joystick-bg { 
    position: relative; 
    width: 100%; 
    max-width: 280px;
    aspect-ratio: 1;
    background: rgba(15,23,42,0.5); 
    border: 2px solid rgba(59,130,246,0.3); 
    border-radius: 8px;
}

.joystick-crosshair { 
    position: absolute; 
    inset: 0; 
}

.crosshair-h { 
    position: absolute; 
    top: 50%; 
    left: 0; 
    right: 0; 
    height: 2px; 
    background: rgba(59,130,246,0.2); 
    transform: translateY(-50%);
}

.crosshair-v { 
    position: absolute; 
    left: 50%; 
    top: 0; 
    bottom: 0; 
    width: 2px; 
    background: rgba(59,130,246,0.2); 
    transform: translateX(-50%);
}

.joystick-indicator {
    position: absolute;
    width: 30px;             /* slightly bigger than before */
    height: 30px;
    background: #3b82f6;     /* blue circle */
    border: 3px solid white;
    border-radius: 50%;      /* full circle */
    box-shadow: 0 0 16px rgba(59,130,246,0.8); /* glowing effect */
    left: 50%;
    top: 50%;
    transform: translate(-50%, -50%) rotate(0deg);
    display: flex;
    justify-content: center;
    align-items: flex-start;
}

/* The rotation cut indicator */
.joystick-indicator::after {
    content: '';
    width: 3px;              /* thin vertical line */
    height: 50%;             /* cut halfway from top */
    background: white;       /* contrasting color */
    border-radius: 1px;
}

/* Throttle - Narrower and fits in middle column */
.throttle-container { 
    flex: 1; 
    display: flex; 
    align-items: center;
    justify-content: center;
}

.throttle-bar { 
    position: relative; 
    background: rgba(15,23,42,0.5); 
    border-radius: 8px; 
    overflow: hidden; 
    width: 70px;
    height: 100%;
    max-height: calc(100vh - 320px);
    border: 2px solid rgba(34,197,94,0.3);
}

.throttle-fill { 
    position: absolute; 
    bottom: 0; 
    left: 0; 
    right: 0; 
    background: linear-gradient(to top, #22c55e, #10b981); 
    transition: height 0.1s ease-out; 
    height: 50%;
}

.throttle-value { 
    position: absolute; 
    inset: 0; 
    display: flex; 
    align-items: center; 
    justify-content: center; 
    font-size: 1.6rem; 
    font-weight: bold; 
    z-index: 10; 
    text-shadow: 2px 2px 4px rgba(0,0,0,0.8);
}

/* Buttons */
.buttons-section { 
    flex: 1; 
    display: flex; 
    flex-direction: column; 
    gap: 8px;
}

.hat-switch { 
    background: rgba(15,23,42,0.5); 
    border-radius: 8px; 
    padding: 8px;
}

.hat-label {
    font-size: 0.7rem;
    color: #94a3b8;
    margin-bottom: 6px;
    text-align: center;
}

.hat-container { 
    position: relative; 
    width: 90px; 
    height: 90px; 
    margin: 0 auto;
}

.hat-bg { 
    position: relative; 
    width: 100%; 
    height: 100%; 
    background: rgba(15,23,42,0.5); 
    border: 2px solid rgba(59,130,246,0.3); 
    border-radius: 50%;
}

.hat-crosshair {
    position: absolute;
    inset: 0;
}

.hat-line-h {
    position: absolute;
    top: 50%;
    left: 0;
    right: 0;
    height: 1px;
    background: rgba(59,130,246,0.2);
    transform: translateY(-50%);
}

.hat-line-v {
    position: absolute;
    left: 50%;
    top: 0;
    bottom: 0;
    width: 1px;
    background: rgba(59,130,246,0.2);
    transform: translateX(-50%);
}

.hat-indicator { 
    position: absolute; 
    width: 16px; 
    height: 16px; 
    background: #3b82f6; 
    border: 3px solid white; 
    border-radius: 50%; 
    box-shadow: 0 2px 8px rgba(59,130,246,0.5);
    transition: all 0.1s ease-out;
    left: 50%; 
    top: 50%; 
    transform: translate(-50%, -50%);
}

.buttons-grid { 
    display: grid; 
    grid-template-columns: repeat(4, 1fr); 
    gap: 5px; 
    flex: 1;
}

.button { 
    aspect-ratio: 1; 
    display: flex; 
    align-items: center; 
    justify-content: center; 
    background: rgba(15,23,42,0.5); 
    border-radius: 6px; 
    font-weight: bold; 
    font-size: 0.85rem; 
    transition: all 0.15s ease;
    color: #64748b;
}

.button.pressed { 
    background: #3b82f6; 
    color: white; 
    box-shadow: 0 0 20px rgba(59,130,246,0.5); 
    transform: scale(1.05);
}

/* Velocity Section - Bottom */
.velocity-section {
    background: rgba(30,41,59,0.8); 
    backdrop-filter: blur(10px); 
    border: 1px solid rgba(71,85,105,0.5);
    border-radius: 10px; 
    padding: 12px;
    height: 150px;
    flex-shrink: 0;
}

.velocity-section h2 {
    font-size: 0.95rem;
    margin-bottom: 10px;
    font-weight: 600;
}

.velocity-grid { 
    display: grid; 
    grid-template-columns: repeat(6, 1fr); 
    gap: 8px; 
    height: calc(100% - 30px);
}

.velocity-item { 
    background: rgba(15,23,42,0.5); 
    border-radius: 8px; 
    padding: 8px 4px; 
    text-align: center; 
    display: flex; 
    flex-direction: column; 
    justify-content: center;
}

.velocity-label { 
    font-size: 0.65rem; 
    color: #94a3b8; 
    margin-bottom: 4px;
}

.velocity-value { 
    font-size: 1.3rem; 
    font-weight: bold; 
}

.velocity-value.linear { 
    color: #22c55e; 
}

.velocity-value.angular { 
    color: #3b82f6; 
}

@media (max-width: 1200px) {
    .top-section {
        grid-template-columns: 1fr 140px 1fr;
    }
    .velocity-grid {
        grid-template-columns: repeat(3, 1fr);
    }
}

@media (max-width: 768px) {
    .top-section {
        grid-template-columns: 1fr;
        height: auto;
    }
    .velocity-grid {
        grid-template-columns: repeat(2, 1fr);
    }
}
</style>
</head>
<body>
<div class="container">
    <header>
        <h1>🎮 ROS2 Joystick Monitor</h1>
        <div class="status">
            <div class="status-dot" id="statusDot"></div>
            <span id="statusText">Disconnected</span>
        </div>
    </header>

    <div class="main-content">
        <div class="top-section">
            <!-- Stick Position -->
            <div class="card">
                <h2>🔄 Stick Position</h2>
                <div class="joystick-container">
                    <div class="joystick-bg">
                        <div class="joystick-crosshair">
                            <div class="crosshair-h"></div>
                            <div class="crosshair-v"></div>
                        </div>
                        <div class="joystick-indicator" id="stickIndicator"></div>
                    </div>
                </div>
            </div>

            <!-- Throttle -->
            <div class="card">
                <h2>⚡ Throttle</h2>
                <div class="throttle-container">
                    <div class="throttle-bar">
                        <div class="throttle-fill" id="throttleFill"></div>
                        <div class="throttle-value" id="throttleValue">50%</div>
                    </div>
                </div>
            </div>

            <!-- Buttons -->
            <div class="card">
                <h2>🎯 Controls</h2>
                <div class="buttons-section">
                    <div class="hat-switch">
                        <div class="hat-label">8-Way Hat</div>
                        <div class="hat-container">
                            <div class="hat-bg">
                                <div class="hat-crosshair">
                                    <div class="hat-line-h"></div>
                                    <div class="hat-line-v"></div>
                                </div>
                                <div class="hat-indicator" id="hatIndicator"></div>
                            </div>
                        </div>
                    </div>
                    <div class="buttons-grid" id="buttonsGrid"></div>
                </div>
            </div>
        </div>

        <!-- Velocity Commands - Bottom Section -->
        <div class="velocity-section">
            <h2>📊 /cmd_vel Commands</h2>
            <div class="velocity-grid">
                <div class="velocity-item">
                    <div class="velocity-label">Linear X</div>
                    <div class="velocity-value linear" id="linearX">0.000</div>
                </div>
                <div class="velocity-item">
                    <div class="velocity-label">Linear Y</div>
                    <div class="velocity-value linear" id="linearY">0.000</div>
                </div>
                <div class="velocity-item">
                    <div class="velocity-label">Linear Z</div>
                    <div class="velocity-value linear" id="linearZ">0.000</div>
                </div>
                <div class="velocity-item">
                    <div class="velocity-label">Angular X</div>
                    <div class="velocity-value angular" id="angularX">0.000</div>
                </div>
                <div class="velocity-item">
                    <div class="velocity-label">Angular Y</div>
                    <div class="velocity-value angular" id="angularY">0.000</div>
                </div>
                <div class="velocity-item">
                    <div class="velocity-label">Angular Z</div>
                    <div class="velocity-value angular" id="angularZ">0.000</div>
                </div>
            </div>
        </div>
    </div>
</div>

<script>
const ws = new WebSocket(`ws://${window.location.hostname}:8765`);
const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');
const stickIndicator = document.getElementById('stickIndicator');
const throttleFill = document.getElementById('throttleFill');
const throttleValue = document.getElementById('throttleValue');
const hatIndicator = document.getElementById('hatIndicator');
const buttonsGrid = document.getElementById('buttonsGrid');
const linearX = document.getElementById('linearX');
const linearY = document.getElementById('linearY');
const linearZ = document.getElementById('linearZ');
const angularX = document.getElementById('angularX');
const angularY = document.getElementById('angularY');
const angularZ = document.getElementById('angularZ');

ws.onopen = () => {
    statusDot.classList.add('connected');
    statusText.textContent = 'Connected';
};

ws.onclose = () => {
    statusDot.classList.remove('connected');
    statusText.textContent = 'Disconnected';
};

ws.onerror = (error) => {
    console.error('WebSocket error:', error);
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    
    if (data.type === 'joy') {
        updateJoy(data);
    } else if (data.type === 'twist') {
        updateTwist(data);
    }
};

function updateJoy(data) {
    const axes = data.axes;
    const buttons = data.buttons;

    // Update stick position - INVERTED Y-AXIS
    if (axes.length >= 3) {
        const x = -axes[0];      // left/right
        const y = -axes[1];      // forward/back (inverted for screen)
        const yaw = axes[2];     // twist (rotation)

        const leftPercent = 50 + (x * 45);
        const topPercent  = 50 + (y * 45);

        // Convert yaw [-1,1] → degrees
        const yawDeg = yaw * 90;

        stickIndicator.style.left = leftPercent + '%';
        stickIndicator.style.top  = topPercent + '%';

        stickIndicator.style.transform =
            `translate(-50%, -50%) rotate(${yawDeg}deg)`;
    }

    // Update throttle bar with TWIST control (axis 3 - Z rotation)
    if (axes.length >= 4) {
        const twist = axes[3];  // Z-rotation axis (twist)
        // Convert from [-1, 1] to [0, 100] for display
        const throttle = ((twist + 1) / 2) * 100;
        throttleFill.style.height = throttle + '%';
        throttleValue.textContent = throttle.toFixed(0) + '%';
    }

    // Update hat switch (POV hat on Extreme 3D Pro)
    if (axes.length >= 6) {
        const hatX = -axes[4];       // left/right
        const hatY = -axes[5];      // invert Y so up goes up

        const move = 35; // distance from center (%)

        hatIndicator.style.left = `calc(50% + ${hatX * move}%)`;
        hatIndicator.style.top  = `calc(50% + ${hatY * move}%)`;
    }

    // Update buttons
    updateButtons(buttons);
    }

function updateTwist(data) {
    linearX.textContent = data.linear.x.toFixed(3);
    linearY.textContent = data.linear.y.toFixed(3);
    linearZ.textContent = data.linear.z.toFixed(3);
    angularX.textContent = data.angular.x.toFixed(3);
    angularY.textContent = data.angular.y.toFixed(3);
    angularZ.textContent = data.angular.z.toFixed(3);
    }

function updateButtons(buttons) {
    // Always create exactly 12 buttons
    if (buttonsGrid.children.length !== 12) {
        buttonsGrid.innerHTML = '';
        for (let i = 0; i < 12; i++) {
            const btn = document.createElement('div');
            btn.className = 'button';
            btn.id = `btn-${i}`;
            btn.textContent = i + 1;
            buttonsGrid.appendChild(btn);
        }
    }

    // Update button states
    for (let i = 0; i < 12; i++) {
        const btn = document.getElementById(`btn-${i}`);
        if (btn) {
            const pressed = buttons[i] === 1;
            if (pressed) {
                btn.classList.add('pressed');
            } else {
                btn.classList.remove('pressed');
            }
        }
    }
}
</script>
</body>
</html>
"""


class JoyWebSocketServer(Node):
    def __init__(self):
        super().__init__('joy_websocket_server')
        
        # Parameters
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('websocket_port', 8765)
        self.declare_parameter('http_port', 8000)
        
        joy_topic = self.get_parameter('joy_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.ws_port = self.get_parameter('websocket_port').value
        self.http_port = self.get_parameter('http_port').value
        
        # WebSocket clients
        self.ws_clients = set()
        
        # Message queue for async communication
        self.message_queue = Queue()
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            10
        )
        
        self.twist_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.twist_callback,
            10
        )

        '''
        self.twist_stamped_sub = self.create_subscription(
            TwistStamped,
            cmd_vel_topic,
            self.twist_stamped_callback,
            10
        )
        '''

        self.get_logger().info(f'🚀 Joy WebSocket Server started')
        self.get_logger().info(f'📡 WebSocket port: {self.ws_port}')
        self.get_logger().info(f'🌐 HTTP port: {self.http_port}')
        self.get_logger().info(f'📥 Subscribing to: {joy_topic}, {cmd_vel_topic}')
    
    def joy_callback(self, msg):
        """Handle incoming Joy messages"""
        data = {
            'type': 'joy',
            'axes': list(msg.axes),
            'buttons': list(msg.buttons),
        }
        self.message_queue.put(data)

    
    def twist_callback(self, msg):
        """Handle incoming Twist messages"""
        data = {
            'type': 'twist',
            'linear': {
                'x': msg.linear.x,
                'y': msg.linear.y,
                'z': msg.linear.z
            },
            'angular': {
                'x': msg.angular.x,
                'y': msg.angular.y,
                'z': msg.angular.z
            },
        }
        self.message_queue.put(data)
    
    '''
    def twist_stamped_callback(self, msg):
        """Handle incoming Twist messages"""
        data = {
        'type': 'twist',
        'linear': {
            'x': float(msg.twist.linear.x),   
            'y': float(msg.twist.linear.y),
            'z': float(msg.twist.linear.z)
        },
        'angular': {
            'x': float(msg.twist.angular.x), 
            'y': float(msg.twist.angular.y),
            'z': float(msg.twist.angular.z)
            },
        }
        self.message_queue.put(data)
    '''
    
    async def ws_handler(self, websocket, path):
        """Handle WebSocket connections"""
        self.ws_clients.add(websocket)
        self.get_logger().info(f'✅ Client connected. Total clients: {len(self.ws_clients)}')
        
        try:
            async for message in websocket:
                # Handle incoming messages if needed
                pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)
            self.get_logger().info(f'❌ Client disconnected. Total clients: {len(self.ws_clients)}')
    
    async def broadcast_loop(self):
        """Continuously broadcast messages from queue to all clients"""
        while rclpy.ok():
            try:
                # Check if there are messages to send
                while not self.message_queue.empty():
                    message = self.message_queue.get_nowait()
                    
                    if self.ws_clients:
                        # Broadcast to all connected clients
                        await asyncio.gather(
                            *[client.send(json.dumps(message)) for client in list(self.ws_clients)],
                            return_exceptions=True
                        )
            except Exception as e:
                pass
            
            await asyncio.sleep(0.01)  # Small delay to prevent busy loop


def start_websocket_server(node, port):
    """Start WebSocket server in separate thread"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    start_server = websockets.serve(node.ws_handler, "0.0.0.0", port)
    
    loop.run_until_complete(start_server)
    node.get_logger().info(f'🌐 WebSocket server running on ws://0.0.0.0:{port}')
    
    # Start broadcast loop
    loop.create_task(node.broadcast_loop())
    
    loop.run_forever()


def start_http_server(port, html_content):
    """Start HTTP server to serve the web interface"""
    class CustomHandler(SimpleHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/' or self.path == '/index.html':
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                self.wfile.write(html_content.encode())
            else:
                self.send_response(404)
                self.end_headers()
        
        def log_message(self, format, *args):
            pass  # Suppress HTTP logs
    
    server = HTTPServer(('0.0.0.0', port), CustomHandler)
    print(f'🌐 HTTP server running on http://0.0.0.0:{port}')
    print(f'📱 Open in browser: http://localhost:{port}')
    server.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    
    node = JoyWebSocketServer()
    
    # Start WebSocket server in separate thread
    ws_thread = threading.Thread(
        target=start_websocket_server,
        args=(node, node.ws_port),
        daemon=True
    )
    ws_thread.start()
    
    # Start HTTP server in separate thread
    http_thread = threading.Thread(
        target=start_http_server,
        args=(node.http_port, HTML_CONTENT),
        daemon=True
    )
    http_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





