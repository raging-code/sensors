import asyncio
import websockets
import sqlite3
import json
from datetime import datetime
from aiohttp import web
import time

# Database setup
def init_database():
    conn = sqlite3.connect('do_data.db')
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS do_readings
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  timestamp DATETIME,
                  do_value REAL,
                  do_adc INTEGER)''')
    conn.commit()
    conn.close()

init_database()

# Store connected WebSocket clients
connected_clients = set()
esp32_ws = None  # Store ESP32 connection
current_do = 0.0
current_adc = 0

# WebSocket handler for ESP32
async def esp32_handler(websocket):
    global esp32_ws, current_do, current_adc
    esp32_ws = websocket
    print("ESP32 connected")
    
    try:
        async for message in websocket:
            # Handle data from ESP32
            if message.startswith("DO:"):
                # Parse DO data
                parts = message.split(",")
                do_part = parts[0].split(":")
                adc_part = parts[1].split(":")
                
                current_do = float(do_part[1])
                current_adc = int(adc_part[1])
                
                # Save to database
                conn = sqlite3.connect('do_data.db')
                c = conn.cursor()
                # Use ISO format string for datetime
                timestamp_str = datetime.now().isoformat()
                c.execute("INSERT INTO do_readings (timestamp, do_value, do_adc) VALUES (?, ?, ?)",
                         (timestamp_str, current_do, current_adc))
                conn.commit()
                conn.close()
                
                # Broadcast to web clients
                data = {
                    'do': current_do,
                    'adc': current_adc,
                    'timestamp': datetime.now().strftime('%H:%M:%S')
                }
                await broadcast(json.dumps(data))
                
            elif message == "CALIBRATION_COMPLETE":
                print("Calibration complete from ESP32")
                await broadcast(json.dumps({'calibration': 'complete'}))
                
    except websockets.exceptions.ConnectionClosed:
        print("ESP32 disconnected")
    finally:
        esp32_ws = None
        print("ESP32 connection closed")

# WebSocket handler for web clients
async def web_client_handler(websocket):
    connected_clients.add(websocket)
    print(f"Web client connected. Total clients: {len(connected_clients)}")
    
    try:
        # Send current data to new client
        data = {
            'do': current_do,
            'adc': current_adc,
            'timestamp': datetime.now().strftime('%H:%M:%S')
        }
        await websocket.send(json.dumps(data))
        
        # Keep connection alive
        async for message in websocket:
            if message == "START_CALIBRATION" and esp32_ws:
                # Send calibration command to ESP32
                await esp32_ws.send("START_CALIBRATION")
                print("Calibration command sent to ESP32")
            elif message == "STOP_CALIBRATION" and esp32_ws:
                await esp32_ws.send("STOP_CALIBRATION")
                print("Stop calibration command sent to ESP32")
                
    except websockets.exceptions.ConnectionClosed:
        print("Web client disconnected")
    finally:
        connected_clients.remove(websocket)
        print(f"Web client disconnected. Total clients: {len(connected_clients)}")

async def broadcast(message):
    if connected_clients:
        disconnected = set()
        for client in connected_clients:
            try:
                await client.send(message)
            except:
                disconnected.add(client)
        
        # Remove disconnected clients
        for client in disconnected:
            connected_clients.remove(client)

# HTTP handler for serving HTML
async def handle_index(request):
    with open('index.html', 'r') as f:
        return web.Response(text=f.read(), content_type='text/html')

async def handle_calibration(request):
    try:
        data = await request.json()
        action = data.get('action')
        
        if action == 'start' and esp32_ws:
            await esp32_ws.send("START_CALIBRATION")
            return web.json_response({'status': 'calibration_started'})
        elif action == 'stop' and esp32_ws:
            await esp32_ws.send("STOP_CALIBRATION")
            return web.json_response({'status': 'calibration_stopped'})
        
        return web.json_response({'status': 'esp32_not_connected'})
    except Exception as e:
        return web.json_response({'status': 'error', 'message': str(e)})

# Add endpoint to get historical data
async def handle_history(request):
    conn = sqlite3.connect('do_data.db')
    c = conn.cursor()
    
    # Get last 100 readings
    c.execute("SELECT timestamp, do_value, do_adc FROM do_readings ORDER BY timestamp DESC LIMIT 100")
    rows = c.fetchall()
    conn.close()
    
    # Format data for chart
    history = []
    for row in rows:
        # Convert ISO timestamp to time string
        try:
            timestamp = datetime.fromisoformat(row[0]).strftime('%H:%M:%S')
        except:
            timestamp = str(row[0])[:8]  # Just show first 8 chars if not ISO format
        
        history.append({
            'timestamp': timestamp,
            'do': row[1],
            'adc': row[2]
        })
    
    # Reverse to show oldest first
    history.reverse()
    return web.json_response(history)

# Start servers
async def main():
    # Start ESP32 WebSocket server on port 8765
    esp32_server = await websockets.serve(esp32_handler, "0.0.0.0", 8765)
    print("ESP32 WebSocket server started on port 8765")
    
    # Start Web client WebSocket server on port 8766
    web_server = await websockets.serve(web_client_handler, "0.0.0.0", 8766)
    print("Web client WebSocket server started on port 8766")
    
    # Start HTTP server on port 8080
    app = web.Application()
    app.router.add_get('/', handle_index)
    app.router.add_post('/calibrate', handle_calibration)
    app.router.add_get('/history', handle_history)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    print("HTTP server started on http://0.0.0.0:8080")
    print("\nAccess the web interface at: http://localhost:8080")
    print("Or from other devices: http://YOUR-PC-IP:8080")
    print("\nMake sure ESP32 is connected to the same network and")
    print("has the correct server IP address configured.")
    
    # Keep running
    await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
