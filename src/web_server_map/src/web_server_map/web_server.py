#!/usr/bin/env python3
"""
Web server for map visualization.
Serves static HTML/JS files and provides WebSocket endpoint for real-time updates.
"""

import asyncio
import json
import logging
import os
from pathlib import Path
from typing import Dict, List, Set, Optional

from aiohttp import web, WSMsgType
import threading


class MapWebServer:
    """
    HTTP and WebSocket server for map visualization.
    
    Serves static files from web/ directory and broadcasts state updates
    to all connected WebSocket clients.
    """
    
    def __init__(self, port: int = 8080, update_rate_hz: float = 10.0):
        """
        Initialize web server.
        
        Args:
            port: HTTP server port
            update_rate_hz: Rate at which to broadcast updates to clients
        """
        self.port = port
        self.update_rate_hz = update_rate_hz
        self.update_interval = 1.0 / update_rate_hz
        
        self.app = web.Application()
        self.runner = None
        self.site = None
        
        # WebSocket clients
        self.ws_clients: Set[web.WebSocketResponse] = set()
        
        # State (thread-safe access via lock)
        self.state_lock = threading.Lock()
        self.boat_state: Optional[Dict] = None
        self.detections: List[Dict] = []
        
        # Find web directory
        self.web_dir = self._find_web_directory()
        
        # Setup routes
        # Note: register explicit handlers for CSS/JS before the generic
        # static route to avoid any subtle issues with static mapping.
        self.app.router.add_get('/', self.handle_index)
        self.app.router.add_get('/ws', self.handle_websocket)
        self.app.router.add_get('/web/style.css', self.handle_style_css)
        self.app.router.add_get('/web/map.js', self.handle_map_js)
        self.app.router.add_static('/web', self.web_dir, show_index=False)
        
        self.logger = logging.getLogger('MapWebServer')
        self.logger.setLevel(logging.INFO)
        
        # Broadcast task
        self.broadcast_task = None
        
    def _find_web_directory(self) -> Path:
        """Find the web directory containing static files."""
        # Try installed location first (share/web_server_map/web/)
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('web_server_map')
            web_dir = Path(pkg_share) / 'web'
            if web_dir.exists():
                return web_dir
        except Exception:
            pass
        
        # Try relative to this file (development)
        current_file = Path(__file__).resolve()
        web_dir = current_file.parent.parent.parent / 'web'
        if web_dir.exists():
            return web_dir
        
        # Fallback: create empty directory
        web_dir = Path.cwd() / 'web'
        web_dir.mkdir(exist_ok=True)
        return web_dir
    
    async def handle_index(self, request):
        """Serve index.html."""
        index_path = self.web_dir / 'index.html'
        if not index_path.exists():
            return web.Response(text='index.html not found', status=404)
        return web.FileResponse(index_path)
    
    async def handle_style_css(self, request):
        """Serve style.css explicitly (defensive against static routing issues)."""
        css_path = self.web_dir / 'style.css'
        if not css_path.exists():
            return web.Response(text='style.css not found', status=404)
        return web.FileResponse(css_path)
    
    async def handle_map_js(self, request):
        """Serve map.js explicitly (defensive against static routing issues)."""
        js_path = self.web_dir / 'map.js'
        if not js_path.exists():
            return web.Response(text='map.js not found', status=404)
        return web.FileResponse(js_path)
    
    async def handle_websocket(self, request):
        """Handle WebSocket connections."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self.ws_clients.add(ws)
        self.logger.info(f'WebSocket client connected. Total clients: {len(self.ws_clients)}')
        
        try:
            # Send initial state
            await self._send_state_to_client(ws)
            
            # Handle incoming messages
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        if data.get('type') == 'request_update':
                            await self._send_state_to_client(ws)
                    except json.JSONDecodeError:
                        self.logger.warning('Invalid JSON from client')
                elif msg.type == WSMsgType.ERROR:
                    self.logger.error(f'WebSocket error: {ws.exception()}')
        finally:
            self.ws_clients.discard(ws)
            self.logger.info(f'WebSocket client disconnected. Total clients: {len(self.ws_clients)}')
        
        return ws
    
    async def _send_state_to_client(self, ws: web.WebSocketResponse):
        """Send current state to a single WebSocket client."""
        with self.state_lock:
            state = {
                'type': 'update',
                'boat': self.boat_state,
                'detections': self.detections
            }
        
        try:
            await ws.send_json(state)
        except Exception as e:
            self.logger.error(f'Error sending state to client: {e}')
    
    async def _broadcast_loop(self):
        """Periodically broadcast state to all connected clients."""
        while True:
            try:
                await asyncio.sleep(self.update_interval)
                
                if not self.ws_clients:
                    continue
                
                with self.state_lock:
                    state = {
                        'type': 'update',
                        'boat': self.boat_state,
                        'detections': self.detections
                    }
                
                # Broadcast to all clients
                disconnected = set()
                for ws in self.ws_clients:
                    try:
                        await ws.send_json(state)
                    except Exception as e:
                        self.logger.error(f'Error broadcasting to client: {e}')
                        disconnected.add(ws)
                
                # Remove disconnected clients
                self.ws_clients -= disconnected
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error(f'Error in broadcast loop: {e}')
    
    def update_boat_state(self, east: float, north: float, heading_rad: float, timestamp: float):
        """
        Update boat state (thread-safe).
        
        Args:
            east: East coordinate (meters)
            north: North coordinate (meters)
            heading_rad: Heading in radians (0 = East, CCW positive)
            timestamp: ROS timestamp (seconds)
        """
        with self.state_lock:
            self.boat_state = {
                'east': east,
                'north': north,
                'heading_rad': heading_rad,
                'timestamp': timestamp
            }
    
    def update_detections(self, detections: List[Dict]):
        """
        Update detection list (thread-safe).
        
        Args:
            detections: List of detection dicts with keys:
                east, north, class_name, source, id, range_m
        """
        with self.state_lock:
            self.detections = detections
    
    async def start(self):
        """Start the web server."""
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, '0.0.0.0', self.port)
        await self.site.start()
        
        # Start broadcast task
        self.broadcast_task = asyncio.create_task(self._broadcast_loop())
        
        self.logger.info(f'Web server started on http://0.0.0.0:{self.port}')
        self.logger.info(f'Web directory: {self.web_dir}')
    
    async def stop(self):
        """Stop the web server."""
        if self.broadcast_task:
            self.broadcast_task.cancel()
            try:
                await self.broadcast_task
            except asyncio.CancelledError:
                pass
        
        if self.site:
            await self.site.stop()
        if self.runner:
            await self.runner.cleanup()
        
        # Close all WebSocket connections
        for ws in self.ws_clients:
            await ws.close()
        self.ws_clients.clear()
        
        self.logger.info('Web server stopped')
