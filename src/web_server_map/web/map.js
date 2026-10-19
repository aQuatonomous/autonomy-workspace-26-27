// Map Visualization JavaScript
// Handles WebSocket connection, coordinate transforms, and canvas rendering

class MapVisualizer {
    constructor() {
        this.canvas = document.getElementById('map-canvas');
        this.ctx = this.canvas.getContext('2d');
        
        // State
        this.boat = null;
        this.detections = [];
        
        // WebSocket
        this.ws = null;
        this.reconnectInterval = 2000;
        
        // Viewport
        this.viewBounds = { minEast: -10, maxEast: 10, minNorth: -10, maxNorth: 10 };
        this.padding = 0.15; // 15% padding around data
        // Global bounds: grow over time but never shrink, to avoid jittery rescaling
        this.globalBounds = { minEast: null, maxEast: null, minNorth: null, maxNorth: null };
        
        // Colors - matches class_mapping.yaml and tracked_buoy_visualizer.py
        this.colors = {
            // Buoys
            'red_buoy': '#FF0000',
            'green_buoy': '#00FF00',
            'yellow_buoy': '#FFFF00',
            'black_buoy': '#000000',
            'red_pole_buoy': '#CC0000',
            'green_pole_buoy': '#00CC00',
            // Indicator buoys (Task 2/3)
            'red_indicator': '#FF3333',
            'green_indicator': '#33FF33',
            'red_indicator_buoy': '#FF3333',
            'green_indicator_buoy': '#33FF33',
            // Infrastructure
            'dock': '#666699',
            // Supply drops (Task 4)
            'yellow_supply_drop': '#FFD700',
            'black_supply_drop': '#333333',
            // Docking numbers (Task 5)
            'digit_1': '#CCCCCC',
            'digit_2': '#CCCCCC',
            'digit_3': '#CCCCCC',
            // Fallback
            'unknown': '#808080',
            'boat': '#0066FF',
            'grid': '#CCCCCC',
            'background': '#FFFFFF'
        };
        
        // Setup
        this.setupCanvas();
        this.setupWebSocket();
        this.setupMouseTracking();
        
        // Start render loop
        this.render();
    }
    
    setupCanvas() {
        // Resize canvas to fit container
        const container = document.getElementById('canvas-container');
        this.canvas.width = container.clientWidth;
        this.canvas.height = container.clientHeight;
        
        // Handle window resize
        window.addEventListener('resize', () => {
            this.canvas.width = container.clientWidth;
            this.canvas.height = container.clientHeight;
            this.render();
        });
    }
    
    setupWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws`;
        
        this.ws = new WebSocket(wsUrl);
        
        this.ws.onopen = () => {
            console.log('WebSocket connected');
            this.updateConnectionStatus(true);
        };
        
        this.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                if (data.type === 'update') {
                    this.boat = data.boat;
                    this.detections = data.detections || [];
                    this.updateUI();
                    this.render();
                }
            } catch (e) {
                console.error('Error parsing WebSocket message:', e);
            }
        };
        
        this.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
        };
        
        this.ws.onclose = () => {
            console.log('WebSocket disconnected');
            this.updateConnectionStatus(false);
            
            // Attempt to reconnect
            setTimeout(() => this.setupWebSocket(), this.reconnectInterval);
        };
    }
    
    setupMouseTracking() {
        this.canvas.addEventListener('mousemove', (event) => {
            const rect = this.canvas.getBoundingClientRect();
            const canvasX = event.clientX - rect.left;
            const canvasY = event.clientY - rect.top;
            
            // Convert canvas coordinates to map coordinates
            const mapCoords = this.canvasToMap(canvasX, canvasY);
            
            document.getElementById('mouse-east').textContent = mapCoords.east.toFixed(2);
            document.getElementById('mouse-north').textContent = mapCoords.north.toFixed(2);
        });
    }
    
    updateConnectionStatus(connected) {
        const statusEl = document.getElementById('connection-status');
        if (connected) {
            statusEl.textContent = 'Connected';
            statusEl.className = 'status-connected';
        } else {
            statusEl.textContent = 'Disconnected';
            statusEl.className = 'status-disconnected';
        }
    }
    
    updateUI() {
        // Update boat info
        if (this.boat) {
            const headingDeg = (this.boat.heading_rad * 180 / Math.PI).toFixed(1);
            document.getElementById('boat-info').textContent = 
                `Boat: E=${this.boat.east.toFixed(2)}m, N=${this.boat.north.toFixed(2)}m`;
            document.getElementById('boat-east').textContent = this.boat.east.toFixed(2);
            document.getElementById('boat-north').textContent = this.boat.north.toFixed(2);
            document.getElementById('boat-heading').textContent = headingDeg;
        }
        
        // Update detection count (excluding cross and triangle)
        const visibleCount = this.detections.filter(d => d.class_name !== 'cross' && d.class_name !== 'triangle').length;
        document.getElementById('detection-count').textContent = 
            `Detections: ${visibleCount}`;
    }
    
    calculateViewBounds() {
        // Calculate instantaneous bounding box from current boat and detections
        let minEast = 0, maxEast = 0, minNorth = 0, maxNorth = 0;
        let hasData = false;
        
        if (this.boat) {
            minEast = maxEast = this.boat.east;
            minNorth = maxNorth = this.boat.north;
            hasData = true;
        }
        
        const visibleDetections = this.detections.filter(d => d.class_name !== 'cross' && d.class_name !== 'triangle');
        for (const det of visibleDetections) {
            if (!hasData) {
                minEast = maxEast = det.east;
                minNorth = maxNorth = det.north;
                hasData = true;
            } else {
                minEast = Math.min(minEast, det.east);
                maxEast = Math.max(maxEast, det.east);
                minNorth = Math.min(minNorth, det.north);
                maxNorth = Math.max(maxNorth, det.north);
            }
        }
        
        if (!hasData) {
            // If we have no new data but already have global bounds, keep them;
            // otherwise use a small default view.
            if (this.globalBounds.minEast == null) {
                this.viewBounds = { minEast: -10, maxEast: 10, minNorth: -10, maxNorth: 10 };
            }
            return;
        }
        
        // Update global bounds – they only ever expand, never shrink
        if (this.globalBounds.minEast == null) {
            this.globalBounds = { minEast, maxEast, minNorth, maxNorth };
        } else {
            this.globalBounds.minEast = Math.min(this.globalBounds.minEast, minEast);
            this.globalBounds.maxEast = Math.max(this.globalBounds.maxEast, maxEast);
            this.globalBounds.minNorth = Math.min(this.globalBounds.minNorth, minNorth);
            this.globalBounds.maxNorth = Math.max(this.globalBounds.maxNorth, maxNorth);
        }
        
        // Add padding based on global bounds
        const rangeEast = this.globalBounds.maxEast - this.globalBounds.minEast;
        const rangeNorth = this.globalBounds.maxNorth - this.globalBounds.minNorth;
        const paddingEast = Math.max(rangeEast * this.padding, 5); // At least 5m padding
        const paddingNorth = Math.max(rangeNorth * this.padding, 5);
        
        this.viewBounds = {
            minEast: this.globalBounds.minEast - paddingEast,
            maxEast: this.globalBounds.maxEast + paddingEast,
            minNorth: this.globalBounds.minNorth - paddingNorth,
            maxNorth: this.globalBounds.maxNorth + paddingNorth
        };
        
        // Ensure aspect ratio matches canvas
        const dataAspect = (this.globalBounds.maxEast - this.globalBounds.minEast) /
                           (this.globalBounds.maxNorth - this.globalBounds.minNorth);
        const canvasAspect = this.canvas.width / this.canvas.height;
        
        if (canvasAspect > dataAspect) {
            // Canvas is wider, expand east range
            const center = (this.viewBounds.minEast + this.viewBounds.maxEast) / 2;
            const halfRange = (this.viewBounds.maxNorth - this.viewBounds.minNorth) * canvasAspect / 2;
            this.viewBounds.minEast = center - halfRange;
            this.viewBounds.maxEast = center + halfRange;
        } else {
            // Canvas is taller, expand north range
            const center = (this.viewBounds.minNorth + this.viewBounds.maxNorth) / 2;
            const halfRange = (this.viewBounds.maxEast - this.viewBounds.minEast) / canvasAspect / 2;
            this.viewBounds.minNorth = center - halfRange;
            this.viewBounds.maxNorth = center + halfRange;
        }
    }
    
    mapToCanvas(east, north) {
        // Convert map coordinates (east, north) to canvas coordinates (x, y)
        const x = (east - this.viewBounds.minEast) / (this.viewBounds.maxEast - this.viewBounds.minEast) * this.canvas.width;
        const y = this.canvas.height - (north - this.viewBounds.minNorth) / (this.viewBounds.maxNorth - this.viewBounds.minNorth) * this.canvas.height;
        return { x, y };
    }
    
    canvasToMap(x, y) {
        // Convert canvas coordinates to map coordinates
        const east = this.viewBounds.minEast + (x / this.canvas.width) * (this.viewBounds.maxEast - this.viewBounds.minEast);
        const north = this.viewBounds.maxNorth - (y / this.canvas.height) * (this.viewBounds.maxNorth - this.viewBounds.minNorth);
        return { east, north };
    }
    
    drawGrid() {
        this.ctx.strokeStyle = this.colors.grid;
        this.ctx.lineWidth = 1;
        this.ctx.font = '12px Arial';
        this.ctx.fillStyle = '#666666';
        
        // Calculate grid spacing (aim for ~10 lines)
        const rangeEast = this.viewBounds.maxEast - this.viewBounds.minEast;
        const rangeNorth = this.viewBounds.maxNorth - this.viewBounds.minNorth;
        const gridSpacing = Math.pow(10, Math.floor(Math.log10(Math.max(rangeEast, rangeNorth) / 10)));
        
        // Vertical lines (constant east)
        const startEast = Math.floor(this.viewBounds.minEast / gridSpacing) * gridSpacing;
        for (let east = startEast; east <= this.viewBounds.maxEast; east += gridSpacing) {
            const pos = this.mapToCanvas(east, 0);
            this.ctx.beginPath();
            this.ctx.moveTo(pos.x, 0);
            this.ctx.lineTo(pos.x, this.canvas.height);
            this.ctx.stroke();
            
            // Label
            this.ctx.fillText(`E=${east.toFixed(0)}`, pos.x + 2, 15);
        }
        
        // Horizontal lines (constant north)
        const startNorth = Math.floor(this.viewBounds.minNorth / gridSpacing) * gridSpacing;
        for (let north = startNorth; north <= this.viewBounds.maxNorth; north += gridSpacing) {
            const pos = this.mapToCanvas(0, north);
            this.ctx.beginPath();
            this.ctx.moveTo(0, pos.y);
            this.ctx.lineTo(this.canvas.width, pos.y);
            this.ctx.stroke();
            
            // Label
            this.ctx.fillText(`N=${north.toFixed(0)}`, 2, pos.y - 2);
        }
        
        // Draw origin
        const origin = this.mapToCanvas(0, 0);
        this.ctx.strokeStyle = '#FF0000';
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.moveTo(origin.x - 10, origin.y);
        this.ctx.lineTo(origin.x + 10, origin.y);
        this.ctx.moveTo(origin.x, origin.y - 10);
        this.ctx.lineTo(origin.x, origin.y + 10);
        this.ctx.stroke();
    }
    
    drawBoat(boat) {
        const pos = this.mapToCanvas(boat.east, boat.north);
        
        // Draw boat as arrow showing heading
        const arrowLength = 20;
        const arrowWidth = 10;
        
        // Heading: 0 = East, positive = CCW
        // Canvas: 0 = right, positive = CCW (same!)
        const angle = boat.heading_rad;
        
        this.ctx.save();
        this.ctx.translate(pos.x, pos.y);
        this.ctx.rotate(angle);
        
        // Draw arrow
        this.ctx.fillStyle = this.colors.boat;
        this.ctx.beginPath();
        this.ctx.moveTo(arrowLength, 0);
        this.ctx.lineTo(-arrowLength / 2, arrowWidth);
        this.ctx.lineTo(-arrowLength / 2, -arrowWidth);
        this.ctx.closePath();
        this.ctx.fill();
        
        // Draw outline
        this.ctx.strokeStyle = '#FFFFFF';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();
        
        this.ctx.restore();
        
        // Draw label
        this.ctx.fillStyle = '#000000';
        this.ctx.font = 'bold 14px Arial';
        this.ctx.fillText('BOAT', pos.x + 15, pos.y - 15);
    }
    
    drawDetections(detections) {
        const filtered = detections.filter(d => d.class_name !== 'cross' && d.class_name !== 'triangle');
        for (const det of filtered) {
            const pos = this.mapToCanvas(det.east, det.north);
            
            // Get color based on class name
            const color = this.colors[det.class_name] || this.colors.unknown;
            
            // Draw circle
            this.ctx.fillStyle = color;
            this.ctx.beginPath();
            this.ctx.arc(pos.x, pos.y, 8, 0, 2 * Math.PI);
            this.ctx.fill();
            
            // Draw outline
            this.ctx.strokeStyle = '#FFFFFF';
            this.ctx.lineWidth = 2;
            this.ctx.stroke();
            
            // Draw label (class name and ID)
            this.ctx.fillStyle = '#000000';
            this.ctx.font = '10px Arial';
            const label = `${det.class_name} (${det.id})`;
            this.ctx.fillText(label, pos.x + 10, pos.y - 10);
        }
    }
    
    render() {
        // Clear canvas
        this.ctx.fillStyle = this.colors.background;
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        
        // Calculate view bounds
        this.calculateViewBounds();
        
        // Draw grid
        this.drawGrid();
        
        // Draw detections
        if (this.detections && this.detections.length > 0) {
            this.drawDetections(this.detections);
        }
        
        // Draw boat
        if (this.boat) {
            this.drawBoat(this.boat);
        }
        
        // Draw "No data" message if no boat
        if (!this.boat) {
            this.ctx.fillStyle = '#666666';
            this.ctx.font = '20px Arial';
            this.ctx.textAlign = 'center';
            this.ctx.fillText('Waiting for data...', this.canvas.width / 2, this.canvas.height / 2);
            this.ctx.textAlign = 'left';
        }
    }
}

// Initialize when page loads
window.addEventListener('DOMContentLoaded', () => {
    const visualizer = new MapVisualizer();
});
