/**
 * LiDAR Control System - Main Application Logic
 * Modularized for better maintainability and performance.
 */

// === Configuration ===
const CONFIG = {
    magic: 0x4C494441, // "LIDA"
    headerSize: 8,
    pointSize: 13,
    defaultWsUrl: `ws://${window.location.hostname}:81`,
    reconnectInterval: 2000,
    maxPoints: 100000
};

// === State Management ===
const state = {
    ws: null,
    wsUrl: localStorage.getItem('lidar_ws_url') || CONFIG.defaultWsUrl,
    isConnected: false,
    lastPacketId: 0,
    fps: 0,
    pointCount: 0,
    motor: {
        A: { action: 'stop', speed: 512 },
        B: { action: 'stop', speed: 512 }
    }
};

// === 3D Visualization Engine (Three.js) ===
const Visualizer = {
    scene: null,
    camera: null,
    renderer: null,
    pointCloud: null,
    geometry: null,
    controls: null,
    animationId: null,

    init() {
        const container = document.getElementById('canvas-container');
        
        // Scene Setup
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x000000);
        
        // Camera Setup
        this.camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 1000);
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);

        // Renderer Setup
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        container.appendChild(this.renderer.domElement);

        // Helpers
        const gridHelper = new THREE.GridHelper(20, 20, 0x333333, 0x111111);
        this.scene.add(gridHelper);
        
        const axesHelper = new THREE.AxesHelper(2);
        this.scene.add(axesHelper);

        // Point Cloud Setup
        this.initPointCloud();

        // Event Listeners
        window.addEventListener('resize', () => this.onResize());
        this.setupControls(container);
        
        // Start Loop
        this.animate();
    },

    initPointCloud() {
        this.geometry = new THREE.BufferGeometry();
        
        // Pre-allocate buffers for better performance
        const positions = new Float32Array(CONFIG.maxPoints * 3);
        const colors = new Float32Array(CONFIG.maxPoints * 3);
        
        this.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        this.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        this.geometry.setDrawRange(0, 0);

        const material = new THREE.PointsMaterial({
            size: 0.05,
            vertexColors: true,
            sizeAttenuation: true,
            transparent: true,
            opacity: 0.9
        });

        this.pointCloud = new THREE.Points(this.geometry, material);
        this.scene.add(this.pointCloud);
    },

    updatePoints(positions, colors, count) {
        if (!this.geometry) return;
        
        // Update buffers
        const posAttr = this.geometry.attributes.position;
        const colAttr = this.geometry.attributes.color;
        
        posAttr.array.set(positions);
        colAttr.array.set(colors);
        
        posAttr.needsUpdate = true;
        colAttr.needsUpdate = true;
        
        this.geometry.setDrawRange(0, count);
    },

    setupControls(element) {
        // Simple Orbit Controls implementation
        let isDragging = false;
        let previousMousePosition = { x: 0, y: 0 };

        element.addEventListener('mousedown', (e) => {
            isDragging = true;
            element.style.cursor = 'grabbing';
        });

        document.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const deltaMove = {
                    x: e.offsetX - previousMousePosition.x,
                    y: e.offsetY - previousMousePosition.y
                };

                // Rotate camera around origin
                const theta = -deltaMove.x * 0.005;
                const phi = -deltaMove.y * 0.005;

                const x = this.camera.position.x;
                const y = this.camera.position.y;
                const z = this.camera.position.z;

                this.camera.position.x = x * Math.cos(theta) - z * Math.sin(theta);
                this.camera.position.z = x * Math.sin(theta) + z * Math.cos(theta);
                
                // Vertical rotation (simplified)
                this.camera.position.y += deltaMove.y * 0.05;
                
                this.camera.lookAt(0, 0, 0);
            }
            previousMousePosition = { x: e.offsetX, y: e.offsetY };
        });

        document.addEventListener('mouseup', () => {
            isDragging = false;
            element.style.cursor = 'default';
        });

        // Zoom
        element.addEventListener('wheel', (e) => {
            e.preventDefault();
            const scale = e.deltaY > 0 ? 1.1 : 0.9;
            this.camera.position.multiplyScalar(scale);
        });
    },

    onResize() {
        const container = document.getElementById('canvas-container');
        this.camera.aspect = container.clientWidth / container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(container.clientWidth, container.clientHeight);
    },

    animate() {
        this.animationId = requestAnimationFrame(() => this.animate());
        
        // Auto-rotate if idle? (Optional)
        // if (this.pointCloud) this.pointCloud.rotation.y += 0.0005;

        this.renderer.render(this.scene, this.camera);
    }
};

// === Network Manager ===
const Network = {
    connect() {
        if (state.ws) {
            state.ws.close();
        }

        UI.updateConnectionStatus('connecting');
        console.log(`Connecting to ${state.wsUrl}...`);

        try {
            state.ws = new WebSocket(state.wsUrl);
            state.ws.binaryType = 'arraybuffer';

            state.ws.onopen = () => {
                console.log('WebSocket Connected');
                state.isConnected = true;
                UI.updateConnectionStatus('connected');
                UI.showToast('Connected to LiDAR Server', 'success');
            };

            state.ws.onclose = () => {
                console.log('WebSocket Disconnected');
                state.isConnected = false;
                UI.updateConnectionStatus('disconnected');
                
                // Auto reconnect
                setTimeout(() => this.connect(), CONFIG.reconnectInterval);
            };

            state.ws.onerror = (err) => {
                console.error('WebSocket Error', err);
                state.isConnected = false;
                UI.updateConnectionStatus('error');
            };

            state.ws.onmessage = (event) => {
                if (event.data instanceof ArrayBuffer) {
                    this.parseData(event.data);
                }
            };

        } catch (e) {
            console.error('Connection failed', e);
            UI.updateConnectionStatus('error');
        }
    },

    parseData(buffer) {
        const view = new DataView(buffer);
        
        if (view.byteLength < CONFIG.headerSize) return;
        if (view.getUint32(0, true) !== CONFIG.magic) return;

        const packetId = view.getUint16(4, true);
        const pointCount = view.getUint8(6);
        
        if (pointCount === 0) return;

        const positions = new Float32Array(pointCount * 3);
        const colors = new Float32Array(pointCount * 3);
        
        let offset = CONFIG.headerSize;
        
        for (let i = 0; i < pointCount; i++) {
            if (offset + CONFIG.pointSize > view.byteLength) break;

            const x = view.getFloat32(offset, true);
            const y = view.getFloat32(offset + 4, true);
            const z = view.getFloat32(offset + 8, true);
            const intensity = view.getUint8(offset + 12) / 255.0;

            positions[i * 3] = x;
            positions[i * 3 + 1] = y;
            positions[i * 3 + 2] = z;

            // Color map based on intensity (Blue -> Cyan -> White)
            colors[i * 3] = intensity * 0.5;     // R
            colors[i * 3 + 1] = intensity * 0.8; // G
            colors[i * 3 + 2] = 0.5 + intensity * 0.5; // B

            offset += CONFIG.pointSize;
        }

        // Update Visualizer
        Visualizer.updatePoints(positions, colors, pointCount);
        
        // Update Stats
        state.lastPacketId = packetId;
        state.pointCount = pointCount;
        UI.updateStats();
    },

    sendCommand(cmd) {
        if (!state.isConnected) {
            UI.showToast('Not connected to server', 'error');
            return;
        }
        state.ws.send(JSON.stringify(cmd));
    }
};

// === UI Manager ===
const UI = {
    fpsCounter: 0,
    lastFpsTime: Date.now(),

    init() {
        this.bindEvents();
        this.updateConnectionStatus('disconnected');
        
        // FPS Loop
        setInterval(() => {
            const now = Date.now();
            if (now - this.lastFpsTime >= 1000) {
                state.fps = this.fpsCounter;
                this.fpsCounter = 0;
                this.lastFpsTime = now;
                document.getElementById('fps-display').textContent = `${state.fps} FPS`;
            }
            this.fpsCounter++; // Increment roughly per frame render (simulated here, ideally in animate loop)
        }, 1000/60);
    },

    bindEvents() {
        // Motor Controls
        document.querySelectorAll('.btn-control').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const motor = e.target.dataset.motor;
                const action = e.target.dataset.action;
                
                if (motor && action) {
                    this.setMotorAction(motor, action);
                }
            });
        });

        // Sliders
        ['A', 'B'].forEach(motor => {
            const slider = document.getElementById(`speed-${motor}`);
            slider.addEventListener('input', (e) => {
                const val = e.target.value;
                document.getElementById(`val-${motor}`).textContent = val;
                state.motor[motor].speed = parseInt(val);
            });
        });

        // Send Button
        document.getElementById('send-cmd').addEventListener('click', () => {
            const cmd = {
                type: 'motor_control',
                motorA: state.motor.A,
                motorB: state.motor.B,
                timestamp: Date.now()
            };
            Network.sendCommand(cmd);
            UI.showToast('Commands Sent', 'success');
        });

        // Settings Modal
        const modal = document.getElementById('settings-modal');
        const overlay = document.getElementById('modal-overlay');
        
        document.getElementById('btn-settings').addEventListener('click', () => {
            document.getElementById('input-ws-url').value = state.wsUrl;
            overlay.classList.add('open');
        });

        document.getElementById('btn-close-modal').addEventListener('click', () => {
            overlay.classList.remove('open');
        });

        document.getElementById('btn-save-settings').addEventListener('click', () => {
            const newUrl = document.getElementById('input-ws-url').value;
            if (newUrl) {
                state.wsUrl = newUrl;
                localStorage.setItem('lidar_ws_url', newUrl);
                Network.connect(); // Reconnect with new URL
                overlay.classList.remove('open');
            }
        });
    },

    setMotorAction(motor, action) {
        state.motor[motor].action = action;
        
        // Update UI classes
        document.querySelectorAll(`.btn-control[data-motor="${motor}"]`).forEach(btn => {
            btn.classList.remove('active');
            if (btn.dataset.action === action) {
                btn.classList.add('active');
            }
        });
    },

    updateConnectionStatus(status) {
        const badge = document.getElementById('status-badge');
        const text = document.getElementById('status-text');
        
        badge.className = 'status-badge';
        
        switch(status) {
            case 'connected':
                badge.classList.add('connected');
                text.textContent = 'Connected';
                break;
            case 'connecting':
                badge.classList.add('disconnected'); // Use disconnected style for connecting
                text.textContent = 'Connecting...';
                break;
            default:
                badge.classList.add('disconnected');
                text.textContent = 'Disconnected';
        }
    },

    updateStats() {
        document.getElementById('packet-id').textContent = state.lastPacketId;
        document.getElementById('point-count').textContent = state.pointCount;
        this.fpsCounter++; // Actually count data frames
    },

    showToast(msg, type = 'info') {
        // Simple console log for now, could be expanded to a real toast
        console.log(`[${type.toUpperCase()}] ${msg}`);
        
        const btn = document.getElementById('send-cmd');
        const originalText = btn.innerHTML;
        
        if (msg === 'Commands Sent') {
            btn.innerHTML = '<span>Sent ✓</span>';
            setTimeout(() => {
                btn.innerHTML = originalText;
            }, 2000);
        }
    }
};

// === Boot ===
window.addEventListener('DOMContentLoaded', () => {
    Visualizer.init();
    UI.init();
    Network.connect();
});
