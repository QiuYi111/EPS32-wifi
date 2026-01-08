/**
 * LiDAR Control System - Main Application Logic
 * Modularized for better maintainability and performance.
 */

// === Configuration ===
const CONFIG = {
    magic: 0xDEADBEEF,          // ESP32 packet magic
    headerSize: 11,             // PacketHeader: magic(4) + timestamp(4) + type(1) + length(2)
    defaultWsUrl: `ws://${window.location.hostname}:81`,
    reconnectInterval: 2000,
    maxPoints: 100000
};

// Packet Types (match main.cpp)
const PacketType = {
    LIDAR_RAW: 1,
    MOTOR_STATUS: 2,
    SYSTEM_STATUS: 3,
    MOTOR_CONTROL: 4,
    CAM_JPEG: 5,
    MIXED_DATA: 6
};

// === MAVLink Parser (Unitree LiDAR) ===
const MAVLINK_MAGIC = 0xFD;
const MSG_ID_DISTANCE = 16;
const MSG_ID_AUXILIARY = 17;

class MAVLinkParser {
    constructor() {
        this.buffer = new Uint8Array(0);
        this.auxCache = new Map();  // packet_id -> aux data
        this.distCache = new Map(); // packet_id -> distance data
        this.stats = { distance: 0, auxiliary: 0, clouds: 0 };
    }

    feed(data) {
        // Append new data to buffer
        const newBuffer = new Uint8Array(this.buffer.length + data.length);
        newBuffer.set(this.buffer);
        newBuffer.set(new Uint8Array(data), this.buffer.length);
        this.buffer = newBuffer;
        return this._parse();
    }

    _parse() {
        const points = [];

        while (this.buffer.length >= 12) {
            // Find frame header
            let idx = -1;
            for (let i = 0; i < this.buffer.length; i++) {
                if (this.buffer[i] === MAVLINK_MAGIC) {
                    idx = i;
                    break;
                }
            }

            if (idx < 0) {
                if (this.buffer.length > 1) {
                    this.buffer = this.buffer.slice(-1);
                }
                break;
            }
            if (idx > 0) {
                this.buffer = this.buffer.slice(idx);
            }

            if (this.buffer.length < 12) break;

            const payloadLen = this.buffer[1];
            const incompatFlags = this.buffer[2];
            const hasSignature = (incompatFlags & 0x01) !== 0;
            const packetLen = 10 + payloadLen + 2 + (hasSignature ? 13 : 0);

            if (this.buffer.length < packetLen) break;

            const packet = this.buffer.slice(0, packetLen);
            this.buffer = this.buffer.slice(packetLen);

            // Message ID (3 bytes, little endian)
            const msgId = packet[7] | (packet[8] << 8) | (packet[9] << 16);
            const payload = packet.slice(10, 10 + payloadLen);

            const newPoints = this._dispatch(msgId, payload);
            if (newPoints) points.push(...newPoints);
        }

        // Cleanup old cache entries
        if (this.auxCache.size > 50) {
            const oldest = [...this.auxCache.keys()].slice(0, 10);
            oldest.forEach(k => this.auxCache.delete(k));
        }
        if (this.distCache.size > 50) {
            const oldest = [...this.distCache.keys()].slice(0, 10);
            oldest.forEach(k => this.distCache.delete(k));
        }

        return points;
    }

    _dispatch(msgId, payload) {
        if (msgId === MSG_ID_DISTANCE) {
            this.stats.distance++;
            return this._handleDistance(payload);
        } else if (msgId === MSG_ID_AUXILIARY) {
            this.stats.auxiliary++;
            return this._handleAuxiliary(payload);
        }
        return null;
    }

    _handleDistance(payload) {
        if (payload.length < 246) return null;
        const view = new DataView(payload.buffer, payload.byteOffset);
        const packetId = view.getUint16(0, true);
        this.distCache.set(packetId, payload.slice(6, 246)); // 240 bytes of distance
        return this._tryCombine(packetId);
    }

    _handleAuxiliary(payload) {
        if (payload.length < 209) return null;
        const view = new DataView(payload.buffer, payload.byteOffset);

        // Parse all required fields from auxiliary packet
        // Byte offsets from mavlink header file:
        this.auxCache.set(view.getUint16(84, true), {  // packet_id @ 84
            comHorizontalAngleStart: view.getFloat32(20, true),  // @ 20
            comHorizontalAngleStep: view.getFloat32(24, true),   // @ 24
            sysVerticalAngleStart: view.getFloat32(28, true),    // @ 28
            sysVerticalAngleSpan: view.getFloat32(32, true),     // @ 32
            bAxisDist: view.getFloat32(72, true),                // @ 72
            thetaAngle: view.getFloat32(76, true),               // @ 76
            ksiAngle: view.getFloat32(80, true),                 // @ 80
            reflectData: payload.slice(89, 209)                   // @ 89, 120 bytes
        });

        return this._tryCombine(view.getUint16(84, true));
    }

    _tryCombine(packetId) {
        if (!this.auxCache.has(packetId) || !this.distCache.has(packetId)) {
            return null;
        }

        const aux = this.auxCache.get(packetId);
        const distData = this.distCache.get(packetId);

        this.auxCache.delete(packetId);
        this.distCache.delete(packetId);
        this.stats.clouds++;

        // Constants from SDK
        const rangeScale = 0.001;
        const zBias = 0.0445;
        const pointsNumOfScan = 120;
        const biasLaserBeam = aux.bAxisDist / 1000;

        // Calibration trig values
        const sinTheta = Math.sin(aux.thetaAngle);
        const cosTheta = Math.cos(aux.thetaAngle);
        const sinKsi = Math.sin(aux.ksiAngle);
        const cosKsi = Math.cos(aux.ksiAngle);

        // Angle stepping
        let pitchCur = aux.sysVerticalAngleStart * Math.PI / 180.0;
        const pitchStep = aux.sysVerticalAngleSpan * Math.PI / 180.0;
        let yawCur = aux.comHorizontalAngleStart * Math.PI / 180.0;
        const yawStep = aux.comHorizontalAngleStep / pointsNumOfScan * Math.PI / 180.0;

        const points = [];
        const distView = new DataView(distData.buffer, distData.byteOffset);

        for (let j = 0; j < pointsNumOfScan; j++) {
            // Read distance (little endian uint16)
            const range = distView.getUint16(j * 2, true);
            if (range === 0) {
                pitchCur += pitchStep;
                yawCur += yawStep;
                continue;
            }

            const rangeFloat = rangeScale * range;

            // Trig values for current point
            const sinAlpha = Math.sin(pitchCur);
            const cosAlpha = Math.cos(pitchCur);
            const sinBeta = Math.sin(yawCur);
            const cosBeta = Math.cos(yawCur);

            // SDK transformation formula
            const A = (-cosTheta * sinKsi + sinTheta * sinAlpha * cosKsi) * rangeFloat + biasLaserBeam;
            const B = cosAlpha * cosKsi * rangeFloat;

            const x = cosBeta * A - sinBeta * B;
            const y = sinBeta * A + cosBeta * B;
            const z = (sinTheta * sinKsi + cosTheta * sinAlpha * cosKsi) * rangeFloat + zBias;

            const intensity = aux.reflectData[j] / 255.0;
            points.push({ x, y, z, intensity });

            pitchCur += pitchStep;
            yawCur += yawStep;
        }

        return points;
    }
}

// Global MAVLink parser instance
const mavlinkParser = new MAVLinkParser();


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
    },
    // Visitor Mode State
    isVisitorMode: localStorage.getItem('is_visitor_mode') === 'true',
    visitorSpeed: 400,
    onboardingComplete: localStorage.getItem('onboarding_complete') === 'true'
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
        if (!container) return;
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
        console.log(`正在连接到 ${state.wsUrl}...`);

        try {
            state.ws = new WebSocket(state.wsUrl);
            state.ws.binaryType = 'arraybuffer';

            state.ws.onopen = () => {
                console.log('WebSocket 已连接');
                state.isConnected = true;
                UI.updateConnectionStatus('connected');
                UI.showToast('已连接到 LiDAR 服务器', 'success');
            };

            state.ws.onclose = () => {
                console.log('WebSocket 已断开');
                state.isConnected = false;
                UI.updateConnectionStatus('disconnected');

                // Auto reconnect
                setTimeout(() => this.connect(), CONFIG.reconnectInterval);
            };

            state.ws.onerror = (err) => {
                console.error('WebSocket 错误', err);
                state.isConnected = false;
                UI.updateConnectionStatus('error');
            };

            state.ws.onmessage = (event) => {
                if (event.data instanceof ArrayBuffer) {
                    this.parseData(event.data);
                }
            };

        } catch (e) {
            console.error('连接失败', e);
            UI.updateConnectionStatus('error');
        }
    },

    _processLidarPoints(points) {
        if (points.length > 0) {
            // Accumulate points for display
            if (!state.pointBuffer) state.pointBuffer = [];
            state.pointBuffer.push(...points);

            // Keep max 10000 recent points for performance
            if (state.pointBuffer.length > 10000) {
                state.pointBuffer = state.pointBuffer.slice(-10000);
            }

            // Update visualizer
            const positions = new Float32Array(state.pointBuffer.length * 3);
            const colors = new Float32Array(state.pointBuffer.length * 3);

            state.pointBuffer.forEach((p, i) => {
                positions[i * 3] = p.x;
                positions[i * 3 + 1] = p.z;  // Swap Y/Z for typical 3D view
                positions[i * 3 + 2] = p.y;

                // Color based on intensity (blue -> cyan -> white)
                colors[i * 3] = p.intensity * 0.5;
                colors[i * 3 + 1] = p.intensity * 0.8;
                colors[i * 3 + 2] = 0.5 + p.intensity * 0.5;
            });

            Visualizer.updatePoints(positions, colors, state.pointBuffer.length);
            state.pointCount = state.pointBuffer.length;
        }
    },

    parseData(buffer) {
        const view = new DataView(buffer);

        if (view.byteLength < CONFIG.headerSize) return;

        const magic = view.getUint32(0, true);
        if (magic !== CONFIG.magic) {
            console.warn(`Invalid magic: 0x${magic.toString(16)}`);
            return;
        }

        const timestamp = view.getUint32(4, true);
        const packetType = view.getUint8(8);
        const payloadLength = view.getUint16(9, true);

        if (view.byteLength < CONFIG.headerSize + payloadLength) return;

        switch (packetType) {
            case PacketType.LIDAR_RAW:
                // Parse MAVLink and extract points
                const payload = new Uint8Array(buffer, CONFIG.headerSize, payloadLength);
                const points = mavlinkParser.feed(payload);
                this._processLidarPoints(points);
                state.lidarBytes = (state.lidarBytes || 0) + payloadLength;
                break;

            case PacketType.MOTOR_STATUS:
                if (payloadLength >= 6) {
                    state.motorStatus = {
                        speedA: view.getInt16(11, true),
                        speedB: view.getInt16(13, true),
                        stateA: view.getUint8(15),
                        stateB: view.getUint8(16)
                    };
                    UI.updateMotorStatus();
                }
                break;

            case PacketType.SYSTEM_STATUS:
                if (payloadLength >= 18) {
                    state.systemStatus = {
                        cpuTemp: view.getFloat32(11, true),
                        wifiRssi: view.getInt8(15),
                        freeHeapKb: view.getUint16(16, true),
                        lidarBytes: view.getUint32(18, true),
                        uptimeSeconds: view.getUint32(22, true),
                        wsClients: view.getUint8(26)
                    };
                    UI.updateSystemStatus();
                }
                break;

            case PacketType.CAM_JPEG:
                const blob = new Blob([view.buffer.slice(CONFIG.headerSize)], { type: 'image/jpeg' });
                const url = URL.createObjectURL(blob);
                const img = document.getElementById('camera-feed');
                // Revoke old URL to avoid memory leak
                if (img.src && img.src.startsWith('blob:')) {
                    URL.revokeObjectURL(img.src);
                }
                img.src = url;
                if (!img.classList.contains('active')) {
                    img.classList.add('active');
                    // Hide placeholder
                    const ph = document.querySelector('.camera-placeholder');
                    if (ph) ph.style.display = 'none';
                }

                // Update Cam FPS
                UI.updateCamFps();
                break;

            case PacketType.MIXED_DATA:
                // Format: [CamLen 4] [CamData] [LidarData...]
                if (view.byteLength < CONFIG.headerSize + 4) return;

                const camLen = view.getUint32(CONFIG.headerSize, true); // Little Endian
                const camOffset = CONFIG.headerSize + 4;

                // 1. Process Camera
                if (camLen > 0 && view.byteLength >= camOffset + camLen) {
                    const blob = new Blob([view.buffer.slice(camOffset, camOffset + camLen)], { type: 'image/jpeg' });
                    const url = URL.createObjectURL(blob);
                    const img = document.getElementById('camera-feed');
                    if (img.dataset.lastUrl) URL.revokeObjectURL(img.dataset.lastUrl);
                    img.src = url;
                    img.dataset.lastUrl = url;
                    if (!img.classList.contains('active')) {
                        img.classList.add('active');
                        const ph = document.querySelector('.camera-placeholder');
                        if (ph) ph.style.display = 'none';
                    }
                    UI.updateCamFps();
                }

                // 2. Process LiDAR (Rest of the packet)
                const lidarOffset = camOffset + camLen;
                if (view.byteLength > lidarOffset) {
                    // Extract LiDAR payload
                    const lidarData = new Uint8Array(buffer, lidarOffset);
                    // Feed to MAVLink parser
                    // Note: We need to feed raw bytes to mavlinkParser which expects Uint8Array
                    const points = mavlinkParser.feed(lidarData);
                    this._processLidarPoints(points);
                    state.lidarBytes = (state.lidarBytes || 0) + lidarData.length;
                }
                break;
        }

        state.lastPacketId++;
        UI.updateStats();
    },

    /**
     * Send binary motor control command
     * @param {number} speedA - Motor A speed (-1024 to +1024)
     * @param {number} speedB - Motor B speed (-1024 to +1024)  
     * @param {boolean} brakeA - Apply brake to motor A
     * @param {boolean} brakeB - Apply brake to motor B
     */
    sendMotorCommand(speedA, speedB, brakeA = false, brakeB = false) {
        if (!state.isConnected) {
            UI.showToast('未连接到服务器', 'error');
            return;
        }

        // Clamp speeds to valid range
        speedA = Math.max(-1024, Math.min(1024, speedA));
        speedB = Math.max(-1024, Math.min(1024, speedB));

        // Create binary packet: header(11) + payload(5) = 16 bytes
        const buffer = new ArrayBuffer(16);
        const view = new DataView(buffer);

        // PacketHeader
        view.setUint32(0, CONFIG.magic, true);           // magic
        view.setUint32(4, Date.now() & 0xFFFFFFFF, true); // timestamp
        view.setUint8(8, PacketType.MOTOR_CONTROL);      // type
        view.setUint16(9, 5, true);                      // length (MotorControlCmd size)

        // MotorControlCmd payload
        view.setInt16(11, speedA, true);                 // speed_a
        view.setInt16(13, speedB, true);                 // speed_b
        view.setUint8(15, (brakeA ? 1 : 0) | (brakeB ? 2 : 0)); // flags

        state.ws.send(buffer);
        console.log(`[Motor] Sent: A=${speedA}, B=${speedB}, brake=${brakeA},${brakeB}`);
    }
};

// === UI Manager ===
const UI = {
    fpsCounter: 0,
    camFpsCounter: 0,
    lastFpsTime: Date.now(),
    lastCamFpsTime: Date.now(),

    init() {
        this.bindEvents();
        this.updateConnectionStatus('disconnected');

        // FPS Loop (Render)
        setInterval(() => {
            const now = Date.now();
            if (now - this.lastFpsTime >= 1000) {
                state.fps = this.fpsCounter;
                this.fpsCounter = 0;
                this.lastFpsTime = now;
                document.getElementById('fps-display').textContent = `${state.fps} FPS`;
            }
            this.fpsCounter++;
        }, 1000 / 60);

        // Cam FPS reset logic (check every 1s)
        setInterval(() => {
            const now = Date.now();
            if (now - this.lastCamFpsTime >= 1000) {
                const fps = this.camFpsCounter;
                document.getElementById('cam-fps').textContent = fps;
                this.camFpsCounter = 0;
                this.lastCamFpsTime = now;
            }
        }, 1000);
    },

    updateCamFps() {
        this.camFpsCounter++;
    },

    bindEvents() {
        // Visitor Mode Toggle
        const modeToggle = document.getElementById('mode-toggle-checkbox');
        modeToggle.checked = state.isVisitorMode;
        modeToggle.addEventListener('change', (e) => {
            this.toggleVisitorMode(e.target.checked);
        });

        // Visitor D-Pad
        ['up', 'down', 'left', 'right', 'stop'].forEach(dir => {
            const btn = document.getElementById(`dpad-${dir}`);
            if (btn) {
                btn.addEventListener('mousedown', () => this.handleDpadCommand(dir, true));
                btn.addEventListener('mouseup', () => this.handleDpadCommand(dir, false));
                btn.addEventListener('mouseleave', () => this.handleDpadCommand(dir, false));
                // Touch support
                btn.addEventListener('touchstart', (e) => {
                    e.preventDefault();
                    this.handleDpadCommand(dir, true);
                });
                btn.addEventListener('touchend', () => this.handleDpadCommand(dir, false));
            }
        });

        // Speed Chips
        document.querySelectorAll('.speed-chip').forEach(chip => {
            chip.addEventListener('click', (e) => {
                const speed = parseInt(e.target.dataset.speed);
                state.visitorSpeed = speed;
                document.querySelectorAll('.speed-chip').forEach(c => c.classList.remove('active'));
                e.target.classList.add('active');
            });
        });

        // Onboarding Navigation
        document.querySelectorAll('.next-step').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const nextStep = e.target.dataset.next;
                if (nextStep === 'done') {
                    Tour.finish();
                } else {
                    Tour.goToStep(nextStep);
                }
            });
        });

        document.querySelector('.skip-tour').addEventListener('click', () => Tour.finish());

        // Keyboard Support
        window.addEventListener('keydown', (e) => this.handleKeyboard(e, true));
        window.addEventListener('keyup', (e) => this.handleKeyboard(e, false));

        // Motor Controls (Original)
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
            // Convert action + speed to signed speed values
            const getSignedSpeed = (motor) => {
                const m = state.motor[motor];
                const speed = m.speed || 512;
                switch (m.action) {
                    case 'forward': return speed;
                    case 'backward': return -speed;
                    case 'stop': return 0;
                    case 'brake': return 0; // handled by brake flag
                    default: return 0;
                }
            };

            const speedA = getSignedSpeed('A');
            const speedB = getSignedSpeed('B');
            const brakeA = state.motor.A.action === 'brake';
            const brakeB = state.motor.B.action === 'brake';

            Network.sendMotorCommand(speedA, speedB, brakeA, brakeB);
            UI.showToast('命令已发送', 'success');
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

        // Help Button
        document.getElementById('btn-help').addEventListener('click', () => Tour.start());

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

        switch (status) {
            case 'connected':
                badge.classList.add('connected');
                text.textContent = '已连接';
                break;
            case 'connecting':
                badge.classList.add('disconnected'); // Use disconnected style for connecting
                text.textContent = '正在连接...';
                break;
            default:
                badge.classList.add('disconnected');
                text.textContent = '已断开';
        }
    },

    toggleVisitorMode(enabled) {
        state.isVisitorMode = enabled;
        localStorage.setItem('is_visitor_mode', enabled);

        const devControls = document.getElementById('developer-controls');
        const visitorControls = document.getElementById('visitor-controls');
        const hudElements = document.querySelectorAll('.hud-overlay:not(.hud-top-right)');

        if (enabled) {
            devControls.classList.add('hidden');
            visitorControls.classList.remove('hidden');
            hudElements.forEach(el => el.classList.add('hidden'));
            UI.showToast('访客模式已激活', 'info');
        } else {
            devControls.classList.remove('hidden');
            visitorControls.classList.add('hidden');
            hudElements.forEach(el => el.classList.remove('hidden'));
            UI.showToast('开发模式已激活', 'info');
        }
    },

    handleDpadCommand(dir, isPressed) {
        if (!isPressed) {
            Network.sendMotorCommand(0, 0, true, true);
            return;
        }

        const speed = state.visitorSpeed;
        switch (dir) {
            case 'up':
                Network.sendMotorCommand(speed, speed);
                break;
            case 'down':
                Network.sendMotorCommand(-speed, -speed);
                break;
            case 'left':
                Network.sendMotorCommand(-speed, speed);
                break;
            case 'right':
                Network.sendMotorCommand(speed, -speed);
                break;
            case 'stop':
                Network.sendMotorCommand(0, 0, true, true);
                break;
        }
    },

    handleKeyboard(e, isPressed) {
        // Only handle if not in an input field
        if (e.target.tagName === 'INPUT') return;

        const key = e.key.toLowerCase();
        const keyMap = {
            'w': 'up',
            's': 'down',
            'a': 'left',
            'd': 'right',
            ' ': 'stop',
            'arrowup': 'up',
            'arrowdown': 'down',
            'arrowleft': 'left',
            'arrowright': 'right'
        };

        if (keyMap[key]) {
            e.preventDefault();
            this.handleDpadCommand(keyMap[key], isPressed);

            // Visual feedback for D-Pad buttons
            const btn = document.getElementById(`dpad-${keyMap[key]}`);
            if (btn) {
                if (isPressed) btn.classList.add('active');
                else btn.classList.remove('active');
            }
        }
    },

    updateStats() {
        document.getElementById('packet-id').textContent = state.lastPacketId;
        const pointCount = state.pointCount || 0;
        document.getElementById('point-count').textContent = pointCount;

        // Visitor View Stats
        const visitorPoints = document.getElementById('visitor-points');
        if (visitorPoints) visitorPoints.textContent = pointCount;

        const visitorFps = document.getElementById('visitor-fps');
        if (visitorFps) visitorFps.textContent = state.fps;

        this.fpsCounter++; // Actually count data frames
    },

    updateMotorStatus() {
        if (!state.motorStatus) return;
        const stateNames = ['停止', '前进', '后退', '制动'];
        // Could update UI elements if added to HTML
        console.log(`[MotorStatus] A=${state.motorStatus.speedA}(${stateNames[state.motorStatus.stateA]}), ` +
            `B=${state.motorStatus.speedB}(${stateNames[state.motorStatus.stateB]})`);
    },

    updateSystemStatus() {
        if (!state.systemStatus) return;
        const s = state.systemStatus;
        // Update data grid if elements exist
        const cpuEl = document.querySelector('.data-value.cpu-temp');
        const heapEl = document.querySelector('.data-value.heap');
        const uptimeEl = document.querySelector('.data-value.uptime');

        if (cpuEl) cpuEl.textContent = `${s.cpuTemp.toFixed(1)}°C`;
        if (heapEl) heapEl.textContent = `${s.freeHeapKb} KB`;
        if (uptimeEl) uptimeEl.textContent = `${s.uptimeSeconds}s`;

        console.log(`[SystemStatus] CPU=${s.cpuTemp.toFixed(1)}°C, Heap=${s.freeHeapKb}KB, ` +
            `Uptime=${s.uptimeSeconds}s, Clients=${s.wsClients}`);
    },

    showToast(msg, type = 'info') {
        console.log(`[${type.toUpperCase()}] ${msg}`);
        // Simple visual feedback for motor commands if in dev mode
        if (!state.isVisitorMode) {
            const btn = document.getElementById('send-cmd');
            const originalText = btn.innerHTML;
            if (msg === '命令已发送') {
                btn.innerHTML = '<span>已发送 ✓</span>';
                setTimeout(() => { btn.innerHTML = originalText; }, 2000);
            }
        }
    }
};

// === Tour Manager ===
const Tour = {
    init() {
        if (!state.onboardingComplete) {
            this.start();
        } else {
            document.getElementById('onboarding-overlay').classList.add('hidden');
            document.getElementById('onboarding-card').classList.add('hidden');
            // If visitor mode was previously saved, apply it
            UI.toggleVisitorMode(state.isVisitorMode);
        }
    },

    start() {
        const overlay = document.getElementById('onboarding-overlay');
        const card = document.getElementById('onboarding-card');

        // Reset styles in case they were left at 0 opacity
        overlay.style.opacity = '1';
        card.style.opacity = '1';

        overlay.classList.remove('hidden');
        card.classList.remove('hidden');
        this.goToStep('step-1');
    },

    goToStep(stepId) {
        this.cleanupHighlight(); // Clear previous highlight first

        document.querySelectorAll('.onboarding-content').forEach(el => el.classList.add('hidden'));
        const nextStep = document.getElementById(stepId);

        if (nextStep) {
            nextStep.classList.remove('hidden');

            // Special handling: Enable visitor mode for control-related steps
            if (['step-4', 'step-5', 'step-6'].includes(stepId)) {
                if (!state.isVisitorMode) {
                    UI.toggleVisitorMode(true);
                    document.getElementById('mode-toggle-checkbox').checked = true;
                }
            }

            // Highlight targets (optional feature)
            const targetSelector = nextStep.querySelector('.tour-highlight-ref')?.dataset.target;
            if (targetSelector) {
                const target = document.querySelector(targetSelector);
                if (target) {
                    target.style.outline = '4px solid var(--md-sys-color-primary)';
                    target.style.boxShadow = '0 0 0 1000vw rgba(0,0,0,0.5)';
                    target.style.zIndex = '500';
                    target.style.position = 'relative'; // Ensure z-index works
                    target.scrollIntoView({ behavior: 'smooth', block: 'center' });

                    // Cleanup highlight when moving to next step
                    this.currentTarget = target;
                }
            }
        }
    },

    cleanupHighlight() {
        if (this.currentTarget) {
            this.currentTarget.style.outline = '';
            this.currentTarget.style.boxShadow = '';
            this.currentTarget.style.zIndex = '';
            this.currentTarget.style.position = '';
            this.currentTarget = null;
        }
    },

    finish() {
        this.cleanupHighlight();
        document.getElementById('onboarding-overlay').style.opacity = '0';
        document.getElementById('onboarding-card').style.opacity = '0';
        setTimeout(() => {
            document.getElementById('onboarding-overlay').classList.add('hidden');
            document.getElementById('onboarding-card').classList.add('hidden');
            state.onboardingComplete = true;
            localStorage.setItem('onboarding_complete', 'true');
            UI.toggleVisitorMode(true); // Default to visitor mode after tour
        }, 500);
    }
};

// === Boot ===
window.addEventListener('DOMContentLoaded', () => {
    Visualizer.init();
    UI.init();
    Tour.init(); // Initialize tour after UI
    Network.connect();
});

