/**
 * AeroCommand GCS - Web Serial Ground Control Station
 * Real-time 3D attitude visualizer, sensor oscilloscope, and motor dashboard.
 */

// =============================================================================
// 1. State & Telemetry Variables
// =============================================================================
const state = {
  port: null,
  reader: null,
  writer: null,
  connected: false,
  demoMode: false,
  demoTimer: null,
  
  // Latest Telemetry Frame
  telem: {
    systemState: 'DISCONNECTED',
    roll: 0.0,
    pitch: 0.0,
    yawRate: 0.0,
    ax: 0.0,
    ay: 0.0,
    az: 1.0,
    gx: 0.0,
    gy: 0.0,
    gz: 0.0,
    m1: 0,
    m2: 0,
    m3: 0,
    m4: 0,
    battV: 3.85,
    battPct: 100,
    loopHz: 500.0,
    armed: false
  },

  // Oscilloscope History Buffers (140 points)
  maxSamples: 140,
  sensorTab: 'accel', // 'accel' or 'gyro'
  accelHistory: { x: [], y: [], z: [] },
  gyroHistory:  { x: [], y: [], z: [] }
};

// =============================================================================
// 2. DOM Elements Cache
// =============================================================================
const dom = {
  connectBtn: document.getElementById('connect-btn'),
  connectBtnText: document.getElementById('connect-btn-text'),
  demoModeBtn: document.getElementById('demo-mode-btn'),
  emgStopBtn: document.getElementById('emg-stop-btn'),
  statusDot: document.getElementById('system-status-dot'),
  stateLabel: document.getElementById('system-state-label'),
  loopFreqLabel: document.getElementById('loop-freq-label'),
  batteryLabel: document.getElementById('battery-label'),
  
  hudRoll: document.getElementById('hud-roll-val'),
  hudPitch: document.getElementById('hud-pitch-val'),
  hudYawRate: document.getElementById('hud-yawrate-val'),
  resetCamBtn: document.getElementById('reset-cam-btn'),
  
  // Motors & Hub
  hubArmState: document.getElementById('hub-arm-state'),
  gaugeM1: document.getElementById('gauge-m1'),
  gaugeM2: document.getElementById('gauge-m2'),
  gaugeM3: document.getElementById('gauge-m3'),
  gaugeM4: document.getElementById('gauge-m4'),
  dutyM1: document.getElementById('duty-m1'),
  dutyM2: document.getElementById('duty-m2'),
  dutyM3: document.getElementById('duty-m3'),
  dutyM4: document.getElementById('duty-m4'),
  pctM1: document.getElementById('pct-m1'),
  pctM2: document.getElementById('pct-m2'),
  pctM3: document.getElementById('pct-m3'),
  pctM4: document.getElementById('pct-m4'),
  satBadge: document.getElementById('sat-badge'),
  
  // Sensors
  tabAccel: document.getElementById('tab-accel'),
  tabGyro: document.getElementById('tab-gyro'),
  canvas: document.getElementById('oscilloscope-canvas'),
  valX: document.getElementById('val-x'),
  valY: document.getElementById('val-y'),
  valZ: document.getElementById('val-z'),
  
  // Tools
  btnCalibrate: document.getElementById('btn-calibrate'),
  btnArmToggle: document.getElementById('btn-arm-toggle'),
  btnDisarm: document.getElementById('btn-disarm'),
  motorSelect: document.getElementById('motor-select'),
  motorSlider: document.getElementById('motor-test-slider'),
  motorSliderVal: document.getElementById('motor-test-val'),
  btnPulseMotor: document.getElementById('btn-pulse-motor'),
  simRollSlider: document.getElementById('sim-roll-slider'),
  simRollVal: document.getElementById('sim-roll-val'),
  simPitchSlider: document.getElementById('sim-pitch-slider'),
  simPitchVal: document.getElementById('sim-pitch-val'),
  btnInjectSim: document.getElementById('btn-inject-sim'),
  
  // Terminal
  termOutput: document.getElementById('terminal-output'),
  termForm: document.getElementById('terminal-form'),
  termInput: document.getElementById('terminal-input'),
  clearTermBtn: document.getElementById('clear-term-btn'),
  autoscrollChk: document.getElementById('autoscroll-chk')
};

// =============================================================================
// 3. Three.js 3D Quadcopter Attitude Viewport
// =============================================================================
let scene, camera, renderer, droneGroup;
const targetRotation = { x: 0, y: 0, z: 0 };
let isUserInteracting = false;
let pointerStartX = 0, pointerStartY = 0;
let camPhi = 0.5, camTheta = 0.0, camRadius = 7.0;

function initThreeJS() {
  const container = document.getElementById('three-container');
  const width = container.clientWidth || 600;
  const height = container.clientHeight || 380;

  scene = new THREE.Scene();

  camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
  updateCameraPosition();

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.appendChild(renderer.domElement);

  // Lighting
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.75);
  scene.add(ambientLight);

  const cyanKeyLight = new THREE.DirectionalLight(0x38bdf8, 1.4);
  cyanKeyLight.position.set(5, 12, 7);
  scene.add(cyanKeyLight);

  const rimLight = new THREE.DirectionalLight(0x10b981, 0.7);
  rimLight.position.set(-6, -4, -6);
  scene.add(rimLight);

  // Grid Plane with subtle perspective
  const grid = new THREE.GridHelper(12, 24, 0x38bdf8, 0x1e293b);
  grid.position.y = -1.6;
  scene.add(grid);

  // Build Procedural High-Fidelity Drone Model
  droneGroup = new THREE.Group();

  // Central Fuselage / Electronics Enclosure
  const bodyGeo = new THREE.BoxGeometry(0.85, 0.16, 0.85);
  const carbonMat = new THREE.MeshStandardMaterial({ 
    color: 0x0f172a, 
    roughness: 0.35, 
    metalness: 0.85 
  });
  const bodyMesh = new THREE.Mesh(bodyGeo, carbonMat);
  droneGroup.add(bodyMesh);

  // Top Shield / Battery Pod
  const shieldGeo = new THREE.BoxGeometry(0.5, 0.08, 0.5);
  const shieldMat = new THREE.MeshStandardMaterial({ 
    color: 0x1e293b, 
    roughness: 0.2, 
    metalness: 0.6 
  });
  const shieldMesh = new THREE.Mesh(shieldGeo, shieldMat);
  shieldMesh.position.y = 0.12;
  droneGroup.add(shieldMesh);

  // Forward Heading Beacon (Electric Cyan arrow)
  const arrowGeo = new THREE.ConeGeometry(0.16, 0.42, 16);
  const arrowMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });
  const arrowMesh = new THREE.Mesh(arrowGeo, arrowMat);
  arrowMesh.rotation.x = -Math.PI / 2;
  arrowMesh.position.set(0, 0.14, -0.68);
  droneGroup.add(arrowMesh);

  // Quad-X Carbon Arms & Motors (X Configuration)
  const armLen = 1.45;
  const armConfigs = [
    { name: 'M1_FL', angle: -Math.PI * 0.75, color: 0x38bdf8, isFront: true },  // Front-Left (CW)
    { name: 'M2_FR', angle: -Math.PI * 0.25, color: 0x10b981, isFront: true },  // Front-Right (CCW)
    { name: 'M3_RR', angle:  Math.PI * 0.25, color: 0x38bdf8, isFront: false }, // Rear-Right (CW)
    { name: 'M4_RL', angle:  Math.PI * 0.75, color: 0x10b981, isFront: false }  // Rear-Left (CCW)
  ];

  armConfigs.forEach(arm => {
    // Carbon Fiber Arm Tube
    const armGeo = new THREE.CylinderGeometry(0.042, 0.042, armLen, 12);
    const armMesh = new THREE.Mesh(armGeo, carbonMat);
    armMesh.rotation.z = Math.PI / 2;
    armMesh.rotation.y = arm.angle;
    armMesh.position.set(
      Math.sin(arm.angle) * (armLen / 2),
      0,
      Math.cos(arm.angle) * (armLen / 2)
    );
    droneGroup.add(armMesh);

    // Motor Mount / Bell (Brushed Aluminum)
    const motorGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.22, 16);
    const motorMat = new THREE.MeshStandardMaterial({ 
      color: 0x94a3b8, 
      metalness: 0.9, 
      roughness: 0.25 
    });
    const motorMesh = new THREE.Mesh(motorGeo, motorMat);
    const posX = Math.sin(arm.angle) * armLen;
    const posZ = Math.cos(arm.angle) * armLen;
    motorMesh.position.set(posX, 0.1, posZ);
    droneGroup.add(motorMesh);

    // Propeller Disc / Blades
    const propGeo = new THREE.BoxGeometry(0.85, 0.012, 0.075);
    const propMat = new THREE.MeshBasicMaterial({ 
      color: arm.color, 
      transparent: true, 
      opacity: 0.85 
    });
    const propMesh = new THREE.Mesh(propGeo, propMat);
    propMesh.position.set(posX, 0.22, posZ);
    propMesh.name = `prop_${arm.name}`;
    droneGroup.add(propMesh);
  });

  scene.add(droneGroup);

  // Mouse & Touch Orbit Controls
  container.addEventListener('pointerdown', (e) => {
    isUserInteracting = true;
    pointerStartX = e.clientX;
    pointerStartY = e.clientY;
  });

  window.addEventListener('pointermove', (e) => {
    if (!isUserInteracting) return;
    const dx = e.clientX - pointerStartX;
    const dy = e.clientY - pointerStartY;
    pointerStartX = e.clientX;
    pointerStartY = e.clientY;

    camTheta -= dx * 0.008;
    camPhi = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, camPhi + dy * 0.008));
    updateCameraPosition();
  });

  window.addEventListener('pointerup', () => {
    isUserInteracting = false;
  });

  // Responsive Resize
  window.addEventListener('resize', () => {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  });

  animateThreeJS();
}

function updateCameraPosition() {
  camera.position.x = camRadius * Math.sin(camPhi) * Math.sin(camTheta);
  camera.position.y = camRadius * Math.cos(camPhi);
  camera.position.z = camRadius * Math.sin(camPhi) * Math.cos(camTheta);
  camera.lookAt(0, 0, 0);
}

function resetCamera() {
  camPhi = 0.52;
  camTheta = 0.0;
  camRadius = 7.0;
  updateCameraPosition();
}

function animateThreeJS() {
  requestAnimationFrame(animateThreeJS);

  if (droneGroup) {
    // Smooth, jitter-free Euler interpolation towards target orientation
    droneGroup.rotation.x += (targetRotation.x - droneGroup.rotation.x) * 0.18;
    droneGroup.rotation.z += (targetRotation.z - droneGroup.rotation.z) * 0.18;
    droneGroup.rotation.y += (targetRotation.y - droneGroup.rotation.y) * 0.18;

    // Propeller spinning
    const spinSpeed = (state.telem.armed || state.demoMode) ? 0.38 : 0.02;
    droneGroup.children.forEach(child => {
      if (child.name && child.name.startsWith('prop_')) {
        child.rotation.y += spinSpeed;
      }
    });
  }

  renderer.render(scene, camera);
}

// =============================================================================
// 4. Sensor Waveform Canvas Oscilloscope
// =============================================================================
function initOscilloscope() {
  const canvas = dom.canvas;
  const ctx = canvas.getContext('2d');

  function resizeCanvas() {
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = canvas.parentElement.clientHeight;
  }
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);

  function drawScope() {
    requestAnimationFrame(drawScope);

    const w = canvas.width;
    const h = canvas.height;
    const midY = h / 2;

    ctx.clearRect(0, 0, w, h);

    // Center Reference Line
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(0, midY);
    ctx.lineTo(w, midY);
    ctx.stroke();
    ctx.setLineDash([]);

    const data = (state.sensorTab === 'accel') ? state.accelHistory : state.gyroHistory;
    const colors = { x: '#f43f5e', y: '#10b981', z: '#38bdf8' };
    const scale = (state.sensorTab === 'accel') ? (h / 3.2) : (h / 260);

    ['x', 'y', 'z'].forEach(channel => {
      const arr = data[channel];
      if (arr.length < 2) return;

      ctx.strokeStyle = colors[channel];
      ctx.lineWidth = 1.8;
      ctx.shadowColor = colors[channel];
      ctx.shadowBlur = 5;
      ctx.beginPath();

      const step = w / (state.maxSamples - 1);
      for (let i = 0; i < arr.length; i++) {
        const x = i * step;
        const y = midY - (arr[i] * scale);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.shadowBlur = 0;
    });
  }

  drawScope();
}

// =============================================================================
// 5. Telemetry Processing & UI Updates
// =============================================================================
function updateTelemetryUI() {
  const t = state.telem;

  // 1. Status Dot & System State
  dom.stateLabel.textContent = t.systemState;
  dom.statusDot.className = 'pulse-dot';
  if (t.systemState === 'ARMED' || t.systemState === 'FLIGHT') {
    dom.statusDot.classList.add('armed');
  } else if (t.systemState === 'FAILSAFE' || t.systemState === 'ERROR') {
    dom.statusDot.classList.add('alert');
  } else if (state.connected || state.demoMode) {
    dom.statusDot.classList.add('online');
  }

  // Hub Center Status
  if (dom.hubArmState) {
    dom.hubArmState.textContent = t.armed ? 'ARMED' : 'DISARMED';
    dom.hubArmState.style.color = t.armed ? 'var(--accent-amber)' : 'var(--accent-emerald)';
  }

  dom.loopFreqLabel.textContent = `${t.loopHz.toFixed(1)} Hz`;
  dom.batteryLabel.textContent = `${t.battV.toFixed(2)} V (${t.battPct.toFixed(0)}%)`;

  // 2. HUD & 3D Drone Orientation
  targetRotation.x = (t.pitch * Math.PI) / 180;
  targetRotation.z = -(t.roll * Math.PI) / 180; // Right wing down is positive roll
  targetRotation.y = 0;

  dom.hudRoll.textContent = `${t.roll >= 0 ? '+' : ''}${t.roll.toFixed(2)}°`;
  dom.hudPitch.textContent = `${t.pitch >= 0 ? '+' : ''}${t.pitch.toFixed(2)}°`;
  dom.hudYawRate.textContent = `${t.yawRate >= 0 ? '+' : ''}${t.yawRate.toFixed(2)}°/s`;

  // 3. Quad-X Motor Gauges (Radius 23 -> Circumference 144.51)
  const circumference = 144.51;
  const updateMotor = (gauge, dutyEl, pctEl, val) => {
    const clamped = Math.min(1023, Math.max(0, val));
    const offset = circumference * (1 - clamped / 1023);
    gauge.style.strokeDashoffset = offset;

    // Heat color on high duty
    if (clamped > 950) gauge.style.stroke = '#f43f5e';
    else if (clamped > 750) gauge.style.stroke = '#f59e0b';
    else gauge.style.stroke = '#38bdf8';

    dutyEl.textContent = Math.round(clamped);
    pctEl.textContent = `${((clamped / 1023) * 100).toFixed(1)}%`;
  };

  updateMotor(dom.gaugeM1, dom.dutyM1, dom.pctM1, t.m1);
  updateMotor(dom.gaugeM2, dom.dutyM2, dom.pctM2, t.m2);
  updateMotor(dom.gaugeM3, dom.dutyM3, dom.pctM3, t.m3);
  updateMotor(dom.gaugeM4, dom.dutyM4, dom.pctM4, t.m4);

  // Saturation Badge
  const maxM = Math.max(t.m1, t.m2, t.m3, t.m4);
  if (maxM >= 1020) {
    dom.satBadge.textContent = 'PWM: THROTTLE SCALED';
    dom.satBadge.style.color = '#f43f5e';
  } else {
    dom.satBadge.textContent = 'PWM: NORMAL';
    dom.satBadge.style.color = 'var(--accent-cyan)';
  }

  // 4. Oscilloscope Buffers
  const pushSample = (buf, val) => {
    buf.push(val);
    if (buf.length > state.maxSamples) buf.shift();
  };

  pushSample(state.accelHistory.x, t.ax);
  pushSample(state.accelHistory.y, t.ay);
  pushSample(state.accelHistory.z, t.az);

  pushSample(state.gyroHistory.x, t.gx);
  pushSample(state.gyroHistory.y, t.gy);
  pushSample(state.gyroHistory.z, t.gz);

  if (state.sensorTab === 'accel') {
    dom.valX.textContent = `${t.ax >= 0 ? '+' : ''}${t.ax.toFixed(3)} g`;
    dom.valY.textContent = `${t.ay >= 0 ? '+' : ''}${t.ay.toFixed(3)} g`;
    dom.valZ.textContent = `${t.az >= 0 ? '+' : ''}${t.az.toFixed(3)} g`;
  } else {
    dom.valX.textContent = `${t.gx >= 0 ? '+' : ''}${t.gx.toFixed(1)} °/s`;
    dom.valY.textContent = `${t.gy >= 0 ? '+' : ''}${t.gy.toFixed(1)} °/s`;
    dom.valZ.textContent = `${t.gz >= 0 ? '+' : ''}${t.gz.toFixed(1)} °/s`;
  }
}

// Parse $TELEM packet: $TELEM,state,roll,pitch,yaw_rate,ax,ay,az,gx,gy,gz,m1,m2,m3,m4,batt_v,batt_pct,loop_hz,armed
function parseTelemetryLine(line) {
  if (!line.startsWith('$TELEM,')) return false;

  const parts = line.trim().split(',');
  if (parts.length < 18) return false;

  state.telem.systemState = parts[1];
  state.telem.roll        = parseFloat(parts[2]);
  state.telem.pitch       = parseFloat(parts[3]);
  state.telem.yawRate     = parseFloat(parts[4]);
  state.telem.ax          = parseFloat(parts[5]);
  state.telem.ay          = parseFloat(parts[6]);
  state.telem.az          = parseFloat(parts[7]);
  state.telem.gx          = parseFloat(parts[8]);
  state.telem.gy          = parseFloat(parts[9]);
  state.telem.gz          = parseFloat(parts[10]);
  state.telem.m1          = parseFloat(parts[11]);
  state.telem.m2          = parseFloat(parts[12]);
  state.telem.m3          = parseFloat(parts[13]);
  state.telem.m4          = parseFloat(parts[14]);
  state.telem.battV       = parseFloat(parts[15]);
  state.telem.battPct     = parseFloat(parts[16]);
  state.telem.loopHz      = parseFloat(parts[17]);
  state.telem.armed       = (parts[18] === '1');

  updateTelemetryUI();
  return true;
}

// =============================================================================
// 6. Web Serial API Driver
// =============================================================================
async function connectSerial() {
  if (!('serial' in navigator)) {
    logTerminal('ERROR: Web Serial API is not supported in this browser. Please use Google Chrome or Microsoft Edge.', 'error');
    alert('Web Serial API requires Google Chrome or Microsoft Edge.');
    return;
  }

  try {
    state.port = await navigator.serial.requestPort();
    await state.port.open({ baudRate: 115200 });

    state.connected = true;
    dom.connectBtnText.textContent = 'Disconnect';
    dom.connectBtn.classList.remove('btn-primary');
    dom.connectBtn.classList.add('btn-subtle');

    logTerminal('Connected to Seeed Studio XIAO ESP32-S3 at 115200 baud.', 'info');

    // Turn off Demo mode if active
    if (state.demoMode) toggleDemoMode();

    // Start reading stream
    readSerialLoop();

    // Send command to enable 20 Hz streaming
    setTimeout(() => {
      sendSerialCommand('stream on');
    }, 350);

  } catch (err) {
    logTerminal(`Connection error: ${err.message}`, 'error');
  }
}

async function disconnectSerial() {
  try {
    if (state.connected) {
      await sendSerialCommand('stream off');
    }
    if (state.reader) {
      await state.reader.cancel();
      state.reader = null;
    }
    if (state.port) {
      await state.port.close();
      state.port = null;
    }
  } catch (err) {
    console.error(err);
  } finally {
    state.connected = false;
    dom.connectBtnText.textContent = 'Connect Drone';
    dom.connectBtn.classList.remove('btn-subtle');
    dom.connectBtn.classList.add('btn-primary');
    dom.stateLabel.textContent = 'DISCONNECTED';
    dom.statusDot.className = 'pulse-dot';
    logTerminal('Serial Port disconnected.', 'warn');
  }
}

async function readSerialLoop() {
  const textDecoder = new TextDecoderStream();
  const readableStreamClosed = state.port.readable.pipeTo(textDecoder.writable);
  state.reader = textDecoder.readable.getReader();

  let lineBuffer = '';

  try {
    while (state.connected) {
      const { value, done } = await state.reader.read();
      if (done) break;
      if (value) {
        lineBuffer += value;
        const lines = lineBuffer.split('\n');
        lineBuffer = lines.pop(); // Retain incomplete chunk

        for (const line of lines) {
          const clean = line.trim();
          if (!clean) continue;

          // Check if line is telemetry packet
          const isTelem = parseTelemetryLine(clean);
          if (!isTelem) {
            logTerminal(clean);
          }
        }
      }
    }
  } catch (err) {
    console.warn('Read loop terminated:', err);
  }
}

async function sendSerialCommand(cmd) {
  if (!state.connected || !state.port || !state.port.writable) {
    logTerminal(`Cannot send '${cmd}': not connected to drone.`, 'warn');
    return;
  }

  try {
    const encoder = new TextEncoder();
    const writer = state.port.writable.getWriter();
    await writer.write(encoder.encode(cmd + '\r\n'));
    writer.releaseLock();
    logTerminal(`> ${cmd}`, 'info');
  } catch (err) {
    logTerminal(`TX Error: ${err.message}`, 'error');
  }
}

// =============================================================================
// 7. Interactive Terminal Logger
// =============================================================================
function logTerminal(msg, type = '') {
  const line = document.createElement('div');
  line.className = `log-line ${type}`.trim();
  line.textContent = msg;
  dom.termOutput.appendChild(line);

  // Keep terminal buffer bounded to 250 items
  if (dom.termOutput.children.length > 250) {
    dom.termOutput.removeChild(dom.termOutput.firstChild);
  }

  if (dom.autoscrollChk.checked) {
    dom.termOutput.scrollTop = dom.termOutput.scrollHeight;
  }
}

// =============================================================================
// 8. Simulated Demo Mode (Physics Generator)
// =============================================================================
function toggleDemoMode() {
  state.demoMode = !state.demoMode;

  if (state.demoMode) {
    dom.demoModeBtn.classList.add('btn-primary');
    dom.demoModeBtn.classList.remove('btn-subtle');
    logTerminal('Demo Physics Activated: Simulating real-time 3D flight dynamics.', 'info');

    let t = 0;
    state.demoTimer = setInterval(() => {
      t += 0.05;
      state.telem.systemState = 'FLIGHT';
      state.telem.armed = true;
      state.telem.roll = Math.sin(t * 1.5) * 18.0;
      state.telem.pitch = Math.cos(t * 1.2) * 12.0;
      state.telem.yawRate = Math.sin(t * 0.8) * 25.0;

      state.telem.ax = -Math.sin((state.telem.pitch * Math.PI) / 180);
      state.telem.ay = Math.sin((state.telem.roll * Math.PI) / 180);
      state.telem.az = Math.cos((state.telem.roll * Math.PI) / 180);

      state.telem.gx = Math.cos(t * 1.5) * 27.0;
      state.telem.gy = -Math.sin(t * 1.2) * 14.0;
      state.telem.gz = state.telem.yawRate;

      // Realistic Quad-X motor mixing responses
      const baseThr = 512;
      const rollCorr = state.telem.roll * 8.0;
      const pitchCorr = state.telem.pitch * 8.0;

      state.telem.m1 = baseThr + rollCorr - pitchCorr;
      state.telem.m2 = baseThr - rollCorr - pitchCorr;
      state.telem.m3 = baseThr - rollCorr + pitchCorr;
      state.telem.m4 = baseThr + rollCorr + pitchCorr;

      state.telem.battV = 3.92 - (t * 0.001);
      state.telem.battPct = Math.max(0, 85 - (t * 0.02));
      state.telem.loopHz = 500.0 + (Math.random() * 1.2 - 0.6);

      updateTelemetryUI();
    }, 50);

  } else {
    clearInterval(state.demoTimer);
    state.demoTimer = null;
    dom.demoModeBtn.classList.remove('btn-primary');
    dom.demoModeBtn.classList.add('btn-subtle');
    logTerminal('Demo Physics Deactivated.', 'warn');
  }
}

// =============================================================================
// 9. Event Listeners & Controls Binding
// =============================================================================
function initEventListeners() {
  // Connect / Disconnect
  dom.connectBtn.addEventListener('click', () => {
    if (state.connected) disconnectSerial();
    else connectSerial();
  });

  // Demo Mode
  dom.demoModeBtn.addEventListener('click', toggleDemoMode);

  // Emergency Stop
  dom.emgStopBtn.addEventListener('click', () => {
    sendSerialCommand('disarm');
    logTerminal('EMERGENCY KILL TRIGGERED! Disarming all motors.', 'error');
  });

  // Reset Camera View
  dom.resetCamBtn.addEventListener('click', resetCamera);

  // Oscilloscope Tabs
  dom.tabAccel.addEventListener('click', () => {
    state.sensorTab = 'accel';
    dom.tabAccel.classList.add('active');
    dom.tabGyro.classList.remove('active');
  });

  dom.tabGyro.addEventListener('click', () => {
    state.sensorTab = 'gyro';
    dom.tabGyro.classList.add('active');
    dom.tabAccel.classList.remove('active');
  });

  // Diagnostics Actions
  dom.btnCalibrate.addEventListener('click', () => {
    sendSerialCommand('calibrate');
  });

  dom.btnArmToggle.addEventListener('click', () => {
    sendSerialCommand('arm');
  });

  dom.btnDisarm.addEventListener('click', () => {
    sendSerialCommand('disarm');
  });

  // Motor Bench Pulse Slider
  dom.motorSlider.addEventListener('input', (e) => {
    dom.motorSliderVal.textContent = `${e.target.value}%`;
  });

  dom.btnPulseMotor.addEventListener('click', () => {
    const motorId = dom.motorSelect.value;
    const dutyPct = dom.motorSlider.value;
    sendSerialCommand(`test_motor ${motorId} ${dutyPct}`);
  });

  // Sim Sliders
  dom.simRollSlider.addEventListener('input', (e) => {
    dom.simRollVal.textContent = `${e.target.value}°`;
  });

  dom.simPitchSlider.addEventListener('input', (e) => {
    dom.simPitchVal.textContent = `${e.target.value}°`;
  });

  dom.btnInjectSim.addEventListener('click', () => {
    const r = dom.simRollSlider.value;
    const p = dom.simPitchSlider.value;
    sendSerialCommand(`sim ${r} ${p}`);
  });

  // Terminal CLI Input
  dom.termForm.addEventListener('submit', (e) => {
    e.preventDefault();
    const cmd = dom.termInput.value.trim();
    if (cmd) {
      sendSerialCommand(cmd);
      dom.termInput.value = '';
    }
  });

  dom.clearTermBtn.addEventListener('click', () => {
    dom.termOutput.innerHTML = '';
  });
}

// =============================================================================
// 10. Application Entrypoint
// =============================================================================
window.addEventListener('DOMContentLoaded', () => {
  initThreeJS();
  initOscilloscope();
  initEventListeners();
  updateTelemetryUI();
  logTerminal('AeroCommand GCS ready. Click "Connect Drone" to link to COM10 via Web Serial.', 'info');
});
