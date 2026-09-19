/**
 * AeroCommand GCS - Aerospace Swarm Mission Ground Control Station
 * Real-time 3D attitude visualizer, sensor oscilloscope, tactical mesh radar,
 * and decentralized ESP-NOW swarm command deck.
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
  viewMode: 'dual', // 'cockpit', 'radar', or 'dual'
  
  // Latest Telemetry Frame for Local Drone (Node #1)
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
  gyroHistory:  { x: [], y: [], z: [] },

  // Swarm State & Decentralized Peer Table
  swarm: {
    nodeId: 1,
    role: 'follower',
    peers: new Map() // key: node_id, value: { id, role, state, battMv, battPct, roll, pitch, armed, lastSeen, relX, relY, targetX, targetY }
  },

  // Tactical Radar Scope State
  radar: {
    rangeMeters: 10,
    sweepAngle: 0.0,
    sweepSpeed: 0.035, // radians per frame
    formation: 'v_formation',
    selectedNodeId: 1,
    hoveredNodeId: null,
    mousePos: { x: 0, y: 0, active: false }
  }
};

// =============================================================================
// 2. DOM Elements Cache
// =============================================================================
const dom = {
  // View Mode Switcher
  btnViewCockpit: document.getElementById('btn-view-cockpit'),
  btnViewDual: document.getElementById('btn-view-dual'),
  btnViewRadar: document.getElementById('btn-view-radar'),
  workspaceContainer: document.getElementById('workspace-container'),

  // Header & Status
  connectBtn: document.getElementById('connect-btn'),
  connectBtnText: document.getElementById('connect-btn-text'),
  demoModeBtn: document.getElementById('demo-mode-btn'),
  emgStopBtn: document.getElementById('emg-stop-btn'),
  statusDot: document.getElementById('system-status-dot'),
  stateLabel: document.getElementById('system-state-label'),
  loopFreqLabel: document.getElementById('loop-freq-label'),
  batteryLabel: document.getElementById('battery-label'),
  
  // HUD
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
  
  // Sensors Oscilloscope
  tabAccel: document.getElementById('tab-accel'),
  tabGyro: document.getElementById('tab-gyro'),
  canvas: document.getElementById('oscilloscope-canvas'),
  valX: document.getElementById('val-x'),
  valY: document.getElementById('val-y'),
  valZ: document.getElementById('val-z'),
  
  // Swarm Tactical Radar Canvas & HUD
  radarCanvas: document.getElementById('swarm-radar-canvas'),
  radarActiveNodes: document.getElementById('radar-active-nodes-num'),
  radarArmedNodes: document.getElementById('radar-armed-nodes-num'),
  radarInspectNode: document.getElementById('radar-inspect-node'),
  footerPeerCount: document.getElementById('footer-peer-count'),
  footerLocalRole: document.getElementById('footer-local-role'),
  footerMeshHealth: document.getElementById('footer-mesh-health'),

  // Swarm Controls & Matrix
  swarmPeersBadge: document.getElementById('swarm-peers-badge'),
  swarmNodeIdInput: document.getElementById('swarm-node-id-input'),
  btnSetNodeId: document.getElementById('btn-set-node-id'),
  swarmRoleSelect: document.getElementById('swarm-role-select'),
  btnSetRole: document.getElementById('btn-set-role'),
  swarmTargetSelect: document.getElementById('swarm-target-select'),
  btnSwarmArm: document.getElementById('btn-swarm-arm'),
  btnSwarmDisarm: document.getElementById('btn-swarm-disarm'),
  btnSwarmCalib: document.getElementById('btn-swarm-calib'),
  btnSwarmTakeoff: document.getElementById('btn-swarm-takeoff'),
  btnSwarmKill: document.getElementById('btn-swarm-kill'),
  swarmFleetGrid: document.getElementById('swarm-fleet-grid'),
  swarmEmptyState: document.getElementById('swarm-empty-state'),

  // Inspector Box
  inspectorBox: document.getElementById('node-inspector-box'),
  inspTitle: document.getElementById('insp-node-title'),
  inspCloseBtn: document.getElementById('insp-close-btn'),
  inspRole: document.getElementById('insp-role'),
  inspState: document.getElementById('insp-state'),
  inspBatt: document.getElementById('insp-batt'),
  inspAtt: document.getElementById('insp-att'),
  inspDist: document.getElementById('insp-dist'),
  inspBearing: document.getElementById('insp-bearing'),

  // Diagnostics Tools
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
  if (!container) return;
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

  // Central Fuselage
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

  // Quad-X Carbon Arms & Motors
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

    // Motor Mount
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

  // Resize Handler
  const handleThreeResize = () => {
    if (!container || !renderer || !camera) return;
    const w = container.clientWidth;
    const h = container.clientHeight;
    if (w === 0 || h === 0) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  };

  window.addEventListener('resize', handleThreeResize);
  animateThreeJS();
}

function updateCameraPosition() {
  if (!camera) return;
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

  if (droneGroup && renderer && scene && camera) {
    // Smooth Euler interpolation towards target orientation
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

    renderer.render(scene, camera);
  }
}

// =============================================================================
// 4. Sensor Waveform Canvas Oscilloscope
// =============================================================================
function initOscilloscope() {
  const canvas = dom.canvas;
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  function resizeCanvas() {
    if (!canvas.parentElement) return;
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
// 5. Tactical Swarm Radar Scope & Mesh Geometry (Canvas 2D)
// =============================================================================
function initSwarmRadar() {
  const canvas = dom.radarCanvas;
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  state.radar.canvas = canvas;
  state.radar.ctx = ctx;

  function resizeRadar() {
    const parent = canvas.parentElement;
    if (!parent) return;
    const rect = parent.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
  }

  resizeRadar();
  window.addEventListener('resize', resizeRadar);

  // Mouse move and click handling on radar blips
  canvas.addEventListener('mousemove', (e) => {
    const rect = canvas.getBoundingClientRect();
    state.radar.mousePos = {
      x: e.clientX - rect.left,
      y: e.clientY - rect.top,
      active: true
    };
  });

  canvas.addEventListener('mouseleave', () => {
    state.radar.mousePos.active = false;
    state.radar.hoveredNodeId = null;
  });

  canvas.addEventListener('click', () => {
    if (state.radar.hoveredNodeId !== null) {
      selectInspectedNode(state.radar.hoveredNodeId);
    }
  });

  // Range Selector Buttons
  document.querySelectorAll('.seg-btn-radar').forEach(btn => {
    btn.addEventListener('click', (e) => {
      document.querySelectorAll('.seg-btn-radar').forEach(b => b.classList.remove('active'));
      e.target.classList.add('active');
      state.radar.rangeMeters = parseFloat(e.target.getAttribute('data-range'));
      logTerminal(`Radar scale switched to ${state.radar.rangeMeters}m range.`, 'info');
    });
  });

  // Formation Selector Pills
  document.querySelectorAll('.formation-pill').forEach(pill => {
    pill.addEventListener('click', (e) => {
      document.querySelectorAll('.formation-pill').forEach(p => p.classList.remove('active'));
      e.target.classList.add('active');
      const form = e.target.getAttribute('data-formation');
      state.radar.formation = form;
      updateFormationTargets();
      logTerminal(`Swarm Formation set to: ${e.target.textContent.toUpperCase()}.`, 'info');
    });
  });

  // Radar Animation Loop
  function drawRadar() {
    requestAnimationFrame(drawRadar);

    const w = canvas.width;
    const h = canvas.height;
    if (w === 0 || h === 0) return;

    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const centerX = w / 2;
    const centerY = h / 2;
    const maxRadius = Math.min(centerX, centerY) - (35 * dpr);

    ctx.clearRect(0, 0, w, h);

    // Increment phosphor sweep angle
    state.radar.sweepAngle += state.radar.sweepSpeed;
    if (state.radar.sweepAngle >= Math.PI * 2) {
      state.radar.sweepAngle -= Math.PI * 2;
    }

    // 1. Outer Compass Ring & Azimuth Heading Ticks
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.25)';
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    ctx.arc(centerX, centerY, maxRadius, 0, Math.PI * 2);
    ctx.stroke();

    // Azimuth markings every 30 degrees
    ctx.font = `${9 * dpr}px 'JetBrains Mono', monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = 'rgba(148, 163, 184, 0.7)';

    const cardinals = { 0: '000° N', 90: '090° E', 180: '180° S', 270: '270° W' };

    for (let deg = 0; deg < 360; deg += 30) {
      const rad = (deg - 90) * (Math.PI / 180);
      const isCard = (deg % 90 === 0);
      const tickLen = isCard ? (12 * dpr) : (6 * dpr);
      
      const x1 = centerX + Math.cos(rad) * maxRadius;
      const y1 = centerY + Math.sin(rad) * maxRadius;
      const x2 = centerX + Math.cos(rad) * (maxRadius - tickLen);
      const y2 = centerY + Math.sin(rad) * (maxRadius - tickLen);

      ctx.strokeStyle = isCard ? 'rgba(56, 189, 248, 0.6)' : 'rgba(255, 255, 255, 0.15)';
      ctx.lineWidth = (isCard ? 1.5 : 1) * dpr;
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();

      // Cardinal / Degree Labels
      const labelDist = maxRadius + (16 * dpr);
      const lx = centerX + Math.cos(rad) * labelDist;
      const ly = centerY + Math.sin(rad) * labelDist;
      ctx.fillStyle = isCard ? '#38bdf8' : 'rgba(100, 116, 139, 0.8)';
      ctx.fillText(cardinals[deg] || `${String(deg).padStart(3, '0')}°`, lx, ly);
    }

    // 2. Concentric Distance Range Rings
    const ringSteps = [0.25, 0.5, 0.75, 1.0];
    ringSteps.forEach(step => {
      const r = maxRadius * step;
      ctx.strokeStyle = (step === 1.0) ? 'rgba(56, 189, 248, 0.25)' : 'rgba(255, 255, 255, 0.08)';
      ctx.lineWidth = 1 * dpr;
      ctx.setLineDash([4 * dpr, 4 * dpr]);
      ctx.beginPath();
      ctx.arc(centerX, centerY, r, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);

      // Range distance labels
      const distM = (state.radar.rangeMeters * step).toFixed(1);
      ctx.fillStyle = 'rgba(100, 116, 139, 0.85)';
      ctx.font = `${8 * dpr}px 'JetBrains Mono', monospace`;
      ctx.textAlign = 'left';
      ctx.fillText(`${distM}m`, centerX + 6 * dpr, centerY - r + (9 * dpr));
    });

    // 3. Crosshair Axes
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
    ctx.lineWidth = 1 * dpr;
    ctx.beginPath();
    ctx.moveTo(centerX - maxRadius, centerY);
    ctx.lineTo(centerX + maxRadius, centerY);
    ctx.moveTo(centerX, centerY - maxRadius);
    ctx.lineTo(centerX, centerY + maxRadius);
    ctx.stroke();

    // 4. Rotating Phosphor Sweep Cone
    const sweepAngle = state.radar.sweepAngle;
    const trailAngle = Math.PI * 0.35; // sector size
    const sweepGrad = ctx.createRadialGradient(centerX, centerY, 0, centerX, centerY, maxRadius);
    sweepGrad.addColorStop(0, 'rgba(16, 185, 129, 0.16)');
    sweepGrad.addColorStop(1, 'rgba(16, 185, 129, 0.01)');

    ctx.save();
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.arc(centerX, centerY, maxRadius, sweepAngle - trailAngle, sweepAngle, false);
    ctx.closePath();
    ctx.fillStyle = sweepGrad;
    ctx.fill();

    // Bright Leading Sweep Ray
    const rayX = centerX + Math.cos(sweepAngle) * maxRadius;
    const rayY = centerY + Math.sin(sweepAngle) * maxRadius;
    ctx.strokeStyle = 'rgba(16, 185, 129, 0.65)';
    ctx.lineWidth = 1.5 * dpr;
    ctx.shadowColor = '#10b981';
    ctx.shadowBlur = 8 * dpr;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(rayX, rayY);
    ctx.stroke();
    ctx.shadowBlur = 0;
    ctx.restore();

    // Helper: Map meter coordinates (x, y) to radar canvas pixel coordinates
    const scaleFactor = maxRadius / state.radar.rangeMeters;
    const toCanvasCoords = (mX, mY) => {
      // mX: positive right, mY: positive forward (North)
      return {
        x: centerX + (mX * scaleFactor),
        y: centerY - (mY * scaleFactor)
      };
    };

    // 5. Formation Geometry Overlay Lines
    if (state.radar.formation !== 'free') {
      drawFormationOverlay(ctx, centerX, centerY, scaleFactor, dpr);
    }

    // 6. Decentralized Swarm Mesh Connection Links
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.2)';
    ctx.lineWidth = 1 * dpr;
    ctx.setLineDash([3 * dpr, 3 * dpr]);

    const allNodes = [
      { id: state.swarm.nodeId, x: 0, y: 0, role: state.swarm.role, state: state.telem.systemState, armed: state.telem.armed, battPct: state.telem.battPct, roll: state.telem.roll, pitch: state.telem.pitch }
    ];

    state.swarm.peers.forEach(peer => {
      allNodes.push({
        id: peer.id,
        x: peer.relX || 0,
        y: peer.relY || 0,
        role: peer.role,
        state: peer.state,
        armed: peer.armed,
        battPct: peer.battPct,
        roll: peer.roll,
        pitch: peer.pitch
      });
    });

    for (let i = 0; i < allNodes.length; i++) {
      for (let j = i + 1; j < allNodes.length; j++) {
        const ptA = toCanvasCoords(allNodes[i].x, allNodes[i].y);
        const ptB = toCanvasCoords(allNodes[j].x, allNodes[j].y);
        ctx.beginPath();
        ctx.moveTo(ptA.x, ptA.y);
        ctx.lineTo(ptB.x, ptB.y);
        ctx.stroke();
      }
    }
    ctx.setLineDash([]);

    // 7. Render Central Local Drone (Node #1)
    const localPt = toCanvasCoords(0, 0);
    drawDroneBlip(ctx, localPt.x, localPt.y, state.swarm.nodeId, state.swarm.role, state.telem.systemState, state.telem.armed, state.telem.battPct, state.telem.roll, state.telem.pitch, dpr, true);

    // 8. Render Peer Drone Blips
    let hovered = null;
    const mouseX = state.radar.mousePos.x * dpr;
    const mouseY = state.radar.mousePos.y * dpr;

    state.swarm.peers.forEach(peer => {
      const pt = toCanvasCoords(peer.relX || 0, peer.relY || 0);
      
      // Check mouse hover hit
      const distToMouse = Math.hypot(pt.x - mouseX, pt.y - mouseY);
      if (state.radar.mousePos.active && distToMouse < 22 * dpr) {
        hovered = peer.id;
      }

      drawDroneBlip(ctx, pt.x, pt.y, peer.id, peer.role, peer.state, peer.armed, peer.battPct, peer.roll, peer.pitch, dpr, false);
    });

    // Check hover hit on local drone
    if (state.radar.mousePos.active && Math.hypot(localPt.x - mouseX, localPt.y - mouseY) < 22 * dpr) {
      hovered = state.swarm.nodeId;
    }

    state.radar.hoveredNodeId = hovered;
    if (hovered !== null) {
      canvas.style.cursor = 'pointer';
    } else {
      canvas.style.cursor = 'crosshair';
    }
  }

  drawRadar();
}

function drawDroneBlip(ctx, x, y, id, role, sysState, armed, battPct, roll, pitch, dpr, isLocal) {
  const isSelected = (state.radar.selectedNodeId === id);
  const isHovered = (state.radar.hoveredNodeId === id);

  // Color mapping
  let color = '#38bdf8'; // cyan default
  if (armed || sysState === 'FLIGHT' || sysState === 'ARMED') color = '#10b981'; // emerald
  if (sysState === 'FAILSAFE' || sysState === 'ERROR') color = '#f43f5e'; // rose
  if (role === 'LEADER' || role === 'LEAD') color = '#f59e0b'; // amber leader

  ctx.save();

  // Pulse halo
  ctx.strokeStyle = color;
  ctx.fillStyle = color;

  if (isSelected) {
    // Selection brackets
    ctx.lineWidth = 1.5 * dpr;
    const bSize = 14 * dpr;
    ctx.strokeStyle = '#38bdf8';
    ctx.beginPath();
    // Top-left
    ctx.moveTo(x - bSize, y - bSize + 5 * dpr);
    ctx.lineTo(x - bSize, y - bSize);
    ctx.lineTo(x - bSize + 5 * dpr, y - bSize);
    // Top-right
    ctx.moveTo(x + bSize - 5 * dpr, y - bSize);
    ctx.lineTo(x + bSize, y - bSize);
    ctx.lineTo(x + bSize, y - bSize + 5 * dpr);
    // Bottom-left
    ctx.moveTo(x - bSize, y + bSize - 5 * dpr);
    ctx.lineTo(x - bSize, y + bSize);
    ctx.lineTo(x - bSize + 5 * dpr, y + bSize);
    // Bottom-right
    ctx.moveTo(x + bSize - 5 * dpr, y + bSize);
    ctx.lineTo(x + bSize, y + bSize);
    ctx.lineTo(x + bSize, y + bSize - 5 * dpr);
    ctx.stroke();
  }

  // Blip Beacon
  ctx.beginPath();
  ctx.arc(x, y, (isHovered ? 6 : 4.5) * dpr, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowColor = color;
  ctx.shadowBlur = 8 * dpr;
  ctx.stroke();
  ctx.shadowBlur = 0;

  // Concentric ring around node
  ctx.lineWidth = 1 * dpr;
  ctx.strokeStyle = color;
  ctx.beginPath();
  ctx.arc(x, y, 9 * dpr, 0, Math.PI * 2);
  ctx.stroke();

  // Label tag
  ctx.font = `600 ${8.5 * dpr}px 'JetBrains Mono', monospace`;
  ctx.fillStyle = '#f8fafc';
  ctx.textAlign = 'left';
  const tag = `NODE #${id} [${role.toUpperCase().slice(0, 4)}]`;
  ctx.fillText(tag, x + (12 * dpr), y - (4 * dpr));

  ctx.font = `${7.5 * dpr}px 'JetBrains Mono', monospace`;
  ctx.fillStyle = 'rgba(148, 163, 184, 0.9)';
  ctx.fillText(`${battPct}% // ${armed ? 'ARMED' : 'DISARMED'}`, x + (12 * dpr), y + (7 * dpr));

  ctx.restore();
}

function drawFormationOverlay(ctx, centerX, centerY, scaleFactor, dpr) {
  ctx.save();
  ctx.strokeStyle = 'rgba(16, 185, 129, 0.35)';
  ctx.lineWidth = 1.2 * dpr;
  ctx.setLineDash([4 * dpr, 4 * dpr]);

  const form = state.radar.formation;
  if (form === 'v_formation') {
    // V-shape lines from leader (0, 0)
    const leftWing = { x: centerX - (3.0 * scaleFactor), y: centerY + (3.0 * scaleFactor) };
    const rightWing = { x: centerX + (3.0 * scaleFactor), y: centerY + (3.0 * scaleFactor) };
    
    ctx.beginPath();
    ctx.moveTo(leftWing.x, leftWing.y);
    ctx.lineTo(centerX, centerY);
    ctx.lineTo(rightWing.x, rightWing.y);
    ctx.stroke();
  } else if (form === 'diamond') {
    const top = { x: centerX, y: centerY - (3.0 * scaleFactor) };
    const bottom = { x: centerX, y: centerY + (3.0 * scaleFactor) };
    const left = { x: centerX - (3.0 * scaleFactor), y: centerY };
    const right = { x: centerX + (3.0 * scaleFactor), y: centerY };

    ctx.beginPath();
    ctx.moveTo(top.x, top.y);
    ctx.lineTo(right.x, right.y);
    ctx.lineTo(bottom.x, bottom.y);
    ctx.lineTo(left.x, left.y);
    ctx.closePath();
    ctx.stroke();
  } else if (form === 'line') {
    ctx.beginPath();
    ctx.moveTo(centerX - (4.5 * scaleFactor), centerY);
    ctx.lineTo(centerX + (4.5 * scaleFactor), centerY);
    ctx.stroke();
  } else if (form === 'circle') {
    ctx.beginPath();
    ctx.arc(centerX, centerY, 3.5 * scaleFactor, 0, Math.PI * 2);
    ctx.stroke();
  }

  ctx.restore();
}

function updateFormationTargets() {
  const form = state.radar.formation;
  const peerList = Array.from(state.swarm.peers.values());

  if (form === 'v_formation') {
    if (peerList[0]) { peerList[0].targetX = -2.8; peerList[0].targetY = -2.4; }
    if (peerList[1]) { peerList[1].targetX = 2.8;  peerList[1].targetY = -2.4; }
    if (peerList[2]) { peerList[2].targetX = 0.0;  peerList[2].targetY = -4.6; }
  } else if (form === 'diamond') {
    if (peerList[0]) { peerList[0].targetX = -2.8; peerList[0].targetY = 0.0; }
    if (peerList[1]) { peerList[1].targetX = 2.8;  peerList[1].targetY = 0.0; }
    if (peerList[2]) { peerList[2].targetX = 0.0;  peerList[2].targetY = -3.2; }
  } else if (form === 'line') {
    if (peerList[0]) { peerList[0].targetX = -3.2; peerList[0].targetY = 0.0; }
    if (peerList[1]) { peerList[1].targetX = 3.2;  peerList[1].targetY = 0.0; }
    if (peerList[2]) { peerList[2].targetX = 6.4;  peerList[2].targetY = 0.0; }
  } else if (form === 'circle') {
    const r = 3.5;
    const n = peerList.length + 1;
    peerList.forEach((p, idx) => {
      const ang = ((idx + 1) / n) * Math.PI * 2 - Math.PI / 2;
      p.targetX = Math.cos(ang) * r;
      p.targetY = Math.sin(ang) * r;
    });
  }
}

function selectInspectedNode(nodeId) {
  state.radar.selectedNodeId = nodeId;
  if (dom.inspectorBox) dom.inspectorBox.style.display = 'block';

  let nodeInfo = null;
  if (nodeId === state.swarm.nodeId) {
    nodeInfo = {
      id: state.swarm.nodeId,
      role: state.swarm.role,
      state: state.telem.systemState,
      battV: state.telem.battV.toFixed(2),
      battPct: state.telem.battPct,
      roll: state.telem.roll,
      pitch: state.telem.pitch,
      dist: '0.0 m (Local)',
      bearing: '000° REF'
    };
    if (dom.radarInspectNode) {
      dom.radarInspectNode.textContent = `CENTER: NODE #${nodeId} (LOCAL FC)`;
    }
  } else {
    const peer = state.swarm.peers.get(nodeId);
    if (peer) {
      const dist = Math.hypot(peer.relX || 0, peer.relY || 0).toFixed(1);
      let angle = (Math.atan2(peer.relX || 0, peer.relY || 0) * 180 / Math.PI);
      if (angle < 0) angle += 360;
      nodeInfo = {
        id: peer.id,
        role: peer.role,
        state: peer.state,
        battV: (peer.battMv / 1000).toFixed(2),
        battPct: peer.battPct,
        roll: peer.roll,
        pitch: peer.pitch,
        dist: `${dist} m`,
        bearing: `${Math.round(angle).toString().padStart(3, '0')}°`
      };
      if (dom.radarInspectNode) {
        dom.radarInspectNode.textContent = `TARGET: NODE #${nodeId} // ${dist}m @ ${Math.round(angle)}°`;
      }
    }
  }

  if (nodeInfo) {
    dom.inspTitle.textContent = `Node #${nodeInfo.id} (${nodeInfo.role.toUpperCase()})`;
    dom.inspRole.textContent = nodeInfo.role.toUpperCase();
    dom.inspState.textContent = nodeInfo.state;
    dom.inspBatt.textContent = `${nodeInfo.battV}V (${nodeInfo.battPct}%)`;
    dom.inspAtt.textContent = `R: ${nodeInfo.roll >= 0 ? '+' : ''}${nodeInfo.roll.toFixed(1)}° P: ${nodeInfo.pitch >= 0 ? '+' : ''}${nodeInfo.pitch.toFixed(1)}°`;
    dom.inspDist.textContent = nodeInfo.dist;
    dom.inspBearing.textContent = nodeInfo.bearing;
  }
}

// =============================================================================
// 6. View Mode Switcher
// =============================================================================
function initViewSwitcher() {
  const switchView = (mode) => {
    state.viewMode = mode;
    document.body.className = `view-${mode}`;

    [dom.btnViewCockpit, dom.btnViewDual, dom.btnViewRadar].forEach(btn => {
      if (!btn) return;
      if (btn.getAttribute('data-view') === mode) btn.classList.add('active');
      else btn.classList.remove('active');
    });

    // Force redraw on canvases and Three.js
    setTimeout(() => {
      window.dispatchEvent(new Event('resize'));
    }, 150);

    logTerminal(`View Mode switched to: ${mode.toUpperCase()} MISSION DISPLAY.`, 'info');
  };

  if (dom.btnViewCockpit) dom.btnViewCockpit.addEventListener('click', () => switchView('cockpit'));
  if (dom.btnViewDual)    dom.btnViewDual.addEventListener('click', () => switchView('dual'));
  if (dom.btnViewRadar)   dom.btnViewRadar.addEventListener('click', () => switchView('radar'));

  if (dom.inspCloseBtn) {
    dom.inspCloseBtn.addEventListener('click', () => {
      if (dom.inspectorBox) dom.inspectorBox.style.display = 'none';
      state.radar.selectedNodeId = null;
      if (dom.radarInspectNode) dom.radarInspectNode.textContent = 'CENTER: NODE #1 (LOCAL)';
    });
  }
}

// =============================================================================
// 7. Telemetry Processing & UI Updates
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
  targetRotation.z = -(t.roll * Math.PI) / 180;
  targetRotation.y = 0;

  dom.hudRoll.textContent = `${t.roll >= 0 ? '+' : ''}${t.roll.toFixed(2)}°`;
  dom.hudPitch.textContent = `${t.pitch >= 0 ? '+' : ''}${t.pitch.toFixed(2)}°`;
  dom.hudYawRate.textContent = `${t.yawRate >= 0 ? '+' : ''}${t.yawRate.toFixed(2)}°/s`;

  // 3. Quad-X Motor Gauges
  const circumference = 144.51;
  const updateMotor = (gauge, dutyEl, pctEl, val) => {
    if (!gauge || !dutyEl || !pctEl) return;
    const clamped = Math.min(1023, Math.max(0, val));
    const offset = circumference * (1 - clamped / 1023);
    gauge.style.strokeDashoffset = offset;

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

// Parse Telemetry packets ($TELEM, $SWARM, $PEER)
function parseTelemetryLine(line) {
  if (line.startsWith('$SWARM,')) {
    const p = line.trim().split(',');
    if (p.length >= 4) {
      state.swarm.nodeId = parseInt(p[1], 10);
      const roleCode = parseInt(p[2], 10);
      state.swarm.role = (roleCode === 1) ? 'leader' : (roleCode === 2 ? 'follower' : 'standalone');
      if (dom.swarmNodeIdInput && document.activeElement !== dom.swarmNodeIdInput) {
        dom.swarmNodeIdInput.value = state.swarm.nodeId;
      }
      if (dom.swarmRoleSelect && document.activeElement !== dom.swarmRoleSelect) {
        dom.swarmRoleSelect.value = state.swarm.role;
      }
      if (dom.footerLocalRole) {
        dom.footerLocalRole.textContent = state.swarm.role.toUpperCase();
      }
    }
    return true;
  }

  if (line.startsWith('$PEER,')) {
    const p = line.trim().split(',');
    if (p.length >= 9) {
      const peerId = parseInt(p[1], 10);
      const roleNum = parseInt(p[2], 10);
      const peerRole = (roleNum === 1) ? 'LEAD' : (roleNum === 2 ? 'FOLL' : 'NODE');
      const stateNum = parseInt(p[3], 10);
      const battMv = parseInt(p[4], 10);
      const battPct = parseInt(p[5], 10);
      const roll = parseFloat(p[6]);
      const pitch = parseFloat(p[7]);
      const armed = (p[8] === '1');

      let peer = state.swarm.peers.get(peerId);
      if (!peer) {
        peer = { id: peerId, relX: 0, relY: 0, targetX: 0, targetY: 0 };
        state.swarm.peers.set(peerId, peer);
      }
      peer.role = peerRole;
      peer.state = stateNum === 4 ? 'FLIGHT' : (stateNum === 3 ? 'ARMED' : 'DISARMED');
      peer.battMv = battMv;
      peer.battPct = battPct;
      peer.roll = roll;
      peer.pitch = pitch;
      peer.armed = armed;
      peer.lastSeen = Date.now();

      renderSwarmFleet();
    }
    return true;
  }

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

function renderSwarmFleet() {
  if (!dom.swarmFleetGrid) return;
  const now = Date.now();

  // Purge stale peers > 4.5 seconds
  for (const [id, peer] of state.swarm.peers.entries()) {
    if (now - peer.lastSeen > 4500) {
      state.swarm.peers.delete(id);
      const oldCard = document.getElementById(`swarm-peer-${id}`);
      if (oldCard) oldCard.remove();
    }
  }

  const count = state.swarm.peers.size;
  if (dom.swarmPeersBadge) {
    dom.swarmPeersBadge.textContent = `${count} PEER${count === 1 ? '' : 'S'} DETECTED`;
    dom.swarmPeersBadge.style.color = count > 0 ? 'var(--accent-emerald)' : 'var(--text-muted)';
    dom.swarmPeersBadge.style.background = count > 0 ? 'rgba(16, 185, 129, 0.15)' : 'rgba(255, 255, 255, 0.05)';
  }

  // Airspace Header Counters
  let armedCount = state.telem.armed ? 1 : 0;
  state.swarm.peers.forEach(p => { if (p.armed) armedCount++; });

  if (dom.radarActiveNodes) dom.radarActiveNodes.textContent = (count + 1);
  if (dom.radarArmedNodes)  dom.radarArmedNodes.textContent = armedCount;
  if (dom.footerPeerCount)  dom.footerPeerCount.textContent = `${count} Neighbor Nodes`;

  const bentoPeerStat = document.getElementById('bento-peer-stat');
  if (bentoPeerStat) {
    bentoPeerStat.textContent = `${count} Peer${count === 1 ? '' : 's'} Active`;
    bentoPeerStat.style.color = count > 0 ? 'var(--accent-emerald)' : '#a78bfa';
  }

  if (count === 0) {
    if (dom.swarmEmptyState) dom.swarmEmptyState.style.display = 'flex';
    return;
  }

  if (dom.swarmEmptyState) dom.swarmEmptyState.style.display = 'none';

  state.swarm.peers.forEach(peer => {
    let card = document.getElementById(`swarm-peer-${peer.id}`);
    if (!card) {
      card = document.createElement('div');
      card.id = `swarm-peer-${peer.id}`;
      dom.swarmFleetGrid.appendChild(card);
    }

    const isArmed = peer.armed;
    const isFlight = (peer.state === 'FLIGHT');
    card.className = `swarm-peer-card ${peer.role.toLowerCase()} ${isArmed ? 'armed' : ''}`;
    const battV = (peer.battMv / 1000).toFixed(2);

    card.innerHTML = `
      <div class="swarm-peer-header">
        <span class="swarm-peer-id">
          <span class="swarm-peer-dot" style="background: ${isArmed ? 'var(--accent-emerald)' : (isFlight ? 'var(--accent-cyan)' : 'var(--accent-rose)')};"></span>
          Node #${peer.id}
        </span>
        <span class="swarm-peer-badge ${peer.role.toLowerCase()}">${peer.role}</span>
      </div>
      <div class="swarm-peer-stat-row">
        <span>State</span>
        <span class="swarm-peer-stat-val" style="color: ${isArmed ? 'var(--accent-emerald)' : 'var(--text-muted)'};">${peer.state}</span>
      </div>
      <div class="swarm-peer-stat-row">
        <span>Battery</span>
        <span class="swarm-peer-stat-val">${battV}V (${peer.battPct}%)</span>
      </div>
      <div class="swarm-batt-bar-bg">
        <div class="swarm-batt-bar-fg" style="width: ${peer.battPct}%; background: ${peer.battPct > 25 ? 'var(--accent-cyan)' : 'var(--accent-rose)'};"></div>
      </div>
      <div class="swarm-peer-stat-row" style="margin-top: 3px;">
        <span>R: ${peer.roll >= 0 ? '+' : ''}${peer.roll.toFixed(1)}°</span>
        <span>P: ${peer.pitch >= 0 ? '+' : ''}${peer.pitch.toFixed(1)}°</span>
      </div>
    `;

    card.onclick = () => selectInspectedNode(peer.id);
  });
}

// =============================================================================
// 8. Web Serial API Driver
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

    if (state.demoMode) toggleDemoMode();
    readSerialLoop();

    setTimeout(() => {
      sendSerialCommand('stream on');
    }, 350);

  } catch (err) {
    logTerminal(`Connection failed: ${err.message}`, 'error');
  }
}

async function disconnectSerial() {
  try {
    if (state.reader) {
      await state.reader.cancel();
      state.reader = null;
    }
    if (state.port) {
      await state.port.close();
      state.port = null;
    }
  } catch (err) {
    console.warn('Error during disconnect:', err);
  } finally {
    state.connected = false;
    dom.connectBtnText.textContent = 'Connect Drone';
    dom.connectBtn.classList.add('btn-primary');
    dom.connectBtn.classList.remove('btn-subtle');
    dom.stateLabel.textContent = 'DISCONNECTED';
    dom.statusDot.className = 'pulse-dot';
    logTerminal('Disconnected from drone.', 'warn');
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
        lineBuffer = lines.pop(); // Keep last partial line

        for (const raw of lines) {
          const clean = raw.trim();
          if (!clean) continue;

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
// 9. Interactive Terminal Logger
// =============================================================================
function logTerminal(msg, type = '') {
  if (!dom.termOutput) return;
  const line = document.createElement('div');
  line.className = `log-line ${type}`.trim();
  line.textContent = msg;
  dom.termOutput.appendChild(line);

  if (dom.termOutput.children.length > 250) {
    dom.termOutput.removeChild(dom.termOutput.firstChild);
  }

  if (dom.autoscrollChk && dom.autoscrollChk.checked) {
    dom.termOutput.scrollTop = dom.termOutput.scrollHeight;
  }
}

// =============================================================================
// 10. Simulated Demo Mode (Multi-Drone Physics & Formation Dynamics)
// =============================================================================
function toggleDemoMode() {
  state.demoMode = !state.demoMode;

  if (state.demoMode) {
    dom.demoModeBtn.classList.add('btn-primary');
    dom.demoModeBtn.classList.remove('btn-subtle');
    logTerminal('Demo Physics Activated: Simulating real-time 3D flight & 4-drone swarm formation.', 'info');

    let t = 0;
    state.demoTimer = setInterval(() => {
      t += 0.05;
      state.telem.systemState = 'FLIGHT';
      state.telem.armed = true;
      state.telem.roll = Math.sin(t * 1.5) * 16.0;
      state.telem.pitch = Math.cos(t * 1.2) * 11.0;
      state.telem.yawRate = Math.sin(t * 0.8) * 22.0;

      state.telem.ax = -Math.sin((state.telem.pitch * Math.PI) / 180);
      state.telem.ay = Math.sin((state.telem.roll * Math.PI) / 180);
      state.telem.az = Math.cos((state.telem.roll * Math.PI) / 180);

      state.telem.gx = Math.cos(t * 1.5) * 24.0;
      state.telem.gy = -Math.sin(t * 1.2) * 13.0;
      state.telem.gz = state.telem.yawRate;

      // Realistic Quad-X motor mixing responses
      const baseThr = 512;
      const rollCorr = state.telem.roll * 7.5;
      const pitchCorr = state.telem.pitch * 7.5;

      state.telem.m1 = baseThr + rollCorr - pitchCorr;
      state.telem.m2 = baseThr - rollCorr - pitchCorr;
      state.telem.m3 = baseThr - rollCorr + pitchCorr;
      state.telem.m4 = baseThr + rollCorr + pitchCorr;

      state.telem.battV = 3.92 - (t * 0.0008);
      state.telem.battPct = Math.max(0, 88 - (t * 0.015));
      state.telem.loopHz = 500.0 + (Math.random() * 1.2 - 0.6);

      // Smoothly animate swarm peer positions towards target formation slots
      state.swarm.peers.forEach(peer => {
        const driftX = Math.sin(t * 1.1 + peer.id) * 0.15;
        const driftY = Math.cos(t * 0.9 + peer.id) * 0.15;
        peer.relX += ((peer.targetX || 0) + driftX - (peer.relX || 0)) * 0.08;
        peer.relY += ((peer.targetY || 0) + driftY - (peer.relY || 0)) * 0.08;
        peer.lastSeen = Date.now();
      });

      renderSwarmFleet();
      updateTelemetryUI();
    }, 50);

    // Seed 3 Swarm Peer Nodes for Demo Mode
    state.swarm.peers.set(2, {
      id: 2,
      role: 'FOLL',
      state: 'FLIGHT',
      battMv: 3880,
      battPct: 84,
      roll: 2.1,
      pitch: -1.2,
      armed: true,
      relX: -2.8,
      relY: -2.4,
      targetX: -2.8,
      targetY: -2.4,
      lastSeen: Date.now()
    });

    state.swarm.peers.set(3, {
      id: 3,
      role: 'FOLL',
      state: 'FLIGHT',
      battMv: 3790,
      battPct: 76,
      roll: -1.8,
      pitch: 0.9,
      armed: true,
      relX: 2.8,
      relY: -2.4,
      targetX: 2.8,
      targetY: -2.4,
      lastSeen: Date.now()
    });

    state.swarm.peers.set(4, {
      id: 4,
      role: 'FOLL',
      state: 'FLIGHT',
      battMv: 3950,
      battPct: 91,
      roll: 0.5,
      pitch: -0.4,
      armed: true,
      relX: 0.0,
      relY: -4.6,
      targetX: 0.0,
      targetY: -4.6,
      lastSeen: Date.now()
    });

    updateFormationTargets();
    renderSwarmFleet();

  } else {
    clearInterval(state.demoTimer);
    state.demoTimer = null;
    dom.demoModeBtn.classList.remove('btn-primary');
    dom.demoModeBtn.classList.add('btn-subtle');
    state.swarm.peers.clear();
    renderSwarmFleet();
    logTerminal('Demo Physics Deactivated.', 'warn');
  }
}

// =============================================================================
// 11. Event Listeners & Controls Binding
// =============================================================================
function initEventListeners() {
  // Connect / Disconnect
  dom.connectBtn.addEventListener('click', () => {
    if (state.connected) disconnectSerial();
    else connectSerial();
  });

  // Demo Mode
  dom.demoModeBtn.addEventListener('click', toggleDemoMode);

  // Emergency Stop Local
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

  // Swarm Command Deck Event Listeners
  if (dom.btnSetNodeId) {
    dom.btnSetNodeId.addEventListener('click', () => {
      const id = parseInt(dom.swarmNodeIdInput.value, 10);
      if (id >= 1 && id <= 254) {
        sendSerialCommand(`node ${id}`);
        state.swarm.nodeId = id;
        logTerminal(`Requested local Node ID change to ${id}.`, 'info');
      } else {
        alert('Node ID must be between 1 and 254.');
      }
    });
  }

  if (dom.btnSetRole) {
    dom.btnSetRole.addEventListener('click', () => {
      const role = dom.swarmRoleSelect.value;
      sendSerialCommand(`role ${role}`);
      state.swarm.role = role;
      if (dom.footerLocalRole) dom.footerLocalRole.textContent = role.toUpperCase();
      logTerminal(`Applied Swarm Role: ${role.toUpperCase()}.`, 'info');
    });
  }

  if (dom.btnSwarmArm) {
    dom.btnSwarmArm.addEventListener('click', () => {
      sendSerialCommand('swarm_cmd arm');
      logTerminal('Broadcasted SWARM ARM command.', 'info');
      if (state.demoMode) {
        state.swarm.peers.forEach(p => { p.armed = true; p.state = 'FLIGHT'; });
        state.telem.armed = true;
        renderSwarmFleet();
      }
    });
  }

  if (dom.btnSwarmDisarm) {
    dom.btnSwarmDisarm.addEventListener('click', () => {
      sendSerialCommand('swarm_cmd disarm');
      logTerminal('Broadcasted SWARM DISARM command.', 'warn');
      if (state.demoMode) {
        state.swarm.peers.forEach(p => { p.armed = false; p.state = 'DISARMED'; });
        state.telem.armed = false;
        renderSwarmFleet();
      }
    });
  }

  if (dom.btnSwarmCalib) {
    dom.btnSwarmCalib.addEventListener('click', () => {
      sendSerialCommand('swarm_cmd calib');
      logTerminal('Broadcasted SWARM CALIBRATE command.', 'info');
    });
  }

  if (dom.btnSwarmTakeoff) {
    dom.btnSwarmTakeoff.addEventListener('click', () => {
      sendSerialCommand('swarm_cmd arm');
      logTerminal('Broadcasted SYNCHRONIZED SWARM TAKEOFF SEQUENCE.', 'info');
      if (state.demoMode) {
        state.swarm.peers.forEach(p => { p.armed = true; p.state = 'FLIGHT'; });
        state.telem.armed = true;
        renderSwarmFleet();
      }
    });
  }

  if (dom.btnSwarmKill) {
    dom.btnSwarmKill.addEventListener('click', () => {
      sendSerialCommand('swarm_cmd stop');
      logTerminal('EMERGENCY KILL BROADCASTED TO ENTIRE SWARM!', 'error');
      if (state.demoMode) {
        state.swarm.peers.forEach(p => { p.armed = false; p.state = 'DISARMED'; });
        state.telem.armed = false;
        renderSwarmFleet();
      }
    });
  }

  // Diagnostics Actions
  dom.btnCalibrate.addEventListener('click', () => sendSerialCommand('calibrate'));
  dom.btnArmToggle.addEventListener('click', () => sendSerialCommand('arm'));
  dom.btnDisarm.addEventListener('click', () => sendSerialCommand('disarm'));

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
// 12. Application Entrypoint
// =============================================================================
window.addEventListener('DOMContentLoaded', () => {
  initThreeJS();
  initOscilloscope();
  initSwarmRadar();
  initViewSwitcher();
  initEventListeners();
  updateTelemetryUI();
  logTerminal('AeroCommand GCS initialized. Click Connect Drone to select COM port.', 'info');
});
