(function(){
  "use strict";

  if(typeof CANNON === 'undefined'){
    document.body.innerHTML = '<div style="color:#ff5d5d;font-family:monospace;padding:40px;font-size:14px;">Physics engine (cannon.js) failed to load from the CDN — check your internet connection and reload. This simulator needs it for real rigid-body flight dynamics.</div>';
    return;
  }

  /* ============================================================
     STATE
  ============================================================ */
  const state = {
    phase: "idle",
    armed: false,
    throttle: 0,          // 0..1, average commanded motor thrust fraction
    motorRpm: 0,           // visual proxy derived from commanded thrust
    altitude: 0,            // meters, height above the launch pad (AGL)
    vspeed: 0,               // m/s, real physics-body vertical velocity
    targetAlt: 4.2,
    t: 0,
    startTime: performance.now(),
    tilt: 0,                 // real bank angle (deg) derived from body orientation, drives the 2D sprite rotation
    attitudeRoll: 0,          // deg, captured for the incident report
    attitudePitch: 0,
    crashed: false,
    failureMode: null,
    failureStartAlt: 0,
    failureStartTime: 0,
    strikeApplied: false,

    // autonomous recovery
    recoverStableSince: null,
    recoveryStartTime: null,
    recoveryCompleting: false,
    recoveryAngleErrDeg: 0,
    recoveryOmega: 0,
    recoveryTorqueMag: 0,
    crashRecoveryDelayTimer: null,
    recoveryFollowupTimer: null,
    reportPending: false,
    reportAvailable: false,

    // diagnostic spin sweep — a deliberate, visible pre-correction phase
    // (real torque, real revolutions) that runs before the corrective
    // controller commits — see runDiagnosticSpin()
    recoveryPhase: null,          // 'spin' | 'correct' | null
    recoverySpinAccumRad: 0,
    recoverySpinStartTime: 0,

    // smooth lock-in glide (torque hands off to a short eased animation
    // instead of teleporting) — see runLockAnimation()
    lockAnimDone: false,
    lockStartTime: 0,
    lockFromQuat: null,
    lockFromCageSpin: 0.55,

    // real-time cage spin angle, driven by the physics body's actual
    // orientation while tumbling (failure/crashed/recovering) — locked to
    // a fixed angle otherwise so it doesn't drift during normal flight.
    // See syncStateFromBody() and completeRecovery().
    cageSpin: 0.55,

    // full-flight mission timeline for the incident report: every phase
    // transition from takeoff through failure, crash, recovery, relaunch,
    // and landing, with timestamps — see logMission()
    missionLog: [],

    // derived physics telemetry (recomputed every tick, for HUD + log)
    lastThrustN: 0,
    lastDragN: 0,
    lastTorqueMag: 0,
    lastAngAccel: 0,
    lastTWR: 0,

    // real telemetry samples recorded through the flight (~10Hz), spanning
    // the same ARM-to-now window as missionLog. Feeds the genuine MAVLink
    // .tlog export and the flight-history replay feature — see
    // sampleTelemetry(), buildMavlinkTlog(), and REPLAY below.
    telemetrySamples: [],
    lastTelemetrySampleT: -999,

    // scrub-bar replay of a past flight loaded from local flight history —
    // drives the same render()/updateHud() as live flight but from
    // recorded samples instead of physics. See REPLAY below.
    replay: {
      active: false,
      flight: null,
      index: 0,
      playing: false,
      speed: 1,
    },
  };

  const el = {
    console: document.getElementById('console'),
    phaseBanner: document.getElementById('phase-banner'),
    hudMode: document.getElementById('hud-mode'),
    hudArmed: document.getElementById('hud-armed'),
    hudAlt: document.getElementById('hud-alt'),
    hudVspd: document.getElementById('hud-vspd'),
    hudThr: document.getElementById('hud-thr'),
    clock: document.getElementById('clock'),
    mAlt: document.getElementById('mAlt'),
    mAltBar: document.getElementById('mAltBar'),
    mThr: document.getElementById('mThr'),
    mThrBar: document.getElementById('mThrBar'),
    mRpm: document.getElementById('mRpm'),
    mRpmBar: document.getElementById('mRpmBar'),
    btnArm: document.getElementById('btnArm'),
    btnTakeoff: document.getElementById('btnTakeoff'),
    btnLand: document.getElementById('btnLand'),
    btnFault: document.getElementById('btnFault'),
    failureSelect: document.getElementById('failureSelect'),
    btnReset: document.getElementById('btnReset'),
    btnKiosk: document.getElementById('btnKiosk'),
    btnCrashAck: document.getElementById('btnCrashAck'),
    btnDownloadReport: document.getElementById('btnDownloadReport'),
    btnDownloadTlog: document.getElementById('btnDownloadTlog'),
    btnHistory: document.getElementById('btnHistory'),
    historyOverlay: document.getElementById('history-overlay'),
    historyList: document.getElementById('historyList'),
    historyCount: document.getElementById('historyCount'),
    btnHistoryClose: document.getElementById('btnHistoryClose'),
    btnHistoryClearAll: document.getElementById('btnHistoryClearAll'),
    replayBar: document.getElementById('replay-bar'),
    replayScrub: document.getElementById('replayScrub'),
    replayClock: document.getElementById('replayClock'),
    btnReplayPlay: document.getElementById('btnReplayPlay'),
    btnReplayExit: document.getElementById('btnReplayExit'),
    btnViewReport: document.getElementById('btnViewReport'),
    crashOverlay: document.getElementById('crash-overlay'),
  };

  const FAIL_MODES = [
    { name:'ESC desync — Motor 2', log1:'ESC desync detected on motor 2 — PWM feedback lost', diag:'motor_2/esc', msg:'desync',
      cause:'The electronic speed controller for motor 2 lost synchronization, producing asymmetric thrust and an uncontrolled roll.' },
    { name:'LiPo cell voltage collapse', log1:'battery cell 3 voltage collapse — under-voltage lockout', diag:'power/battery', msg:'cell_undervoltage',
      cause:'A battery cell voltage dropped below the safe threshold, triggering an automatic power cutback mid-flight.' },
    { name:'Propeller delamination — Motor 4', log1:'vibration spike on motor 4 — propeller structural failure suspected', diag:'airframe/prop_4', msg:'delamination',
      cause:'A propeller blade separated from its hub, eliminating thrust from motor 4 and inducing a violent spin.' },
    { name:'Flight controller IMU fault', log1:'IMU0 gyro saturation — sensor fusion rejected', diag:'fcu/imu0', msg:'gyro_saturation',
      cause:'The primary IMU produced inconsistent readings, causing the attitude estimator to lose lock on true orientation.' },
    { name:'GPS lock loss — uncontrolled descent', log1:'GPS 3D fix lost — position hold unavailable', diag:'nav/gps', msg:'fix_lost',
      cause:"Loss of GPS lock during position-hold removed the drone's horizontal reference, and residual heading drift went uncorrected." },
    { name:'RC link loss — failsafe rejected', log1:'RC link timeout (2.1s) — entering FAILSAFE', diag:'radio/rc_link', msg:'link_timeout',
      cause:'The control link to the transmitter was lost for longer than the failsafe timeout, and the configured return-to-launch action failed to engage — the flight controller froze its outputs open-loop.' },
    { name:'Motor bearing seizure — Motor 1', log1:'motor 1 stall detected — back-EMF signature abnormal', diag:'motor_1/bearing', msg:'bearing_seizure',
      cause:'A worn motor bearing seized under load, abruptly stopping motor 1 and removing a quarter of total lift.' },
    { name:'Flight controller firmware watchdog reset', log1:'FCU watchdog timeout — firmware forced reboot', diag:'fcu/firmware', msg:'watchdog_reset',
      cause:'The flight controller firmware hung and was force-rebooted by its watchdog, briefly zeroing all motor outputs mid-flight.' },
    { name:'Wind gust exceeding control authority', log1:'commanded attitude vs actual attitude divergence > 35°', diag:'env/wind', msg:'gust_exceedance',
      cause:"A sudden wind gust exceeded the airframe's maximum control authority, overpowering the attitude controller." },
    { name:'Foreign object strike (bird / debris)', log1:'sudden asymmetric drag + motor 3 overcurrent', diag:'airframe/impact', msg:'foreign_object_strike',
      cause:'An in-flight collision with a foreign object imparted a sudden impulse and damaged a propeller, destabilizing the airframe.' },
    { name:'Carbon arm structural failure — Arm 3', log1:'motor 3 mount displacement — structural integrity lost', diag:'airframe/arm_3', msg:'structural_failure',
      cause:'A carbon-fiber arm fractured under load, dropping a motor out of alignment and removing effective thrust on that side.' },
    { name:'Compass / magnetometer interference', log1:'compass variance exceeds 45° — yaw estimate rejected', diag:'nav/compass', msg:'mag_interference',
      cause:'Nearby magnetic interference corrupted the compass heading, causing the flight controller to command a hard, incorrect yaw correction.' },
    { name:'Battery thermal runaway', log1:'battery pack temperature 87°C — thermal event in progress', diag:'power/battery_temp', msg:'thermal_runaway',
      cause:'Internal cell damage triggered a thermal runaway event, causing a rapidly worsening power loss across all four motors.' },
    { name:'State estimator crash (EKF NaN)', log1:'EKF covariance matrix non-finite — estimator reset', diag:'ekf/state_estimator', msg:'nan_covariance',
      cause:'The state estimator produced a non-finite (NaN) covariance value and reset, leaving the flight controller with no attitude correction at all.' },
    { name:'Operator input error — abrupt throttle cut', log1:'commanded throttle dropped 0.9→0.1 in 40ms', diag:'input/rc_throttle', msg:'abrupt_input',
      cause:'An abrupt manual throttle reduction removed lift faster than the airframe could recover from, at an altitude too low to correct.' },
  ];

  /* ============================================================
     FAKE ROS CONSOLE
  ============================================================ */
  function nowStamp(){
    const s = (performance.now() - state.startTime)/1000;
    return s.toFixed(3);
  }

  function log(html){
    const line = document.createElement('div');
    line.className = 'ln';
    line.innerHTML = `<span class="t">[${nowStamp()}]</span> ${html}`;
    el.console.appendChild(line);
    while(el.console.children.length > 260){
      el.console.removeChild(el.console.firstChild);
    }
    el.console.scrollTop = el.console.scrollHeight;
  }

  function logTopic(topic, body){
    log(`<span class="topic">${topic}</span> ${body}`);
  }

  // Records one entry in the full-flight mission timeline (takeoff -> ...
  // -> landing) shown in the incident report once it becomes available.
  function logMission(text){
    state.missionLog.push({ t: state.t, clock: el.clock.textContent, text });
  }

  // Samples real flight telemetry (~10Hz) into state.telemetrySamples —
  // shares the ARM-to-now window as missionLog (both reset together on a
  // fresh manual ARM). This buffer is the single source of truth behind
  // both the MAVLink .tlog export and the flight-history replay feature,
  // so what gets exported/replayed is exactly what the sim computed, not
  // a re-derived approximation.
  function sampleTelemetry(){
    if(state.phase === 'idle' || state.replay.active) return;
    if(state.t - state.lastTelemetrySampleT < 0.1) return;
    state.lastTelemetrySampleT = state.t;
    const yaw = Math.atan2(
      2*(droneBody.quaternion.w*droneBody.quaternion.y + droneBody.quaternion.x*droneBody.quaternion.z),
      1 - 2*(droneBody.quaternion.y*droneBody.quaternion.y + droneBody.quaternion.z*droneBody.quaternion.z)
    );
    state.telemetrySamples.push({
      t: state.t,
      clock: el.clock.textContent,
      alt: state.altitude,
      vspeed: state.vspeed,
      posX: droneBody.position.x,
      posZ: droneBody.position.z,
      rollDeg: state.attitudeRoll,
      pitchDeg: state.attitudePitch,
      yaw: yaw,
      rollRate: droneBody.angularVelocity.z,
      pitchRate: droneBody.angularVelocity.x,
      yawRate: droneBody.angularVelocity.y,
      tilt: state.tilt,
      cageSpin: state.cageSpin,
      motorRpm: state.motorRpm,
      throttle: state.throttle,
      armed: state.armed,
      phase: state.phase,
    });
    // cap the buffer (≈33 minutes at 10Hz) so a very long idle session can't
    // grow this unbounded
    if(state.telemetrySamples.length > 20000) state.telemetrySamples.shift();
  }

  function activateChip(id){
    const c = document.getElementById(id);
    if(c) c.classList.add('live');
  }
  function deactivateChip(id){
    const c = document.getElementById(id);
    if(c) c.classList.remove('live');
  }

  function bootSequence(){
    const boot = [
      ['<span class="ok">[INFO]</span> Started roscore. master URI: <span class="field">http://localhost:11311</span>', 0],
      ['<span class="ok">[INFO]</span> Physics engine <span class="field">cannon.js</span> initialized — 1.2kg rigid-body airframe, 4-motor thrust/torque mixer, PID altitude+attitude hold', 150],
      [`<span class="ok">[INFO]</span> Derived airframe model — I_roll/I_pitch: <span class="val">${MOI_ROLL.toFixed(4)}</span> kg·m² &nbsp; I_yaw: <span class="val">${MOI_YAW.toFixed(4)}</span> kg·m² &nbsp; max T/W: <span class="val">${MAX_TWR.toFixed(2)}</span>`, 260],
      ['<span class="ok">[INFO]</span> Aerodynamic drag model loaded — <span class="field">ρ 1.225 kg/m³ · Cd 0.9 · A 0.045m²</span>', 300],
      ['<span class="ok">[INFO]</span> <span class="node">/mavros</span> connected on <span class="field">udp://:14540@</span>', 320, ()=>activateChip('chip-mavros')],
      ['<span class="ok">[INFO]</span> <span class="node">/ekf_localization</span> subscribing <span class="topic">/mavros/imu/data</span>, <span class="topic">/mavros/global_position/raw/fix</span>', 520, ()=>activateChip('chip-ekf')],
      ['<span class="ok">[INFO]</span> <span class="node">/imu_filter_node</span> publishing <span class="topic">/imu/filtered</span> @ 200Hz', 680, ()=>activateChip('chip-imu')],
      ['<span class="ok">[INFO]</span> <span class="node">/takeoff_controller</span> waiting for <span class="field">ARM</span> service call', 860, ()=>activateChip('chip-ctrl')],
      ['<span class="ok">[INFO]</span> <span class="node">/drone_state_pub</span> publishing <span class="topic">/mavros/state</span> @ 5Hz', 1020, ()=>activateChip('chip-state')],
      ['<span class="ok">[INFO]</span> <span class="node">/cage_monitor</span> protective cage ring integrity: <span class="val">6/6 OK</span>', 1120, ()=>activateChip('chip-cage')],
      ['<span class="ok">[INFO]</span> <span class="node">/recovery_manager</span> standing by — autonomous self-righting &amp; relaunch armed', 1170, ()=>activateChip('chip-recovery')],
      ['<span class="dim">rosnode list</span>', 1200],
      ['<span class="dim">  /mavros\n  /ekf_localization\n  /imu_filter_node\n  /takeoff_controller\n  /drone_state_pub\n  /recovery_manager\n  /rosout</span>', 1250],
      ['<span class="warn">[WARN]</span> EKF variance nominal, waiting for GPS 3D fix...', 1550],
      ['<span class="ok">[INFO]</span> GPS 3D fix acquired · 11 satellites · HDOP 0.9', 2150, ()=>activateChip('chip-master')],
      ['<span class="topic">/mavros/state</span> connected: <span class="val">true</span>  armed: <span class="val">false</span>  mode: <span class="val">STABILIZE</span>', 2450],
      ['<span class="dim">System nominal. Awaiting operator command.</span>', 2750],
    ];
    boot.forEach(([html, delay, cb])=>{
      setTimeout(()=>{ log(html); if(cb) cb(); }, delay);
    });
  }

  let imuTimer = null;
  function startImuStream(){
    if(imuTimer) return;
    imuTimer = setInterval(()=>{
      const ax = (Math.random()*0.06-0.03).toFixed(3);
      const ay = (Math.random()*0.06-0.03).toFixed(3);
      const az = (9.81 + (Math.random()*0.04-0.02)).toFixed(3);
      logTopic('/mavros/imu/data', `lin_acc: [<span class="val">${ax}</span>, <span class="val">${ay}</span>, <span class="val">${az}</span>]`);
    }, 1400);
  }

  let poseTimer = null;
  function startPoseStream(){
    if(poseTimer) return;
    poseTimer = setInterval(()=>{
      const z = state.altitude.toFixed(3);
      const vz = state.vspeed.toFixed(3);
      logTopic('/mavros/local_position/pose', `z: <span class="val">${z}</span> m &nbsp; vz: <span class="val">${vz}</span> m/s`);
    }, 700);
  }

  // Derived physics telemetry — real thrust/torque/drag numbers computed
  // every tick by runController()/applyAeroDrag(), streamed to the log at
  // a readable rate rather than once per frame.
  let physicsTelemetryTimer = null;
  function startPhysicsTelemetryStream(){
    if(physicsTelemetryTimer) return;
    physicsTelemetryTimer = setInterval(()=>{
      logTopic('/physics/derived',
        `thrust: <span class="val">${state.lastThrustN.toFixed(2)}</span> N` +
        `&nbsp; T/W: <span class="val">${state.lastTWR.toFixed(2)}</span>` +
        `&nbsp; τ: <span class="val">${state.lastTorqueMag.toFixed(3)}</span> N·m` +
        `&nbsp; α: <span class="val">${state.lastAngAccel.toFixed(2)}</span> rad/s²` +
        `&nbsp; drag: <span class="val">${state.lastDragN.toFixed(2)}</span> N`);
    }, 950);
  }
  function stopPhysicsTelemetryStream(){
    if(physicsTelemetryTimer){ clearInterval(physicsTelemetryTimer); physicsTelemetryTimer = null; }
  }

  /* ============================================================
     PHYSICS ENGINE — cannon.js rigid-body simulation
     A real quadcopter model: 4 motors at fixed arm offsets, each
     producing thrust along the body's local +Y axis. Roll/pitch
     torque emerges from the lever-arm offset of each force (r×F)
     via cannon.js's own rigid-body solver — it is not scripted.
     Yaw comes from alternating motor spin-reaction torque. A small
     PID loop holds altitude and attitude the way a real flight
     controller's inner/outer loops work. The protective cage is
     modeled as the drone's actual collision hull, so it is what
     physically absorbs ground impact.
  ============================================================ */
  const DRONE_MASS = 1.2;            // kg
  const ARM = 0.16;                  // m, motor offset from center of mass
  const MAX_THRUST_PER_MOTOR = 5.4;  // N
  const TORQUE_CONST = 0.02;         // yaw reaction torque per N of thrust
  const CAGE_RADIUS = 0.35;          // m — also the collision hull radius
  const HOVER_THROTTLE = (DRONE_MASS*9.81)/(4*MAX_THRUST_PER_MOTOR);
  const STOP_TIME = 0.05;            // s, assumed crumple/stopping time for the G-force estimate
  const GROUND_OFFSET = CAGE_RADIUS; // the cage rests on the ground at this physics-Y height
  const GRAVITY = 9.81;              // m/s^2
  const WEIGHT_N = DRONE_MASS * GRAVITY;                       // N, total airframe weight
  const MAX_THRUST_N = 4 * MAX_THRUST_PER_MOTOR;               // N, full-throttle total thrust
  const MAX_TWR = MAX_THRUST_N / WEIGHT_N;                     // thrust-to-weight ratio at 100% throttle

  // ---- derived moment of inertia (simplified point-mass-at-arms model) ----
  // Each motor is treated as a point mass (mass/4) at radius ARM. This is an
  // independent, simplified estimate used only to drive the recovery
  // controller and the physics-telemetry readouts — cannon.js computes its
  // own inertia tensor from the actual box+sphere collision shapes for the
  // real simulation, so this value is explicitly an engineering estimate,
  // not the number the solver itself uses internally.
  const MOI_ROLL = DRONE_MASS * ARM * ARM;                     // kg·m^2, about X axis
  const MOI_PITCH = MOI_ROLL;                                  // kg·m^2, about Z axis (symmetric airframe)
  const MOI_YAW = 2 * DRONE_MASS * ARM * ARM;                  // kg·m^2, about Y axis (diagonal motor radius)

  // ---- aerodynamic drag model (applied as a real opposing force) ----
  const AIR_DENSITY = 1.225;         // kg/m^3, sea-level standard atmosphere
  const DRAG_COEFF = 0.9;            // dimensionless, bluff-body estimate for an open quad frame
  const FRONTAL_AREA = 0.045;        // m^2, approximate horizontal cross-section

  // ---- autonomous recovery controller gains ----
  // Heavily damped on purpose (Kd >> Kp relative to the airframe's real,
  // solver-computed inertia) so the self-righting maneuver settles smoothly
  // instead of overshooting and oscillating back and forth in the cage.
  const RECOVER_KP = 2.4;            // N·m per radian of tilt error
  const RECOVER_KD = 1.6;            // N·m per rad/s of angular velocity (damping)
  const RECOVER_DONE_ANGLE = 10 * Math.PI/180; // rad (~10°) — "close enough," then hard-locked level
  const RECOVER_DONE_OMEGA = 1.2;    // rad/s — considered "settled" (ground-contact jitter tolerant)
  const RECOVER_STABLE_TIME = 0.3;   // s the airframe must hold that state before the lock-in
  const RECOVER_MAX_TIME = 10.0;     // s safety fallback for the WHOLE recovery (spin sweep + correction) — proceed anyway rather than hang

  // ---- diagnostic spin sweep (runs before the corrective controller) ----
  // A deliberate, visible pre-correction phase, driven by real motor
  // thrust (see runDiagnosticSpin) — not a torque applied out of nowhere.
  // The cage is rigidly bonded to the same body as the motors, so it only
  // rotates because they do; two motors spin at full authority while the
  // other two idle, producing ~0.22 N·m of real spin-reaction yaw torque
  // (thrust × TORQUE_CONST, same formula as normal flight) against the
  // airframe's real ~0.098 kg·m² yaw inertia — good for a visible, but not
  // instant, ~2.5-revolution twirl before the PD controller commits.
  const RECOVER_SPIN_REVS = 2.5;     // target full revolutions before committing to correction
  const RECOVER_SPIN_DURATION = 5.0; // s safety cap, in case ground friction chews into the spin rate

  const world = new CANNON.World();
  world.gravity.set(0, -9.81, 0);
  world.broadphase = new CANNON.NaiveBroadphase();
  world.solver.iterations = 10;

  const groundMat = new CANNON.Material('ground');
  const droneMat = new CANNON.Material('drone');
  world.addContactMaterial(new CANNON.ContactMaterial(groundMat, droneMat, { friction: 0.4, restitution: 0.15 }));

  const groundBody = new CANNON.Body({ mass: 0, shape: new CANNON.Plane(), material: groundMat });
  groundBody.quaternion.setFromAxisAngle(new CANNON.Vec3(1,0,0), -Math.PI/2);
  world.addBody(groundBody);

  const droneBody = new CANNON.Body({ mass: DRONE_MASS, position: new CANNON.Vec3(0, GROUND_OFFSET+0.001, 0), material: droneMat });
  droneBody.addShape(new CANNON.Box(new CANNON.Vec3(0.09,0.04,0.09))); // chassis (fully enclosed by the cage sphere below)
  droneBody.addShape(new CANNON.Sphere(CAGE_RADIUS));                  // protective cage — the real collision hull
  droneBody.linearDamping = 0.05;
  droneBody.angularDamping = 0.4;
  world.addBody(droneBody);

  const MOTOR_POS = {
    FL: new CANNON.Vec3(-ARM, 0, -ARM),
    FR: new CANNON.Vec3( ARM, 0, -ARM),
    RL: new CANNON.Vec3(-ARM, 0,  ARM),
    RR: new CANNON.Vec3( ARM, 0,  ARM),
  };
  const MOTOR_SPIN = { FL: 1, FR: -1, RL: -1, RR: 1 }; // alternating CW/CCW for yaw authority
  const MOTOR_KEYS = ['FL','FR','RL','RR'];
  let lastMix = { FL:0, FR:0, RL:0, RR:0 };

  function eulerFromQuat(q){
    const sinr_cosp = 2*(q.w*q.x + q.y*q.z);
    const cosr_cosp = 1 - 2*(q.x*q.x + q.y*q.y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);
    const sinp = 2*(q.w*q.y - q.z*q.x);
    const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp)*Math.PI/2 : Math.asin(sinp);
    return { roll, pitch };
  }

  let altIntegral = 0, altPrevErr = 0;
  let crashHandled = false;

  function resetPhysicsBody(){
    droneBody.position.set(0, GROUND_OFFSET+0.001, 0);
    droneBody.velocity.set(0,0,0);
    droneBody.angularVelocity.set(0,0,0);
    droneBody.quaternion.set(0,0,0,1);
    droneBody.force.set(0,0,0);
    droneBody.torque.set(0,0,0);
    altIntegral = 0; altPrevErr = 0;
    crashHandled = false;
    state.strikeApplied = false;
    lastMix = { FL:0, FR:0, RL:0, RR:0 };
  }

  droneBody.addEventListener('collide', function(e){
    if(crashHandled) return;
    if(state.phase !== 'failure') return; // a controlled touchdown is handled separately, not as a "crash"
    const impactVel = Math.abs(e.contact.getImpactVelocityAlongNormal());
    if(impactVel < 0.4) return; // ignore tiny settling contacts
    crashHandled = true;
    const { roll, pitch } = eulerFromQuat(droneBody.quaternion);
    state.attitudeRoll = Math.round(roll*57.2958);
    state.attitudePitch = Math.round(pitch*57.2958);
    state.crashed = true;
    setPhase('crashed');
    logMission(`Impact detected — crash confirmed at ${impactVel.toFixed(2)} m/s`);
    finalizeCrash(impactVel);
  });

  // One control-loop tick: altitude + attitude PID -> motor mixer -> real
  // forces/torques on the rigid body. An active failure corrupts either
  // the sensor feedback or the mixer output exactly the way that failure
  // would in reality — the resulting tumble/fall is computed by the
  // physics engine, never animated by hand.
  function runController(dt){
    const p = state.phase;
    const flying = ['spooling','ascending','hover','landing','failure'].includes(p);
    if(!flying){ lastMix = { FL:0, FR:0, RL:0, RR:0 }; return; }

    const fm = state.failureMode;
    const failing = (p === 'failure') && !!fm;
    const openLoop = failing && (fm.msg === 'link_timeout' || fm.msg === 'nan_covariance');

    let mix;
    if(openLoop){
      // total loss of the correction loop: motors hold roughly their last
      // command with no feedback at all. Real airframes are never perfectly
      // symmetric, so a small uncorrected bias torque is what actually
      // sends an "uncontrolled" drone into a slow, inevitable tumble.
      mix = { FL:HOVER_THROTTLE, FR:HOVER_THROTTLE, RL:HOVER_THROTTLE, RR:HOVER_THROTTLE };
      droneBody.torque.vadd(new CANNON.Vec3(0.045, 0.01, 0.03), droneBody.torque);
    } else {
      let { roll, pitch } = eulerFromQuat(droneBody.quaternion);
      let rollRate = droneBody.angularVelocity.z;
      let pitchRate = droneBody.angularVelocity.x;
      let yawRate = droneBody.angularVelocity.y;

      if(failing && fm.msg === 'gyro_saturation'){
        // corrupted sensor feedback — the controller reacts to bad data
        roll += (Math.random()-0.5)*1.4;
        pitch += (Math.random()-0.5)*1.4;
      }
      if(failing && fm.msg === 'mag_interference'){
        yawRate += 3.5; // corrupted heading drives a hard, wrong yaw correction
      }

      const targetY = (p === 'landing') ? GROUND_OFFSET : GROUND_OFFSET + state.targetAlt;
      const altErr = targetY - droneBody.position.y;
      altIntegral = Math.max(-1.2, Math.min(1.2, altIntegral + altErr*dt));
      const altDeriv = (altErr - altPrevErr)/dt;
      altPrevErr = altErr;

      let throttle = HOVER_THROTTLE + 0.65*altErr + 0.08*altIntegral + 0.85*altDeriv;
      if(failing && fm.msg === 'abrupt_input') throttle = 0.08; // operator yanked the stick down
      throttle = Math.max(0, Math.min(1, throttle));

      const rollCmd = -0.6*roll - 0.15*rollRate;
      const pitchCmd = -0.6*pitch - 0.15*pitchRate;
      const yawCmd = -0.02*yawRate;

      mix = {
        FL: throttle - rollCmd + pitchCmd + yawCmd,
        FR: throttle + rollCmd + pitchCmd - yawCmd,
        RL: throttle - rollCmd - pitchCmd - yawCmd,
        RR: throttle + rollCmd - pitchCmd + yawCmd,
      };
      for(const k of MOTOR_KEYS) mix[k] = Math.max(0, Math.min(1, mix[k]));

      if(failing){
        switch(fm.msg){
          case 'desync':             mix.FR = Math.random()*0.9; break;
          case 'delamination':       mix.RR = 0; break;
          case 'bearing_seizure':    mix.FL = 0; break;
          case 'structural_failure': mix.RL = 0; break;
          case 'foreign_object_strike': mix.RL *= 0.35; break;
          case 'watchdog_reset':
            if(state.t - state.failureStartTime < 0.4){ mix = {FL:0,FR:0,RL:0,RR:0}; }
            break;
          case 'cell_undervoltage':
          case 'thermal_runaway': {
            const rate = fm.msg === 'thermal_runaway' ? 0.7 : 0.35;
            const decay = Math.max(0.12, 1 - (state.t - state.failureStartTime)*rate);
            for(const k of MOTOR_KEYS) mix[k] *= decay;
            break;
          }
          default: break; // fix_lost / gust_exceedance handled as external disturbances below
        }
      }
    }

    for(const k of MOTOR_KEYS) mix[k] = Math.max(0, Math.min(1, mix[k]));
    lastMix = mix;

    let yawTorque = 0;
    let totalThrust = 0;
    for(const k of MOTOR_KEYS){
      const thrust = mix[k]*MAX_THRUST_PER_MOTOR;
      totalThrust += thrust;
      droneBody.applyLocalForce(new CANNON.Vec3(0, thrust, 0), MOTOR_POS[k]);
      yawTorque += thrust*TORQUE_CONST*MOTOR_SPIN[k];
    }
    const worldYaw = droneBody.quaternion.vmult(new CANNON.Vec3(0, yawTorque, 0));
    droneBody.torque.vadd(worldYaw, droneBody.torque);

    // physics telemetry for the log panel / HUD — recomputed every tick,
    // never guessed: real commanded thrust, real accumulated torque so far
    // this step, and derived ratios from both.
    state.lastThrustN = totalThrust;
    state.lastTWR = totalThrust / WEIGHT_N;
    state.lastTorqueMag = droneBody.torque.length();
    state.lastAngAccel = state.lastTorqueMag / MOI_ROLL;

    if(failing && fm.msg === 'gust_exceedance'){
      // a real external disturbance applied to the body, not a scripted wobble
      droneBody.applyForce(new CANNON.Vec3((Math.random()-0.5)*14, 0, (Math.random()-0.5)*14), droneBody.position);
      droneBody.torque.vadd(new CANNON.Vec3((Math.random()-0.5)*2.5,(Math.random()-0.5)*1,(Math.random()-0.5)*2.5), droneBody.torque);
    }
    if(failing && fm.msg === 'foreign_object_strike' && !state.strikeApplied){
      state.strikeApplied = true;
      droneBody.applyImpulse(new CANNON.Vec3((Math.random()-0.5)*1.2, -0.3, (Math.random()-0.5)*1.2), droneBody.position);
    }
    if(failing && fm.msg === 'fix_lost'){
      droneBody.torque.vadd(new CANNON.Vec3(0,0.03,0), droneBody.torque); // slow uncorrected heading drift
    }
  }

  // Real quadratic aerodynamic drag (F = 1/2 * rho * Cd * A * v^2), applied
  // as an actual opposing force on the rigid body every tick — not just a
  // displayed number. Runs regardless of phase, same as drag would on a
  // real airframe; it's negligible at rest and only becomes meaningful
  // during a fast tumble, gust disturbance, or free fall.
  function applyAeroDrag(){
    const vx = droneBody.velocity.x, vz = droneBody.velocity.z;
    const speed = Math.sqrt(vx*vx + vz*vz);
    if(speed > 0.02){
      const dragMag = 0.5 * AIR_DENSITY * DRAG_COEFF * FRONTAL_AREA * speed * speed;
      state.lastDragN = dragMag;
      droneBody.applyForce(new CANNON.Vec3(-dragMag*(vx/speed), 0, -dragMag*(vz/speed)), droneBody.position);
    } else {
      state.lastDragN = 0;
    }
  }

  /* ============================================================
     AUTONOMOUS RECOVERY CONTROLLER
     Runs only during the 'recovering' phase, right after a crash.
     It is a separate, self-righting attitude controller — not the
     flight PID — because the airframe is resting/tumbled on the
     ground with zero thrust context. It drives the body's "up"
     vector back toward world-up using real applied torque (derived
     from the quaternion, not animated), damped by angular velocity,
     until the airframe is level and settled, then hands off to a
     normal auto re-arm + takeoff sequence.

     Before the corrective PD controller engages, runDiagnosticSpin() spins
     the airframe visibly through a couple of full revolutions first. The
     cage has no actuator of its own — it's rigidly bonded to the same
     chassis as the motors, so it can only move because the motors do. This
     spin is driven by the exact same mechanism as in-flight yaw control
     (see the motor mixer above, section 5): two diagonal motors (FL/RR)
     spin up while the other two (FR/RL) idle, and their real, unequal
     spin-reaction torque — thrust × TORQUE_CONST × spin direction, summed
     per motor, identical formula to normal flight — twirls the whole
     rigid body, cage included. It's framed in the log as a gyroscopic
     reference reacquisition sweep, but the motion is real motor-driven
     physics, not an animation.
  ============================================================ */
  function runDiagnosticSpin(dt){
    if(state.recoveryPhase !== 'spin') return;

    // FL + RR spin (same handedness, MOTOR_SPIN=+1) while FR + RL idle —
    // the maximum pure-yaw differential this airframe's motors can
    // produce, using the identical per-motor force + spin-reaction-torque
    // application as runController()'s normal flight mixer.
    const spinMix = { FL: 1.0, FR: 0, RL: 0, RR: 1.0 };
    lastMix = spinMix; // drives the visible rotor RPM (syncStateFromBody) — props actually spin during the sweep
    let yawTorque = 0, totalThrust = 0;
    for(const k of MOTOR_KEYS){
      const thrust = spinMix[k]*MAX_THRUST_PER_MOTOR;
      totalThrust += thrust;
      droneBody.applyLocalForce(new CANNON.Vec3(0, thrust, 0), MOTOR_POS[k]);
      yawTorque += thrust*TORQUE_CONST*MOTOR_SPIN[k];
    }
    const worldYaw = droneBody.quaternion.vmult(new CANNON.Vec3(0, yawTorque, 0));
    droneBody.torque.vadd(worldYaw, droneBody.torque);

    state.lastThrustN = totalThrust;
    state.recoverySpinAccumRad += droneBody.angularVelocity.length() * dt;
    const upWorldSpin = droneBody.quaternion.vmult(new CANNON.Vec3(0,1,0));
    state.recoveryAngleErrDeg = Math.acos(Math.max(-1, Math.min(1, upWorldSpin.y))) * 57.2958;
    state.recoveryOmega = droneBody.angularVelocity.length();
    state.recoveryTorqueMag = Math.abs(yawTorque);
    const revs = state.recoverySpinAccumRad / (2*Math.PI);
    const elapsed = state.t - state.recoverySpinStartTime;

    if(revs >= RECOVER_SPIN_REVS || elapsed >= RECOVER_SPIN_DURATION){
      state.recoveryPhase = 'correct';
      state.recoverStableSince = null; // give the corrective controller a clean settling window
      logMission(`Diagnostic motor-driven spin sweep complete — ${revs.toFixed(1)} revolutions — committing to attitude correction`);
      log(`<span class="ok">[INFO]</span> <span class="node">/recovery_manager</span> gyro sweep complete (${revs.toFixed(1)} rev, ${elapsed.toFixed(1)}s) — engaging corrective torque`);
    }
  }

  function runRecoveryController(dt){
    if(state.recoveryCompleting) return; // already handed off — hold the locked state, apply no more correction
    if(state.recoveryPhase !== 'correct') return; // still in the diagnostic spin sweep — see runDiagnosticSpin()
    const upWorld = droneBody.quaternion.vmult(new CANNON.Vec3(0,1,0));
    const dot = Math.max(-1, Math.min(1, upWorld.y));
    const angleErr = Math.acos(dot); // 0 rad when perfectly upright

    // torque axis = upWorld × worldUp — rotating about this axis by
    // angleErr brings the body's up vector back to true up.
    //
    // I tried deriving this stage's torque the same way as the spin sweep
    // — solved backward through the real per-motor thrust mixer — and
    // validated it numerically. It's mathematically exact when unclamped,
    // but real motor thrust can only push along the body's OWN local "up"
    // axis: from a rest orientation close to upside-down, that axis points
    // mostly into the ground, so no achievable motor mix can lift it back
    // over — a real crashed quad resting inverted has exactly this problem
    // too. So this final righting stage stays an idealized attitude-hold
    // torque (the same simplification the normal in-flight roll/pitch
    // hold already makes with small-angle euler terms) rather than a
    // literal motor solve that would leave some crashes unrecoverable.
    const axisX = -upWorld.z;
    const axisZ = upWorld.x;
    const torque = new CANNON.Vec3(axisX*RECOVER_KP, 0, axisZ*RECOVER_KP);
    const damp = droneBody.angularVelocity.scale(RECOVER_KD);
    torque.vsub(damp, torque);
    droneBody.torque.vadd(torque, droneBody.torque);

    // a small lift assist that grows as the airframe approaches level,
    // so it isn't fighting full ground friction through the whole roll
    const liftAssist = Math.max(0, 1 - angleErr/(Math.PI*0.5)) * 1.6;
    droneBody.applyLocalForce(new CANNON.Vec3(0, liftAssist, 0), new CANNON.Vec3(0,0,0));

    // the rotors still visibly spin through this stage — real motors are
    // what the torque above represents — scaled to how hard the
    // correction is working, uniformly across all four since (unlike the
    // spin sweep or normal flight) this stage doesn't solve an exact
    // per-motor split
    const effort = Math.min(1, torque.length()/6 + liftAssist/1.6*0.15);
    const spinLevel = 0.28 + effort*0.5;
    lastMix = { FL: spinLevel, FR: spinLevel, RL: spinLevel, RR: spinLevel };

    const omega = droneBody.angularVelocity.length();
    state.recoveryAngleErrDeg = angleErr * 57.2958;
    state.recoveryOmega = omega;
    state.recoveryTorqueMag = torque.length();

    if(angleErr < RECOVER_DONE_ANGLE && omega < RECOVER_DONE_OMEGA){
      if(state.recoverStableSince === null) state.recoverStableSince = state.t;
      if(state.t - state.recoverStableSince >= RECOVER_STABLE_TIME){
        completeRecovery();
        return;
      }
    } else {
      state.recoverStableSince = null;
    }

    // safety fallback — never hang in an endless correction loop
    if(state.recoveryStartTime !== null && (state.t - state.recoveryStartTime) > RECOVER_MAX_TIME){
      log('<span class="warn">[WARN]</span> <span class="node">/recovery_manager</span> reorientation timeout — proceeding with best-effort attitude');
      completeRecovery();
    }
  }

  let recoveryStreamTimer = null;
  function startRecoveryStream(){
    if(recoveryStreamTimer) return;
    recoveryStreamTimer = setInterval(()=>{
      logTopic('/recovery/attitude',
        `angle_err: <span class="val">${state.recoveryAngleErrDeg.toFixed(1)}°</span>` +
        `&nbsp; ω: <span class="val">${state.recoveryOmega.toFixed(2)}</span> rad/s` +
        `&nbsp; correction_torque: <span class="val">${state.recoveryTorqueMag.toFixed(2)}</span> N·m`);
    }, 220);
  }
  function stopRecoveryStream(){
    if(recoveryStreamTimer){ clearInterval(recoveryStreamTimer); recoveryStreamTimer = null; }
  }

  function beginAutoRecovery(){
    if(state.phase !== 'crashed') return;
    crashHandled = false;
    state.crashed = false;
    state.recoverStableSince = null;
    state.recoveryStartTime = state.t;
    state.recoveryCompleting = false;
    state.lockAnimDone = false;
    state.recoveryPhase = 'spin';
    state.recoverySpinAccumRad = 0;
    state.recoverySpinStartTime = state.t;
    lastMix = { FL:0, FR:0, RL:0, RR:0 };
    el.btnArm.disabled = true;
    el.btnTakeoff.disabled = true;
    el.btnLand.disabled = true;
    el.btnFault.disabled = true;
    el.failureSelect.disabled = true;
    setPhase('recovering');
    log('<span class="warn">[WARN]</span> <span class="node">/recovery_manager</span> post-impact self-check — initiating autonomous reorientation');
    logTopic('/diagnostics', 'level: <span class="val">1 (WARN)</span> name: "airframe/recovery" message: "autonomous reorientation engaged"');
    logTopic('/recovery/attitude', `initial angle_err: <span class="val">${(Math.acos(Math.max(-1,Math.min(1,droneBody.quaternion.vmult(new CANNON.Vec3(0,1,0)).y)))*57.2958).toFixed(1)}°</span>`);
    log('<span class="warn">[WARN]</span> <span class="node">/ekf_localization</span> attitude estimate low-confidence post-impact — running open-loop gyro sweep to reacquire reference before correcting');
    logMission('Diagnostic gyro sweep engaged — reacquiring attitude reference before correction');
    startRecoveryStream();
  }

  // Torque brings the airframe close to upright, then this hands off to a
  // short, eased kinematic glide (see runLockAnimation) that smoothly
  // finishes leveling it out — rather than teleporting instantly, which
  // reads as an artificial snap. This is a deliberate design choice, not a
  // claim that the whole recovery is torque-only: asymptotically settling
  // under real ground-contact friction can jitter indefinitely, so once the
  // torque controller gets it close, physics stops driving the rotation and
  // a smooth animation finishes the job. The cage (rendered from real
  // orientation while tumbling) eases to its locked angle at the same time,
  // instead of snapping or continuing to spin.
  function completeRecovery(){
    if(state.phase !== 'recovering' || state.recoveryCompleting) return;
    state.recoveryCompleting = true;
    stopRecoveryStream();

    state.lockFromQuat = { x: droneBody.quaternion.x, y: droneBody.quaternion.y, z: droneBody.quaternion.z, w: droneBody.quaternion.w };
    state.lockFromCageSpin = state.cageSpin;
    state.lockStartTime = state.t;
    state.lockAnimDone = false;
    droneBody.angularVelocity.set(0, 0, 0);
    droneBody.torque.set(0, 0, 0);
    droneBody.velocity.x *= 0.3;
    droneBody.velocity.z *= 0.3;
    if(droneBody.velocity.y < 0) droneBody.velocity.y *= 0.3;
    // motors settle to a steady, even hover-prep throttle for the glide,
    // rather than holding whatever asymmetric correction mix they had the
    // instant convergence was detected
    lastMix = { FL: 0.3, FR: 0.3, RL: 0.3, RR: 0.3 };

    log('<span class="ok">[INFO]</span> <span class="node">/recovery_manager</span> reorientation converged — easing to level lock');
    logTopic('/cage_monitor', 'structural self-check: <span class="ok">PASS</span> — cage integrity sufficient for relaunch');
    logMission('Airframe reoriented and locked upright — preparing to relaunch');
    state.failureMode = null;
    state.recoveryStartTime = null;
  }

  // Slerp between two plain {x,y,z,w} quaternions, t in [0,1]. cannon.js
  // 0.6.2 has no built-in slerp, so this is a standard implementation
  // (shortest-path, with a linear fallback when the quaternions are
  // nearly identical to avoid a division by ~0).
  function slerpQuat(qa, qb, t){
    let x0=qa.x, y0=qa.y, z0=qa.z, w0=qa.w;
    let x1=qb.x, y1=qb.y, z1=qb.z, w1=qb.w;
    let dot = x0*x1 + y0*y1 + z0*z1 + w0*w1;
    if(dot < 0){ x1=-x1; y1=-y1; z1=-z1; w1=-w1; dot=-dot; }
    if(dot > 0.9995){
      const x=x0+(x1-x0)*t, y=y0+(y1-y0)*t, z=z0+(z1-z0)*t, w=w0+(w1-w0)*t;
      const len = Math.sqrt(x*x+y*y+z*z+w*w) || 1;
      return { x:x/len, y:y/len, z:z/len, w:w/len };
    }
    const theta0 = Math.acos(dot);
    const theta = theta0*t;
    const sinTheta0 = Math.sin(theta0);
    const s1 = Math.sin(theta)/sinTheta0;
    const s0 = Math.cos(theta) - dot*s1;
    return { x:s0*x0+s1*x1, y:s0*y0+s1*y1, z:s0*z0+s1*z1, w:s0*w0+s1*w1 };
  }

  const LOCK_ANIM_DURATION = 0.6; // s — eased glide from converged attitude to perfectly level
  function easeOutCubic(x){ return 1 - Math.pow(1-x, 3); }

  // Runs every frame during the post-recovery lock-in: eases the airframe's
  // orientation and the cage's rendered angle from wherever the torque
  // controller left them to a perfectly level, fixed lock — smoothly,
  // instead of the instantaneous snap this replaced.
  function runLockAnimation(){
    if(state.lockAnimDone) return;
    const elapsed = state.t - state.lockStartTime;
    const frac = Math.min(1, elapsed / LOCK_ANIM_DURATION);
    const eased = easeOutCubic(frac);

    const q = slerpQuat(state.lockFromQuat, {x:0,y:0,z:0,w:1}, eased);
    droneBody.quaternion.set(q.x, q.y, q.z, q.w);
    droneBody.angularVelocity.set(0, 0, 0);
    droneBody.velocity.x *= 0.9;
    droneBody.velocity.z *= 0.9;

    let diff = 0.55 - state.lockFromCageSpin;
    diff = Math.atan2(Math.sin(diff), Math.cos(diff)); // shortest angular path
    state.cageSpin = state.lockFromCageSpin + diff*eased;

    if(frac >= 1){
      state.lockAnimDone = true;
      state.tilt = 0;
      state.attitudeRoll = 0;
      state.attitudePitch = 0;
      state.cageSpin = 0.55;
      state.recoveryFollowupTimer = setTimeout(()=>{ autoReArm(); }, 400);
    }
  }

  function autoReArm(){
    if(state.phase !== 'recovering') return;
    logTopic('/takeoff_controller', 'autonomous relaunch authorized — no operator input required');
    logMission('Autonomous relaunch authorized');
    performArmSequence(()=>{ startTakeoffSequence(); });
  }

  function syncStateFromBody(){
    state.altitude = Math.max(0, droneBody.position.y - GROUND_OFFSET);
    state.vspeed = droneBody.velocity.y;
    const { roll, pitch } = eulerFromQuat(droneBody.quaternion);
    state.attitudeRoll = Math.round(roll*57.2958);
    state.attitudePitch = Math.round(pitch*57.2958);
    // reduce full 3D orientation to the single bank angle the 2D side-view needs
    const upWorld = droneBody.quaternion.vmult(new CANNON.Vec3(0,1,0));
    const bankMag = Math.acos(Math.max(-1, Math.min(1, upWorld.y))) * 57.2958;
    state.tilt = (upWorld.x < 0 ? -1 : 1) * bankMag;
    // the cage is the drone's real collision hull, so while the airframe
    // is actually tumbling (failure / crashed / recovering) it visually
    // rolls with the body's ACTUAL orientation instead of a fixed angle.
    // Once recovery locks the airframe level (completeRecovery), or during
    // any normal flight phase, the cage holds its last angle rather than
    // continuing to drift — it doesn't spin during ordinary operation.
    if(['failure','crashed','recovering'].includes(state.phase) && !state.recoveryCompleting){
      const localX = droneBody.quaternion.vmult(new CANNON.Vec3(1,0,0));
      state.cageSpin = Math.atan2(localX.z, localX.x) + 0.55;
    }
    const avgMix = (lastMix.FL+lastMix.FR+lastMix.RL+lastMix.RR)/4;
    const targetRpm = avgMix>0 ? 900 + avgMix*6400 : 0;
    state.motorRpm += (targetRpm - state.motorRpm) * 0.25;
    state.throttle = avgMix;
  }

  /* ============================================================
     CONTROL HANDLERS
  ============================================================ */
  function setPhase(p){
    state.phase = p;
    el.phaseBanner.className = '';
    const map = {
      idle: ['STANDBY', ''],
      arming: ['ARMING…', 'armed'],
      armed: ['ARMED · READY', 'armed'],
      spooling: ['MOTORS SPOOLING', 'armed'],
      ascending: ['TAKEOFF · ASCENDING', 'flying'],
      hover: ['HOVER · HOLDING ALTITUDE', 'flying'],
      landing: ['LANDING', 'armed'],
      failure: ['MECHANICAL FAILURE · LOSS OF CONTROL', 'fail'],
      crashed: ['CRASHED · SYSTEM HALTED', 'fail'],
      recovering: ['AUTONOMOUS RECOVERY · REORIENTING', 'armed'],
    };
    const [label, cls] = map[p] || [p.toUpperCase(), ''];
    el.phaseBanner.textContent = label;
    if(cls) el.phaseBanner.classList.add(cls);
  }

  // Shared arm sequence — used by the manual ARM button and by the
  // autonomous post-recovery relaunch. `onArmed` fires once armed.
  function performArmSequence(onArmed){
    el.btnArm.disabled = true;
    setPhase('arming');
    log('<span class="dim">operator@gcs:~$</span> rosservice call <span class="field">/mavros/cmd/arming</span> "value: true"');
    logTopic('/mavros/state', 'armed: <span class="val">false</span> → arming sequence initiated');
    startImuStream();
    setTimeout(()=>{
      if(state.phase !== 'arming') return; // superseded by a reset/abort in the meantime
      state.armed = true;
      logTopic('/mavros/state', 'connected: <span class="val">true</span>  armed: <span class="val">true</span>  mode: <span class="val">GUIDED</span>');
      log('<span class="ok">[INFO]</span> <span class="node">/takeoff_controller</span> pre-arm checks: <span class="ok">PASSED</span>');
      setPhase('armed');
      el.hudArmed.textContent = 'TRUE';
      el.hudArmed.classList.add('armed');
      el.hudMode.textContent = 'GUIDED';
      el.btnTakeoff.disabled = false;
      el.btnArm.classList.add('engaged');
      el.btnArm.textContent = 'DISARM';
      el.btnArm.disabled = false;
      logMission('Airframe armed — pre-arm checks passed');
      if(typeof onArmed === 'function') onArmed();
    }, 1300);
  }

  // Shared takeoff sequence — used by the manual TAKEOFF button and by the
  // autonomous post-recovery relaunch.
  function startTakeoffSequence(){
    el.btnTakeoff.disabled = true;
    el.btnArm.disabled = true;
    setPhase('spooling');
    log('<span class="dim">operator@gcs:~$</span> rostopic pub <span class="field">/takeoff_controller/goal</span> <span class="field">altitude: 4.2</span>');
    log('<span class="ok">[INFO]</span> <span class="node">/takeoff_controller</span> executing TAKEOFF behavior · target alt <span class="val">4.2m</span>');
    logMission('Takeoff command issued — climbing to target altitude');
    startPoseStream();
    startPhysicsTelemetryStream();
    setTimeout(()=>{
      if(state.phase !== 'spooling') return; // superseded by a reset/abort in the meantime
      setPhase('ascending');
      el.btnLand.disabled = false;
      el.btnFault.disabled = false;
      el.failureSelect.disabled = false;
      // fresh takeoff, fresh slate — no failure scenario carried over from
      // a previous flight (or left selected from before a crash)
      state.failureMode = null;
      state.crashed = false;
      state.strikeApplied = false;
      el.failureSelect.value = 'random';
      logTopic('/diagnostics', 'level: <span class="val">0 (OK)</span> name: "airframe/failure_state" message: "cleared — no active failure scenario"');
    }, 900);
  }

  el.btnArm.addEventListener('click', ()=>{
    if(state.armed) return;
    state.missionLog = []; // a fresh manual ARM starts a new flight's timeline
    state.telemetrySamples = [];
    state.lastTelemetrySampleT = -999;
    state.flightStartWallClock = Date.now();
    logMission('Operator armed the airframe');
    performArmSequence();
  });

  // convert ARM button into a toggle once armed
  el.btnArm.addEventListener('click', function toggle(){
    if(el.btnArm.textContent === 'DISARM' && state.phase === 'armed'){
      state.armed = false;
      el.hudArmed.textContent = 'FALSE';
      el.hudArmed.classList.remove('armed');
      el.hudMode.textContent = 'STABILIZE';
      el.btnArm.textContent = 'ARM';
      el.btnArm.classList.remove('engaged');
      el.btnTakeoff.disabled = true;
      setPhase('idle');
      log('<span class="dim">operator@gcs:~$</span> rosservice call <span class="field">/mavros/cmd/arming</span> "value: false"');
      logTopic('/mavros/state', 'armed: <span class="val">false</span>');
    }
  });

  el.btnTakeoff.addEventListener('click', ()=>{
    if(state.phase !== 'armed') return;
    startTakeoffSequence();
  });

  el.btnLand.addEventListener('click', ()=>{
    if(state.phase !== 'hover' && state.phase !== 'ascending') return;
    el.btnLand.disabled = true;
    el.btnFault.disabled = true;
    el.failureSelect.disabled = true;
    setPhase('landing');
    log('<span class="dim">operator@gcs:~$</span> rostopic pub <span class="field">/takeoff_controller/goal</span> <span class="field">altitude: 0.0</span>');
    log('<span class="ok">[INFO]</span> <span class="node">/takeoff_controller</span> executing LAND behavior');
    logMission('Land command issued — descending');
  });

  // populate the failure-mode picker from FAIL_MODES
  FAIL_MODES.forEach((m, i)=>{
    const opt = document.createElement('option');
    opt.value = String(i);
    opt.textContent = m.name.toUpperCase();
    el.failureSelect.appendChild(opt);
  });

  function triggerFailure(forced){
    if(state.crashed) return;
    if(!(state.phase==='ascending' || state.phase==='hover' || state.phase==='spooling')) return;
    let mode;
    if(forced === true){
      const sel = el.failureSelect.value;
      mode = sel === 'random' ? FAIL_MODES[Math.floor(Math.random()*FAIL_MODES.length)] : FAIL_MODES[parseInt(sel,10)];
    } else {
      mode = FAIL_MODES[Math.floor(Math.random()*FAIL_MODES.length)];
    }
    state.failureMode = mode;
    state.failureStartAlt = state.altitude;
    state.failureStartTime = state.t;
    state.strikeApplied = false;
    el.btnLand.disabled = true;
    el.btnFault.disabled = true;
    el.failureSelect.disabled = true;
    setPhase('failure');
    log(`<span class="err">[ERROR]</span> <span class="node">/mavros</span> ${mode.log1}`);
    log('<span class="err">[ERROR]</span> <span class="node">/ekf_localization</span> covariance divergence — attitude estimate unreliable');
    logTopic('/diagnostics', `level: <span class="val">2 (ERROR)</span> name: <span class="val">"${mode.diag}"</span> message: "<span class="val">${mode.msg}</span>"`);
    logTopic('/mavros/state', 'armed: <span class="val">true</span>  mode: <span class="val">EMERGENCY</span>');
    log('<span class="warn">[WARN]</span> <span class="node">/takeoff_controller</span> control loop watchdog timeout — recovery unavailable');
    logMission(`Mechanical failure triggered — ${mode.name}`);
  }

  el.btnFault.addEventListener('click', ()=>triggerFailure(true));

  function finalizeCrash(impactV){
    const fallDuration = Math.max(0, state.t - state.failureStartTime);
    const incId = 'INC-' + Math.floor(100000 + Math.random()*899999);
    const fm = state.failureMode;
    log('<span class="err">[FATAL]</span> <span class="node">/drone_state_pub</span> impact detected — telemetry link lost');
    logTopic('/mavros/state', 'connected: <span class="val">false</span>');
    log('<span class="err">[FATAL]</span> <span class="node">/cage_monitor</span> impact absorbed by protective cage — airframe integrity: <span class="val">DAMAGED</span>');
    deactivateChip('chip-mavros');
    deactivateChip('chip-state');

    // real, physics-computed telemetry
    const gForce = (Math.abs(impactV) / (9.81*STOP_TIME)).toFixed(1) + ' G';
    const roll = state.attitudeRoll;
    const pitch = state.attitudePitch;

    // simulated (not physically modeled) supporting telemetry
    const homeDist = (Math.random()*1.8).toFixed(1) + ' m from launch point';
    const windOptions = ['Calm, <2 kt','6 kt SW, gusting 10 kt','13 kt ESE, gusting 19 kt','Light breeze, 4 kt N'];
    const wind = fm.msg === 'gust_exceedance' ? '22 kt gusting 31 kt, NW' : windOptions[Math.floor(Math.random()*windOptions.length)];

    let battPct = Math.max(4, Math.round(94 - state.t*0.7 - Math.random()*6));
    let battV = (16.8 * (battPct/94)).toFixed(1);
    let battTemp = Math.round(34 + Math.random()*8);
    if(fm.msg === 'cell_undervoltage'){ battPct = Math.max(2, battPct-30); battV = (3.1*4).toFixed(1); }
    if(fm.msg === 'thermal_runaway'){ battTemp = Math.round(82 + Math.random()*13); battPct = Math.max(2, battPct-20); }

    const rssi = fm.msg === 'link_timeout' ? '-97dBm · LINK LOST' : `-${Math.round(58+Math.random()*20)}dBm · DEGRADED`;
    const gps = fm.msg === 'fix_lost' ? 'NO FIX · LOST' : `3D FIX · ${Math.round(6+Math.random()*5)} SATS (DEGRADED)`;
    const firmware = ['PX4 v1.14.3','ArduCopter 4.5.7','PX4 v1.15.0-rc2','ArduCopter 4.4.4'][Math.floor(Math.random()*4)];
    const cageDamage = ['2 of 6 rings deformed — payload protected','Outer ring cracked, inner frame intact','No visible damage — cage absorbed full impact','4 of 6 rings deformed, airframe still enclosed'][Math.floor(Math.random()*4)];

    document.getElementById('crIncidentId').textContent = incId;
    document.getElementById('crMode').textContent = fm.name;
    document.getElementById('crTime').textContent = el.clock.textContent;
    document.getElementById('crFlightTime').textContent = state.failureStartTime.toFixed(1) + 's';
    document.getElementById('crAlt').textContent = state.failureStartAlt.toFixed(2) + ' m';
    document.getElementById('crFall').textContent = fallDuration.toFixed(2) + 's';
    document.getElementById('crImpact').textContent = Math.abs(impactV).toFixed(2) + ' m/s';
    document.getElementById('crGforce').textContent = gForce;
    document.getElementById('crAttitude').textContent = `roll ${roll}° / pitch ${pitch}°`;
    document.getElementById('crHome').textContent = homeDist;
    document.getElementById('crWind').textContent = wind;
    document.getElementById('crBattery').textContent = `${battV}V · ${battPct}%`;
    document.getElementById('crBattTemp').textContent = battTemp + '°C';
    document.getElementById('crRssi').textContent = rssi;
    document.getElementById('crGps').textContent = gps;
    document.getElementById('crFirmware').textContent = firmware;
    document.getElementById('crCage').textContent = 'Deployed — absorbed impact';
    document.getElementById('crCageDamage').textContent = cageDamage;
    document.getElementById('crBlackbox').textContent =
`[BLACK BOX — LAST TELEMETRY]
mode: EMERGENCY   armed: true
throttle: ${Math.round(state.throttle*100)}%   motor_rpm: ${Math.round(state.motorRpm)}
battery: ${battV}V (${battPct}%)   gps: ${gps}
rc_link: ${rssi}   wind: ${wind}
diagnostic: ${fm.diag} -> ${fm.msg}`;
    document.getElementById('crRec').textContent = fm.cause + ' Recommend grounding the airframe for inspection before next flight.';

    setTimeout(()=>{
      try{
        document.getElementById('crCrashImg').src = canvas.toDataURL('image/png');
      }catch(e){ /* canvas capture unavailable — report still shows without it */ }
    }, 220);

    // The report is captured and held, but not shown — the airframe goes
    // straight into autonomous recovery. It becomes available to view/
    // download once this flight safely lands (see the loop()'s landing
    // check). A short beat lets the crash register visually first.
    state.reportPending = true;
    state.reportAvailable = true;
    log('<span class="dim">— incident report captured, held for review after next safe landing —</span>');
    state.crashRecoveryDelayTimer = setTimeout(()=>{
      beginAutoRecovery();
    }, 1200);
  }

  // Renders the full-flight mission timeline (state.missionLog) into the
  // incident report. Called once the recovered flight lands safely, so the
  // report tells the whole story — takeoff, failure, crash, stabilize,
  // relaunch, and landing — not just the moment of impact.
  function renderMissionTimeline(){
    const wrap = document.getElementById('crTimeline');
    if(!wrap) return;
    if(!state.missionLog.length){
      wrap.innerHTML = '<div class="tl-empty">No timeline recorded.</div>';
      return;
    }
    wrap.innerHTML = state.missionLog.map(ev=>
      `<div class="tl-row"><span class="tl-t">${ev.clock}</span><span class="tl-label">${ev.text}</span></div>`
    ).join('');
  }

  /* ============================================================
     INCIDENT REPORT -> PNG EXPORT
     Redraws the current incident report (crash frame + every data
     row + black box + recommendation) onto an offscreen canvas so
     it can be saved as a single shareable image, independent of
     whatever's on screen.
  ============================================================ */
  function wrapCanvasText(ctx, text, maxWidth, font){
    ctx.font = font;
    const words = String(text).split(' ');
    const lines = [];
    let line = '';
    words.forEach(w=>{
      const test = line ? line + ' ' + w : w;
      if(ctx.measureText(test).width > maxWidth && line){
        lines.push(line);
        line = w;
      } else {
        line = test;
      }
    });
    if(line) lines.push(line);
    return lines;
  }

  /* ============================================================
     MAVLINK v2 .tlog EXPORT
     A genuine MAVLink v2 binary encoder — real packet framing, real
     CRC16/MCRF4XX checksums with per-message CRC_EXTRA — built from the
     samples in state.telemetrySamples (recorded every ~100ms through the
     actual flight, see sampleTelemetry()). The output is a standard
     QGroundControl/MAVProxy .tlog: a stream of
     [8-byte big-endian microsecond timestamp][raw MAVLink packet] records.
     It is honestly scoped as a *simulator* speaking the real protocol over
     synthetic data, not a capture from a real autopilot — HEARTBEAT
     reports MAV_AUTOPILOT_GENERIC (0), never PX4/ArduPilot IDs.
  ============================================================ */

  // MAVLink's standard CRC-16/MCRF4XX (X.25) accumulator — same bit
  // pattern used by every real MAVLink implementation (pymavlink, the C
  // library, etc.) so third-party tools can verify these packets.
  function mavlinkCrcAccumulate(data, crc){
    let tmp = (data ^ (crc & 0xFF)) & 0xFF;
    tmp = (tmp ^ (tmp << 4)) & 0xFF;
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
  }
  function mavlinkCrc16(bytes, crcExtra){
    let crc = 0xFFFF;
    for(let i=0;i<bytes.length;i++) crc = mavlinkCrcAccumulate(bytes[i], crc);
    crc = mavlinkCrcAccumulate(crcExtra, crc);
    return crc;
  }

  // CRC_EXTRA is derived from each message's real XML field layout (part
  // of the MAVLink spec itself) — these four are the well-known, unchanged
  // constants for common.xml's HEARTBEAT / ATTITUDE / GLOBAL_POSITION_INT /
  // STATUSTEXT.
  const MAV_CRC_EXTRA = { HEARTBEAT: 50, ATTITUDE: 39, GLOBAL_POSITION_INT: 104, STATUSTEXT: 83 };
  const MAV_MSG_ID = { HEARTBEAT: 0, ATTITUDE: 30, GLOBAL_POSITION_INT: 33, STATUSTEXT: 253 };
  const MAV_SYSID = 1, MAV_COMPID = 1;
  const HOME_LAT = 47.397742, HOME_LON = 8.545594, HOME_ALT_MSL = 488; // Zurich — a conventional SITL-style home fix, cosmetic only

  function packMavlinkV2(msgName, payload){
    const msgid = MAV_MSG_ID[msgName];
    const crcExtra = MAV_CRC_EXTRA[msgName];
    const len = payload.length;
    const seq = packMavlinkV2._seq = ((packMavlinkV2._seq||0) + 1) & 0xFF;
    // header = everything after the STX marker, which is what the CRC covers
    const header = new Uint8Array([len, 0, 0, seq, MAV_SYSID, MAV_COMPID, msgid & 0xFF, (msgid>>8)&0xFF, (msgid>>16)&0xFF]);
    const crcBuf = new Uint8Array(header.length + payload.length);
    crcBuf.set(header, 0);
    crcBuf.set(payload, header.length);
    const crc = mavlinkCrc16(crcBuf, crcExtra);
    const packet = new Uint8Array(1 + header.length + payload.length + 2);
    packet[0] = 0xFD; // MAVLink v2 STX
    packet.set(header, 1);
    packet.set(payload, 1 + header.length);
    packet[packet.length-2] = crc & 0xFF;
    packet[packet.length-1] = (crc>>8) & 0xFF;
    return packet;
  }

  function mavEncodeHeartbeat(sample){
    const buf = new ArrayBuffer(9);
    const dv = new DataView(buf);
    dv.setUint32(0, 0, true);                 // custom_mode
    dv.setUint8(4, 2);                        // type: MAV_TYPE_QUADROTOR
    dv.setUint8(5, 0);                        // autopilot: MAV_AUTOPILOT_GENERIC — honest: this is a simulator, not a real FC
    dv.setUint8(6, sample.armed ? 0x80 : 0);  // base_mode: MAV_MODE_FLAG_SAFETY_ARMED bit
    const status = sample.phase==='crashed' ? 6 : (sample.phase==='recovering' ? 5 : (sample.armed ? 4 : 3)); // EMERGENCY/CRITICAL/ACTIVE/STANDBY
    dv.setUint8(7, status);
    dv.setUint8(8, 3);                        // mavlink_version
    return new Uint8Array(buf);
  }

  function mavEncodeAttitude(sample){
    const buf = new ArrayBuffer(28);
    const dv = new DataView(buf);
    dv.setUint32(0, Math.max(0, Math.round(sample.t*1000)) >>> 0, true);
    dv.setFloat32(4, sample.rollDeg*Math.PI/180, true);
    dv.setFloat32(8, sample.pitchDeg*Math.PI/180, true);
    dv.setFloat32(12, sample.yaw, true);
    dv.setFloat32(16, sample.rollRate, true);
    dv.setFloat32(20, sample.pitchRate, true);
    dv.setFloat32(24, sample.yawRate, true);
    return new Uint8Array(buf);
  }

  function mavEncodeGlobalPositionInt(sample, prevSample){
    const buf = new ArrayBuffer(28);
    const dv = new DataView(buf);
    const lat = HOME_LAT + (sample.posZ*0.000009);
    const lon = HOME_LON + (sample.posX*0.000009/Math.cos(HOME_LAT*Math.PI/180));
    let vxCm = 0, vzCm = 0;
    if(prevSample){
      const ddt = Math.max(0.001, sample.t - prevSample.t);
      vxCm = (sample.posX - prevSample.posX) / ddt * 100;
      vzCm = (sample.posZ - prevSample.posZ) / ddt * 100;
    }
    const clamp16 = v => Math.max(-32768, Math.min(32767, Math.round(v)));
    dv.setUint32(0, Math.max(0, Math.round(sample.t*1000)) >>> 0, true);
    dv.setInt32(4, Math.round(lat*1e7), true);
    dv.setInt32(8, Math.round(lon*1e7), true);
    dv.setInt32(12, Math.round((HOME_ALT_MSL+sample.alt)*1000), true);
    dv.setInt32(16, Math.round(sample.alt*1000), true);
    dv.setInt16(20, clamp16(vxCm), true);
    dv.setInt16(22, clamp16(vzCm), true);
    dv.setInt16(24, clamp16(-sample.vspeed*100), true);
    dv.setUint16(26, sample.yaw>=0 ? Math.round(sample.yaw*5729.58)%36000 : Math.round((sample.yaw+2*Math.PI)*5729.58)%36000, true); // hdg, centidegrees
    return new Uint8Array(buf);
  }

  function mavEncodeStatustext(text, severity){
    const buf = new Uint8Array(51);
    buf[0] = severity;
    // encode first, THEN truncate to 50 bytes — slicing the JS string by
    // character count first can still overflow, since UTF-8 multi-byte
    // characters (e.g. the em dashes used throughout the mission log) can
    // take more than one byte each
    let bytes = new TextEncoder().encode(String(text));
    if(bytes.length > 50) bytes = bytes.slice(0, 50);
    buf.set(bytes, 1);
    return buf;
  }

  // Assembles the full .tlog byte stream from state.telemetrySamples +
  // state.missionLog (the mission events become real STATUSTEXT packets,
  // interleaved at their actual timestamps) — a real ATTITUDE +
  // GLOBAL_POSITION_INT pair per sample, plus a HEARTBEAT roughly once a
  // second, exactly like a real MAVLink stream.
  // samples/missionLog/baseWallClockMs default to the live flight's own
  // buffers, but can be passed explicitly to export a past flight loaded
  // from local history instead (see the per-entry export button below).
  function buildMavlinkTlog(samples, missionLog, baseWallClockMs){
    samples = samples || state.telemetrySamples;
    missionLog = missionLog || state.missionLog;
    baseWallClockMs = baseWallClockMs || state.flightStartWallClock || Date.now();
    packMavlinkV2._seq = 0;
    const parts = [];
    const baseUnixUs = baseWallClockMs * 1000;
    function pushRecord(tSeconds, packetBytes){
      const us = Math.round(baseUnixUs + tSeconds*1e6);
      const tsBuf = new ArrayBuffer(8);
      const dv = new DataView(tsBuf);
      dv.setUint32(0, Math.floor(us/4294967296), false);
      dv.setUint32(4, us>>>0, false);
      parts.push(new Uint8Array(tsBuf));
      parts.push(packetBytes);
    }
    const events = missionLog.slice();
    let ei = 0;
    let lastHeartbeatT = -999;
    let prevSample = null;
    for(const sample of samples){
      while(ei < events.length && events[ei].t <= sample.t){
        pushRecord(events[ei].t, packMavlinkV2('STATUSTEXT', mavEncodeStatustext(events[ei].text.replace(/<[^>]*>/g,''), 6)));
        ei++;
      }
      if(sample.t - lastHeartbeatT >= 1.0){
        pushRecord(sample.t, packMavlinkV2('HEARTBEAT', mavEncodeHeartbeat(sample)));
        lastHeartbeatT = sample.t;
      }
      pushRecord(sample.t, packMavlinkV2('ATTITUDE', mavEncodeAttitude(sample)));
      pushRecord(sample.t, packMavlinkV2('GLOBAL_POSITION_INT', mavEncodeGlobalPositionInt(sample, prevSample)));
      prevSample = sample;
    }
    while(ei < events.length){
      pushRecord(events[ei].t, packMavlinkV2('STATUSTEXT', mavEncodeStatustext(events[ei].text.replace(/<[^>]*>/g,''), 6)));
      ei++;
    }
    return new Blob(parts, { type: 'application/octet-stream' });
  }

  function triggerBlobDownload(blob, filename){
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(()=>URL.revokeObjectURL(a.href), 4000);
  }

  function downloadMavlinkTlog(){
    if(!state.telemetrySamples.length){
      log('<span class="warn">[WARN]</span> <span class="node">/mavros</span> no recorded telemetry for this flight yet — nothing to export');
      return;
    }
    const blob = buildMavlinkTlog();
    const incId = el.crIncidentId ? el.crIncidentId.textContent.replace(/[^\w-]/g,'') : String(Date.now());
    triggerBlobDownload(blob, `${incId}_flight.tlog`);
    log(`<span class="ok">[INFO]</span> <span class="node">/mavros</span> exported ${state.telemetrySamples.length} samples as genuine MAVLink v2 .tlog`);
  }

  // Exports a past flight loaded from local history as a .tlog — same
  // encoder, fed from that flight's saved (downsampled) samples instead of
  // the live buffer.
  function downloadMavlinkTlogForRecord(rec){
    if(!rec || !rec.samples.length) return;
    const blob = buildMavlinkTlog(rec.samples, rec.missionLog, rec.ts);
    triggerBlobDownload(blob, `${rec.id}_flight.tlog`);
  }

  /* ============================================================
     LOCAL FLIGHT HISTORY + REPLAY
     Every completed flight (armed → landed, whatever happened in between)
     is saved to localStorage — real recorded telemetry samples, not a
     re-simulation — so it survives a page reload and can be scrubbed back
     through later. Persists only in this browser; nothing leaves the
     device.
  ============================================================ */
  const HISTORY_KEY = 'droneSimFlightHistory_v1';
  const HISTORY_MAX = 15;
  const HISTORY_SAMPLE_CAP = 240; // keep localStorage entries small

  function downsampleSamples(samples, cap){
    if(samples.length <= cap) return samples.slice();
    const out = [];
    const step = (samples.length-1) / (cap-1);
    for(let i=0;i<cap;i++) out.push(samples[Math.round(i*step)]);
    return out;
  }

  function loadFlightHistory(){
    try{
      return JSON.parse(localStorage.getItem(HISTORY_KEY) || '[]');
    }catch(e){ return []; }
  }

  function saveFlightHistory(){
    if(!state.telemetrySamples.length) return;
    try{
      const failureLine = state.missionLog.find(e => e.text.includes('Mechanical failure triggered'));
      const crashed = state.missionLog.some(e => e.text.includes('Impact detected'));
      const relaunched = state.missionLog.some(e => e.text.includes('Autonomous relaunch authorized'));
      const record = {
        id: 'FL-' + Date.now(),
        ts: Date.now(),
        durationSec: state.telemetrySamples[state.telemetrySamples.length-1].t,
        peakAlt: state.telemetrySamples.reduce((m,s)=>Math.max(m,s.alt), 0),
        crashed: crashed,
        relaunched: relaunched,
        cause: failureLine ? failureLine.text.replace(/^Mechanical failure triggered — /, '') : null,
        missionLog: state.missionLog.slice(),
        samples: downsampleSamples(state.telemetrySamples, HISTORY_SAMPLE_CAP),
      };
      const list = loadFlightHistory();
      list.unshift(record);
      while(list.length > HISTORY_MAX) list.pop();
      localStorage.setItem(HISTORY_KEY, JSON.stringify(list));
    }catch(e){
      // localStorage full/unavailable (private browsing, quota) — flight
      // history is a convenience feature, so fail silently rather than
      // interrupting the flight
    }
  }

  function fmtHistoryClock(ts){
    const d = new Date(ts);
    return d.toLocaleString(undefined, { month:'short', day:'numeric', hour:'2-digit', minute:'2-digit' });
  }

  function renderHistoryList(){
    const list = loadFlightHistory();
    el.historyCount.textContent = `${list.length} saved`;
    if(!list.length){
      el.historyList.innerHTML = '<div class="hist-empty">No flights recorded yet on this device — fly and land once to start building history.</div>';
      return;
    }
    el.historyList.innerHTML = list.map(rec => `
      <div class="hist-item" data-id="${rec.id}">
        <div class="hist-meta">
          <div class="hist-title ${rec.crashed?'crashed':''}">${rec.crashed ? (rec.relaunched ? 'CRASHED · SELF-RECOVERED' : 'CRASHED') : 'NORMAL FLIGHT'}${rec.cause ? ' — '+rec.cause : ''}</div>
          <div class="hist-sub">${fmtHistoryClock(rec.ts)} · ${rec.durationSec.toFixed(1)}s · peak ${rec.peakAlt.toFixed(1)}m</div>
        </div>
        <div class="hist-actions">
          <button class="hist-replay">▶ REPLAY</button>
          <button class="hist-tlog">⬇ .TLOG</button>
          <button class="hist-delete danger">DELETE</button>
        </div>
      </div>
    `).join('');
    el.historyList.querySelectorAll('.hist-replay').forEach(btn=>{
      btn.addEventListener('click', (e)=>{
        const id = e.target.closest('.hist-item').dataset.id;
        const rec = loadFlightHistory().find(r=>r.id===id);
        if(rec) startReplay(rec);
      });
    });
    el.historyList.querySelectorAll('.hist-tlog').forEach(btn=>{
      btn.addEventListener('click', (e)=>{
        const id = e.target.closest('.hist-item').dataset.id;
        const rec = loadFlightHistory().find(r=>r.id===id);
        if(rec) downloadMavlinkTlogForRecord(rec);
      });
    });
    el.historyList.querySelectorAll('.hist-delete').forEach(btn=>{
      btn.addEventListener('click', (e)=>{
        const id = e.target.closest('.hist-item').dataset.id;
        const list2 = loadFlightHistory().filter(r=>r.id!==id);
        localStorage.setItem(HISTORY_KEY, JSON.stringify(list2));
        renderHistoryList();
      });
    });
  }

  el.btnHistory.addEventListener('click', ()=>{
    renderHistoryList();
    el.historyOverlay.classList.add('show');
  });
  el.btnHistoryClose.addEventListener('click', ()=> el.historyOverlay.classList.remove('show'));
  el.btnHistoryClearAll.addEventListener('click', ()=>{
    if(!confirm('Clear all saved flight history on this device? This cannot be undone.')) return;
    localStorage.removeItem(HISTORY_KEY);
    renderHistoryList();
  });

  // Drives the same render()/updateHud() as a live flight, but from a
  // recorded sample instead of physics — see applyReplaySample() and its
  // call in loop().
  function startReplay(rec){
    el.historyOverlay.classList.remove('show');
    state.replay.active = true;
    state.replay.flight = rec;
    state.replay.index = 0;
    state.replay.playing = true;
    el.replayScrub.max = String(Math.max(0, rec.samples.length-1));
    el.replayScrub.value = '0';
    el.btnReplayPlay.textContent = '⏸ PAUSE';
    el.replayBar.classList.add('show');
    // disable live flight controls while replaying a recorded flight
    [el.btnArm, el.btnTakeoff, el.btnLand, el.btnFault, el.failureSelect].forEach(b=>b.disabled = true);
    log(`<span class="dim">— entering replay of ${fmtHistoryClock(rec.ts)} (${rec.samples.length} samples) —</span>`);
  }

  function exitReplay(){
    state.replay.active = false;
    state.replay.flight = null;
    el.replayBar.classList.remove('show');
    resetSimulation();
    log('<span class="dim">— replay ended, standing by for operator command —</span>');
  }

  function applyReplaySample(){
    const rec = state.replay.flight;
    if(!rec) return;
    const idx = Math.max(0, Math.min(Math.floor(state.replay.index), rec.samples.length-1));
    const s = rec.samples[idx];
    if(!s) return;
    state.t = s.t;
    state.altitude = s.alt;
    state.vspeed = s.vspeed;
    state.tilt = s.tilt;
    state.attitudeRoll = s.rollDeg;
    state.attitudePitch = s.pitchDeg;
    state.cageSpin = s.cageSpin;
    state.motorRpm = s.motorRpm;
    state.throttle = s.throttle;
    state.armed = s.armed;
    setPhase(s.phase);
    el.hudArmed.textContent = s.armed ? 'TRUE' : 'FALSE';
    el.hudArmed.classList.toggle('armed', s.armed);
    el.replayScrub.value = String(idx);
    updateHud();
    // updateHud() stamps the topbar clock from live wall-clock time — in
    // replay we want it (and the dedicated replay-bar readout) to show the
    // recorded flight's own timestamp instead, so set these after.
    el.replayClock.textContent = s.clock || 'T+00:00:00.0';
    el.clock.textContent = s.clock || el.clock.textContent;
    render();
  }

  el.replayScrub.addEventListener('input', ()=>{
    if(!state.replay.active) return;
    state.replay.playing = false;
    el.btnReplayPlay.textContent = '▶ PLAY';
    state.replay.index = parseInt(el.replayScrub.value, 10) || 0;
    applyReplaySample();
  });
  el.btnReplayPlay.addEventListener('click', ()=>{
    if(!state.replay.active) return;
    state.replay.playing = !state.replay.playing;
    el.btnReplayPlay.textContent = state.replay.playing ? '⏸ PAUSE' : '▶ PLAY';
  });
  el.btnReplayExit.addEventListener('click', exitReplay);

  function downloadCrashReportPNG(){
    const rows = Array.from(document.querySelectorAll('#crash-card .body > .row')).map(r=>({
      k: r.querySelector('.k').textContent,
      v: r.querySelector('.v').textContent,
    }));
    const blackboxLines = document.getElementById('crBlackbox').textContent.split('\n');
    const recText = document.getElementById('crRec').textContent;
    const noteText = document.querySelector('#crash-card .note').textContent;
    const incId = document.getElementById('crIncidentId').textContent;
    const modeText = document.getElementById('crMode').textContent;
    const imgEl = document.getElementById('crCrashImg');

    const W = 860;
    const M = 36;
    const contentW = W - M*2;

    const measure = document.createElement('canvas').getContext('2d');
    const recLines = wrapCanvasText(measure, recText, contentW, '13px "JetBrains Mono", monospace');
    const noteLines = wrapCanvasText(measure, noteText, contentW, '11px "JetBrains Mono", monospace');

    const hasImg = imgEl && imgEl.complete && imgEl.naturalWidth > 0;
    const imgAspect = hasImg ? (imgEl.naturalHeight / imgEl.naturalWidth) : 0.5625;
    const imgH = Math.round(contentW * imgAspect);
    const timelineRowH = 20;
    const timeline = state.missionLog.slice();

    const rowH = 28;
    let H = 100;                        // header
    H += (hasImg ? imgH + 24 : 10);     // crash frame
    H += 26 + Math.max(1, timeline.length) * timelineRowH + 20; // mission timeline
    H += rows.length * rowH + 16;       // data rows
    H += 30 + blackboxLines.length*17 + 18; // black box
    H += recLines.length*19 + 22;       // recommendation
    H += noteLines.length*15 + 30;      // fine-print note
    H += M;

    const DPR = 2;
    const canvas = document.createElement('canvas');
    canvas.width = W*DPR; canvas.height = H*DPR;
    const ctx = canvas.getContext('2d');
    ctx.scale(DPR, DPR);
    ctx.textBaseline = 'top';

    ctx.fillStyle = '#000000';
    ctx.fillRect(0, 0, W, H);

    ctx.fillStyle = '#ff6b5d';
    ctx.font = '700 20px Rajdhani, sans-serif';
    ctx.fillText('INCIDENT REPORT', M, 30);
    ctx.font = '12px "JetBrains Mono", monospace';
    ctx.fillStyle = '#8a7d6f';
    ctx.textAlign = 'right';
    ctx.fillText(incId, W-M, 32);
    ctx.textAlign = 'left';
    ctx.font = '13px "JetBrains Mono", monospace';
    ctx.fillStyle = '#ede6dc';
    ctx.fillText(modeText, M, 58);

    let cy = 96;
    if(hasImg){
      try{
        ctx.drawImage(imgEl, M, cy, contentW, imgH);
        ctx.strokeStyle = '#332821';
        ctx.strokeRect(M, cy, contentW, imgH);
      }catch(e){ /* tainted or unavailable — skip the image, report still exports */ }
      cy += imgH + 24;
    }

    ctx.fillStyle = '#ede6dc';
    ctx.font = '700 12px Rajdhani, sans-serif';
    ctx.fillText('MISSION TIMELINE — TAKEOFF TO LANDING', M, cy);
    cy += 22;
    ctx.font = '11.5px "JetBrains Mono", monospace';
    if(timeline.length){
      timeline.forEach(ev=>{
        ctx.fillStyle = '#e8905c';
        ctx.fillText(ev.clock, M, cy);
        ctx.fillStyle = '#ede6dc';
        ctx.fillText(ev.text, M + 88, cy);
        cy += timelineRowH;
      });
    } else {
      ctx.fillStyle = '#8a7d6f';
      ctx.fillText('No timeline recorded.', M, cy);
      cy += timelineRowH;
    }
    cy += 16;

    ctx.font = '12.5px "JetBrains Mono", monospace';
    rows.forEach(r=>{
      ctx.strokeStyle = '#332821';
      ctx.beginPath();
      ctx.moveTo(M, cy+rowH-6);
      ctx.lineTo(W-M, cy+rowH-6);
      ctx.stroke();
      ctx.fillStyle = '#8a7d6f';
      ctx.fillText(r.k, M, cy);
      ctx.fillStyle = '#ede6dc';
      ctx.textAlign = 'right';
      ctx.fillText(r.v, W-M, cy);
      ctx.textAlign = 'left';
      cy += rowH;
    });
    cy += 10;

    ctx.fillStyle = '#ede6dc';
    ctx.font = '700 12px Rajdhani, sans-serif';
    ctx.fillText('BLACK BOX — LAST TELEMETRY', M, cy);
    cy += 22;
    ctx.font = '11.5px "JetBrains Mono", monospace';
    ctx.fillStyle = '#8a7d6f';
    blackboxLines.forEach(line=>{ ctx.fillText(line, M, cy); cy += 17; });
    cy += 14;

    ctx.fillStyle = '#ffb020';
    ctx.font = '13px "JetBrains Mono", monospace';
    recLines.forEach(line=>{ ctx.fillText(line, M, cy); cy += 19; });
    cy += 12;

    ctx.fillStyle = '#5f574d';
    ctx.font = '11px "JetBrains Mono", monospace';
    noteLines.forEach(line=>{ ctx.fillText(line, M, cy); cy += 15; });

    const link = document.createElement('a');
    link.download = `${incId}_incident_report.png`;
    link.href = canvas.toDataURL('image/png');
    link.click();
  }

  function resetSimulation(){
    // cancel any pending autonomous recovery / relaunch so a manual reset
    // always wins, whatever stage the auto sequence was in
    clearTimeout(state.crashRecoveryDelayTimer); state.crashRecoveryDelayTimer = null;
    clearTimeout(state.recoveryFollowupTimer); state.recoveryFollowupTimer = null;
    stopRecoveryStream();
    stopPhysicsTelemetryStream();
    state.recoverStableSince = null;
    state.recoveryStartTime = null;
    state.recoveryCompleting = false;
    state.lockAnimDone = false;
    state.recoveryPhase = null;
    state.recoverySpinAccumRad = 0;
    state.reportPending = false;
    state.reportAvailable = false;
    state.missionLog = [];
    state.telemetrySamples = [];
    state.lastTelemetrySampleT = -999;
    state.cageSpin = 0.55;

    state.phase = 'idle';
    state.armed = false;
    state.throttle = 0;
    state.motorRpm = 0;
    state.altitude = 0;
    state.vspeed = 0;
    state.tilt = 0;
    state.attitudeRoll = 0;
    state.attitudePitch = 0;
    state.crashed = false;
    state.failureMode = null;

    resetPhysicsBody();

    el.crashOverlay.classList.remove('show');
    el.btnViewReport.disabled = true;
    el.hudArmed.textContent = 'FALSE';
    el.hudArmed.classList.remove('armed');
    el.hudMode.textContent = 'STABILIZE';
    el.btnArm.textContent = 'ARM';
    el.btnArm.classList.remove('engaged');
    el.btnArm.disabled = false;
    el.btnTakeoff.disabled = true;
    el.btnLand.disabled = true;
    el.btnFault.disabled = true;
    el.failureSelect.disabled = true;
    el.failureSelect.value = 'random';

    activateChip('chip-mavros');
    activateChip('chip-state');
    setPhase('idle');
    log('<span class="dim">— system reset — standing by for operator command —</span>');
  }

  el.btnReset.addEventListener('click', resetSimulation);
  el.btnCrashAck.addEventListener('click', ()=> el.crashOverlay.classList.remove('show'));
  el.btnDownloadReport.addEventListener('click', downloadCrashReportPNG);
  el.btnDownloadTlog.addEventListener('click', downloadMavlinkTlog);
  el.btnViewReport.addEventListener('click', ()=> el.crashOverlay.classList.add('show'));

  el.btnKiosk.addEventListener('click', ()=>{
    if(!document.fullscreenElement){
      document.documentElement.requestFullscreen().catch(()=>{});
    } else {
      document.exitFullscreen();
    }
  });
  document.addEventListener('fullscreenchange', ()=>{
    const isFull = !!document.fullscreenElement;
    document.body.classList.toggle('kiosk', isFull);
    el.btnKiosk.textContent = isFull ? '⛶ EXIT KIOSK' : '⛶ KIOSK';
  });

  /* ============================================================
     HUD + METERS
  ============================================================ */
  function updateHud(){
    el.hudAlt.textContent = state.altitude.toFixed(2) + ' m';
    el.hudVspd.textContent = (state.vspeed>=0?'+':'') + state.vspeed.toFixed(2) + ' m/s';
    el.hudThr.textContent = Math.round(state.throttle*100) + '%';

    el.mAlt.innerHTML = state.altitude.toFixed(1) + '<small>m</small>';
    el.mAltBar.style.width = Math.min(100, (state.altitude/state.targetAlt)*100) + '%';

    el.mThr.innerHTML = Math.round(state.throttle*100) + '<small>%</small>';
    el.mThrBar.style.width = Math.round(state.throttle*100) + '%';

    el.mRpm.textContent = Math.round(state.motorRpm);
    el.mRpmBar.style.width = Math.min(100, (state.motorRpm/7300)*100) + '%';

    const sElapsed = (performance.now() - state.startTime)/1000;
    const h = String(Math.floor(sElapsed/3600)).padStart(2,'0');
    const m = String(Math.floor((sElapsed%3600)/60)).padStart(2,'0');
    const s = (sElapsed%60).toFixed(1).padStart(4,'0');
    el.clock.textContent = `T+${h}:${m}:${s}`;
  }

  /* ============================================================
     CANVAS SCENE — drone, ground, sky
  ============================================================ */
  const canvas = document.getElementById('scene');
  const ctx = canvas.getContext('2d');
  let W, H, DPR;

  function resize(){
    DPR = Math.min(window.devicePixelRatio || 1, 2);
    const rect = canvas.parentElement.getBoundingClientRect();
    W = rect.width; H = rect.height;
    canvas.width = W*DPR; canvas.height = H*DPR;
    ctx.setTransform(DPR,0,0,DPR,0,0);
  }
  window.addEventListener('resize', resize);
  resize();

  function drawSky(){
    const grad = ctx.createLinearGradient(0,0,0,H);
    grad.addColorStop(0, '#0a0806');
    grad.addColorStop(0.55, '#241408');
    grad.addColorStop(1, '#3a2010');
    ctx.fillStyle = grad;
    ctx.fillRect(0,0,W,H);

    ctx.fillStyle = 'rgba(255,255,255,0.35)';
    for(let i=0;i<40;i++){
      const sx = (i*137.5) % W;
      const sy = (i*67.3) % (H*0.5);
      ctx.globalAlpha = 0.15 + 0.15*Math.sin(state.t*2+i);
      ctx.fillRect(sx, sy, 1.4, 1.4);
    }
    ctx.globalAlpha = 1;
  }

  const horizonY = () => H*0.66;

  function drawGround(cx, hz, groundY){
    const gGrad = ctx.createLinearGradient(0,hz,0,H);
    gGrad.addColorStop(0, '#231609');
    gGrad.addColorStop(1, '#0f0904');
    ctx.fillStyle = gGrad;
    ctx.fillRect(0,hz,W,H-hz);

    const PAD_Z = 6;
    const K = (groundY - hz) * PAD_Z;
    const LAT = 12;
    const unitMeters = 2;

    for(let z=3; z<=42; z++){
      const y = hz + K/z;
      if(y > H+2) continue;
      const depthFrac = Math.min(1, (z-3)/24);
      const alpha = 0.34*(1-depthFrac) + 0.05;
      const halfW = 9*(y-hz)/LAT;
      ctx.strokeStyle = `rgba(232,144,92,${alpha.toFixed(3)})`;
      ctx.lineWidth = z===PAD_Z ? 1.6 : 1;
      ctx.beginPath();
      ctx.moveTo(cx-halfW, y);
      ctx.lineTo(cx+halfW, y);
      ctx.stroke();
      if(z%6===0 && z>PAD_Z){
        ctx.fillStyle = `rgba(232,144,92,${(alpha+0.25).toFixed(3)})`;
        ctx.font = '10px JetBrains Mono, monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`${(z-PAD_Z)*unitMeters}m`, cx+halfW+6, y+3);
      }
    }

    const yNear = hz + K/3;
    const yFar = hz + K/42;
    for(let x=-9; x<=9; x++){
      const xNear = cx + x*(yNear-hz)/LAT;
      const xFar = cx + x*(yFar-hz)/LAT;
      const depthFrac = Math.abs(x)/9;
      ctx.strokeStyle = x===0 ? 'rgba(232,144,92,0.5)' : `rgba(232,144,92,${(0.22*(1-depthFrac)+0.05).toFixed(3)})`;
      ctx.lineWidth = x===0 ? 1.4 : 1;
      ctx.beginPath();
      ctx.moveTo(xNear, yNear);
      ctx.lineTo(xFar, yFar);
      ctx.stroke();
    }

    const padHalf = 9*(groundY-hz)/LAT / 4.5;
    ctx.save();
    ctx.translate(cx, groundY);
    ctx.scale(1, 0.34);
    const glow = ctx.createRadialGradient(0,0,0,0,0,padHalf*2.2);
    glow.addColorStop(0, 'rgba(232,144,92,0.10)');
    glow.addColorStop(1, 'rgba(232,144,92,0)');
    ctx.fillStyle = glow;
    ctx.beginPath();
    ctx.arc(0,0, padHalf*2.2, 0, Math.PI*2);
    ctx.fill();
    ctx.restore();

    ctx.strokeStyle = 'rgba(232,144,92,0.4)';
    ctx.beginPath(); ctx.moveTo(0,hz); ctx.lineTo(W,hz); ctx.stroke();
  }

  function drawPad(cx, groundY){
    ctx.save();
    ctx.translate(cx, groundY);
    ctx.scale(1, 0.34);
    ctx.strokeStyle = 'rgba(232,144,92,0.55)';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.arc(0,0, 62, 0, Math.PI*2);
    ctx.stroke();
    ctx.strokeStyle = 'rgba(232,144,92,0.35)';
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(0,0,40,0,Math.PI*2); ctx.stroke();
    ctx.restore();
  }

  function drawShadow(cx, groundY, altPx, scale){
    const spread = Math.max(0.18, 1 - altPx/ (H*0.4));
    ctx.save();
    ctx.translate(cx, groundY);
    ctx.scale(1,0.32);
    const grad = ctx.createRadialGradient(0,0,0,0,0,70*scale*spread);
    grad.addColorStop(0,'rgba(0,0,0,0.55)');
    grad.addColorStop(1,'rgba(0,0,0,0)');
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(0,0, 70*scale*spread, 0, Math.PI*2);
    ctx.fill();
    ctx.restore();
  }

  function drawProp(x, y, r, angle, rpm){
    const blur = Math.min(1, rpm/6500);
    ctx.save();
    ctx.translate(x,y);
    if(blur < 0.12){
      ctx.rotate(angle);
      ctx.strokeStyle = '#cfe3f0';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(-r,0); ctx.lineTo(r,0); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0,-r*0.5); ctx.lineTo(0,r*0.5); ctx.stroke();
    } else {
      ctx.beginPath();
      ctx.fillStyle = `rgba(207,227,240,${0.10+blur*0.16})`;
      ctx.ellipse(0,0, r, r*0.42, 0, 0, Math.PI*2);
      ctx.fill();
      ctx.strokeStyle = `rgba(232,144,92,${0.25+blur*0.35})`;
      ctx.lineWidth = 1;
      for(let k=0;k<3;k++){
        ctx.beginPath();
        ctx.ellipse(0,0, r*(0.5+k*0.22), r*0.18*(0.5+k*0.22)*2.3, 0, 0, Math.PI*2);
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  function drawDrone(cx, cy, scale, tiltDeg, rpm, armed){
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(tiltDeg*Math.PI/180);
    ctx.scale(scale, scale);

    const armLen = 54;
    const armY = -6;

    ctx.strokeStyle = '#3a4b5c';
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    [[-1,-1],[1,-1],[-1,1],[1,1]].forEach(([sx,sy])=>{
      ctx.beginPath();
      ctx.moveTo(0, armY);
      ctx.lineTo(sx*armLen, armY + sy*30);
      ctx.stroke();
    });

    const angle = state.t*(rpm/60)*Math.PI*2;
    [[-1,-1],[1,-1],[-1,1],[1,1]].forEach(([sx,sy])=>{
      const mx = sx*armLen, my = armY + sy*30;
      ctx.beginPath();
      ctx.fillStyle = armed ? '#16324a' : '#20242c';
      ctx.arc(mx,my,7,0,Math.PI*2);
      ctx.fill();
      ctx.strokeStyle = '#e8905c44';
      ctx.lineWidth = 1;
      ctx.stroke();
      drawProp(mx, my, 26, angle + (sx*sy>0?0:Math.PI/2), rpm);
      ctx.beginPath();
      ctx.fillStyle = armed ? '#ff5d5d' : '#3a4b5c';
      ctx.arc(mx, my, 2.4, 0, Math.PI*2);
      ctx.fill();
    });

    ctx.strokeStyle = '#3a4b5c';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(-16, 10); ctx.lineTo(-24, 34);
    ctx.moveTo(16, 10); ctx.lineTo(24, 34);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(-30,34); ctx.lineTo(-18,34);
    ctx.moveTo(18,34); ctx.lineTo(30,34);
    ctx.stroke();

    const bodyGrad = ctx.createLinearGradient(0,-20,0,14);
    bodyGrad.addColorStop(0,'#2a3a4a');
    bodyGrad.addColorStop(1,'#12202c');
    ctx.fillStyle = bodyGrad;
    roundRect(ctx, -20,-16, 40, 30, 7);
    ctx.fill();
    ctx.strokeStyle = '#e8905c88';
    ctx.lineWidth = 1.2;
    roundRect(ctx, -20,-16, 40, 30, 7);
    ctx.stroke();

    ctx.beginPath();
    ctx.fillStyle = armed ? (rpm>3500 ? '#3dffa0' : '#ffb020') : '#5f7c90';
    ctx.arc(0,-2, 3, 0, Math.PI*2);
    ctx.fill();
    if(armed){
      ctx.save();
      ctx.globalAlpha = 0.5;
      ctx.beginPath();
      ctx.fillStyle = rpm>3500 ? '#3dffa0' : '#ffb020';
      ctx.arc(0,-2, 7, 0, Math.PI*2);
      ctx.fill();
      ctx.restore();
    }

    ctx.beginPath();
    ctx.fillStyle = '#0a1620';
    ctx.arc(0, 16, 5, 0, Math.PI*2);
    ctx.fill();

    ctx.restore();
  }

  function cagePoint(theta, phi, R, rotY, tiltX){
    let x0 = R*Math.sin(theta)*Math.cos(phi);
    let y0 = -R*Math.cos(theta);
    let z0 = R*Math.sin(theta)*Math.sin(phi);
    const x1 = x0*Math.cos(rotY) + z0*Math.sin(rotY);
    const z1 = -x0*Math.sin(rotY) + z0*Math.cos(rotY);
    const y1 = y0;
    const y2 = y1*Math.cos(tiltX) - z1*Math.sin(tiltX);
    const z2 = y1*Math.sin(tiltX) + z1*Math.cos(tiltX);
    return {x:x1, y:y2, z:z2};
  }

  function drawCage(cx, cy, scale, rotY, armed, bankDeg){
    const R = 84;
    // base perspective tilt, plus a real contribution from the body's
    // actual bank angle so the cage visibly tumbles during a crash/
    // recovery instead of always sitting at the same fixed angle
    const tiltX = 0.52 + Math.max(-0.9, Math.min(0.9, ((bankDeg||0)*Math.PI/180)*0.5));
    ctx.save();
    ctx.translate(cx, cy - 4*scale);
    ctx.scale(scale, scale);

    const baseCol = armed ? '198,214,224' : '150,164,174';

    const numLat = 6, latSteps = 30;
    for(let i=1;i<numLat;i++){
      const theta = Math.PI*i/numLat;
      const thick = (i===Math.round(numLat/2)) ? 2.4 : 1.3;
      let prev = null;
      for(let s=0;s<=latSteps;s++){
        const phi = (s/latSteps)*Math.PI*2;
        const p = cagePoint(theta, phi, R, rotY, tiltX);
        if(prev){
          const depth = (p.z/R + 1)/2;
          const alpha = 0.10 + 0.34*depth;
          ctx.strokeStyle = `rgba(${baseCol},${alpha.toFixed(3)})`;
          ctx.lineWidth = thick;
          ctx.beginPath();
          ctx.moveTo(prev.x, prev.y);
          ctx.lineTo(p.x, p.y);
          ctx.stroke();
        }
        prev = p;
      }
    }

    const numLong = 10, longSteps = 22;
    for(let i=0;i<numLong;i++){
      const phi = (i/numLong)*Math.PI*2;
      let prev = null;
      for(let s=0;s<=longSteps;s++){
        const theta = (s/longSteps)*Math.PI;
        const p = cagePoint(theta, phi, R, rotY, tiltX);
        if(prev){
          const depth = (p.z/R + 1)/2;
          const alpha = 0.08 + 0.28*depth;
          ctx.strokeStyle = `rgba(${baseCol},${alpha.toFixed(3)})`;
          ctx.lineWidth = 1.1;
          ctx.beginPath();
          ctx.moveTo(prev.x, prev.y);
          ctx.lineTo(p.x, p.y);
          ctx.stroke();
        }
        prev = p;
      }
    }

    ctx.restore();
  }

  function roundRect(ctx,x,y,w,h,r){
    ctx.beginPath();
    ctx.moveTo(x+r,y);
    ctx.arcTo(x+w,y,x+w,y+h,r);
    ctx.arcTo(x+w,y+h,x,y+h,r);
    ctx.arcTo(x,y+h,x,y,r);
    ctx.arcTo(x,y,x+w,y,r);
    ctx.closePath();
  }

  function render(){
    const hz = horizonY();
    const cx = W/2;
    const groundY = hz + (H-hz)*0.42;

    drawSky();
    drawGround(cx, hz, groundY);
    drawPad(cx, groundY);

    const maxAltPx = H*0.46;
    const altFrac = Math.min(1, state.altitude / (state.targetAlt*1.05));
    const altPx = maxAltPx * (1 - Math.pow(1-altFrac,1.6));
    const droneY = groundY - altPx - 30;
    const scale = 1.55 - Math.min(0.35, altPx/(H*1.4));

    drawShadow(cx, groundY, altPx, scale);
    const isLive = state.armed || ['spooling','ascending','hover','landing','failure','crashed','recovering'].includes(state.phase);
    drawDrone(cx, droneY, scale, state.tilt, state.motorRpm, isLive);
    drawCage(cx, droneY, scale, state.cageSpin, isLive, state.tilt);

    if((state.phase==='spooling' || (state.phase==='ascending' && state.altitude < 0.6))){
      ctx.save();
      ctx.globalAlpha = 0.22;
      ctx.fillStyle = '#cfe3f0';
      for(let i=0;i<10;i++){
        const dx = cx + Math.sin(state.t*7+i)*40*(1+i*0.15);
        const dy = groundY - (i*3 + (state.t*40)%20);
        ctx.beginPath();
        ctx.arc(dx, dy, 1.6, 0, Math.PI*2);
        ctx.fill();
      }
      ctx.restore();
    }

    if(state.phase === 'crashed'){
      ctx.save();
      for(let i=0;i<14;i++){
        const seed = i*13.7;
        const rise = (state.t*18 + seed*4) % 90;
        const dx = cx + Math.sin(state.t*0.6 + seed)*(14 + rise*0.35);
        const dy = groundY - rise;
        const a = Math.max(0, 0.32 - rise/90*0.32);
        ctx.beginPath();
        ctx.fillStyle = `rgba(150,150,150,${a.toFixed(3)})`;
        ctx.arc(dx, dy, 4 + rise*0.08, 0, Math.PI*2);
        ctx.fill();
      }
      ctx.beginPath();
      const wgrad = ctx.createRadialGradient(cx,groundY,0,cx,groundY,60);
      wgrad.addColorStop(0, `rgba(255,93,93,${0.18+0.1*Math.sin(state.t*6)})`);
      wgrad.addColorStop(1, 'rgba(255,93,93,0)');
      ctx.fillStyle = wgrad;
      ctx.ellipse(cx, groundY, 60, 20, 0, 0, Math.PI*2);
      ctx.fill();
      ctx.restore();
    }
  }

  /* ============================================================
     MAIN LOOP
  ============================================================ */
  let lastT = performance.now();
  function loop(now){
    const dt = Math.min(0.05, (now-lastT)/1000);
    lastT = now;

    if(state.replay.active){
      if(state.replay.playing){
        // real recorded samples are ~10Hz (0.1s apart); advance at that
        // pace scaled by wall-clock dt so scrubbing feels like real time
        state.replay.index += dt * 10 * state.replay.speed;
        if(state.replay.index >= state.replay.flight.samples.length-1){
          state.replay.index = state.replay.flight.samples.length-1;
          state.replay.playing = false;
          el.btnReplayPlay.textContent = '▶ PLAY';
        }
      }
      applyReplaySample();
      requestAnimationFrame(loop);
      return;
    }

    state.t += dt;

    // random chance of an in-flight mechanical failure (can also be forced
    // any time via the SIMULATE FAILURE button)
    if((state.phase === 'ascending' || state.phase === 'hover') && Math.random() < dt*0.004){
      triggerFailure();
    }

    if(state.phase === 'recovering'){
      if(state.recoveryCompleting){
        runLockAnimation();
      } else if(state.recoveryPhase === 'spin'){
        runDiagnosticSpin(dt);
      } else {
        runRecoveryController(dt);
      }
    } else {
      runController(dt);
    }
    applyAeroDrag();
    world.step(1/120, dt, 10);
    syncStateFromBody();
    sampleTelemetry();

    if(state.phase === 'ascending'){
      if(Math.abs(state.targetAlt - state.altitude) < 0.15 && Math.abs(state.vspeed) < 0.3){
        setPhase('hover');
        logTopic('/takeoff_controller/status', 'state: <span class="val">HOLD</span> · reached target altitude');
        log('<span class="ok">[INFO]</span> <span class="node">/takeoff_controller</span> goal reached · switching to position hold');
        logMission('Reached target altitude — holding hover');
      }
    } else if(state.phase === 'landing'){
      if(state.altitude < 0.08 && Math.abs(state.vspeed) < 0.2){
        setPhase('armed');
        log('<span class="ok">[INFO]</span> <span class="node">/takeoff_controller</span> touchdown confirmed · motors idle');
        logTopic('/mavros/state', 'armed: <span class="val">true</span>  mode: <span class="val">GUIDED</span>  landed_state: <span class="val">ON_GROUND</span>');
        logMission('Touchdown confirmed — motors idle');
        el.btnTakeoff.disabled = false;
        el.btnArm.disabled = false;
        saveFlightHistory();

        if(state.reportPending){
          state.reportPending = false;
          renderMissionTimeline();
          el.btnViewReport.disabled = false;
          log('<span class="ok">[INFO]</span> <span class="node">/recovery_manager</span> safe landing confirmed — incident report from the previous crash is now available (📋 LAST REPORT)');
        }
      }
    }

    updateHud();
    render();
    requestAnimationFrame(loop);
  }

  /* ============================================================
     HOME <-> APP navigation
  ============================================================ */
  const homeScreen = document.getElementById('home-screen');
  const appScreen = document.getElementById('app-screen');
  const btnLaunch = document.getElementById('btnLaunch');
  const btnHome = document.getElementById('btnHome');
  let started = false;

  function launchSim(){
    homeScreen.classList.add('hidden');
    appScreen.classList.add('active');
    resize(); // canvas has real dimensions only once its container is visible
    if(!started){
      started = true;
      bootSequence();
      requestAnimationFrame(loop);
    }
  }

  btnLaunch.addEventListener('click', launchSim);
  btnHome.addEventListener('click', ()=>{
    appScreen.classList.remove('active');
    homeScreen.classList.remove('hidden');
  });

  /* ============================================================
     METHODOLOGY MODAL — parameters & math, on demand
  ============================================================ */
  const methOverlay = document.getElementById('methodology-overlay');
  const btnMethodology = document.getElementById('btnMethodology');
  const btnMethClose = document.getElementById('btnMethClose');
  if(btnMethodology && methOverlay){
    btnMethodology.addEventListener('click', ()=> methOverlay.classList.add('show'));
    btnMethClose.addEventListener('click', ()=> methOverlay.classList.remove('show'));
    methOverlay.addEventListener('click', (e)=>{
      if(e.target === methOverlay) methOverlay.classList.remove('show');
    });
    document.addEventListener('keydown', (e)=>{
      if(e.key === 'Escape') methOverlay.classList.remove('show');
    });
  }

})();
