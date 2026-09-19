# 🚁 Drone Crash Simulator — V1.0

A single-page, physics-driven quadcopter simulator that takes off, holds a real
PID-stabilized hover, and — on command or at random — suffers a genuine mechanical
failure and crashes under an actual rigid-body physics simulation. Wrapped around
it is a simulated ROS 1 (Noetic-style) ground-control console, styled like a real
flight-test HUD.

Live demo: *add your Vercel URL here once deployed*

---

## ✨ What it does

- **ARM → TAKEOFF → HOVER → LAND**, driven by a real flight-control loop, not a
  canned animation timeline.
- **SIMULATE FAILURE**, with a dropdown of 15 distinct failure modes (or random),
  each of which breaks the simulated aircraft in a physically different way.
- A **live incident report** generated after every crash — impact velocity,
  G-force, attitude at impact, freefall duration, and a snapshot image of the
  wreck — all computed from the actual simulation state, not scripted numbers.
- A **fake ROS 1 ground-control console** streaming boot logs, node status,
  `rostopic echo`-style telemetry, and `/diagnostics` error messages.
- **Kiosk mode** (real Fullscreen API) for unattended display.
- A wireframe **protective cage** around the airframe that doubles as its actual
  collision hull in the physics simulation.

---

## 🧠 The physics engine

This is the part that makes it more than a pretty animation.

**Engine:** [cannon.js](https://github.com/schteppe/cannon.js) (loaded from a
CDN) — a real 3D rigid-body physics engine, the same class of library used for
game and robotics prototyping.

### The airframe
- Modeled as a rigid body with **mass = 1.2 kg**, gravity = 9.81 m/s².
- Collision geometry is a compound shape: a small box (the chassis) fully
  enclosed inside a **0.35m-radius sphere** — the sphere *is* the protective
  cage, so the cage is what actually absorbs ground impact, not a cosmetic
  overlay.
- Four motors are placed at fixed offsets (±0.16m) from the center of mass in
  the standard **X-quad configuration**.

### Why the roll/pitch/yaw are "real," not scripted
Each motor applies real thrust force at its physical position via
`applyLocalForce`. Because that force acts *away from the center of mass*,
cannon.js's own solver computes the resulting torque as **τ = r × F**
automatically — exactly how a real airframe behaves. Roll and pitch are never
hand-animated; they're a byproduct of applying the right forces in the right
places and letting the physics engine do the rest.

Yaw comes from a second, independent effect: alternating motor spin direction
(CW/CCW diagonal pairs) produces a small reaction torque proportional to each
motor's thrust — the same mechanism real quadcopters use for yaw authority.

### Flight control
A small cascaded control loop holds altitude and attitude, structured the way
a real flight controller's inner/outer loops work:

| Loop | Gains (P / I / D) | Target |
|---|---|---|
| Altitude | 0.65 / 0.08 / 0.85 | commanded hover/target altitude |
| Roll / Pitch | 0.6 / — / 0.15 | 0° (level) |
| Yaw | — / — / 0.02 | angular-rate damping only |

Output from these loops feeds a standard quadcopter **motor mixer**:

```
FL = throttle − roll + pitch + yaw
FR = throttle + roll + pitch − yaw
RL = throttle − roll − pitch − yaw
RR = throttle + roll − pitch + yaw
```

Hover throttle is derived, not guessed:
`HOVER_THROTTLE = (mass × 9.81) / (4 × max_thrust_per_motor)`.

### Failure modes — each one breaks something specific
Fifteen failure modes are modeled, and each corrupts the simulation at a
different, physically appropriate point rather than just triggering a generic
"fall over" animation:

| Category | Examples | What actually happens in the sim |
|---|---|---|
| Motor / ESC | ESC desync, bearing seizure, propeller delamination | One motor's commanded thrust is zeroed or randomized directly in the mixer |
| Structural | Carbon-arm failure, foreign-object strike | A motor is disabled, or a real impulse (`applyImpulse`) is injected |
| Power | LiPo cell collapse, thermal runaway | All four motors' thrust decays exponentially over time |
| Sensors / software | IMU gyro saturation, EKF NaN reset, compass interference | Corrupted values are fed *into the controller's feedback*, so the controller destabilizes itself — the instability is emergent, not scripted |
| Link / control | RC link loss, firmware watchdog reset | The control loop is frozen open-loop; an inherent (unmodeled) airframe asymmetry torque then causes an uncorrected, gradual tumble — just like a real aircraft with no active stabilization |
| Environment | Wind gust exceeding control authority | A real external force + torque is applied directly to the rigid body, which the controller then has to (and fails to) counteract |
| Human factor | Abrupt operator throttle cut | Throttle command is manually forced down |

### The incident report
Generated from real simulation output wherever possible:

- **Impact velocity, freefall duration, attitude at impact** — read directly
  from the physics body via a cannon.js `collide` event.
- **Impact G-force** — estimated as `Δv / (g × 0.05s)`, a standard simplified
  impact-mechanics approximation (assumed 50ms stopping/crumple time). This is
  explicitly labeled as an estimate, not a measured quantity — no crumple/
  deformation model exists.
- **A snapshot image** of the wreck is captured directly from the `<canvas>`
  via `toDataURL()` a moment after impact.
- Battery %, wind speed, RF link quality, and GPS status are **simulated
  telemetry flavor** for realism — not physically modeled. The report says so.

---

## 🛰️ The ROS console

The left/HUD side is real physics. The console on the right is a **styled
narrative simulation**, not a live ROS connection:

- Topic and node names (`/mavros/state`, `/mavros/imu/data`,
  `/ekf_localization`, `/takeoff_controller`, `/diagnostics`, etc.) are real,
  accurate ROS/MAVROS conventions.
- The *values* streamed through them are generated client-side in JavaScript
  and reflect the real simulation state where applicable (e.g. altitude and
  vertical speed in `/mavros/local_position/pose` are the actual physics
  numbers), and are otherwise randomized narrative flavor.
- There is no `roscore`, no real ROS nodes, and no network traffic — everything
  runs client-side in the browser.

---

## 🧰 Tech stack

| Layer | Technology |
|---|---|
| Physics | [cannon.js](https://github.com/schteppe/cannon.js) 0.6.2 (CDN) |
| Rendering | HTML5 Canvas 2D (hand-built ground grid, drone sprite, wireframe cage, particle effects) |
| Logic / UI | Vanilla JavaScript (no framework), HTML, CSS |
| Fonts | Rajdhani + JetBrains Mono (Google Fonts) |

No build step, no bundler, no package manager required to run it — the whole
thing is one `index.html` file plus two CDN script/font includes.

---

## ▶️ Run locally

Just open `index.html` in any modern browser. An internet connection is needed
once, to load cannon.js and the fonts from their CDNs.

## 🚀 Deploy

This is a static site — deploy it as-is on [Vercel](https://vercel.com):

1. Push this repo to GitHub (or upload `index.html` directly through GitHub's
   web UI).
2. On [vercel.com](https://vercel.com), **Add New → Project → Import** this
   repo.
3. Leave all settings at their defaults — no framework, no build command.
4. **Deploy.** You'll have a live URL in seconds, and it auto-redeploys on
   every future push.

---

## ⚠️ Honest scope — what's real vs. simulated

This project is upfront about where the line is:

**Physically simulated (real rigid-body dynamics):**
mass, gravity, thrust, motor-position torque (roll/pitch), spin-reaction
torque (yaw), PID control response, collision detection, impact velocity,
attitude at impact.

**Estimated with a stated formula (not measured):**
impact G-force (`Δv / (g × assumed stopping time)`).

**Simulated for narrative/UI realism only (not physically modeled):**
battery voltage/temperature, wind speed, RF link quality/RSSI, GPS satellite
count, ROS log timing, firmware version strings.

**Not present at all:**
a real ROS installation, real MAVROS/PX4/ArduPilot firmware, a real drone,
and any network communication.

---

## 📄 License

No license file included — add one if you plan to share or reuse this
publicly.
