# zeph-engine

A simple 3D physics engine written in [Zig](https://ziglang.org/), built for learning and small projects. Not a production engine — just something that works well enough for rigid bodies, cloth, and basic simulations.

Licensed under the [R3D License](LICENSE).

---

## Features

- **Rigid body simulation** — single-joint bodies with mass, elasticity, and friction
- **Cloth / soft body** — joint-connection networks solved with XPBD constraints
- **Collision detection** — uniform spatial grid broadphase (O(n) average), sphere-vs-sphere narrowphase
- **Continuous Collision Detection (CCD)** — integrated into the world step to prevent tunneling at high speeds
- **Sleep system** — bodies deactivate automatically when velocity drops below threshold
- **Custom environment function** — plug in any ground/wall surface via `EnvFunc`
- **Symplectic Euler integration** — stable enough for game-speed timesteps

---

## Limitations

This is a simple engine. It has rough edges:

- No rigid body rotation (single-joint bodies use `FLAG_NONROTATING`)
- Ball-to-ball collision is not handled by the engine itself — must be done manually in the application layer
- No broadphase for cloth vs cloth
- No constraint warmstarting
- Grid overflow silently falls back to O(n²)
- Not tested under heavy load

---

## Project Structure

```
zeph-engine/
├── src/
│   ├── zeph.zig
│   ├── cloth.zig
│   └── box.zig
├── build.zig
└── README.md
```

---

## Quick Start

### Requirements

- Zig `0.15.x` or newer
- [Raylib](https://www.raylib.com/) (for the demo only)

### Build

```bash
zig build
```

### Run the demo

```bash
zig build run
```

**Demo controls:**

| Key / Input | Action |
|---|---|
| `SPACE` | Spawn new ball |
| `R` | Reset scene |
| `1` | Select concrete ball |
| `2` | Select iron ball |
| `3` | Select rubber ball |
| `4` | Select ping-pong ball |
| `Mouse drag` | Orbit camera |
| `Scroll` | Zoom |

---

## Basic Usage

```zig
const zeph = @import("zeph");

// Create joints and body
var joints = [1]zeph.Joint{
    zeph.makeJoint(zeph.mkVec3(0, 3, 0), 0.12), // position + radius
};
var body: zeph.Body = undefined;
zeph.bodyInit(&body, &joints, &.{}, 1.0); // joints, connections, mass
body.elasticity     = 0.6;
body.friction       = 0.5;
body.linear_damping = 0.002;
body.flags |= zeph.FLAG_ALWAYS_ACTIVE | zeph.FLAG_NONROTATING;

// Create world
var world = zeph.World{};
zeph.worldInit(&world, body_slice[0..n], zeph.envGround);
world.gravity = zeph.mkVec3(0, -9.8 / (60.0 * 60.0), 0);

// Step every frame (call at 60 fps)
zeph.worldStep(&world);
```

---

## Body Flags

| Flag | Description |
|---|---|
| `FLAG_NONROTATING` | Treat body as a non-rotating sphere (required for single-joint rigid bodies) |
| `FLAG_ALWAYS_ACTIVE` | Disable sleep for this body |
| `FLAG_SOFT` | Enable soft body / cloth behavior |
| `FLAG_VERLET` | Use Verlet integration instead of symplectic Euler |
| `FLAG_DISABLED` | Exclude body from simulation entirely |
| `FLAG_DEACTIVATED` | Body is sleeping |
| `FLAG_SIMPLE_CONN` | Use simplified connection solver |
| `FLAG_NO_BSPHERE` | Skip bounding sphere check in broadphase |

---

## License

Copyright 2026 danko1122q.  
Licensed under the **R3D License (Rendering Three-Dimensional Library License), Version 1.0**.  
See [LICENSE](LICENSE) for full terms.
