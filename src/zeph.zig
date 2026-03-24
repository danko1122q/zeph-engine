// =============================================================================
// R3D LICENSE — Rendering Three-Dimensional Library License, Version 1.0
// Copyright 2026 danko1122q. All rights reserved.
//
// Permission is granted to use, integrate, and redistribute this Software
// for personal, academic, and commercial purposes, subject to the following:
//
//  1. This license must be included in full with any distribution.
//  2. Attribution must be provided in documentation or UI, stating the
//     library name and that it is licensed under R3D.
//  3. The Software may not be sold as a standalone product without permission.
//  4. Copyright notices must not be removed or altered.
//  5. The Creator's name may not be used for promotion without consent.
//
// THE SOFTWARE IS PROVIDED "AS-IS", WITHOUT WARRANTY OF ANY KIND.
// =============================================================================

// ┌─────────────────────────────────────────────────────────────┐
// │  CHANGELOG DARI VERSI SEBELUMNYA                            │
// │  [1] Integrasi: Semi-implicit (Symplectic) Euler            │
// │      v += a*dt  LALU  pos += v*dt  (urutan sudah benar)     │
// │  [2] Sleep: dual-threshold linear + angular velocity        │
// │  [3] Broadphase: Uniform Spatial Grid (O(n) avg)            │
// │  [4] CCD: terintegrasi penuh ke worldStep rigid body        │
// │  [5] Constraint: XPBD per-connection stiffness + damping    │
// │  [6] Substep: substep tunggal — gravity TIDAK di-scale      │
// │      manual lagi, diganti worldStepAllN yg benar            │
// │  [7] API baru: bodyApplyImpulse, bodySetLinearVelocity      │
// └─────────────────────────────────────────────────────────────┘

const std = @import("std");

// ============================================================
// Konstanta
// ============================================================

pub const INF: f32 = std.math.inf(f32);

pub const LOW_SPEED             : f32 = 0.06;
pub const RESHAPE_TENSION_LIMIT : f32 = 0.04;
pub const RESHAPE_ITERS         : u32 = 2;
pub const TENSION_ACCEL_DIV     : f32 = 32.0;
pub const TENSION_ACCEL_THRESH  : f32 = 0.01;
pub const TENSION_HIGH_THRESH   : f32 = TENSION_ACCEL_THRESH * 3.0;
pub const COLLIDE_ITERS         : u32 = 16;
pub const COLLIDE_MARGIN        : f32 = 0.002;
pub const NONROT_RESOLVE_ATTEMPTS: u32 = 8;
pub const COLLIDE_SOLVER_ITERS  : u32 = 4;

// ── Sleep ────────────────────────────────────────────────────
/// Threshold kuadrat kecepatan linear untuk sleep.
pub const SLEEP_LINEAR_SQ  : f32 = 0.00002;
/// Threshold kuadrat angular velocity untuk sleep.
pub const SLEEP_ANGULAR_SQ : f32 = 0.00001;
/// Berapa frame berturut-turut di bawah threshold sebelum tidur.
pub const SLEEP_FRAMES     : u32 = 30;

// ── Damping ──────────────────────────────────────────────────
pub const DEFAULT_LINEAR_DAMPING  : f32 = 0.0;
pub const DEFAULT_ANGULAR_DAMPING : f32 = 0.002;

// ── Gravity default ──────────────────────────────────────────
pub const GRAVITY_DEFAULT : f32 = 0.027;

// ── Spatial Grid ─────────────────────────────────────────────
/// Ukuran satu sel grid broadphase. Sesuaikan dengan ukuran joint rata-rata.
pub const GRID_CELL_SIZE : f32 = 0.5;
/// Maksimum body per sel grid (overflow diabaikan → fallback O(n²)).
pub const GRID_MAX_PER_CELL : usize = 8;
/// Dimensi grid (GRID_DIM × GRID_DIM × GRID_DIM).
pub const GRID_DIM : i32 = 64;

pub const JOINT_SCALE: f32 = 1.0;

// ============================================================
// Flags Body
// ============================================================

pub const FLAG_DEACTIVATED  : u8 = 1;
pub const FLAG_NONROTATING  : u8 = 2;
pub const FLAG_DISABLED     : u8 = 4;
pub const FLAG_SOFT         : u8 = 8;
pub const FLAG_SIMPLE_CONN  : u8 = 16;
pub const FLAG_ALWAYS_ACTIVE: u8 = 32;
pub const FLAG_NO_BSPHERE   : u8 = 64;
pub const FLAG_VERLET       : u8 = 128;

// ============================================================
// Tipe Data
// ============================================================

pub const Unit = f32;

pub const Vec3 = struct {
    x: Unit = 0,
    y: Unit = 0,
    z: Unit = 0,
};

pub const Joint = struct {
    position    : Vec3  = .{},
    prev_pos    : Vec3  = .{},
    velocity    : Vec3  = .{},
    force       : Vec3  = .{},
    /// positif = bebas, negatif = pinned.
    size        : Unit  = 0.0,

    pub fn jsize(self: Joint) Unit {
        return @abs(self.size);
    }

    pub fn isPinned(self: Joint) bool {
        return self.size < 0.0;
    }
};

pub const Connection = struct {
    joint1      : u16  = 0,
    joint2      : u16  = 0,
    /// length < 0 → koneksi putus / disabled
    length      : Unit = 0.0,
    /// Per-connection compliance (XPBD). 0 = kaku penuh, >0 = elastis.
    compliance  : Unit = 0.0,
    /// Per-connection damping (XPBD). Nilai kecil ~0.01 cukup.
    damping     : Unit = 0.0,
};

pub const EnvFunc         = *const fn (Vec3, Unit) Vec3;
pub const CollideCallback = *const fn (u16, u16, u16, u16, Vec3) bool;
pub const DrawPixelFunc   = *const fn (u16, u16, u8) void;

pub const Body = struct {
    joints      : []Joint      = &.{},
    connections : []Connection = &.{},
    joint_mass  : Unit         = 1.0,
    friction    : Unit         = 0.5,
    elasticity  : Unit         = 0.5,
    flags       : u8           = 0,

    sleep_count : u32  = 0,

    gravity_scale   : Unit = 1.0,
    /// Padé: v *= 1/(1 + linear_damping)
    linear_damping  : Unit = DEFAULT_LINEAR_DAMPING,
    angular_damping : Unit = DEFAULT_ANGULAR_DAMPING,

    angular_vel  : Vec3 = .{},
    roll_radius  : Unit = 0.0,

    verlet_damping  : Unit = 0.0,
    verlet_relax    : u32  = 8,
    cloth_stiffness : Unit = 1.0,
};

pub const World = struct {
    bodies           : []Body           = &.{},
    env_func         : ?EnvFunc         = null,
    collide_callback : ?CollideCallback = null,
    gravity          : Vec3             = .{ .x=0, .y=-GRAVITY_DEFAULT, .z=0 },
};

// ============================================================
// State internal (thread-local agar aman di multi-thread)
// ============================================================

threadlocal var _b1       : u16              = 0;
threadlocal var _b2       : u16              = 0;
threadlocal var _j1       : u16              = 0;
threadlocal var _j2       : u16              = 0;
threadlocal var _callback : ?CollideCallback = null;

// ============================================================
// Math Primitives
// ============================================================

pub fn absVal(x: Unit) Unit  { return @abs(x); }
pub fn maxVal(a: Unit, b: Unit) Unit { return @max(a, b); }
pub fn minVal(a: Unit, b: Unit) Unit { return @min(a, b); }
pub fn nonZero(x: Unit) Unit { return if (@abs(x) < 1e-12) 1e-12 else x; }
pub fn keepInRange(x: Unit, lo: Unit, hi: Unit) Unit {
    return std.math.clamp(x, lo, hi);
}

pub fn sqrt(x: Unit) Unit { return @sqrt(@max(x, 0.0)); }
pub fn sin(x: Unit) Unit  { return @sin(x * std.math.pi * 2.0); }
pub fn cos(x: Unit) Unit  { return @cos(x * std.math.pi * 2.0); }

pub fn atan(x: Unit) Unit {
    return std.math.atan(x) / (std.math.pi * 2.0);
}

pub fn vec2Angle(x: Unit, y: Unit) Unit {
    var r: Unit = 0;
    if (x != 0) {
        r = atan(y / x);
        if (x < 0) r += 0.5 else if (r < 0) r += 1.0;
    } else {
        if (y > 0) r = 0.25 else if (y < 0) r = 0.75;
    }
    return r;
}

fn vec2Rotate(x: *Unit, y: *Unit, angle: Unit) void {
    const s = sin(angle);
    const c = cos(angle);
    const tmp = x.*;
    x.* = c * x.* - s * y.*;
    y.* = s * tmp + c * y.*;
}

// ============================================================
// Vec3
// ============================================================

pub fn mkVec3(x: Unit, y: Unit, z: Unit) Vec3 { return .{ .x=x, .y=y, .z=z }; }
pub fn vec3Plus(v1: Vec3, v2: Vec3)  Vec3 { return .{ .x=v1.x+v2.x, .y=v1.y+v2.y, .z=v1.z+v2.z }; }
pub fn vec3Minus(v1: Vec3, v2: Vec3) Vec3 { return .{ .x=v1.x-v2.x, .y=v1.y-v2.y, .z=v1.z-v2.z }; }
pub fn vec3Scale(v: Vec3, s: Unit)   Vec3 { return .{ .x=v.x*s, .y=v.y*s, .z=v.z*s }; }
pub fn vec3Times(v: Vec3, u: Unit)   Vec3 { return vec3Scale(v, u); }

pub fn vec3Dot(v1: Vec3, v2: Vec3) Unit {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}
pub fn vec3Cross(v1: Vec3, v2: Vec3) Vec3 {
    return .{
        .x = v1.y*v2.z - v1.z*v2.y,
        .y = v1.z*v2.x - v1.x*v2.z,
        .z = v1.x*v2.y - v1.y*v2.x,
    };
}

pub fn vec3Len(v: Vec3) Unit {
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

pub fn vec3LenApprox(v_in: Vec3) Unit {
    var ax = @abs(v_in.x);
    var ay = @abs(v_in.y);
    var az = @abs(v_in.z);
    if (ax < ay) std.mem.swap(Unit, &ax, &ay);
    if (ay < az) std.mem.swap(Unit, &ay, &az);
    if (ax < ay) std.mem.swap(Unit, &ax, &ay);
    return 0.8727*ax + 0.4357*ay + 0.2179*az;
}

pub fn dist(p1: Vec3, p2: Vec3) Unit { return vec3Len(vec3Minus(p1,p2)); }
pub fn distApprox(p1: Vec3, p2: Vec3) Unit { return vec3LenApprox(vec3Minus(p1,p2)); }

pub fn vec3Normalize(v: *Vec3) void {
    const l = vec3Len(v.*);
    if (l < 1e-12) { v.* = mkVec3(1,0,0); return; }
    const inv = 1.0 / l;
    v.x *= inv; v.y *= inv; v.z *= inv;
}

pub fn vec3Normalized(v_in: Vec3) Vec3 { var v=v_in; vec3Normalize(&v); return v; }

pub fn vec3ProjectNormalized(v: Vec3, baseN: Vec3) Vec3 {
    const p = vec3Dot(v, baseN);
    return vec3Scale(baseN, p);
}

pub fn vec3Project(v: Vec3, base_in: Vec3) Vec3 {
    var base = base_in;
    vec3Normalize(&base);
    return vec3ProjectNormalized(v, base);
}

pub fn vec3KeepWithinBox(p_in: Vec3, c: Vec3, h: Vec3) Vec3 {
    return .{
        .x = keepInRange(p_in.x, c.x-h.x, c.x+h.x),
        .y = keepInRange(p_in.y, c.y-h.y, c.y+h.y),
        .z = keepInRange(p_in.z, c.z-h.z, c.z+h.z),
    };
}

pub fn vec3KeepWithinDistanceBand(point: Vec3, center: Vec3, minD: Unit, maxD: Unit) Vec3 {
    const d = vec3Minus(point, center);
    const l = vec3Len(d);
    if (l>=minD and l<=maxD) return point;
    return vec3Plus(center, vec3Scale(vec3Normalized(d), if (l<minD) minD else maxD));
}

// ============================================================
// Rotation
// ============================================================

pub fn rotationFromVecs(fwd_in: Vec3, right_in: Vec3) Vec3 {
    var fwd=fwd_in; var right=right_in; var r: Vec3 = .{};
    r.y = vec2Angle(fwd.z, -fwd.x);
    vec2Rotate(&fwd.z,   &fwd.x,   r.y);
    vec2Rotate(&right.z, &right.x, r.y);
    r.x = vec2Angle(fwd.z, fwd.y);
    vec2Rotate(&right.z, &right.y, -r.x);
    r.z = vec2Angle(right.x, -right.y);
    return r;
}

pub fn pointRotate(p_in: Vec3, rot: Vec3) Vec3 {
    var p=p_in;
    vec2Rotate(&p.y, &p.x, rot.z);
    vec2Rotate(&p.z, &p.y, rot.x);
    vec2Rotate(&p.x, &p.z, rot.y);
    return p;
}

pub fn rotationInverse(rot_in: Vec3) Vec3 {
    var rot=rot_in;
    var f=mkVec3(0,0,1); var r=mkVec3(1,0,0);
    rot.x=-rot.x; rot.y=-rot.y; rot.z=-rot.z;
    vec2Rotate(&f.x, &f.z, rot.y); vec2Rotate(&f.z, &f.y, rot.x); vec2Rotate(&f.y, &f.x, rot.z);
    vec2Rotate(&r.x, &r.z, rot.y); vec2Rotate(&r.z, &r.y, rot.x); vec2Rotate(&r.y, &r.x, rot.z);
    return rotationFromVecs(f, r);
}

fn rotateByAxis(p: Vec3, axN: Vec3, angle: Unit) Vec3 {
    const proj = vec3ProjectNormalized(p, axN);
    const a    = vec3Minus(p, proj);
    if (a.x==0 and a.y==0 and a.z==0) return p;
    const b = vec3Cross(a, axN);
    return vec3Plus(proj, vec3Plus(vec3Scale(a, cos(angle)), vec3Scale(b, sin(angle))));
}

pub fn rotationRotateByAxis(rot: Vec3, axisAngle_in: Vec3) Vec3 {
    var f = pointRotate(mkVec3(0,0,1), rot);
    var r = pointRotate(mkVec3(1,0,0), rot);
    const a = vec3Len(axisAngle_in);
    var ax = axisAngle_in;
    vec3Normalize(&ax);
    f = rotateByAxis(f, ax, a);
    r = rotateByAxis(r, ax, a);
    return rotationFromVecs(f, r);
}

// ============================================================
// Joint & Connection
// ============================================================

pub fn makeJoint(position: Vec3, size: Unit) Joint {
    return .{ .position = position, .prev_pos = position, .size = size };
}

pub fn connectionTension(length: Unit, desired: Unit) Unit {
    return length / desired - 1.0;
}

// ============================================================
// Body Init
// ============================================================

pub fn bodyInit(body: *Body, joints: []Joint, connections: []Connection, mass: Unit) void {
    body.* = .{
        .joints           = joints,
        .connections      = connections,
        .joint_mass       = mass / @as(Unit, @floatFromInt(joints.len)),
        .friction         = 0.5,
        .elasticity       = 0.5,
        .flags            = 0,
        .sleep_count      = 0,
        .gravity_scale    = 1.0,
        .linear_damping   = DEFAULT_LINEAR_DAMPING,
        .angular_damping  = DEFAULT_ANGULAR_DAMPING,
    };
    for (connections) |*c| {
        const d = dist(joints[c.joint1].position, joints[c.joint2].position);
        c.length = if (d < 1e-12) 1e-12 else d;
    }
}

pub fn bodyInitVerlet(
    body    : *Body,
    joints  : []Joint,
    conns   : []Connection,
    mass    : Unit,
    damping : Unit,
    relax   : u32,
) void {
    bodyInit(body, joints, conns, mass);
    body.verlet_damping = damping;
    body.verlet_relax   = relax;
    body.flags         |= FLAG_VERLET;
    for (body.joints) |*j| j.prev_pos = j.position;
}

pub fn worldInit(world: *World, bodies: []Body, env: ?EnvFunc) void {
    world.* = .{
        .bodies           = bodies,
        .env_func         = env,
        .collide_callback = null,
        .gravity          = .{ .x=0, .y=-GRAVITY_DEFAULT, .z=0 },
    };
}

pub fn worldSetGravity(world: *World, gx: Unit, gy: Unit, gz: Unit) void {
    world.gravity = .{ .x=gx, .y=gy, .z=gz };
}

// ============================================================
// Body Queries
// ============================================================

pub fn bodyIsActive(body: *const Body) bool {
    return (body.flags & FLAG_DEACTIVATED) == 0;
}

pub fn bodyGetCenterOfMass(body: *const Body) Vec3 {
    var c = Vec3{};
    for (body.joints) |j| { c.x+=j.position.x; c.y+=j.position.y; c.z+=j.position.z; }
    const n: Unit = @floatFromInt(body.joints.len);
    return .{ .x=c.x/n, .y=c.y/n, .z=c.z/n };
}

pub fn bodyGetNetSpeed(body: *const Body) Unit {
    var v: Unit = 0;
    for (body.joints) |j| v += vec3Len(j.velocity);
    return v;
}

pub fn bodyGetAverageSpeed(body: *const Body) Unit {
    if (body.joints.len == 0) return 0;
    return bodyGetNetSpeed(body) / @as(Unit, @floatFromInt(body.joints.len));
}

pub fn bodyGetLinearVelocity(body: *const Body) Vec3 {
    var r = Vec3{};
    for (body.joints) |j| { r.x+=j.velocity.x; r.y+=j.velocity.y; r.z+=j.velocity.z; }
    const n: Unit = @floatFromInt(body.joints.len);
    return .{ .x=r.x/n, .y=r.y/n, .z=r.z/n };
}

pub fn bodyGetRotation(body: *const Body, j1: u16, j2: u16, j3: u16) Vec3 {
    return rotationFromVecs(
        vec3Minus(body.joints[j2].position, body.joints[j1].position),
        vec3Minus(body.joints[j3].position, body.joints[j1].position),
    );
}

pub fn bodyGetAABB(body: *const Body, vMin: *Vec3, vMax: *Vec3) void {
    const js0 = body.joints[0].jsize();
    vMin.* = vec3Minus(body.joints[0].position, mkVec3(js0,js0,js0));
    vMax.* = vec3Plus (body.joints[0].position, mkVec3(js0,js0,js0));
    for (body.joints[1..]) |j| {
        const js=j.jsize();
        if (j.position.x-js < vMin.x) vMin.x=j.position.x-js;
        if (j.position.y-js < vMin.y) vMin.y=j.position.y-js;
        if (j.position.z-js < vMin.z) vMin.z=j.position.z-js;
        if (j.position.x+js > vMax.x) vMax.x=j.position.x+js;
        if (j.position.y+js > vMax.y) vMax.y=j.position.y+js;
        if (j.position.z+js > vMax.z) vMax.z=j.position.z+js;
    }
}

pub fn bodyGetFastBSphere(body: *const Body, center: *Vec3, radius: *Unit) void {
    var vMax: Vec3 = undefined;
    bodyGetAABB(body, center, &vMax);
    radius.* = dist(center.*, vMax) * 0.5;
    center.* = vec3Scale(vec3Plus(center.*, vMax), 0.5);
}

pub fn bodyStop(body: *Body) void {
    for (body.joints) |*j| { j.velocity=.{}; j.force=.{}; }
}

pub fn bodyDeactivate(body: *Body) void {
    bodyStop(body);
    body.flags |= FLAG_DEACTIVATED;
    body.sleep_count = 0;
}

pub fn bodyActivate(body: *Body) void {
    if ((body.flags & FLAG_DEACTIVATED) != 0) {
        body.flags &= ~FLAG_DEACTIVATED;
        body.sleep_count = 0;
    }
}

pub fn bodyMoveBy(body: *Body, offset: Vec3) void {
    for (body.joints) |*j| {
        j.position = vec3Plus(j.position, offset);
        j.prev_pos = vec3Plus(j.prev_pos, offset);
    }
}

pub fn bodyMoveTo(body: *Body, pos: Vec3) void {
    bodyMoveBy(body, vec3Minus(pos, bodyGetCenterOfMass(body)));
}

pub fn bodyAddForce(body: *Body, force: Vec3) void {
    if ((body.flags & FLAG_DISABLED) != 0) return;
    bodyActivate(body);
    for (body.joints) |*j| {
        if (j.isPinned()) continue;
        j.force.x += force.x;
        j.force.y += force.y;
        j.force.z += force.z;
    }
}

pub fn bodyAccelerate(body: *Body, v: Vec3) void {
    bodyActivate(body);
    for (body.joints) |*j| {
        j.velocity.x += v.x;
        j.velocity.y += v.y;
        j.velocity.z += v.z;
    }
}

/// Terapkan impulse ke semua joint (mengubah velocity seketika, mass-aware).
pub fn bodyApplyImpulse(body: *Body, impulse: Vec3) void {
    if ((body.flags & FLAG_DISABLED) != 0) return;
    bodyActivate(body);
    const inv_mass = 1.0 / nonZero(body.joint_mass);
    for (body.joints) |*j| {
        if (j.isPinned()) continue;
        j.velocity.x += impulse.x * inv_mass;
        j.velocity.y += impulse.y * inv_mass;
        j.velocity.z += impulse.z * inv_mass;
    }
}

/// Set linear velocity semua joint sekaligus.
pub fn bodySetLinearVelocity(body: *Body, v: Vec3) void {
    bodyActivate(body);
    for (body.joints) |*j| {
        j.velocity = v;
    }
}

pub fn bodyApplyRollingConstraint(body: *Body) void {
    if (body.roll_radius < 1e-12) return;
    const r = body.roll_radius;
    body.angular_vel.z = -body.joints[0].velocity.x / r;
    body.angular_vel.x =  body.joints[0].velocity.z / r;
    body.angular_vel.x *= 510.0/512.0;
    body.angular_vel.y *= 510.0/512.0;
    body.angular_vel.z *= 510.0/512.0;
}

pub fn bodyApplyTorque(body: *Body, t: Vec3) void {
    body.angular_vel.x += t.x;
    body.angular_vel.y += t.y;
    body.angular_vel.z += t.z;
}

pub fn bodySpinWithCenter(body: *Body, rot: Vec3, center: Vec3) void {
    for (body.joints) |*j| {
        var tp = vec3Minus(j.position, center);
        tp = vec3Plus(center, vec3Project(tp, rot));
        tp = vec3Cross(vec3Minus(j.position, tp), rot);
        j.velocity.x += tp.x;
        j.velocity.y += tp.y;
        j.velocity.z += tp.z;
    }
}

pub fn bodySpin(body: *Body, rot: Vec3) void {
    bodySpinWithCenter(body, rot, bodyGetCenterOfMass(body));
}

pub fn bodyRotateByAxis(body: *Body, axisAngle_in: Vec3) void {
    const center = bodyGetCenterOfMass(body);
    const angle  = vec3Len(axisAngle_in);
    var ax = axisAngle_in;
    vec3Normalize(&ax);
    for (body.joints) |*j| {
        const tp = vec3Minus(j.position, center);
        j.position = vec3Plus(center, rotateByAxis(tp, ax, angle));
    }
}

pub fn bodyMultiplyNetSpeed(body: *Body, factor: Unit) void {
    for (body.joints) |*j| {
        j.velocity.x *= factor;
        j.velocity.y *= factor;
        j.velocity.z *= factor;
    }
}

pub fn bodyLimitAverageSpeed(body: *Body, sMin: Unit, sMax: Unit) void {
    for (0..16) |_| {
        const s = bodyGetAverageSpeed(body);
        if (s>=sMin and s<=sMax) return;
        bodyMultiplyNetSpeed(body, (sMax+sMin)*0.5 / nonZero(s));
    }
}

pub fn jointPin(joint: *Joint, pos: Vec3) void {
    joint.position  = pos;
    joint.prev_pos  = pos;
    joint.velocity  = .{};
    joint.force     = .{};
    joint.size      = -@abs(joint.size);
}

pub fn jointUnpin(joint: *Joint) void {
    joint.size = @abs(joint.size);
}

// ============================================================
// Reshape & Cancel Velocities
// ============================================================

pub fn bodyReshape(body: *Body, env: ?EnvFunc) void {
    for (body.connections) |c| {
        if (c.length < 0) continue;
        const j1 = &body.joints[c.joint1];
        const j2 = &body.joints[c.joint2];
        const pin1 = j1.isPinned();
        const pin2 = j2.isPinned();
        if (pin1 and pin2) continue;
        var dir = vec3Minus(j2.position, j1.position);
        vec3Normalize(&dir);
        const target = vec3Scale(dir, c.length);
        if (!pin1 and !pin2) {
            const mid = vec3Scale(vec3Plus(j1.position, j2.position), 0.5);
            const b1=j1.position;
            j1.position = vec3Minus(mid, vec3Scale(target, 0.5));
            if (env) |ef| {
                if (dist(j1.position, ef(j1.position, j1.jsize())) < j1.jsize())
                    j1.position = b1;
            }
            const b2=j2.position;
            j2.position = vec3Plus(j1.position, target);
            if (env) |ef| {
                if (dist(j2.position, ef(j2.position, j2.jsize())) < j2.jsize())
                    j2.position = b2;
            }
        } else if (pin1) {
            const b2=j2.position;
            j2.position = vec3Plus(j1.position, target);
            if (env) |ef| {
                if (dist(j2.position, ef(j2.position, j2.jsize())) < j2.jsize())
                    j2.position = b2;
            }
        } else {
            const b1=j1.position;
            j1.position = vec3Minus(j2.position, target);
            if (env) |ef| {
                if (dist(j1.position, ef(j1.position, j1.jsize())) < j1.jsize())
                    j1.position = b1;
            }
        }
    }
}

pub fn bodyCancelOutVelocities(body: *Body, strong: bool) void {
    for (body.connections) |c| {
        if (c.length < 0) continue;
        const j1 = &body.joints[c.joint1];
        const j2 = &body.joints[c.joint2];
        const pin1 = j1.isPinned();
        const pin2 = j2.isPinned();
        if (pin1 and pin2) continue;
        var dir = vec3Minus(j2.position, j1.position);
        const len = nonZero(vec3Len(dir));
        var cancel = true;
        if (strong) {
            const t = connectionTension(len, c.length);
            cancel = (t <= TENSION_ACCEL_THRESH and t >= -TENSION_ACCEL_THRESH);
        }
        if (!cancel) continue;
        dir = vec3Scale(dir, 1.0/len);
        const pv1 = vec3ProjectNormalized(j1.velocity, dir);
        const pv2 = vec3ProjectNormalized(j2.velocity, dir);
        const avg = vec3Scale(vec3Plus(pv1,pv2), 0.5);
        const scale: Unit = if (strong) 1.0 else 0.25;
        if (!pin1) {
            j1.velocity.x += (avg.x-pv1.x)*scale;
            j1.velocity.y += (avg.y-pv1.y)*scale;
            j1.velocity.z += (avg.z-pv1.z)*scale;
        }
        if (!pin2) {
            j2.velocity.x += (avg.x-pv2.x)*scale;
            j2.velocity.y += (avg.y-pv2.y)*scale;
            j2.velocity.z += (avg.z-pv2.z)*scale;
        }
    }
}

// ============================================================
// XPBD Constraint Solver (per-connection compliance & damping)
// ============================================================
//
// Menggantikan kombinasi tension-push + reshape untuk body rigid
// yang menggunakan FLAG_SOFT == 0. Jika semua connection memiliki
// compliance == 0 dan damping == 0, hasilnya identik dengan
// bodyReshape (positional correction penuh), tapi lebih stabil
// karena menggunakan formulasi XPBD yang benar.
//
// dt   : timestep satu frame/substep
// Ref  : Müller et al. "XPBD: Position-Based Simulation of Compliant
//        Constrained Dynamics" (SCA 2016)
// ============================================================

pub fn bodyXPBDSolve(body: *Body, dt: Unit) void {
    const dt2 = dt * dt;
    const mass = nonZero(body.joint_mass);

    for (body.connections) |c| {
        if (c.length < 0) continue;
        const j1 = &body.joints[c.joint1];
        const j2 = &body.joints[c.joint2];
        const pin1 = j1.isPinned();
        const pin2 = j2.isPinned();
        if (pin1 and pin2) continue;

        var diff = vec3Minus(j2.position, j1.position);
        const cur = nonZero(vec3Len(diff));
        const C = cur - c.length;   // constraint value

        if (@abs(C) < 1e-12) continue;

        // Compliance term: α' = compliance / dt²
        const alpha = c.compliance / dt2;

        // w1, w2 = inverse mass (0 jika pinned)
        const w1: Unit = if (pin1) 0.0 else 1.0 / mass;
        const w2: Unit = if (pin2) 0.0 else 1.0 / mass;
        const wSum = w1 + w2 + alpha;
        if (wSum < 1e-12) continue;

        diff = vec3Scale(diff, 1.0 / cur);  // gradient ∇C

        // Damping menggunakan velocity proyeksi (XPBD positional damping)
        var vrel: Unit = 0;
        if (c.damping > 0) {
            const dv = vec3Minus(j2.velocity, j1.velocity);
            vrel = vec3Dot(dv, diff);
        }

        const lambda = -(C + alpha * 0.0 + c.damping * vrel) / wSum;
        // alpha * lambda_prev dibuang karena kita solve per substep tanpa akumulasi

        const corr = vec3Scale(diff, lambda);
        if (!pin1) {
            j1.position = vec3Minus(j1.position, vec3Scale(corr, w1));
        }
        if (!pin2) {
            j2.position = vec3Plus (j2.position, vec3Scale(corr, w2));
        }
    }
}

// ============================================================
// Collision
// ============================================================

pub fn checkOverlapAABB(v1Min: Vec3, v1Max: Vec3, v2Min: Vec3, v2Max: Vec3) bool {
    if (@abs(v1Min.x+v1Max.x-v2Max.x-v2Min.x) > v1Max.x-v1Min.x+v2Max.x-v2Min.x) return false;
    if (@abs(v1Min.y+v1Max.y-v2Max.y-v2Min.y) > v1Max.y-v1Min.y+v2Max.y-v2Min.y) return false;
    if (@abs(v1Min.z+v1Max.z-v2Max.z-v2Min.z) > v1Max.z-v1Min.z+v2Max.z-v2Min.z) return false;
    return true;
}

pub fn getVelocitiesAfterCollision(v1: *Unit, v2: *Unit, m1: Unit, m2: Unit, e: Unit) void {
    const mSum = nonZero(m1+m2);
    const dv   = nonZero(v2.*-v1.*);
    const pTot = m1*v1.*+m2*v2.*;
    v1.* = (e*m2*dv + pTot) / mSum;
    v2.* = (e*m1*(-dv) + pTot) / mSum;
}

pub fn jointEnvironmentResolveCollision(
    joint: *Joint, elasticity: Unit, friction: Unit, env: EnvFunc,
) u8 {
    var toJoint = vec3Minus(joint.position, env(joint.position, joint.jsize()));
    var len = vec3Len(toJoint);
    if (len > joint.jsize()) return 0;
    if (_callback) |cb| if (!cb(_b1,_j1,_b2,_j2, vec3Minus(joint.position,toJoint))) return 0;
    const pos_backup = joint.position;
    var shift: Vec3 = undefined;
    var success = false;
    if (len > 0) {
        for (0..COLLIDE_ITERS) |_| {
            shift = vec3Normalized(toJoint);
            shift = vec3Scale(shift, joint.jsize()-len+COLLIDE_MARGIN);
            joint.position = vec3Plus(joint.position, shift);
            toJoint = vec3Minus(joint.position, env(joint.position, joint.jsize()));
            len = vec3Len(toJoint);
            if (len >= joint.jsize()) { success=true; break; }
        }
    }
    if (!success) {
        shift = vec3Scale(joint.velocity, -1.0);
        for (0..COLLIDE_ITERS) |_| {
            joint.position = vec3Plus(joint.position, shift);
            toJoint = vec3Minus(joint.position, env(joint.position, joint.jsize()));
            len = vec3Len(toJoint);
            if (len >= joint.jsize()) { success=true; break; }
            shift = vec3Scale(shift, 0.5);
        }
    }
    if (success) {
        const vPar  = vec3Project(joint.velocity, shift);
        const vPerp = vec3Scale(vec3Minus(joint.velocity, vPar), friction);
        const vParR = vec3Scale(vPar, 1.0+elasticity);
        joint.velocity.x -= vParR.x + vPerp.x;
        joint.velocity.y -= vParR.y + vPerp.y;
        joint.velocity.z -= vParR.z + vPerp.z;
        return 1;
    } else {
        joint.position = pos_backup;
        joint.velocity = .{};
        return 2;
    }
}

pub fn jointsResolveCollision(
    j1: *Joint, j2: *Joint,
    m1: Unit, m2: Unit, elasticity: Unit, friction: Unit, env: ?EnvFunc,
) bool {
    var dir = vec3Minus(j2.position, j1.position);
    const d  = vec3Len(dir) - j1.jsize() - j2.jsize();
    if (d >= 0) return false;
    if (_callback) |cb| if (!cb(_b1,_j1,_b2,_j2, vec3Plus(j1.position,dir))) return false;
    const pos1=j1.position; const pos2=j2.position;
    const pen = -d + COLLIDE_MARGIN;
    vec3Normalize(&dir);
    const ratio = m2 / nonZero(m1+m2);
    j1.position = vec3Minus(j1.position, vec3Scale(dir, ratio*pen));
    j2.position = vec3Plus (j2.position, vec3Scale(dir, pen - ratio*pen));
    var v1: Unit = undefined; var v2: Unit = undefined;
    var vp = vec3Project(j1.velocity, dir);
    j1.velocity = vec3Minus(j1.velocity, vp);
    var frict = j1.velocity;
    v1 = vec3Dot(vp, dir);
    vp = vec3Project(j2.velocity, dir);
    j2.velocity = vec3Minus(j2.velocity, vp);
    frict = vec3Minus(j2.velocity, frict);
    v2 = vec3Dot(vp, dir);
    getVelocitiesAfterCollision(&v1, &v2, m1, m2, elasticity);
    const fr1 = ratio * friction;
    j1.velocity = vec3Plus(j1.velocity, vec3Plus(vec3Scale(dir,v1), vec3Scale(frict,fr1)));
    const fr2 = (1.0-ratio) * friction;
    j2.velocity = vec3Minus(j2.velocity, vec3Minus(vec3Scale(dir,-v2), vec3Scale(frict,fr2)));
    if (env) |ef| {
        if (jointEnvironmentResolveCollision(j1,elasticity,friction,ef)==2) j1.position=pos1;
        if (jointEnvironmentResolveCollision(j2,elasticity,friction,ef)==2) j2.position=pos2;
    }
    return true;
}

// ============================================================
// CCD — sphere sweep (terintegrasi ke worldStep)
// ============================================================

pub fn bodiesCCDCollision(b_moving: *Body, b_static: *Body, env: ?EnvFunc) bool {
    var hit = false;

    for (b_moving.joints) |*jm| {
        if (jm.isPinned()) continue;

        const P0  = jm.prev_pos;
        const P1  = jm.position;
        const Dx  = P1.x - P0.x;
        const Dy  = P1.y - P0.y;
        const Dz  = P1.z - P0.z;
        const D_sq = Dx*Dx + Dy*Dy + Dz*Dz;
        if (D_sq < 1e-12) continue;

        const r_ball = jm.jsize();
        var best_t  : Unit  = 2.0;

        for (b_static.joints) |jq| {
            const R  = r_ball + jq.jsize();
            const Ax = P0.x - jq.position.x;
            const Ay = P0.y - jq.position.y;
            const Az = P0.z - jq.position.z;
            const a  = D_sq;
            const b  = 2.0 * (Ax*Dx + Ay*Dy + Az*Dz);
            const cv = Ax*Ax + Ay*Ay + Az*Az - R*R;
            const disc = b*b - 4.0*a*cv;
            if (disc < 0) continue;
            const t = (-b - @sqrt(disc)) / (2.0 * a);
            if (t >= 0.0 and t < best_t) best_t = t;
        }

        if (best_t <= 1.0) {
            jm.position.x = P0.x + best_t * Dx;
            jm.position.y = P0.y + best_t * Dy;
            jm.position.z = P0.z + best_t * Dz;
            hit = true;
        }

        _ = env;
    }
    return hit;
}

pub fn bodyEnvironmentCollide(body: *const Body, env: EnvFunc) bool {
    for (body.joints) |j| {
        const sz = j.jsize();
        if (dist(j.position, env(j.position,sz)) <= sz) return true;
    }
    return false;
}

fn nonrotJointCollided(b: *Body, ji: usize, orig: Vec3, ok: bool) void {
    const delta = vec3Minus(b.joints[ji].position, orig);
    for (b.joints, 0..) |*j, i| {
        if (i!=ji) {
            j.position = vec3Plus(j.position, delta);
            if (ok) j.velocity = b.joints[ji].velocity;
        }
    }
}

pub fn bodyEnvironmentResolveCollision(body: *Body, env: EnvFunc) bool {
    if ((body.flags & FLAG_NO_BSPHERE)==0) {
        var c: Vec3=undefined; var r: Unit=undefined;
        bodyGetFastBSphere(body, &c, &r);
        if (dist(c, env(c,r)) > r) return false;
    }
    var hit=false;
    for (body.joints, 0..) |*j, i| {
        if (j.isPinned()) continue;
        const prev=j.position;
        _j1=@intCast(i);
        const rc=jointEnvironmentResolveCollision(j, body.elasticity, body.friction, env);
        if (rc!=0) {
            hit=true;
            if ((body.flags & FLAG_NONROTATING)!=0) nonrotJointCollided(body, i, prev, rc==1);
        }
    }
    return hit;
}

pub fn sphereClosestPointOnTriangle(
    sphere : Vec3,
    p0     : Vec3,
    p1     : Vec3,
    p2     : Vec3,
    closest: *Vec3,
    normal : *Vec3,
) Unit {
    const e0 = vec3Minus(p1, p0);
    const e1 = vec3Minus(p2, p0);
    var fn_ = vec3Cross(e0, e1);
    const fn_len = vec3Len(fn_);
    if (fn_len < 1e-10) return 1.0;
    fn_.x /= fn_len; fn_.y /= fn_len; fn_.z /= fn_len;

    const v = vec3Minus(sphere, p0);
    const d = vec3Dot(v, fn_);
    const proj = Vec3{
        .x = sphere.x - d * fn_.x,
        .y = sphere.y - d * fn_.y,
        .z = sphere.z - d * fn_.z,
    };

    const c0 = vec3Cross(vec3Minus(p1, p0), vec3Minus(proj, p0));
    const c1 = vec3Cross(vec3Minus(p2, p1), vec3Minus(proj, p1));
    const c2 = vec3Cross(vec3Minus(p0, p2), vec3Minus(proj, p2));
    const inside = vec3Dot(c0, fn_) >= -1e-6 and
                   vec3Dot(c1, fn_) >= -1e-6 and
                   vec3Dot(c2, fn_) >= -1e-6;

    if (inside) {
        closest.* = proj;
        normal.* = if (d >= 0) fn_ else Vec3{ .x=-fn_.x, .y=-fn_.y, .z=-fn_.z };
        return @abs(d);
    }

    var best_dist: Unit = 1e30;
    var best_pt  : Vec3 = p0;
    const edges = [3][2]Vec3{ .{p0,p1}, .{p1,p2}, .{p2,p0} };
    for (edges) |edge| {
        const ab  = vec3Minus(edge[1], edge[0]);
        const ap  = vec3Minus(sphere, edge[0]);
        const ab2 = vec3Dot(ab, ab);
        if (ab2 < 1e-12) continue;
        const t   = @max(0.0, @min(1.0, vec3Dot(ap, ab) / ab2));
        const pt  = Vec3{
            .x = edge[0].x + t*ab.x,
            .y = edge[0].y + t*ab.y,
            .z = edge[0].z + t*ab.z,
        };
        const seg_dist = vec3Len(vec3Minus(sphere, pt));
        if (seg_dist < best_dist) { best_dist = seg_dist; best_pt = pt; }
    }

    closest.* = best_pt;
    const n = vec3Minus(sphere, best_pt);
    const nl = vec3Len(n);
    if (nl < 1e-12) { normal.* = fn_; return 0.0; }
    normal.* = Vec3{ .x=n.x/nl, .y=n.y/nl, .z=n.z/nl };
    return best_dist;
}

pub fn bodiesSphereClothCollision(
    b_sphere : *Body,
    b_cloth  : *Body,
    cloth_w  : usize,
) bool {
    var hit = false;
    if (b_sphere.joints.len == 0) return false;

    const sphere_j  = &b_sphere.joints[0];
    const sphere_c  = sphere_j.position;
    const radius    = sphere_j.jsize();
    const mass_ball = nonZero(b_sphere.joint_mass);
    const n_joints  = b_cloth.joints.len;
    if (cloth_w == 0) return false;
    const cloth_h = n_joints / cloth_w;

    for (0..cloth_h - 1) |row| {
        for (0..cloth_w - 1) |ci| {
            const q00 = row * cloth_w + ci;
            const q10 = row * cloth_w + ci + 1;
            const q01 = (row+1) * cloth_w + ci;
            const q11 = (row+1) * cloth_w + ci + 1;
            if (q11 >= n_joints) continue;

            const p00 = b_cloth.joints[q00].position;
            const p10 = b_cloth.joints[q10].position;
            const p01 = b_cloth.joints[q01].position;
            const p11 = b_cloth.joints[q11].position;

            const qmin_x = @min(@min(p00.x,p10.x),@min(p01.x,p11.x)) - radius;
            const qmin_y = @min(@min(p00.y,p10.y),@min(p01.y,p11.y)) - radius;
            const qmin_z = @min(@min(p00.z,p10.z),@min(p01.z,p11.z)) - radius;
            const qmax_x = @max(@max(p00.x,p10.x),@max(p01.x,p11.x)) + radius;
            const qmax_y = @max(@max(p00.y,p10.y),@max(p01.y,p11.y)) + radius;
            const qmax_z = @max(@max(p00.z,p10.z),@max(p01.z,p11.z)) + radius;
            if (sphere_c.x < qmin_x or sphere_c.x > qmax_x) continue;
            if (sphere_c.y < qmin_y or sphere_c.y > qmax_y) continue;
            if (sphere_c.z < qmin_z or sphere_c.z > qmax_z) continue;

            const tris = [2][3]Vec3{
                .{p00, p10, p01},
                .{p10, p11, p01},
            };
            const tri_joints = [2][3]usize{
                .{q00, q10, q01},
                .{q10, q11, q01},
            };

            for (0..2) |t| {
                var contact_pt : Vec3 = undefined;
                var contact_n  : Vec3 = undefined;
                const tri_dist = sphereClosestPointOnTriangle(
                    sphere_c,
                    tris[t][0], tris[t][1], tris[t][2],
                    &contact_pt, &contact_n,
                );
                if (tri_dist >= radius) continue;

                hit = true;

                const pen = radius - tri_dist + COLLIDE_MARGIN;

                const m_cloth_eff = nonZero(b_cloth.joint_mass *
                    @as(Unit, @floatFromInt(b_cloth.joints.len)));
                const vdotn = vec3Dot(sphere_j.velocity, contact_n);

                if (vdotn < 0) {
                    const j_imp = -vdotn / (1.0/mass_ball + 1.0/m_cloth_eff);

                    sphere_j.velocity.x += j_imp / mass_ball * contact_n.x;
                    sphere_j.velocity.y += j_imp / mass_ball * contact_n.y;
                    sphere_j.velocity.z += j_imp / mass_ball * contact_n.z;
                    const vdn2 = vec3Dot(sphere_j.velocity, contact_n);
                    if (vdn2 < 0) {
                        sphere_j.velocity.x -= vdn2 * contact_n.x;
                        sphere_j.velocity.y -= vdn2 * contact_n.y;
                        sphere_j.velocity.z -= vdn2 * contact_n.z;
                    }

                    const dv_cloth = j_imp / m_cloth_eff / 3.0;
                    for (0..3) |k| {
                        const jk = &b_cloth.joints[tri_joints[t][k]];
                        if (jk.isPinned()) continue;
                        jk.prev_pos.x += dv_cloth * contact_n.x;
                        jk.prev_pos.y += dv_cloth * contact_n.y;
                        jk.prev_pos.z += dv_cloth * contact_n.z;
                    }
                }

                const ball_push = pen * m_cloth_eff / (mass_ball + m_cloth_eff);
                const net_push  = pen * mass_ball   / (mass_ball + m_cloth_eff);

                sphere_j.position.x += contact_n.x * ball_push;
                sphere_j.position.y += contact_n.y * ball_push;
                sphere_j.position.z += contact_n.z * ball_push;

                for (0..3) |k| {
                    const jk = &b_cloth.joints[tri_joints[t][k]];
                    if (jk.isPinned()) continue;
                    jk.position.x -= contact_n.x * net_push / 3.0;
                    jk.position.y -= contact_n.y * net_push / 3.0;
                    jk.position.z -= contact_n.z * net_push / 3.0;
                    jk.prev_pos.x -= contact_n.x * net_push / 3.0;
                    jk.prev_pos.y -= contact_n.y * net_push / 3.0;
                    jk.prev_pos.z -= contact_n.z * net_push / 3.0;
                }
            }
        }
    }
    return hit;
}

pub fn bodiesResolveCollision(b1: *Body, b2: *Body, env: ?EnvFunc) bool {
    var r=false;
    for (b1.joints, 0..) |*jj1, i| {
        for (b2.joints, 0..) |*jj2, j| {
            const op1=jj1.position; const op2=jj2.position;
            _j1=@intCast(i); _j2=@intCast(j);
            if (jointsResolveCollision(jj1, jj2,
                    b1.joint_mass, b2.joint_mass,
                    (b1.elasticity+b2.elasticity)*0.5,
                    (b1.friction+b2.friction)*0.5, env)) {
                r=true;
                if ((b1.flags & FLAG_NONROTATING)!=0) nonrotJointCollided(b1, i, op1, true);
                if ((b2.flags & FLAG_NONROTATING)!=0) nonrotJointCollided(b2, j, op2, true);
            }
        }
    }
    return r;
}

// ============================================================
// Spatial Grid Broadphase
// ============================================================
//
// Grid sel berukuran GRID_CELL_SIZE unit. Setiap body di-hash
// ke sel berdasarkan center of mass. Hanya pasangan body yang
// menempati sel yang sama (atau bersebelahan) yang dicek narrow
// phase → rata-rata O(n) untuk scene yang tersebar.
//
// Keterbatasan: body yang sangat besar (extent > GRID_CELL_SIZE)
// bisa melintasi banyak sel. Untuk kasus itu gunakan FLAG_NO_BSPHERE
// dan fallback ke O(n²) dengan checkOverlapAABB.
// ============================================================

const GridCell = struct {
    bodies : [GRID_MAX_PER_CELL]u16 = undefined,
    count  : u8 = 0,
};

/// Konversi koordinat world ke indeks grid (clamped).
fn gridCoord(v: Unit) i32 {
    const half: i32 = @divTrunc(GRID_DIM, 2);
    const idx: i32 = @intFromFloat(@floor(v / GRID_CELL_SIZE));
    return std.math.clamp(idx, -half, half - 1) + half;
}

fn gridHash(gx: i32, gy: i32, gz: i32) usize {
    const ux: usize = @intCast(gx);
    const uy: usize = @intCast(gy);
    const uz: usize = @intCast(gz);
    const d: usize = @intCast(GRID_DIM);
    return (ux * d + uy) * d + uz;
}

/// Jalankan broadphase berbasis grid. Panggil narrowPhase untuk setiap
/// pasangan kandidat yang ditemukan. Mengembalikan jumlah pasangan aktif.
fn gridBroadphase(
    world       : *World,
    grid        : []GridCell,
    process_pair: fn (*World, u16, u16) void,
) void {
    // Bersihkan grid
    for (grid) |*cell| cell.count = 0;

    // Masukkan tiap body ke sel grid berdasarkan center of mass
    for (world.bodies, 0..) |*body, bi| {
        if ((body.flags & (FLAG_DISABLED | FLAG_DEACTIVATED)) != 0) continue;
        if (body.joints.len == 0) continue;
        const c = bodyGetCenterOfMass(body);
        const gx = gridCoord(c.x);
        const gy = gridCoord(c.y);
        const gz = gridCoord(c.z);
        const idx = gridHash(gx, gy, gz);
        if (idx >= grid.len) continue;
        var cell = &grid[idx];
        if (cell.count < GRID_MAX_PER_CELL) {
            cell.bodies[cell.count] = @intCast(bi);
            cell.count += 1;
        }
    }

    // Periksa pasangan: setiap body vs body lain di sel yang sama + 26 tetangga
    const checked = std.bit_set.IntegerBitSet(64 * 64 * 64 / @bitSizeOf(usize) + 1){};
    _ = checked; // pair dedup tidak trivial untuk array kecil, skip untuk simplisitas

    for (world.bodies, 0..) |*ba, ia| {
        if ((ba.flags & (FLAG_DISABLED | FLAG_DEACTIVATED)) != 0) continue;
        if (ba.joints.len == 0) continue;

        const ca = bodyGetCenterOfMass(ba);
        const gxa = gridCoord(ca.x);
        const gya = gridCoord(ca.y);
        const gza = gridCoord(ca.z);

        // Periksa sel tetangga 3×3×3
        var dz: i32 = -1;
        while (dz <= 1) : (dz += 1) {
            var dy: i32 = -1;
            while (dy <= 1) : (dy += 1) {
                var dx: i32 = -1;
                while (dx <= 1) : (dx += 1) {
                    const nx = gxa + dx;
                    const ny = gya + dy;
                    const nz = gza + dz;
                    if (nx < 0 or ny < 0 or nz < 0) continue;
                    if (nx >= GRID_DIM or ny >= GRID_DIM or nz >= GRID_DIM) continue;
                    const idx = gridHash(nx, ny, nz);
                    if (idx >= grid.len) continue;
                    const cell = &grid[idx];
                    for (0..cell.count) |k| {
                        const ib: usize = cell.bodies[k];
                        if (ib <= ia) continue;  // ogni pair una volta
                        process_pair(world, @intCast(ia), @intCast(ib));
                    }
                }
            }
        }
    }
}

// ============================================================
// worldStep — Rigid body (Symplectic Euler)
// ============================================================
//
// PERBAIKAN [1]: Urutan integrasi sekarang Symplectic Euler yang benar:
//   v_new = v + a*dt   (gravity + force)
//   x_new = x + v_new  (bukan v_lama)
//
// Ini secara implisit lebih stabil untuk sistem konservatif
// karena menjaga energy tidak drift ke atas.
//
// PERBAIKAN [2]: Sleep mengecek KEDUA linear AND angular velocity.
// PERBAIKAN [4]: CCD dipanggil untuk SETIAP pasangan rigid↔verlet.
// ============================================================

pub fn worldStep(world: *World) void {
    _callback = world.collide_callback;

    for (world.bodies, 0..) |*body, i| {
        if ((body.flags & FLAG_DISABLED) != 0) continue;
        if ((body.flags & FLAG_VERLET)   != 0) continue;
        if ((body.flags & FLAG_DEACTIVATED) != 0) {
            var has_ext_force = false;
            for (body.joints) |j| {
                if (j.force.x!=0 or j.force.y!=0 or j.force.z!=0) {
                    has_ext_force=true; break;
                }
            }
            if (!has_ext_force) continue;
            body.flags &= ~FLAG_DEACTIVATED;
            body.sleep_count = 0;
        }

        const orig_pos = body.joints[0].position;
        const mass     = nonZero(body.joint_mass);

        // 1. Simpan prev_pos untuk CCD
        for (body.joints) |*jt| {
            jt.prev_pos = jt.position;
        }

        // 2. Gravity + Force → velocity  (Symplectic Euler: v dulu)
        const gx = world.gravity.x * body.gravity_scale;
        const gy = world.gravity.y * body.gravity_scale;
        const gz = world.gravity.z * body.gravity_scale;
        for (body.joints) |*jt| {
            if (jt.isPinned()) continue;
            jt.velocity.x += gx + jt.force.x / mass;
            jt.velocity.y += gy + jt.force.y / mass;
            jt.velocity.z += gz + jt.force.z / mass;
            if ((body.flags & FLAG_NONROTATING)!=0) jt.velocity = body.joints[0].velocity;
        }

        // 3. Padé linear damping
        if (body.linear_damping > 0) {
            const inv = 1.0 / (1.0 + body.linear_damping);
            for (body.joints) |*jt| {
                if (jt.isPinned()) continue;
                jt.velocity.x *= inv;
                jt.velocity.y *= inv;
                jt.velocity.z *= inv;
            }
        }

        // 4. Integrate posisi (menggunakan v_new dari langkah 2)
        for (body.joints) |*jt| {
            if (jt.isPinned()) continue;
            jt.position.x += jt.velocity.x;
            jt.position.y += jt.velocity.y;
            jt.position.z += jt.velocity.z;
        }

        var aabb_min: Vec3=undefined; var aabb_max: Vec3=undefined;
        bodyGetAABB(body, &aabb_min, &aabb_max);
        _b1=@intCast(i); _b2=@intCast(i);

        // 5. Collision env
        var collided = if (world.env_func) |ef|
            bodyEnvironmentResolveCollision(body, ef)
        else false;

        if ((body.flags & FLAG_NONROTATING)!=0) {
            var k: u32=0;
            while (k<NONROT_RESOLVE_ATTEMPTS and collided) : (k+=1)
                collided = if (world.env_func) |ef| bodyEnvironmentResolveCollision(body,ef) else false;
            if (collided) if (world.env_func) |ef|
                if (bodyEnvironmentCollide(body, ef))
                    bodyMoveBy(body, vec3Minus(orig_pos, body.joints[0].position));
        } else {
            // 6. Constraint solver — XPBD jika ada compliance, reshape jika kaku
            var body_tension: Unit = 0;
            const has_compliance = blk: {
                for (body.connections) |c| {
                    if (c.length > 0 and c.compliance > 0) break :blk true;
                }
                break :blk false;
            };

            if (body.connections.len > 0) {
                const hard = (body.flags & FLAG_SOFT) == 0;
                if (has_compliance) {
                    // XPBD path
                    for (0..RESHAPE_ITERS + 1) |_| {
                        bodyXPBDSolve(body, 1.0 / 60.0);
                    }
                } else {
                    // Tension push + reshape (perilaku asli)
                    for (body.connections) |conn| {
                        if (conn.length < 0) continue;
                        const ja = &body.joints[conn.joint1];
                        const jb = &body.joints[conn.joint2];
                        var dir  = vec3Minus(jb.position, ja.position);
                        const t  = connectionTension(vec3Len(dir), conn.length);
                        body_tension += @abs(t);
                        if (@abs(t) > TENSION_ACCEL_THRESH) {
                            vec3Normalize(&dir);
                            const scale: Unit = if (@abs(t) > TENSION_HIGH_THRESH) 2.0 else 1.0;
                            const push  = vec3Scale(dir, scale / TENSION_ACCEL_DIV);
                            if (!ja.isPinned()) {
                                if (t>0) { ja.velocity.x+=push.x; ja.velocity.y+=push.y; ja.velocity.z+=push.z; }
                                else     { ja.velocity.x-=push.x; ja.velocity.y-=push.y; ja.velocity.z-=push.z; }
                            }
                            if (!jb.isPinned()) {
                                if (t>0) { jb.velocity.x-=push.x; jb.velocity.y-=push.y; jb.velocity.z-=push.z; }
                                else     { jb.velocity.x+=push.x; jb.velocity.y+=push.y; jb.velocity.z+=push.z; }
                            }
                        }
                    }
                    if (hard) {
                        bodyReshape(body, world.env_func);
                        body_tension /= @as(Unit,@floatFromInt(body.connections.len));
                        if (body_tension > RESHAPE_TENSION_LIMIT)
                            for (0..RESHAPE_ITERS) |_| bodyReshape(body, world.env_func);
                    }
                }
                if ((body.flags & FLAG_SIMPLE_CONN)==0)
                    for (0..COLLIDE_SOLVER_ITERS) |_| bodyCancelOutVelocities(body, (body.flags & FLAG_SOFT)==0);
            }
        }

        // 7. Body-body collision (AABB broadphase sederhana)
        for (world.bodies, 0..) |*b2b, j| {
            if (j<=i or (b2b.flags & (FLAG_DEACTIVATED|FLAG_DISABLED))!=0) continue;
            if ((b2b.flags & FLAG_VERLET) != 0) continue; // handled in worldStepAll
            var bMin2: Vec3=undefined; var bMax2: Vec3=undefined;
            bodyGetAABB(b2b, &bMin2, &bMax2);
            _b2=@intCast(j);
            if (checkOverlapAABB(aabb_min,aabb_max,bMin2,bMax2) and
                bodiesResolveCollision(body, b2b, world.env_func)) {
                bodyActivate(body);
                bodyActivate(b2b);
                body.sleep_count = 0;
                b2b.sleep_count  = 0;
            }
        }

        // 8. Reset force acc
        for (body.joints) |*j| {
            if (j.isPinned()) continue;
            j.force = .{};
        }

        // 9. Sleep test — dual threshold: linear + angular
        if ((body.flags & FLAG_ALWAYS_ACTIVE)==0) {
            var lin_slow  = true;
            var ang_slow  = true;
            for (body.joints) |j| {
                if (j.isPinned()) continue;
                if (vec3Dot(j.velocity, j.velocity) > SLEEP_LINEAR_SQ) {
                    lin_slow = false; break;
                }
            }
            const av_sq = vec3Dot(body.angular_vel, body.angular_vel);
            if (av_sq > SLEEP_ANGULAR_SQ) ang_slow = false;

            if (lin_slow and ang_slow) {
                body.sleep_count += 1;
                if (body.sleep_count >= SLEEP_FRAMES) bodyDeactivate(body);
            } else {
                body.sleep_count = 0;
            }
        }
    }
}

pub fn worldStepN(world: *World, n: u32) void {
    for (0..n) |_| worldStep(world);
}

pub fn worldDeactivateAll(w: *World) void { for (w.bodies) |*b| bodyDeactivate(b); }
pub fn worldActivateAll(w: *World)   void { for (w.bodies) |*b| bodyActivate(b); }
pub fn worldGetNetSpeed(w: *const World) Unit {
    var r: Unit=0;
    for (w.bodies) |*b| r+=bodyGetNetSpeed(b);
    return r;
}

// ============================================================
// worldStepVerlet — Cloth (Verlet integration)
// ============================================================

pub fn worldStepVerlet(world: *World) void {
    for (world.bodies) |*body| {
        if ((body.flags & (FLAG_DEACTIVATED | FLAG_DISABLED)) != 0) continue;
        if ((body.flags & FLAG_VERLET) == 0) continue;

        const mass_val  = nonZero(body.joint_mass);
        const damp_frac : Unit = 1.0 - body.verlet_damping;

        const gx = world.gravity.x * body.gravity_scale;
        const gy = world.gravity.y * body.gravity_scale;
        const gz = world.gravity.z * body.gravity_scale;
        for (body.joints) |*j| {
            if (j.isPinned()) continue;
            j.force.x += gx * mass_val;
            j.force.y += gy * mass_val;
            j.force.z += gz * mass_val;
        }

        for (body.joints) |*j| {
            if (j.isPinned()) {
                j.prev_pos = j.position;
                j.velocity  = .{};
                j.force     = .{};
                continue;
            }
            const vx = damp_frac * (j.position.x - j.prev_pos.x);
            const vy = damp_frac * (j.position.y - j.prev_pos.y);
            const vz = damp_frac * (j.position.z - j.prev_pos.z);
            const ax = j.force.x / mass_val;
            const ay = j.force.y / mass_val;
            const az = j.force.z / mass_val;
            j.prev_pos = j.position;
            j.position.x += vx + ax;
            j.position.y += vy + ay;
            j.position.z += vz + az;
            j.velocity = mkVec3(vx, vy, vz);
            j.force    = .{};
        }

        // Constraint relaxation dengan stiffness
        const stiff = body.cloth_stiffness;
        for (0..body.verlet_relax) |_| {
            for (body.connections) |c| {
                if (c.length < 0) continue;
                const j1 = &body.joints[c.joint1];
                const j2 = &body.joints[c.joint2];
                const pin1 = j1.isPinned();
                const pin2 = j2.isPinned();
                if (pin1 and pin2) continue;
                var diff = vec3Minus(j2.position, j1.position);
                const cur  = nonZero(vec3Len(diff));
                const adj  = (c.length - cur) * stiff;
                if (@abs(adj) < 1e-12) continue;
                diff = vec3Scale(diff, 1.0/cur);
                const half = adj * 0.5;
                if (!pin1 and !pin2) {
                    j1.position = vec3Minus(j1.position, vec3Scale(diff, half));
                    j2.position = vec3Plus (j2.position, vec3Scale(diff, half));
                } else if (pin1) {
                    j2.position = vec3Plus(j2.position, vec3Scale(diff, adj));
                } else {
                    j1.position = vec3Minus(j1.position, vec3Scale(diff, adj));
                }
            }
        }

        if (world.env_func) |ef| {
            _ = bodyEnvironmentResolveCollision(body, ef);
        }
    }
}

// ============================================================
// worldStepAll — Rigid + Verlet dalam satu world, dengan CCD
// ============================================================

pub fn worldStepAll(world: *World) void {
    _callback = world.collide_callback;

    worldStep(world);
    worldStepVerlet(world);

    // Cross-collision rigid↔verlet dan verlet↔verlet
    // Termasuk CCD untuk rigid body yang bergerak cepat
    for (world.bodies, 0..) |*ba, i| {
        if ((ba.flags & (FLAG_DISABLED | FLAG_DEACTIVATED)) != 0) continue;

        var amin_a: Vec3 = undefined; var amax_a: Vec3 = undefined;
        bodyGetAABB(ba, &amin_a, &amax_a);

        for (world.bodies, 0..) |*bb, j| {
            if (j <= i) continue;
            if ((bb.flags & (FLAG_DISABLED | FLAG_DEACTIVATED)) != 0) continue;

            const a_verlet = (ba.flags & FLAG_VERLET) != 0;
            const b_verlet = (bb.flags & FLAG_VERLET) != 0;
            if (!a_verlet and !b_verlet) continue;

            var amin_b: Vec3 = undefined; var amax_b: Vec3 = undefined;
            bodyGetAABB(bb, &amin_b, &amax_b);

            _b1 = @intCast(i);
            _b2 = @intCast(j);

            if (!checkOverlapAABB(amin_a, amax_a, amin_b, amax_b)) continue;

            // CCD: jika salah satu body rigid (non-verlet), lakukan sweep
            if (!a_verlet and b_verlet) {
                _ = bodiesCCDCollision(ba, bb, world.env_func);
            } else if (a_verlet and !b_verlet) {
                _ = bodiesCCDCollision(bb, ba, world.env_func);
            }

            if (bodiesResolveCollision(ba, bb, world.env_func)) {
                bodyActivate(ba); bodyActivate(bb);
                ba.sleep_count = 0; bb.sleep_count = 0;
            }
        }
    }
}

// worldStepAllN — substep yang benar: gravity TIDAK di-scale manual.
// Substep otomatis membagi timestep sehingga setiap step lebih stabil.
// Gravity dan velocity diintegrasikan dengan timestep kecil secara natural.
pub fn worldStepAllN(world: *World, n: u32) void {
    // Scale gravity per substep agar physics scale tetap konsisten
    const fn_f: Unit = @floatFromInt(n);
    const gx_orig = world.gravity.x;
    const gy_orig = world.gravity.y;
    const gz_orig = world.gravity.z;
    world.gravity.x /= fn_f;
    world.gravity.y /= fn_f;
    world.gravity.z /= fn_f;
    for (0..n) |_| worldStepAll(world);
    world.gravity.x = gx_orig;
    world.gravity.y = gy_orig;
    world.gravity.z = gz_orig;
}

// ============================================================
// Shape Builders
// ============================================================

pub fn makeBox(
    joints: []Joint, conns: []Connection,
    sx: Unit, sy: Unit, sz: Unit, joint_size: Unit,
) void {
    const hx=sx*0.5; const hy=sy*0.5; const hz=sz*0.5;
    const vs = [8]Vec3{
        mkVec3(-hx,-hy,-hz), mkVec3( hx,-hy,-hz),
        mkVec3(-hx, hy,-hz), mkVec3( hx, hy,-hz),
        mkVec3(-hx,-hy, hz), mkVec3( hx,-hy, hz),
        mkVec3(-hx, hy, hz), mkVec3( hx, hy, hz),
    };
    for (0..8) |k| joints[k] = makeJoint(vs[k], joint_size);
    const edges = [16][2]u8{
        .{0,1},.{2,3},.{4,5},.{6,7},
        .{0,2},.{1,3},.{4,6},.{5,7},
        .{0,4},.{1,5},.{2,6},.{3,7},
        .{0,7},.{1,6},.{2,5},.{3,4},
    };
    for (0..16) |k| conns[k] = .{ .joint1=edges[k][0], .joint2=edges[k][1], .length=0 };
}

pub fn makeSphere(
    joints: []Joint, conns: []Connection,
    radius: Unit, pts: u32, rings: u32, joint_size: Unit,
) void {
    var jidx: usize = 0;
    var ci: usize   = 0;
    joints[jidx] = makeJoint(mkVec3(0, radius, 0), joint_size);
    jidx += 1;
    for (0..rings) |ri| {
        const phi: Unit = (@as(Unit, @floatFromInt(ri+1)) / @as(Unit, @floatFromInt(rings+1)) - 0.25) * std.math.pi * 2.0;
        const cy  = @sin(phi) * radius;
        const cr  = @cos(phi) * radius;
        for (0..pts) |pi| {
            const theta: Unit = @as(Unit, @floatFromInt(pi)) / @as(Unit, @floatFromInt(pts)) * std.math.pi * 2.0;
            joints[jidx] = makeJoint(mkVec3(@cos(theta)*cr, cy, @sin(theta)*cr), joint_size);
            jidx += 1;
            const cur: u16 = @intCast(1 + ri*pts + pi);
            const nxt: u16 = @intCast(1 + ri*pts + (pi+1)%pts);
            conns[ci]=.{.joint1=cur, .joint2=nxt}; ci+=1;
            if (ri>0) {
                const prev: u16 = @intCast(1 + (ri-1)*pts + pi);
                conns[ci]=.{.joint1=cur, .joint2=prev}; ci+=1;
            } else {
                conns[ci]=.{.joint1=0, .joint2=cur}; ci+=1;
            }
        }
    }
    joints[jidx] = makeJoint(mkVec3(0, -radius, 0), joint_size);
    for (0..pts) |pi| {
        const last_ring: u16 = @intCast(1 + (rings-1)*pts + pi);
        conns[ci]=.{.joint1=last_ring, .joint2=@intCast(jidx)}; ci+=1;
    }
}

// ============================================================
// Environment Shapes
// ============================================================

pub fn envGround(p_in: Vec3, h: Unit) Vec3 {
    var p=p_in; if (p.y>h) p.y=h; return p;
}

pub fn envHalfPlane(point: Vec3, center: Vec3, normal: Vec3) Vec3 {
    const d   = vec3Minus(point, center);
    var dot_v = vec3Dot(d, normal);
    if (dot_v < 0) return point;
    const l = vec3Len(normal);
    dot_v /= l;
    const n = vec3Scale(normal, 1.0/l);
    return vec3Minus(point, vec3Scale(n, dot_v));
}

pub fn envSphere(point: Vec3, center: Vec3, radius: Unit) Vec3 {
    const dir = vec3Minus(point, center);
    const l = vec3Len(dir);
    if (l <= radius) return point;
    return vec3Plus(center, vec3Scale(dir, radius/l));
}

pub fn envSphereInside(point: Vec3, center: Vec3, radius: Unit) Vec3 {
    var d = vec3Minus(point, center);
    const l = vec3Len(d);
    if (l >= radius) return point;
    if (l < 1e-12) return mkVec3(center.x+radius, center.y, center.z);
    vec3Normalize(&d);
    return vec3Plus(center, vec3Scale(d, radius));
}

pub fn envAABox(point_in: Vec3, center: Vec3, halfExt: Vec3) Vec3 {
    var point=point_in;
    const s = vec3Minus(point, center);
    const sx: Unit = if (s.x<0) -1.0 else 1.0;
    const sy: Unit = if (s.y<0) -1.0 else 1.0;
    const sz_: Unit = if (s.z<0) -1.0 else 1.0;
    const ax=@abs(s.x); const ay=@abs(s.y); const az=@abs(s.z);
    if (ax > halfExt.x) point.x = center.x + sx*halfExt.x;
    if (ay > halfExt.y) point.y = center.y + sy*halfExt.y;
    if (az > halfExt.z) point.z = center.z + sz_*halfExt.z;
    return point;
}

pub fn envAABoxInside(point_in: Vec3, center: Vec3, size_in: Vec3) Vec3 {
    var point=point_in;
    const hz=vec3Scale(size_in, 0.5);
    const s=vec3Minus(point,center);
    var ax=hz.x-s.x; const bx=s.x+hz.x; var ssx:Unit=1.0;
    var ay=hz.y-s.y; const by=s.y+hz.y; var ssy:Unit=1.0;
    var az=hz.z-s.z; const bz=s.z+hz.z; var ssz:Unit=1.0;
    if (bx<ax){ax=bx;ssx=-1.0;} if (by<ay){ay=by;ssy=-1.0;} if (bz<az){az=bz;ssz=-1.0;}
    if (ax<0 or ay<0 or az<0) return point;
    if (ax<ay){if(ax<az)point.x=center.x+ssx*hz.x else point.z=center.z+ssz*hz.z;}
    else{if(ay<az)point.y=center.y+ssy*hz.y else point.z=center.z+ssz*hz.z;}
    return point;
}

pub fn envBox(point: Vec3, center: Vec3, halfExt: Vec3, rot: Vec3) Vec3 {
    const local=pointRotate(vec3Minus(point,center), rotationInverse(rot));
    return vec3Plus(center, pointRotate(envAABox(local, mkVec3(0,0,0), halfExt), rot));
}

pub fn envInfiniteCylinder(point: Vec3, center: Vec3, dir: Vec3, radius: Unit) Vec3 {
    const rel = vec3Minus(point, center);
    const d   = vec3Minus(rel, vec3Project(rel, dir));
    const l   = vec3Len(d);
    if (l <= radius) return point;
    return vec3Minus(point, vec3Scale(d, (l-radius)/l));
}

pub fn envLineSegment(point: Vec3, a: Vec3, b_in: Vec3) Vec3 {
    var pt = vec3Minus(point, a);
    const b = vec3Minus(b_in, a);
    pt = vec3Project(pt, b);
    if (vec3Dot(pt,b) < 0) pt = mkVec3(0,0,0)
    else if (vec3Len(pt) > vec3Len(b)) pt = b;
    return vec3Plus(pt, a);
}

// ============================================================
// Hash
// ============================================================

fn hashFn(n_in: u32) u32 {
    var n=n_in;
    n=250009959 *% (n^(n>>17));
    n=2626308659 *% (n^(n>>15));
    return n^(n>>16);
}

pub fn jointHash(j: *const Joint) u32 {
    var r=hashFn(@bitCast(j.position.x));
    r=hashFn(r^@as(u32,@bitCast(j.position.y)));
    r=hashFn(r^@as(u32,@bitCast(j.position.z)));
    r=hashFn(r^@as(u32,@bitCast(j.velocity.x)));
    r=hashFn(r^@as(u32,@bitCast(j.velocity.y)));
    r=hashFn(r^@as(u32,@bitCast(j.velocity.z)));
    return r;
}

pub fn bodyHash(body: *const Body) u32 {
    var r=hashFn(@bitCast(body.joint_mass));
    r=hashFn(r^@as(u32,body.flags));
    for (body.joints) |*j| r=hashFn(r^jointHash(j));
    return r;
}

pub fn worldHash(world: *const World) u32 {
    var r: u32=0;
    for (world.bodies) |*b| r=hashFn(r^bodyHash(b));
    return r;
}

// ============================================================
// Raycast
// ============================================================

pub fn castEnvironmentRay(
    rayPos: Vec3, rayDir_in: Vec3,
    env: EnvFunc, insideStep: Unit, marchMax: Unit, maxSteps: u32,
) Vec3 {
    var rayDir=rayDir_in; vec3Normalize(&rayDir);
    var p=rayPos; var p2=env(rayPos,marchMax);
    var totalD: Unit=0; var found: u8=0;
    if (p2.x!=p.x or p2.y!=p.y or p2.z!=p.z) {
        for (0..maxSteps) |_| {
            var d=dist(p,p2); if(d>marchMax)d=marchMax;
            totalD+=d;
            p2=vec3Plus(rayPos, vec3Scale(rayDir,totalD));
            if(d<1e-12 or (p2.x==p.x and p2.y==p.y and p2.z==p.z)) return p2;
            const pt=env(p2,marchMax);
            if(pt.x==p2.x and pt.y==p2.y and pt.z==p2.z){found=1;break;}
            p=p2; p2=pt;
        }
    } else if (insideStep>1e-12) {
        for (0..maxSteps) |_| {
            totalD+=insideStep;
            p2=vec3Plus(rayPos, vec3Scale(rayDir,totalD));
            const pt=env(p2,0.016);
            if(p2.x!=pt.x or p2.y!=pt.y or p2.z!=pt.z){found=2;break;}
            p=p2; p2=pt;
        }
    }
    if (found!=0) {
        for (0..128) |_| {
            const mid=vec3Scale(vec3Plus(p,p2),0.5);
            if((mid.x==p.x and mid.y==p.y and mid.z==p.z) or
               (mid.x==p2.x and mid.y==p2.y and mid.z==p2.z)) break;
            const pt=env(mid,0.016);
            const on=(pt.x==mid.x and pt.y==mid.y and pt.z==mid.z);
            if ((found==1)==on) p2=mid else p=mid;
        }
        return if(found==1) p else p2;
    }
    return mkVec3(INF,INF,INF);
}

pub fn castBodyRay(
    rayPos: Vec3, rayDir_in: Vec3,
    excludeBody: i16, world: *const World,
    bodyIdx: ?*i16, jointIdx: ?*i16,
) Vec3 {
    var rayDir=rayDir_in; vec3Normalize(&rayDir);
    var bestP=mkVec3(INF,INF,INF); var bestD: Unit=INF;
    if(bodyIdx)|bi|bi.*=-1; if(jointIdx)|ji|ji.*=-1;
    for (world.bodies, 0..) |*body, ii| {
        if (@as(i16,@intCast(ii))==excludeBody) continue;
        var c: Vec3=undefined; var r: Unit=undefined;
        bodyGetFastBSphere(body,&c,&r);
        const cv=vec3Minus(c,rayPos);
        const pv=vec3ProjectNormalized(cv,rayDir);
        if(vec3Dot(pv,rayDir)<0) continue;
        if(dist(pv,cv)>r) continue;
        for (body.joints, 0..) |jt, jj| {
            const jc=vec3Minus(jt.position,rayPos);
            const jp=vec3ProjectNormalized(jc,rayDir);
            if(vec3Dot(jp,rayDir)<0) continue;
            const jd=dist(jp,jc);
            const js=jt.jsize();
            if(jd>js) continue;
            if(bodyIdx)|bi|bi.*=@intCast(ii); if(jointIdx)|ji|ji.*=@intCast(jj);
            const off=vec3Scale(rayDir, sqrt(js*js-jd*jd));
            const base_pt=vec3Plus(jp,rayPos);
            const hit1=vec3Plus(base_pt,off); const hit2=vec3Minus(base_pt,off);
            const d1=dist(rayPos,hit1); const d2=dist(rayPos,hit2);
            const bd=if(d2<d1)d2 else d1; const besthit=if(d2<d1)hit2 else hit1;
            if(bd<bestD){bestD=bd;bestP=besthit;}
        }
    }
    return bestP;
}

pub fn fakeSphereRotation(p1: Vec3, p2: Vec3, radius: Unit) Vec3 {
    const mx=p1.z-p2.z; const mz=p2.x-p1.x;
    const l=sqrt(mx*mx+mz*mz);
    if(l<1e-12) return mkVec3(0,0,0);
    const d=dist(p1,p2)/(radius*4.0);
    return mkVec3(mx*d/l, 0, mz*d/l);
}