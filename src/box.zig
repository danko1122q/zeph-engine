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
// │  [1] Sleep system: is_awake berbasis energi kinematik       │
// │      lin + ang, dengan SLEEP_FRAMES countdown               │
// │  [2] API baru: boxBodyApplyForce, boxBodyApplyTorque,       │
// │      boxBodyApplyImpulse, boxBodySetAwake                   │
// │  [3] Broadphase: lewatkan pasangan di mana kedua body tidur │
// │  [4] integrateBodies: gunakan is_awake lebih ketat          │
// │  [5] boxBodyAddFull: helper dengan friction + restitution   │
// └─────────────────────────────────────────────────────────────┘

const std = @import("std");

// ─────────────────────────────────────────────────────────────
// Tipe dasar
// ─────────────────────────────────────────────────────────────
pub const R = f32;
pub const R32_MAX = std.math.floatMax(R);

// ─────────────────────────────────────────────────────────────
// Vec3
// ─────────────────────────────────────────────────────────────
pub const Vec3 = struct {
    x: R = 0, y: R = 0, z: R = 0,

    pub fn set(x: R, y: R, z: R) Vec3 { return .{ .x = x, .y = y, .z = z }; }
    pub fn add(a: Vec3, b: Vec3) Vec3 { return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z }; }
    pub fn sub(a: Vec3, b: Vec3) Vec3 { return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z }; }
    pub fn scale(a: Vec3, f: R) Vec3 { return .{ .x = a.x * f, .y = a.y * f, .z = a.z * f }; }
    pub fn neg(a: Vec3) Vec3         { return .{ .x = -a.x, .y = -a.y, .z = -a.z }; }
    pub fn dot(a: Vec3, b: Vec3) R   { return a.x * b.x + a.y * b.y + a.z * b.z; }
    pub fn len(a: Vec3) R            { return @sqrt(dot(a, a)); }
    pub fn lenSq(a: Vec3) R          { return dot(a, a); }
    pub fn abs(a: Vec3) Vec3         { return .{ .x = @abs(a.x), .y = @abs(a.y), .z = @abs(a.z) }; }

    pub fn get(a: Vec3, i: usize) R {
        return switch (i) { 0 => a.x, 1 => a.y, else => a.z };
    }

    pub fn normalized(a: Vec3) Vec3 {
        const l = len(a);
        if (l < 1e-10) return .{};
        return scale(a, 1.0 / l);
    }

    pub fn cross(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.y * b.z - a.z * b.y,
            .y = a.z * b.x - a.x * b.z,
            .z = a.x * b.y - a.y * b.x,
        };
    }

    pub fn vmin(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = @min(a.x, b.x), .y = @min(a.y, b.y), .z = @min(a.z, b.z) };
    }
    pub fn vmax(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = @max(a.x, b.x), .y = @max(a.y, b.y), .z = @max(a.z, b.z) };
    }
};

// ─────────────────────────────────────────────────────────────
// Mat3 — kolom-mayor: M*v = ex*v.x + ey*v.y + ez*v.z
// ─────────────────────────────────────────────────────────────
pub const Mat3 = struct {
    ex: Vec3 = .{ .x = 1 },
    ey: Vec3 = .{ .y = 1 },
    ez: Vec3 = .{ .z = 1 },

    pub fn identity() Mat3 {
        return .{
            .ex = .{ .x = 1, .y = 0, .z = 0 },
            .ey = .{ .x = 0, .y = 1, .z = 0 },
            .ez = .{ .x = 0, .y = 0, .z = 1 },
        };
    }

    pub fn diagonal(a: R, b: R, c: R) Mat3 {
        return .{
            .ex = .{ .x = a, .y = 0, .z = 0 },
            .ey = .{ .x = 0, .y = b, .z = 0 },
            .ez = .{ .x = 0, .y = 0, .z = c },
        };
    }

    pub fn mulV(m: Mat3, v: Vec3) Vec3 {
        return .{
            .x = m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z,
            .y = m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z,
            .z = m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z,
        };
    }

    pub fn mul(a: Mat3, b: Mat3) Mat3 {
        return .{ .ex = a.mulV(b.ex), .ey = a.mulV(b.ey), .ez = a.mulV(b.ez) };
    }

    pub fn transpose(m: Mat3) Mat3 {
        return .{
            .ex = .{ .x = m.ex.x, .y = m.ey.x, .z = m.ez.x },
            .ey = .{ .x = m.ex.y, .y = m.ey.y, .z = m.ez.y },
            .ez = .{ .x = m.ex.z, .y = m.ey.z, .z = m.ez.z },
        };
    }

    pub fn add(a: Mat3, b: Mat3) Mat3 {
        return .{
            .ex = Vec3.add(a.ex, b.ex),
            .ey = Vec3.add(a.ey, b.ey),
            .ez = Vec3.add(a.ez, b.ez),
        };
    }

    pub fn scale(m: Mat3, f: R) Mat3 {
        return .{
            .ex = Vec3.scale(m.ex, f),
            .ey = Vec3.scale(m.ey, f),
            .ez = Vec3.scale(m.ez, f),
        };
    }

    pub fn absM(m: Mat3) Mat3 {
        return .{ .ex = Vec3.abs(m.ex), .ey = Vec3.abs(m.ey), .ez = Vec3.abs(m.ez) };
    }

    pub fn col(m: Mat3, i: usize) Vec3 {
        return switch (i) { 0 => m.ex, 1 => m.ey, else => m.ez };
    }

    pub fn row(m: Mat3, i: usize) Vec3 {
        return switch (i) {
            0 => .{ .x = m.ex.x, .y = m.ey.x, .z = m.ez.x },
            1 => .{ .x = m.ex.y, .y = m.ey.y, .z = m.ez.y },
            else => .{ .x = m.ex.z, .y = m.ey.z, .z = m.ez.z },
        };
    }

    pub fn get(m: Mat3, r: usize, c: usize) R {
        return Vec3.get(m.col(c), r);
    }

    pub fn outerProduct(u: Vec3, v: Vec3) Mat3 {
        return .{
            .ex = Vec3.scale(v, u.x),
            .ey = Vec3.scale(v, u.y),
            .ez = Vec3.scale(v, u.z),
        };
    }

    pub fn inverse(m: Mat3) Mat3 {
        const a = m.ex.x; const b = m.ey.x; const c = m.ez.x;
        const d = m.ex.y; const e = m.ey.y; const f = m.ez.y;
        const g = m.ex.z; const h = m.ey.z; const k = m.ez.z;
        const det = a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g);
        if (@abs(det) < 1e-12) return identity();
        const inv = 1.0 / det;
        return .{
            .ex = .{ .x = (e * k - f * h) * inv,  .y = -(d * k - f * g) * inv, .z = (d * h - e * g) * inv  },
            .ey = .{ .x = -(b * k - c * h) * inv, .y = (a * k - c * g) * inv,  .z = -(a * h - b * g) * inv },
            .ez = .{ .x = (b * f - c * e) * inv,  .y = -(a * f - c * d) * inv, .z = (a * e - b * d) * inv  },
        };
    }

    pub fn fromAxisAngle(axis: Vec3, angle: R) Mat3 {
        const c = @cos(angle);
        const s = @sin(angle);
        const t = 1.0 - c;
        const n = Vec3.normalized(axis);
        const x = n.x; const y = n.y; const z = n.z;
        return .{
            .ex = .{ .x = t * x * x + c,     .y = t * x * y + s * z, .z = t * x * z - s * y },
            .ey = .{ .x = t * x * y - s * z, .y = t * y * y + c,     .z = t * y * z + s * x },
            .ez = .{ .x = t * x * z + s * y, .y = t * y * z - s * x, .z = t * z * z + c     },
        };
    }
};

// ─────────────────────────────────────────────────────────────
// Quaternion
// ─────────────────────────────────────────────────────────────
pub const Quat = struct {
    x: R = 0, y: R = 0, z: R = 0, w: R = 1,

    pub fn identity() Quat { return .{ .x = 0, .y = 0, .z = 0, .w = 1 }; }

    pub fn fromAxisAngle(axis: Vec3, angle: R) Quat {
        const half = angle * 0.5;
        const s    = @sin(half);
        const n    = Vec3.normalized(axis);
        return .{ .x = n.x * s, .y = n.y * s, .z = n.z * s, .w = @cos(half) };
    }

    pub fn normalized(q: Quat) Quat {
        const d = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (d < 1e-16) return identity();
        const inv = 1.0 / @sqrt(d);
        return .{ .x = q.x * inv, .y = q.y * inv, .z = q.z * inv, .w = q.w * inv };
    }

    pub fn mul(a: Quat, b: Quat) Quat {
        return .{
            .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        };
    }

    pub fn integrate(q: Quat, dv: Vec3, dt: R) Quat {
        const qq   = Quat{ .x = dv.x * dt, .y = dv.y * dt, .z = dv.z * dt, .w = 0 };
        const half = Quat{ .x = qq.x * 0.5, .y = qq.y * 0.5, .z = qq.z * 0.5, .w = qq.w * 0.5 };
        return Quat.normalized(.{
            .x = q.x + half.w * q.x + half.x * q.w + half.y * q.z - half.z * q.y,
            .y = q.y + half.w * q.y - half.x * q.z + half.y * q.w + half.z * q.x,
            .z = q.z + half.w * q.z + half.x * q.y - half.y * q.x + half.z * q.w,
            .w = q.w + half.w * q.w - half.x * q.x - half.y * q.y - half.z * q.z,
        });
    }

    pub fn toMat3(q: Quat) Mat3 {
        const x = q.x; const y = q.y; const z = q.z; const w = q.w;
        return .{
            .ex = .{ .x = 1 - 2 * (y * y + z * z), .y = 2 * (x * y + z * w),   .z = 2 * (x * z - y * w) },
            .ey = .{ .x = 2 * (x * y - z * w),     .y = 1 - 2 * (x * x + z * z),.z = 2 * (y * z + x * w) },
            .ez = .{ .x = 2 * (x * z + y * w),     .y = 2 * (y * z - x * w),   .z = 1 - 2 * (x * x + y * y) },
        };
    }
};

// ─────────────────────────────────────────────────────────────
// Transform
// ─────────────────────────────────────────────────────────────
pub const Transform = struct {
    position: Vec3 = .{},
    rotation: Mat3 = Mat3.identity(),

    pub fn identity() Transform {
        return .{ .position = .{}, .rotation = Mat3.identity() };
    }

    pub fn mulPoint(tx: Transform, p: Vec3) Vec3 {
        return Vec3.add(tx.rotation.mulV(p), tx.position);
    }

    pub fn mulPointInv(tx: Transform, p: Vec3) Vec3 {
        return tx.rotation.transpose().mulV(Vec3.sub(p, tx.position));
    }

    pub fn compose(a: Transform, b: Transform) Transform {
        return .{
            .position = Vec3.add(a.rotation.mulV(b.position), a.position),
            .rotation = a.rotation.mul(b.rotation),
        };
    }
};

// ─────────────────────────────────────────────────────────────
// Contact data
//
// FeaturePair digunakan untuk matching kontak antar frame
// (warm starting persistent). Harus konsisten dengan SAT output.
// ─────────────────────────────────────────────────────────────
pub const MAX_CONTACTS = 8;

pub const FeaturePair = packed struct {
    inR:  u8 = 0,
    outR: u8 = 0,
    inI:  u8 = 0,
    outI: u8 = 0,
};

// ContactConstraint menyimpan semua data yang dibutuhkan solver
// per-contact, termasuk accumulated impulse untuk warm starting.
pub const ContactConstraint = struct {
    position         : Vec3        = .{},
    penetration      : R           = 0,
    fp               : FeaturePair = .{},

    // Lever arms (dihitung sekali saat pre-solve, reused tiap iterasi)
    ra               : Vec3        = .{},
    rb               : Vec3        = .{},

    // Effective mass (dihitung sekali saat pre-solve)
    normal_mass      : R           = 0,
    tangent_mass     : [2]R        = .{ 0, 0 },

    // Bias untuk posisi dan restitusi — dihitung di pre-solve
    // menggunakan velocity PRE-solver (supaya tidak jitter)
    bias             : R           = 0,

    // Accumulated impulse — INI yang membuat warm starting bekerja
    // Nilainya DIPERTAHANKAN antar frame jika contact masih ada
    normal_impulse   : R           = 0,
    tangent_impulse  : [2]R        = .{ 0, 0 },
};

pub const Manifold = struct {
    normal        : Vec3                            = .{},
    tangent       : [2]Vec3                         = .{ .{}, .{} },
    contacts      : [MAX_CONTACTS]ContactConstraint = undefined,
    contact_count : i32                             = 0,
    body_a        : usize                           = 0,
    body_b        : usize                           = 0,
};

// ─────────────────────────────────────────────────────────────
// BoxBody
// ─────────────────────────────────────────────────────────────
pub const BoxBody = struct {
    position          : Vec3      = .{},
    quaternion        : Quat      = Quat.identity(),
    lin_vel           : Vec3      = .{},
    ang_vel           : Vec3      = .{},
    force             : Vec3      = .{},
    torque            : Vec3      = .{},

    extent            : Vec3      = .{ .x = 0.5, .y = 0.5, .z = 0.5 },
    local_tx          : Transform = Transform.identity(),

    mass              : R         = 1.0,
    inv_mass          : R         = 1.0,
    inv_inertia_local : Mat3      = Mat3.identity(),
    inv_inertia_world : Mat3      = Mat3.identity(),

    friction          : R         = 0.5,
    restitution       : R         = 0.2,
    lin_damping       : R         = 0.05,
    ang_damping       : R         = 0.08,
    gravity_scale     : R         = 1.0,
    is_static         : bool      = false,
    is_awake          : bool      = true,
    /// Frame counter untuk sleep system. Reset ke 0 saat ada aktivitas.
    sleep_count       : u32       = 0,

    world_tx          : Transform = Transform.identity(),

    pub fn computeTransform(b: *BoxBody) void {
        b.world_tx.rotation = b.quaternion.toMat3();
        b.world_tx.position = b.position;
        const rot = b.world_tx.rotation;
        const Rt  = rot.transpose();
        b.inv_inertia_world = rot.mul(b.inv_inertia_local).mul(Rt);
    }

    pub fn computeMass(b: *BoxBody) void {
        if (b.is_static) {
            b.mass              = 0;
            b.inv_mass          = 0;
            b.inv_inertia_local = Mat3.diagonal(0, 0, 0);
            b.inv_inertia_world = Mat3.diagonal(0, 0, 0);
            return;
        }
        const e    = b.extent;
        const mass = 8.0 * e.x * e.y * e.z;
        b.mass     = mass;
        b.inv_mass = if (mass > 1e-10) 1.0 / mass else 0;

        const ex2 = 4.0 * e.x * e.x;
        const ey2 = 4.0 * e.y * e.y;
        const ez2 = 4.0 * e.z * e.z;
        const ix   = (1.0 / 12.0) * mass * (ey2 + ez2);
        const iy   = (1.0 / 12.0) * mass * (ex2 + ez2);
        const iz   = (1.0 / 12.0) * mass * (ex2 + ey2);
        b.inv_inertia_local = Mat3.inverse(Mat3.diagonal(ix, iy, iz));
    }

    pub fn getTransform(b: *const BoxBody) Transform {
        return Transform.compose(b.world_tx, b.local_tx);
    }

    pub fn applyImpulse(b: *BoxBody, impulse: Vec3, point: Vec3) void {
        if (b.is_static) return;
        b.lin_vel = Vec3.add(b.lin_vel, Vec3.scale(impulse, b.inv_mass));
        const r   = Vec3.sub(point, b.world_tx.position);
        b.ang_vel = Vec3.add(b.ang_vel, b.inv_inertia_world.mulV(Vec3.cross(r, impulse)));
    }

    pub fn velocityAt(b: *const BoxBody, point: Vec3) Vec3 {
        const r = Vec3.sub(point, b.world_tx.position);
        return Vec3.add(b.lin_vel, Vec3.cross(b.ang_vel, r));
    }
};

// ─────────────────────────────────────────────────────────────
// Sleep constants
// ─────────────────────────────────────────────────────────────
pub const BOX_SLEEP_LINEAR_SQ  : R = 0.005 * 0.005;   // (m/s)²
pub const BOX_SLEEP_ANGULAR_SQ : R = 0.01  * 0.01;    // (rad/s)²
pub const BOX_SLEEP_FRAMES     : u32 = 30;

// ─────────────────────────────────────────────────────────────
// BoxWorld
//
// manifolds_prev menyimpan manifold dari frame SEBELUMNYA
// untuk warm starting persistent.
// ─────────────────────────────────────────────────────────────
pub const MAX_BODIES    = 64;
pub const MAX_MANIFOLDS = MAX_BODIES * MAX_BODIES / 2;

pub const BoxWorld = struct {
    bodies        : [MAX_BODIES]BoxBody       = undefined,
    n_bodies      : usize                     = 0,

    // Manifold frame ini (diisi ulang setiap step)
    manifolds     : [MAX_MANIFOLDS]Manifold   = undefined,
    n_manifolds   : usize                     = 0,

    // Manifold frame sebelumnya (untuk warm starting)
    manifolds_prev: [MAX_MANIFOLDS]Manifold   = undefined,
    n_manifolds_prev: usize                   = 0,

    gravity       : Vec3                      = .{ .x = 0, .y = -9.8, .z = 0 },
    dt            : R                         = 1.0 / 60.0,
};

// ─────────────────────────────────────────────────────────────
// API Publik
// ─────────────────────────────────────────────────────────────
pub fn boxWorldInit(world: *BoxWorld) void {
    world.n_bodies         = 0;
    world.n_manifolds      = 0;
    world.n_manifolds_prev = 0;
}

pub fn boxBodyAdd(
    world    : *BoxWorld,
    position : Vec3,
    half_ext : Vec3,
    mass     : R,
    axis     : Vec3,
    angle    : R,
) usize {
    const idx = world.n_bodies;
    world.n_bodies += 1;
    world.bodies[idx] = BoxBody{};
    const b = &world.bodies[idx];
    b.position   = position;
    b.quaternion = Quat.fromAxisAngle(axis, angle);
    b.extent     = half_ext;
    b.mass       = mass;
    b.is_static  = false;
    b.computeMass();
    b.computeTransform();
    return idx;
}

pub fn boxBodyAddStatic(
    world    : *BoxWorld,
    position : Vec3,
    half_ext : Vec3,
    axis     : Vec3,
    angle    : R,
) usize {
    const idx = world.n_bodies;
    world.n_bodies += 1;
    world.bodies[idx] = BoxBody{};
    const b = &world.bodies[idx];
    b.position   = position;
    b.quaternion = Quat.fromAxisAngle(axis, angle);
    b.extent     = half_ext;
    b.is_static  = true;
    b.computeMass();
    b.computeTransform();
    return idx;
}

// Shortcut: tambah lantai horizontal di y = floor_y
// half_ext.y menentukan ketebalan lantai (0.5 cukup)
pub fn boxAddFloor(
    world   : *BoxWorld,
    floor_y : R,
    half_ext: Vec3,
) usize {
    return boxBodyAddStatic(
        world,
        .{ .x = 0, .y = floor_y - half_ext.y, .z = 0 },
        half_ext,
        .{ .x = 0, .y = 1, .z = 0 },
        0,
    );
}

/// Tambah body dinamis dengan friction dan restitution kustom.
pub fn boxBodyAddFull(
    world       : *BoxWorld,
    position    : Vec3,
    half_ext    : Vec3,
    mass        : R,
    axis        : Vec3,
    angle       : R,
    friction    : R,
    restitution : R,
) usize {
    const idx = boxBodyAdd(world, position, half_ext, mass, axis, angle);
    world.bodies[idx].friction    = friction;
    world.bodies[idx].restitution = restitution;
    return idx;
}

/// Akumulasikan gaya eksternal pada body (direset tiap frame).
pub fn boxBodyApplyForce(b: *BoxBody, force: Vec3) void {
    if (b.is_static) return;
    b.force = Vec3.add(b.force, force);
    b.is_awake = true;
    b.sleep_count = 0;
}

/// Akumulasikan torque eksternal pada body (direset tiap frame).
pub fn boxBodyApplyTorque(b: *BoxBody, torque: Vec3) void {
    if (b.is_static) return;
    b.torque = Vec3.add(b.torque, torque);
    b.is_awake = true;
    b.sleep_count = 0;
}

/// Terapkan impulse linear dan angular seketika.
pub fn boxBodyApplyImpulse(b: *BoxBody, impulse: Vec3, point: Vec3) void {
    if (b.is_static) return;
    b.applyImpulse(impulse, point);
    b.is_awake = true;
    b.sleep_count = 0;
}

/// Paksa body bangun (is_awake = true).
pub fn boxBodySetAwake(b: *BoxBody) void {
    b.is_awake = true;
    b.sleep_count = 0;
}

/// Paksa body tidur (is_awake = false).
pub fn boxBodySetSleep(b: *BoxBody) void {
    if (b.is_static) return;
    b.is_awake = false;
    b.lin_vel = .{};
    b.ang_vel = .{};
}

// ─────────────────────────────────────────────────────────────
// SAT Collision (port qu3e BoxToBox)
// ─────────────────────────────────────────────────────────────

fn trackFaceAxis(
    axis: *i32, n: i32, s: R, sMax: *R,
    normal: Vec3, axisNormal: *Vec3,
) bool {
    if (s > 0) return true;
    if (s > sMax.*) {
        sMax.*      = s;
        axis.*      = n;
        axisNormal.* = normal;
    }
    return false;
}

fn trackEdgeAxis(
    axis: *i32, n: i32, s_in: R, sMax: *R,
    normal: Vec3, axisNormal: *Vec3,
) bool {
    if (s_in > 0) return true;
    const l = Vec3.len(normal);
    const s = s_in / l;
    if (s > sMax.*) {
        sMax.*      = s;
        axis.*      = n;
        axisNormal.* = Vec3.scale(normal, 1.0 / l);
    }
    return false;
}

const ClipVertex = struct {
    v  : Vec3        = .{},
    fp : FeaturePair = .{},
};

fn computeIncidentFace(itx: Transform, e: Vec3, n: Vec3, out: []ClipVertex) void {
    const ni   = Vec3.neg(itx.rotation.transpose().mulV(n));
    const absN = Vec3.abs(ni);

    if (absN.x > absN.y and absN.x > absN.z) {
        if (ni.x > 0) {
            out[0].v = .{ .x =  e.x, .y =  e.y, .z = -e.z };
            out[1].v = .{ .x =  e.x, .y =  e.y, .z =  e.z };
            out[2].v = .{ .x =  e.x, .y = -e.y, .z =  e.z };
            out[3].v = .{ .x =  e.x, .y = -e.y, .z = -e.z };
            out[0].fp = .{ .inI = 9,  .outI = 1,  .inR = 0, .outR = 0 };
            out[1].fp = .{ .inI = 1,  .outI = 8,  .inR = 0, .outR = 0 };
            out[2].fp = .{ .inI = 8,  .outI = 7,  .inR = 0, .outR = 0 };
            out[3].fp = .{ .inI = 7,  .outI = 9,  .inR = 0, .outR = 0 };
        } else {
            out[0].v = .{ .x = -e.x, .y = -e.y, .z =  e.z };
            out[1].v = .{ .x = -e.x, .y =  e.y, .z =  e.z };
            out[2].v = .{ .x = -e.x, .y =  e.y, .z = -e.z };
            out[3].v = .{ .x = -e.x, .y = -e.y, .z = -e.z };
            out[0].fp = .{ .inI = 5,  .outI = 11, .inR = 0, .outR = 0 };
            out[1].fp = .{ .inI = 11, .outI = 3,  .inR = 0, .outR = 0 };
            out[2].fp = .{ .inI = 3,  .outI = 10, .inR = 0, .outR = 0 };
            out[3].fp = .{ .inI = 10, .outI = 5,  .inR = 0, .outR = 0 };
        }
    } else if (absN.y > absN.x and absN.y > absN.z) {
        if (ni.y > 0) {
            out[0].v = .{ .x = -e.x, .y =  e.y, .z =  e.z };
            out[1].v = .{ .x =  e.x, .y =  e.y, .z =  e.z };
            out[2].v = .{ .x =  e.x, .y =  e.y, .z = -e.z };
            out[3].v = .{ .x = -e.x, .y =  e.y, .z = -e.z };
            out[0].fp = .{ .inI = 3, .outI = 0, .inR = 0, .outR = 0 };
            out[1].fp = .{ .inI = 0, .outI = 1, .inR = 0, .outR = 0 };
            out[2].fp = .{ .inI = 1, .outI = 2, .inR = 0, .outR = 0 };
            out[3].fp = .{ .inI = 2, .outI = 3, .inR = 0, .outR = 0 };
        } else {
            out[0].v = .{ .x =  e.x, .y = -e.y, .z =  e.z };
            out[1].v = .{ .x = -e.x, .y = -e.y, .z =  e.z };
            out[2].v = .{ .x = -e.x, .y = -e.y, .z = -e.z };
            out[3].v = .{ .x =  e.x, .y = -e.y, .z = -e.z };
            out[0].fp = .{ .inI = 7, .outI = 4, .inR = 0, .outR = 0 };
            out[1].fp = .{ .inI = 4, .outI = 5, .inR = 0, .outR = 0 };
            out[2].fp = .{ .inI = 5, .outI = 6, .inR = 0, .outR = 0 };
            out[3].fp = .{ .inI = 6, .outI = 7, .inR = 0, .outR = 0 };
        }
    } else {
        if (ni.z > 0) {
            out[0].v = .{ .x = -e.x, .y =  e.y, .z =  e.z };
            out[1].v = .{ .x = -e.x, .y = -e.y, .z =  e.z };
            out[2].v = .{ .x =  e.x, .y = -e.y, .z =  e.z };
            out[3].v = .{ .x =  e.x, .y =  e.y, .z =  e.z };
            out[0].fp = .{ .inI = 0,  .outI = 11, .inR = 0, .outR = 0 };
            out[1].fp = .{ .inI = 11, .outI = 4,  .inR = 0, .outR = 0 };
            out[2].fp = .{ .inI = 4,  .outI = 8,  .inR = 0, .outR = 0 };
            out[3].fp = .{ .inI = 8,  .outI = 0,  .inR = 0, .outR = 0 };
        } else {
            out[0].v = .{ .x =  e.x, .y = -e.y, .z = -e.z };
            out[1].v = .{ .x = -e.x, .y = -e.y, .z = -e.z };
            out[2].v = .{ .x = -e.x, .y =  e.y, .z = -e.z };
            out[3].v = .{ .x =  e.x, .y =  e.y, .z = -e.z };
            out[0].fp = .{ .inI = 9,  .outI = 6,  .inR = 0, .outR = 0 };
            out[1].fp = .{ .inI = 6,  .outI = 10, .inR = 0, .outR = 0 };
            out[2].fp = .{ .inI = 10, .outI = 2,  .inR = 0, .outR = 0 };
            out[3].fp = .{ .inI = 2,  .outI = 9,  .inR = 0, .outR = 0 };
        }
    }
    for (0..4) |i| out[i].v = itx.mulPoint(out[i].v);
}

fn computeReferenceEdgesAndBasis(
    eR: Vec3, rtx: Transform, n: Vec3, axis_in: i32,
    out: []u8, basis: *Mat3, e: *Vec3,
) void {
    const nl = rtx.rotation.transpose().mulV(n);
    var axis = axis_in;
    if (axis >= 3) axis -= 3;

    switch (axis) {
        0 => {
            if (nl.x > 0) {
                out[0] = 1; out[1] = 8; out[2] = 7; out[3] = 9;
                e.*    = .{ .x = eR.y, .y = eR.z, .z = eR.x };
                basis.* = Mat3{ .ex = rtx.rotation.ey, .ey = rtx.rotation.ez, .ez = rtx.rotation.ex };
            } else {
                out[0] = 11; out[1] = 3; out[2] = 10; out[3] = 5;
                e.*    = .{ .x = eR.z, .y = eR.y, .z = eR.x };
                basis.* = Mat3{ .ex = rtx.rotation.ez, .ey = rtx.rotation.ey, .ez = Vec3.neg(rtx.rotation.ex) };
            }
        },
        1 => {
            if (nl.y > 0) {
                out[0] = 0; out[1] = 1; out[2] = 2; out[3] = 3;
                e.*    = .{ .x = eR.z, .y = eR.x, .z = eR.y };
                basis.* = Mat3{ .ex = rtx.rotation.ez, .ey = rtx.rotation.ex, .ez = rtx.rotation.ey };
            } else {
                out[0] = 4; out[1] = 5; out[2] = 6; out[3] = 7;
                e.*    = .{ .x = eR.z, .y = eR.x, .z = eR.y };
                basis.* = Mat3{ .ex = rtx.rotation.ez, .ey = Vec3.neg(rtx.rotation.ex), .ez = Vec3.neg(rtx.rotation.ey) };
            }
        },
        else => {
            if (nl.z > 0) {
                out[0] = 11; out[1] = 4; out[2] = 8; out[3] = 0;
                e.*    = .{ .x = eR.y, .y = eR.x, .z = eR.z };
                basis.* = Mat3{ .ex = Vec3.neg(rtx.rotation.ey), .ey = rtx.rotation.ex, .ez = rtx.rotation.ez };
            } else {
                out[0] = 6; out[1] = 10; out[2] = 2; out[3] = 9;
                e.*    = .{ .x = eR.y, .y = eR.x, .z = eR.z };
                basis.* = Mat3{ .ex = Vec3.neg(rtx.rotation.ey), .ey = Vec3.neg(rtx.rotation.ex), .ez = Vec3.neg(rtx.rotation.ez) };
            }
        },
    }
}

fn orthographic(
    sign: R, e: R, axis: usize, clip_edge: u8,
    in_verts: []ClipVertex, in_count: usize,
    out_verts: []ClipVertex,
) usize {
    var out_count: usize = 0;
    var a = in_verts[in_count - 1];

    for (0..in_count) |i| {
        const b  = in_verts[i];
        const da = sign * Vec3.get(a.v, axis) - e;
        const db = sign * Vec3.get(b.v, axis) - e;

        if (da < 0 and db < 0) {
            out_verts[out_count] = b;
            out_count += 1;
        } else if (da < 0 and db >= 0) {
            var cv = ClipVertex{};
            cv.fp      = b.fp;
            const t    = da / (da - db);
            cv.v       = Vec3.add(a.v, Vec3.scale(Vec3.sub(b.v, a.v), t));
            cv.fp.outR = clip_edge;
            cv.fp.outI = 0;
            out_verts[out_count] = cv;
            out_count += 1;
        } else if (da >= 0 and db < 0) {
            var cv = ClipVertex{};
            cv.fp      = a.fp;
            const t    = da / (da - db);
            cv.v       = Vec3.add(a.v, Vec3.scale(Vec3.sub(b.v, a.v), t));
            cv.fp.inR  = clip_edge;
            cv.fp.inI  = 0;
            out_verts[out_count] = cv;
            out_count += 1;
            out_verts[out_count] = b;
            out_count += 1;
        }
        a = b;
    }
    return out_count;
}

fn clip(
    rPos: Vec3, e: Vec3, clip_edges: []u8,
    basis: Mat3, incident: []ClipVertex,
    out_verts: []ClipVertex, out_depths: []R,
) usize {
    var in_buf   : [8]ClipVertex = undefined;
    var out_buf  : [8]ClipVertex = undefined;
    var in_count : usize = 4;
    var out_count: usize = 0;

    for (0..4) |i| {
        in_buf[i].v  = basis.transpose().mulV(Vec3.sub(incident[i].v, rPos));
        in_buf[i].fp = incident[i].fp;
    }

    out_count = orthographic( 1.0, e.x, 0, clip_edges[0], &in_buf, in_count, &out_buf);
    if (out_count == 0) return 0;
    in_count  = orthographic( 1.0, e.y, 1, clip_edges[1], &out_buf, out_count, &in_buf);
    if (in_count == 0) return 0;
    out_count = orthographic(-1.0, e.x, 0, clip_edges[2], &in_buf, in_count, &out_buf);
    if (out_count == 0) return 0;
    in_count  = orthographic(-1.0, e.y, 1, clip_edges[3], &out_buf, out_count, &in_buf);

    out_count = 0;
    for (0..in_count) |i| {
        const d = Vec3.get(in_buf[i].v, 2) - e.z;
        if (d <= 0) {
            out_verts[out_count].v  = Vec3.add(basis.mulV(in_buf[i].v), rPos);
            out_verts[out_count].fp = in_buf[i].fp;
            out_depths[out_count]   = d;
            out_count += 1;
        }
    }
    return out_count;
}

fn edgesContact(CA: *Vec3, CB: *Vec3, PA: Vec3, QA: Vec3, PB: Vec3, QB: Vec3) void {
    const DA    = Vec3.sub(QA, PA);
    const DB    = Vec3.sub(QB, PB);
    const r     = Vec3.sub(PA, PB);
    const a     = Vec3.dot(DA, DA);
    const e     = Vec3.dot(DB, DB);
    const f     = Vec3.dot(DB, r);
    const c     = Vec3.dot(DA, r);
    const b     = Vec3.dot(DA, DB);
    const denom = a * e - b * b;
    if (@abs(denom) < 1e-10) { CA.* = PA; CB.* = PB; return; }
    const TA = (b * f - c * e) / denom;
    const TB = (b * TA + f) / e;
    CA.* = Vec3.add(PA, Vec3.scale(DA, TA));
    CB.* = Vec3.add(PB, Vec3.scale(DB, TB));
}

fn supportEdge(tx: Transform, e: Vec3, n: Vec3, aOut: *Vec3, bOut: *Vec3) void {
    const nl   = tx.rotation.transpose().mulV(n);
    const absN = Vec3.abs(nl);
    var a = Vec3{};
    var b = Vec3{};

    if (absN.x > absN.y) {
        if (absN.y > absN.z) {
            a = .{ .x = e.x, .y =  e.y, .z =  e.z };
            b = .{ .x = e.x, .y =  e.y, .z = -e.z };
        } else {
            a = .{ .x = e.x, .y =  e.y, .z = e.z };
            b = .{ .x = e.x, .y = -e.y, .z = e.z };
        }
    } else {
        if (absN.x > absN.z) {
            a = .{ .x =  e.x, .y = e.y, .z = e.z };
            b = .{ .x =  e.x, .y = e.y, .z = -e.z };
        } else {
            a = .{ .x =  e.x, .y = e.y, .z = e.z };
            b = .{ .x = -e.x, .y = e.y, .z = e.z };
        }
    }

    const sx = if (nl.x >= 0) @as(R, 1.0) else @as(R, -1.0);
    const sy = if (nl.y >= 0) @as(R, 1.0) else @as(R, -1.0);
    const sz = if (nl.z >= 0) @as(R, 1.0) else @as(R, -1.0);
    a.x *= sx; a.y *= sy; a.z *= sz;
    b.x *= sx; b.y *= sy; b.z *= sz;
    aOut.* = tx.mulPoint(a);
    bOut.* = tx.mulPoint(b);
}

// Mengisi Manifold dengan ContactConstraint (tanpa bias/mass — dilakukan di preSolve)
pub fn boxToBox(m: *Manifold, ba: *BoxBody, bb: *BoxBody, ia: usize, ib: usize) bool {
    const atx  = ba.getTransform();
    const btx  = bb.getTransform();
    const eA   = ba.extent;
    const eB   = bb.extent;
    const C    = atx.rotation.transpose().mul(btx.rotation);
    const absC = C.absM();

    var parallel = false;
    const kCosTol: R = 1e-6;
    outer: for (0..3) |i| {
        for (0..3) |j| {
            if (absC.get(i, j) + kCosTol >= 1.0) { parallel = true; break :outer; }
        }
    }

    const t = atx.rotation.transpose().mulV(Vec3.sub(btx.position, atx.position));

    var s: R = 0;
    var aMax: R = -R32_MAX;
    var bMax: R = -R32_MAX;
    var eMax: R = -R32_MAX;
    var aAxis: i32 = ~@as(i32, 0);
    var bAxis: i32 = ~@as(i32, 0);
    var eAxis: i32 = ~@as(i32, 0);
    var nA = Vec3{};
    var nB = Vec3{};
    var nE = Vec3{};

    s = @abs(t.x) - (eA.x + Vec3.dot(absC.col(0), eB));
    if (trackFaceAxis(&aAxis, 0, s, &aMax, atx.rotation.ex, &nA)) return false;
    s = @abs(t.y) - (eA.y + Vec3.dot(absC.col(1), eB));
    if (trackFaceAxis(&aAxis, 1, s, &aMax, atx.rotation.ey, &nA)) return false;
    s = @abs(t.z) - (eA.z + Vec3.dot(absC.col(2), eB));
    if (trackFaceAxis(&aAxis, 2, s, &aMax, atx.rotation.ez, &nA)) return false;

    s = @abs(Vec3.dot(t, C.ex)) - (eB.x + Vec3.dot(absC.ex, eA));
    if (trackFaceAxis(&bAxis, 3, s, &bMax, btx.rotation.ex, &nB)) return false;
    s = @abs(Vec3.dot(t, C.ey)) - (eB.y + Vec3.dot(absC.ey, eA));
    if (trackFaceAxis(&bAxis, 4, s, &bMax, btx.rotation.ey, &nB)) return false;
    s = @abs(Vec3.dot(t, C.ez)) - (eB.z + Vec3.dot(absC.ez, eA));
    if (trackFaceAxis(&bAxis, 5, s, &bMax, btx.rotation.ez, &nB)) return false;

    if (!parallel) {
        var rA: R = 0;
        var rB: R = 0;

        rA = eA.y * absC.get(0, 2) + eA.z * absC.get(0, 1);
        rB = eB.y * absC.get(2, 0) + eB.z * absC.get(1, 0);
        s  = @abs(t.z * C.get(0, 1) - t.y * C.get(0, 2)) - (rA + rB);
        if (trackEdgeAxis(&eAxis, 6,  s, &eMax, .{ .x = 0, .y = -C.get(0, 2), .z =  C.get(0, 1) }, &nE)) return false;

        rA = eA.y * absC.get(1, 2) + eA.z * absC.get(1, 1);
        rB = eB.x * absC.get(2, 0) + eB.z * absC.get(0, 0);
        s  = @abs(t.z * C.get(1, 1) - t.y * C.get(1, 2)) - (rA + rB);
        if (trackEdgeAxis(&eAxis, 7,  s, &eMax, .{ .x = 0, .y = -C.get(1, 2), .z =  C.get(1, 1) }, &nE)) return false;

        rA = eA.y * absC.get(2, 2) + eA.z * absC.get(2, 1);
        rB = eB.x * absC.get(1, 0) + eB.y * absC.get(0, 0);
        s  = @abs(t.z * C.get(2, 1) - t.y * C.get(2, 2)) - (rA + rB);
        if (trackEdgeAxis(&eAxis, 8,  s, &eMax, .{ .x = 0, .y = -C.get(2, 2), .z =  C.get(2, 1) }, &nE)) return false;

        rA = eA.x * absC.get(0, 2) + eA.z * absC.get(0, 0);
        rB = eB.y * absC.get(2, 1) + eB.z * absC.get(1, 1);
        s  = @abs(t.x * C.get(0, 2) - t.z * C.get(0, 0)) - (rA + rB);
        if (trackEdgeAxis(&eAxis, 9,  s, &eMax, .{ .x =  C.get(0, 2), .y = 0, .z = -C.get(0, 0) }, &nE)) return false;

        rA = eA.x * absC.get(1, 2) + eA.z * absC.get(1, 0);
        rB = eB.x * absC.get(2, 1) + eB.z * absC.get(0, 1);
        s  = @abs(t.x * C.get(1, 2) - t.z * C.get(1, 0)) - (rA + rB);
        if (trackEdgeAxis(&eAxis, 10, s, &eMax, .{ .x =  C.get(1, 2), .y = 0, .z = -C.get(1, 0) }, &nE)) return false;

        rA = eA.x * absC.get(2, 2) + eA.z * absC.get(2, 0);
        rB = eB.x * absC.get(1, 1) + eB.y * absC.get(0, 1);
        s  = @abs(t.x * C.get(2, 2) - t.z * C.get(2, 0)) - (rA + rB);
        if (trackEdgeAxis(&eAxis, 11, s, &eMax, .{ .x =  C.get(2, 2), .y = 0, .z = -C.get(2, 0) }, &nE)) return false;

        rA = eA.x * absC.get(0, 1) + eA.y * absC.get(0, 0);
        rB = eB.y * absC.get(2, 2) + eB.z * absC.get(1, 2);
        s  = @abs(t.y * C.get(0, 0) - t.x * C.get(0, 1)) - (rA + rB);
        if (trackEdgeAxis(&eAxis, 12, s, &eMax, .{ .x = -C.get(0, 1), .y =  C.get(0, 0), .z = 0 }, &nE)) return false;

        rA = eA.x * absC.get(1, 1) + eA.y * absC.get(1, 0);
        rB = eB.x * absC.get(2, 2) + eB.z * absC.get(0, 2);
        s  = @abs(t.y * C.get(1, 0) - t.x * C.get(1, 1)) - (rA + rB);
        if (trackEdgeAxis(&eAxis, 13, s, &eMax, .{ .x = -C.get(1, 1), .y =  C.get(1, 0), .z = 0 }, &nE)) return false;

        rA = eA.x * absC.get(2, 1) + eA.y * absC.get(2, 0);
        rB = eB.x * absC.get(1, 2) + eB.y * absC.get(0, 2);
        s  = @abs(t.y * C.get(2, 0) - t.x * C.get(2, 1)) - (rA + rB);
        if (trackEdgeAxis(&eAxis, 14, s, &eMax, .{ .x = -C.get(2, 1), .y =  C.get(2, 0), .z = 0 }, &nE)) return false;
    }

    const kRelTol: R = 0.95;
    const kAbsTol: R = 0.01;
    var axis_sel: i32 = 0;
    var sMax_sel: R   = 0;
    var n_sel = Vec3{};
    const faceMax = @max(aMax, bMax);
    if (kRelTol * eMax > faceMax + kAbsTol) {
        axis_sel = eAxis; sMax_sel = eMax; n_sel = nE;
    } else {
        if (kRelTol * bMax > aMax + kAbsTol) {
            axis_sel = bAxis; sMax_sel = bMax; n_sel = nB;
        } else {
            axis_sel = aAxis; sMax_sel = aMax; n_sel = nA;
        }
    }

    if (Vec3.dot(n_sel, Vec3.sub(btx.position, atx.position)) < 0)
        n_sel = Vec3.neg(n_sel);

    if (axis_sel == ~@as(i32, 0)) return false;

    m.body_a        = ia;
    m.body_b        = ib;
    m.contact_count = 0;

    if (axis_sel < 6) {
        var rtx: Transform = undefined;
        var itx: Transform = undefined;
        var eR  = Vec3{};
        var eI  = Vec3{};
        var flip = false;

        if (axis_sel < 3) {
            rtx = atx; itx = btx; eR = eA; eI = eB; flip = false;
        } else {
            rtx = btx; itx = atx; eR = eB; eI = eA; flip = true;
            n_sel = Vec3.neg(n_sel);
        }

        var incident: [4]ClipVertex = undefined;
        computeIncidentFace(itx, eI, n_sel, &incident);

        var clip_edges: [4]u8 = undefined;
        var basis = Mat3.identity();
        var e_ref = Vec3{};
        computeReferenceEdgesAndBasis(eR, rtx, n_sel, axis_sel, &clip_edges, &basis, &e_ref);

        var out_verts  : [8]ClipVertex = undefined;
        var out_depths : [8]R          = undefined;
        const out_num = clip(rtx.position, e_ref, &clip_edges, basis, &incident, &out_verts, &out_depths);

        if (out_num > 0) {
            m.contact_count = @intCast(out_num);
            m.normal        = if (flip) Vec3.neg(n_sel) else n_sel;
            for (0..out_num) |i| {
                var c  = &m.contacts[i];
                c.fp   = out_verts[i].fp;
                if (flip) {
                    std.mem.swap(u8, &c.fp.inI, &c.fp.inR);
                    std.mem.swap(u8, &c.fp.outI, &c.fp.outR);
                }
                c.position    = out_verts[i].v;
                c.penetration = out_depths[i];
                // Accumulated impulse di-reset dulu; warm starting akan isi dari prev frame
                c.normal_impulse   = 0;
                c.tangent_impulse  = .{ 0, 0 };
            }
            return true;
        }
    } else {
        var nn = atx.rotation.mulV(n_sel);
        if (Vec3.dot(nn, Vec3.sub(btx.position, atx.position)) < 0)
            nn = Vec3.neg(nn);

        var PA = Vec3{}; var QA = Vec3{};
        var PB = Vec3{}; var QB = Vec3{};
        supportEdge(atx, eA,  nn, &PA, &QA);
        supportEdge(btx, eB, Vec3.neg(nn), &PB, &QB);

        var CA = Vec3{}; var CB = Vec3{};
        edgesContact(&CA, &CB, PA, QA, PB, QB);

        m.normal        = nn;
        m.contact_count = 1;
        m.contacts[0] = .{
            .penetration    = sMax_sel,
            .position       = Vec3.scale(Vec3.add(CA, CB), 0.5),
            .fp             = .{ .inR = 0, .outR = 0, .inI = @intCast(axis_sel), .outI = 0 },
            .normal_impulse  = 0,
            .tangent_impulse = .{ 0, 0 },
        };
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
// Sequential Impulses Solver
// ─────────────────────────────────────────────────────────────

// Konstanta solver — tuning di sini
const BAUMGARTE        : R = 0.20;   // faktor koreksi posisi (0.1–0.3 umum)
const PENETRATION_SLOP : R = 0.005;  // toleransi sinking sebelum dikoreksi (meter)
const RESTITUTION_SLOP : R = 1.0;    // minimum vn (m/s) untuk restitusi aktif
const SOLVER_ITERS     : usize = 10; // iterasi per frame

fn computeTangentBasis(n: Vec3, t0: *Vec3, t1: *Vec3) void {
    if (@abs(n.x) >= 0.57735027) {
        t0.* = Vec3.normalized(.{ .x = n.y, .y = -n.x, .z = 0 });
    } else {
        t0.* = Vec3.normalized(.{ .x = 0, .y = n.z, .z = -n.y });
    }
    t1.* = Vec3.cross(n, t0.*);
}

// Warm starting persistent:
// Cari contact di frame SEBELUMNYA dengan feature pair yang sama,
// copy accumulated impulse-nya ke frame ini.
fn warmStartManifold(world: *BoxWorld, m: *Manifold) void {
    // Cari manifold lama dengan body pair yang sama
    const ba = &world.bodies[m.body_a];
    const bb = &world.bodies[m.body_b];

    var prev: ?*Manifold = null;
    for (0..world.n_manifolds_prev) |pi| {
        const p = &world.manifolds_prev[pi];
        if (p.body_a == m.body_a and p.body_b == m.body_b) {
            prev = p;
            break;
        }
    }

    if (prev == null) return;
    const pm = prev.?;

    // Untuk setiap contact baru, cari contact lama dengan feature pair cocok
    for (0..@intCast(m.contact_count)) |ci| {
        var c = &m.contacts[ci];
        for (0..@intCast(pm.contact_count)) |pi| {
            const pc = &pm.contacts[pi];
            // Cek feature pair — jika cocok, copy impulse
            if (c.fp.inI == pc.fp.inI and c.fp.outI == pc.fp.outI) {
                c.normal_impulse    = pc.normal_impulse;
                c.tangent_impulse   = pc.tangent_impulse;
                break;
            }
        }
        // Apply impulse dari frame sebelumnya langsung ke velocity
        const P_n = Vec3.scale(m.normal, c.normal_impulse);
        const P_t0 = Vec3.scale(m.tangent[0], c.tangent_impulse[0]);
        const P_t1 = Vec3.scale(m.tangent[1], c.tangent_impulse[1]);
        const P = Vec3.add(P_n, Vec3.add(P_t0, P_t1));
        ba.applyImpulse(Vec3.neg(P), c.position);
        bb.applyImpulse(P, c.position);
    }
}

// Pre-solve: hitung effective mass, bias, dan tangent basis
// Dipanggil SEKALI per manifold per frame, SEBELUM iterasi solver.
// Menggunakan velocity SAAT INI (setelah integrasi, sebelum solving)
// untuk menghitung restitusi yang benar.
fn preSolveManifold(world: *BoxWorld, m: *Manifold) void {
    const ba  = &world.bodies[m.body_a];
    const bb  = &world.bodies[m.body_b];
    const mA  = ba.inv_mass;
    const mB  = bb.inv_mass;
    const inv_dt: R = 1.0 / world.dt;

    // Hitung tangent basis
    var t0 = Vec3{};
    var t1 = Vec3{};
    computeTangentBasis(m.normal, &t0, &t1);
    m.tangent[0] = t0;
    m.tangent[1] = t1;

    const friction = @sqrt(ba.friction * bb.friction);
    const e_rest   = @min(ba.restitution, bb.restitution);

    for (0..@intCast(m.contact_count)) |ci| {
        var c = &m.contacts[ci];

        c.ra = Vec3.sub(c.position, ba.world_tx.position);
        c.rb = Vec3.sub(c.position, bb.world_tx.position);

        // ── Normal effective mass ──────────────────────────────
        const raCn = Vec3.cross(c.ra, m.normal);
        const rbCn = Vec3.cross(c.rb, m.normal);
        var nm = mA + mB;
        nm += Vec3.dot(raCn, ba.inv_inertia_world.mulV(raCn));
        nm += Vec3.dot(rbCn, bb.inv_inertia_world.mulV(rbCn));
        c.normal_mass = if (nm > 1e-10) 1.0 / nm else 0;

        // ── Tangent effective mass ─────────────────────────────
        for (0..2) |ti| {
            const tang  = m.tangent[ti];
            const raCt  = Vec3.cross(c.ra, tang);
            const rbCt  = Vec3.cross(c.rb, tang);
            var tm = mA + mB;
            tm += Vec3.dot(raCt, ba.inv_inertia_world.mulV(raCt));
            tm += Vec3.dot(rbCt, bb.inv_inertia_world.mulV(rbCt));
            c.tangent_mass[ti] = if (tm > 1e-10) 1.0 / tm else 0;
        }

        // ── Bias (Baumgarte position correction + restitution) ──
        //
        // Restitusi dihitung dari velocity PRE-solver (velocity saat ini,
        // tepat setelah integrasi). Ini mencegah restitusi ganda.
        const vA_c = ba.velocityAt(c.position);
        const vB_c = bb.velocityAt(c.position);
        const vn   = Vec3.dot(Vec3.sub(vB_c, vA_c), m.normal);

        // Baumgarte: dorong keluar dari penetrasi secara bertahap
        const baumgarte_bias = -BAUMGARTE * inv_dt *
            @min(@as(R, 0.0), c.penetration + PENETRATION_SLOP);

        // Restitusi: hanya jika kecepatan menutup cukup besar
        const restitution_bias = if (vn < -RESTITUTION_SLOP) e_rest * vn else 0.0;

        c.bias = baumgarte_bias + restitution_bias;

        // Friction factor disimpan — dipakai saat solve
        _ = friction; // dipakai di solveManifold
    }
}

// Solve satu iterasi — dipanggil SOLVER_ITERS kali per frame
fn solveManifold(world: *BoxWorld, m: *Manifold) void {
    const ba      = &world.bodies[m.body_a];
    const bb      = &world.bodies[m.body_b];
    const friction = @sqrt(ba.friction * bb.friction);

    for (0..@intCast(m.contact_count)) |ci| {
        var c = &m.contacts[ci];

        // ── Normal impulse ────────────────────────────────────
        {
            // Kecepatan relatif saat iterasi ini (sudah termasuk impulse sebelumnya)
            const vA = ba.velocityAt(c.position);
            const vB = bb.velocityAt(c.position);
            const vn = Vec3.dot(Vec3.sub(vB, vA), m.normal);

            // lambda = effective_mass * (-vn + bias)
            var lambda = c.normal_mass * (-vn + c.bias);

            // Clamp accumulated impulse ≥ 0 (tidak boleh tarik)
            const old_Pn = c.normal_impulse;
            c.normal_impulse = @max(old_Pn + lambda, 0.0);
            lambda = c.normal_impulse - old_Pn;

            const P = Vec3.scale(m.normal, lambda);
            ba.applyImpulse(Vec3.neg(P), c.position);
            bb.applyImpulse(P, c.position);
        }

        // ── Tangent impulse (gesekan) ─────────────────────────
        //
        // Friction cone: |Pt| ≤ mu * Pn
        // Kunci: clamp pakai ACCUMULATED normal impulse (c.normal_impulse),
        // BUKAN impulse normal iterasi ini.
        inline for (0..2) |ti| {
            const tang = m.tangent[ti];
            const vA   = ba.velocityAt(c.position);
            const vB   = bb.velocityAt(c.position);
            const vt   = Vec3.dot(Vec3.sub(vB, vA), tang);

            var lambda_t = c.tangent_mass[ti] * (-vt);
            const max_t  = friction * c.normal_impulse; // ← kunci tidak drift

            const old_Pt = c.tangent_impulse[ti];
            c.tangent_impulse[ti] = std.math.clamp(old_Pt + lambda_t, -max_t, max_t);
            lambda_t = c.tangent_impulse[ti] - old_Pt;

            const Pt = Vec3.scale(tang, lambda_t);
            ba.applyImpulse(Vec3.neg(Pt), c.position);
            bb.applyImpulse(Pt, c.position);
        }
    }
}

// ─────────────────────────────────────────────────────────────
// Integrasi — Symplectic Euler dengan sleep system
// ─────────────────────────────────────────────────────────────
fn integrateBodies(world: *BoxWorld) void {
    const g  = world.gravity;
    const dt = world.dt;

    for (0..world.n_bodies) |i| {
        var b = &world.bodies[i];
        if (b.is_static or !b.is_awake) continue;

        b.computeTransform();

        // Gravity + external force → velocity
        const grav = Vec3.scale(g, b.gravity_scale);
        b.lin_vel  = Vec3.add(b.lin_vel, Vec3.scale(
            Vec3.add(grav, Vec3.scale(b.force, b.inv_mass)), dt));
        b.ang_vel  = Vec3.add(b.ang_vel,
            b.inv_inertia_world.mulV(Vec3.scale(b.torque, dt)));

        // Padé damping
        const ld = 1.0 / (1.0 + dt * b.lin_damping);
        const ad = 1.0 / (1.0 + dt * b.ang_damping);
        b.lin_vel = Vec3.scale(b.lin_vel, ld);
        b.ang_vel = Vec3.scale(b.ang_vel, ad);

        // Integrate position
        b.position   = Vec3.add(b.position, Vec3.scale(b.lin_vel, dt));
        b.quaternion = b.quaternion.integrate(b.ang_vel, dt);
        b.quaternion = b.quaternion.normalized();

        b.computeTransform();

        b.force  = .{};
        b.torque = .{};

        // Sleep test — dual threshold linear + angular
        const lin_sq = Vec3.lenSq(b.lin_vel);
        const ang_sq = Vec3.lenSq(b.ang_vel);
        if (lin_sq < BOX_SLEEP_LINEAR_SQ and ang_sq < BOX_SLEEP_ANGULAR_SQ) {
            b.sleep_count += 1;
            if (b.sleep_count >= BOX_SLEEP_FRAMES) {
                b.is_awake    = false;
                b.lin_vel     = .{};
                b.ang_vel     = .{};
            }
        } else {
            b.sleep_count = 0;
        }
    }
}

// ─────────────────────────────────────────────────────────────
// boxWorldStep — satu frame
//
// Urutan operasi Sequential Impulses yang benar:
//   1. Integrasi (velocity + posisi)
//   2. Broad/narrow phase → bangun manifold baru
//   3. Warm starting: transfer impulse dari frame lalu
//   4. Pre-solve: hitung effective mass + bias (SEKALI)
//   5. Solver loop (SOLVER_ITERS kali)
//   6. Simpan manifold untuk frame berikutnya
// ─────────────────────────────────────────────────────────────
pub fn boxWorldStep(world: *BoxWorld) void {
    // 1. Integrasi
    integrateBodies(world);

    // 2. Narrow phase — bangun manifold baru
    const old_n = world.n_manifolds;
    _ = old_n;
    world.n_manifolds = 0;

    for (0..world.n_bodies) |i| {
        for (i + 1..world.n_bodies) |j| {
            const ba = &world.bodies[i];
            const bb = &world.bodies[j];
            if (ba.is_static and bb.is_static) continue;
            // Skip jika kedua body tidur — tidak ada yang bergerak
            if (!ba.is_awake and !bb.is_awake) continue;

            var m = Manifold{};
            if (boxToBox(&m, ba, bb, i, j)) {
                if (world.n_manifolds < MAX_MANIFOLDS) {
                    world.manifolds[world.n_manifolds] = m;
                    world.n_manifolds += 1;
                    // Bangunkan keduanya jika ada kontak baru
                    if (!ba.is_static) { ba.is_awake = true; ba.sleep_count = 0; }
                    if (!bb.is_static) { bb.is_awake = true; bb.sleep_count = 0; }
                }
            }
        }
    }

    // 3. Warm starting — transfer accumulated impulse dari frame lalu
    //    Harus dilakukan SEBELUM pre-solve supaya basis tangent sudah terset.
    //    Tapi basis dihitung di pre-solve... jadi kita hitung basis dulu,
    //    lalu warm start, lalu pre-solve lagi dengan velocity yang sudah warm.
    //
    //    Cara benar (qu3e / Box2D):
    //      a. Hitung tangent basis
    //      b. Warm start
    //      c. Pre-solve (hitung mass, bias — velocity sudah warm)
    for (0..world.n_manifolds) |mi| {
        var m = &world.manifolds[mi];
        // Hitung basis dulu supaya warm start bisa pakai m.tangent
        var t0 = Vec3{}; var t1 = Vec3{};
        computeTangentBasis(m.normal, &t0, &t1);
        m.tangent[0] = t0;
        m.tangent[1] = t1;
        warmStartManifold(world, m);
    }

    // 4. Pre-solve (velocity sekarang sudah include warm start impulse)
    for (0..world.n_manifolds) |mi| {
        preSolveManifold(world, &world.manifolds[mi]);
    }

    // 5. Solver loop
    for (0..SOLVER_ITERS) |_| {
        for (0..world.n_manifolds) |mi| {
            solveManifold(world, &world.manifolds[mi]);
        }
    }

    // 6. Simpan manifold frame ini untuk warm starting frame berikutnya
    world.n_manifolds_prev = world.n_manifolds;
    for (0..world.n_manifolds) |mi| {
        world.manifolds_prev[mi] = world.manifolds[mi];
    }
}

// ─────────────────────────────────────────────────────────────
// Helper render
// ─────────────────────────────────────────────────────────────
pub fn boxGetVertices(b: *const BoxBody, out: *[8]Vec3) void {
    const tx = b.getTransform();
    const e  = b.extent;
    const signs = [8][3]R{
        .{ -1, -1, -1 }, .{  1, -1, -1 }, .{ -1,  1, -1 }, .{  1,  1, -1 },
        .{ -1, -1,  1 }, .{  1, -1,  1 }, .{ -1,  1,  1 }, .{  1,  1,  1 },
    };
    for (0..8) |i|
        out[i] = tx.mulPoint(.{
            .x = signs[i][0] * e.x,
            .y = signs[i][1] * e.y,
            .z = signs[i][2] * e.z,
        });
}

pub const BOX_EDGES = [24]u8{
    0, 1,  2, 3,  4, 5,  6, 7,
    0, 2,  1, 3,  4, 6,  5, 7,
    0, 4,  1, 5,  2, 6,  3, 7,
};

pub const BOX_INDICES = [36]u8{
    0, 2, 1,  2, 3, 1,
    4, 5, 6,  5, 7, 6,
    0, 1, 4,  1, 5, 4,
    2, 6, 3,  3, 6, 7,
    0, 4, 2,  4, 6, 2,
    1, 3, 5,  3, 7, 5,
};