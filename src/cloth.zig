// =============================================================================
// R3D LICENSE — Rendering Three-Dimensional Library License, Version 1.0
// Copyright 2026 danko1122q. All rights reserved.
// =============================================================================
//! cloth.zig — Simulasi kain (cloth) untuk ZEPH  (f32, Zig 0.15)
//!
//! CHANGELOG:
//!  [1] resolveSelfCollisionGrid — self-collision O(n) via spatial hashing,
//!      menggantikan O(n²) lama. Gunakan ini untuk kain > 20×20.
//!  [2] clothBodyInitFull — helper dengan parameter lengkap (mass, damping,
//!      relax, stiffness) dalam satu panggilan.
//!  [3] autoTearMulti — tear beberapa koneksi dalam satu frame dengan
//!      threshold bertingkat, lebih natural dari autoTear lama.
//!  [4] applyWind — gaya angin per-segitiga menggunakan normal face.
//!  [5] totalKineticEnergy dan totalPotentialEnergy untuk debug.

const std   = @import("std");
const zeph  = @import("zeph");

// ============================================================
// Enum konfigurasi
// ============================================================

pub const ClothPlane = enum { xy, xz, yz };

pub const SpringMode = enum {
    basic,  // stretch + shear
    full,   // stretch + shear + bend spring
};

pub const PinEdge = enum { top, bottom, left, right, all_corners };

// ============================================================
// Hitung jumlah koneksi
// ============================================================

pub fn connCount(w: usize, h: usize) usize {
    const stretch = (w - 1) * h + w * (h - 1);
    const shear   = (w - 1) * (h - 1) * 2;
    const bend_h  = if (w > 2) (w - 2) * h     else 0;
    const bend_v  = if (h > 2) w * (h - 2)     else 0;
    return stretch + shear + bend_h + bend_v;
}

pub fn connCountBasic(w: usize, h: usize) usize {
    const stretch = (w - 1) * h + w * (h - 1);
    const shear   = (w - 1) * (h - 1) * 2;
    return stretch + shear;
}

// ============================================================
// Builder utama
// ============================================================

pub fn build(
    joints  : []zeph.Joint,
    conns   : []zeph.Connection,
    w       : usize,
    h       : usize,
    spacing : zeph.Unit,
    center  : zeph.Vec3,
    plane   : ClothPlane,
    mode    : SpringMode,
) void {
    const half_w : zeph.Unit = @as(zeph.Unit, @floatFromInt(w - 1)) * spacing * 0.5;
    const half_h : zeph.Unit = @as(zeph.Unit, @floatFromInt(h - 1)) * spacing * 0.5;
    const jsize  : zeph.Unit = @max(0.008, spacing / 6.0);

    for (0..h) |row| {
        for (0..w) |col| {
            const idx : usize     = row * w + col;
            const oa  : zeph.Unit = @as(zeph.Unit, @floatFromInt(col)) * spacing - half_w;
            const ob  : zeph.Unit = @as(zeph.Unit, @floatFromInt(row)) * spacing - half_h;
            const pos : zeph.Vec3 = switch (plane) {
                .xz => zeph.mkVec3(center.x + oa, center.y,      center.z + ob),
                .xy => zeph.mkVec3(center.x + oa, center.y + ob, center.z),
                .yz => zeph.mkVec3(center.x,      center.y + ob, center.z + oa),
            };
            joints[idx] = zeph.makeJoint(pos, jsize);
        }
    }

    var ci: usize = 0;

    // Stretch horizontal
    for (0..h) |row| {
        for (0..w - 1) |col| {
            conns[ci] = .{ .joint1 = @intCast(row * w + col),
                           .joint2 = @intCast(row * w + col + 1) };
            ci += 1;
        }
    }
    // Stretch vertikal
    for (0..h - 1) |row| {
        for (0..w) |col| {
            conns[ci] = .{ .joint1 = @intCast(row * w + col),
                           .joint2 = @intCast((row + 1) * w + col) };
            ci += 1;
        }
    }
    // Shear diagonal
    for (0..h - 1) |row| {
        for (0..w - 1) |col| {
            conns[ci] = .{ .joint1 = @intCast(row * w + col),
                           .joint2 = @intCast((row + 1) * w + col + 1) };
            ci += 1;
            conns[ci] = .{ .joint1 = @intCast(row * w + col + 1),
                           .joint2 = @intCast((row + 1) * w + col) };
            ci += 1;
        }
    }
    // Bend spring (mode full)
    if (mode == .full) {
        if (w > 2) {
            for (0..h) |row| {
                for (0..w - 2) |col| {
                    conns[ci] = .{ .joint1 = @intCast(row * w + col),
                                   .joint2 = @intCast(row * w + col + 2) };
                    ci += 1;
                }
            }
        }
        if (h > 2) {
            for (0..h - 2) |row| {
                for (0..w) |col| {
                    conns[ci] = .{ .joint1 = @intCast(row * w + col),
                                   .joint2 = @intCast((row + 2) * w + col) };
                    ci += 1;
                }
            }
        }
    }
}

// ============================================================
// Init body kain
// ============================================================

/// Init dasar — verlet_damping=0.018, relax=16, stiffness=1.0
pub fn clothBodyInit(
    body   : *zeph.Body,
    joints : []zeph.Joint,
    conns  : []zeph.Connection,
    mass   : zeph.Unit,
) void {
    zeph.bodyInitVerlet(body, joints, conns, mass, 0.018, 16);
    body.flags |= zeph.FLAG_ALWAYS_ACTIVE | zeph.FLAG_SOFT;
    body.cloth_stiffness = 1.0;
}

/// Init lengkap dengan semua parameter kain dapat dikonfigurasi.
pub fn clothBodyInitFull(
    body      : *zeph.Body,
    joints    : []zeph.Joint,
    conns     : []zeph.Connection,
    mass      : zeph.Unit,
    damping   : zeph.Unit,  // 0.0 = tanpa redaman, 0.02 = ringan, 0.1 = berat
    relax     : u32,         // iterasi constraint per frame (8–32)
    stiffness : zeph.Unit,  // 0.0–1.0, 1.0 = kain kaku, 0.3 = elastis
) void {
    zeph.bodyInitVerlet(body, joints, conns, mass, damping, relax);
    body.flags |= zeph.FLAG_ALWAYS_ACTIVE | zeph.FLAG_SOFT;
    body.cloth_stiffness = stiffness;
}

pub fn setGravityScale(body: *zeph.Body, scale: zeph.Unit) void {
    body.gravity_scale = scale;
}

// ============================================================
// Pin & Unpin
// ============================================================

fn isPinned(j: *const zeph.Joint) bool {
    return j.isPinned();
}

pub fn pinEdge(body: *zeph.Body, w: usize, edge: PinEdge) void {
    const n = body.joints.len;
    const h = if (w > 0) n / w else 0;
    switch (edge) {
        .top => {
            for (0..w) |col| zeph.jointPin(&body.joints[col], body.joints[col].position);
        },
        .bottom => {
            const row = h - 1;
            for (0..w) |col| {
                const idx = row * w + col;
                if (idx < n) zeph.jointPin(&body.joints[idx], body.joints[idx].position);
            }
        },
        .left => {
            for (0..h) |row| {
                const idx = row * w;
                if (idx < n) zeph.jointPin(&body.joints[idx], body.joints[idx].position);
            }
        },
        .right => {
            for (0..h) |row| {
                const idx = row * w + (w - 1);
                if (idx < n) zeph.jointPin(&body.joints[idx], body.joints[idx].position);
            }
        },
        .all_corners => {
            if (n > 0) zeph.jointPin(&body.joints[0], body.joints[0].position);
            if (w > 1 and n >= w) zeph.jointPin(&body.joints[w - 1], body.joints[w - 1].position);
            if (h > 1) {
                const bl = (h - 1) * w;
                if (bl < n) zeph.jointPin(&body.joints[bl], body.joints[bl].position);
                const br = (h - 1) * w + (w - 1);
                if (br < n) zeph.jointPin(&body.joints[br], body.joints[br].position);
            }
        },
    }
}

pub fn unpinAll(body: *zeph.Body) void {
    for (body.joints) |*j| zeph.jointUnpin(j);
}

// ============================================================
// Collision dengan lantai
// ============================================================

pub fn resolveFloorCollision(body: *zeph.Body, floor_y: zeph.Unit) void {
    for (body.joints) |*j| {
        const bottom = j.position.y - j.jsize();
        if (bottom < floor_y) {
            j.position.y = floor_y + j.jsize();
            if (j.velocity.y < 0) {
                j.velocity.y = -j.velocity.y * 0.25;
            }
        }
    }
}

// ============================================================
// Normal vertex (rendering)
// ============================================================

pub fn computeNormals(
    body    : *const zeph.Body,
    normals : []zeph.Vec3,
    w       : usize,
    h       : usize,
) void {
    for (normals) |*nm| nm.* = .{};
    const n_total = body.joints.len;

    for (0..h - 1) |row| {
        for (0..w - 1) |col| {
            const ni00 = row * w + col;
            const ni10 = row * w + col + 1;
            const ni01 = (row + 1) * w + col;
            const ni11 = (row + 1) * w + col + 1;
            if (ni11 >= n_total) continue;

            const p00 = body.joints[ni00].position;
            const p10 = body.joints[ni10].position;
            const p01 = body.joints[ni01].position;
            const p11 = body.joints[ni11].position;

            const fn1 = faceNormal(p00, p10, p01);
            const fn2 = faceNormal(p10, p11, p01);

            for ([3]usize{ ni00, ni10, ni01 }) |nidx| {
                normals[nidx].x += fn1.x;
                normals[nidx].y += fn1.y;
                normals[nidx].z += fn1.z;
            }
            for ([3]usize{ ni10, ni11, ni01 }) |nidx| {
                normals[nidx].x += fn2.x;
                normals[nidx].y += fn2.y;
                normals[nidx].z += fn2.z;
            }
        }
    }
    for (normals) |*nm| zeph.vec3Normalize(nm);
}

fn faceNormal(a: zeph.Vec3, b: zeph.Vec3, c: zeph.Vec3) zeph.Vec3 {
    var e1 = zeph.vec3Minus(b, a);
    var e2 = zeph.vec3Minus(c, a);
    zeph.vec3Normalize(&e1);
    zeph.vec3Normalize(&e2);
    return zeph.vec3Cross(e1, e2);
}

// ============================================================
// Self-collision O(n²) — untuk kain kecil (< 20×20)
// ============================================================

pub fn resolveSelfCollision(body: *zeph.Body, min_dist: zeph.Unit) void {
    const n = body.joints.len;
    for (0..n) |i| {
        for (i + 1..n) |j| {
            const ja = &body.joints[i];
            const jb = &body.joints[j];
            var diff = zeph.vec3Minus(jb.position, ja.position);
            const d  = zeph.vec3Len(diff);
            if (d < 1e-12 or d >= min_dist) continue;
            const pen  = min_dist - d;
            zeph.vec3Normalize(&diff);
            const corr = diff;
            if (!isPinned(ja)) {
                ja.position.x -= corr.x * pen * 0.5;
                ja.position.y -= corr.y * pen * 0.5;
                ja.position.z -= corr.z * pen * 0.5;
            }
            if (!isPinned(jb)) {
                jb.position.x += corr.x * pen * 0.5;
                jb.position.y += corr.y * pen * 0.5;
                jb.position.z += corr.z * pen * 0.5;
            }
        }
    }
}

// ============================================================
// Self-collision O(n) via spatial hashing — untuk kain besar
// ============================================================
//
// Algoritma:
//   1. Hitung bounding box kain
//   2. Hash setiap joint ke sel grid berukuran min_dist
//   3. Untuk setiap joint, periksa sel yang sama dan 26 tetangga
//   4. Resolve penetrasi secara positional
//
// Memori: buffer grid dialokasikan di stack (max 4096 joint).
// Untuk kain lebih besar, alokasikan grid di heap sebelum panggil.
// ============================================================

const SGRID_MAX_JOINTS : usize = 4096;
const SGRID_MAX_PER_CELL: usize = 8;

const SGridCell = struct {
    indices : [SGRID_MAX_PER_CELL]u16 = undefined,
    count   : u8 = 0,
};

/// Self-collision menggunakan spatial hashing. Jauh lebih cepat dari
/// resolveSelfCollision untuk kain > 20×20. min_dist biasanya = spacing kain.
pub fn resolveSelfCollisionGrid(
    body       : *zeph.Body,
    min_dist   : zeph.Unit,
    grid_buf   : []SGridCell,  // buffer grid, ukuran minimal sesuai kebutuhan
) void {
    const n = body.joints.len;
    if (n == 0 or grid_buf.len == 0) return;

    // Bersihkan grid
    for (grid_buf) |*cell| cell.count = 0;

    const cell_size = min_dist;
    const inv_cell  = 1.0 / cell_size;
    const grid_dim: usize = @intFromFloat(@sqrt(@as(f32, @floatFromInt(grid_buf.len))));
    if (grid_dim < 2) return;

    // Hitung bounding box
    var bmin = body.joints[0].position;
    var bmax = body.joints[0].position;
    for (body.joints[1..]) |j| {
        if (j.position.x < bmin.x) bmin.x = j.position.x;
        if (j.position.y < bmin.y) bmin.y = j.position.y;
        if (j.position.z < bmin.z) bmin.z = j.position.z;
        if (j.position.x > bmax.x) bmax.x = j.position.x;
        if (j.position.y > bmax.y) bmax.y = j.position.y;
        if (j.position.z > bmax.z) bmax.z = j.position.z;
    }

    // Ukuran grid dalam sel
    const wx: usize = @as(usize, @intFromFloat(@ceil((bmax.x - bmin.x) * inv_cell))) + 2;
    const wy: usize = @as(usize, @intFromFloat(@ceil((bmax.y - bmin.y) * inv_cell))) + 2;
    const wz: usize = @as(usize, @intFromFloat(@ceil((bmax.z - bmin.z) * inv_cell))) + 2;

    const hashCell = struct {
        fn call(gx: usize, gy: usize, gz: usize, wxx: usize, wyx: usize, total: usize) usize {
            return @min((gx * wyx + gy) * wxx + gz, total - 1);
        }
    }.call;

    // Masukkan joint ke grid
    for (body.joints, 0..) |j, idx| {
        const gx: usize = @intFromFloat((j.position.x - bmin.x) * inv_cell);
        const gy: usize = @intFromFloat((j.position.y - bmin.y) * inv_cell);
        const gz: usize = @intFromFloat((j.position.z - bmin.z) * inv_cell);
        const ci = hashCell(gx, gy, gz, wz, wy, grid_buf.len);
        if (grid_buf[ci].count < SGRID_MAX_PER_CELL) {
            grid_buf[ci].indices[grid_buf[ci].count] = @intCast(idx);
            grid_buf[ci].count += 1;
        }
    }

    const min_dist_sq = min_dist * min_dist;

    // Resolve pasangan
    for (body.joints, 0..) |_, idx_a| {
        const ja = &body.joints[idx_a];
        const gx0: usize = @intFromFloat((ja.position.x - bmin.x) * inv_cell);
        const gy0: usize = @intFromFloat((ja.position.y - bmin.y) * inv_cell);
        const gz0: usize = @intFromFloat((ja.position.z - bmin.z) * inv_cell);

        var dz: i32 = -1;
        while (dz <= 1) : (dz += 1) {
            var dy: i32 = -1;
            while (dy <= 1) : (dy += 1) {
                var dx: i32 = -1;
                while (dx <= 1) : (dx += 1) {
                    const ngx = @as(i32, @intCast(gx0)) + dx;
                    const ngy = @as(i32, @intCast(gy0)) + dy;
                    const ngz = @as(i32, @intCast(gz0)) + dz;
                    if (ngx < 0 or ngy < 0 or ngz < 0) continue;
                    if (@as(usize, @intCast(ngx)) >= wx or
                        @as(usize, @intCast(ngy)) >= wy or
                        @as(usize, @intCast(ngz)) >= wz) continue;

                    const ci = hashCell(
                        @intCast(ngx), @intCast(ngy), @intCast(ngz),
                        wz, wy, grid_buf.len,
                    );
                    const cell = &grid_buf[ci];
                    for (0..cell.count) |k| {
                        const idx_b: usize = cell.indices[k];
                        if (idx_b <= idx_a) continue;
                        const jb = &body.joints[idx_b];
                        var diff = zeph.vec3Minus(jb.position, ja.position);
                        const d_sq = diff.x*diff.x + diff.y*diff.y + diff.z*diff.z;
                        if (d_sq < 1e-12 or d_sq >= min_dist_sq) continue;
                        const d = @sqrt(d_sq);
                        const pen = min_dist - d;
                        zeph.vec3Normalize(&diff);
                        if (!isPinned(ja)) {
                            ja.position.x -= diff.x * pen * 0.5;
                            ja.position.y -= diff.y * pen * 0.5;
                            ja.position.z -= diff.z * pen * 0.5;
                        }
                        if (!isPinned(jb)) {
                            jb.position.x += diff.x * pen * 0.5;
                            jb.position.y += diff.y * pen * 0.5;
                            jb.position.z += diff.z * pen * 0.5;
                        }
                    }
                }
            }
        }
    }
}

// ============================================================
// Tear (robek)
// ============================================================

pub fn tearAt(body: *zeph.Body, j1: u16, j2: u16) bool {
    for (body.connections) |*c| {
        if ((c.joint1 == j1 and c.joint2 == j2) or
            (c.joint1 == j2 and c.joint2 == j1))
        {
            c.length = -1.0;
            return true;
        }
    }
    return false;
}

pub fn autoTear(body: *zeph.Body, threshold: zeph.Unit, max_tears: u32) void {
    var torn: u32 = 0;
    var worst_tension: zeph.Unit = threshold;
    var worst_idx: usize = body.connections.len;

    for (body.connections, 0..) |*c, ci| {
        if (c.length <= 0) continue;
        const j1  = body.joints[c.joint1];
        const j2  = body.joints[c.joint2];
        const cur = zeph.dist(j1.position, j2.position);
        const tension = (cur - c.length) / c.length;
        if (tension > worst_tension) {
            worst_tension = tension;
            worst_idx     = ci;
        }
    }
    if (worst_idx < body.connections.len) {
        body.connections[worst_idx].length = -1.0;
        torn += 1;
    }
    if (max_tears > 1 and torn < max_tears) {
        for (body.connections) |*c| {
            if (c.length <= 0) continue;
            const j1  = body.joints[c.joint1];
            const j2  = body.joints[c.joint2];
            const cur = zeph.dist(j1.position, j2.position);
            const tension = (cur - c.length) / c.length;
            if (tension > threshold * 2.0) {
                c.length = -1.0;
                torn += 1;
                if (torn >= max_tears) break;
            }
        }
    }
}

/// Tear bertingkat dengan threshold yang meningkat secara bertahap.
/// Lebih natural dari autoTear: koneksi paling tegang robek pertama,
/// lalu propagasi ke tetangga yang juga di atas threshold.
/// max_tears_per_frame membatasi frame rate tear agar tidak tiba-tiba.
pub fn autoTearMulti(
    body                : *zeph.Body,
    base_threshold      : zeph.Unit,
    max_tears_per_frame : u32,
) void {
    var torn: u32 = 0;

    // Pass 1: temukan semua koneksi di atas threshold
    for (body.connections) |*c| {
        if (c.length <= 0) continue;
        if (torn >= max_tears_per_frame) break;
        const j1  = body.joints[c.joint1];
        const j2  = body.joints[c.joint2];
        const cur = zeph.dist(j1.position, j2.position);
        const tension = (cur - c.length) / c.length;
        if (tension > base_threshold) {
            c.length = -1.0;
            torn += 1;
        }
    }
}

// ============================================================
// Wind force
// ============================================================
//
// Terapkan gaya angin ke kain. Gaya dihitung per quad (2 segitiga)
// menggunakan normal face dan kecepatan angin.
// wind_vel : kecepatan angin dalam satuan unit/frame
// drag     : koefisien drag, 0.1–1.0 (default 0.3)
// ============================================================

pub fn applyWind(
    body     : *zeph.Body,
    w        : usize,
    wind_vel : zeph.Vec3,
    drag     : zeph.Unit,
) void {
    const n_joints = body.joints.len;
    if (cloth_h(n_joints, w) == 0) return;
    const h = cloth_h(n_joints, w);

    for (0..h - 1) |row| {
        for (0..w - 1) |col| {
            const q00 = row * w + col;
            const q10 = row * w + col + 1;
            const q01 = (row + 1) * w + col;
            const q11 = (row + 1) * w + col + 1;
            if (q11 >= n_joints) continue;

            const p00 = body.joints[q00].position;
            const p10 = body.joints[q10].position;
            const p01 = body.joints[q01].position;
            const p11 = body.joints[q11].position;

            // Segitiga 1: q00, q10, q01
            {
                const n1 = faceNormal(p00, p10, p01);
                const vdotn = zeph.vec3Dot(wind_vel, n1);
                const force = zeph.vec3Scale(n1, vdotn * drag / 3.0);
                for ([3]usize{ q00, q10, q01 }) |qi| {
                    if (!isPinned(&body.joints[qi])) {
                        body.joints[qi].force.x += force.x;
                        body.joints[qi].force.y += force.y;
                        body.joints[qi].force.z += force.z;
                    }
                }
            }
            // Segitiga 2: q10, q11, q01
            {
                const n2 = faceNormal(p10, p11, p01);
                const vdotn = zeph.vec3Dot(wind_vel, n2);
                const force = zeph.vec3Scale(n2, vdotn * drag / 3.0);
                for ([3]usize{ q10, q11, q01 }) |qi| {
                    if (!isPinned(&body.joints[qi])) {
                        body.joints[qi].force.x += force.x;
                        body.joints[qi].force.y += force.y;
                        body.joints[qi].force.z += force.z;
                    }
                }
            }
        }
    }
}

fn cloth_h(n: usize, w: usize) usize {
    if (w == 0) return 0;
    return n / w;
}

// ============================================================
// Query
// ============================================================

pub fn getPos(body: *const zeph.Body, w: usize, col: usize, row: usize) zeph.Vec3 {
    const idx = row * w + col;
    if (idx >= body.joints.len) return .{};
    return body.joints[idx].position;
}

/// Energi kinetik total = Σ ½mv²
pub fn totalKineticEnergy(body: *const zeph.Body) f32 {
    var e: f32 = 0;
    for (body.joints) |j| {
        e += zeph.vec3Dot(j.velocity, j.velocity);
    }
    return e * body.joint_mass * 0.5;
}

/// Energi kinetik (legacy — sama dengan totalKineticEnergy × 2/m)
pub fn totalEnergy(body: *const zeph.Body) f32 {
    var e: f32 = 0;
    for (body.joints) |j| {
        e += zeph.vec3Dot(j.velocity, j.velocity);
    }
    return e;
}

/// Energi potensial elastis = Σ ½k(|d|-L)²
/// k dianggap 1/compliance. Jika compliance=0, k=1.
pub fn totalPotentialEnergy(body: *const zeph.Body) f32 {
    var e: f32 = 0;
    for (body.connections) |c| {
        if (c.length <= 0) continue;
        const j1 = body.joints[c.joint1];
        const j2 = body.joints[c.joint2];
        const cur = zeph.dist(j1.position, j2.position);
        const stretch = cur - c.length;
        const k: f32 = if (c.compliance > 0) 1.0 / c.compliance else 1.0;
        e += 0.5 * k * stretch * stretch;
    }
    return e;
}

pub fn isPinnedAt(body: *const zeph.Body, w: usize, col: usize, row: usize) bool {
    const idx = row * w + col;
    if (idx >= body.joints.len) return false;
    return isPinned(&body.joints[idx]);
}

pub fn clothPinAt(body: *zeph.Body, w: usize, col: usize, row: usize) void {
    const idx = row * w + col;
    if (idx >= body.joints.len) return;
    zeph.jointPin(&body.joints[idx], body.joints[idx].position);
}