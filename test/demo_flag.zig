//! demo_flag.zig — Bendera berkibar (ZEPH f32)
//! Bendera dipasang vertikal, pin di sisi kiri (tiang).
//! Angin bertiup dari kiri ke kanan (arah +X).
//!
//! Mouse kiri = orbit  |  Scroll = zoom  |  R = reset
//! 1=Master  2=Wind on/off  3=Wind Strength
//! 4=Gust on/off  5=Gust Amp  6=Turbulence
//! 7=Damping  8=Stiffness

const std   = @import("std");
const rl    = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
});
const zeph  = @import("zeph");
const cloth = @import("cloth");

// ─────────────────────────────────────────────────────────────
// Grid — bendera tipis landscape (W > H)
// ─────────────────────────────────────────────────────────────
const W          : usize     = 20;   // lebar bendera (horizontal)
const H          : usize     = 12;   // tinggi bendera (vertikal)
const CONN_COUNT              = cloth.connCount(W, H);

/// Jarak antar joint (meter)
const SPACING    : zeph.Unit = 0.08;

/// Lebar dan tinggi bendera dalam meter
const FLAG_W     : zeph.Unit = @as(zeph.Unit, W-1) * SPACING; // 1.52 m
const FLAG_H     : zeph.Unit = @as(zeph.Unit, H-1) * SPACING; // 0.88 m

/// Massa total bendera (kg) — bendera ringan
const MASS_TOTAL : zeph.Unit = 0.3;

/// g = 9.8 m/s² / 60fps² per frame
const G_PER_FRAME: zeph.Unit = 9.8 / (60.0 * 60.0);

// ─────────────────────────────────────────────────────────────
// Tiang — posisi tiang di kiri
// ─────────────────────────────────────────────────────────────
/// Tinggi tiang dari bawah bendera ke atas (meter)
const POLE_EXTRA : f32 = 0.15;
const POLE_TOP_Y : f32 = FLAG_H + 1.8 + POLE_EXTRA;
const POLE_BOT_Y : f32 = -0.05;
const POLE_X     : f32 = 0.0;

// ─────────────────────────────────────────────────────────────
// Engine State
// ─────────────────────────────────────────────────────────────
var g_joints : [W * H]zeph.Joint           = undefined;
var g_conns  : [CONN_COUNT]zeph.Connection = undefined;
var g_bodies : [1]zeph.Body                = .{.{}};
var g_world  : zeph.World                  = .{};

// ─────────────────────────────────────────────────────────────
// Wind State — angin horizontal arah +X
// Bendera di plane XY, normal menghadap ±Z.
// Angin dari -X ke +X → force.x positif.
//
// Perhitungan skala:
//   joint_mass = 0.3 / (20*12) = 0.00125 kg
//   G_PER_FRAME = 9.8/3600 = 0.002722 m/frame²
//   Angin "sedang" ≈ 0.5g pada kain:
//     accel_target = 0.5 * G_PER_FRAME = 0.001361 m/frame²
//     force/joint  = accel * mass = 1.7e-6 N
//     wind_str     = force/joint / avg_quads_per_joint(≈2) ≈ 8.5e-7 N
// ─────────────────────────────────────────────────────────────
var wind_master  : bool = true;
var wind_on      : bool = true;
var wind_str     : f32  = 8.5e-7;   // N per quad per joint — angin sedang
var gust_on      : bool = true;
var gust_amp     : f32  = 0.35;     // amplitudo gust (fraksi wind_str)
var wind_turb    : f32  = 0.20;     // turbulensi spasial
var wind_time    : f32  = 0.0;
const GUST_FREQ  : f32  = 0.45;     // Hz gust

// ─────────────────────────────────────────────────────────────
// Cloth params — bisa diubah live
// ─────────────────────────────────────────────────────────────
var cloth_damp   : zeph.Unit = 0.015;
var cloth_stiff  : zeph.Unit = 0.95;
const CLOTH_RELAX: u32       = 16;

// ─────────────────────────────────────────────────────────────
// HUD
// ─────────────────────────────────────────────────────────────
var sel      : usize = 0;
const N_ITEMS: usize = 8;

// ─────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────
inline fn toRl(v: zeph.Vec3) rl.Vector3 { return .{ .x=v.x, .y=v.y, .z=v.z }; }
inline fn vsub(a: rl.Vector3, b: rl.Vector3) rl.Vector3 { return .{.x=a.x-b.x,.y=a.y-b.y,.z=a.z-b.z}; }
inline fn vlen(a: rl.Vector3) f32 { return @sqrt(a.x*a.x + a.y*a.y + a.z*a.z); }
inline fn gid(r: usize, c: usize) usize { return r * W + c; }
inline fn jpos(idx: usize) rl.Vector3 { return toRl(g_bodies[0].joints[idx].position); }
inline fn clampf(x: f32, lo: f32, hi: f32) f32 { return if (x<lo) lo else if (x>hi) hi else x; }
inline fn col(r: u8, g: u8, b: u8, a: u8) rl.Color { return .{.r=r,.g=g,.b=b,.a=a}; }

// ─────────────────────────────────────────────────────────────
// Init
// ─────────────────────────────────────────────────────────────
fn initScene() void {
    // Bangun kain di plane XY, center di origin
    // col=0 = kiri (dekat tiang), col=W-1 = kanan (ujung bebas)
    // row=0 = atas, row=H-1 = bawah
    // Center di (FLAG_W/2, FLAG_H/2, 0) → col=0 ada di x=0 (tiang)
    //                                       → col=W-1 ada di x=FLAG_W (ujung bebas)
    cloth.build(&g_joints, &g_conns, W, H, SPACING,
        zeph.mkVec3(FLAG_W * 0.5, FLAG_H * 0.5 + 1.8, 0), .xy, .full);

    cloth.clothBodyInit(&g_bodies[0], &g_joints, &g_conns, MASS_TOTAL);
    g_bodies[0].verlet_damping  = cloth_damp;
    g_bodies[0].verlet_relax    = CLOTH_RELAX;
    g_bodies[0].cloth_stiffness = cloth_stiff;

    // Pin seluruh sisi kiri (col=0, semua row) = tiang bendera
    for (0..H) |row| {
        cloth.clothPinAt(&g_bodies[0], W, 0, row);
    }

    zeph.worldInit(&g_world, &g_bodies, null);
    g_world.gravity = zeph.mkVec3(0, -G_PER_FRAME, 0);
    wind_time = 0.0;
}

// ─────────────────────────────────────────────────────────────
// Wind — angin dari -X ke +X, mengenai permukaan XY (normal ±Z)
// Untuk bendera di plane XY, normal quad = arah Z.
// Dot product angin (arah X) dengan normal (arah Z) = 0 → tidak efektif!
// Jadi kita apply force langsung ke arah X tanpa dot product,
// karena angin "menekan" permukaan dari depan (Z-).
// ─────────────────────────────────────────────────────────────
fn applyWind(strength: f32) void {
    const n = g_bodies[0].joints.len;
    for (0..H - 1) |row| {
        for (0..W - 1) |c2| {
            const ja = row * W + c2;
            const jb = row * W + c2 + 1;
            const jc = (row + 1) * W + c2;
            const jd = (row + 1) * W + c2 + 1;
            if (jd >= n) continue;

            // Normal quad dari cross product tepi
            const pa = toRl(g_bodies[0].joints[ja].position);
            const pb = toRl(g_bodies[0].joints[jb].position);
            const pc = toRl(g_bodies[0].joints[jc].position);
            const eax = pb.x-pa.x; const eay = pb.y-pa.y; const eaz = pb.z-pa.z;
            const ebx = pc.x-pa.x; const eby = pc.y-pa.y; const ebz = pc.z-pa.z;
            const nnx = eay*ebz - eaz*eby;
            const nny = eaz*ebx - eax*ebz;
            const nnz = eax*eby - eay*ebx;
            const nl  = @sqrt(nnx*nnx + nny*nny + nnz*nnz);
            if (nl < 1e-12) continue;

            // Dot angin (arah +Z, karena bendera di plane XY menghadap Z)
            // dengan normal quad — saat bendera datar, normal = (0,0,1) → dot = 1
            // saat bendera miring karena angin, dot berkurang (efek realistis)
            var dot = nnz / nl;   // komponen Z dari normal ternormalisasi
            if (dot < 0) dot = -dot;

            // Turbulensi spasial berbeda tiap quad
            const turb = 1.0 + wind_turb *
                @sin(@as(f32, @floatFromInt(row * 5 + c2 * 11)));

            // Force ke arah +Z (angin dari depan menekan ke belakang)
            const fz : f32 = strength * dot * turb;

            for ([4]usize{ ja, jb, jc, jd }) |fi| {
                if (g_bodies[0].joints[fi].isPinned()) continue;
                g_bodies[0].joints[fi].force.z += fz;
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────
// Physics Step
// ─────────────────────────────────────────────────────────────
fn physicsStep(dt: f32) void {
    // Sinkronkan params live
    g_bodies[0].verlet_damping  = cloth_damp;
    g_bodies[0].cloth_stiffness = cloth_stiff;

    if (wind_master and wind_on) {
        wind_time += dt;
        var strength = wind_str;
        if (gust_on) {
            const gust = 1.0 + gust_amp *
                @sin(wind_time * GUST_FREQ * std.math.pi * 2.0);
            strength *= gust;
        }
        applyWind(strength);
    }
    zeph.worldStepVerlet(&g_world);
}

// ─────────────────────────────────────────────────────────────
// Render
// ─────────────────────────────────────────────────────────────
fn renderScene() void {
    // Tiang bendera
    const pole_top = rl.Vector3{ .x=POLE_X, .y=POLE_TOP_Y, .z=0 };
    const pole_bot = rl.Vector3{ .x=POLE_X, .y=POLE_BOT_Y, .z=0 };
    rl.DrawCylinder(pole_bot, 0.012, 0.012, POLE_TOP_Y-POLE_BOT_Y, 8, col(160,130,80,255));
    rl.DrawSphere(pole_top, 0.025, col(220,180,80,255));

    // Kain
    for (g_bodies[0].connections) |conn| {
        if (conn.length < 0) continue;
        const pa   = jpos(conn.joint1);
        const pb   = jpos(conn.joint2);
        const di   = vlen(vsub(pa, pb));
        const rest = conn.length;
        const el   = (di - rest) / rest;
        const tt   = std.math.clamp(el * 3.0, 0.0, 1.0);

        // Warna bendera merah-putih berdasarkan posisi kolom (stripe vertikal)
        const col_idx = conn.joint1 % W;
        const stripe  = (col_idx * 2) / W; // 0 = kiri, 1 = kanan
        const base_r  : f32 = if (stripe == 0) 220.0 else 240.0;
        const base_g  : f32 = if (stripe == 0) 40.0  else 240.0;
        const base_b  : f32 = if (stripe == 0) 40.0  else 240.0;

        rl.DrawLine3D(pa, pb, if (tt < 0.5) col(
            @intFromFloat(base_r * (1.0 - tt*0.3)),
            @intFromFloat(base_g * (1.0 - tt*0.3)),
            @intFromFloat(base_b * (1.0 - tt*0.3)),
            220,
        ) else col(
            255,
            @intFromFloat(base_g * (1.0 - (tt-0.5)*0.6)),
            0, 235,
        ));
    }

    // Pin joints (tiang)
    for (0..H) |row| {
        const idx = gid(row, 0);
        rl.DrawSphere(jpos(idx), 0.012, col(220,180,80,255));
    }

    // Ground line
    rl.DrawLine3D(
        .{.x=-0.5,.y=0,.z=0},
        .{.x=FLAG_W+0.5,.y=0,.z=0},
        col(60,60,80,120)
    );
}

// ─────────────────────────────────────────────────────────────
// HUD
// ─────────────────────────────────────────────────────────────
const PX : i32 = 10;
const PW : i32 = 290;
const BH : i32 = 18;

fn drawRow(py: i32, label: [*:0]const u8, val_str: [*:0]const u8, pct: f32, active: bool, bar_col: rl.Color) void {
    rl.DrawRectangle(PX, py, PW, BH, if (active) col(30,50,80,200) else col(0,0,0,0));
    if (active) rl.DrawRectangleLines(PX, py, PW, BH, col(80,160,255,200));
    rl.DrawText(label, PX+6, py+3, 11, if (active) col(180,220,255,255) else col(150,160,180,220));
    const bx: i32 = PX+150; const bw: i32 = 90;
    rl.DrawRectangle(bx, py+5, bw, 8, col(20,20,35,200));
    const fill: i32 = @intFromFloat(@as(f32,bw) * std.math.clamp(pct,0,1));
    if (fill > 0) rl.DrawRectangle(bx, py+5, fill, 8, bar_col);
    rl.DrawText(val_str, bx+bw+6, py+3, 11, col(200,200,200,230));
}

fn drawToggle(py: i32, label: [*:0]const u8, on: bool, active: bool) void {
    drawRow(py, label, if (on) "ON " else "OFF", if (on) 1.0 else 0.0, active,
        if (on) col(60,200,80,220) else col(150,40,40,200));
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────
pub fn main() void {
    rl.SetConfigFlags(rl.FLAG_MSAA_4X_HINT | rl.FLAG_WINDOW_RESIZABLE);
    rl.InitWindow(1280, 720, "Flag in the Wind  |  ZEPH f32  |  Mouse=orbit  Scroll=zoom");
    defer rl.CloseWindow();
    rl.SetTargetFPS(60);
    initScene();

    var cam = rl.Camera3D{
        .position   = .{.x=0.5, .y=1.8, .z=5.0},
        .target     = .{.x=0.7, .y=1.8, .z=0.0},
        .up         = .{.x=0,   .y=1,   .z=0  },
        .fovy       = 50.0,
        .projection = rl.CAMERA_PERSPECTIVE,
    };
    var yaw        : f32 = 0.15;
    var pitch      : f32 = 0.05;
    var cdist      : f32 = 5.0;
    var prev_mouse      = rl.GetMousePosition();

    while (rl.WindowShouldClose() == false) {
        const dt = @min(rl.GetFrameTime(), 0.033);

        // Key input
        if (rl.IsKeyPressed(rl.KEY_ONE))   sel = 0;
        if (rl.IsKeyPressed(rl.KEY_TWO))   sel = 1;
        if (rl.IsKeyPressed(rl.KEY_THREE)) sel = 2;
        if (rl.IsKeyPressed(rl.KEY_FOUR))  sel = 3;
        if (rl.IsKeyPressed(rl.KEY_FIVE))  sel = 4;
        if (rl.IsKeyPressed(rl.KEY_SIX))   sel = 5;
        if (rl.IsKeyPressed(rl.KEY_SEVEN)) sel = 6;
        if (rl.IsKeyPressed(rl.KEY_EIGHT)) sel = 7;

        const left  = rl.IsKeyPressed(rl.KEY_LEFT)  or rl.IsKeyPressed(rl.KEY_A);
        const right = rl.IsKeyPressed(rl.KEY_RIGHT) or rl.IsKeyPressed(rl.KEY_D);

        switch (sel) {
            0 => { if (left or right) wind_master = !wind_master; },
            1 => { if (left or right) wind_on     = !wind_on;     },
            2 => {
                if (right) wind_str  = clampf(wind_str  + 1.0e-7, 0, 5.0e-6);
                if (left)  wind_str  = clampf(wind_str  - 1.0e-7, 0, 5.0e-6);
            },
            3 => { if (left or right) gust_on = !gust_on; },
            4 => {
                if (right) gust_amp  = clampf(gust_amp  + 0.05, 0, 1.0);
                if (left)  gust_amp  = clampf(gust_amp  - 0.05, 0, 1.0);
            },
            5 => {
                if (right) wind_turb = clampf(wind_turb + 0.05, 0, 0.8);
                if (left)  wind_turb = clampf(wind_turb - 0.05, 0, 0.8);
            },
            6 => {
                if (right) cloth_damp  = clampf(cloth_damp  + 0.002, 0, 0.08);
                if (left)  cloth_damp  = clampf(cloth_damp  - 0.002, 0, 0.08);
            },
            7 => {
                if (right) cloth_stiff = clampf(cloth_stiff + 0.05, 0.1, 1.0);
                if (left)  cloth_stiff = clampf(cloth_stiff - 0.05, 0.1, 1.0);
            },
            else => {},
        }

        if (rl.IsKeyPressed(rl.KEY_R)) initScene();
        physicsStep(dt);

        // Camera orbit
        const cur_mouse = rl.GetMousePosition();
        if (rl.IsMouseButtonDown(rl.MOUSE_BUTTON_LEFT)) {
            yaw   -= (cur_mouse.x - prev_mouse.x) * 0.005;
            pitch += (cur_mouse.y - prev_mouse.y) * 0.005;
            pitch  = std.math.clamp(pitch, -0.4, 1.2);
        }
        prev_mouse = cur_mouse;
        cdist     -= rl.GetMouseWheelMove() * 0.2;
        cdist      = std.math.clamp(cdist, 1.5, 12.0);
        const cx = @cos(pitch) * @sin(yaw) * cdist;
        const cy = @sin(pitch) * cdist;
        const cz = @cos(pitch) * @cos(yaw) * cdist;
        cam.position = .{.x=cx, .y=cy, .z=cz};
        cam.target   = .{.x=FLAG_W*0.5, .y=FLAG_H*0.5 + 1.8, .z=0.0};

        // Draw
        rl.BeginDrawing();
        defer rl.EndDrawing();
        rl.ClearBackground(col(10,12,22,255));
        rl.BeginMode3D(cam);
            renderScene();
        rl.EndMode3D();

        // HUD
        const total_h: i32 = 28 + @as(i32, N_ITEMS) * BH + 20;
        rl.DrawRectangle(PX-4, 4, PW+8, total_h, col(5,8,18,210));
        rl.DrawRectangleLines(PX-4, 4, PW+8, total_h, col(40,60,100,180));
        rl.DrawText("FLAG DEMO (f32)", PX, 10, 15, col(255,180,60,255));

        const py0: i32 = 30;
        drawToggle(py0 + 0*BH, "1 MASTER",        wind_master, sel==0);
        drawToggle(py0 + 1*BH, "2 Wind",          wind_on,     sel==1);
        drawRow   (py0 + 2*BH, "3   Strength",
            rl.TextFormat("%.1f uN", @as(f64, wind_str) * 1_000_000.0),
            wind_str / 5.0e-6, sel==2, col(100,180,255,220));
        drawToggle(py0 + 3*BH, "4 Gust",          gust_on,     sel==3);
        drawRow   (py0 + 4*BH, "5   Gust Amp",
            rl.TextFormat("%.2f", gust_amp),
            gust_amp, sel==4, col(100,200,140,220));
        drawRow   (py0 + 5*BH, "6 Turbulence",
            rl.TextFormat("%.2f", wind_turb),
            wind_turb/0.8, sel==5, col(200,160,60,220));
        drawRow   (py0 + 6*BH, "7 Damping",
            rl.TextFormat("%.3f", cloth_damp),
            cloth_damp/0.08, sel==6, col(160,100,220,220));
        drawRow   (py0 + 7*BH, "8 Stiffness",
            rl.TextFormat("%.2f", cloth_stiff),
            cloth_stiff, sel==7, col(220,120,100,220));

        const hint_y = py0 + @as(i32, N_ITEMS) * BH + 4;
        rl.DrawText("1-8 select   <-/-> change   R = reset",
            PX, hint_y, 10, col(80,90,120,200));
        rl.DrawFPS(1190, 8);
    }
}
