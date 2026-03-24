//! demo_cloth.zig — Cloth in the wind  (ZEPH f32)
//! Left mouse = orbit  |  Scroll = zoom  |  R = reset
//!
//! 1=Master  2=Front on/off  3=Front Strength
//! 4=Back on/off  5=Back Strength
//! 6=Right on/off  7=Left on/off  8=Side Strength  9=Turbulence

const std   = @import("std");
const rl    = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
});
const zeph  = @import("zeph");
const cloth = @import("cloth");

// ─────────────────────────────────────────────────────────────
// Grid & Physics  (satuan: meter, detik → unit/frame @ 60 fps)
// ─────────────────────────────────────────────────────────────
const W          : usize     = 16;
const H          : usize     = 16;
const CONN_COUNT              = cloth.connCount(W, H);

/// Jarak antar joint dalam meter.
const SPACING    : zeph.Unit = 0.10;
const HALF_H_N   : zeph.Unit = @as(zeph.Unit, H-1) * SPACING * 0.5;

/// Massa total kain (kg).  joint_mass = MASS_TOTAL / (W*H).
const MASS_TOTAL : zeph.Unit = 2.0;

/// Gravitasi dalam unit/frame².  g = 9.8 m/s² / 60² ≈ 0.00272.
const G_PER_FRAME: zeph.Unit = 9.8 / (60.0 * 60.0);

// ─────────────────────────────────────────────────────────────
// Engine State
// ─────────────────────────────────────────────────────────────
var g_joints : [W * H]zeph.Joint           = undefined;
var g_conns  : [CONN_COUNT]zeph.Connection = undefined;
var g_bodies : [1]zeph.Body                = .{.{}};
var g_world  : zeph.World                  = .{};

// ─────────────────────────────────────────────────────────────
// Wind State
// ─────────────────────────────────────────────────────────────
var wind_master   : bool = true;
var wind_front_on : bool = true;
var wind_front_str: f32  = 2.5e-6;
var wind_back_on  : bool = false;
var wind_back_str : f32  = 2.0e-6;
var wind_right_on : bool = false;
var wind_left_on  : bool = false;
var wind_side_str : f32  = 1.5e-6;
var wind_turb     : f32  = 0.15;
var wind_time     : f32  = 0.0;
const GUST_FREQ   : f32  = 0.30;
const GUST_AMP    : f32  = 0.20;

// ─────────────────────────────────────────────────────────────
// Cloth damping & relax
// ─────────────────────────────────────────────────────────────
const CLOTH_RELAX : u32       = 16;
const CLOTH_DAMP  : zeph.Unit = 0.018;

// ─────────────────────────────────────────────────────────────
// HUD Panel
// ─────────────────────────────────────────────────────────────
var sel      : usize = 0;
const N_ITEMS: usize = 9;

// ─────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────
inline fn toRl(v: zeph.Vec3) rl.Vector3 {
    return .{ .x = v.x, .y = v.y, .z = v.z };
}
inline fn vsub(a: rl.Vector3, b: rl.Vector3) rl.Vector3 {
    return .{.x=a.x-b.x, .y=a.y-b.y, .z=a.z-b.z};
}
inline fn vlen(a: rl.Vector3) f32 {
    return @sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}
inline fn gid(r: usize, c: usize) usize { return r * W + c; }
inline fn jpos(idx: usize) rl.Vector3   { return toRl(g_bodies[0].joints[idx].position); }
inline fn clampf(x: f32, lo: f32, hi: f32) f32 {
    return if (x < lo) lo else if (x > hi) hi else x;
}
inline fn col(r: u8, g: u8, b: u8, a: u8) rl.Color {
    return .{ .r=r, .g=g, .b=b, .a=a };
}

// ─────────────────────────────────────────────────────────────
// Init
// ─────────────────────────────────────────────────────────────
fn initScene() void {
    cloth.build(&g_joints, &g_conns, W, H, SPACING,
        zeph.mkVec3(0, -HALF_H_N, 0), .xy, .full);
    cloth.clothBodyInit(&g_bodies[0], &g_joints, &g_conns, MASS_TOTAL);
    g_bodies[0].verlet_damping  = CLOTH_DAMP;
    g_bodies[0].verlet_relax    = CLOTH_RELAX;
    g_bodies[0].cloth_stiffness = 1.0;
    cloth.clothPinAt(&g_bodies[0], W, 0,     H - 1);
    cloth.clothPinAt(&g_bodies[0], W, W - 1, H - 1);
    zeph.worldInit(&g_world, &g_bodies, null);
    g_world.gravity = zeph.mkVec3(0, -G_PER_FRAME, 0);
    wind_time = 0.0;
}

// ─────────────────────────────────────────────────────────────
// Wind  (force langsung ke force accumulator joint)
// ─────────────────────────────────────────────────────────────
fn applyWind(fx_str: f32, fz_str: f32) void {
    const wl = @sqrt(fx_str*fx_str + fz_str*fz_str);
    if (wl < 1e-12) return;
    const n = g_bodies[0].joints.len;
    for (0..H - 1) |row| {
        for (0..W - 1) |c2| {
            const ja = row * W + c2;
            const jb = row * W + c2 + 1;
            const jc = (row + 1) * W + c2;
            const jd = (row + 1) * W + c2 + 1;
            if (jd >= n) continue;
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
            var dot = (nnx*(fx_str/wl) + nnz*(fz_str/wl)) / nl;
            if (dot < 0) dot = -dot;
            const turb = 1.0 + wind_turb *
                @sin(@as(f32, @floatFromInt(row * 3 + c2 * 7)));
            const fx : f32 = fx_str * dot * turb;
            const fz : f32 = fz_str * dot * turb;
            for ([4]usize{ ja, jb, jc, jd }) |fi| {
                if (g_bodies[0].joints[fi].isPinned()) continue;
                g_bodies[0].joints[fi].force.x += fx;
                g_bodies[0].joints[fi].force.z += fz;
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────
// Physics Step
// ─────────────────────────────────────────────────────────────
fn physicsStep(dt: f32) void {
    if (wind_master) {
        const any = wind_front_on or wind_back_on or
                    wind_right_on or wind_left_on;
        if (any) {
            wind_time += dt;
            const gust = 1.0 + GUST_AMP *
                @sin(wind_time * GUST_FREQ * std.math.pi * 2.0);
            var gz: f32 = 0.0;
            var gx: f32 = 0.0;
            if (wind_front_on)  gz += wind_front_str * gust;
            if (wind_back_on)   gz -= wind_back_str  * gust;
            if (wind_right_on)  gx += wind_side_str  * gust;
            if (wind_left_on)   gx -= wind_side_str  * gust;
            if (gz != 0.0 or gx != 0.0) applyWind(gx, gz);
        }
    }
    zeph.worldStepVerlet(&g_world);
}

// ─────────────────────────────────────────────────────────────
// Render Mesh
// ─────────────────────────────────────────────────────────────
fn renderCloth() void {
    for (g_bodies[0].connections) |conn| {
        if (conn.length < 0) continue;
        const pa   = jpos(conn.joint1);
        const pb   = jpos(conn.joint2);
        const di   = vlen(vsub(pa, pb));
        const rest = conn.length;
        const el   = (di - rest) / rest;
        const tt   = std.math.clamp(el * 2.5, 0.0, 1.0);
        rl.DrawLine3D(pa, pb, if (tt < 0.5) col(
            @intFromFloat(tt * 2.0 * 160.0),
            @intFromFloat(120.0 + (1.0 - tt*2.0) * 120.0),
            @intFromFloat((1.0 - tt*2.0) * 170.0), 220,
        ) else col(
            255,
            @intFromFloat((1.0 - (tt-0.5)*2.0) * 130.0),
            0, 235,
        ));
    }
    for (g_bodies[0].joints) |j| {
        const pinned = j.isPinned();
        rl.DrawSphere(toRl(j.position),
            if (pinned) @as(f32, 0.025) else @as(f32, 0.008),
            if (pinned) col(255,140,40,255) else col(50,170,255,110));
    }
    const pin0 = jpos(gid(H-1, 0));
    const pin1 = jpos(gid(H-1, W-1));
    rl.DrawLine3D(pin0, .{.x=pin0.x,.y=0.45,.z=0}, col(180,120,60,200));
    rl.DrawLine3D(pin1, .{.x=pin1.x,.y=0.45,.z=0}, col(180,120,60,200));
}

// ─────────────────────────────────────────────────────────────
// HUD helpers
// ─────────────────────────────────────────────────────────────
const PX : i32 = 10;
const PW : i32 = 280;
const BH : i32 = 18;

fn drawRow(
    py      : i32,
    label   : [*:0]const u8,
    val_str : [*:0]const u8,
    pct     : f32,
    active  : bool,
    bar_col : rl.Color,
) void {
    rl.DrawRectangle(PX, py, PW, BH,
        if (active) col(30,50,80,200) else col(0,0,0,0));
    if (active)
        rl.DrawRectangleLines(PX, py, PW, BH, col(80,160,255,200));
    rl.DrawText(label, PX + 6, py + 3, 11,
        if (active) col(180,220,255,255) else col(150,160,180,220));
    const bx : i32 = PX + 140;
    const bw : i32 = 90;
    rl.DrawRectangle(bx, py + 5, bw, 8, col(20,20,35,200));
    const fill : i32 = @intFromFloat(@as(f32, bw) * std.math.clamp(pct, 0, 1));
    if (fill > 0) rl.DrawRectangle(bx, py + 5, fill, 8, bar_col);
    rl.DrawText(val_str, bx + bw + 6, py + 3, 11, col(200,200,200,230));
}

fn drawToggle(py: i32, label: [*:0]const u8, on: bool, active: bool) void {
    drawRow(py, label, if (on) "ON " else "OFF",
        if (on) 1.0 else 0.0, active,
        if (on) col(60,200,80,220) else col(150,40,40,200));
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────
pub fn main() void {
    rl.SetConfigFlags(rl.FLAG_MSAA_4X_HINT | rl.FLAG_WINDOW_RESIZABLE);
    rl.InitWindow(1280, 720, "Cloth in the Wind  |  Mouse=orbit  Scroll=zoom");
    defer rl.CloseWindow();
    rl.SetTargetFPS(60);
    initScene();

    var cam = rl.Camera3D{
        .position   = .{.x=0, .y=1.0,  .z=3.0},
        .target     = .{.x=0, .y=-0.5, .z=0 },
        .up         = .{.x=0, .y=1,    .z=0 },
        .fovy       = 55.0,
        .projection = rl.CAMERA_PERSPECTIVE,
    };
    var yaw        : f32 = 0.0;
    var pitch      : f32 = 0.28;
    var cdist      : f32 = 3.5;
    var prev_mouse      = rl.GetMousePosition();

    while (rl.WindowShouldClose() == false) {
        const dt = @min(rl.GetFrameTime(), 0.033);

        if (rl.IsKeyPressed(rl.KEY_ONE))   sel = 0;
        if (rl.IsKeyPressed(rl.KEY_TWO))   sel = 1;
        if (rl.IsKeyPressed(rl.KEY_THREE)) sel = 2;
        if (rl.IsKeyPressed(rl.KEY_FOUR))  sel = 3;
        if (rl.IsKeyPressed(rl.KEY_FIVE))  sel = 4;
        if (rl.IsKeyPressed(rl.KEY_SIX))   sel = 5;
        if (rl.IsKeyPressed(rl.KEY_SEVEN)) sel = 6;
        if (rl.IsKeyPressed(rl.KEY_EIGHT)) sel = 7;
        if (rl.IsKeyPressed(rl.KEY_NINE))  sel = 8;

        const left  = rl.IsKeyPressed(rl.KEY_LEFT)  or rl.IsKeyPressed(rl.KEY_A);
        const right = rl.IsKeyPressed(rl.KEY_RIGHT) or rl.IsKeyPressed(rl.KEY_D);

        switch (sel) {
            0 => { if (left or right) wind_master    = !wind_master;    },
            1 => { if (left or right) wind_front_on  = !wind_front_on;  },
            2 => {
                if (right) wind_front_str = clampf(wind_front_str + 5e-7, 0, 2e-5);
                if (left)  wind_front_str = clampf(wind_front_str - 5e-7, 0, 2e-5);
            },
            3 => { if (left or right) wind_back_on   = !wind_back_on;   },
            4 => {
                if (right) wind_back_str  = clampf(wind_back_str  + 5e-7, 0, 2e-5);
                if (left)  wind_back_str  = clampf(wind_back_str  - 5e-7, 0, 2e-5);
            },
            5 => { if (left or right) wind_right_on  = !wind_right_on;  },
            6 => { if (left or right) wind_left_on   = !wind_left_on;   },
            7 => {
                if (right) wind_side_str  = clampf(wind_side_str  + 3e-7, 0, 1.2e-5);
                if (left)  wind_side_str  = clampf(wind_side_str  - 3e-7, 0, 1.2e-5);
            },
            8 => {
                if (right) wind_turb      = clampf(wind_turb      + 0.05, 0, 0.5);
                if (left)  wind_turb      = clampf(wind_turb      - 0.05, 0, 0.5);
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
            pitch  = std.math.clamp(pitch, -0.3, 1.4);
        }
        prev_mouse  = cur_mouse;
        cdist      -= rl.GetMouseWheelMove() * 0.3;
        cdist       = std.math.clamp(cdist, 1.0, 8.0);
        cam.position.x = @cos(pitch) * @sin(yaw) * cdist;
        cam.position.y = @sin(pitch) * cdist;
        cam.position.z = @cos(pitch) * @cos(yaw) * cdist;

        var alive: u32 = 0; var torn: u32 = 0;
        for (g_bodies[0].connections) |c|
            if (c.length < 0) { torn+=1; } else { alive+=1; };

        rl.BeginDrawing();
        defer rl.EndDrawing();
        rl.ClearBackground(col(8,8,16,255));
        rl.BeginMode3D(cam);
            renderCloth();
        rl.EndMode3D();

        // HUD
        const total_h : i32 = 28 + @as(i32, N_ITEMS) * BH + 26;
        rl.DrawRectangle(PX-4, 4, PW+8, total_h, col(5,8,18,210));
        rl.DrawRectangleLines(PX-4, 4, PW+8, total_h, col(40,60,100,180));
        rl.DrawText("CLOTH DEMO (f32)", PX, 10, 15, col(155,215,255,255));
        rl.DrawText(rl.TextFormat("Springs: %d  Torn: %d", alive, torn),
            PX, 26, 11, rl.LIGHTGRAY);

        const py0 : i32 = 40;
        drawToggle(py0 + 0*BH, "1 MASTER WIND",   wind_master,   sel==0);
        drawToggle(py0 + 1*BH, "2 Front",          wind_front_on, sel==1);
        drawRow   (py0 + 2*BH, "3   Front Str",
            rl.TextFormat("%.4f", wind_front_str),
            wind_front_str/2e-5, sel==2, col(60,160,255,220));
        drawToggle(py0 + 3*BH, "4 Back",           wind_back_on,  sel==3);
        drawRow   (py0 + 4*BH, "5   Back Str",
            rl.TextFormat("%.4f", wind_back_str),
            wind_back_str/2e-5, sel==4, col(60,160,255,220));
        drawToggle(py0 + 5*BH, "6 Right",          wind_right_on, sel==5);
        drawToggle(py0 + 6*BH, "7 Left",           wind_left_on,  sel==6);
        drawRow   (py0 + 7*BH, "8   Side Str",
            rl.TextFormat("%.4f", wind_side_str),
            wind_side_str/1.2e-5, sel==7, col(60,220,120,220));
        drawRow   (py0 + 8*BH, "9 Turbulence",
            rl.TextFormat("%.2f", wind_turb),
            wind_turb/0.5, sel==8, col(200,160,60,220));

        const hint_y = py0 + @as(i32, N_ITEMS) * BH + 4;
        rl.DrawText("1-9 select   <-/-> change   R = reset",
            PX, hint_y, 10, col(80,90,120,200));

        rl.DrawFPS(1190, 8);
    }
}
