//! demo_bounceball.zig — Bola jatuh ke lantai (ZEPH f32)
//! Murni rigid body — tidak ada cloth sama sekali.
//! SPACE = lempar bola baru | R = reset | Mouse = orbit | Scroll = zoom

const std = @import("std");
const rl  = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
});
const zeph = @import("zeph");

// ─────────────────────────────────────────────────────────────
// Bola — single joint sphere
// ─────────────────────────────────────────────────────────────
const MAX_BALLS  : usize     = 8;
const BALL_RADIUS: zeph.Unit = 0.12;
const BALL_MASS  : zeph.Unit = 2.0;
const DROP_Y     : zeph.Unit = 3.0;
const G_PER_FRAME: zeph.Unit = 9.8 / (60.0 * 60.0);

// ─────────────────────────────────────────────────────────────
// State
// ─────────────────────────────────────────────────────────────
var ball_joints : [MAX_BALLS][1]zeph.Joint  = undefined;
var ball_bodies : [MAX_BALLS]zeph.Body      = undefined;
var g_world     : zeph.World                = .{};
var n_balls     : usize                     = 0;
// Rotasi akumulasi per bola (euler angles dalam radian)
var ball_rot_x  : [MAX_BALLS]f32            = [_]f32{0} ** MAX_BALLS;
var ball_rot_z  : [MAX_BALLS]f32            = [_]f32{0} ** MAX_BALLS;
// Massa per bola — disimpan terpisah untuk collision impulse manual
var ball_mass   : [MAX_BALLS]f32            = [_]f32{1} ** MAX_BALLS;

// ─────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────
inline fn toRl(v: zeph.Vec3) rl.Vector3 { return .{.x=v.x,.y=v.y,.z=v.z}; }
inline fn col(r: u8, g: u8, b: u8, a: u8) rl.Color { return .{.r=r,.g=g,.b=b,.a=a}; }
inline fn clampf(x: f32, lo: f32, hi: f32) f32 { return if(x<lo)lo else if(x>hi)hi else x; }

// Warna bola berdasarkan index
fn ballColor(i: usize) rl.Color {
    const colors = [8]rl.Color{
        col(210, 50,  50,  255), // merah
        col(50,  150, 220, 255), // biru
        col(50,  200, 80,  255), // hijau
        col(230, 180, 40,  255), // kuning
        col(180, 80,  220, 255), // ungu
        col(230, 120, 40,  255), // oranye
        col(40,  200, 200, 255), // cyan
        col(220, 100, 150, 255), // pink
    };
    return colors[i % 8];
}

// ─────────────────────────────────────────────────────────────
// Tambah bola baru
// ─────────────────────────────────────────────────────────────
fn spawnBall(x: f32, z: f32, elasticity: f32) void {
    if (n_balls >= MAX_BALLS) {
        // Geser semua ke kiri, hapus yang pertama
        var i: usize = 0;
        while (i < MAX_BALLS - 1) : (i += 1) {
            ball_joints[i] = ball_joints[i+1];
            ball_bodies[i] = ball_bodies[i+1];
            ball_bodies[i].joints = &ball_joints[i];
        }
        n_balls = MAX_BALLS - 1;
    }

    const idx = n_balls;
    ball_joints[idx][0] = zeph.makeJoint(
        zeph.mkVec3(x, DROP_Y, z), BALL_RADIUS,
    );
    // Velocity awal nol — jatuh lurus ke bawah
    ball_joints[idx][0].velocity = zeph.mkVec3(0, 0, 0);

    // Massa & damping berdasarkan tipe bola
    const this_mass: zeph.Unit = if (elasticity < 0.1) 2000.0
        else if (elasticity < 0.3) 20.0
        else if (elasticity < 0.8) 0.5
        else 0.027;
    const this_damping: f32 = if (elasticity < 0.1) 0.015
        else if (elasticity < 0.3) 0.005
        else if (elasticity < 0.8) 0.0018
        else 0.0008;
    ball_mass[idx] = this_mass;
    zeph.bodyInit(&ball_bodies[idx], &ball_joints[idx], &.{}, this_mass);
    ball_bodies[idx].elasticity      = elasticity;
    ball_bodies[idx].friction        = 0.5;
    ball_bodies[idx].linear_damping  = this_damping;
    // FLAG_NONROTATING: bola single-joint bergerak sebagai rigid sphere
    // tanpa ini collision resolve tidak bekerja dengan benar
    ball_bodies[idx].flags |= zeph.FLAG_ALWAYS_ACTIVE | zeph.FLAG_NONROTATING;

    ball_rot_x[idx] = 0;
    ball_rot_z[idx] = 0;
    n_balls += 1;
    zeph.worldInit(&g_world, ball_bodies[0..n_balls], zeph.envGround);
    g_world.gravity = zeph.mkVec3(0, -G_PER_FRAME, 0);
}

// ─────────────────────────────────────────────────────────────
// Reset
// ─────────────────────────────────────────────────────────────
fn resetScene() void {
    n_balls = 0;
    zeph.worldInit(&g_world, ball_bodies[0..0], zeph.envGround);
    g_world.gravity = zeph.mkVec3(0, -G_PER_FRAME, 0);
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────
pub fn main() void {
    rl.SetConfigFlags(rl.FLAG_MSAA_4X_HINT | rl.FLAG_WINDOW_RESIZABLE);
    rl.InitWindow(1280, 720,
        "Ball Drop — ZEPH f32  |  SPACE=lempar  R=reset  1-3=elasticity  Mouse=orbit");
    defer rl.CloseWindow();
    rl.SetTargetFPS(60);

    resetScene();
    // Langsung spawn satu bola
    spawnBall(0, 0, 0.7);

    var cam = rl.Camera3D{
        .position   = .{.x=0, .y=2.0, .z=4.0},
        .target     = .{.x=0, .y=0.5, .z=0  },
        .up         = .{.x=0, .y=1,   .z=0  },
        .fovy       = 50.0,
        .projection = rl.CAMERA_PERSPECTIVE,
    };
    var yaw   : f32 = 0.0;
    var pitch : f32 = 0.38;
    var cdist : f32 = 4.0;
    var prev_mouse = rl.GetMousePosition();

    // Elasticity yang dipilih untuk bola berikutnya
    var next_elast: f32 = 0.7;

    while (rl.WindowShouldClose() == false) {

        // ── Input ──────────────────────────────────────────────
        if (rl.IsKeyPressed(rl.KEY_R)) {
            resetScene();
            spawnBall(0, 0, next_elast);
        }
        if (rl.IsKeyPressed(rl.KEY_ONE))   next_elast = 0.05; // bola beton
        if (rl.IsKeyPressed(rl.KEY_TWO))   next_elast = 0.2;  // bola besi
        if (rl.IsKeyPressed(rl.KEY_THREE)) next_elast = 0.6;  // bola karet
        if (rl.IsKeyPressed(rl.KEY_FOUR))  next_elast = 0.95; // bola pingpong

        if (rl.IsKeyPressed(rl.KEY_SPACE)) {
            // Spawn di posisi sedikit acak
            const t = @as(f32, @floatFromInt(n_balls));
            const ox = @sin(t * 1.3) * 0.4;
            const oz = @cos(t * 0.9) * 0.4;
            spawnBall(ox, oz, next_elast);
        }

        // ── Physics ────────────────────────────────────────────
        zeph.worldStep(&g_world);

        // ── Ball-ball collision (manual) ────────────────────────
        // zeph hanya handle bola vs lantai. Collision antar bola harus
        // kita resolve sendiri: deteksi overlap → pisahkan → impulse.
        const MIN_DIST = BALL_RADIUS * 2.0;
        const MIN_DIST_SQ = MIN_DIST * MIN_DIST;
        for (0..n_balls) |i| {
            for (i+1..n_balls) |j| {
                const pi = &ball_bodies[i].joints[0];
                const pj = &ball_bodies[j].joints[0];
                const dx = pi.position.x - pj.position.x;
                const dy = pi.position.y - pj.position.y;
                const dz = pi.position.z - pj.position.z;
                const dist_sq = dx*dx + dy*dy + dz*dz;
                if (dist_sq >= MIN_DIST_SQ or dist_sq < 0.000001) continue;

                const dist = @sqrt(dist_sq);
                const nx = dx / dist;
                const ny = dy / dist;
                const nz = dz / dist;

                // Pisahkan posisi agar tidak overlap (weighted by mass)
                const mi = ball_mass[i];
                const mj = ball_mass[j];
                const total_m = mi + mj;
                const overlap = (MIN_DIST - dist);
                const fi = mj / total_m; // bola berat bergerak lebih sedikit
                const fj = mi / total_m;
                pi.position.x += nx * overlap * fi;
                pi.position.y += ny * overlap * fi;
                pi.position.z += nz * overlap * fi;
                pj.position.x -= nx * overlap * fj;
                pj.position.y -= ny * overlap * fj;
                pj.position.z -= nz * overlap * fj;

                // Velocity impulse — elastic collision dengan massa berbeda
                const rvx = pi.velocity.x - pj.velocity.x;
                const rvy = pi.velocity.y - pj.velocity.y;
                const rvz = pi.velocity.z - pj.velocity.z;
                const vn = rvx*nx + rvy*ny + rvz*nz;
                if (vn >= 0) continue; // sudah menjauh, skip

                const e = (ball_bodies[i].elasticity + ball_bodies[j].elasticity) * 0.5;
                const imp = -(1.0 + e) * vn / (1.0/mi + 1.0/mj);
                pi.velocity.x += imp/mi * nx;
                pi.velocity.y += imp/mi * ny;
                pi.velocity.z += imp/mi * nz;
                pj.velocity.x -= imp/mj * nx;
                pj.velocity.y -= imp/mj * ny;
                pj.velocity.z -= imp/mj * nz;
            }
        }

        // Update rotasi visual dari velocity bola
        for (0..n_balls) |i| {
            const v = ball_bodies[i].joints[0].velocity;
            // Rolling: v.x → rotasi sumbu Z, v.z → rotasi sumbu X
            ball_rot_z[i] -= v.x / BALL_RADIUS;
            ball_rot_x[i] += v.z / BALL_RADIUS;
        }

        // ── Camera orbit ───────────────────────────────────────
        const cur = rl.GetMousePosition();
        if (rl.IsMouseButtonDown(rl.MOUSE_BUTTON_LEFT)) {
            yaw   -= (cur.x - prev_mouse.x) * 0.005;
            pitch += (cur.y - prev_mouse.y) * 0.005;
            pitch  = clampf(pitch, 0.05, 1.3);
        }
        prev_mouse = cur;
        cdist -= rl.GetMouseWheelMove() * 0.3;
        cdist  = clampf(cdist, 1.5, 10.0);
        cam.position.x = @cos(pitch) * @sin(yaw) * cdist;
        cam.position.y = @sin(pitch) * cdist;
        cam.position.z = @cos(pitch) * @cos(yaw) * cdist;

        // ── Render ─────────────────────────────────────────────
        rl.BeginDrawing();
        defer rl.EndDrawing();
        rl.ClearBackground(col(10, 12, 22, 255));

        rl.BeginMode3D(cam);

            // Lantai — y=0 sesuai physics ground zeph.envGround
            rl.DrawPlane(.{.x=0,.y=0,.z=0}, .{.x=8,.y=8}, col(35,38,52,255));
            rl.DrawGrid(12, 0.25);

            // Bola
            for (0..n_balls) |i| {
                const bp  = toRl(ball_bodies[i].joints[0].position);
                const bc  = ballColor(i);
                const bw  = col(
                    @min(255, @as(u16,bc.r)+40),
                    @min(255, @as(u16,bc.g)+40),
                    @min(255, @as(u16,bc.b)+40),
                    80,
                );
                rl.DrawSphere(bp, BALL_RADIUS, bc);
                rl.DrawSphereWires(bp, BALL_RADIUS, 8, 8, bw);

                // ── Garis rotasi — ikut berputar sesuai velocity ──────
                const rx = ball_rot_x[i];
                const rz = ball_rot_z[i];
                const R  = BALL_RADIUS * 1.01; // sedikit di luar bola

                // Garis khatulistiwa — lingkaran yang berputar di sumbu Z
                // Titik di khatulistiwa: (R*cos(a+rz), R*sin(a+rz)*cos(rx), R*sin(a+rz)*sin(rx))
                var prev_eq = rl.Vector3{
                    .x = bp.x + R * @cos(rz),
                    .y = bp.y + R * @sin(rz) * @cos(rx),
                    .z = bp.z + R * @sin(rz) * @sin(rx),
                };
                const N_SEG = 12;
                for (1..N_SEG+1) |si| {
                    const a = @as(f32,@floatFromInt(si)) / @as(f32,N_SEG) * std.math.pi * 2.0;
                    const next_eq = rl.Vector3{
                        .x = bp.x + R * @cos(a + rz),
                        .y = bp.y + R * @sin(a + rz) * @cos(rx),
                        .z = bp.z + R * @sin(a + rz) * @sin(rx),
                    };
                    rl.DrawLine3D(prev_eq, next_eq, col(255,255,255,180));
                    prev_eq = next_eq;
                }

                // Garis meridian — tegak lurus khatulistiwa
                var prev_mer = rl.Vector3{
                    .x = bp.x + R * @sin(rx) * @cos(rz),
                    .y = bp.y + R * @cos(rx),
                    .z = bp.z - R * @sin(rz),
                };
                for (1..N_SEG+1) |si| {
                    const a = @as(f32,@floatFromInt(si)) / @as(f32,N_SEG) * std.math.pi * 2.0;
                    const next_mer = rl.Vector3{
                        .x = bp.x + R * @sin(a + rx) * @cos(rz),
                        .y = bp.y + R * @cos(a + rx),
                        .z = bp.z - R * @sin(rz),
                    };
                    rl.DrawLine3D(prev_mer, next_mer, col(255,255,255,180));
                    prev_mer = next_mer;
                }

                // Titik kutub (dot kecil menunjuk ke atas saat tidak rolling)
                const pole = rl.Vector3{
                    .x = bp.x + R * @sin(rx) * @cos(rz),
                    .y = bp.y + R * @cos(rx) * @cos(rz),
                    .z = bp.z + R * @sin(rz),
                };
                rl.DrawSphere(pole, 0.012, col(255,255,255,220));

                // Bayangan di lantai — sedikit di atas y=0 agar tidak z-fighting
                const sh = rl.Vector3{.x=bp.x, .y=0.002, .z=bp.z};
                const shadow_r = BALL_RADIUS * 0.7 *
                    clampf(1.0 - bp.y * 0.3, 0.1, 1.0);
                rl.DrawCircle3D(sh, shadow_r,
                    .{.x=1,.y=0,.z=0}, 90.0, col(0,0,0,120));
            }

        rl.EndMode3D();

        // ── HUD ────────────────────────────────────────────────
        rl.DrawRectangle(8, 8, 280, 110, col(5,8,18,210));
        rl.DrawRectangleLines(8, 8, 280, 110, col(40,60,100,180));
        rl.DrawText("BALL DROP (f32)", 14, 13, 16, col(100,200,255,255));

        const elast_name: [*:0]const u8 = if (next_elast < 0.1) "Bola Beton 2 ton"
            else if (next_elast < 0.4) "Bola Besi"
            else if (next_elast < 0.8) "Bola Karet"
            else "Bola Pingpong";
        rl.DrawText(rl.TextFormat("Elasticity: %.2f  (%s)",
            @as(f64,next_elast), elast_name),
            14, 34, 13, col(180,220,180,230));
        rl.DrawText("1=beton  2=besi  3=karet  4=pingpong",
            14, 52, 12, col(120,130,150,200));
        rl.DrawText("SPACE = lempar bola baru",
            14, 68, 12, col(120,130,150,200));
        rl.DrawText("R = reset  Mouse = orbit  Scroll = zoom",
            14, 84, 12, col(120,130,150,200));
        rl.DrawText(rl.TextFormat("Bola: %d/%d", n_balls, MAX_BALLS),
            14, 100, 12, col(150,180,120,200));

        rl.DrawFPS(1190, 8);
    }
}
