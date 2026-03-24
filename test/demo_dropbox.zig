//! demo_dropbox.zig — Demo menjatuhkan box ke lantai menggunakan Raylib
const std = @import("std");
const ray = @cImport({
    @cInclude("raylib.h");
});
const zeph = @import("zeph");
const box  = @import("box");

// Warna-warni untuk box yang dijatuhkan
const BOX_COLORS = [_]ray.Color{
    .{ .r = 230, .g =  80, .b =  80, .a = 255 }, // merah
    .{ .r =  80, .g = 180, .b =  80, .a = 255 }, // hijau
    .{ .r =  80, .g = 130, .b = 230, .a = 255 }, // biru
    .{ .r = 230, .g = 180, .b =  50, .a = 255 }, // kuning
    .{ .r = 180, .g =  80, .b = 220, .a = 255 }, // ungu
    .{ .r =  60, .g = 200, .b = 200, .a = 255 }, // cyan
    .{ .r = 230, .g = 130, .b =  50, .a = 255 }, // oranye
    .{ .r = 220, .g =  80, .b = 160, .a = 255 }, // pink
};

pub fn main() !void {
    const screen_width  = 960;
    const screen_height = 720;

    ray.InitWindow(screen_width, screen_height, "ZEPH Engine - Drop Box Demo");
    defer ray.CloseWindow();
    ray.SetTargetFPS(60);

    // ── Physics world ────────────────────────────────────────────
    var world = box.BoxWorld{};
    box.boxWorldInit(&world);
    world.dt = 1.0 / 60.0;
    const floor_idx = box.boxAddFloor(&world, 0, .{ .x = 10, .y = 0.5, .z = 10 });
    // Lantai es: friction sangat rendah
    world.bodies[floor_idx].friction    = 0.03;
    world.bodies[floor_idx].restitution = 0.15;

    // ── Cache mesh & material — dibuat SEKALI, dipakai ulang ─────
    // Mesh 1x1x1 → di-scale via extent saat draw
    const cube_mesh = ray.GenMeshCube(1.0, 1.0, 1.0);

    // Buat material per warna (8 warna)
    var materials: [BOX_COLORS.len]ray.Material = undefined;
    for (BOX_COLORS, 0..) |col, ci| {
        materials[ci] = ray.LoadMaterialDefault();
        // Set warna diffuse via maps[0].color
        materials[ci].maps[0].color = col;
    }
    // Material abu-abu untuk lantai
    var floor_mat = ray.LoadMaterialDefault();
    floor_mat.maps[0].color = .{ .r = 160, .g = 160, .b = 160, .a = 255 };

    // ── Kamera ───────────────────────────────────────────────────
    // Kita kelola sendiri supaya bisa klik-kanan drag untuk rotasi
    var cam_yaw:   f32 = 45.0;   // derajat
    var cam_pitch: f32 = 35.0;   // derajat
    var cam_dist:  f32 = 22.0;   // jarak dari target
    const cam_target = ray.Vector3{ .x = 0, .y = 2, .z = 0 };

    var color_counter: usize = 0;
    var prng  = std.Random.DefaultPrng.init(0);
    const rng = prng.random();

    while (!ray.WindowShouldClose()) {

        // ── Kamera: drag klik kanan ──────────────────────────────
        if (ray.IsMouseButtonDown(ray.MOUSE_BUTTON_RIGHT)) {
            const delta = ray.GetMouseDelta();
            cam_yaw   -= delta.x * 0.4;
            cam_pitch  -= delta.y * 0.4;
            // Batasi pitch agar tidak terbalik
            if (cam_pitch <  5.0) cam_pitch =  5.0;
            if (cam_pitch > 89.0) cam_pitch = 89.0;
        }
        // Scroll wheel zoom
        const wheel = ray.GetMouseWheelMove();
        cam_dist -= wheel * 1.5;
        if (cam_dist <  3.0) cam_dist =  3.0;
        if (cam_dist > 60.0) cam_dist = 60.0;

        // Hitung posisi kamera dari yaw/pitch/dist (spherical coords)
        const yaw_r   = cam_yaw   * std.math.pi / 180.0;
        const pitch_r = cam_pitch * std.math.pi / 180.0;
        const cam_pos = ray.Vector3{
            .x = cam_target.x + cam_dist * @cos(pitch_r) * @sin(yaw_r),
            .y = cam_target.y + cam_dist * @sin(pitch_r),
            .z = cam_target.z + cam_dist * @cos(pitch_r) * @cos(yaw_r),
        };
        const camera = ray.Camera3D{
            .position   = cam_pos,
            .target     = cam_target,
            .up         = .{ .x = 0, .y = 1, .z = 0 },
            .fovy       = 45.0,
            .projection = ray.CAMERA_PERSPECTIVE,
        };

        // ── Input: spasi → drop box ──────────────────────────────
        if (ray.IsKeyPressed(ray.KEY_SPACE)) {
            if (world.n_bodies < box.MAX_BODIES) {
                const rx = rng.float(f32) * 4.0 - 2.0;
                const rz = rng.float(f32) * 4.0 - 2.0;
                const ra = rng.float(f32) * 6.28;
                const new_idx = box.boxBodyAdd(
                    &world,
                    .{ .x = rx, .y = 12.0, .z = rz },
                    .{ .x = 0.5, .y = 0.5, .z = 0.5 },
                    1.0,
                    .{ .x = rng.float(f32), .y = 1.0, .z = rng.float(f32) },
                    ra,
                );
                color_counter += 1;
                // Box licin di atas es: friction rendah, damping rendah
                world.bodies[new_idx].friction    = 0.05;
                world.bodies[new_idx].restitution = 0.2;
                world.bodies[new_idx].lin_damping  = 0.01;
                world.bodies[new_idx].ang_damping  = 0.02;
            }
        }

        // ── Physics step ─────────────────────────────────────────
        box.boxWorldStep(&world);

        // ── Draw ─────────────────────────────────────────────────
        ray.BeginDrawing();
        defer ray.EndDrawing();

        ray.ClearBackground(.{ .r = 30, .g = 30, .b = 35, .a = 255 });
        ray.BeginMode3D(camera);
        ray.DrawGrid(24, 1.0);

        for (0..world.n_bodies) |i| {
            const b   = &world.bodies[i];
            const pos = b.position;
            const ext = b.extent;

            // Quaternion → Matrix transform (column-major, Raylib convention)
            const qx = b.quaternion.x;
            const qy = b.quaternion.y;
            const qz = b.quaternion.z;
            const qw = b.quaternion.w;
            // Scale + rotasi + translasi dalam satu matrix
            const sx = ext.x * 2.0;
            const sy = ext.y * 2.0;
            const sz = ext.z * 2.0;
            const mat = ray.Matrix{
                .m0  = (1.0 - 2.0*(qy*qy + qz*qz)) * sx,
                .m1  = (2.0*(qx*qy + qz*qw))        * sx,
                .m2  = (2.0*(qx*qz - qy*qw))        * sx,
                .m3  = 0.0,
                .m4  = (2.0*(qx*qy - qz*qw))        * sy,
                .m5  = (1.0 - 2.0*(qx*qx + qz*qz)) * sy,
                .m6  = (2.0*(qy*qz + qx*qw))        * sy,
                .m7  = 0.0,
                .m8  = (2.0*(qx*qz + qy*qw))        * sz,
                .m9  = (2.0*(qy*qz - qx*qw))        * sz,
                .m10 = (1.0 - 2.0*(qx*qx + qy*qy)) * sz,
                .m11 = 0.0,
                .m12 = pos.x,
                .m13 = pos.y,
                .m14 = pos.z,
                .m15 = 1.0,
            };

            if (b.is_static) {
                ray.DrawMesh(cube_mesh, floor_mat, mat);
            } else {
                // Warna berdasarkan index body (tetap konsisten per body)
                const ci = i % BOX_COLORS.len;
                ray.DrawMesh(cube_mesh, materials[ci], mat);
            }
        }

        ray.EndMode3D();

        // HUD
        const n_dynamic: usize = if (world.n_bodies > 0) world.n_bodies - 1 else 0;
        ray.DrawText("SPACE: drop box", 10, 10, 20, ray.RAYWHITE);
        ray.DrawText("Right Click + Drag: rotate camera", 10, 35, 20, ray.RAYWHITE);
        ray.DrawText("Scroll: zoom", 10, 60, 20, ray.RAYWHITE);
        var buf: [64]u8 = undefined;
        const txt = std.fmt.bufPrintZ(&buf, "Box: {d} / {d}", .{ n_dynamic, box.MAX_BODIES - 1 }) catch "?";
        ray.DrawText(txt.ptr, 10, 90, 20, ray.YELLOW);
        ray.DrawFPS(screen_width - 90, 10);
    }
}
