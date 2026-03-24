//! test/build.zig — Zeph demos (raylib)
const std = @import("std");

pub fn build(b: *std.Build) void {
    const target   = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{
        .preferred_optimize_mode = .ReleaseFast,
    });

    // ── Modul dari folder src/ ─────────────────────────────
    const zeph_mod = b.createModule(.{
        .root_source_file = b.path("../src/zeph.zig"),
        .target   = target,
        .optimize = optimize,
    });

    const cloth_mod = b.createModule(.{
        .root_source_file = b.path("../src/cloth.zig"),
        .target   = target,
        .optimize = optimize,
    });
    cloth_mod.addImport("zeph", zeph_mod);

    const box_mod = b.createModule(.{
        .root_source_file = b.path("../src/box.zig"),
        .target   = target,
        .optimize = optimize,
    });
    box_mod.addImport("zeph", zeph_mod);

    // ── Helper buat executable ─────────────────────────────
    const Demo = struct {
        fn add(
            bb  : *std.Build,
            t   : std.Build.ResolvedTarget,
            o   : std.builtin.OptimizeMode,
            zm  : *std.Build.Module,
            cm  : *std.Build.Module,
            bm  : *std.Build.Module,
            src : []const u8,
            name: []const u8,
            step: []const u8,
            desc: []const u8,
        ) void {
            const mod = bb.createModule(.{
                .root_source_file = bb.path(src),
                .target   = t,
                .optimize = o,
            });
            mod.addImport("zeph",  zm);
            mod.addImport("cloth", cm);
            mod.addImport("box",   bm);

            const exe = bb.addExecutable(.{ .name = name, .root_module = mod });
            exe.linkSystemLibrary("raylib");
            exe.linkSystemLibrary("m");
            exe.linkLibC();

            bb.installArtifact(exe);
            const run = bb.addRunArtifact(exe);
            run.step.dependOn(bb.getInstallStep());
            bb.step(step, desc).dependOn(&run.step);
        }
    };

    // ── Registrasi Demos ───────────────────────────────────
    Demo.add(b, target, optimize, zeph_mod, cloth_mod, box_mod,
        "demo_cloth.zig",     "demo_cloth",     "cloth",   "Demo kain berkibar");

    Demo.add(b, target, optimize, zeph_mod, cloth_mod, box_mod,
        "demo_flag.zig",      "demo_flag",      "flag",    "Demo bendera berkibar");

    Demo.add(b, target, optimize, zeph_mod, cloth_mod, box_mod,
        "demo_bounceball.zig","demo_bounceball", "bounce",  "Demo bola jatuh ke lantai");

    Demo.add(b, target, optimize, zeph_mod, cloth_mod, box_mod,
        "demo_dropbox.zig",   "demo_dropbox",   "dropbox", "Demo box physics drop");
}
