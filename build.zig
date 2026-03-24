//! build.zig — Zeph Engine root (static libraries)
const std = @import("std");

pub fn build(b: *std.Build) void {
    const target   = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{
        .preferred_optimize_mode = .ReleaseFast,
    });

    // ── zeph (core) ────────────────────────────────────────
    const zeph_mod = b.createModule(.{
        .root_source_file = b.path("src/zeph.zig"),
        .target   = target,
        .optimize = optimize,
    });

    const zeph_lib = b.addLibrary(.{
        .name        = "zeph",
        .root_module = zeph_mod,
    });
    b.installArtifact(zeph_lib);

    // ── cloth (depends on zeph) ────────────────────────────
    const cloth_mod = b.createModule(.{
        .root_source_file = b.path("src/cloth.zig"),
        .target   = target,
        .optimize = optimize,
    });
    cloth_mod.addImport("zeph", zeph_mod);

    const cloth_lib = b.addLibrary(.{
        .name        = "cloth",
        .root_module = cloth_mod,
    });
    b.installArtifact(cloth_lib);

    // ── box (depends on zeph) ──────────────────────────────
    const box_mod = b.createModule(.{
        .root_source_file = b.path("src/box.zig"),
        .target   = target,
        .optimize = optimize,
    });
    box_mod.addImport("zeph", zeph_mod);

    const box_lib = b.addLibrary(.{
        .name        = "box",
        .root_module = box_mod,
    });
    b.installArtifact(box_lib);
}
