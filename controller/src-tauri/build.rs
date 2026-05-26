// Tauri's own build script does the icon / capability / metadata
// codegen. We chain it with two cargo:rustc-env exports that bake
// the canonical build version + build timestamp into the binary so
// the runtime `get_build_info` command can hand them back to the UI.
//
// Why bake it instead of reading it at runtime: the running AppImage
// has no checkout under it. The git directory only exists at build
// time. Capturing it now into env-baked constants is the cheapest
// way to surface "which build is this?" in the About + Software
// Update screens.
//
// SAINT_BUILD_VERSION is the canonical, human-facing string. Format
// matches the AppImage filename:
//   `<cargo-version>-<channel>.<short-sha>`
// e.g. `0.5.0-local.d85dad7`. The dist build script (build-bundle.sh)
// computes that VERSION the same way and exports it before invoking
// `npm run tauri:build`, so the embedded string is exactly the same
// one used to name the .AppImage. When no override is present (raw
// `cargo build`, `tauri dev`), build.rs falls back to constructing
// the local-channel form itself.

use std::process::Command;

fn git_short_hash() -> String {
    Command::new("git")
        .args(["rev-parse", "--short=7", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".to_string())
}

fn git_dirty_suffix() -> &'static str {
    let clean = Command::new("git")
        .args(["diff", "--quiet", "HEAD"])
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    if clean { "" } else { "-dirty" }
}

fn build_timestamp() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
        .to_string()
}

fn resolve_build_version() -> String {
    // 1. Explicit override (dist build script — build-bundle.sh sets
    //    this so the embedded version matches the AppImage filename
    //    byte-for-byte; CI can also set its own `-ci.<sha>` value).
    if let Ok(v) = std::env::var("SAINT_BUILD_VERSION") {
        let trimmed = v.trim();
        if !trimmed.is_empty() {
            return trimmed.to_string();
        }
    }
    // 2. Reconstruct the local-channel form. Mirrors the logic in
    //    controller/appimage/build-bundle.sh so a raw `cargo build`
    //    produces a version string that's *recognizable* even if
    //    not bit-identical to a dist artifact.
    let base = env!("CARGO_PKG_VERSION");
    let sha = git_short_hash();
    let dirty = git_dirty_suffix();
    format!("{base}-local.{sha}{dirty}")
}

fn main() {
    // Re-run if HEAD moves or anything in the index changes — those
    // are the two things that can shift the short SHA or the
    // dirty/clean flag between builds.
    println!("cargo:rerun-if-changed=../../.git/HEAD");
    println!("cargo:rerun-if-changed=../../.git/index");
    println!("cargo:rerun-if-env-changed=SAINT_BUILD_VERSION");

    let version = resolve_build_version();
    let timestamp = build_timestamp();

    println!("cargo:rustc-env=SAINT_BUILD_VERSION={version}");
    println!("cargo:rustc-env=SAINT_BUILD_UNIX={timestamp}");

    tauri_build::build()
}
