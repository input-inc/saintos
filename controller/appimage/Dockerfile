# Builder image for the SAINT Controller .AppImage (linux/amd64).
#
# Produces an AppImage installable on the Steam Deck. Designed to be
# driven by controller/appimage/build-docker.sh from a Mac, or
# directly on a GitHub Linux amd64 runner via build-bundle.sh in CI.
#
# Why AppImage instead of flatpak: see controller/docs/APPIMAGE_MIGRATION.md
# for the long version. Short version: flatpak-builder needs bwrap,
# bwrap needs to install a seccomp filter, and on Apple Silicon
# Docker Desktop with Rosetta enabled that filter conflicts with the
# one Rosetta installs for x86_64 syscall translation. AppImage uses
# linuxdeploy, which is just a normal userspace tool — no namespaces,
# no seccomp, no kernel-level sandboxing. Builds cleanly under
# Rosetta with no special docker flags.
#
# Runtime requirements (set by build-docker.sh):
#   - --platform=linux/amd64  (Steam Deck is x86_64)
#   - bind mount /work        (repo source)
#   - bind mount /build       (persistent cache: cargo, target, npm, node_modules)
# No --privileged, no --security-opt — none of it is needed.

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Build deps:
#   nodejs/npm         — 20.x on noble; required by tauri build's
#                        beforeBuildCommand ("npm run build")
#   libwebkit2gtk-4.1, libsoup-3, libgtk-3 — link-time deps for Tauri 2.
#                        The .so files get copied into the AppImage's
#                        bundled lib dir; the Deck doesn't need them
#                        preinstalled (webkit2gtk-4.1 specifically is
#                        NOT on stock SteamOS, which is the whole
#                        reason we're bundling).
#   librsvg2           — Tauri uses it for icon processing
#   fuse + libfuse2    — required at *build* time so appimagetool can
#                        assemble the squashfs runtime stub. The Deck
#                        has its own FUSE for running the AppImage.
#
# Notably absent: rustc/cargo. Ubuntu 24.04 ships rustc 1.75 but our
# Cargo.lock is format v4, which requires Cargo 1.78+ to parse. We
# install a current stable toolchain via rustup below.
RUN apt-get update && apt-get install -y --no-install-recommends \
        nodejs npm \
        libwebkit2gtk-4.1-dev \
        libsoup-3.0-dev \
        libgtk-3-dev \
        librsvg2-dev \
        libssl-dev \
        libudev-dev \
        pkg-config build-essential file \
        fuse libfuse2 \
        ca-certificates curl git \
    && rm -rf /var/lib/apt/lists/*

# Rust toolchain via rustup. Stable channel is fine for now — pin
# with --default-toolchain 1.x.y here (and add controller/src-tauri/
# rust-toolchain.toml) if reproducibility ever bites us.
ENV PATH=/root/.cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
        sh -s -- -y --default-toolchain stable --profile minimal --no-modify-path

# Pre-install AppImage bundling tools with the AppImage ABI signature
# zeroed out. Background: linuxdeploy + linuxdeploy-plugin-appimage +
# appimagetool are themselves distributed as AppImages. Each has the
# AppImage "magic" at ELF header offset 8 ('A','I',\x02) which sets
# e_ident[EI_OSABI]/EI_ABIVERSION to a non-zero value. On Apple
# Silicon Docker Desktop with Rosetta translation, that non-standard
# byte makes execve() return ENOEXEC ("cannot execute binary file:
# Exec format error") even though the binary is a valid x86_64 ELF.
#
# Zeroing those three bytes makes the kernel accept the binary as a
# normal ELF. The AppImage's *own* runtime doesn't read the magic
# bytes during execution (they're a "this is an AppImage" detection
# signature for external tools), so the tools still work normally.
# APPIMAGE_EXTRACT_AND_RUN=1 (set in build-bundle.sh) makes them
# extract themselves to /tmp instead of trying to mount via FUSE.
#
# Tauri's bundler downloads its own copies of these tools and runs
# them directly — it doesn't check PATH first — so our patched copies
# here are used by build-bundle.sh's fallback step, NOT by tauri-cli.
RUN mkdir -p /opt/appimage-tools && \
    for url in \
        https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage \
        https://github.com/linuxdeploy/linuxdeploy-plugin-appimage/releases/download/continuous/linuxdeploy-plugin-appimage-x86_64.AppImage \
        https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage \
    ; do \
        name=$(basename "$url"); \
        curl -fsSL -o "/opt/appimage-tools/$name" "$url" && \
        chmod +x "/opt/appimage-tools/$name" && \
        dd if=/dev/zero of="/opt/appimage-tools/$name" bs=1 count=3 seek=8 conv=notrunc status=none; \
    done && \
    ln -s /opt/appimage-tools/linuxdeploy-x86_64.AppImage /usr/local/bin/linuxdeploy && \
    ln -s /opt/appimage-tools/linuxdeploy-plugin-appimage-x86_64.AppImage /usr/local/bin/linuxdeploy-plugin-appimage && \
    ln -s /opt/appimage-tools/appimagetool-x86_64.AppImage /usr/local/bin/appimagetool

# Compile the LD_PRELOAD path-mapping shim that redirects webkit2gtk's
# hardcoded helper-process path (/usr/lib/x86_64-linux-gnu/webkit2gtk-
# 4.1/) to the AppImage-bundled equivalent at runtime. See
# path-shim.c for the full explanation — short version: Ubuntu's
# webkit2gtk has the WEBKIT_EXEC_PATH env-var override compiled out
# (gated behind ENABLE(DEVELOPER_MODE) which distros disable), so the
# only way to make webkit find our bundled helpers is to rewrite the
# syscalls it uses to spawn them.
COPY path-shim.c /tmp/path-shim.c
RUN mkdir -p /opt/saint-shim && \
    gcc -shared -fPIC -O2 -Wall \
        -o /opt/saint-shim/libpath-shim.so /tmp/path-shim.c -ldl && \
    rm /tmp/path-shim.c

WORKDIR /work

# build-bundle.sh is the same script CI runs natively on its Linux
# amd64 runners — keeps the two paths in lockstep.
CMD ["/work/controller/appimage/build-bundle.sh"]
