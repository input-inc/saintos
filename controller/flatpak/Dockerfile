# Builder image for the SAINT Controller .flatpak (linux/amd64).
#
# Produces a flatpak bundle installable on the Steam Deck. Designed to
# be driven by controller/flatpak/build-docker.sh, which bind-mounts:
#
#   /work             — the repo source
#   /var/lib/flatpak  — host-cached flatpak install dir (GNOME runtime
#                       + SDK extensions; ~1 GB once populated)
#   /build/state      — flatpak-builder --state-dir (build metadata)
#   /build/persistent — cargo registry + target/ + node_modules + npm
#                       cache; mirrors the on-Deck build.sh layout so
#                       both paths share the same incremental story
#
# Runtime requirements (set by build-docker.sh):
#   - --platform=linux/amd64 (Steam Deck is x86_64; on Apple Silicon
#     this routes through Rosetta if Docker Desktop has it enabled)
#   - --privileged           (flatpak-builder uses bubblewrap, which
#                             needs user-namespace creation)

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        flatpak \
        flatpak-builder \
        ostree \
        ca-certificates \
        curl \
        git \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Flathub remote registration lands in /var/lib/flatpak. At runtime the
# caller bind-mounts that path from a host cache, so this image-build
# registration is best-effort; build-bundle.sh re-registers idempotently
# in case the bind-mounted cache is fresh.
RUN flatpak --system remote-add --if-not-exists \
        flathub https://dl.flathub.org/repo/flathub.flatpakrepo \
    || true

WORKDIR /work

# build-bundle.sh is the same script CI runs natively on its Linux
# amd64 runners — keeps the two paths in lockstep.
CMD ["/work/controller/flatpak/build-bundle.sh"]
