# Changelog

## v0.2.4 — 2026-11-19

Changes since v0.2.3 (2026-10-07): fixes for current Raspberry Pi OS
(Trixie) and SteamOS installs, plus a few controller and dashboard fixes.
No firmware changes.

### Upgrade notes — read before installing

- **Pick the server tarball for your Pi's OS.** There are now two:
  - `saint-os_0.2.4_arm64_kilted_trixie.tar.zst` for Raspberry Pi OS
    Trixie (Debian 13, the current Pi OS image).
  - `saint-os_0.2.4_arm64_kilted.tar.zst` for Bookworm (Debian 12) and
    Ubuntu 24.04.

  `grep VERSION_CODENAME /etc/os-release` tells you which one you have.
  `install.sh` refuses the wrong one and names the right one. Servers
  already running v0.2.3 on Bookworm update as before: their self-updater
  only sees the Bookworm tarball.
- **Copy the controller AppImage to the Steam Deck by hand this once.** The
  v0.2.3 AppImage can't start on SteamOS (see below), so it can't
  self-update. Use `saint_firmware_controller_*.AppImage` from the release
  or from the server's firmware page. Later updates work over the air again.
- **No reflash needed.** Firmware is unchanged from v0.2.3.

### Highlights

- **Server installs on Raspberry Pi OS Trixie.** v0.2.3 only installed on
  older Bookworm images.
- **The controller starts on SteamOS again.** v0.2.3's AppImage failed with
  `Failed to spawn child process ././/lib/x86_64-linux-gnu/webkit2gtk-4.1/WebKitNetworkProcess`.

### Server install (Raspberry Pi OS Trixie)

- One server tarball per Debian release. The Trixie build uses Trixie's
  libraries (Python 3.13, `libssl3t64`) and its own ROS 2 tree. v0.2.3's
  single Bookworm build carried Debian 12 packages, and its `libssl3`
  conflicted with Trixie's `libssl3t64`.
- `install.sh` checks that the tarball matches the host's release before
  installing anything.
- The offline install now uses only the bundled packages. It used to also
  read the image's stale online package lists and could hang for over
  10 minutes waiting on downloads with no network.
- Packages that are newer on the Pi OS image are kept. v0.2.3 pinned exact
  versions (e.g. `systemd`), which failed on any image newer than the
  bundle. When a package in the bundle requires an exact version that the
  image has a newer one of, only that family of packages is moved to the
  bundled version, and the installer says which ones.
- The installer never removes packages from the image.
- The server's self-update (GitHub, USB stick or browser upload) only
  offers or accepts the tarball built for the host's OS.

### Controller

- **SteamOS:** the AppImage starts again.
  - Tauri's bundler had rewritten WebKit's helper paths to relative
    `././` paths that only work with Tauri's own launcher, which we
    replace. AppImages built on CI from v0.2.3 on had them; local builds
    didn't. The original paths are restored, and the build fails if any
    are left.
  - The AppImage no longer bundles `libwayland`. Its older Ubuntu copy
    replaced the one SteamOS's graphics drivers need, so once the
    helpers could start, the webview crashed with `Could not create
    default EGL display: EGL_BAD_PARAMETER`.
- The dashboard shows the Steam Deck's own battery (or any Linux device's)
  next to the robot's packs.
- The connection status no longer stays red over a working link after a
  reconnect.
- A Wi-Fi drop is noticed within seconds instead of when TCP gives up,
  which could take minutes.

### Dashboard

- Sounds can be edited after they're created.
- Renaming a sound no longer resets to the old name while you type.
- Animation editor: selecting another keyframe while a number field is
  focused no longer writes the old keyframe's value onto the new one.

### Build / CI

- CI builds and releases both server tarballs (Bookworm and Trixie).
- `build-local-dist.sh --debian-release trixie` builds the Trixie tarball
  locally.
- Runtime dependencies are split into a shared list and per-release lists
  (`packaging/runtime-deps*.txt`).
- New SteamOS smoke test. Every CI build launches the controller AppImage
  on SteamOS's own libraries, taken from Valve's current recovery image,
  and checks that the webview and frontend come up. It caught both
  controller bugs above. See `docs/BUILD.md`.

### Known issues

- The SteamOS smoke test uses the current recovery image (SteamOS 3.8.14)
  under a virtual display. It doesn't cover gamescope, Steam's launcher or
  the GPU.

## v0.2.3 — 2026-10-07

Changes since v0.2.2 (2026-07-21): 17 commits across firmware, server,
dashboard and controller.

### Upgrade notes — read before installing

- **Reflash every RP2040 and Teensy node.** The flash layout went from v12
  to v15 in this release. Config saved by older firmware is discarded on
  first boot: each node comes up unadopted, and the server pushes its
  config again automatically. Expect one re-sync per node, including UART
  pin pairs (RoboClaw, Kangaroo).
- **Update the server and the firmware together.** Kangaroo channels per unit went from
  6 to 10, which moves the internal channel map for Kangaroo units above
  the first one. A mismatched server and firmware decode those units' readings against
  the wrong unit.
- **Groups become playlists.** On first start the server converts every
  animation, pose and sound group into a playlist, once. The controller
  needs this server version: it asks for playlists, not groups.
- **Check Feather RP2040 pin assignments.** The board definition now
  matches Adafruit's pinout:
  - GPIO13 (onboard LED) and GPIO24 (W5500 alternate chip select) are
    reserved. Move anything assigned to either: the firmware takes those
    pins back at boot, so a peripheral there stops working until the next
    Sync.
  - GPIO5 is not on a header pad. Anything assigned to it can't have been
    wired, so move it.
  - Pins that showed as "D5"/"D6" were really GPIO5/GPIO6; D4/D5/D6 are
    GPIO6/7/8. A peripheral on GPIO6 is physically on the D4 pad.
  - D11 (GPIO11) is now free.
- **Raspberry Pi nodes** gain optional dependencies for NeoPixel strips:
  `rpi_ws281x` (Pi 3/4) and `spidev` (Pi 5, which needs SPI enabled and
  data on GPIO 10).
- **Server:** install `ffprobe` so non-WAV sound lengths can be measured.
- **Build scripts:** `build-local-dist.sh` now stops when a firmware build
  fails instead of shipping the previously staged firmware. Pass
  `--skip-firmware-build` or `--fetch-firmware` to ship staged or CI
  firmware deliberately.

### Highlights

- **Joystick run-on fixed.** Motors used to keep moving for seconds after the stick was
  released. It came down from about 6.8 s to about 0.3 s on the robot.
  Fixes on the controller, server and firmware, plus a RoboClaw dead-man
  timer that stops a motor if setpoints stop arriving.
- **Playlists replace groups** for animations, poses and sounds. An item can
  be in several playlists, on the dashboard and the controller.
- **Animations can fire sounds and other animations** as board items on
  the timeline, drawn as bars the length of the clip.
- **Kangaroo teach tune and limit switches,** with interlocks that act on the
  node itself, so they keep working if the network link drops.
- **Three-file robot model** (URDF + SRDF + rig file), with rig controls,
  pose tracks and self-collision detection in the animation editor.
- **Pimoroni Servo 2040** support (preliminary) and **NeoPixel strips on any
  pin** on every node type.

### Firmware

*All firmware items need a reflash or OTA update.*

- Flash layout v15. The saved pin table grew from 16 to 48 slots: a Teensy
  with a 24-channel Maestro used to write past the table and boot with
  corrupt config.
- Peripheral config restored from flash is no longer overwritten with
  defaults at boot. A Maestro used to come back from every reboot with its
  default range and home at 0 until someone pressed Sync.
- RoboClaw:
  - Dead-man timer: a running motor that gets no new setpoint within 1.25 s
    is stopped and a warning is logged.
  - Duty writes no longer wait for the reply byte, which could cost up to
    50 ms per message.
  - A unit that keeps failing status polls is polled once every 10 s instead
    of every cycle.
  - A stored config that fails sanity checks is ignored instead of putting
    the UART at a bogus baud rate.
- RP2040 drains every queued control message each loop pass (up to 16 per
  pass, 20 ms), so stale setpoints can't pile up, and logs a loop timing
  profile every 5 s.
- Teensy no longer drops off the network when its messages get too large.
  `/state` is capped to one frame and cut short cleanly, and `/announce`
  lists only the peripherals the node has.
- A Teensy with a USB-attached Maestro no longer resets about 10 s after
  boot. The periodic status poll on that link is off, and the USB transfer
  code no longer has the race that crashed it.
- Nodes store and report a tag for the config they hold, and accept small
  Maestro per-channel patches instead of a full push.
- Maestro:
  - `idle_disengage_ms` can be set once at peripheral level, which keeps
    24-channel configs under the wire limit.
  - Hardware min/max are now the servo's absolute limits, so the extent
    dialer can widen a channel's range.
- Kangaroo (RP2040 and Teensy):
  - Teach tune over packet serial from the dashboard.
  - The firmware enforces a keep-alive, a jog dead-man and a cap on jog
    power during the tune.
  - Soft limits inside the taught travel.
  - Telemetry is polled every 100 ms instead of every loop pass.
- New `switch_input` peripheral reads limit switches, end-stops and e-stop
  buttons, on a digital pin or on an analog pin with a threshold.
  - Debounce and latching.
  - A tripped switch can e-stop the node or stop up to 4 named peripherals
    locally.
  - Each target can block only the direction that drives into the switch.
  - Kangaroo and RoboClaw accept a latched per-peripheral `estop`.
- New `peripheral_command` messages carry named commands with arguments
  to one peripheral (for example the Kangaroo tune). The parser is shared
  between RP2040 and Teensy.
- NeoPixel strips on any data pin on RP2040 (PIO, up to 4 × 300 pixels)
  and Raspberry Pi nodes, matching the existing Teensy driver.
  - Strips start dark and go dark on e-stop.
  - On RP2040, a strip whose id contains "neopixel" is no longer routed to
    the onboard status LED.
- Pimoroni Servo 2040 (preliminary): a stand-alone board image you flash
  once by hand. It drives 18 servos and 6 LEDs and relaxes every servo if
  the host heartbeat stops. Host drivers on Teensy, RP2040 and Pi.
- Pi node soundboard can boost quiet clips above 100% volume, up to 200%.
- Feather RP2040 pin constants corrected to Adafruit's pinout. The firmware
  only uses D13, which didn't change, so this alone needs no reflash.

### Server

- A single channel arbiter now decides whether a channel write goes out,
  replacing two caches that couldn't see each other's writes.
  - Re-clicking a pose board always re-applies it.
  - A slider can move a channel back to a value a pose had overwritten.
  - Background streams no longer snap a hand-moved servo back.
  - Its memory of what each node holds is cleared on reconnect, sync,
    e-stop and peripheral re-enumeration.
- Controller stick inputs are collected for 20 ms and the routing sheet is
  evaluated once per batch, instead of once per message. This is the main
  run-on fix.
- Held motor values are re-sent periodically so the firmware dead-man can
  tell a held stick from a dead link. Servos are excluded so their idle
  disengage still works.
- Idle keepalive: an adopted node the server hasn't written to in 15 s gets
  a harmless message. This stops the 60-second offline/reconnect cycle on
  idle Teensy nodes.
- Config sync is confirmed by the node: it only shows "synced" when the tag
  it reports matches. Config still only moves when the operator presses
  Sync, and you can revert to the last confirmed config. A push larger
  than the wire limit is refused with an explanation instead of crashing
  the node.
- Config pushes leave out default-valued and server-only settings (Kangaroo,
  Maestro, Servo 2040) to stay under the ~2 KB node limit. Regression tests
  guard the size.
- `pin_state` broadcasts are limited to one per node per 200 ms
  (`websocket.pin_state_interval_ms`), and a slow client no longer stalls
  everyone else.
- Playlists replace groups. The migration runs once, when the `playlists/`
  directory doesn't exist yet.
  - Deleting an item removes it from every playlist.
  - Deleting a playlist never deletes its items.
  - Sound lists still report a `group` field for older clients.
- Animations:
  - Timeline keyframes can fire sounds and other animations. A looping item
    can be given a length.
  - An animation can't trigger itself, and nesting is capped at 4 levels.
  - Pose tracks blend in layers with per-joint overrides.
  - Stopping returns every joint the animation touched to neutral.
  - Stop, or reaching the end, now also stops the sounds and nested
    animations it started. A clip that already finished is left alone.
  - Triggers at t=0 now fire, including on loop and resume.
  - Play can run the editor's unsaved draft without saving it.
- Sound lengths are measured on save (WAV header, then ffprobe, then VLC).
  - Older sounds are measured in the background at startup.
  - New actions: `measure_sound` (on demand) and `reprobe_sound` (after a
    file is replaced).
  - Volume range is now 0–200%.
- Routing inputs can read peripheral channels, so a limit switch or BMS
  value on one node can drive behaviour on another.
- Three-file robot model: URDF, SRDF and rig file, uploadable together as a
  `.zip`.
  - Meshes keep their folder paths, which fixes same-named meshes in
    different folders overwriting each other. Re-upload older models
    affected by that.
  - SRDF poses can be imported.
  - New rig evaluator for sliders, XY pads, blend curves and mimic joints.
- The firmware listing now offers flashable files (`.uf2`, `.hex`, images),
  not just the `.elf`.
- Dashboard widget order is saved in `system_routing.yaml`.
- The systemd service restarts on any exit, and the launch stops when the
  server process dies, so a crashed server no longer shows as "active".
- Feather RP2040 board definition corrected (see Upgrade notes), with a new
  built-in `onboard_led` peripheral.

### Dashboard

- Boards page:
  - Playlist sidebar with drag-to-add and drag-to-reorder.
  - Sound volume is editable from the list.
- Animation editor:
  - Board items on the timeline, drawn as bars the length of the clip.
  - Pose tracks show all their setpoints.
  - Rig controls panel and SRDF pose import.
  - Self-collision badge and timeline bands. The scan runs only while the
    editor is idle and uses cached convex hulls, so detailed CAD models no
    longer freeze the page.
  - Play uses your unsaved edits.
  - Numeric fields let you type decimals without resetting mid-keystroke.
- Peripherals tab:
  - Saving no longer wipes Maestro channel names and extents.
  - Lowering the channel count warns which channels would be lost.
  - Sync shows Syncing… / Unsynced changes / Revert, and a Retry dialog if
    the node never confirms.
- Kangaroo live card, tune wizard, and soft-limit handles.
- Servo 2040 card.
- Limit-switch targets are picked from a checklist instead of typed ids.
- Servo extents editor:
  - Live dial-in panel with a current bar and peak hold.
  - "Move servo to handle" toggle.
  - The current sensor can be on any node.
- Node pages:
  - The Control tab merged into Overview.
  - New header with State, CPU temperature and Last seen.
- Firmware updates:
  - "Update all" shows progress immediately.
  - Download buttons per build.
- Widgets can be reordered and share one header style.
- Slide-out navigation menu on phones and tablets.

### Controller

- Run-on fix: each stick target keeps only its newest value instead of
  sharing a 100-message queue that could fill with stale values and drop
  the stop command.
  - Pending values are cleared on reconnect.
  - Unchanged values are re-sent every 150 ms to feed the motor dead-man.
  - A stop that failed to send is retried on the next pass.
  - Losing focus zeroes every bound stick output.
- Battery savings on the Steam Deck:
  - The IMU is powered only when something needs it.
  - mDNS discovery runs only when needed.
  - Input processing pauses in the background.
  - Live data subscriptions run only while a screen shows them.
- Preset panels:
  - A playlist list replaces the group dropdown.
  - Show-panel bindings store the playlist id, so renaming a playlist
    doesn't break them.
- New "Activate Board Item" binding and a Boards tab in Bindings.
- Steam Deck back buttons (L4/R4/L5/R5) can be bound.
- **Breaking:** the "+ Add Panel" option is gone; the panels are the
  server's three boards. Existing bindings to old static panels still load.
- Frontend console errors and warnings are written to `saint-controller.log`.

### Documentation

- Docs reorganised around installing prebuilt releases: a three-step root
  README, `docs/INSTALL.md` and `docs/BUILD.md`.
- New guides:
  - Kangaroo bring-up
  - Sensor inputs
  - Collision avoidance
  - Robot model
  - Rig schema (`rig-1.1.xsd`)
  - Channel arbitration
  - Config sync
  - Latency reduction
  - Pimoroni Servo 2040 bring-up
- Feather RP2040 pinout in `firmware/rp2040/docs/INSTALL.md` rewritten as a
  table checked against Adafruit's pinout.

### Build / CI

- The Teensy firmware builds on Apple Silicon without Rosetta. The platform
  is pinned to 5.2.0.
- Controller AppImage builds:
  - Fetch Tauri's AppRun themselves.
  - Work on Docker Desktop for Mac.
  - Fail fast on the file-sharing permission problem.
- The dashboard dev mock server simulates Kangaroo tunes, switch inputs and
  firmware listings, and runs the real Python rig code.

### Known issues

- **Kangaroo soft limits switch off when the retract limit is 0.** The
  firmware only applies soft limits when `soft_min`, `soft_max` and
  `soft_center` all arrive. The server leaves out settings that equal
  their defaults, and `soft_min` defaults to 0. With the retract limit at 0
  the firmware gets two of the three values and turns soft limits off,
  with no warning.
- Switch-input configs, Kangaroo jog power and soft limits are not saved to
  flash. After a reboot, limit-switch interlocks and soft limits stay off
  until the server pushes the config.
- The `onboard_led` built-in has no RP2040 firmware handler yet, so its
  dashboard control does nothing on Feather nodes.
