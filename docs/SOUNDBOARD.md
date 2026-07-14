# Soundboard

The Soundboard lets an operator register named audio clips in the server
web interface and trigger them — from the dashboard or from the Steam Deck
controller — to play on a robot node's speaker.

It lives under **Boards → Sounds** in the web UI, alongside Animations and
Poses.

---

## Key concept: sounds are node-scoped

A sound belongs to **one node** — the node that plays it.

- The audio file must already exist on **that node's own storage** (SD
  card / internal disk). Files are **not** copied between nodes or uploaded
  from your browser.
- Playback comes out of **that node's** selected audio output (ALSA
  device), defaulting to the node's default output.
- Only one clip plays per node at a time: triggering a new sound **stops**
  whatever that node was already playing (stop-and-replace).

Because playback is python-vlc → ALSA, sounds can only be assigned to
**Raspberry Pi** nodes (the only node type that runs VLC).

---

## Setup (operator guide)

### 1. Prerequisites on the Pi node

The node that will play sound needs VLC and ALSA tools installed. On
Raspberry Pi OS:

```bash
sudo apt install vlc-bin libvlc-dev python3-vlc alsa-utils
```

- `vlc-bin` / `libvlc-dev` / `python3-vlc` — the playback engine.
- `alsa-utils` — provides `aplay`, used to enumerate output devices.

Plug in / confirm your speaker or amp. If it's a USB DAC, HDMI, or an I²S
HAT, make sure it shows up in `aplay -l` on the node.

> These are already present on a standard SAINT.OS Pi image; you only need
> this step on a hand-built node.

### 2. Put your audio files on the node

Copy the clips you want onto the node's filesystem. Any location works —
the file picker browses the whole filesystem — but a dedicated folder is
tidiest, e.g.:

```bash
mkdir -p /var/lib/saint-os/audio
# copy fanfare.mp3, hello.wav, … into it
```

Supported formats are anything libVLC handles: `.wav`, `.mp3`, `.ogg`,
`.flac`, `.aac`, `.m4a`, `.opus`.

### 3. Adopt the node

The node must be **adopted** in SAINT.OS (Nodes page) and online. Only
adopted Raspberry Pi nodes appear in the sound editor's node picker.

### 4. Add a sound

1. Open **Boards** in the top navigation.
2. In the left sidebar under **Sounds**, click **New Sound**.
3. Fill in the modal:
   - **Icon / Name** — how the entry appears in the list and on the
     controller.
   - **Node** — which Pi will play it.
   - **File** — click **Browse** to open the node's file picker, navigate
     to your clip, and select it. (Tick *Show all files* if a file isn't
     detected as audio.)
   - **Output device** — the node's speaker/output. Leave on *Node default
     output* unless you have multiple outputs.
   - **Volume** — 0–100 %.
   - **Start time** — seconds to skip at the start of the clip.
   - **Loop** — off = play once. On = repeat *forever*, or untick *Loop
     forever* and set a play count.
   - **Group** — optional; groups appear as sub-folders in the Sounds
     sidebar (like pose groups).
4. Click **Create**.

### 5. Organize

- **Reorder** within a group with the ▲/▼ buttons on each row. This order
  is what the Steam Deck controller shows.
- **Rename / regroup** inline by clicking the name or the group field.
- **Delete** with the trash button.

### 6. Trigger a sound

- **From the dashboard:** click ▶ on a sound row to play it, ■ to stop that
  node. Errors (missing file, offline node) surface as a message under the
  list.
- **From the Steam Deck controller:** press **START** to open the Sounds
  panel. It lists every sound on the server in your saved order; select one
  to play it. (The panel shows *Connect to the robot…* / *Loading…* / *No
  sounds saved* until the list arrives.)

---

## How it works (architecture)

Three tiers cooperate; the server is the hub.

```
┌── Web UI / Steam Deck ──┐        ┌── Server (ROS2 + web) ──┐        ┌── Pi node ──┐
│ Boards → Sounds         │  WS    │ SoundStore (JSON)       │  ROS   │ soundboard  │
│ New Sound / ▶ / reorder │ ─────▶ │ websocket_handler       │ ─────▶ │  .py        │
│ Steam Deck START panel  │        │ server_node publishers  │ /command│ VLC → ALSA  │
└─────────────────────────┘        └─────────────────────────┘        └─────────────┘
        ▲  library-sounds / soundboard_fs|devices|result (keyed by request_id)  │
        └───────────────────────────────────────────────────────────────────────┘
```

### Persistence

Each sound is one JSON file at `{config_dir}/sounds/<slug>.json`
(`/etc/saint-os/sounds/` in production). Managed by `SoundStore`
(`server/saint_server/animation/store.py`), mirroring `PoseStore` but with
an explicit `position` field; `list()` returns entries sorted by
`(group, position, name)`.

### WebSocket API (browser + controller both use these)

`management` actions handled in
`server/saint_server/webserver/websocket_handler.py`:

| Action | Purpose |
|---|---|
| `list_sounds` | summaries for the list / controller panel |
| `get_sound` / `save_sound` / `delete_sound` | CRUD |
| `reorder_sounds` `{ordered_ids}` | rewrite `position` |
| `sound_list_nodes` | adopted Pi nodes for the node picker |
| `sound_list_dir` `{node_id, path, request_id}` | browse node filesystem (async) |
| `sound_list_devices` `{node_id, request_id}` | enumerate node ALSA outputs (async) |
| `play_sound` `{id}` | resolve entry → play on its node |
| `stop_sound` `{node_id}` | stop that node's current clip |

### Server ↔ node wire contract

The server publishes a JSON command on `/saint/nodes/<id>/command`
(RELIABLE QoS) via `server_node.send_soundboard_command()`:

```json
{ "action": "soundboard_play",
  "request_id": "sb-…",
  "args": { "path": "/var/lib/saint-os/audio/fanfare.mp3",
            "device": "default", "volume": 0.8,
            "start_time_s": 0.0, "loop": false, "loop_count": 0 } }
```

`action` is one of `soundboard_list_dir`, `soundboard_list_devices`,
`soundboard_play`, `soundboard_stop`.

The node replies on dedicated topics, echoing the `request_id` so the
caller can correlate the response (the same async pattern as BLE scan):

| Reply topic | For |
|---|---|
| `/saint/nodes/<id>/soundboard_fs` | directory listings |
| `/saint/nodes/<id>/soundboard_devices` | ALSA device lists |
| `/saint/nodes/<id>/soundboard_result` | play/stop `{status, message}` |

`server_node` re-broadcasts each reply to WebSocket subscribers as
`soundboard_fs/<node_id>` etc. The web store
(`server/web/src/stores/sounds.js`) subscribes to the topic, fires the
request with a fresh `request_id`, and resolves on the matching frame.

### Node playback

`firmware/raspberrypi/saint_node/soundboard.py`:

- `list_directory(path)` — dirs + files, flags audio by extension.
- `list_audio_devices()` — parses `aplay -l`; always includes `default`.
- `SoundboardPlayer` — one voice per node. Caches a `vlc.Instance` per
  ALSA device (the output device is an Instance-level argument), creates a
  fresh `MediaPlayer` per play, applies `start-time` and `input-repeat`
  (finite = `loop_count − 1`; infinite = a large sentinel) media options,
  sets volume, and stops+releases any prior player first.

### Controller

The Steam Deck's Sounds panel is `source: 'sounds'`
(`controller/src/composables/useBindings.ts`). `useLibrary` fetches via
`invoke('list_sounds')` and listens for the `library-sounds` Tauri event;
selecting an item calls `invoke('play_sound', {id})`. The Rust side
(`src-tauri/src/protocol/`) forwards these to the same server WebSocket
actions listed above.

---

## Sound fields

| Field | Meaning |
|---|---|
| `name`, `icon` | display in list + controller |
| `group` | sidebar grouping (`""` → Ungrouped) |
| `node_id` | node that plays it |
| `file_path` | absolute path on that node |
| `output_device` | ALSA device id (`""`/`default` → node default) |
| `volume` | 0.0–1.0 |
| `start_time` | seek offset (seconds) |
| `loop` | repeat when true |
| `loop_count` | total plays when looping; `0` = infinite |
| `position` | explicit order within its group |

---

## Troubleshooting

- **No nodes in the picker** — the node must be an *adopted, online
  Raspberry Pi*. Teensy/RP2040 nodes can't play audio.
- **Browse/device list spins then errors** — the node didn't respond
  within 15 s: it's offline, or `aplay`/VLC isn't installed.
- **Play does nothing / "VLC refused to play"** — check the file still
  exists at that path on the node, the format is supported, and the chosen
  output device is the one your speaker is on. Test on the node directly
  with `cvlc --play-and-exit <file>`.
- **Wrong speaker** — pick a specific **Output device** instead of *Node
  default*; confirm the target shows in `aplay -l`.
- **Clip won't stop** — Stop is per node; if two dashboards are open, the
  last play wins. Press ■ again.

## Deploying node changes

The Pi-side code (`soundboard.py`, `node.py`) ships through the normal
SAINT.OS dist build + install — **not** by copying files onto the Pi. After
updating node code, rebuild/redeploy the node package so the running node
picks up the new handlers.
