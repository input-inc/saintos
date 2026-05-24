# Binding controller inputs to server routing sheets

This doc covers the end-to-end "make this joystick drive that motor" flow.
It assumes:

- The SAINT.OS server is running and reachable from the Deck (default
  `ws://opensaint.local/api/ws`).
- At least one peripheral node (RP2040 / Teensy / Pi-5) has been adopted
  and has a peripheral installed — e.g. a RoboClaw motor controller, a
  servo, a NeoPixel. If you haven't done that yet, see
  [`saint_os/docs/SERVER_GUIDE.md`](../../saint_os/docs/SERVER_GUIDE.md).
- The controller Flatpak is installed on the Deck (see
  [`../README.md`](../README.md)).

## How a binding actually moves a value

Three pieces conspire to turn an axis push into a motor command:

```
┌────────────────┐        WebSocket             ┌──────────────────────┐
│                │  router.set_input            │                      │
│   Controller   │  {sheet_id, input_id, value} │   SAINT.OS server    │
│   (the Deck)   │ ───────────────────────────► │   routing evaluator  │
│                │                              │                      │
└────────────────┘                              └──────────┬───────────┘
                                                           │
                                                           ▼
                                            ┌──────────────────────────┐
                                            │ Sheet on the server:     │
                                            │  WS-input → operators →  │
                                            │  peripheral channel sink │
                                            └──────────┬───────────────┘
                                                       │ ROS / micro-ROS
                                                       ▼
                                                  ┌──────────┐
                                                  │  Node    │
                                                  │ + actuator│
                                                  └──────────┘
```

The piece you author in the controller UI is the leftmost arrow: which
gamepad input addresses which `(sheet_id, input_id)` on the server. The
rest (math, ROS publishing, peripheral channel routing) is configured in
the **Routing** page of the server's web UI.

## Step 1 — declare a WS-input on the server

Open the server's web UI in a browser at `http://opensaint.local/` (or
whatever IP / hostname your server resolves to) and go to **Routing**.

The Routing page is one **sheet per controllable node**. Pick the sheet
for the node whose peripheral you want to drive — sheets are titled by
the node's display name ("Track Drive Right", "Head", …).

On that sheet:

1. Click **Add → WebSocket Input** (or use the "+" button in the toolbar
   and pick "WebSocket input").
2. Give it a label that'll be obvious in the controller picker — e.g.
   "Forward Throttle", "Pan Servo", "Headlight Brightness".
3. Wire the new WS-input's output to whatever should consume it:
   - **Directly to a peripheral channel** for the simplest "joystick →
     motor" case.
   - **Through operator nodes** (curve / scale / deadzone / mix) if the
     raw axis value needs shaping before it hits the peripheral.
   - **To an Output node** if you also want the value published as a ROS
     topic for other consumers.

The kind on the WS-input matters: leave it as **command** (the default
for any WS-input you add by hand). The controller's binding picker
filters out `state` WS-inputs because those echo sensor data and a
controller can't meaningfully drive them.

Save the sheet. The server commits it to disk and notifies the
evaluator; the evaluator now knows it has a slot waiting for values at
`(sheet_id, input_id)`.

## Step 2 — bind a controller input

On the Deck, launch the controller. Make sure the connection indicator
in the top bar is green — bindings can't enumerate WS-inputs if the
WebSocket isn't connected.

1. Open the **Bindings** page (left rail).
2. Click **Edit Profile** (or pick a profile and click "Add Binding").
3. Pick the source:
   - **Analog** for sticks and triggers (`left_stick_x`, `right_trigger`,
     …)
   - **Digital** for buttons + D-pad
4. For the action, pick **Direct Control** (most common — feeds the raw
   transformed value into the slot). Tank/skid-steer setups can use
   **Differential Drive** with a throttle + turn pair.
5. The target picker switches between two modes:
   - **Routing sheet** — pick a sheet, then pick a WS-input on that
     sheet. This is the recommended path. Both lists come from the
     server's live state (`list_websocket_inputs`); they'll be empty if
     step 1 hasn't been done.
   - **ROS topic** — addresses a raw topic + channel. Legacy mode kept
     around so binding profiles written before the routing sheets
     existed still parse.
6. Tune the **Transform**:
   - **Deadzone** — small inputs near zero are discarded. `0.1` is sane
     for sticks; triggers usually don't need any.
   - **Scale** — multiplier on the value. Use `-1` to flip direction
     when the actuator is wired backwards.
   - **Expo** — curves the response so small inputs are gentle and big
     inputs still hit max. `0` is linear; `0.5` is a good default for
     servos.
   - **Invert** — sign flip independent of scale; mostly there for
     "stick up should be positive" wiring on inverted axes.
7. Save the profile. The binding fires immediately — push the input and
   the affected peripheral on the sheet should react.

## Step 3 — verify on the server

Back in the server's Routing page, hover over (or click) the WS-input
node. Live values flow through the sheet via the
`router.values_changed` push channel and are surfaced as small badges
on every node along the path. Push the gamepad axis and you should
see the value light up at the WS-input, propagate through the
operators, and land at the peripheral channel.

If the WS-input badge doesn't move when you push the input:

- The controller is sending to a different `(sheet_id, input_id)`.
  Re-open the binding and re-pick from the routing-sheet picker.
- The controller isn't connected. Check the indicator on the
  controller, and check `Clients` on the server's web UI — there should
  be a row for the Deck.
- The WebSocket password changed. Re-enter it in the controller's
  connection panel; if you've forgotten it, change it from the server's
  Settings page (it persists back to `server_config.yaml`).

If the WS-input badge moves but downstream nodes don't:

- A wire in the chain is unhooked. Wires are saved when you draw them,
  but a sheet edited in another tab can clobber them.
- An operator has wrong defaults. Operator nodes accept literal
  fallback constants for unwired inputs ("scale to range" needs an
  explicit min/max, for example) — those defaults are shown directly
  on the node card.

## Differential drive (skid-steer / tank tracks)

Skid-steer needs a single throttle + a single turn axis to be combined
into a left-track and right-track value. The controller supports this
natively without you having to build the mixer with operator nodes:

1. Add two WS-inputs to the routing sheet — one for the left track
   peripheral channel, one for the right.
2. Wire each WS-input directly to its peripheral channel sink.
3. In the controller: **Bindings → Add → Analog → Differential Drive**.
4. Pick **Throttle** = `left_stick_y`, **Turn** = `left_stick_x` (or
   whichever pair you prefer).
5. Pick both WS-inputs in the picker — the UI asks for "Left target"
   and "Right target" explicitly.
6. Adjust **mix** to taste — `0.5` is a 50/50 throttle-vs-turn blend,
   higher values bias toward turning.

The controller does the mixing locally and sends two `router.set_input`
messages per tick.

## Modifier bindings (precision mode, speed boost)

A modifier binding doesn't drive a WS-input directly — it scales
*other* bindings while held. Useful for trigger-based precision:

1. **Bindings → Add → Analog → Modifier**
2. Pick the trigger (e.g. `left_trigger`).
3. Pick an effect:
   - **precision_mode** — scales all other analog outputs by
     `(1 - trigger * (1 - min_scale))`. Held full, outputs are at
     `min_scale`. Released, outputs are at `1.0`.
   - **speed_boost** — scales other outputs up by `(1 + trigger * max_boost)`.
   - **scale_target** — scales one specific binding only.

Modifiers are evaluated client-side before the WS message goes out, so
the server doesn't need any extra routing.

## Preset panels (mood / animation / sound triggers)

For digital bindings that fire a preset rather than drive a continuous
value:

1. Build the presets on the server's Routing page (or via the
   `routing.add_preset` WS action when scripting).
2. In the controller: **Bindings → Add → Digital → Activate Preset**
   (fire one immediately) or **Show Preset Panel** (open an overlay
   for D-pad navigation).
3. The button trigger (`press`, `release`, `hold`, `double_tap`,
   `long_press`) decides when the action fires.

The full preset data model and panel-overlay layout are in
[`BINDINGS_SYSTEM.md`](BINDINGS_SYSTEM.md).

## Storing / sharing profiles

Profiles are persisted under the Flatpak app data dir:

```
~/.var/app/com.saintos.Controller/config/saint-controller/profiles.json
```

That file can be checked in to a git repo, copied between Decks, or
shared with another operator. The format is documented at the top of
`src/app/core/services/bindings.service.ts`.

## See also

- [`BINDINGS_SYSTEM.md`](BINDINGS_SYSTEM.md) — full data model
  (input sources, action types, preset panels)
- [`../../saint_os/docs/SERVER_GUIDE.md`](../../saint_os/docs/SERVER_GUIDE.md) —
  server install, node adoption, peripheral install, routing-sheet
  authoring
