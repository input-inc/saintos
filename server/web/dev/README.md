# SAINT.OS Mock WebSocket Server

A dev-only Node WebSocket server that speaks the same JSON protocol as
the real `saint_server` Pi backend. Lets you iterate on the Vue
operator UI without flashing firmware, running ROS, or owning hardware.

## Run

Two terminals from `server/web`:

```sh
npm install            # one-time — picks up the `ws` devDependency
npm run mock           # starts the mock on :8081
SAINT_HOST=http://localhost:8081 npm run dev   # Vite on :5173, WS proxied
```

Then browse <http://localhost:5173>. The Vue store auto-connects via
`/api/ws`, Vite proxies it to the mock, and every page should render
with the seeded fixture data.

Override the port with `MOCK_PORT=9000 npm run mock` if 8081 is taken.

## Files

| File                | What it holds                                       |
| ------------------- | --------------------------------------------------- |
| `mock-server.js`    | HTTP + WS plumbing, broadcast loops, per-client subs |
| `mock-handlers.js`  | Management/command/router/control action handlers    |
| `mock-state.js`     | Seed fixtures + in-memory mutable state              |

## Adding a new mock action

1. Find the action name (grep the Vue source for the string).
2. Add an entry to `managementHandlers` in `mock-handlers.js`. Return
   `{ ok: true, data }` for success, `{ ok: false, message }` for an error.
3. If the action mutates state, also call `ctx.broadcast('<topic>', data)`
   so the UI's `useWsTopic` re-renders without a manual refetch.

## What it does broadcast

- `system_status` every 2 s
- `pin_state/host_controller` every 1 s
- `pin_state/<node_id>` every 250 ms per adopted node
- `routing_values` every 500 ms
- `system_routing` on routing mutations (add wire, etc.)
- `activity` randomly every 5–15 s

Subscriptions are honored — broadcasts only land on clients that
subscribed via the `{type: "subscribe", action: "subscribe"}` frame.

## What it cannot fake

- ROS topic catalog is a tiny canned list (`/joy`, `/cmd_vel`).
- LiveLink, terminal, and binary frames return acks but no live data.
- Firmware update flow returns success immediately — no progress events.

## Notes

- `auth_required` is always false; the login screen is bypassed.
- The mock is not exposed in the Vite build — only used via `npm run mock`.
