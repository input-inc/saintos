use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::sync::atomic::{AtomicU64, Ordering};

/// Monotonic per-process message-id source. Replaces uuid::new_v4() on
/// every outgoing message: ids only need to be unique within a single
/// connection's pending-request window (for response correlation), and
/// a process-lifetime counter guarantees that without the v4 RNG draw +
/// 36-char hyphenated formatting. On the 20 Hz streaming-control path
/// that allocation+RNG was a measurable slice of per-message cost.
static MSG_SEQ: AtomicU64 = AtomicU64::new(0);

fn next_id() -> String {
    format!("c{}", MSG_SEQ.fetch_add(1, Ordering::Relaxed))
}

/// Outgoing message to the SAINT.OS server
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutgoingMessage {
    pub id: String,
    #[serde(rename = "type")]
    pub msg_type: String,
    pub action: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub params: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub password: Option<String>,
}

impl OutgoingMessage {
    pub fn auth(password: &str) -> Self {
        Self {
            id: next_id(),
            msg_type: "auth".to_string(),
            action: "login".to_string(),
            params: None,
            password: Some(password.to_string()),
        }
    }

    /// Legacy command using node_id + pin_id (deprecated)
    pub fn command(node_id: &str, pin_id: u32, value: Value) -> Self {
        Self {
            id: next_id(),
            msg_type: "control".to_string(),
            action: "set_pin_value".to_string(),
            params: Some(serde_json::json!({
                "node_id": node_id,
                "gpio": pin_id,
                "value": value
            })),
            password: None,
        }
    }

    /// High-level control using role + function (preferred)
    pub fn control_function(role: &str, function: &str, value: Value) -> Self {
        Self {
            id: next_id(),
            msg_type: "control".to_string(),
            action: "set_function_value".to_string(),
            params: Some(serde_json::json!({
                "role": role,
                "function": function,
                "value": value
            })),
            password: None,
        }
    }

    /// Discovery request to get available roles
    pub fn discover_roles() -> Self {
        Self {
            id: next_id(),
            msg_type: "discovery".to_string(),
            action: "get_roles".to_string(),
            params: None,
            password: None,
        }
    }

    /// Discovery request to get active roles (roles assigned to nodes)
    #[allow(dead_code)] // part of the discovery API; not wired into a UI yet
    pub fn discover_active_roles() -> Self {
        Self {
            id: next_id(),
            msg_type: "discovery".to_string(),
            action: "get_active_roles".to_string(),
            params: None,
            password: None,
        }
    }

    /// Discovery request to get controllable functions
    pub fn discover_controllable() -> Self {
        Self {
            id: next_id(),
            msg_type: "discovery".to_string(),
            action: "get_controllable_functions".to_string(),
            params: None,
            password: None,
        }
    }

    /// Discovery request to enumerate ROS topic channels (topic + the
    /// scalar fields inside each message). Replaces the legacy
    /// role/function picker in the bindings UI with a topic/channel
    /// picker that mirrors the server-side routing graph.
    pub fn discover_topic_channels() -> Self {
        Self {
            id: next_id(),
            msg_type: "ros".to_string(),
            action: "list_topic_channels".to_string(),
            params: None,
            password: None,
        }
    }

    /// Enumerate WebSocket-input nodes defined across routing sheets.
    /// Each sheet exposes WS inputs as named scratch slots the
    /// controller can write into; the binding picker uses this list
    /// instead of the old topic/channel picker.
    pub fn list_websocket_inputs() -> Self {
        Self {
            id: next_id(),
            msg_type: "router".to_string(),
            action: "list_websocket_inputs".to_string(),
            params: None,
            password: None,
        }
    }

    /// Push a scalar value onto a WebSocket-input slot on a routing
    /// sheet. Addressed by (sheet_id, input_id); the server routes the
    /// value straight into the routing evaluator's source cache so
    /// downstream operators and peripheral sinks see it immediately.
    pub fn set_ws_input(sheet_id: &str, input_id: &str, value: Value) -> Self {
        Self {
            id: next_id(),
            msg_type: "router".to_string(),
            action: "set_input".to_string(),
            params: Some(serde_json::json!({
                "sheet_id": sheet_id,
                "input_id": input_id,
                "value": value,
            })),
            password: None,
        }
    }

    /// Push a single scalar onto a ROS topic channel. The server-side
    /// `set_topic_channel` handler maintains a per-topic buffer so we
    /// only need to send the one field that changed; the merged
    /// message is published with the existing throttle.
    pub fn set_topic_channel(topic: &str, channel: &str, value: Value) -> Self {
        Self {
            id: next_id(),
            msg_type: "ros".to_string(),
            action: "set_topic_channel".to_string(),
            params: Some(serde_json::json!({
                "endpoint": topic,
                "field": channel,
                "value": value,
            })),
            password: None,
        }
    }

    pub fn subscribe(topics: &[&str]) -> Self {
        Self {
            id: next_id(),
            msg_type: "subscribe".to_string(),
            action: "subscribe".to_string(),
            params: Some(serde_json::json!({ "topics": topics })),
            password: None,
        }
    }

    /// Subscribe to a dynamic set of owned topic strings (e.g. one
    /// `pin_state/<node_id>` per adopted node). Same wire shape as
    /// `subscribe`; separate constructor so callers building a Vec at
    /// runtime don't have to juggle `&str` lifetimes.
    pub fn subscribe_owned(topics: &[String]) -> Self {
        Self {
            id: next_id(),
            msg_type: "subscribe".to_string(),
            action: "subscribe".to_string(),
            params: Some(serde_json::json!({ "topics": topics })),
            password: None,
        }
    }

    /// List adopted nodes. Server replies with `{ nodes: [{node_id, …}] }`.
    /// The battery panel uses this to learn which `pin_state/<node>`
    /// topics to subscribe to.
    pub fn list_adopted() -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "list_adopted".to_string(),
            params: None,
            password: None,
        }
    }

    /// List saved animations. Server replies with
    /// `{ animations: [{id, name, icon, …}] }`. Forwarded to the
    /// frontend on the `library-animations` event.
    pub fn list_animations() -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "list_animations".to_string(),
            params: None,
            password: None,
        }
    }

    /// List saved poses. Server replies with
    /// `{ poses: [{id, name, icon, …}] }`. Forwarded on `library-poses`.
    pub fn list_poses() -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "list_poses".to_string(),
            params: None,
            password: None,
        }
    }

    /// List saved soundboard sounds. Server replies with
    /// `{ sounds: [{id, name, icon, …}] }`. Forwarded on `library-sounds`.
    pub fn list_sounds() -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "list_sounds".to_string(),
            params: None,
            password: None,
        }
    }

    /// Start (play) a saved animation by id.
    pub fn start_animation(id: &str) -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "start_animation".to_string(),
            params: Some(serde_json::json!({ "id": id })),
            password: None,
        }
    }

    /// Stop (cancel) a currently-playing animation by id. Used to
    /// toggle-stop looping animations from the preset panel.
    pub fn stop_animation(id: &str) -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "stop_animation".to_string(),
            params: Some(serde_json::json!({ "id": id })),
            password: None,
        }
    }

    /// Apply (snap to) a saved pose by id.
    pub fn apply_pose(id: &str) -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "apply_pose".to_string(),
            params: Some(serde_json::json!({ "id": id })),
            password: None,
        }
    }

    /// Play a saved soundboard sound by id.
    pub fn play_sound(id: &str) -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "play_sound".to_string(),
            params: Some(serde_json::json!({ "id": id })),
            password: None,
        }
    }

    pub fn emergency_stop() -> Self {
        // Server handler matches on action == "estop" (latching toggle:
        // each press flips system-wide state, fans out to all adopted
        // nodes, broadcasts on the "estop" topic). The legacy
        // "emergency_stop" action name we used to send fell through the
        // server's match and silently produced "Unknown command action"
        // errors — pressing the button did nothing. Keep the Rust
        // method name (it's the user-facing semantic) but emit the wire
        // action the server actually accepts.
        Self {
            id: next_id(),
            msg_type: "command".to_string(),
            action: "estop".to_string(),
            params: None,
            password: None,
        }
    }

    /// One-shot query for the current system-wide e-stop latch state.
    /// Sent right after auth so a freshly-connected controller picks
    /// up the latched state without waiting for the next toggle
    /// broadcast. Server responds with `{ active, changed_at }`.
    pub fn get_estop_state() -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "get_estop_state".to_string(),
            params: None,
            password: None,
        }
    }

    /// Current AP WiFi config. Server replies with
    /// `{ ok, ssid, password, band, channel, iface }` — same handler
    /// the web dashboard's WiFi card uses. The client strips
    /// `password` before forwarding to the frontend (`wifi-config`
    /// event); the panel has no use for it.
    pub fn wifi_get_config() -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "wifi_get_config".to_string(),
            params: None,
            password: None,
        }
    }

    /// Scan for neighboring APs and report per-channel congestion.
    /// Server replies (~10 s later — `iw scan` runs on the Pi) with
    /// `{ ok, iface, current_channel, channels: [{band, channel,
    /// freq_mhz, ap_count, strongest_signal_dbm, is_dfs, is_current}],
    /// error }`. Forwarded on the `wifi-survey` event.
    pub fn wifi_survey() -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "wifi_survey".to_string(),
            params: None,
            password: None,
        }
    }

    /// Move the server's AP to a different channel. `band` is the wire
    /// encoding ("bg" = 2.4 GHz, "a" = 5 GHz). The server ACKs with
    /// `{ switching: true, band, channel }` BEFORE restarting the AP —
    /// expect the WebSocket to drop for ~5-10 s right after (60 s+ if
    /// a DFS channel needs its radar-listen period).
    pub fn wifi_set_channel(band: &str, channel: u32) -> Self {
        Self {
            id: next_id(),
            msg_type: "management".to_string(),
            action: "wifi_set_channel".to_string(),
            params: Some(serde_json::json!({
                "band": band,
                "channel": channel,
            })),
            password: None,
        }
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }
}

/// Incoming message from the SAINT.OS server
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IncomingMessage {
    #[serde(rename = "type", default)]
    pub msg_type: String,
    /// Topic name for `type == "state"` broadcasts (e.g. "estop",
    /// "system_routing", "pin_state/<node_id>"). Set by the server's
    /// `broadcast_state(topic, data)` helper.
    #[serde(default)]
    pub node: Option<String>,
    #[serde(default)]
    pub status: Option<String>,
    #[serde(default)]
    pub message: Option<String>,
    #[serde(default)]
    pub data: Option<Value>,
}

impl IncomingMessage {
    pub fn is_auth_success(&self) -> bool {
        self.msg_type == "auth_result" && self.status.as_deref() == Some("ok")
    }

    #[allow(dead_code)] // companion to is_auth_success; kept for completeness
    pub fn is_auth_failure(&self) -> bool {
        self.msg_type == "auth_result" && self.status.as_deref() != Some("ok")
    }

    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }
}

/// Connection status for UI updates
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ConnectionStatus {
    Disconnected,
    Connecting,
    Authenticating,
    Connected,
    Error,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectionState {
    pub status: ConnectionStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Mirror of the server's system-wide e-stop latch. Set from the
    /// `estop` state-topic broadcast and the `get_estop_state`
    /// management response. Used to gate outgoing streaming control
    /// at the client layer so a stuck stick can't keep republishing
    /// values into the WS while the system is supposed to be safed.
    #[serde(default)]
    pub estop_active: bool,
}

impl Default for ConnectionState {
    fn default() -> Self {
        Self {
            status: ConnectionStatus::Disconnected,
            error: None,
            estop_active: false,
        }
    }
}

#[cfg(test)]
mod tests {
    //! Shape lock-down + serialize benchmark for outgoing messages.
    //! Bench is #[ignore]d; run with:
    //!   cargo test --release --lib -- --ignored --nocapture bench
    use super::*;
    use serde_json::json;
    use std::time::Instant;

    #[test]
    fn topic_channel_message_shape() {
        let m = OutgoingMessage::set_topic_channel("/tracks", "left_velocity", json!(0.5));
        assert_eq!(m.msg_type, "ros");
        assert_eq!(m.action, "set_topic_channel");
        let p = m.params.as_ref().expect("params present");
        assert_eq!(p["endpoint"], "/tracks");
        assert_eq!(p["field"], "left_velocity");
        assert_eq!(p["value"], 0.5);
        assert!(!m.id.is_empty(), "id must be present");
        let s = m.to_json();
        assert!(s.contains("\"type\":\"ros\""), "json: {s}");
        assert!(s.contains("set_topic_channel"), "json: {s}");
    }

    #[test]
    fn ws_input_message_shape() {
        let m = OutgoingMessage::set_ws_input("sheetA", "in1", json!(-1.0));
        assert_eq!(m.msg_type, "router");
        assert_eq!(m.action, "set_input");
        let p = m.params.as_ref().expect("params present");
        assert_eq!(p["sheet_id"], "sheetA");
        assert_eq!(p["input_id"], "in1");
        assert!(!m.id.is_empty());
    }

    #[test]
    fn message_ids_are_unique() {
        // Per-message-unique ids back request/response correlation. The
        // uuid→counter optimization MUST preserve this invariant.
        let a = OutgoingMessage::set_topic_channel("/t", "c", json!(1));
        let b = OutgoingMessage::set_topic_channel("/t", "c", json!(1));
        let c = OutgoingMessage::auth("pw");
        assert_ne!(a.id, b.id);
        assert_ne!(b.id, c.id);
        assert_ne!(a.id, c.id);
    }

    #[test]
    fn auth_message_carries_password() {
        let m = OutgoingMessage::auth("hunter2");
        assert_eq!(m.msg_type, "auth");
        assert_eq!(m.action, "login");
        assert_eq!(m.password.as_deref(), Some("hunter2"));
        // password must serialize so the server can authenticate.
        assert!(m.to_json().contains("hunter2"));
    }

    #[test]
    fn control_function_message_shape() {
        let m = OutgoingMessage::control_function("driver", "throttle", json!(0.75));
        assert_eq!(m.msg_type, "control");
        assert_eq!(m.action, "set_function_value");
        let p = m.params.as_ref().expect("params present");
        assert_eq!(p["role"], "driver");
        assert_eq!(p["function"], "throttle");
        assert_eq!(p["value"], 0.75);
    }

    #[test]
    fn estop_uses_the_action_the_server_accepts() {
        // Regression guard: the server matches on action == "estop". We
        // previously sent "emergency_stop", which fell through the match
        // and silently did nothing — the e-stop button was dead. Lock the
        // wire action (and type) so that can't regress.
        let m = OutgoingMessage::emergency_stop();
        assert_eq!(m.msg_type, "command");
        assert_eq!(m.action, "estop");
        let s = m.to_json();
        assert!(s.contains("\"action\":\"estop\""), "json: {s}");
        assert!(!s.contains("emergency_stop"), "must not send the dead action name: {s}");
    }

    #[test]
    fn set_topic_channel_preserves_value_types() {
        // Negative, zero, and fractional values must round-trip exactly —
        // the streaming control path sends all three constantly.
        for v in [json!(-1.0), json!(0.0), json!(0.5), json!(1.0)] {
            let m = OutgoingMessage::set_topic_channel("/t", "c", v.clone());
            assert_eq!(m.params.as_ref().unwrap()["value"], v);
        }
    }

    #[test]
    fn discovery_and_library_message_shapes() {
        assert_eq!(OutgoingMessage::discover_topic_channels().action, "list_topic_channels");
        assert_eq!(OutgoingMessage::list_websocket_inputs().action, "list_websocket_inputs");
        assert_eq!(OutgoingMessage::list_animations().action, "list_animations");
        assert_eq!(OutgoingMessage::list_poses().action, "list_poses");
        assert_eq!(OutgoingMessage::list_sounds().action, "list_sounds");
        assert_eq!(OutgoingMessage::start_animation("anim1").params.unwrap()["id"], "anim1");
        assert_eq!(OutgoingMessage::apply_pose("pose1").params.unwrap()["id"], "pose1");
        assert_eq!(OutgoingMessage::play_sound("snd1").params.unwrap()["id"], "snd1");
    }

    #[test]
    fn wifi_message_shapes() {
        // Wire actions must match the server's management handler
        // (websocket_handler.py) exactly — a drifted action name fails
        // silently as "unknown action" (same failure mode the estop
        // button had).
        assert_eq!(OutgoingMessage::wifi_get_config().action, "wifi_get_config");
        assert_eq!(OutgoingMessage::wifi_get_config().msg_type, "management");
        assert_eq!(OutgoingMessage::wifi_survey().action, "wifi_survey");
        assert_eq!(OutgoingMessage::wifi_survey().msg_type, "management");

        let m = OutgoingMessage::wifi_set_channel("bg", 6);
        assert_eq!(m.msg_type, "management");
        assert_eq!(m.action, "wifi_set_channel");
        let p = m.params.as_ref().expect("params present");
        assert_eq!(p["band"], "bg");
        assert_eq!(p["channel"], 6);
    }

    #[test]
    fn incoming_auth_result_predicates() {
        let ok = IncomingMessage::from_json(r#"{"type":"auth_result","status":"ok"}"#).unwrap();
        assert!(ok.is_auth_success());
        assert!(!ok.is_auth_failure());

        let bad = IncomingMessage::from_json(
            r#"{"type":"auth_result","status":"error","message":"bad password"}"#).unwrap();
        assert!(!bad.is_auth_success());
        assert!(bad.is_auth_failure());
        assert_eq!(bad.message.as_deref(), Some("bad password"));
    }

    #[test]
    fn incoming_state_broadcast_parses_topic_and_data() {
        // The estop state topic the client reads to gate outgoing control.
        let msg = IncomingMessage::from_json(
            r#"{"type":"state","node":"estop","data":{"active":true}}"#).unwrap();
        assert_eq!(msg.msg_type, "state");
        assert_eq!(msg.node.as_deref(), Some("estop"));
        assert_eq!(msg.data.unwrap()["active"], true);
    }

    #[test]
    fn incoming_tolerates_missing_optional_fields() {
        // Server greetings/acks often omit most fields; parsing must not
        // fail or the connection handshake aborts.
        let msg = IncomingMessage::from_json(r#"{"type":"connected"}"#).unwrap();
        assert_eq!(msg.msg_type, "connected");
        assert!(msg.status.is_none() && msg.data.is_none() && msg.node.is_none());
    }

    #[test]
    fn connection_state_defaults_to_safe_disconnected() {
        let s = ConnectionState::default();
        assert_eq!(s.status, ConnectionStatus::Disconnected);
        assert!(s.error.is_none());
        assert!(!s.estop_active, "must default to NOT e-stopped");
    }

    #[test]
    #[ignore = "benchmark — run explicitly with --ignored --nocapture"]
    fn bench_build_and_serialize() {
        let n: u32 = 2_000_000;
        for i in 0..10_000u32 {
            let m = OutgoingMessage::set_topic_channel("/tracks", "left_velocity", json!(i));
            let _ = m.to_json();
        }
        let mut sink = 0usize;
        let t0 = Instant::now();
        for i in 0..n {
            let m = OutgoingMessage::set_topic_channel(
                "/tracks", "left_velocity", json!(i as f64 * 0.001));
            sink += m.to_json().len();
        }
        let el = t0.elapsed();
        println!(
            "\n[bench] build set_topic_channel + to_json: {:.1} ns/msg  ({:.0} msg/s, n={}, sink={})",
            el.as_nanos() as f64 / n as f64,
            n as f64 / el.as_secs_f64(),
            n,
            sink,
        );
    }
}
