use serde::{Deserialize, Serialize};
use serde_json::Value;

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
            id: uuid::Uuid::new_v4().to_string(),
            msg_type: "auth".to_string(),
            action: "login".to_string(),
            params: None,
            password: Some(password.to_string()),
        }
    }

    /// Legacy command using node_id + pin_id (deprecated)
    pub fn command(node_id: &str, pin_id: u32, value: Value) -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
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
            id: uuid::Uuid::new_v4().to_string(),
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
            id: uuid::Uuid::new_v4().to_string(),
            msg_type: "discovery".to_string(),
            action: "get_roles".to_string(),
            params: None,
            password: None,
        }
    }

    /// Discovery request to get active roles (roles assigned to nodes)
    pub fn discover_active_roles() -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
            msg_type: "discovery".to_string(),
            action: "get_active_roles".to_string(),
            params: None,
            password: None,
        }
    }

    /// Discovery request to get controllable functions
    pub fn discover_controllable() -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
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
            id: uuid::Uuid::new_v4().to_string(),
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
            id: uuid::Uuid::new_v4().to_string(),
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
            id: uuid::Uuid::new_v4().to_string(),
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
            id: uuid::Uuid::new_v4().to_string(),
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
            id: uuid::Uuid::new_v4().to_string(),
            msg_type: "subscribe".to_string(),
            action: "subscribe".to_string(),
            params: Some(serde_json::json!({ "topics": topics })),
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
            id: uuid::Uuid::new_v4().to_string(),
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
            id: uuid::Uuid::new_v4().to_string(),
            msg_type: "management".to_string(),
            action: "get_estop_state".to_string(),
            params: None,
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
