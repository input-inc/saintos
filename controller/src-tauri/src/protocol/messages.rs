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

    pub fn command(node_id: &str, pin_id: u32, value: Value) -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
            msg_type: "command".to_string(),
            action: "set_value".to_string(),
            params: Some(serde_json::json!({
                "node_id": node_id,
                "pin_id": pin_id,
                "value": value
            })),
            password: None,
        }
    }

    pub fn subscribe(topics: &[&str]) -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
            msg_type: "subscribe".to_string(),
            action: "topics".to_string(),
            params: Some(serde_json::json!({ "topics": topics })),
            password: None,
        }
    }

    pub fn emergency_stop() -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
            msg_type: "command".to_string(),
            action: "emergency_stop".to_string(),
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
    #[serde(rename = "type")]
    pub msg_type: String,
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
}

impl Default for ConnectionState {
    fn default() -> Self {
        Self {
            status: ConnectionStatus::Disconnected,
            error: None,
        }
    }
}
