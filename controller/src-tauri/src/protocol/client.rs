use super::messages::{ConnectionState, ConnectionStatus, IncomingMessage, OutgoingMessage};
use futures_util::{SinkExt, StreamExt};
use parking_lot::RwLock;
use serde_json::Value;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tauri::{AppHandle, Emitter, Runtime};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use tokio_tungstenite::{connect_async, tungstenite::Message, MaybeTlsStream, WebSocketStream};

const THROTTLE_MS: u64 = 50;
const RECONNECT_DELAY_MS: u64 = 1000;
const MAX_RECONNECT_DELAY_MS: u64 = 30000;

type WsStream = WebSocketStream<MaybeTlsStream<TcpStream>>;

pub struct WebSocketClient {
    state: Arc<RwLock<ConnectionState>>,
    command_tx: Arc<RwLock<Option<mpsc::Sender<OutgoingMessage>>>>,
    shutdown_tx: Arc<RwLock<Option<mpsc::Sender<()>>>>,
    /// Per-target throttle timers (key = "role:function")
    last_command_times: Arc<RwLock<HashMap<String, Instant>>>,
}

impl WebSocketClient {
    pub fn new() -> Self {
        Self {
            state: Arc::new(RwLock::new(ConnectionState::default())),
            command_tx: Arc::new(RwLock::new(None)),
            shutdown_tx: Arc::new(RwLock::new(None)),
            last_command_times: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    pub fn state(&self) -> ConnectionState {
        self.state.read().clone()
    }

    pub fn is_connected(&self) -> bool {
        self.state.read().status == ConnectionStatus::Connected
    }

    pub fn connect<R: Runtime + 'static>(
        &self,
        app_handle: AppHandle<R>,
        host: &str,
        port: u16,
        password: &str,
    ) -> Result<(), String> {
        // Disconnect existing connection
        self.disconnect();

        let url = format!("ws://{}:{}/api/ws", host, port);
        let password = password.to_string();
        let state = self.state.clone();
        let command_tx_holder = self.command_tx.clone();
        let shutdown_tx_holder = self.shutdown_tx.clone();

        // Update state
        {
            let mut s = state.write();
            s.status = ConnectionStatus::Connecting;
            s.error = None;
        }
        emit_state(&app_handle, &state.read());

        // Create channels
        let (command_tx, command_rx) = mpsc::channel::<OutgoingMessage>(100);
        let (shutdown_tx, shutdown_rx) = mpsc::channel::<()>(1);

        *command_tx_holder.write() = Some(command_tx);
        *shutdown_tx_holder.write() = Some(shutdown_tx);

        // Spawn connection task using std::thread + tokio runtime
        std::thread::spawn(move || {
            let rt = tokio::runtime::Runtime::new().unwrap();
            rt.block_on(async move {
                run_connection_loop(
                    url,
                    password,
                    state,
                    app_handle,
                    command_rx,
                    shutdown_rx,
                )
                .await;
            });
        });

        Ok(())
    }

    pub fn disconnect(&self) {
        if let Some(tx) = self.shutdown_tx.write().take() {
            let _ = tx.blocking_send(());
        }
        *self.command_tx.write() = None;

        let mut s = self.state.write();
        s.status = ConnectionStatus::Disconnected;
        s.error = None;
        // Clear the estop mirror on disconnect. The bootstrap on the
        // next connect (subscribe + get_estop_state) will re-set it
        // from the authoritative server-side value. Without this we
        // could end up gating outgoing streaming control while
        // disconnected, then briefly while reconnecting to a different
        // server that has estop released.
        s.estop_active = false;
    }

    /// Legacy command using node_id + pin_id (deprecated)
    pub fn send_command(&self, node_id: &str, pin_id: u32, value: Value) -> Result<(), String> {
        // Per-target throttle key
        let throttle_key = format!("{}:{}", node_id, pin_id);

        // Throttle commands
        {
            let mut times = self.last_command_times.write();
            let now = Instant::now();
            if let Some(last_time) = times.get(&throttle_key) {
                if now.duration_since(*last_time) < Duration::from_millis(THROTTLE_MS) {
                    return Ok(());
                }
            }
            times.insert(throttle_key, now);
        }

        let tx = self
            .command_tx
            .read()
            .clone()
            .ok_or_else(|| "Not connected".to_string())?;

        let msg = OutgoingMessage::command(node_id, pin_id, value);

        tx.blocking_send(msg)
            .map_err(|e| format!("Failed to queue command: {}", e))
    }

    /// High-level control using role + function (preferred)
    pub fn send_function_control(&self, role: &str, function: &str, value: Value) -> Result<(), String> {
        // Check if this is a "stop" command (value near zero) - these bypass throttle
        let is_stop_command = match &value {
            Value::Number(n) => n.as_f64().map(|v| v.abs() < 0.01).unwrap_or(false),
            _ => false,
        };

        // Per-target throttle key
        let throttle_key = format!("{}:{}", role, function);

        // Throttle commands (but not stop commands - those are critical)
        if !is_stop_command {
            let mut times = self.last_command_times.write();
            let now = Instant::now();
            if let Some(last_time) = times.get(&throttle_key) {
                if now.duration_since(*last_time) < Duration::from_millis(THROTTLE_MS) {
                    return Ok(());
                }
            }
            times.insert(throttle_key, now);
        }

        let tx = self
            .command_tx
            .read()
            .clone()
            .ok_or_else(|| "Not connected".to_string())?;

        let msg = OutgoingMessage::control_function(role, function, value);

        // Use try_send to avoid blocking - drop command if channel is full
        match tx.try_send(msg) {
            Ok(()) => Ok(()),
            Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                // Channel full, drop this command (next one will go through)
                log::trace!("Channel full, dropped command for {}:{}", role, function);
                Ok(())
            }
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => {
                Err("Connection closed".to_string())
            }
        }
    }

    /// Request discovery of available roles
    pub fn request_discover_roles(&self) -> Result<(), String> {
        log::info!("Requesting role discovery from server...");

        let tx = self
            .command_tx
            .read()
            .clone()
            .ok_or_else(|| {
                log::warn!("request_discover_roles: Not connected");
                "Not connected".to_string()
            })?;

        let msg = OutgoingMessage::discover_roles();
        log::debug!("Sending discover_roles: {}", msg.to_json());

        tx.blocking_send(msg)
            .map_err(|e| {
                log::error!("Failed to send discovery request: {}", e);
                format!("Failed to send discovery request: {}", e)
            })
    }

    /// Request discovery of controllable functions
    pub fn request_discover_controllable(&self) -> Result<(), String> {
        log::info!("Requesting controllable functions discovery from server...");

        let tx = self
            .command_tx
            .read()
            .clone()
            .ok_or_else(|| {
                log::warn!("request_discover_controllable: Not connected");
                "Not connected".to_string()
            })?;

        let msg = OutgoingMessage::discover_controllable();
        log::debug!("Sending discover_controllable: {}", msg.to_json());

        tx.blocking_send(msg)
            .map_err(|e| {
                log::error!("Failed to send discovery request: {}", e);
                format!("Failed to send discovery request: {}", e)
            })
    }

    /// Request enumeration of ROS topics and the scalar channels each
    /// message exposes. Server responds via a `discovery-topic-channels`
    /// Tauri event so the binding UI can populate its Topic/Channel
    /// pickers (the role/function picker's replacement).
    pub fn request_discover_topic_channels(&self) -> Result<(), String> {
        log::info!("Requesting topic-channel discovery from server...");

        let tx = self
            .command_tx
            .read()
            .clone()
            .ok_or_else(|| {
                log::warn!("request_discover_topic_channels: Not connected");
                "Not connected".to_string()
            })?;

        let msg = OutgoingMessage::discover_topic_channels();
        log::debug!("Sending discover_topic_channels: {}", msg.to_json());

        tx.blocking_send(msg)
            .map_err(|e| {
                log::error!("Failed to send discovery request: {}", e);
                format!("Failed to send discovery request: {}", e)
            })
    }

    /// Request enumeration of WebSocket-input slots defined across all
    /// routing sheets. Server replies with `discovery-ws-inputs`.
    pub fn request_discover_ws_inputs(&self) -> Result<(), String> {
        log::info!("Requesting WS-input discovery from server...");

        let tx = self
            .command_tx
            .read()
            .clone()
            .ok_or_else(|| {
                log::warn!("request_discover_ws_inputs: Not connected");
                "Not connected".to_string()
            })?;

        let msg = OutgoingMessage::list_websocket_inputs();
        log::debug!("Sending list_websocket_inputs: {}", msg.to_json());

        tx.blocking_send(msg)
            .map_err(|e| {
                log::error!("Failed to send WS-input discovery: {}", e);
                format!("Failed to send WS-input discovery: {}", e)
            })
    }

    /// Push a scalar onto a sheet's WebSocket-input slot. The binding
    /// runtime calls this on every gamepad-axis tick that maps to a WS
    /// input — same throttle policy as the legacy topic/channel path.
    pub fn send_ws_input_value(&self, sheet_id: &str, input_id: &str, value: Value)
        -> Result<(), String>
    {
        if sheet_id.is_empty() || input_id.is_empty() {
            log::warn!(
                "send_ws_input_value: empty sheet_id ('{}') or input_id ('{}'); \
                 re-bind this axis in the bindings UI",
                sheet_id, input_id
            );
            return Ok(());
        }
        // E-Stop gate. Server's routing evaluator drops these too, but
        // we drop at the client to keep the WS quiet (an engaged
        // estop on a tank with both sticks deflected would otherwise
        // dump 100 Hz of unused traffic into the socket for every
        // axis).
        if self.state.read().estop_active {
            return Ok(());
        }
        let is_stop_command = match &value {
            Value::Number(n) => n.as_f64().map(|v| v.abs() < 0.01).unwrap_or(false),
            _ => false,
        };
        let throttle_key = format!("ws::{}::{}", sheet_id, input_id);
        if !is_stop_command {
            let mut times = self.last_command_times.write();
            let now = Instant::now();
            if let Some(last_time) = times.get(&throttle_key) {
                if now.duration_since(*last_time) < Duration::from_millis(THROTTLE_MS) {
                    return Ok(());
                }
            }
            times.insert(throttle_key, now);
        }
        log::debug!("→ set_ws_input {}::{} = {}", sheet_id, input_id, value);

        let tx = self
            .command_tx
            .read()
            .clone()
            .ok_or_else(|| "Not connected".to_string())?;

        let msg = OutgoingMessage::set_ws_input(sheet_id, input_id, value);
        match tx.try_send(msg) {
            Ok(()) => Ok(()),
            Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                log::trace!("Channel full, dropped ws_input write for {}::{}", sheet_id, input_id);
                Ok(())
            }
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => {
                Err("Connection closed".to_string())
            }
        }
    }

    /// Push a single scalar onto a ROS topic channel. Used by the
    /// bindings runtime: a joystick axis movement results in
    /// `set_topic_channel(topic, channel, value)` which the server
    /// merges into its per-topic buffer and republishes.
    pub fn send_topic_channel_value(&self, topic: &str, channel: &str, value: Value)
        -> Result<(), String>
    {
        if topic.is_empty() || channel.is_empty() {
            // Common failure mode: a binding was saved with the old
            // role/function schema and the migration left topic/channel
            // empty. Flag it once at warn so the operator sees why
            // nothing is happening.
            log::warn!(
                "send_topic_channel_value: empty topic ('{}') or channel ('{}'); \
                 re-bind this axis in the bindings UI",
                topic, channel
            );
            return Ok(());
        }
        // E-Stop gate — same rationale as send_ws_input_value.
        if self.state.read().estop_active {
            return Ok(());
        }
        let is_stop_command = match &value {
            Value::Number(n) => n.as_f64().map(|v| v.abs() < 0.01).unwrap_or(false),
            _ => false,
        };
        let throttle_key = format!("{}::{}", topic, channel);
        if !is_stop_command {
            let mut times = self.last_command_times.write();
            let now = Instant::now();
            if let Some(last_time) = times.get(&throttle_key) {
                if now.duration_since(*last_time) < Duration::from_millis(THROTTLE_MS) {
                    return Ok(());
                }
            }
            times.insert(throttle_key, now);
        }
        log::debug!("→ set_topic_channel {}::{} = {}", topic, channel, value);

        let tx = self
            .command_tx
            .read()
            .clone()
            .ok_or_else(|| "Not connected".to_string())?;

        let msg = OutgoingMessage::set_topic_channel(topic, channel, value);
        match tx.try_send(msg) {
            Ok(()) => Ok(()),
            Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                log::trace!("Channel full, dropped command for {}::{}", topic, channel);
                Ok(())
            }
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => {
                Err("Connection closed".to_string())
            }
        }
    }

    /// Send emergency stop command (bypasses throttling)
    pub fn send_emergency_stop(&self) -> Result<(), String> {
        let tx = self
            .command_tx
            .read()
            .clone()
            .ok_or_else(|| "Not connected".to_string())?;

        let msg = OutgoingMessage::emergency_stop();

        tx.blocking_send(msg)
            .map_err(|e| format!("Failed to send emergency stop: {}", e))
    }
}

async fn run_connection_loop<R: Runtime>(
    url: String,
    password: String,
    state: Arc<RwLock<ConnectionState>>,
    app_handle: AppHandle<R>,
    mut command_rx: mpsc::Receiver<OutgoingMessage>,
    mut shutdown_rx: mpsc::Receiver<()>,
) {
    let mut reconnect_delay = RECONNECT_DELAY_MS;

    loop {
        match connect_ws(&url, &password, &state, &app_handle).await {
            Ok(ws_stream) => {
                reconnect_delay = RECONNECT_DELAY_MS;

                let should_reconnect = handle_connection(
                    ws_stream,
                    &state,
                    &app_handle,
                    &mut command_rx,
                    &mut shutdown_rx,
                )
                .await;

                if !should_reconnect {
                    break;
                }
            }
            Err(e) => {
                tracing::error!("Connection failed: {}", e);
                {
                    let mut s = state.write();
                    s.status = ConnectionStatus::Error;
                    s.error = Some(e);
                }
                emit_state(&app_handle, &state.read());
            }
        }

        // Check for shutdown
        if shutdown_rx.try_recv().is_ok() {
            break;
        }

        tracing::info!("Reconnecting in {}ms", reconnect_delay);
        tokio::time::sleep(Duration::from_millis(reconnect_delay)).await;
        reconnect_delay = (reconnect_delay * 2).min(MAX_RECONNECT_DELAY_MS);
    }

    {
        let mut s = state.write();
        s.status = ConnectionStatus::Disconnected;
        s.error = None;
    }
    emit_state(&app_handle, &state.read());
}

async fn connect_ws<R: Runtime>(
    url: &str,
    password: &str,
    state: &Arc<RwLock<ConnectionState>>,
    app_handle: &AppHandle<R>,
) -> Result<WsStream, String> {
    tracing::info!("Connecting to {}", url);

    let (ws_stream, _) = connect_async(url)
        .await
        .map_err(|e| format!("WebSocket connection failed: {}", e))?;

    tracing::info!("WebSocket connected, waiting for server greeting...");

    let (mut write, mut read) = ws_stream.split();

    // First, wait for the server's "connected" message
    let connected_msg = tokio::time::timeout(Duration::from_secs(10), read.next())
        .await
        .map_err(|_| "Connection timeout".to_string())?
        .ok_or_else(|| "Connection closed".to_string())?
        .map_err(|e| format!("WebSocket error: {}", e))?;

    if let Message::Text(text) = connected_msg {
        let msg = IncomingMessage::from_json(&text)
            .map_err(|e| format!("Invalid server message: {}", e))?;

        if msg.msg_type != "connected" {
            return Err(format!("Expected 'connected' message, got '{}'", msg.msg_type));
        }

        tracing::info!("Received server greeting, authenticating...");
    } else {
        return Err("Unexpected message type from server".to_string());
    }

    {
        let mut s = state.write();
        s.status = ConnectionStatus::Authenticating;
    }
    emit_state(app_handle, &state.read());

    // Now send authentication
    let auth_msg = OutgoingMessage::auth(password);
    write
        .send(Message::Text(auth_msg.to_json()))
        .await
        .map_err(|e| format!("Failed to send auth: {}", e))?;

    // Wait for auth_result
    let auth_result = tokio::time::timeout(Duration::from_secs(10), read.next())
        .await
        .map_err(|_| "Auth timeout".to_string())?
        .ok_or_else(|| "Connection closed during auth".to_string())?
        .map_err(|e| format!("WebSocket error: {}", e))?;

    if let Message::Text(text) = auth_result {
        let msg =
            IncomingMessage::from_json(&text).map_err(|e| format!("Invalid auth response: {}", e))?;

        if msg.is_auth_success() {
            tracing::info!("Authentication successful");
            {
                let mut s = state.write();
                s.status = ConnectionStatus::Connected;
                s.error = None;
            }
            emit_state(app_handle, &state.read());

            Ok(write.reunite(read).unwrap())
        } else {
            Err(msg
                .message
                .unwrap_or_else(|| "Authentication failed".to_string()))
        }
    } else {
        Err("Unexpected auth response".to_string())
    }
}

async fn handle_connection<R: Runtime>(
    ws_stream: WsStream,
    state: &Arc<RwLock<ConnectionState>>,
    app_handle: &AppHandle<R>,
    command_rx: &mut mpsc::Receiver<OutgoingMessage>,
    shutdown_rx: &mut mpsc::Receiver<()>,
) -> bool {
    let (mut write, mut read) = ws_stream.split();

    // Bootstrap the e-stop mirror so a controller joining mid-session
    // immediately picks up the latched state. The subscribe registers
    // us for future toggle broadcasts on the 'estop' topic; the
    // get_estop_state management call returns the current value right
    // away without waiting for the next toggle. Failures here are
    // non-fatal — worst case the controller doesn't know the system
    // is safed and the server's routing gate (added in the same
    // change) still blocks motor commands.
    let estop_subscribe = OutgoingMessage::subscribe(&["estop"]);
    if let Err(e) = write.send(Message::Text(estop_subscribe.to_json())).await {
        tracing::warn!("Failed to subscribe to estop topic: {}", e);
    }
    let estop_query = OutgoingMessage::get_estop_state();
    if let Err(e) = write.send(Message::Text(estop_query.to_json())).await {
        tracing::warn!("Failed to query initial estop state: {}", e);
    }

    loop {
        tokio::select! {
            msg = read.next() => {
                match msg {
                    Some(Ok(Message::Text(text))) => {
                        // Log all incoming messages at info level for debugging
                        tracing::info!("Received from server: {}", text);
                        if let Ok(incoming) = IncomingMessage::from_json(&text) {
                            tracing::debug!("Parsed message: type={}, status={:?}", incoming.msg_type, incoming.status);

                            // Forward discovery responses to the frontend
                            if incoming.status.as_deref() == Some("ok") {
                                if let Some(ref data) = incoming.data {
                                    // Check for controllable functions response
                                    if data.get("controllable").is_some() {
                                        tracing::info!("Emitting discovery-controllable event to frontend");
                                        if let Err(e) = app_handle.emit("discovery-controllable", data) {
                                            tracing::error!("Failed to emit discovery-controllable: {}", e);
                                        }
                                    }
                                    // Check for roles response
                                    if data.get("roles").is_some() {
                                        tracing::info!("Emitting discovery-roles event to frontend");
                                        if let Err(e) = app_handle.emit("discovery-roles", data) {
                                            tracing::error!("Failed to emit discovery-roles: {}", e);
                                        }
                                    }
                                    // Check for active roles response
                                    if data.get("active_roles").is_some() {
                                        tracing::info!("Emitting discovery-active-roles event to frontend");
                                        if let Err(e) = app_handle.emit("discovery-active-roles", data) {
                                            tracing::error!("Failed to emit discovery-active-roles: {}", e);
                                        }
                                    }
                                    // Check for topic-channels response (legacy bindings picker).
                                    if data.get("topics").is_some() {
                                        tracing::info!("Emitting discovery-topic-channels event to frontend");
                                        if let Err(e) = app_handle.emit("discovery-topic-channels", data) {
                                            tracing::error!("Failed to emit discovery-topic-channels: {}", e);
                                        }
                                    }
                                    // Check for WS-input response (new bindings picker).
                                    if data.get("ws_inputs").is_some() {
                                        tracing::info!("Emitting discovery-ws-inputs event to frontend");
                                        if let Err(e) = app_handle.emit("discovery-ws-inputs", data) {
                                            tracing::error!("Failed to emit discovery-ws-inputs: {}", e);
                                        }
                                    }
                                    // Response to our get_estop_state bootstrap.
                                    // `data` shape: { "active": bool, "changed_at": float }.
                                    // The state-topic broadcast handler below
                                    // catches subsequent toggles; this catches
                                    // the initial value so a freshly-connected
                                    // controller knows the latched state
                                    // without waiting for the next toggle.
                                    if let Some(active) = data.get("active").and_then(|v| v.as_bool()) {
                                        update_estop_state(state, app_handle, active);
                                    }
                                }
                            }

                            // State-topic broadcast handler. The server
                            // broadcasts on 'estop' with shape:
                            //   { type: "state", node: "estop", data: { active, changed_at, node_count } }
                            if incoming.msg_type == "state"
                                && incoming.node.as_deref() == Some("estop")
                            {
                                if let Some(active) = incoming
                                    .data
                                    .as_ref()
                                    .and_then(|d| d.get("active"))
                                    .and_then(|v| v.as_bool())
                                {
                                    update_estop_state(state, app_handle, active);
                                }
                            }
                        } else {
                            tracing::warn!("Failed to parse incoming message");
                        }
                    }
                    Some(Ok(Message::Close(_))) | None => {
                        tracing::info!("Connection closed by server");
                        {
                            let mut s = state.write();
                            s.status = ConnectionStatus::Disconnected;
                        }
                        emit_state(app_handle, &state.read());
                        return true;
                    }
                    Some(Ok(Message::Ping(data))) => {
                        if let Err(e) = write.send(Message::Pong(data)).await {
                            tracing::error!("Failed to send pong: {}", e);
                            return true;
                        }
                    }
                    Some(Err(e)) => {
                        tracing::error!("WebSocket error: {}", e);
                        {
                            let mut s = state.write();
                            s.status = ConnectionStatus::Error;
                            s.error = Some(e.to_string());
                        }
                        emit_state(app_handle, &state.read());
                        return true;
                    }
                    _ => {}
                }
            }

            cmd = command_rx.recv() => {
                if let Some(cmd) = cmd {
                    let json = cmd.to_json();
                    tracing::info!("Sending to server: {}", json);
                    if let Err(e) = write.send(Message::Text(json)).await {
                        tracing::error!("Failed to send command: {}", e);
                        return true;
                    }
                    tracing::debug!("Command sent successfully");
                }
            }

            _ = shutdown_rx.recv() => {
                tracing::info!("Shutdown requested");
                let _ = write.send(Message::Close(None)).await;
                return false;
            }
        }
    }
}

/// Mirror an authoritative server-side e-stop value into our local
/// `ConnectionState.estop_active` and emit a Tauri event so the
/// frontend can show the banner + skip rendering joystick output
/// while engaged. Called from both the `get_estop_state` response
/// branch (bootstrap on connect) and the `state/estop` broadcast
/// branch (subsequent toggles).
fn update_estop_state<R: Runtime>(
    state: &Arc<RwLock<ConnectionState>>,
    app_handle: &AppHandle<R>,
    active: bool,
) {
    let changed;
    {
        let mut s = state.write();
        changed = s.estop_active != active;
        s.estop_active = active;
    }
    if changed {
        tracing::warn!(
            "E-STOP state from server: {}",
            if active { "ENGAGED" } else { "RELEASED" }
        );
    }
    if let Err(e) = app_handle.emit("estop-state", serde_json::json!({ "active": active })) {
        tracing::error!("Failed to emit estop-state event: {}", e);
    }
}

fn emit_state<R: Runtime>(app_handle: &AppHandle<R>, state: &ConnectionState) {
    if let Err(e) = app_handle.emit("connection-status", state) {
        tracing::error!("Failed to emit connection state: {}", e);
    }
}

impl Default for WebSocketClient {
    fn default() -> Self {
        Self::new()
    }
}
