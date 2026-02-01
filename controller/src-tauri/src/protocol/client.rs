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
    }

    /// Legacy command using node_id + pin_id (deprecated)
    pub fn send_command(&self, node_id: &str, pin_id: u32, value: Value) -> Result<(), String> {
        // Throttle commands
        {
            let mut last_time = self.last_command_time.write();
            let elapsed = last_time.elapsed();
            if elapsed < Duration::from_millis(THROTTLE_MS) {
                return Ok(());
            }
            *last_time = Instant::now();
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
