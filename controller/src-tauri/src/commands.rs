use crate::bindings::{BindingProfile, InputMapper};
use crate::discovery::{DiscoveredServer, DiscoveryService};
use crate::input::InputManager;
use crate::protocol::{ConnectionState, WebSocketClient};
use parking_lot::RwLock;
use serde_json::Value;
use std::sync::Arc;
use std::time::Duration;
use tauri::{AppHandle, Runtime, State, WebviewWindow};

pub struct AppState {
    pub input_manager: InputManager,
    pub ws_client: WebSocketClient,
    pub mapper: RwLock<InputMapper>,
    /// mDNS discovery / resolution helper. Optional because the daemon
    /// can fail to bind its UDP socket on locked-down hosts (the
    /// Steam Deck Game Mode sandbox notably) — in that case we still
    /// want the rest of the app to work, just without auto-discovery.
    pub discovery: Option<DiscoveryService>,
}

impl AppState {
    pub fn new() -> Self {
        // Best-effort start. If mDNS doesn't come up we log it and
        // keep going; the operator can still connect by typing an IP
        // address manually.
        let discovery = match DiscoveryService::start() {
            Ok(d) => Some(d),
            Err(e) => {
                log::warn!("mDNS discovery disabled: {}", e);
                None
            }
        };
        Self {
            input_manager: InputManager::new(),
            ws_client: WebSocketClient::new(),
            mapper: RwLock::new(InputMapper::new()),
            discovery,
        }
    }
}

impl Default for AppState {
    fn default() -> Self {
        Self::new()
    }
}

/// Connect to the SAINT.OS server.
///
/// If `host` is a `.local` hostname, we try to resolve it through our
/// own embedded mDNS client BEFORE handing it to the WebSocket library.
/// `tokio-tungstenite` calls getaddrinfo internally, and on Steam Deck
/// (and many other Linux setups) getaddrinfo doesn't know how to
/// resolve `.local` — so resolving in-process is the difference
/// between "Connect" working and the operator having to type an IP.
/// Non-`.local` hosts (IPs, normal DNS names) skip the resolution and
/// flow straight through to the existing connect path.
#[tauri::command]
pub fn connect<R: Runtime + 'static>(
    app_handle: AppHandle<R>,
    state: State<'_, Arc<AppState>>,
    host: String,
    port: u16,
    password: String,
) -> Result<(), String> {
    let effective_host = maybe_resolve_local(&state, &host);
    state
        .ws_client
        .connect(app_handle, &effective_host, port, &password)
}

/// Return the bare-IP form of `host` if it's a `.local` name that
/// resolves through the mDNS client, else `host` unchanged. Two-second
/// timeout — long enough for the multicast round-trip on a quiet LAN,
/// short enough that a wrong hostname surfaces a connect error
/// promptly instead of looking like a hang.
fn maybe_resolve_local(state: &State<'_, Arc<AppState>>, host: &str) -> String {
    if !host.ends_with(".local") && !host.ends_with(".local.") {
        return host.to_string();
    }
    let Some(d) = state.discovery.as_ref() else {
        log::warn!("Host '{}' looks like mDNS but discovery is disabled", host);
        return host.to_string();
    };
    match d.resolve(host, Duration::from_secs(2)) {
        Some(ip) => {
            log::info!("Resolved '{}' → {} via embedded mDNS", host, ip);
            ip.to_string()
        }
        None => {
            // Let the existing connect path try anyway — it'll fail
            // with a clear error in the dashboard. We don't want to
            // pre-empt the connect attempt with our own error here
            // because some hosts (Linux desktops with nss-mdns) will
            // resolve through the OS even though mDNS didn't answer
            // our query in time.
            log::warn!("mDNS resolve for '{}' returned no answer", host);
            host.to_string()
        }
    }
}

/// Disconnect from the server
#[tauri::command]
pub fn disconnect(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.disconnect();
    Ok(())
}

/// Get current connection status
#[tauri::command]
pub fn get_connection_status(state: State<'_, Arc<AppState>>) -> ConnectionState {
    state.ws_client.state()
}

/// Get current input state (gamepad + gyro)
#[tauri::command]
pub fn get_input_state(state: State<'_, Arc<AppState>>) -> crate::input::manager::InputState {
    state.input_manager.get_state()
}

/// Get all binding profiles
#[tauri::command]
pub fn get_binding_profiles(state: State<'_, Arc<AppState>>) -> Vec<BindingProfile> {
    state.mapper.read().get_profiles().to_vec()
}

/// Set binding profiles
#[tauri::command]
pub fn set_binding_profiles(state: State<'_, Arc<AppState>>, profiles: Vec<BindingProfile>) {
    state.mapper.write().set_profiles(profiles);
}

/// Set active binding profile
#[tauri::command]
pub fn set_active_profile(state: State<'_, Arc<AppState>>, profile_id: String) {
    state.mapper.write().set_active_profile(&profile_id);
}

/// Send a command to the server (legacy - uses node_id + pin_id)
#[tauri::command]
pub fn send_command(
    state: State<'_, Arc<AppState>>,
    node_id: String,
    pin_id: u32,
    value: Value,
) -> Result<(), String> {
    state.ws_client.send_command(&node_id, pin_id, value)
}

/// Send a function control command (preferred - uses role + function)
#[tauri::command]
pub fn send_function_control(
    state: State<'_, Arc<AppState>>,
    role: String,
    function: String,
    value: Value,
) -> Result<(), String> {
    state.ws_client.send_function_control(&role, &function, value)
}

/// Request discovery of available roles from the server
#[tauri::command]
pub fn discover_roles(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.request_discover_roles()
}

/// Request discovery of controllable functions from the server
#[tauri::command]
pub fn discover_controllable(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.request_discover_controllable()
}

/// Request enumeration of ROS topics + their scalar channels. Result
/// is delivered out-of-band via the `discovery-topic-channels` Tauri
/// event for the bindings UI.
#[tauri::command]
pub fn discover_topic_channels(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.request_discover_topic_channels()
}

/// Push a single scalar value onto a ROS topic channel via the server.
/// Replaces send_function_control for bindings authored against the new
/// topic/channel picker.
#[tauri::command]
pub fn send_topic_channel_value(
    state: State<'_, Arc<AppState>>,
    topic: String,
    channel: String,
    value: serde_json::Value,
) -> Result<(), String> {
    state.ws_client.send_topic_channel_value(&topic, &channel, value)
}

/// Request enumeration of WS-input slots across all routing sheets.
/// Result is delivered on the `discovery-ws-inputs` Tauri event.
#[tauri::command]
pub fn discover_ws_inputs(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.request_discover_ws_inputs()
}

/// Push a scalar value into a sheet's WS-input slot. The bindings
/// runtime calls this on every gamepad-axis tick that maps to a WS
/// input — the value flows straight into the routing evaluator.
#[tauri::command]
pub fn send_ws_input_value(
    state: State<'_, Arc<AppState>>,
    sheet_id: String,
    input_id: String,
    value: serde_json::Value,
) -> Result<(), String> {
    state.ws_client.send_ws_input_value(&sheet_id, &input_id, value)
}

/// Ask the server for the adopted-node list. The response is delivered
/// out-of-band on the `adopted-nodes` Tauri event; the battery panel
/// uses it to decide which `pin_state/<node>` topics to subscribe to.
#[tauri::command]
pub fn get_adopted_nodes(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.request_adopted_nodes()
}

/// Subscribe to a set of state topics (e.g. `pin_state/<node>` per
/// node). Matching broadcasts arrive on the `pin-state` Tauri event.
#[tauri::command]
pub fn subscribe_topics(
    state: State<'_, Arc<AppState>>,
    topics: Vec<String>,
) -> Result<(), String> {
    state.ws_client.subscribe_topics(topics)
}

/// Ask for the server AP's WiFi config (SSID, band, channel). Result
/// arrives on the `wifi-config` Tauri event, password stripped.
#[tauri::command]
pub fn get_wifi_config(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.request_wifi_config()
}

/// Run a channel-congestion survey on the server (~10 s). Result
/// arrives on the `wifi-survey` Tauri event.
#[tauri::command]
pub fn wifi_survey(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.request_wifi_survey()
}

/// Move the server's AP to a new channel. `band` is the wire encoding
/// ("bg" = 2.4 GHz, "a" = 5 GHz). The connection drops ~5-10 s while
/// the AP restarts; the ACK arrives first on `wifi-switching`.
#[tauri::command]
pub fn set_wifi_channel(
    state: State<'_, Arc<AppState>>,
    band: String,
    channel: u32,
) -> Result<(), String> {
    state.ws_client.request_wifi_set_channel(&band, channel)
}

/// Request the saved-animation list. Result arrives on the
/// `library-animations` Tauri event (see useLibrary).
#[tauri::command]
pub fn list_animations(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.request_list_animations()
}

/// Request the saved-pose list. Result arrives on `library-poses`.
#[tauri::command]
pub fn list_poses(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.request_list_poses()
}

/// Play a saved animation by id.
#[tauri::command]
pub fn start_animation(state: State<'_, Arc<AppState>>, id: String) -> Result<(), String> {
    state.ws_client.start_animation(&id)
}

/// Apply a saved pose by id.
#[tauri::command]
pub fn apply_pose(state: State<'_, Arc<AppState>>, id: String) -> Result<(), String> {
    state.ws_client.apply_pose(&id)
}

/// Check if a gamepad is connected
#[tauri::command]
pub fn is_gamepad_connected(state: State<'_, Arc<AppState>>) -> bool {
    state.input_manager.is_gamepad_connected()
}

/// Send emergency stop command
#[tauri::command]
pub fn emergency_stop(state: State<'_, Arc<AppState>>) -> Result<(), String> {
    state.ws_client.send_emergency_stop()
}

/// Activate a preset by ID
#[tauri::command]
pub fn activate_preset(state: State<'_, Arc<AppState>>, preset_id: String) -> Result<(), String> {
    let mapper = state.mapper.read();
    let profile = mapper
        .active_profile()
        .ok_or("No active profile")?;

    let preset = profile
        .get_preset(&preset_id)
        .ok_or_else(|| format!("Preset not found: {}", preset_id))?;

    // Execute the preset based on its type
    match &preset.data {
        crate::bindings::config::PresetData::Servo(servo_data) => {
            // Send each servo position as a topic/channel scalar.
            for position in &servo_data.positions {
                state.ws_client.send_topic_channel_value(
                    &position.topic,
                    &position.channel,
                    serde_json::Value::from(position.value),
                )?;
            }
        }
        crate::bindings::config::PresetData::Animation(anim_data) => {
            // TODO: Implement animation playback
            log::info!("Playing animation preset: {} ({} keyframes)", preset_id, anim_data.keyframes.len());
        }
        crate::bindings::config::PresetData::Sound(sound_data) => {
            // Sound presets publish onto /sound with a structured payload.
            state.ws_client.send_topic_channel_value(
                "/sound",
                "play",
                serde_json::json!({
                    "soundId": sound_data.sound_id,
                    "volume": sound_data.volume,
                    "priority": sound_data.priority,
                }),
            )?;
        }
    }

    Ok(())
}

/// Get gamepad debug info
#[tauri::command]
pub fn get_gamepad_debug_info() -> String {
    // Only the macOS branch shells out (ioreg); gating the import keeps
    // the Linux build from warning about an unused import.
    #[cfg(target_os = "macos")]
    use std::process::Command;

    let mut info = String::new();

    // Try to get USB device info on macOS
    #[cfg(target_os = "macos")]
    {
        info.push_str("=== macOS Gamepad Debug Info ===\n\n");

        // Check for HID devices
        if let Ok(output) = Command::new("ioreg")
            .args(["-p", "IOUSB", "-l", "-w", "0"])
            .output()
        {
            let stdout = String::from_utf8_lossy(&output.stdout);
            if stdout.contains("Xbox") || stdout.contains("045e") {
                info.push_str("Xbox controller found in IORegistry (USB)\n");
            } else {
                info.push_str("No Xbox controller found in USB IORegistry\n");
            }
        }

        // Check HID devices
        if let Ok(output) = Command::new("ioreg")
            .args(["-p", "IOService", "-n", "IOHIDDevice", "-r"])
            .output()
        {
            let stdout = String::from_utf8_lossy(&output.stdout);
            let hid_count = stdout.matches("IOHIDDevice").count();
            info.push_str(&format!("HID devices found: {}\n", hid_count));
        }

        info.push_str("\nNote: Xbox controllers on macOS may require:\n");
        info.push_str("1. macOS 11+ for native Game Controller support\n");
        info.push_str("2. The controller to be in the correct mode\n");
        info.push_str("3. Bluetooth pairing for wireless controllers\n");
    }

    #[cfg(not(target_os = "macos"))]
    {
        info.push_str("Debug info not available for this platform\n");
    }

    info
}

/// Open the web inspector/devtools
#[tauri::command]
pub fn open_devtools(window: WebviewWindow) {
    window.open_devtools();
}

/// Close the web inspector/devtools
#[tauri::command]
pub fn close_devtools(window: WebviewWindow) {
    window.close_devtools();
}

/// Check if devtools are open
#[tauri::command]
pub fn is_devtools_open(window: WebviewWindow) -> bool {
    window.is_devtools_open()
}

/// Set the webview zoom level (1.0 = 100%, 1.5 = 150%, etc.)
#[tauri::command]
pub fn set_zoom(window: WebviewWindow, scale: f64) -> Result<(), String> {
    log::info!("Setting zoom level to {}%", scale * 100.0);
    window.set_zoom(scale).map_err(|e| format!("Failed to set zoom: {}", e))
}

/// Get the current webview zoom level
/// Note: Tauri 2.x doesn't provide a getter, so we track it in localStorage on the frontend
#[tauri::command]
pub fn get_zoom() -> Result<f64, String> {
    // Tauri 2.x WebviewWindow doesn't have a zoom() getter
    // Return default; frontend tracks actual value in localStorage
    Ok(1.0)
}

/// Quit the application
#[tauri::command]
pub fn quit_app<R: Runtime>(app_handle: AppHandle<R>) {
    log::info!("Application quit requested");
    app_handle.exit(0);
}

/// Log a message from the frontend
#[tauri::command]
pub fn log_frontend(level: String, message: String, context: Option<String>) {
    let ctx = context.unwrap_or_default();
    match level.as_str() {
        "error" => log::error!(target: "frontend", "[{}] {}", ctx, message),
        "warn" => log::warn!(target: "frontend", "[{}] {}", ctx, message),
        "info" => log::info!(target: "frontend", "[{}] {}", ctx, message),
        "debug" => log::debug!(target: "frontend", "[{}] {}", ctx, message),
        _ => log::trace!(target: "frontend", "[{}] {}", ctx, message),
    }
}

/// Check if we're running in SteamOS Gaming Mode (gamescope) or Desktop Mode (KDE)
#[cfg(target_os = "linux")]
fn is_gaming_mode() -> bool {
    // Check for gamescope (Gaming Mode compositor)
    if std::env::var("GAMESCOPE_WAYLAND_DISPLAY").is_ok() {
        return true;
    }
    // Check XDG_CURRENT_DESKTOP - Gaming Mode doesn't set this, Desktop Mode sets it to KDE
    if let Ok(desktop) = std::env::var("XDG_CURRENT_DESKTOP") {
        if desktop.contains("KDE") || desktop.contains("plasma") || desktop.contains("GNOME") {
            return false;
        }
    }
    // Default to Gaming Mode if we can't determine
    true
}

/// Show the virtual keyboard (Steam Deck / SteamOS)
/// Uses Steam's keyboard in Gaming Mode, CoreKeyboard in Desktop Mode
#[tauri::command]
pub fn show_keyboard() -> Result<(), String> {
    log::info!("show_keyboard command invoked");

    #[cfg(target_os = "linux")]
    {
        use std::process::Command;

        let gaming_mode = is_gaming_mode();
        log::info!("Detected mode: {}", if gaming_mode { "Gaming Mode" } else { "Desktop Mode" });

        if gaming_mode {
            // Gaming Mode: Use Steam's keyboard via DBus
            log::info!("Gaming Mode: Using Steam keyboard via DBus");
            let dbus_result = Command::new("dbus-send")
                .args([
                    "--type=method_call",
                    "--dest=com.steampowered.Steamclient",
                    "/com/steampowered/Steamclient",
                    "com.steampowered.Steamclient.ShowFloatingGamepadTextInput",
                    "int32:0",  // keyboard mode (0 = normal)
                    "int32:0",  // x position
                    "int32:0",  // y position
                    "int32:800", // width
                    "int32:400", // height
                ])
                .output();

            match dbus_result {
                Ok(output) => {
                    log::info!(
                        "Steam keyboard DBus call - status: {:?}",
                        output.status
                    );
                    if output.status.success() {
                        return Ok(());
                    }
                }
                Err(e) => {
                    log::warn!("Failed to call Steam keyboard: {}", e);
                }
            }

            // Fallback: steam:// URL
            log::info!("Fallback: steam steam://open/keyboard");
            let _ = Command::new("steam")
                .arg("steam://open/keyboard")
                .spawn();
        } else {
            // Desktop Mode: Use CoreKeyboard (flatpak) as primary option
            log::info!("Desktop Mode: Trying CoreKeyboard");

            // Try CoreKeyboard (flatpak)
            let corekeyboard_result = Command::new("flatpak")
                .args(["run", "org.nickvision.keyboard"])
                .spawn();

            match corekeyboard_result {
                Ok(_) => {
                    log::info!("CoreKeyboard launched successfully");
                    return Ok(());
                }
                Err(e) => {
                    log::warn!("CoreKeyboard not available: {}", e);
                }
            }

            // Fallback: Try maliit-keyboard
            log::info!("Fallback: Trying maliit-keyboard");
            let maliit_result = Command::new("maliit-keyboard")
                .spawn();

            match maliit_result {
                Ok(_) => {
                    log::info!("maliit-keyboard launched");
                    return Ok(());
                }
                Err(e) => {
                    log::warn!("maliit-keyboard not available: {}", e);
                }
            }

            // Last resort: Try steam command anyway
            log::info!("Last resort: steam steam://open/keyboard");
            let _ = Command::new("steam")
                .arg("steam://open/keyboard")
                .spawn();
        }

        Ok(())
    }

    #[cfg(not(target_os = "linux"))]
    {
        log::info!("show_keyboard called on non-Linux platform (no-op)");
        Ok(())
    }
}

/// Hide the virtual keyboard (Steam Deck / SteamOS)
/// Uses appropriate method based on Gaming Mode vs Desktop Mode
#[tauri::command]
pub fn hide_keyboard() -> Result<(), String> {
    log::info!("hide_keyboard command invoked");

    #[cfg(target_os = "linux")]
    {
        use std::process::Command;

        let gaming_mode = is_gaming_mode();
        log::info!("Detected mode for hide: {}", if gaming_mode { "Gaming Mode" } else { "Desktop Mode" });

        if gaming_mode {
            // Gaming Mode: Close Steam keyboard
            log::info!("Gaming Mode: Closing Steam keyboard");
            let _ = Command::new("steam")
                .arg("steam://close/keyboard")
                .spawn();
        } else {
            // Desktop Mode: Kill CoreKeyboard or maliit-keyboard processes
            log::info!("Desktop Mode: Closing keyboard processes");
            let _ = Command::new("pkill")
                .args(["-f", "org.nickvision.keyboard"])
                .output();
            let _ = Command::new("pkill")
                .args(["-f", "maliit-keyboard"])
                .output();
        }

        Ok(())
    }

    #[cfg(not(target_os = "linux"))]
    {
        log::info!("hide_keyboard called on non-Linux platform (no-op)");
        Ok(())
    }
}

/// Snapshot of SAINT.OS servers the embedded mDNS browser has seen
/// since the controller launched. Returns an empty list (not an
/// error) when discovery is disabled — the UI treats missing results
/// the same as "found nothing" and falls back to the typed-host form.
#[tauri::command]
pub fn discover_servers(state: State<'_, Arc<AppState>>) -> Vec<DiscoveredServer> {
    match state.discovery.as_ref() {
        Some(d) => d.snapshot(),
        None => Vec::new(),
    }
}

/// Resolve a `.local` hostname through the embedded mDNS client and
/// return the dotted-IP string. Exposed for the connect form so the
/// frontend can pre-flight a hostname and show a helpful error before
/// the user clicks Connect. Returns Err on timeout or when discovery
/// is disabled.
#[tauri::command]
pub fn resolve_host(
    state: State<'_, Arc<AppState>>,
    host: String,
) -> Result<String, String> {
    let Some(d) = state.discovery.as_ref() else {
        return Err("mDNS discovery is disabled on this host".into());
    };
    match d.resolve(&host, Duration::from_secs(2)) {
        Some(ip) => Ok(ip.to_string()),
        None => Err(format!("No mDNS answer for '{}' within 2 s", host)),
    }
}

/// Pick the path on disk to write a new AppImage to.
///
/// Prefers `$APPIMAGE` (the AppImage runtime sets this to the absolute
/// path of the file that was launched) so OTA updates overwrite the
/// same file the Steam shortcut points at — the shortcut keeps working
/// across versions without us touching `shortcuts.vdf`. Falls back to a
/// canonical XDG location when not running inside an AppImage (dev
/// mode, `tauri dev`, raw binary), so first-time installs from inside
/// the app still go somewhere predictable.
fn target_appimage_path() -> Result<std::path::PathBuf, String> {
    use std::path::PathBuf;
    if let Ok(p) = std::env::var("APPIMAGE") {
        let path = PathBuf::from(p);
        if path.parent().is_some_and(|parent| parent.is_dir()) {
            return Ok(path);
        }
    }
    let data_home = dirs::data_dir()
        .ok_or_else(|| "no XDG data dir available".to_string())?;
    let install_dir = data_home.join("saint-controller");
    std::fs::create_dir_all(&install_dir)
        .map_err(|e| format!("mkdir {}: {}", install_dir.display(), e))?;
    Ok(install_dir.join("SAINT-Controller.AppImage"))
}

/// Stable XDG location for install metadata (`installed.json`).
///
/// Distinct from the AppImage's path on purpose: the binary could land
/// anywhere depending on `$APPIMAGE`, but `installed.json` always lives
/// here so update-detection can find it deterministically.
fn metadata_dir() -> Result<std::path::PathBuf, String> {
    let data_home = dirs::data_dir()
        .ok_or_else(|| "no XDG data dir available".to_string())?;
    let dir = data_home.join("saint-controller");
    std::fs::create_dir_all(&dir)
        .map_err(|e| format!("mkdir {}: {}", dir.display(), e))?;
    Ok(dir)
}

/// Install a freshly-downloaded controller .AppImage.
///
/// JS gates this behind a SHA-256 verification on the downloaded body,
/// so the bytes we receive here are trusted. `version`, `filename`,
/// and `checksum` come from the same `info.json` the bytes were
/// fetched against — they get recorded into `installed.json` so the
/// next update check can compare both fields against the server. This
/// is what catches "same version string, different content" rebuilds:
/// the version may not have changed but the checksum did, so an
/// update is still offered.
///
/// Target path resolution: see `target_appimage_path()` — prefers
/// `$APPIMAGE` so the file the Steam shortcut points at gets
/// overwritten in-place. Linux's `rename(2)` is safe to apply over a
/// busy executable; the running process keeps the old inode via its
/// open fd, while the next launch execs the new file.
///
/// Returning `Ok` does NOT mean the new version is running — the
/// operator still has to relaunch the app (Steam tile, file launcher,
/// or running the AppImage path directly) before the new code takes
/// effect.
#[tauri::command]
pub fn install_controller_update(
    bytes: Vec<u8>,
    version: String,
    filename: String,
    checksum: String,
) -> Result<(), String> {
    use std::os::unix::fs::PermissionsExt;
    use std::time::{SystemTime, UNIX_EPOCH};

    let target = target_appimage_path()?;

    // If $APPIMAGE is unset we're guessing at the install location (XDG
    // fallback). The launcher (Steam tile / .desktop) almost certainly
    // points at the ORIGINAL file elsewhere, so the bytes land where
    // nothing launches and the operator sees "installed but still old
    // version after restart". Make that loud in the log; the UI also
    // surfaces the running-vs-installed path mismatch from get_build_info.
    if std::env::var("APPIMAGE").is_err() {
        log::warn!(
            "install_controller_update: $APPIMAGE unset — writing to fallback \
             {} which the launcher may not point at; relaunch from there or \
             repoint the launcher, or the new version won't take effect.",
            target.display());
    }

    // `with_extension` would replace `.AppImage` outright; we want to
    // append, so build the staging filename manually.
    let mut staging = target.clone();
    let staging_name = format!("{}.new", target
        .file_name()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "SAINT-Controller.AppImage".to_string()));
    staging.set_file_name(staging_name);

    std::fs::write(&staging, &bytes)
        .map_err(|e| format!("write {}: {}", staging.display(), e))?;
    let mut perms = std::fs::metadata(&staging)
        .map_err(|e| format!("stat {}: {}", staging.display(), e))?
        .permissions();
    perms.set_mode(0o755);
    std::fs::set_permissions(&staging, perms)
        .map_err(|e| format!("chmod {}: {}", staging.display(), e))?;

    std::fs::rename(&staging, &target)
        .map_err(|e| format!("rename {} -> {}: {}", staging.display(), target.display(), e))?;
    log::info!(
        "Installed {} byte controller AppImage at {}",
        bytes.len(),
        target.display(),
    );

    // installed.json sidecar — version + checksum stored alongside in
    // a stable XDG location, regardless of where the binary went. The
    // update-detection logic in settings.component.ts reads this back
    // and compares both fields.
    let meta_path = metadata_dir()?.join("installed.json");
    let installed_at_unix = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let installed = serde_json::json!({
        "version": version,
        "checksum": checksum,
        "filename": filename,
        "installed_at_unix": installed_at_unix,
        "path": target.to_string_lossy(),
    });
    std::fs::write(
        &meta_path,
        serde_json::to_string_pretty(&installed).unwrap_or_else(|_| "{}".to_string()),
    )
    .map_err(|e| format!("write {}: {}", meta_path.display(), e))?;

    // .desktop entry pointing at the install path. For OTA updates
    // this just keeps the entry in sync if the path changed; for
    // first-time installs it's how GUI launchers find the app.
    let applications_dir = dirs::data_dir()
        .ok_or_else(|| "no XDG data dir available".to_string())?
        .join("applications");
    std::fs::create_dir_all(&applications_dir)
        .map_err(|e| format!("mkdir {}: {}", applications_dir.display(), e))?;
    let desktop_path = applications_dir.join("saint-controller.desktop");
    let desktop_contents = format!(
        "[Desktop Entry]\n\
         Type=Application\n\
         Name=SAINT Controller\n\
         Comment=Operator interface for SAINT.OS robots\n\
         Exec={} %U\n\
         Icon=saint-controller\n\
         Terminal=false\n\
         Categories=Utility;\n\
         StartupNotify=true\n",
        target.display()
    );
    std::fs::write(&desktop_path, desktop_contents)
        .map_err(|e| format!("write {}: {}", desktop_path.display(), e))?;

    log::info!("Operator must relaunch the app to pick up the new version");
    Ok(())
}

/// Read the `installed.json` sidecar that records what was installed
/// by the last successful `install_controller_update` call. Returns
/// `None` when the file doesn't exist — typically a fresh install
/// from a hand-downloaded AppImage where the in-app OTA never ran.
/// The caller falls back to a looser version-string comparison in
/// that case.
#[tauri::command]
pub fn get_installed_controller_info() -> Result<Option<serde_json::Value>, String> {
    let meta_path = metadata_dir()?.join("installed.json");
    if !meta_path.exists() {
        return Ok(None);
    }
    let text = std::fs::read_to_string(&meta_path)
        .map_err(|e| format!("read {}: {}", meta_path.display(), e))?;
    let json: serde_json::Value = serde_json::from_str(&text)
        .map_err(|e| format!("parse {}: {}", meta_path.display(), e))?;
    Ok(Some(json))
}

/// Return the build identity baked into this binary at compile time.
///
/// `version` is the canonical SAINT version string — same format the
/// AppImage filename uses (e.g. `0.5.0-local.d85dad7`). When the dist
/// build script (`build-bundle.sh`) drives the build, it exports
/// `SAINT_BUILD_VERSION` so the embedded string is byte-identical to
/// the AppImage filename. For raw `cargo build` / `tauri dev`, build.rs
/// reconstructs the local-channel form itself.
///
/// `built_at_unix` is the build's Unix timestamp; the UI converts it
/// to the local timezone for display.
///
/// Both fields are compile-time constants — zero runtime cost, no
/// filesystem reads, can't go stale relative to the running code.
#[tauri::command]
pub fn get_build_info() -> serde_json::Value {
    // `appimage_path` is the file THIS process was launched from (the
    // AppImage runtime exports $APPIMAGE). The UI compares it to the
    // `path` recorded in installed.json by the last OTA: if they
    // differ, the launcher (Steam tile, .desktop) is pointing at a
    // DIFFERENT file than the one the update overwrote — which is
    // exactly the "installed the new version but restart still shows
    // the old one" failure. Null means we're not running from an
    // AppImage (dev build / raw binary), where OTA self-update doesn't
    // apply anyway.
    let appimage_path = std::env::var("APPIMAGE").ok();
    serde_json::json!({
        "version": env!("SAINT_BUILD_VERSION"),
        "built_at_unix": env!("SAINT_BUILD_UNIX").parse::<u64>().unwrap_or(0),
        "appimage_path": appimage_path,
    })
}
