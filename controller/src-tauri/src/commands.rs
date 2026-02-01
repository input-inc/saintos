use crate::bindings::{BindingProfile, InputMapper};
use crate::input::InputManager;
use crate::protocol::{ConnectionState, WebSocketClient};
use parking_lot::RwLock;
use serde_json::Value;
use tauri::{AppHandle, Runtime, State, WebviewWindow};

pub struct AppState {
    pub input_manager: InputManager,
    pub ws_client: WebSocketClient,
    pub mapper: RwLock<InputMapper>,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            input_manager: InputManager::new(),
            ws_client: WebSocketClient::new(),
            mapper: RwLock::new(InputMapper::new()),
        }
    }
}

impl Default for AppState {
    fn default() -> Self {
        Self::new()
    }
}

/// Connect to the SAINT.OS server
#[tauri::command]
pub fn connect<R: Runtime + 'static>(
    app_handle: AppHandle<R>,
    state: State<'_, AppState>,
    host: String,
    port: u16,
    password: String,
) -> Result<(), String> {
    state.ws_client.connect(app_handle, &host, port, &password)
}

/// Disconnect from the server
#[tauri::command]
pub fn disconnect(state: State<'_, AppState>) -> Result<(), String> {
    state.ws_client.disconnect();
    Ok(())
}

/// Get current connection status
#[tauri::command]
pub fn get_connection_status(state: State<'_, AppState>) -> ConnectionState {
    state.ws_client.state()
}

/// Get current input state (gamepad + gyro)
#[tauri::command]
pub fn get_input_state(state: State<'_, AppState>) -> crate::input::manager::InputState {
    state.input_manager.get_state()
}

/// Get all binding profiles
#[tauri::command]
pub fn get_binding_profiles(state: State<'_, AppState>) -> Vec<BindingProfile> {
    state.mapper.read().get_profiles().to_vec()
}

/// Set binding profiles
#[tauri::command]
pub fn set_binding_profiles(state: State<'_, AppState>, profiles: Vec<BindingProfile>) {
    state.mapper.write().set_profiles(profiles);
}

/// Set active binding profile
#[tauri::command]
pub fn set_active_profile(state: State<'_, AppState>, profile_id: String) {
    state.mapper.write().set_active_profile(&profile_id);
}

/// Send a command to the server (legacy - uses node_id + pin_id)
#[tauri::command]
pub fn send_command(
    state: State<'_, AppState>,
    node_id: String,
    pin_id: u32,
    value: Value,
) -> Result<(), String> {
    state.ws_client.send_command(&node_id, pin_id, value)
}

/// Send a function control command (preferred - uses role + function)
#[tauri::command]
pub fn send_function_control(
    state: State<'_, AppState>,
    role: String,
    function: String,
    value: Value,
) -> Result<(), String> {
    state.ws_client.send_function_control(&role, &function, value)
}

/// Request discovery of available roles from the server
#[tauri::command]
pub fn discover_roles(state: State<'_, AppState>) -> Result<(), String> {
    state.ws_client.request_discover_roles()
}

/// Request discovery of controllable functions from the server
#[tauri::command]
pub fn discover_controllable(state: State<'_, AppState>) -> Result<(), String> {
    state.ws_client.request_discover_controllable()
}

/// Check if a gamepad is connected
#[tauri::command]
pub fn is_gamepad_connected(state: State<'_, AppState>) -> bool {
    state.input_manager.is_gamepad_connected()
}

/// Send emergency stop command
#[tauri::command]
pub fn emergency_stop(state: State<'_, AppState>) -> Result<(), String> {
    state.ws_client.send_emergency_stop()
}

/// Activate a preset by ID
#[tauri::command]
pub fn activate_preset(state: State<'_, AppState>, preset_id: String) -> Result<(), String> {
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
            // Send servo positions using role/function abstraction
            for position in &servo_data.positions {
                state.ws_client.send_function_control(
                    &position.role,
                    &position.function,
                    serde_json::Value::from(position.value),
                )?;
            }
        }
        crate::bindings::config::PresetData::Animation(anim_data) => {
            // TODO: Implement animation playback
            log::info!("Playing animation preset: {} ({} keyframes)", preset_id, anim_data.keyframes.len());
        }
        crate::bindings::config::PresetData::Sound(sound_data) => {
            // Send sound play command using role/function abstraction
            state.ws_client.send_function_control(
                "sound",
                "play",
                serde_json::json!({
                    "soundId": sound_data.sound_id,
                    "volume": sound_data.volume,
                    "priority": sound_data.priority
                }),
            )?;
        }
    }

    Ok(())
}

/// Get gamepad debug info
#[tauri::command]
pub fn get_gamepad_debug_info() -> String {
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
#[tauri::command]
pub fn get_zoom(window: WebviewWindow) -> Result<f64, String> {
    window.zoom().map_err(|e| format!("Failed to get zoom: {}", e))
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
