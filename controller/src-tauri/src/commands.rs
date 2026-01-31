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

/// Send a command to the server
#[tauri::command]
pub fn send_command(
    state: State<'_, AppState>,
    node_id: String,
    pin_id: u32,
    value: Value,
) -> Result<(), String> {
    state.ws_client.send_command(&node_id, pin_id, value)
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
            // Send servo positions
            for position in &servo_data.positions {
                state.ws_client.send_command(
                    &position.node_id,
                    position.pin_id,
                    serde_json::Value::from(position.value),
                )?;
            }
        }
        crate::bindings::config::PresetData::Animation(anim_data) => {
            // TODO: Implement animation playback
            log::info!("Playing animation preset: {} ({} keyframes)", preset_id, anim_data.keyframes.len());
        }
        crate::bindings::config::PresetData::Sound(sound_data) => {
            // Send sound play command
            state.ws_client.send_command(
                "sound",
                0,
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

/// Show the virtual keyboard (Steam Deck / SteamOS)
/// Works in both Gaming Mode and Desktop Mode by invoking Steam directly
#[tauri::command]
pub fn show_keyboard() -> Result<(), String> {
    log::info!("show_keyboard command invoked");

    #[cfg(target_os = "linux")]
    {
        use std::process::Command;

        log::info!("Platform: Linux - opening Steam keyboard");

        // Open the keyboard via steam:// URL - run through shell for proper environment
        log::info!("Trying: steam steam://open/keyboard (via shell)");
        let result = Command::new("sh")
            .args(["-c", "steam steam://open/keyboard"])
            .spawn();

        match result {
            Ok(_) => {
                log::info!("Steam keyboard command sent via shell");
                // Don't return yet - this may not work in Desktop Mode
            }
            Err(e) => {
                log::warn!("Failed to execute steam command via shell: {}", e);
            }
        }

        // Also try xdg-open as fallback (might work differently)
        log::info!("Also trying: xdg-open steam://open/keyboard");
        let xdg_result = Command::new("xdg-open")
            .arg("steam://open/keyboard")
            .spawn();

        match xdg_result {
            Ok(_) => {
                log::info!("xdg-open keyboard command sent");
            }
            Err(e) => {
                log::warn!("Failed to execute xdg-open: {}", e);
            }
        }

        // Fallback: DBus call to Steam (works in Gaming Mode)
        log::info!("Fallback: dbus-send to Steam");
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
                    "dbus-send Steam executed - status: {:?}, stderr: {}",
                    output.status,
                    String::from_utf8_lossy(&output.stderr)
                );
                if output.status.success() {
                    log::info!("Steam keyboard opened via dbus-send");
                    return Ok(());
                }
            }
            Err(e) => {
                log::warn!("Failed to execute dbus-send to Steam: {}", e);
            }
        }

        log::warn!("All keyboard open methods failed");
        Ok(())
    }

    #[cfg(not(target_os = "linux"))]
    {
        log::info!("show_keyboard called on non-Linux platform (no-op)");
        Ok(())
    }
}

/// Hide the virtual keyboard (Steam Deck / SteamOS)
/// Works in both Gaming Mode and Desktop Mode by invoking Steam directly
#[tauri::command]
pub fn hide_keyboard() -> Result<(), String> {
    log::info!("hide_keyboard command invoked");

    #[cfg(target_os = "linux")]
    {
        use std::process::Command;

        // Close the keyboard via steam:// URL
        log::info!("Trying: steam steam://close/keyboard");
        let result = Command::new("steam")
            .arg("steam://close/keyboard")
            .spawn();

        match result {
            Ok(_) => {
                log::info!("Steam keyboard close command sent successfully");
            }
            Err(e) => {
                log::warn!("Failed to execute steam close command: {}", e);
            }
        }

        Ok(())
    }

    #[cfg(not(target_os = "linux"))]
    {
        log::info!("hide_keyboard called on non-Linux platform (no-op)");
        Ok(())
    }
}
