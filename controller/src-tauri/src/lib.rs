mod bindings;
mod commands;
mod discovery;
mod input;
mod protocol;

use bindings::mapper::ActionEvent;
use commands::AppState;
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tauri::{Emitter, Manager};
use tauri_plugin_log::{Target, TargetKind, RotationStrategy};

/// Input sample + binding-process cadence. Decoupled from the WIRE send
/// rate: the WebSocket client throttles outgoing control to THROTTLE_MS
/// (protocol/client.rs), so sampling/processing faster than that does
/// NOT increase traffic to the server — it only makes the value that
/// eventually passes the throttle FRESHER (sampled ~4 ms ago instead of
/// up to ~16 ms ago) and detects stick/button changes sooner. Cheap to
/// run at this rate now that process() is ~3 µs/frame (see
/// bindings/mapper.rs). Effective freshness is ultimately bounded by the
/// gamepad's HID report rate; the Steam Deck reports at ~250 Hz, so 4 ms
/// matches the hardware without busy-spinning past it.
const INPUT_LOOP_MS: u64 = 4; // 250 Hz sample + process

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_shell::init())
        .plugin(tauri_plugin_process::init())
        .plugin(
            tauri_plugin_log::Builder::new()
                .targets([
                    Target::new(TargetKind::Stdout),
                    Target::new(TargetKind::LogDir { file_name: Some("saint-controller".into()) }),
                    Target::new(TargetKind::Webview),  // Forward Rust logs to browser console
                ])
                .level(log::LevelFilter::Debug)
                .rotation_strategy(RotationStrategy::KeepAll)  // Don't truncate, keep appending
                .build(),
        )
        .setup(|app| {
            let state = Arc::new(AppState::new());

            // Start input polling (emits input-state events to frontend for UI).
            // Sampled fast (INPUT_LOOP_MS); the wire send rate is gated
            // separately by the WS client throttle — see INPUT_LOOP_MS.
            let app_handle = app.handle().clone();
            state.input_manager.start(app_handle.clone(), INPUT_LOOP_MS);

            // Start input processing loop (processes bindings and sends commands)
            let state_for_processing = state.clone();
            thread::spawn(move || {
                log::info!("Input processing thread started");
                loop {
                    // Get current input state
                    let input_state = state_for_processing.input_manager.get_state();

                    // Process through mapper to generate action events
                    let events = {
                        let mut mapper = state_for_processing.mapper.write();
                        mapper.process(&input_state)
                    };

                    // Handle each action event
                    for event in events {
                        match event {
                            ActionEvent::Command(cmd) => {
                                use bindings::mapper::MappedCommand;
                                let result = match cmd {
                                    MappedCommand::Topic { topic, channel, value } =>
                                        state_for_processing.ws_client.send_topic_channel_value(
                                            &topic, &channel, value),
                                    MappedCommand::WsInput { sheet_id, input_id, value } =>
                                        state_for_processing.ws_client.send_ws_input_value(
                                            &sheet_id, &input_id, value),
                                };
                                if let Err(e) = result {
                                    if !e.contains("Not connected") {
                                        log::error!("Failed to send command: {}", e);
                                    }
                                }
                            }
                            ActionEvent::EStop => {
                                log::warn!("E-STOP triggered!");
                                let _ = state_for_processing.ws_client.send_emergency_stop();
                            }
                            // UI events are handled by frontend via action-event emission
                            _ => {
                                if let Err(e) = app_handle.emit("action-event", &event) {
                                    log::error!("Failed to emit action event: {}", e);
                                }
                            }
                        }
                    }

                    // Process at the sample cadence so a stick/button change
                    // is turned into a (throttled) send within INPUT_LOOP_MS,
                    // not up to a 60 Hz frame later. The WS throttle still
                    // caps what actually reaches the server.
                    thread::sleep(Duration::from_millis(INPUT_LOOP_MS));
                }
            });

            // Use Arc for state management
            app.manage(state);

            // Log the log file location
            if let Ok(log_dir) = app.path().app_log_dir() {
                log::info!("Log files location: {:?}", log_dir);
            }

            // Open devtools in debug mode
            #[cfg(debug_assertions)]
            {
                if let Some(window) = app.get_webview_window("main") {
                    window.open_devtools();
                }
            }

            // Zoom is handled entirely by the frontend via localStorage
            // This avoids double-application of zoom on startup

            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            commands::connect,
            commands::disconnect,
            commands::get_connection_status,
            commands::get_input_state,
            commands::get_binding_profiles,
            commands::set_binding_profiles,
            commands::set_active_profile,
            commands::send_command,
            commands::send_function_control,
            commands::discover_roles,
            commands::discover_controllable,
            commands::discover_topic_channels,
            commands::send_topic_channel_value,
            commands::discover_ws_inputs,
            commands::send_ws_input_value,
            commands::get_adopted_nodes,
            commands::subscribe_topics,
            commands::list_animations,
            commands::list_poses,
            commands::start_animation,
            commands::apply_pose,
            commands::is_gamepad_connected,
            commands::emergency_stop,
            commands::get_gamepad_debug_info,
            commands::activate_preset,
            commands::open_devtools,
            commands::close_devtools,
            commands::is_devtools_open,
            commands::set_zoom,
            commands::get_zoom,
            commands::quit_app,
            commands::log_frontend,
            commands::show_keyboard,
            commands::hide_keyboard,
            commands::discover_servers,
            commands::resolve_host,
            commands::install_controller_update,
            commands::get_installed_controller_info,
            commands::get_build_info,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
