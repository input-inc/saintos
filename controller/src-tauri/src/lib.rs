mod bindings;
mod commands;
mod input;
mod protocol;

use commands::AppState;
use tauri::Manager;
use tauri_plugin_log::{Target, TargetKind, RotationStrategy};

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
            let state = AppState::new();

            // Start input polling
            let app_handle = app.handle().clone();
            state.input_manager.start(app_handle, 16); // 60Hz polling

            app.manage(state);

            // Log the log file location
            if let Some(log_dir) = app.path().app_log_dir().ok() {
                log::info!("Log files location: {:?}", log_dir);
            }

            // Open devtools in debug mode
            #[cfg(debug_assertions)]
            {
                if let Some(window) = app.get_webview_window("main") {
                    window.open_devtools();
                }
            }

            // Set default zoom level for Steam Deck (1280x800 display)
            // The frontend will override this with saved user preference
            #[cfg(target_os = "linux")]
            {
                if let Some(window) = app.get_webview_window("main") {
                    // Check if we're likely on Steam Deck by looking for gaming mode
                    let is_steam_deck = std::env::var("GAMESCOPE_WAYLAND_DISPLAY").is_ok()
                        || std::env::var("SteamDeck").is_ok();

                    if is_steam_deck {
                        log::info!("Steam Deck detected, setting default zoom to 125%");
                        let _ = window.set_zoom(1.25);
                    }
                }
            }

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
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
