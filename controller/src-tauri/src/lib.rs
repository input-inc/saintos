mod bindings;
mod commands;
mod discovery;
mod input;
mod protocol;

use bindings::mapper::ActionEvent;
use commands::AppState;
use std::sync::atomic::Ordering;
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

/// Cadence at which the input manager publishes `input-state` to the
/// frontend. Deliberately decoupled from INPUT_LOOP_MS: that constant
/// governs how fresh the value reaching the ROBOT is, this one governs
/// how often the UI is told, and the two have nothing to do with each
/// other. Nothing on screen renders faster than the panel, so anything
/// above ~60 Hz is pure cost — each emit is a serde serialize plus a
/// Tauri IPC hop that wakes the webview's main thread, and it happened
/// 250x/s regardless of whether a view was even displaying input.
/// The emit is additionally change-gated in the input manager, so an
/// untouched controller publishes at the idle heartbeat rate (1 Hz)
/// rather than continuously.
const UI_EMIT_MS: u64 = 16; // ~60 Hz ceiling, change-gated below that

/// Tick period for the input/processing loops while the window is NOT
/// focused. Coarse on purpose: nothing is rendering and nothing is being
/// commanded, so this only has to stay responsive enough to notice focus
/// coming back. 100 ms turns a 250 Hz loop into 10 Hz.
const UNFOCUSED_TICK_MS: u64 = 100;

/// Bytes per log file before rotating. The plugin's default is 40 KB,
/// which at this app's log volume rotates every few seconds; each
/// rotation is a flush + rename + create, and the debris makes the log
/// directory hostile to search. 16 MB holds a long session in one
/// greppable file.
const MAX_LOG_FILE_BYTES: u128 = 16 * 1024 * 1024;

/// Where log lines are delivered.
///
/// Stdout + a rotating file always. The file is the diagnostic record
/// and carries 100% of what is logged, at every level.
///
/// The Webview target is debug-only, and even there it is inert until
/// someone wires it up. It is the most expensive sink per line by a
/// wide margin: for EVERY record the plugin allocates a String, clones
/// the app handle, SPAWNS A TOKIO TASK, and emits an IPC event to the
/// webview — all to duplicate a line the file already has.
///
/// Worth knowing before re-enabling it in release: nothing in the
/// frontend currently listens for `log://log`. The JS side has to call
/// `attachConsole()` from `@tauri-apps/plugin-log` for these events to
/// surface anywhere, and no module imports it. Until something does,
/// every emit is pure cost with no reader.
fn log_targets() -> Vec<Target> {
    let mut targets = vec![
        Target::new(TargetKind::Stdout),
        Target::new(TargetKind::LogDir { file_name: Some("saint-controller".into()) }),
    ];
    #[cfg(debug_assertions)]
    targets.push(Target::new(TargetKind::Webview));
    targets
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_shell::init())
        .plugin(tauri_plugin_process::init())
        .plugin(
            tauri_plugin_log::Builder::new()
                .targets(log_targets())
                // Debug stays on in release ON PURPOSE. These builds ship
                // to a robot, and the log file is the only artifact that
                // explains a field failure after the fact — the cost of
                // the extra lines is paid back the first time one is
                // needed. What was made cheaper is how a line is
                // DELIVERED (see log_targets and max_file_size), not how
                // much is recorded.
                .level(log::LevelFilter::Debug)
                // Keep every rotated file: "see all the log information
                // at any time" means history survives a restart.
                //
                // NOTE: this is unbounded on disk by design. Pair it with
                // the large max_file_size below — at the plugin's 40 KB
                // default a driving session rotated every couple of
                // seconds, so a session left running produced thousands
                // of tiny files that were miserable to grep and cost a
                // rename + create + metadata stat each time. Switch to
                // RotationStrategy::KeepSome(n) if disk pressure ever
                // matters more than complete history.
                .max_file_size(MAX_LOG_FILE_BYTES)
                .rotation_strategy(RotationStrategy::KeepAll)
                .build(),
        )
        .setup(|app| {
            let state = Arc::new(AppState::new());

            // Start the input readers + the UI publish thread. The
            // readers (gilrs / HID / evdev) run on their own cadences
            // and write into shared state; UI_EMIT_MS only paces how
            // often that state is pushed to the frontend. The wire send
            // rate is gated separately again by the WS client throttle.
            let app_handle = app.handle().clone();
            state.input_manager.start(app_handle.clone(), UI_EMIT_MS, state.focused.clone());

            // Start input processing loop (processes bindings and sends commands)
            let state_for_processing = state.clone();
            thread::spawn(move || {
                log::info!("Input processing thread started");

                // Tracks the link so the loop can spot the moment it
                // comes up. Sampled once per tick rather than per
                // command: a command send took a write lock on the WS
                // client's throttle map and inserted into it BEFORE
                // discovering there was nothing to send to.
                let mut was_connected = false;
                // Starts true to match AppState::focused — the window is
                // focused when it opens and no event announces that.
                let mut was_focused = true;

                loop {
                    let connected = state_for_processing.ws_client.is_connected();
                    let focused = state_for_processing.focused.load(Ordering::Relaxed);

                    if was_focused && !focused {
                        // Going quiet is not enough. A deflected stick is
                        // already commanded, the server change-gates
                        // unchanged values, and only the firmware
                        // dead-man (1250 ms) would stop the motor. Say
                        // zero explicitly on the way out.
                        log::info!("Backgrounded: releasing all analog targets");
                        let events = {
                            let mut mapper = state_for_processing.mapper.write();
                            mapper.release_all_analog()
                        };
                        for event in events {
                            if let ActionEvent::Command(cmd) = event {
                                use bindings::mapper::MappedCommand;
                                let result = match &cmd {
                                    MappedCommand::Topic { topic, channel, value } =>
                                        state_for_processing.ws_client.send_topic_channel_value(
                                            topic, channel, value.clone()),
                                    MappedCommand::WsInput { sheet_id, input_id, value } =>
                                        state_for_processing.ws_client.send_ws_input_value(
                                            sheet_id, input_id, value.clone()),
                                };
                                if let Err(e) = result {
                                    if !protocol::client::is_transient_send_error(&e) {
                                        log::error!("Release-on-blur send failed: {}", e);
                                    }
                                }
                            }
                        }
                    }

                    if !was_focused && focused {
                        // Re-assert every axis from LIVE input on the next
                        // tick. If the operator comes back still holding
                        // the stick, motion resumes — same semantics as a
                        // link coming up, and the honest reading of a
                        // held control.
                        log::info!("Focused: re-asserting all bound axes from live input");
                        state_for_processing.mapper.write().forget_all_sends();
                    }

                    was_focused = focused;

                    if !focused {
                        // Track the link so a drop/restore while hidden
                        // can't look like a transition on the tick after
                        // we come back. (forget_all_sends on refocus
                        // covers the state either way; this just keeps
                        // the log honest.)
                        was_connected = connected;
                        thread::sleep(Duration::from_millis(UNFOCUSED_TICK_MS));
                        continue;
                    }

                    if connected && !was_connected {
                        // Link just came up. Everything the mapper
                        // believes it sent while the link was down went
                        // nowhere, so drop that bookkeeping and let the
                        // next few microseconds re-assert every bound
                        // axis against the server that actually exists
                        // now. Previously this waited on the 500 ms
                        // heartbeat.
                        //
                        // A disconnect and reconnect entirely between two
                        // samples would be missed, but a reconnect takes
                        // at least RECONNECT_DELAY_MS (1 s) against a
                        // 4 ms tick, and the heartbeat is still there as
                        // a backstop — so the worst case is the old
                        // behavior, not a stranded axis.
                        log::info!("Link up: re-asserting all bound axes");
                        state_for_processing.mapper.write().forget_all_sends();
                    }
                    was_connected = connected;

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
                            ActionEvent::Command(cmd) if !connected => {
                                // No link: don't touch the send path at
                                // all. It would take the throttle write
                                // lock, insert a timestamp, and only then
                                // return "Not connected" — and the
                                // note_send_failed below would answer
                                // that by scanning the mapper's two maps,
                                // per command, per 4 ms tick, forever.
                                //
                                // Dropping it silently is safe because
                                // the link-up branch above re-asserts
                                // everything wholesale. Bound with a
                                // guard rather than an early `continue`
                                // so UI events below still run while
                                // disconnected — the operator navigates
                                // menus with no server attached.
                                let _ = cmd;
                            }
                            ActionEvent::Command(cmd) => {
                                use bindings::mapper::MappedCommand;
                                // Borrow rather than move: a refused send
                                // has to be handed back to the mapper so
                                // it can un-commit its optimistic "sent"
                                // bookkeeping (see note_send_failed).
                                let result = match &cmd {
                                    MappedCommand::Topic { topic, channel, value } =>
                                        state_for_processing.ws_client.send_topic_channel_value(
                                            topic, channel, value.clone()),
                                    MappedCommand::WsInput { sheet_id, input_id, value } =>
                                        state_for_processing.ws_client.send_ws_input_value(
                                            sheet_id, input_id, value.clone()),
                                };
                                if let Err(e) = result {
                                    // Retry on the NEXT tick (~4 ms), not
                                    // the 500 ms heartbeat. This is what
                                    // bounds how long a motor can keep
                                    // running after a refused deadstick.
                                    state_for_processing
                                        .mapper
                                        .write()
                                        .note_send_failed(&cmd);
                                    if !protocol::client::is_transient_send_error(&e) {
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

            // Window focus drives whether the input pipeline runs. A
            // backgrounded controller (Steam overlay, QAM, task switch)
            // is not being looked at and must not be driving a robot —
            // see the blur branch in the processing loop, which commands
            // neutral before going quiet.
            //
            // The frontend is told too: it uses focus regain to kick an
            // immediate reconnect rather than waiting out the WS
            // backoff, and to drop telemetry subscriptions while hidden.
            {
                let focused = state.focused.clone();
                let focus_app_handle = app.handle().clone();
                if let Some(window) = app.get_webview_window("main") {
                    window.on_window_event(move |event| {
                        if let tauri::WindowEvent::Focused(is_focused) = event {
                            focused.store(*is_focused, Ordering::Relaxed);
                            log::info!(
                                "Window {}",
                                if *is_focused { "focused" } else { "backgrounded" }
                            );
                            if let Err(e) =
                                focus_app_handle.emit("app-focus", *is_focused)
                            {
                                log::error!("Failed to emit app-focus: {}", e);
                            }
                        }
                    });
                } else {
                    log::warn!("No main window; input pipeline will never pause");
                }
            }

            // Saved profiles are loaded by InputMapper::new, so the
            // IMU's initial state has to be evaluated here rather than
            // waiting for the first profile edit — otherwise a operator
            // whose profile uses the gyro would find it dead until they
            // touched the bindings screen.
            commands::refresh_gyro_binding_demand(&state);

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
            commands::set_gyro_diagnostics,
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
            commands::unsubscribe_topics,
            commands::get_wifi_config,
            commands::wifi_survey,
            commands::set_wifi_channel,
            commands::list_animations,
            commands::list_poses,
            commands::list_sounds,
            commands::start_animation,
            commands::stop_animation,
            commands::apply_pose,
            commands::play_sound,
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
