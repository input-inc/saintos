use super::gamepad::{GamepadHandler, GamepadState};
use super::gyro::{GyroHandler, GyroState, TouchpadState};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::time::Duration;
use tauri::{AppHandle, Emitter, Runtime};
use std::thread;

#[cfg(target_os = "linux")]
use super::steamdeck_hid::SteamDeckHidReader;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputState {
    pub gamepad: GamepadState,
    pub gyro: GyroState,
    #[serde(rename = "leftTouchpad")]
    pub left_touchpad: TouchpadState,
    #[serde(rename = "rightTouchpad")]
    pub right_touchpad: TouchpadState,
}

pub struct InputManager {
    gamepad: GamepadHandler,
    gyro: GyroHandler,
    #[cfg(target_os = "linux")]
    steamdeck_hid: SteamDeckHidReader,
    running: Arc<RwLock<bool>>,
}

impl InputManager {
    pub fn new() -> Self {
        Self {
            gamepad: GamepadHandler::new(),
            gyro: GyroHandler::new(),
            #[cfg(target_os = "linux")]
            steamdeck_hid: SteamDeckHidReader::new(),
            running: Arc::new(RwLock::new(false)),
        }
    }

    pub fn start<R: Runtime + 'static>(&self, app_handle: AppHandle<R>, poll_interval_ms: u64) {
        let gamepad_state = self.gamepad.state();
        let gyro_state = self.gyro.state();
        let extras_state = self.gyro.extras_state();
        let running = self.running.clone();

        // Start Steam Deck HID reader on Linux
        #[cfg(target_os = "linux")]
        {
            self.steamdeck_hid.start();
        }

        #[cfg(target_os = "linux")]
        let hid_state = self.steamdeck_hid.state();

        *running.write() = true;

        // Spawn a thread for emitting events to the frontend
        thread::spawn(move || {
            log::info!("Input emit thread started");
            while *running.read() {
                let mut gamepad = gamepad_state.read().clone();

                // On Linux, merge HID data for back buttons, gyro, and touchpads
                #[cfg(target_os = "linux")]
                let (gyro, left_touchpad, right_touchpad) = {
                    let hid = hid_state.read();

                    // Merge back button states from HID into gamepad buttons
                    // Always set the state (true or false) so buttons properly release
                    gamepad.buttons.insert("L4".to_string(), hid.l4_pressed);
                    gamepad.buttons.insert("R4".to_string(), hid.r4_pressed);
                    gamepad.buttons.insert("L5".to_string(), hid.l5_pressed);
                    gamepad.buttons.insert("R5".to_string(), hid.r5_pressed);
                    gamepad.buttons.insert("Steam".to_string(), hid.steam_pressed);
                    gamepad.buttons.insert("QAM".to_string(), hid.qam_pressed);

                    // Filter out right stick when right touchpad is active
                    // Steam Input mirrors touchpad to right stick, so ignore stick when pad is touched
                    if hid.right_pad_touched || hid.right_pad_x.abs() > 0.01 || hid.right_pad_y.abs() > 0.01 {
                        gamepad.right_stick.x = 0.0;
                        gamepad.right_stick.y = 0.0;
                    }

                    // Use HID data for gyro if available
                    let gyro = if hid.gyro_pitch != 0.0 || hid.gyro_roll != 0.0 || hid.gyro_yaw != 0.0 {
                        GyroState {
                            pitch: hid.gyro_pitch,
                            roll: hid.gyro_roll,
                            yaw: hid.gyro_yaw,
                        }
                    } else {
                        gyro_state.read().clone()
                    };

                    // Use HID data for touchpads if available
                    let left_touchpad = if hid.left_pad_touched || hid.left_pad_x != 0.0 || hid.left_pad_y != 0.0 {
                        TouchpadState {
                            x: hid.left_pad_x,
                            y: hid.left_pad_y,
                            touched: hid.left_pad_touched,
                            clicked: hid.left_pad_clicked,
                        }
                    } else {
                        extras_state.read().left_touchpad.clone()
                    };

                    let right_touchpad = if hid.right_pad_touched || hid.right_pad_x != 0.0 || hid.right_pad_y != 0.0 {
                        TouchpadState {
                            x: hid.right_pad_x,
                            y: hid.right_pad_y,
                            touched: hid.right_pad_touched,
                            clicked: hid.right_pad_clicked,
                        }
                    } else {
                        extras_state.read().right_touchpad.clone()
                    };

                    (gyro, left_touchpad, right_touchpad)
                };

                #[cfg(not(target_os = "linux"))]
                let (gyro, left_touchpad, right_touchpad) = {
                    let extras = extras_state.read();
                    (
                        gyro_state.read().clone(),
                        extras.left_touchpad.clone(),
                        extras.right_touchpad.clone(),
                    )
                };

                let state = InputState {
                    gamepad,
                    gyro,
                    left_touchpad,
                    right_touchpad,
                };

                if let Err(e) = app_handle.emit("input-state", &state) {
                    log::error!("Failed to emit input state: {}", e);
                }

                thread::sleep(Duration::from_millis(poll_interval_ms));
            }
            log::info!("Input manager stopped");
        });

        log::info!(
            "Input manager started with {}ms poll interval",
            poll_interval_ms
        );
    }

    pub fn stop(&self) {
        *self.running.write() = false;
        #[cfg(target_os = "linux")]
        self.steamdeck_hid.stop();
    }

    pub fn get_state(&self) -> InputState {
        let mut gamepad = self.gamepad.get_state();

        #[cfg(target_os = "linux")]
        let (gyro, left_touchpad, right_touchpad) = {
            let hid = self.steamdeck_hid.get_state();

            // Merge back button states
            gamepad.buttons.insert("L4".to_string(), hid.l4_pressed);
            gamepad.buttons.insert("R4".to_string(), hid.r4_pressed);
            gamepad.buttons.insert("L5".to_string(), hid.l5_pressed);
            gamepad.buttons.insert("R5".to_string(), hid.r5_pressed);
            gamepad.buttons.insert("Steam".to_string(), hid.steam_pressed);
            gamepad.buttons.insert("QAM".to_string(), hid.qam_pressed);

            // Filter out right stick when right touchpad is active
            // Steam Input mirrors touchpad to right stick, so ignore stick when pad is touched
            if hid.right_pad_touched || hid.right_pad_x.abs() > 0.01 || hid.right_pad_y.abs() > 0.01 {
                gamepad.right_stick.x = 0.0;
                gamepad.right_stick.y = 0.0;
            }

            let gyro = if hid.gyro_pitch != 0.0 || hid.gyro_roll != 0.0 || hid.gyro_yaw != 0.0 {
                GyroState {
                    pitch: hid.gyro_pitch,
                    roll: hid.gyro_roll,
                    yaw: hid.gyro_yaw,
                }
            } else {
                self.gyro.get_state()
            };

            let extras = self.gyro.get_extras();
            let left_touchpad = if hid.left_pad_touched {
                TouchpadState {
                    x: hid.left_pad_x,
                    y: hid.left_pad_y,
                    touched: hid.left_pad_touched,
                    clicked: hid.left_pad_clicked,
                }
            } else {
                extras.left_touchpad
            };

            let right_touchpad = if hid.right_pad_touched {
                TouchpadState {
                    x: hid.right_pad_x,
                    y: hid.right_pad_y,
                    touched: hid.right_pad_touched,
                    clicked: hid.right_pad_clicked,
                }
            } else {
                extras.right_touchpad
            };

            (gyro, left_touchpad, right_touchpad)
        };

        #[cfg(not(target_os = "linux"))]
        let (gyro, left_touchpad, right_touchpad) = {
            let extras = self.gyro.get_extras();
            (self.gyro.get_state(), extras.left_touchpad, extras.right_touchpad)
        };

        InputState {
            gamepad,
            gyro,
            left_touchpad,
            right_touchpad,
        }
    }

    pub fn is_gamepad_connected(&self) -> bool {
        self.gamepad.get_state().connected
    }
}

impl Default for InputManager {
    fn default() -> Self {
        Self::new()
    }
}

// InputManager is Send+Sync because all its fields are
unsafe impl Send for InputManager {}
unsafe impl Sync for InputManager {}
