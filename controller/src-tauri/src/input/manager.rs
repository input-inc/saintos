use super::gamepad::{GamepadHandler, GamepadState};
use super::gyro::{GyroHandler, GyroState};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::time::Duration;
use tauri::{AppHandle, Emitter, Runtime};
use std::thread;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputState {
    pub gamepad: GamepadState,
    pub gyro: GyroState,
}

pub struct InputManager {
    gamepad: GamepadHandler,
    gyro: GyroHandler,
    running: Arc<RwLock<bool>>,
}

impl InputManager {
    pub fn new() -> Self {
        Self {
            gamepad: GamepadHandler::new(),
            gyro: GyroHandler::new(),
            running: Arc::new(RwLock::new(false)),
        }
    }

    pub fn start<R: Runtime + 'static>(&self, app_handle: AppHandle<R>, poll_interval_ms: u64) {
        let gamepad_state = self.gamepad.state();
        let gyro_state = self.gyro.state();
        let running = self.running.clone();

        *running.write() = true;

        // Spawn a thread for emitting events to the frontend
        thread::spawn(move || {
            while *running.read() {
                let state = InputState {
                    gamepad: gamepad_state.read().clone(),
                    gyro: gyro_state.read().clone(),
                };

                if let Err(e) = app_handle.emit("input-state", &state) {
                    tracing::error!("Failed to emit input state: {}", e);
                }

                thread::sleep(Duration::from_millis(poll_interval_ms));
            }
            tracing::info!("Input manager stopped");
        });

        tracing::info!(
            "Input manager started with {}ms poll interval",
            poll_interval_ms
        );
    }

    pub fn stop(&self) {
        *self.running.write() = false;
    }

    pub fn get_state(&self) -> InputState {
        InputState {
            gamepad: self.gamepad.get_state(),
            gyro: self.gyro.get_state(),
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
