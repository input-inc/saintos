use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use std::thread;

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AxisState {
    pub x: f32,
    pub y: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct GamepadState {
    pub connected: bool,
    pub name: String,
    #[serde(rename = "leftStick")]
    pub left_stick: AxisState,
    #[serde(rename = "rightStick")]
    pub right_stick: AxisState,
    #[serde(rename = "leftTrigger")]
    pub left_trigger: f32,
    #[serde(rename = "rightTrigger")]
    pub right_trigger: f32,
    pub buttons: HashMap<String, bool>,
}

pub struct GamepadHandler {
    state: Arc<RwLock<GamepadState>>,
}

impl GamepadHandler {
    pub fn new() -> Self {
        let state = Arc::new(RwLock::new(GamepadState::default()));
        let state_clone = state.clone();

        thread::spawn(move || {
            Self::run_gamepad_loop(state_clone);
        });

        Self { state }
    }

    #[cfg(target_os = "macos")]
    fn run_gamepad_loop(state: Arc<RwLock<GamepadState>>) {
        use objc2_game_controller::GCController;

        tracing::info!("Starting macOS Game Controller detection loop...");

        loop {
            // Get all connected controllers
            let controllers = unsafe { GCController::controllers() };
            let count = controllers.len();

            if count > 0 {
                // Use the first controller
                if let Some(controller) = controllers.first() {
                    // Get the extended gamepad profile (for Xbox-style controllers)
                    let gamepad = unsafe { controller.extendedGamepad() };

                    if let Some(gp) = gamepad {
                        let mut s = state.write();

                        if !s.connected {
                            tracing::info!("Controller connected: Xbox Controller");
                        }
                        s.connected = true;
                        s.name = "Xbox Controller".to_string();

                        unsafe {
                            // Read stick values
                            let left_stick = gp.leftThumbstick();
                            s.left_stick.x = left_stick.xAxis().value();
                            s.left_stick.y = left_stick.yAxis().value();

                            let right_stick = gp.rightThumbstick();
                            s.right_stick.x = right_stick.xAxis().value();
                            s.right_stick.y = right_stick.yAxis().value();

                            // Read triggers
                            s.left_trigger = gp.leftTrigger().value();
                            s.right_trigger = gp.rightTrigger().value();

                            // Read face buttons
                            s.buttons.insert("A".to_string(), gp.buttonA().isPressed());
                            s.buttons.insert("B".to_string(), gp.buttonB().isPressed());
                            s.buttons.insert("X".to_string(), gp.buttonX().isPressed());
                            s.buttons.insert("Y".to_string(), gp.buttonY().isPressed());

                            // Shoulder buttons
                            s.buttons.insert("LB".to_string(), gp.leftShoulder().isPressed());
                            s.buttons.insert("RB".to_string(), gp.rightShoulder().isPressed());

                            // D-Pad
                            let dpad = gp.dpad();
                            s.buttons.insert("DPadUp".to_string(), dpad.up().isPressed());
                            s.buttons.insert("DPadDown".to_string(), dpad.down().isPressed());
                            s.buttons.insert("DPadLeft".to_string(), dpad.left().isPressed());
                            s.buttons.insert("DPadRight".to_string(), dpad.right().isPressed());

                            // Menu button (Start) - always present on extended gamepad
                            s.buttons.insert("Start".to_string(), gp.buttonMenu().isPressed());

                            // Optional buttons - check if they exist
                            if let Some(btn) = gp.buttonOptions() {
                                s.buttons.insert("Select".to_string(), btn.isPressed());
                            }
                            if let Some(btn) = gp.leftThumbstickButton() {
                                s.buttons.insert("LeftStick".to_string(), btn.isPressed());
                            }
                            if let Some(btn) = gp.rightThumbstickButton() {
                                s.buttons.insert("RightStick".to_string(), btn.isPressed());
                            }
                        }
                    }
                }
            } else {
                let mut s = state.write();
                if s.connected {
                    tracing::info!("Controller disconnected");
                    s.connected = false;
                    s.name.clear();
                }
            }

            thread::sleep(std::time::Duration::from_millis(16));
        }
    }

    #[cfg(not(target_os = "macos"))]
    fn run_gamepad_loop(state: Arc<RwLock<GamepadState>>) {
        use gilrs::{Axis, Button, Event, EventType, Gilrs};

        tracing::info!("Starting gamepad detection loop (gilrs)...");

        let gilrs_result = Gilrs::new();
        let mut gilrs = match gilrs_result {
            Ok(g) => {
                tracing::info!("gilrs initialized successfully");
                g
            }
            Err(e) => {
                tracing::error!("Failed to initialize gilrs: {:?}", e);
                return;
            }
        };

        // Log all detected gamepads at startup
        let gamepad_count = gilrs.gamepads().count();
        tracing::info!("Found {} gamepad(s) at startup", gamepad_count);
        for (id, gamepad) in gilrs.gamepads() {
            tracing::info!(
                "  Gamepad {:?}: {} (connected: {})",
                id,
                gamepad.name(),
                gamepad.is_connected()
            );
        }

        // Set initial connected state
        if let Some((_id, gamepad)) = gilrs.gamepads().find(|(_, g)| g.is_connected()) {
            let mut s = state.write();
            s.connected = true;
            s.name = gamepad.name().to_string();
            tracing::info!("Initial gamepad set: {}", s.name);
        }

        loop {
            while let Some(Event { id, event, .. }) = gilrs.next_event() {
                let mut s = state.write();

                match event {
                    EventType::Connected => {
                        if let Some(gamepad) = gilrs.connected_gamepad(id) {
                            s.connected = true;
                            s.name = gamepad.name().to_string();
                            tracing::info!("Gamepad connected: {}", s.name);
                        }
                    }
                    EventType::Disconnected => {
                        s.connected = false;
                        s.name.clear();
                        tracing::info!("Gamepad disconnected");
                    }
                    EventType::ButtonPressed(button, _) => {
                        if let Some(name) = button_name(button) {
                            s.buttons.insert(name.to_string(), true);
                        }
                    }
                    EventType::ButtonReleased(button, _) => {
                        if let Some(name) = button_name(button) {
                            s.buttons.insert(name.to_string(), false);
                        }
                    }
                    EventType::AxisChanged(axis, value, _) => match axis {
                        Axis::LeftStickX => s.left_stick.x = value,
                        Axis::LeftStickY => s.left_stick.y = -value,
                        Axis::RightStickX => s.right_stick.x = value,
                        Axis::RightStickY => s.right_stick.y = -value,
                        _ => {}
                    },
                    EventType::ButtonChanged(button, value, _) => match button {
                        Button::LeftTrigger2 => s.left_trigger = value,
                        Button::RightTrigger2 => s.right_trigger = value,
                        _ => {}
                    },
                    _ => {}
                }
            }

            if let Some((_id, gamepad)) = gilrs.gamepads().find(|(_, g)| g.is_connected()) {
                let mut s = state.write();
                if !s.connected {
                    s.connected = true;
                    s.name = gamepad.name().to_string();
                }
            }

            thread::sleep(std::time::Duration::from_millis(16));
        }
    }

    pub fn state(&self) -> Arc<RwLock<GamepadState>> {
        self.state.clone()
    }

    pub fn get_state(&self) -> GamepadState {
        self.state.read().clone()
    }
}

#[cfg(not(target_os = "macos"))]
fn button_name(button: gilrs::Button) -> Option<&'static str> {
    use gilrs::Button;
    match button {
        Button::South => Some("A"),
        Button::East => Some("B"),
        Button::West => Some("X"),
        Button::North => Some("Y"),
        Button::LeftTrigger => Some("LB"),
        Button::RightTrigger => Some("RB"),
        Button::Select => Some("Select"),
        Button::Start => Some("Start"),
        Button::DPadUp => Some("DPadUp"),
        Button::DPadDown => Some("DPadDown"),
        Button::DPadLeft => Some("DPadLeft"),
        Button::DPadRight => Some("DPadRight"),
        Button::LeftThumb => Some("LeftStick"),
        Button::RightThumb => Some("RightStick"),
        // Steam Deck back buttons (may be mapped to these)
        Button::C => Some("L4"),
        Button::Z => Some("R4"),
        Button::LeftTrigger2 => Some("L5"),  // Full trigger pull
        Button::RightTrigger2 => Some("R5"), // Full trigger pull
        Button::Mode => Some("Steam"),       // Steam button
        _ => {
            // Log unknown buttons for debugging
            tracing::debug!("Unknown button pressed: {:?}", button);
            None
        }
    }
}

impl Default for GamepadHandler {
    fn default() -> Self {
        Self::new()
    }
}

unsafe impl Send for GamepadHandler {}
unsafe impl Sync for GamepadHandler {}
