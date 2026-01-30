use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct GyroState {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct TouchpadState {
    pub x: f32,      // -1 to 1
    pub y: f32,      // -1 to 1
    pub touched: bool,
    pub clicked: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SteamDeckExtras {
    pub gyro: GyroState,
    pub left_touchpad: TouchpadState,
    pub right_touchpad: TouchpadState,
}

pub struct GyroHandler {
    state: Arc<RwLock<GyroState>>,
    extras: Arc<RwLock<SteamDeckExtras>>,
}

impl GyroHandler {
    pub fn new() -> Self {
        let state = Arc::new(RwLock::new(GyroState::default()));
        let extras = Arc::new(RwLock::new(SteamDeckExtras::default()));

        #[cfg(target_os = "linux")]
        {
            // Log all input devices for debugging
            Self::log_input_devices();

            let state_clone = state.clone();
            let extras_clone = extras.clone();
            std::thread::spawn(move || {
                Self::run_steamdeck_input_loop(state_clone, extras_clone);
            });
        }

        Self { state, extras }
    }

    #[cfg(target_os = "linux")]
    fn log_input_devices() {
        use evdev::Device;

        tracing::info!("=== Scanning input devices ===");
        if let Ok(entries) = std::fs::read_dir("/dev/input") {
            for entry in entries.flatten() {
                let path = entry.path();
                if !path.to_string_lossy().contains("event") {
                    continue;
                }
                if let Ok(device) = Device::open(&path) {
                    let name = device.name().unwrap_or("Unknown");
                    tracing::info!("  {:?}: {}", path, name);
                }
            }
        }
        tracing::info!("=== End device scan ===");
    }

    #[cfg(target_os = "linux")]
    fn run_steamdeck_input_loop(
        gyro_state: Arc<RwLock<GyroState>>,
        extras: Arc<RwLock<SteamDeckExtras>>,
    ) {
        use evdev::{AbsoluteAxisType, Device, InputEventKind, Key};
        use std::collections::HashMap;
        use std::os::unix::io::AsRawFd;

        // Find Steam Deck specific devices
        let mut devices: HashMap<String, Device> = HashMap::new();

        if let Ok(entries) = std::fs::read_dir("/dev/input") {
            for entry in entries.flatten() {
                let path = entry.path();
                if !path.to_string_lossy().contains("event") {
                    continue;
                }
                if let Ok(device) = Device::open(&path) {
                    let name = device.name().unwrap_or("Unknown").to_lowercase();

                    // Steam Deck IMU / Gyro
                    if name.contains("gyro") || name.contains("imu") || name.contains("motion") {
                        tracing::info!("Found gyro device: {} at {:?}", device.name().unwrap_or("Unknown"), path);
                        devices.insert("gyro".to_string(), device);
                    }
                    // Steam Deck left touchpad
                    else if name.contains("left") && name.contains("trackpad") {
                        tracing::info!("Found left touchpad: {} at {:?}", device.name().unwrap_or("Unknown"), path);
                        devices.insert("left_touchpad".to_string(), device);
                    }
                    // Steam Deck right touchpad
                    else if name.contains("right") && name.contains("trackpad") {
                        tracing::info!("Found right touchpad: {} at {:?}", device.name().unwrap_or("Unknown"), path);
                        devices.insert("right_touchpad".to_string(), device);
                    }
                    // Generic touchpad detection
                    else if name.contains("touchpad") || name.contains("trackpad") {
                        tracing::info!("Found touchpad: {} at {:?}", device.name().unwrap_or("Unknown"), path);
                        if !devices.contains_key("left_touchpad") {
                            devices.insert("left_touchpad".to_string(), device);
                        } else if !devices.contains_key("right_touchpad") {
                            devices.insert("right_touchpad".to_string(), device);
                        }
                    }
                }
            }
        }

        if devices.is_empty() {
            tracing::warn!("No Steam Deck input devices found (gyro, touchpads)");
            return;
        }

        tracing::info!("Monitoring {} Steam Deck input device(s)", devices.len());

        // Create poll structure for all devices
        let mut poll_fds: Vec<libc::pollfd> = devices
            .values()
            .map(|d| libc::pollfd {
                fd: d.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            })
            .collect();

        let device_names: Vec<String> = devices.keys().cloned().collect();
        let mut device_list: Vec<Device> = devices.into_values().collect();

        loop {
            // Poll with 16ms timeout
            let ret = unsafe {
                libc::poll(
                    poll_fds.as_mut_ptr(),
                    poll_fds.len() as libc::nfds_t,
                    16,
                )
            };

            if ret > 0 {
                for (i, pfd) in poll_fds.iter().enumerate() {
                    if pfd.revents & libc::POLLIN != 0 {
                        if let Ok(events) = device_list[i].fetch_events() {
                            let device_name: &str = device_names[i].as_str();

                            for event in events {
                                match device_name {
                                    "gyro" => {
                                        if let InputEventKind::AbsAxis(axis) = event.kind() {
                                            let mut s = gyro_state.write();
                                            let mut e = extras.write();
                                            // Scale to degrees (typical gyro range)
                                            let value = event.value() as f32 / 32768.0 * 2000.0;
                                            match axis {
                                                AbsoluteAxisType::ABS_RX | AbsoluteAxisType::ABS_X => {
                                                    s.pitch = value;
                                                    e.gyro.pitch = value;
                                                }
                                                AbsoluteAxisType::ABS_RY | AbsoluteAxisType::ABS_Y => {
                                                    s.roll = value;
                                                    e.gyro.roll = value;
                                                }
                                                AbsoluteAxisType::ABS_RZ | AbsoluteAxisType::ABS_Z => {
                                                    s.yaw = value;
                                                    e.gyro.yaw = value;
                                                }
                                                _ => {}
                                            }
                                        }
                                    }
                                    "left_touchpad" => {
                                        let mut e = extras.write();
                                        match event.kind() {
                                            InputEventKind::AbsAxis(axis) => {
                                                match axis {
                                                    AbsoluteAxisType::ABS_X | AbsoluteAxisType::ABS_MT_POSITION_X => {
                                                        // Normalize to -1 to 1
                                                        e.left_touchpad.x = (event.value() as f32 / 32768.0) * 2.0 - 1.0;
                                                    }
                                                    AbsoluteAxisType::ABS_Y | AbsoluteAxisType::ABS_MT_POSITION_Y => {
                                                        e.left_touchpad.y = (event.value() as f32 / 32768.0) * 2.0 - 1.0;
                                                    }
                                                    _ => {}
                                                }
                                            }
                                            InputEventKind::Key(key) => {
                                                match key {
                                                    Key::BTN_TOUCH => {
                                                        e.left_touchpad.touched = event.value() != 0;
                                                    }
                                                    Key::BTN_LEFT => {
                                                        e.left_touchpad.clicked = event.value() != 0;
                                                    }
                                                    _ => {}
                                                }
                                            }
                                            _ => {}
                                        }
                                    }
                                    "right_touchpad" => {
                                        let mut e = extras.write();
                                        match event.kind() {
                                            InputEventKind::AbsAxis(axis) => {
                                                match axis {
                                                    AbsoluteAxisType::ABS_X | AbsoluteAxisType::ABS_MT_POSITION_X => {
                                                        e.right_touchpad.x = (event.value() as f32 / 32768.0) * 2.0 - 1.0;
                                                    }
                                                    AbsoluteAxisType::ABS_Y | AbsoluteAxisType::ABS_MT_POSITION_Y => {
                                                        e.right_touchpad.y = (event.value() as f32 / 32768.0) * 2.0 - 1.0;
                                                    }
                                                    _ => {}
                                                }
                                            }
                                            InputEventKind::Key(key) => {
                                                match key {
                                                    Key::BTN_TOUCH => {
                                                        e.right_touchpad.touched = event.value() != 0;
                                                    }
                                                    Key::BTN_LEFT => {
                                                        e.right_touchpad.clicked = event.value() != 0;
                                                    }
                                                    _ => {}
                                                }
                                            }
                                            _ => {}
                                        }
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn state(&self) -> Arc<RwLock<GyroState>> {
        self.state.clone()
    }

    pub fn extras_state(&self) -> Arc<RwLock<SteamDeckExtras>> {
        self.extras.clone()
    }

    pub fn get_state(&self) -> GyroState {
        self.state.read().clone()
    }

    pub fn get_extras(&self) -> SteamDeckExtras {
        self.extras.read().clone()
    }
}

impl Default for GyroHandler {
    fn default() -> Self {
        Self::new()
    }
}
