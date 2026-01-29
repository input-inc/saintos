use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct GyroState {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

pub struct GyroHandler {
    state: Arc<RwLock<GyroState>>,
}

impl GyroHandler {
    pub fn new() -> Self {
        let state = Arc::new(RwLock::new(GyroState::default()));

        #[cfg(target_os = "linux")]
        {
            let state_clone = state.clone();
            std::thread::spawn(move || {
                Self::run_gyro_loop_linux(state_clone);
            });
        }

        Self { state }
    }

    #[cfg(target_os = "linux")]
    fn run_gyro_loop_linux(state: Arc<RwLock<GyroState>>) {
        use evdev::{AbsoluteAxisType, Device, InputEventKind};

        // Look for Steam Deck gyro device
        let device = std::fs::read_dir("/dev/input")
            .ok()
            .and_then(|entries| {
                for entry in entries.flatten() {
                    let path = entry.path();
                    if !path.to_string_lossy().contains("event") {
                        continue;
                    }
                    if let Ok(device) = Device::open(&path) {
                        let name = device.name().unwrap_or_default();
                        if name.contains("Gyroscope")
                            || name.contains("gyro")
                            || name.contains("Motion")
                        {
                            tracing::info!("Found gyro device: {} at {:?}", name, path);
                            return Some(device);
                        }
                    }
                }
                None
            });

        let Some(mut device) = device else {
            tracing::info!("No gyro device found");
            return;
        };

        loop {
            if let Ok(events) = device.fetch_events() {
                let mut s = state.write();
                for event in events {
                    if let InputEventKind::AbsAxis(axis) = event.kind() {
                        let value = event.value() as f32 / 32768.0 * 180.0;
                        match axis {
                            AbsoluteAxisType::ABS_RX => s.pitch = value,
                            AbsoluteAxisType::ABS_RY => s.roll = value,
                            AbsoluteAxisType::ABS_RZ => s.yaw = value,
                            _ => {}
                        }
                    }
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(16));
        }
    }

    pub fn state(&self) -> Arc<RwLock<GyroState>> {
        self.state.clone()
    }

    pub fn get_state(&self) -> GyroState {
        self.state.read().clone()
    }
}

impl Default for GyroHandler {
    fn default() -> Self {
        Self::new()
    }
}
