pub mod gamepad;
pub mod gyro;
pub mod manager;

#[cfg(target_os = "linux")]
pub mod steamdeck_hid;

pub use manager::InputManager;
