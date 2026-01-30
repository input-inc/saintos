//! Direct HID access to Steam Deck controller for gyro, touchpads, and back buttons.
//!
//! This bypasses Steam Input to get raw sensor and button data.

use parking_lot::RwLock;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

// Steam Deck USB IDs
const STEAM_DECK_VID: u16 = 0x28DE;
const STEAM_DECK_PID: u16 = 0x1205;

// Button masks for ulButtonsL (lower 32 bits)
const STEAMDECK_LBUTTON_R2: u32 = 0x00000001;
const STEAMDECK_LBUTTON_L2: u32 = 0x00000002;
const STEAMDECK_LBUTTON_R: u32 = 0x00000004;
const STEAMDECK_LBUTTON_L: u32 = 0x00000008;
const STEAMDECK_LBUTTON_Y: u32 = 0x00000010;
const STEAMDECK_LBUTTON_B: u32 = 0x00000020;
const STEAMDECK_LBUTTON_X: u32 = 0x00000040;
const STEAMDECK_LBUTTON_A: u32 = 0x00000080;
const STEAMDECK_LBUTTON_DPAD_UP: u32 = 0x00000100;
const STEAMDECK_LBUTTON_DPAD_RIGHT: u32 = 0x00000200;
const STEAMDECK_LBUTTON_DPAD_LEFT: u32 = 0x00000400;
const STEAMDECK_LBUTTON_DPAD_DOWN: u32 = 0x00000800;
const STEAMDECK_LBUTTON_VIEW: u32 = 0x00001000;  // Select
const STEAMDECK_LBUTTON_STEAM: u32 = 0x00002000;
const STEAMDECK_LBUTTON_MENU: u32 = 0x00004000;  // Start
const STEAMDECK_LBUTTON_L5: u32 = 0x00008000;
const STEAMDECK_LBUTTON_R5: u32 = 0x00010000;
const STEAMDECK_LBUTTON_LEFT_PAD: u32 = 0x00020000;
const STEAMDECK_LBUTTON_RIGHT_PAD: u32 = 0x00040000;
const STEAMDECK_LBUTTON_L3: u32 = 0x00400000;
const STEAMDECK_LBUTTON_R3: u32 = 0x04000000;

// Button masks for ulButtonsH (upper 32 bits)
const STEAMDECK_HBUTTON_L4: u32 = 0x00000200;
const STEAMDECK_HBUTTON_R4: u32 = 0x00000400;
const STEAMDECK_HBUTTON_QAM: u32 = 0x00040000;  // Quick Access Menu (...)

/// Raw Steam Deck input state from HID
#[derive(Debug, Clone, Default)]
pub struct SteamDeckHidState {
    // Gyroscope (degrees per second)
    pub gyro_pitch: f32,
    pub gyro_roll: f32,
    pub gyro_yaw: f32,

    // Accelerometer (m/s²)
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,

    // Left touchpad (-1.0 to 1.0)
    pub left_pad_x: f32,
    pub left_pad_y: f32,
    pub left_pad_pressure: f32,
    pub left_pad_touched: bool,
    pub left_pad_clicked: bool,

    // Right touchpad (-1.0 to 1.0)
    pub right_pad_x: f32,
    pub right_pad_y: f32,
    pub right_pad_pressure: f32,
    pub right_pad_touched: bool,
    pub right_pad_clicked: bool,

    // Back buttons
    pub l4_pressed: bool,
    pub r4_pressed: bool,
    pub l5_pressed: bool,
    pub r5_pressed: bool,

    // Other buttons (from HID, may differ from gilrs)
    pub steam_pressed: bool,
    pub qam_pressed: bool,  // Quick Access Menu (...)
}

pub struct SteamDeckHidReader {
    state: Arc<RwLock<SteamDeckHidState>>,
    running: Arc<RwLock<bool>>,
}

impl SteamDeckHidReader {
    pub fn new() -> Self {
        let state = Arc::new(RwLock::new(SteamDeckHidState::default()));
        let running = Arc::new(RwLock::new(false));

        Self { state, running }
    }

    pub fn start(&self) {
        let state = self.state.clone();
        let running = self.running.clone();

        *running.write() = true;

        thread::spawn(move || {
            Self::run_hid_loop(state, running);
        });
    }

    pub fn stop(&self) {
        *self.running.write() = false;
    }

    pub fn state(&self) -> Arc<RwLock<SteamDeckHidState>> {
        self.state.clone()
    }

    pub fn get_state(&self) -> SteamDeckHidState {
        self.state.read().clone()
    }

    #[cfg(target_os = "linux")]
    fn run_hid_loop(state: Arc<RwLock<SteamDeckHidState>>, running: Arc<RwLock<bool>>) {
        use hidapi::HidApi;

        log::info!("Starting Steam Deck HID reader...");

        let api = match HidApi::new() {
            Ok(api) => api,
            Err(e) => {
                log::error!("Failed to initialize HID API: {}", e);
                return;
            }
        };

        // List all HID devices for debugging
        log::info!("=== Scanning HID devices ===");
        for device_info in api.device_list() {
            log::info!(
                "  VID:{:04X} PID:{:04X} - {} ({})",
                device_info.vendor_id(),
                device_info.product_id(),
                device_info.product_string().unwrap_or("Unknown"),
                device_info.path().to_string_lossy()
            );
        }
        log::info!("=== End HID scan ===");

        // Try to open Steam Deck controller
        let device = match api.open(STEAM_DECK_VID, STEAM_DECK_PID) {
            Ok(dev) => {
                log::info!("Opened Steam Deck HID device (VID:{:04X} PID:{:04X})",
                    STEAM_DECK_VID, STEAM_DECK_PID);
                dev
            }
            Err(e) => {
                log::warn!("Failed to open Steam Deck HID device: {}", e);
                log::info!("This is normal if not running on Steam Deck or if Steam is using the device");
                return;
            }
        };

        // Set non-blocking mode
        if let Err(e) = device.set_blocking_mode(false) {
            log::warn!("Failed to set non-blocking mode: {}", e);
        }

        let mut buf = [0u8; 64];

        log::info!("Steam Deck HID reader running");

        while *running.read() {
            match device.read_timeout(&mut buf, 16) {
                Ok(size) if size > 0 => {
                    Self::parse_hid_report(&buf[..size], &state);
                }
                Ok(_) => {
                    // No data available, that's ok
                }
                Err(e) => {
                    log::error!("HID read error: {}", e);
                    thread::sleep(Duration::from_millis(100));
                }
            }

            thread::sleep(Duration::from_millis(1));
        }

        log::info!("Steam Deck HID reader stopped");
    }

    #[cfg(not(target_os = "linux"))]
    fn run_hid_loop(_state: Arc<RwLock<SteamDeckHidState>>, _running: Arc<RwLock<bool>>) {
        log::info!("Steam Deck HID reader not available on this platform");
    }

    #[cfg(target_os = "linux")]
    fn parse_hid_report(data: &[u8], state: &Arc<RwLock<SteamDeckHidState>>) {
        // The Steam Deck HID report structure:
        // Based on SDL's SteamDeckStatePacket_t
        // Report includes header bytes before the payload

        if data.len() < 56 {
            return;  // Not enough data
        }

        // Skip report header (typically 4 bytes: report ID + header)
        // The exact offset depends on the report format
        // For Steam Deck, try offset 4 for the state packet
        let offset = 4;

        if data.len() < offset + 56 {
            return;
        }

        let payload = &data[offset..];

        // Parse button states
        let buttons_l = u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
        let buttons_h = u32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]);

        // Parse touchpad coordinates (signed 16-bit, range roughly -32768 to 32767)
        let left_pad_x = i16::from_le_bytes([payload[12], payload[13]]);
        let left_pad_y = i16::from_le_bytes([payload[14], payload[15]]);
        let right_pad_x = i16::from_le_bytes([payload[16], payload[17]]);
        let right_pad_y = i16::from_le_bytes([payload[18], payload[19]]);

        // Parse accelerometer (signed 16-bit)
        let accel_x = i16::from_le_bytes([payload[20], payload[21]]);
        let accel_y = i16::from_le_bytes([payload[22], payload[23]]);
        let accel_z = i16::from_le_bytes([payload[24], payload[25]]);

        // Parse gyroscope (signed 16-bit)
        let gyro_x = i16::from_le_bytes([payload[26], payload[27]]);
        let gyro_y = i16::from_le_bytes([payload[28], payload[29]]);
        let gyro_z = i16::from_le_bytes([payload[30], payload[31]]);

        // Parse touchpad pressure (unsigned 16-bit)
        let pressure_left = u16::from_le_bytes([payload[52], payload[53]]);
        let pressure_right = u16::from_le_bytes([payload[54], payload[55]]);

        // Update state
        let mut s = state.write();

        // Gyro conversion: raw value to degrees per second
        // Scale factor is approximately 2000 dps full scale / 32768
        const GYRO_SCALE: f32 = 2000.0 / 32768.0;
        s.gyro_pitch = gyro_x as f32 * GYRO_SCALE;
        s.gyro_roll = gyro_z as f32 * GYRO_SCALE;
        s.gyro_yaw = -(gyro_y as f32) * GYRO_SCALE;  // Negated per SDL

        // Accelerometer conversion: raw to m/s²
        // Scale factor is approximately 2g full scale / 32768 * 9.81
        const ACCEL_SCALE: f32 = (2.0 * 9.81) / 32768.0;
        s.accel_x = accel_x as f32 * ACCEL_SCALE;
        s.accel_y = -(accel_y as f32) * ACCEL_SCALE;  // Negated per SDL
        s.accel_z = accel_z as f32 * ACCEL_SCALE;

        // Touchpad coordinates: normalize to -1.0 to 1.0
        const PAD_SCALE: f32 = 1.0 / 32768.0;
        s.left_pad_x = left_pad_x as f32 * PAD_SCALE;
        s.left_pad_y = left_pad_y as f32 * PAD_SCALE;
        s.right_pad_x = right_pad_x as f32 * PAD_SCALE;
        s.right_pad_y = right_pad_y as f32 * PAD_SCALE;

        // Touchpad pressure: normalize to 0.0 to 1.0
        const PRESSURE_SCALE: f32 = 1.0 / 65535.0;
        s.left_pad_pressure = pressure_left as f32 * PRESSURE_SCALE;
        s.right_pad_pressure = pressure_right as f32 * PRESSURE_SCALE;

        // Touchpad touched/clicked from button masks
        s.left_pad_touched = (buttons_l & STEAMDECK_LBUTTON_LEFT_PAD) != 0;
        s.left_pad_clicked = s.left_pad_pressure > 0.5;  // Pressure threshold for click
        s.right_pad_touched = (buttons_l & STEAMDECK_LBUTTON_RIGHT_PAD) != 0;
        s.right_pad_clicked = s.right_pad_pressure > 0.5;

        // Back buttons
        s.l4_pressed = (buttons_h & STEAMDECK_HBUTTON_L4) != 0;
        s.r4_pressed = (buttons_h & STEAMDECK_HBUTTON_R4) != 0;
        s.l5_pressed = (buttons_l & STEAMDECK_LBUTTON_L5) != 0;
        s.r5_pressed = (buttons_l & STEAMDECK_LBUTTON_R5) != 0;

        // Other buttons
        s.steam_pressed = (buttons_l & STEAMDECK_LBUTTON_STEAM) != 0;
        s.qam_pressed = (buttons_h & STEAMDECK_HBUTTON_QAM) != 0;
    }
}

impl Default for SteamDeckHidReader {
    fn default() -> Self {
        Self::new()
    }
}
