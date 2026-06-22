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

// Feature report ID for the Steam Controller protocol: enable
// gyro/sensors. (0x81 "clear mappings" is deliberately never sent —
// see enable_sensors() — and 0x83 "get settings" is used inline there,
// so neither needs a named constant.)
const STEAM_FEATURE_REPORT_ENABLE_SENSORS: u8 = 0x87;

// Button masks for ulButtonsL (lower 32 bits). Only the inputs the
// standard gamepad API can't surface are decoded here: the Steam/QAM
// buttons, the back paddles' stick-click neighbors, the trackpad
// touch/click bits, and the stick clicks. Face buttons, the D-pad,
// Start/Select, shoulders and triggers all come through gilrs / the
// macOS GameController API in gamepad.rs (same `gamepad.buttons` map,
// same names the binding mapper reads), so their masks aren't kept here.
const STEAMDECK_LBUTTON_STEAM: u32 = 0x00002000;
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

    // Stick clicks
    pub l3_pressed: bool,  // Left stick click
    pub r3_pressed: bool,  // Right stick click

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

        // Collect all Steam Deck HID devices
        let deck_devices: Vec<_> = api.device_list()
            .filter(|d| d.vendor_id() == STEAM_DECK_VID && d.product_id() == STEAM_DECK_PID)
            .collect();

        if deck_devices.is_empty() {
            log::warn!("No Steam Deck HID devices found");
            return;
        }

        log::info!("Found {} Steam Deck HID device(s), trying each...", deck_devices.len());

        // Try each device to find one that provides controller input data (report ID 0x09)
        let mut working_device = None;
        for device_info in &deck_devices {
            let path = device_info.path();
            let interface = device_info.interface_number();
            log::info!("Trying device: {} (interface {})", path.to_string_lossy(), interface);

            match api.open_path(path) {
                Ok(dev) => {
                    // Set non-blocking mode
                    if let Err(e) = dev.set_blocking_mode(false) {
                        log::warn!("  Failed to set non-blocking mode: {}", e);
                    }

                    // Try reading several times to catch controller data
                    let mut test_buf = [0u8; 64];
                    let mut found_controller = false;

                    for attempt in 0..10 {
                        thread::sleep(Duration::from_millis(20));
                        match dev.read_timeout(&mut test_buf, 50) {
                            Ok(size) if size >= 64 => {
                                let report_id = test_buf[0];
                                log::info!("  Attempt {}: {} bytes, report ID: 0x{:02X}", attempt, size, report_id);
                                log::info!("  First 32 bytes: {:02X?}", &test_buf[..32.min(size)]);

                                // Report ID 0x09 is controller input state
                                if report_id == 0x09 || report_id == 0x01 {
                                    log::info!("  Found controller input device!");
                                    found_controller = true;
                                    break;
                                }
                            }
                            Ok(size) if size > 0 => {
                                log::info!("  Attempt {}: {} bytes (too small for controller)", attempt, size);
                            }
                            Ok(_) => {}
                            Err(e) => {
                                log::warn!("  Attempt {}: Read error: {}", attempt, e);
                            }
                        }
                    }

                    if found_controller {
                        working_device = Some(dev);
                        break;
                    } else {
                        log::info!("  No controller data from this device");
                    }
                }
                Err(e) => {
                    log::warn!("  Failed to open: {}", e);
                }
            }
        }

        let device = match working_device {
            Some(dev) => dev,
            None => {
                log::warn!("Could not find a Steam Deck HID device that provides input data");
                return;
            }
        };

        // Try to enable gyro/sensors by sending feature reports
        Self::enable_sensors(&device);

        log::info!("Steam Deck HID reader running");

        let mut buf = [0u8; 64];
        let mut report_count = 0u64;
        let mut last_log_time = std::time::Instant::now();
        let mut last_value_log_time = std::time::Instant::now();

        while *running.read() {
            match device.read_timeout(&mut buf, 16) {
                Ok(size) if size > 0 => {
                    report_count += 1;

                    // Log first few reports and then periodically
                    if report_count <= 5 || last_log_time.elapsed().as_secs() >= 30 {
                        log::info!("HID report #{}: {} bytes, report_id: 0x{:02X}",
                            report_count, size, buf[0]);
                        log::info!("  Raw bytes [0-31]: {:02X?}", &buf[..32.min(size)]);
                        log::info!("  Raw bytes [32-63]: {:02X?}", &buf[32..64.min(size)]);
                        last_log_time = std::time::Instant::now();
                    }

                    Self::parse_hid_report(&buf[..size], &state);

                    // Log parsed values periodically to verify parsing
                    if last_value_log_time.elapsed().as_secs() >= 5 {
                        let s = state.read();
                        log::info!("Parsed HID state - Gyro: pitch={:.1}, roll={:.1}, yaw={:.1}",
                            s.gyro_pitch, s.gyro_roll, s.gyro_yaw);
                        log::info!("  Left pad: ({:.2}, {:.2}) touched={}, Right pad: ({:.2}, {:.2}) touched={}",
                            s.left_pad_x, s.left_pad_y, s.left_pad_touched,
                            s.right_pad_x, s.right_pad_y, s.right_pad_touched);
                        log::info!("  Back buttons: L4={}, R4={}, L5={}, R5={}, Steam={}, QAM={}",
                            s.l4_pressed, s.r4_pressed, s.l5_pressed, s.r5_pressed,
                            s.steam_pressed, s.qam_pressed);
                        last_value_log_time = std::time::Instant::now();
                    }
                }
                Ok(_) => {
                    // No data available
                }
                Err(e) => {
                    log::error!("HID read error: {}", e);
                    thread::sleep(Duration::from_millis(100));
                }
            }

            thread::sleep(Duration::from_millis(1));
        }

        log::info!("Steam Deck HID reader stopped (processed {} reports)", report_count);
    }

    #[cfg(not(target_os = "linux"))]
    fn run_hid_loop(_state: Arc<RwLock<SteamDeckHidState>>, _running: Arc<RwLock<bool>>) {
        log::info!("Steam Deck HID reader not available on this platform");
    }

    #[cfg(target_os = "linux")]
    fn enable_sensors(device: &hidapi::HidDevice) {
        // Send feature reports to enable gyro and sensors
        // Based on SDL's Steam Controller initialization

        // Feature report to enable gyro (0x87 = set settings)
        // This tells the controller to include gyro data in input reports
        let mut enable_gyro = [0u8; 64];
        enable_gyro[0] = STEAM_FEATURE_REPORT_ENABLE_SENSORS; // 0x87
        enable_gyro[1] = 0x03; // Length
        enable_gyro[2] = 0x30; // Setting: Enable gyro
        enable_gyro[3] = 0x18; // Gyro mode (raw)
        enable_gyro[4] = 0x00;

        match device.send_feature_report(&enable_gyro) {
            Ok(_) => log::info!("Sent gyro enable feature report"),
            Err(e) => log::warn!("Failed to send gyro enable: {} (this may be expected)", e),
        }

        thread::sleep(Duration::from_millis(50));

        // NOTE: We intentionally do NOT send the "clear mappings" (0x81) feature report here.
        // That would disable Steam Input's button mappings, breaking features like
        // X button → "Show Keyboard" in Desktop Mode.

        // Request settings to confirm device is responsive
        let mut get_settings = [0u8; 64];
        get_settings[0] = 0x83; // Get settings
        get_settings[1] = 0x00;

        match device.get_feature_report(&mut get_settings) {
            Ok(len) => log::info!("Got feature report response: {} bytes", len),
            Err(e) => log::warn!("Failed to get settings: {} (this may be expected)", e),
        }
    }

    #[cfg(target_os = "linux")]
    fn parse_hid_report(data: &[u8], state: &Arc<RwLock<SteamDeckHidState>>) {
        // The Steam Deck HID report structure:
        // Report ID 0x09 = Input state (controller data)
        // Report ID 0x04 = Mouse/keyboard emulation
        //
        // Structure (64 bytes total):
        // [0]: Report ID
        // [1-3]: Header/sequence
        // [4-7]: ulButtonsL (lower 32 bits)
        // [8-11]: ulButtonsH (upper 32 bits)
        // [12-13]: Left pad X
        // [14-15]: Left pad Y
        // [16-17]: Right pad X
        // [18-19]: Right pad Y
        // [20-21]: Accel X
        // [22-23]: Accel Y
        // [24-25]: Accel Z
        // [26-27]: Gyro X (pitch)
        // [28-29]: Gyro Y (yaw)
        // [30-31]: Gyro Z (roll)
        // ... more data including analog sticks, triggers, pressure

        if data.len() < 64 {
            return;  // Not enough data for a full report
        }

        // Check report ID - 0x09 is input state, 0x01 is also used sometimes
        let report_id = data[0];
        if report_id != 0x09 && report_id != 0x01 {
            // Not a controller input report, skip
            return;
        }

        // Payload starts after report ID (byte 0)
        // hidapi may or may not include report ID depending on platform
        let offset = if report_id == 0x09 || report_id == 0x01 { 4 } else { 0 };

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

        // Stick clicks
        s.l3_pressed = (buttons_l & STEAMDECK_LBUTTON_L3) != 0;
        s.r3_pressed = (buttons_l & STEAMDECK_LBUTTON_R3) != 0;

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

// Gated to Linux because parse_hid_report (and the whole module) is
// Steam-Deck-only — it isn't compiled on macOS/Windows, so neither are
// these tests. They run under `cargo test` on Linux (CI / on a Deck).
#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;
    use parking_lot::RwLock;
    use std::sync::Arc;

    // Build a 64-byte report. Field offsets are payload-relative + the
    // 4-byte header skip the parser applies (payload = data[4..]), so a
    // payload byte P lives at data index 4 + P. The helpers below take
    // the already-resolved data index.
    fn report(id: u8) -> [u8; 64] {
        let mut b = [0u8; 64];
        b[0] = id;
        b
    }
    fn put_u32(b: &mut [u8; 64], i: usize, v: u32) { b[i..i + 4].copy_from_slice(&v.to_le_bytes()); }
    fn put_i16(b: &mut [u8; 64], i: usize, v: i16) { b[i..i + 2].copy_from_slice(&v.to_le_bytes()); }
    fn put_u16(b: &mut [u8; 64], i: usize, v: u16) { b[i..i + 2].copy_from_slice(&v.to_le_bytes()); }

    // data indices (4 + payload offset)
    const BTN_L: usize = 8;
    const BTN_H: usize = 12;
    const LPAD_X: usize = 16;
    const LPAD_Y: usize = 18;
    const RPAD_X: usize = 20;
    const ACC_X: usize = 24;
    const ACC_Y: usize = 26;
    const ACC_Z: usize = 28;
    const GYRO_X: usize = 30;
    const GYRO_Y: usize = 32;
    const GYRO_Z: usize = 34;
    const PRESS_L: usize = 56;

    fn parse(buf: &[u8]) -> SteamDeckHidState {
        let st = Arc::new(RwLock::new(SteamDeckHidState::default()));
        SteamDeckHidReader::parse_hid_report(buf, &st);
        let out = st.read().clone();
        out
    }
    fn approx(a: f32, b: f32) { assert!((a - b).abs() < 0.02, "expected {b}, got {a}"); }

    #[test]
    fn ignores_short_reports() {
        let st = parse(&[0u8; 32]);
        approx(st.gyro_pitch, 0.0);
        assert!(!st.steam_pressed);
    }

    #[test]
    fn ignores_non_input_report_ids() {
        let mut b = report(0x04); // mouse/keyboard emulation, not input
        put_i16(&mut b, GYRO_X, 16384);
        let st = parse(&b);
        approx(st.gyro_pitch, 0.0); // untouched
    }

    #[test]
    fn decodes_gyro_with_scale_and_yaw_negation() {
        let mut b = report(0x09);
        put_i16(&mut b, GYRO_X, 16384); // pitch
        put_i16(&mut b, GYRO_Y, 16384); // yaw (negated)
        put_i16(&mut b, GYRO_Z, 8192);  // roll
        let st = parse(&b);
        approx(st.gyro_pitch, 1000.0);  // 16384 * 2000/32768
        approx(st.gyro_yaw, -1000.0);   // negated per SDL
        approx(st.gyro_roll, 500.0);
    }

    #[test]
    fn normalizes_touchpads_to_unit_range_with_sign() {
        let mut b = report(0x09);
        put_i16(&mut b, LPAD_X, 16384);   // +0.5
        put_i16(&mut b, LPAD_Y, -16384);  // -0.5
        put_i16(&mut b, RPAD_X, 32767);   // ~+1.0
        let st = parse(&b);
        approx(st.left_pad_x, 0.5);
        approx(st.left_pad_y, -0.5);
        approx(st.right_pad_x, 1.0);
    }

    #[test]
    fn decodes_accel_with_scale_and_y_negation() {
        let mut b = report(0x09);
        put_i16(&mut b, ACC_X, 16384);
        put_i16(&mut b, ACC_Y, 16384); // negated
        put_i16(&mut b, ACC_Z, 8192);
        let st = parse(&b);
        approx(st.accel_x, 9.81);
        approx(st.accel_y, -9.81);
        approx(st.accel_z, 4.905);
    }

    #[test]
    fn pressure_drives_the_click_threshold() {
        let mut hard = report(0x09);
        put_u32(&mut hard, BTN_L, STEAMDECK_LBUTTON_LEFT_PAD);
        put_u16(&mut hard, PRESS_L, 65535); // full pressure → 1.0
        let st = parse(&hard);
        approx(st.left_pad_pressure, 1.0);
        assert!(st.left_pad_touched); // from the LEFT_PAD button bit
        assert!(st.left_pad_clicked); // pressure > 0.5

        let mut soft = report(0x09);
        put_u16(&mut soft, PRESS_L, 16384); // 0.25 → below threshold
        let st2 = parse(&soft);
        assert!(!st2.left_pad_clicked);
    }

    #[test]
    fn decodes_button_masks() {
        let mut b = report(0x09);
        put_u32(&mut b, BTN_L,
            STEAMDECK_LBUTTON_STEAM | STEAMDECK_LBUTTON_L5
            | STEAMDECK_LBUTTON_R5 | STEAMDECK_LBUTTON_L3 | STEAMDECK_LBUTTON_R3);
        put_u32(&mut b, BTN_H,
            STEAMDECK_HBUTTON_L4 | STEAMDECK_HBUTTON_R4 | STEAMDECK_HBUTTON_QAM);
        let st = parse(&b);
        assert!(st.steam_pressed);
        assert!(st.l5_pressed && st.r5_pressed);
        assert!(st.l3_pressed && st.r3_pressed);
        assert!(st.l4_pressed && st.r4_pressed);
        assert!(st.qam_pressed);
    }

    #[test]
    fn report_id_01_is_also_parsed() {
        let mut b = report(0x01);
        put_i16(&mut b, GYRO_X, 16384);
        let st = parse(&b);
        approx(st.gyro_pitch, 1000.0);
    }
}
