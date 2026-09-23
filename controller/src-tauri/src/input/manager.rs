use super::gamepad::{GamepadHandler, GamepadState};
use super::gyro::{GyroHandler, GyroState, TouchpadState};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tauri::{AppHandle, Emitter, Runtime};
use std::thread;

#[cfg(target_os = "linux")]
use super::steamdeck_hid::SteamDeckHidReader;

/// Longest gap between `input-state` emits while nothing is moving.
///
/// The emit loop is change-gated, so a resting controller publishes
/// nothing at all. That leaves one hole: a listener attaching late (a
/// webview reload, or ControllerView mounting minutes after boot) would
/// sit on stale data until the operator next touches a control. This
/// floor bounds that wait at 1 s. At 1 Hz it costs nothing.
const IDLE_HEARTBEAT_MS: u64 = 1000;

/// Tick period while the window is not focused. Nothing is rendering, so
/// there is nothing to publish; this only has to notice focus returning.
const UNFOCUSED_TICK_MS: u64 = 100;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct InputState {
    pub gamepad: GamepadState,
    pub gyro: GyroState,
    #[serde(rename = "leftTouchpad")]
    pub left_touchpad: TouchpadState,
    #[serde(rename = "rightTouchpad")]
    pub right_touchpad: TouchpadState,
}

/// Who currently wants IMU data. The sensor is powered iff at least one
/// of these is true — it draws battery the whole time it streams, and
/// for most of a session nothing needs it.
///
/// Two independent sources, tracked separately because either can change
/// without the other: the operator can open the diagnostics view while a
/// gyro binding is active, or edit bindings while that view is open. A
/// single bool would let whichever fired last clobber the other.
#[derive(Default)]
struct GyroDemand {
    /// The active profile has an enabled binding on a gyro axis.
    bindings: bool,
    /// The Controller diagnostics view is on screen showing live rates.
    diagnostics: bool,
}

impl GyroDemand {
    fn any(&self) -> bool {
        self.bindings || self.diagnostics
    }
}

/// Pick which reader's view of one trackpad to use.
///
/// Both the HID reports (steamdeck_hid.rs) and a separate evdev node
/// (gyro.rs) carry the pads; the evdev path is the fallback for hosts
/// where the HID probe bails, which it does silently in three places.
/// HID wins whenever it has something to say — a touch in progress, or
/// a position left over from one.
///
/// Extracted because this decision was written out four times (two pads
/// x the emit thread and get_state) and the copies had DRIFTED: the emit
/// thread tested `touched || x != 0 || y != 0` while get_state tested
/// `touched` alone. So on release the UI read the pad off HID and the
/// bindings mapper read it off evdev — two different stale positions
/// from one lift. One function, one answer, both callers.
///
/// Linux-only: both callers sit behind the same cfg, since the HID
/// reader only exists there.
#[cfg(target_os = "linux")]
fn select_touchpad(
    hid_x: f32,
    hid_y: f32,
    hid_touched: bool,
    hid_clicked: bool,
    evdev: TouchpadState,
) -> TouchpadState {
    if hid_touched || hid_x != 0.0 || hid_y != 0.0 {
        TouchpadState {
            x: hid_x,
            y: hid_y,
            touched: hid_touched,
            clicked: hid_clicked,
        }
    } else {
        evdev
    }
}

pub struct InputManager {
    gamepad: GamepadHandler,
    gyro: GyroHandler,
    #[cfg(target_os = "linux")]
    steamdeck_hid: SteamDeckHidReader,
    running: Arc<RwLock<bool>>,
    gyro_demand: Arc<RwLock<GyroDemand>>,
}

impl InputManager {
    pub fn new() -> Self {
        Self {
            gamepad: GamepadHandler::new(),
            gyro: GyroHandler::new(),
            #[cfg(target_os = "linux")]
            steamdeck_hid: SteamDeckHidReader::new(),
            running: Arc::new(RwLock::new(false)),
            gyro_demand: Arc::new(RwLock::new(GyroDemand::default())),
        }
    }

    /// Record that the active binding profile does (or no longer does)
    /// drive something from the IMU. Call on every profile edit and
    /// profile switch.
    pub fn set_gyro_binding_demand(&self, wanted: bool) {
        let changed = {
            let mut d = self.gyro_demand.write();
            let before = d.any();
            d.bindings = wanted;
            before != d.any()
        };
        if changed {
            self.apply_gyro_demand();
        }
    }

    /// Record that the Controller diagnostics view is (or is no longer)
    /// on screen. That view renders live gyro rates, so it needs the
    /// sensor even with nothing bound.
    pub fn set_gyro_diagnostic_demand(&self, wanted: bool) {
        let changed = {
            let mut d = self.gyro_demand.write();
            let before = d.any();
            d.diagnostics = wanted;
            before != d.any()
        };
        if changed {
            self.apply_gyro_demand();
        }
    }

    fn apply_gyro_demand(&self) {
        let wanted = self.gyro_demand.read().any();
        log::info!(
            "IMU demand changed -> {}",
            if wanted { "on" } else { "off" }
        );
        #[cfg(target_os = "linux")]
        self.steamdeck_hid.set_sensors_enabled(wanted);
    }

    /// Spawn the thread that publishes `input-state` to the frontend.
    ///
    /// `emit_interval_ms` is the UI publish cadence and is deliberately
    /// NOT the input sample rate — the gamepad/HID/evdev readers write
    /// into shared state on their own schedules, and the bindings
    /// processing loop in lib.rs reads that state at INPUT_LOOP_MS. This
    /// thread only decides how often the *UI* is told. Nothing on screen
    /// can render faster than the display, so publishing above ~60 Hz
    /// buys nothing and costs a JSON serialize + an IPC hop + a webview
    /// wake-up every time.
    /// `focused` gates the whole pipeline: while it is false the emit
    /// thread publishes nothing and the HID reader stops polling. A
    /// hidden webview cannot render an input frame, so producing one is
    /// pure cost.
    pub fn start<R: Runtime + 'static>(
        &self,
        app_handle: AppHandle<R>,
        emit_interval_ms: u64,
        focused: Arc<AtomicBool>,
    ) {
        let gamepad_state = self.gamepad.state();
        let gyro_state = self.gyro.state();
        let extras_state = self.gyro.extras_state();
        let running = self.running.clone();
        let gyro_demand = self.gyro_demand.clone();

        // Start Steam Deck HID reader on Linux
        #[cfg(target_os = "linux")]
        {
            self.steamdeck_hid.start(focused.clone());
        }

        #[cfg(target_os = "linux")]
        let hid_state = self.steamdeck_hid.state();

        *running.write() = true;

        // Spawn a thread for emitting events to the frontend
        thread::spawn(move || {
            log::info!("Input emit thread started");

            // Change-gate state. `last_emitted` is compared against each
            // freshly-built frame; identical frames are dropped rather
            // than serialized and shipped across IPC. Seeded so the very
            // first pass always emits.
            let mut last_emitted: Option<InputState> = None;
            let mut last_emit_at = Instant::now() - Duration::from_millis(IDLE_HEARTBEAT_MS);

            while *running.read() {
                if !focused.load(Ordering::Relaxed) {
                    // Backgrounded: nothing to publish to. Drop the
                    // change-gate memory so the first frame after refocus
                    // is always emitted rather than being suppressed as
                    // "unchanged" against a pre-blur snapshot.
                    last_emitted = None;
                    thread::sleep(Duration::from_millis(UNFOCUSED_TICK_MS));
                    continue;
                }

                // `mut` is only needed on Linux, where the HID merge below
                // inserts back-button/stick-click states; allow it so the
                // non-Linux build doesn't warn about an unused `mut`.
                #[allow(unused_mut)]
                let mut gamepad = gamepad_state.read().clone();

                // On Linux, merge HID data for back buttons, gyro, and touchpads
                #[cfg(target_os = "linux")]
                let (gyro, left_touchpad, right_touchpad) = {
                    let hid = hid_state.read();

                    // Merge back button and stick click states from HID into gamepad buttons
                    // Always set the state (true or false) so buttons properly release
                    gamepad.buttons.insert("L4".to_string(), hid.l4_pressed);
                    gamepad.buttons.insert("R4".to_string(), hid.r4_pressed);
                    gamepad.buttons.insert("L5".to_string(), hid.l5_pressed);
                    gamepad.buttons.insert("R5".to_string(), hid.r5_pressed);
                    gamepad.buttons.insert("L3".to_string(), hid.l3_pressed);
                    gamepad.buttons.insert("R3".to_string(), hid.r3_pressed);
                    gamepad.buttons.insert("Steam".to_string(), hid.steam_pressed);
                    gamepad.buttons.insert("QAM".to_string(), hid.qam_pressed);

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
                    let left_touchpad = select_touchpad(
                        hid.left_pad_x,
                        hid.left_pad_y,
                        hid.left_pad_touched,
                        hid.left_pad_clicked,
                        extras_state.read().left_touchpad.clone(),
                    );

                    let right_touchpad = select_touchpad(
                        hid.right_pad_x,
                        hid.right_pad_y,
                        hid.right_pad_touched,
                        hid.right_pad_clicked,
                        extras_state.read().right_touchpad.clone(),
                    );

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

                let mut state = InputState {
                    gamepad,
                    gyro,
                    left_touchpad,
                    right_touchpad,
                };

                // Enforce the demand gate here rather than at each
                // reader. Gyro arrives from TWO independent sources —
                // the HID reports (steamdeck_hid.rs) and a separate
                // evdev IMU node (gyro.rs) — and only the first can be
                // switched off with a feature report. Zeroing at the
                // merge makes "nothing is bound to the IMU" mean no
                // IMU data at all, whichever reader produced it,
                // instead of a contract that holds for one path only.
                if !gyro_demand.read().any() {
                    state.gyro = GyroState::default();
                }

                // Only publish when something actually moved. A
                // controller sitting untouched produces byte-identical
                // frames, and the UI already shows that state — emitting
                // it again wakes the webview to redraw nothing.
                let changed = last_emitted.as_ref() != Some(&state);
                let heartbeat_due =
                    last_emit_at.elapsed() >= Duration::from_millis(IDLE_HEARTBEAT_MS);

                if changed || heartbeat_due {
                    if let Err(e) = app_handle.emit("input-state", &state) {
                        log::error!("Failed to emit input state: {}", e);
                    }
                    last_emit_at = Instant::now();
                    last_emitted = Some(state);
                }

                thread::sleep(Duration::from_millis(emit_interval_ms));
            }
            log::info!("Input manager stopped");
        });

        log::info!(
            "Input manager started with {}ms UI emit interval (change-gated, \
             {}ms idle heartbeat)",
            emit_interval_ms,
            IDLE_HEARTBEAT_MS
        );
    }

    #[allow(dead_code)] // lifecycle hook; the input thread runs for the app's lifetime
    pub fn stop(&self) {
        *self.running.write() = false;
        #[cfg(target_os = "linux")]
        self.steamdeck_hid.stop();
    }

    pub fn get_state(&self) -> InputState {
        // `mut` is only needed on Linux (HID button merge below).
        #[allow(unused_mut)]
        let mut gamepad = self.gamepad.get_state();

        #[cfg(target_os = "linux")]
        let (gyro, left_touchpad, right_touchpad) = {
            let hid = self.steamdeck_hid.get_state();

            // Merge back button and stick click states
            gamepad.buttons.insert("L4".to_string(), hid.l4_pressed);
            gamepad.buttons.insert("R4".to_string(), hid.r4_pressed);
            gamepad.buttons.insert("L5".to_string(), hid.l5_pressed);
            gamepad.buttons.insert("R5".to_string(), hid.r5_pressed);
            gamepad.buttons.insert("L3".to_string(), hid.l3_pressed);
            gamepad.buttons.insert("R3".to_string(), hid.r3_pressed);
            gamepad.buttons.insert("Steam".to_string(), hid.steam_pressed);
            gamepad.buttons.insert("QAM".to_string(), hid.qam_pressed);

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
            let left_touchpad = select_touchpad(
                hid.left_pad_x,
                hid.left_pad_y,
                hid.left_pad_touched,
                hid.left_pad_clicked,
                extras.left_touchpad,
            );

            let right_touchpad = select_touchpad(
                hid.right_pad_x,
                hid.right_pad_y,
                hid.right_pad_touched,
                hid.right_pad_clicked,
                extras.right_touchpad,
            );

            (gyro, left_touchpad, right_touchpad)
        };

        #[cfg(not(target_os = "linux"))]
        let (gyro, left_touchpad, right_touchpad) = {
            let extras = self.gyro.get_extras();
            (self.gyro.get_state(), extras.left_touchpad, extras.right_touchpad)
        };

        InputState {
            gamepad,
            // Same demand gate as the emit loop — this is the path the
            // bindings mapper reads, so an ungated read here would let
            // the evdev IMU drive a binding while the sensor is
            // supposed to be off.
            gyro: if self.gyro_demand.read().any() { gyro } else { GyroState::default() },
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::gamepad::AxisState;

    fn resting() -> InputState {
        InputState {
            gamepad: GamepadState::default(),
            gyro: GyroState::default(),
            left_touchpad: TouchpadState::default(),
            right_touchpad: TouchpadState::default(),
        }
    }

    /// The emit loop drops a frame when it equals the last one it sent.
    /// That only saves anything if an untouched controller really does
    /// produce equal frames — if any field carried a timestamp, a
    /// counter, or NaN, every frame would differ and the gate would be
    /// a no-op that still paid for the comparison.
    #[test]
    fn identical_frames_compare_equal() {
        assert_eq!(resting(), resting(), "a resting controller must gate");
    }

    #[test]
    fn stick_movement_breaks_equality() {
        let mut moved = resting();
        moved.gamepad.left_stick = AxisState { x: 0.5, y: 0.0 };
        assert_ne!(resting(), moved, "a moved stick must always be published");
    }

    #[test]
    fn button_press_breaks_equality() {
        let mut pressed = resting();
        pressed.gamepad.buttons.insert("A".to_string(), true);
        assert_ne!(resting(), pressed, "a button edge must always be published");
    }

    /// Button maps are compared by content, not insertion order — the
    /// HID merge in the emit loop inserts L4/R4/L5/R5/L3/R3/Steam/QAM on
    /// every pass, and HashMap iteration order is not stable.
    #[test]
    fn button_map_ordering_does_not_affect_equality() {
        let mut a = resting();
        a.gamepad.buttons.insert("A".to_string(), false);
        a.gamepad.buttons.insert("L4".to_string(), true);

        let mut b = resting();
        b.gamepad.buttons.insert("L4".to_string(), true);
        b.gamepad.buttons.insert("A".to_string(), false);

        assert_eq!(a, b, "insertion order must not force a spurious emit");
    }
}
