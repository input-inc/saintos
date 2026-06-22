use serde::{Deserialize, Serialize};

// ============================================================================
// Input Sources
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum AnalogInput {
    LeftStickX,
    LeftStickY,
    RightStickX,
    RightStickY,
    LeftTrigger,
    RightTrigger,
    // Steam Deck trackpads. X/Y are -1..1, sourced from InputState's
    // left_touchpad/right_touchpad (not gamepad) — see mapper.rs.
    LeftPadX,
    LeftPadY,
    RightPadX,
    RightPadY,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum DigitalInput {
    A,
    B,
    X,
    Y,
    #[serde(rename = "lb")]
    LB,
    #[serde(rename = "rb")]
    RB,
    DPadUp,
    DPadDown,
    DPadLeft,
    DPadRight,
    Start,
    Select,
    LeftStick,
    RightStick,
    // Steam Deck back buttons
    #[serde(rename = "l4")]
    L4,
    #[serde(rename = "r4")]
    R4,
    #[serde(rename = "l5")]
    L5,
    #[serde(rename = "r5")]
    R5,
    Steam,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ButtonTrigger {
    Press,
    Release,
    Hold,
    DoubleTap,
    LongPress,
}

// ============================================================================
// Actions
// ============================================================================

/// Action for analog inputs (sticks, triggers)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum AnalogAction {
    /// Direct control - send value to a robot control
    DirectControl {
        target: ControlTarget,
        transform: InputTransform,
    },
    /// Differential drive - channel mixing for tank drive
    /// Takes stick X (turn) and Y (throttle) and outputs to left/right channels
    /// of the same topic. Mixing: left = throttle + turn, right = throttle - turn
    DifferentialDrive {
        /// ROS topic (e.g., "/tracks")
        topic: String,
        /// Channel field for the left motor (e.g., "left_velocity")
        left_channel: String,
        /// Channel field for the right motor (e.g., "right_velocity")
        right_channel: String,
        /// Transform applied to throttle (Y axis)
        throttle_transform: InputTransform,
        /// Transform applied to turn (X axis)
        turn_transform: InputTransform,
    },
    /// Modifier - affects other inputs while active
    Modifier {
        effect: ModifierEffect,
    },
}

/// Action for digital inputs (buttons, d-pad)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum DigitalAction {
    /// Show a preset panel overlay
    ShowPanel { panel_id: String },
    /// Hide the current panel
    HidePanel,
    /// Activate a specific preset
    ActivatePreset { preset_id: String },
    /// Navigate within an open panel
    NavigatePanel { direction: NavigateDirection },
    /// Select the current item in a panel
    SelectPanelItem,
    /// Toggle another control's output on/off
    ToggleOutput { target_id: String },
    /// Cycle through values for a control
    CycleOutput {
        target_id: String,
        values: Vec<String>,
    },
    /// Direct control with a fixed value
    DirectControl {
        target: ControlTarget,
        value: f32,
    },
    /// Emergency stop
    EStop,
    /// No action
    None,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum NavigateDirection {
    // Grid navigation
    Up,
    Down,
    Left,
    Right,
    // Linear navigation
    NextItem,
    PrevItem,
    NextPage,
    PrevPage,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ModifierEffect {
    /// Scale all outputs by a factor based on input value
    PrecisionMode { min_scale: f32 },
    /// Boost movement speed
    SpeedBoost { max_boost: f32 },
    /// Custom scaling of a specific target
    ScaleTarget { target_id: String, scale: f32 },
}

// ============================================================================
// Targets and Transforms
// ============================================================================

/// What a binding writes into. WsInput addresses a sheet's WebSocket
/// input slot (the new path); Topic addresses a raw ROS topic/channel
/// pair (legacy, kept so existing profiles.json files load unchanged).
///
/// Untagged so the JSON shape on disk decides the variant — old
/// profiles persist with {topic, channel, name} and parse as Topic; new
/// bindings authored against the WS-input picker persist with
/// {sheet_id, input_id, name} and parse as WsInput.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ControlTarget {
    WsInput {
        sheet_id: String,
        input_id: String,
        name: Option<String>,
    },
    Topic {
        topic: String,
        channel: String,
        name: Option<String>,
    },
}

impl ControlTarget {
    #[allow(dead_code)] // accessor kept alongside the ControlTarget variants
    pub fn name(&self) -> Option<&str> {
        match self {
            ControlTarget::Topic { name, .. } | ControlTarget::WsInput { name, .. } =>
                name.as_deref(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputTransform {
    pub deadzone: f32,
    pub scale: f32,
    pub expo: f32,
    pub invert: bool,
}

impl Default for InputTransform {
    fn default() -> Self {
        Self {
            deadzone: 0.1,
            scale: 1.0,
            expo: 1.0,
            invert: false,
        }
    }
}

impl InputTransform {
    pub fn apply(&self, mut value: f32) -> f32 {
        // Apply deadzone
        if value.abs() < self.deadzone {
            return 0.0;
        }

        // Rescale after deadzone
        let sign = value.signum();
        value = (value.abs() - self.deadzone) / (1.0 - self.deadzone);

        // Apply expo curve
        value = value.powf(self.expo);

        // Apply scale and sign
        value = value * self.scale * sign;

        // Apply invert
        if self.invert {
            value = -value;
        }

        value.clamp(-1.0, 1.0)
    }
}

// ============================================================================
// Presets
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum PresetType {
    Servo,
    Animation,
    Sound,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Preset {
    pub id: String,
    pub name: String,
    pub icon: Option<String>,
    pub color: Option<String>,
    #[serde(rename = "type")]
    pub preset_type: PresetType,
    pub data: PresetData,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum PresetData {
    Servo(ServoPresetData),
    Animation(AnimationPresetData),
    Sound(SoundPresetData),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServoPresetData {
    pub positions: Vec<ServoPosition>,
    #[serde(rename = "transitionMs")]
    pub transition_ms: u32,
    pub easing: EasingType,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServoPosition {
    pub topic: String,
    pub channel: String,
    pub value: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnimationPresetData {
    pub keyframes: Vec<AnimationKeyframe>,
    pub loop_animation: bool,
    pub loop_count: Option<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnimationKeyframe {
    #[serde(rename = "timeMs")]
    pub time_ms: u32,
    pub positions: Vec<ServoPosition>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SoundPresetData {
    #[serde(rename = "soundId")]
    pub sound_id: String,
    pub volume: f32,
    pub priority: u32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum EasingType {
    Linear,
    EaseIn,
    EaseOut,
    EaseInOut,
}

// ============================================================================
// Preset Panels
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PresetPanel {
    pub id: String,
    pub name: String,
    pub icon: String,
    pub color: String,
    pub presets: Vec<Preset>,
    pub layout: PanelLayout,
    pub columns: u32,
    #[serde(rename = "itemsPerPage")]
    pub items_per_page: u32,
    /// Live data source for the panel's items. `None`/absent = static
    /// (use `presets`). `"animations"`/`"poses"` = populated at runtime
    /// from the server's library (see the frontend's useLibrary), with
    /// selection triggering start_animation / apply_pose.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum PanelLayout {
    Grid,
    List,
}

// ============================================================================
// Bindings
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalogBinding {
    pub input: AnalogInput,
    pub action: AnalogAction,
    pub enabled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DigitalBinding {
    pub input: DigitalInput,
    pub trigger: ButtonTrigger,
    pub action: DigitalAction,
    pub enabled: bool,
}

// ============================================================================
// Binding Profile
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BindingProfile {
    pub id: String,
    pub name: String,
    pub description: Option<String>,
    #[serde(rename = "isDefault")]
    pub is_default: bool,

    #[serde(rename = "analogBindings")]
    pub analog_bindings: Vec<AnalogBinding>,
    #[serde(rename = "digitalBindings")]
    pub digital_bindings: Vec<DigitalBinding>,
    #[serde(rename = "presetPanels")]
    pub preset_panels: Vec<PresetPanel>,

    pub settings: ProfileSettings,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileSettings {
    #[serde(rename = "globalDeadzone")]
    pub global_deadzone: f32,
    #[serde(rename = "hapticFeedback")]
    pub haptic_feedback: bool,
    #[serde(rename = "doubleTapTimeMs")]
    pub double_tap_time_ms: u32,
    #[serde(rename = "longPressTimeMs")]
    pub long_press_time_ms: u32,
}

impl Default for ProfileSettings {
    fn default() -> Self {
        Self {
            global_deadzone: 0.1,
            haptic_feedback: true,
            double_tap_time_ms: 300,
            long_press_time_ms: 500,
        }
    }
}

impl BindingProfile {
    pub fn new(id: &str, name: &str) -> Self {
        Self {
            id: id.to_string(),
            name: name.to_string(),
            description: None,
            is_default: false,
            analog_bindings: Vec::new(),
            digital_bindings: Vec::new(),
            preset_panels: Vec::new(),
            settings: ProfileSettings::default(),
        }
    }

    #[allow(dead_code)] // lookup helper; mapping currently iterates directly
    pub fn get_analog_binding(&self, input: &AnalogInput) -> Option<&AnalogBinding> {
        self.analog_bindings.iter().find(|b| &b.input == input && b.enabled)
    }

    #[allow(dead_code)] // lookup helper; mapping currently iterates directly
    pub fn get_digital_binding(&self, input: &DigitalInput, trigger: &ButtonTrigger) -> Option<&DigitalBinding> {
        self.digital_bindings
            .iter()
            .find(|b| &b.input == input && &b.trigger == trigger && b.enabled)
    }

    pub fn get_panel(&self, panel_id: &str) -> Option<&PresetPanel> {
        self.preset_panels.iter().find(|p| p.id == panel_id)
    }

    pub fn get_preset(&self, preset_id: &str) -> Option<&Preset> {
        for panel in &self.preset_panels {
            if let Some(preset) = panel.presets.iter().find(|p| p.id == preset_id) {
                return Some(preset);
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    //! Lock down InputTransform::apply — the per-axis movement transform
    //! every analog binding runs raw stick/trigger/pad values through
    //! (deadzone → rescale → expo → scale → sign → invert → clamp). This
    //! is the math that decides how hard the robot actually moves, so a
    //! regression here is a safety/feel bug.
    use super::*;

    fn tf(deadzone: f32, scale: f32, expo: f32, invert: bool) -> InputTransform {
        InputTransform { deadzone, scale, expo, invert }
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }

    #[test]
    fn below_deadzone_is_zero() {
        let t = tf(0.1, 1.0, 1.0, false);
        approx(t.apply(0.0), 0.0);
        approx(t.apply(0.05), 0.0);
        approx(t.apply(-0.099), 0.0);
        // Exactly at the deadzone edge maps to 0 (rescale numerator is 0).
        approx(t.apply(0.1), 0.0);
    }

    #[test]
    fn full_deflection_maps_to_unit() {
        let t = tf(0.1, 1.0, 1.0, false);
        approx(t.apply(1.0), 1.0);
        approx(t.apply(-1.0), -1.0);
    }

    #[test]
    fn rescales_above_deadzone() {
        // (0.55 - 0.1) / (1 - 0.1) = 0.45 / 0.9 = 0.5
        let t = tf(0.1, 1.0, 1.0, false);
        approx(t.apply(0.55), 0.5);
    }

    #[test]
    fn sign_is_preserved() {
        let t = tf(0.1, 1.0, 1.0, false);
        approx(t.apply(-0.55), -0.5);
    }

    #[test]
    fn expo_curve_softens_midrange() {
        // deadzone 0 so rescale is identity: 0.5^2 = 0.25, sign reapplied.
        let t = tf(0.0, 1.0, 2.0, false);
        approx(t.apply(0.5), 0.25);
        approx(t.apply(-0.5), -0.25);
        // Endpoints stay put under any expo.
        approx(t.apply(1.0), 1.0);
        approx(t.apply(-1.0), -1.0);
    }

    #[test]
    fn scale_multiplies_output() {
        let t = tf(0.0, 0.5, 1.0, false);
        approx(t.apply(1.0), 0.5);
        approx(t.apply(0.4), 0.2);
    }

    #[test]
    fn invert_flips_sign() {
        let t = tf(0.0, 1.0, 1.0, true);
        approx(t.apply(0.5), -0.5);
        approx(t.apply(-0.5), 0.5);
    }

    #[test]
    fn output_is_clamped_to_unit_range() {
        // scale > 1 would push past ±1 without the final clamp.
        let t = tf(0.0, 5.0, 1.0, false);
        approx(t.apply(1.0), 1.0);
        approx(t.apply(-1.0), -1.0);
        approx(t.apply(0.5), 1.0);   // 0.5 * 5 = 2.5 → clamp 1.0
    }

    #[test]
    fn default_transform_is_light_deadzone_unity_gain() {
        let t = InputTransform::default();
        approx(t.apply(0.05), 0.0);   // inside 0.1 deadzone
        approx(t.apply(1.0), 1.0);    // unity gain at full deflection
        assert!(!t.invert);
    }
}
