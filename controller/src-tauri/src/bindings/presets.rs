use super::config::*;

/// Create the default binding profile
pub fn create_default_profile() -> BindingProfile {
    let mut profile = BindingProfile::new("default", "Default");
    profile.description = Some("Standard robot control layout".to_string());
    profile.is_default = true;

    // Analog bindings
    profile.analog_bindings = vec![
        // Left stick - differential drive for tracks
        // Uses channel mixing: left = throttle + turn, right = throttle - turn
        AnalogBinding {
            input: AnalogInput::LeftStickY,  // Bind to Y axis, both X and Y are used
            action: AnalogAction::DifferentialDrive {
                topic: "/tracks".to_string(),
                left_channel: "left_velocity".to_string(),
                right_channel: "right_velocity".to_string(),
                throttle_transform: InputTransform {
                    deadzone: 0.1,
                    scale: 1.0,
                    expo: 1.0,
                    invert: false,
                },
                turn_transform: InputTransform {
                    deadzone: 0.1,
                    scale: 1.0,
                    expo: 1.0,
                    invert: false,
                },
            },
            enabled: true,
        },
        // Right stick - head control
        AnalogBinding {
            input: AnalogInput::RightStickX,
            action: AnalogAction::DirectControl {
                target: ControlTarget::Topic {
                    topic: "/saint/head".to_string(),
                    channel: "pan".to_string(),
                    name: Some("Head Pan".to_string()),
                },
                transform: InputTransform {
                    deadzone: 0.05,
                    scale: 0.8,
                    expo: 1.0,
                    invert: false,
                },
            },
            enabled: true,
        },
        AnalogBinding {
            input: AnalogInput::RightStickY,
            action: AnalogAction::DirectControl {
                target: ControlTarget::Topic {
                    topic: "/saint/head".to_string(),
                    channel: "tilt".to_string(),
                    name: Some("Head Tilt".to_string()),
                },
                transform: InputTransform {
                    deadzone: 0.05,
                    scale: 0.8,
                    expo: 1.0,
                    invert: false,
                },
            },
            enabled: true,
        },
        // Left trigger - precision mode
        AnalogBinding {
            input: AnalogInput::LeftTrigger,
            action: AnalogAction::Modifier {
                effect: ModifierEffect::PrecisionMode { min_scale: 0.3 },
            },
            enabled: true,
        },
        // Right trigger - speed boost
        AnalogBinding {
            input: AnalogInput::RightTrigger,
            action: AnalogAction::Modifier {
                effect: ModifierEffect::SpeedBoost { max_boost: 1.5 },
            },
            enabled: true,
        },
    ];

    // Digital bindings
    profile.digital_bindings = vec![
        // Y - Show poses panel (server-backed)
        DigitalBinding {
            input: DigitalInput::Y,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::ShowPanel {
                panel_id: "poses".to_string(),
            },
            enabled: true,
        },
        // X - Show animations panel
        DigitalBinding {
            input: DigitalInput::X,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::ShowPanel {
                panel_id: "animations".to_string(),
            },
            enabled: true,
        },
        // A - Select / Confirm
        DigitalBinding {
            input: DigitalInput::A,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::SelectPanelItem,
            enabled: true,
        },
        // B - Close panel / Cancel
        DigitalBinding {
            input: DigitalInput::B,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::HidePanel,
            enabled: true,
        },
        // D-Pad - Grid navigation
        DigitalBinding {
            input: DigitalInput::DPadUp,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::NavigatePanel {
                direction: NavigateDirection::Up,
            },
            enabled: true,
        },
        DigitalBinding {
            input: DigitalInput::DPadDown,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::NavigatePanel {
                direction: NavigateDirection::Down,
            },
            enabled: true,
        },
        DigitalBinding {
            input: DigitalInput::DPadLeft,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::NavigatePanel {
                direction: NavigateDirection::Left,
            },
            enabled: true,
        },
        DigitalBinding {
            input: DigitalInput::DPadRight,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::NavigatePanel {
                direction: NavigateDirection::Right,
            },
            enabled: true,
        },
        // LB - Previous page
        DigitalBinding {
            input: DigitalInput::LB,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::NavigatePanel {
                direction: NavigateDirection::PrevPage,
            },
            enabled: true,
        },
        // RB - Next page
        DigitalBinding {
            input: DigitalInput::RB,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::NavigatePanel {
                direction: NavigateDirection::NextPage,
            },
            enabled: true,
        },
        // Select - E-Stop
        DigitalBinding {
            input: DigitalInput::Select,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::EStop,
            enabled: true,
        },
        // Start - Show sounds panel
        DigitalBinding {
            input: DigitalInput::Start,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::ShowPanel {
                panel_id: "sounds".to_string(),
            },
            enabled: true,
        },
    ];

    // Preset panels. Animations and Poses are server-backed (source
    // set) — their items stream live from the server's library via the
    // frontend's useLibrary, and selecting one fires start_animation /
    // apply_pose. Sounds stays a static local panel for now. (The old
    // hardcoded Moods panel is dropped; Y now opens Poses.)
    profile.preset_panels = vec![
        create_animations_panel(),
        create_poses_panel(),
        create_sounds_panel(),
    ];

    profile
}

fn create_animations_panel() -> PresetPanel {
    PresetPanel {
        id: "animations".to_string(),
        name: "Animations".to_string(),
        icon: "animation".to_string(),
        color: "#8b5cf6".to_string(),
        layout: PanelLayout::Grid,
        columns: 4,
        items_per_page: 8,
        source: Some("animations".to_string()),
        presets: vec![],
    }
}

fn create_poses_panel() -> PresetPanel {
    PresetPanel {
        id: "poses".to_string(),
        name: "Poses".to_string(),
        icon: "accessibility".to_string(),
        color: "#3b82f6".to_string(),
        layout: PanelLayout::Grid,
        columns: 4,
        items_per_page: 8,
        source: Some("poses".to_string()),
        presets: vec![],
    }
}

fn create_sounds_panel() -> PresetPanel {
    PresetPanel {
        id: "sounds".to_string(),
        name: "Sounds".to_string(),
        icon: "volume_up".to_string(),
        color: "#22c55e".to_string(),
        layout: PanelLayout::Grid,
        columns: 4,
        items_per_page: 8,
        source: None,
        presets: vec![
            create_sound_preset("hello", "Hello", "record_voice_over"),
            create_sound_preset("goodbye", "Goodbye", "waving_hand"),
            create_sound_preset("yes", "Yes", "thumb_up"),
            create_sound_preset("no", "No", "thumb_down"),
            create_sound_preset("laugh", "Laugh", "sentiment_very_satisfied"),
            create_sound_preset("alert", "Alert", "notifications"),
            create_sound_preset("error", "Error", "error"),
            create_sound_preset("success", "Success", "check_circle"),
        ],
    }
}

fn create_sound_preset(id: &str, name: &str, icon: &str) -> Preset {
    Preset {
        id: format!("sound_{}", id),
        name: name.to_string(),
        icon: Some(icon.to_string()),
        color: None,
        preset_type: PresetType::Sound,
        data: PresetData::Sound(SoundPresetData {
            sound_id: format!("{}.wav", id),
            volume: 1.0,
            priority: 1,
        }),
    }
}
