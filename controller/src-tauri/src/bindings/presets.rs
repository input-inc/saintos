use super::config::*;

/// Create the default binding profile
pub fn create_default_profile() -> BindingProfile {
    let mut profile = BindingProfile::new("default", "Default");
    profile.description = Some("Standard robot control layout".to_string());
    profile.is_default = true;

    // Analog bindings
    profile.analog_bindings = vec![
        // Left stick - movement
        AnalogBinding {
            input: AnalogInput::LeftStickX,
            action: AnalogAction::DirectControl {
                target: ControlTarget {
                    node_id: "track".to_string(),
                    pin_id: 1, // angular velocity
                    name: Some("Track Angular".to_string()),
                },
                transform: InputTransform {
                    deadzone: 0.1,
                    scale: 1.0,
                    expo: 1.0,
                    invert: false,
                },
            },
            enabled: true,
        },
        AnalogBinding {
            input: AnalogInput::LeftStickY,
            action: AnalogAction::DirectControl {
                target: ControlTarget {
                    node_id: "track".to_string(),
                    pin_id: 0, // linear velocity
                    name: Some("Track Linear".to_string()),
                },
                transform: InputTransform {
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
                target: ControlTarget {
                    node_id: "head".to_string(),
                    pin_id: 0, // pan
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
                target: ControlTarget {
                    node_id: "head".to_string(),
                    pin_id: 1, // tilt
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
        // Y - Show moods panel
        DigitalBinding {
            input: DigitalInput::Y,
            trigger: ButtonTrigger::Press,
            action: DigitalAction::ShowPanel {
                panel_id: "moods".to_string(),
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

    // Preset panels
    profile.preset_panels = vec![
        create_moods_panel(),
        create_animations_panel(),
        create_sounds_panel(),
        create_poses_panel(),
    ];

    profile
}

fn create_moods_panel() -> PresetPanel {
    PresetPanel {
        id: "moods".to_string(),
        name: "Moods".to_string(),
        icon: "mood".to_string(),
        color: "#f59e0b".to_string(),
        layout: PanelLayout::Grid,
        columns: 4,
        items_per_page: 8,
        presets: vec![
            // Page 1
            create_mood_preset("happy", "Happy", "sentiment_very_satisfied", "#22c55e"),
            create_mood_preset("sad", "Sad", "sentiment_dissatisfied", "#3b82f6"),
            create_mood_preset("angry", "Angry", "mood_bad", "#ef4444"),
            create_mood_preset("curious", "Curious", "psychology", "#8b5cf6"),
            create_mood_preset("sleepy", "Sleepy", "bedtime", "#6b7280"),
            create_mood_preset("surprised", "Surprised", "sentiment_excited", "#f59e0b"),
            create_mood_preset("love", "Love", "favorite", "#ec4899"),
            create_mood_preset("neutral", "Neutral", "sentiment_neutral", "#9ca3af"),
            // Page 2
            create_mood_preset("excited", "Excited", "celebration", "#f97316"),
            create_mood_preset("confused", "Confused", "help", "#a855f7"),
            create_mood_preset("scared", "Scared", "warning", "#facc15"),
            create_mood_preset("proud", "Proud", "military_tech", "#eab308"),
            create_mood_preset("shy", "Shy", "face_retouching_off", "#f472b6"),
            create_mood_preset("bored", "Bored", "sentiment_dissatisfied", "#78716c"),
            create_mood_preset("playful", "Playful", "toys", "#06b6d4"),
            create_mood_preset("focused", "Focused", "center_focus_strong", "#0ea5e9"),
        ],
    }
}

fn create_mood_preset(id: &str, name: &str, icon: &str, color: &str) -> Preset {
    Preset {
        id: format!("mood_{}", id),
        name: name.to_string(),
        icon: Some(icon.to_string()),
        color: Some(color.to_string()),
        preset_type: PresetType::Servo,
        data: PresetData::Servo(ServoPresetData {
            positions: vec![],
            transition_ms: 500,
            easing: EasingType::EaseInOut,
        }),
    }
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
        presets: vec![
            create_animation_preset("wave", "Wave", "waving_hand"),
            create_animation_preset("nod", "Nod", "check_circle"),
            create_animation_preset("shake", "Shake Head", "cancel"),
            create_animation_preset("dance", "Dance", "music_note"),
            create_animation_preset("look_around", "Look Around", "visibility"),
            create_animation_preset("bow", "Bow", "arrow_downward"),
        ],
    }
}

fn create_animation_preset(id: &str, name: &str, icon: &str) -> Preset {
    Preset {
        id: format!("anim_{}", id),
        name: name.to_string(),
        icon: Some(icon.to_string()),
        color: None,
        preset_type: PresetType::Animation,
        data: PresetData::Animation(AnimationPresetData {
            keyframes: vec![],
            loop_animation: false,
            loop_count: None,
        }),
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

fn create_poses_panel() -> PresetPanel {
    PresetPanel {
        id: "poses".to_string(),
        name: "Poses".to_string(),
        icon: "accessibility".to_string(),
        color: "#3b82f6".to_string(),
        layout: PanelLayout::Grid,
        columns: 4,
        items_per_page: 8,
        presets: vec![
            create_pose_preset("center", "Center", "center_focus_strong"),
            create_pose_preset("look_left", "Look Left", "arrow_back"),
            create_pose_preset("look_right", "Look Right", "arrow_forward"),
            create_pose_preset("look_up", "Look Up", "arrow_upward"),
            create_pose_preset("look_down", "Look Down", "arrow_downward"),
            create_pose_preset("tilt_left", "Tilt Left", "rotate_left"),
            create_pose_preset("tilt_right", "Tilt Right", "rotate_right"),
        ],
    }
}

fn create_pose_preset(id: &str, name: &str, icon: &str) -> Preset {
    Preset {
        id: format!("pose_{}", id),
        name: name.to_string(),
        icon: Some(icon.to_string()),
        color: None,
        preset_type: PresetType::Servo,
        data: PresetData::Servo(ServoPresetData {
            positions: vec![],
            transition_ms: 300,
            easing: EasingType::EaseOut,
        }),
    }
}
