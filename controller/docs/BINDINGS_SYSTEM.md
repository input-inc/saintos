# Controller Bindings System

## Overview

The bindings system allows users to map gamepad inputs (buttons, triggers, sticks, D-pad) to various actions that control the robot. This includes direct control, preset activation, panel navigation, and output modulation.

## Input Sources

### Analog Inputs (Continuous Values -1.0 to 1.0 or 0.0 to 1.0)
- **Left Stick X/Y** - Movement control (linear/angular velocity)
- **Right Stick X/Y** - Head/camera control (pan/tilt)
- **Left Trigger (LT)** - Analog input (0.0 to 1.0)
- **Right Trigger (RT)** - Analog input (0.0 to 1.0)

### Digital Inputs (Press/Release)
- **Face Buttons** - A, B, X, Y
- **Shoulder Buttons** - LB, RB
- **D-Pad** - Up, Down, Left, Right
- **Stick Buttons** - Left Stick Press, Right Stick Press
- **Menu Buttons** - Start, Select

## Action Types

### 1. Direct Control
Map analog inputs directly to robot controls with transform options.

```
Input: Left Stick Y
Target: Track Linear Velocity
Transform:
  - Deadzone: 0.1
  - Scale: 1.0
  - Expo: 0.0 (linear) to 1.0 (exponential)
  - Invert: false
```

### 2. Show Preset Panel
Opens a preset panel overlay when pressed. Panels contain presets organized in pages.

```
Input: Y Button (Press)
Action: Show Preset Panel
Panel: "Moods"
```

**Panel Types:**
- **Moods** - Emotional expression presets (happy, sad, angry, curious, etc.)
- **Animations** - Canned animation sequences (wave, nod, shake head, dance, etc.)
- **Poses** - Static servo position presets (look left, look up, neutral, etc.)
- **Sounds** - Audio clips to play (greetings, acknowledgments, alerts, etc.)
- **Custom** - User-defined preset groups

### 3. Activate Preset
Directly activates a specific preset without showing a panel.

```
Input: A Button (Press)
Action: Activate Preset
Preset: "Wave"
Category: "Animations"
```

### 4. Navigate Preset Panel
When a panel is open, navigate between pages or items.

```
Input: D-Pad Left/Right
Action: Navigate Preset Panel
Direction: Previous/Next Page

Input: D-Pad Up/Down
Action: Navigate Preset Panel
Direction: Previous/Next Item
```

### 5. Toggle Control Output
Toggles another control's output on/off. Useful for enabling/disabling features.

```
Input: LB Button (Press)
Action: Toggle Control Output
Target: "Head Tracking"
```

### 6. Cycle Control Output
Cycles through predefined values for a control.

```
Input: RB Button (Press)
Action: Cycle Control Output
Target: "Speed Mode"
Values: ["Slow", "Normal", "Fast"]
```

### 7. Modifier
Modifies other inputs while held (e.g., precision mode, speed boost).

```
Input: Left Trigger (Analog)
Action: Modifier
Effect: Scale other inputs by (1.0 - LT * 0.7)  # Precision mode
```

## Preset System

### Preset Definition

```typescript
interface Preset {
  id: string;
  name: string;
  icon?: string;           // Material icon name
  color?: string;          // Accent color
  type: PresetType;
  data: ServoPreset | AnimationPreset | SoundPreset;
}

type PresetType = 'servo' | 'animation' | 'sound';

interface ServoPreset {
  positions: {
    nodeId: string;        // Servo node ID
    pinId: number;
    value: number;         // Target position (-1.0 to 1.0)
  }[];
  transitionMs: number;    // Transition duration
  easing: 'linear' | 'ease-in' | 'ease-out' | 'ease-in-out';
}

interface AnimationPreset {
  keyframes: {
    timeMs: number;
    positions: { nodeId: string; pinId: number; value: number; }[];
  }[];
  loop: boolean;
  loopCount?: number;      // undefined = infinite when loop=true
}

interface SoundPreset {
  soundId: string;         // Reference to sound file on robot
  volume: number;          // 0.0 to 1.0
  priority: number;        // For sound queue management
}
```

### Preset Panel Definition

```typescript
interface PresetPanel {
  id: string;
  name: string;
  icon: string;
  color: string;
  presets: Preset[];
  layout: 'grid' | 'list';
  columns: number;         // For grid layout
  itemsPerPage: number;
}
```

## Binding Profile

A complete binding configuration that can be saved/loaded.

```typescript
interface BindingProfile {
  id: string;
  name: string;
  description?: string;
  isDefault: boolean;

  // Analog bindings (sticks, triggers)
  analogBindings: AnalogBinding[];

  // Digital bindings (buttons, d-pad)
  digitalBindings: DigitalBinding[];

  // Preset panels available in this profile
  presetPanels: PresetPanel[];

  // Global settings
  settings: {
    globalDeadzone: number;
    hapticFeedback: boolean;
    doubleTapTimeMs: number;
    longPressTimeMs: number;
  };
}

interface AnalogBinding {
  inputSource: AnalogInputSource;
  action: DirectControlAction | ModifierAction;
}

interface DigitalBinding {
  inputSource: DigitalInputSource;
  trigger: 'press' | 'release' | 'hold' | 'double-tap' | 'long-press';
  action:
    | ShowPanelAction
    | ActivatePresetAction
    | NavigatePanelAction
    | ToggleOutputAction
    | CycleOutputAction
    | DirectControlAction;
}
```

## UI Components

### Bindings Page (`/bindings`)

```
┌─────────────────────────────────────────────────────────────┐
│ Bindings                                    [Profile ▼]     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐  ┌─────────────────────────────────────┐  │
│  │   ANALOG    │  │  Left Stick X                       │  │
│  │             │  │  → Track Angular Velocity           │  │
│  │  Left Stick │  │  Deadzone: 0.1  Scale: 1.0         │  │
│  │  Right Stick│  ├─────────────────────────────────────┤  │
│  │  LT / RT    │  │  Left Stick Y                       │  │
│  │             │  │  → Track Linear Velocity            │  │
│  ├─────────────┤  │  Deadzone: 0.1  Scale: 1.0         │  │
│  │   BUTTONS   │  ├─────────────────────────────────────┤  │
│  │             │  │  Right Stick X                      │  │
│  │  A B X Y    │  │  → Head Pan                         │  │
│  │  LB / RB    │  │  Deadzone: 0.05  Scale: 0.8        │  │
│  │  D-Pad      │  └─────────────────────────────────────┘  │
│  │  Start/Sel  │                                           │
│  └─────────────┘                                           │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ PRESET PANELS                           [+ Add Panel] │  │
│  │                                                        │  │
│  │  🎭 Moods (8 presets)      📢 Sounds (12 presets)    │  │
│  │  🎬 Animations (6 presets)  📐 Poses (10 presets)    │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### Preset Panel Overlay (shown during gameplay)

```
┌─────────────────────────────────────────────────────────────┐
│                         Moods                    Page 1/2   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│    ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐     │
│    │  😊     │  │  😢     │  │  😠     │  │  🤔     │     │
│    │ Happy   │  │  Sad    │  │ Angry   │  │ Curious │     │
│    └─────────┘  └─────────┘  └─────────┘  └─────────┘     │
│                                                             │
│    ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐     │
│    │  😴     │  │  😮     │  │  😍     │  │  😎     │     │
│    │ Sleepy  │  │Surprised│  │  Love   │  │  Cool   │     │
│    └─────────┘  └─────────┘  └─────────┘  └─────────┘     │
│                                                             │
│  [D-Pad: Navigate]  [A: Select]  [B: Close]               │
└─────────────────────────────────────────────────────────────┘
```

## Default Binding Profile

```yaml
name: "Default"
description: "Standard robot control layout"

analogBindings:
  - input: LeftStickX
    action: DirectControl
    target: track_angular_velocity
    deadzone: 0.1
    scale: 1.0

  - input: LeftStickY
    action: DirectControl
    target: track_linear_velocity
    deadzone: 0.1
    scale: 1.0

  - input: RightStickX
    action: DirectControl
    target: head_pan
    deadzone: 0.05
    scale: 0.8

  - input: RightStickY
    action: DirectControl
    target: head_tilt
    deadzone: 0.05
    scale: 0.8

  - input: LeftTrigger
    action: Modifier
    effect: precision_mode  # Reduces all outputs by 70%

  - input: RightTrigger
    action: Modifier
    effect: speed_boost     # Increases movement speed by 50%

digitalBindings:
  - input: A
    trigger: press
    action: ActivatePreset
    preset: "acknowledge"
    category: "sounds"

  - input: B
    trigger: press
    action: ClosePanelOrEStop

  - input: X
    trigger: press
    action: ShowPanel
    panel: "animations"

  - input: Y
    trigger: press
    action: ShowPanel
    panel: "moods"

  - input: LB
    trigger: press
    action: CycleOutput
    target: "speed_mode"
    values: ["slow", "normal", "fast"]

  - input: RB
    trigger: press
    action: ToggleOutput
    target: "head_tracking"

  - input: DPadUp
    trigger: press
    action: NavigatePanel
    direction: "prev_item"

  - input: DPadDown
    trigger: press
    action: NavigatePanel
    direction: "next_item"

  - input: DPadLeft
    trigger: press
    action: NavigatePanel
    direction: "prev_page"

  - input: DPadRight
    trigger: press
    action: NavigatePanel
    direction: "next_page"

  - input: Select
    trigger: press
    action: EStop

  - input: Start
    trigger: press
    action: ShowPanel
    panel: "quick_menu"
```

## Implementation Files

### Rust Backend
- `src-tauri/src/bindings/actions.rs` - Action type definitions
- `src-tauri/src/bindings/presets.rs` - Preset and panel structures
- `src-tauri/src/bindings/profile.rs` - Binding profile management
- `src-tauri/src/bindings/executor.rs` - Action execution engine

### Angular Frontend
- `src/app/features/bindings/` - Bindings editor UI
- `src/app/shared/components/preset-panel/` - Preset panel overlay
- `src/app/core/services/bindings.service.ts` - Bindings state management
- `src/app/core/services/preset.service.ts` - Preset execution
