# Wheel-E - System Change Document

## Background

This VESC package (Refloat/Wheel-E) was originally developed for onewheels and self-balancing skateboards. This change adapts it for use in a **self-balancing electric bike** in the style of the Future Motion Antic and The Float Life WFB.

The bike balances on its **rear wheel only**, with the front wheel lifted (wheelie). The fore-aft pitch axis and PID loop are identical to the onewheel use case — no IMU axis changes were needed. Lateral (left/right) balance is handled by the rider; the controller does not attempt to compensate for roll.

---

## Change Specification (as provided)

### 1. Throttle and brake inputs
- **ADC1** = analog throttle (0–1 mapped to 0–`throttle_current_max`)
- **ADC2** = analog regenerative brake (0–1 mapped to 0–`throttle_brake_current_max`)
- Both inputs use a 5% deadband to avoid creep at rest

### 2. Wheelie entry trigger
- The balance/wheelie loop engages automatically when pitch rises to within `wheelie_entry_threshold` degrees below `wheelie_target_pitch`
- Example: target = 20°, threshold = 5° → balance loop engages at 15°
- Both `wheelie_target_pitch` and `wheelie_entry_threshold` are user-configurable parameters
- Throttle input is **ignored** while in wheelie/balance mode; only leaning affects speed

### 3. Wheelie exit
- Pressing the brake (ADC2 > 5%) while in wheelie mode:
  1. Immediately applies a small forward braking current (`wheelie_exit_brake_current`) to ensure the nose tips down and the front wheel lands first
  2. Returns to normal throttle mode (`STATE_THROTTLE`)
- Throttle (ADC1) is ignored in wheelie mode

### 4. Safety / rider presence
- No footpad/rider-presence detection — the bike uses an emergency-stop safety leash (deadman switch) like outboard motors
- Footpad thresholds should be set to 0 in config to bypass all footpad logic

---

## Rationale

### Why the existing PID loop is reused unchanged
The onewheel and this rear-wheel-balance bike share the same physical problem: balancing a single contact point on a fore-aft pitch axis. The Mahony filter, PID gains, ATR, TorqueTilt, BrakeTilt, and booster all apply directly. Only the **target setpoint** changes (from 0° to `wheelie_target_pitch`).

### Why a separate `STATE_THROTTLE` rather than using `STATE_READY`
`STATE_READY` has logic that constantly checks for engage conditions (pitch tolerance, footpad state) and can run an RC idle move. A dedicated `STATE_THROTTLE` state gives the normal riding mode a clean, unambiguous identity in the state machine and avoids coupling with onewheel-specific startup conditions. It also makes the motor control layer aware that the motor should be actively driven (no parking brake).

### Why `setpoint_target = wheelie_target_pitch` not 0 during running
The original code sets `setpoint_target = 0` in the `SAT_NONE` (normal running) branch. For a bike balanced at ~20°, the PID would immediately try to pitch the bike back to level (0°). Changing the normal running target to `wheelie_target_pitch` means the balance loop holds the configured wheelie angle as its equilibrium.

### Why a negative current on wheelie exit
When exiting wheelie mode, a brief **negative** (forward-braking) current pulse actively pushes the front wheel back to the ground. Without this, the bike could hang momentarily at or above the balance point before falling. The exit current magnitude is configurable (`wheelie_exit_brake_current`).

### Why ADC values bypass the existing footpad threshold logic
`footpad_sensor_update()` converts the raw ADC float into an enumerated `FootpadSensorState` (NONE / LEFT / RIGHT / BOTH) by comparing to configured thresholds. This discards the analog value. For throttle and brake control, the raw `adc1` and `adc2` floats on the `FootpadSensor` struct are read directly in `STATE_THROTTLE`, before the thresholding step discards them.

Setting `fault_adc1 = 0` and `fault_adc2 = 0` in config causes `footpad_sensor_update()` to always return `FS_BOTH`, satisfying any remaining footpad checks in `can_engage()` and `check_faults()`, while the raw float values are still available for throttle/brake use.

---

## Files Changed

### `src/state.h`
- Added `STATE_THROTTLE = 4` to the `RunState` enum
- Declared `void state_throttle(State *state)`

### `src/state.c`
- Implemented `state_throttle()`: sets state to `STATE_THROTTLE`, clears `sat`, `stop_condition`, and `wheelslip`
- Added `case STATE_THROTTLE` to `state_compat()` returning `16` (new compat ID)

### `src/conf/datatypes.h`
- Added 5 new fields to `RefloatConfig` (before `CfgMeta meta`):

| Field | Type | Default | Description |
|---|---|---|---|
| `wheelie_target_pitch` | `float` | 20° | Pitch angle the balance loop holds in wheelie mode |
| `wheelie_entry_threshold` | `float` | 5° | Degrees below target at which balance loop engages |
| `throttle_current_max` | `float` | 20A | Max motor current from ADC1 throttle |
| `throttle_brake_current_max` | `float` | 15A | Max regen current from ADC2 brake |
| `wheelie_exit_brake_current` | `float` | 5A | Forward braking current applied on wheelie exit |

### `src/conf/settings.xml`
- Added full parameter definitions for all 5 new fields (type, range, step, unit, description)
- Added serialization order entries after `remote_throttle_grace_period`
- Added two new UI subgroup sections under the existing Remote subgroup:
  - **Wheelie (Bike Mode)**: `wheelie_target_pitch`, `wheelie_entry_threshold`, `wheelie_exit_brake_current`
  - **ADC Throttle (Bike Mode)**: `throttle_current_max`, `throttle_brake_current_max`

### `src/data.h`
- Added `float throttle_current` to the `Data` struct — holds the IIR-filtered current output in `STATE_THROTTLE`

### `src/motor_control.c`
- `motor_control_apply()`: `STATE_THROTTLE` is now treated alongside `STATE_RUNNING` in the parking brake logic — parking brake is deactivated and current commands are passed through normally

### `src/main.c`

#### `calculate_setpoint_target()` — normal running setpoint
```c
// Before:
d->setpoint_target = 0;

// After:
d->setpoint_target = d->float_conf.wheelie_target_pitch;
```
The balance equilibrium is now the configured wheelie angle, not level.

#### `STATE_STARTUP` transition
```c
// Before: → STATE_READY
// After:  → STATE_THROTTLE
```
After IMU calibration, the bike goes directly to normal riding mode. There is no waiting-for-engage step.

#### `STATE_RUNNING` — fault exit redirect
After `check_faults()` stops the balance loop (e.g., pitch fault from landing), the state machine previously landed in `STATE_READY`. For the bike it redirects to `STATE_THROTTLE` so the rider can continue riding on two wheels after the front wheel touches down.

#### `STATE_RUNNING` — wheelie exit on brake
Added at the top of the running loop (before setpoint calculation):
```c
if (d->footpad.adc2 > 0.05f) {
    motor_control_request_current(&d->motor_control,
        -d->float_conf.wheelie_exit_brake_current);
    state_throttle(&d->state);
    d->throttle_current = 0;
    break;
}
```

#### New `STATE_THROTTLE` case
```c
case STATE_THROTTLE: {
    float throttle = d->footpad.adc1 > 0.05f ? d->footpad.adc1 : 0.0f;
    float brake    = d->footpad.adc2 > 0.05f ? d->footpad.adc2 : 0.0f;

    float target_current;
    if (brake > 0.0f) {
        target_current = -brake * d->float_conf.throttle_brake_current_max;
    } else {
        target_current = throttle * d->float_conf.throttle_current_max;
    }

    // 10% IIR smoothing
    d->throttle_current = d->throttle_current * 0.9f + target_current * 0.1f;
    motor_control_request_current(&d->motor_control, d->throttle_current);

    // Wheelie entry: pitch within threshold of target → engage balance loop
    if (d->imu.balance_pitch >=
        (d->float_conf.wheelie_target_pitch - d->float_conf.wheelie_entry_threshold)) {
        engage(d);
        d->setpoint_target = d->float_conf.wheelie_target_pitch;
    }
    break;
}
```

---

## Configuration Checklist

When deploying on a bike, set the following in the VESC Tool UI:

| Parameter | Recommended value | Reason |
|---|---|---|
| `fault_adc1` | `0` | Disables footpad switch logic; ADC1 raw value still readable for throttle |
| `fault_adc2` | `0` | Disables footpad switch logic; ADC2 raw value still readable for brake |
| `fault_pitch` | `60–80°` | Must be above wheelie angle to avoid spurious pitch faults during balance |
| `startup_pitch_tolerance` | `80°` | Not strictly needed (no READY state used), but avoids any residual engage guard |
| `inputtilt_remote_type` | `NONE` | PPM pin is used for the beeper; do not configure PPM remote |
| `wheelie_target_pitch` | Tune per bike | Physical balance point — start at 20° and adjust |
| `wheelie_entry_threshold` | `4–6°` | Smaller = later entry (less time to catch); larger = earlier but may trigger unintentionally |
| `throttle_current_max` | Per motor spec | Limit to safe value for your motor and battery |
| `throttle_brake_current_max` | Per motor spec | Limit to safe regen value |
| `wheelie_exit_brake_current` | `3–8A` | Enough to tip nose down reliably; too high risks abrupt front-wheel slam |

---

## State Machine Diagram

```
STARTUP
   │ (IMU ready)
   ▼
STATE_THROTTLE ◄───────────────────────────────────────────┐
   │  ADC1 → throttle current                              │
   │  ADC2 → regen brake current                           │
   │                                                       │
   │ pitch ≥ (target − threshold)                          │
   ▼                                                       │
STATE_RUNNING (wheelie balance loop)                       │
   │  pitch PID holds wheelie_target_pitch                 │
   │  ADC1 ignored / ADC2 ignored for speed control        │
   │                                                       │
   ├── ADC2 > 5% ──► apply exit brake current ────────────►│
   │                                                       │
   └── fault (pitch/roll/temp/voltage) ──► STATE_THROTTLE ─┘
```
