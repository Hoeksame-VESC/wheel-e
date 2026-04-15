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
  1. Returns to normal throttle mode (`STATE_THROTTLE`), seeding `throttle_current` from the live `balance_current` for a bumpless handover
  2. The ADC2 regen current ramps in naturally via the IIR filter
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

### Why `balance_current` is seeded into `throttle_current` on exit
When the balance loop is running, it has wound up a `balance_current` value that represents the steady-state motor drive needed to maintain speed at the wheelie angle. If that is discarded on exit, the motor output drops to zero and the bike decelerates sharply before the ADC2 regen ramps back in.

Seeding `d->throttle_current = d->balance_current` before returning to `STATE_THROTTLE` means the IIR filter continues from the live balance output. Since ADC2 is already > 0.05 (it triggered the exit), `target_current` is immediately negative (regen), and `throttle_current` ramps smoothly from the seeded positive value through zero and into regen — no current step, no deceleration spike.

A forced negative exit-brake pulse is not needed: the PID was already applying positive drive to hold the wheelie, so simply stopping that drive (and letting gravity do its work) is sufficient for the nose to come down. The regen current requested via ADC2 decelerates the bike on top of that.

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
- Added 4 new fields to `RefloatConfig` (before `CfgMeta meta`):

| Field | Type | Default | Description |
|---|---|---|---|
| `wheelie_target_pitch` | `float` | 20° | Pitch angle the balance loop holds in wheelie mode |
| `wheelie_entry_threshold` | `float` | 5° | Degrees below target at which balance loop engages |
| `throttle_current_max` | `float` | 20A | Max motor current from ADC1 throttle |
| `throttle_brake_current_max` | `float` | 15A | Max regen current from ADC2 brake |
| `throttle_adc_voltage_max` | `float` | 3.2V | ADC voltage that maps to 100% throttle/brake |

### `src/conf/settings.xml`
- Added full parameter definitions for all 5 new fields (type, range, step, unit, description)
- Added serialization order entries after `remote_throttle_grace_period`
- Added two new UI subgroup sections under the existing Remote subgroup:
  - **Wheelie (Bike Mode)**: `wheelie_target_pitch`, `wheelie_entry_threshold`
  - **ADC Throttle (Bike Mode)**: `throttle_current_max`, `throttle_brake_current_max`, `throttle_adc_voltage_max`

### `src/data.h`
- Added `float throttle_current` to the `Data` struct — holds the IIR-filtered current output in `STATE_THROTTLE`

### `src/motor_control.c`
- `motor_control_apply()`: `STATE_THROTTLE` is now treated alongside `STATE_RUNNING` in the parking brake logic — parking brake is deactivated and current commands are passed through normally
- Added `brake_current_requested` / `requested_brake_current` fields to `MotorControl`
- Added `motor_control_request_brake_current()`: sets the new fields; `motor_control_apply()` routes these to `VESC_IF->mc_set_brake_current()` (positive value, signed-current path bypassed) so regen never drives the motor in reverse

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
// Wheelie exit: brake pressed on ADC2 -> return to throttle mode.
// Seed throttle_current from balance_current so the motor output does not
// drop abruptly. The regen requested via ADC2 will then ramp in naturally
// through the IIR filter in STATE_THROTTLE.
if (d->footpad.adc2 > 0.05f) {
    state_throttle(&d->state);
    d->throttle_current = d->balance_current;
    break;
}
```

#### New `STATE_THROTTLE` case

ADC readings are normalized by `throttle_adc_voltage_max` so that the configured voltage equals 100% input. Values above the configured voltage are clamped to 1.0.

```c
case STATE_THROTTLE: {
    float adc_scale = d->float_conf.throttle_adc_voltage_max > 0.0f
        ? 1.0f / d->float_conf.throttle_adc_voltage_max
        : 1.0f;
    float throttle = d->footpad.adc1 > 0.05f
        ? fminf(d->footpad.adc1 * adc_scale, 1.0f)
        : 0.0f;
    float brake = d->footpad.adc2 > 0.05f
        ? fminf(d->footpad.adc2 * adc_scale, 1.0f)
        : 0.0f;

    float target_current;
    if (brake > 0.0f) {
        // Store negative internally for IIR smoothing (positive = throttle, negative = brake)
        target_current = -brake * d->float_conf.throttle_brake_current_max;
    } else {
        target_current = throttle * d->float_conf.throttle_current_max;
    }

    // 10% IIR smoothing
    d->throttle_current = d->throttle_current * 0.9f + target_current * 0.1f;

    // Route to the correct VESC call: mc_set_brake_current for regen (never reverses
    // the motor), mc_set_current for forward throttle.
    if (d->throttle_current < 0.0f) {
        motor_control_request_brake_current(&d->motor_control, -d->throttle_current);
    } else {
        motor_control_request_current(&d->motor_control, d->throttle_current);
    }

    // Wheelie entry: pitch within threshold of target → engage balance loop
    if (d->imu.balance_pitch >=
        (d->float_conf.wheelie_target_pitch - d->float_conf.wheelie_entry_threshold)) {
        engage(d);
        d->setpoint_target = d->float_conf.wheelie_target_pitch;
        // Bumpless transfer: seed balance_current from the live throttle current so
        // the motor output does not drop to zero at the moment of handover.
        d->balance_current = d->throttle_current;
    }
    break;
}
```

**Bug fix — regen threw wheel in reverse:** The original implementation passed a negative value to `motor_control_request_current()`, which calls `mc_set_current()`. VESC interprets a signed current as a torque direction, so a negative value drives the motor in reverse. The fix uses `motor_control_request_brake_current()` instead, which calls `mc_set_brake_current()` with a positive value — this always regenerates energy regardless of direction and cannot reverse the motor.

#### Bumpless transfer on wheelie entry

`engage()` calls `reset_runtime_vars()`, which zeros both `balance_current` and the PID state. Without correction this causes a current step from whatever `throttle_current` was down to 0A the instant the balance loop takes over, producing an immediate deceleration kick.

The fix seeds `balance_current` from `throttle_current` immediately after `engage()` returns. Because `STATE_RUNNING` integrates `balance_current` with an 0.8/0.2 IIR (`balance_current = balance_current * 0.8 + new_current * 0.2`), starting from the live throttle value gives the PID integral time to wind up to the correct steady-state current before the seed decays. No I-term preload is required — the seeded `balance_current` provides enough continuity.

The PID setpoint itself is not an issue: `reset_runtime_vars()` seeds `setpoint` and `setpoint_target_interpolated` to the **current** pitch, so the first PID error is near zero regardless of how far the target pitch is from the entry pitch. The centering ramp (`SAT_CENTERING`) then moves the setpoint toward `wheelie_target_pitch` at `startup_step_size`.

---

## Configuration Checklist

When deploying on a bike, set the following in the VESC Tool UI:

|Path| Parameter | Recommended value | Reason |
|---|---|---|---|
|Refloat Cfg -> Specs | ADC1 Switch Voltage (`fault_adc1`) | `0` | Disables footpad switch logic; ADC1 raw value still readable for throttle |
|Refloat Cfg -> Specs | ADC2 Switch Voltage (`fault_adc2`) | `0` | Disables footpad switch logic; ADC2 raw value still readable for brake |
|Refloat Cfg -> Stop | Pitch Axis Fault Cutoff (`fault_pitch`) | `60–80°` | Must be above wheelie angle to avoid spurious pitch faults during balance |
|Refloat Cfg -> Startup | Startup Pitch Axis Angle Tolerance (`startup_pitch_tolerance`) | `80°` | Not strictly needed (no READY state used), but avoids any residual engage guard |
|Refloat Cfg -> Remote | Remote Type (`inputtilt_remote_type`) | `NONE` | PPM pin is used for the beeper; do not configure PPM remote |
|Refloat Cfg -> Tune | Wheelie Target Pitch (`wheelie_target_pitch`) | Tune per bike | Physical balance point — start at 20° and adjust |
|Refloat Cfg -> Tune | Wheelie Entry Threshold (`wheelie_entry_threshold`) | `4–6°` | Smaller = later entry (less time to catch); larger = earlier but may trigger unintentionally |
|Refloat Cfg -> Tune | Throttle Current Max (`throttle_current_max`) | Per motor spec | Limit to safe value for your motor and battery |
|Refloat Cfg -> Tune | Throttle Brake Current Max (`throttle_brake_current_max`) | Per motor spec | Limit to safe regen value |
|Refloat Cfg -> Tune | Throttle ADC Full-Scale Voltage (`throttle_adc_voltage_max`) | `3.2` | Voltage at which throttle/brake reads as 100%; adjust to match your throttle hardware |

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
