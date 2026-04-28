# Wheel-E - System Change Document

## Background

This VESC package (Refloat/Wheel-E) was originally developed for onewheels and self-balancing skateboards. This change adapts it for use in a **self-balancing electric bike** in the style of the Future Motion Antic and The Float Life WFB.

The bike balances on its **rear wheel only**, with the front wheel lifted (wheelie). The fore-aft pitch axis and PID loop are identical to the onewheel use case — no IMU axis changes were needed. Lateral (left/right) balance is handled by the rider; the controller does not attempt to compensate for roll.

---

## Change Specification (as provided)

### 1. Throttle and brake inputs
- **ADC1** = analog throttle, **ADC2** = analog brake
- Raw ADC voltages are first smoothed with a configurable IIR low-pass filter (`throttle_adc_filter`)
- Each ADC channel is then mapped from voltage to a 0–1 (0-100%) range using the following calibration points:
  - `voltage_min` → 0 (0% current)
  - `voltage_center` → 0.5 (50% current) (Only ADC1)
  - `voltage_max` → 1 (100% current)
  - Piecewise linear interpolation between min↔center (0–0.5) and center↔max (0.5–1.0)
  - Values at or below min clamp to 0; values at or above max clamp to 1
- Each ADC channel has an `invert` boolean that flips the mapping (min→1, max→0)
- ADC1 and ADC2 are combined into a single `throttle_val` in the range -1 to +1:
  - Brake (ADC2) has absolute priority: any non-zero brake produces a negative value
  - Throttle (ADC1) only contributes when brake is exactly 0
- `throttle_val` is multiplied by the Motor Cfg max current or brake current to produce a current command
- A configurable current deadband (`throttle_current_deadband`, default 1A) suppresses commands below the deadband to prevent ADC noise from energizing the motor
- `throttle_val` is exposed as a realtime data item for UI display

### 2. Wheelie entry trigger
- The balance/wheelie loop engages automatically when pitch rises close to `wheelie_target_pitch`. 
- The tolerance for engange is defined by `startup_pitch_tolerance` which is a ReFloat setting (ReFloat -> Startup -> Tolerances -> Startup Pitch Axis Angle Tolerance)
- Example: target = 25°, tolerance = 4° → balance loop engages at 21°
- Both `wheelie_target_pitch` and `startup_pitch_tolerance` are user-configurable parameters
- Throttle input is **ignored** while in wheelie/balance mode; only leaning affects speed

### 3. Wheelie exit
- Pressing the brake (ADC2 mapped > 0) while in wheelie mode triggers an exit sequence:
  - **With exit ramp** (`wheelie_exit_ramp_time` > 0): the balance loop stays active and the pitch setpoint ramps from `wheelie_target_pitch` down to 0° over the configured time, gently lowering the front wheel. Once the setpoint reaches ~0°, the state transitions to `STATE_THROTTLE` with `throttle_current` seeded from `balance_current` for a bumpless handover.
  - **Without exit ramp** (`wheelie_exit_ramp_time` = 0): instant exit to `STATE_THROTTLE`, seeding `throttle_current` from `balance_current`.
- **Re-entry hysteresis**: after exiting wheelie, re-entry is blocked until pitch drops below `startup_pitch_tolerance` (near level). This prevents a brief brake tap from immediately re-engaging wheelie.
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

Seeding `d->throttle_current = d->balance_current` before returning to `STATE_THROTTLE` means the motor output continues from the live balance value rather than dropping to zero. Since ADC2 is already > 0.05 (it triggered the exit), the brake input is immediately active. The ADC input filter (`throttle_adc_filter`) smooths the transition from the seeded value toward the new ADC-derived target.

A forced negative exit-brake pulse is not needed: the PID was already applying positive drive to hold the wheelie, so simply stopping that drive (and letting gravity do its work) is sufficient for the nose to come down. The regen current requested via ADC2 decelerates the bike on top of that.

### Why ADC values bypass the existing footpad threshold logic
`footpad_sensor_update()` converts the raw ADC float into an enumerated `FootpadSensorState` (NONE / LEFT / RIGHT / BOTH) by comparing to configured thresholds. This discards the analog value. For throttle and brake control, the raw `adc1` and `adc2` floats on the `FootpadSensor` struct are read directly in `STATE_THROTTLE`, before the thresholding step discards them.

Setting `fault_adc1 = 0` and `fault_adc2 = 0` in config causes `footpad_sensor_update()` to always return `FS_BOTH`, satisfying any remaining footpad checks in `can_engage()` and `check_faults()`, while the raw float values are still available for throttle/brake use.

### Why ADC deadband is voltage-domain with a current-domain cutoff
Each ADC channel uses a voltage calibration for deadband and range mapping. Voltages at or below `voltage_min` read as 0%; voltages at or above `voltage_max` read as 100%. This makes the deadband boundaries explicit in hardware voltage terms and independent of the current scaling. ADC1 has an additional `voltage_center` point for better throttle control. Set the center higher to have more resolution at low throttle, or lower for a more aggressive throttle response.

On top of this, a configurable current deadband (`throttle_current_deadband`) suppresses final current commands below the deadband. This catches residual ADC noise that survives the voltage mapping and prevents it from energizing the motor. The default is 1A.

---

## Files Changed

### `src/state.h`
- Added `STATE_THROTTLE = 4` to the `RunState` enum
- Declared `void state_throttle(State *state)`

### `src/state.c`
- Implemented `state_throttle()`: sets state to `STATE_THROTTLE`, clears `sat`, `stop_condition`, and `wheelslip`
- Added `case STATE_THROTTLE` to `state_compat()` returning `16` (new compat ID)

### `src/conf/datatypes.h`
- Added fields to `RefloatConfig` (before `CfgMeta meta`):

| Field | Type | Default | Description |
|---|---|---|---|
| `wheelie_target_pitch` | `float` | 25° | Pitch angle the balance loop holds in wheelie mode |
| `wheelie_exit_ramp_time` | `float` | 0s | Time to ramp setpoint from wheelie pitch to 0° on exit (0 = instant) |
| `throttle_current_deadband` | `float` | 1.0A | Current commands below this are suppressed to zero |
| `throttle_adc1_voltage_min` | `float` | 0.5V | ADC1 voltage mapping to 0% current |
| `throttle_adc1_voltage_center` | `float` | 1.65V | ADC1 voltage mapping to 50% current |
| `throttle_adc1_voltage_max` | `float` | 3.2V | ADC1 voltage mapping to 100% current |
| `throttle_adc1_invert` | `bool` | false | Invert ADC1 so min voltage maps to 100% and max to 0% |
| `throttle_adc2_voltage_min` | `float` | 0.5V | ADC2 voltage mapping to 0% current |
| `throttle_adc2_voltage_max` | `float` | 3.2V | ADC2 voltage mapping to 100% current |
| `throttle_adc2_invert` | `bool` | false | Invert ADC2 so min voltage maps to 100% and max to 0% |
| `throttle_adc_filter` | `float` | 0.1 | IIR low-pass filter coefficient for raw ADC voltages (0 = off, 0.99 = heavy) |

### `src/conf/settings.xml`
- Added full parameter definitions for all new fields (type, range, step, unit, description)
- Added serialization order entries after `remote_throttle_grace_period`
- Added a new **Bike** subgroup under the **General** group with two UI separator sections:
  - **Throttle mode**: `throttle_current_deadband`, `throttle_adc1_voltage_min`, `throttle_adc1_voltage_center`, `throttle_adc1_voltage_max`, `throttle_adc1_invert`, `throttle_adc2_voltage_min`, `throttle_adc2_voltage_max`, `throttle_adc2_invert`, `throttle_adc_filter`
  - **Wheelie mode**: `wheelie_target_pitch`, `wheelie_exit_ramp_time`

### `src/data.h`
- Added `float throttle_current` to the `Data` struct — holds the current output in `STATE_THROTTLE`
- Added `float throttle_adc1_filtered` and `float throttle_adc2_filtered` — IIR-filtered raw ADC voltages
- Added `float throttle_adc1_mapped` and `float throttle_adc2_mapped` — calibrated 0–1 values after min/center/max mapping and optional inversion
- Added `float throttle_val` — combined throttle/brake value (-1 to +1) exposed to UI via realtime data
- Added `bool wheelie_entry_armed` — hysteresis flag preventing immediate wheelie re-entry after a brake exit
- Added `bool wheelie_exiting` — flag indicating the wheelie exit ramp is in progress
- Added `float wheelie_exit_step_size` — precomputed degrees-per-iteration for the exit ramp

### `src/rt_data.h`
- Added `S(throttle_val)` to the `RT_DATA_ITEMS` macro — sends `d->throttle_val` as a realtime data item to the UI

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
After `check_faults()` stops the balance loop (e.g., pitch fault from landing), the state machine previously landed in `STATE_READY`. For the bike it redirects to `STATE_THROTTLE` with `throttle_current` zeroed so the motor is released cleanly rather than carrying through whatever current the balance loop was commanding.

#### `STATE_RUNNING` — wheelie exit on brake
When the brake is pressed (ADC2 mapped > 0), the exit behaviour depends on `wheelie_exit_ramp_time`:

**With ramp (ramp time > 0):** The balance loop stays active but `setpoint_target` is set to 0°. The existing `rate_limitf` interpolation ramps `setpoint_target_interpolated` down at a rate of `wheelie_target_pitch / (ramp_time × hertz)` degrees per iteration. Once the interpolated setpoint reaches ≤ `startup_pitch_tolerance`, the state transitions to `STATE_THROTTLE` with a bumpless handover.

**Without ramp (ramp time = 0):** Instant exit.

```c
// Wheelie exit: brake pressed on ADC2 -> begin exit sequence.
// If ramp time is configured, gradually lower the setpoint to 0 while
// the balance loop keeps running. Otherwise exit instantly.
if (d->throttle_adc2_mapped > 0.0f && !d->wheelie_exiting) {
    if (d->wheelie_exit_step_size > 0.0f) {
        // Start ramping the setpoint down to 0
        d->wheelie_exiting = true;
        d->setpoint_target = 0;
    } else {
        // Instant exit
        state_throttle(&d->state);
        d->throttle_current = d->balance_current;
        d->wheelie_entry_armed = false;
        break;
    }
}

// Wheelie exit ramp: once setpoint has reached entry threshold, transition to throttle
if (d->wheelie_exiting && d->setpoint_target_interpolated <= d->float_conf.startup_pitch_tolerance) {
    state_throttle(&d->state);
    d->throttle_current = d->balance_current;
    d->wheelie_entry_armed = false;
    d->wheelie_exiting = false;
    break;
}
```

During the ramp, `calculate_setpoint_target()` skips overwriting `setpoint_target` back to `wheelie_target_pitch` when `wheelie_exiting` is true. The `rate_limitf` step size is switched from the normal `get_setpoint_adjustment_step_size()` to `wheelie_exit_step_size`.

#### ADC filtering and mapping

ADC filtering and piecewise min/center/max mapping runs every cycle **before** the state machine switch, so the mapped values (`d->throttle_adc1_mapped`, `d->throttle_adc2_mapped`) are available in both `STATE_THROTTLE` and `STATE_RUNNING`. Raw ADC voltages are smoothed with an IIR low-pass filter, then mapped through per-channel min/center/max calibration to a 0–1 range with optional inversion.

#### New `STATE_THROTTLE` case

The pre-computed mapped values are combined into a single `throttle_val` (-1 to +1) where brake has absolute priority. The final current command is suppressed below a configurable deadband to prevent ADC noise from energizing the motor.

```c
case STATE_THROTTLE: {
    // Normal two-wheel riding: ADC1 = throttle, ADC2 = brake
    // ADC filtering and mapping is done before the switch.

    // Combine into a single -1..1 value: brake wins if non-zero
    if (d->throttle_adc2_mapped > 0.0f) {
        d->throttle_val = clampf(-d->throttle_adc2_mapped, -1.0f, 0.0f);
    } else {
        d->throttle_val = clampf(d->throttle_adc1_mapped, 0.0f, 1.0f);
    }
    float current = 0.0f;
    if (d->throttle_val < 0) {
        current = d->throttle_val * d->motor.current_min;
    } else if (d->throttle_val > 0) {
        current = d->throttle_val * d->motor.current_max;
    }
    // set current request. Ignore current below deadband
    float deadband = d->float_conf.throttle_current_deadband;
    if (current < -deadband) {
        motor_control_request_brake_current(&d->motor_control, -current);
    } else if (current > deadband) {
        motor_control_request_current(&d->motor_control, current);
    } else {
        motor_control_request_current(&d->motor_control, 0.0f);
    }

    // Wheelie re-entry hysteresis: pitch must drop below threshold before
    // we allow re-entering wheelie, preventing immediate re-engage after
    // a brief brake tap.
    if (!d->wheelie_entry_armed) {
        if (d->imu.balance_pitch < d->float_conf.startup_pitch_tolerance) {
            d->wheelie_entry_armed = true;
        }
    }

    // Wheelie entry: engage balance loop when pitch approaches the target angle
    if (d->wheelie_entry_armed &&
        d->imu.balance_pitch >=
            (d->float_conf.wheelie_target_pitch - d->float_conf.startup_pitch_tolerance)) {
        engage(d);
        // Set centering target to the wheelie balance point, not 0
        d->setpoint_target = d->float_conf.wheelie_target_pitch;
        // Bumpless transfer: seed balance_current from the live throttle current so
        // the motor output does not drop to zero at the moment of handover.
        d->balance_current = d->throttle_current;
    }
    break;
}
```

**Regen braking:** Brake current is routed through `motor_control_request_brake_current()`, which calls `mc_set_brake_current()` with a positive value. This always regenerates energy regardless of motor direction and cannot reverse the motor. Using `mc_set_current()` with a negative value would instead command reverse torque.

**Brake priority:** ADC2 (brake) has absolute priority over ADC1 (throttle). Any non-zero brake input produces a negative `throttle_val`, which maps to regen current. Throttle only contributes when the brake mapped value is exactly zero.

**Current deadband:** Both throttle and brake current commands below `throttle_current_deadband` (default 1A) are suppressed to zero. This prevents residual ADC voltage on an unused input from producing a small but nonzero current command that would keep the motor energized.

#### Bumpless transfer on wheelie entry

`engage()` calls `reset_runtime_vars()`, which zeros both `balance_current` and the PID state. Without correction this causes a current step from whatever `throttle_current` was down to 0A the instant the balance loop takes over, producing an immediate deceleration kick.

The fix seeds `balance_current` from `throttle_current` immediately after `engage()` returns. Because `STATE_RUNNING` integrates `balance_current` with an 0.8/0.2 IIR (`balance_current = balance_current * 0.8 + new_current * 0.2`), starting from the live throttle value gives the PID integral time to wind up to the correct steady-state current before the seed decays. No I-term preload is required — the seeded `balance_current` provides enough continuity.

The PID setpoint itself is not an issue: `reset_runtime_vars()` seeds `setpoint` and `setpoint_target_interpolated` to the **current** pitch, so the first PID error is near zero regardless of how far the target pitch is from the entry pitch. The centering ramp (`SAT_CENTERING`) then moves the setpoint toward `wheelie_target_pitch` at `startup_step_size`.

### Why wheelie re-entry uses hysteresis
Without hysteresis, a brief brake tap exits wheelie mode (`STATE_RUNNING` → `STATE_THROTTLE`) but the pitch is still near the entry tolerance. On the very next control loop iteration the entry condition is met again and the bike immediately re-enters wheelie, making it impossible to exit with a short brake press.

The fix adds a `wheelie_entry_armed` flag:
1. On wheelie exit (brake press), the flag is cleared (`false`)
2. While `false`, the wheelie entry check is skipped regardless of pitch
3. The flag is re-armed (`true`) only when pitch drops below `startup_pitch_tolerance` (near level)
4. On startup, the flag is initialized to `true` via `reset_runtime_vars()` so the first entry works

This forces the rider to bring the front wheel down past the tolerance before wheelie mode can engage again, giving a clean and intentional transition.

---

## Configuration Checklist

When deploying on a bike, set the following in the VESC Tool UI:

|Path| Parameter | Recommended value | Reason |
|---|---|---|---|
|Refloat Cfg → Specs | ADC1 Switch Voltage (`fault_adc1`) | `0` | Disables footpad switch logic; ADC1 raw value still readable for throttle |
|Refloat Cfg → Specs | ADC2 Switch Voltage (`fault_adc2`) | `0` | Disables footpad switch logic; ADC2 raw value still readable for brake |
|App Cfg → General | App to use | `No App` | Disables the VESC built in ADC app. Prevents interference with the current commands. Alternatively set to `UART` |
|Motor Cfg → General → Current | Max current | Safe value | This is the max motor current. Set this low when you start to tune to prevent damage |
|Refloat Cfg → Bike | Wheelie Target Pitch (`wheelie_target_pitch`) | Tune per bike | Physical balance point — start at 25° and adjust |
|Refloat Cfg → Bike | Wheelie Exit Ramp Time (`wheelie_exit_ramp_time`) | `0.0` or `0.3-0.6` | Seconds to gently lower the front wheel; 0 = instant drop to throttle mode |
|Refloat Cfg → Bike | Throttle Current Deadband (`throttle_current_deadband`) | `1.0` | Current commands below this (A) are suppressed to zero |
|Refloat Cfg → Bike | Throttle ADC1 Voltage Min (`throttle_adc1_voltage_min`) | `0.5` | ADC1 voltage at 0% throttle; adjust to match hardware rest voltage |
|Refloat Cfg → Bike | Throttle ADC1 Voltage Center (`throttle_adc1_voltage_center`) | `1.65` | ADC1 voltage at 50% current |
|Refloat Cfg → Bike | Throttle ADC1 Voltage Max (`throttle_adc1_voltage_max`) | `3.2` | ADC1 voltage at 100% throttle; adjust to match hardware full-scale |
|Refloat Cfg → Bike | Throttle ADC1 Invert (`throttle_adc1_invert`) | `false` | Flip ADC1 direction if wired in reverse |
|Refloat Cfg → Bike | Brake ADC2 Voltage Min (`throttle_adc2_voltage_min`) | `0.5` | ADC2 voltage at 0% brake |
|Refloat Cfg → Bike | Brake ADC2 Voltage Max (`throttle_adc2_voltage_max`) | `3.2` | ADC2 voltage at 100% brake |
|Refloat Cfg → Bike | Brake ADC2 Invert (`throttle_adc2_invert`) | `false` | Flip ADC2 direction if wired in reverse |
|Refloat Cfg → Bike | Throttle ADC Filter (`throttle_adc_filter`) | `0.1` | Low-pass filter strength for raw ADC voltages; 0 = off, higher = smoother but more lag |
|Refloat Cfg → Startup → Tolerances | Startup Pitch Axis Angle Tolerance (`startup_pitch_tolerance`) | `2–6°` | Smaller = later entry (less time to catch); larger = earlier but may trigger unintentionally |
|Refloat Cfg → Stop | Pitch Axis Fault Cutoff (`fault_pitch`) | Tune per bike | Must be above wheelie angle to avoid spurious pitch faults during balance |

---

## State Machine Diagram

```
STARTUP
   │ (IMU ready)
   ▼
 ┌─────────────────────────────────────────────────────────┐
 │  Every cycle (before switch):                           │
 │    IIR low-pass filter on raw ADC voltages              │
 │    ADC1 → piecewise mapped (min/center/max → 0..1)      │
 │    ADC2 → piecewise mapped (min/max → 0..1)             │
 └─────────────────────────────────────────────────────────┘
   │
   ▼
STATE_THROTTLE ◄───────────────────────────────────────────┐
   │  Brake priority: adc2_mapped > 0 → throttle_val < 0   │
   │  Current deadband: |current| < deadband → 0           │
   │                                                       │
   │  Re-entry blocked until pitch < tolerance             │
   │  (hysteresis re-arm)                                  │
   │                                                       │
   │ pitch ≥ (target − tolerance) AND re-entry armed       │
   ▼                                                       │
STATE_RUNNING (wheelie balance loop)                       │
   │  pitch PID holds wheelie_target_pitch                 │
   │  Throttle/brake inputs ignored for speed control      │
   │                                                       │
   ├── adc2_mapped > 0 ┬─ ramp_time > 0:                   │
   │                   │   setpoint ramps toward 0°        │
   │                   │   (balance loop active), exits    │
   │                   │   at tolerance ──────────────────►│
   │                   │                                   │
   │                   └─ ramp_time = 0:                   │
   │                       instant exit ──────────────────►│
   │                                                       │
   └── fault (pitch/roll/temp/voltage) ──► STATE_THROTTLE ─┘
```
