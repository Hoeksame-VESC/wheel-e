# Wheel-E - VESC Package
Wheel-E is a VESC Package for electric mini-bikes with a wheelie mode. The package is a fork of [ReFloat](https://github.com/lukash/refloat) used in self-balancing skateboards.

## Functionality summary
* A "THROTTLE" state used when riding on two wheels. Using ADC1 for throttle and ADC2 for brake.
* A "WHEELIE" state which is the same as "RUNNING" state on onewheels, but with a different angle setpoint.
    * This mode is triggered when the pitch value gets close to the angle setpoint, or the optional button is pressed.
    * It exits back to "THROTTLE" when pressing the brake, or or the optional button is pressed or released
* The IU displays a mini-bike and throttle/brake gauge instead of footpads.
* Configurable parameters for the angle setpoint, throttle/brake current, and state transition thresholds.

## Hardware requirements
* VESC with IMU and ADC inputs
* Motor with hall sensors
* Analog trottle
* Analog or digital brake
* _Optional: Button between TX and GND which can be used to enter and/or exit wheelie mode._
* _Optional: Button between RX and GND which can be used for cruise control._

## Configuration
* Configure as a onewheel (Use tutorials for onewheels, not bikes)
* Configure parameters under ReFloat Cfg -> Bike.
* Disable the foot sensors by setting ADC Switch voltage to 0v (ReFloat Cfg -> Spec -> ADC1&2 Switch voltage: `0.0v`)
* Make sure that the built in ADC app is not enabled. (App Cfg -> General -> App to Use: `No App` or `UART`)
* To be safe, start with low motor current setting! (Motor Cfg -> General -> Current -> Motor Current Max)

# Refloat

A full-featured self-balancing skateboard package.

## New in 1.2
- BMS alerting support (Pushback and Haptic)
- Speed-based alerting (Pushback and Haptic)
- Automatic config migration when updating the package
- Support for per-cell High/Low Voltage Thresholds (no more needing to adjust them in Specs after a fresh install, 6.05+ only)
- Groundworks for a new alerting system, for now only providing firmware fault reporting
- New Rainbow Fade, Rainbow Cycle and Rainbow Roll LED effects

For more details, read the [1.2 release post](https://pev.dev/t/refloat-version-1-2/2795).

## Installation
### Upgrading
Back up your package config (either by **Backup Configs** on the Start page, or by saving the XML on **Refloat Cfg**).

If upgrading from Refloat 1.1 to 1.2, an automatic config restore dialog should pop up. Confirm it to restore. This is the preferred way to update the config from now on, as it will also migrate any changed options to the new version. Other methods of restoring the config will not do that.

In case the automatic config restore fails, restore it the traditional way and please report the issue, so that it can be fixed.

### Fresh Installation
If doing a fresh board installation, you need to do the **motor** and **IMU** calibration and configuration. If you install the package before that, you need to disable the package before running the **motor** _calibration_ and re-enable it afterwards.

For a detailed guide, read the [Initial Board Setup guide on pev.dev](https://pev.dev/t/initial-board-setup-in-vesc-tool/2190).

On 6.05+ firmware, the package should ride well without the need to configure anything in Refloat Cfg. On 6.02, the **Low and High Tiltback voltages** on the **Specs** tab of **Refloat Cfg** still need to be set according to your battery specs.

## Disclaimer
**Use at your own risk!** Electric vehicles are inherently dangerous, authors of this package shall not be liable for any damage or harm caused by errors in the software. Not endorsed by the VESC project.

## Credits
Author: Lukáš Hrázký

Original Float package authors: Mitch Lustig, Dado Mista, Nico Aleman

## Downloads and Changelogs
[https://github.com/lukash/refloat/releases](https://github.com/lukash/refloat/releases)
