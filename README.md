# Wheel-E - VESC Package
Wheel-E is a VESC Package for electric mini-bikes with a wheelie mode. The package is a fork of [ReFloat](ReFloat_README.md) used in self-balancing skateboards.

## Functionality summary
* A "THROTTLE" state used when riding on two wheels. Using ADC1 for throttle and ADC2 for brake.
* A "WHEELIE" state which is the same as "RUNNING" state on onewheels, but with a different angle setpoint.
    * This mode is triggered when the pitch value gets close to the angle setpoint.
    * It exits back to "THROTTLE" when pressing the brake.
* The IU displays a mini-bike and throttle/brake gauge instead of footpads.
* Configurable parameters for the angle setpoint, throttle/brake current, and state transition thresholds.

## Setup
* Configure as a onewheel (Use tutorials for onewheels, not bikes)
* Configure parameters under ReFloat Cfg -> Bike.
* Disable the foot sensors by setting ADC Switch voltage to 0v (ReFloat Cfg -> Spec -> ADC1&2 Switch voltage: `0.0v`)
* Make sure that the built in ADC app is not enabled. (App Cfg -> General -> App to Use: `No App` or `UART`)

## Full code documentation
[Wheel-E code modifications](doc/wheel-e_mods.md) This document describes the code changes made to ReFloat to implement the Wheel-E functionality. It includes explanations of the new state machine logic, parameter handling, and UI changes.

[ReFloat README](ReFloat_README.md) This is the original README for the ReFloat project, which provides an overview of the base code that Wheel-E is built upon. It includes setup instructions, functionality summaries, and documentation links for ReFloat itself.

## Project status
As of 15.04.2026: Some code implemented. Compiles and runs. Tested handheld on a VESCed Onewheel Pint. Bike not yet fabricated.

Planned hardware for the bike:
* Custom steel frame similar to Coleman CT200U / FM Antic
* Suspension fork
* Coleman CT200U seat
* Front wheel with stock Pint tire and disc brake
* Spintend UBox Lite 100v 100A
* Pint motor w. stock tire
* 14s7p 50s battery pack