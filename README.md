# Wheel-E - VESC Package
Wheel-E is a VESC Package for electric mini-bikes with a wheelie mode. The package is a fork of [ReFloat](ReFloat_README.md) used in self-balancing skateboards.

Status as of 15.04.2026: Some code implemented. Compiles and runs. Tested handheld on a vesc Pint. Bike not yet fabricated...

## Code summary
* A "THROTTLE" state when not wheelieing using ADC1 for throttle and ADC2 for brake
* A "WHEELIE" state whitch is the the same as "RUNNING" state on onewheels, but with a different angle setpoint
* Foot pad sensors disabled
* Changed the IU to display a mini-bike and throttle/brake gauge instead of footpads

[code change doc](doc/wheel-e_mods.md)

## Prototype hardware
* Spintend UBox Lite 100v 100A
* Pint motor w. stock tire
* 14s7p 50s battery pack

[ReFloat README](ReFloat_README.md)
