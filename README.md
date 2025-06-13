# BDShot Motor Control

## BDShot Overview
Bi-Directional Shot embeds an 11-bit command (or data) field, a 1-bit telemetry flag, and a 4-bit checksum. With the telemetry flag set to True on outoging packets, the ESC is prompted to return a 20 bit telemetry packet. By default, these packets contain eRPM data, but extended dshot telemetry (EDT) can be enabled using commands, providing temperature, voltage, current, and other motor feedback. Simply enabling telemetry does NOT turn DShot into BDShot. DShot uses a seperate backchannel wire to route telemetry signals, while BDShot allows outgoing throttle/command packets and incoming telemetry packets to share the same wire using precise timing protocols. For this project, we are using an Electronic Speed Controller (ESC) flashed with [BlueJay v21](https://github.com/bird-sanctuary/bluejay?tab=readme-ov-file), and an STM32F405RGT6 as our flight controller. 

## 1. Sending outgoing BDShot Signals (Flight Controller --> Electronic Speed Controller)
Outgoing signals use 

## to be continued
