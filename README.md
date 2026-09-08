# NASA USLI Payload — Flight Telemetry & Audio Broadcast System

This repository is a fork of [LilyGO/T-TWR](https://github.com/Xinyuan-LilyGO/T-TWR),
used as the firmware base for our NASA University Student Launch Initiative (USLI)
payload. The TWR framework (menu system, radio driver, OLED display library, board
configs) is LilyGO's — see their repo for full hardware documentation and specs.

## My Contributions

Working within `examples/Factory/Factory.ino`, I built the payload's flight
telemetry and post-landing audio broadcast system:

- [`demoVelocitySensor`](https://github.com/OvershotChimera/nasa-usli-payload-telemetry/blob/master/examples/Factory/Factory.ino#L893) —
  live vertical velocity readout computed from barometric altitude deltas.
- [`demoFlightTelemetry`](https://github.com/OvershotChimera/nasa-usli-payload-telemetry/blob/master/examples/Factory/Factory.ino#L969) —
  full flight computer: tracks altitude and velocity from the BMP390 barometric
  sensor, pulls landing time from GPS, and runs a simple survivability scoring
  heuristic based on altitude, descent velocity, and battery thresholds.
- [`broadcastAudioTelemetry`](https://github.com/OvershotChimera/nasa-usli-payload-telemetry/blob/master/examples/Factory/Factory.ino#L1695) —
  on landing detection, transmits a spoken summary of flight data (altitude,
  velocity, battery, survivability score, time of landing) over radio using
  pre-recorded WAV clips, with numeric values spoken digit-by-digit.
- [`playWavFile`, `playVariableDigits`, `playDigitWAV`](https://github.com/OvershotChimera/nasa-usli-payload-telemetry/blob/master/examples/Factory/Factory.ino#L1952) —
  audio playback helpers built on top of the ESP8266Audio library, routed
  through the TWR's radio transmit path.

**Why:** the payload needed to report flight data after landing without relying
on a laptop or phone nearby to read logs — so instead of just logging data to
SD, it announces a summary out loud over the radio on landing, audible to
anyone with a receiver tuned to the frequency.

## DEMO VIDEO

[![field recording from the launch site](https://img.youtube.com/vi/vBzKQt7xYfI/hqdefault.jpg)](https://youtu.be/vBzKQt7xYfI)

—
receiver picking up the audio telemetry broadcast. Audio's rough (wind, rocket
noise in the background), apologies :).

**Stack:** C++ (Arduino/ESP32), barometric + GPS sensor fusion, I2S audio
output, SA868 radio module control.

---

## Building the firmware

### PlatformIO (recommended)
1. Install [VS Code](https://code.visualstudio.com/) and the `PlatformIO` extension.
2. Open this repository's root folder in VS Code.
3. In `platformio.ini`, uncomment the one environment line you want to build (leave only one active).
4. Build (✔) and upload (→) using the PlatformIO toolbar.

### Arduino IDE
1. Install [Arduino IDE](https://www.arduino.cc/en/software) and the [Arduino ESP32 core](https://docs.espressif.com/projects/arduino-esp32/en/latest/).
2. Copy each folder inside `lib/` into your Arduino `libraries/` directory (copy the folders *inside* `lib/`, not `lib/` itself).
3. Board settings: **ESP32S3 Dev Module**, Flash Size **16MB**, Partition Scheme **16M Flash (3M APP/9.9MB FATFS)**, PSRAM **OPI PSRAM**, Upload Mode **UART0/Hardware CDC**.
4. Open `examples/Factory/Factory.ino` and upload.

Full hardware specs, schematics, and troubleshooting for the base TWR board
are in the [original LilyGO repository](https://github.com/Xinyuan-LilyGO/T-TWR).
