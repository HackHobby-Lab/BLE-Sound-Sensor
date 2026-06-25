# Mic-Sense Baby Cry Detector Firmware

Mic-Sense is a small battery-powered device that listens to a room, detects when a baby is crying, and sends an alert to a phone app over Bluetooth. This repository holds the device firmware (the program that runs on the chip).

The device has **two different microphones** on the board, and there is **one firmware build for each**. They do the same job they only differ in which microphone they read.

| Build | Microphone | How it reads sound |
|-------|-----------|--------------------|
| **ICS-43434 build** | ICS-43434 (digital) | Digital audio over I²S |
| **MAX9814 build** | MAX9814 (analog) | Analog signal through the chip's ADC |

Both builds talk to the same phone app in exactly the same way.

---

## What the device does (in plain words)

1. It stays powered by a battery and turns on when you press the button.
2. It constantly measures how loud the room is.
3. If the sound stays above a set loudness level for 2 seconds, it decides "this is a cry."
4. It then sends an alert to the phone app, and keeps alerting every 3 seconds for 2 minutes.
5. The phone app shows the live sound level and receives the cry alerts.

> **Important:** This is a *loudness* detector. It alerts on **any** sound loud enough to cross the threshold not only crying. It works on the assumption that the area near the baby is normally quiet.

---

## The hardware

- **Board:** Sound Sensor V0.3
- **Main chip / module:** u-blox NINA-B406 (contains a Nordic nRF52833 Bluetooth chip)
- **Microphones:**
  - ICS-43434 digital (I²S)
  - MAX9814 analog (read by the chip's ADC)
- **Other parts:** battery charger, fuel gauge (battery % meter), RGB LED, one main button, power-latch circuit
- **Antenna:** built into the NINA module (no external antenna)

### The one button does everything

| Action | What happens |
|--------|-------------|
| Press to turn on | Board powers up and stays on (firmware "latches" the power) |
| Short press | Turns Bluetooth pairing (advertising) on/off |
| Long press (~3 seconds) | Powers the whole device off |

---

## Software you need

- **nRF Connect SDK v3.2.4** (this also installs Zephyr and the build tools)
- **VS Code** with the **nRF Connect extension** (recommended), or the command line
- **SEGGER J-Link** programmer
- **nRF Connect for Desktop → Programmer app** (for flashing)
- **SEGGER RTT Viewer** (to see the device's log messages)

---

## Project layout

Each build is its own folder with the same structure:

```
<project-folder>/
  CMakeLists.txt      # tells the build system what to compile
  prj.conf            # turns features on/off (Bluetooth, ADC, etc.)
  app.overlay         # maps the chip pins to the hardware
  src/
    main.c            # the actual firmware code
```

---

## How to build

> Tip: the build tools dislike spaces in folder paths. If your project lives in a path with a space (like `D:\New folder\...`), make a junction to a clean path and build from there:
> ```
> mklink /J C:\ncs\work\myproject "D:\New folder\myproject"
> ```

From inside the project folder (or the junction), run a clean ("pristine") build:

```
west build -b nrf52833dk/nrf52833 . --pristine --build-dir build
```

Notes:
- The build target is `nrf52833dk/nrf52833`. This is correct for **both** the development kit and the real board, because the NINA module uses the same nRF52833 chip.
- A successful build prints a memory summary and creates `build/merged.hex`.
- You may see a few harmless "defined but not used" warnings these are fine to ignore.

---

## How to flash (load the firmware onto the board)

1. Connect the **J-Link** to the board's SWD header:
   - Vtref → 3V3
   - SWDIO → DIO
   - SWDCLK → CLK
   - GND → GND
2. Power the board (USB-C or battery) and make sure the front switch is **ON**.
3. Open **nRF Connect for Desktop → Programmer**.
4. Select your J-Link device.
5. Add the file `build/merged.hex`.
6. Click **Erase & write**.

The file is a "merged" hex, so it already knows where to go in memory no address needed.

---

## How to see what the device is doing (logs)

This board has no USB serial port, so logs come out over **SEGGER RTT** instead.

1. Open **J-Link RTT Viewer**.
2. Connect to the nRF52833.
3. You'll see live messages, for example:
   ```
   Power latch: EN_CONTROLL (P0.17) held HIGH
   Main button on P0.26 (short=pairing, long 3s=off)
   BLE advertising as "Mic-Sense"
   [STAT] SPL=40 thr=75 state=IDLE alerts=0 stream_sub=0 alert_sub=0
   ```
   - `SPL` = current sound level
   - `thr` = the loudness threshold for a cry
   - `state` = IDLE / RISING / ACTIVE
   - `stream_sub` / `alert_sub` = whether the app has connected and subscribed (0 = not yet, 1 = yes)

---

## First-time setup (set the threshold)

The two microphones produce sound numbers on **different scales**, so each one needs a quick one-time setup. This is normal and expected.

> **Note:** A self-calibration feature exists inside the firmware, but the current phone app does not have a Calibrate button wired to it yet, so calibration is **not used right now**. Instead, you set the alert level directly with the app's **Set Sound Threshold** slider. (Automatic or manual calibration is planned for a future version.)

1. Flash the firmware and open RTT.
2. Make some noise near the mic and confirm the `SPL` number **goes up** this proves the mic is being read.
3. Open the phone app and connect to **Mic-Sense**. In RTT you should now see `stream_sub=1` and `alert_sub=1`.
4. Watch the quiet-room `SPL` value, then make a loud, cry-like sound and note how high `SPL` climbs.
5. In the app, drag the **Set Sound Threshold** slider to a value that sits comfortably between "quiet room" and "loud cry."
6. Turn the **Alerts** toggle on and make sure the device shows **Connected**.

That's it a sound that crosses the threshold will now fire an alert.

### Typical threshold values

These differ because the two mics have different sensitivity, so the right slider value is different for each. **Both are correct** just set each one to fit its own range.

| Build | Quiet room (about) | Loud cry (about) | Good threshold (about) |
|-------|--------------------|------------------|------------------------|
| ICS-43434 | ~43 | ~80 | ~75 |
| MAX9814 | ~40 | varies with gain | ~45 |

---

## MAX9814 build extra notes

The analog mic has two small slider switches on the board that configure the mic itself (the firmware does not control these):

- **AGC slider → OFF.** AGC (Automatic Gain Control) flattens loud and quiet to the same level, which hides the difference a cry detector needs. Keep it **off**.
- **GAIN slider → higher gain.** This sets how strong the mic signal is:
  - 3.3 V (VDD) = 40 dB (lowest)
  - GND = 50 dB
  - floating = 60 dB (highest)

  More gain gives a wider, clearer gap between a quiet room and a cry. Pick the position that gives you a comfortable gap, then keep it fixed.

> If you ever change the gain slider, the sound scale shifts so re-check the threshold after moving it.

The MAX9814's analog output is wired to chip pin **P0.02 (ADC input AIN0)**. The firmware samples it at a steady 4,000 times per second and works out the loudness from that.

---

## Pin reference (for developers)

These are the connections the firmware relies on (NINA module pin → nRF chip pin):

| Function | NINA pin | nRF pin |
|----------|----------|---------|
| Power latch (EN_CONTROLL) | GPIO_5 | P0.17 |
| Main button | GPIO_42 | P0.26 |
| Digital mic clock (I²S SCK) | GPIO_20 | P0.31 |
| Digital mic word clock (I²S WS) | GPIO_24 | P0.30 |
| Digital mic data (I²S SD) | GPIO_23 | P0.29 |
| Analog mic output (MAX9814) | GPIO_18 | P0.02 (AIN0) |
| Battery gauge I²C data (SDA) | GPIO_44 | P0.27 |
| Battery gauge I²C clock (SCL) | GPIO_43 | P0.15 |

---

## Things to know / current limitations

- **Calibration is not active yet.** The firmware can self-calibrate the sound scale, but the current app has no Calibrate button to trigger it, so for now you set the alert level with the threshold slider. Automatic/manual calibration is planned for a future version.
- **It detects loudness, not crying specifically.** Any loud-enough sound near the mic will trigger an alert.
- **The battery percentage feature is currently turned off** in the firmware. Reading the battery gauge over I²C was interfering with the Bluetooth connection, so it's disabled for now and will be added back in a way that doesn't block Bluetooth.
- **Power-off only works on battery.** If USB-C is plugged in, the USB supply keeps the board awake even after a long-press, so test the long-press power-off on **battery only**.
- **This board uses the chip's internal clock** (it has no 32.768 kHz crystal). The firmware is configured for that; without it, Bluetooth connections drop shortly after connecting.

---

## Quick start summary

1. Build: `west build -b nrf52833dk/nrf52833 . --pristine --build-dir build`
2. Flash `build/merged.hex` with the Programmer app over J-Link.
3. Open RTT and confirm the sound level reacts to noise.
4. Connect the phone app, set the **Set Sound Threshold** slider between quiet and loud, and turn on **Alerts**.
5. Make a loud sound the alert fires in the app.
