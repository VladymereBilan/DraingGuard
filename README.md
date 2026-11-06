# DrainGuard

A self-clearing drain screen. Two ultrasonic sensors watch the water level upstream
and downstream of a trash screen; when the upstream level rises far enough above its
learned baseline for long enough, a motorised rake sweeps the screen clear, and the
firmware verifies the level actually dropped before standing down. Motor current is
monitored so a jammed rake backs off and retries instead of stalling.

Hardware is an ESP32 DevKit v1. The same firmware builds for real hardware, for the
Wokwi simulator, and — for the pure-logic parts — for the host machine so logic can be
unit-tested without a board.

**Status: scaffold (v0.3.0-scaffold).** The project structure, configuration, build
environments and simulator model are in place. Sensing, motor control, the state machine
and logging are not implemented yet.

## Layout

```
firmware/        PlatformIO project (run all pio commands from here)
  platformio.ini three environments: esp32, wokwi, native
  include/       config.h (all tunables), secrets.example.h
  src/           hardware-facing code (needs Arduino.h)
  lib/dg_core/   pure logic, compiles without Arduino.h, unit-tested natively
  test/          Unity tests, run on the native environment
  diagram.json   Wokwi circuit for the current firmware
  wokwi.toml     points the Wokwi extension at the wokwi build output
  legacy/        v0.2 Wokwi prototype, kept for reference - never built
app/             Expo mobile app (later step)
tools/           fake data writer + serial capture scripts (later step)
docs/            spec, decisions, wiring, calibration, trial logs
```

`firmware/legacy/` sits outside `src/`, `lib/`, `include/` and `test/`, so PlatformIO
never compiles it.

## Build, test, simulate

Run these from the `firmware/` directory.

```powershell
# build for real hardware (the default environment)
pio run -e esp32

# build the simulator variant (adds -DDG_WOKWI=1)
pio run -e wokwi

# run the pure-logic unit tests on this machine
pio test -e native

# flash and watch the serial output
pio run -e esp32 -t upload
pio device monitor -b 115200
```

To simulate: build `-e wokwi`, then press **F1 → Wokwi: Start Simulator** in VS Code.
`wokwi.toml` tells the extension to load `.pio/build/wokwi/firmware.elf`, and
`firmware/diagram.json` describes the circuit.

The first `pio run` downloads the ESP32 toolchain (several hundred MB). That is normal
and happens once per machine.

## Setup on a new machine

This project is developed across two Windows machines, so setup is deliberately
reproducible. On a fresh machine:

1. **Clone the repo.**

   ```powershell
   git clone https://github.com/VladymereBilan/DraingGuard.git
   ```

2. **Install VS Code** — https://code.visualstudio.com/

3. **Install the PlatformIO IDE extension** (`platformio.platformio-ide`). Opening the
   repo in VS Code will offer it, since it is listed in `.vscode/extensions.json`.
   Let it finish installing PlatformIO Core before continuing.

4. **Install the Wokwi for VS Code extension** (`wokwi.wokwi-vscode`), then request a
   free licence: **F1 → Wokwi: Request a New License**. It is free for personal and
   educational use and the licence is tied to your machine, so this step is needed on
   both machines.

5. **Install WinLibs GCC** (needed for the `native` unit-test environment; the ESP32
   builds use their own toolchain). Download the UCRT x86_64 build from
   https://winlibs.com/, extract it so that `gcc.exe` ends up in `C:\mingw64\bin`, and
   add `C:\mingw64\bin` to your `PATH`.

6. **Add PlatformIO Core to `PATH`** so `pio` works in any terminal:

   ```
   %USERPROFILE%\.platformio\penv\Scripts
   ```

7. **Create your local secrets file:**

   ```powershell
   copy firmware\include\secrets.example.h firmware\include\secrets.h
   ```

   Then edit `firmware\include\secrets.h` and fill in your Wi-Fi SSID/password and
   ThingSpeak channel ID / write key. This file is gitignored and must never be
   committed. The build will not compile without it, even for the `wokwi` environment
   (which uses the open `Wokwi-GUEST` network rather than your real credentials).

8. **Verify** — open a new terminal so the `PATH` changes take effect, then:

   ```powershell
   pio --version
   gcc --version
   cd firmware
   pio run -e esp32
   pio test -e native
   ```

## Conventions

- All tunable values live in `firmware/include/config.h`. Anything that must be measured
  on the real rig is marked `// CALIBRATE`.
- No `delay()` and no `pulseIn()` anywhere in `firmware/src` or `firmware/lib`. Timing is
  non-blocking and driven from `millis()`, so the safety checks never stall.
- Pure logic goes in `firmware/lib/dg_core` and must compile without `Arduino.h`, so it
  can be unit-tested on the host.
- Serial output is CSV: one `#`-prefixed banner, then the header
  `ms,U_cm,D_cm,diff_cm,state,current_A`, then data rows.

## Docs

- `docs/spec.md` — what the system must do
- `docs/DECISIONS.md` — design decisions and why
- `docs/wiring.md` — pin assignments and the hardware-only divider networks
- `docs/calibration.md` — how to measure the `// CALIBRATE` values
- `docs/trials/` — trial run logs
