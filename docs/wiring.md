# Wiring

ESP32 DevKit v1. All GPIO numbers match `firmware/include/config.h` — that header is the
single source of truth; this table is the human-readable view of it.

## Pin assignments

| Signal | GPIO | Direction | Notes |
|---|---|---|---|
| `US_TRIG` | 18 | output | Upstream HC-SR04 trigger |
| `US_ECHO` | 19 | input | Upstream HC-SR04 echo — **via 2.2k/3.3k divider on hardware** |
| `DS_TRIG` | 21 | output | Downstream HC-SR04 trigger |
| `DS_ECHO` | 22 | input | Downstream HC-SR04 echo — **via 2.2k/3.3k divider on hardware** |
| `MOTOR_IN1` | 25 | output | H-bridge IN1 — HIGH = lower (rake sweeps down) |
| `MOTOR_IN2` | 26 | output | H-bridge IN2 — HIGH = raise (rake returns home) |
| `SW_HOME` | 32 | input, pull-up | Home / fully-raised limit switch, active LOW |
| `SW_END` | 33 | input, pull-up | End-of-travel / fully-lowered limit switch, active LOW |
| `BTN_BIN` | 27 | input, pull-up | "Bin emptied" button, active LOW; hold 3 s to clear a fault |
| `ACS712` | 34 | analog input | Motor current — **via 10k/20k divider on hardware**. ADC1, input-only pin |
| `LED_STATUS` | 23 | output | Green — heartbeat / monitoring |
| `LED_FAULT` | 13 | output | Red — fault latched |
| `LED_CLEARING` | 14 | output | Yellow — clearing cycle running |

Each LED needs a 220 Ω series resistor. The switches and the button are wired between
their GPIO and GND; `INPUT_PULLUP` supplies the pull-up, so a closed contact reads 0.

## Power

HC-SR04 modules are 5 V parts. Run their `VCC` from the board's `VIN` (5 V) rail, not
from `3V3` — at 3.3 V they range unreliably or not at all.

## Hardware-only divider networks

Two resistor dividers exist **only on the physical build, not in the Wokwi model**:

| Divider | Where | Why |
|---|---|---|
| 2.2k / 3.3k | Each HC-SR04 `ECHO` → ESP32 | The echo pin swings to 5 V. The ESP32 GPIO is 3.3 V and not 5 V-tolerant. The divider drops 5 V to ≈3.0 V. |
| 10k / 20k | ACS712 output → GPIO 34 | The ACS712 output is centred on 2.5 V and swings above it. The divider scales it by 20/(10+20) to stay inside the ADC range. That same 20/30 factor appears in `cfg::ACS_SENS_V_PER_A`. |

In `firmware/diagram.json` the echo pins connect straight to the ESP32 and a
potentiometer stands in for the ACS712, because the simulator has no 5 V damage model
and no ACS712 part. **When you move from simulation to hardware, fit both dividers
before powering up**, or the echo pins will see 5 V.

## Simulator stand-ins

**The Wokwi board has only two ground pins.** `wokwi-esp32-devkit-v1` exposes exactly
`GND.1` and `GND.2` — there is no `GND.3`, even though the physical DOIT board has three
grounds. A connection naming a pin the board does not have is not a warning you will see:
it silently takes out the connections declared after it, which looks like dead sensors and
a dead ADC. The authoritative pin list is
[wokwi-boards/boards/esp32-devkit-v1/board.json](https://github.com/wokwi/wokwi-boards/blob/main/boards/esp32-devkit-v1/board.json);
check new pin names against it rather than against the silkscreen.

| Hardware | Wokwi stand-in |
|---|---|
| ACS712 current sensor | `wokwi-potentiometer` on GPIO 34 |
| Limit switches / bin button | `wokwi-pushbutton` to GND, labelled HOME / END / BIN |
| H-bridge + motor | Blue LED on GPIO 25 (IN1, lower) and white LED on GPIO 26 (IN2, raise) |
| Echo level shifters | Omitted — direct connection |
