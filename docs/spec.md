# DrainGuard — Project Spec

> Original spec, written Nov 6, 2026. Where `docs/DECISIONS.md` differs from this file, DECISIONS wins (D30).

## Project
DrainGuard: an IoT-based automated debris clearing system for a street drainage inlet, built as a bench prototype.
- Two HC-SR04 sensors measure water level upstream and downstream of an inclined screen. A sustained upstream–downstream differential means a blockage; both rising together means rain.
- An ESP32 then runs a JGY-370 worm gear motor (via an L298N) that winches a rake up the screen. An ACS712 detects jams.
- After each sweep it re-measures, for up to 3 cycles, then flags a fault.
- All control runs on the ESP32. Wi-Fi is only for logging to ThingSpeak.
- An Expo/React Native app reads ThingSpeak for monitoring only.

## Hardware and pins (ESP32 DevKit V1, CH340C)
| GPIO | Connection |
|---|---|
| 18 / 19 | Upstream Trig / Echo (Echo through a 2.2k/3.3k divider) |
| 21 / 22 | Downstream Trig / Echo (same divider) |
| 25 / 26 | L298N IN1 / IN2 (10k pull-downs, ENA jumper on, no PWM) |
| 32 | Home limit switch (INPUT_PULLUP, LOW = pressed) |
| 33 | End-of-travel limit switch (INPUT_PULLUP, LOW = pressed) |
| 27 | "Bin emptied" button (resets cycle count; 3 s hold clears a fault) |
| 34 | ACS712 5A through a 10k/20k divider: SENS = 0.185 × 20/30 V/A, zero calibrated at boot, 64-sample average using analogReadMilliVolts |
| 23 | Status LED |
| 13 | Red fault LED |
| 14 | Yellow clearing LED |

## Firmware architecture (non-blocking, no delay() anywhere)
- **config.h**: every pin, threshold and timing in one place. Wi-Fi and API keys go in a gitignored secrets.h, with a committed secrets.example.h.
- **sensors**: trigger the two sensors 50 ms apart; reject readings outside 2–200 cm; median of 5, then a 5-sample moving average. Level = mount height − distance.
- **motor**: raise(), lower(), stop(), currentA(). Never drive IN1 and IN2 HIGH together. Always stop() for 100 ms before reversing.
- **controller**: state machine ticked every 200 ms via millis().
- **logger**: FreeRTOS task pinned to core 0, fed by a queue. The controller calls xQueueSend(q, &rec, 0) and never blocks.
  - Uploads are spaced at least 15.5 s apart (ThingSpeak free tier).
  - If an upload fails, the record goes to a LittleFS ring buffer: 200 records, oldest overwritten, dropped-record counter.
  - The buffer is flushed via POST /channels/<id>/bulk_update.json with created_at timestamps.
  - Time comes from NTP: configTime(8*3600, 0, "pool.ntp.org").
- **Serial**: one CSV line per sample: `ms,U_cm,D_cm,diff_cm,state,current_A`.

### States
| State | What it does | Moves on when |
|---|---|---|
| HOMING | Raise until the home switch closes (15 s timeout) | Home pressed → BASELINE; timeout → FAULT (code 4) |
| BASELINE | Average the differential for 30 s, save to flash | Done → MONITORING |
| MONITORING (0) | Sample levels; upload a snapshot every 20 s | diff > baseline + threshold → CONFIRMING |
| CONFIRMING (1) | Check the differential stays above threshold for the confirm time | Held → CLEARING; dropped → MONITORING |
| CLEARING (2) | Lower to the end switch, then raise to home | Home reached → SETTLING |
| SETTLING (3) | Wait the settle time, then re-measure | diff ≤ baseline + 1.0 cm → MONITORING (event: cleared, verified); otherwise cycle + 1 and back to CLEARING; after 3 cycles → FAULT (code 3) |
| FAULT (4) | Motor off, red LED, fault uploaded | 3 s button hold, or diff back to baseline for 60 s → MONITORING (event 3) |

Jam handling:
- Jam = current ≥ max(1.5 × I_run, I_run + 0.3 A) for 200 ms, with 300 ms inrush blanking after the motor starts.
- Recovery: reverse 0.5 s, pause 1 s, retry. At most 2 retries per cycle; beyond that, fault code 2.
- On the way down, the 15 s travel timeout is the jam detector, because a stuck rake just leaves the cord slack.

Starting parameters (tuned during calibration):

| Parameter | Start value |
|---|---|
| Threshold | 2.0 cm |
| Confirm time | 10 s |
| Settle time | 15 s |
| Travel timeout | 15 s each way |
| Max cycles | 3 (fixed) |
| Max retries per cycle | 2 (fixed) |

### ThingSpeak fields
| Field | Content |
|---|---|
| 1 | Upstream level (cm) |
| 2 | Downstream level (cm) |
| 3 | Differential (cm) |
| 4 | State code (0–4, as above) |
| 5 | Cycles since the bin was emptied |
| 6 | Fault code: 0 none, 1 jam cleared by retry, 2 jam not cleared, 3 still blocked after 3 cycles, 4 homing failed |
| 7 | Event: 0 snapshot, 1 cleared and verified, 2 fault, 3 fault recovered, 4 bin emptied |
| 8 | Dropped-record counter |

## Mobile app (Expo, TypeScript)
- One shared hook, useFeed, polls `channels/<id>/feeds.json?results=240` every 15 s.
- Status screen: levels, differential, and a state badge. Show "Device offline" if the newest entry is more than 2 minutes old.
- Trend screen: upstream and downstream lines over the last hour.
- History screen: entries where field 7 ≠ 0, newest first.
- Alerts screen: fault banner, a bin reminder at 10 cycles, and a local notification (expo-notifications) the first time a new fault appears.
- Develop against a fake-data writer first. The app never talks to the ESP32.
- Ship an installable APK via EAS (preview profile, buildType apk).

## Status and deadline (as of Nov 6, 2026)
- The control logic has been validated in Wokwi.
- Received: L298N, ACS712, JGY-370, 1 HC-SR04, pump, 12 V 5 A adapter, LM2596.
- The ESP32 and the second HC-SR04 are still in shipping (expected around Nov 8).
- Priorities until the ESP32 arrives:
  - Finish the firmware in Wokwi, including Wi-Fi, ThingSpeak, the offline buffer and the full state machine.
  - Set up the ThingSpeak channel.
  - Build the app against fake data.

| Milestone | Date |
|---|---|
| Hardware integration | Nov 8–12 |
| Calibration | Nov 12–14 |
| Feature freeze (bug fixes only after this) | Nov 15 |
| 30 evaluation trials | Nov 15–20 |
| Defense | Nov 23–27 |