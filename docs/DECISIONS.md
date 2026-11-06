# Decisions

D1  PlatformIO + Wokwi VS Code ext; envs esp32 / wokwi / native; espressif32@6.9.0 (Arduino core 2.0.17).
D2  Pure logic in lib/dg_core (no Arduino), configured via Params structs, unit-tested natively.
D3  Fast safety loop ~10 ms (switches, current, jam, motor dead-time) + 200 ms controller tick.
D4  HC-SR04 echo timed by interrupt + micros(); no pulseIn().
D5  Snapshot every 20 s in ALL states; an event record replaces the next snapshot.
D6  State codes 5=HOMING, 6=BASELINE, uploaded to field 4; paper table updated to match.
D7  Baseline + cyclesSinceEmptied in NVS; LittleFS only for the ring buffer.
D8  Ring records carry bootId + uptime; epoch backfilled after NTP; unsynced old-boot records dropped and counted.
D9  ThingSpeak over plain HTTP; channel public-read; write key only on device + fake writer. [PENDING]
D10 Current jam detection on raise only; down uses travel timeout -> same retry path; I_run calibrated.
D11 stop() = IN1=IN2=LOW (coast); worm gear self-locks.
D12 Motor pins driven LOW as the first statements in setup().
D13 Separate dev ThingSpeak channel for fake data; app channel ID configurable.
D14 IN1 HIGH = lower (sweep down), IN2 HIGH = raise. If reversed on the bench, swap motor leads, not code.
D15 Boot baseline rejected if |new - saved| > threshold; keep saved, log it. First boot accepts any value.
D16 FAULT exit (3 s hold, or diff at baseline 60 s) -> HOMING -> MONITORING, event 3 after re-home; re-home fail -> fault 4.
D17 Travel timeout restarts on each retry attempt.
D18 v0.2 sketch archived in firmware/legacy/; its rake model becomes the native-test plant model.
D19 Teammate owns app/ day to day; Vlad owns firmware + hardware and reviews app changes.
D20 gnu++17 on all envs.
D21 Each sensor pinged every 100 ms, sensors staggered 50 ms.
D22 Default partition table (its data partition hosts LittleFS); revisit only if the logger needs more.
D23 No PWM: ENA jumper on, direction-only drive.
D24 config.h holds no secrets or Wi-Fi logic; that lives in src/net_config.h.
D25 A sensor with no valid median for 10 pushes is invalid; the controller must not start clearing on invalid data.
D26 CLAUDE.md holds the standing rules for Claude Code sessions.
D27 Speed of sound 0.0343 cm/us, no temperature compensation.
