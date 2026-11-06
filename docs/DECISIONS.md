# Decisions

D1  PlatformIO + Wokwi VS Code ext; envs esp32 / wokwi / native; espressif32@6.9.0 (Arduino core 2.0.17).
D2  Pure logic in lib/dg_core (no Arduino), configured via Params structs, unit-tested natively.
D3  Fast safety loop ~10 ms (switches, current, jam, motor dead-time) + 200 ms controller tick.
D4  HC-SR04 echo timed by interrupt + micros(); no pulseIn().
D5  Snapshot every 20 s in ALL states; an event record replaces the next snapshot.
D6  State codes 5=HOMING, 6=BASELINE, uploaded to field 4; paper table updated to match.
D7  Baseline + cyclesSinceEmptied in NVS; LittleFS only for the ring buffer.
D8  Ring records carry bootId + uptime; epoch backfilled after NTP; unsynced old-boot records dropped and counted.
D9  RESOLVED: ThingSpeak over plain HTTP; channels public-read; write keys only on device + fake writer.
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
D25 A sensor is invalid after 10 pushes with no new median (~1.2 s for a total dropout); the controller must not start clearing on invalid data.
D26 CLAUDE.md holds the standing rules for Claude Code sessions.
D27 Speed of sound 0.0343 cm/us, no temperature compensation.
D28 Channels: DrainGuard-dev = 3568459 (Wokwi + fake writer), DrainGuard = 3568466 (real hardware + trials). Firmware uses dev until the trial run.
D29 Control split: a fast MotionSupervisor (10 ms) executes motion jobs; the Controller (200 ms) owns states and records. Final shape set by the S2 plan.
D30 Where docs/DECISIONS.md and docs/spec.md differ, DECISIONS wins.
D31 Event 5 = baseline rejected (saved kept); event 6 = jam cleared by retry (fault 1 record).
D32 Fault 5 = sensor fault: either sensor invalid continuously for SENSOR_FAULT_MS (60 s) in BASELINE, MONITORING, CONFIRMING or SETTLING -> FAULT 5. No unbounded holds.
D33 FAULT auto-recovery (diff <= baseline + VERIFY_MARGIN_CM for 60 s, both sensors valid) applies only to faults 3 and 5. Faults 2 and 4 clear only with the 3 s button hold.
D34 After the post-fault re-home: -> MONITORING if a baseline exists, else -> BASELINE.
D35 Field 5 = cyclesSinceEmptied, incremented on each completed sweep (DONE / DONE_AFTER_RETRY); the per-blockage cycle count (max 3) is internal.
D36 Fault 1 is informational: set on DONE_AFTER_RETRY, persists through the episode, reset to 0 on event 1 or event 3; overwritten by 2/3/4/5.
D37 Short press = bin emptied in any state; the 3 s hold acts only in FAULT; a hold never also counts as a short press.
D38 Yellow LED = motor moving: HOMING and CLEARING, including jam recovery.
D39 Events are never dropped: the Controller queues pending events (capacity >= 4) and emits at most one record per tick, events before snapshots.
D40 Motion jobs check switches before moving: HOME with home already pressed -> DONE, no motor movement; SWEEP with end already pressed skips the lower stroke; home and end pressed together at any time -> STOP + FAILED_TIMEOUT.
D41 Accepted from the S2 plan: CONFIRMING aborts to MONITORING on invalid data; jam during HOME -> immediate FAILED_TIMEOUT; no jam detection during the 0.5 s reverse pulse; records carry the filter's frozen value when a sensor is invalid.
D42 ACS712 zero is calibrated at boot with the motor stopped (64-sample avg). If it is outside ACS_ZERO_MIN/MAX_MV, use 1667 mV and print a "#WARN" line; no fault code. Travel timeouts still protect the motor.
D43 The 10 us HC-SR04 trigger pulse (delayMicroseconds) is the only permitted busy-wait.
D44 Wokwi build only (DG_WOKWI): a simulated rake drives the limit-switch inputs from the motor commands (3 s full stroke, starts at mid-travel); the HOME/END buttons can still force a switch pressed.
D45 Serial: one CSV line per controller tick (200 ms) "ms,U_cm,D_cm,diff_cm,state,current_A"; each Record also prints as "#REC,ms,U,D,diff,state,cycles,fault,event"; "#STAT,..." every 10 s; all lines CRLF.
D46 Event records carry the state the event happened in (fault records read 4), as implemented in S2b.
D47 A limit switch counts as pressed after 2 consecutive LOW reads (20 ms).
