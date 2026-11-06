# DrainGuard — standing rules for Claude Code sessions

Read `docs/spec.md` and `docs/DECISIONS.md` before starting any task. The decisions log
is authoritative: if an instruction conflicts with it, stop and ask rather than guessing.

**Where `docs/DECISIONS.md` and `docs/spec.md` differ, DECISIONS wins (D30).** The spec is
the original design document and is not rewritten as decisions supersede it.

## Firmware rules

- **No blocking.** No `delay()`, no `pulseIn()`, no busy-waits anywhere in `firmware/src`
  or `firmware/lib`. Timing comes from `millis()` / `micros()`. The single exception is
  the 10 µs `delayMicroseconds()` HC-SR04 trigger pulse in `hal_sensors.cpp` (D43);
  nothing else may busy-wait. `firmware/legacy/` is exempt — it is the archived v0.2
  prototype and is never built.
- **Pure logic goes in `firmware/lib/dg_core`.** No `Arduino.h`, no heap, fixed-size
  arrays. Configure it through `Params` structs passed in by the caller, never by reading
  globals, and unit-test it on the `native` environment.
- **Motor safety.** Never drive `MOTOR_IN1` and `MOTOR_IN2` HIGH at the same time. Always
  stop (both LOW) for 100 ms before reversing direction. Motor pins are driven LOW as the
  first statements in `setup()`.
- **Never commit `firmware/include/secrets.h`.** It is gitignored. `src/net_config.h` is
  the only file allowed to include it; `include/config.h` holds no secrets or Wi-Fi logic.
- **Never reveal a secret value (D58).** Wi-Fi passwords and ThingSpeak write keys must
  never appear in Serial output, source, tests, docs, plans, commit messages or reports —
  not even ones you read from `secrets.h`. Use placeholders such as `<WRITE_KEY>` and
  `<WIFI_PASS>`. This is a hard rule with no exceptions.

## Building on Windows

Run `pio` **without piping its output**. Piping through `tail` / `Select-Object` /
`Out-File` triggers Windows process-spawn failures on this machine. If you see

```
xtensa-esp32-elf-g++: error: CreateProcess: No such file or directory
```

it is not a code error — simply re-run the command. Builds are incremental and pick up
where they stopped.

## Verify before committing

```
cd firmware
pio run -e esp32
pio run -e wokwi
pio test -e native
```

All three must pass, with zero warnings from project code.
