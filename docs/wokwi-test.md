# Wokwi manual test script

A step-by-step walkthrough of the whole system in the simulator. No hardware needed.
Written for someone who has never used Wokwi before — every step says exactly what to
click and exactly what you should see.

Allow about 15 minutes for all six tests.

---

## Before you start

### Build and launch

```powershell
cd firmware
pio run -e wokwi
```

Then in VS Code press **F1** → type `Wokwi: Start Simulator` → Enter. The circuit opens in
a panel, and the Serial Monitor appears at the bottom.

If the build fails with `CreateProcess: No such file or directory`, that is a known
Windows quirk on this machine — just run `pio run -e wokwi` again.

### What you are looking at

| Part | Where | What it is |
|---|---|---|
| **ESP32** | left | the board running the firmware |
| **"up" HC-SR04** | top right | **upstream** sensor — the one *before* the screen |
| **"down" HC-SR04** | below it | **downstream** sensor — the one *after* the screen |
| Green LED "STATUS" | right | heartbeat — blinks whenever the system is alive |
| Red LED "FAULT" | right | a fault is latched |
| Yellow LED "CLEARING" | right | the motor is moving |
| Blue LED "IN1 LOWER" | right | rake is being lowered |
| White LED "IN2 RAISE" | right | rake is being raised |
| Potentiometer | left | stands in for the ACS712 **motor current** sensor |
| HOME / END / BIN buttons | left | limit switches and the bin button |

### The one idea you must hold on to

Each HC-SR04 has a **distance slider**. Click the sensor part in the simulator and a
slider appears showing a distance in centimetres — that is how far the sensor thinks the
water surface is below it.

> **A SMALLER distance means HIGHER water.**

The sensors are mounted 40 cm above the dry channel bed, so:

```
water level = 40 cm − slider distance
```

Both sliders start at 30 cm, so both levels start at 10 cm and the **differential**
(upstream level − downstream level) starts at **0 cm**. A blockage is what makes water
pile up *upstream*, so to fake a blockage you **drag the "up" slider DOWN to a smaller
number**.

### Reading the Serial Monitor

Four kinds of line:

| Line | Meaning |
|---|---|
| `ms,U_cm,D_cm,diff_cm,state,current_A` | the header, printed once |
| `31400,10.00,10.00,0.00,0,0.012` | a data line, every 0.2 s |
| `#REC,...` | a **record** — the things that get logged/uploaded |
| `#STAT,...` / `#WARN,...` / `#INFO,...` | diagnostics |

The `state` column is the number you will watch most:

| 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| MONITORING | CONFIRMING | CLEARING | SETTLING | FAULT | HOMING | BASELINE |

A `#REC` line ends with an **event** code:

| 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| snapshot | cleared & verified | fault | fault recovered | bin emptied | baseline rejected | jam cleared by retry |

### About the potentiometer

The pot stands in for the ACS712 current sensor, and the firmware measures its "zero" in
the first moment after boot. The circuit now starts it at its **middle position**
(`"value": "512"` in `diagram.json`), so you should not have to touch it: the
`current_A` column reads close to **0.000** and no warning appears.

**Keep the pot centred except during test 4**, or you will trigger phantom jams.

If you *do* move it before the firmware boots and then restart, you may see:

```
#WARN,acs_zero_out_of_range,measured_mV=0,using_mV=1667
```

That means the reading was outside the believable window, so the firmware fell back to
the nominal zero of 1667 mV and carried on (no fault is raised). Re-centre the pot and
restart the simulation to clear it.

---

## Test 1 — Boot: homing, baseline, monitoring

**Do:** nothing. Just start the simulator and watch.

**Expect, in order:**

1. A banner, then the storage and header lines:

   ```
   # DrainGuard 0.4.0-s3 env=WOKWI
   #INFO,baseline_valid=0,baseline_cm=0.00,cycles=0
   ms,U_cm,D_cm,diff_cm,state,current_A
   ```

2. **`state` = 5 (HOMING)** for about 1.5 seconds. The **yellow LED is ON** and the
   **white LED (IN2 RAISE) is ON** — the simulated rake starts half way down the screen
   and is winching itself back to the home switch.

3. **`state` = 6 (BASELINE)** for **30 seconds**. Yellow LED goes off, green LED keeps
   blinking. The firmware is averaging the differential to learn what "clear" looks like.
   You will see one `#REC,...,6,0,0,0` snapshot land during this window.

4. **`state` = 0 (MONITORING)**. Both levels read 10.00, `diff_cm` reads 0.00.

> **Checkpoint:** you are in state 0, the green LED blinks, every other LED is dark.
> Every test below starts from here.

---

## Test 2 — A blockage that clears in one cycle

**Do:**

1. Click the **"up"** sensor (top right) and drag its slider from **30 cm down to 27 cm**.
   That raises the upstream level from 10 cm to 13 cm, so `diff_cm` becomes about 3.00 —
   above the 2 cm threshold.
2. **Wait and do not touch anything for 10 seconds.**
3. When `state` becomes 2, **drag the "up" slider back to 30 cm**. (In real life the rake
   clearing the screen is what makes the water drop; here you play the part of the water.)
4. Wait about 20 more seconds.

**Expect:**

| Time | What you see |
|---|---|
| immediately | `diff_cm` rises to ~3.00, **`state` = 1 (CONFIRMING)** |
| after 10 s | **`state` = 2 (CLEARING)**, **yellow LED ON** |
| | **blue LED (IN1 LOWER) ON** for 3 s — rake sweeping down |
| | brief pause, then **white LED (IN2 RAISE) ON** for 3 s — rake returning |
| ~6 s later | **`state` = 3 (SETTLING)**, yellow LED off |
| after 15 s | **`state` = 0** and a record with **event 1**: |

```
#REC,58200,10.00,10.00,0.00,3,1,0,1
                              ^ ^ ^ ^
                              | | | event 1 = cleared and verified
                              | | fault 0 = all good
                              | cycles = 1 (one sweep since the bin was emptied)
                              state 3 = the verdict was made in SETTLING
```

> **Why state 3 on the record?** The record says where the event *happened*. The "cleared
> and verified" decision is made at the end of SETTLING, so it is stamped state 3, and the
> very next data line shows state 0.

---

## Test 3 — Still blocked after 3 cycles, then clear the fault by hand

**Do:**

1. Drag the **"up"** slider down to **27 cm** again.
2. **Leave it there.** Do not put it back this time.
3. Wait about **80 seconds** while the rake tries three times.
4. When the red LED comes on, **click and HOLD the BIN button for a full 3 seconds**, then
   release.

**Expect:**

| Time | What you see |
|---|---|
| after 10 s | state 1 → **state 2**, sweep 1 (blue then white LED) |
| | **state 3** for 15 s, water still high → **state 2** again, sweep 2 |
| | **state 3** for 15 s, still high → **state 2** again, sweep 3 |
| after 3rd settle | **`state` = 4 (FAULT)**, **red LED ON**, **green LED OFF**, and: |

```
#REC,142600,13.00,10.00,3.00,4,4,3,2
                             ^ ^ ^ ^
                             | | | event 2 = fault
                             | | fault 3 = still blocked after 3 cycles
                             | cycles = total sweeps since the bin was emptied
                             state 4 = FAULT
```

> The `cycles` column counts **every** completed sweep since the last bin press, not the
> three cycles of this blockage. If you ran test 2 first it reads 4 (1 + 3); on a fresh
> boot it reads 3. Test 5 resets it to 0.

Then after you hold BIN for 3 s:

| | What you see |
|---|---|
| on release | **`state` = 5 (HOMING)**, yellow LED on, rake re-homes |
| ~1 s later | a record with **event 3** (fault recovered), then **`state` = 0** |

```
#REC,151400,13.00,10.00,3.00,5,3,0,3
```

The green LED starts blinking again and the red LED goes out. The learned baseline is
**not** re-measured — the firmware goes straight back to monitoring.

> Put the "up" slider back to 30 cm before the next test, or the system will immediately
> start another clearing cycle.

---

## Test 4 — A jam, cleared by a retry

This is the one test where you touch the potentiometer.

**Do:**

1. Drag the **"up"** slider down to **27 cm** and wait 10 s for **state 2 (CLEARING)**.
2. Watch the LEDs. Wait until the **white LED (IN2 RAISE)** comes on — that is the rake
   *raising*, which is the only stroke where current is used to detect a jam. On the
   down stroke the firmware ignores current entirely, so turning the pot then does
   nothing.
3. **While the white LED is on, turn the potentiometer slightly CLOCKWISE** — about
   **+100 mV above centre**, which is roughly 3 % of the knob's travel. That is 0.8 A,
   the jam threshold (`max(1.5 × 0.5 A, 0.5 A + 0.3 A)`), because the scale is
   ~123 mV per amp.
4. Watch `current_A` in the Serial Monitor climb above **0.800**. A bigger turn is fine —
   anything over the threshold counts as a jam.
5. After about half a second, **turn the pot back to the middle** — you have simulated the
   obstruction coming loose.
6. Drag the **"up"** slider back to **30 cm** and wait.

**Expect:**

| | What you see |
|---|---|
| pot up | `current_A` climbs past 0.800 |
| ~0.2 s later | the motor **stops**, then the **blue LED flashes for 0.5 s** (backing off), then everything is dark for 1 s (settling) |
| then | **white LED on again** — the retry |
| rake reaches home | a record with **event 6** and **fault 1**: |

```
#REC,201800,13.00,10.00,3.00,2,5,1,6
                             ^ ^ ^ ^
                             | | | event 6 = jam cleared by retry
                             | | fault 1 = informational: we fought a jam and won
                             | cycles = sweeps so far (whatever your running total is)
                             state 2 = it happened during CLEARING
```

| | |
|---|---|
| then | **state 3**, and 15 s later **event 1** with **fault back to 0** |

> Fault 1 is *informational*. The system does not stop; it finishes the job and tells you
> it had to work for it. If you had left the pot turned up, the rake would have retried
> twice and then given up with **fault 2** and a red LED.

---

## Test 5 — Bin emptied

**Do:** click the **BIN** button once, briefly (a quick click — do **not** hold it).

**Expect:** immediately, one record with **event 4** and `cycles` reset to **0**:

```
#REC,215600,10.00,10.00,0.00,0,0,0,4
                             ^ ^   ^
                             | |   event 4 = bin emptied
                             | cycles = 0 (the counter was reset)
                             state 0 = you pressed it while monitoring
```

Nothing else changes — no LED change, no motor movement. A short press works in **any**
state, including FAULT.

> A *quick click* means "I emptied the bin". A *3 second hold* means "clear the fault".
> They never both happen from one press.

---

## Test 6 — Sensor fault

**Do:**

1. Click the **"down"** sensor and drag its slider to something **larger than 200 cm** —
   for example **300 cm**. The firmware treats anything beyond 200 cm as a missing echo.
2. Wait. You need to wait a full **60 seconds**.

**Expect:**

| Time | What you see |
|---|---|
| ~1.2 s | the downstream readings stop changing — `D_cm` **freezes** at its last good value |
| | `state` stays 0; nothing dramatic happens yet |
| after 60 s | **`state` = 4 (FAULT)**, **red LED ON**, and: |

```
#REC,290400,10.00,10.00,0.00,4,0,5,2
                             ^ ^ ^ ^
                             | | | event 2 = fault
                             | | fault 5 = sensor fault
                             | cycles = 0
                             state 4 = FAULT
```

**Then recover it:** drag the "down" slider back to **30 cm** and wait **60 more seconds**.
Because fault 5 clears itself once the sensors are healthy and the water is back at
baseline, you will get **state 5 (HOMING)** → **event 3** → **state 0** with no button
press at all.

> If you would rather not wait, the 3 second BIN hold clears it immediately.
>
> Note that during those 60 seconds the frozen `D_cm` value keeps being reported. That is
> deliberate: the firmware never invents a number, it reports the last one it trusted and
> refuses to make decisions on it.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `current_A` is a large negative number | the pot sits below the zero the firmware calibrated — re-centre it and restart the simulation |
| A clearing cycle starts on its own | the "up" slider is still below 28 cm |
| Nothing happens when you move a slider by 1 cm | the threshold is 2 cm; you need at least 3 cm of difference |
| `state` never leaves 6 | the 30 s baseline window; wait it out |
| `state` never leaves 5 and then goes to 4 | the rake never reached home — check the HOME button is not stuck |
| `#STAT,a,b,c,d,e,f,g` | diagnostics: worst loop µs, then the sensors / safety-tick / control-tick / serial µs that make it up, then events dropped (must stay 0) and the current zero in mV. The first number should stay under 5000. |
