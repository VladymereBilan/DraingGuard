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

## Test 7 — Wi-Fi, ThingSpeak, and the offline buffer

This is the only test that uses the network. Wokwi provides an open access point called
`Wokwi-GUEST` and real outbound internet, so the firmware genuinely uploads to ThingSpeak.

### How to send serial commands to the simulation

**The Wokwi Terminal panel in VS Code is output-only — you cannot type into it.** To send
commands you attach a second, interactive monitor to the simulator's forwarded serial
port. `firmware/wokwi.toml` enables that with:

```toml
rfc2217ServerPort = 4000
```

Do this in two windows:

**1. Start the simulator as usual** — **F1** → `Wokwi: Start Simulator`. Leave the
simulator tab **visible**; if you hide it, Wokwi pauses the simulation and you will lose
both the output and the port.

**2. Open a separate PowerShell**, go to the `firmware` folder, and run:

```powershell
cd C:\Users\ADMIN\Documents\DraingGuard\firmware
pio device monitor --port rfc2217://localhost:4000 --baud 115200 --filter default --filter send_on_enter --echo --eol LF
```

What each part is for:

| Flag | Why |
|---|---|
| `--port rfc2217://localhost:4000` | the port `wokwi.toml` forwards |
| `--baud 115200` | matches `cfg::SERIAL_BAUD` |
| `--filter send_on_enter` | buffers your typing and sends the whole line when you press **Enter**, so you can fix typos first |
| `--filter default` | keeps normal output handling, which naming any filter would otherwise replace |
| `--echo` | shows what you are typing (the firmware does not echo it back) |
| `--eol LF` | what Enter transmits; the firmware accepts CR, LF or CRLF, so this is belt-and-braces |

You should immediately see the firmware's CSV lines streaming in this window too — that
confirms the port is attached.

**3. Type the command and press Enter.** Two commands exist, and only in the Wokwi build:

| Command | Effect |
|---|---|
| `net off` | pretends Wi-Fi died — uploads stop, records pile up in flash |
| `net on` | pretends Wi-Fi came back — the backlog is flushed |

Case and spacing do not matter: `net off`, `NET OFF` and `  Net   Off  ` are all accepted.
Every accepted command is echoed back by the firmware so you know it landed:

```
#NET,cmd,net off
#NET,debug,net=off
```

A command it does not recognise answers `#NET,debug,unknown=<what you typed>`.

**To quit the monitor:** press **Ctrl+C** in that PowerShell window. That closes the
monitor only — the simulation keeps running, and you can re-attach with the same command.

> If the monitor cannot connect, the usual causes are: the simulator is not running, the
> simulator tab is hidden (so Wokwi paused it), or a previous monitor still holds port
> 4000 — close it with Ctrl+C first.

### Part A — boot and the first upload

**Do:** start the simulator and watch for about 40 seconds.

**Expect, in order:**

```
#NET,fs,mount,ok=1,formatted=1,slots=200,count=0,bad=0,acked=0
#NET,boot,id=1,dropped=0,ack=0,channel=3568459
#NET,wifi,connecting,ssid=Wokwi-GUEST
#NET,wifi,up,ip=10.13.37.2
#NET,ntp,ok,epoch=1793923200
#NET,up,ok,id=1
```

| Line | What it tells you |
|---|---|
| `#NET,fs,mount` | LittleFS came up. `formatted=1` on the very first run is normal — there was no filesystem yet. `count=0` means the ring is empty. |
| `#NET,boot,id=1` | boot counter. It increments every restart, so run 2 shows `id=2`. |
| `#NET,wifi,up` | associated, with the IP Wokwi handed out |
| `#NET,ntp,ok` | the clock is set — uploads need a real timestamp |
| `#NET,up,ok,id=N` | ThingSpeak accepted one record and gave it entry number N |

Uploads are spaced **15.5 s** apart (the free-tier limit), so you will see roughly four
`#NET,up,ok` lines per minute, not one per snapshot.

> The `#DBG,adc,...` line near the top shows both ADC paths. In Wokwi `mv_cal` sticks at
> ~142 regardless of the knob while `mv_scaled` tracks it — that is why the simulator
> build uses the scaled reading.

### Part B — go offline and watch the ring fill

**Do:**

1. In the PowerShell monitor window, type `net off` and press Enter.
2. Wait about **3 real minutes**. Do not touch anything else.

> **The simulator runs at roughly 45 % of real speed**, so simulated time passes much more
> slowly than your wall clock. Three minutes of your time is only about 80 s of firmware
> time — enough for 4 snapshots at one every 20 s. Judge progress by the `ms` column in
> the CSV output and by the `#NET,ring,append,count=` lines, not by your watch. The same
> correction applies to every wait in this document.

**Expect:**

```
#NET,debug,net=off
#NET,wifi,down,debug_off
```

Then — and this is the point of the test — **the CSV lines keep coming every 200 ms**. The
control loop is completely unaffected; sensing, clearing and the LEDs carry on as normal
because all the networking lives on the other core.

**Immediately** you will see the records going to flash, one line per write:

```
#NET,pending,to_ring,n=1,why=link_down,ring=1
#NET,ring,append,count=2
#NET,ring,append,count=3
```

Every record made while offline is written to flash straight away — none of them wait in
RAM, and none are coalesced away. The `count=` number grows by one per snapshot (one every
20 s of *simulated* time), so after ~3 real minutes expect roughly **4–5**.

Check it in the 60-second diagnostics line:

```
#NET,STAT,heap=215432,minheap=198440,stack_hw=2104,ring=7,ok=4,fail=0,drop=0
```

| Field | Meaning |
|---|---|
| `heap` / `minheap` | free heap now, and the worst it has ever been |
| `stack_hw` | bytes of the logger task's 6 KB stack never used — if this ever nears 0, the stack is too small |
| `ring` | records waiting in flash |
| `ok` / `fail` | uploads accepted / rejected |
| `drop` | **must stay 0** — records lost to a full queue or a full ring |

### Part C — come back online and flush

**Do:** type `net on` in the PowerShell monitor window and press Enter. Wait about 30 s.

**Expect:**

```
#NET,debug,net=on
#NET,wifi,connecting,ssid=Wokwi-GUEST
#NET,wifi,up,ip=10.13.37.2
#NET,bulk,n=4,ok,ring=0
```

**One** `#NET,bulk` line clears the whole backlog — a bulk call carries up to 60 records in
a single request, so `n=` should equal the `count=` the ring reached. `ring=0` means flash
is empty again, and the next `#NET,STAT` confirms `ring=0` with `drop=0`.

> **This is the check that matters.** `n=` must equal the number of snapshots you saw go
> into the ring. If three snapshots went offline and only one came back, records were
> being coalesced while offline — the bug fixed in D65.

### Part D — check the channel

Open **https://thingspeak.com/channels/3568459** in a browser.

**Expect:** the Field 1/2/3 charts show your levels, and — the thing worth checking — the
backlog entries appear at **the times they were recorded**, not the time they were
uploaded. The gap you created with `net off` is filled in correctly and in order, with no
duplicates and no entries out of sequence. That is what the stored timestamps are for.

Click **Data Import / Export → Export** if you want the raw rows; field 8 is the dropped
counter and should read 0 throughout.

> **If `net on` produces `#NET,bulk,...,fail,code=400`:** that usually means duplicate
> timestamps, which happens if you restarted the simulation and re-sent records the
> channel already has. ThingSpeak rejects an entire batch on a duplicate. Clearing the
> channel's data (Channel Settings → Clear Channel) resets it.
>
> **If you see `code=429`:** you uploaded too fast. The firmware spaces calls 15.5 s
> apart, so this only appears if something else is writing to the same channel — for
> example the fake-data writer from `tools/`.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `current_A` is a large negative number | the pot sits below the zero the firmware calibrated — re-centre it and restart the simulation |
| A clearing cycle starts on its own | the "up" slider is still below 28 cm |
| Nothing happens when you move a slider by 1 cm | the threshold is 2 cm; you need at least 3 cm of difference |
| `state` never leaves 6 | the 30 s baseline window; wait it out |
| `state` never leaves 5 and then goes to 4 | the rake never reached home — check the HOME button is not stuck |
| `#STAT,a,b,c,d,e,f,g` | diagnostics: worst loop µs, then the sensors / safety-tick / control-tick / serial µs that make it up, then events dropped (must stay 0) and the current zero in mV. Under 10 ms is acceptable in the simulator (D60); the real board is re-measured before freeze. |
| `#NET,...` lines never appear | the logger task failed to start — look for `#NET,begin,fail` at boot |
| `net off` / `net on` do nothing | you typed into the Wokwi Terminal panel, which is output-only — use the PowerShell `pio device monitor` window instead (test 7) |
| the monitor will not connect to port 4000 | the simulator is not running, its tab is hidden (Wokwi pauses a hidden tab), or an earlier monitor still holds the port — Ctrl+C that one first |
| you see `#NET,debug,unknown=...` | the firmware got your line but does not know that command; only `net off` and `net on` exist |
| `#NET,wifi,connecting` repeats forever | Wokwi's `Wokwi-GUEST` network needs the simulator to have internet access; check the Wokwi licence is active |
| `#NET,STAT` shows `drop` climbing | records are being lost: either the queue filled (control loop far ahead of the logger) or the 200-slot ring wrapped while offline |
