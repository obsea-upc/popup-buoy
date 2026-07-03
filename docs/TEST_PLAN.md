# Pop-Up Buoy — Test Plan

This is a plan, not an implementation: it lays out how to get automated test coverage over the
buoy firmware described in [ARCHITECTURE.md](ARCHITECTURE.md), given that today the project has
**zero test infrastructure** (plain Arduino-IDE sketch, no PlatformIO, no test framework, all
logic entangled with `Serial`/`SD`/`WiFi`/`EEPROM` globals).

For the companion server's test plan (and the buoy↔server integration tests), see
[../../popup-server/docs/TEST_PLAN.md](../../popup-server/docs/TEST_PLAN.md).

## Implementation status

| Phase | Status |
|---|---|
| Phase 1 (`previpass`/`previpass_util` native tests) | **Implemented** — `test/native/test_previpass_util.c`, `test_previpass_status.c`, `test_previpass_pass_prediction.c`, `test_previpass_known_issues.c`. Compiled against the real, unmodified `previpass.c`/`previpass_util.c` with ASan+UBSan. Found a real out-of-bounds read, see [ARCHITECTURE.md#known-rough-edges](ARCHITECTURE.md#known-rough-edges). |
| Phase 2 (parser/EEPROM/line-split extractions) | **Implemented and wired in.** `src/logic/parsers.{h,cpp}`, `gps_mask.{h,cpp}`, `misc.{h,cpp}` mirror the original `.ino` functions' behavior exactly (including one bug found along the way, see ARCHITECTURE.md), tested in `test/native/test_parsers.cpp`, `test_gps_mask.cpp`, `test_misc.cpp`. `popup-buoy.ino` now delegates `parseTimeResponse`, `parsePermissionResponse`, `parseSyncTimeResponse`, `maskGPS` (the active `WORK_ADC` branch), `splitLineProgressFile`, `splitLineDataFile`, `splitLineSuccessFile`, `ChangeSecondsInHoursAndMinutes`, and `eepromSaveTimeCoverage` to the tested pure functions instead of duplicating the logic inline. Verified with a real `arduino-cli compile --fqbn esp32:esp32:esp32` against ESP32 core 2.0.14 (see below) — not just "should compile." |
| Phase 3 (retry/backoff/battery-transition extraction) | Not implemented this round. |
| Phase 4 (state machine table) | Not implemented this round. |
| Phase 5 (HIL) | Not implemented — requires physical hardware. |

Run the native (host-machine) tests with:

```bash
test/native/run_tests.sh
```

No PlatformIO, no ESP32 toolchain, no network access needed for that — just `gcc`/`g++` and `libm`.

To verify the actual firmware still compiles for the ESP32 target after changing anything in
`src/logic/` or the functions in `popup-buoy.ino` that call it, install `arduino-cli` + ESP32 core
2.0.14 + the sketch's libraries (`RTClib`, `NTPClient`, `TinyGPSPlus`, `FastCRC`,
`EspSoftwareSerial`) and run:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 .
```

`src/` is the Arduino-standard subfolder for extra sketch source files that need real
subdirectory structure — the sketch's own top-level folder is only scanned for `.cpp`/`.c` files
non-recursively, but `src/` is compiled recursively, which is why `src/logic/` (not a top-level
`logic/`) is where the extracted files live.

## Table of contents

- [Constraints](#constraints)
- [Strategy](#strategy)
- [Phase 0 — toolchain](#phase-0--toolchain)
- [Phase 1 — pure-logic unit tests (no refactor needed)](#phase-1--pure-logic-unit-tests-no-refactor-needed)
- [Phase 2 — unit tests requiring a small extraction refactor](#phase-2--unit-tests-requiring-a-small-extraction-refactor)
- [Phase 3 — hardware-abstraction tests (mocked peripherals)](#phase-3--hardware-abstraction-tests-mocked-peripherals)
- [Phase 4 — state machine tests](#phase-4--state-machine-tests)
- [Phase 5 — hardware-in-the-loop (HIL) tests](#phase-5--hardware-in-the-loop-hil-tests)
- [Test data fixtures](#test-data-fixtures)
- [CI](#ci)
- [Prioritization](#prioritization)

## Constraints

- `popup-buoy.ino` is a single ~2900-line sketch: state machine, business logic, and hardware
  I/O (`SD`, `WiFi`, `HTTPClient`, `KIM`/`kimSerial`, `gpsSerial`, `rtcExt`, `EEPROM`) are all
  interleaved in the same functions, using file-scope globals (`currentState`, `idBuoy`,
  `gpsLat`, ...).
- `previpass.c`/`previpass_util.c` are **already pure C with no Arduino dependency** — this is
  the easiest, highest-value place to start.
- `KIM.cpp`/`ESP32_FTPClient.cpp` are Arduino-`HardwareSerial`/`WiFiClient`-coupled drivers —
  testing them means mocking those interfaces, not testing real hardware.
- There is no build system that runs on a dev machine today; Arduino sketches only compile via
  the Arduino IDE/CLI against an ESP32 target. Native (host-machine) unit testing requires a
  separate build path.

## Strategy

Four layers, cheapest/highest-value first:

1. **Pure-logic unit tests** — functions with no hardware dependency, testable as-is on the host
   machine with zero firmware changes (the `previpass`/`previpass_util` library, plus a handful of
   string/math helpers in the `.ino` that only need extraction, not rewriting).
2. **Small extraction refactors** — pull currently-inline parsing/formatting logic (HTTP JSON
   parsing, EEPROM byte packing, GPS message masking, config-file line parsing) out of functions
   that also touch hardware, into standalone functions taking plain arguments and returning plain
   values. Behavior-preserving, mechanical, low risk.
3. **Mocked-peripheral tests** — for logic that inherently needs to call `SD`/`WiFi`/`KIM`, build
   thin interfaces (or reuse PlatformIO's `native` test environment with a fake `Arduino.h`) so
   the *decision logic* (retry counts, timeouts, sleep durations) can run without real hardware.
4. **Hardware-in-the-loop (HIL)** — a bench rig with a real ESP32 + a mock HTTP/FTP server (reuse
   the actual `popup-server` `app.py` pointed at test fixtures) validating the end-to-end
   sequences documented in `ARCHITECTURE.md`.

## Phase 0 — toolchain

Adopt **PlatformIO** alongside (not instead of) the existing `.ino` for testability:

- `pio test -e native` runs Phase-1/2 tests on the host machine using GoogleTest or Unity,
  compiling only the extracted pure-C/C++ files (`previpass.c`, `previpass_util.c`, and new
  `*_logic.cpp` files from Phase 2) — no ESP32/Arduino core required.
- `pio test -e esp32dev` (optional, later) runs Phase 3/4 tests that need a real or emulated
  ESP32 (e.g. via [Wokwi](https://wokwi.com/) or [renode](https://renode.io/) simulation) with
  mocked peripherals.
- Recommended framework: **Unity** (PlatformIO's default, C-friendly — matches `previpass.c`)
  for Phase 1, or **GoogleTest** if the extracted Phase-2 code ends up mostly C++.

## Phase 1 — pure-logic unit tests (no refactor needed)

`previpass.c` / `previpass_util.c` are vendor library code with a documented public API
(`previpass.h`) and no hardware coupling — test them directly.

| Function | Test cases |
|---|---|
| `PREVIPASS_UTIL_date_calendar_stu90` / `_date_stu90_calendar` | Round-trip conversion (calendar → sec90 → calendar) for known dates; epoch boundary (1990-01-01); leap year (2024-02-29); year rollover (Dec 31 → Jan 1) |
| `PREVIPASS_UTIL_sat_elevation_distance2` | Known elevation/axis pairs against hand-computed expected distances; 0° and 90° elevation edge cases |
| `PREVIPASS_compute_next_pass` | Feed a fixed `AopSatelliteEntry_t` + known beacon lat/long + fixed "now" → assert a pass is found with plausible epoch/duration/elevation; assert `false` returned when no satellite is above the configured `minElevation` in the 24h window |
| `PREVIPASS_compute_next_pass_with_status` | Same, with downlink/uplink status filters — assert satellites below the requested capacity are excluded |
| `PREVIPASS_status_format_a_to_generic` / `_format_b_to_generic` / `_generic_to_format_a` | Every enumerated input combination from the header's documented conversion table (small, exhaustive) |
| `PREVIPASS_compute_new_prediction_pass_times[_with_status]` | Memory-pool overflow flag set correctly when `maxPasses`/pool size is exceeded (this is a `malloc`-based linked list — also a good target for a Valgrind/ASan pass) |

**Also target with ASan/UBSan** (host build, no hardware needed): these two files do manual
`malloc`/pointer arithmetic (`__mallocBytesPool`) — a sanitizer run is cheap and catches an
entire class of bug this code is structurally prone to.

## Phase 2 — unit tests requiring a small extraction refactor

These currently live inside functions that also touch `SD`/`WiFi`/`writeLogFile`. Extract the
pure transformation into a standalone function (e.g. `parseTimeResponse` already *is* almost
pure — it only calls `writeLogFile` for side-channel logging, which should become a return value
or be injected).

| Current function (file:approx line) | Extract to | Test cases |
|---|---|---|
| `parseTimeResponse` (popup-buoy.ino:1373) | pure `bool parseTimeResponse(payload, &y,&m,&d,&h,&min,&s)` | Well-formed payload; missing `success`; `success:false`; missing each individual field one at a time; extra whitespace around values; malformed JSON (truncated) |
| `parsePermissionResponse` (popup-buoy.ino:1458) | pure, same pattern | `releaseFlag=0`/`1`; `releaseMode` = `"FRM"`/`"DM"`/unknown string (assert falls back to `DM`); missing `sleeptime_h`/`_m`; `success:false` with and without a `message` field |
| `parseSyncTimeResponse` (popup-buoy.ino:1540) | pure | Present `sync_time`; missing key; non-numeric value |
| `splitLineProgressFile` / `splitLineDataFile` / `splitLineSuccessFile` | already pure, just move out of `.ino` into a header the native test can include | Normal `"12:3"` line; missing separator; empty string; separator at position 0; separator at end; negative numbers (`toInt()` behavior) |
| `maskGPS` (popup-buoy.ino:2006) | pure given inputs, writes to `char*` buffers — already mockable, just needs `writeLogFile` stubbed or removed from the pure path | No-fix sentinel (`lat==200 && long==200` → `FFFFFFFF`/`FFFFFFFF`); negative latitude/longitude (two's-complement-style wrap); positive lat/long; epoch=0; epoch=`UINT32_MAX`; `WORK_ADC` defined vs. CRC8 branch (both compile paths) |
| `ChangeSecondsInHoursAndMinutes` (popup-buoy.ino:2270) | already pure | 0 seconds; exactly 1h/1m boundaries; large values (>24h, since callers don't cap this before conversion); negative input (current signature is `int*`, callers never pass negative today — decide whether to assert or defensively test) |
| EEPROM byte-packing: `eepromSaveTimeCoverage` read side `(EEPROM.read(2) << 8) \| EEPROM.read(3)` | extract `uint16_t packCoverageDuration(hi,lo)` / `unpackCoverageDuration(uint16_t)` | 0; 0xFFFF; 0x0100 (byte-order regression guard — this is exactly the kind of bug a unit test catches instantly and a field test does not) |
| `getWiFiFailureReason` | already pure | Every `wl_status_t` enum value including one not explicitly cased (falls to `"Unknown"`) |
| `updateMinElev` (popup-buoy.ino:2061) | already pure (once `readSuccessFile`'s SD read is injected as a parameter instead of called inline) | Boundary values at every `RowProgress` breakpoint (993/994, 1986/1987, ...); 0 and negative `RowProgress`; value above the last breakpoint — **note**: also flag in the test file that this function is currently dead code per `ARCHITECTURE.md`, so these tests document intended behavior for if/when it's wired back in, not verify current runtime behavior |

## Phase 3 — hardware-abstraction tests (mocked peripherals)

For logic that's inherently about *deciding what to do with hardware results* (not the hardware
call itself), introduce a thin seam and test the decision logic with a fake implementation.

| Area | Seam to introduce | Test cases |
|---|---|---|
| Wi-Fi retry/backoff in state 2 | Extract the `Counter_FailWIFI` branch logic (`if (Counter_FailWIFI>=2) {...} else {...}`) into a pure function `computeWifiRetrySleep(counter, sleeptime_h, sleeptime_m, sleepTimeWifiAttempt) -> {hours, minutes}` | counter=0,1 → short retry; counter>=2 → long retry using configured `sleeptime_h`; `sleeptime_h==0` fallback to 24 |
| GPS-fail backoff (states 4/5/6) | Extract `computeGpsFailSleep(counter, ...)` mirroring the `Counter_FailGPS` branches | counter 1,2 → short sleep; counter>=3 → recurrent sleep + counter reset; state-5 variant multiplies by `CRIT_FACTOR` |
| SPP overlap/error handling | Extract the `secondsBeforeNextStatellite <= 0` branch logic from states 4/5/6 into one shared pure function given `(secondsBeforeNextStatellite, Decimal_CoverageDuration)` returning the action (continue/overlap/error) + updated duration | Positive seconds (normal case); overlap case (`<=0` and remaining duration `>70`); SPP-error case (`<=0` and remaining duration `<=70`); exact boundary at 70 |
| Battery/state transition | Extract `nextStateForBattery(currentState, Vin_ADC, Bat_critlevel) -> nextState` from the repeated `if (Vin_ADC>Bat_critlevel) {...state 4...} else {...state 5...}` blocks in states 4/5/6 | Above/below/equal to threshold, from each of states 4, 5, 6 |
| Max-sleep cap | Extract the `if (secondsBeforeNextStatellite > max_sleep_time_s)` clamp | Below cap (unchanged); above cap (clamped + `CoverageState` forced to 0) |
| `KIM` driver | Fake `SerialUART` (in-memory ring buffer standing in for `HardwareSerial`) feeding canned AT-command responses | `check()` true/false on `+ID:...` vs. garbage/timeout; `send_data()` builds the exact `AT+TX=<data>\r` frame for boundary-length payloads; `set_PWR`/`set_AFMT` command framing |
| `ESP32_FTPClient` | Fake `WiFiClient` returning scripted FTP response lines | `GetFTPAnswer` timeout handling; `DownloadFileToSD` return codes (-1 not connected / -2 command error / -3 timeout) each independently triggerable |

This phase is the highest-effort, highest-value one: it's where most of the buoy's actual
"business logic" (retry policy, mode transitions, battery safety) lives, and it's currently
**only verifiable by field-testing a physical buoy**.

## Phase 4 — state machine tests

Once Phase 2/3 extractions exist, the `loop()` state `switch` itself becomes testable as a
state-transition table, independent of the side effects each state performs:

| From state | Trigger (mocked) | Expected next state | Covered by |
|---|---|---|---|
| 0 | setup completes | 1 | unit (pure transition, once extracted from `setup()`) |
| 1 | `PB_1` pressed | 2 | unit (button-press injection) |
| 2 | GPS satellites found before release | 4 | unit |
| 2 | release succeeds, mode `DM` | 4 | unit |
| 2 | release succeeds, mode `FRM` | 6 | unit |
| 2 | release succeeds, mode unknown/garbage | 4 (documented fallback) | unit — regression guard for the explicit fallback in `sendHttpGetRequest`'s permission handling |
| 2 | Wi-Fi/permission/FTP/release each fail independently | stays 2 | unit, one case per failure point |
| 4 | battery below `Bat_critlevel` | 5 | unit |
| 5 | battery recovers | 4 | unit |
| 6 | `maxFRM` hours elapsed | 4 | unit |
| 6 | battery critical mid-loop | 5 | unit |

Model this as a single table-driven test (`{fromState, mockedInputs, expectedNextState}` cases)
once the transition decision is extracted from the I/O-heavy `case` bodies — this is the test
that would have caught the current bench-test shortcuts (`PBState` 2/3 jumping straight to
4/6) being live in a production build, if it asserts which *inputs* are allowed to reach which
states.

## Phase 5 — hardware-in-the-loop (HIL) tests

Requires physical hardware (or a simulator) and is inherently slower/flakier — reserve for
release-candidate validation, not every commit.

1. **Bench rig**: real ESP32 + KIM1 + GPS module, `WORK_office`-style Wi-Fi pointed at a laptop
   running the actual `popup-server` `app.py` with a `config.yaml` fixture (short `date`s,
   short `release_time_secs`) and a stub FTP daemon serving a known `dataFile.txt`.
2. **Scenarios to script end-to-end**, asserting on `LogFile.txt` contents pulled off the SD card
   afterward:
   - Full happy path: state 0 → 1 (button) → 2 (permission denied then granted) → FTP download →
     release → state 4/6 → at least one simulated ARGOS transmission cycle.
   - Wi-Fi unreachable during state 2: verify the two-tier backoff (short retries, then
     `sleeptime_h`-hour retries) actually happens by timestamp deltas in the log.
   - Battery-critical injection (drop the ADC input voltage on the bench supply): verify
     transition 4→5 and the `CRIT_FACTOR` sleep stretch.
   - Power-cycle mid-state: verify EEPROM state resumption (kill power in state 4, confirm reboot
     resumes in state 4 with the same `CoverageState`/`Decimal_CoverageDuration`, not a reset to
     state 0).
   - No-GPS-fix run (indoors / GPS antenna disconnected): verify the fail-counter escalation
     (short → short → recurrent sleep) over three consecutive wake cycles.
3. **Kineis/ARGOS uplink itself** is out of scope for automated testing (requires live satellite
   visibility and Kineis backend access) — validate the KIM AT-command framing in Phase 3 instead,
   and treat an actual received-by-ground-segment message as a manual pre-deployment smoke test.

## Test data fixtures

Create a `test/fixtures/` directory (both here and shared with the server's tests) with:

- `conf.txt` — minimal valid config, one per test scenario (fast timings for CI-speed HIL runs).
- `AOP.txt` — a handful of hand-verified `AopSatelliteEntry_t` rows (real orbital bulletins
  pinned to a fixed date, so pass predictions are deterministic and can be hand-checked once).
- `dataFile.txt` / `progressFile.txt` pairs at various progress states (fresh, mid-file,
  end-of-file) to drive `SendFileKim`/`readSuccessFile` logic in Phase 2/3 tests.
- Canned HTTP payloads (valid/invalid/partial) for `/gettime`, `/permission/<id>`,
  `/getsynctime` — ideally generated *from* the server's own Phase-1 route tests
  (see server test plan) so both sides of the contract are verified against the same fixture.

## CI

- `pio test -e native` (Phase 1+2, and eventually 3) on every push/PR — fast (seconds), no
  hardware, safe to gate merges on.
- Add `-fsanitize=address,undefined` to the native test build flags given the manual memory
  management in `previpass.c`.
- Phase 4 table-driven state tests run in the same `native` job.
- Phase 5 HIL stays manual/scheduled (nightly or pre-release), not part of PR gating — no
  self-hosted runner with real hardware is assumed to exist yet.

## Prioritization

1. Phase 1 (previpass tests + ASan) — pure win, no firmware changes, catches memory bugs in
   vendor code that's central to every ARGOS transmission decision.
2. Phase 2 extractions for the four parser functions (`parseTimeResponse`,
   `parsePermissionResponse`, `parseSyncTimeResponse`, `maskGPS`) — these parse untrusted
   network input and hand-rolled `indexOf`/`substring` string scanning is exactly where off-by-one
   bugs hide.
3. Phase 3 retry/backoff/battery-transition extractions — this is the buoy's actual safety logic
   (when to give up, when to protect the battery) and is currently unverifiable without a field
   deployment.
4. Phase 4 state table — cheap once 2/3 exist, high regression-catching value (e.g. would catch
   the current debug `PBState` shortcuts if promoted to an assertion).
5. Phase 5 HIL scripts — do last, and only automate the scenarios that are expensive to verify by
   hand (multi-cycle backoff timing, EEPROM-resume-after-power-loss).
