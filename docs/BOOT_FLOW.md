# Boot flow, restarts and button (Arduino firmware)

Specification of the boot flow of the [Arduino firmware](../arduino/README.md) and of how it handles restarts, waits
and the button. It applies to every kind of start: power-on, wake-up from deep sleep, reset.

## Goals

- Every boot is the same and gradual, without concentrated current peaks.
- One exit point for restarts and waits: no `ESP.restart()` scattered around the code.
- The reason of every reset is recorded and reported to Zigbee2MQTT.
- No unchecked restart loops.

## Why (observed problem)

With the battery around 3.3-3.45 V (ADC reading) the device went into restart loops at wake-up (LED blue, then
green, then blue again), and once it stayed stuck until the power was removed. With D1 (1N5819) in series, the
radio peaks pull the ESP's 3V3 below the ESP32-H2 minimum and the brown-out detector fires. Every boot is also the
phase with the highest current (radio always listening, LED, laser), so the restart itself causes the next voltage
drop. The previous firmware did not check the battery before starting the radio, did not count resets and did not
record why it restarted.

## Verified context (arduino-esp32 3.3.12)

- Every wake-up from deep sleep is a full boot: `setup()`, `Zigbee.begin()`, rejoin. The parent forgets the child
  after 64 minutes (the library's default aging timeout), so every morning there is a full rejoin.
- For a device that is already paired, `Zigbee.connected()` becomes true as soon as the stack reloads its data from
  flash, **before** any exchange with the parent: it does not prove that the network answers.
- The library restarts the board on its own in three cases: "leave with rejoin" calls `ESP.restart()`, "leave"
  performs a factory reset, an `esp_zb_start()` error causes a panic. If commissioning fails it retries every
  0.5-1 s with no limit.
- If `begin()` times out the library resets the channel mask: the next boot scans all channels.
- **Light sleep is most likely not active**: the precompiled Arduino libraries have neither `CONFIG_PM_ENABLE` nor
  tickless idle, and the core's signal handler does not handle `CAN_SLEEP`. `esp_zb_sleep_enable(true)` almost
  certainly has no effect: the saving comes only from `rx_on_when_idle = false` (radio off between polls).
- The task watchdog does not cover `setup()`, and on `loop()` it must be enabled explicitly.

## Boot flow

The same for every kind of start; only the LED differs (no LED on timer wake-ups).

1. **Start with the radio off**: `esp_reset_reason()` first, then the reset log in RTC memory is updated; laser off
   (XSHUT low); battery read at rest.
2. **Decision**: battery below `VBAT_MIN_BOOT` (charger not connected), or at least `DIRTY_STREAK_MAX` consecutive
   dirty resets: deep sleep for `RETRY_SLEEP_S` (30 min) with the matching cause.
3. **`Zigbee.begin()`**, a single call, with a `ZB_BEGIN_TIMEOUT_MS` timeout (10 s, as in Espressif's sleepy
   example). If the stack does not initialize: deep sleep for 30 min. The radio is on at boot.
4. **Join and network confirmation**: wait for `connected()` (2 min, 10 min at the first pairing), then for the
   coordinator's **Time response** (30 s). If either is missing: deep sleep for 30 min.
5. **Sequential steps, once the network is confirmed**: sleepy right away (at the first pairing after the
   Zigbee2MQTT interview, 3 min) -> reset log published -> OTA query -> laser -> configuration. The first battery
   reading comes 60 s after the join.
6. **Normal operation** (`loop()`). The first battery reading clears the consecutive dirty reset counter.

## Single exit point

- `sleepAndRetry(cause, seconds)`: saves the cause in RTC memory, turns off the laser (XSHUT low with hold) and the
  LED, enables timer and ext1 wake-up (charger on GPIO10 and button, both active high), enters deep sleep. The night
  deep sleep goes through here too.
- `plannedRestart(cause)`: the only `ESP.restart()` in the firmware, used by a button click.
- The only other exception: the factory reset (button held for 10 s).

| Cause (`PlannedCause`) | Value | When |
| :--- | :--- | :--- |
| `CAUSE_NONE` | 0 | No cause recorded: power-on, or a restart requested by the Zigbee library |
| `CAUSE_NIGHT` | 1 | Night deep sleep |
| `CAUSE_LOW_BATT` | 2 | Battery below `VBAT_MIN_BOOT` at boot |
| `CAUSE_JOIN_TIMEOUT` | 3 | Network not reached (join or Time response) |
| `CAUSE_ZB_INIT_FAIL` | 4 | Zigbee stack not initialized |
| `CAUSE_RESET_LOOP` | 5 | Too many consecutive dirty resets |
| `CAUSE_BUTTON_RESTART` | 6 | Button click |
| `CAUSE_FACTORY_RESET` | 7 | Button held for 10 s |

## Reset log

- In RTC memory (`RTC_NOINIT_ATTR`, valid when the magic number matches): last planned cause, consecutive dirty
  resets, total dirty resets, reason of the last dirty reset. It survives deep sleep, `restart`, panic, watchdog and
  normally brown-out; it is lost when the power is removed.
- **Dirty** resets: `BROWNOUT`, `PANIC`, `INT_WDT`, `TASK_WDT`, `WDT`.
- The totals are copied to NVS only once the battery has been checked (never on every boot: a write during a
  brown-out can corrupt the flash) and are reloaded from NVS at power-on.
- Published to Zigbee2MQTT once per boot, once the network is confirmed, in cluster 0xFC00:

| Attribute | ID | Type | Content |
| :--- | :--- | :--- | :--- |
| `ResetReason` | 0x0014 | U8 | `esp_reset_reason_t` of this boot |
| `PlannedCause` | 0x0015 | U8 | Planned cause before this boot (table above) |
| `DirtyResets` | 0x0016 | U16 | Total dirty resets |
| `LastDirtyReason` | 0x0017 | U8 | `esp_reset_reason_t` of the last dirty reset |

A failed boot cannot transmit: its reason arrives with the first successful boot, like a black box.

## External button

- THT push button between **GPIO12** (LP IO, it can also wake the board from deep sleep) and **3V3**, with a
  **100 kΩ resistor to GND**, optionally 100 nF in parallel against bounce. Active high. The internal pull-down stays
  enabled as a safety net; if the button is not fitted, set `BUTTON_INSTALLED = false`.
- **Click** (released within 1 s): clean restart.
- **Held for 10 s**: the LED turns white; on release, Zigbee factory reset and new pairing.
- Releases between 1 and 10 s: no action. Never during an OTA. It also works while waiting for the join.
- The board's **BOOT** button (GPIO9) is no longer read by the firmware: it is only for download mode. If it is
  pressed while the chip resets, the chip enters download mode and stays there until the power is removed.

## Watchdog

Task watchdog of `WDT_TIMEOUT_MS` (30 s) on the `setup()`/`loop()` task, enabled first thing. The core feeds it on
every `loop()` iteration; the waits in `setup()` feed it by hand. A hang (even inside `Zigbee.begin()`) becomes a
`TASK_WDT` reset, counted as dirty: after `DIRTY_STREAK_MAX` consecutive ones the 30-minute pause kicks in.

## Constants

| Constant | Value | Meaning |
| :--- | :--- | :--- |
| `VBAT_MIN_BOOT` | 3.40 V | ADC reading below which the radio is not started. **To be calibrated** on the real circuit |
| `RETRY_SLEEP_S` | 1800 | Deep sleep before retrying |
| `DIRTY_STREAK_MAX` | 3 | Consecutive dirty resets before the pause |
| `ZB_BEGIN_TIMEOUT_MS` | 10000 | `Zigbee.begin()` timeout |
| `JOIN_WAIT_MS` | 120000 | Join wait on a normal boot |
| `JOIN_WAIT_PAIRING_MS` | 600000 | Join wait at the first pairing |
| `NET_CONFIRM_MS` | 30000 | Wait for the Time response after the join |
| `WDT_TIMEOUT_MS` | 30000 | Task watchdog timeout |
| `BUTTON_LONG_MS` | 10000 | Long press for the factory reset |

## Open checks

1. **Current between two polls**, measured with a multimeter, to confirm that light sleep is not active.
2. **Radio switched off at run time**: in sleepy mode, a parameter written from Home Assistant must arrive (the parent
   must queue the messages). If they get lost, consider booting directly as sleepy on normal boots.
3. **`VBAT_MIN_BOOT`** to be calibrated after checking the drop across D1 (0.2-0.3 V expected).
4. **RTC after a brown-out** on the ESP32-H2: if the counters turn out to be cleared, count that case in NVS.
5. **Pinout**: confirm that GPIO12 is exposed on the Waveshare ESP32-H2-Zero.
6. **Core version**: pin it in the `README` and in `sketch.yaml`.
