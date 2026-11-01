# Arduino firmware

Arduino-ESP32 flavour of the [ESP32 Zigbee ToF Awning Sensor](../README.md). Hardware, BOM and wiring are described in the main README; this page only covers the firmware.

## Why this version exists

The [ESPHome firmware](../esphome/README.md) cannot expose parameters that are writable over Zigbee (Zigbee `number` entities are not supported on ESP32). This firmware fills that gap: calibration (closed zero, max length), night window, laser poll interval and a few switches are writable Zigbee attributes, stored in NVS, so they can be changed from Home Assistant / Zigbee2MQTT without reflashing.

## Status

**Work in progress.** [`esp32_zigbee_awning/esp32_zigbee_awning.ino`](esp32_zigbee_awning/esp32_zigbee_awning.ino) has been brought up on the real board: the sketch already contains hardware-specific tweaks (swapped red/green channels on the onboard LED, a Zigbee rejoin diagnostic mode) and it is not considered stable yet. The open points are listed at the end of this page.

## Features

- Zigbee end device, one endpoint (10) of type **Simple Sensor**. It is not exposed as a Window Covering: the awning position is the custom attribute `OpenPercent`. Manufacturer `Ivan`, model `AwningToF` (the Zigbee2MQTT converter matches on the model)
- **Power Configuration** (`0x0001`): battery percentage (reported to the coordinator) and voltage (100 mV resolution). The precise voltage in mV is in the custom cluster
- **Time** client (`0x000A`): asks the coordinator for the network time at boot and every 6 hours, and syncs the internal clock. The time zone is hard-coded to Italy (`TZ_STRING` in the sketch)
- **Custom cluster** `0xFC00`. Every attribute is also reported to the coordinator (address `0x0000`, endpoint 1), without depending on bindings. The writable attributes are persisted in NVS (`Preferences`) and sanity-checked on write (hours 0-23, interval at least 100 ms, max length greater than zero position):

  | Attribute | Access | Type | Meaning | Default |
  | :--- | :--- | :--- | :--- | :--- |
  | `0x0000` | read/write | U16 | Awning closed position zero (cm) | 20 |
  | `0x0001` | read/write | U16 | Awning max length (cm) | 220 |
  | `0x0002` | read/write | U8 | Deep sleep start hour | 20 |
  | `0x0003` | read/write | U8 | Deep sleep end hour | 7 |
  | `0x0004` | read/write | U16 | Laser read interval (ms) | 500 |
  | `0x0005` | read/write | U8 | LED mode: 0 always-on, 1 blink, 2 boot-only | 0 |
  | `0x0006` | read/write | U8 | Night deep sleep enabled (0 = no, 1 = yes) | 0 |
  | `0x0007` | read/write | U8 | Laser enabled (1 = on, 0 = off via XSHUT, for debugging) | 1 |
  | `0x0010` | read only | U16 | Distance (mm) | - |
  | `0x0011` | read only | U8 | Opening percentage (0 closed - 100 fully open) | - |
  | `0x0012` | read only | U16 | Battery voltage (mV) | - |
  | `0x0013` | read only | U8 | Status: 0 active, 1 deep sleep, 2 charging, 3 charge complete | - |

- **Daytime**: CPU awake, laser read every interval, report only when the distance changes by 2 cm or more. The percentage is computed from the zero and max length parameters
- **Night**: only when night deep sleep is enabled (off by default) and the time has been synced. Laser off through XSHUT (GPIO 5, held low during sleep), then a single deep sleep until the end hour. If start and end hour are equal there is no night window
- **Laser**: if the VL53L1X is not found, the initialization is retried every 15 s
- **Battery**: 15 ADC samples averaged, divider compensated, multiplied by `VBAT_CAL` (calibration factor, default 1.0: set it to multimeter voltage / reported voltage), converted with a piecewise-linear Li-Ion curve (3.2 V empty, 4.2 V full) and reported every 10 minutes (every minute while the charger is connected)
- **Charger and status**: the TP4056H does not expose its CHRG pin, so GPIO 10 reads the USB input through a divider (see the main README). The `Status` attribute is:
  - `active` (0): running on battery
  - `deep sleep` (1): sent right before entering the night deep sleep; it stays until the next wake-up, because a sleeping device cannot transmit
  - `charging` (2): charger connected
  - `charge complete` (3): an estimate, not the charger's own signal: charger connected and battery at or above `VBAT_CHARGED` (4.15 V) for `CHARGED_CONFIRM_MS` (15 minutes). In the final constant-voltage phase "almost full" and "full" look the same from the voltage alone

  While the charger is connected the battery percentage is not updated (the terminal voltage is inflated by the charge current), only the voltage is; at charge complete it is set to 100 %. After unplugging, the percentage resumes after `BATTERY_SETTLE_MS` (2 minutes). With the charger connected the board never enters the night deep sleep, and plugging the charger in during the night wakes it up (ext1 wake-up on GPIO 10)
- **Status LED**: blue while the Zigbee stack starts and joins, then green according to LED mode (always on, blinking, or only for 5 s after boot). The red and green channels are swapped on this board: `LED_RG_SWAPPED` in the sketch
- **Join**: if no network is found after 10 minutes the board restarts and tries again
- **Factory reset**: hold the BOOT button for 3 s, also while waiting for the join

## Pinout used by the firmware

| GPIO | Function |
| :--- | :--- |
| 1 | Battery voltage (ADC1_CH0, 100k/100k divider with 100 nF capacitor) |
| 2 | VL53L1X SDA (`Wire.begin(2, 3)`) |
| 3 | VL53L1X SCL |
| 5 | VL53L1X XSHUT (laser off during deep sleep) |
| 8 | Onboard WS2812B status LED |
| 9 | BOOT button (factory reset when held 3 s) |
| 10 | Charger detection (TP4056H `IN+` through a 100k/150k divider, high = USB connected, deep sleep wake-up) |

Wiring details are in the [main README](../README.md#-wiring-list).

## Layout

```
arduino/
├── README.md
├── .gitignore
└── esp32_zigbee_awning/        # sketch folder (name must match the .ino file)
    ├── esp32_zigbee_awning.ino
    └── sketch.yaml             # default FQBN and board options
```

The Zigbee2MQTT converter for this firmware lives in [`../z2m/`](../z2m/awning_tof.mjs).

## Toolchain

- [`arduino-cli`](https://arduino.github.io/arduino-cli/) or the Arduino IDE
- Arduino-ESP32 core (`esp32:esp32`) with ESP32-H2 Zigbee support (3.3.x or newer, as stated in the sketch header)

Setup (once):

```bash
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

The exact core version of the last working build has not been pinned yet; record it here (`arduino-cli core list`) once it is settled.

## Board options

[`sketch.yaml`](esp32_zigbee_awning/sketch.yaml) sets the default FQBN:

```
esp32:esp32:esp32h2:ZigbeeMode=ed,PartitionScheme=zigbee
```

- `ZigbeeMode=ed`: Zigbee end device (battery powered). The sketch refuses to compile with another mode
- `PartitionScheme=zigbee`: Zigbee 4MB with spiffs (the ESP32-H2FH4S has 4 MB of flash)

In the Arduino IDE set the same options under *Tools*. If you do not see `Serial` output over the USB-C port, enable *USB CDC On Boot*.

## Libraries

- SparkFun VL53L1X 4m Laser Distance Sensor (`SparkFun_VL53L1X.h`):

```bash
arduino-cli lib install "SparkFun VL53L1X 4m Laser Distance Sensor"
```

## Build and flash

From the repository root:

```bash
# Compile
arduino-cli compile arduino/esp32_zigbee_awning

# Upload (replace the port, e.g. /dev/ttyACM0 or COM5)
arduino-cli upload -p /dev/ttyACM0 arduino/esp32_zigbee_awning

# Serial monitor
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

If the board is not detected: unplug the USB cable, press and hold the onboard **BOOT** button, plug the cable back in, release the button and retry.

## Zigbee2MQTT

The custom cluster is not understood by Zigbee2MQTT out of the box. The external converter [`z2m/awning_tof.mjs`](../z2m/awning_tof.mjs) maps it to entities:

1. Copy the file to `<Zigbee2MQTT data folder>/external_converters/awning_tof.mjs` (next to `configuration.yaml`).
2. With Zigbee2MQTT 2.11 or newer, new installations have external converters disabled: set `enable_external_js: true` in the `advanced` section.
3. Restart Zigbee2MQTT, remove the old device if it was already paired and pair it again, because clusters and attributes changed between firmware versions.

Entities exposed: opening percentage, distance (cm), battery percentage and voltage, status (active, deep sleep, charging, charge complete), link quality, and as configuration the parameters listed in the attribute table (zero, max length, laser on/off, deep sleep on/off, start and end hour, read interval, LED mode).

Attribute IDs and types must stay identical in the sketch and in the converter: change both together.

## Open points

- **Status LED**: the onboard WS2812B draws about 1 mA even when off, so the deep sleep target (under 10 µA) is only reachable if the LED is removed. The firmware works either way; without the LED you only lose the pairing feedback. With the default LED mode (always-on) the LED also stays lit after pairing: use `boot-only` to save battery.
- **Radio idle mode**: the sketch currently sets `DIAG_RX_ON_IDLE` to 1, a diagnostic setting for the rejoin problem: the Zigbee radio stays always listening and the stack never sleeps, which costs far more battery than a normal low-power end device. Setting it to 0 restores the low-power behaviour (`esp_zb_sleep_enable`) once the rejoin issue is solved.
- **Deep sleep**: night deep sleep is disabled by default and needs a synced clock. While the chip sleeps the sensor does not answer and the USB port disappears until wake-up.
- **Charger detection**: not yet verified on the real board. In particular the ext1 wake-up on GPIO 10 relies on the pin keeping the internal pull resistors disabled during deep sleep; if plugging the charger in does not wake the board, it still detects the charger at the next timed wake-up. `VBAT_CAL` also needs to be measured, otherwise with a divider/ADC reading below the real voltage the 4.15 V threshold may never be reached and `charge complete` never shows up
- **Home Assistant with ZHA**: only a Zigbee2MQTT converter exists; the custom cluster and the Time cluster would need a ZHA quirk to show up as entities.
- **Time zone**: hard-coded to Italy, edit `TZ_STRING` in the sketch for other regions.
