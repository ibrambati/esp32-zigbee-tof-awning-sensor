# Arduino firmware

Arduino-ESP32 flavour of the [ESP32 Zigbee ToF Awning Sensor](../README.md). Hardware, BOM and wiring are described in the main README; this page only covers the firmware.

## Why this version exists

The [ESPHome firmware](../esphome/README.md) cannot expose parameters that are writable over Zigbee (Zigbee `number` entities are not supported on ESP32). This firmware fills that gap: calibration (closed zero, max length), sleep window and laser poll interval are writable Zigbee attributes, stored in NVS, so they can be changed from Home Assistant / Zigbee2MQTT without reflashing.

## Status

**First draft, not yet compiled or tested on hardware.** [`esp32_zigbee_awning/esp32_zigbee_awning.ino`](esp32_zigbee_awning/esp32_zigbee_awning.ino) implements the full feature set below. Some calls of the Arduino-ESP32 Zigbee API depend on the core version; they are marked `[V1]`, `[V2]`, `[V3]` in the source and are the first things to check when compiling.

## Features

- Zigbee end device, one endpoint (10) exposed as **Window Covering** (cluster `0x0102`, `CurrentPositionLiftPercentage` `0x0008`)
- **Power Configuration** (`0x0001`): battery voltage and percentage
- **Time** client (`0x000A`): asks the coordinator for the network time and syncs the internal clock
- **Custom cluster** `0xFC00` with writable attributes, persisted in NVS (`Preferences`):

  | Attribute | Meaning | Default |
  | :--- | :--- | :--- |
  | `0x0000` | Awning closed position zero (cm) | 20 |
  | `0x0001` | Awning max length (cm) | 220 |
  | `0x0002` | Deep sleep start hour | 20 |
  | `0x0003` | Deep sleep end hour | 7 |
  | `0x0004` | Laser read interval (ms), extra attribute | 500 |

- **Daytime**: CPU awake, laser read every interval, report only when the distance changes by 2 cm or more
- **Night**: laser off through XSHUT (GPIO 5), then a single deep sleep until the end hour
- Status LED: blinking blue until the Zigbee join, steady green for 5 s, then off
- Factory reset: hold the BOOT button for 3 s

## Layout

```
arduino/
├── README.md
├── .gitignore
└── esp32_zigbee_awning/        # sketch folder (name must match the .ino file)
    ├── esp32_zigbee_awning.ino
    └── sketch.yaml             # default FQBN and board options
```

## Toolchain

- [`arduino-cli`](https://arduino.github.io/arduino-cli/) or the Arduino IDE
- Arduino-ESP32 core (`esp32:esp32`) with ESP32-H2 Zigbee support (3.1.x or newer)

Setup (once):

```bash
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

The core version used for the first successful build has not been pinned yet; it will be recorded here once the firmware compiles.

## Board options

[`sketch.yaml`](esp32_zigbee_awning/sketch.yaml) sets the default FQBN:

```
esp32:esp32:esp32h2:ZigbeeMode=ed,PartitionScheme=zigbee
```

- `ZigbeeMode=ed`: Zigbee end device (battery powered)
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

## Open points

- **I2C pins**: the sketch uses SDA GPIO 2 and SCL GPIO 3 (`PIN_SDA`, `PIN_SCL`), the main README and the ESPHome config use GPIO 6 and 7. Confirm the real wiring and align the documentation.
- **Status LED**: the main README suggests removing the onboard WS2812B to save about 1 mA in sleep; this firmware uses it during the Zigbee join.
- **Home Assistant**: the custom cluster and the Time cluster need a ZHA quirk or a Zigbee2MQTT external converter to show up as entities.
