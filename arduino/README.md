# Arduino firmware

Arduino-ESP32 flavour of the [ESP32 Zigbee ToF Awning Sensor](../README.md). Hardware, BOM and wiring are described in the main README; this page only covers the firmware.

## Why this version exists

The [ESPHome firmware](../esphome/README.md) cannot expose parameters that are writable over Zigbee (Zigbee `number` entities are not supported on ESP32). This firmware is meant to fill that gap: calibration (closed zero, max length), deep sleep duration and laser poll interval should become writable Zigbee attributes, so they can be changed from Home Assistant / Zigbee2MQTT without reflashing.

## Status

**Work in progress.** [`awning_sensor/awning_sensor.ino`](awning_sensor/awning_sensor.ino) is a compilable skeleton: pin mapping and default values are in place, while the Zigbee end device, the VL53L1X driver and the deep sleep cycle are still TODO.

## Layout

```
arduino/
├── README.md
├── .gitignore
└── awning_sensor/        # sketch folder (name must match the .ino file)
    ├── awning_sensor.ino
    └── sketch.yaml       # default FQBN and board options
```

## Toolchain

- [`arduino-cli`](https://arduino.github.io/arduino-cli/)
- Arduino-ESP32 core (`esp32:esp32`) with ESP32-H2 Zigbee support

Setup (once):

```bash
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

The core version used for the first successful build has not been pinned yet; it will be recorded here once the firmware compiles with the Zigbee stack.

## Board options

[`sketch.yaml`](awning_sensor/sketch.yaml) sets the default FQBN:

```
esp32:esp32:esp32h2:ZigbeeMode=ed,PartitionScheme=zigbee
```

- `ZigbeeMode=ed`: Zigbee end device (battery powered)
- `PartitionScheme=zigbee`: Zigbee 4MB with spiffs (the ESP32-H2FH4S has 4 MB of flash)

If you do not see `Serial` output over the USB-C port, enable the *USB CDC On Boot* board option for the H2.

## Build and flash

From the repository root:

```bash
# Compile
arduino-cli compile arduino/awning_sensor

# Upload (replace the port, e.g. /dev/ttyACM0 or COM5)
arduino-cli upload -p /dev/ttyACM0 arduino/awning_sensor

# Serial monitor
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

If the board is not detected: unplug the USB cable, press and hold the onboard **BOOT** button, plug the cable back in, release the button and retry.

## Libraries

The VL53L1X driver library has not been chosen yet. Once selected, it will be listed here with its install command (`arduino-cli lib install ...`).
