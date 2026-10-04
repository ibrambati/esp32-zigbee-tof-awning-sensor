# ESPHome firmware

ESPHome flavour of the [ESP32 Zigbee ToF Awning Sensor](../README.md). Hardware, BOM and wiring are described in the main README; this page only covers the firmware.

## What it does

The configuration in [`esphome-zigbee-tof-awning-sensor.yaml`](esphome-zigbee-tof-awning-sensor.yaml) runs the ESP32-H2 as a battery-powered Zigbee **end device** (ESP-IDF framework). Each cycle it wakes up, reads the VL53L1X, reports over Zigbee and goes back to deep sleep (15 s awake, 30 min asleep by default).

Exposed over Zigbee (each sensor on its own endpoint):

| Endpoint | Entity | Notes |
| :--- | :--- | :--- |
| 1 | Awning Opening Percentage | 0 % closed - 100 % fully open, computed from the ToF distance |
| 2 | Battery Voltage | ADC reading, scaled x2 for the 100k/100k divider |
| 3 | Battery Percentage | Linear 3.2 V - 4.2 V Li-Ion scale |

## Known limitation: no remote configuration

The calibration values (closed zero, max length), the deep sleep duration and the laser poll interval are defined as `globals` and `number` template entities, but **Zigbee `number` entities are not supported by ESPHome on ESP32**, and this firmware has no Wi-Fi/API path either. In practice they only act as defaults: to change them, edit `initial_value` in the YAML and reflash.

If you need these parameters to be settable from Home Assistant / Zigbee2MQTT without reflashing, use the [Arduino firmware](../arduino/README.md) instead (work in progress).

## How to flash

The ESP32-H2-Zero uses a native **USB Type-C port** (USB CDC) managed directly by the main chip, without a dedicated UART chip. Since the board speaks Zigbee and not Wi-Fi, the first flash must be done over USB.

### 1. Environment preparation
1. Open your **Home Assistant** instance and go to the **ESPHome** dashboard.
2. Click **New Device** and name it (e.g. `esphome-zigbee-tof-awning-sensor`).
3. Pick **ESP32** as the platform. When prompted, select the `esp-idf` framework (required for native Zigbee support on H2/C6 chips).
4. Replace the generated YAML with the content of [`esphome-zigbee-tof-awning-sensor.yaml`](esphome-zigbee-tof-awning-sensor.yaml) and save it.

Alternatively, with the ESPHome CLI installed, compile directly from this folder:

```bash
esphome compile esphome-zigbee-tof-awning-sensor.yaml
```

### 2. Initial wired flashing (web method)
1. Connect the ESP32-H2-Zero to your PC using a proper USB-C **data** cable.
2. Inside ESPHome, click the three dots on your device and select **Install** -> **Manual Download**.
3. Wait for compilation to finish, then download the **Factory (`.bin`)** file.
4. Open the official [ESPHome Web tool](https://web.esphome.io) with a Chromium-based browser (Chrome or Edge).
5. Click **Connect** and pick the COM port linked to the board (it shows up as *USB JTAG/serial debug unit*).
   * If the board isn't detected: unplug the USB cable, press and hold the tiny onboard **BOOT** button, plug the cable back in, release the button and try again.
6. Select the downloaded `.bin` file and click **Install**.

### 3. Pairing with Zigbee2MQTT / ZHA
After flashing, the board reboots and starts in Zigbee pairing mode:
1. Open your **Zigbee2MQTT** dashboard (or ZHA) in Home Assistant.
2. Click **Permit Join (All)**.
3. Within a few seconds the sensor is discovered as a new Zigbee node and exposes the three sensors listed above.
