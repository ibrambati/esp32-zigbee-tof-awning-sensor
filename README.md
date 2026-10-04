# ESP32 Zigbee ToF Awning Sensor

A smart, low-power, battery-operated distance sensor designed to monitor and control motorized garden awnings in real-time.

By utilizing the **VL53L1X Time-of-Flight (ToF) laser sensor** combined with an **ESP32-H2 / ESP32-C6** microcontroller speaking **Zigbee (Zigbee2MQTT/ZHA)**, this project exposes the exact opening percentage directly to **Home Assistant**, even if your awning uses local wall switches or standard dry-contact relays.

### 🔋 Key Features
* **100% Wireless & Smart Power Management:** Optimized for battery operation with a deep sleep duty cycle to keep battery drain minimal.
* **ToF Precision:** Measures the exact distance between the awning body and the front bar, converting raw millimeters into an accurate 0-100% position entity.
* **Configurable Calibration:** Calibration points (zero position and max length), sleep duration and laser poll interval are configurable. With the ESPHome firmware they are compile-time defaults; the Arduino firmware aims to make them writable over Zigbee, without reflashing.

## 🧩 Two Firmware Options

The hardware is the same; pick the firmware that fits your needs.

| Folder | Framework | Status | Remote configuration over Zigbee |
| :--- | :--- | :--- | :--- |
| [`esphome/`](esphome/README.md) | ESPHome (ESP-IDF) | Available | No: calibration and sleep values are set at compile time |
| [`arduino/`](arduino/README.md) | Arduino-ESP32 (`arduino-cli`) | Work in progress (first draft, untested) | Yes: writable Zigbee attributes stored in NVS |

Repository layout:

```
.
├── README.md     # this file: hardware, BOM, wiring
├── docs/         # photos, schematics, datasheets
├── esphome/      # ESPHome firmware + README
└── arduino/      # Arduino firmware, build setup + README
```

## 📦 Bill of Materials (BOM)

| Component | Description | Qty | Notes |
| :--- | :--- | :--- | :--- |
| **Home Assistant** | Central smart home server | 1 | The main automation hub |
| **Zigbee2MQTT / ZHA** | Zigbee bridge / coordinator | 1 | Handles Zigbee communication with the sensor |
| **ESPHome** *or* **Arduino-ESP32** | Firmware framework | 1 | See [Two Firmware Options](#-two-firmware-options) |
| **Waveshare ESP32-H2-Zero** | Ultra-compact Zigbee microcontroller | 1 | Based on the ESP32-H2FH4S chip with ceramic antenna |
| **VL53L1X** | Time-of-Flight (ToF) laser distance sensor | 1 | Range up to 4 meters (Long Mode) |
| **TP4056H Charging Module** | USB-C Li-Ion battery charger with protection | 1 | Must include double protection (6 pads, e.g., HW-107) |
| **18650 Li-Ion Battery** | 3.7V ricaricabile cell (3200 mAh) | 1 | *Unprotected* (flat top) model recommended for outdoors |
| **100kΩ Resistors** | Carbon or metal film resistors | 2 | 1% tolerance for the ADC voltage divider |

### Hardware photos

| ESP32-H2-Zero | VL53L1X |
| :---: | :---: |
| ![Waveshare ESP32-H2-Zero](docs/Waveshare%20ESP32-H2-Zero.jpg) | ![VL53L1X](docs/VL53L1X.jpg) |

---

## 📌 Pinout & Hardware Layout (ESP32-H2-Zero)

According to the official specs of the [Waveshare ESP32-H2-Zero](https://waveshare.com), this board exposes essential pins in an ultra-small form factor. Below is the mapping used for this project:

| Board Pin | Native Function | Project Connection | Description |
| :--- | :--- | :--- | :--- |
| **5V** | Power Input (VCC) | Connected to **`OUT+`** of the TP4056H | Receives regulated battery power (3.7V - 4.2V) |
| **GND** | Ground | Connected to **`OUT-`** / Common Ground | Main reference ground for the entire system |
| **GPIO 2** | GPIO | Connected to **`SDA`** of the VL53L1X | I2C Data line for the laser sensor (`Wire.begin(2, 3)`) |
| **GPIO 3** | GPIO | Connected to **`SCL`** of the VL53L1X | I2C Clock line for the laser sensor |
| **GPIO 5** | MTMS / GPIO | Connected to **`XSHUT`** of the VL53L1X | Shutdown control pin to turn off the laser in Deep Sleep |
| **GPIO 1** | ADC1_CH0 / GPIO | Connected to the center of the divider | Analog pin used to measure battery voltage |
| **GPIO 8** | Onboard WS2812B | Status LED, no external wiring | Arduino firmware: blinking blue while joining Zigbee, steady green for 5 s once paired, then off |

> 💡 **Power Saving Tip:** To fully eliminate parasitic power drain during Deep Sleep, it is highly recommended to desolder or cut the trace of the onboard **WS2812B** RGB LED. Otherwise, it will continuously draw around 1mA even when the chip is asleep. The Arduino firmware uses this LED for the pairing feedback: if you remove it, you only lose the status light.

---

## 🔌 Wiring List

All grounds must merge into a single logical point (**Common GND**). Follow this step-by-step wiring guide:

1. **Power & Charging Circuit:**
   * Battery `(+)` terminal ──► **`B+`** pad on the TP4056H module
   * Battery `(-)` terminal ──► **`B-`** pad on the TP4056H module
   * **`OUT+`** pad on the TP4056H ──► **`5V`** pin on the ESP32-H2-Zero **AND** **`VCC`** pin on the VL53L1X sensor
   * **`OUT-`** pad on the TP4056H ──► **`GND`** pin on the ESP32-H2-Zero **AND** **`GND`** pin on the VL53L1X sensor

2. **Laser Sensor Data Bus:**
   * Laser **`SDA`** ──► **`GPIO 2`** pin on the ESP32-H2-Zero
   * Laser **`SCL`** ──► **`GPIO 3`** pin on the ESP32-H2-Zero
   * Laser **`XSHUT`** ──► **`GPIO 5`** pin on the ESP32-H2-Zero

3. **Battery Monitor Voltage Divider:**
   * **`OUT+`** pad on the TP4056H ──► Input of the first 100kΩ resistor (**R1**)
   * Output of R1 ──► Input of the second 100kΩ resistor (**R2**) **AND** **`GPIO 1`** pin on the ESP32-H2-Zero
   * Output of R2 ──► **Common GND** (`OUT-`)

---

## ⚡ Firmware

Once the hardware is assembled, flash one of the two firmwares:

* **[ESPHome firmware](esphome/README.md)**: YAML configuration, flashing via the ESPHome dashboard / web tool, pairing with Zigbee2MQTT or ZHA. Settings are fixed at compile time.
* **[Arduino firmware](arduino/README.md)**: `arduino-cli` project with build and flash instructions. First draft, aimed at parameters writable over Zigbee.

In both cases the ESP32-H2-Zero is flashed over its native **USB Type-C port** the first time, since it speaks Zigbee and not Wi-Fi.
