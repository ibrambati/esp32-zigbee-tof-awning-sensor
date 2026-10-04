/*
 * ESP32 Zigbee ToF Awning Sensor - Arduino firmware (skeleton)
 *
 * Hardware: Waveshare ESP32-H2-Zero + VL53L1X ToF laser sensor.
 * Pin mapping and default calibration values mirror the ESPHome version
 * (see ../../esphome/esphome-zigbee-tof-awning-sensor.yaml).
 *
 * STATUS: skeleton only. It compiles and prints the configuration, but the
 * Zigbee end device, the VL53L1X driver and the deep sleep cycle are still
 * to be implemented. The goal of this firmware is to expose the calibration
 * and sleep parameters as writable Zigbee attributes, which ESPHome cannot do.
 */

#include <Wire.h>

// ---- Pin mapping (see main README, "Pinout & Hardware Layout") ----
constexpr int PIN_I2C_SDA  = 6;  // VL53L1X SDA
constexpr int PIN_I2C_SCL  = 7;  // VL53L1X SCL
constexpr int PIN_XSHUT    = 5;  // VL53L1X XSHUT (laser off during deep sleep)
constexpr int PIN_BATT_ADC = 1;  // Battery voltage divider (100k/100k)

// ---- Default parameters (same defaults as the ESPHome version) ----
constexpr float    DEFAULT_CLOSED_ZERO_MM   = 200.0f;      // 20 cm
constexpr float    DEFAULT_MAX_LENGTH_MM    = 2200.0f;     // 220 cm
constexpr uint32_t DEFAULT_SLEEP_MS         = 30UL * 60UL * 1000UL;  // 30 min
constexpr uint32_t DEFAULT_POLL_INTERVAL_MS = 500;
constexpr uint32_t AWAKE_WINDOW_MS          = 15UL * 1000UL;         // 15 s

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);  // laser off until the driver is implemented

  Serial.println("ESP32 Zigbee ToF Awning Sensor - skeleton");
  Serial.printf("closed zero: %.0f mm, max length: %.0f mm, sleep: %lu ms\n",
                DEFAULT_CLOSED_ZERO_MM, DEFAULT_MAX_LENGTH_MM,
                (unsigned long)DEFAULT_SLEEP_MS);

  // TODO: Zigbee end device + endpoints (opening %, battery V, battery %)
  // TODO: writable Zigbee attributes for calibration / sleep / poll interval
  // TODO: VL53L1X driver (LONG distance mode, 50 ms timing budget)
  // TODO: deep sleep cycle (awake window, then sleep for the configured time)
}

void loop() {
  delay(1000);
}
