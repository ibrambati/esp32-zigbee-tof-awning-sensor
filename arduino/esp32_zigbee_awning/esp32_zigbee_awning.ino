/*
 * esp32-zigbee-tof-awning-sensor  -  Firmware nativo Zigbee (End Device a batteria)
 * Board: Waveshare ESP32-H2-Zero
 *
 * ---- IMPOSTAZIONI ARDUINO IDE (Strumenti) ----
 *  Board:             ESP32H2 Dev Module
 *  Zigbee Mode:       Zigbee ED (end device)
 *  Partition Scheme:  Zigbee 4MB with spiffs   (serve la partizione zb_storage)
 *  USB CDC On Boot:   Enabled (per il Serial)
 *  Core esp32:        >= 3.1.x (Zigbee.h / ZigbeeCover.h)
 *  Librerie:          SparkFun VL53L1X 4m Laser Distance Sensor
 *  (con arduino-cli le stesse opzioni arrivano da sketch.yaml)
 *
 * ---- PUNTI DA VERIFICARE AL PRIMO COMPILE (API Zigbee variano tra versioni del core) ----
 *  [V1] accesso a _cluster_list dentro la sottoclasse di ZigbeeCover
 *  [V2] firma di zbAttributeRead() / zbAttributeSet()
 *  [V3] nomi esatti: setLiftPercentage(), setPowerSource(), setBatteryPercentage/Voltage()
 *  Sono marcati nel codice con "// [Vn]".
 *
 * Mappa cluster esposti (endpoint 10):
 *  0x0102 Window Covering  -> CurrentPositionLiftPercentage (0x0008)
 *  0x0001 Power Config     -> BatteryVoltage (0x0020, unita' 100 mV) + BatteryPercentageRemaining (0x0021)
 *  0x000A Time (client)    -> letto dal coordinatore all'avvio e ogni TIME_RESYNC_S
 *  0xFC00 Custom (server)  -> 0x0000 ClosedPositionZero (cm), 0x0001 MaxLength (cm),
 *                             0x0002 SleepStartHour, 0x0003 SleepEndHour, 0x0004 ReadIntervalMs
 */

#ifndef ZIGBEE_MODE_ED
#error "Seleziona Strumenti > Zigbee Mode > Zigbee ED (end device)"
#endif

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>
#include "Zigbee.h"
#include "ZigbeeCover.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "SparkFun_VL53L1X.h"

// ===================== PIN =====================
static const uint8_t PIN_VBAT   = 1;   // ADC1_CH0, partitore 100k/100k + 100nF
static const uint8_t PIN_SDA    = 2;   // VL53L1X SDA
static const uint8_t PIN_SCL    = 3;   // VL53L1X SCL
static const uint8_t PIN_XSHUT  = 5;   // VL53L1X shutdown (LOW = laser spento)
static const uint8_t PIN_RGB    = 8;   // WS2812B integrato
static const uint8_t PIN_BOOT   = 9;   // tasto BOOT: tenuto 3 s = factory reset Zigbee

// ===================== ZIGBEE =====================
static const uint8_t  ZB_ENDPOINT      = 10;
static const uint16_t CUSTOM_CLUSTER   = 0xFC00;
enum : uint16_t {
  ATTR_ZERO_CM = 0x0000,
  ATTR_MAX_CM  = 0x0001,
  ATTR_SLEEP_START = 0x0002,
  ATTR_SLEEP_END   = 0x0003,
  ATTR_INTERVAL_MS = 0x0004
};

// ===================== PARAMETRI =====================
static const char*    TZ_STRING        = "CET-1CEST,M3.5.0,M10.5.0/3"; // Italia (il coordinatore invia UTC)
static const uint32_t TIME_RESYNC_S    = 6UL * 3600;   // risincronizzo l'ora ogni 6 h
static const uint32_t TIME_WAIT_MS     = 20000;        // attesa risposta Time all'avvio
static const float    DELTA_CM         = 2.0f;         // soglia di variazione per il report
static const uint8_t  ADC_SAMPLES      = 15;
static const bool     INVERT_PERCENT   = false;        // true se in HA risulta invertita
static const float    VBAT_EMPTY       = 3.2f;
static const float    VBAT_FULL        = 4.2f;

// ===================== STATO / NVS =====================
struct Config {
  uint16_t zeroCm      = 20;
  uint16_t maxCm       = 220;
  uint8_t  sleepStart  = 20;
  uint8_t  sleepEnd    = 7;
  uint16_t intervalMs  = 500;
} cfg;

Preferences prefs;
SFEVL53L1X distanceSensor;

RTC_DATA_ATTR uint32_t lastTimeSyncEpoch = 0;   // sopravvive al deep sleep
RTC_DATA_ATTR bool     timeValid         = false;

volatile bool    timeResponseReceived = false;
float            lastReportedCm = -100.0f;
uint8_t          lastPercent    = 255;
bool             isTimerWake    = false;

// ===================== LED =====================
void ledColor(uint8_t r, uint8_t g, uint8_t b) { neopixelWrite(PIN_RGB, r, g, b); }
void ledOff() { neopixelWrite(PIN_RGB, 0, 0, 0); }

// ===================== NVS =====================
void loadConfig() {
  prefs.begin("awning", true);
  cfg.zeroCm     = prefs.getUShort("zero", 20);
  cfg.maxCm      = prefs.getUShort("max", 220);
  cfg.sleepStart = prefs.getUChar("slStart", 20);
  cfg.sleepEnd   = prefs.getUChar("slEnd", 7);
  cfg.intervalMs = prefs.getUShort("intMs", 500);
  prefs.end();
}

void saveConfig() {
  prefs.begin("awning", false);
  prefs.putUShort("zero", cfg.zeroCm);
  prefs.putUShort("max", cfg.maxCm);
  prefs.putUChar("slStart", cfg.sleepStart);
  prefs.putUChar("slEnd", cfg.sleepEnd);
  prefs.putUShort("intMs", cfg.intervalMs);
  prefs.end();
}

// ===================== BATTERIA =====================
float readBatteryVolts() {
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);   // ~12 dB nel core 3.x (ADC_11db e' il valore massimo)
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += analogReadMilliVolts(PIN_VBAT);       // usa la calibrazione eFuse
    delay(1);
  }
  float mv = sum / (float)ADC_SAMPLES;
  return (mv * 2.0f) / 1000.0f;                  // partitore 1:2
}

uint8_t batteryPercent(float v) {
  // curva LiPo semplificata (piecewise lineare)
  static const float tv[] = {3.20f, 3.50f, 3.70f, 3.85f, 4.00f, 4.20f};
  static const uint8_t tp[] = {0, 5, 25, 55, 80, 100};
  if (v <= tv[0]) return 0;
  if (v >= tv[5]) return 100;
  for (int i = 0; i < 5; i++) {
    if (v <= tv[i + 1]) {
      float f = (v - tv[i]) / (tv[i + 1] - tv[i]);
      return (uint8_t)(tp[i] + f * (tp[i + 1] - tp[i]));
    }
  }
  return 100;
}

// ===================== ENDPOINT ZIGBEE CUSTOM =====================
class AwningEndpoint : public ZigbeeCover {
public:
  explicit AwningEndpoint(uint8_t ep) : ZigbeeCover(ep) {
    addCustomCluster();
    addTimeClientCluster();
  }

  // Richiesta lettura Time (0x0000) al coordinatore (short addr 0x0000, ep 1)
  void requestTime() {
    static uint16_t attrId = 0x0000;
    esp_zb_zcl_read_attr_cmd_t cmd = {};
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    cmd.zcl_basic_cmd.dst_endpoint = 1;
    cmd.zcl_basic_cmd.src_endpoint = ZB_ENDPOINT;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TIME;
    cmd.attr_number = 1;
    cmd.attr_field = &attrId;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_read_attr_cmd_req(&cmd);
    esp_zb_lock_release();
  }

  // Scrittura attributi custom da HA  [V2]
  void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *msg) override {
    if (msg->info.cluster == CUSTOM_CLUSTER) {
      bool changed = true;
      switch (msg->attribute.id) {
        case ATTR_ZERO_CM:     cfg.zeroCm     = *(uint16_t *)msg->attribute.data.value; break;
        case ATTR_MAX_CM:      cfg.maxCm      = *(uint16_t *)msg->attribute.data.value; break;
        case ATTR_SLEEP_START: cfg.sleepStart = *(uint8_t  *)msg->attribute.data.value; break;
        case ATTR_SLEEP_END:   cfg.sleepEnd   = *(uint8_t  *)msg->attribute.data.value; break;
        case ATTR_INTERVAL_MS: cfg.intervalMs = *(uint16_t *)msg->attribute.data.value; break;
        default: changed = false;
      }
      if (changed) {
        saveConfig();
        lastReportedCm = -100.0f;  // forza ricalcolo percentuale con i nuovi parametri
        Serial.printf("Attr custom 0x%04x aggiornato e salvato in NVS\n", msg->attribute.id);
      }
      return;
    }
    ZigbeeCover::zbAttributeSet(msg);
  }

  // Risposta alla lettura del cluster Time  [V2]
  void zbAttributeRead(uint16_t cluster_id, const esp_zb_zcl_attribute_t *attr) override {
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TIME && attr->id == 0x0000 &&
        attr->data.type == ESP_ZB_ZCL_ATTR_TYPE_UTC_TIME && attr->data.value) {
      uint32_t zbSeconds = *(uint32_t *)attr->data.value;     // secondi dal 1/1/2000
      struct timeval tv = {.tv_sec = (time_t)(zbSeconds + 946684800UL), .tv_usec = 0};
      settimeofday(&tv, nullptr);
      lastTimeSyncEpoch = (uint32_t)tv.tv_sec;
      timeValid = true;
      timeResponseReceived = true;
      Serial.printf("Ora sincronizzata via Zigbee (epoch %lu)\n", (unsigned long)tv.tv_sec);
      return;
    }
    ZigbeeCover::zbAttributeRead(cluster_id, attr);
  }

private:
  void addCustomCluster() {   // [V1]
    esp_zb_attribute_list_t *c = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER);
    uint16_t zero = cfg.zeroCm, mx = cfg.maxCm, itv = cfg.intervalMs;
    uint8_t  s = cfg.sleepStart, e = cfg.sleepEnd;
    const uint8_t rw = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_ZERO_CM,     ESP_ZB_ZCL_ATTR_TYPE_U16, rw, &zero);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_MAX_CM,      ESP_ZB_ZCL_ATTR_TYPE_U16, rw, &mx);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_SLEEP_START, ESP_ZB_ZCL_ATTR_TYPE_U8,  rw, &s);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_SLEEP_END,   ESP_ZB_ZCL_ATTR_TYPE_U8,  rw, &e);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_INTERVAL_MS, ESP_ZB_ZCL_ATTR_TYPE_U16, rw, &itv);
    esp_zb_cluster_list_add_custom_cluster(_cluster_list, c, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  }

  void addTimeClientCluster() {   // [V1]
    esp_zb_attribute_list_t *t = esp_zb_time_cluster_create(nullptr);
    esp_zb_cluster_list_add_time_cluster(_cluster_list, t, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
  }
};

AwningEndpoint zbAwning(ZB_ENDPOINT);

// ===================== SLEEP =====================
bool inSleepWindow(const tm &t) {
  uint8_t h = t.tm_hour;
  if (cfg.sleepStart == cfg.sleepEnd) return false;
  if (cfg.sleepStart < cfg.sleepEnd) return h >= cfg.sleepStart && h < cfg.sleepEnd;
  return h >= cfg.sleepStart || h < cfg.sleepEnd;   // finestra a cavallo di mezzanotte (20 -> 7)
}

uint32_t secondsToSleepEnd(const tm &now) {
  tm end = now;
  end.tm_hour = cfg.sleepEnd; end.tm_min = 0; end.tm_sec = 0;
  time_t endT = mktime(&end);
  time_t nowT = mktime((tm *)&now);
  if (endT <= nowT) endT += 24 * 3600;
  return (uint32_t)(endT - nowT);
}

void enterNightDeepSleep(uint32_t seconds) {
  Serial.printf("Deep sleep notturno per %lu s\n", (unsigned long)seconds);
  Serial.flush();
  distanceSensor.stopRanging();
  digitalWrite(PIN_XSHUT, LOW);                 // laser spento
  gpio_hold_en((gpio_num_t)PIN_XSHUT);          // mantiene LOW durante il deep sleep
  gpio_deep_sleep_hold_en();
  ledOff();
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  loadConfig();

  isTimerWake = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER);
  gpio_hold_dis((gpio_num_t)PIN_XSHUT);
  gpio_deep_sleep_hold_dis();

  setenv("TZ", TZ_STRING, 1);
  tzset();

  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);   // laser spento finche' non serve

  // --- Zigbee ---
  zbAwning.setManufacturerAndModel("Ivan", "AwningToF");     // [V3]
  zbAwning.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, 42); // [V3]
  Zigbee.addEndpoint(&zbAwning);
  Zigbee.setRxOnWhenIdle(false);                             // End Device a basso consumo

  if (!Zigbee.begin(ZIGBEE_END_DEVICE)) {
    Serial.println("Zigbee start fallito, riavvio");
    delay(1000);
    ESP.restart();
  }
  esp_zb_sleep_enable(true);   // abilita il light sleep dello stack quando idle

  // --- Attesa join (LED blu lampeggiante solo se NON e' un risveglio notturno) ---
  bool led = !isTimerWake;
  bool ledState = false;
  while (!Zigbee.connected()) {
    if (led) { ledState = !ledState; ledState ? ledColor(0, 0, 40) : ledOff(); }
    delay(150);
  }
  if (led) { ledColor(0, 40, 0); delay(5000); ledOff(); }   // verde fisso 5 s poi spento

  // --- Sincronizzazione ora ---
  bool needSync = !timeValid || (time(nullptr) - lastTimeSyncEpoch) > TIME_RESYNC_S;
  if (needSync) {
    uint32_t t0 = millis();
    while (!timeResponseReceived && millis() - t0 < TIME_WAIT_MS) {
      zbAwning.requestTime();
      delay(2000);
    }
  }

  // --- Sensore laser ---
  Wire.begin(PIN_SDA, PIN_SCL);
  digitalWrite(PIN_XSHUT, HIGH);
  delay(10);
  if (distanceSensor.begin() != 0) {
    Serial.println("VL53L1X non trovato!");
  } else {
    distanceSensor.setDistanceModeLong();
    distanceSensor.setTimingBudgetInMs(100);
    distanceSensor.setIntervalInMs(100);
    distanceSensor.startRanging();
  }

  // prima misura batteria
  float v = readBatteryVolts();
  zbAwning.setBatteryVoltage((uint8_t)roundf(v * 10.0f));   // [V3] unita' 100 mV
  zbAwning.setBatteryPercentage(batteryPercent(v));         // [V3]
  zbAwning.reportBatteryPercentage();
}

// ===================== LOOP =====================
uint32_t lastBatteryMs = 0;
uint32_t bootPressStart = 0;

void loop() {
  // Factory reset Zigbee (tasto BOOT 3 s)
  if (digitalRead(PIN_BOOT) == LOW) {
    if (!bootPressStart) bootPressStart = millis();
    if (millis() - bootPressStart > 3000) {
      Serial.println("Factory reset Zigbee");
      Zigbee.factoryReset();      // riavvia
    }
  } else bootPressStart = 0;

  // --- Gestione fascia notturna ---
  if (timeValid) {
    time_t now = time(nullptr);
    tm lt; localtime_r(&now, &lt);
    if (inSleepWindow(lt)) {
      enterNightDeepSleep(secondsToSleepEnd(lt));   // non ritorna
    }
  }

  // --- Lettura laser ---
  if (distanceSensor.checkForDataReady()) {
    float cm = distanceSensor.getDistance() / 10.0f;   // mm -> cm
    distanceSensor.clearInterrupt();

    if (fabsf(cm - lastReportedCm) >= DELTA_CM) {
      float span = (float)cfg.maxCm - (float)cfg.zeroCm;
      float pct  = span > 0 ? ((cm - cfg.zeroCm) / span) * 100.0f : 0;
      pct = constrain(pct, 0.0f, 100.0f);
      if (INVERT_PERCENT) pct = 100.0f - pct;
      uint8_t p = (uint8_t)roundf(pct);

      lastReportedCm = cm;
      if (p != lastPercent) {
        lastPercent = p;
        zbAwning.setLiftPercentage(p);   // [V3] aggiorna 0x0102/0x0008 e invia il report
        Serial.printf("Dist %.1f cm -> %u%%\n", cm, p);
      }
    }
  }

  // --- Batteria ogni 10 minuti ---
  if (millis() - lastBatteryMs > 600000UL || lastBatteryMs == 0) {
    lastBatteryMs = millis();
    float v = readBatteryVolts();
    zbAwning.setBatteryVoltage((uint8_t)roundf(v * 10.0f));
    zbAwning.setBatteryPercentage(batteryPercent(v));
    zbAwning.reportBatteryPercentage();
    Serial.printf("Batteria %.2f V (%u%%)\n", v, batteryPercent(v));
  }

  // --- Resync ora periodico ---
  if (timeValid && (time(nullptr) - lastTimeSyncEpoch) > TIME_RESYNC_S) {
    timeResponseReceived = false;
    zbAwning.requestTime();
  }

  delay(cfg.intervalMs);   // con sleep abilitato la CPU puo' andare in light sleep durante il delay
}
