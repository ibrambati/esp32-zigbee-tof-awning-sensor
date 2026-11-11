/*
 * esp32-zigbee-tof-awning-sensor  -  Firmware nativo Zigbee (End Device a batteria)
 * Board: Waveshare ESP32-H2-Zero
 *
 * ---- IMPOSTAZIONI ARDUINO IDE (Strumenti) ----
 *  Board:             ESP32H2 Dev Module
 *  Zigbee Mode:       Zigbee ED (end device)
 *  Partition Scheme:  Zigbee 4MB with spiffs   (serve la partizione zb_storage)
 *  USB CDC On Boot:   Enabled (per il Serial)
 *  Core esp32:        >= 3.3.x (Zigbee.h)
 *  Librerie:          SparkFun VL53L1X 4m Laser Distance Sensor
 *  (con arduino-cli le stesse opzioni arrivano da sketch.yaml)
 *
 * ---- DISPOSITIVO ----
 *  Endpoint 10 di tipo "Simple Sensor": NON e' una tenda, quindi niente cluster Window Covering.
 *  Lato Zigbee2MQTT serve il converter z2m/awning_tof.mjs (ID e tipi degli attributi devono coincidere).
 *
 * Mappa cluster esposti (endpoint 10):
 *  0x0001 Power Config     -> BatteryPercentageRemaining (0x0021, unita' 0,5 %)
 *  0x000A Time (client)    -> letto dal coordinatore all'avvio e ogni TIME_RESYNC_S
 *  0xFC00 Custom (server), tutti gli attributi sono anche riportati al coordinatore:
 *     sola lettura: 0x0010 DistanceMm (U16, mm)      0x0011 OpenPercent (U8, %)
 *                   0x0012 BatteryMv  (U16, mV)
 *                   0x0013 Status     (U8: 0 attivo, 1 deep sleep, 2 in carica, 3 carica completa)
 *     scrittura:    0x0000 ClosedPositionZero (U16, cm)  0x0001 MaxLength (U16, cm)
 *                   0x0002 SleepStartHour (U8)           0x0003 SleepEndHour (U8)
 *                   0x0004 ReadIntervalMs (U16)      0x0005 LedMode (U8: 0 always-on, 1 blink, 2 boot-only)
 *                   0x0006 SleepEnabled (U8)         0x0007 SensorEnabled (U8: 1 laser acceso, 0 spento - debug)
 *  I report vanno all'indirizzo del coordinatore (0x0000, ep 1) senza dipendere dal binding.
 *
 * ---- OTA (aggiornamento firmware via Zigbee2MQTT) ----
 *  Cluster OTA Upgrade (0x0019, client) sull'endpoint 10. Z2M lo serve da un indice locale (vedi z2m/ota/).
 *  FW_VERSION va incrementata a ogni build da distribuire: Z2M aggiorna solo se la versione del file e' maggiore.
 *  Durante un OTA il dispositivo resta sveglio (niente deep sleep, niente light sleep) e dopo il riavvio
 *  tiene aperta una "finestra OTA" (OTA_WINDOW_MS) in cui il deep sleep notturno e' sospeso.
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
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "SparkFun_VL53L1X.h"

// ===================== PIN =====================
static const uint8_t PIN_VBAT   = 1;   // ADC1_CH0, partitore 100k/100k + 100nF
static const uint8_t PIN_SDA    = 2;   // VL53L1X SDA
static const uint8_t PIN_SCL    = 3;   // VL53L1X SCL
static const uint8_t PIN_XSHUT  = 5;   // VL53L1X shutdown (LOW = laser spento)
static const uint8_t PIN_VIN    = 10;   // presenza caricatore: V_IN del TP4056 con partitore 100k/150k (alto = USB collegato)
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
  ATTR_INTERVAL_MS = 0x0004,
  ATTR_LED_MODE    = 0x0005,
  ATTR_SLEEP_ENABLED = 0x0006,   // 0 = deep sleep notturno disabilitato (default), 1 = abilitato
  ATTR_SENSOR_ENABLED = 0x0007,  // 1 = laser acceso (default), 0 = spento via XSHUT (debug)
  ATTR_DIST_MM     = 0x0010,   // sola lettura
  ATTR_OPEN_PCT    = 0x0011,   // sola lettura
  ATTR_VBAT_MV     = 0x0012,   // sola lettura
  ATTR_STATUS      = 0x0013    // sola lettura, valori STATUS_*
};
enum : uint8_t { STATUS_ACTIVE = 0, STATUS_DEEP_SLEEP = 1, STATUS_CHARGING = 2, STATUS_CHARGED = 3 };
static const uint16_t COORD_SHORT_ADDR = 0x0000;   // il coordinatore e' sempre 0x0000
static const uint8_t  COORD_ENDPOINT   = 1;
static const uint16_t ATTR_BATT_PCT_REMAINING = 0x0021;  // ZCL Power Config: batteryPercentageRemaining

// ===================== OTA =====================
// Versione del firmware in formato 0xMMmmpppp (Zigbee file_version, 32 bit). INCREMENTARLA a ogni release.
static const uint32_t FW_VERSION           = 0x00010001;     // 0.1.0
static const uint32_t OTA_DOWNLOADED_VER   = 0xFFFFFFFF;     // "nessun file scaricato" (valore di default ZCL)
static const uint16_t OTA_HW_VERSION       = 0x0001;
// Manufacturer code e image type: DEVONO coincidere con l'indice OTA di Z2M (z2m/ota/index.json).
// 0x1001 e' il codice usato dall'SDK Espressif per i test; il valore e' libero per un dispositivo DIY.
static const uint16_t OTA_MANUFACTURER     = 0x1001;
static const uint16_t OTA_IMAGE_TYPE       = 0x1011;
static const uint8_t  OTA_MAX_DATA_SIZE    = 64;             // blocchi piccoli: piu' compatibili con Z2M (default 50-64 byte)
// Dopo ogni boot (e quindi dopo ogni risveglio) il deep sleep notturno resta sospeso per questo tempo:
// lascia al client OTA il tempo di interrogare Z2M e di scaricare l'immagine.
static const uint32_t OTA_WINDOW_MS        = 10UL * 60 * 1000;

// ===================== PARAMETRI =====================
static const char*    TZ_STRING        = "CET-1CEST,M3.5.0,M10.5.0/3"; // Italia (il coordinatore invia UTC)
static const uint32_t TIME_RESYNC_S    = 6UL * 3600;   // risincronizzo l'ora ogni 6 h
static const uint32_t TIME_WAIT_MS     = 20000;        // attesa risposta Time all'avvio
static const float    DELTA_CM         = 2.0f;         // soglia di variazione per il report
static const uint8_t  ADC_SAMPLES      = 15;
static const uint16_t LASER_BUDGET_MS  = 100;
// Ciclo del laser = ReadIntervalMs impostato da FE (min 100 ms): LASER_BUDGET_MS di misura + (intervallo - budget) di riposo,
// gestito dal sensore stesso (inter-measurement period). Es.: 500 ms -> 100 ms misura + 400 ms riposo.
// Batteria: si legge a laser fermo, dopo BATTERY_PRE_SETTLE_MS, con mediana di ADC_SAMPLES campioni.
// La prima lettura dopo il boot avviene BATTERY_FIRST_DELAY_MS dopo il join (evita i valori bassi del boot).
static const uint32_t BATTERY_PRE_SETTLE_MS  = 150;
static const uint32_t BATTERY_FIRST_DELAY_MS = 60000UL;
// Poll interval lungo dell'end device (radio spenta tra un poll e l'altro). Tenerlo sotto ~7 s:
// oltre, il parent puo' scartare le scritture in coda (comandi da Z2M). Le notifiche in uscita non dipendono dal poll.
static const uint32_t ZB_POLL_INTERVAL_MS = 4000;
// Il boot parte SEMPRE con la radio accesa (rejoin affidabile). A rete stabile enterSleepy() passa a End Device
// sleepy a runtime: PAIRING_SETTLE_MS dopo il primo accoppiamento (lascia finire l'interview di Z2M),
// SLEEPY_DELAY_MS dopo il join nei boot normali.
static const uint32_t PAIRING_SETTLE_MS   = 3UL * 60 * 1000;
static const uint32_t SLEEPY_DELAY_MS     = 20UL * 1000;
static const bool     INVERT_PERCENT   = false;        // true se in HA risulta invertita
static const float    VBAT_EMPTY       = 3.2f;
static const float    VBAT_FULL        = 4.2f;
static const float    VBAT_CAL         = 1.0f;         // correzione partitore/ADC: tensione multimetro / tensione letta
static const float    VBAT_CHARGED     = 4.15f;        // in carica, sopra questa soglia si conta la fine carica
static const uint32_t CHARGED_CONFIRM_MS = 15UL * 60 * 1000;  // ...per almeno 15 minuti di fila
static const uint32_t BATTERY_INTERVAL_MS          = 600000UL;   // lettura batteria a batteria: 10 min
static const uint32_t BATTERY_INTERVAL_CHARGING_MS = 60000UL;    // con caricatore collegato: 1 min
static const uint32_t BATTERY_SETTLE_MS            = 120000UL;   // dopo lo stacco del caricatore la % aspetta 2 min
enum : uint8_t { LED_ALWAYS_ON = 0, LED_BLINK = 1, LED_BOOT_ONLY = 2 };   // parametro ledMode
static const uint32_t LED_BOOT_MS      = 5000;         // durata del verde in modalita' "boot-only"

// ===================== STATO / NVS =====================
struct Config {
  uint16_t zeroCm      = 20;
  uint16_t maxCm       = 220;
  uint8_t  sleepStart  = 20;
  uint8_t  sleepEnd    = 7;
  uint16_t intervalMs  = 500;
  uint8_t  ledMode     = LED_ALWAYS_ON;
  uint8_t  sleepEnabled = 0;               // deep sleep notturno: 0 = disabilitato (default)
  uint8_t  sensorEnabled = 1;              // switch di debug: 0 = laser spento (XSHUT LOW), 1 = acceso
} cfg;

Preferences prefs;
SFEVL53L1X distanceSensor;
bool     sensorOk = false;        // true se il VL53L1X e' stato inizializzato correttamente
uint32_t lastSensorTryMs = 0;     // ultimo tentativo di inizializzazione

// Periodo del ciclo laser: ReadIntervalMs, mai sotto il budget di misura.
uint16_t laserPeriodMs() { return cfg.intervalMs < LASER_BUDGET_MS ? LASER_BUDGET_MS : cfg.intervalMs; }
uint16_t laserPeriodApplied = 0;   // periodo gia' configurato nel sensore

// Inizializza il laser (XSHUT alto + begin + parametri + startRanging). Ritorna true se risponde.
bool initSensor() {
  digitalWrite(PIN_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_XSHUT, HIGH);
  delay(10);
  if (distanceSensor.begin() != 0) return false;
  distanceSensor.setDistanceModeLong();
  distanceSensor.setTimingBudgetInMs(LASER_BUDGET_MS);
  distanceSensor.setIntermeasurementPeriod(laserPeriodMs());   // libreria SparkFun: nome corretto del metodo
  laserPeriodApplied = laserPeriodMs();
  distanceSensor.startRanging();
  return true;
}

// Cambio intervallo da FE a sensore gia' acceso: ferma, riconfigura il periodo, riparte.
void applyLaserTiming() {
  if (!sensorOk || laserPeriodApplied == laserPeriodMs()) return;
  laserPeriodApplied = laserPeriodMs();
  distanceSensor.stopRanging();
  distanceSensor.setTimingBudgetInMs(LASER_BUDGET_MS);
  distanceSensor.setIntermeasurementPeriod(laserPeriodMs());
  distanceSensor.startRanging();
}

// Switch di debug: accende o spegne fisicamente il laser (XSHUT). Va chiamata solo dal loop()/setup(),
// mai dal callback Zigbee (tocca il bus I2C).
void applySensorEnabled() {
  if (cfg.sensorEnabled) {
    if (!sensorOk) {
      lastSensorTryMs = millis() - 15001UL;   // forza un tentativo di init subito, nel prossimo giro del loop
      Serial.println("Sensore: acceso da switch");
    }
  } else {
    if (sensorOk) distanceSensor.stopRanging();
    sensorOk = false;
    digitalWrite(PIN_XSHUT, LOW);              // laser spento
    Serial.println("Sensore: spento da switch (XSHUT LOW)");
  }
}

RTC_DATA_ATTR uint32_t lastTimeSyncEpoch = 0;   // sopravvive al deep sleep
RTC_DATA_ATTR bool     timeValid         = false;

volatile bool    timeResponseReceived = false;
volatile bool    cfgDirty             = false;   // settato dal callback Zigbee, gestito nel loop
volatile bool    otaRunning           = false;   // true mentre Z2M sta trasferendo un'immagine
float            lastReportedCm = -100.0f;
uint8_t          lastPercent    = 255;
bool             isTimerWake    = false;

// ===================== LED =====================
void ledColor(uint8_t r, uint8_t g, uint8_t b) { rgbLedWrite(PIN_RGB, r, g, b); }
void ledOff() { rgbLedWrite(PIN_RGB, 0, 0, 0); }
// Su questa scheda il LED mostrava ROSSO scrivendo (0,40,0): canali R/G invertiti. Se il verde esce rosso, lascia true; se esce verde, metti false.
static const bool LED_RG_SWAPPED = true;
void ledGreen() { if (LED_RG_SWAPPED) ledColor(40, 0, 0); else ledColor(0, 40, 0); }

bool     zbJoined       = false;   // true quando il dispositivo e' connesso al controller
bool     sleepyActive   = false;   // true dopo il passaggio a End Device sleepy (a runtime)
bool     pairingMode    = false;   // true finche' il primo accoppiamento non e' completato (flag NVS "paired")
uint32_t joinedAtMs     = 0;       // istante (millis) del join in questo boot
uint32_t otaWindowUntilMs = 0;     // fino a questo istante (millis) il deep sleep notturno e' sospeso per l'OTA
uint32_t ledBootUntilMs = 0;       // fine del verde in modalita' boot-only
uint8_t  lastLedState   = 255;     // ultimo stato scritto sul LED (255 = da riapplicare)

// Dopo il join il LED e' verde secondo ledMode: sempre acceso, lampeggiante, o solo 5 s all'avvio.
void updateLed() {
  if (!zbJoined) return;           // prima del join lo gestisce setup() (blu lampeggiante)
  bool on;
  switch (cfg.ledMode) {
    case LED_BLINK:     on = (millis() % 2000UL) < 200UL; break;                  // 200 ms ogni 2 s
    case LED_BOOT_ONLY: on = (int32_t)(ledBootUntilMs - millis()) > 0; break;
    default:            on = true; break;                                        // LED_ALWAYS_ON
  }
  if (lastLedState != (uint8_t)on) {
    lastLedState = on;
    if (on) ledGreen(); else ledOff();
  }
}

// ===================== OTA CALLBACK =====================
// Chiamata dal task Zigbee all'inizio e alla fine del trasferimento: solo flag, niente I/O pesante.
void otaStateCallback(bool active) {
  otaRunning = active;
  Serial.println(active ? "OTA: trasferimento avviato" : "OTA: trasferimento terminato");
}

// ===================== NVS =====================
void loadConfig() {
  prefs.begin("awning", true);
  cfg.zeroCm     = prefs.getUShort("zero", 20);
  cfg.maxCm      = prefs.getUShort("max", 220);
  cfg.sleepStart = prefs.getUChar("slStart", 20);
  cfg.sleepEnd   = prefs.getUChar("slEnd", 7);
  cfg.intervalMs = prefs.getUShort("intMs", 500);
  cfg.ledMode    = prefs.getUChar("ledMode", LED_ALWAYS_ON);
  if (cfg.ledMode > LED_BOOT_ONLY) cfg.ledMode = LED_ALWAYS_ON;
  cfg.sleepEnabled = prefs.getUChar("slEn", 0) ? 1 : 0;
  cfg.sensorEnabled = prefs.getUChar("sensEn", 1) ? 1 : 0;
  prefs.end();
}

// Flag "accoppiamento completato" (NVS). Va azzerato prima di ogni factory reset Zigbee.
bool loadPaired() {
  prefs.begin("awning", true);
  bool p = prefs.getUChar("paired", 0) != 0;
  prefs.end();
  return p;
}

void setPaired(bool v) {
  prefs.begin("awning", false);
  prefs.putUChar("paired", v ? 1 : 0);
  prefs.end();
}

// Passa a End Device sleepy senza riavviare: rx_on_when_idle off, poll lungo, light sleep dello stack.
void enterSleepy() {
  Serial.println("Passo a modalita' sleepy (basso consumo)");
  if (pairingMode) { setPaired(true); pairingMode = false; }
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_set_rx_on_when_idle(false);
  esp_zb_set_default_long_poll_interval(ZB_POLL_INTERVAL_MS);
  esp_zb_lock_release();
  esp_zb_sleep_enable(true);
  sleepyActive = true;
  Serial.printf("rx_on_when_idle = %d\n", (int)esp_zb_get_rx_on_when_idle());
}

void saveConfig() {
  prefs.begin("awning", false);
  prefs.putUShort("zero", cfg.zeroCm);
  prefs.putUShort("max", cfg.maxCm);
  prefs.putUChar("slStart", cfg.sleepStart);
  prefs.putUChar("slEnd", cfg.sleepEnd);
  prefs.putUShort("intMs", cfg.intervalMs);
  prefs.putUChar("ledMode", cfg.ledMode);
  prefs.putUChar("slEn", cfg.sleepEnabled);
  prefs.putUChar("sensEn", cfg.sensorEnabled);
  prefs.end();
}

// ===================== BATTERIA =====================
float readBatteryVolts() {
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);   // ~12 dB nel core 3.x (ADC_11db e' il valore massimo)
  uint32_t v[ADC_SAMPLES];
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    v[i] = analogReadMilliVolts(PIN_VBAT);       // usa la calibrazione eFuse
    delay(1);
  }
  for (uint8_t i = 1; i < ADC_SAMPLES; i++) {    // ordinamento a inserimento: serve solo la mediana
    uint32_t x = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  float mv = v[ADC_SAMPLES / 2];                 // mediana: scarta i picchi (es. trasmissione radio)
  return (mv * 2.0f) / 1000.0f * VBAT_CAL;       // partitore 1:2
}

// Lettura a laser fermo: ferma il ranging, attende che il partitore si stabilizzi, legge, riparte.
float readBatteryQuiet() {
  bool wasRanging = sensorOk;
  if (wasRanging) distanceSensor.stopRanging();
  delay(BATTERY_PRE_SETTLE_MS);
  float v = readBatteryVolts();
  if (wasRanging) distanceSensor.startRanging();
  return v;
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

// ===================== CARICATORE =====================
// Il TP4056 non espone il pin CHRG: la presenza dell'USB si legge da V_IN (partitore su GPIO10) e la
// fine carica si deduce dalla tensione (sopra VBAT_CHARGED per CHARGED_CONFIRM_MS). Stima, non il
// segnale del chip: nella fase finale a 4,2 V "quasi carica" e "carica" non si distinguono.
bool     chargerConnected = false;
uint32_t chargedSinceMs   = 0;    // prima lettura >= VBAT_CHARGED con caricatore collegato (0 = nessuna)
uint32_t chargerRemovedMs = 0;    // istante dello stacco del caricatore (0 = mai staccato da questo boot)

// Pull interni disattivati: il pull-down (~45k) in parallelo ai 150k del partitore porterebbe il pin a ~1,3 V (basso).
void configureChargerPin() {
  pinMode(PIN_VIN, INPUT);
  gpio_pullup_dis((gpio_num_t)PIN_VIN);
  gpio_pulldown_dis((gpio_num_t)PIN_VIN);
}

// Ritorna true se lo stato del caricatore e' cambiato.
bool updateCharger() {
  bool now = digitalRead(PIN_VIN) == HIGH;
  if (now == chargerConnected) return false;
  chargerConnected = now;
  chargedSinceMs = 0;
  if (!now) chargerRemovedMs = millis() | 1;
  Serial.println(now ? "Caricatore collegato" : "Caricatore scollegato");
  return true;
}

uint8_t currentStatus() {
  if (!chargerConnected) return STATUS_ACTIVE;
  if (chargedSinceMs && millis() - chargedSinceMs >= CHARGED_CONFIRM_MS) return STATUS_CHARGED;
  return STATUS_CHARGING;
}

// ===================== ENDPOINT ZIGBEE =====================
// Endpoint generico "Simple Sensor" (nessun Window Covering). I cluster si costruiscono con
// buildClusters(), da chiamare in setup() DOPO loadConfig(): cosi' gli attributi custom
// partono dai valori salvati in NVS.
class AwningEndpoint : public ZigbeeEP {
public:
  explicit AwningEndpoint(uint8_t ep) : ZigbeeEP(ep) {
    _device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID;
  }

  void buildClusters() {
    _cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(_cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(_cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    addCustomCluster();
    addTimeClientCluster();
    // Client OTA sullo stesso endpoint: va aggiunto dopo la creazione di _cluster_list e prima di Zigbee.begin()
    if (!addOTAClient(FW_VERSION, OTA_DOWNLOADED_VER, OTA_HW_VERSION, OTA_MANUFACTURER, OTA_IMAGE_TYPE, OTA_MAX_DATA_SIZE)) {
      Serial.println("OTA: aggiunta del cluster client fallita");
    }
    _ep_config = {
      .endpoint = _endpoint,
      .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
      .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
      .app_device_version = 0
    };
  }

  // ---- pubblicazione valori: aggiorna l'attributo e lo riporta al coordinatore ----
  void publishDistanceMm(uint16_t mm) { publishCustom(ATTR_DIST_MM, &mm); }
  void publishOpenPercent(uint8_t p)  { publishCustom(ATTR_OPEN_PCT, &p); }

  void publishStatus(uint8_t st) { publishCustom(ATTR_STATUS, &st); }

  // updatePct = false: si riporta solo la tensione e la percentuale resta all'ultimo valore valido
  // (in carica la tensione ai morsetti e' gonfiata dalla corrente di carica).
  void publishBattery(float volts, uint8_t pct, bool updatePct = true) {
    uint16_t mv = (uint16_t)roundf(volts * 1000.0f);
    publishCustom(ATTR_VBAT_MV, &mv);
    setBatteryVoltage((uint8_t)roundf(volts * 10.0f));   // cluster standard, unita' 100 mV
    if (!updatePct) return;
    setBatteryPercentage(pct);                           // il core moltiplica per 2 (unita' ZCL)
    reportToCoordinator(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ATTR_BATT_PCT_REMAINING);
  }

  void publishConfig() {
    uint16_t zero = cfg.zeroCm, mx = cfg.maxCm, itv = cfg.intervalMs;
    uint8_t  s = cfg.sleepStart, e = cfg.sleepEnd, lm = cfg.ledMode, se = cfg.sleepEnabled, sn = cfg.sensorEnabled;
    publishCustom(ATTR_ZERO_CM, &zero);
    publishCustom(ATTR_MAX_CM, &mx);
    publishCustom(ATTR_SLEEP_START, &s);
    publishCustom(ATTR_SLEEP_END, &e);
    publishCustom(ATTR_INTERVAL_MS, &itv);
    publishCustom(ATTR_LED_MODE, &lm);
    publishCustom(ATTR_SLEEP_ENABLED, &se);
    publishCustom(ATTR_SENSOR_ENABLED, &sn);
  }

  // Richiesta lettura Time (0x0000) al coordinatore (short addr 0x0000, ep 1)
  void requestTime() {
    static uint16_t attrId = 0x0000;
    esp_zb_zcl_read_attr_cmd_t cmd = {};
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = COORD_SHORT_ADDR;
    cmd.zcl_basic_cmd.dst_endpoint = COORD_ENDPOINT;
    cmd.zcl_basic_cmd.src_endpoint = _endpoint;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TIME;
    cmd.attr_number = 1;
    cmd.attr_field = &attrId;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_read_attr_cmd_req(&cmd);
    esp_zb_lock_release();
  }

  // Scrittura attributi custom da Home Assistant / Zigbee2MQTT.
  // Gira nel task Zigbee: qui si aggiorna solo cfg, il salvataggio in NVS e il re-report
  // avvengono nel loop() (cfgDirty).
  void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *msg) override {
    if (msg->info.cluster != CUSTOM_CLUSTER) return;
    bool changed = true;
    switch (msg->attribute.id) {
      case ATTR_ZERO_CM:     cfg.zeroCm     = *(uint16_t *)msg->attribute.data.value; break;
      case ATTR_MAX_CM:      cfg.maxCm      = *(uint16_t *)msg->attribute.data.value; break;
      case ATTR_SLEEP_START: cfg.sleepStart = *(uint8_t  *)msg->attribute.data.value; break;
      case ATTR_SLEEP_END:   cfg.sleepEnd   = *(uint8_t  *)msg->attribute.data.value; break;
      case ATTR_INTERVAL_MS: cfg.intervalMs = *(uint16_t *)msg->attribute.data.value; break;
      case ATTR_LED_MODE:    cfg.ledMode    = *(uint8_t  *)msg->attribute.data.value; break;
      case ATTR_SLEEP_ENABLED: cfg.sleepEnabled = *(uint8_t *)msg->attribute.data.value ? 1 : 0; break;
      case ATTR_SENSOR_ENABLED: cfg.sensorEnabled = *(uint8_t *)msg->attribute.data.value ? 1 : 0; break;
      default: changed = false;
    }
    if (changed) cfgDirty = true;
  }

  // Risposta alla lettura del cluster Time: il core la instrada qui, NON a zbAttributeRead().
  void zbReadTimeCluster(const esp_zb_zcl_attribute_t *attr) override {
    if (attr->id == ESP_ZB_ZCL_ATTR_TIME_TIME_ID && attr->data.type == ESP_ZB_ZCL_ATTR_TYPE_UTC_TIME && attr->data.value) {
      uint32_t zbSeconds = *(uint32_t *)attr->data.value;     // secondi dal 1/1/2000 UTC
      struct timeval tv = {.tv_sec = (time_t)(zbSeconds + 946684800UL), .tv_usec = 0};
      settimeofday(&tv, nullptr);
      lastTimeSyncEpoch = (uint32_t)tv.tv_sec;
      timeValid = true;
      timeResponseReceived = true;
    }
  }

private:
  bool publishCustom(uint16_t attrId, void *value) {
    if (setClusterAttribute(CUSTOM_CLUSTER, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attrId, value, false) != ESP_ZB_ZCL_STATUS_SUCCESS) return false;
    return reportToCoordinator(CUSTOM_CLUSTER, attrId);
  }

  // Report diretto al coordinatore (0x0000, ep 1): non richiede che il cluster sia in binding.
  bool reportToCoordinator(uint16_t clusterId, uint16_t attrId) {
    esp_zb_zcl_report_attr_cmd_t r;
    memset(&r, 0, sizeof(r));
    r.zcl_basic_cmd.dst_addr_u.addr_short = COORD_SHORT_ADDR;
    r.zcl_basic_cmd.dst_endpoint = COORD_ENDPOINT;
    r.zcl_basic_cmd.src_endpoint = _endpoint;
    r.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    r.clusterID = clusterId;
    r.attributeID = attrId;
    r.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    r.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
    return reportClusterAttribute(&r);
  }

  void addCustomCluster() {
    esp_zb_attribute_list_t *c = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER);
    uint16_t zero = cfg.zeroCm, mx = cfg.maxCm, itv = cfg.intervalMs, dist = 0, vbat = 0;
    uint8_t  s = cfg.sleepStart, e = cfg.sleepEnd, pct = 0, lm = cfg.ledMode, se = cfg.sleepEnabled, sn = cfg.sensorEnabled, st = STATUS_ACTIVE;
    const uint8_t rw = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
    const uint8_t ro = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY  | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_ZERO_CM,     ESP_ZB_ZCL_ATTR_TYPE_U16, rw, &zero);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_MAX_CM,      ESP_ZB_ZCL_ATTR_TYPE_U16, rw, &mx);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_SLEEP_START, ESP_ZB_ZCL_ATTR_TYPE_U8,  rw, &s);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_SLEEP_END,   ESP_ZB_ZCL_ATTR_TYPE_U8,  rw, &e);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_INTERVAL_MS, ESP_ZB_ZCL_ATTR_TYPE_U16, rw, &itv);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_LED_MODE,    ESP_ZB_ZCL_ATTR_TYPE_U8,  rw, &lm);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_SLEEP_ENABLED, ESP_ZB_ZCL_ATTR_TYPE_U8, rw, &se);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_SENSOR_ENABLED, ESP_ZB_ZCL_ATTR_TYPE_U8, rw, &sn);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_DIST_MM,     ESP_ZB_ZCL_ATTR_TYPE_U16, ro, &dist);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_OPEN_PCT,    ESP_ZB_ZCL_ATTR_TYPE_U8,  ro, &pct);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_VBAT_MV,     ESP_ZB_ZCL_ATTR_TYPE_U16, ro, &vbat);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_STATUS,      ESP_ZB_ZCL_ATTR_TYPE_U8,  ro, &st);
    esp_zb_cluster_list_add_custom_cluster(_cluster_list, c, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  }

  void addTimeClientCluster() {
    esp_zb_attribute_list_t *t = esp_zb_time_cluster_create(nullptr);
    esp_zb_cluster_list_add_time_cluster(_cluster_list, t, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
  }
};

// Applica una modifica di configurazione arrivata via Zigbee: valida, salva in NVS e rimanda
// al coordinatore i valori (eventualmente corretti), cosi' l'interfaccia resta allineata.
void applyConfigChange();

AwningEndpoint zbAwning(ZB_ENDPOINT);

void applyConfigChange() {
  if (cfg.sleepStart > 23) cfg.sleepStart = 23;
  if (cfg.sleepEnd   > 23) cfg.sleepEnd   = 23;
  if (cfg.intervalMs < 100) cfg.intervalMs = 100;
  if (cfg.ledMode > LED_BOOT_ONLY) cfg.ledMode = LED_ALWAYS_ON;
  if (cfg.sleepEnabled > 1) cfg.sleepEnabled = 1;
  if (cfg.sensorEnabled > 1) cfg.sensorEnabled = 1;
  applySensorEnabled();
  applyLaserTiming();          // il ReadIntervalMs e' anche il periodo del ciclo laser
  lastLedState = 255;          // riapplica subito la nuova modalita' del LED
  if (cfg.ledMode == LED_BOOT_ONLY) ledBootUntilMs = millis() + LED_BOOT_MS;   // mostra l'effetto per 5 s
  if (cfg.maxCm <= cfg.zeroCm) cfg.maxCm = cfg.zeroCm + 1;   // evita divisione per zero / span negativo
  saveConfig();
  zbAwning.publishConfig();
  lastReportedCm = -100.0f;   // forza il ricalcolo della percentuale con i nuovi parametri
  lastPercent = 255;
  Serial.printf("Config aggiornata: zero=%u max=%u notte %u->%u (deep sleep %s) intervallo=%u ms led=%u sensore=%s\n",
                cfg.zeroCm, cfg.maxCm, cfg.sleepStart, cfg.sleepEnd, cfg.sleepEnabled ? "ON" : "OFF", cfg.intervalMs, cfg.ledMode, cfg.sensorEnabled ? "ON" : "OFF");
}

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
  zbAwning.publishStatus(STATUS_DEEP_SLEEP);    // da addormentato non puo' trasmettere: lo dice prima
  delay(500);                                   // tempo per far partire il report
  if (sensorOk) distanceSensor.stopRanging();
  digitalWrite(PIN_XSHUT, LOW);                 // laser spento
  gpio_hold_en((gpio_num_t)PIN_XSHUT);          // mantiene LOW durante il deep sleep
#if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP
  gpio_deep_sleep_hold_en();                    // solo sui chip che lo richiedono (non dichiarata su ESP32-H2)
#endif
  ledOff();
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
#if SOC_PM_SUPPORT_EXT1_WAKEUP
  // Risveglio anche quando si collega il caricatore (GPIO10 e' un LP IO). ext1 e non gpio_wakeup:
  // quest'ultimo con la configurazione di default attiva il pull-down interno e falserebbe il partitore.
  gpio_pullup_dis((gpio_num_t)PIN_VIN);
  gpio_pulldown_dis((gpio_num_t)PIN_VIN);
  esp_sleep_enable_ext1_wakeup(1ULL << PIN_VIN, ESP_EXT1_WAKEUP_ANY_HIGH);
#endif
  esp_deep_sleep_start();
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  loadConfig();

  isTimerWake = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER);
  gpio_hold_dis((gpio_num_t)PIN_XSHUT);
#if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP
  gpio_deep_sleep_hold_dis();
#endif

  setenv("TZ", TZ_STRING, 1);
  tzset();

  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);   // laser spento finche' non serve
  configureChargerPin();
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) Serial.println("Risveglio: caricatore collegato");

  // --- Zigbee ---
  zbAwning.buildClusters();                                  // dopo loadConfig(): attributi custom = valori NVS
  zbAwning.setManufacturerAndModel("Ivan", "AwningToF");
  zbAwning.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, 42); // aggiunge il cluster Power Config
  zbAwning.onOTAStateChange(otaStateCallback);
  Zigbee.addEndpoint(&zbAwning);
  Serial.printf("Firmware 0x%08lX\n", (unsigned long)FW_VERSION);
  // Il boot parte SEMPRE a radio accesa (rejoin affidabile). Il passaggio a End Device sleepy avviene
  // a runtime, a rete stabile (vedi enterSleepy()). DIAG_RX_ON_IDLE 1 = resta sempre acceso (diagnosi).
  #define DIAG_RX_ON_IDLE 0
  pairingMode = !loadPaired();
  Serial.printf("Modalita': %s (boot a radio accesa)\n", pairingMode ? "accoppiamento" : "normale");
  Zigbee.setRxOnWhenIdle(true);

  if (!isTimerWake) ledColor(0, 0, 40);   // blu fisso mentre lo stack parte (begin() puo' bloccare fino a 30 s)
  if (!Zigbee.begin(ZIGBEE_END_DEVICE)) {
    if (!Zigbee.initialized()) {          // errore vero di inizializzazione: riavvio
      Serial.println("Zigbee init fallito, riavvio");
      delay(1000);
      ESP.restart();
    }
    // Stack avviato ma rete non ancora trovata entro 30 s (es. permit join chiuso): NON riavviare,
    // lo stack continua a cercare in background. Si aspetta nel ciclo qui sotto con il LED blu lampeggiante.
    Serial.println("Zigbee: rete non ancora trovata, continuo a cercare");
  }

  // --- Attesa join (LED blu lampeggiante solo se NON e' un risveglio notturno) ---
  bool led = !isTimerWake;
  bool ledState = false;
  uint32_t joinBootStart = 0;
  const uint32_t joinWaitStart = millis();
  while (!Zigbee.connected()) {
    if (millis() - joinWaitStart > 600000UL) {   // 10 minuti senza rete: riavvio e riprovo
      Serial.println("Nessuna rete dopo 10 minuti, riavvio");
      ESP.restart();
    }
    if (led) { ledState = !ledState; ledState ? ledColor(0, 0, 40) : ledOff(); }
    // BOOT tenuto 3 s anche durante l'attesa del join = factory reset Zigbee
    if (digitalRead(PIN_BOOT) == LOW) {
      if (!joinBootStart) joinBootStart = millis();
      if (millis() - joinBootStart > 3000) {
        Serial.println("Factory reset Zigbee (attesa join)");
        setPaired(false);           // al prossimo avvio si riparte in modalita' accoppiamento
        Zigbee.factoryReset();      // riavvia
      }
    } else joinBootStart = 0;
    delay(150);
  }
  // Connesso al controller: da qui il LED segue ledMode (verde). In boot-only niente LED dopo un risveglio notturno.
  zbJoined = true;
  joinedAtMs = millis();
  Serial.println(pairingMode ? "Join completato: resto sveglio per l'interview, poi passo a sleepy"
                             : "Join completato");
  ledBootUntilMs = isTimerWake ? 0 : millis() + LED_BOOT_MS;
  updateLed();

  // --- Sincronizzazione ora ---
  bool needSync = !timeValid || (time(nullptr) - lastTimeSyncEpoch) > TIME_RESYNC_S;
  if (needSync) {
    uint32_t t0 = millis();
    while (!timeResponseReceived && millis() - t0 < TIME_WAIT_MS) {
      zbAwning.requestTime();
      delay(2000);
    }
  }

  // --- OTA: interroga Z2M (prima richiesta entro ~1 minuto, poi ogni ora in automatico) ---
  zbAwning.requestOTAUpdate();
  otaWindowUntilMs = millis() + OTA_WINDOW_MS;

  // --- Sensore laser ---
  Wire.begin(PIN_SDA, PIN_SCL);
  if (cfg.sensorEnabled) {
    sensorOk = initSensor();
    lastSensorTryMs = millis();
    if (!sensorOk) Serial.println("VL53L1X non trovato! (riprovo ogni 15 s)");
    else Serial.println("VL53L1X inizializzato");
  } else {
    Serial.println("Sensore spento da switch (XSHUT LOW)");   // XSHUT e' gia' LOW da prima di Zigbee.begin()
  }

  // la prima misura batteria (BATTERY_FIRST_DELAY_MS dopo il join) e il primo stato li invia il loop
  zbAwning.publishConfig();   // allinea l'interfaccia ai valori salvati in NVS
}

// ===================== LOOP =====================
uint32_t lastBatteryMs = 0;
uint32_t bootPressStart = 0;
uint8_t  lastStatus = 255;

void loop() {
  // Passaggio a sleepy a rete stabile (mai durante un OTA): dopo il primo accoppiamento attende che finisca
  // l'interview di Z2M, nei boot normali solo SLEEPY_DELAY_MS.
  if (!DIAG_RX_ON_IDLE && !sleepyActive && zbJoined && !otaRunning &&
      millis() - joinedAtMs > (pairingMode ? PAIRING_SETTLE_MS : SLEEPY_DELAY_MS)) {
    enterSleepy();
  }
  if (cfgDirty) { cfgDirty = false; applyConfigChange(); }
  updateLed();

  // Factory reset Zigbee (tasto BOOT 3 s)
  if (digitalRead(PIN_BOOT) == LOW) {
    if (!bootPressStart) bootPressStart = millis();
    if (millis() - bootPressStart > 3000 && !otaRunning) {   // mai un factory reset a OTA in corso
      Serial.println("Factory reset Zigbee");
      setPaired(false);           // al prossimo avvio si riparte in modalita' accoppiamento
      Zigbee.factoryReset();      // riavvia
    }
  } else bootPressStart = 0;

  // --- Caricatore: a ogni cambio rilegge subito la batteria ---
  if (updateCharger()) lastBatteryMs = 0;

  // --- Gestione fascia notturna (con il caricatore collegato resta sveglio: l'alimentazione c'e') ---
  // Con OTA in corso o nella finestra OTA post-boot il deep sleep e' sospeso.
  bool otaBusy = otaRunning || (int32_t)(otaWindowUntilMs - millis()) > 0;
  if (cfg.sleepEnabled && timeValid && !chargerConnected && !otaBusy && !pairingMode) {
    time_t now = time(nullptr);
    tm lt; localtime_r(&now, &lt);
    if (inSleepWindow(lt)) {
      enterNightDeepSleep(secondsToSleepEnd(lt));   // non ritorna
    }
  }

  // --- Sensore assente: niente letture (evita errori I2C a raffica), ritenta l'inizializzazione ogni 15 s ---
  if (cfg.sensorEnabled && !sensorOk && millis() - lastSensorTryMs > 15000UL) {
    lastSensorTryMs = millis();
    sensorOk = initSensor();
    Serial.println(sensorOk ? "VL53L1X inizializzato" : "VL53L1X ancora non trovato");
  }

  // --- Lettura laser ---
  bool gotReading = false;
  if (sensorOk && distanceSensor.checkForDataReady()) {
    gotReading = true;
    float cm = distanceSensor.getDistance() / 10.0f;   // mm -> cm
    distanceSensor.clearInterrupt();

    if (fabsf(cm - lastReportedCm) >= DELTA_CM) {
      float span = (float)cfg.maxCm - (float)cfg.zeroCm;
      float pct  = span > 0 ? ((cm - cfg.zeroCm) / span) * 100.0f : 0;
      pct = constrain(pct, 0.0f, 100.0f);
      if (INVERT_PERCENT) pct = 100.0f - pct;
      uint8_t p = (uint8_t)roundf(pct);

      lastReportedCm = cm;
      zbAwning.publishDistanceMm((uint16_t)constrain(roundf(cm * 10.0f), 0.0f, 65535.0f));
      if (p != lastPercent) {
        lastPercent = p;
        zbAwning.publishOpenPercent(p);
      }
      Serial.printf("Dist %.1f cm -> %u%%\n", cm, p);
    }
  }

  // --- Batteria: ogni 10 minuti, ogni minuto con il caricatore collegato. Prima lettura 60 s dopo il join. ---
  uint32_t batteryItv = chargerConnected ? BATTERY_INTERVAL_CHARGING_MS : BATTERY_INTERVAL_MS;
  bool batteryDue = lastBatteryMs == 0 ? (millis() - joinedAtMs > BATTERY_FIRST_DELAY_MS)
                                       : (millis() - lastBatteryMs > batteryItv);
  if (batteryDue) {
    lastBatteryMs = millis();
    float v = readBatteryQuiet();
    if (chargerConnected) {
      if (v < VBAT_CHARGED) chargedSinceMs = 0;
      else if (!chargedSinceMs) chargedSinceMs = millis() | 1;
    }
    // La % si aggiorna solo a batteria, dopo BATTERY_SETTLE_MS dallo stacco; a carica completa vale 100.
    bool settled = !chargerRemovedMs || millis() - chargerRemovedMs > BATTERY_SETTLE_MS;
    uint8_t pct = currentStatus() == STATUS_CHARGED ? 100 : batteryPercent(v);
    bool pctValid = currentStatus() == STATUS_CHARGED || (!chargerConnected && settled);
    zbAwning.publishBattery(v, pct, pctValid);
    Serial.printf("Batteria %.2f V (%u%%%s)\n", v, pct, pctValid ? "" : ", non pubblicata");
  }

  // --- Stato: attivo / in carica / carica completa (deep sleep lo invia enterNightDeepSleep) ---
  uint8_t st = currentStatus();
  if (st != lastStatus) {
    lastStatus = st;
    zbAwning.publishStatus(st);
    Serial.printf("Stato %u\n", st);
  }

  // --- Resync ora periodico ---
  if (timeValid && (time(nullptr) - lastTimeSyncEpoch) > TIME_RESYNC_S) {
    timeResponseReceived = false;
    zbAwning.requestTime();
  }

  // In modalita' blink il loop deve girare piu' spesso per far lampeggiare il LED (consuma di piu')
  // Il sensore misura da solo ogni laserPeriodMs(): dopo una lettura si dorme fin quasi alla prossima, poi si fa polling stretto.
  uint16_t waitMs = (sensorOk && cfg.intervalMs > 40) ? (gotReading ? cfg.intervalMs - 30 : 10) : cfg.intervalMs;
  if (otaRunning) waitMs = 100;     // durante l'OTA il loop non deve interferire: solo lavoro leggero
  if (cfg.ledMode == LED_BLINK && waitMs > 100) waitMs = 100;
  delay(waitMs);   // con sleep abilitato la CPU puo' andare in light sleep durante il delay
}
