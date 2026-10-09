/**
 * Zigbee2MQTT external converter - esp32-zigbee-tof-awning-sensor
 *
 * Sensore ToF (VL53L1X) per misurare l'estensione della tenda da sole.
 * Firmware: arduino/esp32_zigbee_awning (ESP32-H2, End Device a batteria).
 *
 * Il cluster custom 0xFC00 deve coincidere con il firmware (ID e tipi degli attributi):
 *   sola lettura  0x0010 distanza (mm, U16)   0x0011 apertura (%, U8)   0x0012 tensione batteria (mV, U16)
 *   scrittura     0x0000 zero tenda chiusa (cm)   0x0001 lunghezza massima (cm)
 *                 0x0002 ora inizio deep sleep    0x0003 ora fine deep sleep   0x0004 intervallo lettura (ms)
 *                 0x0005 modalita LED (0 always-on, 1 blink, 2 boot-only)
 *                 0x0006 deep sleep notturno abilitato (0 = no, default; 1 = si)
 *                 0x0007 sensore laser acceso (1 = si, default; 0 = spento via XSHUT, per debug)
 *
 * Installazione: copiare il file in <dati Zigbee2MQTT>/external_converters/awning_tof.mjs
 * (stessa cartella di configuration.yaml). Con Zigbee2MQTT >= 2.11 in nuove installazioni i
 * converter esterni sono disattivati: serve `enable_external_js: true` nella sezione `advanced`.
 * Poi riavviare Zigbee2MQTT, rimuovere il vecchio dispositivo e rifare l'accoppiamento.
 */
import {Zcl} from 'zigbee-herdsman';
import * as m from 'zigbee-herdsman-converters/lib/modernExtend';

const CLUSTER = 'awningTof';
const T = Zcl.DataType;

const attr = (name, ID, type, write = false) => ({name, ID, type, write});

const awningCluster = m.deviceAddCustomCluster(CLUSTER, {
    name: CLUSTER,
    ID: 0xfc00,
    attributes: {
        closedZeroCm: attr('closedZeroCm', 0x0000, T.UINT16, true),
        maxLengthCm: attr('maxLengthCm', 0x0001, T.UINT16, true),
        sleepStartHour: attr('sleepStartHour', 0x0002, T.UINT8, true),
        sleepEndHour: attr('sleepEndHour', 0x0003, T.UINT8, true),
        readIntervalMs: attr('readIntervalMs', 0x0004, T.UINT16, true),
        ledMode: attr('ledMode', 0x0005, T.UINT8, true),
        sleepEnabled: attr('sleepEnabled', 0x0006, T.UINT8, true),
        sensorEnabled: attr('sensorEnabled', 0x0007, T.UINT8, true),
        distanceMm: attr('distanceMm', 0x0010, T.UINT16),
        openPercent: attr('openPercent', 0x0011, T.UINT8),
        batteryMv: attr('batteryMv', 0x0012, T.UINT16),
    },
    commands: {},
    commandsResponse: {},
});

export default {
    zigbeeModel: ['AwningToF'],
    model: 'AwningToF',
    vendor: 'Ivan',
    description: 'Sensore ToF per estensione tenda da sole (ESP32-H2, a batteria)',
    extend: [
        awningCluster,

        // ---- misure (il dispositivo le invia da solo: access STATE, nessun reporting da configurare) ----
        m.numeric({
            name: 'awning_open',
            label: 'Apertura tenda',
            description: 'Percentuale di apertura della tenda (0 = chiusa, 100 = tutta aperta)',
            cluster: CLUSTER,
            attribute: 'openPercent',
            unit: '%',
            valueMin: 0,
            valueMax: 100,
            access: 'STATE',
        }),
        m.numeric({
            name: 'distance',
            label: 'Distanza',
            description: 'Distanza letta dal laser',
            cluster: CLUSTER,
            attribute: 'distanceMm',
            unit: 'cm',
            scale: 10,
            precision: 1,
            access: 'STATE',
        }),

        // ---- batteria: percentuale (cluster standard) e tensione (precisa, in V) ----
        m.battery({percentage: true, voltage: false, percentageReporting: false}),
        m.numeric({
            name: 'voltage',
            label: 'Tensione batteria',
            description: 'Tensione della batteria',
            cluster: CLUSTER,
            attribute: 'batteryMv',
            unit: 'V',
            scale: 1000,
            precision: 2,
            access: 'STATE',
            entityCategory: 'diagnostic',
        }),

        // ---- parametri (scrivibili da Home Assistant, salvati in NVS dal firmware) ----
        m.numeric({
            name: 'awning_zero_cm',
            label: 'Zero tenda chiusa',
            description: 'Distanza letta dal laser a tenda completamente chiusa (taratura dello 0 %)',
            cluster: CLUSTER,
            attribute: 'closedZeroCm',
            unit: 'cm',
            valueMin: 0,
            valueMax: 400,
            valueStep: 1,
            entityCategory: 'config',
        }),
        m.numeric({
            name: 'awning_max_cm',
            label: 'Lunghezza massima tenda',
            description: 'Distanza letta dal laser a tenda completamente aperta (taratura del 100 %)',
            cluster: CLUSTER,
            attribute: 'maxLengthCm',
            unit: 'cm',
            valueMin: 1,
            valueMax: 400,
            valueStep: 1,
            entityCategory: 'config',
        }),
        m.binary({
            name: 'sensor_enabled',
            label: 'Sensore laser',
            description:
                'Debug: accende o spegne fisicamente il laser VL53L1X (pin XSHUT). Da spento non vengono inviate nuove misure di distanza. ' +
                'Il valore viene salvato e resta valido anche dopo un riavvio',
            cluster: CLUSTER,
            attribute: 'sensorEnabled',
            valueOn: ['ON', 1],
            valueOff: ['OFF', 0],
            entityCategory: 'config',
        }),
        m.binary({
            name: 'sleep_enabled',
            label: 'Deep sleep notturno',
            description:
                'Abilita il deep sleep nella fascia notturna (inizio/fine notte). Disabilitato di default. ' +
                'In deep sleep il sensore non risponde e la porta USB sparisce fino al risveglio',
            cluster: CLUSTER,
            attribute: 'sleepEnabled',
            valueOn: ['ON', 1],
            valueOff: ['OFF', 0],
            entityCategory: 'config',
        }),
        m.numeric({
            name: 'sleep_start_hour',
            label: 'Inizio notte (deep sleep)',
            description: 'Ora in cui inizia la notte e il sensore va in deep sleep (0-23). Se uguale alla fine, il deep sleep e disattivato',
            cluster: CLUSTER,
            attribute: 'sleepStartHour',
            unit: 'h',
            valueMin: 0,
            valueMax: 23,
            valueStep: 1,
            entityCategory: 'config',
        }),
        m.numeric({
            name: 'sleep_end_hour',
            label: 'Fine notte (risveglio)',
            description: 'Ora in cui finisce la notte e il sensore si risveglia (0-23)',
            cluster: CLUSTER,
            attribute: 'sleepEndHour',
            unit: 'h',
            valueMin: 0,
            valueMax: 23,
            valueStep: 1,
            entityCategory: 'config',
        }),
        m.numeric({
            name: 'read_interval',
            label: 'Intervallo di lettura',
            description: 'Pausa tra due controlli del laser durante il giorno (minimo 100 ms)',
            cluster: CLUSTER,
            attribute: 'readIntervalMs',
            unit: 'ms',
            valueMin: 100,
            valueMax: 10000,
            valueStep: 50,
            entityCategory: 'config',
        }),

        m.enumLookup({
            name: 'led_mode',
            label: 'Modalita LED',
            description:
                'LED verde dopo la connessione al controller: always-on = sempre acceso, blink = lampeggia, boot-only = acceso solo 5 s all\'avvio. ' +
                'Il LED acceso consuma batteria',
            cluster: CLUSTER,
            attribute: 'ledMode',
            lookup: {'always-on': 0, blink: 1, 'boot-only': 2},
            entityCategory: 'config',
        }),

        m.identify({isSleepy: true}),
    ],
    // linkquality viene esposta automaticamente da Zigbee2MQTT
};
