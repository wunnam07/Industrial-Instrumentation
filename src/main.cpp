#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <ModbusIP_ESP8266.h>

const uint16_t IR_TEMP1             = 0;
const uint16_t IR_TEMP2             = 1;
// Legacy address 2 is retained, but this is comparator evidence only. The
// PLC must select the authoritative process-control temperature.
const uint16_t IR_ESP32_TEMP_CANDIDATE = 2;
const uint16_t IR_SENSOR_DIFF       = 3;
const uint16_t IR_POT_RAW           = 4;
const uint16_t IR_POT_SCALED        = 5;
const uint16_t IR_DIAGNOSTIC_STATUS = 6;
// Legacy process-state address. The simulated instrument-node build always
// publishes MODBUS_INVALID_VALUE because the PLC now owns process state.
const uint16_t IR_LEGACY_PROCESS_STATE = 7;
const uint16_t IR_ESP32_HEARTBEAT   = 8;
// Three-sensor additions are append-only. Existing address locations remain
// stable; changed legacy semantics are documented above and in the interface
// contract. IR_SENSOR_DIFF remains the T1/T2 difference.
const uint16_t IR_TEMP3             = 9;
const uint16_t IR_SENSOR_DIFF_13    = 10;
const uint16_t IR_SENSOR_DIFF_23    = 11;
const uint16_t IR_VOTING_STATUS     = 12;
// Step 2 channel-health additions are append-only. Health values use
// ChannelHealthState; recovery registers count consecutive qualifying reads.
const uint16_t IR_SENSOR1_HEALTH    = 13;
const uint16_t IR_SENSOR2_HEALTH    = 14;
const uint16_t IR_SENSOR3_HEALTH    = 15;
const uint16_t IR_SENSOR1_RECOVERY  = 16;
const uint16_t IR_SENSOR2_RECOVERY  = 17;
const uint16_t IR_SENSOR3_RECOVERY  = 18;

const uint16_t DI_INSTRUMENT_NODE_READY = 0;
const uint16_t DI_SENSOR1_VALID      = 1;
const uint16_t DI_SENSOR2_VALID      = 2;
const uint16_t DI_SENSORS_AGREE      = 3;
const uint16_t DI_TEMP_CANDIDATE_VALID = 4;
// Addresses 5..7 are retained for map compatibility. They are false when
// ENABLE_PHYSICAL_HEATER_OUTPUT=0 and have no simulated-control authority.
const uint16_t DI_PHYSICAL_OUTPUT_PERMISSIVE = 5;
const uint16_t DI_PHYSICAL_HEATER_OUTPUT = 6;
const uint16_t DI_PHYSICAL_PLC_COMMS_OK = 7;
const uint16_t DI_SENSOR3_VALID      = 8;
const uint16_t DI_SENSORS_AGREE_13   = 9;
const uint16_t DI_SENSORS_AGREE_23   = 10;
const uint16_t DI_VOTING_RESOLVED    = 11;

const uint16_t COIL_PHYSICAL_HEATER_DEMAND = 0;
const uint16_t HR_PHYSICAL_PLC_HEARTBEAT = 0;
const uint16_t HR_FAULT_MODE = 1;

const uint16_t MODBUS_INVALID_VALUE = 65535;

#ifndef ENABLE_PHYSICAL_HEATER_OUTPUT
#define ENABLE_PHYSICAL_HEATER_OUTPUT 0
#endif

const int TEMP1_PIN = 18;
const int TEMP2_PIN = 19;
// Sensor 3 uses its own OneWire bus on otherwise-unused GPIO 21.
const int TEMP3_PIN = 21;
const int POT_PIN = 34;
const int LED_PIN = 23;

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

ModbusIP mb;

OneWire oneWire1(TEMP1_PIN);
DallasTemperature sensor1(&oneWire1);

OneWire oneWire2(TEMP2_PIN);
DallasTemperature sensor2(&oneWire2);

OneWire oneWire3(TEMP3_PIN);
DallasTemperature sensor3(&oneWire3);

int potValue = 0;
float scaled_ADC = 0.0;
float acquiredTemperature1 = 0.0;
float acquiredTemperature2 = 0.0;
float acquiredTemperature3 = 0.0;
float temperature1 = 0.0;
float temperature2 = 0.0;
float temperature3 = 0.0;
float filteredPotValue = 0.0;

bool filterInitialized = false;
const float FILTER_ALPHA = 0.2;

unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 1000;

const float CHANNEL_MIN_TEMPERATURE_C = 0.0;
const float CHANNEL_MAX_TEMPERATURE_C = 80.0;
// A channel becomes stale after three missed one-second refresh opportunities.
// This tolerates normal conversion scheduling jitter without treating a
// numerically constant but freshly read process as stale.
const unsigned long CHANNEL_STALE_TIMEOUT_MS =
    3 * SAMPLE_INTERVAL;
// Three consecutive healthy reads are short enough for this prototype to
// recover promptly while preventing a single good read from causing re-entry.
const uint16_t CHANNEL_RECOVERY_SAMPLES = 3;

enum ChannelHealthState : uint16_t {
    CHANNEL_HEALTHY = 0,
    CHANNEL_READ_FAILURE = 1,
    CHANNEL_OUT_OF_RANGE = 2,
    CHANNEL_STALE = 3,
    CHANNEL_RECOVERING = 4
};

struct ChannelHealth {
    bool acquisitionSucceededThisCycle = false;
    bool hasSuccessfulAcquisition = false;
    bool readHealthy = false;
    bool rangeValid = false;
    bool fresh = false;
    bool usableForVoting = false;
    bool recoveryRequired = false;
    uint16_t recoveryCount = 0;
    unsigned long lastSuccessfulAcquisitionTime = 0;
    uint32_t successfulAcquisitionSequence = 0;
    ChannelHealthState state = CHANNEL_READ_FAILURE;
};

ChannelHealth channel1Health;
ChannelHealth channel2Health;
ChannelHealth channel3Health;

// These legacy names now explicitly mirror channel usability for voting and
// remain published at the existing validity discrete-input addresses.
bool sensor1Valid = false;
bool sensor2Valid = false;
bool sensor3Valid = false;
int validSensorCount = 0;

const float SENSOR_AGREEMENT_TOLERANCE_C = 1.0;
float sensorDifference12 = 0.0;
float sensorDifference13 = 0.0;
float sensorDifference23 = 0.0;
bool sensorsAgree12 = false;
bool sensorsAgree13 = false;
bool sensorsAgree23 = false;
int agreeingPairCount = 0;
int isolatedSensor = 0;
bool allThreeSensorsAgree = false;
bool agreeingPairAvailable = false;
bool unresolvedDisagreement = false;

enum VotingStatus : uint16_t {
    VOTE_UNRESOLVED = 0,
    VOTE_ALL_THREE = 1,
    VOTE_PAIR_12 = 2,
    VOTE_PAIR_13 = 3,
    VOTE_PAIR_23 = 4,
    VOTE_SINGLE_1 = 5,
    VOTE_SINGLE_2 = 6,
    VOTE_SINGLE_3 = 7
};

VotingStatus votingStatus = VOTE_UNRESOLVED;

float diagnosticTempCandidate = 0.0;
bool diagnosticTempCandidateValid = false;

enum DiagnosticStatus {
    ALL_THREE_SENSORS_VALID = 0,
    SENSOR_1_FAULT = 1,
    SENSOR_2_FAULT = 2,
    SENSOR_DISAGREEMENT = 3,
    NO_VALID_SENSOR = 4,
    SENSOR_1_STUCK = 5,
    SENSOR_2_STUCK = 6,
    SENSOR_DISAGREEMENT_PENDING = 7,
    SENSOR_3_FAULT = 8,
    SENSOR_1_OUTLIER = 9,
    SENSOR_2_OUTLIER = 10,
    SENSOR_3_OUTLIER = 11,
    SENSOR_3_STUCK = 12,
    INSUFFICIENT_REDUNDANCY = 13
};

DiagnosticStatus diagnosticStatus = NO_VALID_SENSOR;

bool instrumentNodeReady = false;

bool temperatureConversionInProgress = false;
unsigned long temperatureRequestTime = 0;
const unsigned long TEMP_CONVERSION_TIME = 750;

uint16_t esp32Heartbeat = 0;

#if ENABLE_PHYSICAL_HEATER_OUTPUT
bool physicalPlcCommsOK = false;
bool physicalOutputPermissive = false;
bool physicalHeaterDemand = false;
bool physicalHeaterOutput = false;
uint16_t lastPhysicalPlcHeartbeat = 0;
unsigned long lastPhysicalPlcHeartbeatTime = 0;
const unsigned long PHYSICAL_PLC_COMMS_TIMEOUT = 3000;
#endif

enum FaultMode : uint16_t {
    NONE = 0,
    SENSOR1_DISCONNECTED = 1,
    SENSOR2_DISCONNECTED = 2,
    BOTH_SENSORS_DISCONNECTED = 3,
    SENSOR1_BIAS = 4,
    SENSOR2_BIAS = 5,
    SENSOR1_STUCK = 6,
    SENSOR2_STUCK = 7,
    TEMPORARY_DISAGREEMENT = 8,
    PERSISTENT_DISAGREEMENT = 9,
    COMMON_PROCESS_CHANGE = 10,
    SENSOR1_DRIFT = 11,
    SENSOR3_DISCONNECTED = 12,
    SENSOR3_BIAS = 13,
    SENSOR3_STUCK = 14,
    SENSOR3_DISCONNECTED_SENSOR1_BIAS = 15,
    THREE_WAY_DISAGREEMENT = 16,
    ALL_SENSORS_DISCONNECTED = 17,
    SENSOR1_OUT_OF_RANGE = 18,
    SENSOR2_OUT_OF_RANGE = 19,
    SENSOR3_OUT_OF_RANGE = 20,
    SENSOR1_STALE_ACQUISITION = 21,
    SENSOR2_STALE_ACQUISITION = 22,
    SENSOR3_STALE_ACQUISITION = 23,
    // Robustness-characterization profiles are append-only combinations of
    // measurement/acquisition conditions. They never assign diagnostics.
    SENSOR1_DISCONNECTED_SENSOR2_BIAS = 24,
    SENSOR1_DISCONNECTED_SENSOR2_STUCK = 25,
    SENSOR1_OUT_OF_RANGE_SENSOR3_BIAS = 26,
    SENSOR1_SENSOR2_CORRELATED_BIAS = 27,
    SENSOR2_SENSOR3_CORRELATED_BIAS = 28,
    SENSOR1_SENSOR2_CORRELATED_STUCK = 29,
    SENSOR1_SENSOR2_CORRELATED_DRIFT = 30,
    SENSOR1_HIGH_SENSOR2_LOW = 31,
    SENSOR1_SENSOR3_DISCONNECTED = 32,
    SENSOR2_SENSOR3_DISCONNECTED = 33
};

FaultMode activeFaultMode = NONE;
unsigned long faultModeSample = 0;
float frozenSensor1 = NAN;
float frozenSensor2 = NAN;
float frozenSensor3 = NAN;

const float BIAS_OFFSET_C = 2.0;
const float DISAGREEMENT_OFFSET_C = 5.0;
const float PROCESS_RAMP_STEP_C = 0.25;
const float COMMON_PROCESS_MAX_OFFSET_C = 2.0;
const float DRIFT_MAX_OFFSET_C = 5.0;
const float OUT_OF_RANGE_TEMPERATURE_C = 100.0;

bool sensor1Stuck = false;
bool sensor2Stuck = false;
bool sensor3Stuck = false;

float stuckWindowStartT1 = NAN;
float stuckWindowStartT2 = NAN;
float stuckWindowStartT3 = NAN;

int stuckWindowSamples = 0;

const int STUCK_WINDOW_SAMPLES = 5;
const float STUCK_MAX_CHANGE = 0.0625;
const float PROCESS_CHANGE_MIN = 0.50;

bool sensorDisagreementConfirmed = false;
bool disagreementPending = false;

int disagreementBadCount = 0;
int disagreementGoodCount = 0;

const int DISAGREEMENT_CONFIRM_SAMPLES = 3;
const int DISAGREEMENT_CLEAR_SAMPLES = 3;

float lastDiagnosticTempCandidate = NAN;

void connectWiFi() {
    Serial.println("Connecting to WiFi...");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 6);

    unsigned long wifiStartTime = millis();

    while (
        WiFi.status() != WL_CONNECTED &&
        millis() - wifiStartTime < 10000
    ) {
        delay(250);
        Serial.print(".");
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected");
        Serial.print("ESP32 IP address: ");
        Serial.println(WiFi.localIP());
    }
    else {
        Serial.println("WiFi connection failed");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    sensor1.begin();
    sensor2.begin();
    sensor3.begin();

    sensor1.setWaitForConversion(false);
    sensor2.setWaitForConversion(false);
    sensor3.setWaitForConversion(false);

    connectWiFi();

    mb.server();

    mb.addIreg(IR_TEMP1);
    mb.addIreg(IR_TEMP2);
    mb.addIreg(
        IR_ESP32_TEMP_CANDIDATE,
        MODBUS_INVALID_VALUE
    );
    mb.addIreg(IR_SENSOR_DIFF);
    mb.addIreg(IR_POT_RAW);
    mb.addIreg(IR_POT_SCALED);
    mb.addIreg(IR_DIAGNOSTIC_STATUS);
    mb.addIreg(
        IR_LEGACY_PROCESS_STATE,
        MODBUS_INVALID_VALUE
    );
    mb.addIreg(IR_ESP32_HEARTBEAT);
    mb.addIreg(IR_TEMP3);
    mb.addIreg(IR_SENSOR_DIFF_13);
    mb.addIreg(IR_SENSOR_DIFF_23);
    mb.addIreg(IR_VOTING_STATUS);
    mb.addIreg(IR_SENSOR1_HEALTH);
    mb.addIreg(IR_SENSOR2_HEALTH);
    mb.addIreg(IR_SENSOR3_HEALTH);
    mb.addIreg(IR_SENSOR1_RECOVERY);
    mb.addIreg(IR_SENSOR2_RECOVERY);
    mb.addIreg(IR_SENSOR3_RECOVERY);

    mb.addIsts(DI_INSTRUMENT_NODE_READY);
    mb.addIsts(DI_SENSOR1_VALID);
    mb.addIsts(DI_SENSOR2_VALID);
    mb.addIsts(DI_SENSORS_AGREE);
    mb.addIsts(DI_TEMP_CANDIDATE_VALID);
    mb.addIsts(DI_PHYSICAL_OUTPUT_PERMISSIVE);
    mb.addIsts(DI_PHYSICAL_HEATER_OUTPUT);
    mb.addIsts(DI_PHYSICAL_PLC_COMMS_OK);
    mb.addIsts(DI_SENSOR3_VALID);
    mb.addIsts(DI_SENSORS_AGREE_13);
    mb.addIsts(DI_SENSORS_AGREE_23);
    mb.addIsts(DI_VOTING_RESOLVED);

    // Retain these addresses for a later physical-output build. In the
    // current simulated build they are accepted but have no control effect.
    mb.addCoil(COIL_PHYSICAL_HEATER_DEMAND, false);
    mb.addHreg(HR_PHYSICAL_PLC_HEARTBEAT, 0);
    mb.addHreg(HR_FAULT_MODE, NONE);

    Serial.println("Three-channel DS18B20 redundant instrument node");
}

void startTemperatureConversion(unsigned long currentTime) {
    sensor1.requestTemperatures();
    sensor2.requestTemperatures();
    sensor3.requestTemperatures();

    temperatureRequestTime = currentTime;
    temperatureConversionInProgress = true;
}

bool isDallasReadFailure(float temperature) {
    return
        temperature == DEVICE_DISCONNECTED_C
#ifdef DEVICE_FAULT_OPEN_C
        || temperature == DEVICE_FAULT_OPEN_C
#endif
#ifdef DEVICE_FAULT_SHORTGND_C
        || temperature == DEVICE_FAULT_SHORTGND_C
#endif
#ifdef DEVICE_FAULT_SHORTVDD_C
        || temperature == DEVICE_FAULT_SHORTVDD_C
#endif
#ifdef DEVICE_POWER_ON_RESET_C
        || temperature == DEVICE_POWER_ON_RESET_C
#endif
#ifdef DEVICE_INSUFFICIENT_POWER_C
        || temperature == DEVICE_INSUFFICIENT_POWER_C
#endif
        ;
}

bool suppressAcquisitionForChannel(uint8_t channelNumber) {
    return
        (channelNumber == 1 &&
         activeFaultMode == SENSOR1_STALE_ACQUISITION) ||
        (channelNumber == 2 &&
         activeFaultMode == SENSOR2_STALE_ACQUISITION) ||
        (channelNumber == 3 &&
         activeFaultMode == SENSOR3_STALE_ACQUISITION);
}

void acquireTemperatureChannel(
    DallasTemperature& sensor,
    float& acquiredTemperature,
    ChannelHealth& channel,
    uint8_t channelNumber,
    unsigned long currentTime
) {
    channel.acquisitionSucceededThisCycle = false;

    // Stale injection suppresses the normal read/refresh event. It does not
    // assign channel state; freshness expires through the normal timestamp
    // comparison in updateChannelHealth().
    if (suppressAcquisitionForChannel(channelNumber)) {
        return;
    }

    float reading = sensor.getTempCByIndex(0);
    acquiredTemperature = reading;

    // getTempCByIndex() reads the scratchpad and validates its CRC. A Dallas
    // failure sentinel means no successful acquisition event occurred.
    if (!isDallasReadFailure(reading)) {
        channel.acquisitionSucceededThisCycle = true;
        channel.hasSuccessfulAcquisition = true;
        channel.lastSuccessfulAcquisitionTime = currentTime;
        channel.successfulAcquisitionSequence++;
    }
}

void finishInputRead(unsigned long currentTime) {
    acquireTemperatureChannel(
        sensor1,
        acquiredTemperature1,
        channel1Health,
        1,
        currentTime
    );
    acquireTemperatureChannel(
        sensor2,
        acquiredTemperature2,
        channel2Health,
        2,
        currentTime
    );
    acquireTemperatureChannel(
        sensor3,
        acquiredTemperature3,
        channel3Health,
        3,
        currentTime
    );

    potValue = analogRead(POT_PIN);

    temperatureConversionInProgress = false;
}

bool isFaultModeValid(uint16_t modeValue) {
    return modeValue <= SENSOR2_SENSOR3_DISCONNECTED;
}

const char* faultModeToString(FaultMode mode) {
    switch (mode) {
        case NONE:
            return "NONE";
        case SENSOR1_DISCONNECTED:
            return "SENSOR1_DISCONNECTED";
        case SENSOR2_DISCONNECTED:
            return "SENSOR2_DISCONNECTED";
        case BOTH_SENSORS_DISCONNECTED:
            return "BOTH_SENSORS_DISCONNECTED";
        case SENSOR1_BIAS:
            return "SENSOR1_BIAS";
        case SENSOR2_BIAS:
            return "SENSOR2_BIAS";
        case SENSOR1_STUCK:
            return "SENSOR1_STUCK";
        case SENSOR2_STUCK:
            return "SENSOR2_STUCK";
        case TEMPORARY_DISAGREEMENT:
            return "TEMPORARY_DISAGREEMENT";
        case PERSISTENT_DISAGREEMENT:
            return "PERSISTENT_DISAGREEMENT";
        case COMMON_PROCESS_CHANGE:
            return "COMMON_PROCESS_CHANGE";
        case SENSOR1_DRIFT:
            return "SENSOR1_DRIFT";
        case SENSOR3_DISCONNECTED:
            return "SENSOR3_DISCONNECTED";
        case SENSOR3_BIAS:
            return "SENSOR3_BIAS";
        case SENSOR3_STUCK:
            return "SENSOR3_STUCK";
        case SENSOR3_DISCONNECTED_SENSOR1_BIAS:
            return "SENSOR3_DISCONNECTED_SENSOR1_BIAS";
        case THREE_WAY_DISAGREEMENT:
            return "THREE_WAY_DISAGREEMENT";
        case ALL_SENSORS_DISCONNECTED:
            return "ALL_SENSORS_DISCONNECTED";
        case SENSOR1_OUT_OF_RANGE:
            return "SENSOR1_OUT_OF_RANGE";
        case SENSOR2_OUT_OF_RANGE:
            return "SENSOR2_OUT_OF_RANGE";
        case SENSOR3_OUT_OF_RANGE:
            return "SENSOR3_OUT_OF_RANGE";
        case SENSOR1_STALE_ACQUISITION:
            return "SENSOR1_STALE_ACQUISITION";
        case SENSOR2_STALE_ACQUISITION:
            return "SENSOR2_STALE_ACQUISITION";
        case SENSOR3_STALE_ACQUISITION:
            return "SENSOR3_STALE_ACQUISITION";
        case SENSOR1_DISCONNECTED_SENSOR2_BIAS:
            return "SENSOR1_DISCONNECTED_SENSOR2_BIAS";
        case SENSOR1_DISCONNECTED_SENSOR2_STUCK:
            return "SENSOR1_DISCONNECTED_SENSOR2_STUCK";
        case SENSOR1_OUT_OF_RANGE_SENSOR3_BIAS:
            return "SENSOR1_OUT_OF_RANGE_SENSOR3_BIAS";
        case SENSOR1_SENSOR2_CORRELATED_BIAS:
            return "SENSOR1_SENSOR2_CORRELATED_BIAS";
        case SENSOR2_SENSOR3_CORRELATED_BIAS:
            return "SENSOR2_SENSOR3_CORRELATED_BIAS";
        case SENSOR1_SENSOR2_CORRELATED_STUCK:
            return "SENSOR1_SENSOR2_CORRELATED_STUCK";
        case SENSOR1_SENSOR2_CORRELATED_DRIFT:
            return "SENSOR1_SENSOR2_CORRELATED_DRIFT";
        case SENSOR1_HIGH_SENSOR2_LOW:
            return "SENSOR1_HIGH_SENSOR2_LOW";
        case SENSOR1_SENSOR3_DISCONNECTED:
            return "SENSOR1_SENSOR3_DISCONNECTED";
        case SENSOR2_SENSOR3_DISCONNECTED:
            return "SENSOR2_SENSOR3_DISCONNECTED";
        default:
            return "INVALID";
    }
}

float boundedFaultRamp(float maximumOffset) {
    float offset =
        (faultModeSample + 1)
        * PROCESS_RAMP_STEP_C;

    if (offset > maximumOffset) {
        offset = maximumOffset;
    }

    return offset;
}

void selectFaultMode() {
    uint16_t requestedValue =
        mb.Hreg(HR_FAULT_MODE);

    if (!isFaultModeValid(requestedValue)) {
        Serial.print("Invalid fault mode ");
        Serial.print(requestedValue);
        Serial.println("; restoring NONE");

        requestedValue = NONE;
        mb.Hreg(HR_FAULT_MODE, NONE);
    }

    FaultMode requestedMode =
        static_cast<FaultMode>(requestedValue);

    if (requestedMode == activeFaultMode) {
        return;
    }

    activeFaultMode = requestedMode;
    faultModeSample = 0;
    frozenSensor1 = acquiredTemperature1;
    frozenSensor2 = acquiredTemperature2;
    frozenSensor3 = acquiredTemperature3;

    Serial.print("Fault mode changed to ");
    Serial.println(faultModeToString(activeFaultMode));
}

void applyFaultInjection() {
    // Every cycle begins with fresh acquisition. Test modes modify only
    // these effective measurements; diagnostics are never set directly.
    temperature1 = acquiredTemperature1;
    temperature2 = acquiredTemperature2;
    temperature3 = acquiredTemperature3;

    switch (activeFaultMode) {
        case NONE:
            break;

        case SENSOR1_DISCONNECTED:
            temperature1 = DEVICE_DISCONNECTED_C;
            break;

        case SENSOR2_DISCONNECTED:
            temperature2 = DEVICE_DISCONNECTED_C;
            break;

        case BOTH_SENSORS_DISCONNECTED:
            temperature1 = DEVICE_DISCONNECTED_C;
            temperature2 = DEVICE_DISCONNECTED_C;
            break;

        case SENSOR1_BIAS:
            temperature1 += BIAS_OFFSET_C;
            break;

        case SENSOR2_BIAS:
            temperature2 += BIAS_OFFSET_C;
            break;

        case SENSOR1_STUCK:
            temperature1 = frozenSensor1;
            temperature2 +=
                boundedFaultRamp(
                    COMMON_PROCESS_MAX_OFFSET_C
                );
            temperature3 +=
                boundedFaultRamp(
                    COMMON_PROCESS_MAX_OFFSET_C
                );
            break;

        case SENSOR2_STUCK:
            temperature1 +=
                boundedFaultRamp(
                    COMMON_PROCESS_MAX_OFFSET_C
                );
            temperature2 = frozenSensor2;
            temperature3 +=
                boundedFaultRamp(
                    COMMON_PROCESS_MAX_OFFSET_C
                );
            break;

        case TEMPORARY_DISAGREEMENT:
            if (faultModeSample == 0) {
                temperature1 +=
                    DISAGREEMENT_OFFSET_C;
            }
            break;

        case PERSISTENT_DISAGREEMENT:
            temperature1 +=
                DISAGREEMENT_OFFSET_C;
            break;

        case COMMON_PROCESS_CHANGE: {
            float processOffset =
                boundedFaultRamp(
                    COMMON_PROCESS_MAX_OFFSET_C
                );

            temperature1 += processOffset;
            temperature2 += processOffset;
            temperature3 += processOffset;
            break;
        }

        case SENSOR1_DRIFT:
            temperature1 +=
                boundedFaultRamp(
                    DRIFT_MAX_OFFSET_C
                );
            break;

        case SENSOR3_DISCONNECTED:
            temperature3 = DEVICE_DISCONNECTED_C;
            break;

        case SENSOR3_BIAS:
            temperature3 += BIAS_OFFSET_C;
            break;

        case SENSOR3_STUCK:
            temperature1 +=
                boundedFaultRamp(
                    COMMON_PROCESS_MAX_OFFSET_C
                );
            temperature2 +=
                boundedFaultRamp(
                    COMMON_PROCESS_MAX_OFFSET_C
                );
            temperature3 = frozenSensor3;
            break;

        case SENSOR3_DISCONNECTED_SENSOR1_BIAS:
            temperature1 += DISAGREEMENT_OFFSET_C;
            temperature3 = DEVICE_DISCONNECTED_C;
            break;

        case THREE_WAY_DISAGREEMENT:
            temperature1 += DISAGREEMENT_OFFSET_C;
            temperature3 -= DISAGREEMENT_OFFSET_C;
            break;

        case ALL_SENSORS_DISCONNECTED:
            temperature1 = DEVICE_DISCONNECTED_C;
            temperature2 = DEVICE_DISCONNECTED_C;
            temperature3 = DEVICE_DISCONNECTED_C;
            break;

        case SENSOR1_OUT_OF_RANGE:
            temperature1 = OUT_OF_RANGE_TEMPERATURE_C;
            break;

        case SENSOR2_OUT_OF_RANGE:
            temperature2 = OUT_OF_RANGE_TEMPERATURE_C;
            break;

        case SENSOR3_OUT_OF_RANGE:
            temperature3 = OUT_OF_RANGE_TEMPERATURE_C;
            break;

        case SENSOR1_STALE_ACQUISITION:
        case SENSOR2_STALE_ACQUISITION:
        case SENSOR3_STALE_ACQUISITION:
            // The corresponding acquisition was suppressed before this
            // boundary. Keeping the last value here lets freshness—not
            // numerical change—exclude the channel after the timeout.
            break;

        case SENSOR1_DISCONNECTED_SENSOR2_BIAS:
            temperature1 = DEVICE_DISCONNECTED_C;
            temperature2 += DISAGREEMENT_OFFSET_C;
            break;

        case SENSOR1_DISCONNECTED_SENSOR2_STUCK:
            temperature1 = DEVICE_DISCONNECTED_C;
            temperature2 = frozenSensor2;
            temperature3 +=
                boundedFaultRamp(
                    COMMON_PROCESS_MAX_OFFSET_C
                );
            break;

        case SENSOR1_OUT_OF_RANGE_SENSOR3_BIAS:
            temperature1 = OUT_OF_RANGE_TEMPERATURE_C;
            temperature3 += DISAGREEMENT_OFFSET_C;
            break;

        case SENSOR1_SENSOR2_CORRELATED_BIAS:
            temperature1 += DISAGREEMENT_OFFSET_C;
            temperature2 += DISAGREEMENT_OFFSET_C;
            break;

        case SENSOR2_SENSOR3_CORRELATED_BIAS:
            temperature2 += DISAGREEMENT_OFFSET_C;
            temperature3 += DISAGREEMENT_OFFSET_C;
            break;

        case SENSOR1_SENSOR2_CORRELATED_STUCK:
            temperature1 = frozenSensor1;
            temperature2 = frozenSensor2;
            temperature3 +=
                boundedFaultRamp(
                    COMMON_PROCESS_MAX_OFFSET_C
                );
            break;

        case SENSOR1_SENSOR2_CORRELATED_DRIFT: {
            float correlatedDrift =
                boundedFaultRamp(
                    DRIFT_MAX_OFFSET_C
                );

            temperature1 += correlatedDrift;
            temperature2 += correlatedDrift;
            break;
        }

        case SENSOR1_HIGH_SENSOR2_LOW:
            temperature1 += DISAGREEMENT_OFFSET_C;
            temperature2 -= DISAGREEMENT_OFFSET_C;
            break;

        case SENSOR1_SENSOR3_DISCONNECTED:
            temperature1 = DEVICE_DISCONNECTED_C;
            temperature3 = DEVICE_DISCONNECTED_C;
            break;

        case SENSOR2_SENSOR3_DISCONNECTED:
            temperature2 = DEVICE_DISCONNECTED_C;
            temperature3 = DEVICE_DISCONNECTED_C;
            break;
    }

    faultModeSample++;
}

void updateChannelHealth(
    ChannelHealth& channel,
    float effectiveTemperature,
    unsigned long currentTime
) {
    channel.readHealthy =
        !isDallasReadFailure(effectiveTemperature);
    channel.rangeValid =
        channel.readHealthy &&
        effectiveTemperature >= CHANNEL_MIN_TEMPERATURE_C &&
        effectiveTemperature <= CHANNEL_MAX_TEMPERATURE_C;
    channel.fresh =
        channel.hasSuccessfulAcquisition &&
        currentTime - channel.lastSuccessfulAcquisitionTime <=
            CHANNEL_STALE_TIMEOUT_MS;

    if (!channel.readHealthy) {
        channel.state = CHANNEL_READ_FAILURE;
        channel.usableForVoting = false;
        channel.recoveryRequired = true;
        channel.recoveryCount = 0;
        return;
    }

    if (!channel.rangeValid) {
        channel.state = CHANNEL_OUT_OF_RANGE;
        channel.usableForVoting = false;
        channel.recoveryRequired = true;
        channel.recoveryCount = 0;
        return;
    }

    if (!channel.fresh) {
        channel.state = CHANNEL_STALE;
        channel.usableForVoting = false;
        channel.recoveryRequired = true;
        channel.recoveryCount = 0;
        return;
    }

    if (channel.recoveryRequired) {
        channel.state = CHANNEL_RECOVERING;
        channel.usableForVoting = false;

        if (channel.acquisitionSucceededThisCycle) {
            channel.recoveryCount++;
        }

        if (channel.recoveryCount >= CHANNEL_RECOVERY_SAMPLES) {
            channel.recoveryCount = CHANNEL_RECOVERY_SAMPLES;
            channel.recoveryRequired = false;
            channel.usableForVoting = true;
            channel.state = CHANNEL_HEALTHY;
        }

        return;
    }

    channel.state = CHANNEL_HEALTHY;
    channel.usableForVoting = true;
    channel.recoveryCount = CHANNEL_RECOVERY_SAMPLES;
}

void runDiagnostics(unsigned long currentTime) {
    updateChannelHealth(
        channel1Health,
        temperature1,
        currentTime
    );
    updateChannelHealth(
        channel2Health,
        temperature2,
        currentTime
    );
    updateChannelHealth(
        channel3Health,
        temperature3,
        currentTime
    );

    sensor1Valid = channel1Health.usableForVoting;
    sensor2Valid = channel2Health.usableForVoting;
    sensor3Valid = channel3Health.usableForVoting;

    validSensorCount =
        (sensor1Valid ? 1 : 0) +
        (sensor2Valid ? 1 : 0) +
        (sensor3Valid ? 1 : 0);

    sensorDifference12 = sensor1Valid && sensor2Valid
        ? abs(temperature1 - temperature2)
        : 0.0;
    sensorDifference13 = sensor1Valid && sensor3Valid
        ? abs(temperature1 - temperature3)
        : 0.0;
    sensorDifference23 = sensor2Valid && sensor3Valid
        ? abs(temperature2 - temperature3)
        : 0.0;

    sensorsAgree12 =
        sensor1Valid && sensor2Valid &&
        sensorDifference12 <= SENSOR_AGREEMENT_TOLERANCE_C;
    sensorsAgree13 =
        sensor1Valid && sensor3Valid &&
        sensorDifference13 <= SENSOR_AGREEMENT_TOLERANCE_C;
    sensorsAgree23 =
        sensor2Valid && sensor3Valid &&
        sensorDifference23 <= SENSOR_AGREEMENT_TOLERANCE_C;

    agreeingPairCount =
        (sensorsAgree12 ? 1 : 0) +
        (sensorsAgree13 ? 1 : 0) +
        (sensorsAgree23 ? 1 : 0);

    allThreeSensorsAgree =
        validSensorCount == 3 &&
        agreeingPairCount == 3;

    isolatedSensor = 0;
    if (validSensorCount == 3 && agreeingPairCount == 1) {
        if (sensorsAgree23) {
            isolatedSensor = 1;
        }
        else if (sensorsAgree13) {
            isolatedSensor = 2;
        }
        else if (sensorsAgree12) {
            isolatedSensor = 3;
        }
    }

    votingStatus = VOTE_UNRESOLVED;
    if (allThreeSensorsAgree) {
        votingStatus = VOTE_ALL_THREE;
    }
    else if (validSensorCount == 3 && isolatedSensor != 0) {
        votingStatus = isolatedSensor == 1
            ? VOTE_PAIR_23
            : isolatedSensor == 2
                ? VOTE_PAIR_13
                : VOTE_PAIR_12;
    }
    else if (validSensorCount == 2) {
        if (sensorsAgree12) {
            votingStatus = VOTE_PAIR_12;
        }
        else if (sensorsAgree13) {
            votingStatus = VOTE_PAIR_13;
        }
        else if (sensorsAgree23) {
            votingStatus = VOTE_PAIR_23;
        }
    }
    else if (validSensorCount == 1) {
        votingStatus = sensor1Valid
            ? VOTE_SINGLE_1
            : sensor2Valid
                ? VOTE_SINGLE_2
                : VOTE_SINGLE_3;
    }

    agreeingPairAvailable =
        votingStatus == VOTE_PAIR_12 ||
        votingStatus == VOTE_PAIR_13 ||
        votingStatus == VOTE_PAIR_23;

    unresolvedDisagreement =
        validSensorCount >= 2 &&
        votingStatus == VOTE_UNRESOLVED;

    if (unresolvedDisagreement) {
        disagreementGoodCount = 0;

        if (!sensorDisagreementConfirmed) {
            disagreementBadCount++;
            disagreementPending = true;

            if (
                disagreementBadCount >=
                DISAGREEMENT_CONFIRM_SAMPLES
            ) {
                sensorDisagreementConfirmed = true;
                disagreementPending = false;
            }
        }
    }
    else {
        disagreementBadCount = 0;
        disagreementPending = false;

        if (sensorDisagreementConfirmed) {
            disagreementGoodCount++;

            if (
                disagreementGoodCount >=
                DISAGREEMENT_CLEAR_SAMPLES
            ) {
                sensorDisagreementConfirmed = false;
                disagreementGoodCount = 0;
            }
        }
        else {
            disagreementGoodCount = 0;
        }
    }
}

void updateDiagnosticTemperatureCandidate() {
    diagnosticTempCandidateValid = false;

    if (sensorDisagreementConfirmed) {
        diagnosticTempCandidate = NAN;
        diagnosticTempCandidateValid = false;
    }
    else if (
        unresolvedDisagreement &&
        disagreementPending &&
        !isnan(lastDiagnosticTempCandidate)
    ) {
        diagnosticTempCandidate = lastDiagnosticTempCandidate;
        diagnosticTempCandidateValid = true;
    }
    else {
        switch (votingStatus) {
            case VOTE_ALL_THREE:
                diagnosticTempCandidate =
                    (temperature1 + temperature2 + temperature3) /
                    3.0;
                diagnosticTempCandidateValid = true;
                break;
            case VOTE_PAIR_12:
                diagnosticTempCandidate =
                    (temperature1 + temperature2) / 2.0;
                diagnosticTempCandidateValid = true;
                break;
            case VOTE_PAIR_13:
                diagnosticTempCandidate =
                    (temperature1 + temperature3) / 2.0;
                diagnosticTempCandidateValid = true;
                break;
            case VOTE_PAIR_23:
                diagnosticTempCandidate =
                    (temperature2 + temperature3) / 2.0;
                diagnosticTempCandidateValid = true;
                break;
            case VOTE_SINGLE_1:
                diagnosticTempCandidate = temperature1;
                diagnosticTempCandidateValid = true;
                break;
            case VOTE_SINGLE_2:
                diagnosticTempCandidate = temperature2;
                diagnosticTempCandidateValid = true;
                break;
            case VOTE_SINGLE_3:
                diagnosticTempCandidate = temperature3;
                diagnosticTempCandidateValid = true;
                break;
            default:
                diagnosticTempCandidate = NAN;
                diagnosticTempCandidateValid = false;
                break;
        }

        if (diagnosticTempCandidateValid) {
            lastDiagnosticTempCandidate =
                diagnosticTempCandidate;
        }
    }
}

void updateDiagnosticsStatus() {
    // Priority: loss of all measurements, unresolved disagreement,
    // corroborated stuck isolation, 2-out-of-3 outlier isolation,
    // healthy/degraded explicit-validity states.
    if (validSensorCount == 0) {
        diagnosticStatus = NO_VALID_SENSOR;
    }
    else if (sensorDisagreementConfirmed) {
        diagnosticStatus = SENSOR_DISAGREEMENT;
    }
    else if (disagreementPending) {
        diagnosticStatus = SENSOR_DISAGREEMENT_PENDING;
    }
    else if (sensor1Stuck) {
        diagnosticStatus = SENSOR_1_STUCK;
    }
    else if (sensor2Stuck) {
        diagnosticStatus = SENSOR_2_STUCK;
    }
    else if (sensor3Stuck) {
        diagnosticStatus = SENSOR_3_STUCK;
    }
    else if (isolatedSensor == 1) {
        diagnosticStatus = SENSOR_1_OUTLIER;
    }
    else if (isolatedSensor == 2) {
        diagnosticStatus = SENSOR_2_OUTLIER;
    }
    else if (isolatedSensor == 3) {
        diagnosticStatus = SENSOR_3_OUTLIER;
    }
    else if (validSensorCount == 3 && allThreeSensorsAgree) {
        diagnosticStatus = ALL_THREE_SENSORS_VALID;
    }
    else if (validSensorCount == 2 && agreeingPairAvailable) {
        diagnosticStatus = !sensor1Valid
            ? SENSOR_1_FAULT
            : !sensor2Valid
                ? SENSOR_2_FAULT
                : SENSOR_3_FAULT;
    }
    else if (validSensorCount == 1) {
        diagnosticStatus = INSUFFICIENT_REDUNDANCY;
    }
    else {
        diagnosticStatus = SENSOR_DISAGREEMENT_PENDING;
    }
}

#if ENABLE_PHYSICAL_HEATER_OUTPUT
void checkPhysicalPLCHeartbeat(
    unsigned long currentTime
) {
    uint16_t currentHeartbeat =
        mb.Hreg(HR_PHYSICAL_PLC_HEARTBEAT);

    if (
        currentHeartbeat
        != lastPhysicalPlcHeartbeat
    ) {
        lastPhysicalPlcHeartbeat =
            currentHeartbeat;

        lastPhysicalPlcHeartbeatTime =
            currentTime;

        physicalPlcCommsOK = true;
    }

    if (
        physicalPlcCommsOK &&
        currentTime - lastPhysicalPlcHeartbeatTime
            > PHYSICAL_PLC_COMMS_TIMEOUT
    ) {
        physicalPlcCommsOK = false;
    }
}

void readPhysicalOutputCommand() {
    physicalHeaterDemand =
        mb.Coil(COIL_PHYSICAL_HEATER_DEMAND);
}

void updatePhysicalHeaterOutput() {
    // The PLC owns every process permissive and demand decision. This local
    // layer only rejects an expired command channel before driving hardware.
    physicalOutputPermissive = physicalPlcCommsOK;
    physicalHeaterOutput =
        physicalOutputPermissive &&
        physicalHeaterDemand;
    digitalWrite(
        LED_PIN,
        physicalHeaterOutput ? HIGH : LOW
    );
}
#endif

void filterInputs() {
    if (!filterInitialized) {
        filteredPotValue = potValue;
        filterInitialized = true;
    }
    else {
        filteredPotValue =
            FILTER_ALPHA * potValue +
            (1.0 - FILTER_ALPHA)
            * filteredPotValue;
    }
}

void ADCscaling() {
    scaled_ADC =
        (filteredPotValue / 4095.0)
        * 100.0;
}

const char* diagnosticStatusToString() {
    switch (diagnosticStatus) {
        case ALL_THREE_SENSORS_VALID:
            return "ALL_THREE_SENSORS_VALID";

        case SENSOR_1_FAULT:
            return "SENSOR_1_FAULT";

        case SENSOR_2_FAULT:
            return "SENSOR_2_FAULT";

        case SENSOR_DISAGREEMENT:
            return "SENSOR_DISAGREEMENT";

        case NO_VALID_SENSOR:
            return "NO_VALID_SENSOR";

        case SENSOR_1_STUCK:
            return "SENSOR_1_STUCK";

        case SENSOR_2_STUCK:
            return "SENSOR_2_STUCK";

        case SENSOR_DISAGREEMENT_PENDING:
            return "SENSOR_DISAGREEMENT_PENDING";

        case SENSOR_3_FAULT:
            return "SENSOR_3_FAULT";

        case SENSOR_1_OUTLIER:
            return "SENSOR_1_OUTLIER";

        case SENSOR_2_OUTLIER:
            return "SENSOR_2_OUTLIER";

        case SENSOR_3_OUTLIER:
            return "SENSOR_3_OUTLIER";

        case SENSOR_3_STUCK:
            return "SENSOR_3_STUCK";

        case INSUFFICIENT_REDUNDANCY:
            return "INSUFFICIENT_REDUNDANCY";

        default:
            return "UNKNOWN";
    }
}

void updateModbusInputRegisters() {
    uint16_t temp1Modbus = sensor1Valid
        ? (uint16_t)round(temperature1 * 100.0)
        : MODBUS_INVALID_VALUE;

    uint16_t temp2Modbus = sensor2Valid
        ? (uint16_t)round(temperature2 * 100.0)
        : MODBUS_INVALID_VALUE;

    uint16_t temp3Modbus = sensor3Valid
        ? (uint16_t)round(temperature3 * 100.0)
        : MODBUS_INVALID_VALUE;

    uint16_t tempCandidateModbus;
    if (diagnosticTempCandidateValid) {
        tempCandidateModbus =
            (uint16_t)
            round(diagnosticTempCandidate * 100.0);
    }
    else {
        tempCandidateModbus = MODBUS_INVALID_VALUE;
    }

    uint16_t sensorDiff12Modbus = sensor1Valid && sensor2Valid
        ? (uint16_t)round(sensorDifference12 * 100.0)
        : MODBUS_INVALID_VALUE;

    uint16_t sensorDiff13Modbus = sensor1Valid && sensor3Valid
        ? (uint16_t)round(sensorDifference13 * 100.0)
        : MODBUS_INVALID_VALUE;

    uint16_t sensorDiff23Modbus = sensor2Valid && sensor3Valid
        ? (uint16_t)round(sensorDifference23 * 100.0)
        : MODBUS_INVALID_VALUE;

    uint16_t potScaledModbus =
        (uint16_t)
        round(scaled_ADC * 10.0);

    esp32Heartbeat++;

    mb.Ireg(
        IR_TEMP1,
        temp1Modbus
    );

    mb.Ireg(
        IR_TEMP2,
        temp2Modbus
    );

    mb.Ireg(
        IR_ESP32_TEMP_CANDIDATE,
        tempCandidateModbus
    );

    mb.Ireg(
        IR_SENSOR_DIFF,
        sensorDiff12Modbus
    );

    mb.Ireg(
        IR_POT_RAW,
        potValue
    );

    mb.Ireg(
        IR_POT_SCALED,
        potScaledModbus
    );

    mb.Ireg(
        IR_DIAGNOSTIC_STATUS,
        (uint16_t)diagnosticStatus
    );

    mb.Ireg(
        IR_LEGACY_PROCESS_STATE,
        MODBUS_INVALID_VALUE
    );

    mb.Ireg(
        IR_ESP32_HEARTBEAT,
        esp32Heartbeat
    );

    mb.Ireg(
        IR_TEMP3,
        temp3Modbus
    );

    mb.Ireg(
        IR_SENSOR_DIFF_13,
        sensorDiff13Modbus
    );

    mb.Ireg(
        IR_SENSOR_DIFF_23,
        sensorDiff23Modbus
    );

    mb.Ireg(
        IR_VOTING_STATUS,
        (uint16_t)votingStatus
    );

    mb.Ireg(
        IR_SENSOR1_HEALTH,
        (uint16_t)channel1Health.state
    );

    mb.Ireg(
        IR_SENSOR2_HEALTH,
        (uint16_t)channel2Health.state
    );

    mb.Ireg(
        IR_SENSOR3_HEALTH,
        (uint16_t)channel3Health.state
    );

    mb.Ireg(
        IR_SENSOR1_RECOVERY,
        channel1Health.recoveryCount
    );

    mb.Ireg(
        IR_SENSOR2_RECOVERY,
        channel2Health.recoveryCount
    );

    mb.Ireg(
        IR_SENSOR3_RECOVERY,
        channel3Health.recoveryCount
    );
}

void updateModbusDiscreteInputs() {
    mb.Ists(
        DI_INSTRUMENT_NODE_READY,
        instrumentNodeReady
    );

    mb.Ists(
        DI_SENSOR1_VALID,
        sensor1Valid
    );

    mb.Ists(
        DI_SENSOR2_VALID,
        sensor2Valid
    );

    mb.Ists(
        DI_SENSORS_AGREE,
        sensorsAgree12
    );

    mb.Ists(
        DI_TEMP_CANDIDATE_VALID,
        diagnosticTempCandidateValid
    );

#if ENABLE_PHYSICAL_HEATER_OUTPUT
    mb.Ists(
        DI_PHYSICAL_OUTPUT_PERMISSIVE,
        physicalOutputPermissive
    );

    mb.Ists(
        DI_PHYSICAL_HEATER_OUTPUT,
        physicalHeaterOutput
    );

    mb.Ists(
        DI_PHYSICAL_PLC_COMMS_OK,
        physicalPlcCommsOK
    );
#else
    mb.Ists(DI_PHYSICAL_OUTPUT_PERMISSIVE, false);
    mb.Ists(DI_PHYSICAL_HEATER_OUTPUT, false);
    mb.Ists(DI_PHYSICAL_PLC_COMMS_OK, false);
#endif

    mb.Ists(
        DI_SENSOR3_VALID,
        sensor3Valid
    );

    mb.Ists(
        DI_SENSORS_AGREE_13,
        sensorsAgree13
    );

    mb.Ists(
        DI_SENSORS_AGREE_23,
        sensorsAgree23
    );

    mb.Ists(
        DI_VOTING_RESOLVED,
        votingStatus != VOTE_UNRESOLVED
    );
}

void detectStuckSensors() {

    if (!sensor1Valid || !sensor2Valid || !sensor3Valid) {

        sensor1Stuck = false;
        sensor2Stuck = false;
        sensor3Stuck = false;

        stuckWindowStartT1 = NAN;
        stuckWindowStartT2 = NAN;
        stuckWindowStartT3 = NAN;

        stuckWindowSamples = 0;

        return;
    }

    if (
        isnan(stuckWindowStartT1) ||
        isnan(stuckWindowStartT2) ||
        isnan(stuckWindowStartT3)
    ) {

        stuckWindowStartT1 = temperature1;
        stuckWindowStartT2 = temperature2;
        stuckWindowStartT3 = temperature3;

        stuckWindowSamples = 0;

        return;
    }

    stuckWindowSamples++;

    if (stuckWindowSamples < STUCK_WINDOW_SAMPLES) {
        return;
    }

    float sensor1Delta = temperature1 - stuckWindowStartT1;
    float sensor2Delta = temperature2 - stuckWindowStartT2;
    float sensor3Delta = temperature3 - stuckWindowStartT3;

    float sensor1Change = abs(sensor1Delta);
    float sensor2Change = abs(sensor2Delta);
    float sensor3Change = abs(sensor3Delta);

    bool sensors23MoveTogether =
        sensorsAgree23 &&
        sensor2Change >= PROCESS_CHANGE_MIN &&
        sensor3Change >= PROCESS_CHANGE_MIN &&
        abs(sensor2Delta - sensor3Delta) <=
            SENSOR_AGREEMENT_TOLERANCE_C;

    bool sensors13MoveTogether =
        sensorsAgree13 &&
        sensor1Change >= PROCESS_CHANGE_MIN &&
        sensor3Change >= PROCESS_CHANGE_MIN &&
        abs(sensor1Delta - sensor3Delta) <=
            SENSOR_AGREEMENT_TOLERANCE_C;

    bool sensors12MoveTogether =
        sensorsAgree12 &&
        sensor1Change >= PROCESS_CHANGE_MIN &&
        sensor2Change >= PROCESS_CHANGE_MIN &&
        abs(sensor1Delta - sensor2Delta) <=
            SENSOR_AGREEMENT_TOLERANCE_C;

    sensor1Stuck =
        sensor1Change <= STUCK_MAX_CHANGE &&
        sensors23MoveTogether;

    sensor2Stuck =
        sensor2Change <= STUCK_MAX_CHANGE &&
        sensors13MoveTogether;

    sensor3Stuck =
        sensor3Change <= STUCK_MAX_CHANGE &&
        sensors12MoveTogether;

    stuckWindowStartT1 = temperature1;
    stuckWindowStartT2 = temperature2;
    stuckWindowStartT3 = temperature3;

    stuckWindowSamples = 0;
}

void logMeasurements(unsigned long timestamp) {
    Serial.print("Fault mode: ");
    Serial.println(faultModeToString(activeFaultMode));

    if (activeFaultMode != NONE) {
        Serial.print("Acquired temperatures: ");
        Serial.print(acquiredTemperature1);
        Serial.print(" C, ");
        Serial.print(acquiredTemperature2);
        Serial.print(" C, ");
        Serial.print(acquiredTemperature3);
        Serial.println(" C");
    }

    Serial.print("[");
    Serial.print(timestamp);
    Serial.print(" ms] Temperature 1: ");
    Serial.print(temperature1);
    Serial.println(" C");

    Serial.print("[");
    Serial.print(timestamp);
    Serial.print(" ms] Temperature 2: ");
    Serial.print(temperature2);
    Serial.println(" C");

    Serial.print("[");
    Serial.print(timestamp);
    Serial.print(" ms] Temperature 3: ");
    Serial.print(temperature3);
    Serial.println(" C");

    Serial.print("[");
    Serial.print(timestamp);
    Serial.print(" ms] Potentiometer RAW: ");
    Serial.println(potValue);

    Serial.print("[");
    Serial.print(timestamp);
    Serial.print(" ms] Potentiometer FILTERED: ");
    Serial.println(filteredPotValue);

    Serial.print("[");
    Serial.print(timestamp);
    Serial.print(" ms] Potentiometer SCALED: ");
    Serial.print(scaled_ADC);
    Serial.println(" %");

    Serial.print("instrumentNodeReady: ");
    Serial.println(instrumentNodeReady);

    Serial.print("sensor1Valid: ");
    Serial.println(sensor1Valid);

    Serial.print("sensor2Valid: ");
    Serial.println(sensor2Valid);

    Serial.print("sensor3Valid: ");
    Serial.println(sensor3Valid);

    Serial.print("channel health S1/S2/S3: ");
    Serial.print((uint16_t)channel1Health.state);
    Serial.print("/");
    Serial.print((uint16_t)channel2Health.state);
    Serial.print("/");
    Serial.println((uint16_t)channel3Health.state);

    Serial.print("recovery count S1/S2/S3: ");
    Serial.print(channel1Health.recoveryCount);
    Serial.print("/");
    Serial.print(channel2Health.recoveryCount);
    Serial.print("/");
    Serial.println(channel3Health.recoveryCount);

    Serial.print("agree12/agree13/agree23: ");
    Serial.print(sensorsAgree12);
    Serial.print("/");
    Serial.print(sensorsAgree13);
    Serial.print("/");
    Serial.println(sensorsAgree23);

    Serial.print("differences d12/d13/d23: ");
    Serial.print(sensorDifference12);
    Serial.print("/");
    Serial.print(sensorDifference13);
    Serial.print("/");
    Serial.println(sensorDifference23);

    Serial.print("diagnosticStatus: ");
    Serial.println(diagnosticStatusToString());

    Serial.print("sensor1Stuck: ");
    Serial.println(sensor1Stuck);

    Serial.print("sensor2Stuck: ");
    Serial.println(sensor2Stuck);

    Serial.print("sensor3Stuck: ");
    Serial.println(sensor3Stuck);

    Serial.print("votingStatus: ");
    Serial.println((uint16_t)votingStatus);

    Serial.print("disagreementPending: ");
    Serial.println(disagreementPending);

    Serial.print("disagreementBadCount: ");
    Serial.println(disagreementBadCount);

    Serial.print("disagreementGoodCount: ");
    Serial.println(disagreementGoodCount);

    Serial.print("sensorDisagreementConfirmed: ");
    Serial.println(sensorDisagreementConfirmed);

    Serial.print("diagnosticTempCandidateValid: ");
    Serial.println(diagnosticTempCandidateValid);

    Serial.print("diagnosticTempCandidate: ");
    Serial.println(diagnosticTempCandidate);

    Serial.print("ESP32 heartbeat: ");
    Serial.println(esp32Heartbeat);

    Serial.println("--------------------");
}

void loop() {
    mb.task();

    unsigned long currentTime = millis();

#if ENABLE_PHYSICAL_HEATER_OUTPUT
    readPhysicalOutputCommand();
    checkPhysicalPLCHeartbeat(currentTime);
    updatePhysicalHeaterOutput();
#else
    // The active Wokwi/PLCSIM build never drives a simulated plant output.
    digitalWrite(LED_PIN, LOW);
#endif

    updateModbusDiscreteInputs();

    if (
        !temperatureConversionInProgress &&
        currentTime - lastSampleTime
            >= SAMPLE_INTERVAL
    ) {
        lastSampleTime = currentTime;

        startTemperatureConversion(
            currentTime
        );
    }

    if (
        temperatureConversionInProgress &&
        currentTime - temperatureRequestTime
            >= TEMP_CONVERSION_TIME
    ) {
        // Select before acquisition so stale test modes can suppress the
        // normal channel refresh without assigning diagnostic state.
        selectFaultMode();

        finishInputRead(currentTime);

        applyFaultInjection();

        filterInputs();

        ADCscaling();

        runDiagnostics(currentTime);

        detectStuckSensors();

        updateDiagnosticsStatus();

        updateDiagnosticTemperatureCandidate();
        instrumentNodeReady = true;

#if ENABLE_PHYSICAL_HEATER_OUTPUT
        updatePhysicalHeaterOutput();
#endif

        updateModbusInputRegisters();

        updateModbusDiscreteInputs();

        logMeasurements(
            currentTime
        );
    }
}
