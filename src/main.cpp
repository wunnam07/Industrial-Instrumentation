#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <ModbusIP_ESP8266.h>

// Virtual Modbus Interface v2.
// The ESP32 publishes only measurement and acquisition-side diagnostic evidence.
// The PLC owns agreement/voting, trusted temperature, process state, trips,
// permissives, heater demand, and the final simulated HEATER_OUTPUT.
const uint16_t IR_TEMP1 = 0;
const uint16_t IR_TEMP2 = 1;
const uint16_t IR_TEMP3 = 2;
const uint16_t IR_ESP32_HEARTBEAT = 3;
const uint16_t IR_SENSOR1_HEALTH = 4;
const uint16_t IR_SENSOR2_HEALTH = 5;
const uint16_t IR_SENSOR3_HEALTH = 6;

// Test/commissioning-only interface. The PLC control program does not depend on it.
const uint16_t HR_FAULT_MODE = 0;

const uint16_t MODBUS_INVALID_VALUE = 65535;

const int TEMP1_PIN = 18;
const int TEMP2_PIN = 19;
const int TEMP3_PIN = 21;

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

ModbusIP mb;

OneWire oneWire1(TEMP1_PIN);
OneWire oneWire2(TEMP2_PIN);
OneWire oneWire3(TEMP3_PIN);

DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);
DallasTemperature sensor3(&oneWire3);

const unsigned long SAMPLE_INTERVAL_MS = 1000;
const unsigned long TEMP_CONVERSION_TIME_MS = 750;
const unsigned long CHANNEL_STALE_TIMEOUT_MS = 3000;

const float CHANNEL_MIN_TEMPERATURE_C = 0.0;
const float CHANNEL_MAX_TEMPERATURE_C = 80.0;
const uint16_t CHANNEL_QUALIFICATION_SAMPLES = 3;
const uint16_t CHANNEL_RECOVERY_SAMPLES = 3;

// Stuck detection is intentionally acquisition-side diagnostic evidence only.
// It does not select a process-control temperature or perform PLC voting.
const int STUCK_WINDOW_SAMPLES = 5;
const float STUCK_MAX_CHANGE_C = 0.0625;
const float PROCESS_CHANGE_MIN_C = 0.50;
const float STUCK_CORROBORATION_TOLERANCE_C = 1.0;
const uint16_t STUCK_CLEAR_SAMPLES = 3;

enum ChannelHealthState : uint16_t {
    CHANNEL_HEALTHY = 0,
    CHANNEL_READ_FAILURE = 1,
    CHANNEL_OUT_OF_RANGE = 2,
    CHANNEL_STALE = 3,
    CHANNEL_STUCK = 4,
    CHANNEL_RECOVERING = 5,
    CHANNEL_NOT_READY = 6
};

struct ChannelHealth {
    bool acquisitionSucceededThisCycle = false;
    bool hasSuccessfulAcquisition = false;
    bool readHealthy = false;
    bool rangeValid = false;
    bool fresh = false;
    bool everQualified = false;
    bool recoveryRequired = false;
    bool stuckLatched = false;

    uint16_t qualificationCount = 0;
    uint16_t stuckClearCount = 0;

    unsigned long lastSuccessfulAcquisitionTime = 0;

    float stuckReferenceTemperature = NAN;

    ChannelHealthState state = CHANNEL_NOT_READY;
};

ChannelHealth channel1Health;
ChannelHealth channel2Health;
ChannelHealth channel3Health;

float acquiredTemperature1 = 0.0;
float acquiredTemperature2 = 0.0;
float acquiredTemperature3 = 0.0;

float temperature1 = 0.0;
float temperature2 = 0.0;
float temperature3 = 0.0;

unsigned long lastSampleTime = 0;
unsigned long temperatureRequestTime = 0;
bool temperatureConversionInProgress = false;
uint16_t esp32Heartbeat = 0;

float stuckWindowStartT1 = NAN;
float stuckWindowStartT2 = NAN;
float stuckWindowStartT3 = NAN;
int stuckWindowSampleCount = 0;

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

void connectWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 6);

    unsigned long start = millis();
    while (
        WiFi.status() != WL_CONNECTED &&
        millis() - start < 10000
    ) {
        delay(250);
        Serial.print(".");
    }

    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("WiFi connected. ESP32 IP: ");
        Serial.println(WiFi.localIP());
    }
    else {
        Serial.println("WiFi connection failed");
    }
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

bool isFaultModeValid(uint16_t value) {
    return value <= SENSOR2_SENSOR3_DISCONNECTED;
}

bool suppressAcquisitionForChannel(uint8_t channelNumber) {
    return
        (channelNumber == 1 && activeFaultMode == SENSOR1_STALE_ACQUISITION) ||
        (channelNumber == 2 && activeFaultMode == SENSOR2_STALE_ACQUISITION) ||
        (channelNumber == 3 && activeFaultMode == SENSOR3_STALE_ACQUISITION);
}

float boundedFaultRamp(float maximumOffset) {
    float offset = (faultModeSample + 1) * PROCESS_RAMP_STEP_C;
    if (offset > maximumOffset) {
        offset = maximumOffset;
    }
    return offset;
}

void selectFaultMode() {
    uint16_t requested = mb.Hreg(HR_FAULT_MODE);

    if (!isFaultModeValid(requested)) {
        requested = NONE;
        mb.Hreg(HR_FAULT_MODE, NONE);
    }

    FaultMode requestedMode = static_cast<FaultMode>(requested);
    if (requestedMode == activeFaultMode) {
        return;
    }

    activeFaultMode = requestedMode;
    faultModeSample = 0;
    frozenSensor1 = acquiredTemperature1;
    frozenSensor2 = acquiredTemperature2;
    frozenSensor3 = acquiredTemperature3;

    Serial.print("Fault mode changed to ");
    Serial.println((uint16_t)activeFaultMode);
}

void startTemperatureConversion(unsigned long currentTime) {
    sensor1.requestTemperatures();
    sensor2.requestTemperatures();
    sensor3.requestTemperatures();

    temperatureRequestTime = currentTime;
    temperatureConversionInProgress = true;
}

void acquireTemperatureChannel(
    DallasTemperature& sensor,
    float& acquiredTemperature,
    ChannelHealth& channel,
    uint8_t channelNumber,
    unsigned long currentTime
) {
    channel.acquisitionSucceededThisCycle = false;

    if (suppressAcquisitionForChannel(channelNumber)) {
        return;
    }

    float reading = sensor.getTempCByIndex(0);
    acquiredTemperature = reading;

    if (!isDallasReadFailure(reading)) {
        channel.acquisitionSucceededThisCycle = true;
        channel.hasSuccessfulAcquisition = true;
        channel.lastSuccessfulAcquisitionTime = currentTime;
    }
}

void finishTemperatureRead(unsigned long currentTime) {
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

    temperatureConversionInProgress = false;
}

void applyFaultInjection() {
    // Test modes modify only the effective measurement stream.
    // They never directly force a published health state.
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
            temperature2 += boundedFaultRamp(COMMON_PROCESS_MAX_OFFSET_C);
            temperature3 += boundedFaultRamp(COMMON_PROCESS_MAX_OFFSET_C);
            break;

        case SENSOR2_STUCK:
            temperature1 += boundedFaultRamp(COMMON_PROCESS_MAX_OFFSET_C);
            temperature2 = frozenSensor2;
            temperature3 += boundedFaultRamp(COMMON_PROCESS_MAX_OFFSET_C);
            break;

        case TEMPORARY_DISAGREEMENT:
            if (faultModeSample == 0) {
                temperature1 += DISAGREEMENT_OFFSET_C;
            }
            break;

        case PERSISTENT_DISAGREEMENT:
            temperature1 += DISAGREEMENT_OFFSET_C;
            break;

        case COMMON_PROCESS_CHANGE: {
            float offset = boundedFaultRamp(COMMON_PROCESS_MAX_OFFSET_C);
            temperature1 += offset;
            temperature2 += offset;
            temperature3 += offset;
            break;
        }

        case SENSOR1_DRIFT:
            temperature1 += boundedFaultRamp(DRIFT_MAX_OFFSET_C);
            break;

        case SENSOR3_DISCONNECTED:
            temperature3 = DEVICE_DISCONNECTED_C;
            break;

        case SENSOR3_BIAS:
            temperature3 += BIAS_OFFSET_C;
            break;

        case SENSOR3_STUCK:
            temperature1 += boundedFaultRamp(COMMON_PROCESS_MAX_OFFSET_C);
            temperature2 += boundedFaultRamp(COMMON_PROCESS_MAX_OFFSET_C);
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
            // Acquisition suppression already happened before this point.
            break;

        case SENSOR1_DISCONNECTED_SENSOR2_BIAS:
            temperature1 = DEVICE_DISCONNECTED_C;
            temperature2 += DISAGREEMENT_OFFSET_C;
            break;

        case SENSOR1_DISCONNECTED_SENSOR2_STUCK:
            temperature1 = DEVICE_DISCONNECTED_C;
            temperature2 = frozenSensor2;
            temperature3 += boundedFaultRamp(COMMON_PROCESS_MAX_OFFSET_C);
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
            temperature3 += boundedFaultRamp(COMMON_PROCESS_MAX_OFFSET_C);
            break;

        case SENSOR1_SENSOR2_CORRELATED_DRIFT: {
            float drift = boundedFaultRamp(DRIFT_MAX_OFFSET_C);
            temperature1 += drift;
            temperature2 += drift;
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

void setPrimaryHealthFault(
    ChannelHealth& channel,
    ChannelHealthState state
) {
    channel.state = state;
    channel.qualificationCount = 0;
    channel.stuckClearCount = 0;

    if (channel.everQualified) {
        channel.recoveryRequired = true;
    }
}

void updateChannelHealth(
    ChannelHealth& channel,
    float effectiveTemperature,
    unsigned long currentTime
) {
    channel.readHealthy = !isDallasReadFailure(effectiveTemperature);
    channel.rangeValid =
        channel.readHealthy &&
        effectiveTemperature >= CHANNEL_MIN_TEMPERATURE_C &&
        effectiveTemperature <= CHANNEL_MAX_TEMPERATURE_C;
    channel.fresh =
        channel.hasSuccessfulAcquisition &&
        currentTime - channel.lastSuccessfulAcquisitionTime <=
            CHANNEL_STALE_TIMEOUT_MS;

    // Deterministic dominant priority:
    // READ_FAILURE > OUT_OF_RANGE > STALE > STUCK > RECOVERING/NOT_READY > HEALTHY.
    if (!channel.readHealthy) {
        setPrimaryHealthFault(channel, CHANNEL_READ_FAILURE);
        return;
    }

    if (!channel.rangeValid) {
        setPrimaryHealthFault(channel, CHANNEL_OUT_OF_RANGE);
        return;
    }

    if (!channel.fresh) {
        setPrimaryHealthFault(channel, CHANNEL_STALE);
        return;
    }

    if (channel.stuckLatched) {
        channel.state = CHANNEL_STUCK;
        channel.qualificationCount = 0;
        return;
    }

    if (!channel.everQualified) {
        channel.state = CHANNEL_NOT_READY;

        if (channel.acquisitionSucceededThisCycle) {
            channel.qualificationCount++;
        }

        if (channel.qualificationCount >= CHANNEL_QUALIFICATION_SAMPLES) {
            channel.qualificationCount = CHANNEL_QUALIFICATION_SAMPLES;
            channel.everQualified = true;
            channel.recoveryRequired = false;
            channel.state = CHANNEL_HEALTHY;
        }

        return;
    }

    if (channel.recoveryRequired) {
        channel.state = CHANNEL_RECOVERING;

        if (channel.acquisitionSucceededThisCycle) {
            channel.qualificationCount++;
        }

        if (channel.qualificationCount >= CHANNEL_RECOVERY_SAMPLES) {
            channel.qualificationCount = CHANNEL_RECOVERY_SAMPLES;
            channel.recoveryRequired = false;
            channel.state = CHANNEL_HEALTHY;
        }

        return;
    }

    channel.state = CHANNEL_HEALTHY;
    channel.qualificationCount = CHANNEL_RECOVERY_SAMPLES;
}

bool baseEvidenceHealthy(const ChannelHealth& channel) {
    return channel.readHealthy && channel.rangeValid && channel.fresh;
}

bool allThreeTemperaturesAgree() {
    return
        fabsf(temperature1 - temperature2) <= STUCK_CORROBORATION_TOLERANCE_C &&
        fabsf(temperature1 - temperature3) <= STUCK_CORROBORATION_TOLERANCE_C &&
        fabsf(temperature2 - temperature3) <= STUCK_CORROBORATION_TOLERANCE_C;
}

void evaluateOneStuckRecovery(ChannelHealth& channel) {
    if (!channel.stuckLatched) {
        channel.stuckClearCount = 0;
        return;
    }

    if (
        baseEvidenceHealthy(channel1Health) &&
        baseEvidenceHealthy(channel2Health) &&
        baseEvidenceHealthy(channel3Health) &&
        allThreeTemperaturesAgree()
    ) {
        channel.stuckClearCount++;

        if (channel.stuckClearCount >= STUCK_CLEAR_SAMPLES) {
            channel.stuckLatched = false;
            channel.stuckReferenceTemperature = NAN;
            channel.stuckClearCount = 0;
            channel.recoveryRequired = true;
            channel.qualificationCount = 0;
            channel.state = CHANNEL_RECOVERING;
        }
    }
    else {
        channel.stuckClearCount = 0;
    }
}

void evaluateStuckRecovery() {
    evaluateOneStuckRecovery(channel1Health);
    evaluateOneStuckRecovery(channel2Health);
    evaluateOneStuckRecovery(channel3Health);
}

void latchStuck(ChannelHealth& channel, float temperature) {
    channel.stuckLatched = true;
    channel.stuckReferenceTemperature = temperature;
    channel.stuckClearCount = 0;
    channel.qualificationCount = 0;
    channel.recoveryRequired = true;
    channel.state = CHANNEL_STUCK;
}

void resetStuckWindow() {
    stuckWindowStartT1 = NAN;
    stuckWindowStartT2 = NAN;
    stuckWindowStartT3 = NAN;
    stuckWindowSampleCount = 0;
}

void detectStuckSensors() {
    // Only diagnose stuck behavior while all three channels are otherwise
    // healthy. The corroborating pair is used only as acquisition evidence;
    // no pair/vote result is exported to Modbus.
    if (
        channel1Health.state != CHANNEL_HEALTHY ||
        channel2Health.state != CHANNEL_HEALTHY ||
        channel3Health.state != CHANNEL_HEALTHY
    ) {
        resetStuckWindow();
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
        stuckWindowSampleCount = 0;
        return;
    }

    stuckWindowSampleCount++;
    if (stuckWindowSampleCount < STUCK_WINDOW_SAMPLES) {
        return;
    }

    float delta1 = temperature1 - stuckWindowStartT1;
    float delta2 = temperature2 - stuckWindowStartT2;
    float delta3 = temperature3 - stuckWindowStartT3;

    float change1 = fabsf(delta1);
    float change2 = fabsf(delta2);
    float change3 = fabsf(delta3);

    bool pair23CorroboratesMotion =
        fabsf(temperature2 - temperature3) <= STUCK_CORROBORATION_TOLERANCE_C &&
        change2 >= PROCESS_CHANGE_MIN_C &&
        change3 >= PROCESS_CHANGE_MIN_C &&
        fabsf(delta2 - delta3) <= STUCK_CORROBORATION_TOLERANCE_C;

    bool pair13CorroboratesMotion =
        fabsf(temperature1 - temperature3) <= STUCK_CORROBORATION_TOLERANCE_C &&
        change1 >= PROCESS_CHANGE_MIN_C &&
        change3 >= PROCESS_CHANGE_MIN_C &&
        fabsf(delta1 - delta3) <= STUCK_CORROBORATION_TOLERANCE_C;

    bool pair12CorroboratesMotion =
        fabsf(temperature1 - temperature2) <= STUCK_CORROBORATION_TOLERANCE_C &&
        change1 >= PROCESS_CHANGE_MIN_C &&
        change2 >= PROCESS_CHANGE_MIN_C &&
        fabsf(delta1 - delta2) <= STUCK_CORROBORATION_TOLERANCE_C;

    if (change1 <= STUCK_MAX_CHANGE_C && pair23CorroboratesMotion) {
        latchStuck(channel1Health, temperature1);
    }

    if (change2 <= STUCK_MAX_CHANGE_C && pair13CorroboratesMotion) {
        latchStuck(channel2Health, temperature2);
    }

    if (change3 <= STUCK_MAX_CHANGE_C && pair12CorroboratesMotion) {
        latchStuck(channel3Health, temperature3);
    }

    stuckWindowStartT1 = temperature1;
    stuckWindowStartT2 = temperature2;
    stuckWindowStartT3 = temperature3;
    stuckWindowSampleCount = 0;
}

uint16_t encodeTemperature(
    float temperature,
    const ChannelHealth& channel
) {
    if (channel.state != CHANNEL_HEALTHY) {
        return MODBUS_INVALID_VALUE;
    }

    return (uint16_t)roundf(temperature * 100.0f);
}

void publishModbusData() {
    esp32Heartbeat++;

    mb.Ireg(IR_TEMP1, encodeTemperature(temperature1, channel1Health));
    mb.Ireg(IR_TEMP2, encodeTemperature(temperature2, channel2Health));
    mb.Ireg(IR_TEMP3, encodeTemperature(temperature3, channel3Health));
    mb.Ireg(IR_ESP32_HEARTBEAT, esp32Heartbeat);
    mb.Ireg(IR_SENSOR1_HEALTH, (uint16_t)channel1Health.state);
    mb.Ireg(IR_SENSOR2_HEALTH, (uint16_t)channel2Health.state);
    mb.Ireg(IR_SENSOR3_HEALTH, (uint16_t)channel3Health.state);
}

void logCycle(unsigned long currentTime) {
    Serial.print("[");
    Serial.print(currentTime);
    Serial.print(" ms] fault=");
    Serial.print((uint16_t)activeFaultMode);

    Serial.print(" T1/T2/T3=");
    Serial.print(temperature1);
    Serial.print("/");
    Serial.print(temperature2);
    Serial.print("/");
    Serial.print(temperature3);

    Serial.print(" health=");
    Serial.print((uint16_t)channel1Health.state);
    Serial.print("/");
    Serial.print((uint16_t)channel2Health.state);
    Serial.print("/");
    Serial.print((uint16_t)channel3Health.state);

    Serial.print(" heartbeat=");
    Serial.println(esp32Heartbeat);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    sensor1.begin();
    sensor2.begin();
    sensor3.begin();

    sensor1.setWaitForConversion(false);
    sensor2.setWaitForConversion(false);
    sensor3.setWaitForConversion(false);

    connectWiFi();

    mb.server();

    mb.addIreg(IR_TEMP1, MODBUS_INVALID_VALUE);
    mb.addIreg(IR_TEMP2, MODBUS_INVALID_VALUE);
    mb.addIreg(IR_TEMP3, MODBUS_INVALID_VALUE);
    mb.addIreg(IR_ESP32_HEARTBEAT, 0);
    mb.addIreg(IR_SENSOR1_HEALTH, CHANNEL_NOT_READY);
    mb.addIreg(IR_SENSOR2_HEALTH, CHANNEL_NOT_READY);
    mb.addIreg(IR_SENSOR3_HEALTH, CHANNEL_NOT_READY);

    mb.addHreg(HR_FAULT_MODE, NONE);

    Serial.println("Virtual v2: three-channel DS18B20 acquisition/diagnostic node");
}

void loop() {
    mb.task();

    unsigned long currentTime = millis();

    if (
        !temperatureConversionInProgress &&
        currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS
    ) {
        lastSampleTime = currentTime;
        startTemperatureConversion(currentTime);
    }

    if (
        temperatureConversionInProgress &&
        currentTime - temperatureRequestTime >= TEMP_CONVERSION_TIME_MS
    ) {
        // Read the test selector before acquisition so stale-injection modes can
        // suppress the normal refresh event without directly assigning health.
        selectFaultMode();

        finishTemperatureRead(currentTime);
        applyFaultInjection();

        updateChannelHealth(channel1Health, temperature1, currentTime);
        updateChannelHealth(channel2Health, temperature2, currentTime);
        updateChannelHealth(channel3Health, temperature3, currentTime);

        evaluateStuckRecovery();
        detectStuckSensors();

        publishModbusData();
        logCycle(currentTime);
    }
}
