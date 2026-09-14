#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <ModbusIP_ESP8266.h>

const uint16_t IR_TEMP1             = 0;
const uint16_t IR_TEMP2             = 1;
const uint16_t IR_TCONTROL          = 2;
const uint16_t IR_SENSOR_DIFF       = 3;
const uint16_t IR_POT_RAW           = 4;
const uint16_t IR_POT_SCALED        = 5;
const uint16_t IR_DIAGNOSTIC_STATUS = 6;
const uint16_t IR_PROCESS_STATE     = 7;
const uint16_t IR_ESP32_HEARTBEAT   = 8;

const uint16_t DI_SYSTEM_INITIALIZED = 0;
const uint16_t DI_SENSOR1_VALID      = 1;
const uint16_t DI_SENSOR2_VALID      = 2;
const uint16_t DI_SENSORS_AGREE      = 3;
const uint16_t DI_TCONTROL_VALID     = 4;
const uint16_t DI_HEATER_PERMISSIVE  = 5;
const uint16_t DI_HEATER_OUTPUT      = 6;
const uint16_t DI_PLC_COMMS_OK       = 7;

const uint16_t COIL_HEATER_CMD = 0;
const uint16_t HR_PLC_HEARTBEAT = 0;

const int TEMP1_PIN = 18;
const int TEMP2_PIN = 19;
const int POT_PIN = 34;
const int LED_PIN = 23;

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

ModbusIP mb;

OneWire oneWire1(TEMP1_PIN);
DallasTemperature sensor1(&oneWire1);

OneWire oneWire2(TEMP2_PIN);
DallasTemperature sensor2(&oneWire2);

int potValue = 0;
float scaled_ADC = 0.0;
float temperature1 = 0.0;
float temperature2 = 0.0;
float filteredPotValue = 0.0;

bool filterInitialized = false;
const float FILTER_ALPHA = 0.2;

unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 1000;

bool sensor1Valid = false;
bool sensor2Valid = false;
bool bothSensorsValid = false;
bool oneSensorValid = false;
bool noSensorsValid = false;

float sensorDifference = 0.0;
bool sensorsAgree = false;

float tControl = 0.0;
bool tControlValid = false;

enum DiagnosticStatus {
    BOTH_SENSORS_VALID = 0,
    SENSOR_1_FAULT = 1,
    SENSOR_2_FAULT = 2,
    SENSOR_DISAGREEMENT = 3,
    NO_VALID_SENSOR = 4,
    SENSOR_1_STUCK = 5,
    SENSOR_2_STUCK = 6,
    SENSOR_DISAGREEMENT_PENDING = 7
};

DiagnosticStatus diagnosticStatus = NO_VALID_SENSOR;

enum ProcessState {
    INITIALIZING,
    NORMAL,
    WARNING,
    TRIP
};

ProcessState processState = INITIALIZING;

bool systemInitialized = false;
const unsigned long INITIALIZATION_TIME = 3000;
unsigned long startupTime = 0;

bool HeaterPermissive = false;
bool heaterCommand = false;
bool heaterOutput = false;

bool temperatureConversionInProgress = false;
unsigned long temperatureRequestTime = 0;
const unsigned long TEMP_CONVERSION_TIME = 750;

uint16_t esp32Heartbeat = 0;

bool plcCommsOK = false;
uint16_t lastPLCHeartbeat = 0;
unsigned long lastPLCHeartbeatTime = 0;
const unsigned long PLC_COMMS_TIMEOUT = 3000;

bool driftTestEnabled = false;
float driftOffset = 0.0;

bool sensor1Stuck = false;
bool sensor2Stuck = false;

float stuckWindowStartT1 = NAN;
float stuckWindowStartT2 = NAN;

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

float lastTrustedTControl = NAN;

bool debounceTestEnabled = true;
int debounceTestSample = 0;

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

    startupTime = millis();

    sensor1.begin();
    sensor2.begin();

    sensor1.setWaitForConversion(false);
    sensor2.setWaitForConversion(false);

    connectWiFi();

    mb.server();

    mb.addIreg(IR_TEMP1);
    mb.addIreg(IR_TEMP2);
    mb.addIreg(IR_TCONTROL);
    mb.addIreg(IR_SENSOR_DIFF);
    mb.addIreg(IR_POT_RAW);
    mb.addIreg(IR_POT_SCALED);
    mb.addIreg(IR_DIAGNOSTIC_STATUS);
    mb.addIreg(IR_PROCESS_STATE);
    mb.addIreg(IR_ESP32_HEARTBEAT);

    mb.addIsts(DI_SYSTEM_INITIALIZED);
    mb.addIsts(DI_SENSOR1_VALID);
    mb.addIsts(DI_SENSOR2_VALID);
    mb.addIsts(DI_SENSORS_AGREE);
    mb.addIsts(DI_TCONTROL_VALID);
    mb.addIsts(DI_HEATER_PERMISSIVE);
    mb.addIsts(DI_HEATER_OUTPUT);
    mb.addIsts(DI_PLC_COMMS_OK);

    mb.addCoil(COIL_HEATER_CMD, false);
    mb.addHreg(HR_PLC_HEARTBEAT, 0);

    Serial.println("DS18B20 Sensor 1&2 test");
}

void startTemperatureConversion(unsigned long currentTime) {
    sensor1.requestTemperatures();
    sensor2.requestTemperatures();

    temperatureRequestTime = currentTime;
    temperatureConversionInProgress = true;
}

void finishInputRead() {
    temperature1 = sensor1.getTempCByIndex(0);
    temperature2 = sensor2.getTempCByIndex(0);

    potValue = analogRead(POT_PIN);

    temperatureConversionInProgress = false;
}

void runDiagnostics() {
    sensor1Valid =
        (temperature1 >= 0.0 &&
         temperature1 <= 80.0);

    sensor2Valid =
        (temperature2 >= 0.0 &&
         temperature2 <= 80.0);

    if (sensor1Valid && sensor2Valid) {
        bothSensorsValid = true;
        oneSensorValid = false;
        noSensorsValid = false;
    }
    else if (sensor1Valid || sensor2Valid) {
        bothSensorsValid = false;
        oneSensorValid = true;
        noSensorsValid = false;
    }
    else {
        bothSensorsValid = false;
        oneSensorValid = false;
        noSensorsValid = true;
    }

    if (bothSensorsValid) {
        sensorDifference =
            abs(temperature1 - temperature2);

        sensorsAgree =
            (sensorDifference <= 1.0);

        if (!sensorsAgree) {
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
    else {
        sensorsAgree = false;
        sensorDifference = 0.0;

        disagreementBadCount = 0;
        disagreementGoodCount = 0;
        disagreementPending = false;
        sensorDisagreementConfirmed = false;
    }
}

void updateTrustedTemperature() {
    tControlValid = false;

    if (sensor1Stuck && sensor2Valid) {
        tControl = temperature2;
        tControlValid = true;
        lastTrustedTControl = tControl;
    }
    else if (sensor2Stuck && sensor1Valid) {
        tControl = temperature1;
        tControlValid = true;
        lastTrustedTControl = tControl;
    }
    else if (oneSensorValid) {
        tControl =
            sensor1Valid
            ? temperature1
            : temperature2;

        tControlValid = true;
        lastTrustedTControl = tControl;
    }
    else if (sensorDisagreementConfirmed) {
        tControl = NAN;
        tControlValid = false;
    }
    else if (sensorsAgree) {
        tControl =
            (temperature1 + temperature2) / 2.0;

        tControlValid = true;
        lastTrustedTControl = tControl;
    }
    else if (
        disagreementPending &&
        !isnan(lastTrustedTControl)
    ) {
        tControl = lastTrustedTControl;
        tControlValid = true;
    }
    else {
        tControl = NAN;
        tControlValid = false;
    }
}

void updateDiagnosticsStatus() {

    if (sensor1Stuck) {
        diagnosticStatus = SENSOR_1_STUCK;
    }

    else if (sensor2Stuck) {
        diagnosticStatus = SENSOR_2_STUCK;
    }

    else if (sensorDisagreementConfirmed) {
        diagnosticStatus =
            SENSOR_DISAGREEMENT;
    }

    else if (disagreementPending) {
        diagnosticStatus =
            SENSOR_DISAGREEMENT_PENDING;
    }

    else if (sensorsAgree) {
        diagnosticStatus =
            BOTH_SENSORS_VALID;
    }

    else if (sensor1Valid && !sensor2Valid) {
        diagnosticStatus =
            SENSOR_2_FAULT;
    }

    else if (!sensor1Valid && sensor2Valid) {
        diagnosticStatus =
            SENSOR_1_FAULT;
    }

    else {
        diagnosticStatus =
            NO_VALID_SENSOR;
    }
}

void updateProcessState(
    unsigned long currentTime
) {
    if (!systemInitialized) {
        processState = INITIALIZING;

        if (
            currentTime - startupTime
                >= INITIALIZATION_TIME
            &&
            tControlValid
        ) {
            systemInitialized = true;
        }

        return;
    }

    if (!tControlValid) {
        return;
    }

    if (tControl >= 60.0) {
        processState = TRIP;
    }
    else if (tControl >= 50.0) {
        processState = WARNING;
    }
    else {
        processState = NORMAL;
    }
}

void checkPLCHeartbeat(
    unsigned long currentTime
) {
    uint16_t currentHeartbeat =
        mb.Hreg(HR_PLC_HEARTBEAT);

    if (
        currentHeartbeat
        != lastPLCHeartbeat
    ) {
        lastPLCHeartbeat =
            currentHeartbeat;

        lastPLCHeartbeatTime =
            currentTime;

        plcCommsOK = true;
    }

    if (
        plcCommsOK &&
        currentTime - lastPLCHeartbeatTime
            > PLC_COMMS_TIMEOUT
    ) {
        plcCommsOK = false;
    }
}

void updateHeaterPermissive() {
    if (
        systemInitialized &&
        tControlValid &&
        plcCommsOK &&
        (
            processState == NORMAL ||
            processState == WARNING
        )
    ) {
        HeaterPermissive = true;
    }
    else {
        HeaterPermissive = false;
    }
}

void updateHeaterOutput() {
    if (
        HeaterPermissive &&
        heaterCommand
    ) {
        heaterOutput = true;
        digitalWrite(LED_PIN, HIGH);
    }
    else {
        heaterOutput = false;
        digitalWrite(LED_PIN, LOW);
    }
}

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
        case BOTH_SENSORS_VALID:
            return "BOTH_SENSORS_VALID";

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

        default:
            return "UNKNOWN";
    }
}

void updateModbusInputRegisters() {
    uint16_t temp1Modbus =
        (uint16_t)
        round(temperature1 * 100.0);

    uint16_t temp2Modbus =
        (uint16_t)
        round(temperature2 * 100.0);

    uint16_t tControlModbus;
    if (tControlValid) {
        tControlModbus =
            (uint16_t)
            round(tControl * 100.0);
    }
    else {
        tControlModbus = 65535;
    }

    uint16_t sensorDiffModbus =
        (uint16_t)
        round(sensorDifference * 100.0);

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
        IR_TCONTROL,
        tControlModbus
    );

    mb.Ireg(
        IR_SENSOR_DIFF,
        sensorDiffModbus
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
        IR_PROCESS_STATE,
        (uint16_t)processState
    );

    mb.Ireg(
        IR_ESP32_HEARTBEAT,
        esp32Heartbeat
    );
}

void updateModbusDiscreteInputs() {
    mb.Ists(
        DI_SYSTEM_INITIALIZED,
        systemInitialized
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
        sensorsAgree
    );

    mb.Ists(
        DI_TCONTROL_VALID,
        tControlValid
    );

    mb.Ists(
        DI_HEATER_PERMISSIVE,
        HeaterPermissive
    );

    mb.Ists(
        DI_HEATER_OUTPUT,
        heaterOutput
    );

    mb.Ists(
        DI_PLC_COMMS_OK,
        plcCommsOK
    );
}

void readModbusCommands() {
    heaterCommand =
        mb.Coil(
            COIL_HEATER_CMD
        );
}

void applyDriftTest() {

    if (!driftTestEnabled) {
        return;
    }

    driftOffset += 0.25;

    temperature1 = temperature2 + driftOffset;
}

void detectStuckSensors() {

    if (!sensor1Valid || !sensor2Valid) {

        sensor1Stuck = false;
        sensor2Stuck = false;

        stuckWindowStartT1 = NAN;
        stuckWindowStartT2 = NAN;

        stuckWindowSamples = 0;

        return;
    }

    if (isnan(stuckWindowStartT1) || isnan(stuckWindowStartT2)) {

        stuckWindowStartT1 = temperature1;
        stuckWindowStartT2 = temperature2;

        stuckWindowSamples = 0;

        return;
    }

    stuckWindowSamples++;

    if (stuckWindowSamples < STUCK_WINDOW_SAMPLES) {
        return;
    }

    float sensor1Change =
        abs(temperature1 - stuckWindowStartT1);

    float sensor2Change =
        abs(temperature2 - stuckWindowStartT2);

    sensor1Stuck =
        sensor1Change <= STUCK_MAX_CHANGE &&
        sensor2Change >= PROCESS_CHANGE_MIN;

    sensor2Stuck =
        sensor2Change <= STUCK_MAX_CHANGE &&
        sensor1Change >= PROCESS_CHANGE_MIN;

    stuckWindowStartT1 = temperature1;
    stuckWindowStartT2 = temperature2;

    stuckWindowSamples = 0;
}

void logMeasurements(unsigned long timestamp) {
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

    Serial.print("systemInitialized: ");
    Serial.println(systemInitialized);

    Serial.print("sensor1Valid: ");
    Serial.println(sensor1Valid);

    Serial.print("sensor2Valid: ");
    Serial.println(sensor2Valid);

    Serial.print("sensorsAgree: ");
    Serial.println(sensorsAgree);

    Serial.print("sensorDifference: ");
    Serial.println(sensorDifference);

    Serial.print("diagnosticStatus: ");
    Serial.println(diagnosticStatusToString());

    Serial.print("sensor1Stuck: ");
    Serial.println(sensor1Stuck);

    Serial.print("sensor2Stuck: ");
    Serial.println(sensor2Stuck);

    Serial.print("disagreementPending: ");
    Serial.println(disagreementPending);

    Serial.print("disagreementBadCount: ");
    Serial.println(disagreementBadCount);

    Serial.print("disagreementGoodCount: ");
    Serial.println(disagreementGoodCount);

    Serial.print("sensorDisagreementConfirmed: ");
    Serial.println(sensorDisagreementConfirmed);

    Serial.print("tControlValid: ");
    Serial.println(tControlValid);

    Serial.print("tControl: ");
    Serial.println(tControl);

    Serial.print("processState: ");
    
    if (!tControlValid) {
        Serial.println("INVALID -no trusted temperature");
    }

    else {
        Serial.println(processState);
    }

    Serial.print("PLC heartbeat: ");
    Serial.println(lastPLCHeartbeat);

    Serial.print("plcCommsOK: ");
    Serial.println(plcCommsOK);

    Serial.print("HeaterPermissive: ");
    Serial.println(HeaterPermissive);

    Serial.print("heaterCommand: ");
    Serial.println(heaterCommand);

    Serial.print("heaterOutput: ");
    Serial.println(heaterOutput);

    Serial.print("ESP32 heartbeat: ");
    Serial.println(esp32Heartbeat);

    Serial.println("--------------------");
}

void applyDebounceTest() {

    if (!debounceTestEnabled) {
        return;
    }

    if (!plcCommsOK || !heaterCommand) {
        return;
    }

    debounceTestSample++;

    if (debounceTestSample <= 3) {
        temperature1 = 25.0;
        temperature2 = 25.0;
    }

    else if (debounceTestSample == 4) {
        temperature1 = 30.0;
        temperature2 = 25.0;
    }

    else if (debounceTestSample <= 7) {
        temperature1 = 25.0;
        temperature2 = 25.0;
    }

    else if (debounceTestSample <= 10) {
        temperature1 = 30.0;
        temperature2 = 25.0;
    }

    else {
        temperature1 = 25.0;
        temperature2 = 25.0;
    }
}

void loop() {
    mb.task();

    unsigned long currentTime = millis();

    readModbusCommands();

    checkPLCHeartbeat(currentTime);

    updateHeaterPermissive();

    updateHeaterOutput();

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
        finishInputRead();

        applyDebounceTest();
        applyDriftTest();

        filterInputs();

        ADCscaling();

        runDiagnostics();

        detectStuckSensors();

        updateDiagnosticsStatus();

        updateTrustedTemperature();

        updateProcessState(
            currentTime
        );

        updateHeaterPermissive();

        updateHeaterOutput();

        updateModbusInputRegisters();

        updateModbusDiscreteInputs();

        logMeasurements(
            currentTime
        );
    }
}