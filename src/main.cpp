#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const int TEMP1_PIN = 18;
const int TEMP2_PIN = 19;

OneWire oneWire1(TEMP1_PIN);
DallasTemperature sensor1(&oneWire1);

OneWire oneWire2(TEMP2_PIN);
DallasTemperature sensor2(&oneWire2);


const int POT_PIN = 34;
const int LED_PIN = 23;
int potValue = 0;

unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 1000;

float temperature1 = 0.0;
float temperature2 = 0.0;


float filteredPotValue = 0.0;
bool filterInitialized = false;
const float FILTER_ALPHA = 0.2;

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
    BOTH_SENSORS_VALID,
    SENSOR_1_FAULT,
    SENSOR_2_FAULT,
    SENSOR_DISAGREEMENT,
    NO_VALID_SENSOR
};

DiagnosticStatus diagnosticStatus = NO_VALID_SENSOR;

enum ProcessState {
    INITIALIZING,
    NORMAL,
    WARNING,
    TRIP
};

ProcessState processState = INITIALIZING;

void setup() {
    Serial.begin(115200);
    delay(1000);

    sensor1.begin();
    sensor2.begin();
    
    sensor1.setWaitForConversion(false);
    sensor2.setWaitForConversion(false);

    Serial.println("DS18B20 Sensor 1&2 test");
}

void readInput() {
    sensor1.requestTemperatures();
    sensor2.requestTemperatures();  

    temperature1 = sensor1.getTempCByIndex(0);
    temperature2 = sensor2.getTempCByIndex(0);

    potValue = analogRead(POT_PIN);
}

void runDiagnostics() {

    sensor1Valid = (temperature1 >= 0.0 && temperature1 <= 80.0);
    sensor2Valid = (temperature2 >= 0.0 && temperature2 <= 80.0);

    if (sensor1Valid && sensor2Valid) {
        bothSensorsValid = true;
        oneSensorValid = false;
        noSensorsValid = false;
    } else if (sensor1Valid || sensor2Valid) {
        bothSensorsValid = false;
        oneSensorValid = true;
        noSensorsValid = false;
    } else {
        bothSensorsValid = false;
        oneSensorValid = false;
        noSensorsValid = true;
    }

    if (bothSensorsValid) {
        sensorDifference = abs(temperature1 - temperature2);
        sensorsAgree = (sensorDifference <= 1.0);
        if (sensorDifference < 1.0) {
            sensorsAgree = true;
        }
        else {
            sensorsAgree = false;
        }
    } 
    else {
        sensorsAgree = false;
    }
}

void updateTrustedTemperature() {
    tControlValid = false;
    if (sensorsAgree) {
        tControl = (temperature1 + temperature2) / 2.0;
        tControlValid = true;
    } 
    else if (oneSensorValid) {
        tControl = sensor1Valid ? temperature1 : temperature2;
        tControlValid = true;
    } 
    else {
        tControlValid = false;
    }
}

void updateDiagnosticsStatus() {
    if (sensorsAgree) {
        diagnosticStatus = BOTH_SENSORS_VALID;
    } 
    else if (sensor1Valid && !sensor2Valid) {
        diagnosticStatus = SENSOR_2_FAULT;
    } 
    else if (!sensor1Valid && sensor2Valid) {
        diagnosticStatus = SENSOR_1_FAULT;
    } 
    else if (bothSensorsValid && !sensorsAgree) {
        diagnosticStatus = SENSOR_DISAGREEMENT;
    } 
    else {
        diagnosticStatus = NO_VALID_SENSOR;
    } 
}

void updateProcessState() {
    if (!tControlValid) {
        processState = INITIALIZING;
    }
    else if (tControl >= 60.0) {
        processState = TRIP;
    }
    else if (tControl >= 50.0) {
        processState = WARNING;
    }
    else {
        processState = NORMAL;
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
            (1.0 - FILTER_ALPHA) * filteredPotValue;
    }
}

void logMeasurements(unsigned long timestamp) {
    Serial.print("[");
    Serial.print(timestamp);
    Serial.print(" ms] ");
    Serial.print("Temperature 1:");
    Serial.print(temperature1);
    Serial.println(" C");

    Serial.print("[");
    Serial.print(timestamp);
    Serial.print(" ms] ");
    Serial.print("Temperature 2:");
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
}



void loop() {
    unsigned long timestamp= millis();
    unsigned long currentTime = millis();

    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
        lastSampleTime = currentTime;
        
        readInput();
        filterInputs();
        runDiagnostics();
        updateDiagnosticsStatus();
        updateTrustedTemperature();
        updateProcessState();
        logMeasurements(timestamp);
    }
}
