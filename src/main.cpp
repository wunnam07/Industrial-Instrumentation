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
void setup() {
    Serial.begin(115200);
    delay(1000);

    sensor1.begin();
    sensor2.begin();
    pinMode(LED_PIN, OUTPUT);
    

    Serial.println("DS18B20 Sensor 1&2 test");
}

void loop() {
    unsigned long timestamp= millis();
    unsigned long currentTime = millis();

    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
        lastSampleTime = currentTime;
    }
        sensor1.requestTemperatures();
        sensor2.requestTemperatures();  

        float temperature1 = 
    sensor1.getTempCByIndex(0);
        float temperature2 = 
    sensor2.getTempCByIndex(0);

        
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
        Serial.print(" ms] ");
        potValue = analogRead(POT_PIN);
        Serial.print("Potentiometer raw ADC:");
        Serial.println(potValue);
}