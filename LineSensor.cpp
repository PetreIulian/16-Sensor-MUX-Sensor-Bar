#ifndef LineSensor_h
#define LineSensor_h

#pragma once

#include <Arduino.h>
#include <EEPROM.h>

class LineSensor {
public:
    void SetLineSensor(uint8_t sig, uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3);
    void calibrate(bool calibrate);
    void SerialPrintData();
    long getLinePosition();

private:
    uint8_t SIG, S0, S1, S2, S3;
    static const unsigned int sensorCount = 16;

    long sensorValues[16];
    uint16_t maxValues[16];
    uint16_t minValues[16];

    long weightSum = 0;
    long max_value = 0;
    long total = 0;
    long lastLinePosition = 0;

    const int sensorWeight[16] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 14000, 15000};
    const int sensorThreshold = 150;

    int readMux(int channel);
    void saveCalibration();
    void loadCalibration();
    bool isSerial();
};

void LineSensor::SetLineSensor(uint8_t sig, uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3) {
    SIG = sig;
    S0 = s0;
    S1 = s1;
    S2 = s2;
    S3 = s3;

    pinMode(SIG, INPUT);
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
}

int LineSensor::readMux(int channel) {
    uint8_t controlPin[4] = {S3, S2, S1, S0};

   uint8_t muxChannel[16][4] = {
        {0,0,0,0}, // 1
        {0,0,0,1}, // 2
        {0,0,1,1}, // 3
        {0,0,1,0}, // 4
        {0,1,1,0}, // 5
        {0,1,1,1}, // 6
        {0,1,0,1}, // 7
        {0,1,0,0}, // 8
        {1,1,0,0}, // 9
        {1,1,0,1}, //10
        {1,1,1,1}, //11
        {1,1,1,0}, //12
        {1,0,1,0}, //13
        {1,0,1,1}, //14
        {1,0,0,1}, //15
        {1,0,0,0}  //16
    };

    for (int i = 0; i < 4; i++) {
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }

    return analogRead(SIG);
}

void LineSensor::calibrate(bool calibrate) {
    if (calibrate) {
        for (int i = 0; i < sensorCount; i++) {
            minValues[i] = 1023;
            maxValues[i] = 0;
        }

        if(isSerial()) {
            Serial.println("Calibration has Started...");
        }

        unsigned long startTime = millis();

        while(millis() - startTime < 10000) {
            for (int i = 0; i < sensorCount; i++) {
                long val = readMux(i);

                if (val < minValues[i]) minValues[i] = val;
                if (val > maxValues[i]) maxValues[i] = val;
            }
            delay(20);
        }

        if(isSerial()) {
            Serial.println("Calibration Completed.");

            for(int i = 0; i < sensorCount; i++) {
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.print(": Min = ");
                Serial.print(minValues[i]);
                Serial.print(", Max = ");
                Serial.println(maxValues[i]);
            }
        }

        saveCalibration();
    } else {
        if(isSerial()) {
            Serial.println("Loading Calibration Data from EEPROM...");
        }

        loadCalibration();
    }
}

long LineSensor::getLinePosition() {
    weightSum = 0;
    max_value = 0;
    total = 0;

    for (int i = 0; i < sensorCount; i++) {
        long raw = readMux(i);
        sensorValues[i] = raw;

        long normalized = map(raw, minValues[i], maxValues[i], 0, 1000);
        normalized = constrain(normalized, 0, 1000);

        weightSum += normalized * sensorWeight[i];
        total += normalized;

        if (raw > max_value) max_value = raw;
    }

    long linePosition = (total > 0) ? (weightSum / total) : -1;

    if (max_value < 150 || linePosition == -1) {
        linePosition = (lastLinePosition > 8000) ? 15000 : 0;
    } else {
        lastLinePosition = linePosition;
    }

    return linePosition;
}

void LineSensor::saveCalibration() {
    int addr = 0;
    for (int i = 0; i < sensorCount; i++) {
        EEPROM.put(addr, minValues[i]);
        addr += sizeof(uint16_t);

        EEPROM.put(addr, maxValues[i]);
        addr += sizeof(uint16_t);
    }
    EEPROM.commit();
}

void LineSensor::loadCalibration() {
    int addr = 0;
    for (int i = 0; i < sensorCount; i++) {
        EEPROM.get(addr, minValues[i]);
        addr += sizeof(uint16_t);

        EEPROM.get(addr, maxValues[i]);
        addr += sizeof(uint16_t);
    }
}

void LineSensor::SerialPrintData() {
    bool lineDetected = false;

    if(isSerial()) {
        Serial.print("Sensor Values: ");

        for (int i = 0; i < sensorCount; i++) {
            long val = readMux(i);
            sensorValues[i] = val;

            long normalizedValue = map(val, minValues[i], maxValues[i], 0, 1000);
            normalizedValue = constrain(normalizedValue, 0, 1000);

            if(normalizedValue> sensorThreshold) {
                lineDetected = true;
            }

            Serial.print(normalizedValue);
            Serial.print("|");
        }

        long linePosition = getLinePosition();

        if (!lineDetected) {
            linePosition = (lastLinePosition > 8000) ? 15000 : 0;
            Serial.print(" | Line Position: ");
            Serial.print(linePosition);
        } else {
            Serial.print(" | Line Position: ");
            Serial.print(linePosition);
        }

        Serial.println();
    }
}

bool LineSensor::isSerial() {
    return Serial;
}

#endif
