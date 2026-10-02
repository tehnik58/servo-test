#pragma once

#include "pwm_output.h"

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

class PcaManager : public PwmOutput {
public:
    PcaManager();
    void begin();
    bool isOnline(uint8_t pcaIndex) const override;
    bool writePulseUs(uint8_t servoId, uint16_t pulseUs) override;
    bool disableAll() override;

private:
    Adafruit_PWMServoDriver _pca0;
    Adafruit_PWMServoDriver _pca1;
    bool _online[2];
    Adafruit_PWMServoDriver &driver(uint8_t pcaIndex);
    bool probe(uint8_t address);
    bool writeChannel(uint8_t pcaIndex, uint8_t channel,
                      uint16_t onCount, uint16_t offCount);
};
