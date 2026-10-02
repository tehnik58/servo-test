#pragma once

#include <stdint.h>

// Small hardware boundary so the control logic can be tested with a recording
// mock. The production implementation writes to PCA9685; tests do not simulate
// a whole robot or add a runtime simulation mode to the firmware.
class PwmOutput {
public:
    virtual ~PwmOutput() {}
    virtual bool isOnline(uint8_t pcaIndex) const = 0;
    virtual bool writePulseUs(uint8_t servoId, uint16_t pulseUs) = 0;
    virtual bool disableAll() = 0;
};
