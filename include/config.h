#pragma once

#include <stdint.h>

namespace Config {

constexpr uint32_t SERIAL_BAUD = 115200;

// Arduino Due's default Wire bus uses SDA=20 and SCL=21. The Arduino Due
// Wire API does not provide arbitrary pin remapping; keep these values here
// as wiring documentation rather than pretending they are runtime-selectable.
constexpr uint8_t I2C_SDA_PIN = 20;
constexpr uint8_t I2C_SCL_PIN = 21;
constexpr uint32_t I2C_CLOCK_HZ = 100000;

constexpr uint8_t PCA_COUNT = 2;
constexpr uint8_t CHANNELS_PER_PCA = 16;
constexpr uint8_t TOTAL_SERVOS = PCA_COUNT * CHANNELS_PER_PCA;
constexpr uint8_t PCA_ADDRESSES[PCA_COUNT] = {0x40, 0x41};

constexpr float SERVO_FREQUENCY_HZ = 50.0f;
constexpr uint16_t SERVO_MIN_PULSE_US = 500;
constexpr uint16_t SERVO_MAX_PULSE_US = 2500;
constexpr uint8_t MIN_ANGLE = 0;
constexpr uint8_t MAX_ANGLE = 180;

constexpr bool DEBUG_ENABLED = false;
constexpr uint16_t SERIAL_LINE_CAPACITY = 256;
constexpr uint8_t MAX_GROUPS = 8;
constexpr uint8_t MAX_GROUP_NAME_LENGTH = 15;
constexpr uint8_t MAX_GROUP_MEMBERS = 16;
constexpr uint32_t MAX_MOVE_TIME_MS = 600000;

struct ServoLimit {
    uint8_t minAngle;
    uint8_t maxAngle;
    uint16_t minPulseUs;
    uint16_t maxPulseUs;
};

// Edit the entry at a servo's logical ID to apply per-servo calibration.
// Defaults are intentionally conservative placeholders until verified on the robot.
constexpr ServoLimit SERVO_LIMITS[TOTAL_SERVOS] = {
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500},
    {0, 180, 500, 2500}, {0, 180, 500, 2500}
};

} // namespace Config
