#pragma once

#include "config.h"
#include "pwm_output.h"

#include <stddef.h>
#include <stdint.h>

enum class Result : uint8_t {
    OK,
    INVALID_COMMAND,
    INVALID_SERVO,
    ANGLE_OUT_OF_RANGE,
    SERVO_LIMIT,
    PCA_NOT_FOUND,
    INVALID_ARGUMENTS,
    GROUP_NOT_FOUND,
    GROUP_EXISTS,
    GROUP_FULL,
    POSITION_UNKNOWN
};

class ServoSystem {
public:
    explicit ServoSystem(PwmOutput &output);

    Result setAngle(int32_t servoId, int32_t angle, uint32_t nowMs);
    Result setAngles(const int32_t *servoIds, const int32_t *angles,
                     size_t count, uint32_t nowMs);
    Result createGroup(const char *name, const int32_t *servoIds, size_t count);
    Result setGroup(const char *name, int32_t angle, uint32_t nowMs);

    Result move(int32_t servoId, int32_t angle, uint32_t durationMs, uint32_t nowMs);
    Result moveGroup(const char *name, int32_t angle, uint32_t durationMs,
                     uint32_t nowMs);
    Result moveSet(const int32_t *servoIds, const int32_t *angles, size_t count,
                   uint32_t durationMs, uint32_t nowMs);
    Result stop();
    void update(uint32_t nowMs);

    bool getAngle(int32_t servoId, uint8_t &angle) const;
    bool isPcaOnline(uint8_t pcaIndex) const;

    static bool mapServoId(int32_t servoId, uint8_t &pcaIndex, uint8_t &channel);
    static Result angleToPulseUs(int32_t servoId, int32_t angle, uint16_t &pulseUs);

private:
    struct Group {
        bool used;
        char name[Config::MAX_GROUP_NAME_LENGTH + 1];
        uint8_t ids[Config::MAX_GROUP_MEMBERS];
        uint8_t count;
    };

    struct Motion {
        bool active;
        uint8_t startAngle;
        uint8_t targetAngle;
        uint8_t lastWrittenAngle;
        uint32_t startMs;
        uint32_t durationMs;
    };

    PwmOutput &_output;
    uint8_t _angles[Config::TOTAL_SERVOS];
    bool _known[Config::TOTAL_SERVOS];
    Motion _motions[Config::TOTAL_SERVOS];
    Group _groups[Config::MAX_GROUPS];

    Result validateTarget(int32_t servoId, int32_t angle) const;
    Result validateOutput(int32_t servoId) const;
    Result setGroupOrMove(const char *name, int32_t angle, uint32_t durationMs,
                          uint32_t nowMs, bool moving);
    Group *findGroup(const char *name);
    const Group *findGroup(const char *name) const;
    Result addGroupUnchecked(const char *name, const uint8_t *ids, size_t count);
};
