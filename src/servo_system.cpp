#include "servo_system.h"

#include <string.h>

namespace {
const uint8_t RIGHT_HAND_IDS[] = {7, 8, 9, 10, 11, 12};
const uint8_t LEFT_HAND_IDS[] = {17, 18, 19, 20};

uint8_t interpolateAngle(uint8_t start, uint8_t target, uint32_t elapsed,
                         uint32_t duration) {
    if (elapsed >= duration || duration == 0) return target;
    const int32_t delta = static_cast<int32_t>(target) - start;
    const int64_t scaled = static_cast<int64_t>(delta) * elapsed;
    int32_t offset = static_cast<int32_t>(scaled / duration);
    if (scaled < 0 && (scaled % duration) != 0) --offset;
    int32_t result = static_cast<int32_t>(start) + offset;
    if (result < 0) result = 0;
    if (result > 180) result = 180;
    return static_cast<uint8_t>(result);
}
}

ServoSystem::ServoSystem(PwmOutput &output) : _output(output) {
    for (uint8_t i = 0; i < Config::TOTAL_SERVOS; ++i) {
        _angles[i] = 0;
        _known[i] = false;
        _motions[i].active = false;
        _motions[i].startAngle = 0;
        _motions[i].targetAngle = 0;
        _motions[i].lastWrittenAngle = 0;
        _motions[i].startMs = 0;
        _motions[i].durationMs = 0;
    }
    for (uint8_t i = 0; i < Config::MAX_GROUPS; ++i) {
        _groups[i].used = false;
        _groups[i].count = 0;
        _groups[i].name[0] = '\0';
    }
    addGroupUnchecked("RIGHT_HAND", RIGHT_HAND_IDS,
                      sizeof(RIGHT_HAND_IDS) / sizeof(RIGHT_HAND_IDS[0]));
    addGroupUnchecked("LEFT_HAND", LEFT_HAND_IDS,
                      sizeof(LEFT_HAND_IDS) / sizeof(LEFT_HAND_IDS[0]));
}

bool ServoSystem::mapServoId(int32_t servoId, uint8_t &pcaIndex, uint8_t &channel) {
    if (servoId < 0 || servoId >= Config::TOTAL_SERVOS) return false;
    pcaIndex = static_cast<uint8_t>(servoId / Config::CHANNELS_PER_PCA);
    channel = static_cast<uint8_t>(servoId % Config::CHANNELS_PER_PCA);
    return true;
}

Result ServoSystem::angleToPulseUs(int32_t servoId, int32_t angle, uint16_t &pulseUs) {
    if (servoId < 0 || servoId >= Config::TOTAL_SERVOS) return Result::INVALID_SERVO;
    if (angle < Config::MIN_ANGLE || angle > Config::MAX_ANGLE)
        return Result::ANGLE_OUT_OF_RANGE;

    const Config::ServoLimit &limit = Config::SERVO_LIMITS[servoId];
    if (limit.minAngle > limit.maxAngle || limit.maxAngle > Config::MAX_ANGLE ||
        limit.maxPulseUs <= limit.minPulseUs) {
        return Result::SERVO_LIMIT;
    }
    if (angle < limit.minAngle || angle > limit.maxAngle) return Result::SERVO_LIMIT;

    const uint32_t angleSpan = limit.maxAngle - limit.minAngle;
    const uint32_t pulseSpan = limit.maxPulseUs - limit.minPulseUs;
    if (angleSpan == 0) {
        if (angle != limit.minAngle) return Result::SERVO_LIMIT;
        pulseUs = limit.minPulseUs;
        return Result::OK;
    }
    const uint32_t offset = static_cast<uint32_t>(angle - limit.minAngle) * pulseSpan;
    pulseUs = static_cast<uint16_t>(limit.minPulseUs + (offset + angleSpan / 2) / angleSpan);
    return Result::OK;
}

Result ServoSystem::validateTarget(int32_t servoId, int32_t angle) const {
    uint16_t unusedPulse = 0;
    return angleToPulseUs(servoId, angle, unusedPulse);
}

Result ServoSystem::validateOutput(int32_t servoId) const {
    uint8_t pca = 0;
    uint8_t channel = 0;
    if (!mapServoId(servoId, pca, channel)) return Result::INVALID_SERVO;
    return _output.isOnline(pca) ? Result::OK : Result::PCA_NOT_FOUND;
}

Result ServoSystem::setAngle(int32_t servoId, int32_t angle, uint32_t nowMs) {
    return setAngles(&servoId, &angle, 1, nowMs);
}

Result ServoSystem::setAngles(const int32_t *servoIds, const int32_t *angles,
                              size_t count, uint32_t nowMs) {
    if (servoIds == NULL || angles == NULL || count == 0 || count > Config::TOTAL_SERVOS)
        return Result::INVALID_ARGUMENTS;

    for (size_t i = 0; i < count; ++i) {
        Result result = validateTarget(servoIds[i], angles[i]);
        if (result != Result::OK) return result;
        result = validateOutput(servoIds[i]);
        if (result != Result::OK) return result;
        for (size_t j = 0; j < i; ++j) {
            if (servoIds[i] == servoIds[j]) return Result::INVALID_ARGUMENTS;
        }
    }

    update(nowMs);
    for (size_t i = 0; i < count; ++i) {
        uint16_t pulseUs = 0;
        angleToPulseUs(servoIds[i], angles[i], pulseUs);
        if (!_output.writePulseUs(static_cast<uint8_t>(servoIds[i]), pulseUs)) {
            _known[static_cast<uint8_t>(servoIds[i])] = false;
            return Result::PCA_NOT_FOUND;
        }
        const uint8_t id = static_cast<uint8_t>(servoIds[i]);
        _angles[id] = static_cast<uint8_t>(angles[i]);
        _known[id] = true;
        _motions[id].active = false;
    }
    return Result::OK;
}

Result ServoSystem::addGroupUnchecked(const char *name, const uint8_t *ids, size_t count) {
    if (name == NULL || ids == NULL || count == 0 || count > Config::MAX_GROUP_MEMBERS)
        return Result::INVALID_ARGUMENTS;
    Group *slot = NULL;
    for (uint8_t i = 0; i < Config::MAX_GROUPS; ++i) {
        if (!_groups[i].used && slot == NULL) slot = &_groups[i];
    }
    if (slot == NULL) return Result::GROUP_FULL;

    const size_t nameLength = strlen(name);
    if (nameLength == 0 || nameLength > Config::MAX_GROUP_NAME_LENGTH)
        return Result::INVALID_ARGUMENTS;
    memcpy(slot->name, name, nameLength + 1);
    memcpy(slot->ids, ids, count);
    slot->count = static_cast<uint8_t>(count);
    slot->used = true;
    return Result::OK;
}

Result ServoSystem::createGroup(const char *name, const int32_t *servoIds, size_t count) {
    if (name == NULL || servoIds == NULL || count == 0 ||
        count > Config::MAX_GROUP_MEMBERS) return Result::INVALID_ARGUMENTS;
    const size_t nameLength = strlen(name);
    if (nameLength == 0 || nameLength > Config::MAX_GROUP_NAME_LENGTH)
        return Result::INVALID_ARGUMENTS;
    if (findGroup(name) != NULL) return Result::GROUP_EXISTS;

    uint8_t ids[Config::MAX_GROUP_MEMBERS];
    for (size_t i = 0; i < count; ++i) {
        if (servoIds[i] < 0 || servoIds[i] >= Config::TOTAL_SERVOS)
            return Result::INVALID_SERVO;
        for (size_t j = 0; j < i; ++j) {
            if (servoIds[i] == servoIds[j]) return Result::INVALID_ARGUMENTS;
        }
        ids[i] = static_cast<uint8_t>(servoIds[i]);
    }
    return addGroupUnchecked(name, ids, count);
}

ServoSystem::Group *ServoSystem::findGroup(const char *name) {
    if (name == NULL) return NULL;
    for (uint8_t i = 0; i < Config::MAX_GROUPS; ++i)
        if (_groups[i].used && strcmp(_groups[i].name, name) == 0) return &_groups[i];
    return NULL;
}

const ServoSystem::Group *ServoSystem::findGroup(const char *name) const {
    if (name == NULL) return NULL;
    for (uint8_t i = 0; i < Config::MAX_GROUPS; ++i)
        if (_groups[i].used && strcmp(_groups[i].name, name) == 0) return &_groups[i];
    return NULL;
}

Result ServoSystem::setGroup(const char *name, int32_t angle, uint32_t nowMs) {
    return setGroupOrMove(name, angle, 0, nowMs, false);
}

Result ServoSystem::setGroupOrMove(const char *name, int32_t angle, uint32_t durationMs,
                                   uint32_t nowMs, bool moving) {
    const Group *group = findGroup(name);
    if (group == NULL) return Result::GROUP_NOT_FOUND;
    int32_t ids[Config::MAX_GROUP_MEMBERS];
    int32_t angles[Config::MAX_GROUP_MEMBERS];
    for (uint8_t i = 0; i < group->count; ++i) {
        ids[i] = group->ids[i];
        angles[i] = angle;
    }
    if (moving) return moveSet(ids, angles, group->count, durationMs, nowMs);
    return setAngles(ids, angles, group->count, nowMs);
}

Result ServoSystem::move(int32_t servoId, int32_t angle, uint32_t durationMs,
                         uint32_t nowMs) {
    return moveSet(&servoId, &angle, 1, durationMs, nowMs);
}

Result ServoSystem::moveGroup(const char *name, int32_t angle, uint32_t durationMs,
                              uint32_t nowMs) {
    return setGroupOrMove(name, angle, durationMs, nowMs, true);
}

Result ServoSystem::moveSet(const int32_t *servoIds, const int32_t *angles, size_t count,
                            uint32_t durationMs, uint32_t nowMs) {
    if (servoIds == NULL || angles == NULL || count == 0 || count > Config::TOTAL_SERVOS)
        return Result::INVALID_ARGUMENTS;
    if (durationMs > Config::MAX_MOVE_TIME_MS) return Result::INVALID_ARGUMENTS;

    for (size_t i = 0; i < count; ++i) {
        Result result = validateTarget(servoIds[i], angles[i]);
        if (result != Result::OK) return result;
        result = validateOutput(servoIds[i]);
        if (result != Result::OK) return result;
        for (size_t j = 0; j < i; ++j)
            if (servoIds[i] == servoIds[j]) return Result::INVALID_ARGUMENTS;
        if (durationMs > 0 && servoIds[i] >= 0 && servoIds[i] < Config::TOTAL_SERVOS &&
            !_known[servoIds[i]]) return Result::POSITION_UNKNOWN;
    }

    update(nowMs);
    if (durationMs == 0) return setAngles(servoIds, angles, count, nowMs);
    for (size_t i = 0; i < count; ++i) {
        const uint8_t id = static_cast<uint8_t>(servoIds[i]);
        Motion &motion = _motions[id];
        motion.active = true;
        motion.startAngle = _angles[id];
        motion.targetAngle = static_cast<uint8_t>(angles[i]);
        motion.lastWrittenAngle = _angles[id];
        motion.startMs = nowMs;
        motion.durationMs = durationMs;
    }
    return Result::OK;
}

void ServoSystem::update(uint32_t nowMs) {
    for (uint8_t id = 0; id < Config::TOTAL_SERVOS; ++id) {
        Motion &motion = _motions[id];
        if (!motion.active) continue;
        const uint32_t elapsed = nowMs - motion.startMs; // wrap-safe for bounded durations
        const uint8_t angle = interpolateAngle(motion.startAngle, motion.targetAngle,
                                               elapsed, motion.durationMs);
        if (angle != motion.lastWrittenAngle) {
            uint16_t pulseUs = 0;
            if (angleToPulseUs(id, angle, pulseUs) != Result::OK ||
                !_output.writePulseUs(id, pulseUs)) {
                motion.active = false;
                _known[id] = false;
                continue;
            }
            motion.lastWrittenAngle = angle;
            _angles[id] = angle;
        }
        if (elapsed >= motion.durationMs) {
            _angles[id] = motion.targetAngle;
            motion.active = false;
        }
    }
}

Result ServoSystem::stop() {
    bool disabled = _output.disableAll();
    for (uint8_t id = 0; id < Config::TOTAL_SERVOS; ++id) {
        _motions[id].active = false;
        _known[id] = false;
    }
    return disabled ? Result::OK : Result::PCA_NOT_FOUND;
}

bool ServoSystem::getAngle(int32_t servoId, uint8_t &angle) const {
    if (servoId < 0 || servoId >= Config::TOTAL_SERVOS || !_known[servoId]) return false;
    angle = _angles[servoId];
    return true;
}

bool ServoSystem::isPcaOnline(uint8_t pcaIndex) const {
    return pcaIndex < Config::PCA_COUNT && _output.isOnline(pcaIndex);
}
