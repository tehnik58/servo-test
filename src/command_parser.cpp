#include "command_parser.h"

#include "config.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
constexpr uint8_t MAX_TOKENS = Config::TOTAL_SERVOS + 3;

bool parseInteger(const char *text, long &value) {
    if (text == NULL || *text == '\0') return false;
    errno = 0;
    char *end = NULL;
    const long parsed = strtol(text, &end, 10);
    if (errno == ERANGE || end == text || *end != '\0') return false;
    value = parsed;
    return true;
}

bool parsePair(char *text, int32_t &id, int32_t &angle) {
    if (text == NULL) return false;
    char *colon = strchr(text, ':');
    if (colon == NULL || colon == text || colon[1] == '\0' || strchr(colon + 1, ':') != NULL)
        return false;
    *colon = '\0';
    long parsedId = 0;
    long parsedAngle = 0;
    if (!parseInteger(text, parsedId) || !parseInteger(colon + 1, parsedAngle)) return false;
    if (parsedId < INT32_MIN || parsedId > INT32_MAX ||
        parsedAngle < INT32_MIN || parsedAngle > INT32_MAX) return false;
    id = static_cast<int32_t>(parsedId);
    angle = static_cast<int32_t>(parsedAngle);
    return true;
}
}

CommandParser::CommandParser(ServoSystem &system, TextOutput &output)
    : _system(system), _output(output) {}

void CommandParser::error(Result result) {
    const char *code = "INVALID_ARGUMENTS";
    switch (result) {
    case Result::INVALID_COMMAND: code = "INVALID_COMMAND"; break;
    case Result::INVALID_SERVO: code = "INVALID_SERVO"; break;
    case Result::ANGLE_OUT_OF_RANGE: code = "ANGLE_OUT_OF_RANGE"; break;
    case Result::SERVO_LIMIT: code = "SERVO_LIMIT"; break;
    case Result::PCA_NOT_FOUND: code = "PCA_NOT_FOUND"; break;
    case Result::GROUP_NOT_FOUND: code = "GROUP_NOT_FOUND"; break;
    case Result::GROUP_EXISTS: code = "GROUP_EXISTS"; break;
    case Result::GROUP_FULL: code = "GROUP_FULL"; break;
    case Result::POSITION_UNKNOWN: code = "POSITION_UNKNOWN"; break;
    case Result::INVALID_ARGUMENTS: code = "INVALID_ARGUMENTS"; break;
    case Result::OK: return;
    }
    char line[64];
    snprintf(line, sizeof(line), "ERROR %s", code);
    _output.writeLine(line);
}

void CommandParser::report(Result result) {
    if (result == Result::OK) _output.writeLine("OK");
    else error(result);
}

void CommandParser::help() {
    _output.writeLine("Commands:");
    _output.writeLine("SERVO <id> <angle>");
    _output.writeLine("GROUP_CREATE <name> <id> ...");
    _output.writeLine("GROUP <name> <angle>");
    _output.writeLine("SET <id>:<angle> ...");
    _output.writeLine("MOVE <id> <angle> <time_ms>");
    _output.writeLine("MOVE_GROUP <name> <angle> <time_ms>");
    _output.writeLine("MOVE_SET <time_ms> <id>:<angle> ...");
    _output.writeLine("GET <id>");
    _output.writeLine("GET_ALL");
    _output.writeLine("STATUS");
    _output.writeLine("HELP");
    _output.writeLine("STOP");
}

void CommandParser::status() {
    char line[64];
    _output.writeLine("OK STATUS");
    for (uint8_t i = 0; i < Config::PCA_COUNT; ++i) {
        snprintf(line, sizeof(line), "PCA%u: %s address=0x%02X",
                 static_cast<unsigned>(i), _system.isPcaOnline(i) ? "ONLINE" : "OFFLINE",
                 static_cast<unsigned>(Config::PCA_ADDRESSES[i]));
        _output.writeLine(line);
    }
    snprintf(line, sizeof(line), "SERVOS: %u", static_cast<unsigned>(Config::TOTAL_SERVOS));
    _output.writeLine(line);
}

void CommandParser::debugServo(int32_t servoId, int32_t angle) {
    if (!Config::DEBUG_ENABLED) return;
    uint8_t pca = 0;
    uint8_t channel = 0;
    uint16_t pulseUs = 0;
    if (!ServoSystem::mapServoId(servoId, pca, channel) ||
        ServoSystem::angleToPulseUs(servoId, angle, pulseUs) != Result::OK) return;
    char line[64];
    snprintf(line, sizeof(line), "[DEBUG] Servo: %ld", static_cast<long>(servoId));
    _output.writeLine(line);
    snprintf(line, sizeof(line), "[DEBUG] PCA: %u", static_cast<unsigned>(pca));
    _output.writeLine(line);
    snprintf(line, sizeof(line), "[DEBUG] Channel: %u", static_cast<unsigned>(channel));
    _output.writeLine(line);
    snprintf(line, sizeof(line), "[DEBUG] Angle: %ld", static_cast<long>(angle));
    _output.writeLine(line);
    snprintf(line, sizeof(line), "[DEBUG] Pulse: %u us", static_cast<unsigned>(pulseUs));
    _output.writeLine(line);
}

void CommandParser::process(char *line, uint32_t nowMs) {
    if (line == NULL) {
        error(Result::INVALID_ARGUMENTS);
        return;
    }
    if (Config::DEBUG_ENABLED) {
        char debugLine[Config::SERIAL_LINE_CAPACITY + 16];
        snprintf(debugLine, sizeof(debugLine), "[DEBUG] Command: %s", line);
        _output.writeLine(debugLine);
    }

    char *tokens[MAX_TOKENS];
    uint8_t count = 0;
    char *cursor = line;
    while (*cursor != '\0') {
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n') ++cursor;
        if (*cursor == '\0') break;
        if (count >= MAX_TOKENS) {
            error(Result::INVALID_ARGUMENTS);
            return;
        }
        tokens[count++] = cursor;
        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' &&
               *cursor != '\r' && *cursor != '\n') ++cursor;
        if (*cursor != '\0') *cursor++ = '\0';
    }
    if (count == 0) return;

    long values[3] = {0, 0, 0};
    int32_t ids[Config::TOTAL_SERVOS];
    int32_t angles[Config::TOTAL_SERVOS];

    if (strcmp(tokens[0], "SERVO") == 0 || strcmp(tokens[0], "MOVE") == 0) {
        const bool moving = strcmp(tokens[0], "MOVE") == 0;
        if (count != (moving ? 4 : 3) || !parseInteger(tokens[1], values[0]) ||
            !parseInteger(tokens[2], values[1]) ||
            (moving && !parseInteger(tokens[3], values[2])) ||
            values[0] < INT32_MIN || values[0] > INT32_MAX ||
            values[1] < INT32_MIN || values[1] > INT32_MAX ||
            (moving && (values[2] < 0 ||
                        values[2] > static_cast<long>(Config::MAX_MOVE_TIME_MS)))) {
            error(Result::INVALID_ARGUMENTS);
            return;
        }
        debugServo(static_cast<int32_t>(values[0]), static_cast<int32_t>(values[1]));
        report(moving ? _system.move(static_cast<int32_t>(values[0]),
                                     static_cast<int32_t>(values[1]),
                                     static_cast<uint32_t>(values[2]), nowMs)
                      : _system.setAngle(static_cast<int32_t>(values[0]),
                                         static_cast<int32_t>(values[1]), nowMs));
        return;
    }

    if (strcmp(tokens[0], "SET") == 0 || strcmp(tokens[0], "MOVE_SET") == 0) {
        const bool moving = strcmp(tokens[0], "MOVE_SET") == 0;
        const uint8_t firstPair = moving ? 2 : 1;
        if (count <= firstPair || count - firstPair > Config::TOTAL_SERVOS) {
            error(Result::INVALID_ARGUMENTS);
            return;
        }
        uint32_t duration = 0;
        if (moving) {
            if (!parseInteger(tokens[1], values[0]) || values[0] < 0 ||
                values[0] > static_cast<long>(Config::MAX_MOVE_TIME_MS)) {
                error(Result::INVALID_ARGUMENTS);
                return;
            }
            duration = static_cast<uint32_t>(values[0]);
        }
        const size_t pairCount = count - firstPair;
        for (size_t i = 0; i < pairCount; ++i) {
            if (!parsePair(tokens[firstPair + i], ids[i], angles[i])) {
                error(Result::INVALID_ARGUMENTS);
                return;
            }
        }
        report(moving ? _system.moveSet(ids, angles, pairCount, duration, nowMs)
                      : _system.setAngles(ids, angles, pairCount, nowMs));
        return;
    }

    if (strcmp(tokens[0], "GROUP_CREATE") == 0) {
        if (count < 3 || count - 2 > Config::MAX_GROUP_MEMBERS) {
            error(Result::INVALID_ARGUMENTS);
            return;
        }
        const size_t memberCount = count - 2;
        for (size_t i = 0; i < memberCount; ++i) {
            if (!parseInteger(tokens[i + 2], values[0]) ||
                values[0] < INT32_MIN || values[0] > INT32_MAX) {
                error(Result::INVALID_ARGUMENTS);
                return;
            }
            ids[i] = static_cast<int32_t>(values[0]);
        }
        report(_system.createGroup(tokens[1], ids, memberCount));
        return;
    }

    if (strcmp(tokens[0], "GROUP") == 0 || strcmp(tokens[0], "MOVE_GROUP") == 0) {
        const bool moving = strcmp(tokens[0], "MOVE_GROUP") == 0;
        if (count != (moving ? 4 : 3) || !parseInteger(tokens[2], values[0]) ||
            values[0] < INT32_MIN || values[0] > INT32_MAX ||
            (moving && (!parseInteger(tokens[3], values[1]) || values[1] < 0 ||
                        values[1] > static_cast<long>(Config::MAX_MOVE_TIME_MS)))) {
            error(Result::INVALID_ARGUMENTS);
            return;
        }
        report(moving ? _system.moveGroup(tokens[1], static_cast<int32_t>(values[0]),
                                           static_cast<uint32_t>(values[1]), nowMs)
                      : _system.setGroup(tokens[1], static_cast<int32_t>(values[0]), nowMs));
        return;
    }

    if (strcmp(tokens[0], "GET") == 0) {
        if (count != 2 || !parseInteger(tokens[1], values[0]) ||
            values[0] < INT32_MIN || values[0] > INT32_MAX) {
            error(Result::INVALID_ARGUMENTS);
            return;
        }
        _system.update(nowMs);
        uint8_t angle = 0;
        if (!_system.getAngle(static_cast<int32_t>(values[0]), angle)) {
            uint8_t pca = 0;
            uint8_t channel = 0;
            if (!ServoSystem::mapServoId(static_cast<int32_t>(values[0]), pca, channel))
                error(Result::INVALID_SERVO);
            else error(Result::POSITION_UNKNOWN);
            return;
        }
        char response[48];
        snprintf(response, sizeof(response), "OK SERVO %ld ANGLE %u", values[0],
                 static_cast<unsigned>(angle));
        _output.writeLine(response);
        return;
    }

    if (strcmp(tokens[0], "GET_ALL") == 0) {
        if (count != 1) {
            error(Result::INVALID_ARGUMENTS);
            return;
        }
        _system.update(nowMs);
        _output.writeLine("OK GET_ALL");
        for (uint8_t id = 0; id < Config::TOTAL_SERVOS; ++id) {
            uint8_t angle = 0;
            char response[48];
            if (_system.getAngle(id, angle))
                snprintf(response, sizeof(response), "SERVO %u = %u",
                         static_cast<unsigned>(id), static_cast<unsigned>(angle));
            else
                snprintf(response, sizeof(response), "SERVO %u = UNKNOWN",
                         static_cast<unsigned>(id));
            _output.writeLine(response);
        }
        return;
    }

    if (strcmp(tokens[0], "STATUS") == 0) {
        if (count != 1) error(Result::INVALID_ARGUMENTS);
        else status();
        return;
    }
    if (strcmp(tokens[0], "HELP") == 0) {
        if (count != 1) error(Result::INVALID_ARGUMENTS);
        else help();
        return;
    }
    if (strcmp(tokens[0], "STOP") == 0) {
        if (count != 1) error(Result::INVALID_ARGUMENTS);
        else {
            const Result result = _system.stop();
            if (result == Result::OK) _output.writeLine("OK STOPPED");
            else error(result);
        }
        return;
    }

    error(Result::INVALID_COMMAND);
}
