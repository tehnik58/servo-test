#pragma once

#include "servo_system.h"

class TextOutput {
public:
    virtual ~TextOutput() {}
    virtual void writeLine(const char *line) = 0;
};

class CommandParser {
public:
    CommandParser(ServoSystem &system, TextOutput &output);
    void process(char *line, uint32_t nowMs);

private:
    ServoSystem &_system;
    TextOutput &_output;
    void error(Result result);
    void report(Result result);
    void help();
    void status();
    void debugServo(int32_t servoId, int32_t angle);
};
