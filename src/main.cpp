#include <Arduino.h>

#include "command_parser.h"
#include "config.h"
#include "pca_manager.h"
#include "servo_system.h"

namespace {
class SerialTextOutput : public TextOutput {
public:
    void writeLine(const char *line) override { Serial.println(line); }
};

SerialTextOutput serialOutput;
PcaManager pca;
ServoSystem servoSystem(pca);
CommandParser commandParser(servoSystem, serialOutput);
char lineBuffer[Config::SERIAL_LINE_CAPACITY];
uint16_t lineLength = 0;
bool lineOverflow = false;
}

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    pca.begin();
}

void loop() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n') {
            if (lineOverflow) Serial.println("ERROR INVALID_ARGUMENTS");
            else {
                lineBuffer[lineLength] = '\0';
                commandParser.process(lineBuffer, millis());
            }
            lineLength = 0;
            lineOverflow = false;
        } else if (!lineOverflow) {
            if (static_cast<size_t>(lineLength) + 1 < sizeof(lineBuffer))
                lineBuffer[lineLength++] = c;
            else lineOverflow = true;
        }
    }
    servoSystem.update(millis());
}
