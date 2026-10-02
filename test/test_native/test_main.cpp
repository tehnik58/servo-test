#include <unity.h>

#include "command_parser.h"
#include "config.h"
#include "servo_system.h"

#include <string.h>

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

namespace {
struct Write {
    uint8_t id;
    uint16_t pulseUs;
};

class MockPwm : public PwmOutput {
public:
    bool online[Config::PCA_COUNT];
    Write writes[256];
    size_t writeCount;
    bool disableCalled;

    MockPwm() : writeCount(0), disableCalled(false) {
        for (uint8_t i = 0; i < Config::PCA_COUNT; ++i) online[i] = true;
    }
    bool isOnline(uint8_t pcaIndex) const override {
        return pcaIndex < Config::PCA_COUNT && online[pcaIndex];
    }
    bool writePulseUs(uint8_t servoId, uint16_t pulseUs) override {
        if (writeCount >= sizeof(writes) / sizeof(writes[0])) return false;
        writes[writeCount++] = {servoId, pulseUs};
        return true;
    }
    bool disableAll() override { disableCalled = true; return true; }
};

class MockText : public TextOutput {
public:
    char lines[64][96];
    size_t count;
    MockText() : count(0) {}
    void writeLine(const char *line) override {
        if (count >= 64) return;
        strncpy(lines[count], line, sizeof(lines[count]) - 1);
        lines[count][sizeof(lines[count]) - 1] = '\0';
        ++count;
    }
    bool contains(const char *expected) const {
        for (size_t i = 0; i < count; ++i)
            if (strcmp(lines[i], expected) == 0) return true;
        return false;
    }
};

void test_id_mapping_boundaries() {
    uint8_t pca = 0;
    uint8_t channel = 0;
    TEST_ASSERT_TRUE(ServoSystem::mapServoId(0, pca, channel));
    TEST_ASSERT_EQUAL_UINT8(0, pca);
    TEST_ASSERT_EQUAL_UINT8(0, channel);
    TEST_ASSERT_TRUE(ServoSystem::mapServoId(15, pca, channel));
    TEST_ASSERT_EQUAL_UINT8(0, pca);
    TEST_ASSERT_EQUAL_UINT8(15, channel);
    TEST_ASSERT_TRUE(ServoSystem::mapServoId(16, pca, channel));
    TEST_ASSERT_EQUAL_UINT8(1, pca);
    TEST_ASSERT_EQUAL_UINT8(0, channel);
    TEST_ASSERT_TRUE(ServoSystem::mapServoId(31, pca, channel));
    TEST_ASSERT_EQUAL_UINT8(1, pca);
    TEST_ASSERT_EQUAL_UINT8(15, channel);
    TEST_ASSERT_FALSE(ServoSystem::mapServoId(-1, pca, channel));
    TEST_ASSERT_FALSE(ServoSystem::mapServoId(32, pca, channel));
}

void test_angle_to_pulse_and_validation() {
    uint16_t pulse = 0;
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK),
                      static_cast<int>(ServoSystem::angleToPulseUs(0, 0, pulse)));
    TEST_ASSERT_EQUAL_UINT16(500, pulse);
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK),
                      static_cast<int>(ServoSystem::angleToPulseUs(0, 90, pulse)));
    TEST_ASSERT_EQUAL_UINT16(1500, pulse);
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK),
                      static_cast<int>(ServoSystem::angleToPulseUs(0, 180, pulse)));
    TEST_ASSERT_EQUAL_UINT16(2500, pulse);
    TEST_ASSERT_EQUAL(static_cast<int>(Result::ANGLE_OUT_OF_RANGE),
                      static_cast<int>(ServoSystem::angleToPulseUs(0, -1, pulse)));
    TEST_ASSERT_EQUAL(static_cast<int>(Result::INVALID_SERVO),
                      static_cast<int>(ServoSystem::angleToPulseUs(32, 90, pulse)));
}

void test_invalid_batch_does_not_write() {
    MockPwm output;
    ServoSystem system(output);
    const int32_t ids[] = {0, 1};
    const int32_t angles[] = {45, 181};
    TEST_ASSERT_EQUAL(static_cast<int>(Result::ANGLE_OUT_OF_RANGE),
                      static_cast<int>(system.setAngles(ids, angles, 2, 0)));
    TEST_ASSERT_EQUAL_UINT32(0, output.writeCount);
}

void test_set_and_groups() {
    MockPwm output;
    ServoSystem system(output);
    const int32_t ids[] = {0, 16};
    const int32_t angles[] = {30, 150};
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK),
                      static_cast<int>(system.setAngles(ids, angles, 2, 0)));
    TEST_ASSERT_EQUAL_UINT32(2, output.writeCount);
    TEST_ASSERT_EQUAL_UINT8(0, output.writes[0].id);
    TEST_ASSERT_EQUAL_UINT8(16, output.writes[1].id);
    uint8_t angle = 0;
    TEST_ASSERT_TRUE(system.getAngle(0, angle));
    TEST_ASSERT_EQUAL_UINT8(30, angle);

    const int32_t groupIds[] = {2, 3};
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK),
                      static_cast<int>(system.createGroup("TEST", groupIds, 2)));
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK),
                      static_cast<int>(system.setGroup("TEST", 90, 1)));
    TEST_ASSERT_EQUAL_UINT32(4, output.writeCount);
    TEST_ASSERT_EQUAL(static_cast<int>(Result::GROUP_EXISTS),
                      static_cast<int>(system.createGroup("TEST", groupIds, 2)));
}

void test_motion_and_stop() {
    MockPwm output;
    ServoSystem system(output);
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK),
                      static_cast<int>(system.setAngle(0, 0, 0)));
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK),
                      static_cast<int>(system.move(0, 100, 1000, 0)));
    system.update(500);
    uint8_t angle = 0;
    TEST_ASSERT_TRUE(system.getAngle(0, angle));
    TEST_ASSERT_EQUAL_UINT8(50, angle);
    system.update(1000);
    TEST_ASSERT_TRUE(system.getAngle(0, angle));
    TEST_ASSERT_EQUAL_UINT8(100, angle);
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK), static_cast<int>(system.stop()));
    TEST_ASSERT_TRUE(output.disableCalled);
    TEST_ASSERT_FALSE(system.getAngle(0, angle));
    TEST_ASSERT_EQUAL(static_cast<int>(Result::POSITION_UNKNOWN),
                      static_cast<int>(system.move(0, 90, 1000, 1001)));
}

void test_move_set_advances_all_servos_together() {
    MockPwm output;
    ServoSystem system(output);
    const int32_t ids[] = {0, 16};
    const int32_t startAngles[] = {0, 180};
    const int32_t targetAngles[] = {100, 80};
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK),
                      static_cast<int>(system.setAngles(ids, startAngles, 2, 100)));
    TEST_ASSERT_EQUAL(static_cast<int>(Result::OK),
                      static_cast<int>(system.moveSet(ids, targetAngles, 2, 1000, 100)));
    system.update(600);
    uint8_t angle0 = 0;
    uint8_t angle16 = 0;
    TEST_ASSERT_TRUE(system.getAngle(0, angle0));
    TEST_ASSERT_TRUE(system.getAngle(16, angle16));
    TEST_ASSERT_EQUAL_UINT8(50, angle0);
    TEST_ASSERT_EQUAL_UINT8(130, angle16);
    system.update(1100);
    TEST_ASSERT_TRUE(system.getAngle(0, angle0));
    TEST_ASSERT_TRUE(system.getAngle(16, angle16));
    TEST_ASSERT_EQUAL_UINT8(100, angle0);
    TEST_ASSERT_EQUAL_UINT8(80, angle16);
}

void test_parser_crlf_and_errors() {
    MockPwm output;
    ServoSystem system(output);
    MockText text;
    CommandParser parser(system, text);
    char valid[] = "SERVO 7 90\r";
    parser.process(valid, 0);
    TEST_ASSERT_TRUE(text.contains("OK"));
    TEST_ASSERT_EQUAL_UINT32(1, output.writeCount);

    char invalid[] = "SERVO 7 250";
    parser.process(invalid, 1);
    TEST_ASSERT_TRUE(text.contains("ERROR ANGLE_OUT_OF_RANGE"));
    TEST_ASSERT_EQUAL_UINT32(1, output.writeCount);

    char malformed[] = "SET 1:20 broken";
    parser.process(malformed, 2);
    TEST_ASSERT_TRUE(text.contains("ERROR INVALID_ARGUMENTS"));
    TEST_ASSERT_EQUAL_UINT32(1, output.writeCount);

    char unknown[] = "NOT_A_COMMAND";
    parser.process(unknown, 3);
    TEST_ASSERT_TRUE(text.contains("ERROR INVALID_COMMAND"));
    TEST_ASSERT_EQUAL_UINT32(1, output.writeCount);
}
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_id_mapping_boundaries);
    RUN_TEST(test_angle_to_pulse_and_validation);
    RUN_TEST(test_invalid_batch_does_not_write);
    RUN_TEST(test_set_and_groups);
    RUN_TEST(test_motion_and_stop);
    RUN_TEST(test_move_set_advances_all_servos_together);
    RUN_TEST(test_parser_crlf_and_errors);
    return UNITY_END();
}
