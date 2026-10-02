#include "pca_manager.h"

#include "config.h"

PcaManager::PcaManager()
    : _pca0(Config::PCA_ADDRESSES[0], Wire),
      _pca1(Config::PCA_ADDRESSES[1], Wire) {
    _online[0] = false;
    _online[1] = false;
}

Adafruit_PWMServoDriver &PcaManager::driver(uint8_t pcaIndex) {
    return pcaIndex == 0 ? _pca0 : _pca1;
}

bool PcaManager::probe(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

void PcaManager::begin() {
    Wire.begin();
    Wire.setClock(Config::I2C_CLOCK_HZ);
    for (uint8_t i = 0; i < Config::PCA_COUNT; ++i) {
        _online[i] = probe(Config::PCA_ADDRESSES[i]);
        if (_online[i]) {
            driver(i).begin();
            driver(i).setPWMFreq(Config::SERVO_FREQUENCY_HZ);
        }
    }
}

bool PcaManager::isOnline(uint8_t pcaIndex) const {
    return pcaIndex < Config::PCA_COUNT && _online[pcaIndex];
}

bool PcaManager::writeChannel(uint8_t pcaIndex, uint8_t channel,
                              uint16_t onCount, uint16_t offCount) {
    if (pcaIndex >= Config::PCA_COUNT || channel >= Config::CHANNELS_PER_PCA ||
        onCount > 4095 || offCount > 4096) return false;

    const uint8_t address = Config::PCA_ADDRESSES[pcaIndex];
    const uint8_t firstRegister = static_cast<uint8_t>(0x06 + 4 * channel);
    const uint8_t offHigh = offCount == 4096
        ? 0x10
        : static_cast<uint8_t>((offCount >> 8) & 0x0F);

    Wire.beginTransmission(address);
    const bool queued =
        Wire.write(firstRegister) == 1 &&
        Wire.write(static_cast<uint8_t>(onCount & 0xFF)) == 1 &&
        Wire.write(static_cast<uint8_t>((onCount >> 8) & 0x0F)) == 1 &&
        Wire.write(static_cast<uint8_t>(offCount & 0xFF)) == 1 &&
        Wire.write(offHigh) == 1;
    const uint8_t status = Wire.endTransmission();
    return queued && status == 0;
}

bool PcaManager::writePulseUs(uint8_t servoId, uint16_t pulseUs) {
    if (servoId >= Config::TOTAL_SERVOS) return false;
    const uint8_t pcaIndex = servoId / Config::CHANNELS_PER_PCA;
    const uint8_t channel = servoId % Config::CHANNELS_PER_PCA;
    if (!isOnline(pcaIndex)) return false;

    const uint32_t ticks = static_cast<uint32_t>(
        static_cast<float>(pulseUs) * 4096.0f * Config::SERVO_FREQUENCY_HZ /
        1000000.0f + 0.5f);
    if (ticks >= 4096 || !writeChannel(pcaIndex, channel, 0,
                                       static_cast<uint16_t>(ticks))) {
        _online[pcaIndex] = false;
        return false;
    }
    return true;
}

bool PcaManager::disableAll() {
    bool allDisabled = true;
    for (uint8_t pca = 0; pca < Config::PCA_COUNT; ++pca) {
        if (!_online[pca]) {
            allDisabled = false;
            continue;
        }
        bool pcaDisabled = true;
        for (uint8_t channel = 0; channel < Config::CHANNELS_PER_PCA; ++channel) {
            if (!writeChannel(pca, channel, 0, 4096)) pcaDisabled = false;
        }
        if (!pcaDisabled) {
            _online[pca] = false;
            allDisabled = false;
        }
    }
    return allDisabled;
}
