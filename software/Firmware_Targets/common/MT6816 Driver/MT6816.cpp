/*
 * MT6816.cpp
 *
 * SimpleFOC-compatible SPI driver for the MagnTek MT6816 encoder.
 */

#include "MT6816.h"

namespace {
constexpr float kRadiansPerCount = (2.0f * PI) / MT6816_CPR;
constexpr uint8_t kReadCommand = MT6816_RW;
constexpr uint16_t kNoMagnetBit = 0x0002u;
}

MT6816::MT6816(SPISettings settings, int nCS)
    : spi(nullptr), settings(settings), nCS(nCS) {
}

MT6816::~MT6816() {
}

void MT6816::init(SPIClass* _spi) {
    spi = _spi;
    errorflag = (spi == nullptr);

    if (spi == nullptr) {
        return;
    }

    if (nCS >= 0) {
        pinMode(nCS, OUTPUT);
        digitalWrite(nCS, HIGH);
    }

    spi->begin();
    readRawAngle(); // Prime the cached value used by getFastAngle().
}

float MT6816::getCurrentAngle() {
    return static_cast<float>(readRawAngle()) * kRadiansPerCount;
}

float MT6816::getFastAngle() {
    const float angle = static_cast<float>(lastAngle) * kRadiansPerCount;
    readRawAngle();
    return angle;
}

uint16_t MT6816::readRawAngle() {
    if (spi == nullptr) {
        errorflag = true;
        return lastAngle;
    }

    // Register 0x03 holds angle bits [13:6]. Register 0x04 holds angle
    // bits [5:0], the no-magnet warning, and the even-parity bit.
    const uint16_t frame =
        (static_cast<uint16_t>(spi_transfer8(MT6816_REG_ADDR1 | kReadCommand)) << 8) |
        spi_transfer8(MT6816_REG_ADDR2 | kReadCommand);

    uint8_t parity = 0;
    for (uint8_t bit = 0; bit < 16; ++bit) {
        parity ^= static_cast<uint8_t>((frame >> bit) & 0x01u);
    }

    // The parity bit makes the complete 16-bit frame even-parity.
    errorflag = (parity != 0u) || ((frame & kNoMagnetBit) != 0u);
    if (!errorflag) {
        lastAngle = frame >> 2;
    }

    return lastAngle;
}

uint8_t MT6816::spi_transfer8(uint8_t outdata) {
    if (spi == nullptr) {
        return 0;
    }

    spi->beginTransaction(settings);
    if (nCS >= 0) {
        digitalWrite(nCS, LOW);
    }

    const uint8_t indata = spi->transfer(outdata);

    if (nCS >= 0) {
        digitalWrite(nCS, HIGH);
    }
    spi->endTransaction();
    return indata;
}

