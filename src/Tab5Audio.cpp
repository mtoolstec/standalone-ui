#include "Tab5Audio.h"

#ifdef TAB5_MUI

#include <Arduino.h>
#include <M5Unified.h>
#include <Wire.h>

namespace {

constexpr uint8_t kEs8388Addr = 0x10;
constexpr uint8_t kPi4IoAddr = 0x43;
constexpr uint8_t kPi4IoSpeakerReg = 0x05;
constexpr uint8_t kPi4IoSpeakerBit = 0x02;
constexpr uint32_t kI2cFreq = 400000;
constexpr int kTab5InternalSda = 31;
constexpr int kTab5InternalScl = 32;

constexpr uint8_t kEs8388EnableSeq[] = {
    2, 0, 0x80,
    2, 0, 0x00,
    2, 0, 0x00,
    2, 0, 0x0E,
    2, 1, 0x00,
    2, 2, 0x0A,
    2, 3, 0xFF,
    2, 4, 0x3C,
    2, 5, 0x00,
    2, 6, 0x00,
    2, 7, 0x7C,
    2, 8, 0x00,
    2, 23, 0x18,
    2, 24, 0x00,
    2, 25, 0x20,
    2, 26, 0x00,
    2, 27, 0x00,
    2, 28, 0x08,
    2, 29, 0x00,
    2, 38, 0x00,
    2, 39, 0xB8,
    2, 42, 0xB8,
    2, 43, 0x08,
    2, 45, 0x00,
    2, 46, 0x21,
    2, 47, 0x21,
    2, 48, 0x21,
    2, 49, 0x21,
    0,
};

struct ToneStep {
    uint16_t frequency;
    uint16_t durationMs;
    uint16_t gapMs;
};

struct TonePlaybackState {
    bool active = false;
    uint8_t patternIndex = 0;
    uint8_t stepIndex = 0;
    uint32_t nextStepAtMs = 0;
};

constexpr ToneStep kTonePatterns[][3] = {
    {{1760, 90, 20}, {2093, 120, 0}, {0, 0, 0}},
    {{1568, 80, 25}, {1976, 80, 25}, {2637, 130, 0}},
    {{1319, 70, 15}, {1760, 70, 15}, {2349, 110, 0}},
    {{1047, 80, 20}, {1568, 120, 0}, {0, 0, 0}},
};

bool gSpeakerReady = false;
bool gAudioBusReady = false;
TwoWire gAudioWire(1);
TonePlaybackState gTonePlayback;

constexpr size_t kTonePatternCount = sizeof(kTonePatterns) / sizeof(kTonePatterns[0]);

void stopPlayback()
{
    M5.Speaker.stop();
    gTonePlayback.active = false;
    gTonePlayback.stepIndex = 0;
    gTonePlayback.nextStepAtMs = 0;
}

void startNextStep(uint32_t nowMs)
{
    const auto &pattern = kTonePatterns[gTonePlayback.patternIndex];
    const auto &step = pattern[gTonePlayback.stepIndex];

    if (step.frequency == 0 || step.durationMs == 0) {
        stopPlayback();
        return;
    }

    M5.Speaker.tone(step.frequency, step.durationMs, 0, true);
    gTonePlayback.nextStepAtMs = nowMs + step.durationMs + step.gapMs;
    ++gTonePlayback.stepIndex;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value)
{
    gAudioWire.beginTransmission(address);
    gAudioWire.write(reg);
    gAudioWire.write(value);
    return gAudioWire.endTransmission() == 0;
}

bool readRegister(uint8_t address, uint8_t reg, uint8_t &value)
{
    gAudioWire.beginTransmission(address);
    gAudioWire.write(reg);
    if (gAudioWire.endTransmission(false) != 0) {
        return false;
    }

    if (gAudioWire.requestFrom(static_cast<int>(address), 1) != 1) {
        return false;
    }

    value = gAudioWire.read();
    return true;
}

bool applyCodecInit()
{
    const uint8_t *ptr = kEs8388EnableSeq;
    while (*ptr != 0) {
        const uint8_t len = *ptr++;
        if (len != 2) {
            return false;
        }
        const uint8_t reg = *ptr++;
        const uint8_t value = *ptr++;
        if (!writeRegister(kEs8388Addr, reg, value)) {
            return false;
        }
    }
    return true;
}

bool enableAmplifier()
{
    uint8_t reg = 0;
    if (!readRegister(kPi4IoAddr, kPi4IoSpeakerReg, reg)) {
        return false;
    }
    reg |= kPi4IoSpeakerBit;
    return writeRegister(kPi4IoAddr, kPi4IoSpeakerReg, reg);
}

bool ensureSpeakerReady()
{
    if (gSpeakerReady) {
        return true;
    }

    if (!gAudioBusReady) {
        gAudioBusReady = gAudioWire.begin(kTab5InternalSda, kTab5InternalScl, kI2cFreq);
    }
    if (!gAudioBusReady) {
        return false;
    }

    gAudioWire.setClock(kI2cFreq);

    if (!applyCodecInit() || !enableAmplifier()) {
        return false;
    }

    auto cfg = M5.Speaker.config();
    cfg.pin_mck = GPIO_NUM_30;
    cfg.pin_bck = GPIO_NUM_27;
    cfg.pin_ws = GPIO_NUM_29;
    cfg.pin_data_out = GPIO_NUM_26;
    cfg.i2s_port = I2S_NUM_0;
    cfg.magnification = 4;
    cfg.sample_rate = 48000;
    M5.Speaker.config(cfg);

    if (!M5.Speaker.begin()) {
        return false;
    }

    M5.Speaker.setVolume(180);
    gSpeakerReady = true;
    return true;
}

} // namespace

void tab5PlayNotificationTone(uint8_t ringtoneId)
{
    if (ringtoneId == 0 || !ensureSpeakerReady()) {
        return;
    }

    gTonePlayback.patternIndex = ringtoneId % kTonePatternCount;
    gTonePlayback.stepIndex = 0;
    gTonePlayback.active = true;
    M5.Speaker.stop();
    startNextStep(millis());
}

void tab5AudioTaskHandler()
{
    if (!gTonePlayback.active) {
        return;
    }

    const uint32_t nowMs = millis();
    if (static_cast<int32_t>(nowMs - gTonePlayback.nextStepAtMs) < 0) {
        return;
    }

    startNextStep(nowMs);
}

#else

void tab5PlayNotificationTone(uint8_t ringtoneId)
{
    (void)ringtoneId;
}

void tab5AudioTaskHandler()
{
}

#endif