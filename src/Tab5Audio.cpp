#include "Tab5Audio.h"

#ifdef TAB5_MUI

#include <Arduino.h>
#include <M5Unified.h>
#include <cstring>
#include <graphics/common/Ringtones.h>
#include <util/ILog.h>

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
    constexpr size_t kTonePatternStepCount = sizeof(kTonePatterns[0]) / sizeof(kTonePatterns[0][0]);
    if (gTonePlayback.stepIndex >= kTonePatternStepCount) {
        stopPlayback();
        return;
    }

    const auto &pattern = kTonePatterns[gTonePlayback.patternIndex];
    const auto &step = pattern[gTonePlayback.stepIndex];

    if (step.frequency == 0 || step.durationMs == 0) {
        stopPlayback();
        return;
    }

    const bool toneStarted = M5.Speaker.tone(step.frequency, step.durationMs, 0, true);
    ILOG_INFO("TAB5 audio tone step freq=%u duration=%u started=%d", step.frequency, step.durationMs, toneStarted ? 1 : 0);
    gTonePlayback.nextStepAtMs = nowMs + step.durationMs + step.gapMs;
    ++gTonePlayback.stepIndex;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value)
{
    return M5.In_I2C.writeRegister(address, reg, &value, 1, kI2cFreq);
}

bool readRegister(uint8_t address, uint8_t reg, uint8_t &value)
{
    return M5.In_I2C.readRegister(address, reg, &value, 1, kI2cFreq);
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
            ILOG_WARN("TAB5 audio ES8388 write failed: reg=0x%02x value=0x%02x", reg, value);
            return false;
        }
    }
    return true;
}

bool enableAmplifier()
{
    uint8_t reg = 0;
    if (!readRegister(kPi4IoAddr, kPi4IoSpeakerReg, reg)) {
        ILOG_WARN("TAB5 audio PI4IO read failed: addr=0x%02x reg=0x%02x", kPi4IoAddr, kPi4IoSpeakerReg);
        return false;
    }
    ILOG_INFO("TAB5 audio PI4IO speaker reg before=0x%02x", reg);
    reg |= kPi4IoSpeakerBit;
    const bool ok = writeRegister(kPi4IoAddr, kPi4IoSpeakerReg, reg);
    ILOG_INFO("TAB5 audio PI4IO speaker reg after=0x%02x write=%d", reg, ok ? 1 : 0);
    return ok;
}

bool ensureSpeakerReady()
{
    if (gSpeakerReady) {
        return true;
    }

    if (!gAudioBusReady) {
        M5.In_I2C.setPort(I2C_NUM_1, kTab5InternalSda, kTab5InternalScl);
        gAudioBusReady = M5.In_I2C.begin();
        ILOG_INFO("TAB5 audio In_I2C begin port=%d sda=%d scl=%d ok=%d", M5.In_I2C.getPort(), kTab5InternalSda,
                  kTab5InternalScl, gAudioBusReady ? 1 : 0);
    }
    if (!gAudioBusReady) {
        return false;
    }

    const bool codecReady = applyCodecInit();
    const bool ampReady = codecReady && enableAmplifier();
    ILOG_INFO("TAB5 audio hardware init codec=%d amp=%d", codecReady ? 1 : 0, ampReady ? 1 : 0);
    if (!codecReady || !ampReady) {
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

    const bool speakerStarted = M5.Speaker.begin();
    ILOG_INFO("TAB5 audio speaker begin=%d pin_mck=%d pin_bck=%d pin_ws=%d pin_data=%d port=%d",
              speakerStarted ? 1 : 0, cfg.pin_mck, cfg.pin_bck, cfg.pin_ws, cfg.pin_data_out, cfg.i2s_port);
    if (!speakerStarted) {
        return false;
    }

    M5.Speaker.setVolume(180);
    gSpeakerReady = true;
    ILOG_INFO("TAB5 audio speaker ready volume=%u", M5.Speaker.getVolume());
    return true;
}

} // namespace

void tab5AudioInit()
{
}

void tab5PlayNotificationTone(uint8_t ringtoneId)
{
    ILOG_INFO("TAB5 audio notification tone requested id=%u", ringtoneId);
    if (ringtoneId == 0 || !ensureSpeakerReady()) {
        ILOG_WARN("TAB5 audio notification tone skipped id=%u", ringtoneId);
        return;
    }

    gTonePlayback.patternIndex = ringtoneId % kTonePatternCount;
    gTonePlayback.stepIndex = 0;
    gTonePlayback.active = true;
    M5.Speaker.stop();
    startNextStep(millis());
}

void tab5PlayNotificationRtttl(const char *rtttl)
{
    if (!rtttl || rtttl[0] == '\0') {
        ILOG_WARN("TAB5 audio notification rtttl skipped: empty");
        return;
    }

    ILOG_INFO("TAB5 audio notification rtttl='%.32s'", rtttl);

    uint8_t ringtoneId = 0;
    for (int i = 0; i < numRingtones; ++i) {
        if (std::strncmp(ringtone[i].rtttl, rtttl, 16) == 0) {
            ringtoneId = static_cast<uint8_t>(i);
            break;
        }
    }

    if (ringtoneId == 0 && std::strncmp(rtttl, "Silent:", 7) != 0) {
        ringtoneId = 1;
    }

    tab5PlayNotificationTone(ringtoneId);
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

void tab5AudioInit()
{
}

void tab5PlayNotificationTone(uint8_t ringtoneId)
{
    (void)ringtoneId;
}

void tab5PlayNotificationRtttl(const char *rtttl)
{
    (void)rtttl;
}

void tab5AudioTaskHandler()
{
}

#endif