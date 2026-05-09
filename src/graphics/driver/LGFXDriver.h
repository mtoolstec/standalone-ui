#pragma once

#include "LovyanGFX.h"
#include "graphics/driver/DisplayDriverConfig.h"
#include "graphics/driver/TFTDriver.h"
#include "input/InputDriver.h"
#include "lvgl_private.h"
#include "util/ILog.h"
#include <functional>
#include <vector>

#ifdef TAB5_MUI
#include <M5GFX.h>
#endif

constexpr uint32_t defaultLongPressTime = 600;
constexpr uint32_t defaultGestureLimit = 10;
constexpr uint32_t defaultScreenTimeout = 30 * 1000;
constexpr uint32_t defaultBrightness = 153;

template <class LGFX> class LGFXDriver : public TFTDriver<LGFX>
{
  public:
    LGFXDriver(uint16_t width, uint16_t height);
    LGFXDriver(const DisplayDriverConfig &cfg);
    void init(DeviceGUI *gui) override;
    bool calibrate(uint16_t parameters[8]) override;
    bool hasTouch(void) override;
    bool hasButton(void) override { return lgfx->hasButton(); }
    bool hasLight(void) override { return lgfx->light(); }
    bool isPowersaving(void) override { return powerSaving; }
    void printConfig(void) override;
    void task_handler(void) override;

    uint8_t getBrightness(void) override { return lgfx->getBrightness(); }
    void setBrightness(uint8_t brightness) override;

    uint16_t getScreenTimeout() override { return screenTimeout / 1000; }
    void setScreenTimeout(uint16_t timeout) override { screenTimeout = timeout * 1000; };

  protected:
    static void display_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
    static void rounder_cb(lv_event_t *e);
    static void touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data);

    uint32_t screenTimeout;
    uint32_t lastBrightness;
    bool powerSaving;

  private:
    void init_lgfx(void);

    static LGFX *lgfx;
    size_t bufsize;
    lv_color_t *buf1;
    lv_color_t *buf2;
    bool calibrating;
};

template <class LGFX> LGFX *LGFXDriver<LGFX>::lgfx = nullptr;

#ifdef TAB5_MUI
template <class LGFX> static m5gfx::M5Canvas *&getTab5Canvas()
{
    static m5gfx::M5Canvas *canvas = nullptr;
    return canvas;
}
#endif

template <class LGFX>
LGFXDriver<LGFX>::LGFXDriver(uint16_t width, uint16_t height)
    : TFTDriver<LGFX>(lgfx ? lgfx : new LGFX, width, height), screenTimeout(defaultScreenTimeout),
      lastBrightness(defaultBrightness), powerSaving(false), bufsize(0), buf1(nullptr), buf2(nullptr), calibrating(false)
{
    lgfx = this->tft;
}

template <class LGFX>
LGFXDriver<LGFX>::LGFXDriver(const DisplayDriverConfig &cfg)
    : TFTDriver<LGFX>(lgfx ? lgfx : new LGFX(cfg), cfg.width(), cfg.height()), screenTimeout(defaultScreenTimeout),
      lastBrightness(defaultBrightness), powerSaving(false), bufsize(0), buf1(nullptr), buf2(nullptr), calibrating(false)
{
    lgfx = this->tft;
}

template <class LGFX> bool LGFXDriver<LGFX>::hasTouch(void)
{
#ifdef CUSTOM_TOUCH_DRIVER
    return true;
#else
    return lgfx->touch();
#endif
}

template <class LGFX> void LGFXDriver<LGFX>::task_handler(void)
{
#ifdef TAB5_MUI
    bool touchWake = false;
    int pin_int = -1;
    if (hasTouch()) {
        uint16_t touchX = 0;
        uint16_t touchY = 0;
        touchWake = lgfx->getTouch(&touchX, &touchY);
#ifndef CUSTOM_TOUCH_DRIVER
        pin_int = lgfx->touch()->config().pin_int;
#else
        pin_int = lgfx->getTouchInt();
#endif
    }

    const bool irqWake = false;
    const bool recentActivityWake = (lv_display_get_inactive_time(NULL) < 150);
    const bool wakeRequested = touchWake || irqWake || recentActivityWake;
    const uint8_t restoreBrightness = lastBrightness > 0 ? static_cast<uint8_t>(lastBrightness) : defaultBrightness;

    if ((screenTimeout > 0 && lv_display_get_inactive_time(NULL) > screenTimeout) || powerSaving ||
        (DisplayDriver::view->isScreenLocked())) {
        if (hasLight()) {
            if (!powerSaving && wakeRequested && lgfx->getBrightness() == 0 && !DisplayDriver::view->isScreenLocked()) {
                ILOG_INFO("tab5 backlight forced wake");
                DisplayDriver::view->triggerHeartbeat();
                lgfx->powerSaveOff();
                lgfx->wakeup();
                lgfx->setBrightness(255);
                delay(2);
                lgfx->setBrightness(restoreBrightness);
                ILOG_INFO("tab5 backlight restore(forced): target=%u actual=%u", static_cast<unsigned>(restoreBrightness),
                          static_cast<unsigned>(lgfx->getBrightness()));
                DisplayDriver::view->screenSaving(false);
                lv_display_trigger_activity(NULL);
            }

            if (!powerSaving) {
                uint32_t brightness = lgfx->getBrightness();
                if (brightness > 0) {
                    lgfx->setBrightness(brightness - 1);
                } else {
                    ILOG_INFO("enter tab5 backlight save");
                    DisplayDriver::view->screenSaving(true);
                    powerSaving = true;
                }
            }

            if (powerSaving && wakeRequested) {
                ILOG_INFO("leave tab5 backlight save");
                powerSaving = false;
                DisplayDriver::view->triggerHeartbeat();
                lgfx->powerSaveOff();
                lgfx->wakeup();
                lgfx->setBrightness(255);
                delay(2);
                lgfx->setBrightness(restoreBrightness);
                ILOG_INFO("tab5 backlight restore: target=%u actual=%u", static_cast<unsigned>(restoreBrightness),
                          static_cast<unsigned>(lgfx->getBrightness()));
                DisplayDriver::view->screenSaving(false);
                lv_display_trigger_activity(NULL);
            }
        }
    } else if (lgfx->getBrightness() < restoreBrightness) {
        lgfx->setBrightness(restoreBrightness);
    }

    if (!calibrating) {
        DisplayDriver::task_handler();
    }

    return;
#endif

    if ((screenTimeout > 0 && lv_display_get_inactive_time(NULL) > screenTimeout) || powerSaving ||
        (DisplayDriver::view->isScreenLocked())) {
        if (DisplayDriver::view->getInputDriver()->hasPointerDevice() || hasTouch() ||
            DisplayDriver::view->getInputDriver()->hasKeyboardDevice() || hasButton()) {
            if (hasLight()) {
                if (!powerSaving) {
                    uint32_t brightness = lgfx->getBrightness();
                    if (brightness > 0) {
                        lgfx->setBrightness(brightness - 1);
                    } else {
                        ILOG_INFO("enter powersave");
                        DisplayDriver::view->screenSaving(true);
                        if (hasTouch() && hasButton()) {
                            ILOG_DEBUG("disable touch, enable button input");
                            lv_indev_enable(DisplayDriver::touch, false);
                            lv_indev_enable(InputDriver::instance()->getButton(), true);
                        }
                        lgfx->sleep();
                        lgfx->powerSaveOn();
                        powerSaving = true;
                    }
                }
                if (powerSaving) {
                    int pin_int = -1;
                    bool touched = false;
                    if (hasTouch()) {
#ifndef CUSTOM_TOUCH_DRIVER
                        pin_int = lgfx->touch()->config().pin_int;
                        uint16_t tx = 0;
                        uint16_t ty = 0;
                        touched = lgfx->getTouch(&tx, &ty);
#else
                        pin_int = lgfx->getTouchInt();
                        uint16_t tx = 0;
                        uint16_t ty = 0;
                        touched = lgfx->getTouchXY(&tx, &ty);
#endif
                    }
                    if (hasButton()) {
#ifdef BUTTON_PIN
                        pin_int = BUTTON_PIN;
#endif
                    }
                    if (touched || (pin_int >= 0 && DisplayDriver::view->sleep(pin_int)) ||
                        (lv_display_get_inactive_time(NULL) < 150 && !DisplayDriver::view->isScreenLocked())) {
                        delay(2);
                        ILOG_INFO("leaving powersave");
                        powerSaving = false;
                        DisplayDriver::view->triggerHeartbeat();
                        lgfx->powerSaveOff();
                        lgfx->wakeup();
                        lgfx->setBrightness(lastBrightness);
                        DisplayDriver::view->screenSaving(false);
                        if (hasTouch() && hasButton()) {
                            ILOG_DEBUG("enable touch, disable button input");
                            lv_indev_enable(DisplayDriver::touch, true);
                            lv_indev_enable(InputDriver::instance()->getButton(), false);
                        }
                        lv_display_trigger_activity(NULL);
                    }
                }
            } else {
                if (!powerSaving) {
                    DisplayDriver::view->blankScreen(true);
                    lgfx->sleep();
                    lgfx->powerSaveOn();
                    powerSaving = true;
                }
                if (screenTimeout > lv_display_get_inactive_time(NULL)) {
                    DisplayDriver::view->blankScreen(false);
                    lgfx->powerSaveOff();
                    lgfx->wakeup();
                    powerSaving = false;
                    lv_disp_trig_activity(NULL);
                }
            }
        }
    } else if (lgfx->getBrightness() < lastBrightness) {
        lgfx->setBrightness(lastBrightness);
        lastBrightness = lgfx->getBrightness();
    }

    if (!calibrating) {
        DisplayDriver::task_handler();
    }
}

template <class LGFX> void LGFXDriver<LGFX>::display_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    lv_draw_sw_rgb565_swap(px_map, w * h);

#ifdef TAB5_MUI
    if (lgfx->scale > 1 && lgfx->scaleX == lgfx->scale && lgfx->scaleY == lgfx->scale) {
        const uint16_t scale = lgfx->scale;
        const int32_t dstX = lgfx->offsetX + area->x1 * scale;
        const int32_t dstY = lgfx->offsetY + area->y1 * scale;
        const size_t scaledW = w * scale;
        static std::vector<uint16_t> scaledRow;
        if (scaledRow.size() < scaledW) {
            scaledRow.resize(scaledW);
        }

        auto *src = reinterpret_cast<uint16_t *>(px_map);
        lgfx->startWrite();
        for (uint32_t row = 0; row < h; ++row) {
            const uint16_t *srcRow = src + row * w;
            size_t out = 0;
            for (uint32_t col = 0; col < w; ++col) {
                const uint16_t pixel = srcRow[col];
                for (uint16_t copy = 0; copy < scale; ++copy) {
                    scaledRow[out++] = pixel;
                }
            }

            const int32_t physicalY = dstY + row * scale;
            for (uint16_t copyRow = 0; copyRow < scale; ++copyRow) {
                lgfx->pushImage(dstX, physicalY + copyRow, scaledW, 1, scaledRow.data());
            }
        }
        lgfx->endWrite();
    } else {
        auto &tab5Canvas = getTab5Canvas<LGFX>();
        if (tab5Canvas == nullptr) {
            lv_display_flush_ready(disp);
            return;
        }

        tab5Canvas->pushImage(area->x1, area->y1, w, h, reinterpret_cast<uint16_t *>(px_map));

        if (!lv_display_flush_is_last(disp)) {
            lv_display_flush_ready(disp);
            return;
        }

        lgfx->startWrite();
        if (lgfx->offsetX > 0) {
            lgfx->fillRect(0, 0, lgfx->offsetX, lgfx->physicalHeight, LGFX::color565(0x11, 0x16, 0x1A));
            lgfx->fillRect(lgfx->offsetX + lgfx->scaledWidth, 0,
                           lgfx->physicalWidth - (lgfx->offsetX + lgfx->scaledWidth), lgfx->physicalHeight,
                           LGFX::color565(0x11, 0x16, 0x1A));
            lgfx->drawFastVLine(lgfx->offsetX, 0, lgfx->physicalHeight, LGFX::color565(0x3D, 0xDA, 0x83));
            lgfx->drawFastVLine(lgfx->offsetX + lgfx->scaledWidth, 0, lgfx->physicalHeight, LGFX::color565(0x3D, 0xDA, 0x83));
        }
        tab5Canvas->pushRotateZoom(lgfx, lgfx->offsetX, lgfx->offsetY, 0.0f, lgfx->scaleX, lgfx->scaleY);
        lgfx->endWrite();
    }
#else
    lgfx->pushImage(area->x1, area->y1, w, h, reinterpret_cast<uint16_t *>(px_map));
#endif
    lv_display_flush_ready(disp);
}

#ifdef LGFX_AMOLED_ROUNDER
template <class LGFX> void LGFXDriver<LGFX>::rounder_cb(lv_event_t *e)
{
    lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
#if LGFX_AMOLED_ROUNDER == 1
    area->x1 &= ~1;
    area->y1 &= ~1;
    area->x2 |= 1;
    area->y2 |= 1;
#else
#error "LGFX_AMOLED_AMOLED requires implementation!"
#endif
}
#endif

template <class LGFX> void LGFXDriver<LGFX>::touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data)
{
    (void)indev_driver;
    uint16_t touchX = 0;
    uint16_t touchY = 0;
#ifdef CUSTOM_TOUCH_DRIVER
    bool touched = lgfx->getTouchXY(&touchX, &touchY);
#else
    bool touched = lgfx->getTouch(&touchX, &touchY);
#endif
    if (!touched) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }

#ifdef TAB5_MUI
    if (touchX < lgfx->offsetX || touchX >= lgfx->offsetX + lgfx->scaledWidth || touchY < lgfx->offsetY ||
        touchY >= lgfx->offsetY + lgfx->scaledHeight) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }
    touchX = static_cast<uint16_t>((touchX - lgfx->offsetX) / lgfx->scaleX);
    touchY = static_cast<uint16_t>((touchY - lgfx->offsetY) / lgfx->scaleY);
#endif

    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
}

template <class LGFX> void LGFXDriver<LGFX>::init(DeviceGUI *gui)
{
    ILOG_DEBUG("LGFXDriver<LGFX>::init...");
    init_lgfx();
    TFTDriver<LGFX>::init(gui);

    ILOG_DEBUG("LVGL display driver init...");
    DisplayDriver::display = lv_display_create(DisplayDriver::screenWidth, DisplayDriver::screenHeight);
    lv_display_set_color_format(this->display, LV_COLOR_FORMAT_RGB565);

#if defined(USE_DOUBLE_BUFFER)
    bufsize = screenWidth * screenHeight / 8 * sizeof(lv_color_t);
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
    ILOG_DEBUG("LVGL: allocating %u bytes PSRAM for double buffering"), bufsize;
    assert(ESP.getFreePsram());
    buf1 = (lv_color_t *)heap_caps_aligned_alloc(32, (bufsize + 3) & ~3, MALLOC_CAP_SPIRAM);
    draw_buf = (lv_disp_draw_buf_t *)heap_caps_aligned_alloc(32, sizeof(lv_disp_draw_buf_t), MALLOC_CAP_SPIRAM);
#else
    ILOG_DEBUG("LVGL: allocating %u bytes heap memory for double buffering"), bufsize;
    buf1 = (lv_color_t *)heap_caps_malloc(bufsize, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    buf2 = (lv_color_t *)heap_caps_malloc(bufsize, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
#endif
    assert(buf1 != 0);
    lv_display_set_buffers(disp, buf1, buf2, bufsize, LV_DISPLAY_RENDER_MODE_DIRECT);
#elif defined(BOARD_HAS_PSRAM)
    assert(ESP.getFreePsram());
#ifdef LGFX_BUFSIZE
    bufsize = LGFX_BUFSIZE;
#else
    bufsize = lgfx->screenWidth * lgfx->screenHeight * sizeof(lv_color_t) / 4;
#endif
    ILOG_DEBUG("LVGL: allocating %u bytes PSRAM for draw buffer", bufsize);
    buf1 = (lv_color_t *)LV_MEM_POOL_ALLOC(bufsize);
    assert(buf1 != 0);
    lv_display_set_buffers(this->display, buf1, buf2, bufsize, LV_DISPLAY_RENDER_MODE_PARTIAL);
#else
#ifdef LGFX_BUFSIZE
    bufsize = LGFX_BUFSIZE;
#else
    bufsize = lgfx->screenWidth * lgfx->screenHeight / 8;
#endif
    ILOG_DEBUG("LVGL: allocating %u bytes heap memory for draw buffer", sizeof(lv_color_t) * bufsize);
    buf1 = new lv_color_t[bufsize];
    assert(buf1 != 0);
    lv_display_set_buffers(this->display, buf1, buf2, sizeof(lv_color_t) * bufsize, LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif

    lv_display_set_flush_cb(this->display, LGFXDriver::display_flush);
#ifdef LGFX_AMOLED_ROUNDER
    lv_display_add_event_cb(this->display, rounder_cb, LV_EVENT_INVALIDATE_AREA, this->display);
#endif

#if defined(DISPLAY_SET_RESOLUTION)
    ILOG_DEBUG("Set display resolution: %dx%d", lgfx->screenWidth, lgfx->screenHeight);
    lv_display_set_resolution(this->display, lgfx->screenWidth, lgfx->screenHeight);
#endif

    if (hasTouch()) {
        DisplayDriver::touch = lv_indev_create();
        DisplayDriver::touch->gesture_limit = defaultGestureLimit;
        lv_indev_set_type(DisplayDriver::touch, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(DisplayDriver::touch, touchpad_read);
        lv_indev_set_display(DisplayDriver::touch, this->display);
        lv_indev_set_long_press_time(DisplayDriver::touch, defaultLongPressTime);
#ifdef USE_TOUCH_EVENTS
        if (lgfx->touch()->config()->pin_int > 0) {
            lv_indev_set_mode(DisplayDriver::touch, LV_INDEV_MODE_EVENT);
        }
#else
        lv_timer_t *timer = lv_indev_get_read_timer(DisplayDriver::touch);
        lv_timer_set_period(timer, 10);
#endif
    }
}

template <class LGFX> void LGFXDriver<LGFX>::init_lgfx(void)
{
    ILOG_DEBUG("LGFX init...");
    lgfx->init();
#ifdef TAB5_MUI
    lgfx->setRotation(3);
    lgfx->configureLayout();
#endif
    lgfx->setBrightness(defaultBrightness);
    lgfx->fillScreen(TFT_BLACK);

#ifdef TAB5_MUI
    auto &tab5Canvas = getTab5Canvas<LGFX>();
    if (lgfx->scale <= 1 && (tab5Canvas == nullptr || tab5Canvas->width() != lgfx->screenWidth || tab5Canvas->height() != lgfx->screenHeight)) {
        if (tab5Canvas != nullptr) {
            tab5Canvas->deleteSprite();
            delete tab5Canvas;
            tab5Canvas = nullptr;
        }
        tab5Canvas = new m5gfx::M5Canvas(lgfx);
        tab5Canvas->setColorDepth(16);
        tab5Canvas->createSprite(lgfx->screenWidth, lgfx->screenHeight);
        tab5Canvas->setPivot(0, 0);
        tab5Canvas->fillScreen(LGFX::color565(0x3D, 0xDA, 0x83));
    }
    if (lgfx->offsetX > 0) {
        lgfx->fillRect(0, 0, lgfx->offsetX, lgfx->physicalHeight, LGFX::color565(0x11, 0x16, 0x1A));
        lgfx->fillRect(lgfx->offsetX + lgfx->scaledWidth, 0, lgfx->physicalWidth - (lgfx->offsetX + lgfx->scaledWidth),
                       lgfx->physicalHeight, LGFX::color565(0x11, 0x16, 0x1A));
        lgfx->drawFastVLine(lgfx->offsetX, 0, lgfx->physicalHeight, LGFX::color565(0x3D, 0xDA, 0x83));
        lgfx->drawFastVLine(lgfx->offsetX + lgfx->scaledWidth, 0, lgfx->physicalHeight, LGFX::color565(0x3D, 0xDA, 0x83));
    }
    ILOG_INFO("Tab5 display rotated to landscape: physical=%ux%u scale=%u offset=(%u,%u)", lgfx->physicalWidth,
              lgfx->physicalHeight, lgfx->scale, lgfx->offsetX, lgfx->offsetY);
#endif

    if (hasTouch()) {
#ifndef CUSTOM_TOUCH_DRIVER
#ifdef CALIBRATE_TOUCH
        ILOG_INFO("Calibrating touch...");
#ifdef T_DECK
        uint16_t parameters[8] = {0, 2, 0, 314, 223, 5, 224, 314};
#elif defined(WT32_SC01)
        uint16_t parameters[8] = {0, 2, 0, 479, 319, 0, 319, 479};
#elif defined(T_HMI)
        uint16_t parameters[8] = {399, 293, 309, 3701, 3649, 266, 3678, 3689};
#elif defined(ESP32_2432S022)
        uint16_t parameters[8] = {1, 2, 69, 313, 187, 5, 239, 314};
#elif defined(ESP32_2432S028RV1)
        uint16_t parameters[8] = {278, 3651, 228, 173, 3819, 3648, 3815, 179};
#elif defined(UNPHONE)
        uint16_t parameters[8] = {222, 146, 241, 3812, 3860, 131, 3857, 3813};
#elif defined(NODEMCU_32S) || defined(ARCH_PORTDUINO)
        uint16_t parameters[8] = {255, 3691, 203, 198, 3836, 3659, 3795, 162};
#elif defined(SENSECAP_INDICATOR)
        uint16_t parameters[8] = {23, 3, 0, 479, 476, 2, 475, 479};
#else
        uint16_t parameters[8] = {0, 0, 0, 319, 239, 0, 239, 319};
        ILOG_WARN("Touch screen has no calibration data!!!");
#endif

#if CALIBRATE_TOUCH
        calibrate(parameters);
#else
        lgfx->setTouchCalibrate(parameters);
#endif
#endif
#endif
    }
}

template <class LGFX> bool LGFXDriver<LGFX>::calibrate(uint16_t parameters[8])
{
#ifndef CUSTOM_TOUCH_DRIVER
    if (parameters[0] || parameters[7]) {
        ILOG_DEBUG("setting touch screen calibration data");
        lgfx->setTouchCalibrate(parameters);
    } else {
        calibrating = true;
        std::uint16_t fg = TFT_BLUE;
        std::uint16_t bg = LGFX::color565(0x67, 0xEA, 0x94);
        lgfx->clearDisplay();
        lgfx->fillScreen(LGFX::color565(0x67, 0xEA, 0x94));
        lgfx->setTextSize(1);
        lgfx->setTextDatum(textdatum_t::middle_center);
        lgfx->setTextColor(fg, bg);
        lgfx->drawString("Tap the tip of the arrow marker.", lgfx->width() >> 1, lgfx->height() >> 1);
        lgfx->setTextDatum(textdatum_t::top_left);
        if (lgfx->isEPD())
            std::swap(fg, bg);
        lgfx->calibrateTouch(parameters, fg, bg, std::max(lgfx->width(), lgfx->height()) >> 3);
        calibrating = false;
    }
    ILOG_DEBUG("Touchscreen calibration parameters: {%d, %d, %d, %d, %d, %d, %d, %d}", parameters[0], parameters[1],
               parameters[2], parameters[3], parameters[4], parameters[5], parameters[6], parameters[7]);
#endif
    return true;
}

template <class LGFX> void LGFXDriver<LGFX>::setBrightness(uint8_t brightness)
{
#ifdef TAB5_MUI
    lgfx->setBrightness(brightness);
    lastBrightness = brightness;
#else
    lgfx->setBrightness(brightness);
    lastBrightness = brightness;
#endif
}

template <class LGFX> void LGFXDriver<LGFX>::printConfig(void)
{
    if (lgfx->panel()) {
        auto p = lgfx->panel();
        auto cfg = p->config();
        uint32_t id = p->readCommand(0x04, 0, 4);
        ILOG_DEBUG("Panel id=0x%08x (%dx%d): rst:%d, busy:%d, offX:%d, offY:%d invert:%d, RGB:%d, rotation:%d, offR:%d, read:%d, readP:%d, readB:%d, dlen:%d, colordepth:%d",
                   id, p->width(), p->height(), cfg.pin_rst, cfg.pin_busy, cfg.offset_x, cfg.offset_y, p->getInvert(),
                   cfg.rgb_order, (int)p->getRotation(), cfg.offset_rotation, cfg.readable, cfg.dummy_read_pixel,
                   cfg.dummy_read_bits, cfg.dlen_16bit, LV_COLOR_DEPTH);
    }
    if (lgfx->panel() && lgfx->panel()->getBus()) {
        lgfx::v1::bus_type_t type = lgfx->panel()->getBus()->busType();
        switch (type) {
        case lgfx::v1::bus_unknown:
            ILOG_DEBUG("Bus (unknown)");
            break;
        case lgfx::v1::bus_spi: {
            auto cfg = static_cast<lgfx::Bus_SPI *>(lgfx->panel()->getBus())->config();
#ifdef ARCH_PORTDUINO
            auto p = lgfx->panel();
            ILOG_DEBUG("Bus_SPI(%d): cs:%d, clk:%d, miso:%d, mosi:%d, dc:%d", cfg.spi_host, p->config().pin_cs, cfg.pin_sclk,
                       cfg.pin_miso, cfg.pin_mosi, cfg.pin_dc);
#else
            ILOG_DEBUG("Bus_SPI(%d): clk:%d, miso:%d, mosi:%d, dc:%d, 3wire:%d, dma:%d", cfg.spi_host, cfg.pin_sclk, cfg.pin_miso,
                       cfg.pin_mosi, cfg.pin_dc, cfg.spi_3wire, cfg.dma_channel);
#endif
            break;
        }
        case lgfx::v1::bus_i2c:
            ILOG_DEBUG("Bus_I2C");
            break;
#ifndef ARCH_PORTDUINO
        case lgfx::v1::bus_parallel8:
            ILOG_DEBUG("Bus_Parallel8");
            break;
        case lgfx::v1::bus_parallel16:
            ILOG_DEBUG("Bus_Parallel16");
            break;
#endif
        case lgfx::v1::bus_stream:
            ILOG_DEBUG("Bus (Stream)");
            break;
        case lgfx::v1::bus_image_push:
            ILOG_DEBUG("Bus (ImagePush)");
            break;
        default:
            break;
        }
    }

#ifndef CUSTOM_TOUCH_DRIVER
    if (lgfx->touch()) {
        auto cfg = lgfx->touch()->config();
        ILOG_DEBUG("Touch int:%d, rst:%d, rotation:%d, (%d/%d)-(%d/%d) ", cfg.pin_int, cfg.pin_rst, cfg.offset_rotation,
                   cfg.x_min, cfg.y_min, cfg.x_max, cfg.y_max);
        if (cfg.i2c_addr > 0 && cfg.pin_cs == -1) {
            ILOG_DEBUG("Touch I2C(%d:0x%02x): SCL:%d, SCA:%d, freq:%d ", (int)cfg.i2c_port, cfg.i2c_addr, cfg.pin_scl,
                       cfg.pin_sda, cfg.freq);
        } else {
            if (cfg.pin_cs == -1) {
                ILOG_DEBUG("Touch SPI(spidev%d.%d), clk:%d, mosi:%d, miso:%d ", (int)(cfg.spi_host & 0x0f),
                           (int)((cfg.spi_host & 0xf0) >> 4), cfg.pin_cs, cfg.pin_sclk, cfg.pin_mosi, cfg.pin_miso);
            } else {
                ILOG_DEBUG("Touch SPI(%d): cs:%d, clk:%d, mosi:%d, miso:%d ", (int)cfg.spi_host, cfg.pin_cs, cfg.pin_sclk,
                           cfg.pin_mosi, cfg.pin_miso);
            }
        }
    }
#endif
    if (lgfx->light()) {
        ILOG_DEBUG("BL pin assigned");
    }
}