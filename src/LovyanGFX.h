#pragma once

#ifdef TAB5_MUI
#include <M5GFX.h>

class LGFX_TAB5 : public m5gfx::M5GFX
{
  public:
  static constexpr uint16_t logicalWidth = 320;
  static constexpr uint16_t logicalHeight = 240;

  uint16_t screenWidth = logicalWidth;
  uint16_t screenHeight = logicalHeight;
  uint16_t physicalWidth = 0;
  uint16_t physicalHeight = 0;
  uint16_t scaledWidth = 0;
  uint16_t scaledHeight = 0;
  uint16_t offsetX = 0;
  uint16_t offsetY = 0;
  uint8_t scale = 1;
  float scaleX = 1.0f;
  float scaleY = 1.0f;

  void configureLayout()
  {
    physicalWidth = width();
    physicalHeight = height();

    scale = physicalHeight / logicalHeight;
    if (scale == 0) {
      scale = 1;
    }

    // Keep integer scaling for sharp text/icons and expand logical width to use horizontal space.
    screenHeight = logicalHeight;
    screenWidth = physicalWidth / scale;
    if (screenWidth < logicalWidth) {
      screenWidth = logicalWidth;
    }

    scaledWidth = screenWidth * scale;
    scaledHeight = screenHeight * scale;
    offsetX = (physicalWidth > scaledWidth) ? (physicalWidth - scaledWidth) / 2 : 0;
    offsetY = (physicalHeight > scaledHeight) ? (physicalHeight - scaledHeight) / 2 : 0;
    scaleX = static_cast<float>(scale);
    scaleY = static_cast<float>(scale);
  }

    bool hasButton(void) { return false; }
};
#else
#include_next <LovyanGFX.h>
#endif