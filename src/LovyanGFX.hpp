#pragma once

#ifdef TAB5_MUI
#include <M5GFX.h>
#include <lgfx/v1/panel/Panel_Device.hpp>
#include <lgfx/v1/panel/Panel_ILI9342.hpp>
#include <lgfx/v1/panel/Panel_ST7735.hpp>
#include <lgfx/v1/panel/Panel_ST7789.hpp>
#include <lgfx/v1/touch/Touch_FT5x06.hpp>
#include <lgfx/v1/touch/Touch_GT911.hpp>

namespace lgfx
{
inline namespace v1
{
using Panel_ST7796 = Panel_ST7789;
using Panel_ILI9341 = Panel_ILI9342;
using Panel_ILI9486 = Panel_ST7789;
using Panel_ILI9488 = Panel_ST7789;
using Panel_HX8357D = Panel_ST7789;
using Panel_HX8357B = Panel_ST7789;

using Touch_XPT2046 = Touch_GT911;
using Touch_STMPE610 = Touch_GT911;
} // namespace v1
} // namespace lgfx
#else
#include_next <LovyanGFX.hpp>
#endif