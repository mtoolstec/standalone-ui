#pragma once

#include <cstdint>

void tab5AudioInit();
void tab5PlayNotificationTone(uint8_t ringtoneId);
void tab5PlayNotificationRtttl(const char *rtttl);
void tab5AudioTaskHandler();