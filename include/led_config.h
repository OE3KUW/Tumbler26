#pragma once

#include <Arduino.h>

namespace LedConfig {

constexpr uint8_t DATA_PIN = 15;
constexpr uint8_t LED_COUNT = 8;
constexpr uint32_t TIMER_INTERVAL_US = 1000000UL;
constexpr uint8_t BRIGHTNESS = 40;

}  // namespace LedConfig
