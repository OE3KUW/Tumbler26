#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "led_config.h"

namespace {

Adafruit_NeoPixel leds(
    LedConfig::LED_COUNT,
    LedConfig::DATA_PIN,
    NEO_GRB + NEO_KHZ800);

hw_timer_t* secondTimer = nullptr;
volatile uint32_t elapsedSeconds = 0;
uint32_t displayedSecond = 0;

void IRAM_ATTR onSecondTimer() {
    ++elapsedSeconds;
}

void showColors() {
    constexpr uint8_t colors[LedConfig::LED_COUNT][3] = {
        {255,   0,   0},  // LED 1: Rot
        {255,  80,   0},  // LED 2: Orange
        {255, 255,   0},  // LED 3: Gelb
        {  0, 255,   0},  // LED 4: Gruen
        {  0, 255, 255},  // LED 5: Cyan
        {  0,   0, 255},  // LED 6: Blau
        {128,   0, 255},  // LED 7: Violett
        {255,   0, 128},  // LED 8: Pink
    };

    for (uint8_t index = 0; index < LedConfig::LED_COUNT; ++index) {
        leds.setPixelColor(
            index,
            leds.Color(colors[index][0], colors[index][1], colors[index][2]));
    }
    leds.show();
}

void switchOff() {
    leds.clear();
    leds.show();
}

void startSecondTimer() {
    // 80 MHz / 80 = 1 MHz: ein Timer-Tick entspricht einer Mikrosekunde.
    secondTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(secondTimer, &onSecondTimer, true);
    timerAlarmWrite(secondTimer, LedConfig::TIMER_INTERVAL_US, true);
    timerAlarmEnable(secondTimer);
}

}  // namespace

void setup() {
    Serial.begin(115200);

    leds.begin();
    leds.setBrightness(LedConfig::BRIGHTNESS);
    showColors();

    startSecondTimer();
    Serial.println("Tumbler26 gestartet: 8 RGB-LEDs an GPIO 15");
}

void loop() {
    const uint32_t currentSecond = elapsedSeconds;
    if (currentSecond == displayedSecond) {
        return;
    }

    displayedSecond = currentSecond;
    if ((currentSecond & 1U) == 0U) {
        showColors();
    } else {
        switchOff();
    }
}

