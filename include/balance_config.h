#pragma once

#include <Arduino.h>

namespace BalanceConfig {

constexpr uint32_t CONTROL_PERIOD_US = 5000UL;  // 200 Hz
constexpr uint8_t SPEED_CONTROL_DIVIDER = 8;    // 25 Hz

// Starting values from the proven AVR Tumbller controller.
constexpr float KP_BALANCE = 55.0F;
constexpr float KD_BALANCE = 0.75F;
constexpr float KP_SPEED = 10.0F;
constexpr float KI_SPEED = 0.26F;
constexpr float KD_YAW = 0.5F;

constexpr float ANGLE_ZERO_DEG = 0.0F;
constexpr float ARM_ANGLE_DEG = 7.0F;
constexpr float FALL_ANGLE_DEG = 22.0F;
constexpr uint16_t ARM_STABLE_CYCLES = 200;  // one second at 200 Hz

constexpr int16_t MAX_MOTOR_PWM = 255;
constexpr float MAX_SPEED_INTEGRAL = 3000.0F;

constexpr uint16_t GYRO_CALIBRATION_SAMPLES = 400;
constexpr uint8_t MAX_CONSECUTIVE_IMU_ERRORS = 3;
constexpr uint8_t MAX_TIMER_NOTIFICATIONS = 2;

// The original controller uses the sign of the command until encoder polarity is known.
constexpr int16_t ENCODER_LEARN_MIN_PWM = 60;
constexpr int32_t ENCODER_LEARN_MIN_TICKS = 2;
constexpr uint8_t ENCODER_LEARN_STABLE_PERIODS = 2;
constexpr uint8_t ENCODER_LEARN_EVIDENCE = 3;

}  // namespace BalanceConfig
