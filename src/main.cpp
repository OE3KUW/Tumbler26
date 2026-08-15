#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <driver/gpio.h>

#include "balance_config.h"
#include "hardware_config.h"

namespace {

constexpr uint8_t MPU_REG_SMPLRT_DIV = 0x19;
constexpr uint8_t MPU_REG_CONFIG = 0x1A;
constexpr uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU_REG_WHO_AM_I = 0x75;

constexpr float GYRO_SCALE_LSB_PER_DPS = 131.0F;
constexpr float RAD_TO_DEG_F = 57.2957795F;

Adafruit_NeoPixel rgbLeds(
    HardwareConfig::RGB_LED_COUNT,
    HardwareConfig::RGB_PIN,
    NEO_GRB + NEO_KHZ800);

TaskHandle_t controlTaskHandle = nullptr;
hw_timer_t* controlTimer = nullptr;
volatile int32_t leftEncoderTicks = 0;
volatile int32_t rightEncoderTicks = 0;

struct ImuSample {
    float accelerometerAngleDeg;
    float gyroXDegPerSecond;
    float gyroZDegPerSecond;
};

struct Telemetry {
    float angleDeg;
    float gyroXDegPerSecond;
    float speedFiltered;
    int16_t leftPwm;
    int16_t rightPwm;
    int32_t leftTicks;
    int32_t rightTicks;
    int8_t leftEncoderPolarity;
    int8_t rightEncoderPolarity;
    bool armed;
    bool imuOk;
    uint32_t missedPeriods;
};

Telemetry telemetry = {};
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;

float gyroXOffsetDegPerSecond = 0.0F;
bool mpuReady = false;
int8_t leftMotorDirection = 0;
int8_t rightMotorDirection = 0;

class AngleKalmanFilter {
  public:
    void reset(float initialAngleDeg) {
        angleDeg_ = initialAngleDeg;
        bias_ = 0.0F;
        p00_ = 1.0F;
        p01_ = 0.0F;
        p10_ = 0.0F;
        p11_ = 1.0F;
    }

    float update(float measuredAngleDeg, float gyroRateDegPerSecond, float dtSeconds) {
        constexpr float qAngle = 0.001F;
        constexpr float qBias = 0.005F;
        constexpr float rMeasure = 0.5F;

        const float unbiasedRate = gyroRateDegPerSecond - bias_;
        angleDeg_ += dtSeconds * unbiasedRate;

        p00_ += dtSeconds * (dtSeconds * p11_ - p01_ - p10_ + qAngle);
        p01_ -= dtSeconds * p11_;
        p10_ -= dtSeconds * p11_;
        p11_ += qBias * dtSeconds;

        const float innovationCovariance = p00_ + rMeasure;
        const float gain0 = p00_ / innovationCovariance;
        const float gain1 = p10_ / innovationCovariance;
        const float innovation = measuredAngleDeg - angleDeg_;

        angleDeg_ += gain0 * innovation;
        bias_ += gain1 * innovation;

        const float p00Previous = p00_;
        const float p01Previous = p01_;
        p00_ -= gain0 * p00Previous;
        p01_ -= gain0 * p01Previous;
        p10_ -= gain1 * p00Previous;
        p11_ -= gain1 * p01Previous;

        return angleDeg_;
    }

  private:
    float angleDeg_ = 0.0F;
    float bias_ = 0.0F;
    float p00_ = 1.0F;
    float p01_ = 0.0F;
    float p10_ = 0.0F;
    float p11_ = 1.0F;
};

class EncoderPolarityEstimator {
  public:
    int32_t normalize(int32_t rawDelta, int16_t motorCommand) {
        const int8_t commandSign = signOf(motorCommand);
        if (commandSign == previousCommandSign_ && commandSign != 0) {
            if (stablePeriods_ < 255) {
                ++stablePeriods_;
            }
        } else {
            previousCommandSign_ = commandSign;
            stablePeriods_ = 0;
            evidence_ = 0;
            candidatePolarity_ = 0;
        }

        const int32_t magnitude = rawDelta < 0 ? -rawDelta : rawDelta;
        const int16_t commandMagnitude = motorCommand < 0 ? -motorCommand : motorCommand;

        if (polarity_ == 0 &&
            stablePeriods_ >= BalanceConfig::ENCODER_LEARN_STABLE_PERIODS &&
            commandMagnitude >= BalanceConfig::ENCODER_LEARN_MIN_PWM &&
            magnitude >= BalanceConfig::ENCODER_LEARN_MIN_TICKS) {
            const int8_t observedPolarity = signOf(rawDelta) * commandSign;
            if (observedPolarity == candidatePolarity_) {
                if (evidence_ < 255) {
                    ++evidence_;
                }
            } else {
                candidatePolarity_ = observedPolarity;
                evidence_ = 1;
            }

            if (evidence_ >= BalanceConfig::ENCODER_LEARN_EVIDENCE) {
                polarity_ = candidatePolarity_;
            }
        }

        if (polarity_ != 0) {
            return rawDelta * polarity_;
        }

        // Proven AVR fallback: infer direction from the command until polarity is learned.
        return magnitude * commandSign;
    }

    int8_t polarity() const {
        return polarity_;
    }

  private:
    static int8_t signOf(int32_t value) {
        return value > 0 ? 1 : (value < 0 ? -1 : 0);
    }

    int8_t polarity_ = 0;
    int8_t candidatePolarity_ = 0;
    int8_t previousCommandSign_ = 0;
    uint8_t stablePeriods_ = 0;
    uint8_t evidence_ = 0;
};

int16_t clampMotorPwm(float value) {
    if (value > BalanceConfig::MAX_MOTOR_PWM) {
        return BalanceConfig::MAX_MOTOR_PWM;
    }
    if (value < -BalanceConfig::MAX_MOTOR_PWM) {
        return -BalanceConfig::MAX_MOTOR_PWM;
    }
    return static_cast<int16_t>(lroundf(value));
}

float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

bool writeMpuRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(HardwareConfig::MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
}

bool readMpuRegisters(uint8_t firstRegister, uint8_t* destination, size_t length) {
    Wire.beginTransmission(HardwareConfig::MPU6050_ADDRESS);
    Wire.write(firstRegister);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    const size_t received = Wire.requestFrom(
        static_cast<int>(HardwareConfig::MPU6050_ADDRESS),
        static_cast<int>(length),
        static_cast<int>(true));
    if (received != length) {
        return false;
    }

    for (size_t index = 0; index < length; ++index) {
        destination[index] = Wire.read();
    }
    return true;
}

bool initializeMpu() {
    Wire.begin(HardwareConfig::I2C_SDA_PIN, HardwareConfig::I2C_SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeOut(5);
    delay(100);

    uint8_t identity = 0;
    if (!readMpuRegisters(MPU_REG_WHO_AM_I, &identity, 1) ||
        (identity != 0x68 && identity != 0x69)) {
        return false;
    }

    if (!writeMpuRegister(MPU_REG_PWR_MGMT_1, 0x80)) {
        return false;
    }
    delay(100);

    return writeMpuRegister(MPU_REG_PWR_MGMT_1, 0x01) &&
           writeMpuRegister(MPU_REG_CONFIG, 0x03) &&
           writeMpuRegister(MPU_REG_SMPLRT_DIV, 0x04) &&
           writeMpuRegister(MPU_REG_GYRO_CONFIG, 0x00) &&
           writeMpuRegister(MPU_REG_ACCEL_CONFIG, 0x00);
}

bool readImu(ImuSample& sample) {
    uint8_t data[14] = {};
    if (!readMpuRegisters(MPU_REG_ACCEL_XOUT_H, data, sizeof(data))) {
        return false;
    }

    const int16_t accelerationY = static_cast<int16_t>((data[2] << 8) | data[3]);
    const int16_t accelerationZ = static_cast<int16_t>((data[4] << 8) | data[5]);
    const int16_t gyroX = static_cast<int16_t>((data[8] << 8) | data[9]);
    const int16_t gyroZ = static_cast<int16_t>((data[12] << 8) | data[13]);

    sample.accelerometerAngleDeg =
        atan2f(static_cast<float>(accelerationY), static_cast<float>(accelerationZ)) *
        RAD_TO_DEG_F;
    sample.gyroXDegPerSecond =
        static_cast<float>(gyroX) / GYRO_SCALE_LSB_PER_DPS - gyroXOffsetDegPerSecond;
    sample.gyroZDegPerSecond = static_cast<float>(gyroZ) / GYRO_SCALE_LSB_PER_DPS;
    return true;
}

bool calibrateGyroscope() {
    float gyroSum = 0.0F;
    uint16_t validSamples = 0;

    for (uint16_t index = 0; index < BalanceConfig::GYRO_CALIBRATION_SAMPLES; ++index) {
        uint8_t data[14] = {};
        if (readMpuRegisters(MPU_REG_ACCEL_XOUT_H, data, sizeof(data))) {
            const int16_t gyroX = static_cast<int16_t>((data[8] << 8) | data[9]);
            gyroSum += static_cast<float>(gyroX) / GYRO_SCALE_LSB_PER_DPS;
            ++validSamples;
        }
        delay(5);
    }

    if (validSamples < BalanceConfig::GYRO_CALIBRATION_SAMPLES * 9U / 10U) {
        return false;
    }

    gyroXOffsetDegPerSecond = gyroSum / validSamples;
    return true;
}

void showIdentificationLed() {
    rgbLeds.begin();
    rgbLeds.setBrightness(HardwareConfig::IDENTIFICATION_LED_BRIGHTNESS);
    rgbLeds.clear();
    rgbLeds.setPixelColor(
        HardwareConfig::IDENTIFICATION_LED_INDEX,
        rgbLeds.Color(255, 0, 255));
    rgbLeds.show();
}

void initializeMotorDriver() {
    pinMode(HardwareConfig::LEFT_DIR_PIN, OUTPUT);
    pinMode(HardwareConfig::RIGHT_DIR_PIN, OUTPUT);
    pinMode(HardwareConfig::MOTOR_STANDBY_PIN, OUTPUT);

    digitalWrite(HardwareConfig::LEFT_DIR_PIN, LOW);
    digitalWrite(HardwareConfig::RIGHT_DIR_PIN, LOW);
    digitalWrite(HardwareConfig::MOTOR_STANDBY_PIN, LOW);

    ledcSetup(
        HardwareConfig::LEFT_PWM_CHANNEL,
        HardwareConfig::MOTOR_PWM_FREQUENCY_HZ,
        HardwareConfig::MOTOR_PWM_RESOLUTION_BITS);
    ledcSetup(
        HardwareConfig::RIGHT_PWM_CHANNEL,
        HardwareConfig::MOTOR_PWM_FREQUENCY_HZ,
        HardwareConfig::MOTOR_PWM_RESOLUTION_BITS);
    ledcAttachPin(HardwareConfig::LEFT_PWM_PIN, HardwareConfig::LEFT_PWM_CHANNEL);
    ledcAttachPin(HardwareConfig::RIGHT_PWM_PIN, HardwareConfig::RIGHT_PWM_CHANNEL);
    ledcWrite(HardwareConfig::LEFT_PWM_CHANNEL, 0);
    ledcWrite(HardwareConfig::RIGHT_PWM_CHANNEL, 0);
}

void disableMotors() {
    ledcWrite(HardwareConfig::LEFT_PWM_CHANNEL, 0);
    ledcWrite(HardwareConfig::RIGHT_PWM_CHANNEL, 0);
    digitalWrite(HardwareConfig::MOTOR_STANDBY_PIN, LOW);
    digitalWrite(HardwareConfig::LEFT_DIR_PIN, LOW);
    digitalWrite(HardwareConfig::RIGHT_DIR_PIN, LOW);
    leftMotorDirection = 0;
    rightMotorDirection = 0;
}

void enableMotors() {
    ledcWrite(HardwareConfig::LEFT_PWM_CHANNEL, 0);
    ledcWrite(HardwareConfig::RIGHT_PWM_CHANNEL, 0);
    digitalWrite(HardwareConfig::MOTOR_STANDBY_PIN, HIGH);
}

void writeMotor(
    uint8_t directionPin,
    uint8_t pwmChannel,
    int16_t command,
    int8_t& previousDirection) {
    const int8_t newDirection = command > 0 ? 1 : (command < 0 ? -1 : 0);
    if (newDirection == 0) {
        ledcWrite(pwmChannel, 0);
        return;
    }

    if (newDirection != previousDirection) {
        ledcWrite(pwmChannel, 0);
        digitalWrite(directionPin, newDirection < 0 ? HIGH : LOW);
        previousDirection = newDirection;
    }
    ledcWrite(pwmChannel, static_cast<uint8_t>(command < 0 ? -command : command));
}

void writeMotors(int16_t leftCommand, int16_t rightCommand) {
    writeMotor(
        HardwareConfig::LEFT_DIR_PIN,
        HardwareConfig::LEFT_PWM_CHANNEL,
        leftCommand,
        leftMotorDirection);
    writeMotor(
        HardwareConfig::RIGHT_DIR_PIN,
        HardwareConfig::RIGHT_PWM_CHANNEL,
        rightCommand,
        rightMotorDirection);
}

void IRAM_ATTR onLeftEncoderA() {
    const int levelA = gpio_get_level(
        static_cast<gpio_num_t>(HardwareConfig::LEFT_ENCODER_A_PIN));
    const int levelB = gpio_get_level(
        static_cast<gpio_num_t>(HardwareConfig::LEFT_ENCODER_B_PIN));
    leftEncoderTicks += levelA == levelB ? 1 : -1;
}

void IRAM_ATTR onRightEncoderA() {
    const int levelA = gpio_get_level(
        static_cast<gpio_num_t>(HardwareConfig::RIGHT_ENCODER_A_PIN));
    const int levelB = gpio_get_level(
        static_cast<gpio_num_t>(HardwareConfig::RIGHT_ENCODER_B_PIN));
    rightEncoderTicks += levelA == levelB ? 1 : -1;
}

void initializeEncoders() {
    pinMode(HardwareConfig::LEFT_ENCODER_A_PIN, INPUT);
    pinMode(HardwareConfig::LEFT_ENCODER_B_PIN, INPUT);
    pinMode(HardwareConfig::RIGHT_ENCODER_A_PIN, INPUT);
    pinMode(HardwareConfig::RIGHT_ENCODER_B_PIN, INPUT);
    attachInterrupt(
        digitalPinToInterrupt(HardwareConfig::LEFT_ENCODER_A_PIN),
        onLeftEncoderA,
        CHANGE);
    attachInterrupt(
        digitalPinToInterrupt(HardwareConfig::RIGHT_ENCODER_A_PIN),
        onRightEncoderA,
        CHANGE);
}

void publishTelemetry(const Telemetry& newTelemetry) {
    portENTER_CRITICAL(&telemetryMux);
    telemetry = newTelemetry;
    portEXIT_CRITICAL(&telemetryMux);
}

void IRAM_ATTR onControlTimer() {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    if (controlTaskHandle != nullptr) {
        vTaskNotifyGiveFromISR(controlTaskHandle, &higherPriorityTaskWoken);
    }
    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void controlTask(void*) {
    AngleKalmanFilter angleFilter;
    EncoderPolarityEstimator leftPolarity;
    EncoderPolarityEstimator rightPolarity;

    ImuSample sample = {};
    if (!readImu(sample)) {
        disableMotors();
        mpuReady = false;
        controlTaskHandle = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    angleFilter.reset(sample.accelerometerAngleDeg);

    uint32_t previousMicros = micros();
    uint32_t missedPeriods = 0;
    uint8_t imuErrors = 0;
    uint8_t speedPeriodCounter = 0;
    uint16_t stableArmCycles = 0;
    int32_t previousLeftTicks = leftEncoderTicks;
    int32_t previousRightTicks = rightEncoderTicks;
    int16_t leftPwm = 0;
    int16_t rightPwm = 0;
    float speedFiltered = 0.0F;
    float speedIntegral = 0.0F;
    float speedControlOutput = 0.0F;
    float yawControlOutput = 0.0F;
    bool armed = false;

    for (;;) {
        const uint32_t timerNotifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const uint32_t nowMicros = micros();
        float dtSeconds = static_cast<float>(nowMicros - previousMicros) / 1000000.0F;
        previousMicros = nowMicros;
        dtSeconds = clampFloat(dtSeconds, 0.003F, 0.015F);

        if (timerNotifications > 1) {
            missedPeriods += timerNotifications - 1;
        }
        if (timerNotifications > BalanceConfig::MAX_TIMER_NOTIFICATIONS) {
            armed = false;
            stableArmCycles = 0;
            disableMotors();
        }

        if (!readImu(sample)) {
            if (imuErrors < 255) {
                ++imuErrors;
            }
            if (imuErrors >= BalanceConfig::MAX_CONSECUTIVE_IMU_ERRORS) {
                armed = false;
                stableArmCycles = 0;
                disableMotors();
            }
            continue;
        }
        imuErrors = 0;

        const float angleDeg = angleFilter.update(
            sample.accelerometerAngleDeg,
            sample.gyroXDegPerSecond,
            dtSeconds);

        if (!armed) {
            leftPwm = 0;
            rightPwm = 0;
            speedFiltered = 0.0F;
            speedIntegral = 0.0F;
            speedControlOutput = 0.0F;
            yawControlOutput = 0.0F;

            if (fabsf(angleDeg - BalanceConfig::ANGLE_ZERO_DEG) <=
                BalanceConfig::ARM_ANGLE_DEG) {
                if (stableArmCycles < BalanceConfig::ARM_STABLE_CYCLES) {
                    ++stableArmCycles;
                }
            } else {
                stableArmCycles = 0;
            }

            if (stableArmCycles >= BalanceConfig::ARM_STABLE_CYCLES) {
                previousLeftTicks = leftEncoderTicks;
                previousRightTicks = rightEncoderTicks;
                speedPeriodCounter = 0;
                enableMotors();
                armed = true;
            }
        } else if (fabsf(angleDeg - BalanceConfig::ANGLE_ZERO_DEG) >
                   BalanceConfig::FALL_ANGLE_DEG) {
            armed = false;
            stableArmCycles = 0;
            leftPwm = 0;
            rightPwm = 0;
            disableMotors();
        } else {
            ++speedPeriodCounter;
            if (speedPeriodCounter >= BalanceConfig::SPEED_CONTROL_DIVIDER) {
                speedPeriodCounter = 0;

                const int32_t currentLeftTicks = leftEncoderTicks;
                const int32_t currentRightTicks = rightEncoderTicks;
                const int32_t rawLeftDelta = currentLeftTicks - previousLeftTicks;
                const int32_t rawRightDelta = currentRightTicks - previousRightTicks;
                previousLeftTicks = currentLeftTicks;
                previousRightTicks = currentRightTicks;

                const int32_t leftDelta = leftPolarity.normalize(rawLeftDelta, leftPwm);
                const int32_t rightDelta = rightPolarity.normalize(rawRightDelta, rightPwm);
                const float carSpeed = 0.5F * static_cast<float>(leftDelta + rightDelta);

                speedFiltered = speedFiltered * 0.7F + carSpeed * 0.3F;
                speedIntegral += speedFiltered;
                speedIntegral = clampFloat(
                    speedIntegral,
                    -BalanceConfig::MAX_SPEED_INTEGRAL,
                    BalanceConfig::MAX_SPEED_INTEGRAL);
                speedControlOutput =
                    -BalanceConfig::KP_SPEED * speedFiltered -
                    BalanceConfig::KI_SPEED * speedIntegral;
                yawControlOutput = BalanceConfig::KD_YAW * sample.gyroZDegPerSecond;
            }

            const float balanceControlOutput =
                BalanceConfig::KP_BALANCE *
                    (angleDeg - BalanceConfig::ANGLE_ZERO_DEG) +
                BalanceConfig::KD_BALANCE * sample.gyroXDegPerSecond;

            leftPwm = clampMotorPwm(
                balanceControlOutput - speedControlOutput - yawControlOutput);
            rightPwm = clampMotorPwm(
                balanceControlOutput - speedControlOutput + yawControlOutput);
            writeMotors(leftPwm, rightPwm);
        }

        Telemetry currentTelemetry = {};
        currentTelemetry.angleDeg = angleDeg;
        currentTelemetry.gyroXDegPerSecond = sample.gyroXDegPerSecond;
        currentTelemetry.speedFiltered = speedFiltered;
        currentTelemetry.leftPwm = leftPwm;
        currentTelemetry.rightPwm = rightPwm;
        currentTelemetry.leftTicks = leftEncoderTicks;
        currentTelemetry.rightTicks = rightEncoderTicks;
        currentTelemetry.leftEncoderPolarity = leftPolarity.polarity();
        currentTelemetry.rightEncoderPolarity = rightPolarity.polarity();
        currentTelemetry.armed = armed;
        currentTelemetry.imuOk = imuErrors == 0;
        currentTelemetry.missedPeriods = missedPeriods;
        publishTelemetry(currentTelemetry);
    }
}

bool startControlSystem() {
    const BaseType_t taskResult = xTaskCreatePinnedToCore(
        controlTask,
        "balance-control",
        4096,
        nullptr,
        configMAX_PRIORITIES - 2,
        &controlTaskHandle,
        1);
    if (taskResult != pdPASS) {
        controlTaskHandle = nullptr;
        return false;
    }

    controlTimer = timerBegin(0, 80, true);
    if (controlTimer == nullptr) {
        vTaskDelete(controlTaskHandle);
        controlTaskHandle = nullptr;
        return false;
    }
    timerAttachInterrupt(controlTimer, &onControlTimer, true);
    timerAlarmWrite(controlTimer, BalanceConfig::CONTROL_PERIOD_US, true);
    timerAlarmEnable(controlTimer);
    return true;
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(300);

    showIdentificationLed();
    initializeMotorDriver();
    initializeEncoders();

    Serial.println();
    Serial.println("Tumbler26 Balance-Firmware");
    Serial.println("LED 7 = Magenta, alle anderen RGB-LEDs aus");
    Serial.println("Motoren bleiben waehrend der MPU-Kalibrierung deaktiviert.");

    mpuReady = initializeMpu();
    if (!mpuReady) {
        Serial.println("FEHLER: MPU6050 nicht gefunden. Motoren bleiben aus.");
        return;
    }

    Serial.println("MPU-Kalibrierung: Roboter zwei Sekunden ruhig halten.");
    mpuReady = calibrateGyroscope();
    if (!mpuReady) {
        Serial.println("FEHLER: MPU-Kalibrierung fehlgeschlagen. Motoren bleiben aus.");
        return;
    }

    if (!startControlSystem()) {
        mpuReady = false;
        disableMotors();
        Serial.println("FEHLER: Regel-Task konnte nicht gestartet werden.");
        return;
    }

    Serial.printf("Gyro-X-Offset: %.3f deg/s\n", gyroXOffsetDegPerSecond);
    Serial.println("Zum Aktivieren den Roboter eine Sekunde aufrecht halten.");
}

void loop() {
    static uint32_t previousPrintMillis = 0;
    if (millis() - previousPrintMillis < 100) {
        delay(5);
        return;
    }
    previousPrintMillis = millis();

    if (!mpuReady) {
        Serial.println("Regelung inaktiv: MPU/Initialisierung nicht bereit.");
        return;
    }

    Telemetry snapshot = {};
    portENTER_CRITICAL(&telemetryMux);
    snapshot = telemetry;
    portEXIT_CRITICAL(&telemetryMux);

    Serial.printf(
        "state=%s angle=%7.2f gyro=%7.2f speed=%7.2f pwmL=%4d pwmR=%4d "
        "encL=%ld encR=%ld polL=%d polR=%d missed=%lu\n",
        snapshot.armed ? "BALANCE" : "SAFE",
        snapshot.angleDeg,
        snapshot.gyroXDegPerSecond,
        snapshot.speedFiltered,
        snapshot.leftPwm,
        snapshot.rightPwm,
        static_cast<long>(snapshot.leftTicks),
        static_cast<long>(snapshot.rightTicks),
        snapshot.leftEncoderPolarity,
        snapshot.rightEncoderPolarity,
        static_cast<unsigned long>(snapshot.missedPeriods));
}
