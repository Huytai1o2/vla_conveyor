#include "dc_motor.h"

namespace {
constexpr uint8_t DRIVER_ADDRESS = 0x30;
constexpr uint8_t MOTOR_REGISTER = 0x00;
constexpr uint8_t MOTOR_M4 = 3;

constexpr uint8_t DIR_FORWARD = 0;
constexpr uint8_t DIR_BACKWARD = 1;
constexpr int MOTOR_SPEED = 100;

void setMotorSpeed(uint8_t motor, int speed) {
    speed = constrain(speed, -MOTOR_SPEED, MOTOR_SPEED);

    const uint8_t direction = speed < 0 ? DIR_BACKWARD : DIR_FORWARD;
    const uint16_t magnitude = abs(speed);

    Wire.beginTransmission(DRIVER_ADDRESS);
    Wire.write(MOTOR_REGISTER);
    Wire.write(motor);
    Wire.write(direction);
    Wire.write((magnitude >> 8) & 0xFF);
    Wire.write(magnitude & 0xFF);

    for (int i = 0; i < 4; ++i) {
        Wire.write(0);
    }

    const uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.printf("ERR,I2C,%u\n", error);
    }
}
}  // namespace

void task_motor(void *parameter) {
    Wire.begin(11, 12);
    setMotorSpeed(MOTOR_M4, 0);

    MotorState previous = MOTOR_STOP;

    while (true) {
        const MotorState current = motor_state;

        if (current != previous) {
            if (current == MOTOR_FORWARD) {
                setMotorSpeed(MOTOR_M4, MOTOR_SPEED);
            } else if (current == MOTOR_BACKWARD) {
                setMotorSpeed(MOTOR_M4, -MOTOR_SPEED);
            } else {
                setMotorSpeed(MOTOR_M4, 0);
            }

            previous = current;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
