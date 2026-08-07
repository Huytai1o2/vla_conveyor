#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum MotorState {
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
};

extern volatile MotorState motor_state;

#endif
