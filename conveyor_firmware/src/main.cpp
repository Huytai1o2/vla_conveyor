#include "global.h"
#include "dc_motor.h"
#include "button.h"

namespace {
// Đổi thành false nếu lệnh RIGHT làm vật chạy sang trái.
constexpr bool IMAGE_RIGHT_IS_FORWARD = false;

constexpr uint32_t MIN_DURATION_MS = 50;
constexpr uint32_t MAX_DURATION_MS = 3000;

bool motorRunning = false;
uint32_t stopAtMs = 0;

void stopMotor() {
    motor_state = MOTOR_STOP;
    motorRunning = false;
}

void startMove(bool moveRight, uint32_t durationMs) {
    const bool forward = moveRight
        ? IMAGE_RIGHT_IS_FORWARD
        : !IMAGE_RIGHT_IS_FORWARD;

    motor_state = forward ? MOTOR_FORWARD : MOTOR_BACKWARD;
    stopAtMs = millis() + durationMs;
    motorRunning = true;
}

void handleCommand(String command) {
    command.trim();

    if (command == "PING") {
        Serial.println("PONG");
        return;
    }

    if (command == "STOP") {
        stopMotor();
        Serial.println("STOPPED");
        return;
    }

    char direction[8] = {0};
    unsigned long durationMs = 0;

    const int parsed = sscanf(
        command.c_str(),
        "MOVE,%7[^,],%lu",
        direction,
        &durationMs
    );

    if (parsed != 2) {
        Serial.println("ERR,BAD_FORMAT");
        return;
    }

    const bool moveRight = strcmp(direction, "RIGHT") == 0;
    const bool moveLeft = strcmp(direction, "LEFT") == 0;

    if (!moveRight && !moveLeft) {
        Serial.println("ERR,BAD_DIRECTION");
        return;
    }

    durationMs = constrain(
        durationMs,
        static_cast<unsigned long>(MIN_DURATION_MS),
        static_cast<unsigned long>(MAX_DURATION_MS)
    );

    startMove(moveRight, durationMs);
    Serial.printf("ACK,%s,%lu\n", direction, durationMs);
}
}  // namespace

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(40);
    delay(1000);

    xTaskCreate(
        task_motor,
        "motor",
        2048,
        nullptr,
        2,
        nullptr
    );

    xTaskCreate(task_button, "button",2048,nullptr,2,nullptr);

    Serial.println("READY");
}

void loop() {
    if (
        motorRunning
        && static_cast<int32_t>(millis() - stopAtMs) >= 0
    ) {
        stopMotor();
        Serial.println("DONE");
    }

    if (Serial.available() > 0) {
        handleCommand(Serial.readStringUntil('\n'));
    }

    delay(2);
}
