#include "button.h"

MotorState readButtonState(){
    bool forward = digitalRead(MotorForward_GPIO) == LOW;
    bool backward = digitalRead(MotorBackward_GPIO) == LOW;

    if(forward && !backward) return MOTOR_FORWARD;
    if(backward && !forward) return MOTOR_BACKWARD;

    return MOTOR_STOP;
}

// void task_button(void *pvParameters){
//     pinMode(MotorForward_GPIO, INPUT_PULLUP);
//     pinMode(MotorBackward_GPIO, INPUT_PULLUP);

//     MotorState current_state = MOTOR_STOP;

//     while(1){
//         MotorState new_state = readButtonState();
//         if(new_state != current_state){
//             vTaskDelay(pdMS_TO_TICKS(50));
//             if(readButtonState() == new_state){
//                 current_state = new_state;
//                 motor_state = new_state;
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

void task_button(void *pvParameters){
    pinMode(MotorForward_GPIO, INPUT_PULLUP);
    pinMode(MotorBackward_GPIO, INPUT_PULLUP);

    MotorState current_state = MOTOR_STOP;
    unsigned long start_time = 0;       // Lưu thời điểm bắt đầu nhấn nút
    bool timeout_triggered = false;     // Cờ đánh dấu đã giữ quá 5s

    while(1){
        MotorState new_state = readButtonState();
        
        // 1. Xử lý khi người dùng nhấn/nhả nút (có chống nhiễu)
        if(new_state != current_state){
            vTaskDelay(pdMS_TO_TICKS(50));
            if(readButtonState() == new_state){
                current_state = new_state;
                
                if (current_state != MOTOR_STOP) {
                    // Bắt đầu NHẤN nút (Tiến hoặc Lùi)
                    start_time = millis();       // Ghi nhận thời gian bắt đầu
                    timeout_triggered = false;   // Đặt lại cờ timeout
                    motor_state = current_state; // Báo cho task_motor chạy

                    if (current_state == MOTOR_FORWARD) {
                        Serial.println("BUTTON,FORWARD,PRESSED");
                    } else {
                        Serial.println("BUTTON,BACKWARD,PRESSED");
                    }
                } else {
                    // Đã NHẢ nút
                    timeout_triggered = false;   // Cho phép nhấn lại
                    motor_state = MOTOR_STOP;    // Báo cho task_motor dừng
                    Serial.println("BUTTON,RELEASED");
                }
            }
        }

        // 2. Kiểm tra thời gian nếu đang giữ nút (chạy cả khi Tiến hoặc Lùi)
        if (current_state != MOTOR_STOP && !timeout_triggered) {
            if (millis() - start_time >= 20000) {
                motor_state = MOTOR_STOP;  // Ép trạng thái dừng động cơ
                timeout_triggered = true;  // Bật cờ để không chạy lại cho đến khi nhả nút
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
