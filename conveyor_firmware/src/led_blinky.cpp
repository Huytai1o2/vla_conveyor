#include "led_blinky.h"

void led_blinky(void *pvParameters){
    pinMode(LED_GPIO, OUTPUT);

    while(1){
        Serial.println("Blinky");
        digitalWrite(LED_GPIO, HIGH); 
        vTaskDelay(5000);
        digitalWrite(LED_GPIO, LOW); 
        vTaskDelay(5000);
    }
}