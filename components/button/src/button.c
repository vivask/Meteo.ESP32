#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define BUTTON_DOWN (1)
#define BUTTON_UP (2)

const static char *TAG = "BUTTON";

static int button_pin = -1;
void (*cb_timer_shot)(int) = NULL;

static unsigned long millis()
{
  return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

static void button_task(void *pvParameter)
{
  int buttonState = BUTTON_UP;
  int count = 0;
  unsigned long lastDebounceTime = 0;
  unsigned long debounceDelay = 200;
  unsigned long timer_start = 0;

  while(1) {       
    int levelHIGH = gpio_get_level(button_pin);
    if ( (millis() - lastDebounceTime) > debounceDelay) {
      if( levelHIGH && buttonState == BUTTON_UP) {
        buttonState = BUTTON_DOWN;
        lastDebounceTime = millis();
        if ( timer_start == 0 ){
          timer_start = millis();
        }
        count++;
      }else {
        buttonState = BUTTON_UP;
      }
    }

    if ( timer_start && timer_start+CONFIG_PUSH_BUTTON_TIMER_MS <  millis() ) {
      (*cb_timer_shot)(count);
      timer_start = 0;
      count = 0;
    }

    vTaskDelay(1);
  }
} 

void button_init(int pin, void (*func_ptr)(int)) {
  button_pin = pin;
  cb_timer_shot = func_ptr;

  if ( button_pin < 0 ){
    ESP_LOGE(TAG, "Invalid pin: %d", pin);
    return;
  }

  if ( !cb_timer_shot ){
    ESP_LOGE(TAG, "Call back function undefined");
    return;
  }

  gpio_set_direction(button_pin, GPIO_MODE_INPUT);

  // Spawn a task to monitor the pins
  xTaskCreatePinnedToCore(button_task, "button_task", configMINIMAL_STACK_SIZE*CONFIG_PUSH_BUTTON_TASK_STACK_SIZE, NULL, CONFIG_PUSH_BUTTON_TASK_PRIORITY, NULL, 1);
}