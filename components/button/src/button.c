#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <freertos/timers.h>
#include "esp_timer.h"

#define BUTTON_DOWN (1)
#define BUTTON_UP (2)

const static char *TAG = "BUTTON";

TimerHandle_t push_button_timer = NULL;

static int count = 0;
static long lastDebounceTime = 0;
static long debounceDelay = 200;
static int button_pin = -1;

void (*cb_timer_shot)(int) = NULL;

static void oneshot_timer_callback(TimerHandle_t xTimer)
{
	xTimerStop( xTimer, (TickType_t) 0 );
  (*cb_timer_shot)(count);
  count = 0;
}

unsigned long millis()
{
  return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

static void button_task(void *pvParameter)
{
  gpio_set_direction(button_pin, GPIO_MODE_INPUT);

  push_button_timer = xTimerCreate( NULL, pdMS_TO_TICKS(CONFIG_PUSH_BUTTON_TIMER_MS), pdFALSE, ( void * ) 0, oneshot_timer_callback);
  if (push_button_timer == NULL) {
    ESP_LOGE(TAG, "Can't create timer");
    return;
  }

  int buttonState = BUTTON_UP;

  while(1) {       
    int levelHIGH = gpio_get_level(button_pin);
    if ( (millis() - lastDebounceTime) > debounceDelay) {
      if( levelHIGH && buttonState == BUTTON_UP) {
        buttonState = BUTTON_DOWN;
        lastDebounceTime = millis();
        if(xTimerIsTimerActive(push_button_timer) == pdFALSE ){
          xTimerStart( push_button_timer, (TickType_t)0 );
        }
        count++;
      }else {
        buttonState = BUTTON_UP;
      }
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

  // Spawn a task to monitor the pins
  xTaskCreate(&button_task, "button_task", CONFIG_PUSH_BUTTON_TASK_STACK_SIZE, NULL, 10, NULL);
}