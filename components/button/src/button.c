#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <freertos/timers.h>
#include "esp_log.h"

#define BUTTON_DOWN   (1)
#define BUTTON_UP     (2)

const static char *TAG = "BUTTON";

static int button_pin = -1;
void (*cb_timer_shot)(int) = NULL;
void (*cb_down_shot)(void) = NULL;
void (*cb_up_shot)(unsigned long time) = NULL;

static xQueueHandle interputQueue = NULL;

TimerHandle_t press_count_timer = NULL;

static uint8_t count = 0;

static unsigned long lastDebounceTime = 0;
static const unsigned long debounceDelay = 50;
static unsigned long start_hold = 0;
static int last_level = 0;

static unsigned long millis() {
  return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

static void IRAM_ATTR gpio_interrupt_handler(void *args) {
  int pinNumber = (int)args;
  xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

static void interrupt_task(void *params) {
  int pinNumber;
  BaseType_t xStatus;

  for (;;) {
    xStatus = xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY);
    if (xStatus == pdPASS && pinNumber == button_pin) {
      if ( (millis() - lastDebounceTime) > debounceDelay) {
        if ( press_count_timer && xTimerIsTimerActive(press_count_timer) == pdFALSE ){
          xTimerStart( press_count_timer, (TickType_t)0 );
        }
        count++;
        lastDebounceTime = millis();
        int level = gpio_get_level(button_pin);
        if (level == last_level) {
          last_level = 0;
        }else {
          last_level = level;
          if ( level == 0 && start_hold) {
            if (cb_up_shot) cb_up_shot(millis()-start_hold);
            start_hold = 0;
          }
          if (gpio_get_level(button_pin) == 1) {
            start_hold = millis();
            if (cb_down_shot) cb_down_shot();
          }
        }
      }
    }
  }
}

static void timer_cb( TimerHandle_t xTimer ){

	/* stop the timer */
	xTimerStop( xTimer, (TickType_t) 0 );

  /* callback press count */
  if (cb_timer_shot) (*cb_timer_shot)(count);
  count = 0;
}

void button_init(int pin, void (*func_timer_ptr)(int), void (*func_down_ptr)(void), void (*func_up_ptr)(unsigned long)) {
  button_pin = pin;
  cb_timer_shot = func_timer_ptr;
  cb_down_shot = func_down_ptr;
  cb_up_shot = func_up_ptr;

  if ( button_pin < 0 ){
    ESP_LOGE(TAG, "Invalid pin: %d", pin);
    return;
  }

  if ( !cb_timer_shot || !cb_down_shot || !cb_up_shot){
    ESP_LOGE(TAG, "Call back functions undefined");
    return;
  }

  gpio_pad_select_gpio(button_pin);
  gpio_set_direction(button_pin, GPIO_MODE_INPUT);
  gpio_set_intr_type(button_pin, GPIO_INTR_ANYEDGE);

	/* memory allocation */
  interputQueue = xQueueCreate(10, sizeof(int));

	/* create timer for press count */
	press_count_timer = xTimerCreate( NULL, pdMS_TO_TICKS(CONFIG_PUSH_BUTTON_TIMER_MS), pdFALSE, ( void * ) 0, timer_cb);

  /* create interrupt queue task */
  xTaskCreate(interrupt_task, "interrupt_task", 0x800, NULL, CONFIG_WIFI_MANAGER_TASK_PRIORITY-1, NULL);

  gpio_install_isr_service(0);
  gpio_isr_handler_add(button_pin, gpio_interrupt_handler, (void *)button_pin);    
}