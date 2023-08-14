#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void button_init(int pin, void (*func_timer_ptr)(int), void (*func_down_ptr)(void), void (*func_up_ptr)(unsigned long));

#ifdef __cplusplus
}
#endif


#endif  // __BUTTON_H__
