#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void button_init(int pin, void (*func_ptr)(int));

#ifdef __cplusplus
}
#endif


#endif  // __BUTTON_H__
