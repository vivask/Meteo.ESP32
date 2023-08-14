#ifndef __LIST_H__
#define __LIST_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*Func)(void*);

typedef struct CallBackList {
    Func func;
    struct CallBackList *next;
} CallBackList;

void push_cb(CallBackList **head, Func func);
void run_cb(const CallBackList *head, void* param);
#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __LIST_H__