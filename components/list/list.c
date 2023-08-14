#include <stdlib.h>
#include "list.h"

void push_cb(CallBackList **head, Func func) {
    CallBackList *tmp = (CallBackList*) malloc(sizeof(CallBackList));
    tmp->func = func;
    tmp->next = (*head);
    (*head) = tmp;
}

void run_cb(const CallBackList *head, void* param) {
    while(head) {
        head->func(param);
        head = head->next;
    }
}