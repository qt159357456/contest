#ifndef 	MUTEX_LOCK_AND_MESSAGE_QUEUE_H
#define   MUTEX_LOCK_AND_MESSAGE_QUEUE_H
#include "global.h"

void CustomMutex_Init(CustomMutex_t* mutex, const char* name);
void CustomMutex_Lock(CustomMutex_t* mutex);
void CustomMutex_Unlock(CustomMutex_t* mutex);
QueueStatus_t CustomQueue_Init(CustomQueue_t* queue, 
                              void* buffer, 
                              uint16_t msg_size, 
                              uint16_t max_items, 
                              const char* name);
QueueStatus_t CustomQueue_Send(CustomQueue_t* queue, const void* msg);
QueueStatus_t CustomQueue_Receive(CustomQueue_t* queue, void* msg);
QueueStatus_t CustomQueue_SendTimeout(CustomQueue_t* queue, const void* msg, uint32_t timeout_ms);
QueueStatus_t CustomQueue_ReceiveTimeout(CustomQueue_t* queue, void* msg, uint32_t timeout_ms);
uint16_t CustomQueue_GetCount(CustomQueue_t* queue);

#endif
