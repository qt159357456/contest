#include "mutex_lock_and_message_queue.h"

// 互斥锁初始化函数
void CustomMutex_Init(CustomMutex_t* mutex, const char* name) {
    if (mutex) {
        mutex->locked = 0;
        mutex->owner = NULL;
        mutex->name = name;
        mutex->lock_count = 0;
    }
}

// 获取互斥锁（支持递归调用）
void CustomMutex_Lock(CustomMutex_t* mutex) {
    if (!mutex) return;
    
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    
    // 检查是否是当前任务已持有锁（支持递归）
//    if (mutex->owner == current_task) {
//        mutex->lock_count++;
//        return;
//    }
    
    // 等待锁释放
    while (mutex->locked != 0) {
        vTaskDelay(1);  // 让出CPU，减少占用
    }
    
    // 临界区操作：获取锁
    taskENTER_CRITICAL();
    mutex->locked = 1;
    mutex->owner = current_task;
    mutex->lock_count = 1;
    taskEXIT_CRITICAL();
}

// 释放互斥锁
void CustomMutex_Unlock(CustomMutex_t* mutex) {
    if (!mutex || mutex->locked == 0) return;
    
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    
    // 检查是否是锁的持有者
    if (mutex->owner != current_task) {
        return;
    }
    
    // 减少重入计数
    mutex->lock_count--;
    
    // 只有当重入计数为0时才真正释放锁
    if (mutex->lock_count == 0) {
        taskENTER_CRITICAL();
        mutex->locked = 0;
        mutex->owner = NULL;
        taskEXIT_CRITICAL();
    }
}

// 初始化消息队列
QueueStatus_t CustomQueue_Init(CustomQueue_t* queue, 
                              void* buffer, 
                              uint16_t msg_size, 
                              uint16_t max_items, 
                              const char* name) {
    if (!queue || !buffer || msg_size == 0 || max_items == 0) {
        return QUEUE_ERROR;
    }
    
    queue->buffer = buffer;
    queue->msg_size = msg_size;
    queue->max_items = max_items;
    queue->item_count = 0;
    queue->read_idx = 0;
    queue->write_idx = 0;
    queue->name = name;
    CustomMutex_Init(&queue->mutex, "queue_mutex");  // 初始化队列内置互斥锁
    
    return QUEUE_OK;
}

// 发送消息到队列（非阻塞）
QueueStatus_t CustomQueue_Send(CustomQueue_t* queue, const void* msg) {
    if (!queue || !msg) return QUEUE_ERROR;
    
    CustomMutex_Lock(&queue->mutex);
    
    // 检查队列是否已满
    if (queue->item_count >= queue->max_items) {
        CustomMutex_Unlock(&queue->mutex);
        return QUEUE_FULL;
    }
    
    // 计算消息存储地址并复制数据
    uint8_t* buf_ptr = (uint8_t*)queue->buffer;
    buf_ptr += queue->write_idx * queue->msg_size;
    memcpy(buf_ptr, msg, queue->msg_size);
    
    // 更新写指针和消息计数
    queue->write_idx = (queue->write_idx + 1) % queue->max_items;
    queue->item_count++;
    
    CustomMutex_Unlock(&queue->mutex);
    return QUEUE_OK;
}

// 从队列接收消息（非阻塞）
QueueStatus_t CustomQueue_Receive(CustomQueue_t* queue, void* msg) {
    if (!queue || !msg) return QUEUE_ERROR;
    
    CustomMutex_Lock(&queue->mutex);
    
    // 检查队列是否为空
    if (queue->item_count == 0) {
        CustomMutex_Unlock(&queue->mutex);
        return QUEUE_EMPTY;
    }
    
    // 计算消息读取地址并复制数据
    uint8_t* buf_ptr = (uint8_t*)queue->buffer;
    buf_ptr += queue->read_idx * queue->msg_size;
    memcpy(msg, buf_ptr, queue->msg_size);
    
    // 更新读指针和消息计数
    queue->read_idx = (queue->read_idx + 1) % queue->max_items;
    queue->item_count--;
    
    CustomMutex_Unlock(&queue->mutex);
    return QUEUE_OK;
}

// 带超时的发送消息（阻塞式）
QueueStatus_t CustomQueue_SendTimeout(CustomQueue_t* queue, const void* msg, uint32_t timeout_ms) {
    uint32_t start_time = xTaskGetTickCount();
    
    while (1) {
        QueueStatus_t status = CustomQueue_Send(queue, msg);
        if (status != QUEUE_FULL) {
            return status;
        }
        
        // 检查是否超时
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout_ms)) {
            return QUEUE_TIMEOUT;
        }
        
        vTaskDelay(1);  // 等待1ms再重试
    }
}

// 带超时的接收消息（阻塞式）
QueueStatus_t CustomQueue_ReceiveTimeout(CustomQueue_t* queue, void* msg, uint32_t timeout_ms) {
    uint32_t start_time = xTaskGetTickCount();
    
    while (1) {
        QueueStatus_t status = CustomQueue_Receive(queue, msg);
        if (status != QUEUE_EMPTY) {
            return status;
        }
        
        // 检查是否超时
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout_ms)) {
            return QUEUE_TIMEOUT;
        }
        
        vTaskDelay(1);  // 等待1ms再重试
    }
}

// 获取当前队列中的消息数量
uint16_t CustomQueue_GetCount(CustomQueue_t* queue) {
    if (!queue) return 0;
    
    CustomMutex_Lock(&queue->mutex);
    uint16_t count = queue->item_count;
    CustomMutex_Unlock(&queue->mutex);
    
    return count;
}
