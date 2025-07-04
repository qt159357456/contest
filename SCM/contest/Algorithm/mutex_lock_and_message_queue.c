#include "mutex_lock_and_message_queue.h"

// ��������ʼ������
void CustomMutex_Init(CustomMutex_t* mutex, const char* name) {
    if (mutex) {
        mutex->locked = 0;
        mutex->owner = NULL;
        mutex->name = name;
        mutex->lock_count = 0;
    }
}

// ��ȡ��������֧�ֵݹ���ã�
void CustomMutex_Lock(CustomMutex_t* mutex) {
    if (!mutex) return;
    
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    
    // ����Ƿ��ǵ�ǰ�����ѳ�������֧�ֵݹ飩
//    if (mutex->owner == current_task) {
//        mutex->lock_count++;
//        return;
//    }
    
    // �ȴ����ͷ�
    while (mutex->locked != 0) {
        vTaskDelay(1);  // �ó�CPU������ռ��
    }
    
    // �ٽ�����������ȡ��
    taskENTER_CRITICAL();
    mutex->locked = 1;
    mutex->owner = current_task;
    mutex->lock_count = 1;
    taskEXIT_CRITICAL();
}

// �ͷŻ�����
void CustomMutex_Unlock(CustomMutex_t* mutex) {
    if (!mutex || mutex->locked == 0) return;
    
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    
    // ����Ƿ������ĳ�����
    if (mutex->owner != current_task) {
        return;
    }
    
    // �����������
    mutex->lock_count--;
    
    // ֻ�е��������Ϊ0ʱ�������ͷ���
    if (mutex->lock_count == 0) {
        taskENTER_CRITICAL();
        mutex->locked = 0;
        mutex->owner = NULL;
        taskEXIT_CRITICAL();
    }
}

// ��ʼ����Ϣ����
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
    CustomMutex_Init(&queue->mutex, "queue_mutex");  // ��ʼ���������û�����
    
    return QUEUE_OK;
}

// ������Ϣ�����У���������
QueueStatus_t CustomQueue_Send(CustomQueue_t* queue, const void* msg) {
    if (!queue || !msg) return QUEUE_ERROR;
    
    CustomMutex_Lock(&queue->mutex);
    
    // �������Ƿ�����
    if (queue->item_count >= queue->max_items) {
        CustomMutex_Unlock(&queue->mutex);
        return QUEUE_FULL;
    }
    
    // ������Ϣ�洢��ַ����������
    uint8_t* buf_ptr = (uint8_t*)queue->buffer;
    buf_ptr += queue->write_idx * queue->msg_size;
    memcpy(buf_ptr, msg, queue->msg_size);
    
    // ����дָ�����Ϣ����
    queue->write_idx = (queue->write_idx + 1) % queue->max_items;
    queue->item_count++;
    
    CustomMutex_Unlock(&queue->mutex);
    return QUEUE_OK;
}

// �Ӷ��н�����Ϣ����������
QueueStatus_t CustomQueue_Receive(CustomQueue_t* queue, void* msg) {
    if (!queue || !msg) return QUEUE_ERROR;
    
    CustomMutex_Lock(&queue->mutex);
    
    // �������Ƿ�Ϊ��
    if (queue->item_count == 0) {
        CustomMutex_Unlock(&queue->mutex);
        return QUEUE_EMPTY;
    }
    
    // ������Ϣ��ȡ��ַ����������
    uint8_t* buf_ptr = (uint8_t*)queue->buffer;
    buf_ptr += queue->read_idx * queue->msg_size;
    memcpy(msg, buf_ptr, queue->msg_size);
    
    // ���¶�ָ�����Ϣ����
    queue->read_idx = (queue->read_idx + 1) % queue->max_items;
    queue->item_count--;
    
    CustomMutex_Unlock(&queue->mutex);
    return QUEUE_OK;
}

// ����ʱ�ķ�����Ϣ������ʽ��
QueueStatus_t CustomQueue_SendTimeout(CustomQueue_t* queue, const void* msg, uint32_t timeout_ms) {
    uint32_t start_time = xTaskGetTickCount();
    
    while (1) {
        QueueStatus_t status = CustomQueue_Send(queue, msg);
        if (status != QUEUE_FULL) {
            return status;
        }
        
        // ����Ƿ�ʱ
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout_ms)) {
            return QUEUE_TIMEOUT;
        }
        
        vTaskDelay(1);  // �ȴ�1ms������
    }
}

// ����ʱ�Ľ�����Ϣ������ʽ��
QueueStatus_t CustomQueue_ReceiveTimeout(CustomQueue_t* queue, void* msg, uint32_t timeout_ms) {
    uint32_t start_time = xTaskGetTickCount();
    
    while (1) {
        QueueStatus_t status = CustomQueue_Receive(queue, msg);
        if (status != QUEUE_EMPTY) {
            return status;
        }
        
        // ����Ƿ�ʱ
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout_ms)) {
            return QUEUE_TIMEOUT;
        }
        
        vTaskDelay(1);  // �ȴ�1ms������
    }
}

// ��ȡ��ǰ�����е���Ϣ����
uint16_t CustomQueue_GetCount(CustomQueue_t* queue) {
    if (!queue) return 0;
    
    CustomMutex_Lock(&queue->mutex);
    uint16_t count = queue->item_count;
    CustomMutex_Unlock(&queue->mutex);
    
    return count;
}
