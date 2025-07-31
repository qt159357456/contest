#include "main.h"
#include "timer.h"
#include "task.h"

//pwm控制步进电机的相关变量
volatile uint32_t step_count_y = 0;
volatile uint32_t step_target_y = 0;
volatile uint32_t step_count_x = 0;
volatile uint32_t step_target_x = 0;


