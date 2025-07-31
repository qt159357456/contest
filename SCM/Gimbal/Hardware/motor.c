#include "main.h"
#include "global.h"
#include "timer.h"
#include "motor.h"
/* 全局电机实例 */
StepperMotor motor_x = {
    .current_angle = 0.0f,
    .target_angle = 0.0f,
    .step_target = 0,
    .step_count = 0,
		.step_sum = 0,
    .direction = 0,
    .dir_port = GPIOB,
    .dir_pin = GPIO_PIN_7,
    .timer = &htim2,
    .timer_channel = TIM_CHANNEL_4,
    .step_angle = STEP_ANGLE,     // 0.05625度/步
    .max_frequency = 20000,       // 20kHz最大频率
    .is_moving = 0
};

StepperMotor motor_y = {
    .current_angle = 0.0f,
    .target_angle = 0.0f,
    .step_target = 0,
    .step_count = 0,
		.step_sum = 0,
    .direction = 0,
    .dir_port = GPIOB,
    .dir_pin = GPIO_PIN_9,
    .timer = &htim3,
    .timer_channel = TIM_CHANNEL_4,
    .step_angle = STEP_ANGLE,     // 0.05625度/步
    .max_frequency = 20000,       // 20kHz最大频率
    .is_moving = 0
};

void Step_Init()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}


void Set_Direction(StepperMotor* motor, uint8_t dir) {
    motor->direction = dir;
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, 
                      dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// freq_hz: 步进频率（步/秒）
// 设置驱动频率
void Set_Speed(StepperMotor* motor, uint32_t freq_hz) {
    // 限制频率不超过最大值
    freq_hz = (freq_hz > motor->max_frequency) ? motor->max_frequency : freq_hz;
    
    uint32_t arr = 1000000 / freq_hz;  // 1MHz定时器
    __HAL_TIM_SET_AUTORELOAD(motor->timer, arr - 1);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->timer_channel, arr / 2);
}


// 控制电机转动指定角度
// 角度转步数: 每步 0.05625°
void Move_Angle(StepperMotor* motor, float angle, uint32_t freq) {
    // 1. 计算转动方向
    uint8_t dir = (angle >= 0) ? 0 : 1;
    Set_Direction(motor, dir);
    
    // 2. 计算目标步数
    float abs_angle = fabsf(angle);
    motor->step_target = (uint32_t)(abs_angle / motor->step_angle + 0.5f);
		if(motor->step_target==0)
					return;
    motor->step_count = 0;
    
    // 3. 设置驱动频率
    Set_Speed(motor, freq);
    
    // 4. 启动定时器
    motor->is_moving = 1;
		__HAL_TIM_CLEAR_IT(motor->timer, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(motor->timer);
    HAL_TIM_PWM_Start_IT(motor->timer, motor->timer_channel);
}

void Move_Angle_Global(StepperMotor* motor,float angle, uint32_t freq)
{
		Move_Angle(motor,angle-motor->current_angle,freq);
}


void move_one_step(StepperMotor* motor,uint8_t dir,uint32_t freq){
		Set_Direction(motor,dir);
		motor->step_target = 1;
		motor->step_count = 0;
		Set_Speed(motor, freq);
		motor->is_moving = 1;
		__HAL_TIM_CLEAR_IT(motor->timer, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(motor->timer);
		HAL_TIM_PWM_Start_IT(motor->timer, motor->timer_channel);
}


void move_steps(StepperMotor* motor,int32_t steps,uint32_t freq){
		Set_Direction(motor,steps>=0?0:1);
		motor->step_target = abs(steps);
		if(motor->step_target==0)
				return;
		motor->step_count = 0;
		Set_Speed(motor, freq);
		motor->is_moving = 1;
			__HAL_TIM_CLEAR_IT(motor->timer, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(motor->timer);
    HAL_TIM_PWM_Start_IT(motor->timer, motor->timer_channel);
}


float kp_x = 1,kp_y = 1;
int steps_x,steps_y;
void PID_motors(uint16_t x_t,uint16_t y_t,uint16_t x_c,uint16_t y_c){
		if(!motor_x.is_moving&&!motor_y.is_moving){
			steps_x = (int)(-1.0*(1.0*x_t-x_c)*kp_x);
			steps_y = (int)(-1.0*(1.0*y_t-y_c)*kp_y);
			move_steps(&motor_x,steps_x,1000);
			move_steps(&motor_y,steps_y,1000);
		}
}

int arrive_target(uint16_t x_t,uint16_t y_t,uint16_t x_c,uint16_t y_c,int dis){
		return (abs(x_t-x_c)<dis&&abs(y_t-y_c)<dis);
}

void Reset_Position(void) {
		motor_x.step_sum = 0;
		motor_x.step_count = 0;
		motor_x.step_target = 0;
		motor_x.current_angle = 0.0f;
		motor_x.target_angle = 0.0f;
		motor_x.is_moving = 0;
		motor_x.direction = 0;

		motor_y.step_sum = 0;
		motor_y.step_count = 0;
		motor_y.step_target = 0;
		motor_y.current_angle = 0.0f;
		motor_y.target_angle = 0.0f;
		motor_y.is_moving = 0;
		motor_y.direction = 0;
}


void Reset_Gimbal(void) {
		move_steps(&motor_x,-motor_x.step_sum,100);
		move_steps(&motor_y,-motor_y.step_sum,100);
}
// 生成包含起点和终点的完整路径
Point2D_t interp;
void generate_linear_path(Point2D_t start, Point2D_t end, Angles_t* path) {
    
    
    // 生成中间点
    for (uint16_t i = 0; i < INTERPOLATION_STEPS-1; i++) {
        float ratio = (float)i / (float)(INTERPOLATION_STEPS-1);
				interp.x = start.x + ratio * (end.x - start.x);
				interp.y = start.y + ratio * (end.y - start.y);
				distance_to_angle2(interp,&path[i]);
    }
    // 添加终点
		distance_to_angle2(end,&path[INTERPOLATION_STEPS-1]);
}
