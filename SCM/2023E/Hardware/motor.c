#include "main.h"
#include "global.h"
#include "gpio.h"
#include "timer.h"
#include "tim.h"
#include "motor.h"
/* ȫ�ֵ��ʵ�� */
StepperMotor motor_x = {
    .current_angle = 0.0f,
    .target_angle = 0.0f,
    .step_target = 0,
    .step_count = 0,
		.step_sum = 0,
    .direction = 0,
    .dir_port = GPIOA,
    .dir_pin = GPIO_PIN_1,
    .timer = &htim2,
    .timer_channel = TIM_CHANNEL_4,
    .step_angle = STEP_ANGLE,     // 0.05625��/��
    .max_frequency = 20000,       // 20kHz���Ƶ��
    .is_moving = 0
};

StepperMotor motor_y = {
    .current_angle = 0.0f,
    .target_angle = 0.0f,
    .step_target = 0,
    .step_count = 0,
		.step_sum = 0,
    .direction = 0,
    .dir_port = GPIOA,
    .dir_pin = GPIO_PIN_4,
    .timer = &htim3,
    .timer_channel = TIM_CHANNEL_4,
    .step_angle = STEP_ANGLE,     // 0.05625��/��
    .max_frequency = 20000,       // 20kHz���Ƶ��
    .is_moving = 0
};

void Step_Init()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}


void Set_Direction(StepperMotor* motor, uint8_t dir) {
    motor->direction = dir;
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, 
                      dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// freq_hz: ����Ƶ�ʣ���/�룩
// ��������Ƶ��
void Set_Speed(StepperMotor* motor, uint32_t freq_hz) {
    // ����Ƶ�ʲ��������ֵ
    freq_hz = (freq_hz > motor->max_frequency) ? motor->max_frequency : freq_hz;
    
    uint32_t arr = 1000000 / freq_hz;  // 1MHz��ʱ��
    __HAL_TIM_SET_AUTORELOAD(motor->timer, arr - 1);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->timer_channel, arr / 2);
}


// ���Ƶ��ת��ָ���Ƕ�
// �Ƕ�ת����: ÿ�� 0.05625��
void Move_Angle(StepperMotor* motor, float angle, uint32_t freq) {
    // 1. ����ת������
    uint8_t dir = (angle >= 0) ? 0 : 1;
    Set_Direction(motor, dir);
    
    // 2. ����Ŀ�경��
    float abs_angle = fabsf(angle);
    motor->step_target = (uint32_t)(abs_angle / motor->step_angle + 0.5f);
    motor->step_count = 0;
    
    // 3. ��������Ƶ��
    Set_Speed(motor, freq);
    
    // 4. ������ʱ��
    motor->is_moving = 1;
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
		HAL_TIM_Base_Start_IT(motor->timer);
    HAL_TIM_PWM_Start_IT(motor->timer, motor->timer_channel);
}


void move_steps(StepperMotor* motor,int32_t steps,uint32_t freq){
	Set_Direction(motor,steps>=0?0:1);
		motor->step_target = steps;
		motor->step_count = 0;
		Set_Speed(motor, freq);
		motor->is_moving = 1;
		HAL_TIM_Base_Start_IT(motor->timer);
    HAL_TIM_PWM_Start_IT(motor->timer, motor->timer_channel);
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

// ���ɰ��������յ������·��
Point2D_t interp;
void generate_linear_path(Point2D_t start, Point2D_t end, Angles_t* path) {
    
    
    // �����м��
    for (uint16_t i = 0; i < INTERPOLATION_STEPS-1; i++) {
        float ratio = (float)i / (float)(INTERPOLATION_STEPS-1);
				interp.x = start.x + ratio * (end.x - start.x);
				interp.y = start.y + ratio * (end.y - start.y);
				distance_to_angle2(interp,&path[i]);
    }
    // ����յ�
		distance_to_angle2(end,&path[INTERPOLATION_STEPS-1]);
}
