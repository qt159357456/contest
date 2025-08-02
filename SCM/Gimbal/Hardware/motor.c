#include "main.h"
#include "usart.h"
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

void my_move_steps(StepperMotor* motor,uint32_t steps,uint8_t dir,uint32_t freq){
		if(motor->is_moving||steps==0)
				return;
		Set_Direction(motor,dir);
		motor->step_target = steps;
		if(motor->step_target==0)
				return;
		motor->step_count = 0;
		Set_Speed(motor, freq);
		motor->is_moving = 1;
		__HAL_TIM_CLEAR_IT(motor->timer, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(motor->timer);
    HAL_TIM_PWM_Start_IT(motor->timer, motor->timer_channel);
}

void StepperMotor_Stop(StepperMotor* motor) {
    // 停止PWM输出和中断
    HAL_TIM_PWM_Stop_IT(motor->timer, motor->timer_channel);
    
    // 新增关键步骤：强制重置定时器状态
    __HAL_TIM_DISABLE(motor->timer);          // 禁用定时器
    __HAL_TIM_SET_COUNTER(motor->timer, 0);   // 重置计数器
    __HAL_TIM_CLEAR_FLAG(motor->timer, TIM_FLAG_UPDATE); // 清除更新标志
    
    // 可选：使能定时器但不启动PWM
    // __HAL_TIM_ENABLE(motor->timer);
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


/**********************************************串口控制*******************************************************/
MotorData motor_data[2] = {0};
/* 1. 电机使能控制 */
uint8_t cmd1_x[6] = {
		0x00,        // 设备地址
		CMD_ENABLE,     // 命令码
		0xAB,           // 固定参数
		0x00,         // 使能状态(0:不使能, 1:使能)
		0x00,       // 多机同步标志(0:不同步, 1:同步)
		CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Motor_Enable_X(uint8_t address, uint8_t enable, uint8_t sync_flag) {
		cmd1_x[0] = address;
		cmd1_x[3] = enable;
		cmd1_x[4] = sync_flag;
    return Send_Motor_Command_X(cmd1_x, sizeof(cmd1_x));
}


uint8_t cmd1_y[6] = {
		0x00,        // 设备地址
		CMD_ENABLE,     // 命令码
		0xAB,           // 固定参数
		0x00,         // 使能状态(0:不使能, 1:使能)
		0x00,       // 多机同步标志(0:不同步, 1:同步)
		CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Motor_Enable_Y(uint8_t address, uint8_t enable, uint8_t sync_flag) {
		cmd1_y[0] = address;
		cmd1_y[3] = enable;
		cmd1_y[4] = sync_flag;
    return Send_Motor_Command_Y(cmd1_y, sizeof(cmd1_y));
}

/* 2. 速度模式控制 */
uint8_t cmd2_x[8] = {
		0x00,        // 设备地址
		CMD_ENABLE,     // 命令码
		0xAB,           // 固定参数
		0x00,         // 使能状态(0:不使能, 1:使能)
		0x00,       // 多机同步标志(0:不同步, 1:同步)
		CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Speed_Mode_X(uint8_t address, uint8_t direction, uint16_t speed, 
                            uint8_t acceleration, uint8_t sync_flag) {
		cmd2_x[0] = address;
		cmd2_x[2] = direction;
		cmd2_x[3] =(uint8_t)(speed >> 8);
		cmd2_x[4] =(uint8_t)(speed);
		cmd2_x[5] = acceleration;
		cmd2_x[6] = sync_flag;
    return Send_Motor_Command_X(cmd2_x, sizeof(cmd2_x));
};
														

uint8_t cmd2_y[8] = {
		0x00,        // 设备地址
		CMD_ENABLE,     // 命令码
		0xAB,           // 固定参数
		0x00,         // 使能状态(0:不使能, 1:使能)
		0x00,       // 多机同步标志(0:不同步, 1:同步)
		CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Speed_Mode_Y(uint8_t address, uint8_t direction, uint16_t speed, 
		uint8_t acceleration, uint8_t sync_flag) {
		cmd2_y[0] = address;
		cmd2_y[2] = direction;
		cmd2_y[3] =(uint8_t)(speed >> 8);
		cmd2_y[4] =(uint8_t)(speed);
		cmd2_y[5] = acceleration;
		cmd2_y[6] = sync_flag;
    return Send_Motor_Command_Y(cmd2_y, sizeof(cmd2_y));
}

/* 3. 位置模式控制 */
uint8_t cmd3_x[13] = {
		0x00,               // 设备地址
		CMD_POSITION_MODE,     // 命令码
		0x00,             // 旋转方向(0:CW, 1:CCW)
		0x00, // 速度高字节
		0x00,      // 速度低字节
		0x00,          // 加速度档位
		0x00, // 脉冲数字节3
		0x00, // 脉冲数字节2
		0x00,  // 脉冲数字节1
		0x00,       // 脉冲数字节0
		0x00,              // 相对/绝对模式(0:相对, 1:绝对)
		0x00,             // 多机同步标志
		CHECKSUM               // 校验字节
};
HAL_StatusTypeDef Position_Mode_X(uint8_t address, uint8_t direction, uint16_t speed, 
                               uint8_t acceleration, uint32_t pulses, 
                               uint8_t abs_mode, uint8_t sync_flag) {
		cmd3_x[0] = address;
		cmd3_x[2] = direction;
		cmd3_x[3] = (uint8_t)(speed >> 8);
		cmd3_x[4] = (uint8_t)(speed);
		cmd3_x[5] = acceleration;
		cmd3_x[6] = (uint8_t)(pulses >> 24);
		cmd3_x[7] = (uint8_t)(pulses >> 16);
		cmd3_x[8] = (uint8_t)(pulses >> 8);
		cmd3_x[9] = (uint8_t)(pulses);
		cmd3_x[10]= abs_mode;
		cmd3_x[11]= sync_flag;
    return Send_Motor_Command_X(cmd3_x, sizeof(cmd3_x));
}
															 

uint8_t cmd3_y[13] = {
		0x00,               // 设备地址
		CMD_POSITION_MODE,     // 命令码
		0x00,             // 旋转方向(0:CW, 1:CCW)
		0x00, // 速度高字节
		0x00,      // 速度低字节
		0x00,          // 加速度档位
		0x00, // 脉冲数字节3
		0x00, // 脉冲数字节2
		0x00,  // 脉冲数字节1
		0x00,       // 脉冲数字节0
		0x00,              // 相对/绝对模式(0:相对, 1:绝对)
		0x00,             // 多机同步标志
		CHECKSUM               // 校验字节
};
HAL_StatusTypeDef Position_Mode_Y(uint8_t address, uint8_t direction, uint16_t speed, 
                               uint8_t acceleration, uint32_t pulses, 
                               uint8_t abs_mode, uint8_t sync_flag) {
		cmd3_y[0] = address;
		cmd3_y[2] = direction;
		cmd3_y[3] = (uint8_t)(speed >> 8);
		cmd3_y[4] = (uint8_t)(speed);
		cmd3_y[5] = acceleration;
		cmd3_y[6] = (uint8_t)(pulses >> 24);
		cmd3_y[7] = (uint8_t)(pulses >> 16);
		cmd3_y[8] = (uint8_t)(pulses >> 8);
		cmd3_y[9] = (uint8_t)(pulses);
		cmd3_y[10]= abs_mode;
		cmd3_y[11]= sync_flag;
    return Send_Motor_Command_Y(cmd3_y, sizeof(cmd3_y));
}

/* 4. 立即停止命令 */
uint8_t cmd4_x[5] = {
		0x00,        // 设备地址
		CMD_STOP,       // 命令码
		0x98,           // 固定参数
		0x00,      // 多机同步标志
		CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Motor_Stop_X(uint8_t address, uint8_t sync_flag) {
	  cmd4_x[0] = address;
		cmd4_x[3] = sync_flag;
    return Send_Motor_Command_X(cmd4_x, sizeof(cmd4_x));
}

uint8_t cmd4_y[5] = {
		0x00,        // 设备地址
		CMD_STOP,       // 命令码
		0x98,           // 固定参数
		0x00,      // 多机同步标志
		CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Motor_Stop_Y(uint8_t address, uint8_t sync_flag) {
	  cmd4_y[0] = address;
		cmd4_y[3] = sync_flag;
    return Send_Motor_Command_Y(cmd4_y, sizeof(cmd4_y));
}
/* 5. 多机同步运动命令 */
uint8_t cmd5_x[4] = {
        0x00,        // 设备地址
        CMD_SYNC_MOVE,  // 命令码
        0x66,           // 固定参数
        CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Sync_Move_X(uint8_t address) {
		cmd5_x[0] = address;
    return Send_Motor_Command_X(cmd5_x, sizeof(cmd5_x));
}

uint8_t cmd5_y[4] = {
        0x00,        // 设备地址
        CMD_SYNC_MOVE,  // 命令码
        0x66,           // 固定参数
        CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Sync_Move_Y(uint8_t address) {
		cmd5_y[0] = address;
    return Send_Motor_Command_Y(cmd5_y, sizeof(cmd5_y));
}

/* 6. 设置单圈回零零点 */
uint8_t cmd6_x[5] = {
        0x00,        // 设备地址
        CMD_SET_HOME,   // 命令码
        0x88,           // 固定参数
        0x00,     // 存储标志(0:临时, 1:永久)
        CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Set_Home_Position_X(uint8_t address, uint8_t store_flag) {
	  cmd6_x[0] = address;
		cmd6_x[3] = store_flag;
    return Send_Motor_Command_X(cmd6_x, sizeof(cmd6_x));
}
uint8_t cmd6_y[5] = {
        0x00,        // 设备地址
        CMD_SET_HOME,   // 命令码
        0x88,           // 固定参数
        0x00,     // 存储标志(0:临时, 1:永久)
        CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Set_Home_Position_Y(uint8_t address, uint8_t store_flag) {
	  cmd6_y[0] = address;
		cmd6_y[3] = store_flag;
    return Send_Motor_Command_Y(cmd6_y, sizeof(cmd6_y));
}

/* 7. 触发原点回零 */

uint8_t cmd7_x[5] = {
        0x00,          // 设备地址
        CMD_TRIGGER_HOME, // 命令码
        0x00,        // 回零模式(0-3)
        0x00,        // 多机同步标志
        CHECKSUM          // 校验字节
};
HAL_StatusTypeDef Trigger_Home_X(uint8_t address, uint8_t home_mode, uint8_t sync_flag) {
	  cmd7_x[0] = address;
		cmd7_x[2] = home_mode;
		cmd7_x[3] = sync_flag;
    return Send_Motor_Command_X(cmd7_x, sizeof(cmd7_x));
}

uint8_t cmd7_y[5] = {
        0x00,          // 设备地址
        CMD_TRIGGER_HOME, // 命令码
        0x00,        // 回零模式(0-3)
        0x00,        // 多机同步标志
        CHECKSUM          // 校验字节
};
HAL_StatusTypeDef Trigger_Home_Y(uint8_t address, uint8_t home_mode, uint8_t sync_flag) {
	  cmd7_y[0] = address;
		cmd7_y[2] = home_mode;
		cmd7_y[3] = sync_flag;
    return Send_Motor_Command_Y(cmd7_y, sizeof(cmd7_y));
}

/* 8. 强制中断回零操作 */
uint8_t cmd8_x[4] = {
        0x00,          // 设备地址
        CMD_ABORT_HOME,   // 命令码
        0x48,             // 固定参数
        CHECKSUM          // 校验字节
};
HAL_StatusTypeDef Abort_Home_X(uint8_t address) {
	  cmd8_x[0] = address;
//	  memcpy(cmd,cmd8_x,sizeof(cmd8_x));
    return Send_Motor_Command_X(cmd8_x, sizeof(cmd8_x));
}

uint8_t cmd8_y[4] = {
        0x00,          // 设备地址
        CMD_ABORT_HOME,   // 命令码
        0x48,             // 固定参数
        CHECKSUM          // 校验字节
};
HAL_StatusTypeDef Abort_Home_Y(uint8_t address) {
	  cmd8_y[0] = address;
//	  memcpy(cmd,cmd8_y,sizeof(cmd8_y));
    return Send_Motor_Command_Y(cmd8_y, sizeof(cmd8_y));
}



uint8_t cmd9_x[3] = {
        0x00,        // 设备地址
        CMD_READ_SPEED, // 命令码
        CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Send_Get_Motor_Speed_X(uint8_t address){
	  cmd9_x[0] = address;
		// 发送读取命令
		HAL_StatusTypeDef status = Send_Motor_Command_X(cmd9_x, sizeof(cmd9_x));
    return status;
		
}


uint8_t cmd9_y[3] = {
        0x00,        // 设备地址
        CMD_READ_SPEED, // 命令码
        CHECKSUM        // 校验字节
};
HAL_StatusTypeDef Send_Get_Motor_Speed_Y(uint8_t address){
	  cmd9_y[0] = address;
		// 发送读取命令
		HAL_StatusTypeDef status = Send_Motor_Command_Y(cmd9_y, sizeof(cmd9_y));
    return status;
		
}


void Get_Motor_Speed(uint8_t address,uint8_t* data){
	  if(data[5]!=CHECKSUM)
				return;
		motor_data[address].speed_sign = data[2];
		motor_data[address].speed = (data[3] << 8) | data[4];
		if(data[2])
			motor_data[address].f_speed = -1.0*motor_data[address].speed*6;
		else 
			motor_data[address].f_speed = 1.0*motor_data[address].speed*6;
}





uint8_t cmd10_x[3] = {
        0x00,            // 设备地址
        CMD_READ_POSITION,  // 命令码
        CHECKSUM            // 校验字节
};
HAL_StatusTypeDef Send_Get_Motor_Position_X(uint8_t address){
   	cmd10_x[0] = address;
		// 发送读取命令
		HAL_StatusTypeDef status = Send_Motor_Command_X(cmd10_x, sizeof(cmd10_x));
    return status;		 
}

uint8_t cmd10_y[3] = {
        0x00,            // 设备地址
        CMD_READ_POSITION,  // 命令码
        CHECKSUM            // 校验字节
};
HAL_StatusTypeDef Send_Get_Motor_Position_Y(uint8_t address){
		
   	cmd10_y[0] = address;
		// 发送读取命令
		HAL_StatusTypeDef status = Send_Motor_Command_Y(cmd10_y, sizeof(cmd10_y));
    return status;		 
}

void Get_Motor_Position(uint8_t address,uint8_t* data){
		if(data[7]!=CHECKSUM)
				return;
		motor_data[address].position_sign = data[2];
		motor_data[address].position = 	((uint32_t)data[3] << 24) |
																		((uint32_t)data[4] << 16) |
																		((uint32_t)data[5] << 8)  |
																		data[6];
		if(data[2])
			motor_data[address].angle = -1.0f*(motor_data[address].position* 360.0f) / 65536.0f;
		else 
			motor_data[address].angle = 1.0f*(motor_data[address].position* 360.0f) / 65536.0f;
}

/* 角度控制函数（相对角度） */
HAL_StatusTypeDef Move_Relative_Angle_X(uint8_t address, float angle, uint16_t speed) {
    // 确定方向 (正角度为CCW，负角度为CW)
    uint8_t direction = (angle >= 0) ? DIR_CW : DIR_CCW;
		uint32_t pulses = (uint32_t)(fabs(angle) /STEP_ANGLE);
    
    // 发送位置模式命令
    return Position_Mode_X(address, direction, speed, 0x0A, pulses, 0, 0);
}

/* 角度控制函数（绝对角度） */
HAL_StatusTypeDef Move_Absolute_Angle_X(uint8_t address, float angle, uint16_t speed) {
		uint8_t direction = (angle >= 0) ? DIR_CW : DIR_CCW;
		uint32_t pulses = (uint32_t)(fabs(angle) /STEP_ANGLE);
    
    // 发送位置模式命令
    return Position_Mode_X(address, direction, speed, 0x0A, pulses, 1, 0);
}

/* 角度控制函数（相对角度） */
HAL_StatusTypeDef Move_Relative_Angle_Y(uint8_t address, float angle, uint16_t speed) {
    // 确定方向 (正角度为CCW，负角度为CW)
    uint8_t direction = (angle >= 0) ? DIR_CW : DIR_CCW;
		uint32_t pulses = (uint32_t)(fabs(angle) /STEP_ANGLE);
    
    // 发送位置模式命令
    return Position_Mode_Y(address, direction, speed, 0x0A, pulses, 0, 0);
}

/* 角度控制函数（绝对角度） */
HAL_StatusTypeDef Move_Absolute_Angle_Y(uint8_t address, float angle, uint16_t speed) {
		uint8_t direction = (angle >= 0) ? DIR_CW : DIR_CCW;
		uint32_t pulses = (uint32_t)(fabs(angle) /STEP_ANGLE);
    
    // 发送位置模式命令
    return Position_Mode_Y(address, direction, speed, 0x0A, pulses, 1, 0);
}



float kp_x = 0.15,kp_y = 1;
int steps_x,steps_y;
uint32_t x_steps;
uint8_t acceleration = 0x0A;
void PID_motors(int16_t offset_x,int16_t offset_y){
	uint8_t dir_x = offset_x>=0?0x00:0x01;//
//	uint8_t dir_y = offset_y>=0?0x00:0x01;//y轴朝上正
//	uint16_t speed_x = (uint16_t)(constrain(abs(offset_x)*kp_x,0,30));//转每分钟
//	uint16_t speed_y = (uint16_t)(constrain(abs(offset_y)*kp_y,0,30));
//	if(speed_y&&!send_speed_y_flag){
//		Speed_Mode_Y(0x01, dir_y, speed_y, acceleration, 0x00);//y
//		send_speed_y_flag = 1;
//		//vTaskDelay(1);
//	}
//	if(speed_x&&!send_speed_x_flag){
//		Speed_Mode_X(0x01, dir_x, speed_x,acceleration, 0x00);//x
//		send_speed_x_flag = 1;
//	}
//	offset_x = abs(offset_x);
//	if(offset_x>50)
//			kp_x = 1;
//	else
//			kp_x = 0.1;
//	x_steps = (uint32_t)(kp_x*offset_x);
//	if(x_steps==0)
//		x_steps=1;
	offset_x = abs(offset_x);
	if(offset_x>100){
			x_steps = 5;
	}
	else if(offset_x>50)
			x_steps = 2;
	else
			x_steps = 1;
	my_move_steps(&motor_x,x_steps,dir_x,2000);
}


void PID_motors2(int16_t offset_x,int16_t offset_y){
	uint8_t dir_x = offset_x>=0?0x00:0x01;//
	offset_x = abs(offset_x);
	if(offset_x>50)
			x_steps = 2;
	else if(offset_x>8)
			x_steps = 1;
	else
			x_steps = 0;
	my_move_steps(&motor_x,x_steps,dir_x,1000);
}
