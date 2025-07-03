/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "global.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include "math.h"
#include <stdarg.h>
#include <string.h>
#include "filter.h"
#include "servo.h"
#include "stm32f1xx_hal.h"
#include "oled.h"
#include "usart.h"
#include "mpu6050.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void debug_printf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TaskHandle_t  led1_TaskHandle_t;
TaskHandle_t  led2_TaskHandle_t;
TaskHandle_t  filter_TaskHandle_t;
TaskHandle_t  servo_TaskHandle_t;
TaskHandle_t  oled_TaskHandle_t;
TaskHandle_t  usart_TaskHandle_t;
TaskHandle_t  mpu6050_TaskHandle_t;
TaskHandle_t  motor_TaskHandle_t;
TaskHandle_t  decision_TaskHandle_t;
TaskHandle_t  state_update_TaskHandle_t;


/* ��ȫ�ֱ���������� */
//SemaphoreHandle_t xSensorMutex;  // ���������ݻ�����
float pitch, roll, yaw;
RobotState_t g_robot_state = {0};
SemaphoreHandle_t xStateMutex;

void led1_task(void *argument) {
  (void)argument;
	while(1){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		vTaskDelay(1000);
	}
}

void led2_task(){
	while(1){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
		vTaskDelay(2000);
	}
}

Filter_t Data_Filter = {0.8,0,0};

AVG_Flt_t AVG_Filter;

MEDIAN_Flt_t MEDIAN_Filter;

//FIR_Flt_t FIR_Filter;

Klm_Flt_t KLM_Filter = 
{
	0.02,//����Э����
	0,//����������
	0.001,//��������Э���Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
	0.1,//��������Э���R���󣬶�̬��Ӧ�����������ȶ��Ա��
	0//�������˲����
};

double Data;
double Result;
double t;
double Test;


void filter_task(){
//	//��ͨ�˲�����
//	while(1){
//		Data = sin(t)+0.1*sin(50*t);
//		Result = Low_Pass_Filter(&Data_Filter,Data);
//		t+=0.01;
//		while(huart1.gState != HAL_UART_STATE_READY) {
//			vTaskDelay(1); // ��FreeRTOS���ó�CPU
//		}
//	debug_printf("%.3f,%.3f\n",Data,Result);
//		vTaskDelay(50);
//	}
//	
	
//	//������ֵ�˲�����
//	while(1){
//		Data = sin(t)+0.1*sin(50*t);
////		Data = sin(t);
////		if((int)(t*100)%100 == 0){
////			Data = sin(t) + 2 * sin(50 * t);
////		}	
//		Result = Average_Filter(&AVG_Filter,Data);
//		t += 0.01;
//		debug_printf("%.3f,%.3f\n",Data,Result);
//		vTaskDelay(50);
//	}

////��ֵ�˲�����
//while(1){
////		Data = sin(t) + 0.1 * sin(50 * t);
//		Data = sin(t);
//		if((int)(t*100)%100 == 0){
//			Data = sin(t) + 2 * sin(50 * t);
//		}	
//		Result = Median_Filter(&MEDIAN_Filter,Data);
//		t += 0.01;
//		debug_printf("%.3f,%.3f\n",Data,Result);
//		vTaskDelay(50);
//	}

////��ͨ�˲�����
//FIR_Filter.Fs = 10000;
//FIR_Filter.Fc1 = 500;
//FIR_Filter.Fc2 = 1500;
//FIRBandPassFilter_Init(&FIR_Filter);
//while(1){
//	Data = sin(2000*PI*t)+sin(4000*PI*t);
//	Test = sin(2000*PI*t);
//	Result = FIR_BandPass_Filter(&FIR_Filter,Data);
//	t += 0.01;
//	debug_printf("%.3f,%.3f\n",Data,Result);
//	vTaskDelay(50);
//}	
	
//�������˲�����
while(1){
	Data = sin(200*PI*t)+sin(2000*PI*t)+sin(4000*PI*t);
	Test = sin(2000*PI*t);
	Result = Median_Filter(&MEDIAN_Filter,Data);
	t += 0.0001;
//	debug_printf("%.3f,%.3f\n",Data,Result);
	vTaskDelay(50);
	}

}

int test_pulse = 600;

void servo_task(){
		uint16_t brightness = 0;
    int direction = 1; // 1:��������, 0:��С����
    
    while(1) {
        // ������Ч��
        Servo_SetPWM(&htim1, TIM_CHANNEL_1, brightness);
        
        if(direction) {
            brightness += 100;
            if(brightness >= 19999) direction = 0;
        } else {
            brightness -= 100;
            if(brightness == 0) direction = 1;
        }
        
        vTaskDelay(10); // ������ʱ���Ʊ仯�ٶ�
    }
}


/* �޸� oled_task ���� */
void oled_task() {
    char displayBuffer[64];  // ��ʾ������
    
    while(1) {
        // ʹ�û�������ȡ����������
//        if(xSemaphoreTake(xSensorMutex, portMAX_DELAY) == pdTRUE) {
//            for(int i = 0; i < 3; i++) {
//                accel[i] = g_accel[i];
//                gyro[i] = g_gyro[i];
//            }
//            xSemaphoreGive(xSensorMutex);
//        }
		
        // ����
        OLED_FullyClear();
        
        // ��ʾ����������
        snprintf(displayBuffer, sizeof(displayBuffer), "pitch:%.2f", Att_Angle.pitch);
        OLED_ShowStr(0, 0, (unsigned char *)displayBuffer, 2);
        
        snprintf(displayBuffer, sizeof(displayBuffer), "roll:%.2f", Att_Angle.roll);
        OLED_ShowStr(0, 3, (unsigned char *)displayBuffer, 2);
        
        snprintf(displayBuffer, sizeof(displayBuffer), "yaw:%.2f", Att_Angle.yaw);
        OLED_ShowStr(0, 6, (unsigned char *)displayBuffer, 2);
        
        vTaskDelay(200); // OLEDˢ���ʿ���
    }
}



	//// ����printf��ʹ�ã���λ������1
	int fputc(int ch, FILE *f) // printf
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
		return ch;
	}
	 
	int fgetc(FILE *f) // getchar
	{
		uint8_t ch = 0;
		HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
		return ch;
	}

	int tick_time = 0;
void usart_task(){
	while (1)
  {
//   // HAL_UART_Transmit(&huart1, (uint8_t *)"Hello, World!\r\n", 15, 1000);
//    printf("printf Hello, World!\r\n");
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//    HAL_Delay(1000);
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//    HAL_Delay(1000);
		tick_time++;
    // chelk the time 
    // printf("tick_time: %d\r\n", tick_time);
    if (tick_time % 7200000 == 0)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    if (tick_time % 7200000 == 7200000/2)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
//		debug_printf("123456\n");
		vTaskDelay(500);
    // open the uart1 reciver
  }
}

void mpu6050_task(){      
    while(1) {
//        convert_uint8_to_uint16(accel,gyro);
        // ʹ�û���������ȫ�ֱ���
//        if(xSemaphoreTake(xSensorMutex, portMAX_DELAY) == pdTRUE) {
//            for(int i = 0; i < 3; i++) {
//                g_accel[i] = accel[i];
//                g_gyro[i] = gyro[i];
//            }
//            xSemaphoreGive(xSensorMutex);
//        }
				Prepare_Data(); 																								//��ȡ��̬������������
				IMUupdate(&Gyr_filt,&Acc_filt,&Att_Angle,0.01f); 		
        // ��ӡ��̬��
//        debug_printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n", Att_Angle.pitch, Att_Angle.roll, Att_Angle.yaw);
				
        vTaskDelay(10);
    }
}

QueueHandle_t nav_cmd_queue;  // ����ָ�����

void motor_task(void) {
    // ��������ʼ��
    PID_t speed_pid_left = { .Kp=2.0f, .Ki=0.5f, .Kd=0.1f, .OutMin=-1000, .OutMax=1000 };
    PID_t speed_pid_right = { .Kp=2.0f, .Ki=0.5f, .Kd=0.1f, .OutMin=-1000, .OutMax=1000 };
    PID_t angle_pid = { .Kp=1.5f, .Ki=0.05f, .Kd=0.3f, .OutMin=-1.0f, .OutMax=1.0f };
    TD angle_td = { .r=5.0f, .h=0.01f, .T=0.01f };
    
    const TickType_t xPeriod = pdMS_TO_TICKS(10); // 10ms��������
    TickType_t xLastWakeTime = xTaskGetTickCount();
    RobotState_t local_state;
		NavCmd_t nav_cmd;
		
		float target_angle;
		float omega_cmd;
		float target_left;
		float target_right;
    while(1) {
        
        // 1. ��ȫ��ȡ״̬����
//        if(xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&local_state, &g_robot_state, sizeof(RobotState_t));
//            xSemaphoreGive(xStateMutex);W
//        }
        
        // 2. ��ȡ����ָ��
//        if(xQueueReceive(nav_cmd_queue, &nav_cmd, 0) == pdPASS) {
            angle_td.aim = nav_cmd.target_angle;
//        }
        
        // 3. ����Ƕ�ָ��
        Clip_TD_Function(&angle_td, 30.0f);
        target_angle = angle_td.x1;
        
        // 4. �ǶȻ�����
        angle_pid.Target = target_angle;
        angle_pid.Actual = local_state.global.yaw;
        PID_Update(&angle_pid);
        
        // 5. �˶�ѧ����
         omega_cmd = angle_pid.Out + angle_td.x2 * 0.1f;
         target_left = nav_cmd.base_speed - (omega_cmd * TRACK_WIDTH / 2.0f);
         target_right = nav_cmd.base_speed + (omega_cmd * TRACK_WIDTH / 2.0f);
        
        // 6. �ٶȻ�����
        speed_pid_left.Target = target_left;
        speed_pid_left.Actual = local_state.local_vel.linear - 
                               (local_state.local_vel.angular * TRACK_WIDTH / 2.0f);
        PID_Update(&speed_pid_left);
        leftMotorControl(speed_pid_left.Out);
        
        speed_pid_right.Target = target_right;
        speed_pid_right.Actual = local_state.local_vel.linear + 
                                (local_state.local_vel.angular * TRACK_WIDTH / 2.0f);
        PID_Update(&speed_pid_right);
        rightMotorControl(speed_pid_right.Out);
        
        // 7. ��ȷ�ӳ�
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
				//vTaskDelay(10);
    }
}

void decision_task(void) {
    const TickType_t xPeriod = pdMS_TO_TICKS(100); // 100ms��������
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // ȫ��·�������� (��������ϵ)
    const Point2D_t global_waypoints[] = {
        {1.0f, 0.0f},   // ǰ��1��
        {1.0f, 1.0f},   // ��ת90��
        {0.0f, 1.0f},   // ǰ��2��
        {0.0f, 0.0f}    // �ص����
    };
    
    NavCmd_t nav_cmd = {0};
    
		RobotState_t local_state;
		Point2D_t target;
		float dx;
		float dy;
		float distance;
		float target_yaw;
    while(1) {
        
        // 1. ��ȫ��ȡ״̬����
//        if(xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&local_state, &g_robot_state, sizeof(RobotState_t));
//            xSemaphoreGive(xStateMutex);
//        }
        
        // 2. ·�������߼�
        if (local_state.current_waypoint < sizeof(global_waypoints)/sizeof(global_waypoints[0])) {
            // ����ȫ��Ŀ��
            target = global_waypoints[local_state.current_waypoint];
            
            // ���㵽Ŀ��ľ���
             dx = target.x - local_state.global.x;
             dy = target.y - local_state.global.y;
             distance = sqrtf(dx*dx + dy*dy);
            
            // ����Ŀ��ƫ���� (ȫ������ϵ)
             target_yaw = atan2f(dy, dx) * 180.0f / PI;
            
            // ����Ƿ񵽴�Ŀ��
            if (distance < 0.05f) { // 5cm�ݲ�
                local_state.current_waypoint++;
                
                // ����ȫ��״̬

								g_robot_state.current_waypoint = local_state.current_waypoint;
								g_robot_state.target_yaw = target_yaw;

                
                // �������һ�����ֹͣ
                if (local_state.current_waypoint >= sizeof(global_waypoints)/sizeof(global_waypoints[0])) {
                    nav_cmd.base_speed = 0.0f;
                    nav_cmd.target_angle = local_state.global.yaw;
                }
                
                // ����ͣ��
                vTaskDelay(200);
            } else {
                // ���õ���ָ��
                nav_cmd.base_speed = fminf(0.5f, distance * 0.5f); // �ӽ�ʱ����
                nav_cmd.target_angle = target_yaw;
                
                // ����ȫ��״̬
								g_robot_state.target_yaw = target_yaw;
								g_robot_state.target_speed = nav_cmd.base_speed;
            }
        } else {
            // ����·�������
            nav_cmd.base_speed = 0.0f;
            nav_cmd.target_angle = local_state.global.yaw;
        }
        
        // 3. ���͵���ָ��
//        xQueueSend(nav_cmd_queue, &nav_cmd, portMAX_DELAY);
        
        // 4. ��ȷ�ӳ�
			vTaskDelayUntil(&xLastWakeTime, xPeriod);
				//vTaskDelay(100);
    }
}

void state_update_task(void) {
    const TickType_t xPeriod = pdMS_TO_TICKS(10); // 10ms��������
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // ״̬����
    float prev_speed = 0.0f;
    float prev_yaw = 0.0f;
    
		float left_speed;
		float right_speed;
		float current_yaw;
		float dt;
		float linear_vel;
		float angular_vel;
		float yaw_rad;
		float vx;
		float vy;
		float delta_x;
		float delta_y;
    while(1) {
        // 1. ��ȡ����������
         left_speed = leftWheelSpeed();
         right_speed = rightWheelSpeed();
         current_yaw = Att_Angle.yaw; // ����IMU
        
        // 2. ����ʱ��� (��)
         dt = xPeriod * portTICK_PERIOD_MS * 0.001f;
        
        // 3. ����ֲ��ٶ�
         linear_vel = (left_speed + right_speed) / 2.0f;
         angular_vel = (right_speed - left_speed) / TRACK_WIDTH;
        
        // 4. ����ȫ���ٶ�
         yaw_rad = current_yaw * PI / 180.0f; // תΪ����
         vx = linear_vel * cosf(yaw_rad);
         vy = linear_vel * sinf(yaw_rad);
        
        // 5. ����λ�ñ仯 (���λ�����߾���)
         delta_x = (prev_speed * cosf(prev_yaw) + 
                       linear_vel * cosf(yaw_rad)) * dt / 2.0f;
         delta_y = (prev_speed * sinf(prev_yaw) + 
                       linear_vel * sinf(yaw_rad)) * dt / 2.0f;
        
        // 6. ����ȫ��״̬ (������������)
//        if(xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
            // ����ȫ��λ��
            g_robot_state.global.x += delta_x;
            g_robot_state.global.y += delta_y;
            g_robot_state.global.yaw = current_yaw;
            
            // ����ȫ���ٶ�
            g_robot_state.global.vx = vx;
            g_robot_state.global.vy = vy;
            
            // ���¾ֲ��ٶ�
            g_robot_state.local_vel.linear = linear_vel;
            g_robot_state.local_vel.angular = angular_vel;
            
            // ����ϵͳ״̬
            g_robot_state.update_count++;
            
//            xSemaphoreGive(xStateMutex);
//        }
        
        // 7. ���浱ǰֵ���´�ʹ��
        prev_speed = linear_vel;
        prev_yaw = yaw_rad;
        
        // 8. ��ȷ�ӳ�
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
				//vTaskDelay(10);
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//	OLED_Init();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	uart_init();
	MPU6050_Init();
	motorInit();
//	xTaskCreate((TaskFunction_t )led1_task,
//					(const char*    )"led1_task",
//					(uint16_t       )128,
//					(void*          )NULL,
//					(UBaseType_t    )1,
//					(TaskHandle_t*  )&led1_TaskHandle_t);
	xTaskCreate((TaskFunction_t )led2_task,
					(const char*    )"led2_task",
					(uint16_t       )128,
					(void*          )NULL,
					(UBaseType_t    )1,
					(TaskHandle_t*  )&led2_TaskHandle_t); 
//	xTaskCreate((TaskFunction_t )filter_task,
//					(const char*    )"filter_task",
//					(uint16_t       )128,
//					(void*          )NULL,
//					(UBaseType_t    )1,
//					(TaskHandle_t*  )&filter_TaskHandle_t);
// 	xTaskCreate((TaskFunction_t )servo_task,
//					(const char*    )"servo_task",
//					(uint16_t       )128,
//					(void*          )NULL,
//					(UBaseType_t    )1,
//					(TaskHandle_t*  )&servo_TaskHandle_t);
//	xTaskCreate((TaskFunction_t )oled_task,
//					(const char*    )"oled_task",
//					(uint16_t       )128,
//					(void*          )NULL,
//					(UBaseType_t    )1,
//					(TaskHandle_t*  )&oled_TaskHandle_t);
	xTaskCreate((TaskFunction_t )usart_task,
					(const char*    )"usart_task",
					(uint16_t       )128,
					(void*          )NULL,
					(UBaseType_t    )1,
					(TaskHandle_t*  )&usart_TaskHandle_t);				
	xTaskCreate((TaskFunction_t )mpu6050_task,
					(const char*    )"mpu6050_task",
					(uint16_t       )128,
					(void*          )NULL,
					(UBaseType_t    )3,
					(TaskHandle_t*  )&mpu6050_TaskHandle_t);				
	xTaskCreate((TaskFunction_t )motor_task,
					(const char*    )"motor_task",
					(uint16_t       )256,
					(void*          )NULL,
					(UBaseType_t    )5,
					(TaskHandle_t*  )&motor_TaskHandle_t);		
	xTaskCreate((TaskFunction_t )decision_task,
					(const char*    )"decision_task",
					(uint16_t       )256,  // ��Ҫ�ϴ�ջ�ռ�
					(void*          )NULL,
					(UBaseType_t    )4,    // �е����ȼ�
					(TaskHandle_t*  )&decision_TaskHandle_t);	
	xTaskCreate((TaskFunction_t )state_update_task,
					(const char*    )"state_update_task",
					(uint16_t       )256,  // ��Ҫ�ϴ�ջ�ռ�
					(void*          )NULL,
					(UBaseType_t    )3,    // �е����ȼ�
					(TaskHandle_t*  )&state_update_TaskHandle_t);				
					
	vTaskStartScheduler();  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// �ѷ���ʧ�ܹ���
void vApplicationMallocFailedHook(void) {
    while(1) {
        // �ڴ����ʧ�ܴ���
    }
}

// ��ջ�������
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    while(1) {
        // ��ջ�������
    }
}

// ����������
void vApplicationIdleHook(void) {
    // ����ӵ͹��Ĵ���
}



//uint8_t MPU6050_InIt(void)
//{
//    uint8_t check;
//    uint8_t Data;
//    
//    // 1. ����豸�Ƿ�����
//    if (HAL_I2C_IsDeviceReady(&hi2c2, MPU6050_ADDR << 1, 3, 100) != HAL_OK) {
//        return 1; // �豸δ��Ӧ
//    }
//    
//    // 2. ����豸ID (WHO_AM_I �Ĵ���)
//    if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR << 1, 0x75, I2C_MEMADD_SIZE_8BIT, &check, 1, 1000) != HAL_OK) {
//        return 1; // ��ȡʧ��
//    }
//    
//    if (check != 0x68) { // �豸IDӦΪ0x68
//        return 1; // ��ʼ��ʧ��
//    }
//    
//    // 3. �����豸����ѡ���Ƽ���
//    Data = 0x80; // ��λλ
//    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x6B, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
//        return 1; // ��λʧ��
//    }
//    
//    HAL_Delay(100); // �ȴ���λ���
//    
//    // 4. ����MPU6050��ѡ��ʱ��Դ
//    Data = 0x01; // ѡ��X����������Ϊʱ��Դ
//    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x6B, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
//        return 1; // ����ʧ��
//    }
//    
//    // 5. ���ü��ٶȼ����� (��8g)
//    Data = 0x10; // ��8g (������ 4096 LSB/g)
//    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x1C, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
//        return 1; // ���ٶȼ���������ʧ��
//    }
//    
//    // 6. �������������� (��2000��/s)
//    Data = 0x18; // ��2000��/s (������ 16.4 LSB/��/s)
//    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x1B, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
//        return 1; // ��������������ʧ��
//    }
//    
//    // 7. �������ֵ�ͨ�˲��� (DLPF)
//    Data = 0x06; // ���� 5Hz
//    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
//        return 1; // �˲�������ʧ��
//    }
//    
////    // 8. ���������жϣ���ѡ��
////    Data = 0x00;
////    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x38, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
////        return 1; // �ж�����ʧ��
////    }
////    
////    // 9. ����FIFO����ѡ��
////    Data = 0x00;
////    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x23, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
////        return 1; // FIFO����ʧ��
////    }
//    
//    return 0; // ��ʼ���ɹ�
//}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
