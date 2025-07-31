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
#include "mutex_lock_and_message_queue.h"
#include "data_protocol.h"
#include "eletube.h"
#include "timer.h"
#include <stdbool.h>
#include "keys.h"

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

// 定义机器人状态互斥锁实例
CustomMutex_t robot_state_mutex = {
    .locked = 0,
    .owner = NULL,
    .name = "robot_state_mutex",
    .lock_count = 0
};

// 定义导航指令消息队列（缓冲区大小为5个消息）
NavCmd_t nav_queue_buffer[NAV_QUEUE_SIZE];  // 消息缓冲区
CustomQueue_t nav_cmd_queue;                // 导航指令队列实例


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
TaskHandle_t  openmv_TaskHandle_t;
TaskHandle_t  eletube_TaskHandle_t;
TaskHandle_t  debug_send_TaskHandle_t;



/* 在全局变量区域添加 */
//SemaphoreHandle_t xSensorMutex;  // 传感器数据互斥锁
//float pitch, roll, yaw;
//RobotState_t g_robot_state = {0};
//SemaphoreHandle_t xStateMutex;

//void led1_task() {
//	while(1){
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
//		vTaskDelay(1000);
//	}
//}

//void led2_task(){
//	while(1){
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
//		vTaskDelay(2000);
//	}
//}

//Filter_t Data_Filter = {0.8,0,0};

//AVG_Flt_t AVG_Filter;

//MEDIAN_Flt_t MEDIAN_Filter;

////FIR_Flt_t FIR_Filter;

//Klm_Flt_t KLM_Filter = 
//{
//	0.02,//估算协方差
//	0,//卡尔曼增益
//	0.001,//过程噪声协方差，Q增大，动态响应变快，收敛稳定性变坏
//	0.1,//测量噪声协方差，R增大，动态响应变慢，收敛稳定性变好
//	0//卡尔曼滤波输出
//};

//double Data;
//double Result;
//double t;
//double Test;


//void filter_task(){
//	//低通滤波测试
//	while(1){
//		Data = sin(t)+0.1*sin(50*t);
//		Result = Low_Pass_Filter(&Data_Filter,Data);
//		t+=0.01;
//		while(huart1.gState != HAL_UART_STATE_READY) {
//			vTaskDelay(1); // 在FreeRTOS中让出CPU
//		}
//	debug_printf("%.3f,%.3f\n",Data,Result);
//		vTaskDelay(50);
//	}
//	
	
//	//滑动均值滤波测试
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

////中值滤波测试
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

////带通滤波测试
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
	
////卡尔曼滤波测试
//while(1){
//	Data = sin(200*PI*t)+sin(2000*PI*t)+sin(4000*PI*t);
//	Test = sin(2000*PI*t);
//	Result = Median_Filter(&MEDIAN_Filter,Data);
//	t += 0.0001;
////	debug_printf("%.3f,%.3f\n",Data,Result);
//	vTaskDelay(50);
//	}

//}

//SystemState current_state = STATE_IDLE;
//void servo_task(){    
//	
//		// 启动PWM
//		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//		
//		// 初始复位
//		Reset_Position();
//		
//    while(1) {
//			switch (current_state) {
//      case STATE_RESET:
//        Reset_Position();
//        current_state = STATE_IDLE;
//        break;
//        
//      case STATE_SQUARE_MOVE:
//        Move_Square_Path();
//        current_state = STATE_IDLE;
//        break;
//        
//      case STATE_CUSTOM_PATH:
//        // 此处添加自定义路径数据
//        // Move_Custom_Path(custom_path, path_points);
//        current_state = STATE_IDLE;
//        break;
//        
//      case STATE_IDLE:
//      default:
//        
//        vTaskDelay(100); // 调整延时控制变化速度
//			 break;
//			}
//    }
//}


///* 修改 oled_task 函数 */
//void oled_task() {
//    char displayBuffer[64];  // 显示缓冲区
//    
//    while(1) {
//        // 使用互斥锁获取传感器数据
////        if(xSemaphoreTake(xSensorMutex, portMAX_DELAY) == pdTRUE) {
////            for(int i = 0; i < 3; i++) {
////                accel[i] = g_accel[i];
////                gyro[i] = g_gyro[i];
////            }
////            xSemaphoreGive(xSensorMutex);
////        }
//		
//        // 清屏
//        OLED_FullyClear();
//        
//        // 显示陀螺仪数据
//        snprintf(displayBuffer, sizeof(displayBuffer), "pitch:%.2f", Att_Angle.pitch);
//        OLED_ShowStr(0, 0, (unsigned char *)displayBuffer, 2);
//        
//        snprintf(displayBuffer, sizeof(displayBuffer), "roll:%.2f", Att_Angle.roll);
//        OLED_ShowStr(0, 3, (unsigned char *)displayBuffer, 2);
//        
//        snprintf(displayBuffer, sizeof(displayBuffer), "yaw:%.2f", Att_Angle.yaw);
//        OLED_ShowStr(0, 6, (unsigned char *)displayBuffer, 2);
//        
//        vTaskDelay(200); // OLED刷新率控制
//    }
//}



void usart_task(){
	while (1){
			if(!keyflag){
				key1();
				key2();
				key3();
				key4();
			}
			vTaskDelay(10);
  }
}


int position;
int task1_cnt = 0;
uint8_t task1_path_cnt = 0;
uint8_t task1_flag = 0;
Point2D_t start,end;
Angles_t path[INTERPOLATION_STEPS];
void motor_task(void) {  
    while(1) {			
				if(task1_flag){
						if(!motor_x.is_moving&&!motor_y.is_moving){
							Move_Angle_Global(&motor_x,path[task1_path_cnt].yaw,1000);
							Move_Angle_Global(&motor_y,path[task1_path_cnt].pitch,1000);
							task1_path_cnt++;
						}
						if(task1_path_cnt==INTERPOLATION_STEPS)
								task1_flag = 0;
				}
				vTaskDelay(10);
    }

}

void decision_task(void) {  
    while(1) {			
				if(keyflag && (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_SET)){
						vTaskDelay(100);
						if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_SET){
										if(task1_cnt==5){
												task1_cnt=0;
										}
										while(task1_cnt!=5){
											if(!motor_x.is_moving&&!motor_y.is_moving&&!task1_flag){
													task1_path_cnt = 0;
													if(task1_cnt==0){
													start.x = target_x[4];
													start.y = target_y[4];
													end.x = target_x[0];
													end.y = target_y[0];
													generate_linear_path(start,end,path);
													}else{
															start.x = target_x[task1_cnt-1];
															start.y = target_y[task1_cnt-1];
															end.x = target_x[task1_cnt];
															end.y = target_y[task1_cnt];
															generate_linear_path(start,end,path);
													}
													task1_cnt++;
													task1_flag = 1;
											}
											vTaskDelay(10);
									}
						}
				}
					
				vTaskDelay(10);
    }

}



//// 全局调试信息变量
//DebugInfo_t g_debug_info;
//CustomMutex_t debug_info_mutex = {.locked = 0,.owner = NULL,.name = "debug_info_mutex",.lock_count = 0};

//void debug_send_task(void) {
//    const TickType_t xPeriod = pdMS_TO_TICKS(500); // 发送周期
//    TickType_t xLastWakeTime = xTaskGetTickCount();
//    
//    while(1) {
//        DebugInfo_t local_info;
//        
//        // 安全复制调试信息
//        CustomMutex_Lock(&debug_info_mutex);
//        memcpy(&local_info, &g_debug_info, sizeof(DebugInfo_t));
//        CustomMutex_Unlock(&debug_info_mutex);
//        
//        // 发送调试信息
//        debug_printf("%lu,%.2f,%.2f,%.2f,%.2f\n", 
//                     local_info.timestamp,
//                     local_info.target_angle,
//                     local_info.current_yaw,
//                     local_info.angle_adjust,
//                     local_info.base_speed);
//        
//        // 精确延迟
//        vTaskDelayUntil(&xLastWakeTime, xPeriod);
//    }
//}



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
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	// 步进电机系统初始化
//	// 初始化自定义消息队列（导航指令队列）
//CustomQueue_Init(&nav_cmd_queue, 
//                nav_queue_buffer, 
//                sizeof(NavCmd_t), 
//                NAV_QUEUE_SIZE, 
//                "nav_cmd_queue");
//								
//	OLED_Init();
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//	uart_init();
//	MPU6050_Init();
//	motorInit();
//	xTaskCreate((TaskFunction_t )led1_task,
//					(const char*    )"led1_task",
//					(uint16_t       )128,
//					(void*          )NULL,
//					(UBaseType_t    )1,
//					(TaskHandle_t*  )&led1_TaskHandle_t);
//	xTaskCreate((TaskFunction_t )led2_task,
//					(const char*    )"led2_task",
//					(uint16_t       )128,
//					(void*          )NULL,
//					(UBaseType_t    )1,
//					(TaskHandle_t*  )&led2_TaskHandle_t); 
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
	xTaskCreate((TaskFunction_t )decision_task,
					(const char*    )"decision_task",
					(uint16_t       )256,
					(void*          )NULL,
					(UBaseType_t    )1,
					(TaskHandle_t*  )&decision_TaskHandle_t);
	xTaskCreate((TaskFunction_t )usart_task,
					(const char*    )"usart_task",
					(uint16_t       )128,
					(void*          )NULL,
					(UBaseType_t    )1,
					(TaskHandle_t*  )&usart_TaskHandle_t);							
	xTaskCreate((TaskFunction_t )motor_task,
					(const char*    )"motor_task",
					(uint16_t       )256,
					(void*          )NULL,
					(UBaseType_t    )1,
					(TaskHandle_t*  )&motor_TaskHandle_t);						
//	xTaskCreate((TaskFunction_t )debug_send_task,
//					(const char*    )"debug_send_task",
//					(uint16_t       )128,  // 
//					(void*          )NULL,
//					(UBaseType_t    )3,    // 中等优先级
//					(TaskHandle_t*  )&debug_send_TaskHandle_t);
					
					
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



//// 按键中断处理
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//  if (GPIO_Pin == GPIO_PIN_0) {
//    static uint8_t press_count = 0;
//    press_count = (press_count + 1) % 3;
//    
//    switch (press_count) {
//      case 0: current_state = STATE_RESET; break;
//      case 1: current_state = STATE_SQUARE_MOVE; break;
//      case 2: current_state = STATE_CUSTOM_PATH; break;
//    }
//  }
//}



// 堆分配失败钩子
void vApplicationMallocFailedHook(void) {
    while(1) {
        // 内存分配失败处理
    }
}

// 堆栈溢出钩子
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    while(1) {
        // 堆栈溢出处理
    }
}

// 空闲任务钩子
void vApplicationIdleHook(void) {
    // 可添加低功耗代码
}

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
	if (htim == motor_x.timer && motor_x.step_count < motor_x.step_target) {
			motor_x.step_count++;
			if(motor_x.direction)
				motor_x.step_sum--;
			else
				motor_x.step_sum++;
			if (motor_x.step_count == motor_x.step_target) {
					motor_x.is_moving = 0;
					motor_x.current_angle = motor_x.step_angle*motor_x.step_sum;
					HAL_TIM_PWM_Stop_IT(motor_x.timer, motor_x.timer_channel);
			}
	}
		if (htim == motor_y.timer && motor_y.step_count < motor_y.step_target) {
			motor_y.step_count++;
			if(motor_y.direction)
				motor_y.step_sum--;
			else
				motor_y.step_sum++;
			if (motor_y.step_count == motor_y.step_target) {
					motor_y.is_moving = 0;
					motor_y.current_angle = motor_y.step_angle*motor_y.step_sum;
					HAL_TIM_PWM_Stop_IT(motor_y.timer, motor_y.timer_channel);
			}
	}
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
