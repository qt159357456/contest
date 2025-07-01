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


/* 在全局变量区域添加 */
float g_accel[3] = {0};  // 全局加速度数据
float g_gyro[3] = {0};   // 全局陀螺仪数据
SemaphoreHandle_t xSensorMutex;  // 传感器数据互斥锁


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
	0.02,//估算协方差
	0,//卡尔曼增益
	0.001,//过程噪声协方差，Q增大，动态响应变快，收敛稳定性变坏
	0.1,//测量噪声协方差，R增大，动态响应变慢，收敛稳定性变好
	0//卡尔曼滤波输出
};

double Data;
double Result;
double t;
double Test;


void filter_task(){
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
	
//卡尔曼滤波测试
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
//void servo_task(){
////		const uint16_t servo_positions[] = {
////			SERVO_MIN_PULSE,   // 0°
////			SERVO_MIN_PULSE + (SERVO_MAX_PULSE - SERVO_MIN_PULSE) * 1/4,  // 45°
////			SERVO_MIN_PULSE + (SERVO_MAX_PULSE - SERVO_MIN_PULSE) * 2/4,  // 90°
////			SERVO_MIN_PULSE + (SERVO_MAX_PULSE - SERVO_MIN_PULSE) * 3/4,  // 135°
////			SERVO_MAX_PULSE    // 180°			 
////	};
//	while(1){
//			Servo_SetPWM(&htim1, TIM_CHANNEL_1,test_pulse);
//			vTaskDelay(50);  // FreeRTOS 延时
//	}

////    for(;;) {
////        /* 舵机位置循环 */
////        for(uint8_t pos = 0; pos < 5; pos++) {
////            Servo_SetPulse(&htim2, TIM_CHANNEL_1, servo_positions[pos]);
////            vTaskDelay(300);  // FreeRTOS 延时
////        }
////        
////        /* 舵机平滑运动 (180° -> 0°) */
////        for(uint16_t pulse = SERVO_MAX_PULSE; pulse > SERVO_MIN_PULSE; pulse--) {
////            Servo_SetPulse(&htim2, TIM_CHANNEL_1, pulse);
////            vTaskDelay(2);  // 控制运动速度
////        }
////        
////        vTaskDelay(100);
////        
////        /* 舵机平滑运动 (0° -> 180°) */
////        for(uint16_t pulse = SERVO_MIN_PULSE; pulse < SERVO_MAX_PULSE; pulse++) {
////            Servo_SetPulse(&htim2, TIM_CHANNEL_1, pulse);
////            vTaskDelay(2);  // 控制运动速度
////        }
////    }

//}
void servo_task(){
		uint16_t brightness = 0;
    int direction = 1; // 1:增加亮度, 0:减小亮度
    
    while(1) {
        // 呼吸灯效果
        Servo_SetPWM(&htim1, TIM_CHANNEL_1, brightness);
        
        if(direction) {
            brightness += 100;
            if(brightness >= 19999) direction = 0;
        } else {
            brightness -= 100;
            if(brightness == 0) direction = 1;
        }
        
        vTaskDelay(10); // 调整延时控制变化速度
    }
}


/* 修改 oled_task 函数 */
void oled_task() {
    float accel[3], gyro[3];
    char displayBuffer[64];  // 显示缓冲区
    
    while(1) {
        // 使用互斥锁获取传感器数据
        if(xSemaphoreTake(xSensorMutex, portMAX_DELAY) == pdTRUE) {
            for(int i = 0; i < 3; i++) {
                accel[i] = g_accel[i];
                gyro[i] = g_gyro[i];
            }
            xSemaphoreGive(xSensorMutex);
        }
        
        // 清屏
        OLED_FullyClear();
        
        // 显示加速度数据
        snprintf(displayBuffer, sizeof(displayBuffer), "A.X:%.2fg", accel[0]);
        OLED_ShowStr(0, 0, (unsigned char *)displayBuffer, 1);
        
        snprintf(displayBuffer, sizeof(displayBuffer), "A.Y:%.2fg", accel[1]);
        OLED_ShowStr(0, 10, (unsigned char *)displayBuffer, 1);
        
        snprintf(displayBuffer, sizeof(displayBuffer), "A.Z:%.2fg", accel[2]);
        OLED_ShowStr(0, 20, (unsigned char *)displayBuffer, 1);
        
        // 显示陀螺仪数据
        snprintf(displayBuffer, sizeof(displayBuffer), "G.X:%.2f/s", gyro[0]);
        OLED_ShowStr(0, 35, (unsigned char *)displayBuffer, 1);
        
        snprintf(displayBuffer, sizeof(displayBuffer), "G.Y:%.2f/s", gyro[1]);
        OLED_ShowStr(0, 45, (unsigned char *)displayBuffer, 1);
        
        snprintf(displayBuffer, sizeof(displayBuffer), "G.Z:%.2f/s", gyro[2]);
        OLED_ShowStr(0, 55, (unsigned char *)displayBuffer, 1);
        
        vTaskDelay(200); // OLED刷新率控制
    }
}



	//// 开启printf的使用，定位到串口1
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
		debug_printf("123456\n");
		vTaskDelay(500);
    // open the uart1 reciver
  }
}

void mpu6050_task(){  
    if(MPU6050_Init() != 0) {
        // 初始化失败处理
        while(1);
    }
    
    while(1) {
        float accel[3], gyro[3];
        MPU6050_Read_Accel(accel);
        MPU6050_Read_Gyro(gyro);
        
        // 使用互斥锁保护全局变量
        if(xSemaphoreTake(xSensorMutex, portMAX_DELAY) == pdTRUE) {
            for(int i = 0; i < 3; i++) {
                g_accel[i] = accel[i];
                g_gyro[i] = gyro[i];
            }
            xSemaphoreGive(xSensorMutex);
        }
        
        vTaskDelay(20);
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
  /* USER CODE BEGIN 2 */
//	OLED_Init();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	uart_init();
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
					(UBaseType_t    )2,
					(TaskHandle_t*  )&led2_TaskHandle_t); 
	xTaskCreate((TaskFunction_t )filter_task,
					(const char*    )"filter_task",
					(uint16_t       )256,
					(void*          )NULL,
					(UBaseType_t    )3,
					(TaskHandle_t*  )&filter_TaskHandle_t);
 	xTaskCreate((TaskFunction_t )servo_task,
					(const char*    )"servo_task",
					(uint16_t       )256,
					(void*          )NULL,
					(UBaseType_t    )4,
					(TaskHandle_t*  )&servo_TaskHandle_t);
	xTaskCreate((TaskFunction_t )oled_task,
					(const char*    )"oled_task",
					(uint16_t       )128,
					(void*          )NULL,
					(UBaseType_t    )5,
					(TaskHandle_t*  )&oled_TaskHandle_t);
//	xTaskCreate((TaskFunction_t )usart_task,
//					(const char*    )"usart_task",
//					(uint16_t       )128,
//					(void*          )NULL,
//					(UBaseType_t    )6,
//					(TaskHandle_t*  )&usart_TaskHandle_t);				
	xTaskCreate((TaskFunction_t )mpu6050_task,
					(const char*    )"mpu6050_task",
					(uint16_t       )128,
					(void*          )NULL,
					(UBaseType_t    )7,
					(TaskHandle_t*  )&mpu6050_TaskHandle_t);				
					
					
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
