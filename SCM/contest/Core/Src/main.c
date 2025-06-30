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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "global.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "math.h"
#include <stdarg.h>
#include <string.h>
#include "filter.h"
#include "servo.h"
#include "stm32f1xx_hal.h"
#include "oled.h"
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void debug_printf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void debug_printf(const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    
    // 在此处设置断点
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}
TaskHandle_t  led1_TaskHandle_t;
TaskHandle_t  led2_TaskHandle_t;
TaskHandle_t  filter_TaskHandle_t;
TaskHandle_t  servo_TaskHandle_t;
TaskHandle_t  oled_TaskHandle_t;

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
	debug_printf("%.3f,%.3f\n",Data,Result);
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


int my_num[5] = {1,2,3,4,5};
void oled_task(){
		while(1){
//			OLED_ShowCN(20, 1,0);
//			OLED_ShowCN(40, 1, 1);
//			OLED_ShowCN(60, 1, 2);
//			OLED_ShowCN(80, 1,1);
//			OLED_ShowStr1(1,1,my_num,5,1);
//			OLED_ShowStr(32, 0, (unsigned char *)":25C", 1); // 小字体温度值
//			OLED_ShowStr(0,10, (unsigned char *)"System OK", 2); 
			OLED_ShowIntNum(10,0,646556,1);
			OLED_ShowFloatNum(10,10,65.56f,1);
		vTaskDelay(1000);
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
	
  /* USER CODE BEGIN 2 */
	OLED_Init();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
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
					(uint16_t       )512,
					(void*          )NULL,
					(UBaseType_t    )3,
					(TaskHandle_t*  )&filter_TaskHandle_t);
 	xTaskCreate((TaskFunction_t )servo_task,
					(const char*    )"servo_task",
					(uint16_t       )512,
					(void*          )NULL,
					(UBaseType_t    )4,
					(TaskHandle_t*  )&servo_TaskHandle_t);
	xTaskCreate((TaskFunction_t )oled_task,
					(const char*    )"oled_task",
					(uint16_t       )128,
					(void*          )NULL,
					(UBaseType_t    )5,
					(TaskHandle_t*  )&oled_TaskHandle_t);
					
					
					
	vTaskStartScheduler();  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		vTaskDelay(10);
		}
}


/* USER CODE END 0 */



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

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
