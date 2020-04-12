/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "load_task.h"
#include "remote_control.h"
#include "motor_6020.h"
#include "L298.h"
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
extern uint8_t s1c,s2c;
uint8_t  r0=0,r1=0;//表示右摇杆上次状态，0表示上次居中，1表示上次超过正负临界值，上次为0且本次超值才会转
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const RC_ctrl_t *local_rc_ctrl;

//限位开关中断，触发1电机反转，触发2电机停止
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == SWITCH1_Pin)
    {
        give_current0=-give_current0;//碰到开关1后反转
    }
		else if(GPIO_Pin == SWITCH2_Pin)
    {
        give_current0=0;//碰到开关2后反停止
    }
		else;
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
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
remote_control_init();
imu_communication_init();
local_rc_ctrl = get_remote_control_point();
init(&chassis_move[0],1);
init(&chassis_move[1],2);
Servo_Control(35);//舵机位置初始化
MOTOR_6020_Init();//底部方位角初始化
Linear_Init();//仰角初始化
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//三位开关S2
		if(s2c==1&&local_rc_ctrl->rc.s[1]==3){
		Fire();
		}
		if(s2c==1&&local_rc_ctrl->rc.s[1]==0){
		Load();
		}
		
		//三位开关S1
		if(s1c==1){
		if(local_rc_ctrl->rc.s[0]==1){	//复位
			MOTOR_6020_Init();
			Linear_Init();
		}
		if(local_rc_ctrl->rc.s[0]==2){	//前哨站方向
			MOTOR_6020_POS(ROLL_1);
			Linear_Angel(PITCH_1);
		}		
		if(local_rc_ctrl->rc.s[0]==3){	//基地方向
			MOTOR_6020_POS(ROLL_2);
			Linear_Angel(PITCH_2);
		}		
		}
		
		//右摇杆
		if(local_rc_ctrl->rc.ch[0]<REMID &&local_rc_ctrl->rc.ch[0]>-REMID){
		r0=0;
		}
		if(r0==0&&local_rc_ctrl->rc.ch[0]>CRITICAL){
		MOTOR_6020_ANGLE(0);//左右需根据实际修改！！同下
		r0=1;
		}
		else if(r0==0&&local_rc_ctrl->rc.ch[0]<-CRITICAL){
		MOTOR_6020_ANGLE(1);
		r0=1;
		}
		
		if(local_rc_ctrl->rc.ch[1]<REMID &&local_rc_ctrl->rc.ch[1]>-REMID){
		r1=0;
		}
		if(r1==0&&local_rc_ctrl->rc.ch[1]>CRITICAL){
		Linear_Angel(PITCH_FIX);//正负根据实际修改
		r1=1;
		}
		else if(r1==0&&local_rc_ctrl->rc.ch[1]<-CRITICAL){
		Linear_Angel(-PITCH_FIX);
		r1=1;
		}
				
		//左摇杆
		while(local_rc_ctrl->rc.ch[2]>REMID){
		MOTOR_6020_FIX(1);					//左右需根据实际修改！！同下
		}
		while(local_rc_ctrl->rc.ch[2]<-REMID){
		MOTOR_6020_FIX(0);				
		}
		while(local_rc_ctrl->rc.ch[3]>REMID){
		Linear_Speed(PITCH_SPEED);
		}
		while(local_rc_ctrl->rc.ch[3]<-REMID){
		Linear_Speed(-PITCH_SPEED);
		}
		
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
