/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define m_DirPort GPIOC
#define AIN1 GPIO_PIN_0
#define AIN2 GPIO_PIN_1
#define BIN1 GPIO_PIN_2
#define BIN2 GPIO_PIN_3
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Left_Go()
{
		HAL_GPIO_WritePin(m_DirPort,AIN1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(m_DirPort,AIN2,GPIO_PIN_RESET);
}

void Right_Go()
{
		HAL_GPIO_WritePin(m_DirPort,BIN1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(m_DirPort,BIN2,GPIO_PIN_RESET);
}

void Left_Back()
{
		HAL_GPIO_WritePin(m_DirPort,AIN1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m_DirPort,AIN2,GPIO_PIN_SET);
}

void Right_Back()
{
		HAL_GPIO_WritePin(m_DirPort,BIN1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m_DirPort,BIN2,GPIO_PIN_SET);
}

void Go_Back()
{
		HAL_GPIO_WritePin(m_DirPort,AIN1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m_DirPort,AIN2,GPIO_PIN_SET);

		HAL_GPIO_WritePin(m_DirPort,BIN1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m_DirPort,BIN2,GPIO_PIN_SET);
}

void Left_Stop()
{
		HAL_GPIO_WritePin(m_DirPort,AIN1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m_DirPort,AIN2,GPIO_PIN_RESET);
}

void Right_Stop()
{
		HAL_GPIO_WritePin(m_DirPort,BIN1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m_DirPort,BIN2,GPIO_PIN_RESET);
}


/**
*	@brief Control Motor Speed
*	@param PWM
*	@retval None
*/
void MotorControl(char direction,int leftMotorPWM, int rightMotorPWM)
{
	  switch(direction)
		{
			case 0:
						Left_Go();
						Right_Go();
						__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,leftMotorPWM);
						__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,rightMotorPWM);
			    break;

			case 1:
					Left_Back();
			    Right_Back();
			    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,leftMotorPWM);
					__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,rightMotorPWM);
			    break;

			case 2:
						Left_Stop();
			     Right_Stop();
				    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
						__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
			    break;
			default:break;
		}
}


void Set_Servo_angle(TIM_HandleTypeDef * htim,uint32_t Channel,uint8_t angle)  //舵机驱动
{
	uint16_t compare_value=0;
	if(angle <= 180)   //限制角度为180°
	{
		compare_value=0.5*2000/20+angle*2000/20/90;		//角度转化为数值
		__HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
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
  MX_TIM8_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  //启动定时�??4的PWM模式
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  MotorControl(0,500,500); //设定电机初始速度
  Set_Servo_angle(&htim4,TIM_CHANNEL_3,90);  //方向回正
  HAL_Delay(2000);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    #ifdef ServoControl
//	   MotorControl(0,500,500);
//	   HAL_Delay(1000);
//       Set_Servo_angle(&htim4,TIM_CHANNEL_3,60); //左转30°
//       HAL_Delay(1000);
//       Set_Servo_angle(&htim4,TIM_CHANNEL_3,90);  //方向回正
//       HAL_Delay(1000);
//       Set_Servo_angle(&htim4,TIM_CHANNEL_3,120);  //右转30°
//       HAL_Delay(1000);
//       Set_Servo_angle(&htim4,TIM_CHANNEL_3,90);  //方向回正
//       HAL_Delay(1000);
//       MotorControl(2,500,500);
//       HAL_Delay(1000);
//       MotorControl(1,500,500);
//       Set_Servo_angle(&htim4,TIM_CHANNEL_3,120); //左转30°
//       HAL_Delay(1000);
//       Set_Servo_angle(&htim4,TIM_CHANNEL_3,90);  //方向回正
//       HAL_Delay(1000);
//       Set_Servo_angle(&htim4,TIM_CHANNEL_3,60);  //右转30°
//       HAL_Delay(1000);
//       Set_Servo_angle(&htim4,TIM_CHANNEL_3,90);  //方向回正
//       HAL_Delay(2000);
//       MotorControl(2,500,500);
//       HAL_Delay(1000);

	  Set_Servo_angle(&htim4,TIM_CHANNEL_3,60); //左转30°
	  HAL_Delay(2000);
	  Set_Servo_angle(&htim4,TIM_CHANNEL_3,90);  //方向回正
	  HAL_Delay(2000);
	  Set_Servo_angle(&htim4,TIM_CHANNEL_3,120);  //右转30°
	  HAL_Delay(2000);
    #endif

	#ifdef DifferentialSteering
	  HAL_Delay(2000);
     MotorControl(0,500,500); //直行
      HAL_Delay(2000);
     MotorControl(2,0,0);     //停止
      HAL_Delay(2000);
     MotorControl(1,500,500); //后退
      HAL_Delay(2000);
     MotorControl(0,0,500);  //前进左转
       HAL_Delay(2000);
     MotorControl(0,500,0);  //前进右转
       HAL_Delay(2000);
     MotorControl(1,0,500);  //左转退回
       HAL_Delay(2000);
     MotorControl(1,500,0);  //右转退回
       //HAL_Delay(2000);
	#endif
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

/* USER CODE END 4 */

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
