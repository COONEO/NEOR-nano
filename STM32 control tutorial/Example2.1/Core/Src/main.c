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
#include "usart.h"
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

#define filterNum 3

//#define Velocity_Kp 15.0
//#define Velocity_Ki 1.00
//#define Velocity_Kd 2.0

#define Velocity_Kp 15.0
#define Velocity_Ki 1.0
#define Velocity_Kd 0.0


#define limit_value 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
short leftEncoder[filterNum] = {0};
short rightEncoder[filterNum] = {0};
short encoderPulse[2]={0};
float leftSpeed = 0,rightSpeed = 0;
float leftTargetSpeed = 0;
float rightTargetSpeed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

PID LeftMotor_PID;
PID RightMotor_PID;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int myabs(int a)
{
	int temp;
	if(a<0)
	  temp=-a;
	else
	  temp=a;
	return temp;
}

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

void MotorControl(int leftMotorPWM, int rightMotorPWM)
{
		if(leftMotorPWM>=0&&rightMotorPWM>=0)
		{
//			printf("111\r\n");
				Left_Go();
				Right_Go();
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,leftMotorPWM);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,rightMotorPWM);
//		printf("leftPWM is %d\r\n",leftMotorPWM);
//		printf("rightPWM is %d\r\n",rightMotorPWM);
		}

		else if(leftMotorPWM<0&&rightMotorPWM<0)
		{
				Left_Back();
				Right_Back();
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,myabs(leftMotorPWM));
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,myabs(rightMotorPWM));
		}

		else if(leftMotorPWM==0&&rightMotorPWM==0)
		{
				Left_Stop();
				Right_Stop();
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
		}
}

void Set_Servo_angle(TIM_HandleTypeDef * htim,uint32_t Channel,uint8_t angle)  //舵机驱动
{
	uint16_t compare_value=0;
	if(angle <= 180)   //限制角度�??180°
	{
		compare_value=0.5*2000/20+angle*2000/20/90;		//角度转化为数�??
		__HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
	}
}


void GetEncoderPulse()
{
	  encoderPulse[0] = -((short)__HAL_TIM_GET_COUNTER(&htim2));
	  encoderPulse[1] = -((short)__HAL_TIM_GET_COUNTER(&htim3));

    __HAL_TIM_GET_COUNTER(&htim2) = 0;	 //计数值重新清�??
    __HAL_TIM_GET_COUNTER(&htim3) = 0;
}


float CalActualSpeed(int pulse)
{
    return (float)(0.003092424 * pulse);
}


/**
  * @brief  PID相关参数的初始化
  * @param  PID的结构体指针
  */
void PID_Init(PID *p)
{
    p->Kp = Velocity_Kp;
    p->Ki = Velocity_Ki;
    p->Kd = Velocity_Kd;
    p->last_error = 0;
    p->prev_error = 0;
    p->limit = limit_value;
    p->pwm_add = 0;
}

 /**
  * @brief  PID相关参数的初始化
  * @param  PID的结构体指针
  */
void PID_Cal(int targetSpeed,int currentSpeed,PID *p)
{
    int error = targetSpeed - currentSpeed;
    p->pwm_add += p->Kp*(error - p->last_error) + p->Ki*error + p->Kd*(error - 2*p->last_error+p->prev_error);

		p->prev_error = p->last_error;
	  p->last_error = error;

		if(p->pwm_add>p->limit) p->pwm_add=p->limit;
		if(p->pwm_add<-p->limit) p->pwm_add=-p->limit;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);  //使能定时�??6中断

  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);  //使能定时�??8的PWM模式
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  //启动定时�??4的PWM模式

  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);  //启动定时�??2的编码器模式
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);  //启动定时�??3的编码器模式
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);

  PID_Init(&LeftMotor_PID);
  PID_Init(&RightMotor_PID);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	leftTargetSpeed = 0.2;
	rightTargetSpeed = 0.2;
	//MotorControl(99,99);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //定时器6中断回调函数，每50ms调用
{
	float c_leftSpeed,c_rightSpeed;
  if(htim==(&htim6))
  {
		GetEncoderPulse();
		c_leftSpeed = CalActualSpeed(encoderPulse[0]);   //获得当前的�?�度�??
		c_rightSpeed = CalActualSpeed(encoderPulse[1]);
		printf("{Left_Speed is:%.2f m/s}\r\n",c_leftSpeed);
		printf("{right_Speed is:%.2f m/s}\r\n",c_rightSpeed);

		PID_Cal((leftTargetSpeed)*100,(c_leftSpeed)*100,&LeftMotor_PID);
		PID_Cal(rightTargetSpeed*100,c_rightSpeed*100,&RightMotor_PID);
		MotorControl(LeftMotor_PID.pwm_add,RightMotor_PID.pwm_add);
//		printf("%.2f,%.2f\r\n",c_leftSpeed,c_rightSpeed);

//		printf("leftpwmadd %d\r\n",LeftMotor_PID.pwm_add);
//		printf("rightpwmadd %d\r\n",RightMotor_PID.pwm_add);
//	  printf("currentLeftPulse is %d\r\n",encoderPulse[0]);
//		printf("currentrightPulse is %d\r\n",encoderPulse[1]);
//	  printf("----------------------------\r\n");
		 printf("{currentLeftSpeed is:%.2f}\r\n",c_leftSpeed);
		 printf("{currentrightSpeed is:%.2f}\r\n",c_rightSpeed);
  }
}
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
