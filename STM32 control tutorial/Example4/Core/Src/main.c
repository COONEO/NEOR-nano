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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
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

#define Velocity_Kp 4.0
#define Velocity_Ki 2.0
#define Velocity_Kd 0.2

#define limit_value 1200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
short encoderPulse[2] = {0};
short last_encoderPulse[2] = {0};
short leftEncoder[filterNum] = {0};
short rightEncoder[filterNum] = {0};
short encoderValue[2] = {0};

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

//电机初始化
void MotorInit()
{
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);  //启动定时器8的PWM模式
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);  //启动定时器2的编码器模式
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);  //启动定时器3的编码器模式
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
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

//电机控制
void MotorControl(int leftPWM, int rightPWM)
{
	if( leftPWM >= 0)
	{
		Left_Go();
	}else
	{
		Left_Back();
	}
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, myabs(leftPWM));

	if(rightPWM >= 0)
	{
		Right_Go();
	}else
	{
		Right_Back();
	}
	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, myabs(rightPWM));

}

//计算速度  cm/s
float CalActualSpeed(int pulse)
{
    return (float)(0.3092424 * pulse);
}

/**
@brief      根据实际速度计算脉冲数
@param      speed 速度，单位是 cm/s。
@return

*/
int CalActualPulse(float speed)
{
    return (int)(speed/0.003092424);//*26.821191
}

/**
* @brief    十六进制转换为十进制
						主要是将接收到的电机速度转换为十进制数进行调速
* @param  	hex 传入的十六进制数组，length 需要的数组长度
* @retval 	result 返回的数组
*/
int HextoDec(const unsigned char *hex, int length)
{
	int rslt = 0;

	for(int i=0; i<length; i++)
	{
		rslt += (unsigned long)(hex[i])<<(8*(length-1-i));
	}

	return rslt;
}


/**
* @brief	均值滤波
*	@param 需要滤波的数组，滤波次数
*	@retval 滤波后的值
*/
int AverageFilter(short *array, uint8_t num)//均值滤波
{
    int tmp = 0;
    uint8_t  i;
    for(i = 0; i < num; i++)
    tmp += array[i];
    tmp = tmp / num;
    return tmp;
}


/**
* @brief   获取电机编码器的脉冲值
* @param  	None
* @retval 	存放经过滤波后的编码器脉冲值
*/
void GetEncoderPulse()
{
	uint8_t i = filterNum - 1;
	for(;i > 0;i--)
	{
		leftEncoder[i-1] = leftEncoder[i];  //使用数组来保存两次的编码值
		rightEncoder[i-1] = rightEncoder[i];  //最后一个用于保留上一次的值
	}
	  leftEncoder[filterNum-1] = -((short)__HAL_TIM_GET_COUNTER(&htim2));
    rightEncoder[filterNum-1] = -((short)__HAL_TIM_GET_COUNTER(&htim3));

    last_encoderPulse[0] = encoderPulse[0];
    last_encoderPulse[1] = encoderPulse[1];

	  encoderPulse[0] = AverageFilter(leftEncoder, filterNum);  //左电机的编码器值
    encoderPulse[1] = AverageFilter(rightEncoder, filterNum);  //右电机的编码器值

    encoderValue[0] = encoderPulse[0]-last_encoderPulse[0];
    encoderValue[1] = encoderPulse[1]-last_encoderPulse[1];

     if(encoderPulse[0]>32000||encoderPulse[0]<-32000)
     {
       TIM2->CNT = 0;
     }
     if(encoderPulse[1]>32000||encoderPulse[1]<-32000)
     {
       TIM3->CNT = 0;
     }

//    __HAL_TIM_GET_COUNTER(&htim2) = 0;	 //计数值重新清零
//    __HAL_TIM_GET_COUNTER(&htim3) = 0;
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
int PID_Cal(int targetSpeed,int currentSpeed,PID *p)
{
    int error = targetSpeed - currentSpeed;
    if(error>50)error=0;
    if(error<-50)error=0;  //避免跳动
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能IDLE中断
  HAL_UART_Receive_DMA(&huart1,uart1_rx_buffer,BUFFER_SIZE);//DMA接收函数，此句一定要加，不加接收不到第一次传进来的实数据是空的
  HAL_TIM_Base_Start_IT(&htim6);

  PID_Init(&LeftMotor_PID);
  PID_Init(&RightMotor_PID);

  MotorInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(recv_end_flag == 1)
	  	{
	  	  if(uart1_rx_buffer[0] == 0x6d)
	  	  {
	  	    if(rx_len <=9)
	  	    {
	  	      leftTargetSpeed = atof(uart1_rx_buffer+1);  //将接收的字符串转换为float类型
	  	      rightTargetSpeed = atof(uart1_rx_buffer+4);
	  	      if(uart1_rx_buffer[2] == 0x2d)
	  	      {
	  	        leftTargetSpeed = atof(uart1_rx_buffer+1);  //将接收的字符串转换为float类型
	  	        rightTargetSpeed = atof(uart1_rx_buffer+5);
	  	      }
	  	    }
	  	    else if(rx_len > 9)
	  	    {
	  	      leftTargetSpeed = atof(uart1_rx_buffer+1);
	  	      rightTargetSpeed = atof(uart1_rx_buffer+5);
	  	    }

	  	    printf("OK\r\n");
	  	  }
	  	  else if(uart1_rx_buffer[0] == 0x65)
	  	  {
	  	    int leftPulse=0,rightPulse=0;
	  	    leftPulse = encoderPulse[0];
	  	    rightPulse = encoderPulse[1];
	  	    printf("%d %d\r\n",leftPulse,rightPulse);
	  	  }
	  	  //      else if(uart1_rx_buffer[0] == 0x73)
	  	  //      {
	  	  //        TIM2->CNT=0;
	  	  //        TIM3->CNT=0;
	  	  //      }
	  	  recv_end_flag = 0;
	  	  rx_len = 0;
	  	  memset(uart1_rx_buffer,0,rx_len);
	  	}
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //定时器6中断回调函数
{
	float c_leftSpeed,c_rightSpeed;
  if(htim==(&htim6))
  {
		GetEncoderPulse();

		c_leftSpeed = CalActualSpeed(encoderValue[0]);   //获得当前的速度值
		c_rightSpeed = CalActualSpeed(encoderValue[1]);
//    printf("leftpulse%d\r\n",encoderPulse[0]);
//    printf("rightpulse%d\r\n",encoderPulse[1]);
		PID_Cal(leftTargetSpeed,c_leftSpeed,&LeftMotor_PID);
		PID_Cal(rightTargetSpeed,c_rightSpeed,&RightMotor_PID);

		MotorControl(LeftMotor_PID.pwm_add,RightMotor_PID.pwm_add);  //PID调节

    if(leftTargetSpeed==0)
    {
      LeftMotor_PID.pwm_add = 0;  //PID增量清零
    }
    if(rightTargetSpeed==0)
    {
      RightMotor_PID.pwm_add = 0;
    }
//    memset(encoder,0,4);
//    printf("llll.pwmadd%d\r\n",LeftMotor_PID.pwm_add);
//    printf("rrrr.pwmadd%d\r\n",RightMotor_PID.pwm_add);
//     printf("lefttarget is:%.2f\r\n",leftTargetSpeed);
//		 printf("righttarget is:%.2f\r\n",rightTargetSpeed);
//	  printf("----------------------------\r\n");
//		 printf("{currentLeftSpeed is:%.2f}\r\n",c_leftSpeed);
//		 printf("{currentrightSpeed is:%.2f}\r\n",c_rightSpeed);
//     printf("----------------------------\r\n");
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
