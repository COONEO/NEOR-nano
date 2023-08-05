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
#define L_DirPort GPIOB
#define CIN1 GPIO_PIN_12
#define CIN2 GPIO_PIN_13
#define DIN1 GPIO_PIN_14
#define DIN2 GPIO_PIN_15

// 对于只有两个电机的车子，AIN1和AIN2是左轮，BIN1和BIN2是右�?
#define R_DirPort GPIOC
#define AIN1 GPIO_PIN_0
#define AIN2 GPIO_PIN_1
#define BIN1 GPIO_PIN_2
#define BIN2 GPIO_PIN_3

#define Velocity_Kp 2.0
#define Velocity_Ki 1.3
#define Velocity_Kd 0.8

#define limit_value 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float B_LTargetSpeed = 0;
float B_RTargetSpeed = 0;

short encoderPulse[2]={0};

uint8_t PPM_Okay = 0;    //标志�?帧数据是否捕获完�?
uint16_t PPM_Sample_Cnt=0;    //PWM通道�?
uint32_t PPM_Time = 0;    //记录PPM时间
uint16_t PPM_Databuf[8] = {0};  //用数组存储每�?帧的时间

PID B_L_PID;
PID B_R_PID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

//舵机相关初始�?
void ServoInit()
{
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);  //启动定时�?2的PWM模式

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);  //启动定时�?4的PWM模式
}

void Set_Servo_angle(TIM_HandleTypeDef * htim,uint32_t Channel,uint8_t angle)
{
	uint16_t compare_value=0;
	if(angle <= 180)   //限制角度�?180°
	{
		//角度转化为数�?
		compare_value=0.5*2000/20+angle*2000/20/90;
		__HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
	}
}

void ServoControl()
{

	if((PPM_Databuf[2]<990)||(PPM_Databuf[2]>2010))
	{
		Set_Servo_angle(&htim4,TIM_CHANNEL_3,90);
	}
	else
	{
	if(PPM_Databuf[2]<1600&&PPM_Databuf[2]>1400){PPM_Databuf[2] = 1500;}
	if(PPM_Databuf[3]<1600&&PPM_Databuf[3]>1400){PPM_Databuf[3] = 1500;}
	if(PPM_Databuf[6]<1000&&PPM_Databuf[6]>900){PPM_Databuf[6] = 1100;}
	if(PPM_Databuf[7]<1000&&PPM_Databuf[7]>900){PPM_Databuf[7] = 1000;}
	}


	Set_Servo_angle(&htim4,TIM_CHANNEL_4,myabs(0.18*PPM_Databuf[2]-180)); // 对应舵机2口�?��?�左边上下手柄的控制
	Set_Servo_angle(&htim4,TIM_CHANNEL_3,myabs(0.18*PPM_Databuf[3]-180)); // 对应舵机1口�?��?�左边左右手柄的控制

	Set_Servo_angle(&htim2,TIM_CHANNEL_4,myabs(0.18*PPM_Databuf[6]-180));
	Set_Servo_angle(&htim2,TIM_CHANNEL_3,myabs(0.18*PPM_Databuf[7]-180));
}
/*  电机相关初始�?
*/
void MotorInit()
{
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
}



void EncoderInit()
{
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);  //启动定时�?3的编码器模式
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1);  //启动定时�?5的编码器模式
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
}

/*
*		左轮 JP5  TIM8_CH1
*		右轮 JP4  TIM8_CH2
*/
void B_Left_Go()
{
		HAL_GPIO_WritePin(R_DirPort,AIN1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(R_DirPort,AIN2,GPIO_PIN_RESET);
}

void B_Right_Go()
{
		HAL_GPIO_WritePin(R_DirPort,BIN1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(R_DirPort,BIN2,GPIO_PIN_RESET);
}


void B_Left_Back()
{
		HAL_GPIO_WritePin(R_DirPort,AIN1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(R_DirPort,AIN2,GPIO_PIN_SET);
}

void B_Right_Back()
{
		HAL_GPIO_WritePin(R_DirPort,BIN1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(R_DirPort,BIN2,GPIO_PIN_SET);
}

void B_Left_Stop()
{
		HAL_GPIO_WritePin(R_DirPort,AIN1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(R_DirPort,AIN2,GPIO_PIN_RESET);
}

void B_Right_Stop()
{
		HAL_GPIO_WritePin(R_DirPort,BIN1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(R_DirPort,BIN2,GPIO_PIN_RESET);
}
/**
*	@brief Control Motor Speed
*	@param PWM
*	@retval None
*/
void MotorControl(int B_L_PWM, int B_R_PWM)
{
		if(B_L_PWM >= 0)//前进方向运动
		{
//				B_Left_Go();
			B_Left_Back();
		}
    else
    {
//        B_Left_Back();
    	B_Left_Go();
    }
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,myabs(B_L_PWM));


		if(B_R_PWM >= 0)		//后�??方向运动
		{
//        B_Right_Go();
			B_Right_Back();

    }
    else
    {
//      B_Right_Back();
    	B_Right_Go();
    }
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,myabs(B_R_PWM));

}


void GetEncoderPulse()
{
	  encoderPulse[0] = -((short)__HAL_TIM_GET_COUNTER(&htim5));	//获取左轮编码器�??
		encoderPulse[1] = -((short)__HAL_TIM_GET_COUNTER(&htim3));  //获取右轮的编码器�?

    __HAL_TIM_GET_COUNTER(&htim3) = 0;
    __HAL_TIM_GET_COUNTER(&htim5) = 0;
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
int PID_Cal(int targetSpeed,int currentSpeed,PID *p)
{
    int error = targetSpeed - currentSpeed;
    p->pwm_add += p->Kp*(error - p->last_error) + p->Ki*error + p->Kd*(error - 2*p->last_error+p->prev_error);

		p->prev_error = p->last_error;
	  p->last_error = error;

		if(p->pwm_add>p->limit) p->pwm_add=p->limit;
		if(p->pwm_add<-p->limit) p->pwm_add=-p->limit;
}

void MotorRemoteControl()
{
	static uint8_t flag=0;
	if((PPM_Databuf[0]==0)&&(PPM_Databuf[1]==0))
	{
		  B_Left_Stop();
		  B_Right_Stop();
	}
	else
	{
	if(PPM_Databuf[0]<1600&&PPM_Databuf[0]>1400){PPM_Databuf[0] = 1500;}
	else if(PPM_Databuf[0] != 1500){flag = 0;}
	if(PPM_Databuf[1]<1600&&PPM_Databuf[1]>1400){PPM_Databuf[1] = 1500;}
	else if(PPM_Databuf[1] != 1500){flag = 1;}
	switch(flag)
	{
		case 0:
			if(PPM_Databuf[0] == 1500)
			{
				MotorControl(0,0);
			}
//			if(PPM_Databuf[0]>1600)
//			{
//				MotorControl(myabs(PPM_Databuf[0]-1500),0);
//			}
//			else if(PPM_Databuf[0]<1400)
//			{
//				MotorControl(0,myabs(PPM_Databuf[0]-1500));
//			}
			break;
		case 1:MotorControl(PPM_Databuf[1]-1500,PPM_Databuf[1]-1500);break;


	}
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);  //使能定时�?6中断
  HAL_TIM_Base_Start_IT(&htim7);  //使能定时�?7中断
  MotorInit();
  ServoInit();
  EncoderInit();
  PID_Init(&B_L_PID);
  PID_Init(&B_R_PID);
//  B_Left_Stop();
//  B_Right_Stop();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MotorRemoteControl();
	  	      ServoControl();
	  	  		printf("PPM_Data[0] is %d\r\n",PPM_Databuf[0]);
	  	      printf("PPM_Data[1] is %d\r\n",PPM_Databuf[1]);
	  	      printf("PPM_Data[2] is %d\r\n",PPM_Databuf[2]);
	  	      printf("PPM_Data[3] is %d\r\n",PPM_Databuf[3]);
	  	         printf("PPM_Data[4] is %d\r\n",PPM_Databuf[4]);
	  	      printf("PPM_Data[5] is %d\r\n",PPM_Databuf[5]);
	  	    printf("----------------------------\r\n");
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //定时�?6中断回调函数
{
	float B_LSpeed,B_RSpeed;
  if(htim==(&htim6))
  {
		GetEncoderPulse();
		B_LSpeed = CalActualSpeed(encoderPulse[0]);   //获得当前的�?�度�?
		B_RSpeed = CalActualSpeed(encoderPulse[1]);

		PID_Cal(B_LTargetSpeed*100,B_LSpeed*100,&B_L_PID);
		PID_Cal(B_RTargetSpeed*100,B_RSpeed*100,&B_R_PID);

//		MotorControl(B_L_PID.pwm_add,B_R_PID.pwm_add);
//	printf("leftpwmadd %d\r\n",F_L_PID.pwm_add);
//	printf("rightpwmadd %d\r\n",F_R_PID.pwm_add);
//	printf("F_RRRPulse is %d\r\n",encoderPulse[0]);
//	printf("B_RRRPulse is %d\r\n",encoderPulse[1]);
//	printf("F_LLLPulse is %d\r\n",encoderPulse[2]);
//	printf("B_LLLPulse is %d\r\n",encoderPulse[3]);
//	printf("----------------------------\r\n");
//		printf("{bbbbrrrrSpeed is:%.2f}\r\n",B_RSpeed);
//		printf("{bbbbllllSpeed is:%.2f}\r\n",B_LSpeed);
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		PPM_Time = __HAL_TIM_GET_COUNTER(&htim7);
		__HAL_TIM_GET_COUNTER(&htim7) = 0;
			if(PPM_Time > 0)
			{
			  PPM_Time++;
			}
		if(PPM_Okay == 1)
		{
			PPM_Databuf[PPM_Sample_Cnt] = PPM_Time;
			PPM_Sample_Cnt++;
			if(PPM_Sample_Cnt > 8) PPM_Okay = 0;
		}
		if(PPM_Time>=3000)
		{
			PPM_Okay = 1;
			PPM_Sample_Cnt = 0;
		}
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
