/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <math.h>
#include "sweep.h"
#include "motorcontrol.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int gohome_enable = 1;
int counter;
volatile unsigned int systick=0;
uint8_t state=0;
long start=0;
long end=0;
long mid=0;
long base=0;
long cnter=0;
long old_cnter=0;
long pos=0;
uint8_t find_home_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/*实现扫频辨识功能，该功能将被放置在 main 函数的 while(1)中运行*/ 
void function_sweep_identification(void)
{
	static my_sweep_t sweep = {0};
	int16_t sweep_input = 0;
	int16_t sweep_output = 0; 
	uint32_t sys_tick = 0; 
	static uint32_t init_flag = 0; 
	static uint32_t last_sys_tick = 0; 
	static uint32_t start_sys_tick = 0;
	// 频率在 10s 内 ，从 0.5hz 变化到 10hz，幅度为 1500 digit current 
	uint32_t t_period_ms = 10*1000; //10s
	float f0 = 0.5; 
	float f1 = 10; 
	float Amp = 1500.0f; 
	
	float time = 0.0f; 
	sys_tick = HAL_GetTick(); //获取当前时刻，单位 ms
	time = 0.001f * sys_tick; //单位 s
	
	/*进入的条件时回零成功，且按了 K1 运行键, 就开始执行扫频辨识过程, 注意 find_home_flag 是回零成功的标志位，是一个全局变量,需要在外部实现这个标志位*/
	if ((find_home_flag == 1) && (MC_GetSTMStateMotor1() == RUN))
	{
		if (last_sys_tick != sys_tick) //如果当前时刻发生了变化，这个条件每 ms 都会成立一次
		{
			last_sys_tick = sys_tick;
			if (sys_tick % 10 == 0) //通过 % 把频率从 1000hz 降低到 100hz，即每 10ms 发生一次
			{
				//初始化扫频配置
				if (init_flag == 0)
				{
					init_my_sweep(&sweep, sys_tick, t_period_ms, f0, f1, Amp); 
					printf("sweep-init:k=%.5f,p=%.5f\r\n",(float)sweep.k,(float)sweep.p);
					init_flag = 1; 
				}
				//获取正弦扫频信号
				sweep_input = (int16_t)run_my_sweep(&sweep, sys_tick);
				//将正弦扫频信号输入到 ST MC SDK 的力矩控制 API 中
				MC_ProgramTorqueRampMotor1(sweep_input,0);
				//获取丝杆的转速信息，单位为 0.1h
				sweep_output = MC_GetMecSpeedAverageMotor1();
				//把时间，input, output 发送到 matlab
				send_data_2_matlab(time,(float)sweep_input,(float)sweep_output); 
			}
		}
	}
}
/*实现一个阶跃测试函数，放在 main 的 while(1)中进行阶跃测试*/ 
void function_step_test(void)
{
	/*1.定义局部变量和静态变量*/
	uint32_t sys_tick = 0; 
	static uint32_t init_flag = 0; 
	static uint32_t last_sys_tick = 0; 
	static uint32_t start_sys_tick = 0;
	static float start_time=-1.0f;
	int16_t step_input = 0;
	int16_t step_output = 0; 
	
	// 频率在 10s 内 ，从 0.5hz 变化到 10hz，幅度为 1500 digit current 
	uint32_t t_period_ms = 10*1000; //10s
	static float u_k;
	static float u_k_1;
	static float e_k;
	static float e_k_1;
	static float ref =0.0f;
	static float Amp = 0.0f; 
	static float start_pos = 0.0f; 
	
	float time = 0.0f; 
	sys_tick = HAL_GetTick(); //获取当前时刻，单位 ms
	time = 0.001f * sys_tick; //单位 s
	
	/*2.初始化你的控制器*/
	if ((find_home_flag == 1) && (MC_GetSTMStateMotor1() == RUN))/*3.启动 step 测试功能条件满足*/
	{
		if (last_sys_tick != sys_tick)/*4.分频器确定当前 tick 为控制器运行的 tick*/
		{
			/*5.获取光栅尺的当前位置 fdk (mm)*/
			//pos
			last_sys_tick = sys_tick;
			if (sys_tick % 10 == 0) //通过 % 把频率从 1000hz 降低到 100hz，即每 10ms 发生一次
			{
				//初始化扫频配置,
				if (init_flag == 0)
				{
					/*6.指定阶跃的距离 Amp 统一定义为 20mm*/
					Amp=20.0f/0.005f;
					/*7.一次性计算当前参考值 ref = fdk+Amp*/
					ref = pos-Amp;
					u_k=0.0f;
					u_k_1=0.0f;
					e_k=0.0f;
					e_k_1=0.0f;
					start_time=time;
					start_pos=pos;
					init_flag = 1; 
				}
				else{
					/*8.将 ref 作为你的位置控制器的输入命令，调用控制器*/
					e_k_1=e_k;
					e_k=(ref-pos)*0.005f;
					u_k_1=u_k;
					u_k=0.3333f*u_k_1+2167.0f*e_k-1167.0f*e_k_1;
					//将正弦扫频信号输入到 ST MC SDK 的力矩控制 API 中
					MC_ProgramTorqueRampMotor1(u_k,0);
					//获取丝杆的转速信息，单位为 0.1h
					step_output = MC_GetMecSpeedAverageMotor1();
					/*9.使用 send_data_2_matlab, 把时间，ref, fdk 发送到 matlab 进行显示*/
					float output=pos-start_pos<0?start_pos-pos:pos-start_pos;
					send_data_2_matlab(time,(float)20.0,(float)output*0.005f); 
//					send_data_2_matlab(time,(float)ref,pos); 
					if(time-start_time>4)state++;
				}
				
			}
		}
		else {
		/*10. do some thing if need*/
		}
	}
	else {
	/*11. do some thing if need*/
	}
}

/*实现一个sin函数*/ 
void function_sin_test(void)
{
	/*1.定义局部变量和静态变量*/
	uint32_t sys_tick = 0; 
	static uint32_t init_flag = 0; 
	static uint32_t last_sys_tick = 0; 
	static uint32_t start_sys_tick = 0;
	static float start_time=-1.0f;
	int16_t step_input = 0;
	int16_t step_output = 0; 
	
	// 频率在 10s 内 ，从 0.5hz 变化到 10hz，幅度为 1500 digit current 
	uint32_t t_period_ms = 10*1000; //10s
	static float u_k;
	static float u_k_1;
	static float e_k;
	static float e_k_1;
	static float ref =0.0f;
	static float Amp = 0.0f; 
	static float start_pos = 0.0f; 
	
	float time = 0.0f; 
	sys_tick = HAL_GetTick(); //获取当前时刻，单位 ms
	time = 0.001f * sys_tick; //单位 s
	
	/*2.初始化你的控制器*/
	if ((find_home_flag == 1) && (MC_GetSTMStateMotor1() == RUN))/*3.启动 step 测试功能条件满足*/
	{
		if (last_sys_tick != sys_tick)/*4.分频器确定当前 tick 为控制器运行的 tick*/
		{
			/*5.获取光栅尺的当前位置 fdk (mm)*/
			//pos
			last_sys_tick = sys_tick;
			if (sys_tick % 10 == 0) //通过 % 把频率从 1000hz 降低到 100hz，即每 10ms 发生一次
			{
				//初始化扫频配置,
				if (init_flag == 0)
				{
					
					u_k=0.0f;
					u_k_1=0.0f;
					e_k=0.0f;
					e_k_1=0.0f;
					start_time=time;
					start_pos=pos;
					init_flag = 1; 
				}
				else{
					/*6.指定阶跃的距离 Amp 统一定义为 20mm*/
					Amp=20.0*sin(2*PI*(time-start_time))/0.005f;
					/*7.一次性计算当前参考值 ref = fdk+Amp*/
					ref = start_pos-Amp;
					/*8.将 ref 作为你的位置控制器的输入命令，调用控制器*/
					e_k_1=e_k;
					e_k=(ref-pos)*0.005f;
					u_k_1=u_k;
					u_k=0.3333f*u_k_1+2167.0f*e_k-1167.0f*e_k_1;
					//将正弦扫频信号输入到 ST MC SDK 的力矩控制 API 中
					MC_ProgramTorqueRampMotor1(u_k,0);
					//获取丝杆的转速信息，单位为 0.1h
					step_output = MC_GetMecSpeedAverageMotor1();
					/*9.使用 send_data_2_matlab, 把时间，ref, fdk 发送到 matlab 进行显示*/
					float output=pos-start_pos;
					send_data_2_matlab(time,(float)(-Amp)*0.005f,(float)output*0.005f); 
//					send_data_2_matlab(time,(float)ref,pos); 
					if(time-start_time>4)state++;
				}
				
			}
		}
		else {
		/*10. do some thing if need*/
		}
	}
	else {
	/*11. do some thing if need*/
	}
}
/*实现一个扫频测试函数，放在 main 的 while(1)中进行扫频跟随测试，验证 w 位置闭环带宽*/ 
//void function_sweep_test(void)
//{
//	/*1.定义局部变量和静态变量*/
//	/*2.初始化你的控制器*/
//	/*3.初始化你的正弦信号发生器为 幅值 20mm, 频率在 10s 内从 1hz 变化到 2hz*/
//	if (/*4.启动扫频测试功能条件满足*/)
//	{
//		if (/*5.分频器确定当前 tick 为控制器运行的 tick*/)
//		{
//			/*6.获取光栅尺的当前位置 fdk (mm)*/
//			/*7.每个 tick 都获取最新的扫频信号输出，作为当前参考值 ref = sweep(now)*/
//			/*8.将 ref 作为你的位置控制器的输入命令，调用控制器*/
//			/*9.使用 send_data_2_matlab, 把时间，ref, fdk 发送到 matlab 进行显示*/
//		}
//		else {
//		/*10. do some thing if need*/
//		}
//	}
//	else {
//	/*11. do some thing if need*/
//	}
//}
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_MotorControl_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	
	//int set_position=0;
	
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	MC_AlignEncoderMotor1();
	HAL_Delay(100);

	MC_ProgramSpeedRampMotor1(100,0);
	HAL_Delay(100);
	MC_StartMotor1();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		cnter=__HAL_TIM_GET_COUNTER(&htim3);
		if(state==1){
			state++;
		}
		else{
			if(old_cnter-cnter>20000){
				base+=0xffff;
			}
			else if(cnter-old_cnter>20000){
				base-=0xffff;
			}
		}
		pos=cnter+base;
		old_cnter=cnter;
//		if(state<=5)
//		printf("%f\r\n",(pos-mid)*0.005);
//		printf("%ld\r\n",cnter);
		
		switch(state){
			case 0:{//开关未按下
				MC_StopMotor1();
				break;
			}
			case 2:{
				MC_ProgramSpeedRampMotor1(-40,500);
				MC_StartMotor1();
				state++;
				break;
			}
			case 3:{
				if(HAL_GPIO_ReadPin(LIM_N_GPIO_Port,LIM_N_Pin)==RESET){
					state++;
					start=pos;
					MC_ProgramSpeedRampMotor1(40,500);
					MC_StartMotor1();
					
				}
				else;
				break;
			}
			case 4:{
				if(HAL_GPIO_ReadPin(LIM_P_GPIO_Port,LIM_P_Pin)==RESET){
					state++;
					end=pos;
					mid=start/2+end/2;
					MC_ProgramSpeedRampMotor1(-40,500);
					MC_StartMotor1();
				}
				else;
				break;
			}
			case 5:{
				if((pos-mid)<5&(mid-pos)<5){
					state++;
					MC_ProgramSpeedRampMotor1(0,0);
					MC_StartMotor1();
					find_home_flag=1;
				}
				break;
			}
			case 6:{
				function_step_test();
//				function_sin_test();
				break;
			}
			case 7:{
				MC_StopMotor1();
				break;
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		MC_ProgramSpeedRampMotor1();
//		MC_StartMotor1();
//		last_position=__HAL_TIM_GET_COUNTER(&htim3);
		//
		
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
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM8_UP_TIM13_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* TIM8_BRK_TIM12_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = M1_PULSE_NBR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_ENC_IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = M1_ENC_IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim8.Init.RepetitionCounter = (REP_COUNTER);
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : GoHome_Pin Start_Stop_Pin */
  GPIO_InitStruct.Pin = GoHome_Pin|Start_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LIM_P_Pin LIM_N_Pin */
  GPIO_InitStruct.Pin = LIM_P_Pin|LIM_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GoHome_Pin)
  {
//		HAL_Delay(100);
//		if(HAL_GPIO_ReadPin(GoHome_GPIO_Port,GoHome_Pin)==RESET){
			if(state==0){
				state=1;
			}
			else{
				state=0;
			}
//		}
    
  }
  
}
////void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
////{
////	systick++;
////}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance==TIM3)
//	{
//		int flag=__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
//		if (flag==1)
//		{
//			counter++;
//		}
//		else 
//		{
//			counter--;
//		}
//	}
//}
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
