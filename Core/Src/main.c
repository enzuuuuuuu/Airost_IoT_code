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
#include <stdio.h>
#include <string.h>
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rxByte;
uint8_t buttonPrev = 1;
uint32_t buttonDebounce = 0;
volatile uint8_t mode = 1;
volatile uint8_t manualReturnFlag = 0;
uint8_t prev_mode = 1;
volatile uint8_t control = 's';
volatile uint8_t servo = 'R';
char uartBuffer[32];
uint8_t uartIndex = 0;
volatile uint16_t SB=600, J1B=6000, J1F=125, J2B=1500, J2F=125, R1B=500, RBS=6000, LL=1000, RB=8000, RIF=120, CD2 =400;
uint8_t irValue = 0;

static uint8_t case2_triggered = 0;
static uint32_t case2_startTick = 0;
static uint8_t junctionDetected = 0;
static uint32_t cooldown1 = 0;
static uint16_t junctionCounter = 0;
static uint32_t junctionStartTime = 0;
static uint8_t lastSent = 0;

static uint8_t junction2Detected = 0;
static uint32_t cooldown2 = 0;
static uint16_t junction2Counter = 0;
static uint32_t junction2StartTime = 0;
static uint8_t lastSentJ2 = 0;
static uint8_t j2StopTrigger = 0;
static uint32_t j2StartTick = 0;

static uint32_t RotateStartTime = 0;
static uint8_t rotated = 0;
static uint32_t settleStart = 0;

static uint32_t RotateBackStartTime = 0;
static uint8_t reach = 0;

static uint32_t ReturnStartTime = 0;
static uint8_t returnIndicationDetected = 0;
static uint16_t returnIndicationCounter = 0;
static uint32_t returnIndicationStartTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Forward(void);
void ForwardFast(void);
void Backward(void);
void Left(void);
void LeftFast(void);
void Right(void);
void Stop(void);
void Grip(void);
void Release(void);
void ModeLED(uint8_t);
void parseCommand(char *);
void parseMultipleCommands(char *);
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rxByte, 1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // motor
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // motor
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // servo
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 24);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint8_t buttonNow = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);

	if (buttonPrev == 1 && buttonNow == 0)
	{
	    if (HAL_GetTick() - buttonDebounce > 10)
	    {
	        mode++;
	        if (mode > 4)
	        	mode = 1;
	        buttonDebounce = HAL_GetTick();
	    }
	}

	buttonPrev = buttonNow;

	irValue =  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) << 3)|(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) << 2)|
			(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) << 1)|HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);

	if (mode != prev_mode)
	{
		case2_triggered = 0;
		prev_mode = mode;
		ModeLED(mode);
	}

	switch(mode)
	{
		case 1:
		  switch(control)
		  {
			  case 'f': Forward(); break;
			  case 'b': Backward(); break;
			  case 'l': Left(); break;
			  case 'r': Right(); break;
			  case 's': Stop(); break;
			  default: Stop(); break;
		  }
		  switch(servo)
		  {
			  case 'G': Grip(); break;
			  case 'R': Release(); break;
		  }
		  break;
		case 2:
		case 3:
		case 4:
			if(!case2_triggered && (irValue == 0x0))
			{
			    Forward();
			    case2_triggered = 1;
			    case2_startTick = HAL_GetTick();

			    junctionStartTime = 0;
			    junctionDetected = 0;
			    junctionCounter = 0;
			    cooldown1 = 0;
			    lastSent = 0;

			    junction2StartTime = 0;
			    junction2Detected = 0;
			    junction2Counter = 0;
			    cooldown2 = 0;
			    lastSentJ2 = 0;

			    RotateStartTime = 0;
			    rotated = 0;

			    RotateBackStartTime = 0;
			    reach = 0;

			    ReturnStartTime = 0;
			    returnIndicationStartTime = 0;
			    returnIndicationCounter = 0;
			    returnIndicationDetected = 0;

			    j2StopTrigger = 0;
			}

			if(case2_triggered && !j2StopTrigger)
			{
				if(HAL_GetTick() - case2_startTick >= SB)
				{
					if ((mode == 3 && junctionCounter >= 2 && junction2Counter <= 1) ||
							(mode == 4 && junctionCounter >= 2 && junction2Counter <= 2))
					{
						switch(irValue)
						{
							case 0b0110:case 0b1110:
								Forward();
								break;
							case 0b0100:case 0b1000:case 0b1100:
								Left();
								break;
							case 0b0010:case 0b0001:case 0b0011:case 0b0111:case 0b0000:case 0b1111:
								Right();
								break;
							default:
								Stop();
								break;
						}
					}
					else if((mode == 4 && junctionCounter >= 2 && junction2Counter > 2)||(mode == 3 && junctionCounter >= 2 && junction2Counter > 1))
					{
						switch(irValue)
						{
							case 0b0110:case 0b1110:
								Forward();
								break;
							case 0b0100:case 0b1000:case 0b1100:case 0b0000:case 0b1111:
								Left();
								break;
							case 0b0010:case 0b0001:case 0b0011:case 0b0111:
								Right();
								break;
							default:
								Stop();
								break;
						}
					}
					else
					{
						switch(irValue)
						{
							case 0b0110:
								Forward();
								break;
							case 0b0100:case 0b1110:case 0b1000:case 0b1100:
								Left();
								break;
							case 0b0010:case 0b0001:case 0b0011:case 0b0111:case 0b0000:case 0b1111:
								Right();
								break;
							default:
								Stop();
								break;
						}
					}

			        if(HAL_GetTick() - case2_startTick >= J1B)
			        {
						if (irValue == 0b0111)
						{
		                    if(junctionStartTime == 0)
		                        junctionStartTime = HAL_GetTick();
		                    else if(HAL_GetTick() - junctionStartTime >= J1F)
		                    {
		                        if(!junctionDetected && (HAL_GetTick() - cooldown1 >= 3000))
		                        {
		                            junctionCounter++;
		                            junctionDetected = 1;
		                            cooldown1 = HAL_GetTick();
		                        }
		                    }
						}

		                else
		                {
		                    junctionStartTime = 0;
		                    junctionDetected = 0;
		                }

						if (junctionCounter != lastSent)
						{
							char msg[20];
							int len = sprintf(msg, "J:%d\n", junctionCounter);
							HAL_UART_Transmit(&huart1, (uint16_t*)msg, len, 10);
							lastSent = junctionCounter;
						}
			        }

			        if(junctionCounter >= 2)
					{
			            if(j2StartTick == 0)
			            	j2StartTick = HAL_GetTick();

			            if(HAL_GetTick() - j2StartTick >= J2B)
			            {
			            	if(irValue == 0b1110)
			            	{
								if(junction2StartTime == 0)
									junction2StartTime = HAL_GetTick();
								else if(HAL_GetTick() - junction2StartTime >= J2F)
								{
									if(!junction2Detected && (HAL_GetTick()-cooldown2 >= CD2))
									{
										junction2Counter++;
										junction2Detected = 1;
										cooldown2 = HAL_GetTick();
										if(junction2Counter >= mode-1)
										{
											j2StopTrigger = 1;
											Stop();
										}
									}
								}
			            	}
			            	else
			            	{
			            		junction2StartTime = 0;
								junction2Detected = 0;
			            	}
			            	if(junction2Counter != lastSentJ2)
			            	{
								char msg2[20];
								int len2 = sprintf(msg2, "J2:%d\n", junction2Counter);
								HAL_UART_Transmit(&huart1, (uint8_t*)msg2, len2, 10);
								lastSentJ2 = junction2Counter;
			            	}
			            }
					}
			        else
			        {
			        	j2StartTick = 0;
			        }
				}
			}
			else if (j2StopTrigger == 1)
			{
			    if (RotateStartTime == 0)
			    {
			    	Left();
			    	RotateStartTime = HAL_GetTick();
			    }

			    if (!rotated && (HAL_GetTick() - RotateStartTime >= R1B))
			    {
			        if (irValue == 0b0110)
			        {
			        	rotated = 1;
			        	settleStart = HAL_GetTick();
			            Stop();
			        }
			    }

			    if (rotated)
				{
			        if (HAL_GetTick() - settleStart < 100)
			        {
			            Stop();
			        }
			        else
			        {
			        	switch(irValue)
						{
							case 0b0110:
								Forward();
								break;
							case 0b0100:case 0b1110:case 0b1000:case 0b1100:case 0b0000:
								Left();
								break;
							case 0b0010:case 0b0001:case 0b0011:case 0b0111:
								Right();
								break;
							case 0b1111:
								j2StopTrigger=2;
								Grip();
								break;
							default:
								Stop();
								break;
						}
			        }
				}
			}
			else if(j2StopTrigger == 2)
			{
				if(manualReturnFlag == 1)
				{
					mode = 1;
				}

				if(RotateBackStartTime == 0)
				{
					LeftFast();
					RotateBackStartTime = HAL_GetTick();
				}

				if (HAL_GetTick() - RotateBackStartTime < RBS)
				{
					if(!reach && (HAL_GetTick() - RotateBackStartTime >= LL))
					{
						LeftFast();

						if (irValue == 0b0110)
						{
							reach = 1;
							settleStart = HAL_GetTick();
							Stop();
						}
					}
					else if(reach)
					{
						if(HAL_GetTick() - settleStart < 200)
							Stop();
						else
						{
							if (mode==2 || mode==3)
							{
								switch(irValue)
								{
									case 0b0110:case 0b1111:
										ForwardFast();
										break;
									case 0b0100:case 0b1000:case 0b1100:case 0b1110:
										Left();
										break;
									case 0b0010:case 0b0001:case 0b0011:case 0b0111:case 0b0000:
										Right();
										break;
									default:
										Left();
										break;
								}
							}
							else
							{
								switch(irValue)
								{
									case 0b0110:case 0b1111:
										ForwardFast();
										break;
									case 0b0100:case 0b1000:case 0b1100:case 0b1110:
										Left();
										break;
									case 0b0010:case 0b0001:case 0b0011:case 0b0111:case 0b0000:
										Right();
										break;
									default:
										Left();
										break;
								}
							}
						}
					}
				}
				else
					j2StopTrigger = 3;
			}
			else if(j2StopTrigger==3)
			{
				if (ReturnStartTime == 0)
					ReturnStartTime = HAL_GetTick();

				switch(irValue)
				{
					case 0b0110:
						Forward();
						break;
					case 0b0100:case 0b1110:case 0b1000:case 0b1100:case 0b0000:case 0b1111:
						Left();
						break;
					case 0b0010:case 0b0001:case 0b0011:case 0b0111:
						Right();
						break;
					default:
						Left();
						break;
				}

		        if(HAL_GetTick() - ReturnStartTime >= RB)
		        {
					if (irValue == 0b1111)
					{
	                    if(returnIndicationStartTime == 0)
	                    	returnIndicationStartTime = HAL_GetTick();
	                    else if(HAL_GetTick() - returnIndicationStartTime >= RIF)
	                    {
	                        if(!returnIndicationDetected)
	                        {
	                            returnIndicationCounter++;
	                            returnIndicationDetected = 1;
	                        }
	                    }
					}

	                else
	                {
	                	returnIndicationStartTime = 0;
	                	returnIndicationDetected = 0;
	                }

					if (returnIndicationCounter==1)
					{
						j2StopTrigger = 4;
						Stop();
					}
		        }
			}
			else if (j2StopTrigger == 4)
			{
				Stop();
				Release();
			}
			break;
		default:
			break;
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
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA11
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
	    if (rxByte != '\n' && rxByte != '\r')
	    {
	        if (uartIndex < sizeof(uartBuffer) - 1)
	            uartBuffer[uartIndex++] = rxByte;
	        else
	            uartIndex = 0;
	    }
	    else
	    {
	        if (uartIndex == 0)
	        {
	            HAL_UART_Receive_IT(&huart1, &rxByte, 1);
	            return;
	        }
	        uartBuffer[uartIndex] = '\0';

	        if (strchr(uartBuffer, '=') != NULL)
	        {
	            parseMultipleCommands(uartBuffer);
	        }
	        else if (uartIndex == 1)
	        {
	            char c = uartBuffer[0];

	            switch(c)
	            {
	                case 'm': mode = 1; break;
	                case '1': manualReturnFlag = 0; mode = 2; break;
	                case '2': manualReturnFlag = 0; mode = 3; break;
	                case '3': manualReturnFlag = 0; mode = 4; break;

	                case 'f':
	                case 'b':
	                case 'l':
	                case 'r':
	                case 's':
	                    if (mode == 1) control = c;
	                    break;

	                case 'G':
	                case 'R':
	                    if (mode == 1) servo = c;
	                    break;

	                default:
	                    break;
	            }
	        }
	        uartIndex = 0;
	    }
	    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
	}
}

void parseCommand(char *cmd)
{
    char name[10];
    int value;

    if (sscanf(cmd, "%[^=]=%d", name, &value) == 2)
    {
        if (value < 0) value = 0;
        if (value > 10000) value = 10000;

        if      (strcmp(name, "SB") == 0)   SB = value;
        else if (strcmp(name, "J1B") == 0)  J1B = value;
        else if (strcmp(name, "J1F") == 0)  J1F = value;
        else if (strcmp(name, "J2B") == 0)  J2B = value;
        else if (strcmp(name, "J2F") == 0)  J2F = value;
        else if (strcmp(name, "R1B") == 0)  R1B = value;
        else if (strcmp(name, "RBS") == 0)  RBS = value;
        else if (strcmp(name, "LL") == 0)  LL = value;
        else if (strcmp(name, "CD2") == 0)  CD2 = value;
        else if (strcmp(name, "1m") == 0)  {mode = 2 ; manualReturnFlag = value;}
        else if (strcmp(name, "2m") == 0)  {mode = 3 ; manualReturnFlag = value;}
        else if (strcmp(name, "3m") == 0)  {mode = 4 ; manualReturnFlag = value;}
        else if (strcmp(name, "RB") == 0)   RB = value;
        else if (strcmp(name, "RIF") == 0)  RIF = value;
    }
}

void parseMultipleCommands(char *line)
{
    char *cmd = strtok(line, ",");

    while (cmd != NULL)
    {
        parseCommand(cmd);
        cmd = strtok(NULL, ",");
    }
}

void ModeLED(uint8_t mode)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (mode & 0x01) ? 1 : 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (mode & 0x02) ? 1 : 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (mode & 0x04) ? 1 : 0);
}


void Forward(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 59);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 59);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
}

void ForwardFast(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 79);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 79);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
}

void Backward(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 49);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 49);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
}

void Left(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 54);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 54);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
}

void LeftFast(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 69);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 69);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
}

void Right(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 54);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 54);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
}

void Stop(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
}

void Grip(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 18);
}

void Release(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 24);
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
