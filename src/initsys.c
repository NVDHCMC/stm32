/**
  ******************************************************************************
  * @file    initsys.c
  * @author  Azure
  * @version V0.2.0
  * @date    8th-October-2017
  * @brief 	 Contain main kernel initialization, OS startup sequence
  * 		 and peripheral interrupt request handlers and callbacks
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <initsys.h>
#include <lime_mpacket.h>
#include <lime_mdriver.h>
#include <litve_adc.h>

extern void _Error_Handler(void);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
initHandle_struct 				initHandles;

/* Private functions ---------------------------------------------------------*/


/**
  * @brief 	RCC: Initialized USART2 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
static void RCC_INIT(void)
{
    RCC_OscInitTypeDef rccOsc;
	RCC_ClkInitTypeDef rccClk;

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	rccOsc.OscillatorType 	= RCC_OSCILLATORTYPE_HSE;
	rccOsc.HSEState 		= RCC_HSE_ON;
	rccOsc.PLL.PLLSource 	= RCC_PLLSOURCE_HSE;
	rccOsc.PLL.PLLState 	= RCC_PLL_ON;
	rccOsc.PLL.PLLM 		= 4;
	rccOsc.PLL.PLLN 		= 168;
	rccOsc.PLL.PLLQ 		= 4;
	rccOsc.PLL.PLLP 		= RCC_PLLP_DIV2;

	HAL_RCC_OscConfig(&rccOsc);

	rccClk.ClockType 		= RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	rccClk.SYSCLKSource 	= RCC_SYSCLKSOURCE_PLLCLK;
	rccClk.AHBCLKDivider 	= RCC_SYSCLK_DIV1;
	rccClk.APB1CLKDivider 	= RCC_HCLK_DIV4;
	rccClk.APB2CLKDivider 	= RCC_HCLK_DIV2;

	HAL_RCC_ClockConfig(&rccClk, FLASH_LATENCY_5);

	/**Configure the Systick interrupt time
	    */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	    */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

	/* Peripheral clocks enable */
	/* ADC ---------------------------------------------------*/
	__HAL_RCC_ADC1_CLK_ENABLE();
	/* DMA ---------------------------------------------------*/
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();
	/* GPIOs -------------------------------------------------*/
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	/* Timers ------------------------------------------------*/
	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_TIM5_CLK_ENABLE();
	__HAL_RCC_TIM9_CLK_ENABLE();
	__HAL_RCC_TIM10_CLK_ENABLE();
	__HAL_RCC_TIM11_CLK_ENABLE();

	/* SPI ---------------------------------------------------*/
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_SPI3_CLK_ENABLE();

	/* USARTs ------------------------------------------------ */
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
}

/**
  * @brief 	Basic GPIO Init: Initialized LEDs indicator for error handling function
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
static void BASIC_GPIO_INIT(void)
{
	/* Define new GPIO handle */
	GPIO_InitTypeDef GPIO_struct;

	GPIO_struct.Pin 		= GPIO_PIN_0;
	GPIO_struct.Mode 		= GPIO_MODE_IT_RISING;
	GPIO_struct.Pull 		= GPIO_PULLDOWN;

	/** @info 	This has already taken care of EXTI interrupt mask
	  *			and interrupt pin routing.
	  */
	HAL_GPIO_Init(GPIOA, &GPIO_struct);

	GPIO_struct.Pin 		= GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_struct.Mode 		= GPIO_MODE_OUTPUT_PP;
	GPIO_struct.Pull		= GPIO_PULLUP;
	GPIO_struct.Speed 		= GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_struct);

	/* Some indicator LEDs */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
/*
	GPIO_struct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	HAL_GPIO_Init(GPIOC, &GPIO_struct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);*/
}

/**
  * @brief  Initializes the Global MSP.
  * @note   This function is called from HAL_Init() function to perform system
  *         level initialization (GPIOs, clock, DMA, interrupt).
  * @retval None
  */
void HAL_MspInit(void)
{
	/* System interrupt init*/
	/* MemoryManagement_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	/* BusFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	/* UsageFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	/* SVCall_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	/* DebugMonitor_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	/* PendSV_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
	/* Clock configuration and PPP enable */
	RCC_INIT();
	/* Basic GPIO configured and enabled */
	BASIC_GPIO_INIT();

	/* Check if the system is properly configured */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(1000);
}

/* Dummy extended ppp init functions */
__weak void EX0_PPP_INIT(void) {}
__weak void EX1_PPP_INIT(void) {}
__weak void EX2_PPP_INIT(void) {}
__weak void EX3_PPP_INIT(void) {}
__weak void EX4_PPP_INIT(void) {}
__weak void EX5_PPP_INIT(void) {}
__weak void EX6_PPP_INIT(void) {}
__weak void EX7_PPP_INIT(void) {}

/* Exported variable ---------------------------------------------------------*/
initHandle_struct initHandles = {
	ADC_INIT,
	TIM1_INIT,
	TIM2_INIT,
	TIM3_INIT,
	TIM4_INIT,
	TIM5_INIT,
	TIM10_INIT,
	USART1_INIT,
	USART2_INIT,
	SPI1_INIT,
	SPI2_INIT,
	EX0_PPP_INIT,
	EX1_PPP_INIT,
	EX2_PPP_INIT,
	EX3_PPP_INIT,
	EX4_PPP_INIT,
	EX5_PPP_INIT,
	EX6_PPP_INIT,
	EX7_PPP_INIT,
	{0},
	{0},
	{0},
	{0},
	{0},
	{0},
	{0},
	{0},
	{0},
	{0},
	{0}
};

/* Exported functions --------------------------------------------------------*/
/**
  * @brief 	ADC_INIT: Initialized ADC peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void ADC_INIT(void)
{

}

/**
  * @brief 	TIM1_INIT: Initialize TIM1 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void TIM1_INIT(void)
{
}

/**
  * @brief 	TIM2_INIT: Initialize TIM2 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void TIM2_INIT(void)
{
	initHandles.TIM2_Handle.Instance 			= TIM2;
}

/**
  * @brief 	TIM3_INIT: Initialize TIM3 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void TIM3_INIT(void)
{
	initHandles.TIM3_Handle.Instance 			= TIM3;
}

/**
  * @brief 	TIM4_INIT: Initialized TIM4 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void TIM4_INIT(void)
{
	initHandles.TIM4_Handle.Instance			= TIM4;
	initHandles.TIM4_Handle.Init.CounterMode 	= TIM_COUNTERMODE_UP;
	initHandles.TIM4_Handle.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;

	// Calculate prescaler and period from sampling frequency
	if (MDRIVE1.MOTOR.SampleFreq == 0)
	{
		_Error_Handler();
	}
	uint32_t prescaler = SystemCoreClock / MDRIVE1.MOTOR.SampleFreq;
	uint16_t period = 0;

	if (prescaler < 0xffff)
	{
		period = prescaler / (prescaler >> 4);
		prescaler = prescaler >> 4;
	}
	else
	{
		period = prescaler / (prescaler >> 8);
		prescaler = prescaler >> 8;
	}
	initHandles.TIM4_Handle.Init.Period 		= period;
	initHandles.TIM4_Handle.Init.Prescaler 		= prescaler;

	HAL_TIM_Base_Init(&initHandles.TIM4_Handle);
	HAL_NVIC_SetPriority(TIM4_IRQn, TIM4_NVIC_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/**
  * @brief 	TIM5_INIT: Initialized TIM5 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void TIM5_INIT(void)
{
	initHandles.TIM5_Handle.Instance 			= TIM5;
}

/**
  * @brief 	TIM10_INIT: Initialized TIM10 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void TIM10_INIT(void)
{

}

/**
  * @brief 	USART2_INIT: Initialized USART1 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void USART1_INIT(void)
{
	initHandles.USART1_Handle.Instance 			= USART1;
	initHandles.USART1_Handle.Init.BaudRate 	= 1382400;
	initHandles.USART1_Handle.Init.Mode 		= UART_MODE_TX_RX;
	initHandles.USART1_Handle.Init.OverSampling	= UART_OVERSAMPLING_16;
	initHandles.USART1_Handle.Init.WordLength 	= UART_WORDLENGTH_8B;
	initHandles.USART1_Handle.Init.StopBits 	= UART_STOPBITS_1;
	initHandles.USART1_Handle.Init.Parity 		= UART_PARITY_NONE;
	initHandles.USART1_Handle.Init.HwFlowCtl 	= UART_HWCONTROL_NONE;

	if (HAL_UART_Init(&initHandles.USART1_Handle) != HAL_OK)
	{
		_Error_Handler();
	}

	GPIO_InitTypeDef GPIO_InitStruct;

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10    ------> USART1_RX
    */
    GPIO_InitStruct.Pin = 						GPIO_PIN_10;
    GPIO_InitStruct.Mode = 						GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = 						GPIO_PULLUP;
    GPIO_InitStruct.Speed = 					GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = 				GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = 						GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART2 interrupt configuration */
   	HAL_NVIC_SetPriority(USART1_IRQn, USART1_NVIC_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief 	USART2_INIT: Initialized USART2 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void USART2_INIT(void)
{
	initHandles.USART2_Handle.Instance 			= USART2;
	initHandles.USART2_Handle.Init.BaudRate 	= 1382400;
	initHandles.USART2_Handle.Init.Mode 		= UART_MODE_TX_RX;
	initHandles.USART2_Handle.Init.OverSampling	= UART_OVERSAMPLING_16;
	initHandles.USART2_Handle.Init.WordLength 	= UART_WORDLENGTH_8B;
	initHandles.USART2_Handle.Init.StopBits 	= UART_STOPBITS_1;
	initHandles.USART2_Handle.Init.Parity 		= UART_PARITY_NONE;
	initHandles.USART2_Handle.Init.HwFlowCtl 	= UART_HWCONTROL_NONE;

	if (HAL_UART_Init(&initHandles.USART2_Handle) != HAL_OK)
	{
		_Error_Handler();
	}

	GPIO_InitTypeDef GPIO_InitStruct;

    /**USART1 GPIO Configuration
    PA2     ------> USART1_TX
    PA3     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = 						GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = 						GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = 						GPIO_PULLUP;
    GPIO_InitStruct.Speed = 					GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = 				GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt configuration */
   	HAL_NVIC_SetPriority(USART2_IRQn, USART2_NVIC_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
  * @brief 	SPI1_INIT: Initialized SPI1 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void SPI1_INIT(void)
{

}

/**
  * @brief 	USART2_INIT: Initialized USART1 peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
__weak void SPI2_INIT(void)
{

}

/** @defgroup initperip_Exported Functions
  * @{
  */
LIME_Status LIME_INIT_SYSTEM(void)
{
	uint8_t status = LIME_OK;
	initHandles.ADC_INIT();
	initHandles.TIM1_INIT();
	initHandles.TIM2_INIT();
	initHandles.TIM3_INIT();
	//initHandles.TIM4_INIT();
	initHandles.USART1_INIT();
	initHandles.USART2_INIT();
	initHandles.SPI1_INIT();
	initHandles.SPI2_INIT();
	initHandles.EX0_PPP_INIT();
	initHandles.EX1_PPP_INIT();
	initHandles.EX2_PPP_INIT();
	initHandles.EX3_PPP_INIT();
	initHandles.EX4_PPP_INIT();
	initHandles.EX5_PPP_INIT();
	initHandles.EX6_PPP_INIT();
	initHandles.EX7_PPP_INIT();

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
	return status;
}
/**
  * @}
  */
