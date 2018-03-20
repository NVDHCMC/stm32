/**
  ******************************************************************************
  * @file    main.c
  * @author  Azure
  * @version V0.2.0
  * @date    8th-October-2017
  * @brief 	 Contain main kernel initialization, OS startup sequence
  * 		 and peripheral interrupt request handlers and callbacks
  *
  * @info 	Handlers: 	Universal Error Handler
  * 					EXTI0 IRQ Handler function
  * 					USART2 IRQ Handler function
  * 					TIM2 IRQ Handler function
  * 					TIM3 IRQ Handler function
  * 					TIM5 IRQ Handler function
  * 					SPI1 IRQ Handler function
  *
  * 		Callbacks: 	UART Receive Complete callback functions
  * 					TIMers Period Elapsed callback
  * 					Input Capture callback
  * 					SPI Receive Complete callback
  *
  *
  ******************************************************************************
*/


/* Includes --------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include <initsys.h>
#include <stdlib.h>
#include <lime_mdriver.h>
#include <lime_mpacket.h>
#include <lime_mpu9255.h>
#include <litve_adc.h>
#include <raspi_spi.h>

/* Private function prototypes -------------------------------------------------*/
extern void 				freertos_init(void);
//extern void 				start_button_init(void);

/* Private variables -----------------------------------------------------------*/
//extern osThreadId 			OPERATION_TaskHandle; 				/* Periodically blink LD15 to indicate operation */
extern osThreadId 			SERIAL_COM_TaskHandle;				/* Serial communication task to communicate with the computer */
extern osThreadId 			ADC_READING_TaskHandle; 			/**/

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Universal Error Handler: handling common errors.
  * @param  none
  * @retval none
  *
  * @notice GPIOD port needs to be initialized for the error handling.
  */
void _Error_Handler()
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /*
   * User can add his own implementation to report the HAL error return state
   */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
	while(1)
	{
		/* Flash all 4 LEDs */
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
		HAL_Delay(500);
	}
  /* USER CODE END Error_Handler_Debug */
}

/* Special functions --------------------------------------------------------*/
/** @defgroup MAIN_Callbacks_and IRQs
  * @{
  */

/** @defgroup MAIN_IRQs
  * @brief    Interrupt request functions for various ISRs
  *
@verbatim
!!! Need to be fixed.
 ===============================================================================
                 ##### Extended features functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Start conversion of injected channel.
      (+) Stop conversion of injected channel.
      (+) Start multimode and enable DMA transfer.
      (+) Stop multimode and disable DMA transfer.
      (+) Get result of injected channel conversion.
      (+) Get result of multimode conversion.
      (+) Configure injected channels.
      (+) Configure multimode.

@endverbatim
  * @{
  */
/** @brief	ADC1 IRQ Handler function: handling ADC Interrupt Request
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void ADC_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&IRLF_ADC.ADC_Handle);
}

/** @brief	EXTI0 IRQ Handler function: handling External Interrupt Request line #0
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/** @brief	USART2 IRQ Handler function: handling USART Interrupt Request line #2
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&initHandles.USART1_Handle);
}

/** @brief	USART2 IRQ Handler function: handling USART Interrupt Request line #2
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&initHandles.USART2_Handle);
}

/** @brief 	TIM2 IRQ Handler function: handling TIMER Interrupt Request device #2
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&initHandles.TIM2_Handle);
}

/** @brief 	TIM3 IRQ Handler function: handling TIMER Interrupt Request device #3
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&initHandles.TIM3_Handle);
}

/** @brief 	TIM5 IRQ Handler function
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void TIM4_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&initHandles.TIM4_Handle);
}

/** @brief 	TIM5 IRQ Handler function
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void TIM5_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&initHandles.TIM5_Handle);
}

/** @brief 	SPI1 IRQ Handler function: handling SPI Interrupt request line #1
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
/*void SPI1_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&SPI1_Handle);
}*/

/** @brief 	SPI2 IRQ Handler function: handling SPI Interrupt request line #2
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
__weak void SPI2_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&initHandles.SPI2_Handle);
}
/**
  * @}
  */

/** @brief 	SPI3 IRQ Handler function: handling SPI Interrupt request line #2
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void SPI3_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&MPU9255DRIVE.SPI_Handle);
}
/**
  * @}
  */

/** @defgroup MAIN_Callbacks
  * @brief    Callback functions for various ISRs
  *
@verbatim
!!! Need to be fixed.
 ===============================================================================
                 ##### Extended features functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Start conversion of injected channel.
      (+) Stop conversion of injected channel.
      (+) Start multimode and enable DMA transfer.
      (+) Stop multimode and disable DMA transfer.
      (+) Get result of injected channel conversion.
      (+) Get result of multimode conversion.
      (+) Configure injected channels.
      (+) Configure multimode.

@endverbatim
  * @{
  */
/** @brief	ADC1 IRQ Handler function: handling ADC Interrupt Request
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* ADC_Handle)
{
	IRLF_ADC.ADC_channel_value[0] 		=	HAL_ADC_GetValue(ADC_Handle);
	osSignalSet(ADC_READING_TaskHandle, limeSignalList.sADC_DONE);
}

/** @brief 	HAL_GPIO_EXTI_Callback
  * @param 	uint16_t GPIO_Pin
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	MDRIVE1.MOTOR.SpeedRef 		= 0x4000;
	MDRIVE2.MOTOR.SpeedRef      = 0x4000;
}

/** @brief 	HAL_UART_TxCpltCallback
  * @param 	UART_HandleTypeDef
  * @retval none
  *
  * @notice unusable if not first initialized in NVIC.
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if ((huart->Instance == USART2) || (huart->Instance == USART1))
	{
		osSignalSet(SERIAL_COM_TaskHandle, limeSignalList.sFINISHED_PUBLISHING);
	}
}

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (uart2Mode == uartNormal)
	{
		if (strncmp((const char *) uart2ComHandle.uartRecvMess.mesContent, "$start", 6) == 0)
		{
			uart2Mode 		= uartAuthenticate;
		}
		uart2ComHandle.uartRecvMess.listening 	= 0;
	}
	else if (uart2Mode == uartAuthenticate)
	{
		if (strncmp((const char *) uart2ComHandle.uartRecvMess.mesContent, ROOT.password, ROOT.pass_len) == 0)
			uart2Mode 		= uartProgram;
		else
		{
			uart2Mode 		= uartNormal;
			uart2ComHandle.uartRecvMess.listening 	= 1;
		}
	}
	else if (uart2Mode == uartProgram)
	{
		motor1.status 		|= MOTOR_SPEED_UPDATE;
		uart2ComHandle.uartRecvMess.listening 	= 0;
	}
}*/

/** @brief 	TIMers Period Elapsed callback
  * @{
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == initHandles.TIM4_Handle.Instance)
	{
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		osSignalSet(SERIAL_COM_TaskHandle, limeSignalList.sDEBUG);
	}
#endif  /* INCLUDE_xTaskGetSchedulerState */
	}
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
}

/** @brief 	Input Capture callback
  * @{
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
}


/** @brief 	SPI Receive Complete callback
  * @{
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
	{
#ifdef DEBUG
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
#endif
	}
	else if (hspi->Instance == SPI2)
	{
		//osSignalSet(SERIAL_COM_TaskHandle, rpi_signals.spi_complete_signal);
#ifdef DEBUG
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
#endif
	}
}
/**
  * @}
  */
/*
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	}
}
*/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2)
	{
#ifdef DEBUG
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
#endif
		osSignalSet(SERIAL_COM_TaskHandle, rpi_signals.spi_complete_signal);
	}
	else if (hspi->Instance == MPU9255DRIVE.SPI_Handle.Instance)
	{
		__HAL_SPI_DISABLE(hspi);
	}
}
/**
  * @}
  */

/**
  * @}
  */

/** @brief 	MAIN: to initialize Low level driver for peripheral devices and the OS system kernel.
  * @param	none
  * @retval none
  */
int main(void)
{
	/* HAL Module Initializations */


 	HAL_Init();

	//initHandles.EX4_PPP_INIT = start_button_init;
	//initHandles.EX5_PPP_INIT = IRLF_ADC.ADC_INIT;
	//initHandles.EX6_PPP_INIT = IRLF_ADC.DMA_INIT;

	/**/
	//MDRIVE1.MOTOR_INIT();
	//MDRIVE2.MOTOR_INIT();

	/**/
	LIME_INIT_SYSTEM();

	/**/
	//L3GD20DRIVE.INIT();
	MPU9255DRIVE.INIT();

	if (MPU9255DRIVE.READ_ID() != I_AM_MPU9255)
	{
		_Error_Handler();
	}

	/**/
	//MDRIVE1.MOTOR_START();
	//MDRIVE2.MOTOR_START();

	/* FreeRTOS task init */
	freertos_init();

	/* Start the scheduler */
	osKernelStart();

	for(;;)
	{
		// The microprocessor should never reach here
	}
}
