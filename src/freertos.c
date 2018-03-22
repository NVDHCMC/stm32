/**
  ******************************************************************************
  * @file    freertos.c
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

/* Includes */
#include "stm32f4xx.h"
#include <math.h>
#include <initsys.h>
#include <lime_mdriver.h>
#include <lime_mpacket.h>
#include <lime_mpu9255.h>
#include <litve_adc.h>
#include <raspi_spi.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <UbloxGPS.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId 		OPERATION_TaskHandle; 				/* Periodically blink LD15 to indicate operation */
osThreadId 		SERIAL_COM_TaskHandle;				/* Serial communication task to communicate with the computer */
osThreadId		GPS_UPDATE_TaskHandle;				/**/
osThreadId 		INS_READING_TaskHandle;				/**/

/* Private function prototypes -----------------------------------------------*/
void 			freertos_init(void);
/* Task/Threads functions ----------------------------------------------------*/
/**
  * @brief 	OPERATION_task: Periodically blink LD15 to indicate operation
  * @param 	argument
  * @retval none
  */
void OPERATION_task(void const *argument)
{
	uint32_t task_wake_time = 0;
	for(;;)
	{
		task_wake_time = osKernelSysTick();
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		raspi_spi_updatewdt();
		osDelayUntil(&task_wake_time, 500);
	}
}

/**
  * @brief 	SERIAL_COM_task : Serial communication task to communicate with the computer
  * @param 	argument
  * @retval none
  */
void SERIAL_COM_task(void const *argument)
{
	uint8_t initonce = 0;
	//float compensated_angle[3] = {0.0, 0.0, 0.0};
	for (;;)
	{
		if (initonce == 0)
		{
			raspi_spi_init();
			initonce = 1;
		}
		else
		{
			raspi_spi_task();
		}
	}
}

/**
  * @brief 	PWM_UPDATE_task : Periodically update PWM value.
  * @param 	argument
  * @retval none
  */
void GPS_UPDATE_task(void const *argument)
{
	//int init_once = 0;
	osEvent evt;
	for (;;)
	{
		evt = osSignalWait(USART1_COMPLETE_FLAG, osWaitForever);
		if ((evt.value.signals & USART1_COMPLETE_FLAG) == USART1_COMPLETE_FLAG) {
			
		}
	}
}

/**
  * @brief 	PWM_UPDATE_task : Serial communication task to communicate with the computer
  * @param 	argument
  * @retval none
  */
void IMU_READING_task(void const *argument)
{
	//int initonce = 0;
	uint8_t data[20] = {0};
	uint32_t task_wake_time = 0;
	osEvent evt;
	for (;;)
	{
		evt = osSignalWait(SPI_COMPLETE_FLAG, osWaitForever);
		if ((evt.value.signals & SPI_COMPLETE_FLAG) == SPI_COMPLETE_FLAG) {
			MPU9255DRIVE.UPDATE(data);
			raspi_spi_get_data(data);
		}
	}
}

/**
  * @brief 	freertos_init function: Initialize all the tasks.
  * @param 	none
  * @retval none
  */
void freertos_init(void)
{
	/* Task def and set task priority */
	osThreadDef(OPERATION, OPERATION_task, osPriorityNormal, 0, 128);
	OPERATION_TaskHandle 		= osThreadCreate(osThread(OPERATION), NULL);

	osThreadDef(uart2Com,  SERIAL_COM_task, osPriorityNormal, 0, 256);
	SERIAL_COM_TaskHandle 			= osThreadCreate(osThread(uart2Com), NULL);

	osThreadDef(PWM_UPDATE, GPS_UPDATE_task, osPriorityHigh, 0, 128);
	PWM_UPDATE_TaskHandle 		= osThreadCreate(osThread(PWM_UPDATE), NULL);

	osThreadDef(ADC_READING, IMU_READING_task, osPriorityHigh, 0, 200);
	ADC_READING_TaskHandle 	 	= osThreadCreate(osThread(ADC_READING), NULL);

	/* Allocate package structure memory */

}
