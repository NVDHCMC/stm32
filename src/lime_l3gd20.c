/**
  ******************************************************************************
  * @file    l3gd20.c
  * @author  Azure
  * @version V0.2.0
  * @date    26-June-2015
  * @brief   This file provides a set of functions needed to manage the L3GD20,
  *          ST MEMS motion sensor, 3-axis digital output gyroscope.
  ******************************************************************************
***/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <initsys.h>
#include <lime_l3gd20.h>

#ifdef __USE_FREERTOS__
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif

extern void _Error_Handler(void);
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief 	LIME_L3GD20_SPI_INIT: Initialize l3gd20 spi interface
  * @param 	none
  * @retval none
  */
static void LIME_L3GD20_SPI_INIT(void)
{
	/* @brief 	New SPI Handle configuration: SPI1 interface
	 * for the nano acceleration MEMS sensor on the STM32F411 Discovery board.
	 */
	L3GD20DRIVE.SPI_Handle.Instance 		= LIME_L3GD20_SPI;
	/* Set SPI1 to identify as master node. */
	L3GD20DRIVE.SPI_Handle.Init.Mode 		= SPI_MODE_MASTER;
	/* Set baud rate at 6.75 MHz (Baud rate has to be less than 10 MHz as
	 * specified in the data sheet of the sensor.
	 */
	L3GD20DRIVE.SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	/* */
	L3GD20DRIVE.SPI_Handle.Init.CLKPhase 	= SPI_PHASE_2EDGE;
	L3GD20DRIVE.SPI_Handle.Init.CLKPolarity= SPI_POLARITY_HIGH;
	/* SPI receives and sends 8-bit data packets. */
	L3GD20DRIVE.SPI_Handle.Init.DataSize 	= SPI_DATASIZE_8BIT;
	/* Set the SPI to use 2 unidirectional lines MISO and MOSI
	 * for reception and transmission of messages.
	 */
	L3GD20DRIVE.SPI_Handle.Init.Direction 	= SPI_DIRECTION_2LINES;
	/* Select the first bit of the transmission to be the most significant bit */
	L3GD20DRIVE.SPI_Handle.Init.FirstBit 	= SPI_FIRSTBIT_MSB;
	/* Disable CRC error checking calculation */
	L3GD20DRIVE.SPI_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	/* Using software slave select */
	L3GD20DRIVE.SPI_Handle.Init.NSS 		= SPI_NSS_SOFT;

	HAL_SPI_Init(&L3GD20DRIVE.SPI_Handle);

	HAL_NVIC_SetPriority(SPI1_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

/**
  * @brief 	LIME_L3GD20_MSPINIT: Initialize l3gd20 communication interface hardware
  * @param 	none
  * @retval none
  */
static void LIME_L3GD20_MSPINIT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* @brief 	Initialize SPI1 pins
	 * @info 	PA5 - SPI1_MOSI
	 * 			PA6 - SPI1_MISO
	 * 			PA7 - SPI1_SCK
	 */
	GPIO_InitStruct.Pin 		= LIME_L3GD20_SPI_SCK_PIN | LIME_L3GD20_SPI_MISO_PIN | LIME_L3GD20_SPI_MOSI_PIN;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Alternate 	= LIME_L3GD20_SPI_AF;
	GPIO_InitStruct.Pull		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(LIME_L3GD20_SPI_GPIO_PORT, &GPIO_InitStruct);

	/* @brief 	Initialize SS/CS pin for the slave selection pin of the sensor
	 * @info 	PE3 - SS/CS
	 */
	GPIO_InitStruct.Pin 		= LIME_L3GD20_CS_PIN;
	GPIO_InitStruct.Mode 		= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull 		= GPIO_PULLUP;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(LIME_L3GD20_CS_GPIO_PORT, &GPIO_InitStruct);
	LIME_L3GD20_CS_HIGH();

	/** @brief
	  * @info
	  */
}

/**
  * @brief 	LIME_L3GD20_HWINIT: Initialize l3gd20 hardware registers
  * @param 	none
  * @retval none
  */
static void LIME_L3GD20_HWINIT(void)
{
	uint16_t reg_cmd = 0;
	/* Update control register 1 */
	reg_cmd 		= L3GD20DRIVE.Init.PowerStatus | L3GD20DRIVE.Init.Axis |\
					  L3GD20DRIVE.Init.Bandwidth | L3GD20DRIVE.Init.OutputRate;
	if (LIME_MEMS_INITCMD(reg_cmd, (uint8_t) L3GD20_CTRL_REG1_ADDR, 1) == LIME_ERROR)
	{
		_Error_Handler();
	}

	/* Update control register 4 */
	reg_cmd 		= L3GD20DRIVE.Init.Endianess | L3GD20DRIVE.Init.Updatemode \
					  | L3GD20DRIVE.Init.Fullscale;
	if (LIME_MEMS_INITCMD(reg_cmd, (uint8_t) L3GD20_CTRL_REG4_ADDR, 1) == LIME_ERROR)
	{
		_Error_Handler();
	}

	reg_cmd 		= L3GD20DRIVE.Init.IntPinConfig;
	if (LIME_MEMS_INITCMD(reg_cmd, (uint8_t) L3GD20_CTRL_REG3_ADDR, 1) == LIME_ERROR)
	{
		_Error_Handler();
	}
}

/* Exported functions ---------------------------------------------------------*/
/**
  * @brief 	LIME_MEMS_INITCMD: Sending init register command to l3gd20
  * @param 	none
  * @retval none
  */
LIME_Status LIME_L3GD20_INITCMD(uint8_t reg_cmd, uint8_t REG_ADDR, uint8_t numByte)
{
	uint8_t status = LIME_OK;
	uint8_t rx[4] = {0, 0, 0, 0};
	uint8_t tx[4] = {REG_ADDR, reg_cmd, (REG_ADDR | READWRITE_CMD), 0};
	if (numByte == 1)
	{
		for (int i = 0; i < 2; i++)
		{
			LIME_L3GD20_CS_LOW();
			if (HAL_SPI_TransmitReceive(&L3GD20DRIVE.SPI_Handle, tx, rx, 2, configL3GD20_MAX_TIMEOUT) != HAL_OK)
			{
				_Error_Handler();
			}
			LIME_L3GD20_CS_HIGH();
			HAL_Delay(1);
		}
		LIME_L3GD20_CS_LOW();
		if (HAL_SPI_TransmitReceive(&L3GD20DRIVE.SPI_Handle, (tx + 2), (rx + 2), 2, configL3GD20_MAX_TIMEOUT) != HAL_OK)
		{
			_Error_Handler();
		}
		LIME_L3GD20_CS_HIGH();

		if (rx[3] != reg_cmd)
		{
			status = LIME_ERROR;
		}
	}
	else
	{

	}

	return status;
}

/**
  * @brief 	LIME_L3GD20_INIT: Initialize l3gd20
  * @param 	none
  * @retval none
  */
void LIME_L3GD20_INIT(void)
{
	LIME_L3GD20_SPI_INIT();
	LIME_L3GD20_MSPINIT();

	/* L3GD20 control register 1 */
	L3GD20DRIVE.Init.PowerStatus 				= L3GD20_MODE_ACTIVE;
	L3GD20DRIVE.Init.Axis 						= L3GD20_AXES_ENABLE;
	L3GD20DRIVE.Init.OutputRate 				= L3GD20_OUTPUT_DATARATE_4;
	L3GD20DRIVE.Init.Bandwidth 					= L3GD20_BANDWIDTH_2;
	/* L3GD20 control register 4 */
	L3GD20DRIVE.Init.Updatemode 				= L3GD20_BlockDataUpdate_Continous;
	L3GD20DRIVE.Init.Endianess 					= L3GD20_BLE_MSB;
	L3GD20DRIVE.Init.Fullscale 					= L3GD20_FULLSCALE_2000;
	/* L3GD20 control register 3 */
	L3GD20DRIVE.Init.IntPinConfig 				= L3GD20_INT2INTERRUPT_ENABLE;

	LIME_L3GD20_HWINIT();
}

/**
  * @brief 	LIME_L3GD20_UPDATE: Initialize l3gd20
  * @param 	none
  * @retval none
  */
void LIME_L3GD20_UPDATE(LIME_L3GD20_DataType* pData)
{

}

/**
  * @brief 	LIME_L3GD20_READID: Confirm L3GD20 id.
  * @param 	none
  * @retval none
  */
uint8_t LIME_L3GD20_READID(void)
{
	uint8_t ID = 0;
	uint8_t rx[2] = {0, 0};
	uint8_t tx[2] = {(L3GD20_WHO_AM_I_ADDR | READWRITE_CMD), 0};

#if (INCLUDE_xTaskGetSchedulerState  == 1 )
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		if (HAL_SPI_TransmitReceive_IT(&L3GD20DRIVE.SPI_Handle, tx, rx, 2) != HAL_OK)
		{
			// insert error handler code here.
		}
	}
	else
	{
#endif  /* INCLUDE_xTaskGetSchedulerState */
	if (HAL_SPI_TransmitReceive(&L3GD20DRIVE.SPI_Handle, tx, rx, 2, configL3GD20_MAX_TIMEOUT) != HAL_OK)
	{
		_Error_Handler();
	}
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
	}
#endif  /* INCLUDE_xTaskGetSchedulerState */
	return ID;
}

/* Private variables ---------------------------------------------------------*/
LIME_MEMSDRIVE_Type L3GD20DRIVE =
{
	{0},
	{0},
	{0},
	LIME_L3GD20_INIT,
	LIME_L3GD20_UPDATE,
	LIME_L3GD20_READID
};

