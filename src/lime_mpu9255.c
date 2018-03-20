/**
  ******************************************************************************
  * @file    lime_mpu9255.c
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
#include <lime_mpu9255.h>

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
  * @brief 	LIME_MPU9255_SPI_INIT: Initialize mpu9255 spi interface
  * @param 	none
  * @retval none
  */
static void LIME_MPU9255_SPI_INIT(void)
{
	/* @brief 	New SPI Handle configuration: SPI1 interface
	 * for the nano acceleration MEMS sensor on the STM32F411 Discovery board.
	 */
	MPU9255DRIVE.SPI_Handle.Instance 		= LIME_MPU9255_SPI;
	/* Set SPI1 to identify as master node. */
	MPU9255DRIVE.SPI_Handle.Init.Mode 		= SPI_MODE_MASTER;
	/* Set baud rate at 6.75 MHz (Baud rate has to be less than 10 MHz as
	 * specified in the data sheet of the sensor.
	 */
	MPU9255DRIVE.SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	/* */
	MPU9255DRIVE.SPI_Handle.Init.CLKPhase 	= SPI_PHASE_2EDGE;
	MPU9255DRIVE.SPI_Handle.Init.CLKPolarity= SPI_POLARITY_HIGH;
	/* SPI receives and sends 8-bit data packets. */
	MPU9255DRIVE.SPI_Handle.Init.DataSize 	= SPI_DATASIZE_8BIT;
	/* Set the SPI to use 2 unidirectional lines MISO and MOSI
	 * for reception and transmission of messages.
	 */
	MPU9255DRIVE.SPI_Handle.Init.Direction 	= SPI_DIRECTION_2LINES;
	/* Select the first bit of the transmission to be the most significant bit */
	MPU9255DRIVE.SPI_Handle.Init.FirstBit 	= SPI_FIRSTBIT_MSB;
	/* Disable CRC error checking calculation */
	MPU9255DRIVE.SPI_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	/* Using software slave select */
	MPU9255DRIVE.SPI_Handle.Init.NSS 		= SPI_NSS_HARD_OUTPUT;

	HAL_SPI_Init(&MPU9255DRIVE.SPI_Handle);

	HAL_NVIC_SetPriority(SPI3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

static void LIME_MPU9255_MSPINIT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* @brief 	Initialize SPI1 pins
	 * @info 	PA4 - SPI1_NSS
	 * 			PA5 - SPI1_SCK
	 * 			PA6 - SPI1_MOSI
	 * 			PA7 - SPI1_MISO
	 */
	GPIO_InitStruct.Pin 		= LIME_MPU9255_SPI_SCK_PIN | LIME_MPU9255_SPI_MISO_PIN | LIME_MPU9255_SPI_MOSI_PIN;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Alternate 	= LIME_MPU9255_SPI_AF;
	GPIO_InitStruct.Pull		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(LIME_MPU9255_SPI_GPIO_PORT, &GPIO_InitStruct);

	/* @brief 	Initialize SS/CS pin for the slave selection pin of the sensor
	 * @info 	PE3 - SS/CS
	 */
	GPIO_InitStruct.Pin 		= LIME_MPU9255_CS_PIN;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Alternate 	= LIME_MPU9255_SPI_AF;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(LIME_MPU9255_CS_GPIO_PORT, &GPIO_InitStruct);

	/** @brief
	  * @info
	  */
}

/**
  * @brief 	LIME_MEMS_INITCMD: Sending init register command to MPU9255
  * @param 	none
  * @retval none
  */

static LIME_Status LIME_MPU9255_INITCMD(uint8_t reg_cmd, uint8_t REG_ADDR, uint8_t numByte)
{
	uint8_t status = LIME_OK;
	uint8_t rx[4] = {0, 0, 0, 0};
	uint8_t tx[4] = {REG_ADDR, reg_cmd, (REG_ADDR | READWRITE_CMD), 0};
	if (numByte == 1)
	{
		for (int i = 0; i < 2; i++)
		{
			if (HAL_SPI_TransmitReceive(&MPU9255DRIVE.SPI_Handle, tx, rx, 2, configMPU9255_MAX_TIMEOUT) != HAL_OK)
			{
				_Error_Handler();
			}
			__HAL_SPI_DISABLE(&MPU9255DRIVE.SPI_Handle);
			HAL_Delay(1);
		}
		if (HAL_SPI_TransmitReceive(&MPU9255DRIVE.SPI_Handle, (tx + 2), (rx + 2), 2, configMPU9255_MAX_TIMEOUT) != HAL_OK)
		{
			_Error_Handler();
		}
		__HAL_SPI_DISABLE(&MPU9255DRIVE.SPI_Handle);

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
  * @brief 	LIME_MEMS_INITCMD: Sending init register command to MPU9255
  * @param 	none
  * @retval none
  */
static void LIME_MPU9255_WRITE_REG(uint8_t reg_cmd, uint8_t REG_ADDR, int num_byte)
{
	uint8_t rx[2] = {0, 0};
	uint8_t tx[2] = {(REG_ADDR), reg_cmd};
	if (HAL_SPI_TransmitReceive(&MPU9255DRIVE.SPI_Handle, tx, rx, 2, configMPU9255_MAX_TIMEOUT) != HAL_OK)
	{
		_Error_Handler();
	}
	__HAL_SPI_DISABLE(&MPU9255DRIVE.SPI_Handle);
}

/**
  * @brief 	LIME_MEMS_INITCMD: Sending init register command to MPU9255
  * @param 	none
  * @retval none
  */
static void LIME_MPU9255_READ_REG(uint8_t REG_ADDR, uint8_t * pData, int num_byte)
{
	uint8_t rx[num_byte + 1];
	uint8_t tx[num_byte + 1];
	tx[0] = (REG_ADDR | READWRITE_CMD);
	if (HAL_SPI_TransmitReceive(&MPU9255DRIVE.SPI_Handle, tx, rx, num_byte + 1, configMPU9255_MAX_TIMEOUT) != HAL_OK)
	{
		_Error_Handler();
	}
	__HAL_SPI_DISABLE(&MPU9255DRIVE.SPI_Handle);
	for (int i = 0; i < num_byte; i++)
		*(pData + i) = rx[i + 1];
}

/**
  * @brief 	LIME_MPU9255_HWINIT: Initialize MPU9255 hardware registers
  * @param 	none
  * @retval none
  */
static void LIME_MPU9255_HWINIT(void)
{
	uint8_t reg_cmd = 0;
	uint8_t error_code = LIME_OK;
	if (LIME_MPU9255_INITCMD(reg_cmd, (uint8_t) MPU9255_PWR_MGMT_1_ADDR, 1) == LIME_ERROR) error_code = LIME_ERROR;

	/* Write to Power management 1 register */
	reg_cmd 		= MPU9255DRIVE.Init.PowerMnt1;
	if (LIME_MPU9255_INITCMD(reg_cmd, (uint8_t) MPU9255_PWR_MGMT_1_ADDR, 1) == LIME_ERROR) error_code = LIME_ERROR;

	/* Write to Power management 2 register */
	reg_cmd 		= ~MPU9255DRIVE.Init.Axis;
	if (LIME_MPU9255_INITCMD(reg_cmd, (uint8_t) MPU9255_PWR_MGMT_2_ADDR, 1) == LIME_ERROR) error_code = LIME_ERROR;

	/* Config gyro operation */
	reg_cmd 		= MPU9255DRIVE.Init.GyroConfig;
	if (LIME_MPU9255_INITCMD(reg_cmd, (uint8_t) MPU9255_GYRO_CONFIG_ADDR, 1) == LIME_ERROR) error_code = LIME_ERROR;

	reg_cmd 		= MPU9255DRIVE.Init.Config | MPU9255DRIVE.Init.Bandwidth;
	if (LIME_MPU9255_INITCMD(reg_cmd, (uint8_t) MPU9255_CONFIG_ADDR, 1) == LIME_ERROR) error_code = LIME_ERROR;

	reg_cmd 		= MPU9255DRIVE.Init.AccelConfig1;
	if (LIME_MPU9255_INITCMD(reg_cmd, (uint8_t) MPU9255_ACCEL_CONFIG_ADDR, 1) == LIME_ERROR) error_code = LIME_ERROR;

	reg_cmd 		= MPU9255DRIVE.Init.AccelConfig2;
	if (LIME_MPU9255_INITCMD(reg_cmd, (uint8_t) MPU9255_ACCEL_CONFIG_2_ADDR, 1) == LIME_ERROR) error_code = LIME_ERROR;

	reg_cmd 		= MPU9255DRIVE.Init.IntPinConfig;
	// Reset all sensors' digital signal path
	reg_cmd 		= 0x07;
	LIME_MPU9255_WRITE_REG(reg_cmd, MPU9255_SIGNAL_PATH_RST_ADDR, 1);

	// Reset all sensors' register and signal path
	reg_cmd 		= 0x01;
	LIME_MPU9255_WRITE_REG(reg_cmd, MPU9255_USER_CTRL_ADDR, 1);

	if (MPU9255DRIVE.Init.MagnetoConfig != 0x00)
	{
		// Reset I2C slave module
		reg_cmd 		= 0x10;
		LIME_MPU9255_WRITE_REG(reg_cmd, MPU9255_USER_CTRL_ADDR, 1);

		// Enable I2C master
		reg_cmd 	= 0x20;
		if (LIME_MPU9255_INITCMD(reg_cmd, (uint8_t) MPU9255_USER_CTRL_ADDR, 1) == LIME_ERROR) error_code = LIME_ERROR;

		// Set I2C channel speed to 400 kHz
		if (LIME_MPU9255_INITCMD(0x0D, (uint8_t) MPU9255_I2C_MASTER_CTRL, 1) == LIME_ERROR) error_code = LIME_ERROR;

		if (LIME_MPU9255_INITCMD(0x0c, (uint8_t) 37, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(0x0b, (uint8_t) 38, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(0x01, (uint8_t) 99, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(0x81, (uint8_t) 39, 1) == LIME_ERROR) error_code = LIME_ERROR;

		// Set the address of AK8963 to slave 0 address register
		reg_cmd 	= 0x0C | READWRITE_CMD;
		if (LIME_MPU9255_INITCMD(reg_cmd, (uint8_t) 37, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(0x00, (uint8_t) 38, 1) == LIME_ERROR) error_code = LIME_ERROR;

		// Enable I2C master module and read 1 byte
		if (LIME_MPU9255_INITCMD(0x81, (uint8_t) 39, 1) == LIME_ERROR) error_code = LIME_ERROR;

		// Read 1 byte retrieved from AK8963
		LIME_MPU9255_READ_REG(0x49, &reg_cmd, 1);
		if (reg_cmd != 0x48) error_code = LIME_ERROR;

		// Set AK8963 address to write mode
		if (LIME_MPU9255_INITCMD(0x0c, (uint8_t) 37, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(0x0b, (uint8_t) 38, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(0x01, (uint8_t) 99, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(0x81, (uint8_t) 39, 1) == LIME_ERROR) error_code = LIME_ERROR;

		if (LIME_MPU9255_INITCMD(0x0a, (uint8_t) 38, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(MPU9255DRIVE.Init.MagnetoConfig, (uint8_t) 99, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(0x81, (uint8_t) 39, 1) == LIME_ERROR) error_code = LIME_ERROR;

		if (LIME_MPU9255_INITCMD((0x0c | READWRITE_CMD), (uint8_t) 37, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(0x03, (uint8_t) 38, 1) == LIME_ERROR) error_code = LIME_ERROR;
		if (LIME_MPU9255_INITCMD(0x87, (uint8_t) 39, 1) == LIME_ERROR) error_code = LIME_ERROR;
	}

	if (error_code != LIME_OK) _Error_Handler();
}

/* Exported functions ---------------------------------------------------------*/
/**
  * @brief 	LIME_MPU9255_INIT: Initialize MPU9255
  * @param 	none
  * @retval none
  */
void LIME_MPU9255_INIT(void)
{
	LIME_MPU9255_SPI_INIT();
	LIME_MPU9255_MSPINIT();

	MPU9255DRIVE.Init.Axis = MPU9255_GYRO_X_ENABLE | MPU9255_GYRO_Y_ENABLE | MPU9255_GYRO_Z_ENABLE | \
			MPU9255_ACCEL_X_ENABLE | MPU9255_ACCEL_Y_ENABLE | MPU9255_ACCEL_Z_ENABLE;
	MPU9255DRIVE.Init.GyroConfig = MPU9255_FULLSCALE_250 | MPU9255_GYRO_FILTER_3;
	MPU9255DRIVE.Init.AccelConfig1 = MPU9255_FULLSCALE_2G;
	MPU9255DRIVE.Init.AccelConfig2 = MPU9255_ACCEL_FILTER_1 | MPU9255_BANDWIDTH_2;
	MPU9255DRIVE.Init.MagnetoConfig = 0x16;
	MPU9255DRIVE.Init.Config = 0;
	MPU9255DRIVE.Init.PowerMnt1 = 0x01;
	MPU9255DRIVE.Init.Bandwidth = MPU9255_BANDWIDTH_4;
	LIME_MPU9255_HWINIT();
}

/**
  * @brief 	LIME_MPU9255_UPDATE: Initialize MPU9255
  * @param 	none
  * @retval none
  */
void LIME_MPU9255_UPDATE(LIME_MPU9255_DataType pData)
{
	/*
	uint8_t temperature_lower_byte = 0;
	uint8_t temperature_higher_byte = 0;
	temperature_lower_byte = LIME_MPU9255_READ_REG(MPU9255_TEMP_OUT_L_ADDR, 1);
	temperature_higher_byte = LIME_MPU9255_READ_REG(MPU9255_TEMP_OUT_H_ADDR, 1);
	*pData = (float) ((((uint16_t) temperature_higher_byte) << 8) + (uint16_t) temperature_lower_byte)/ 333.87 + 21.0;
	*/

	uint8_t pTempData[21] = {0};
	LIME_MPU9255_READ_REG(MPU9255_ACCEL_OUT_X_H_ADDR, pTempData, 21);
	//gyro_x_higher_byte = LIME_MPU9255_READ_REG(MPU9255_ACCEL_OUT_X_H_ADDR, 1);

	//gyro_y_lower_byte = LIME_MPU9255_READ_REG(MPU9255_ACCEL_OUT_Y_L_ADDR, 1);
	//gyro_y_higher_byte = LIME_MPU9255_READ_REG(MPU9255_ACCEL_OUT_Y_H_ADDR, 1);

	//gyro_z_lower_byte = LIME_MPU9255_READ_REG(MPU9255_ACCEL_OUT_Z_L_ADDR, 1);
	//gyro_z_higher_byte = LIME_MPU9255_READ_REG(MPU9255_ACCEL_OUT_Z_H_ADDR, 1);
	//*pData = (float) ((int16_t)((((int16_t) gyro_x_higher_byte) << 8) + (int16_t) gyro_x_lower_byte))/ 131.0f;
	//*(pData + 1) = (float) ((int16_t)((((int16_t) gyro_y_higher_byte) << 8) + (int16_t) gyro_y_lower_byte))/ 131.0f;
	//*(pData + 2) = (float) ((int16_t)((((int16_t) gyro_z_higher_byte) << 8) + (int16_t) gyro_z_lower_byte))/ 131.0f;
	for (int i = 0; i < 20; i++)
	{
		pData[i] = pTempData[i];
	}
}

/**
  * @brief 	LIME_MPU9255_READID: Confirm MPU9255 id.
  * @param 	none
  * @retval none
  */
static uint8_t LIME_MPU9255_READID(void)
{
	uint8_t ID = 0;
	uint8_t rx[2] = {0, 0};
	uint8_t tx[2] = {(MPU9255_WHO_AM_I_ADDR | READWRITE_CMD), 0};
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		if (HAL_SPI_TransmitReceive_IT(&MPU9255DRIVE.SPI_Handle, tx, rx, 2) != HAL_OK)
		{
			_Error_Handler();// insert error handler code here.
		}
		//Need some work on this
		//__HAL_SPI_DISABLE(&MPU9255DRIVE.SPI_Handle);
	}
	else
	{
#endif  /* INCLUDE_xTaskGetSchedulerState */
	if (HAL_SPI_TransmitReceive(&MPU9255DRIVE.SPI_Handle, tx, rx, 2, configMPU9255_MAX_TIMEOUT) != HAL_OK)
	{
		_Error_Handler();
	}
	__HAL_SPI_DISABLE(&MPU9255DRIVE.SPI_Handle);
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
	}
#endif  /* INCLUDE_xTaskGetSchedulerState */
	return ID = rx[1];
}

/* Private variables ---------------------------------------------------------*/
LIME_MEMSDRIVE_Type MPU9255DRIVE =
{
	{0},
	{0},
	{0},
	LIME_MPU9255_INIT,
	LIME_MPU9255_UPDATE,
	LIME_MPU9255_READID
};
