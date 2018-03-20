/**
  ******************************************************************************
  * @file    lime_mpu9255.c
  * @author  Azure
  * @version V0.2.0
  * @date    Mon Jan, 29th 2018
  * @brief   This file provides a set of functions needed to manage the L3GD20,
  *          ST MEMS motion sensor, 3-axis digital output gyroscope.
  ******************************************************************************
***/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <initsys.h>
#include <raspi_spi.h>
#include "cmsis_os.h"

extern void _Error_Handler(void);

/* Exported variables --------------------------------------------------------*/
raspi_signal_t rpi_signals = {
	0
};

/* Private typedef -----------------------------------------------------------*/
__inline static void update_watchdog_timer();
__inline static void get_data(uint8_t * data);

static raspi_com_t rpi_spi_handle = {
	{0},
	{0},
	{0},
	0,
	{0},
	{0},
	0,
	{0},
	{0xff, 'h', 'e', 'l', 'l', 'o', ',', 'R', 'P', 'i'},
	{0x43, 0x85, 0x31, 0x77, 0x8c, 0xc4, 0x8e, 0xc9, 0x4b, 0x11},
	{0xff, 'S', 'T', 'M', '3', '2', 'F', '4', '0', '7'},
	{0},
	update_watchdog_timer,
	get_data
};
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
__inline static void update_watchdog_timer()
{
	rpi_spi_handle.timeout++;
}

__inline static void get_data(uint8_t * data)
{
	for (int i = 0; i < 20; i++) {
		rpi_spi_handle.data[i] = data[i];
	}
}

/* Exported functions ----------------------------------------------------------*/
void raspi_spi_task()
{
	if (rpi_spi_handle.rpi_spi_handle.Instance != SPI2)
	{
		_Error_Handler();
	}

	for (;;)
	{
		int init = 0;
		int paired = 0;
		int flag = 0;
		rpi_spi_handle.timeout = 0;
		while ((init == 0) && (rpi_spi_handle.timeout < config_TIMEOUT)) {
			// Receive init message
			HAL_SPI_TransmitReceive(&rpi_spi_handle.rpi_spi_handle, rpi_spi_handle.sync_mess, rpi_spi_handle.rx_buffer, 10, 2000);

			//osSignalWait(rpi_signals.spi_complete_signal, 2000);
			if (rpi_spi_handle.rx_buffer[0] == 0xff)
			{
				init = 1;
				rpi_spi_handle.rx_buffer[0] = 0x00;
				rpi_spi_handle.timeout = 0;
			}
			else {
				init = 0;
			}
		}


		// Waiting for pair command
		while ((paired == 0) && (rpi_spi_handle.timeout < config_TIMEOUT) && (init == 1)) {
			HAL_SPI_TransmitReceive(&rpi_spi_handle.rpi_spi_handle, rpi_spi_handle.dummy_mess, rpi_spi_handle.rx_buffer, 10, 2000);

			paired = 1;
			for ( int i = 0; i < 10; i++) {
				if (rpi_spi_handle.rx_buffer[i] != rpi_spi_handle.id_mess[i]) {
					paired = 0;
				}
			}

			if (paired == 1) {
				HAL_SPI_TransmitReceive(&rpi_spi_handle.rpi_spi_handle, rpi_spi_handle.pair_mess, rpi_spi_handle.rx_buffer, 10, 2000);

				rpi_spi_handle.timeout = 0;
			}
		}
		if (paired == 1) {
			// Main communication loop
			while ((rpi_spi_handle.timeout <= config_TIMEOUT) && (rpi_spi_handle.error_code == 0x00))
			{
				if (flag == 0) {
					// Receiving opcode + address
					HAL_SPI_Abort(&rpi_spi_handle.rpi_spi_handle);
					HAL_SPI_Abort_IT(&rpi_spi_handle.rpi_spi_handle);

					if (HAL_SPI_TransmitReceive_DMA(&rpi_spi_handle.rpi_spi_handle, rpi_spi_handle.data, rpi_spi_handle.rx_buffer, 20) == HAL_ERROR)
					{
						_Error_Handler();
					}
					flag = 1;
				}
				else {
					rpi_spi_handle.timeout = 0;
					HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
					osDelay(500);
				}
				/*
				switch (rpi_spi_handle.rx_buffer[0]) {
				case SPI_DATA:
					break;
				case SPI_CONF:
					break;
				case SPI_STAT:
					break;
				default:
					rpi_spi_handle.error_code = 0x01;
				}*/
			}
		}
	}
}

void raspi_spi_updatewdt()
{
	rpi_spi_handle.wdt_update();
}

void raspi_spi_get_data(uint8_t * data)
{
	rpi_spi_handle.get_data(data);
}

void raspi_spi_init()
{
	/* @brief 	New SPI Handle configuration: SPI1 interface
	 * for the nano acceleration MEMS sensor on the STM32F411 Discovery board.
	 */
	rpi_spi_handle.rpi_spi_handle.Instance 		= SPI2;
	/* Set SPI1 to identify as slave node. */
	rpi_spi_handle.rpi_spi_handle.Init.Mode 		= SPI_MODE_SLAVE;
	rpi_spi_handle.rpi_spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	/* Disable TI mode */
	rpi_spi_handle.rpi_spi_handle.Init.TIMode 	= SPI_TIMODE_DISABLE;
	/* */
	rpi_spi_handle.rpi_spi_handle.Init.CLKPhase 	= SPI_PHASE_1EDGE;
	rpi_spi_handle.rpi_spi_handle.Init.CLKPolarity= SPI_POLARITY_LOW;
	/* SPI receives and sends 8-bit data packets. */
	rpi_spi_handle.rpi_spi_handle.Init.DataSize 	= SPI_DATASIZE_8BIT;
	/* Set the SPI to use 2 unidirectional lines MISO and MOSI
	 * for reception and transmission of messages.
	 */
	rpi_spi_handle.rpi_spi_handle.Init.Direction 	= SPI_DIRECTION_2LINES;
	/* Select the first bit of the transmission to be the most significant bit */
	rpi_spi_handle.rpi_spi_handle.Init.FirstBit 	= SPI_FIRSTBIT_MSB;
	/* Disable CRC error checking calculation */
	rpi_spi_handle.rpi_spi_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	/* Using software slave select */
	rpi_spi_handle.rpi_spi_handle.Init.NSS 		= SPI_NSS_SOFT;

	rpi_spi_handle.hdma_spi2_tx.Instance 					= DMA1_Stream4;
	rpi_spi_handle.hdma_spi2_tx.Init.Channel  				= DMA_CHANNEL_0;
	rpi_spi_handle.hdma_spi2_tx.Init.Direction 				= DMA_MEMORY_TO_PERIPH;
	rpi_spi_handle.hdma_spi2_tx.Init.FIFOMode 				= DMA_FIFOMODE_DISABLE;
	rpi_spi_handle.hdma_spi2_tx.Init.MemDataAlignment 		= DMA_MDATAALIGN_BYTE;
	rpi_spi_handle.hdma_spi2_tx.Init.MemInc 				= DMA_MINC_ENABLE;
	rpi_spi_handle.hdma_spi2_tx.Init.Mode 					= DMA_CIRCULAR;
	rpi_spi_handle.hdma_spi2_tx.Init.Priority 				= DMA_PRIORITY_HIGH;
	rpi_spi_handle.hdma_spi2_tx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;

	rpi_spi_handle.hdma_spi2_rx.Instance 					= DMA1_Stream3;
	rpi_spi_handle.hdma_spi2_rx.Init.Channel  				= DMA_CHANNEL_0;
	rpi_spi_handle.hdma_spi2_rx.Init.Direction 				= DMA_PERIPH_TO_MEMORY;
	rpi_spi_handle.hdma_spi2_rx.Init.FIFOMode 				= DMA_FIFOMODE_DISABLE;
	rpi_spi_handle.hdma_spi2_rx.Init.MemDataAlignment 		= DMA_MDATAALIGN_BYTE;
	rpi_spi_handle.hdma_spi2_rx.Init.MemInc 				= DMA_MINC_ENABLE;
	rpi_spi_handle.hdma_spi2_rx.Init.Mode 					= DMA_CIRCULAR;
	rpi_spi_handle.hdma_spi2_rx.Init.Priority 				= DMA_PRIORITY_HIGH;
	rpi_spi_handle.hdma_spi2_rx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;

	if (HAL_DMA_Init(&rpi_spi_handle.hdma_spi2_tx) != HAL_OK)
	{
		_Error_Handler();
	}

	if (HAL_DMA_Init(&rpi_spi_handle.hdma_spi2_rx) != HAL_OK)
	{
		_Error_Handler();
	}

	__HAL_LINKDMA(&rpi_spi_handle.rpi_spi_handle, hdmatx, rpi_spi_handle.hdma_spi2_tx);
	__HAL_LINKDMA(&rpi_spi_handle.rpi_spi_handle, hdmarx, rpi_spi_handle.hdma_spi2_rx);

	if (HAL_SPI_Init(&rpi_spi_handle.rpi_spi_handle) != HAL_OK)
	{
		_Error_Handler();
	}

	GPIO_InitTypeDef GPIO_InitStruct;

	/** SPI2 GPIO Configuration
	  * PB12     ------> SPI2_NSS
	  * PB13     ------> SPI2_SCK
	  * PB14 	 ------> SPI2_MISO
	  * PB15 	 ------> SPI2_MOSI
	  */
	GPIO_InitStruct.Pin = 						GPIO_PIN_12;
	GPIO_InitStruct.Mode = 						GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = 						GPIO_PULLUP;
	GPIO_InitStruct.Speed = 					GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = 				GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = 						GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Pull = 						GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(SPI2_IRQn, SPI2_NVIC_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(SPI2_IRQn);
}

void SPI2_IRQHandler(void) {
	HAL_SPI_IRQHandler(&rpi_spi_handle.rpi_spi_handle);
}
