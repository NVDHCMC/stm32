/**
  ******************************************************************************
  * @file    initsys.h
  * @author  Azure
  * @version V0.2.0
  * @date    8th-October-2017
  * @brief 	 Main init handle structure.
  *
  *
  ******************************************************************************
*/

#ifndef RASPI_SPI_H_
#define RASPI_SPI_H_

#define config_TIMEOUT 5
#define config_buf_length 128

/* Private typedef -----------------------------------------------------------*/
typedef uint8_t timeout_t;
typedef unsigned char buffer_t;
typedef uint8_t err_code_t;
typedef uint32_t signal_t;

typedef struct
{
	SPI_HandleTypeDef rpi_spi_handle;
	DMA_HandleTypeDef hdma_spi2_tx;
	DMA_HandleTypeDef hdma_spi2_rx;

	timeout_t 	timeout;
	buffer_t 	tx_buffer[config_buf_length];
	buffer_t 	rx_buffer[config_buf_length];
	err_code_t 	error_code;
	buffer_t	data[20];
	buffer_t 	sync_mess[10];
	buffer_t	pair_mess[10];
	buffer_t    id_mess[10];
	buffer_t 	dummy_mess[255];
	void 		(*wdt_update)(void);
	void 		(*get_data)(uint8_t *);
} raspi_com_t;

typedef struct
{
	signal_t 	spi_complete_signal;
} raspi_signal_t;

/* Private define ------------------------------------------------------------*/
/** @defgroup 	raspi spi opcode
  * @{
  */
#define SPI_SYNC 		0xff
#define SPI_DATA 		0x80
#define SPI_CONF 		0xa0
#define SPI_STAT 		0xb0
#define SPI_END 		0xf0
/**
  * @}
  */

/** @defgroup spi_data registers
  * @{
  */
#define SPI_DATA_ACCEL 	0x01
#define SPI_DATA_GYRO 	0x02
#define SPI_DATA_MAGNET 0x03
/**
  * @}
  */

/* Exported variables -------------------------------------------------------*/
extern raspi_signal_t rpi_signals;

/* Exported function --------------------------------------------------------*/
void raspi_spi_task();
void raspi_spi_init();
void raspi_spi_updatewdt();
void raspi_spi_get_data(uint8_t * data);

void SPI2_IRQHandler(void);

#endif /* RASPI_SPI_H_ */
