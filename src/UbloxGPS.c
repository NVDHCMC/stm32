/**
  ******************************************************************************
  * @file    UbloxGPS.c
  * @author  Azure
  * @version V0.2.0
  * @date    19-March-2018
  * @brief   Ublox GPS parser
  * 		 This parser only works under one condition - the module has been set
  * 		 to send only the UBX-NAV-POSLLH message which contains precisely 35 
  * 		 bytes (this is to make the use of DMA possible which means faster 
  * 		 data tranfer)
  *
  *
  ******************************************************************************
***/

/* Includes ------------------------------------------------------------------*/
#include <UbloxGPS.h>

UbloxGPS ublox_8;

typedef struct 
{
	UART_HandleTypeDef uart_handle;
	char  raw_message[35];
	int  (*init)(void);
	int  (*start_dma)(void);
	void (*parse_message)(void);
} Parser;

typedef struct
{
	Parser parser;
	char   latitude[4];
	char   longtitude[4];
	char   altitude[4];
	char   iTOW[4];
} UbloxGPS;

/**
  * @brief 	init: Initialized communication hardware with Ublox (USART and DMA stream to memory)
  * @param 	none
  * @retval 0 - failed
  * 		1 - successful
  *
  * @notice 
  */
static int init(void) {
	int rv = 1;

	if (ublox_8.parser.uart_handle == 0) { // Check for system default initialization
		rv = 0;
	}
	else {
		// Init DMA
		
	}

	return rv;
}

/**
  * @brief 	start_dma: Start DMA stream from peripheral to memory with preallocated memory space
  * @param 	none
  * @retval 0 - failed
  * 		1 - successful
  *
  * @notice 
  */
static int start_dma(void) {
	
}

/**
  * @brief 	parse_message: Parse received message
  * @param 	none
  * @retval 0 - failed
  * 		1 - successful
  *
  * @notice 
  */
static void parse_message(void) {
	
}

// Initialize UbloxGPS structure
ublox_8 = {
	{
		inithandles.USART1_Handle, 		// USART handle
		{0x00}, 	// raw_message
		init, 		// init function
		start_dma,  // start dma stream
		parse_message
	}, 				// Parser
	{0x00}, 		// lat
	{0x00}, 		// long
	{0x00},			// altitude
	{0x00} 			// iTOW or timestamp
}