/**
  ******************************************************************************
  * @file    lime_mdriver.c
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
#include <cmsis_os.h>
#include <stdlib.h>
#include <initsys.h>
#include <lime_mpacket.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
LIME_MPACKET_Signals_Type 	limeSignalList =
{
	1,
	0,
	1
};

/* Exported functions --------------------------------------------------------*/
/**
  * @brief
  * @param 	Pointer to LIME_MPACKET_Type
  * @retval LIME_Status
  *
  */
LIME_Status LIME_MPACKET_Init(LIME_MPACKET_Type * limePacket)
{
	uint8_t status = LIME_OK;
#if !defined(USE_FREERTOS)
	limePacket->MesID 		= malloc(sizeof(limePacket->MesID));
	limePacket->MesContent  = malloc(sizeof(uint8_t)*configLIME_MPACKET_MAXSIZE);
	limePacket->mes_len 		= 0;//(uint8_t) (configLIME_MPACKET_MAXSIZE + configLIME_MPACKET_HEADERS);
	limePacket->MesComposed = malloc(sizeof(uint8_t)*(configLIME_MPACKET_MAXSIZE + configLIME_MPACKET_HEADERS));
	limePacket->MesReceived = malloc(sizeof(uint8_t)*configLIME_MPACKET_RECEIVE_SIZE);

	if ((limePacket->MesID == NULL) || (limePacket->MesContent == NULL) || (limePacket->MesComposed == NULL))
	{
		status 				= LIME_ERROR;
	}
#else
	limePacket->MesID 		= pvPortMalloc(sizeof(limePacket->MesID));
	limePacket->MesContent  = pvPortMalloc(sizeof(uint8_t)*configLIME_MPACKET_MAXSIZE);
	limePacket->mes_len 		= 0;
	limePacket->MesComposed = pvPortMalloc(sizeof(uint8_t)*(configLIME_MPACKET_MAXSIZE + configLIME_MPACKET_HEADERS));
	limePacket->MesReceived = pvPortMalloc(sizeof(uint8_t)*configLIME_MPACKET_RECEIVE_SIZE);

	if ((limePacket->MesID == NULL) || (limePacket->MesContent == NULL) || (limePacket->MesComposed == NULL))
	{
		status 				= LIME_ERROR;
	}
#endif
	return status;
}

/**
  * @brief
  * @param 	Pointer to LIME_MPACKET_Type
  * @retval LIME_Status
  *
  */
LIME_Status LIME_MPACKET_Publish(LIME_MPACKET_Type *limePacket, initHandle_struct *initHandles)
{
	uint8_t status = LIME_OK;

	/* Send message packet */
	HAL_UART_Transmit_IT(configSERIAL_COM_PERIPH, limePacket->MesContent, limePacket->mes_len);
	osSignalWait(limeSignalList.sFINISHED_PUBLISHING, configMAX_PUBLISH_WAIT);
	return status;
}

/**
  * @brief
  * @param 	Pointer to LIME_MPACKET_Type
  * @retval LIME_Status
  *
  */
LIME_Status LIME_MPACKET_Subcribe(LIME_MPACKET_Type *limePacket, initHandle_struct *initHandles)
{
	uint8_t status = LIME_OK;

	return status;
}

/**
  * @brief
  * @param 	Pointer to LIME_MPACKET_Type
  * @retval LIME_Status
  *
  */
LIME_Status LIME_MPACKET_AddField(char* field_name, uint32_t value, LIME_MPACKET_Type *limePacket)
{
	uint8_t status = LIME_OK;
	/* Temporary values */
	uint16_t field_len 			= strlen(field_name);
	uint8_t * temp_string 		= limePacket->MesContent;
	uint8_t current_pos 		= limePacket->mes_len;
	uint8_t temp_value 			= 0;

	for (uint16_t i = 0; i < field_len; i++)
	{
		*(temp_string + current_pos) 	= *(field_name + i);
		current_pos++;
	}

	*(temp_string + current_pos) = ':';
	current_pos++;

	for (int i = 3; i >= 0; i--)
	{
		temp_value 						= (uint8_t) (value >> (i*8));
		*(temp_string + current_pos) 	= temp_value;
		current_pos++;
	}

	*(temp_string + current_pos) = 13;
	current_pos++;
	limePacket->mes_len = current_pos;
	return status;
}
















