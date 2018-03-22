/**
  ******************************************************************************
  * @file    lime_mdriver.h
  * @author  Azure
  * @version V0.2.0
  * @date    8th-October-2017
  * @brief 	 Ublox GPS parser
  * 		 This parser only works under one condition - the module has been set
  * 		 to send only the UBX-NAV-POSLLH message which contains precisely 35 
  * 		 bytes (this is to make the use of DMA possible which means faster 
  * 		 data tranfer)
  *
  *
  ******************************************************************************
*/

#ifndef LIME_L3GD20_H_
#define LIME_L3GD20_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx.h"
#include <initsys.h>

extern UbloxGPS ublox_8;

#ifdef __cplusplus
  }
#endif

#endif /* LIME_L3GD20_H_ */
