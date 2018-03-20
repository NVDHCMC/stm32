/**
  ******************************************************************************
  * @file    litve_adc.h
  * @author  Azure
  * @version V0.1.0
  * @date    2nd-November-2017
  * @brief 	 Configuration file for Line-tracking vehicle ADC data acquisition.
  *
  *
  ******************************************************************************
*/

#ifndef INCLUDE_LITVE_ADC_H_
#define INCLUDE_LITVE_ADC_H_

/* Private typedef -----------------------------------------------------------*/
typedef uint16_t IRLF_ADC_ValueBaseType;

/* Private define ------------------------------------------------------------*/
#define configUSE_LITVE_LIB
#define configNUM_OF_CHANNELs 			(uint32_t) 7
#define S_a 				IRLF_ADC.ADC_channel_value[0]
#define S_0					IRLF_ADC.ADC_channel_value[1]
#define S_1					IRLF_ADC.ADC_channel_value[2]
#define S_2					IRLF_ADC.ADC_channel_value[3]
#define S_3					IRLF_ADC.ADC_channel_value[4]
#define S_4					IRLF_ADC.ADC_channel_value[5]
#define S_b 				IRLF_ADC.ADC_channel_value[6]

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** @defgroup 	IRLF_ADC_Handle structure
  * @{
  */
typedef struct
{
	ADC_HandleTypeDef					ADC_Handle;
	DMA_HandleTypeDef 					DMA_Handle;
	IRLF_ADC_ValueBaseType				*ADC_channel_value;
	IRLF_ADC_ValueBaseType 				sensor_value_max[configNUM_OF_CHANNELs];
	IRLF_ADC_ValueBaseType				sensor_value_min[configNUM_OF_CHANNELs];
	float				 				sensor_coefficients[configNUM_OF_CHANNELs];
	IRLF_ADC_ValueBaseType 				sensor_calibrated_value[configNUM_OF_CHANNELs];
	void 								(*ADC_INIT)(void);
	void 								(*DMA_INIT)(void);
	void 								(*ROBOT_SENSOR_CALIB)(void);
	void 								(*CALIBRATE_SENSOR_VALUE)(void);
	void 								(*ESTIMATE_LINE_CENTER)(void);
} IRLF_ADC_HandleTypeDef;

/* Exported variables ---------------------------------------------------------*/
extern IRLF_ADC_HandleTypeDef 			IRLF_ADC;

/* Exported function prototypes ----------------------------------------------*/
void 					IRLF_ADC_PERIPH_INIT(void);
void 					IRLF_DMA_INIT(void);
void 					IRLF_SENSOR_CALIBRATION(void);
void 					IRLF_CALIBRATE(void);
void 					IRLF_ESTIMATE_LINE_CENTER(void);

#endif /* INCLUDE_LITVE_ADC_H_ */
