/**
  ******************************************************************************
  * @file    litve_adc.c
  * @author  Azure
  * @version V0.1.0
  * @date    2nd-November-2017
  * @brief 	 Source code of Line-tracking vehicle ADC data acquisition.
  *
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <cmsis_os.h>
#include <stdlib.h>
#include <initsys.h>
#include <lime_mdriver.h>
#include <litve_adc.h>

extern void _Error_Handler(void);
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
IRLF_ADC_HandleTypeDef 		IRLF_ADC = {
		{0},
		{0},
		NULL,
		{0},
		{344, 216, 212, 240, 240, 228, 124},
		//{719, 278, 278, 306, 256, 264, 124},
		//{4000, 4000, 4000, 4000, 4000, 4000, 4000},
		{1.28320801, 1.29089189, 1.33463669, 1.26732671, 1.44632769, 1.28080046, 1.03147817},
		//{1.52380955, 1.46024954, 1.44428778, 1.35944247, 1.51479292, 1.41338849, 1.03147817},
		{0},
		IRLF_ADC_PERIPH_INIT,
		IRLF_DMA_INIT,
		IRLF_SENSOR_CALIBRATION,
		IRLF_CALIBRATE,
		IRLF_ESTIMATE_LINE_CENTER
};
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  IRLF_ADC_PERIPH_INIT: Init ADC peripheral
  * @param 	none
  * @retval none
  *
  * @notice unusable if not first initialized main data bus clock
  */
void IRLF_ADC_PERIPH_INIT(void)
{
	ADC_ChannelConfTypeDef sConfig;

	/* Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
	IRLF_ADC.ADC_Handle.Instance 					= ADC1;
	IRLF_ADC.ADC_Handle.Init.ClockPrescaler 		= ADC_CLOCK_SYNC_PCLK_DIV2;
	IRLF_ADC.ADC_Handle.Init.Resolution 			= ADC_RESOLUTION_12B;
	IRLF_ADC.ADC_Handle.Init.ScanConvMode 			= ENABLE;
	IRLF_ADC.ADC_Handle.Init.ContinuousConvMode 	= ENABLE;
	IRLF_ADC.ADC_Handle.Init.DiscontinuousConvMode 	= DISABLE;
	IRLF_ADC.ADC_Handle.Init.ExternalTrigConvEdge 	= ADC_EXTERNALTRIGCONVEDGE_NONE;
	IRLF_ADC.ADC_Handle.Init.ExternalTrigConv 		= ADC_SOFTWARE_START;
	IRLF_ADC.ADC_Handle.Init.DataAlign 				= ADC_DATAALIGN_RIGHT;
	IRLF_ADC.ADC_Handle.Init.NbrOfConversion 		= configNUM_OF_CHANNELs;
	IRLF_ADC.ADC_Handle.Init.DMAContinuousRequests 	= ENABLE;
	IRLF_ADC.ADC_Handle.Init.EOCSelection 			= ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&IRLF_ADC.ADC_Handle) != HAL_OK)
	{
		_Error_Handler();
	}

	/* Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
	sConfig.Channel 		= ADC_CHANNEL_8;
	sConfig.Rank 			= 1;
	sConfig.SamplingTime 	= ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&IRLF_ADC.ADC_Handle, &sConfig) != HAL_OK)
	{
		_Error_Handler();
	}

	sConfig.Channel 		= ADC_CHANNEL_9;
	sConfig.Rank 			= 2;
	if (HAL_ADC_ConfigChannel(&IRLF_ADC.ADC_Handle, &sConfig) != HAL_OK)
	{
		_Error_Handler();
	}

	sConfig.Channel 		= ADC_CHANNEL_11;
	sConfig.Rank 			= 3;
	if (HAL_ADC_ConfigChannel(&IRLF_ADC.ADC_Handle, &sConfig) != HAL_OK)
	{
		_Error_Handler();
	}

	sConfig.Channel 		= ADC_CHANNEL_12;
	sConfig.Rank 			= 4;
	if (HAL_ADC_ConfigChannel(&IRLF_ADC.ADC_Handle, &sConfig) != HAL_OK)
	{
		_Error_Handler();
	}

	sConfig.Channel 		= ADC_CHANNEL_13;
	sConfig.Rank 			= 5;
	if (HAL_ADC_ConfigChannel(&IRLF_ADC.ADC_Handle, &sConfig) != HAL_OK)
	{
		_Error_Handler();
	}

	sConfig.Channel 		= ADC_CHANNEL_14;
	sConfig.Rank 			= 6;
	if (HAL_ADC_ConfigChannel(&IRLF_ADC.ADC_Handle, &sConfig) != HAL_OK)
	{
		_Error_Handler();
	}

	sConfig.Channel 		= ADC_CHANNEL_15;
	sConfig.Rank 			= 7;
	sConfig.SamplingTime 	= ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&IRLF_ADC.ADC_Handle, &sConfig) != HAL_OK)
	{
		_Error_Handler();
	}

	/**	ADC1 GPIO Configuration
	  * PB0     ------> ADC1_8
	  * PB1 	------> ADC1_9
	  *	PC1     ------> ADC1_11
	  *	PC2 	------> ADC1_12
	  *	PC3 	------> ADC1_13
	  *	PC4 	------> ADC1_14
	  *	PC5 	------> ADC1_15
	  */
	GPIO_InitTypeDef 		GPIO_Handle;

	GPIO_Handle.Pin 		= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
							  GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_Handle.Mode 		= GPIO_MODE_ANALOG;
	GPIO_Handle.Pull 		= GPIO_NOPULL;
	GPIO_Handle.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(GPIOC, &GPIO_Handle);

	GPIO_Handle.Pin 		= GPIO_PIN_0 | GPIO_PIN_1;

	HAL_GPIO_Init(GPIOB, &GPIO_Handle);

	/* Allocating memory slots for ADC channel data*/
#if !defined(USE_FREERTOS)
	IRLF_ADC.ADC_channel_value 		= 	malloc(sizeof(IRLF_ADC_ValueBaseType)*configNUM_OF_CHANNELs);
#else
	IRLF_ADC.ADC_channel_value 		=	pvPortMalloc(sizeof(IRLF_ADC_ValueBaseType)*configNUM_OF_CHANNELs);
#endif
}

void IRLF_DMA_INIT(void)
{
	IRLF_ADC.DMA_Handle.Instance 				= DMA2_Stream0;
	IRLF_ADC.DMA_Handle.Init.Channel 			= DMA_CHANNEL_0;
	IRLF_ADC.DMA_Handle.Init.Direction 			= DMA_PERIPH_TO_MEMORY;
	IRLF_ADC.DMA_Handle.Init.PeriphInc 			= DMA_PINC_DISABLE;
	IRLF_ADC.DMA_Handle.Init.MemInc 			= DMA_MINC_ENABLE;
	IRLF_ADC.DMA_Handle.Init.PeriphDataAlignment= DMA_PDATAALIGN_HALFWORD;
	IRLF_ADC.DMA_Handle.Init.MemDataAlignment 	= DMA_MDATAALIGN_HALFWORD;
	IRLF_ADC.DMA_Handle.Init.Mode 				= DMA_CIRCULAR;
	IRLF_ADC.DMA_Handle.Init.Priority 			= DMA_PRIORITY_HIGH;
	IRLF_ADC.DMA_Handle.Init.FIFOMode 			= DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&IRLF_ADC.DMA_Handle) != HAL_OK)
	{
		_Error_Handler();
	}

	__HAL_LINKDMA(&IRLF_ADC.ADC_Handle,DMA_Handle,IRLF_ADC.DMA_Handle);
}
/**
  * Calibration of sensor input
  */
void IRLF_CALIBRATE(void)
{
	for (int i = 0; i < configNUM_OF_CHANNELs; i++)
	{
		if (IRLF_ADC.ADC_channel_value[i] > IRLF_ADC.sensor_value_min[i])
		{
			IRLF_ADC.sensor_calibrated_value[i] = (IRLF_ADC.ADC_channel_value[i] - IRLF_ADC.sensor_value_min[i])*IRLF_ADC.sensor_coefficients[i];
		}
		else
		{
			IRLF_ADC.sensor_calibrated_value[i] = 0;
		}
	}
}

/**
  * Calibration code
  */
void IRLF_SENSOR_CALIBRATION(void)
{
	// Start of the calibration
	M_LEFT = 0x3000;
	M_RIGHT = 0x3000;

	HAL_ADC_Start_DMA(&IRLF_ADC.ADC_Handle, (uint32_t *)IRLF_ADC.ADC_channel_value, configNUM_OF_CHANNELs);

	osDelay(50);

	while (MDRIVE2.MOTOR.Encoder < 6200)
	{
		for (int i = 0; i < configNUM_OF_CHANNELs; i++)
		{
			if (IRLF_ADC.sensor_value_max[i] < IRLF_ADC.ADC_channel_value[i])
			{
				IRLF_ADC.sensor_value_max[i] = IRLF_ADC.ADC_channel_value[i];
			}
			if (IRLF_ADC.sensor_value_min[i] > IRLF_ADC.ADC_channel_value[i])
			{
				IRLF_ADC.sensor_value_min[i] = IRLF_ADC.ADC_channel_value[i];
			}
		}
		osDelay(1);
		DRIVE1_BWD();
		DRIVE2_FWD();
	}

	for (int i = 0; i < configNUM_OF_CHANNELs; i++)
	{
		IRLF_ADC.sensor_coefficients[i] = 4096/((float) (IRLF_ADC.sensor_value_max[i] - IRLF_ADC.sensor_value_min[i]));
	}

	// Exception flag
	int flag = 0;

	// Repositioning
	while (!flag)
	{
		IRLF_CALIBRATE();
		if ((IRLF_ADC.sensor_calibrated_value[3] > 2048) && (IRLF_ADC.sensor_calibrated_value[2] < 2048) && (IRLF_ADC.sensor_calibrated_value[4] < 2048))
		{
			M_LEFT = 0;
			M_RIGHT = 0;
			DRIVE1_STOP();
			DRIVE2_STOP();
			flag = 1;
		}
	}
	HAL_ADC_Stop_DMA(&IRLF_ADC.ADC_Handle);
}

void IRLF_ESTIMATE_LINE_CENTER(void)
{
	// Weighted sum
	IRLF_ADC.CALIBRATE_SENSOR_VALUE();
	for (int i = 0; i < 5; i++)
	{
		//pos_err.torque += pos_err.adaptive_weight[i]*IRLF_ADC.ADC_channel_value[i + 2];
		//pos_err.total_mass += IRLF_ADC.ADC_channel_value[i + 2];
		pos_err.torque += pos_err.adaptive_weight[i]*IRLF_ADC.sensor_calibrated_value[i + 1];
		pos_err.total_mass += IRLF_ADC.sensor_calibrated_value[i + 1];
	}

	pos_err.new_error = pos_err.torque/pos_err.total_mass;
}

// Prerequisite: FreeRTOS or any other RTOS system enable to be able to estimate and control the motor at the same time.
void IRLF_CALCULATE_TRACKING_ERROR(void)
{
	IRLF_ADC.ESTIMATE_LINE_CENTER();
	pos_err.old_error = pos_err.new_error;
	osDelay(2);
	IRLF_ADC.ESTIMATE_LINE_CENTER();
}
