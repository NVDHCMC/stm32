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
#include <stdlib.h>
#include <initsys.h>
#include <lime_mdriver.h>
#include <litve_adc.h>

/* Imported functions --------------------------------------------------------*/
extern void _Error_Handler(void);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define configDIVISOR 		(float) 18000
#define configDIVISOR2 		60000000
#define configDIVISOR3		400
#define configKP 			3//0.5//1.2
#define configKI 			0//1.5//1.8
#define configKD 			19.5//4//8.5//20.5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// Experimental
pos_error_t 					pos_err =
{
		0,
		0,
		0,
		0,
		0,
		0,
		{2, 1, 0, 1, 2},
		{-2, -1, 0, 1, 2}
};

/* MDRIVER timers for encoder reading and PWM signal generation */
#ifdef LIME_MDRIVER_1
MDRIVE_Handles 					MDRIVE1 =
{
	{0},
	{0},
	{0},
	LIME_MDRIVE1_INIT,
	LIME_MDRIVE1_START,
	LIME_MDRIVE1_MOTOR_UPDATE,
	LIME_MOTOR_PID
};
LIME_MPACKET_Type 				SERIAL_PACKAGE1;
#endif

#ifdef LIME_MDRIVER_2
MDRIVE_Handles 					MDRIVE2 =
{
	{0},
	{0},
	{0},
	LIME_MDRIVE2_INIT,
	LIME_MDRIVE2_START,
	LIME_MDRIVE2_MOTOR_UPDATE,
	LIME_MOTOR_PID
};
#endif

/* Exported functions --------------------------------------------------------*/
/**
  * @brief 	LIME_MDRIVE2_START
  * @param
  * @retval LIME_Status
  */

LIME_Status LIME_MOTOR_PID(LIME_MDRIVE_MOTOR_t * current_motor, TIM_HandleTypeDef * PWMTimerHandle)
{
	LIME_Status status = LIME_ERROR;
	if ((abs(current_motor->SpeedError) > configSPD_TOLERANCE))
	{
		current_motor->AccumulatingError += ((float) current_motor->SpeedError)*0.01;
		if (current_motor->AccumulatingError > 4.5)
		{
			current_motor->AccumulatingError -= ((float) current_motor->SpeedError)*0.01;
		}
	}

	current_motor->last_caculated_value = current_motor->SpeedError*configKp + current_motor->AccumulatingError*configKi;
	if (current_motor->last_caculated_value >= 0xffff) current_motor->last_caculated_value = 0xffff;
	if (current_motor->DRVNum == 1)
	{
		DRIVE1_FWD();
		M_RIGHT = (uint16_t) current_motor->last_caculated_value;
	}
	else if (current_motor->DRVNum == 2)
	{
		DRIVE2_FWD();
		M_LEFT 	= (uint16_t) current_motor->last_caculated_value;
	}
	return status;
}

/**
  * @brief 	MDRIVE1_ENCODER_INIT
  * @param
  * @retval LIME_Status
  */
void MDRIVE1_ENCODER_INIT(void)
{
#ifdef LIME_MDRIVER_1
	TIM_Encoder_InitTypeDef 	MDRIVE1_EncoderTimerConfig;
	/*
	 *  CODE BEGINS
	 *  Encoder timer initialization
	 */
	MDRIVE1.EncoderTimerHandle.Instance 			= DRIVER1_ENCODER_TIMER;
	MDRIVE1.EncoderTimerHandle.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
	MDRIVE1.EncoderTimerHandle.Init.CounterMode 	= TIM_COUNTERMODE_UP;
	MDRIVE1.EncoderTimerHandle.Init.Period 			= PULSE_PER_REV;
	MDRIVE1.EncoderTimerHandle.Init.Prescaler		= 0;
	MDRIVE1.EncoderTimerHandle.Channel 				= TIM_CHANNEL_1 | TIM_CHANNEL_2;
	MDRIVE1_EncoderTimerConfig.EncoderMode 			= TIM_ENCODERMODE_TI12;
	MDRIVE1_EncoderTimerConfig.IC1Filter 			= 0x07;
	MDRIVE1_EncoderTimerConfig.IC1Polarity  		= TIM_ICPOLARITY_FALLING;
	MDRIVE1_EncoderTimerConfig.IC1Prescaler 		= TIM_ICPSC_DIV1;
	MDRIVE1_EncoderTimerConfig.IC1Selection 		= TIM_ICSELECTION_DIRECTTI;
	MDRIVE1_EncoderTimerConfig.IC2Filter 			= 0x7;
	MDRIVE1_EncoderTimerConfig.IC2Polarity 			= TIM_ICPOLARITY_FALLING;
	MDRIVE1_EncoderTimerConfig.IC2Prescaler			= TIM_ICPSC_DIV1;
	MDRIVE1_EncoderTimerConfig.IC2Selection 		= TIM_ICSELECTION_DIRECTTI;

	if (HAL_TIM_Encoder_Init(&MDRIVE1.EncoderTimerHandle, &MDRIVE1_EncoderTimerConfig) != HAL_OK)
	{
		_Error_Handler();
	}
	/*
	 * MDRIVER_1 Logic pins init
	 */
	GPIO_InitTypeDef GPIO_Handle;

	GPIO_Handle.Pin 	= DRIVER1_LOGIC_PIN_1 | DRIVER1_LOGIC_PIN_2;
	GPIO_Handle.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_Handle.Pull 	= GPIO_NOPULL;
	GPIO_Handle.Speed 	= GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DRIVER1_LOGIC_PORT, &GPIO_Handle);
	/*
	 * CODE ENDS
	 * Sample timer initialization
	 */
	GPIO_InitTypeDef GPIO_InitStruct;
	/* CODE BEGINS Driver 1 encoder timer Msp_Init */
	if (MDRIVE1.EncoderTimerHandle.Instance == DRIVER1_ENCODER_TIMER)
	{
		GPIO_InitStruct.Pin 	= DRIVER1_ENCODER_PIN1;
		GPIO_InitStruct.Mode 	= GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull 	= GPIO_PULLUP;
		GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_VERY_HIGH;
		// This line also needs to be modified if there is a change in the motor's encoder timer
		GPIO_InitStruct.Alternate = DRIVER1_ENCODER_TIMER_AF;
		HAL_GPIO_Init(DRIVER1_ENCODER_PORT1, &GPIO_InitStruct);

		GPIO_InitStruct.Pin 	= DRIVER1_ENCODER_PIN2;
		HAL_GPIO_Init(DRIVER1_ENCODER_PORT2, &GPIO_InitStruct);
	}

	/* Init encoder's timer entry in NVIC table */
	HAL_NVIC_SetPriority(DRIVER1_ENCODER_NVIC_IRQn, DRIVER1_ENCODER_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(DRIVER1_ENCODER_NVIC_IRQn);
	/* CODE ENDS Driver 1 encoder timer Msp_Init */
#endif
}

/**
  * @brief 	LIME_MDRIVE_INIT
  * @param 	none
  * @retval none
  */
void MDRIVE1_PWM_INIT(void)
{
#ifdef LIME_MDRIVER_1
	/*
	 * Timer configuration
	 */
	TIM_OC_InitTypeDef 	MDRIVE1_PWMTimerOCConfig;

	MDRIVE1.PWMTimerHandle.Instance 			= DRIVER1_STEP_TIMER;
	MDRIVE1.PWMTimerHandle.Init.CounterMode 	= TIM_COUNTERMODE_UP;
	MDRIVE1.PWMTimerHandle.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
	MDRIVE1.PWMTimerHandle.Init.Period 			= 0xffff;
	MDRIVE1.PWMTimerHandle.Init.Prescaler 		= 0;
	MDRIVE1.PWMTimerHandle.Init.RepetitionCounter = 1;

	if (HAL_TIM_PWM_Init(&MDRIVE1.PWMTimerHandle) != HAL_OK)
	{
		_Error_Handler();
	}

	MDRIVE1_PWMTimerOCConfig.OCMode 			= TIM_OCMODE_PWM1;
	MDRIVE1_PWMTimerOCConfig.Pulse 				= 0;
	MDRIVE1_PWMTimerOCConfig.OCPolarity 	 	= TIM_OCPOLARITY_HIGH;
	MDRIVE1_PWMTimerOCConfig.OCFastMode 		= TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&MDRIVE1.PWMTimerHandle, &MDRIVE1_PWMTimerOCConfig, DRIVER1_STEP_CHANNEL) != HAL_OK)
	{
		_Error_Handler();
	}

	/*
	 * Timer MSP Initialization
	 */

	GPIO_InitTypeDef GPIO_Handle;

	GPIO_Handle.Pin		= DRIVER1_STEP;
	GPIO_Handle.Mode 	= GPIO_MODE_AF_PP;
	GPIO_Handle.Pull 	= GPIO_NOPULL;
	GPIO_Handle.Speed 	= GPIO_SPEED_FREQ_VERY_HIGH;
	// In case of custom configuration, change this into appropriate GPIO pin mode.
	GPIO_Handle.Alternate = DRIVER1_STEP_TIMER_AF;
	HAL_GPIO_Init(DRIVER1_STEP_PORT, &GPIO_Handle);
#endif
}

/**
  * @brief 	LIME_MDRIVE1_INIT
  * @param
  * @retval LIME_Status
  */
LIME_Status LIME_MDRIVE1_INIT(void)
{
	uint8_t status = LIME_ERROR;

#ifdef LIME_MDRIVER_1
	/* Init motor struct */
	MDRIVE1.MOTOR.DRVStat 	&= (MDRIVE_StatusType) ~(MOTOR_ENABLE | MOTOR_LAPPED_EVENT | MOTOR_UPDATE_EVENT | MOTOR_RESET);
	MDRIVE1.MOTOR.DRVNum 	= 1;
	MDRIVE1.MOTOR.NumRev	= 0;
	MDRIVE1.MOTOR.SpeedRef  = 0;
	MDRIVE1.MOTOR.SpeedVal  = 0;
	MDRIVE1.MOTOR.SpeedError= 0;
	MDRIVE1.MOTOR.AccumulatingError = 0;
	MDRIVE2.MOTOR.last_caculated_value = 0;
	MDRIVE1.MOTOR.Encoder 	= 0;
	MDRIVE1.MOTOR.SampleFreq= SAMPLE_FREQ_MEDIUM;

	/* Replace system handle and init function with custom ones */
	//initHandles.TIM10_Handle= MDRIVE1.PWMTimerHandle;
	initHandles.EX0_PPP_INIT= MDRIVE1_PWM_INIT;
	//initHandles.TIM3_Handle = MDRIVE1.EncoderTimerHandle;
	initHandles.EX1_PPP_INIT= MDRIVE1_ENCODER_INIT;
	if (LIME_MPACKET_Init(&SERIAL_PACKAGE1) != LIME_OK)
	{
		status = LIME_ERROR;
	}
	status 					= LIME_OK;
#endif
	return status;
}

/**
  * @brief 	LIME_MDRIVE1_MOTOR_UPDATE
  * @param
  * @retval LIME_Status
  */
LIME_Status LIME_MDRIVE1_MOTOR_UPDATE(void)
{
	uint8_t status 			= LIME_ERROR;
#ifdef LIME_MDRIVER_1
	int32_t temp 			= (int32_t) MDRIVE1.EncoderTimerHandle.Instance->CNT;
	MDRIVE1.MOTOR.SpeedVal 	= abs(temp - MDRIVE1.MOTOR.Encoder);
	MDRIVE1.MOTOR.Encoder 	= temp;
	MDRIVE1.MOTOR.SpeedError= MDRIVE1.MOTOR.SpeedRef - MDRIVE1.MOTOR.SpeedVal;
	status 					= LIME_OK;
#endif
	return status;
}

/**
  * @brief 	LIME_MDRIVE1_START
  * @param
  * @retval LIME_Status
  */
LIME_Status LIME_MDRIVE1_START(void)
{
	uint8_t status 			= LIME_ERROR;
#ifdef LIME_MDRIVER_1
	/* Start the encoder pulse counter. */
	HAL_TIM_Base_Start(&MDRIVE1.EncoderTimerHandle);
	HAL_TIM_Encoder_Start(&MDRIVE1.EncoderTimerHandle, TIM_CHANNEL_1 | TIM_CHANNEL_2);

	/* */
	HAL_TIM_Base_Start_IT(&initHandles.TIM4_Handle);

	/* Start the PWM generation. */
	HAL_TIM_PWM_Start(&MDRIVE1.PWMTimerHandle, DRIVER1_STEP_CHANNEL);
	/* Stop the motor. */
	HAL_GPIO_WritePin(DRIVER1_LOGIC_PORT, DRIVER1_LOGIC_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DRIVER1_LOGIC_PORT, DRIVER1_LOGIC_PIN_2, GPIO_PIN_RESET);
	status 					= LIME_OK;
#endif
	return status;
}

/**
  * @brief 	MDRIVE2_ENCODER_INIT
  * @param
  * @retval LIME_Status
  */
void MDRIVE2_ENCODER_INIT(void)
{
#ifdef LIME_MDRIVER_2
	TIM_Encoder_InitTypeDef 	MDRIVE2_EncoderTimerConfig;
	/*
	 *  CODE BEGINS
	 *  Encoder timer initialization
	 */
	MDRIVE2.EncoderTimerHandle.Instance 			= DRIVER2_ENCODER_TIMER;
	MDRIVE2.EncoderTimerHandle.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
	MDRIVE2.EncoderTimerHandle.Init.CounterMode 	= TIM_COUNTERMODE_UP;
	MDRIVE2.EncoderTimerHandle.Init.Period 			= PULSE_PER_REV;
	MDRIVE2.EncoderTimerHandle.Init.Prescaler		= 0;
	MDRIVE2.EncoderTimerHandle.Channel 				= TIM_CHANNEL_1 | TIM_CHANNEL_2;
	MDRIVE2_EncoderTimerConfig.EncoderMode 			= TIM_ENCODERMODE_TI12;
	MDRIVE2_EncoderTimerConfig.IC1Filter 			= 0x07;
	MDRIVE2_EncoderTimerConfig.IC1Polarity  		= TIM_ICPOLARITY_FALLING;
	MDRIVE2_EncoderTimerConfig.IC1Prescaler 		= TIM_ICPSC_DIV1;
	MDRIVE2_EncoderTimerConfig.IC1Selection 		= TIM_ICSELECTION_DIRECTTI;
	MDRIVE2_EncoderTimerConfig.IC2Filter 			= 0x7;
	MDRIVE2_EncoderTimerConfig.IC2Polarity 			= TIM_ICPOLARITY_FALLING;
	MDRIVE2_EncoderTimerConfig.IC2Prescaler			= TIM_ICPSC_DIV1;
	MDRIVE2_EncoderTimerConfig.IC2Selection 		= TIM_ICSELECTION_DIRECTTI;

	if (HAL_TIM_Encoder_Init(&MDRIVE2.EncoderTimerHandle, &MDRIVE2_EncoderTimerConfig) != HAL_OK)
	{
		_Error_Handler();
	}
	/*
	 * MDRIVER_2 Logic pins init
	 */
	GPIO_InitTypeDef GPIO_Handle;

	GPIO_Handle.Pin 	= DRIVER2_LOGIC_PIN_1 | DRIVER2_LOGIC_PIN_2;
	GPIO_Handle.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_Handle.Pull 	= GPIO_NOPULL;
	GPIO_Handle.Speed 	= GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DRIVER2_LOGIC_PORT, &GPIO_Handle);
	/*
	 * CODE ENDS
	 * Sample timer initialization
	 */
	GPIO_InitTypeDef GPIO_InitStruct;
	/* CODE BEGINS Driver 2 encoder timer Msp_Init */
	if (MDRIVE2.EncoderTimerHandle.Instance == DRIVER2_ENCODER_TIMER)
	{
		GPIO_InitStruct.Pin 	= DRIVER2_ENCODER_PIN1;
		GPIO_InitStruct.Mode 	= GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull 	= GPIO_PULLUP;
		GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_VERY_HIGH;
		// This line also needs to be modified if there is a change in the motor's encoder timer
		GPIO_InitStruct.Alternate = DRIVER2_ENCODER_TIMER_AF;
		HAL_GPIO_Init(DRIVER2_ENCODER_PORT1, &GPIO_InitStruct);

		GPIO_InitStruct.Pin 	= DRIVER2_ENCODER_PIN2;
		HAL_GPIO_Init(DRIVER2_ENCODER_PORT2, &GPIO_InitStruct);
	}

	/* Init encoder's timer entry in NVIC table */
	HAL_NVIC_SetPriority(DRIVER2_ENCODER_NVIC_IRQn, DRIVER2_ENCODER_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(DRIVER2_ENCODER_NVIC_IRQn);
	/* CODE ENDS Driver 2 encoder timer Msp_Init */
#endif
}
/**
  * @brief 	LIME_MDRIVE_INIT
  * @param 	none
  * @retval none
  */
void MDRIVE2_PWM_INIT(void)
{
#ifdef LIME_MDRIVER_2
	/*
	 * Timer configuration
	 */
	TIM_OC_InitTypeDef 	MDRIVE2_PWMTimerOCConfig;

	MDRIVE2.PWMTimerHandle.Instance 			= DRIVER2_STEP_TIMER;
	MDRIVE2.PWMTimerHandle.Init.CounterMode 	= TIM_COUNTERMODE_UP;
	MDRIVE2.PWMTimerHandle.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
	MDRIVE2.PWMTimerHandle.Init.Period 			= 0xffff;
	MDRIVE2.PWMTimerHandle.Init.Prescaler 		= 0;
	MDRIVE2.PWMTimerHandle.Init.RepetitionCounter = 1;

	if (HAL_TIM_PWM_Init(&MDRIVE2.PWMTimerHandle) != HAL_OK)
	{
		_Error_Handler();
	}

	MDRIVE2_PWMTimerOCConfig.OCMode 			= TIM_OCMODE_PWM1;
	MDRIVE2_PWMTimerOCConfig.Pulse 				= 0;
	MDRIVE2_PWMTimerOCConfig.OCPolarity 	 	= TIM_OCPOLARITY_HIGH;
	MDRIVE2_PWMTimerOCConfig.OCFastMode 		= TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&MDRIVE2.PWMTimerHandle, &MDRIVE2_PWMTimerOCConfig, DRIVER2_STEP_CHANNEL) != HAL_OK)
	{
		_Error_Handler();
	}

	/*
	 * Timer MSP Initialization
	 */

	GPIO_InitTypeDef GPIO_Handle;

	GPIO_Handle.Pin		= DRIVER2_STEP;
	GPIO_Handle.Mode 	= GPIO_MODE_AF_PP;
	GPIO_Handle.Pull 	= GPIO_NOPULL;
	GPIO_Handle.Speed 	= GPIO_SPEED_FREQ_VERY_HIGH;
	// In case of custom configuration, change this into appropriate GPIO pin mode.
	GPIO_Handle.Alternate = DRIVER2_STEP_TIMER_AF;
	HAL_GPIO_Init(DRIVER2_STEP_PORT, &GPIO_Handle);
#endif
}

/**
  * @brief 	LIME_MDRIVE2_INIT
  * @param  none
  * @retval LIME_Status
  */
LIME_Status LIME_MDRIVE2_INIT(void)
{
	uint8_t status 			= LIME_ERROR;
#ifdef LIME_MDRIVER_2
	/* Init motor struct */
	MDRIVE2.MOTOR.DRVStat 	&= (MDRIVE_StatusType) 0;
	MDRIVE2.MOTOR.DRVNum 	= 2;
	MDRIVE2.MOTOR.NumRev	= 0;
	MDRIVE2.MOTOR.SpeedRef  = 0;
	MDRIVE2.MOTOR.SpeedVal  = 0;
	MDRIVE2.MOTOR.SpeedError= 0;
	MDRIVE2.MOTOR.AccumulatingError = 0;
	MDRIVE2.MOTOR.last_caculated_value = 0;
	MDRIVE2.MOTOR.Encoder 	= 0;

	/* Replace system handle and init function with custom ones */
	//initHandles.TIM10_Handle= MDRIVE1.PWMTimerHandle;
	initHandles.EX2_PPP_INIT= MDRIVE2_PWM_INIT;
	//initHandles.TIM3_Handle = MDRIVE1.EncoderTimerHandle;
	initHandles.EX3_PPP_INIT= MDRIVE2_ENCODER_INIT;
	status 					= LIME_OK;
#endif
	return status;
}

/**
  * @brief 	LIME_MDRIVE_MOTOR_UPDATE
  * @param
  * @retval LIME_Status
  */
LIME_Status LIME_MDRIVE2_MOTOR_UPDATE(void)
{
	uint8_t status 			= LIME_ERROR;
#ifdef LIME_MDRIVER_2
	uint32_t temp 			= MDRIVE2.EncoderTimerHandle.Instance->CNT;
	temp 					= (int32_t) temp;
	MDRIVE1.MOTOR.SpeedVal 	= temp - MDRIVE1.MOTOR.Encoder;
	MDRIVE2.MOTOR.Encoder 	= temp;

	MDRIVE2.MOTOR.SpeedError= MDRIVE2.MOTOR.SpeedRef - MDRIVE2.MOTOR.SpeedVal;
	status 					= LIME_OK;
#endif
	return status;
}

/**
  * @brief 	LIME_MDRIVE2_START
  * @param
  * @retval LIME_Status
  */
LIME_Status LIME_MDRIVE2_START(void)
{
	uint8_t status 			= LIME_ERROR;
#ifdef LIME_MDRIVER_2
	/* Start the encoder pulse counter. */
	HAL_TIM_Base_Start(&MDRIVE2.EncoderTimerHandle);
	HAL_TIM_Encoder_Start(&MDRIVE2.EncoderTimerHandle, TIM_CHANNEL_1 | TIM_CHANNEL_2);

	/* Start the PWM generation. */
	HAL_TIM_PWM_Start(&MDRIVE2.PWMTimerHandle, DRIVER2_STEP_CHANNEL);
	/* Stop the motor. */
	HAL_GPIO_WritePin(DRIVER2_LOGIC_PORT, DRIVER2_LOGIC_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DRIVER2_LOGIC_PORT, DRIVER2_LOGIC_PIN_2, GPIO_PIN_RESET);
	status 					= LIME_OK;
#endif
	return status;
}

// Temporal function
void start_button_init(void)
{
	GPIO_InitTypeDef GPIO_Handle;

	GPIO_Handle.Pin 		= GPIO_PIN_0;
	GPIO_Handle.Mode 		= GPIO_MODE_IT_RISING;
	GPIO_Handle.Pull 		= GPIO_PULLDOWN;
	GPIO_Handle.Speed 		= GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOB, &GPIO_Handle);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 8, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

// Experimental
void calc_error(pos_error_t * pos_err)
{
	pos_err->torque = 0;
	pos_err->total_mass = 0;
	IRLF_ADC.ESTIMATE_LINE_CENTER();
	pos_err->error_rate = pos_err->new_error - pos_err->old_error;
	pos_err->old_error = pos_err->new_error;
}
float p = 1;
void calc_velocity(pos_error_t * pos_err, uint32_t max_speed)
{
	float k = configKP*pos_err->new_error + configKI*pos_err->accumulative_error + configKD*pos_err->error_rate;

	  if (pos_err->new_error > 0)
	  {
		  if (k < 0) k = 0;
	    p = 1 + k;
	    //p = p*k;
	    M_RIGHT= (uint16_t) ((2*(float) max_speed)/(p + 1));
	    M_LEFT = (max_speed << 1) - M_RIGHT;
	  }
	  else if (pos_err->new_error < 0)
	  {
		  if (k > 0) k = 0;
	    p = 1 - k;
	    //p = p*k;
	    M_LEFT= (uint16_t) ((2*(float) max_speed)/(p + 1));
	    M_RIGHT = (max_speed << 1) - M_LEFT;
	  }
	  else
	  {
	    M_LEFT = max_speed;
	    M_RIGHT = max_speed;
	  }
}
