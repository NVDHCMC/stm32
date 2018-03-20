/**
  ******************************************************************************
  * @file    lime_mdriver.h
  * @author  Azure
  * @version V0.2.0
  * @date    8th-October-2017
  * @brief 	 FreeRTOS compatible motor driver.
  *
  *
  ******************************************************************************
*/

#ifndef LIME_MDRIVER_H_
#define LIME_MDRIVER_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Peripheral devices ---------------------------------- */
#ifndef LIME_MPACKET_C_
#include <lime_mpacket.h>
#endif

/* Motor configuration --------------------------------- */
#define M_RIGHT  			MDRIVE1.PWMTimerHandle.Instance->CCR1
#define M_LEFT				MDRIVE2.PWMTimerHandle.Instance->CCR2
#ifndef LIME_MDRIVER_1
 /* Uncomment the line below to activate MDRIVER #1 */
#define LIME_MDRIVER_1 			1
#endif

#ifndef LIME_MDRIVER_2
/* Uncomment the line below to activate MDRIVER #2 */
#define LIME_MDRIVER_2 		2
#endif

/* Exported types -------------------------------------- */
/* Basic driver types */
typedef uint32_t 				SPDREF_Type;
typedef int32_t 				ENCODER_Type;
typedef uint32_t 				MDRIVE_BaseType;
typedef uint8_t 				MDRIVE_StatusType;

/** @defgroup MOTOR_Controller_Structure
  * @{
  */
typedef struct
{
	SPDREF_Type 		SpeedRef; 	/*!< Desired speed of the motor. This value is
	 	 	 	 	 	 	 	 	 	 obtained via serial com port message or can be manually set. */

	MDRIVE_BaseType 	SampleFreq; /*!< Timer sampling frequency of the encoder value from encoder
	 	 	 	 	 	 	 	 	 	 timer counter register. This can be any value of @ref SAMPLE_Freq_Selection */

	ENCODER_Type 		Encoder;	/*!< Encoder value is read into this variable */


	MDRIVE_BaseType 	MResolution; /*!< Encoder resolution to compute speed and number of revolutions */


	ENCODER_Type	 	SpeedVal;	/*!< Speed value from input compare timer */


	ENCODER_Type	 	SpeedError; /*!< Speed error value */


	float			 	AccumulatingError; /*!< Accumulating error */


	float 				last_caculated_value; /*!< Last calculated pwm value */


	MDRIVE_BaseType 	NumRev;		/*!< Number of completed revolutions */


	MDRIVE_StatusType 	DRVNum; 	/*!< LIME_MDRIVER number */


	MDRIVE_StatusType 	DRVStat;	/*!< LIME_MDRIVER status register. The list of status register constant
	 	 	 	 	 	 	 	 	 	 can be found at @ref LIME_MDRIVER_Status_Constants */

} LIME_MDRIVE_MOTOR_t;

/**
  * @}
  */

/** @defgroup MOTOR_STATUS_Type
  * @{
  */
typedef enum
{
	MOTOR_OK 	= 0,
	MOTOR_ERROR
} LIME_MDRIVER_Status;
/**
  * @}
  */

/** @defgroup MDRIVE_Handles
  * @{
  */
typedef struct
{
	TIM_HandleTypeDef 			EncoderTimerHandle;
	TIM_HandleTypeDef 			PWMTimerHandle;
	LIME_MDRIVE_MOTOR_t 		MOTOR;
	LIME_Status 				(*MOTOR_INIT)(void);
	LIME_Status 				(*MOTOR_START)(void);
	LIME_Status 				(*MOTOR_UPDATE)(void);
	LIME_Status 				(*MOTOR_PID)(LIME_MDRIVE_MOTOR_t *, TIM_HandleTypeDef *);
} MDRIVE_Handles;
/**
  * @}
  */

/* @brief 	Encoder technical specifications
 * @info	+ Timer clock frequency
 * 			+ Sampling frequency
 * 			+ Timer period
 * 			+ Motor pulse per revolution
 */
#define PERIPH_CLOCK_FREQ 			100000000
#define PULSE_PER_REV 				0xffffffff
#define configSPD_TOLERANCE 		1
#define configKp 					(uint16_t) 771
#define configKi 					(uint16_t) 14337
//#define configKd 					10;

/** @defgroup SAMPLE_Freq_Selection
  * @{
  */
#define SAMPLE_FREQ_SLOW 			((MDRIVE_BaseType) 50)
#define SAMPLE_FREQ_MEDIUM 			((MDRIVE_BaseType) 100)
#define SAMPLE_FREQ_HIGH 			((MDRIVE_BaseType) 500)
#define SAMPLE_FREQ_VERY_HIGH		((MDRIVE_BaseType) 1000)
/**
  * @}
  */

/* @defgroup 	LIME_MDRIVER_Status_Constants
 * @info 		+ enable bit enables the motor controller.
 * 				+ update bit captures overflow event.
 * 				+ lapped bit indicates that the motor has completed a revolution.
 * 				+ reset bit resets the counter and set the number of revolution to zero.
 *
 * @notice		!!! Need update.
 * @{
 */
#define MOTOR_ENABLE 				0x01
#define MOTOR_UPDATE_EVENT 			0x02
#define MOTOR_LAPPED_EVENT 			0x04
#define MOTOR_RESET 				0x08
#define MOTOR_SPEED_UPDATE 			0x10
#define MOTOR_SPD_RECV 				0x20
/**
  * @}
  */

/* Define the port contains the logic pins and step pin of the driver
 * DRIVER 1
 * DRIVER 2
 */
/* LIME_MDRIVER_1 -------------------------------------- */
#ifdef LIME_MDRIVER_1
#define DRIVER1_SAMPLE_TIMER 		TIM2
#define DRIVER1_SAMPLE_PORT 		GPIOA
#define DRIVER1_SAMPLE_CHANNELS 	(GPIO_PIN_0 | GPIO_PIN_1)

/** @defgroup LIME_MDRIVER_1_Encoder_pins
  * @{
  */
#define DRIVER1_ENCODER_TIMER 		TIM5
#define DRIVER1_ENCODER_NVIC_IRQn 	TIM5_IRQn
#define DRIVER1_ENCODER_PRIORITY	((uint32_t) 7)
#define DRIVER1_ENCODER_PORT1 		GPIOA
#define DRIVER1_ENCODER_PIN1 		GPIO_PIN_0
#define DRIVER1_ENCODER_PORT2 		GPIOA
#define DRIVER1_ENCODER_PIN2 		GPIO_PIN_1
#define DRIVER1_ENCODER_TIMER_AF 	GPIO_AF2_TIM5
/**
  * @}
  */

/** @defgroup LIME_MDRIVER_1_Dir_pins
  * @{
  */
#define DRIVER1_LOGIC_PORT 			GPIOB
#define DRIVER1_LOGIC_PIN_1 		GPIO_PIN_14
#define DRIVER1_LOGIC_PIN_2 		GPIO_PIN_13
/**
  * @}
  */

/** @defgroup LIME_MDRIVER_1_PWM_pins
  * @{
  */
#define DRIVER1_STEP_TIMER 			TIM3
#define DRIVER1_STEP_CHANNEL 		TIM_CHANNEL_1
#define DRIVER1_STEP_PORT 			GPIOA
#define DRIVER1_STEP 				GPIO_PIN_6
#define DRIVER1_STEP_TIMER_AF 		GPIO_AF2_TIM3
/**
  * @}
  */

/* Exported macros -----------------------------------------------------*/
#define DRIVE1_FWD() 				do {\
										HAL_GPIO_WritePin(DRIVER1_LOGIC_PORT, DRIVER1_LOGIC_PIN_1, GPIO_PIN_SET); \
										HAL_GPIO_WritePin(DRIVER1_LOGIC_PORT, DRIVER1_LOGIC_PIN_2, GPIO_PIN_RESET); \
										} while(0U)
#define DRIVE1_BWD() 				do {\
										HAL_GPIO_WritePin(DRIVER1_LOGIC_PORT, DRIVER1_LOGIC_PIN_1, GPIO_PIN_RESET); \
										HAL_GPIO_WritePin(DRIVER1_LOGIC_PORT, DRIVER1_LOGIC_PIN_2, GPIO_PIN_SET); \
										} while(0U)
#define DRIVE1_STOP() 				do {\
										HAL_GPIO_WritePin(DRIVER1_LOGIC_PORT, DRIVER1_LOGIC_PIN_1, GPIO_PIN_RESET); \
										HAL_GPIO_WritePin(DRIVER1_LOGIC_PORT, DRIVER1_LOGIC_PIN_2, GPIO_PIN_RESET); \
										} while(0U)

/* Exported variable ---------------------------------------------------*/
extern MDRIVE_Handles 				MDRIVE1;
extern LIME_MPACKET_Type 			SERIAL_PACKAGE1;

/* Exported functions --------------------------------------------------*/
LIME_Status 						LIME_MDRIVE1_INIT(void);
LIME_Status 						LIME_MDRIVE1_START(void);
LIME_Status 						LIME_MDRIVE1_MOTOR_UPDATE(void);
#endif

/* DRIVER 2 ------------------------------------------------------------*/
#ifdef LIME_MDRIVER_2
#define DRIVER2_SAMPLE_TIMER 		TIM2
#define DRIVER2_SAMPLE_PORT 		GPIOA
#define DRIVER2_SAMPLE_CHANNELS 	(GPIO_PIN_0 | GPIO_PIN_1)

/** @defgroup LIME_MDRIVER_1_Encoder_pins
  * @{
  */
#define DRIVER2_ENCODER_TIMER 		TIM2
#define DRIVER2_ENCODER_NVIC_IRQn 	TIM2_IRQn
#define DRIVER2_ENCODER_PRIORITY	((uint32_t) 7)
#define DRIVER2_ENCODER_PORT1 		GPIOB
#define DRIVER2_ENCODER_PIN1 		GPIO_PIN_3
#define DRIVER2_ENCODER_PORT2 		GPIOA
#define DRIVER2_ENCODER_PIN2 		GPIO_PIN_15
#define DRIVER2_ENCODER_TIMER_AF 	GPIO_AF1_TIM2
/**
  * @}
  */

/** @defgroup LIME_MDRIVER_1_Dir_pins
  * @{
  */
#define DRIVER2_LOGIC_PORT 			GPIOC
#define DRIVER2_LOGIC_PIN_1 		GPIO_PIN_11
#define DRIVER2_LOGIC_PIN_2 		GPIO_PIN_10
/**
  * @}
  */

/** @defgroup LIME_MDRIVER_1_PWM_pins
  * @{
  */
#define DRIVER2_STEP_TIMER 			TIM9
#define DRIVER2_STEP_CHANNEL 		TIM_CHANNEL_2
#define DRIVER2_STEP_PORT 			GPIOE
#define DRIVER2_STEP 				GPIO_PIN_6
#define DRIVER2_STEP_TIMER_AF 		GPIO_AF3_TIM9
/**
  * @}
  */

/* Exported macros -----------------------------------------------------*/
#define DRIVE2_FWD() 				do {\
										HAL_GPIO_WritePin(DRIVER2_LOGIC_PORT, DRIVER2_LOGIC_PIN_1, GPIO_PIN_SET); \
										HAL_GPIO_WritePin(DRIVER2_LOGIC_PORT, DRIVER2_LOGIC_PIN_2, GPIO_PIN_RESET); \
										} while(0U)
#define DRIVE2_BWD() 				do {\
										HAL_GPIO_WritePin(DRIVER2_LOGIC_PORT, DRIVER2_LOGIC_PIN_1, GPIO_PIN_RESET); \
										HAL_GPIO_WritePin(DRIVER2_LOGIC_PORT, DRIVER2_LOGIC_PIN_2, GPIO_PIN_SET); \
										} while(0U)
#define DRIVE2_STOP() 				do {\
										HAL_GPIO_WritePin(DRIVER2_LOGIC_PORT, DRIVER2_LOGIC_PIN_1, GPIO_PIN_RESET); \
										HAL_GPIO_WritePin(DRIVER2_LOGIC_PORT, DRIVER2_LOGIC_PIN_2, GPIO_PIN_RESET); \
										} while(0U)

/* Exported variable ---------------------------------------------------*/
extern MDRIVE_Handles 				MDRIVE2;

/* Exported functions --------------------------------------------------*/
LIME_Status 						LIME_MDRIVE2_INIT(void);
LIME_Status 						LIME_MDRIVE2_START(void);
LIME_Status 						LIME_MDRIVE2_MOTOR_UPDATE(void);
#endif

/* Exported functions ----------------------------------------*/
LIME_Status 				LIME_MDRIVE_INIT(void);
LIME_Status 				LIME_MOTOR_PID(LIME_MDRIVE_MOTOR_t * current_motor, TIM_HandleTypeDef * PWMTimerHandle);

/* Experimetal stuff*/
typedef int32_t error_t;
typedef struct
{
	float 		accumulative_error;
	float 		new_error;
	float 		total_mass;
	float 		torque;
	float 		old_error;
	float 		error_rate;
	float 		absolute_weight[5];
	float 		adaptive_weight[5];
} pos_error_t;

void calc_error(pos_error_t * pos_err);
void calc_velocity(pos_error_t * pos_err, uint32_t max_speed);

extern pos_error_t pos_err;

#ifdef __cplusplus
}
#endif

#endif /* LIME_MDRIVER_H_ */
