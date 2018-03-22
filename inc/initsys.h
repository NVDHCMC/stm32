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

#ifndef INITSYS_H_
#define INITSYS_H_

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup 	General configurations
  * @{
  */
#define __USE_FREERTOS__
/**
  * @}
  */

/** @defgroup 	PPP_Interrupt_Priority
  * @{
  */
#define USART1_NVIC_PRIORITY 			((uint32_t) 7)
#define USART2_NVIC_PRIORITY 			((uint32_t) 7)
#define SPI2_NVIC_PRIORITY 				((uint32_t) 6)
#define TIM2_NVIC_PRIORITY 				((uint32_t) 7)
#define TIM3_NVIC_PRIORITY 				((uint32_t) 7)
#define TIM4_NVIC_PRIORITY 				((uint32_t) 6)
#define TIM5_NVIC_PRIORITY 				((uint32_t) 7)
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**/
typedef enum
{
	LIME_OK = 0,
	LIME_ERROR
} LIME_Status;

typedef enum
{
	SPI2_COMPLETE_FLAG = 1,
	USART1_COMPLETE_FLAG
} Event_Flags;

/** @defgroup 	inithandle exported types
  * @{
  */
typedef struct
{
	void						(*ADC_INIT)(void);
	void 						(*TIM1_INIT)(void);
	void		 				(*TIM2_INIT)(void);
	void		 				(*TIM3_INIT)(void);
	void 						(*TIM4_INIT)(void);
	void						(*TIM5_INIT)(void);
	void 						(*TIM10_INIT)(void);
	void		 				(*USART1_INIT)(void);
	void		 				(*USART2_INIT)(void);
	void 						(*SPI1_INIT)(void);
	void 						(*SPI2_INIT)(void);
	void 						(*EX0_PPP_INIT)(void);
	void 						(*EX1_PPP_INIT)(void);
	void 						(*EX2_PPP_INIT)(void);
	void 						(*EX3_PPP_INIT)(void);
	void 						(*EX4_PPP_INIT)(void);
	void 						(*EX5_PPP_INIT)(void);
	void 						(*EX6_PPP_INIT)(void);
	void 						(*EX7_PPP_INIT)(void);
	ADC_HandleTypeDef 			ADC1_Handle;
	TIM_HandleTypeDef 			TIM2_Handle; 			/* TIMer peripherals */
	TIM_HandleTypeDef 			TIM3_Handle;
	TIM_HandleTypeDef 			TIM4_Handle;
	TIM_HandleTypeDef 			TIM5_Handle;
	TIM_HandleTypeDef 			TIM9_Handle;
	TIM_HandleTypeDef 			TIM10_Handle;
	UART_HandleTypeDef			USART1_Handle;
	UART_HandleTypeDef			USART2_Handle; 			/* USART peripherals */
	SPI_HandleTypeDef 			SPI1_Handle;
	SPI_HandleTypeDef 			SPI2_Handle;

} initHandle_struct;
/**
  * @}
  */

/* Exported variables ---------------------------------------------------------*/
extern initHandle_struct 		initHandles;

/* Exported function prototypes -----------------------------------------------*/
void 					ADC_INIT(void);
void 					TIM1_INIT(void);
void 					TIM2_INIT(void);
void 					TIM3_INIT(void);
void					TIM4_INIT(void);
void 					TIM5_INIT(void);
void					TIM10_INIT(void);
void 					USART1_INIT(void);
void 					USART2_INIT(void);
void 					SPI1_INIT(void);
void 					SPI2_INIT(void);
void 					EX0_PPP_INIT(void);
void 					EX1_PPP_INIT(void);
void 					EX2_PPP_INIT(void);
void 					EX3_PPP_INIT(void);
void 					EX4_PPP_INIT(void);
void 					EX5_PPP_INIT(void);
void 					EX6_PPP_INIT(void);
void 					EX7_PPP_INIT(void);
LIME_Status 			LIME_INIT_SYSTEM(void);

#endif /* INITSYS_H_ */
