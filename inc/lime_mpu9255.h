/**
  ******************************************************************************
  * @file    lime_mdriver.h
  * @author  Azure
  * @version V0.2.0
  * @date    8th-October-2017
  * @brief 	 FreeRTOS compatible l3gd20 mems driver.
  *
  *
  ******************************************************************************
*/

#ifndef LIME_MPU9255_H_
#define LIME_MPU9255_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/** @addtogroup LIME_MEMS
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup MPU9255_Config
  * @{
  */
/* ############################# MPU9255 Configuration ############################*/
 /* Set output data type */ // Checked
typedef uint8_t  *								LIME_MPU9255_DataType;
#define configMPU9255_DATA_SIZE 				3
#define configMPU9255_MAX_TIMEOUT 				100
#ifndef configLIME_MPU9255_SPI

/* Uncomment the line below to select SPI */ // Checked
#define configLIME_MPU9255_SPI
#endif

#ifndef configLIME_MPU9255_I2C

/* Uncomment the line below to select I2C*/ // Checked
//#define configLIME_MPU9255_I2C
#endif

#ifdef configLIME_MPU9255_SPI

/* SPI interface configurations */ // Checked
#define LIME_MPU9255_SPI                          SPI3
#define LIME_MPU9255_SPI_GPIO_PORT                GPIOB                      /* GPIOA */
#define LIME_MPU9255_SPI_AF                       GPIO_AF6_SPI3
#define LIME_MPU9255_SPI_SCK_PIN                  GPIO_PIN_3                 /* PA.05 */
#define LIME_MPU9255_SPI_MISO_PIN                 GPIO_PIN_4                 /* PA.06 */
#define LIME_MPU9255_SPI_MOSI_PIN                 GPIO_PIN_5                 /* PA.07 */

/* Read/Write command */ // Not checked
#define READWRITE_CMD                           ((uint8_t)0x80)
/* Multiple byte read/write command */ // Not checked
#define MULTIPLEBYTE_CMD                        ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
// Not checked
#define DUMMY_BYTE                              ((uint8_t)0x00)

/* Chip Select macro definition */ // Checked
#define LIME_MPU9255_CS_LOW()       				HAL_GPIO_WritePin(LIME_MPU9255_CS_GPIO_PORT, LIME_MPU9255_CS_PIN, GPIO_PIN_RESET)
#define LIME_MPU9255_CS_HIGH()      				HAL_GPIO_WritePin(LIME_MPU9255_CS_GPIO_PORT, LIME_MPU9255_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  LIME_MPU9255 SPI Interface pins // Not checked
  */
#define LIME_MPU9255_CS_GPIO_PORT                GPIOA                       /* GPIOA */
#define LIME_MPU9255_CS_PIN                      GPIO_PIN_15                 /* PA.15 */

#define LIME_MPU9255_INT_GPIO_PORT               GPIOE                       /* GPIOE */
#define LIME_MPU9255_INT1_PIN                    GPIO_PIN_0                  /* PE.00 */
#define LIME_MPU9255_INT1_EXTI_IRQn              EXTI0_IRQn
#define LIME_MPU9255_INT2_PIN                    GPIO_PIN_1                  /* PE.01 */
#define LIME_MPU9255_INT2_EXTI_IRQn              EXTI1_IRQn

#endif

#ifdef configLIME_MPU9255_I2C
#endif

 /* ############################# Configuration assert ############################*/
/* Proper configuration assert*/
#if defined(configLIME_MPU9255_SPI) && defined(configLIME_MPU9255_I2C)

#error "Can not select both I2C and SPI."
#endif
#if !defined(configLIME_MPU9255_SPI) && !defined(configLIME_MPU9255_I2C)

#error "You must select at least one interface, either I2C or SPI."
#endif
/** @defgroup MPU9255_Exported_Constants
  * @{
  */
// InDev
/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
#define MPU9255_WHO_AM_I_ADDR          0x75  /* device identification register */				// Checked
#define MPU9255_CONFIG_ADDR        	   0x1a  /* Config register */ 								// Checked
#define MPU9255_GYRO_CONFIG_ADDR       0x1b  /* Gyro configuration register */					// Checked
#define MPU9255_ACCEL_CONFIG_ADDR      0x1c  /* Accelerometer configuration register */ 		// Checked
#define MPU9255_ACCEL_CONFIG_2_ADDR    0x1d  /* Accelerometer configuration register 2 */ 		// Checked
#define MPU9255_LP_ACCEL_CTRL_ADDR     0x1e  /* Low-power mode accelerometer  					// Checked
											  * output rate configuration register */
#define MPU9255_WOM_THR_ADDR           0x1f  /* Wake-on-motion threshold register */			// Checked
#define MPU9255_FIFO_EN_ADDR           0x23  /* FIFO enable register */							// Checked
#define MPU9255_I2C_MASTER_CTRL 	   0x24  /* I2C master control register */ 					// Checked

#define MPU9255_INT_PIN_CFG_ADDR       0x37  /* Interrupt pin configuration register */			// Checked
#define MPU9255_INT_ENABLE_ADDR        0x38  /* Interrupt enable register */					// Checked
#define MPU9255_INT_STATUS_ADDR        0x3a  /* Interrupt status register */					// Checked
#define MPU9255_ACCEL_OUT_X_H_ADDR     0x3b  /* Accelerometer X output L register */			// Checked
#define MPU9255_ACCEL_OUT_X_L_ADDR     0x3c  /* Accelerometer X output H register */			// Checked
#define MPU9255_ACCEL_OUT_Y_H_ADDR     0x3d  /* Accelerometer Y output L register */			// Checked
#define MPU9255_ACCEL_OUT_Y_L_ADDR     0x3e  /* Accelerometer Y output H register */			// Checked
#define MPU9255_ACCEL_OUT_Z_H_ADDR     0x3f  /* Accelerometer Z output L register */			// Checked
#define MPU9255_ACCEL_OUT_Z_L_ADDR     0x40  /* Accelerometer Z output H register */			// Checked
#define MPU9255_TEMP_OUT_H_ADDR        0x41  /* Temperature Z output L register */				// Checked
#define MPU9255_TEMP_OUT_L_ADDR        0x42  /* Temperature Z output H register */				// Checked
#define MPU9255_GYRO_OUT_X_H_ADDR      0x43  /* Gyroscope X output L register */				// Checked
#define MPU9255_GYRO_OUT_X_L_ADDR      0x44  /* Gyroscope X output H register */				// Checked
#define MPU9255_GYRO_OUT_Y_H_ADDR      0x45  /* Gyroscope Y output L register */				// Checked
#define MPU9255_GYRO_OUT_Y_L_ADDR      0x46  /* Gyroscope Y output H register */				// Checked
#define MPU9255_GYRO_OUT_Z_H_ADDR      0x47  /* Gyroscope Z output L register */				// Checked
#define MPU9255_GYRO_OUT_Z_L_ADDR      0x48  /* Gyroscope Z output H register */				// Checked
#define MPU9255_SIGNAL_PATH_RST_ADDR   0x68  /* Signal path reset register */					// Checked
#define MPU9255_MOT_DETECT_CTRL_ADDR   0x69  /* Motion detection controller register */			// Checked
#define MPU9255_USER_CTRL_ADDR         0x6a  /* User controller register */						// Checked
#define MPU9255_PWR_MGMT_1_ADDR        0x6b  /* Power management 1 register */					// Checked
#define MPU9255_PWR_MGMT_2_ADDR        0x6c  /* Power management 2 register */					// Checked

// Not checked
/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

#define I_AM_MPU9255                 ((uint8_t)0x73) 											// Checked

/** @defgroup Power_Management_selection
  * @{
  */
#define MPU9255_MODE_RESET_ALL 		 ((uint8_t)0x80)
#define MPU9255_MODE_SLEEP       	 ((uint8_t)0x40)
#define MPU9255_MODE_CYCLE           ((uint8_t)0x20)
#define MPU9255_MODE_STANDBY 		 ((uint8_t)0x10)
#define MPU9255_MODE_PD_PTAT 		 ((uint8_t)0x08)
/**
  * @}
  */

/** @defgroup Clock_source_select 																// Checked
  * @{
  */
#define MPU9255_CLKSEL_20MHZ_INTERNAL_OSCILLATOR ((uint8_t)0x00)
#define MPU9255_CLKSEL_AUTO_SELECT 	 ((uint8_t)0x01)
#define MPU9255_CLKSEL_STOP 		 ((uint8_t)0x07)
/**
  * @}
  */

/** @defgroup Axes_Selection 																	// Checked
  * @{
  */
#define MPU9255_ACCEL_X_ENABLE      ((uint8_t)0xe0)
#define MPU9255_ACCEL_Y_ENABLE      ((uint8_t)0xd0)
#define MPU9255_ACCEL_Z_ENABLE      ((uint8_t)0xc8)
#define MPU9255_GYRO_X_ENABLE       ((uint8_t)0xc4)
#define MPU9255_GYRO_Y_ENABLE       ((uint8_t)0xc2)
#define MPU9255_GYRO_Z_ENABLE 		((uint8_t)0xc1)
/**
  * @}
  */

/** @defgroup Bandwidth_Selection 																// Checked
  * @{
  */
#define MPU9255_BANDWIDTH_0         ((uint8_t)0x00)
#define MPU9255_BANDWIDTH_1         ((uint8_t)0x01)
#define MPU9255_BANDWIDTH_2         ((uint8_t)0x02)
#define MPU9255_BANDWIDTH_3         ((uint8_t)0x03)
#define MPU9255_BANDWIDTH_4         ((uint8_t)0x04)
#define MPU9255_BANDWIDTH_5         ((uint8_t)0x05)
#define MPU9255_BANDWIDTH_6         ((uint8_t)0x06)
#define MPU9255_BANDWIDTH_7         ((uint8_t)0x07)
/**
  * @}
  */

/** @defgroup Full_Scale_Selection 																// Checked
  * @{
  */
#define MPU9255_FULLSCALE_250       ((uint8_t)0x00)
#define MPU9255_FULLSCALE_500       ((uint8_t)0x08)
#define MPU9255_FULLSCALE_1000      ((uint8_t)0x10)
#define MPU9255_FULLSCALE_2000 		((uint8_t)0x18)
/**
  * @}
  */

/** @defgroup Gyro_Filter_choice 																// Checked
  * @{
  */
#define MPU9255_GYRO_FILTER_1 		((uint8_t) 0x03)
#define MPU9255_GYRO_FILTER_2 		((uint8_t) 0x02)
#define MPU9255_GYRO_FILTER_3 		((uint8_t) 0x00)
/**
  * @}
  */

/** @defgroup Accelerometer_Full_Scale_Selection 												// Checked
  * @{
  */
#define MPU9255_FULLSCALE_2G      	((uint8_t)0x00)         /*!< gyroscope sensitivity with 250 dps full scale [DPS/LSB]  */
#define MPU9255_FULLSCALE_4G 		((uint8_t)0x08)        /*!< gyroscope sensitivity with 500 dps full scale [DPS/LSB]  */
#define MPU9255_FULLSCALE_8G 		((uint8_t)0x10)        /*!< gyroscope sensitivity with 2000 dps full scale [DPS/LSB] */
#define MPU9255_FULLSCALE_16G		((uint8_t)0x18)
/**
  * @}
  */

/** @defgroup Accel_Filter_choice 																// Checked
  * @{
  */
#define MPU9255_ACCEL_FILTER_0 		((uint8_t) 0x80)
#define MPU9255_ACCEL_FILTER_1 		((uint8_t) 0x00)
/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
// Not checked
typedef uint8_t 				LIME_MEMS_BaseType;
typedef struct
{
	LIME_MEMS_BaseType 			PowerMnt1;
	LIME_MEMS_BaseType 			GyroConfig;
	LIME_MEMS_BaseType 			Axis;
	LIME_MEMS_BaseType 			AccelConfig1;
	LIME_MEMS_BaseType 			AccelConfig2;
	LIME_MEMS_BaseType 			Config;
	LIME_MEMS_BaseType			Bandwidth;
	LIME_MEMS_BaseType 			IntPinConfig;
	LIME_MEMS_BaseType 			MagnetoConfig;
} LIME_MPU9255_InitTypeDef;

/** @defgroup 	LIME_MEMSDRIVE structure
  * @{
  */
typedef struct
{
#ifdef configLIME_MPU9255_SPI
	SPI_HandleTypeDef 			SPI_Handle;
#endif
#ifdef configLIME_MPU9255_I2C
#endif
	LIME_MPU9255_InitTypeDef 	Init;
	LIME_MPU9255_DataType 		data[configMPU9255_DATA_SIZE];
	void 						(*INIT)(void);
	void 						(*UPDATE)(LIME_MPU9255_DataType);
	uint8_t 					(*READ_ID)(void);
} LIME_MEMSDRIVE_Type;
/**
  * @}
  */

/** @defgroup MPU9255_Exported_Functions
  * @{
  */
LIME_Status LIME_MEMS_INITCMD(uint8_t reg_cmd, uint8_t REG_ADDR, uint8_t numByte);
/* Sensor Configuration Functions */
void    MPU9255_Init(uint16_t InitStruct);
void    MPU9255_DeInit(void);
void    MPU9255_LowPower(uint16_t InitStruct);
uint8_t MPU9255_ReadID(void);
void    MPU9255_RebootCmd(void);

/* Interrupt Configuration Functions */
void    MPU9255_INT1InterruptConfig(uint16_t Int1Config);
void    MPU9255_EnableIT(uint8_t IntSel);
void    MPU9255_DisableIT(uint8_t IntSel);

/* High Pass Filter Configuration Functions */
void    MPU9255_FilterConfig(uint8_t FilterStruct);
void    MPU9255_FilterCmd(uint8_t HighPassFilterState);
void    MPU9255_ReadXYZAngRate(float *pfData);
uint8_t MPU9255_GetDataStatus(void);

/* Gyroscope IO functions */
void    GYRO_IO_Init(void);
void    GYRO_IO_DeInit(void);
void    GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void    GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

/* Gyroscope driver structure */
extern LIME_MEMSDRIVE_Type MPU9255DRIVE;

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
  }
#endif

#endif /* LIME_MPU9255_H_ */
