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

#ifndef LIME_L3GD20_H_
#define LIME_L3GD20_H_

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

/** @addtogroup L3GD20_Config
  * @{
  */
/* ############################# L3GD20 Configuration ############################*/
 /* Set output data type */
typedef float 									LIME_L3GD20_DataType;
#define configL3GD20_DATA_SIZE 					3
#define configL3GD20_MAX_TIMEOUT 				100
#ifndef configLIME_L3GD20_SPI

/* Uncomment the line below to select SPI */
#define configLIME_L3GD20_SPI
#endif

#ifndef configLIME_L3GD20_I2C

/* Uncomment the line below to select I2C*/
//#define configLIME_L3GD20_I2C
#endif

#ifdef configLIME_L3GD20_SPI

/* SPI interface configurations */
#define LIME_L3GD20_SPI                          SPI1
#define LIME_L3GD20_SPI_GPIO_PORT                GPIOA                      /* GPIOA */
#define LIME_L3GD20_SPI_AF                       GPIO_AF5_SPI1
#define LIME_L3GD20_SPI_SCK_PIN                  GPIO_PIN_5                 /* PA.05 */
#define LIME_L3GD20_SPI_MISO_PIN                 GPIO_PIN_6                 /* PA.06 */
#define LIME_L3GD20_SPI_MOSI_PIN                 GPIO_PIN_7                 /* PA.07 */

/* Read/Write command */
#define READWRITE_CMD                           ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD                        ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                              ((uint8_t)0x00)

/* Chip Select macro definition */
#define LIME_L3GD20_CS_LOW()       				HAL_GPIO_WritePin(LIME_L3GD20_CS_GPIO_PORT, LIME_L3GD20_CS_PIN, GPIO_PIN_RESET)
#define LIME_L3GD20_CS_HIGH()      				HAL_GPIO_WritePin(LIME_L3GD20_CS_GPIO_PORT, LIME_L3GD20_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  LIME_L3GD20 SPI Interface pins
  */
#define LIME_L3GD20_CS_GPIO_PORT                GPIOE                       /* GPIOE */
#define LIME_L3GD20_CS_PIN                      GPIO_PIN_3                  /* PE.03 */

#define LIME_L3GD20_INT_GPIO_PORT               GPIOE                       /* GPIOE */
#define LIME_L3GD20_INT1_PIN                    GPIO_PIN_0                  /* PE.00 */
#define LIME_L3GD20_INT1_EXTI_IRQn              EXTI0_IRQn
#define LIME_L3GD20_INT2_PIN                    GPIO_PIN_1                  /* PE.01 */
#define LIME_L3GD20_INT2_EXTI_IRQn              EXTI1_IRQn

#endif

#ifdef configLIME_L3GD20_I2C
#endif

 /* ############################# Configuration assert ############################*/
/* Proper configuration assert*/
#if defined(configLIME_L3GD20_SPI) && defined(configLIME_L3GD20_I2C)

#error "Can not select both I2C and SPI."
#endif
#if !defined(configLIME_L3GD20_SPI) && !defined(configLIME_L3GD20_I2C)

#error "You must select at least one interface, either I2C or SPI."
#endif
/** @defgroup L3GD20_Exported_Constants
  * @{
  */

/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
#define L3GD20_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define L3GD20_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define L3GD20_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define L3GD20_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define L3GD20_STATUS_REG_ADDR        0x27  /* Status register */
#define L3GD20_OUT_X_L_ADDR           0x28  /* Output Register X */
#define L3GD20_OUT_X_H_ADDR           0x29  /* Output Register X */
#define L3GD20_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define L3GD20_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define L3GD20_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define L3GD20_OUT_Z_H_ADDR           0x2D  /* Output Register Z */
#define L3GD20_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define L3GD20_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

#define L3GD20_INT1_CFG_ADDR          0x30  /* Interrupt 1 configuration Register */
#define L3GD20_INT1_SRC_ADDR          0x31  /* Interrupt 1 source Register */
#define L3GD20_INT1_TSH_XH_ADDR       0x32  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_XL_ADDR       0x33  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_YH_ADDR       0x34  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_YL_ADDR       0x35  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_ZH_ADDR       0x36  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_TSH_ZL_ADDR       0x37  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_DURATION_ADDR     0x38  /* Interrupt 1 DURATION register */

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

#define I_AM_L3GD20                 ((uint8_t)0xD4)
#define I_AM_L3GD20_TR              ((uint8_t)0xD5)

/** @defgroup Power_Mode_selection
  * @{
  */
#define L3GD20_MODE_POWERDOWN       ((uint8_t)0x00)
#define L3GD20_MODE_ACTIVE          ((uint8_t)0x08)
/**
  * @}
  */

/** @defgroup OutPut_DataRate_Selection
  * @{
  */
#define L3GD20_OUTPUT_DATARATE_1    ((uint8_t)0x00)
#define L3GD20_OUTPUT_DATARATE_2    ((uint8_t)0x40)
#define L3GD20_OUTPUT_DATARATE_3    ((uint8_t)0x80)
#define L3GD20_OUTPUT_DATARATE_4    ((uint8_t)0xC0)
/**
  * @}
  */

/** @defgroup Axes_Selection
  * @{
  */
#define L3GD20_X_ENABLE            ((uint8_t)0x02)
#define L3GD20_Y_ENABLE            ((uint8_t)0x01)
#define L3GD20_Z_ENABLE            ((uint8_t)0x04)
#define L3GD20_AXES_ENABLE         ((uint8_t)0x07)
#define L3GD20_AXES_DISABLE        ((uint8_t)0x00)
/**
  * @}
  */

/** @defgroup Bandwidth_Selection
  * @{
  */
#define L3GD20_BANDWIDTH_1         ((uint8_t)0x00)
#define L3GD20_BANDWIDTH_2         ((uint8_t)0x10)
#define L3GD20_BANDWIDTH_3         ((uint8_t)0x20)
#define L3GD20_BANDWIDTH_4         ((uint8_t)0x30)
/**
  * @}
  */

/** @defgroup Full_Scale_Selection
  * @{
  */
#define L3GD20_FULLSCALE_250       ((uint8_t)0x00)
#define L3GD20_FULLSCALE_500       ((uint8_t)0x10)
#define L3GD20_FULLSCALE_2000      ((uint8_t)0x20)
#define L3GD20_FULLSCALE_SELECTION ((uint8_t)0x30)
/**
  * @}
  */

/** @defgroup Full_Scale_Sensitivity
  * @{
  */
#define L3GD20_SENSITIVITY_250DPS  ((float)8.75f)         /*!< gyroscope sensitivity with 250 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_500DPS  ((float)17.50f)        /*!< gyroscope sensitivity with 500 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_2000DPS ((float)70.00f)        /*!< gyroscope sensitivity with 2000 dps full scale [DPS/LSB] */
/**
  * @}
  */


/** @defgroup Block_Data_Update
  * @{
  */
#define L3GD20_BlockDataUpdate_Continous   ((uint8_t)0x00)
#define L3GD20_BlockDataUpdate_Single      ((uint8_t)0x80)
/**
  * @}
  */

/** @defgroup Endian_Data_selection
  * @{
  */
#define L3GD20_BLE_LSB                     ((uint8_t)0x00)
#define L3GD20_BLE_MSB	                   ((uint8_t)0x40)
/**
  * @}
  */

/** @defgroup High_Pass_Filter_status
  * @{
  */
#define L3GD20_HIGHPASSFILTER_DISABLE      	((uint8_t)0x00)
#define L3GD20_HIGHPASSFILTER_ENABLE	    ((uint8_t)0x10)
/**
  * @}
  */

/** @defgroup INT1_Interrupt_status
  * @{
  */
#define L3GD20_INT1INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT1INTERRUPT_ENABLE        ((uint8_t)0x80)
/**
  * @}
  */

/** @defgroup INT2_Interrupt_status
  * @{
  */
#define L3GD20_INT2INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT2INTERRUPT_ENABLE        ((uint8_t)0x08)
/**
  * @}
  */

/** @defgroup INT1_Interrupt_ActiveEdge
  * @{
  */
#define L3GD20_INT1INTERRUPT_LOW_EDGE      ((uint8_t)0x20)
#define L3GD20_INT1INTERRUPT_HIGH_EDGE     ((uint8_t)0x00)
/**
  * @}
  */

/** @defgroup Boot_Mode_selection
  * @{
  */
#define L3GD20_BOOT_NORMALMODE             ((uint8_t)0x00)
#define L3GD20_BOOT_REBOOTMEMORY           ((uint8_t)0x80)
/**
  * @}
  */

/** @defgroup High_Pass_Filter_Mode
  * @{
  */
#define L3GD20_HPM_NORMAL_MODE_RES         ((uint8_t)0x00)
#define L3GD20_HPM_REF_SIGNAL              ((uint8_t)0x10)
#define L3GD20_HPM_NORMAL_MODE             ((uint8_t)0x20)
#define L3GD20_HPM_AUTORESET_INT           ((uint8_t)0x30)
/**
  * @}
  */

/** @defgroup High_Pass_CUT OFF_Frequency
  * @{
  */
#define L3GD20_HPFCF_0              0x00
#define L3GD20_HPFCF_1              0x01
#define L3GD20_HPFCF_2              0x02
#define L3GD20_HPFCF_3              0x03
#define L3GD20_HPFCF_4              0x04
#define L3GD20_HPFCF_5              0x05
#define L3GD20_HPFCF_6              0x06
#define L3GD20_HPFCF_7              0x07
#define L3GD20_HPFCF_8              0x08
#define L3GD20_HPFCF_9              0x09
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
typedef uint8_t 				LIME_MEMS_BaseType;
typedef struct
{
	LIME_MEMS_BaseType 			PowerStatus;
	LIME_MEMS_BaseType 			Axis;
	LIME_MEMS_BaseType 			OutputRate;
	LIME_MEMS_BaseType			Bandwidth;
	LIME_MEMS_BaseType			Fullscale;
	LIME_MEMS_BaseType 			Updatemode;
	LIME_MEMS_BaseType 			Endianess;
	LIME_MEMS_BaseType 			IntPinConfig;
	LIME_MEMS_BaseType 			FilterMode;
	LIME_MEMS_BaseType			FilterHPCutoffFreq;
} LIME_L3GD20_InitTypeDef;

/** @defgroup 	LIME_MEMSDRIVE structure
  * @{
  */
typedef struct
{
#ifdef configLIME_L3GD20_SPI
	SPI_HandleTypeDef 			SPI_Handle;
#endif
#ifdef configLIME_L3GD20_I2C
#endif
	LIME_L3GD20_InitTypeDef 	Init;
	LIME_L3GD20_DataType 		data[configL3GD20_DATA_SIZE];
	void 						(*INIT)(void);
	void 						(*UPDATE)(LIME_L3GD20_DataType *);
	uint8_t 					(*READ_ID)(void);
} LIME_MEMSDRIVE_Type;
/**
  * @}
  */

/** @defgroup L3GD20_Exported_Functions
  * @{
  */
LIME_Status LIME_MEMS_INITCMD(uint8_t reg_cmd, uint8_t REG_ADDR, uint8_t numByte);
/* Sensor Configuration Functions */
void    L3GD20_Init(uint16_t InitStruct);
void    L3GD20_DeInit(void);
void    L3GD20_LowPower(uint16_t InitStruct);
uint8_t L3GD20_ReadID(void);
void    L3GD20_RebootCmd(void);

/* Interrupt Configuration Functions */
void    L3GD20_INT1InterruptConfig(uint16_t Int1Config);
void    L3GD20_EnableIT(uint8_t IntSel);
void    L3GD20_DisableIT(uint8_t IntSel);

/* High Pass Filter Configuration Functions */
void    L3GD20_FilterConfig(uint8_t FilterStruct);
void    L3GD20_FilterCmd(uint8_t HighPassFilterState);
void    L3GD20_ReadXYZAngRate(float *pfData);
uint8_t L3GD20_GetDataStatus(void);

/* Gyroscope IO functions */
void    GYRO_IO_Init(void);
void    GYRO_IO_DeInit(void);
void    GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void    GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

/* Gyroscope driver structure */
extern LIME_MEMSDRIVE_Type L3GD20DRIVE;

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

#endif /* LIME_L3GD20_H_ */
