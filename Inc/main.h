/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LCD_B0_Pin GPIO_PIN_4
#define LCD_B0_GPIO_Port GPIOE
#define OTG_HS_OverCurrent_Pin GPIO_PIN_3
#define OTG_HS_OverCurrent_GPIO_Port GPIOE
#define QSPI_D2_Pin GPIO_PIN_2
#define QSPI_D2_GPIO_Port GPIOE
#define RMII_TXD1_Pin GPIO_PIN_14
#define RMII_TXD1_GPIO_Port GPIOG
#define FMC_NBL1_Pin GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE
#define FMC_NBL0_Pin GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_8
#define KEY1_GPIO_Port GPIOB
#define ULPI_D7_Pin GPIO_PIN_5
#define ULPI_D7_GPIO_Port GPIOB
#define BTN3_Pin GPIO_PIN_4
#define BTN3_GPIO_Port GPIOB
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SPDIF_RX0_Pin GPIO_PIN_7
#define SPDIF_RX0_GPIO_Port GPIOD
#define SDMMC_CK_Pin GPIO_PIN_12
#define SDMMC_CK_GPIO_Port GPIOC
#define TrigIn_Pin GPIO_PIN_15
#define TrigIn_GPIO_Port GPIOA
#define TrigIn_EXTI_IRQn EXTI15_10_IRQn
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define DCMI_D6_Pin GPIO_PIN_5
#define DCMI_D6_GPIO_Port GPIOE
#define DCMI_D7_Pin GPIO_PIN_6
#define DCMI_D7_GPIO_Port GPIOE
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define KEY2_Pin GPIO_PIN_9
#define KEY2_GPIO_Port GPIOB
#define VCP_RX_Pin GPIO_PIN_7
#define VCP_RX_GPIO_Port GPIOB
#define QSPI_NCS_Pin GPIO_PIN_6
#define QSPI_NCS_GPIO_Port GPIOB
#define FMC_SDNCAS_Pin GPIO_PIN_15
#define FMC_SDNCAS_GPIO_Port GPIOG
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define LCD_B1_Pin GPIO_PIN_13
#define LCD_B1_GPIO_Port GPIOJ
#define OTG_FS_VBUS_Pin GPIO_PIN_12
#define OTG_FS_VBUS_GPIO_Port GPIOJ
#define Audio_INT_Pin GPIO_PIN_6
#define Audio_INT_GPIO_Port GPIOD
#define FMC_D2_Pin GPIO_PIN_0
#define FMC_D2_GPIO_Port GPIOD
#define SDMMC_D3_Pin GPIO_PIN_11
#define SDMMC_D3_GPIO_Port GPIOC
#define SDMMC_D2_Pin GPIO_PIN_10
#define SDMMC_D2_GPIO_Port GPIOC
#define OTG_FS_P_Pin GPIO_PIN_12
#define OTG_FS_P_GPIO_Port GPIOA
#define NC1_Pin GPIO_PIN_8
#define NC1_GPIO_Port GPIOI
#define SAI2_MCLKA_Pin GPIO_PIN_4
#define SAI2_MCLKA_GPIO_Port GPIOI
#define LCD_DE_Pin GPIO_PIN_7
#define LCD_DE_GPIO_Port GPIOK
#define LCD_B7_Pin GPIO_PIN_6
#define LCD_B7_GPIO_Port GPIOK
#define LCD_B6_Pin GPIO_PIN_5
#define LCD_B6_GPIO_Port GPIOK
#define LCD_B4_Pin GPIO_PIN_12
#define LCD_B4_GPIO_Port GPIOG
#define SAI2_SDB_Pin GPIO_PIN_10
#define SAI2_SDB_GPIO_Port GPIOG
#define LCD_B2_Pin GPIO_PIN_14
#define LCD_B2_GPIO_Port GPIOJ
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_5
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOD
#define DCMI_D5_Pin GPIO_PIN_3
#define DCMI_D5_GPIO_Port GPIOD
#define FMC_D3_Pin GPIO_PIN_1
#define FMC_D3_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOI
#define ARDUINO_D8_Pin GPIO_PIN_2
#define ARDUINO_D8_GPIO_Port GPIOI
#define OTG_FS_N_Pin GPIO_PIN_11
#define OTG_FS_N_GPIO_Port GPIOA
#define uSD_Detect_Pin GPIO_PIN_13
#define uSD_Detect_GPIO_Port GPIOC
#define FMC_A0_Pin GPIO_PIN_0
#define FMC_A0_GPIO_Port GPIOF
#define SAI2_SCKA_Pin GPIO_PIN_5
#define SAI2_SCKA_GPIO_Port GPIOI
#define SAI2_FSA_Pin GPIO_PIN_7
#define SAI2_FSA_GPIO_Port GPIOI
#define LCD_HSYNC_Pin GPIO_PIN_10
#define LCD_HSYNC_GPIO_Port GPIOI
#define SAI2_SDA_Pin GPIO_PIN_6
#define SAI2_SDA_GPIO_Port GPIOI
#define LCD_B5_Pin GPIO_PIN_4
#define LCD_B5_GPIO_Port GPIOK
#define LCD_BL_CTRL_Pin GPIO_PIN_3
#define LCD_BL_CTRL_GPIO_Port GPIOK
#define DCMI_VSYNC_Pin GPIO_PIN_9
#define DCMI_VSYNC_GPIO_Port GPIOG
#define LCD_B3_Pin GPIO_PIN_15
#define LCD_B3_GPIO_Port GPIOJ
#define OTG_FS_OverCurrent_Pin GPIO_PIN_4
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SDMMC_D0_Pin GPIO_PIN_2
#define SDMMC_D0_GPIO_Port GPIOD
#define TP3_Pin GPIO_PIN_15
#define TP3_GPIO_Port GPIOH
#define LED0_Pin GPIO_PIN_1
#define LED0_GPIO_Port GPIOI
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define FMC_A1_Pin GPIO_PIN_1
#define FMC_A1_GPIO_Port GPIOF
#define LCD_DISP_Pin GPIO_PIN_12
#define LCD_DISP_GPIO_Port GPIOI
#define LCD_VSYNC_Pin GPIO_PIN_9
#define LCD_VSYNC_GPIO_Port GPIOI
#define DCMI_PWR_EN_Pin GPIO_PIN_13
#define DCMI_PWR_EN_GPIO_Port GPIOH
#define DCMI_D4_Pin GPIO_PIN_14
#define DCMI_D4_GPIO_Port GPIOH
#define LED3_Pin GPIO_PIN_0
#define LED3_GPIO_Port GPIOI
#define VCP_TX_Pin GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define BTN0_Pin GPIO_PIN_11
#define BTN0_GPIO_Port GPIOI
#define LCD_G6_Pin GPIO_PIN_1
#define LCD_G6_GPIO_Port GPIOK
#define LCD_G7_Pin GPIO_PIN_2
#define LCD_G7_GPIO_Port GPIOK
#define ARDUINO_PWM_D10_Pin GPIO_PIN_8
#define ARDUINO_PWM_D10_GPIO_Port GPIOA
#define OSC_25M_Pin GPIO_PIN_0
#define OSC_25M_GPIO_Port GPIOH
#define FMC_A2_Pin GPIO_PIN_2
#define FMC_A2_GPIO_Port GPIOF
#define LCD_INT_Pin GPIO_PIN_13
#define LCD_INT_GPIO_Port GPIOI
#define LCD_INT_EXTI_IRQn EXTI15_10_IRQn
#define LCD_R0_Pin GPIO_PIN_15
#define LCD_R0_GPIO_Port GPIOI
#define LCD_G4_Pin GPIO_PIN_11
#define LCD_G4_GPIO_Port GPIOJ
#define LCD_G5_Pin GPIO_PIN_0
#define LCD_G5_GPIO_Port GPIOK
#define ARDUINO_RX_D0_Pin GPIO_PIN_7
#define ARDUINO_RX_D0_GPIO_Port GPIOC
#define FMC_A3_Pin GPIO_PIN_3
#define FMC_A3_GPIO_Port GPIOF
#define LCD_CLK_Pin GPIO_PIN_14
#define LCD_CLK_GPIO_Port GPIOI
#define ULPI_NXT_Pin GPIO_PIN_4
#define ULPI_NXT_GPIO_Port GPIOH
#define LCD_G1_Pin GPIO_PIN_8
#define LCD_G1_GPIO_Port GPIOJ
#define LCD_G3_Pin GPIO_PIN_10
#define LCD_G3_GPIO_Port GPIOJ
#define FMC_SDCLK_Pin GPIO_PIN_8
#define FMC_SDCLK_GPIO_Port GPIOG
#define ARDUINO_TX_D1_Pin GPIO_PIN_6
#define ARDUINO_TX_D1_GPIO_Port GPIOC
#define FMC_A4_Pin GPIO_PIN_4
#define FMC_A4_GPIO_Port GPIOF
#define FMC_SDNME_Pin GPIO_PIN_5
#define FMC_SDNME_GPIO_Port GPIOH
#define FMC_SDNE0_Pin GPIO_PIN_3
#define FMC_SDNE0_GPIO_Port GPIOH
#define LCD_G0_Pin GPIO_PIN_7
#define LCD_G0_GPIO_Port GPIOJ
#define LCD_G2_Pin GPIO_PIN_9
#define LCD_G2_GPIO_Port GPIOJ
#define LED4_Pin GPIO_PIN_7
#define LED4_GPIO_Port GPIOG
#define BTN2_Pin GPIO_PIN_6
#define BTN2_GPIO_Port GPIOG
#define ARDUINO_A4_Pin GPIO_PIN_7
#define ARDUINO_A4_GPIO_Port GPIOF
#define ARDUINO_A5_Pin GPIO_PIN_6
#define ARDUINO_A5_GPIO_Port GPIOF
#define FMC_A5_Pin GPIO_PIN_5
#define FMC_A5_GPIO_Port GPIOF
#define NC2_Pin GPIO_PIN_2
#define NC2_GPIO_Port GPIOH
#define LCD_R7_Pin GPIO_PIN_6
#define LCD_R7_GPIO_Port GPIOJ
#define FMC_D1_Pin GPIO_PIN_15
#define FMC_D1_GPIO_Port GPIOD
#define ULPI_D6_Pin GPIO_PIN_13
#define ULPI_D6_GPIO_Port GPIOB
#define FMC_D15_Pin GPIO_PIN_10
#define FMC_D15_GPIO_Port GPIOD
#define ARDA1_CH4_Pin GPIO_PIN_10
#define ARDA1_CH4_GPIO_Port GPIOF
#define ARDA2_CH3_Pin GPIO_PIN_9
#define ARDA2_CH3_GPIO_Port GPIOF
#define ARDA3_CH2_Pin GPIO_PIN_8
#define ARDA3_CH2_GPIO_Port GPIOF
#define FMC_SDCKE0_Pin GPIO_PIN_3
#define FMC_SDCKE0_GPIO_Port GPIOC
#define FMC_D0_Pin GPIO_PIN_14
#define FMC_D0_GPIO_Port GPIOD
#define ULPI_D5_Pin GPIO_PIN_12
#define ULPI_D5_GPIO_Port GPIOB
#define FMC_D14_Pin GPIO_PIN_9
#define FMC_D14_GPIO_Port GPIOD
#define FMC_D13_Pin GPIO_PIN_8
#define FMC_D13_GPIO_Port GPIOD
#define ULPI_STP_Pin GPIO_PIN_0
#define ULPI_STP_GPIO_Port GPIOC
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define ULPI_DIR_Pin GPIO_PIN_2
#define ULPI_DIR_GPIO_Port GPIOC
#define FMC_A6_Pin GPIO_PIN_12
#define FMC_A6_GPIO_Port GPIOF
#define FMC_A11_Pin GPIO_PIN_1
#define FMC_A11_GPIO_Port GPIOG
#define FMC_A9_Pin GPIO_PIN_15
#define FMC_A9_GPIO_Port GPIOF
#define LCD_R5_Pin GPIO_PIN_4
#define LCD_R5_GPIO_Port GPIOJ
#define QSPI_D1_Pin GPIO_PIN_12
#define QSPI_D1_GPIO_Port GPIOD
#define QSPI_D3_Pin GPIO_PIN_13
#define QSPI_D3_GPIO_Port GPIOD
#define EXT_RST_Pin GPIO_PIN_3
#define EXT_RST_GPIO_Port GPIOG
#define RMII_RXER_Pin GPIO_PIN_2
#define RMII_RXER_GPIO_Port GPIOG
#define LCD_R6_Pin GPIO_PIN_5
#define LCD_R6_GPIO_Port GPIOJ
#define DCMI_D3_Pin GPIO_PIN_12
#define DCMI_D3_GPIO_Port GPIOH
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define ARDA0_CH1_Pin GPIO_PIN_0
#define ARDA0_CH1_GPIO_Port GPIOA
#define DCMI_HSYNC_Pin GPIO_PIN_4
#define DCMI_HSYNC_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define FMC_A7_Pin GPIO_PIN_13
#define FMC_A7_GPIO_Port GPIOF
#define FMC_A10_Pin GPIO_PIN_0
#define FMC_A10_GPIO_Port GPIOG
#define LCD_R4_Pin GPIO_PIN_3
#define LCD_R4_GPIO_Port GPIOJ
#define FMC_D5_Pin GPIO_PIN_8
#define FMC_D5_GPIO_Port GPIOE
#define QSPI_D0_Pin GPIO_PIN_11
#define QSPI_D0_GPIO_Port GPIOD
#define FMC_BA1_Pin GPIO_PIN_5
#define FMC_BA1_GPIO_Port GPIOG
#define FMC_BA0_Pin GPIO_PIN_4
#define FMC_BA0_GPIO_Port GPIOG
#define LCD_SCL_Pin GPIO_PIN_7
#define LCD_SCL_GPIO_Port GPIOH
#define DCMI_D0_Pin GPIO_PIN_9
#define DCMI_D0_GPIO_Port GPIOH
#define DCMI_D2_Pin GPIO_PIN_11
#define DCMI_D2_GPIO_Port GPIOH
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define ULPI_CLK_Pin GPIO_PIN_5
#define ULPI_CLK_GPIO_Port GPIOA
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define FMC_A8_Pin GPIO_PIN_14
#define FMC_A8_GPIO_Port GPIOF
#define LCD_R3_Pin GPIO_PIN_2
#define LCD_R3_GPIO_Port GPIOJ
#define FMC_SDNRAS_Pin GPIO_PIN_11
#define FMC_SDNRAS_GPIO_Port GPIOF
#define FMC_D6_Pin GPIO_PIN_9
#define FMC_D6_GPIO_Port GPIOE
#define FMC_D8_Pin GPIO_PIN_11
#define FMC_D8_GPIO_Port GPIOE
#define FMC_D11_Pin GPIO_PIN_14
#define FMC_D11_GPIO_Port GPIOE
#define ULPI_D3_Pin GPIO_PIN_10
#define ULPI_D3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOH
#define LCD_SDA_Pin GPIO_PIN_8
#define LCD_SDA_GPIO_Port GPIOH
#define DCMI_D1_Pin GPIO_PIN_10
#define DCMI_D1_GPIO_Port GPIOH
#define ULPI_D0_Pin GPIO_PIN_3
#define ULPI_D0_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define ULPI_D2_Pin GPIO_PIN_1
#define ULPI_D2_GPIO_Port GPIOB
#define ULPI_D1_Pin GPIO_PIN_0
#define ULPI_D1_GPIO_Port GPIOB
#define LCD_R1_Pin GPIO_PIN_0
#define LCD_R1_GPIO_Port GPIOJ
#define LCD_R2_Pin GPIO_PIN_1
#define LCD_R2_GPIO_Port GPIOJ
#define FMC_D4_Pin GPIO_PIN_7
#define FMC_D4_GPIO_Port GPIOE
#define FMC_D7_Pin GPIO_PIN_10
#define FMC_D7_GPIO_Port GPIOE
#define FMC_D9_Pin GPIO_PIN_12
#define FMC_D9_GPIO_Port GPIOE
#define FMC_D12_Pin GPIO_PIN_15
#define FMC_D12_GPIO_Port GPIOE
#define FMC_D10_Pin GPIO_PIN_13
#define FMC_D10_GPIO_Port GPIOE
#define ULPI_D4_Pin GPIO_PIN_11
#define ULPI_D4_GPIO_Port GPIOB
#define ARDUINO_MISO_D12_Pin GPIO_PIN_14
#define ARDUINO_MISO_D12_GPIO_Port GPIOB
#define ARDUINO_MOSI_PWM_D11_Pin GPIO_PIN_15
#define ARDUINO_MOSI_PWM_D11_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base address of Sector 0, 32 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08008000) /* Base address of Sector 1, 32 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08010000) /* Base address of Sector 2, 32 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08018000) /* Base address of Sector 3, 32 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08020000) /* Base address of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08040000) /* Base address of Sector 5, 256 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08080000) /* Base address of Sector 6, 256 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x080C0000) /* Base address of Sector 7, 256 Kbytes */

#define MemRead 2
#define MemWrite 3

#define TxRecords 7
#define EndProcess 10
#define N_SAMPLE				120
#define N_RECORD				600
#define N_CH						4

#define LED2_ON() HAL_GPIO_WritePin(GPIOH, LED2_Pin, GPIO_PIN_SET)
#define LED1_ON() HAL_GPIO_WritePin(GPIOI, LED1_Pin, GPIO_PIN_SET)
#define LED3_ON() HAL_GPIO_WritePin(GPIOI, LED3_Pin, GPIO_PIN_SET)
#define LED4_ON() HAL_GPIO_WritePin(GPIOG, LED4_Pin, GPIO_PIN_SET)
#define LED0_ON() HAL_GPIO_WritePin(GPIOI, LED0_Pin, GPIO_PIN_SET)
#define LEDs_ON() LED1_ON() LED2_ON() LED3_ON() LED4_ON()

#define LED2_OFF() HAL_GPIO_WritePin(GPIOH, LED2_Pin, GPIO_PIN_RESET)
#define LED1_OFF() HAL_GPIO_WritePin(GPIOI, LED1_Pin, GPIO_PIN_RESET)
#define LED3_OFF() HAL_GPIO_WritePin(GPIOI, LED3_Pin, GPIO_PIN_RESET)
#define LED4_OFF() HAL_GPIO_WritePin(GPIOG, LED4_Pin, GPIO_PIN_RESET)
#define LED0_OFF() HAL_GPIO_WritePin(GPIOI, LED0_Pin, GPIO_PIN_RESET)
#define LEDs_OFF() LED1_OFF() LED2_OFF() LED3_OFF() LED4_OFF()

#define LED13_Toggle() HAL_GPIO_TogglePin(GPIOI, LED1_Pin|LED3_Pin)
#define LED0_Toggle() HAL_GPIO_TogglePin(GPIOI, LED0_Pin)

#define BTN1_Read() !HAL_GPIO_ReadPin(GPIOI, BTN0_Pin)
#define BTN2_Read() HAL_GPIO_ReadPin(GPIOG, BTN2_Pin)
#define BTN3_Read() HAL_GPIO_ReadPin(GPIOB, BTN3_Pin)

//#define KEY1_Read() HAL_GPIO_ReadPin(GPIOB, KEY1_Pin)
//#define KEY2_Read() HAL_GPIO_ReadPin(GPIOB, KEY2_Pin)

#define KEY1_Read() 1
#define KEY2_Read() 1
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
