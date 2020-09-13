/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define pinPF0_Pin GPIO_PIN_0
#define pinPF0_GPIO_Port GPIOF
#define pinPF1_Pin GPIO_PIN_1
#define pinPF1_GPIO_Port GPIOF
#define pinOCV_Pin GPIO_PIN_0
#define pinOCV_GPIO_Port GPIOA
#define pinBUZZER_Pin GPIO_PIN_2
#define pinBUZZER_GPIO_Port GPIOA
#define pinOCI_Pin GPIO_PIN_4
#define pinOCI_GPIO_Port GPIOA
#define pinCOMPV_Pin GPIO_PIN_7
#define pinCOMPV_GPIO_Port GPIOA
#define pinCOMPI_Pin GPIO_PIN_0
#define pinCOMPI_GPIO_Port GPIOB
#define pinUCCcontrol_Pin GPIO_PIN_1
#define pinUCCcontrol_GPIO_Port GPIOB
#define pinPWMA1_Pin GPIO_PIN_8
#define pinPWMA1_GPIO_Port GPIOA
#define pinPWMA2_Pin GPIO_PIN_9
#define pinPWMA2_GPIO_Port GPIOA
#define pinPWMB1_Pin GPIO_PIN_10
#define pinPWMB1_GPIO_Port GPIOA
#define pinPWMB2_Pin GPIO_PIN_11
#define pinPWMB2_GPIO_Port GPIOA
#define pinPEREK_Pin GPIO_PIN_12
#define pinPEREK_GPIO_Port GPIOA
#define pinDIO_Pin GPIO_PIN_13
#define pinDIO_GPIO_Port GPIOA
#define pinCLK_Pin GPIO_PIN_14
#define pinCLK_GPIO_Port GPIOA
#define pinLEDI_Pin GPIO_PIN_15
#define pinLEDI_GPIO_Port GPIOA
#define pinLEDV_Pin GPIO_PIN_3
#define pinLEDV_GPIO_Port GPIOB
#define pinCOOLER_Pin GPIO_PIN_4
#define pinCOOLER_GPIO_Port GPIOB
#define pinIGLA_Pin GPIO_PIN_5
#define pinIGLA_GPIO_Port GPIOB
#define pinTX_Pin GPIO_PIN_6
#define pinTX_GPIO_Port GPIOB
#define pinRX_Pin GPIO_PIN_7
#define pinRX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void HAL_ADCEx_InjectedCallback(void);
void FASTfeedback(void);

#define ADC_sizeBuffer						9
#define ADC_sizeWindow						5

#define LED_MINIBLINK						62
#define LED_BLINK							31250
#define LED_ON								62499
#define LED_OFF								0

#define Blocked_By_COMP_TICK_MAX			10

#define CURRENT_LEVEL_COMP						3900 //current level COMP
#define VOLTAGE_LEVEL_COMP						3900 //voltage level COMP

#define CURRENT_LEVEL						3000 //current level
#define VOLTAGE_LEVEL						3000 //voltage level

#define comp_cnt_per_wave				10 // max 10
#define comp_cnt_per_sec				80 // max 100

#define wave_ind_top_start 1 // 0.....82=83
#define wave_ind_top_end 83 // 0.....82=83

#define wave_ind_bottom_start 85 // 84.....167=83
#define wave_ind_bottom_end 167 // 84.....167=83

#define SinusTableSize				6


typedef enum
{
   CONVERTER_STOP,          				// 0
   CONVERTER_STARTED,         				// 1
   UNKNOW									// 2
}CONVERTERSTATE;

extern volatile uint32_t Blocked_By_COMP_TICK;

extern volatile CONVERTERSTATE conv_state;

extern volatile uint32_t COMP_1SEC_TICK; // wait 1sec before restart

extern volatile uint8_t wave_ind;

extern volatile uint8_t low_pvd_flag; // low pvd flag

extern volatile uint16_t compi_flag; // comp current cnt flag
extern volatile uint16_t compv_flag;  // comp voltage cnt flag

extern volatile uint16_t compi_flag_sec; // comp current cnt flag 1sec period
extern volatile uint16_t compv_flag_sec;  // comp voltage cnt flag 1sec period

extern volatile uint8_t Stop_ADC_OCV; // Stop ADC OCV
extern volatile uint8_t Stop_ADC_OCI; // Stop ADC OCI

extern volatile uint16_t ADC_OCV [ADC_sizeBuffer];
extern volatile uint16_t ADC_OCI [ADC_sizeBuffer];
extern volatile uint16_t Adc_step;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
