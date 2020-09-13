/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FilterWindowMedium.h"
#include "Pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile CONVERTERSTATE conv_state=UNKNOW;
volatile uint8_t wave_ind=0;
volatile uint8_t low_pvd_flag = 0;

volatile uint16_t compi_flag = 0;
volatile uint16_t compv_flag = 0;

volatile uint16_t compi_flag_sec = 0;
volatile uint16_t compv_flag_sec = 0;

volatile uint32_t Blocked_By_COMP_TICK=0;

volatile uint8_t Stop_ADC_OCV=0; // Stop ADC OCV
volatile uint8_t Stop_ADC_OCI=0; // Stop ADC OCI

volatile uint16_t ADC_OCV [ADC_sizeBuffer]={0};
volatile uint16_t ADC_OCI [ADC_sizeBuffer]={0};
volatile uint16_t Adc_step=0;

PidController pidVoltageMode;
PidController pidCurrentMode;

float _OCV = 0.0f;
float _OCI = 0.0f;
float resultPID = 0.0f;
uint32_t NewDuty=0;

volatile uint16_t SinusTableCOMP [SinusTableSize]={
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP,
		VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP, VOLTAGE_LEVEL_COMP
};

volatile uint16_t SinusTable [SinusTableSize]={
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL,
		VOLTAGE_LEVEL, VOLTAGE_LEVEL, VOLTAGE_LEVEL
};

/*volatile uint16_t SinusTable [SinusTableSize]={
		1, 115, 228, 341, 453, 565, 676, 787, 896, 1003, 1110, 1214, 1317, 1418, 1517, 1614, 1709, 1801,
		1890, 1977, 2061, 2142, 2220, 2295, 2367, 2435, 2499, 2560, 2618, 2671, 2721, 2767, 2809, 2847,
		2881, 2911, 2936, 2958, 2975, 2988, 2996, 3000, 3000, 2996, 2988, 2975, 2958, 2936, 2911, 2881,
		2847, 2809, 2767, 2721, 2671, 2618, 2560, 2499, 2435, 2367, 2295, 2220, 2142, 2061, 1977, 1890,
		1801, 1709, 1614, 1517, 1418, 1317, 1214, 1110, 1003, 896, 787, 676, 565, 453, 341, 228, 115, 0,
		1, 115, 228, 341, 453, 565, 676, 787, 896, 1003, 1110, 1214, 1317, 1418, 1517, 1614, 1709, 1801,
		1890, 1977, 2061, 2142, 2220, 2295, 2367, 2435, 2499, 2560, 2618, 2671, 2721, 2767, 2809, 2847,
		2881, 2911, 2936, 2958, 2975, 2988, 2996, 3000, 3000, 2996, 2988, 2975, 2958, 2936, 2911, 2881,
		2847, 2809, 2767, 2721, 2671, 2618, 2560, 2499, 2435, 2367, 2295, 2220, 2142, 2061, 1977, 1890,
		1801, 1709, 1614, 1517, 1418, 1317, 1214, 1110, 1003, 896, 787, 676, 565, 453, 341, 228, 115, 0
};

volatile uint16_t SinusTableCOMP [SinusTableSize]={
		900, 1014, 1127, 1240, 1352, 1464, 1575, 1686, 1795, 1902, 2009, 2113, 2216, 2317, 2416, 2513,
		2608, 2700, 2789, 2876, 2960, 3041, 3119, 3194, 3266, 3334, 3398, 3459, 3517, 3570, 3620, 3666,
		3708, 3746, 3780, 3810, 3835, 3857, 3874, 3887, 3895, 3899, 3899, 3895, 3887, 3874, 3857, 3835,
		3810, 3780, 3746, 3708, 3666, 3620, 3570, 3517, 3459, 3398, 3334, 3266, 3194, 3119, 3041, 2960,
		2876, 2789, 2700, 2608, 2513, 2416, 2317, 2216, 2113, 2009, 1902, 1795, 1686, 1575, 1464, 1352,
		1240, 1127, 1014, 0,
		900, 1014, 1127, 1240, 1352, 1464, 1575, 1686, 1795, 1902, 2009, 2113, 2216, 2317, 2416, 2513,
		2608, 2700, 2789, 2876, 2960, 3041, 3119, 3194, 3266, 3334, 3398, 3459, 3517, 3570, 3620, 3666,
		3708, 3746, 3780, 3810, 3835, 3857, 3874, 3887, 3895, 3899, 3899, 3895, 3887, 3874, 3857, 3835,
		3810, 3780, 3746, 3708, 3666, 3620, 3570, 3517, 3459, 3398, 3334, 3266, 3194, 3119, 3041, 2960,
		2876, 2789, 2700, 2608, 2513, 2416, 2317, 2216, 2113, 2009, 1902, 1795, 1686, 1575, 1464, 1352,
		1240, 1127, 1014, 0
};*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void ConverterStart();
void ConverterStop();
void ConverterProccess();
void SetDuty(float _Duty);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ConverterStart()
{
	//TIM15->CCR1=LED_OFF;
	// kHZ
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,
		                                  HRTIM_OUTPUT_TB1
		                                  | HRTIM_OUTPUT_TB2);
	HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B
			| HRTIM_TIMERID_MASTER
			| HRTIM_TIMERID_TIMER_C);
	wave_ind=0;

	conv_state=CONVERTER_STARTED;
}

void ConverterStop()
{
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1
	                                  | HRTIM_OUTPUT_TA2
	                                  | HRTIM_OUTPUT_TB1
	                                  | HRTIM_OUTPUT_TB2);
	HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
	conv_state=CONVERTER_STOP;
}

void SetDuty(float _Duty)
{
	NewDuty=HRTIM1->sTimerxRegs[1].CMP2xR+_Duty;
	if (NewDuty<=4600) {
		NewDuty=4620;
	}
	if (NewDuty>23040) {
		NewDuty=23040;
	}
	HRTIM1->sTimerxRegs[1].CMP2xR=NewDuty;
	HRTIM1->sTimerxRegs[1].CMP4xR=23040+NewDuty;
}

void ConverterProccess()
{
		wave_ind++;

		if (wave_ind>wave_ind_bottom_end) {
				wave_ind=0;
		}

		DAC2->DHR12R1=CURRENT_LEVEL_COMP;
		DAC1->DHR12R2=SinusTableCOMP[wave_ind];

		if ((wave_ind==0) || (wave_ind==wave_ind_top_end+1)) {
			if ( ((compi_flag>0) || (compv_flag>0)) && (compi_flag_sec<=comp_cnt_per_sec) && (compv_flag_sec<=comp_cnt_per_sec) ) {
				compi_flag=0;
				compv_flag=0;
				//COMP_1SEC_TICK=0;
				HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
			}
		}

		if ( (compi_flag>comp_cnt_per_wave) || (compv_flag>comp_cnt_per_wave)
				|| (compv_flag_sec>comp_cnt_per_sec)
				|| (compi_flag_sec>comp_cnt_per_sec)) {
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2
						| HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
		} else {

				if ( (wave_ind>=wave_ind_top_start) && (wave_ind<=wave_ind_top_end)) {
					HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2);
					HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
				} else {
					if ((wave_ind>=wave_ind_bottom_start) && (wave_ind<=wave_ind_bottom_end)) {
						HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1);
						HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
					} else {
						//HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
						HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2
												| HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
					}
				}

		}

		// Math OCV  and OCI
		Stop_ADC_OCV=1;
		Stop_ADC_OCI=1;
		__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_JEOC);

		_OCV=FilterWindowMedium::Compute((uint16_t *)ADC_OCV, ADC_sizeBuffer, ADC_sizeWindow);
		_OCI=FilterWindowMedium::Compute((uint16_t *)ADC_OCI, ADC_sizeBuffer, ADC_sizeWindow);

		if (_OCV<SinusTable[wave_ind]- 0.2f) {
			pidCurrentMode
					            .SetReference(CURRENT_LEVEL)
					            .SetSaturation(-18440, 18440)
					            .SetFeedback(_OCI, 0.0002)
					            .SetCoefficient(10, 0, 0, 0, 0)
					            .Compute();
					        resultPID = pidCurrentMode.Get();
			TIM2->CCR1=LED_MINIBLINK;
			TIM2->CCR2=LED_OFF;

		} else {
			pidVoltageMode
			            .SetReference(SinusTable[wave_ind])
			            .SetSaturation(-18440, 18440)
			            .SetFeedback(_OCV, 0.0002)
			            .SetCoefficient(50, 0, 0, 0, 0)
			            .Compute();
					        resultPID = pidVoltageMode.Get();
			TIM2->CCR1=LED_OFF;
			TIM2->CCR2=LED_MINIBLINK;
		}

		SetDuty(resultPID);

		Stop_ADC_OCI=0;
		Stop_ADC_OCV=0;
		__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
}

void FASTfeedback(void)
{
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

	if (__HAL_HRTIM_GET_FLAG(&hhrtim1,HRTIM_IT_FLT1) || (low_pvd_flag==1)) {
		if (conv_state!=CONVERTER_STOP) {
			ConverterStop();
			compi_flag_sec=0;
			compv_flag_sec=0;
		}
		__HAL_HRTIM_CLEAR_FLAG(&hhrtim1, HRTIM_IT_FLT1); // try to start and next cycle
	} else {
		if (conv_state!=CONVERTER_STARTED) {
			ConverterStart();
		}

		ConverterProccess();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		//fast 120msec feedback loop and 50HZ A1 and A2 control
        /*if(htim->Instance == TIM1) //HIGH speed feedback
        {
        	FASTfeedback();
        }*/

        //slow 1sec feedback loop and LEDV and LEDI control
        if(htim->Instance == TIM2) //LOW speed feedback
        {
        	if ( (compi_flag_sec>comp_cnt_per_sec) || (compv_flag_sec>comp_cnt_per_sec))  {
        		if (Blocked_By_COMP_TICK<Blocked_By_COMP_TICK_MAX) {
        			Blocked_By_COMP_TICK++;
        			compi_flag_sec=0;
        			compv_flag_sec=0;
        		}
        	} else {
        		if ((Blocked_By_COMP_TICK>0) && (Blocked_By_COMP_TICK<Blocked_By_COMP_TICK_MAX)) {
        			Blocked_By_COMP_TICK--;
        		}
        	}

        			if ( ( (compi_flag_sec==0) && (compv_flag_sec==0))
        			|| ((Blocked_By_COMP_TICK==0)) ){
                		if (conv_state!=CONVERTER_STARTED) {
                			TIM2->CCR1=LED_BLINK;
                			TIM2->CCR2=LED_BLINK;
                		}
                	}

        }

        //2sec UART
        if(htim->Instance == TIM15) //UART speed feedback
        {
        	TIM15->CCR1=LED_OFF;
        	uint8_t str[]="Hello\r\n";
        	HAL_UART_Transmit_DMA(&huart1, str, sizeof(str)-1);
        }
          /* USER CODE END 2 */
}

static void PVD_Config(void)
{
    PWR_PVDTypeDef sConfigPVD = {0};
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_6; // 2.8V
    sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING_FALLING; // IT settings
    HAL_PWR_ConfigPVD(&sConfigPVD); // configure
    HAL_PWR_EnablePVD(); // activate
}

void HAL_PWR_PVDCallback(void)
{
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO)==1) {
		low_pvd_flag  = 1;
	} else {
		low_pvd_flag  = 0;
	}
}


void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
  if (hcomp->Instance==COMP4) {
	  	//HRTIM1->sTimerxRegs[1].CPT2xR
		compi_flag++;
		compi_flag_sec++;
		ConverterStop();
		TIM2->CCR1=LED_ON;
		TIM15->CCR1=LED_MINIBLINK;
		//COMP_EXTI_DISABLE_IT(COMP_EXTI_LINE_COMP4);
  }

  if (hcomp->Instance==COMP2) {
	    //HRTIM1->sTimerxRegs[1].CPT1xR
		compv_flag++;
		compv_flag_sec++;
		ConverterStop();
		TIM2->CCR2=LED_ON;
		TIM15->CCR1=LED_MINIBLINK;
		//COMP_EXTI_DISABLE_IT(COMP_EXTI_LINE_COMP2);
  }

}

//void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
void  HAL_ADCEx_InjectedCallback(void)
{
	if( __HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS)) {
		__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
	}

	if( __HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS)) {
		__HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
	}
	//__HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
	//__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
	if (!Stop_ADC_OCV) {
		ADC_OCV[Adc_step]=ADC1->JDR1;
	}
	if (!Stop_ADC_OCI) {
		ADC_OCI[Adc_step]=ADC2->JDR1;
	}

	if (Adc_step >= ADC_sizeBuffer) {
		Adc_step = 0;
	}
	Adc_step++;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_HRTIM1_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_COMP2_Init();
  MX_COMP4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // config PVD
  PVD_Config();
  HAL_PWR_PVDCallback();


  // init  kHZ PWM, stop TOP+BOTTOM
  HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1
                                  | HRTIM_OUTPUT_TA2
                                  | HRTIM_OUTPUT_TB1
                                  | HRTIM_OUTPUT_TB2);

  HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER
		  | HRTIM_TIMERID_TIMER_C);

  // igla timer
  HAL_TIM_Base_Start(&htim17);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

  // buzzer timer
  HAL_TIM_Base_Start_IT(&htim15);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

  // cooler timer
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // Unblock UCC
  HAL_GPIO_WritePin(pinUCCcontrol_GPIO_Port, pinUCCcontrol_Pin, GPIO_PIN_RESET);

  //Default State
  if (__HAL_HRTIM_GET_FLAG(&hhrtim1,HRTIM_IT_FLT1)) {
	  ConverterStop();
  }

  // start DAC Current
  //HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, CURRENT_LEVEL);
  DAC2->DHR12R1=CURRENT_LEVEL_COMP;
  HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);

  // start DAC Voltage
  DAC1->DHR12R2=VOLTAGE_LEVEL_COMP;
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);


  //fast 120msec feedback loop and 50HZ A1 and A2 control
  HAL_TIM_Base_Start_IT(&htim1);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  //slow 1sec feedback loop and LEDV and LEDI control
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  // start comparator
  HAL_COMP_Start_IT(&hcomp2);
  HAL_COMP_Start_IT(&hcomp4);

  	//*****************************************************//
    //Start ADC1 and ADC2
    //HAL_ADC_Start_DMA(&hadc1,
    //                        (uint32_t *)ADC1_Data,
    //						  ADC1_Cnt_Channel);
    HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* COMP2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(COMP2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(COMP2_IRQn);
  /* COMP4_6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(COMP4_6_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(COMP4_6_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* PVD_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PVD_IRQn, 12, 0);
  HAL_NVIC_EnableIRQ(PVD_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
