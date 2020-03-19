/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
//LED灯定义
#define LED1_ON()											 HAL_GPIO_WritePin(GPIOB,	GPIO_PIN_15,	 GPIO_PIN_SET);
#define LED1_OFF()										 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,  GPIO_PIN_RESET);
#define LED1_TOGGLE()                  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
#define LED2_ON()											 HAL_GPIO_WritePin(GPIOC,	GPIO_PIN_6,	 GPIO_PIN_SET);
#define LED2_OFF()										 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_RESET);
#define LED2_TOGGLE()                  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
#define LED3_ON()											 HAL_GPIO_WritePin(GPIOC,	GPIO_PIN_7,	 GPIO_PIN_SET);
#define LED3_OFF()										 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,  GPIO_PIN_RESET);
#define LED3_TOGGLE()                  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
#define LED4_ON()											 HAL_GPIO_WritePin(GPIOC,	GPIO_PIN_8,	 GPIO_PIN_SET);
#define LED4_OFF()										 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
#define LED4_TOGGLE()                  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);

//AD9833接口及参数定义
#define FSYNC_0()											 HAL_GPIO_WritePin(GPIOB,	GPIO_PIN_12,	 GPIO_PIN_RESET);
#define FSYNC_1()											 HAL_GPIO_WritePin(GPIOB,	GPIO_PIN_12,	 GPIO_PIN_SET);

#define SCLK_0()											 HAL_GPIO_WritePin(GPIOB,	GPIO_PIN_13,	 GPIO_PIN_RESET);
#define SCLK_1()											 HAL_GPIO_WritePin(GPIOB,	GPIO_PIN_13,	 GPIO_PIN_SET);

#define SDATA_0()											 HAL_GPIO_WritePin(GPIOB,	GPIO_PIN_14,	 GPIO_PIN_RESET);
#define SDATA_1()											 HAL_GPIO_WritePin(GPIOB,	GPIO_PIN_14,	 GPIO_PIN_SET);

#define TRI_WAVE	0		//输出三角波
#define SIN_WAVE	1		//输出正弦波
#define SQU_WAVE	2		//输出方波

//485使能定义
#define Trans_485_En()									 HAL_GPIO_WritePin(GPIOA,	GPIO_PIN_12,	 GPIO_PIN_SET);
#define Rece_485_En()										 HAL_GPIO_WritePin(GPIOA,	GPIO_PIN_12,	 GPIO_PIN_RESET);

//按键触发定义
#define KEY_DOWN_LEVEL  1
typedef enum
{
	KEY_UP		=0,
	KEY_DOWN	=1,
}KEYState_TypeDef;

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
static void AD9833_Delay(void)//短延时
{
	uint16_t i;
	for(i=0;i<1;i++)
		;
}

/*
*********************************************************************************************************
*	函 数 名: AD9833_Write
*	功能说明: 向SPI总线发送16个bit数据
*	形    参: TxData : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
//void AD9833_Write(uint16_t Data);//写指令，16位为一帧数据
/*
*********************************************************************************************************
*	函 数 名: AD9833_WaveSetting
*	功能说明: 设置频率信号
*	形    参: 1.Freq: 频率值, 0.1 hz - 12Mhz
			  2.Freq_SFR: 0 或 1，选择使用哪个频率寄存器
			  3.WaveMode: TRI_WAVE(三角波),SIN_WAVE(正弦波),SQU_WAVE(方波)
			  4.Phase : 波形的初相位
*	返 回 值: 无
*********************************************************************************************************
*/ 
//void AD9833_WaveSetting(double Freq,uint16_t Freq_SFR,uint16_t WaveMode,uint16_t Phase);

/*
//读按键函数
*/
KEYState_TypeDef KEY0_StateRead(void);
KEYState_TypeDef KEY1_StateRead(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
