/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
/*void AD9833_Write(uint16_t Data)
{
	uint8_t i;
	SCLK_1();
	//AD9833_Delay();
	FSYNC_1();
	//AD9833_Delay();
	FSYNC_0();
	//AD9833_Delay();
	for(i=0;i<16;i++)
	{
		if(Data&0x8000)
		{
			SDATA_1();
		}
		else
		{
			SDATA_0();
		}
		AD9833_Delay();
		SCLK_0();
		AD9833_Delay();
		SCLK_1();
		
		Data<<=1;		
	}
	FSYNC_1();
}
*/
/*void AD9833_WaveSetting(double Freq,uint16_t Freq_SFR,uint16_t WaveMode,uint16_t Phase)
{
	int frequence_LSB,frequence_MSB,phs_data;
	double frequence_mid,frequence_data;
	int32_t frequence_hex;
	*/
	/*********************************计算频率的16进制值***********************************/
	/*frequence_mid=268435456/25;//适合25M晶振，前面的数字是2的28次方
	frequence_data=Freq*frequence_mid/1000000;//将频率值转换为28位寄存器变量
	
	frequence_hex=frequence_data;
	frequence_LSB=frequence_hex;	//低16位送给frequence_LSB
	frequence_LSB=frequence_LSB&0x3fff;//去除最高两位
	frequence_MSB=frequence_hex>>14;//取高14位寄存器数据，另加两位多余位
	frequence_MSB=frequence_MSB&0x3fff;//去除最高两位
	
	phs_data=Phase|0xC000;//写相位值时，将D15和D14置11
	AD9833_Write(0x0100); //复位AD9833，即RESET位为1
	AD9833_Write(0x2100); //选择数据一次写入，B28位和RESET为1
	
	if(Freq_SFR==0)
	{
		frequence_LSB=frequence_LSB|0x4000;//D15,D14置为01
		frequence_MSB=frequence_MSB|0x4000;
		//使用频率寄存器0输出波形
		AD9833_Write(frequence_LSB);//先录入低14位
		AD9833_Write(frequence_MSB);//再录入高14位
		AD9833_Write(phs_data);			//设置相位
		//AD9833_Write(0x2000);			//设置FSELECT位为0，芯片进入工作状态，频率寄存器0输出波形
	}
	if(Freq_SFR==1)
	{
		frequence_LSB=frequence_LSB|0x8000;//D15,D14置为10
		frequence_MSB=frequence_MSB|0x8000;
		//使用频率寄存器1输出波形
		AD9833_Write(frequence_LSB);//先录入低14位
		AD9833_Write(frequence_MSB);//再录入高14位
		AD9833_Write(phs_data);			//设置相位
		//AD9833_Write(0x2000);			//设置FSELECT位为0，芯片进入工作状态，频率寄存器0输出波形
	}
	
	if(WaveMode==TRI_WAVE)
		AD9833_Write(0x2002);
	if(WaveMode==SQU_WAVE)
		AD9833_Write(0x2028);
	if(WaveMode==SIN_WAVE)
		AD9833_Write(0x2000);
	
}*/
KEYState_TypeDef KEY0_StateRead(void)
{
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==KEY_DOWN_LEVEL)
	{
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==KEY_DOWN_LEVEL)
		{
			while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==KEY_DOWN_LEVEL);
			return KEY_DOWN;
		}
	}
	return KEY_UP;
}

//按键读取定义
KEYState_TypeDef KEY1_StateRead(void)
{
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==KEY_DOWN_LEVEL)
	{
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==KEY_DOWN_LEVEL)
		{
			while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==KEY_DOWN_LEVEL);
			return KEY_DOWN;
		}
	}
	return KEY_UP;
}
/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
