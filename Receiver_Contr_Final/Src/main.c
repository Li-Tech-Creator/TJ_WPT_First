/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//以下是充电控制的相关参数
uint32_t ADC_ConvertedValue[4];//AD采样结果存储
uint16_t Vot_Limit_Rec,Cur_Limit_Rec,Temp_Limit_Rec;
uint16_t Vot_Rec_Buck_In,Duty_Rec_Buck,Vot_Rec_Buck_Out,Cur_Rec_Buck,Temp_Rec_Battery;
uint16_t WorkMode;//对于WorkMode，第1-2位存储温度控制模式，00表示失能，11表示使能，10和01无意义
//对于WorkMode，第3-4位存储BUCK控制模式，00表示正常PI控制，11表示全关，10表示全开，01无意义
uint8_t SystemState;
float K_VotStart,K_VotStop;//通过BUCK输入电压检测来转换接收端状态，两个K值是输入电压与输出电压的比值
uint8_t Flag_WorkMode_Conversion;
uint16_t TimeCnt_WorkMode_Conversion;
uint16_t Delt_VotStart,Delt_VotStop;
/*******************************************************
SystemState用以决定工作状态切换，暂定义如下：
0：未充电状态；1：正常充电状态。只有这两种状态，中间状态不做特殊处理
*******************************************************/

float Cur_Kp,Cur_Ki,Vot_Kp,Vot_Ki;//PI控制的参数

//以下是红外发射的相关参数
uint8_t Flag1,Flag2,BitCnt,Flag_Infray_Done,Jiaoyan_Infray;//计算数据校验用，低4位有用;
uint8_t Flag_Zhen_No,Flag_Infray_WaitTime;
//只有Flag_Infray_Done标志为1时，才允许发射红外信号
uint32_t Infray_TransDat;//要发送的数据
uint16_t Period[32];//存储每一位数据对应的Pulse信息
uint16_t TimeCnt_Infray;//用以定时3秒钟，在休眠态，每隔3秒钟，发送一次红外，一次发送4帧，循环2遍
uint8_t Cnt_InfrayTrans;//用以记录红外发送循环了几次

//以下是车辆自身信息
uint32_t Bike_ID;

//以下是串口控制信息
uint8_t Flag_StartByte;//表征是否检测到字头
uint8_t Byte_Cnt;//对接收字节的数量进行计数
uint8_t Data_ReceArray[6];//存储接收到的串口数据，6个字节为一组，超时未接收完成则全部舍弃
uint8_t Data_ReceSingle;//单个接收字节
uint8_t Flag_Trans_Done,Flag_Rece_Done,TimeCnt485;
uint8_t SystInfo_Package[18];
uint16_t Jiaoyan485;

//以下是FLASH相关信息
uint32_t ReadFlashData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//红外发射函数，形参为要发送的32位数值
void Ir_Tx(uint32_t Dat);
//红外待发信息打包函数
void Infray_InfoPackage_First_First(void);//打包第一类帧第一分帧
void Infray_InfoPackage_First_Second(void);//打包第一类帧第二分帧
void Infray_InfoPackage_First_Third(void);//打包第一类帧第三分帧
void Infray_InfoPackage_Second(void);//打包第二类帧
void Infray_InfoPackage_Third(void);//打包第三类帧
//FLASH读写函数
uint32_t ReadFlash(uint32_t Address);//读指定位置上的数值，一次读取4个字节
void WriteFlash(uint32_t Address);//向指定地址上写数据，半字操作，即一次写入两个字节
//变量初始化函数
void Variable_Init(void);
//全部信息打包及发送函数
void Info_Trans(void);
void Info_Package(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define FLASH_RW_StartAddress ((uint32_t) 0x0803F804)  //FLASH第127页起始地址向后偏移4个字节
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t ss,sum;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	/*********************************参数调节区间*********************************/
	
	//对于充电控制的电压限值和电流限值，通过上位机软件下发指令进行修改
	//充电控制的两个参数为Vot_Limit_Rec和Cur_Limit_Rec，在AD中断中使用，进行电压电流保护
	//对于状态转换，涉及到4个变量：float K_VotStart,K_VotStop以及uint16_t Delt_VotStart,Delt_VotStop;
	//在未充电状态时，检测到输入电压Vot_Rec_Buck_In大于K_VotStart*(0.465*Vot_Rec_Buck_Out-65+Delt_VotStart)时，开始高电压计时，高电压计时超过1s，转换状态
	//K_VotStart=0.85,Delt_VotStart=100
	//计时过程中，检测到Vot_Rec_Buck_In小于K_VotStart*(0.465*Vot_Rec_Buck_Out-65)时，清除计时标志，重新检测
	//在正常充电状态时，检测到输入电压Vot_Rec_Buck_In小于K_VotStop*（0.465*Vot_Rec_Buck_Out-Delt_VotStop）时，开始低电压计时，低电压计时超过30s，转换状态
	//K_VotStop=0.7,Delt_VotStop=100
	//计时过程中，检测到Vot_Rec_Buck_In大于0.465*Vot_Rec_Buck_Out*K_VotStop时，清除计时标志，重新检测
	K_VotStart=0.55;Delt_VotStart=100;//设置Delt_VotStart和Delt_VotStop主要是避免采样不准和抖动带来的误操作
	K_VotStop=0.25;Delt_VotStop=100;
	//另，在开启温度保护后，当温度高于一定值后，系统会启动保护，禁止充电，上报故障，该温度保护的阈值由Temp_Limit_Rec确定，其值越小，温度越高
	Temp_Limit_Rec=620;

	/*********************************参数调节区间*********************************/
	
		
	//变量初始化，包括电压/电流限定值、工作模式、车辆ID的读取，状态变量的设定，PI控制参数的初始化
	Variable_Init();
		
	Rece_485_En();
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
	
	//根据PI控制，当没有输入电流时，会自动调整Duty_Rec_Buck至满开
	//保持BUCK满开，能够有效避免输入电压过低，对设备安全极为重要，因此，各种状态下，均无需关闭BUCK控制，除非测试指令中的全关指令
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_4);
	
	Flag_Infray_Done=1;//启动红外功能
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		
		if(Flag_StartByte)
		{
			if(TimeCnt485>50)//已检测到字头，但50ms内未接收完成6个字节，则复位重新接收
			{
				Flag_StartByte=0;
				TimeCnt485=0;
				Byte_Cnt=0;
			}
		}
		
		if(Flag_Rece_Done)//接收完成6个字节的有效数据，进行处理
		{
			Flag_Rece_Done=0;
			sum=0;//求和值先清零
			for(ss=1;ss<5;ss++)
			{
				sum+=Data_ReceArray[ss];
			}
			if(sum==Data_ReceArray[5])//判断校验，校验失败则不做任何处理
			{
				switch(Data_ReceArray[0])
				{
					case 0x0F: 
					{
						//反馈接收端所有电路信息
						Info_Trans();
						break;
					}
					case 0x1E: 
					{
						//修改电压限值及电流限值，写入FLASH，然后反馈所有电路信息
						Vot_Limit_Rec=Data_ReceArray[1]+(Data_ReceArray[3]&0xf0)*16;
						Cur_Limit_Rec=Data_ReceArray[2]+(Data_ReceArray[3]&0x0f)*256;
						WriteFlash(FLASH_RW_StartAddress);
						Info_Trans();						
						break;
					}
					case 0x2D: 
					{
						//修改BUCK控制模式为全开，然后反馈所有电路信息
						WorkMode|=0x08;//对第4位置1
						WorkMode&=0xfB;//对第3位清0
						htim1.Instance->CCR4=3600;
						Info_Trans();
						break;
					}
					case 0x3C: 
					{
						//修改BUCK控制模式为全关，然后反馈所有电路信息
						WorkMode|=0x0C;//对第3-4位置1
						htim1.Instance->CCR4=0;
						Info_Trans();
						break;
					}
					case 0x4B: 
					{
						//恢复BUCK控制模式为PI控制，然后反馈所有电路信息
						WorkMode&=0xf3;//对第3-4位清零
						Info_Trans();
						break;
					}
					case 0x5A: 
					{
						//修改车载设备编号，写入FLASH，然后反馈所有电路信息
						Bike_ID=0;
						Bike_ID+=Data_ReceArray[4];
						Bike_ID=Bike_ID<<8;
						Bike_ID+=Data_ReceArray[3];
						Bike_ID=Bike_ID<<8;
						Bike_ID+=Data_ReceArray[2];
						Bike_ID=Bike_ID<<8;
						Bike_ID+=Data_ReceArray[1];
						WriteFlash(FLASH_RW_StartAddress);
						Info_Trans();
						break;
					}
					case 0x69: 
					{
						//失能温度控制，写入FLASH，然后反馈所有电路信息
						WorkMode&=0xfc;//对1-2位清零
						WriteFlash(FLASH_RW_StartAddress);
						Info_Trans();
						break;
					}
					case 0x78: 
					{
						//使能温度控制，写入FLASH，然后反馈所有电路信息
						WorkMode|=0x03;//对1-2位置1
						WriteFlash(FLASH_RW_StartAddress);
						Info_Trans();
						break;
					}
					
					default: break;
				}
			}
		}
		switch(SystemState)
		{
			case 0:
			{
				//半休眠态，需要发送红外第一类帧及第二类帧，共4帧，间隔3秒发送一次
				//工作状态的切换靠输入电压检测
				
				if(Flag_Infray_WaitTime)
				{
					//Flag_Zhen_No的转换在定时器中断中实现
					if(Flag_Infray_Done)
					{
						Flag_Infray_Done=0;
						HAL_Delay(100);
						if(Flag_Zhen_No==0)//发送第一类帧第一分帧
						{
							if(Cnt_InfrayTrans)
								Infray_InfoPackage_First_First();
							Ir_Tx(Infray_TransDat);							
						}
						else if(Flag_Zhen_No==1)//发送第一类帧第二分帧
						{
							if(Cnt_InfrayTrans)
								Infray_InfoPackage_First_Second();
							Ir_Tx(Infray_TransDat);
						}
						else if(Flag_Zhen_No==2)//发送第一类帧第三分帧
						{
							if(Cnt_InfrayTrans)
								Infray_InfoPackage_First_Third();
							Ir_Tx(Infray_TransDat);
						}
						else if(Flag_Zhen_No==3)//发送第二类帧
						{
							if(Cnt_InfrayTrans)
								Infray_InfoPackage_Second();
							Ir_Tx(Infray_TransDat);
							if(Cnt_InfrayTrans)
								Cnt_InfrayTrans--;//Flag_Zhen_No初始化从0开始，发送完第三类帧后循环次数减1
							else
								Flag_Infray_WaitTime=0;//SystTick中置1，发送完第三类帧，且已循环发送2遍后，清零，代表完成了一次发送，等待下次定时时间到再次置1
						}
					}
				}
				if(Vot_Rec_Buck_In>(K_VotStart*0.465*Vot_Rec_Buck_Out+Delt_VotStart))//检测输入电压用以状态切换
				{
					if(!Flag_WorkMode_Conversion)//首次检测到输入电压超出设定阈值
					{
						Flag_WorkMode_Conversion=1;//设置标志位
					}
					else//之前已检测到输入电压过高，此处对处于高电压的时间进行计时判断
					{
						if(Flag_WorkMode_Conversion>2)////值-1即已延时的秒数，此处判断是否电压过高状态是否已持续1s
						{
							SystemState=1;
							Flag_WorkMode_Conversion=0;
							TimeCnt_WorkMode_Conversion=0;
						}//如果持续时间未到，则不动作
					}
				}
				//电压值过高，则设置Flag_WorkMode_Conversion标志位，并开始计数
				//电压值过低，则清除标志位，重新检测，只有电压值持续过高，才会使得Flag_WorkMode_Conversion持续增加
				if(Vot_Rec_Buck_In<K_VotStart*0.465*Vot_Rec_Buck_Out)
				{
					Flag_WorkMode_Conversion=0;
					TimeCnt_WorkMode_Conversion=0;
				}
				break;
			}
			case 1:
			{

				if(Flag_Infray_Done)
				{
					Flag_Infray_Done=0;
					HAL_Delay(100);
					if(Flag_Zhen_No==0)//发送第二类帧
					{
						Infray_InfoPackage_Second();
						Ir_Tx(Infray_TransDat);
					}
					else//发送第三类帧
					{
						Infray_InfoPackage_Third();
						Ir_Tx(Infray_TransDat);
					}
				}
				if(Vot_Rec_Buck_In<200)//检测输入电压，用以状态切换
				{
					if(!Flag_WorkMode_Conversion)//首次检测到输入电压小于设定阈值
					{
						LED2_TOGGLE();
						Flag_WorkMode_Conversion=1;//设置标志位
					}
					else//之前已检测到输入电压过低，此处对处于低电压的时间进行计时判断
					{
						if(Flag_WorkMode_Conversion>6)//值-1即已延时的秒数，此处判断是否电压过低状态是否已持续30s
						{							
							SystemState=0;//持续30s过低，则转换状态，清零两个计数
							Flag_WorkMode_Conversion=0;
							TimeCnt_WorkMode_Conversion=0;
							
							//设定红外要继续发送
							Flag_Zhen_No=0;
							Flag_Infray_Done=1;
							Flag_Infray_WaitTime=1;
							
						}//如果时间未到30s，则不动作
					}
				}
				//电压值过低，则设置Flag_WorkMode_Conversion标志位，并开始计数
				//电压值过高，则清除标志位，重新检测，只有电压值持续过低，才会使得Flag_WorkMode_Conversion持续增加
				if(Vot_Rec_Buck_In>600)
				{
					Flag_WorkMode_Conversion=0;
					TimeCnt_WorkMode_Conversion=0;
				}
				break;
			}
			
			default:
			{
				SystemState=0;
				break;
			}
		}

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//SystemTick 1ms中断
void HAL_SYSTICK_Callback(void)
{
	//485通信相关计时及标志
	if(Flag_StartByte)//检测到485字头，开启计时，超时未接收完，则舍弃全部接收到的数据，重新检测字头
	{
		if(TimeCnt485<255)
		{
			TimeCnt485++;
		}
	}
	//红外发送相关计时及标志
	if(SystemState==0)//只在休眠态需要提供定时信息，其他状态，红外连续发送
	{
		TimeCnt_Infray++;
		if(TimeCnt_Infray>3000)//每隔3秒提供一次定时信息
		{
			TimeCnt_Infray=0;
			Flag_Infray_WaitTime=1;//定时时间到标志置1
			Cnt_InfrayTrans=1;//循环次数设定为2次，主程序中是先发送减1
		}
	}
	//工作状态转换相关计时及标志
	if(Flag_WorkMode_Conversion)
	{
		TimeCnt_WorkMode_Conversion++;
		if(TimeCnt_WorkMode_Conversion>1000)//计时达到1s
		{
			TimeCnt_WorkMode_Conversion=0;
			Flag_WorkMode_Conversion++;
		}
	}
}
//红外发射函数定义
void Ir_Tx(uint32_t Dat)
{
	uint8_t i;
	for(i=0;i<32;i++)
	{
		if(Dat&0x80000000)
			Period[i]=1680;
		else
			Period[i]=560;
		Dat=Dat<<1;
	}
	
	__HAL_TIM_SET_COUNTER(&htim7,0);
	__HAL_TIM_CLEAR_FLAG(&htim7,TIM_FLAG_UPDATE);
	__HAL_TIM_SET_AUTORELOAD(&htim7,9000);	
	Flag1=0;Flag2=0;BitCnt=0;
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim7);
}

//FLASH读数据函数
uint32_t ReadFlash(uint32_t Address)
{
	uint32_t Result;
	Result=*(__IO uint32_t*)(Address);
	return Result;
}
//FLASH写数据函数
void WriteFlash(uint32_t Address)
{
	//FLASH存储操作，对于Stm32F103RCT6，其内存大小为256K，共128页，每页2K	
	FLASH_EraseInitTypeDef Flash;
	uint32_t PageError;
	
	HAL_FLASH_Unlock();//解锁写保护
	//定义擦除对象
	Flash.TypeErase=FLASH_TYPEERASE_PAGES;
	Flash.PageAddress=FLASH_RW_StartAddress;
	Flash.NbPages=1;
	//设置PageError
	PageError=0;//0xFFFFFFFF means that all the pages have been correctly erased
	//调用擦除函数
	HAL_FLASHEx_Erase(&Flash,&PageError);
	//烧写FLASH
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address,Vot_Limit_Rec);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address+2,Cur_Limit_Rec);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Address+4,Bike_ID);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address+8,WorkMode);
	HAL_FLASH_Lock();
}

//红外发射的中断处理
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器更新事件的回调函数
{
	if(htim->Instance==TIM1)
	{
		//TIM1是BUCK控制的中断，暂时不做处理
	}
	//控制红外的TIM3未开中断，因此不需要处理
	if(htim->Instance==TIM7)
	{
		if(Flag1==0)//表明处于引导码发射
		{
			if(Flag2==0)//表明刚完成PWM发射，需要关断一段时间
			{
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
				Flag2=1;				
				__HAL_TIM_SET_AUTORELOAD(&htim7,4500);	
			}
			else//表明引导码高低电平均已完成
			{
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
				Flag1=1;Flag2=0;				
				__HAL_TIM_SET_AUTORELOAD(&htim7,560);//载波560us
			}
		}
		else//完成引导码发射
		{
			if(Flag2==0)
			{
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
				Flag2=1;				
				__HAL_TIM_SET_AUTORELOAD(&htim7,Period[BitCnt]);
				if(BitCnt>31)
				{
					HAL_TIM_Base_Stop_IT(&htim7);
					HAL_TIM_Base_Stop_IT(&htim3);
					Flag_Infray_Done=1;//红外发送完成标志，只有完成发射，才能进行下一帧发射
					//HAL_Delay(100);
					//Flag_Zhen_No的切换要仔细考虑
					if(SystemState)//正常充电状态
					{
						if(Flag_Zhen_No==0)
							Flag_Zhen_No=1;
						else
							Flag_Zhen_No=0;//在0-1中间循环
					}
					else//半休眠状态
					{
						if(Flag_Zhen_No<3)
							Flag_Zhen_No++;
						else
							Flag_Zhen_No=0;//在0-3中间循环
					}
					/*Flag_Zhen_No++;
					if(Flag_Zhen_No>2)
						Flag_Zhen_No=0;*/
				}
				else
					BitCnt++;
			}
			else
			{
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
				Flag2=0;				
				__HAL_TIM_SET_AUTORELOAD(&htim7,560);
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_UPDATE);
	//HAL_Delay(100);
}

//BUCK控制的PWM波输出比较中断（定时器1，通道4），用来开AD--ADC1-3, ADC1-9, ADC1-14, ADC1-15
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{
		HAL_ADC_Start_DMA(&hadc1,ADC_ConvertedValue,4);
	}
	__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_CC4);
}

//以下为PID控制部分，在AD完成中断中进行控制
//float inte=0;

#define SCALE_Cur 10000
#define SCALE_Vot 10000
#define TIM2_Pulse_Upper 3600

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//获得采样值
  Vot_Rec_Buck_In=ADC_ConvertedValue[0]&0xfff;//BUCK输入电压
	Temp_Rec_Battery=ADC_ConvertedValue[1]&0xfff;//电池温度
	Vot_Rec_Buck_Out=ADC_ConvertedValue[2]&0xfff;//电池电压
	Cur_Rec_Buck=ADC_ConvertedValue[3]&0xfff;//充电电流
	
	//控制算法
	
	int32_t err,out,intOldOut;
	static int32_t oldErr=0,oldOut=0;
	int32_t dout,derr;
			
	int32_t Vot_Err;
	static int32_t Vot_OldErr,Cur_Ref_OldOut;
	int32_t Cur_Ref,D_Cur_Ref,D_Vot_Err;
	
	if(Vot_Rec_Buck_Out>2800)
	{
		Duty_Rec_Buck=Duty_Rec_Buck/2;
		oldOut=oldOut/2;
	}
	else
	{		
		Vot_Err=(Vot_Limit_Rec-Vot_Rec_Buck_Out)*SCALE_Vot;//设定值减去实测值
		D_Vot_Err=(Vot_Err-Vot_OldErr);
		Vot_OldErr=Vot_Err;
		
		D_Cur_Ref=D_Vot_Err*Vot_Kp+Vot_Err*Vot_Ki/20000;
		Cur_Ref=Cur_Ref_OldOut+D_Cur_Ref;
		if(Cur_Ref>Cur_Limit_Rec)
		{
			Cur_Ref=Cur_Limit_Rec;//不超过上限值，但可以为负
		}
		Cur_Ref_OldOut=Cur_Ref;//只有Cur_Ref在0-Cur_Limit_Rec之间时，才更新Cur_Ref_OldOut
			
		err=(Cur_Ref-Cur_Rec_Buck)*SCALE_Cur;//设定值减去实测值
		
		derr=err-oldErr;
		oldErr=err;
		
		dout=derr*Cur_Kp+err*Cur_Ki/20000;
		
		out=oldOut+dout;
		intOldOut=out;
		
		out=out/SCALE_Cur;
		
		if(out>TIM2_Pulse_Upper)
		{
			out=TIM2_Pulse_Upper;
		}
		else
		if(out<0)
		{
			out=0;
		}
		else
		{
			oldOut=intOldOut;//只有输出的占空比在0-3600之间时，才更新输出
		}	
		Duty_Rec_Buck=out;
	}
	
	//只在正常PI控制，温度控制未开，或温度控制已开但温度未超标时候，才允许更新PWM波的占空比
	//当处于BUCK全开或BUCK全关，或正常PI控制且开启温度控制但温度超标条件下，不允许更新PWM波占空比，PWM波将保持在主程序中设定的值
	/*if((WorkMode&0x0C)==0)//处于正常PI控制模式
	{
		if((WorkMode&0x03)==0)//温度控制失能
		{
			htim1.Instance->CCR4=Duty_Rec_Buck;
		}
		else if(Temp_Rec_Battery>Temp_Limit_Rec)//WorkMode的1-2位非零，即认为开启了温度控制，此时需要判断温度是否超标，温度越高，温度采样值越小
		{
			htim1.Instance->CCR4=Duty_Rec_Buck;
		}
		else
		{
			htim1.Instance->CCR4=0;//温度超标，关闭BUCK
		}			
	}*/
	htim1.Instance->CCR4=Duty_Rec_Buck;
	
}
//串口发送完成中断
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	Flag_Trans_Done=1;
	Rece_485_En();
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
}

//串口接收完成中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(!Flag_StartByte)//未检测到字头，需要先检测字头
	{
		if(Data_ReceSingle==0x55)
		{
			Flag_StartByte=1;//已检测到字头，标记置1
		}
	}
	else//已检测到字头，需要填充有效数据
	{
		Data_ReceArray[Byte_Cnt]=Data_ReceSingle;
		if(Byte_Cnt<5)//有效数据6个字节，下标最多到5
		{
			Byte_Cnt++;
		}
		else//已接收满，重新检测字头，清除485计时，字节下标归0，设定标志位
		{
			Flag_StartByte=0;
			TimeCnt485=0;
			Byte_Cnt=0;
			Flag_Rece_Done=1;
		}
	}
	
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
}

//变量初始化函数
void Variable_Init()
{
	//读取FLASH中存储的控制信息，以及车辆编码
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress);
	Vot_Limit_Rec=ReadFlashData&0xfff;//电压限值
	Cur_Limit_Rec=(ReadFlashData>>16)&0xfff;//电流限值
	
	if(Vot_Limit_Rec>4000)
		Vot_Limit_Rec=0;
	if(Cur_Limit_Rec>4000)
		Cur_Limit_Rec=0;
	
	//需要对两个值进行非法判定，以后再考虑
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress+4);
	Bike_ID=ReadFlashData;//车辆编码
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress+8);
	WorkMode=ReadFlashData&0x03;//工作模式，有效值只取1-2位，3-4位默认00，表示正常PI控制
	
	Duty_Rec_Buck=3600;//BUCK全开，防止输入端过压
	SystemState=0;//系统处于半休眠态
	Flag_WorkMode_Conversion=0;//工作模式转换标志
	TimeCnt_WorkMode_Conversion=0;//工作模式转换计数
		
	//PI参数初始化
	Cur_Kp=0.001;Cur_Ki=5;
	Vot_Kp=0.001;Vot_Ki=5;
	
	//红外相关变量初始化
	Flag1=0;Flag2=0;
	BitCnt=0;Flag_Infray_Done=0;
	Flag_Zhen_No=0;Jiaoyan_Infray=0;
	Flag_Infray_WaitTime=1;//初始条件默认允许发射
	TimeCnt_Infray=0;
	
	//串口相关变量初始化
	Flag_StartByte=0;Byte_Cnt=0;TimeCnt485=0;
	Flag_Trans_Done=0;Flag_Rece_Done=0;
}

//发射端所有信息发送
void Info_Trans(void)
{
	Trans_485_En();
	Info_Package();
	HAL_UART_Transmit_DMA(&huart1,SystInfo_Package,sizeof(SystInfo_Package));
}

void Info_Package(void)
{
	uint8_t i;
	uint8_t Temp_ReadInfo;
	
	//存储到FLASH中的值，必须再从FLASH中直接读取，以验证确实已正确的写入了FLASH
	
	SystInfo_Package[0]=0xaa;//开始字头
	SystInfo_Package[1]=Vot_Rec_Buck_Out;//输出电压低8位
	SystInfo_Package[2]=Cur_Rec_Buck;//输出电流低8位
	SystInfo_Package[3]=(Vot_Rec_Buck_Out>>4)&0xf0;//取输出电压9-12位，放入高4位
	SystInfo_Package[3]+=(Cur_Rec_Buck>>8)&0x0f;//取输出电流9-12位，放入低4位
	SystInfo_Package[4]=Vot_Rec_Buck_In;//输入电压低8位
	SystInfo_Package[5]=Duty_Rec_Buck;//占空比低8位
	SystInfo_Package[6]=(Vot_Rec_Buck_In>>4)&0xf0;//取输入电压9-12位，放入高4位
	SystInfo_Package[6]+=(Duty_Rec_Buck>>8)&0x0f;//取占空比9-12位，放入低4位
	SystInfo_Package[7]=Temp_Rec_Battery;//电池温度低8位
	SystInfo_Package[8]=(Temp_Rec_Battery>>4)&0xf0;//电池温度9-12位，放入高4位
	
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress+8);
	Temp_ReadInfo=ReadFlashData&0x03;//只取低两位信息，BUCK模式信息不使用
	WorkMode=WorkMode&0xfC;//把低两位数据清零
	WorkMode=WorkMode+Temp_ReadInfo;//BUCK控制模式会写入FLASH，但初始化时会把3-4位清零，即系统复位后，BUCK模式默认为正常PI控制
	//正常发送数据时，则取出BUCK控制模式信息（WorkMode的3-4位）
	SystInfo_Package[8]+=WorkMode&0x0f;//工作模式，放入低4位
	
	//读取FLASH中存储的电压限值和电流限值
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress);
	Vot_Limit_Rec=ReadFlashData&0xfff;//电压限值
	Cur_Limit_Rec=(ReadFlashData>>16)&0xfff;//电流限值
	
	SystInfo_Package[9]=Vot_Limit_Rec;//电压限值低8位
	SystInfo_Package[10]=Cur_Limit_Rec;//电流限值低8位
	SystInfo_Package[11]=(Vot_Limit_Rec>>4)&0xf0;//取电压限值9-12位，放入高4位
	SystInfo_Package[11]+=(Cur_Limit_Rec>>8)&0x0f;//取电流限值9-12位，放入低4位
	
	//读取FLASH中存储的Bike_ID
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress+4);
	Bike_ID=ReadFlashData;//车辆编码
	
	SystInfo_Package[12]=Bike_ID;//车辆编码低8位
	SystInfo_Package[13]=Bike_ID>>8;//车辆编码9-16位
	SystInfo_Package[14]=Bike_ID>>16;//车辆编码17-24位
	SystInfo_Package[15]=Bike_ID>>24;//车辆编码25-32位
	
	//计算校验值
	Jiaoyan485=0;
	for(i=1;i<16;i++)
	{
		Jiaoyan485+=SystInfo_Package[i];
	}
	SystInfo_Package[16]=Jiaoyan485;//校验值低8位
	SystInfo_Package[17]=Jiaoyan485>>8;//校验值高8位
}

void Infray_InfoPackage_First_First(void)
{
	uint8_t Temp;//临时变量，用以按字节拆分Bike_ID
	Jiaoyan_Infray=0;//只在第一类帧第一分帧中清零
	Infray_TransDat=0x33;//先输入帧头
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Bike_ID;
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Bike_ID>>8;
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Bike_ID>>16;
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
}
void Infray_InfoPackage_First_Second(void)
{
	uint8_t Temp;
	
	Infray_TransDat=0x36;//先输入帧头
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Bike_ID>>24;
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Vot_Limit_Rec;
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=(Vot_Limit_Rec>>4)&0xf0;//取电压限值的9-12位，放入高4位
	Temp+=(Cur_Limit_Rec>>8)&0x0f;//取电流限值的9-12位，放入低4位
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
}
void Infray_InfoPackage_First_Third(void)
{
	uint8_t Temp;
	
	Infray_TransDat=0x39;//先输入帧头
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Cur_Limit_Rec;
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=WorkMode&0x0f;//只取低4位
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Infray_TransDat+=Jiaoyan_Infray;//将Jiaoyan值写入第一类帧第三分帧最低8位
}
void Infray_InfoPackage_Second(void)
{
	uint8_t Temp1,Temp2;
	
	Temp1=0;
	
	Infray_TransDat=Cur_Rec_Buck&0xfff;//写入输出电流值，取低12位
	Infray_TransDat=Infray_TransDat<<12;
	Infray_TransDat+=(Vot_Rec_Buck_Out&0xfff);//写入输出电压值，取低12位
	
	Temp2=Infray_TransDat;
	Temp1+=Temp2;
	
	Temp2=Infray_TransDat>>8;
	Temp1+=Temp2;
	
	Temp2=Infray_TransDat>>16;
	Temp1+=Temp2;
	
	Temp1=Temp1>>4;
	Temp1&=0x0f;
	Temp1+=0xC0;
	
	Infray_TransDat+=Temp1*16777216;//2的24次方，相当于左移24位	
}
void Infray_InfoPackage_Third(void)
{
	uint8_t Temp1,Temp2;
	
	Temp1=0;
	
	Infray_TransDat=Duty_Rec_Buck&0xfff;//写入占空比信息，取低12位
	Infray_TransDat=Infray_TransDat<<12;
	Infray_TransDat+=(Vot_Rec_Buck_In&0xfff);//写入输入电压值，取低12位
	
	//计算校验
	Temp2=Infray_TransDat;
	Temp1+=Temp2;
	
	Temp2=Infray_TransDat>>8;
	Temp1+=Temp2;
	
	Temp2=Infray_TransDat>>16;
	Temp1+=Temp2;
	
	Temp1=Temp1>>4;
	Temp1&=0x0f;
	Temp1+=0x90;
	
	Infray_TransDat+=Temp1*16777216;//2的24次方，相当于左移24位	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
