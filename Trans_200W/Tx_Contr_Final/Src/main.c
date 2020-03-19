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
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//以下是单体桩自身信息
uint8_t Charger_ID;//充电桩编号
uint8_t Charger_State;//充电桩当前状态
uint8_t Charger_Start_Permission;//允许开启充电标志
/***********************************************************
允许开启充电标志仅通过开启指令置1，并将在以下状况出现时清0：
-----正常充电结束
-----命令停止充电
-----红外信号长时中断
-----效率持续过低
-----发射端电流过大或其他故障发生

***********************************************************/
//以下是接收端的相关参数
uint16_t Vot_Limit_Rec,Cur_Limit_Rec;//接收端充电的电压电流限值，通过红外发送过来的值
uint16_t Vot_Rec_Buck_In,Duty_Rec_Buck,Vot_Rec_Buck_Out,Cur_Rec_Buck,Temp_Rec_Battery;
uint8_t WorkMode_Rec;//存储接收端的工作模式
uint32_t Bike_ID;//存储接收到的车辆编码

//以下是发射端的相关参数
uint32_t ADC_ConvertedValue[4];//存储AD采样结果
uint16_t Vot_Trans_Buck_In,Duty_Trans_Buck,Vot_Trans_Buck_Out,Cur_Trans_Buck,Temp_Trans_MOS;
uint8_t Relay_State;//存储继电器配置信息
uint16_t Freq_Value;
uint32_t Fre_Trans_Inverter;
uint8_t Fault_Type;//故障类型，值为1-32，高优先级的故障类型可以覆盖低优先级的故障类型
uint8_t Frequency_Scan_State;//表征当前扫频状态
uint16_t Max_Cur_Trans_Buck;//记录最大扫频电流值
uint32_t Max_Cur_Trans_Frequency;//记录最大扫频电流值对应的频率点
uint8_t Trans_OverCurrent_Alert;//发射端过电流告警
uint8_t Trans_OverCurrent_Alert_Read;//置1表示过流告警已被主程序知晓
uint16_t Vot_Limit_Trans,Cur_Limit_Trans;//发射端充电控制的电压电流限值，通过充电指令下发
uint16_t Charging_ComptCnt;//充电完成计数
uint8_t Low_Efficiency_Cnt;//低效率计数
uint8_t Low_Efficiency_Restart_Cnt;//在效率持续过低时，系统会重新扫频，用该变量进行重新扫频计数，超过3次后，结束充电
//在开启充电控制时，应首先对这两个限值进行比较判定

//以下是红外接收所使用的变量
uint8_t RmtSta,RDATA;//RDATA用以标识当前是上升沿捕获还是下降沿捕获
//[7]:收到引导码标志
//[6]:接收到了一个红外的所有信息，标志完成了一次接收
//[5]：保留
//[4]:标记上升沿是否已经被捕获
//[3:0]:溢出计时器
uint16_t Dval[35];//下降沿时计数器的值
uint32_t RmtRec=0;//红外接收到的数据
uint32_t RmtRecTx=0;//用来串口发射
uint8_t CapCnt=0;//捕获次数
uint16_t Tep;
uint8_t Zhen_Jiaoyan,Data_Jiaoyan;//Zhen_Jiaoyan就是取字头，判断是第几帧，Data_Jiaoyan是判断内部的数据校验
uint8_t Flag_Infrared_Error;//红外接收错误计数
uint8_t Infrared_Error_Location;

uint32_t Infrared_Rec_Succ_Cnt;//红外接收成功计数
uint16_t Infrared_Decode_Error_Cnt;//红外解码错误计数
uint32_t Infrared_Waiting_First_Cnt;//第一类帧延时计数
uint32_t Infrared_Waiting_First_First_Cnt;//第一类帧第一分帧延时计数
uint32_t Infrared_Waiting_First_Second_Cnt;//第一类帧第二分帧延时计数
uint32_t Infrared_Waiting_First_Third_Cnt;//第一类帧第三分帧延时计数
uint32_t Infrared_Waiting_Second_Cnt;//第二类帧延时计数
uint32_t Infrared_Waiting_Third_Cnt;//第三类帧延时计数
uint32_t Infrared_Rec_Error_Cnt;//红外接收错误计数
uint8_t  Flag_Infrared_Data_OK;//表征数据解析OK，可根据该标志进行控制
uint16_t Infrared_Charging_Waiting_Cnt;//红外等待时间，超过3秒，降低BUCK值，超过30s，回到休眠态
uint16_t Infrared_Total_Waiting_Cnt;//表征有多长时间未接收到任意类型的红外信号，超过60s，清零接收成功、解码错误、接收错误等计数
uint32_t Infrary_First_Data_First,Infrary_First_Data_Second,Infrary_First_Data_Third;

//485通信相关变量
uint8_t Flag_Trans_Done,Flag_Rece_Done,TimeCnt485;
uint8_t Flag_StartByte;//表征是否检测到字头
uint8_t Byte_Cnt;//对接收字节的数量进行计数
uint8_t Data_ReceArray[8];//存储接收到的串口数据，8个字节为一组，超时未接收完成则全部舍弃
uint8_t Data_ReceSingle;//单个接收字节
uint8_t Flag_485_Reset;//每隔5秒，判断一下485的状态，并作相应处理

uint8_t All_Data_Package[58];//全部信息打包
uint8_t Fault_Data_Package[31];//故障类型及现场数据打包
uint8_t State_Data_Package[11];//状态信息打包
uint8_t Limit_Data_Package[11];//电压及电流限值信息打包
uint8_t Frequency_Data_Package[11];//扫频信息单个频点数据打包

//故障现场数据存储
uint16_t Field_Vot_Trans_Buck_In,Field_Duty_Trans_Buck,Field_Vot_Trans_Buck_Out,Field_Cur_Trans_Buck;
uint16_t Field_Temp_Trans_MOS,Field_Vot_Rec_Buck_In,Field_Duty_Rec_Buck;
uint16_t Field_Vot_Rec_Buck_Out,Field_Cur_Rec_Buck,Field_Temp_Rec_Battery;
uint32_t Field_Freq_Value;
//发射端电压限值电流限值、接收端电压限值电流限值、接收端工作模式在充电过程中并不改变，无需保存现场

//功率效率计算
double Power_Trans,Power_Trans_Refer,Power_Rece,Efficiency,Power_Trans_Refer_Upper;
double K_Power_Trans,K_Power_Rece;

//根据温度值，每隔10s，更新一次功率限值
uint16_t Temp_Refresh_Cnt;
uint8_t  Flag_Temp_Refresh;

//FLASH读取数据
uint32_t ReadFlashData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Variable_Init(void);//变量初始化赋值

void All_Data_Trans_Packing(void);//全部信息打包
void Fault_Data_Trans_Packing(void);//故障类型及现场数据打包
void State_Data_Trans_Packing(void);//状态信息打包
void Limit_Data_Trans_Packing(void);//电压及电流限值信息打包
void Frequency_Data_Trans_Packing(void);//频率信息打包

//红外数据接收解码
void Data_Decode(void);//接收数据解码
void Decode_First(void);//第一类帧解码
void Decode_Second(void);//第二类帧解码
void Decode_Third(void);//第三类帧解码
void Decode_Fault(void);//故障帧解码

//扫频函数声明
void Frequency_Scan(void);//调参阶段不带485的扫频
void Frequency_Scan485(void);//根据指令需要的，带485通信的扫频
void Charging_Preparing(void);//扫频成功，进行充电前的各项准备
void Charging_Stopping(void);//红外中断或充电完成或效率持续过低，终止充电

//故障现场保护
void Fault_Field_Save(void);

//FLASH读写函数声明
uint32_t ReadFlash(uint32_t Address);//读指定位置上的数值，一次读取4个字节
void WriteFlash(uint32_t Address);//向指定地址上写数据，半字操作，即一次写入两个字节

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define FLASH_RW_StartAddress ((uint32_t) 0x0803F800)  //FLASH第127页起始地址
#define Power_Trans_Refer_Waiting 50
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t ss,sum;//ss用来做求和校验时for循环使用，sum用来做求和结果
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();

  /* USER CODE BEGIN 2 */
	Variable_Init();	
	
	//开启定时器2通道1输入捕获，用以红外接收
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	
	//开启BUCK电路的PWM控制
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_OC_Start_IT(&htim5,TIM_CHANNEL_1);//初始化时，默认占空比为0，全关
	HAL_Delay(1);
	
	//开启频率控制
	//HAL_TIM_Base_Start_IT(&htim1);
	//HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);//初始化时，默认占空比为0，全关
	//HAL_Delay(1);
	
	Rece_485_En();
	HAL_UART_Receive_IT(&huart3,&Data_ReceSingle,1);
	
	//开启看门狗
	HAL_IWDG_Start(&hiwdg);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//LED1_ON();
		//485接收到完整指令，解析指令
		if(Flag_StartByte)
		{
			if(TimeCnt485>50)//已检测到字头，但50ms内未接收完成8个字节，则复位重新接收
			{
				Flag_StartByte=0;
				TimeCnt485=0;
				Byte_Cnt=0;
			}
		}
		
		//定时时间到，判断485状态
		if(Flag_485_Reset)
		{
			Flag_485_Reset=0;
			if(HAL_UART_GetState(&huart3)==HAL_UART_STATE_READY)
			{
				Rece_485_En();
				HAL_UART_Receive_IT(&huart3,&Data_ReceSingle,1);
			}
			else if(HAL_UART_GetState(&huart3)==HAL_UART_STATE_RESET)
			{
				MX_USART3_UART_Init();
				Rece_485_En();
				HAL_UART_Receive_IT(&huart3,&Data_ReceSingle,1);
			}
			else if(HAL_UART_GetState(&huart3)==HAL_UART_STATE_ERROR)
			{
				MX_USART3_UART_Init();
				Rece_485_En();
				HAL_UART_Receive_IT(&huart3,&Data_ReceSingle,1);
			}
			else
				;
		}

		//定时时间到，更新温度对应的功率上限值
		if(Flag_Temp_Refresh)
		{
			Flag_Temp_Refresh=0;
			if(Temp_Trans_MOS>900)
			{
				Power_Trans_Refer_Upper=200;
			}
			else
			{
				Power_Trans_Refer_Upper=0.333*Temp_Trans_MOS-100;
			}				
		}
		
		if(Flag_Rece_Done)//接收完成8个字节的有效数据，进行处理
		{
			LED2_TOGGLE();
			HAL_Delay(1);
			Flag_Rece_Done=0;//先清零接收完成标志
			Data_ReceArray[1]=~Data_ReceArray[1];//先把接收到的编号反码取反
			if(Data_ReceArray[0]==Data_ReceArray[1])//判断接收到的编码是否有误
			{//接收正确的情况下，再判断该编码是否为自身编码
				//LED1_TOGGLE();
				if(Data_ReceArray[0]==Charger_ID)
				{//编码接收正确，且为自身编码，才确定是轮询对象，继续解析指令
					
					switch(Data_ReceArray[2])
					{
						case 0x0F://查询状态，返回状态结果
						{
							Trans_485_En();
							State_Data_Trans_Packing();							
							HAL_UART_Transmit_DMA(&huart3,State_Data_Package,sizeof(State_Data_Package));
							break;
						}
						case 0x1E://开启充电，返回电压限值、电流限值以及继电器状态
						{
							//先判断校验
							sum=0;
							for(ss=3;ss<7;ss++)
							{
								sum+=Data_ReceArray[ss];
							}
							if(sum==Data_ReceArray[7])
							{
								//在进行充电模式转换之前，电压限值要与接收端进行核对，不成功则进行报错
								Vot_Limit_Trans=Data_ReceArray[3]+(Data_ReceArray[5]&0xf0)*16;//提取电压限值数据
								Cur_Limit_Trans=Data_ReceArray[4]+(Data_ReceArray[5]&0x0f)*256;//提取电流限值数据
								Relay_State=Data_ReceArray[6];
								if((Charger_State&0x80)&&((Charger_State&0x60)==0x00))//检测到车辆，且系统处于停止态，才能对该指令做出响应
								{	
									Charger_State&=0xf8;//将接收端不可修复故障位，及问题事件标志位，以及充电完成标志位清零，& = 1111, 1000
									if(Fault_Type<16)
										Fault_Type=0;//如果不存在发射端不可修复故障，则将故障类型设为最低
									Charging_ComptCnt=0;//充电完成计数清零
									
									if(Vot_Trans_Buck_Out>3000)//停止态，BUCK输出电压过高，认为BUCK已烧穿，设置故障标志
									{
										Charger_State|=0x10;//0001 0000，BUCK烧穿，对应发射端故障事件，第5位置1
										if(Fault_Type<FaultLevel_Trans_Buck_Short)
										{
											Fault_Type=FaultLevel_Trans_Buck_Short;
											Fault_Field_Save();
										}										
										Charger_Start_Permission=0;//允许充电置0，再次充电则再次下发指令
									}
									else
									{									
										Charger_Start_Permission=1;//允许充电开启
									}
									
									Trans_485_En();
									Limit_Data_Trans_Packing();//充电限值数据打包
									HAL_UART_Transmit_DMA(&huart3,Limit_Data_Package,sizeof(Limit_Data_Package));
								}
							}							
							break;
						}
						case 0x2D://停止充电，返回状态结果
						{
						  Charging_Stopping();
							Trans_485_En();
							State_Data_Trans_Packing();
							HAL_UART_Transmit_DMA(&huart3,State_Data_Package,sizeof(State_Data_Package));
							break;
						}
						case 0x3C://开启扫频
						{
							if((Charger_State&0x60)==0)//先判断是否处于停止态，只有处于停止态，才可以进行扫频动作
							{
								//扫频占空比由指令下发，无论是否检测到车辆，均按指定占空比进行扫频
								Data_ReceArray[4]=~Data_ReceArray[4];//先把继电器反码取反
								if(Data_ReceArray[3]==Data_ReceArray[4])//如果继电器数据有问题，不做应答，总控重下指令
								{
									sum=Data_ReceArray[5]+Data_ReceArray[6];
									if(Data_ReceArray[7]==sum)//判断指令中数据校验是否正确
									{
										Duty_Trans_Buck=Data_ReceArray[6]*256;
										Duty_Trans_Buck+=Data_ReceArray[5];//读取指令指定的占空比信息
										htim5.Instance->CCR1=Duty_Trans_Buck;//先设定扫频占空比
										//设置状态位为指令扫频状态
										Charger_State&=0xBF;//& = 1011 1111 对第七位进行清零
										Charger_State|=0x20;//| = 0010 1111 对第六位进行置1
										Frequency_Scan485();//带485通信的扫频
										Duty_Trans_Buck=0;//扫频结束，关闭BUCK
										htim5.Instance->CCR1=Duty_Trans_Buck;
										Freq_Value=0xffff;
							      TIM1->ARR=Freq_Value;//将定时器重装载寄存器填满，降低频率
	                  htim1.Instance->CCR2=0;
	                  HAL_TIM_Base_Stop_IT(&htim1);
	                  HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2); 
										HAL_Delay(50);
										Charger_State&=0x9F;//扫频完成，对第七和第六位进行清零，恢复状态位为停止态
									}
								}								
							}
							//扫频之前应首先判断是否检测到了车辆，是则使用大占空比，否则使用小占空比
							break;
						}
						case 0x4B://读取单体桩全部信息，返回全部数据
						{
							Trans_485_En();
							All_Data_Trans_Packing();
							HAL_UART_Transmit_DMA(&huart3,All_Data_Package,sizeof(All_Data_Package));
							break;
						}
						case 0x5A://读取单体桩故障类型及故障现场数据，返回故障数据
						{
							Fault_Data_Trans_Packing();
							Trans_485_En();
							HAL_UART_Transmit_DMA(&huart3,Fault_Data_Package,sizeof(Fault_Data_Package));
							break;
						}
						case 0x69://清零复位标志，返回状态信息
						{
							Charger_State&=0xf7;//清零复位标志
							Trans_485_En();
							State_Data_Trans_Packing();
							HAL_UART_Transmit_DMA(&huart3,State_Data_Package,sizeof(State_Data_Package));
							break;
						}
						default: break;
					}
				}
				else if(Data_ReceArray[0]==0xff)//修改单体桩自身ID的指令
				{
					Data_ReceArray[3]=~Data_ReceArray[3];
					if(Data_ReceArray[2]==Data_ReceArray[3])//先校验设定的ID值是否有误
					{
						//设定的ID值通过校验，允许修改及应答
						Charger_ID=Data_ReceArray[2];
						if(Charger_ID>200)
							Charger_ID=0;
						WriteFlash(FLASH_RW_StartAddress);
						Trans_485_En();
						State_Data_Trans_Packing();							
						HAL_UART_Transmit_DMA(&huart3,State_Data_Package,sizeof(State_Data_Package));
					}
				}
				/*else if(Data_ReceArray[0]==0x00)//后门指令，无需核对编号，即可读取单体桩全部信息
				{
					Trans_485_En();
					All_Data_Trans_Packing();
					HAL_UART_Transmit_DMA(&huart3,All_Data_Package,sizeof(All_Data_Package));
				}*/
				else
					;
			}
		}
	
		//红外接收成功，解析数据
		if(RmtSta&(1<<6))
		{
			if(~Infrared_Rec_Succ_Cnt)//当该变量达到满值全1时，取反即为0，计数就会停止增加
				Infrared_Rec_Succ_Cnt++;
			Infrared_Total_Waiting_Cnt=0;//一旦接收成功，就将红外接收总等待计数清零
			RmtSta&=~(1<<6);//接收成功标志位清零
			Data_Decode();//数据解析
		}		
		//红外接收错误，增加错误标志
		if(Flag_Infrared_Error)
		{
			Flag_Infrared_Error=0;
			if(Infrared_Rec_Error_Cnt<0xffffffff)//达到满1时取反为0.停止计数
				Infrared_Rec_Error_Cnt++;
		}
		if(Infrared_Waiting_First_Cnt>10000)//只有处于停止态时，才会进行第一类帧计数，考虑到接收端切换到停止态需要30s的延时计数，因此该时间应大于接收端所需时间
		{
			Charger_State&=0x7f;//超过10s未完整接收第一类帧，认为未检测到车辆，设置标志位
		}
		//如果长时间未接收到任意红外信号，则将接收成功计数、接收失败计数、解码错误计数清零，准备重新计数，该时间段应大于切换到停止态所需要的红外延时计数
		if(Infrared_Total_Waiting_Cnt>60000)//超过60s未接收到任意红外信号
		{
			Infrared_Rec_Succ_Cnt=0;
			Infrared_Decode_Error_Cnt=0;
			Infrared_Rec_Error_Cnt=0;
		}
		
		//未检测到车辆，LED1熄灭
		if(Charger_State&0x80)//取最高位，判断有无车辆
		{
			LED1_ON();
		}
		else
		{
			LED1_OFF();
		}
				
		//已检测到车辆，LED1打开
		
		//状态判断及切换
		/******************************************
		开启充电控制的几个条件：
		-----系统无故障
		-----检测到了车辆红外
		-----开启了充电允许指令
		-----扫频测试通过
		******************************************/
		switch(Charger_State&0x60)//提取当前工作状态，即状态变量第6-7位
		{
			case 0x00://0000，0000 停止态，该状态中进行条件判断，满足条件后切换到工作状态
			{
				
				Power_Trans_Refer=0;//参考功率设为零
				
				/*if(Trans_OverCurrent_Alert)
				{
					Trans_OverCurrent_Alert_Read=1;
					Freq_Value=0xffff;
					TIM1->ARR=Freq_Value;//重装载寄存器填满，降低频率
					htim1.Instance->CCR2=0;
					HAL_TIM_Base_Stop_IT(&htim1);
					HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
					Charger_State|=0x10;//0001 0000，停止态过流，对应发射端故障事件，第5位置1
					if(Fault_Type<FaultLevel_Stop_CurOver_Short)
					{
						Fault_Type=FaultLevel_Stop_CurOver_Short;
						Fault_Field_Save();
					}
				}
				else
				{
					if(Fault_Type==FaultLevel_Stop_CurOver_Short)
					{
						Fault_Type=0;
						Charger_State&=0xef;//1110, 1111 清除第五位发射端故障标志位
					}
				}*/
				
				if(Charger_State&0x10)//提取故障位，判断是否存在发射端不可修复故障，该位置1表示有故障
				{//存在故障
					Charger_Start_Permission=0;//存在发射端故障，关闭充电开启允许标志;
				}
				else
				{//无故障
					if(Charger_State&0x80)//提取车辆检测标志位，检测到车辆才能开启充电
					{
						if(Charger_Start_Permission)//是否有指令允许开启
						{
							if((Vot_Limit_Trans==Vot_Limit_Rec)&&(Cur_Limit_Trans==Cur_Limit_Rec))//指令下发的限值和红外上传的限值应保持一致
							{	
								Duty_Trans_Buck=Duty_Frequency_Scan;
								htim5.Instance->CCR1=Duty_Trans_Buck;//先设定扫频占空比，已检测到车辆，设为指定检测到车辆时的指定占空比
								HAL_Delay(50);//刚开启BUCK，需要给电容器充电，等待充电结束，避免影响扫频结果
								//先设置系统状态位调参扫频态
								Charger_State|=0x40;//第七位置1, |=0100 0000
								Charger_State&=0xDF;//第六位清0, &=1101 1111
								//如果逆变部分烧穿，则扫频最开始就会出现过流的情况，该故障会体现在扫频故障中
								Frequency_Scan();//扫频测试，可能停放正常，也可能停放过距，也可能过流
								//扫频之后的状态要转换为正常充电状态，在Charging_Preparing()函数中进行设置
								//只有停放正常时，才允许开启充电流程
								//停放过距或过流状况时，产生故障工况，不设置状态转换，清零充电允许标志，需要重新调整后再下指令
							}
							else//指令下发的限值和红外上传的限值不一致，可能是私换电池了，设置故障位，
							{
								Charger_Start_Permission=0;//允许充电标志置零，解决问题后重新开启
								Charger_State|=0x04;//发生接收端不可修复故障，设置标志位，再重新下发指令时清零
								if(Fault_Type<FaultLevel_LimitError)//先判断优先级，高优先级故障不可被替代
								{
									Fault_Type=FaultLevel_LimitError;
									Fault_Field_Save();
								}
							}
						}
					}
				}
				break;
			}
			case 0x60://0110，0000 正常充电状态，该状态中进行条件判断，满足条件后切换到停止态
			{
				//判断红外接收数据完成标志，然后进行控制
				//判断红外延时计数，以判断是否产生故障告警
				//判断电流，以判断是否产生告警
				//判断效率，以判断是否产生告警
				//判断功率，以判断是否产生告警
				//判断接收端电压电流，以判断是否完成充电
				//其他状态监管
				if(Infrared_Charging_Waiting_Cnt<3000)//第二类帧和第三类帧都在1s以内时，才会将Infrared_Charging_Waiting_Cnt清零，SysTick每隔1ms加1
				{
					if(Flag_Infrared_Data_OK)
					{
						Flag_Infrared_Data_OK=0;//每次控制清零，再次接收到新的数据再置1
						/*****************************接收端功率不足调节**************************/
						if(Duty_Rec_Buck==3600)//接收端满开，增加发射功率
						{
							if(Vot_Rec_Buck_Out<Vot_Limit_Rec-100)//恒流段
							{	
								//需要先根据接收端电流值判断发射端占空比要增加的量
								if(Cur_Rec_Buck<Cur_Limit_Rec/4)
								{
									Power_Trans_Refer+=10;
								}
								else if(Cur_Rec_Buck<Cur_Limit_Rec/2)
								{
									Power_Trans_Refer+=5;
								}
								else if(Cur_Rec_Buck<Cur_Limit_Rec-500)
								{
									Power_Trans_Refer+=2;
								}
								else
								{
									Power_Trans_Refer+=1;
								}
							}
							else//恒压段
							{
								Power_Trans_Refer+=1;
							}
							if(Power_Trans_Refer>Power_Trans_Refer_Upper)
								Power_Trans_Refer=Power_Trans_Refer_Upper;
						}
						/*******************************接收端过载调节****************************/
						if(Duty_Rec_Buck<3600)//接收端进行了保护，降低发射功率
						{
							//需要先根据接收端占空比判断发射端占空比应下降的量
							if(Duty_Rec_Buck<1000)
							{
								Duty_Trans_Buck=1000;//接收端严重过载，即刻降低发射端占空比，重新调节
								htim5.Instance->CCR1=Duty_Trans_Buck;
								Power_Trans_Refer-=10;
							}
							else if(Duty_Rec_Buck<2000)
							{
								Power_Trans_Refer-=5;
							}
							else if(Duty_Rec_Buck<3000)
							{
								Power_Trans_Refer-=2;
							}
							else
							{
								Power_Trans_Refer-=1;
							}
							if(Power_Trans_Refer<0)
								Power_Trans_Refer=0;
						}
						/********************************充电完成判断******************************/
						//基本思路是：每次红外完整解析并校验成功后，判断充电电压值及充电电流值是否满足充电完成标志
						//如果满足，则计数加1，如果不满足，则计数清零，若持续满足（计数大于100），则认为充电完成
						if(Vot_Rec_Buck_Out>Vot_Limit_Rec-250)//实测电压超过2000，即49V
						{
							if(Cur_Rec_Buck<Cur_Limit_Rec*0.2)//系数由0.35改为0.2
							{
								if(Charging_ComptCnt<100)
									Charging_ComptCnt++;
								else
								{
									Charger_State|=0x02;//Charger_State第二位置1，标志正常完成了充电，在下发充电指令时清零
									Charging_Stopping();
								}
							}
							else
								Charging_ComptCnt=0;//出现反例，清零计数
						}
						else
							Charging_ComptCnt=0;//出现反例，清零计数
						/********************************充电完成判断******************************/
						
						/********************************充电功率、效率计算******************************/
						
						Power_Rece=Vot_Rec_Buck_Out*Cur_Rec_Buck;
						Power_Rece=Power_Rece*K_Power_Rece;
						Efficiency=Power_Rece/Power_Trans;
						
						/********************************充电功率、效率计算******************************/
						if(Efficiency<0.5)
						{
							if(Low_Efficiency_Cnt<100)
							{
								Low_Efficiency_Cnt++;
							}
							else
							{
								Low_Efficiency_Restart_Cnt++;
								if(Low_Efficiency_Restart_Cnt>3)
								{
									Charger_State|=0x01;//将问题事件标志位置1
									//故障记录，先记录现场，再关闭系统
									if(Fault_Type<FaultLevel_EffiencyLow)//先判断优先级，高优先级故障不可被替代
									{
										Fault_Type=FaultLevel_EffiencyLow;
										Fault_Field_Save();
									}
									//终止充电
									Charging_Stopping();
									//低效率重启计数清零
									Low_Efficiency_Restart_Cnt=0;//不能在充电准备中清零，不然永远无法达到3，因此停止时清零
								}
								else
								{
									Charger_State&=0x9f;//&=1001 1111，清零第6-7位，设为停止态，但不清零充电允许标志，主循环中重新开始扫频
								}
							}
						}
						else
						{
							Low_Efficiency_Cnt=0;//出现反例，计数清零
						}
						
						//如果在发射功率较大时，传输效率持续过低，应设置故障标志，设置重新扫频，多次重新扫频后仍然效率持续过低，则拒绝充电
						/********************************充电功率、效率计算******************************/
					}
				}
				else if(Infrared_Charging_Waiting_Cnt<30000)//红外延时在3-30s之间，降低占空比
				{
					Power_Trans_Refer=Power_Trans_Refer_Waiting;
				}
				else//红外延时在30s以外，应回到休眠态
				{
					Charger_State|=0x01;//将问题事件标志位置1
					//故障记录，先记录现场，再关闭系统
					if(Fault_Type<FaultLevel_InfraryStop)//先判断优先级，高优先级故障不可被替代
					{
						Fault_Type=FaultLevel_InfraryStop;
						Fault_Field_Save();
					}
					Charging_Stopping();
				}
				break;
			}
			default:
			{
				break;
			}
		}
		HAL_IWDG_Refresh(&hiwdg);//喂狗
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
			if(TimeCnt485>50)//已检测到字头，但50ms内未接收完成6个字节，则复位重新接收
			{
				Flag_StartByte=0;
				TimeCnt485=0;
				Byte_Cnt=0;
			}
		}
	}
	
	if(Infrared_Total_Waiting_Cnt<0xffff)
	{
		Infrared_Total_Waiting_Cnt++;
	}
	
	if(Temp_Refresh_Cnt<0xffff)//10s钟更新一次功率限值
	{
		Temp_Refresh_Cnt++;
		if(Temp_Refresh_Cnt>5000)
		{
			Flag_485_Reset=1;//每隔5秒判断一下
		}
		if(Temp_Refresh_Cnt>10000)
		{
			Temp_Refresh_Cnt=0;
			Flag_Temp_Refresh=1;
		}
	}
	
	if((Charger_State&0x60)==0)//停止态，进行第一类帧计数和第二类帧计数
	{
		if(Infrared_Waiting_First_Cnt<0xffffffff)//该计数满1时取反为0，不再计数，未满则自增
		{
			Infrared_Waiting_First_Cnt++;
		}
		if(Infrared_Waiting_First_First_Cnt<0xffffffff)//该计数满1时取反为0，不再计数，未满则自增
		{
			Infrared_Waiting_First_First_Cnt++;
		}
		if(Infrared_Waiting_First_Second_Cnt<0xffffffff)//该计数满1时取反为0，不再计数，未满则自增
		{
			Infrared_Waiting_First_Second_Cnt++;
		}
		if(Infrared_Waiting_First_Third_Cnt<0xffffffff)//该计数满1时取反为0，不再计数，未满则自增
		{
			Infrared_Waiting_First_Third_Cnt++;
		}
		if(Infrared_Waiting_Second_Cnt<0xffffffff)
		{
			Infrared_Waiting_Second_Cnt++;
		}
	}
	//SysTick中只进行计数自加，在红外解码函数中进行清零
	if((Charger_State&0x60)==0x60)//正常充电状态，进行第二类帧计数和第三类帧计数
	{
		if(Infrared_Waiting_Second_Cnt<0xffffffff)
		{
			Infrared_Waiting_Second_Cnt++;
		}
		if(Infrared_Waiting_Third_Cnt<0xffffffff)
		{
			Infrared_Waiting_Third_Cnt++;
		}
		if(Infrared_Charging_Waiting_Cnt<0xffff)
		{
			Infrared_Charging_Waiting_Cnt++;
		}
	}
}
//红外接收中断处理，定时器2通道1
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)//本函数处理定时器2通道1的溢出中断
	{
		if(__HAL_TIM_GET_IT_SOURCE(htim,TIM_IT_UPDATE) !=RESET)
		{
			RmtSta&=~0x10;//取消上升沿已被捕获标记
			RmtSta&=~(1<<7);//一旦出现过长延时，即清空引导标志，重新等待引导码
			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
			RDATA=1;
			CapCnt=0;//重新计位
			if((RmtSta&0x0f)<14)
				RmtSta++;//记录溢出次数  
			else
			{
				RmtSta&=0xf0;//清空计数器
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_UPDATE);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)//本函数处理TIM2通道1的输入捕获中断
	{
		if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) !=RESET)
		{
			if(RDATA)//上升沿捕获标志
			{
				//下降沿捕获
				__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
				//清空计数器
				__HAL_TIM_SET_COUNTER(htim,0);
				RmtSta|=0x10;
				RDATA=0;
			}
			else//下降沿捕获
			{				
				__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
				RDATA=1;
				if(RmtSta&0x10)//已捕获一次高电平
				{	
					RmtSta&=~(1<<4);//清除掉已捕获上升沿标志，下次捕获上升沿时再标记
					Dval[CapCnt]=__HAL_TIM_GET_COUNTER(htim);
					if(RmtSta&0x80)//已收到引导码
					{				
						if((Dval[CapCnt]>400)&&(Dval[CapCnt]<800))//560为标准值,560us
						{							
							if(CapCnt<32)
							{
								RmtRec<<=1;//左移一位，低位补0
								CapCnt++;
								if(CapCnt>=32)
								{
									RmtSta|=1<<6;//标志已完成一次接收
									RmtSta&=~(1<<7);//已接收到引导码，且已存储32位信息，停止引导，标记完成接收
									RmtRecTx=RmtRec;
								}
							}
						}
						else if((Dval[CapCnt]>1000)&&(Dval[CapCnt]<1800))//1680为标准值,1680us
						{
							if(CapCnt<32)
							{
								RmtRec<<=1;
								RmtRec+=1;//左移一位，低位补1
								CapCnt++;
								if(CapCnt>=32)
								{
									RmtSta|=1<<6;//标志已完成一次接收
									RmtSta&=~(1<<7);//已接收到引导码，且已存储32位信息，停止引导，标记完成接收
									RmtRecTx=RmtRec;
								}
							}
						}
						else
						{
							Flag_Infrared_Error=1;
							Infrared_Error_Location=CapCnt;
							Tep=Dval[CapCnt];
							RmtSta&=~(1<<7);
						}
					}
					else if(Dval[CapCnt]>4000&&Dval[CapCnt]<5000) //4500为标准值4.5 ms，判定引导码
					{
						RmtSta|=1<<7;//标记成功接收到引导码
						CapCnt=0;
						RmtRec=0;
					}
				}
			}		
		__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_CC4);
		}
	}
}

void Variable_Init(void)
{
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress);
	Charger_ID=ReadFlashData;//取低8位，即第127页第一个字节
	if(Charger_ID>200)
		Charger_ID=101;
	if(Charger_ID==0)
		Charger_ID=101;
	//Charger_ID不可为零
		
	Charger_State=0x08;//未检测到车辆，停止态，无故障，有复位
	
	//开启红外接收
	RDATA=1;
	
	//485标志及计数
	Byte_Cnt=0;
	
	//功率计算系数
	K_Power_Trans=0.0000313;
	K_Power_Rece=0.00002;
	
	//充电完成计数清零
	Charging_ComptCnt=0;
	//低效率计数清零
	Low_Efficiency_Cnt=0;
	//低效率重启计数清零
	Low_Efficiency_Restart_Cnt=0;
	//功率上限值初始化为200W
	Power_Trans_Refer_Upper=200;
  //温度更新标志清零
	Temp_Refresh_Cnt=0;
	Flag_Temp_Refresh=0;
	//485状态判断标志清零
	Flag_485_Reset=0;
	//占空比关断
	Duty_Trans_Buck=0;
	htim5.Instance->CCR1=Duty_Trans_Buck;
	//频率设为0
	Freq_Value=0xffff;
	TIM1->ARR=Freq_Value;//重装载寄存器填满，降低频率
	htim1.Instance->CCR2=0;
	
}

//串口发送完成中断
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	Flag_Trans_Done=1;//提供发送完成标志位
	Rece_485_En();//接收使能
	HAL_UART_Receive_IT(&huart3,&Data_ReceSingle,1);//以中断模式一次接收一个字节
}
//串口接收完成中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t temp;
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
		if(Byte_Cnt<7)//有效数据8个字节，下标最多到7
		{
			Byte_Cnt++;
		}
		else//已接收满，重新检测字头，清除485计时，字节下标归0，设定标志位
		{
			Flag_StartByte=0;
			TimeCnt485=0;
			Byte_Cnt=0;
			temp=Charger_State&0x60;
			if((temp==0x00)||(temp==0x60))//即处于停止态或充电态
			{
				Flag_Rece_Done=1;//允许更新指令，其他情况不更新指令（扫频态），不做应答
			}			
		}
	}	
	HAL_UART_Receive_IT(&huart3,&Data_ReceSingle,1);
}
//红外接收解码
void Data_Decode(void)
{
	Zhen_Jiaoyan=(RmtRecTx>>28)&0x0f;//取帧头
	switch(Zhen_Jiaoyan)
	{
		case 3://帧头为0011
		{
			Infrared_Total_Waiting_Cnt=0;//接收到任意一帧数据，均将红外总计数清零，表示有车辆存在
			Decode_First();
			break;
		}
		case 12://帧头为1100
		{
			Infrared_Total_Waiting_Cnt=0;
			Decode_Second();
			break;
		}
		case 9://帧头为1001
		{
			Infrared_Total_Waiting_Cnt=0;
			Decode_Third();
			break;
		}
		case 6://帧头为0110
		{
			Infrared_Total_Waiting_Cnt=0;
			Decode_Fault();
			break;
		}
		default: 
		{
			if(Infrared_Decode_Error_Cnt<0xffff)//帧头不匹配，解码错误
				Infrared_Decode_Error_Cnt++;
			break;
		}
	}
}

void Decode_First(void)
{
	uint8_t temp1,jiaoyan,sum;
	
	LED2_TOGGLE();
	
	Zhen_Jiaoyan=(RmtRecTx>>24)&0x0f;
	if(Zhen_Jiaoyan==3)
	{
		Infrared_Waiting_First_First_Cnt=0;//解码第一类帧第一分帧
		Infrary_First_Data_First=RmtRecTx;
	}
	else if(Zhen_Jiaoyan==6)
	{
		Infrared_Waiting_First_Second_Cnt=0;//解码第一类帧第二分帧
		Infrary_First_Data_Second=RmtRecTx;
	}
	else if(Zhen_Jiaoyan==9)
	{
		Infrared_Waiting_First_Third_Cnt=0;//解码第一类帧第三分帧
		Infrary_First_Data_Third=RmtRecTx;
	}
	else
	{
		if(Infrared_Decode_Error_Cnt<0xffff)//不满1则自加
		{
			Infrared_Decode_Error_Cnt++;
		}
	}
	if((Infrared_Waiting_First_First_Cnt<1000)&&(Infrared_Waiting_First_Second_Cnt<1000)&&(Infrared_Waiting_First_Third_Cnt<1000))
	{
		jiaoyan=Infrary_First_Data_Third;//取第三分帧低8位作为校验值
		
		sum=0;
		temp1=Infrary_First_Data_First>>16;
		sum+=temp1;
		temp1=Infrary_First_Data_First>>8;
		sum+=temp1;
		temp1=Infrary_First_Data_First;
		sum+=temp1;
		temp1=Infrary_First_Data_Second>>16;
		sum+=temp1;
		temp1=Infrary_First_Data_Second>>8;
		sum+=temp1;
		temp1=Infrary_First_Data_Second;
		sum+=temp1;
		temp1=Infrary_First_Data_Third>>16;
		sum+=temp1;
		temp1=Infrary_First_Data_Third>>8;
		sum+=temp1;

		if(jiaoyan==sum)//三个分帧共同校验成功
		{
			Infrared_Waiting_First_Cnt=0;//三个分帧数据即时性均满足要求（1s以内的数据新鲜度），可认为第一类帧已完整接收
			if(Charger_State&0x80)//取最高位车辆检测标志，已检测到车辆则不做修改
			{
				;
			}
			else//未检测到车辆，则读取编号，设置车辆检测状态值
			{
				
				Bike_ID=0;
				temp1=Infrary_First_Data_Second>>16;
				Bike_ID+=temp1;
				Bike_ID<<=8;
				temp1=Infrary_First_Data_First;
				Bike_ID+=temp1;
				Bike_ID<<=8;
				temp1=Infrary_First_Data_First>>8;
				Bike_ID+=temp1;
				Bike_ID<<=8;
				temp1=Infrary_First_Data_First>>16;
				Bike_ID+=temp1;
				
				temp1=Infrary_First_Data_Second;
				Vot_Limit_Rec=(temp1&0xf0)*16;
				Cur_Limit_Rec=(temp1&0x0f)*256;
				temp1=Infrary_First_Data_Second>>8;
				Vot_Limit_Rec+=temp1;
				temp1=Infrary_First_Data_Third>>16;
				Cur_Limit_Rec+=temp1;
				
				WorkMode_Rec=Infrary_First_Data_Third>>8;
				
				Charger_State|=0x80;//设置为已检测到车辆
			}		
		}		
	}
}

void Decode_Second(void)
{
	uint8_t temp1,temp2;
	
	LED3_TOGGLE();
	
	Data_Jiaoyan=(RmtRecTx>>20)&0xf0;//取出25-28位数据
	
	temp1=0;temp2=RmtRecTx;temp1+=temp2;
	temp2=RmtRecTx>>8;temp1+=temp2;	
	temp2=RmtRecTx>>16;temp1+=temp2;
	temp1&=0xf0;
	
	if(Data_Jiaoyan==temp1)
	{
		//校验成功，解析数据
		Vot_Rec_Buck_Out=RmtRecTx&0xfff;
		Cur_Rec_Buck=(RmtRecTx>>12)&0xfff;
		Infrared_Waiting_Second_Cnt=0;//清零第二类帧计数
	}
	else
	{
		if(Infrared_Decode_Error_Cnt<0xffff)//不满1则自加
		{
			Infrared_Decode_Error_Cnt++;
		}
	}	
}

void Decode_Third(void)
{
	uint8_t temp1,temp2;
	
	LED4_TOGGLE();
	
	Data_Jiaoyan=(RmtRecTx>>20)&0xf0;//取出25-28位数据
	
	temp1=0;temp2=RmtRecTx;temp1+=temp2;
	temp2=RmtRecTx>>8;temp1+=temp2;	
	temp2=RmtRecTx>>16;temp1+=temp2;
	temp1&=0xf0;
	
	if(Data_Jiaoyan==temp1)
	{
		//校验成功，解析数据
		Vot_Rec_Buck_In=RmtRecTx&0xfff;
		Duty_Rec_Buck=(RmtRecTx>>12)&0xfff;
		Infrared_Waiting_Third_Cnt=0;//清零第三类帧计数
		if(Infrared_Waiting_Second_Cnt<1000)//判断第二类帧的实时性
		{
			Infrared_Charging_Waiting_Cnt=0;//清零红外等待计数
			Flag_Infrared_Data_OK=1;//红外数据OK，可进行一次控制
		}
	}
	else
	{
		if(Infrared_Decode_Error_Cnt<0xffff)//不满1则自加
		{
			Infrared_Decode_Error_Cnt++;
		}
	}
}

void Decode_Fault(void)
 {
	;
}
//全部信息打包
void All_Data_Trans_Packing(void)
{
	uint8_t i,temp;
	uint16_t Jiaoyan;
	
	All_Data_Package[0]=0xaa;//代表一帧的开始
	All_Data_Package[1]=0x0f;//代表该帧数据为单体桩全部信息帧
	All_Data_Package[2]=Charger_ID;
	All_Data_Package[3]=~Charger_ID;
	All_Data_Package[4]=Charger_State;
	All_Data_Package[5]=~Charger_State;
	All_Data_Package[6]=Vot_Trans_Buck_In;
	All_Data_Package[7]=Duty_Trans_Buck;
	All_Data_Package[8]=(Vot_Trans_Buck_In>>4)&0xf0;//取输入电压9-12位，放入高4位
	All_Data_Package[8]+=(Duty_Trans_Buck>>8)&0x0f;//取发射端占空比9-12位，放入低4位
	All_Data_Package[9]=Vot_Trans_Buck_Out;
	All_Data_Package[10]=Cur_Trans_Buck;
	All_Data_Package[11]=(Vot_Trans_Buck_Out>>4)&0xf0;//取输出电压9-12位，放入高4位
	All_Data_Package[11]+=(Cur_Trans_Buck>>8)&0x0f;//取输出电流9-12位，放入低4位
	All_Data_Package[12]=Relay_State;
	All_Data_Package[13]=Temp_Trans_MOS;
	
	Fre_Trans_Inverter=Freq_Value;
	All_Data_Package[14]=Fre_Trans_Inverter;
	All_Data_Package[15]=Fre_Trans_Inverter>>8;
	All_Data_Package[16]=(Fre_Trans_Inverter>>12)&0xf0;//取频率的17-20位，放入高4位
	All_Data_Package[16]+=(Temp_Trans_MOS>>8)&0x0f;//取温度的9-12位，放入低4位
	
	All_Data_Package[17]=Infrared_Rec_Succ_Cnt;
	All_Data_Package[18]=Infrared_Rec_Succ_Cnt>>8;
	All_Data_Package[19]=Infrared_Rec_Succ_Cnt>>16;
	All_Data_Package[20]=Infrared_Rec_Succ_Cnt>>24;
	All_Data_Package[21]=Infrared_Decode_Error_Cnt;
	All_Data_Package[22]=Infrared_Decode_Error_Cnt>>8;
	All_Data_Package[23]=Infrared_Waiting_First_Cnt;
	All_Data_Package[24]=Infrared_Waiting_First_Cnt>>8;
	All_Data_Package[25]=Infrared_Waiting_First_Cnt>>16;
	All_Data_Package[26]=Infrared_Waiting_First_Cnt>>24;
	All_Data_Package[27]=Infrared_Waiting_Second_Cnt;
	All_Data_Package[28]=Infrared_Waiting_Second_Cnt>>8;
	All_Data_Package[29]=Infrared_Waiting_Second_Cnt>>16;
	All_Data_Package[30]=Infrared_Waiting_Second_Cnt>>24;
	All_Data_Package[31]=Infrared_Waiting_Third_Cnt;
	All_Data_Package[32]=Infrared_Waiting_Third_Cnt>>8;
	All_Data_Package[33]=Infrared_Waiting_Third_Cnt>>16;
	All_Data_Package[34]=Infrared_Waiting_Third_Cnt>>24;
	All_Data_Package[35]=Infrared_Rec_Error_Cnt;
	All_Data_Package[36]=Infrared_Rec_Error_Cnt>>8;
	
	All_Data_Package[37]=Vot_Limit_Trans;
	All_Data_Package[38]=Cur_Limit_Trans;
	All_Data_Package[39]=(Vot_Limit_Trans>>4)&0xf0;//取发射端电压限值9-12位，放入高4位
	All_Data_Package[39]+=(Cur_Limit_Trans>>8)&0x0f;//取发射端电流限值9-12位，放入低4位
	All_Data_Package[40]=Fault_Type;
	All_Data_Package[41]=Bike_ID;
	All_Data_Package[42]=Bike_ID>>8;
	All_Data_Package[43]=Bike_ID>>16;
	All_Data_Package[44]=Bike_ID>>24;
	All_Data_Package[45]=Vot_Limit_Rec;
	All_Data_Package[46]=Cur_Limit_Rec;
	All_Data_Package[47]=(Vot_Limit_Rec>>4)&0xf0;//取电压限值9-12位，放入高4位
	All_Data_Package[47]+=(Cur_Limit_Rec>>8)&0x0f;//取电流限值9-12位，放入低4位
	All_Data_Package[48]=Vot_Rec_Buck_In;
	All_Data_Package[49]=Duty_Rec_Buck;
	All_Data_Package[50]=(Vot_Rec_Buck_In>>4)&0xf0;//取接收端输入电压值9-12位，放入高4位
	All_Data_Package[50]+=(Duty_Rec_Buck>>8)&0x0f;//取接收端占空比9-12位，放入低4位
	All_Data_Package[51]=Vot_Rec_Buck_Out;
	All_Data_Package[52]=Cur_Rec_Buck;
	All_Data_Package[53]=(Vot_Rec_Buck_Out>>4)&0xf0;//取接收端输出电压值9-12位，放入高4位
	All_Data_Package[53]+=(Cur_Rec_Buck>>8)&0x0f;//取接收端输出电流9-12位，放入低4位
	All_Data_Package[54]=Temp_Rec_Battery;
	All_Data_Package[55]=(Temp_Rec_Battery>>4)&0xf0;//取电池温度9-12位，放入高4位
	temp=WorkMode_Rec&0x0f;//只取低4位有效数据
	All_Data_Package[55]+=temp;
	
	Jiaoyan=0;
	for(i=6;i<56;i++)
	{
		Jiaoyan+=All_Data_Package[i];
	}
	All_Data_Package[56]=Jiaoyan;
	All_Data_Package[57]=Jiaoyan>>8;
}

//故障类型及现场数据打包
void Fault_Data_Trans_Packing(void)
{
	uint8_t i;
	uint16_t Jiaoyan;
	
	Fault_Data_Package[0]=0xaa;//代表一帧的开始
	Fault_Data_Package[1]=0x1E;//代表该帧数据为故障帧数据
	Fault_Data_Package[2]=Charger_ID;
	Fault_Data_Package[3]=~Charger_ID;
	Fault_Data_Package[4]=Fault_Type;
	Fault_Data_Package[5]=Vot_Limit_Trans;
	Fault_Data_Package[6]=Cur_Limit_Trans;
	Fault_Data_Package[7]=(Vot_Limit_Trans>>4)&0xf0;
	Fault_Data_Package[7]+=(Cur_Limit_Trans>>8)&0x0f;
	Fault_Data_Package[8]=Field_Vot_Trans_Buck_In;
	Fault_Data_Package[9]=Field_Duty_Trans_Buck;
	Fault_Data_Package[10]=(Field_Vot_Trans_Buck_In>>4)&0xf0;
	Fault_Data_Package[10]+=(Field_Duty_Trans_Buck>>8)&0x0f;
	Fault_Data_Package[11]=Field_Vot_Trans_Buck_Out;
	Fault_Data_Package[12]=Field_Cur_Trans_Buck;
	Fault_Data_Package[13]=(Field_Vot_Trans_Buck_Out>>4)&0xf0;
	Fault_Data_Package[13]+=(Field_Cur_Trans_Buck>>8)&0x0f;
	Fault_Data_Package[14]=Field_Temp_Trans_MOS;
	Field_Freq_Value=Freq_Value;
	Fault_Data_Package[15]=Field_Freq_Value;
	Fault_Data_Package[16]=Field_Freq_Value>>8;
	Fault_Data_Package[17]=(Field_Freq_Value>>12)&0xf0;
	Fault_Data_Package[17]+=(Field_Temp_Trans_MOS>>8)&0x0f;
	
	Fault_Data_Package[18]=Vot_Limit_Rec;
	Fault_Data_Package[19]=Cur_Limit_Rec;
	Fault_Data_Package[20]=(Vot_Limit_Rec>>4)&0xf0;
	Fault_Data_Package[20]+=(Cur_Limit_Rec>>8)&0x0f;
	
	Fault_Data_Package[21]=Field_Vot_Rec_Buck_In;
	Fault_Data_Package[22]=Field_Duty_Rec_Buck;
	Fault_Data_Package[23]=(Field_Vot_Rec_Buck_In>>4)&0xf0;
	Fault_Data_Package[23]+=(Field_Duty_Rec_Buck>>8)&0x0f;
	Fault_Data_Package[24]=Field_Vot_Rec_Buck_Out;
	Fault_Data_Package[25]=Field_Cur_Rec_Buck;
	Fault_Data_Package[26]=(Field_Vot_Rec_Buck_Out>>4)&0xf0;
	Fault_Data_Package[26]+=(Field_Cur_Rec_Buck>>8)&0x0f;
	Fault_Data_Package[27]=Temp_Rec_Battery;
	Fault_Data_Package[28]=(Temp_Rec_Battery>>8)&0x0f;
	//判断故障是否与接收端有关
	//
	Jiaoyan=0;
	for(i=4;i<29;i++)
	{
		Jiaoyan+=Fault_Data_Package[i];
	}
	Fault_Data_Package[29]=Jiaoyan;
	Fault_Data_Package[30]=Jiaoyan>>8;
}

//状态信息打包
void State_Data_Trans_Packing(void)
{
	uint8_t i,temp;
	
	State_Data_Package[0]=0xaa;//代表一帧的开始
	State_Data_Package[1]=0x2D;//代表该帧数据为状态帧数据
	State_Data_Package[2]=Charger_ID;
	State_Data_Package[3]=~Charger_ID;
	State_Data_Package[4]=Charger_State;
	State_Data_Package[5]=~Charger_State;
	State_Data_Package[6]=Bike_ID;
	State_Data_Package[7]=Bike_ID>>8;
	State_Data_Package[8]=Bike_ID>>16;
	State_Data_Package[9]=Bike_ID>>24;
	
	temp=0;
	for(i=6;i<10;i++)
	{
		temp+=State_Data_Package[i];
	}
	
	State_Data_Package[10]=temp;
}
//电压及电流限值信息打包
void Limit_Data_Trans_Packing(void)
{
	uint8_t i,Jiaoyan;
	
	Limit_Data_Package[0]=0xaa;//代表一阵的开始
	Limit_Data_Package[1]=0x3C;
	Limit_Data_Package[2]=Charger_ID;
	Limit_Data_Package[3]=~Charger_ID;
	Limit_Data_Package[4]=Vot_Limit_Rec;
	Limit_Data_Package[5]=(Vot_Limit_Rec>>8)&0x0f;
	Limit_Data_Package[6]=Cur_Limit_Rec;
	Limit_Data_Package[7]=(Cur_Limit_Rec>>8)&0x0f;
	Limit_Data_Package[8]=Relay_State;
	Limit_Data_Package[9]=~Relay_State;
	
	Jiaoyan=0;
	for(i=4;i<8;i++)
	{
		Jiaoyan+=Limit_Data_Package[i];
	}
	Limit_Data_Package[10]=Jiaoyan;
}
//频率信息打包
void Frequency_Data_Trans_Packing(void)
{
	uint8_t i,Jiaoyan;
	
	Frequency_Data_Package[0]=0xAA;
	Frequency_Data_Package[1]=0x4B;
	Frequency_Data_Package[2]=Charger_ID;
	Frequency_Data_Package[3]=~Charger_ID;
	Frequency_Data_Package[4]=Frequency_Scan_State;
	Frequency_Data_Package[5]=Cur_Trans_Buck;
	Frequency_Data_Package[6]=(Cur_Trans_Buck>>8)&0x0f;//取电流值的9-12位，放入低4位
	
	Fre_Trans_Inverter=Freq_Value-Frequency_Scan_Delt;
	Frequency_Data_Package[7]=Fre_Trans_Inverter;
	Frequency_Data_Package[8]=Fre_Trans_Inverter>>8;
	Frequency_Data_Package[9]=(Fre_Trans_Inverter>>16)&0x0f;//取频率值的17-20位，放入低4位
	
	Jiaoyan=0;
	for(i=5;i<10;i++)
	{
		Jiaoyan+=Frequency_Data_Package[i];
	}
	Frequency_Data_Package[10]=Jiaoyan;
}

void Frequency_Scan(void)
{
	//初始化扫频结果：最大电流值、最大电流值对应的频率值，都设为0
	Max_Cur_Trans_Buck=0;
	Max_Cur_Trans_Frequency=0;
	Frequency_Scan_State=0;//初始化扫频状态，未开始扫频
	
	for(Freq_Value=Frequency_Start;;)
	{
		TIM1->ARR=Freq_Value;
	  htim1.Instance->CCR2=Freq_Value/2;
	  HAL_TIM_Base_Start_IT(&htim1);
	  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
		HAL_Delay(50);
		if(Trans_OverCurrent_Alert)
		{
			Trans_OverCurrent_Alert_Read=1;//已阅读告警标志
			Freq_Value=0xffff;
			TIM1->ARR=Freq_Value;//重装载寄存器填满，降低频率
	    htim1.Instance->CCR2=0;
	    HAL_TIM_Base_Stop_IT(&htim1);
	    HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
			//中断中已关闭BUCK，此处无需再次关闭
			//Duty_Trans_Buck=0;
			//htim5.Instance->CCR1=Duty_Trans_Buck;//关闭BUCK
			Charger_Start_Permission=0;//允许充电标志置0，调整完毕后，再次下发指令
			Charger_State|=0x01;//出现问题事件，可修复，再次下发指令时清零
			if(Fault_Type<FaultLevel_CurOver_Short)
			{
				Fault_Type=FaultLevel_CurOver_Short;
				Fault_Field_Save();
			}
			Frequency_Scan_State=0x2d;
			Charger_State&=0x9F;//扫频结束，对第七和第六位进行清零
			return;//退出函数，避免死循环，不设置状态转换，主程序循环中不切换状态
		}
		else
		{
			if(Cur_Trans_Buck>Max_Cur_Trans_Buck)
			{
				Max_Cur_Trans_Buck=Cur_Trans_Buck;
				Max_Cur_Trans_Frequency=Freq_Value;
			}
			
			Frequency_Scan_State=0x1E;//正常扫频状态
			
			//判断扫频过流，如果扫频过流，说明停放距离较远，拒绝充电
			if(Cur_Trans_Buck>Frequency_Scan_CurLimit)
			{
				//该扫频过流是指：电流未达到很大的极限值，但超出正常扫频可能的值，出现的原因是车辆停放过远，或其他问题，近似无负载
				//LED1_TOGGLE();
				Frequency_Scan_State=0x2D;
				Freq_Value=0xffff;
				TIM1->ARR=Freq_Value;//将重装载寄存器填满，降低频率值
	      htim1.Instance->CCR2=0;
	      HAL_TIM_Base_Stop_IT(&htim1);
	      HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
				Duty_Trans_Buck=0;
				htim5.Instance->CCR1=Duty_Trans_Buck;//关闭BUCK
				Charger_State|=0x01;//出现问题事件（可修复），最低位置1，该标志位在每次下指令启动系统时清零
				if(Fault_Type<FaultLevel_FreqScanError)//扫频过流故障等级为4，在当前故障优先级小于4时，可更新故障类型及故障现场
				{
					Fault_Type=FaultLevel_FreqScanError;//要保存故障现场
					Fault_Field_Save();
				}
				Charger_Start_Permission=0;//清零充电允许标志，调整完毕后，重新下发指令再次开启
				Charger_State&=0x9F;//扫频结束，对第七和第六位进行清零
				return;
			}		
			Freq_Value+=Frequency_Scan_Delt;//降低频率值
			
			if(Freq_Value>Frequency_Stop)//表示正常完成了扫频
			{
				Frequency_Scan_State=0xf0;//0xf0表示扫频正常完成
				Charging_Preparing();
				return;
			}
		}
	}
}

void Frequency_Scan485(void)
{
	//初始化扫频结果：最大电流值、最大电流值对应的频率值，都设为0
	Max_Cur_Trans_Buck=0;
	Max_Cur_Trans_Frequency=0;
	Frequency_Scan_State=0;//初始化扫频状态，未开始扫频
	
	for(Freq_Value=Frequency_Start;;)
	{
		TIM1->ARR=Freq_Value;
	  htim1.Instance->CCR2=Freq_Value/2;
	  HAL_TIM_Base_Start_IT(&htim1);
	  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
		HAL_Delay(50);

		if(Trans_OverCurrent_Alert)
		{
			Trans_OverCurrent_Alert_Read=1;//已阅读告警标志
			Freq_Value=0xffff;
			TIM1->ARR=Freq_Value;//将重装载寄存器填满，降低频率值
	    htim1.Instance->CCR2=0;
	    HAL_TIM_Base_Stop_IT(&htim1);
	    HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
			//中断中已关闭BUCK，此处无需再次关闭
			//Duty_Trans_Buck=0;
			//htim5.Instance->CCR1=Duty_Trans_Buck;//关闭BUCK
			Charger_Start_Permission=0;//允许充电标志置0，调整完毕后，再次下发指令
			Charger_State|=0x01;//出现问题事件，可修复，再次下发指令时清零
			if(Fault_Type<FaultLevel_CurOver_Short)
			{
				Fault_Type=FaultLevel_CurOver_Short;
				Fault_Field_Save();
			}
			Frequency_Scan_State=0x2d;//设置扫频出错标志
			Trans_485_En();
			Frequency_Data_Trans_Packing();
			HAL_UART_Transmit_DMA(&huart3,Frequency_Data_Package,sizeof(Frequency_Data_Package));
			return;//退出函数，避免死循环，不设置状态转换，主程序循环中不切换状态
		}
		else
		{
			if(Cur_Trans_Buck>Max_Cur_Trans_Buck)
			{
				Max_Cur_Trans_Buck=Cur_Trans_Buck;
				Max_Cur_Trans_Frequency=Freq_Value;
			}
			
			Frequency_Scan_State=0x1E;//正常扫频状态
			
			//判断扫频过流，如果扫频过流，说明停放距离较远，拒绝充电
			if(Cur_Trans_Buck>Frequency_Scan_CurLimit)
			{
				//该扫频过流是指：电流未达到很大的极限值，但超出正常扫频可能的值，出现的原因是车辆停放过远，或其他问题，近似无负载
				Frequency_Scan_State=0x2D;//设置扫频出错标志
				Freq_Value=0xffff;
				TIM1->ARR=Freq_Value;//将重装载寄存器填满，降低频率值
	      htim1.Instance->CCR2=0;
	      HAL_TIM_Base_Stop_IT(&htim1);
	      HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
				Duty_Trans_Buck=0;
				htim5.Instance->CCR1=Duty_Trans_Buck;//关闭BUCK
				Charger_State|=0x01;//出现问题事件（可修复），最低位置1，该标志位在每次下指令启动系统时清零
				if(Fault_Type<FaultLevel_FreqScanError)//扫频过流故障等级为4，在当前故障优先级小于4时，可更新故障类型及故障现场
				{
					Fault_Type=FaultLevel_FreqScanError;//要保存故障现场
					Fault_Field_Save();
				}		
				//Charger_Start_Permission=0;//清零充电允许标志，调整完毕后，重新下发指令再次开启
				//该扫频是指令扫频，充电允许未开启，似乎无需清零充电允许标志
				Trans_485_En();
				Frequency_Data_Trans_Packing();
				HAL_UART_Transmit_DMA(&huart3,Frequency_Data_Package,sizeof(Frequency_Data_Package));
				return;
			}		
			Freq_Value+=Frequency_Scan_Delt;//增加频率值
			
			if(Freq_Value>Frequency_Stop)//表示正常完成了扫频
			{
				Frequency_Scan_State=0xf0;//0xf0表示扫频正常完成
			}
			Trans_485_En();
			Frequency_Data_Trans_Packing();
			HAL_UART_Transmit_DMA(&huart3,Frequency_Data_Package,sizeof(Frequency_Data_Package));
			if(Frequency_Scan_State==0xf0)
				return;
		}
	}
}

//PWM输出比较中断，开启AD采样
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM5)
	{
		HAL_ADC_Start_DMA(&hadc1,ADC_ConvertedValue,4);
	}
	__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_CC1);
}
//AD完成中断
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//获得采样值
	Temp_Trans_MOS=ADC_ConvertedValue[0]&0xfff;
	Vot_Trans_Buck_In=ADC_ConvertedValue[1]&0xfff;
	Cur_Trans_Buck=ADC_ConvertedValue[2]&0xfff;
	Vot_Trans_Buck_Out=ADC_ConvertedValue[3]&0xfff;
	Power_Trans=Vot_Trans_Buck_Out*Cur_Trans_Buck;//先把整型转化为double型
	Power_Trans=Power_Trans*K_Power_Trans;//进行比例换算
	
	if((Charger_State&0x60)==0x60)//只有处于充电态才执行占空比调节
	{
		if(Power_Trans>Power_Trans_Refer)
		{
			if(Duty_Trans_Buck>2)
				Duty_Trans_Buck-=2;//为防止在纹波条件下功率超调，降功率的调节可适当增加幅度
			else
				Duty_Trans_Buck=0;
			htim5.Instance->CCR1=Duty_Trans_Buck;
		}
		else
		{
			if(Vot_Trans_Buck_Out<4000)//防止阻抗过大时，输出电压过高，达到AD满值，影响功率计算，即防止电压过大
			{				
				if(Cur_Trans_Buck<3440)//4A--电流不能过大
				{
					if(Duty_Trans_Buck<3600)
						Duty_Trans_Buck+=1;
				}	
				else if(Cur_Trans_Buck<4000)//4.5A
				{
					if(Duty_Trans_Buck>0)
						Duty_Trans_Buck-=1;
				}
				else
				{
					Duty_Trans_Buck/=2;
				}
			}
			else
			{
				if(Duty_Trans_Buck>0)
					Duty_Trans_Buck-=1;
				
				if(Duty_Trans_Buck<20)//充电态，BUCK值很小，输出电压很大，可认为是BUCK烧穿
				{
					Freq_Value=0xffff;
					TIM1->ARR=Freq_Value;//重装载寄存器填满，降低频率
					htim1.Instance->CCR2=0;
					HAL_TIM_Base_Stop_IT(&htim1);
					HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
					Charger_State|=0x10;//0001 0000，BUCK烧穿，对应发射端故障事件，第5位置1
					if(Fault_Type<FaultLevel_Trans_Buck_Short)
					{
						Fault_Type=FaultLevel_Trans_Buck_Short;
						Fault_Field_Save();
					}
					
					Charger_Start_Permission=0;//允许充电置0，再次充电则再次下发指令
					Charger_State&=0x1f;//&=0001 1111，清零第6-7位，设为停止态，同时清零车辆检测标志，等待红外编号再次到来
				}
			}							
			htim5.Instance->CCR1=Duty_Trans_Buck;
		}		
	}
	
	if(Cur_Trans_Buck>4000)//告警临界值需要再次确定，AD的电流放大比例也需要再次确认
	{
		htim5.Instance->CCR1=0;//关闭BUCK
		Trans_OverCurrent_Alert=1;//电流过大，发出告警
		Duty_Trans_Buck=0;
	}
	if(Cur_Trans_Buck<3600)
	{
		//电流值下降到可控范围，且告警标志已被主程序阅读处理后，应取消告警标志
		if(Trans_OverCurrent_Alert_Read==1)
		{
			Trans_OverCurrent_Alert_Read=0;
			Trans_OverCurrent_Alert=0;
		}
	}	
}

void Charging_Preparing()
{
	Power_Trans_Refer=Power_Trans_Refer_Waiting;
	
	//设置频率值
	Freq_Value=Max_Cur_Trans_Frequency-Frequency_Waiting_Delt;
  TIM1->ARR=Freq_Value;
	htim1.Instance->CCR2=Freq_Value/2;
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
	
	//设置状态转换，转换为充电状态
	Charger_State|=0x60;//第七和第六位置1，|=0110 0000
	//设置红外等待计数，一开始就处于等待红外充电信息状态，当接收到充电信息后，Infrared_Charging_Waiting_Cnt自会被清零
	Infrared_Charging_Waiting_Cnt=3001;
	
	//充电完成计数清零
	Charging_ComptCnt=0;
	//低效率计数清零
	Low_Efficiency_Cnt=0;
}

void Charging_Stopping()
{
	Charger_Start_Permission=0;//允许充电置0，再次充电则再次下发指令
	Charger_State&=0x1f;//&=0001 1111，清零第6-7位，设为停止态，同时清零车辆检测标志，等待红外编号再次到来
	
	//占空比设为0
	Duty_Trans_Buck=0;
	htim5.Instance->CCR1=Duty_Trans_Buck;
	HAL_Delay(50);
	//频率设为0
	Freq_Value=0xffff;
	TIM1->ARR=Freq_Value;//重装载寄存器填满，降低频率
	htim1.Instance->CCR2=0;
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
	
	
	
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
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address,Charger_ID);
	HAL_FLASH_Lock();
}
//故障现场保护
void Fault_Field_Save()
{
	Field_Vot_Trans_Buck_In=Vot_Trans_Buck_In;
	Field_Duty_Trans_Buck=Duty_Trans_Buck;
	Field_Vot_Trans_Buck_Out=Vot_Trans_Buck_Out;
	Field_Cur_Trans_Buck=Cur_Trans_Buck;
	Field_Temp_Trans_MOS=Temp_Trans_MOS;
	Field_Freq_Value=Freq_Value;
	
	Field_Vot_Rec_Buck_In=Vot_Rec_Buck_In;
	Field_Vot_Rec_Buck_Out=Vot_Rec_Buck_Out;
	Field_Cur_Rec_Buck=Cur_Rec_Buck;
	Field_Temp_Rec_Battery=Temp_Rec_Battery;
	
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
