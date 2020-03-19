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

//�����ǳ����Ƶ���ز���
uint32_t ADC_ConvertedValue[4];//AD��������洢
uint16_t Vot_Limit_Rec,Cur_Limit_Rec,Temp_Limit_Rec;
uint16_t Vot_Rec_Buck_In,Duty_Rec_Buck,Vot_Rec_Buck_Out,Cur_Rec_Buck,Temp_Rec_Battery;
uint16_t WorkMode;//����WorkMode����1-2λ�洢�¶ȿ���ģʽ��00��ʾʧ�ܣ�11��ʾʹ�ܣ�10��01������
//����WorkMode����3-4λ�洢BUCK����ģʽ��00��ʾ����PI���ƣ�11��ʾȫ�أ�10��ʾȫ����01������
uint8_t SystemState;
float K_VotStart,K_VotStop;//ͨ��BUCK�����ѹ�����ת�����ն�״̬������Kֵ�������ѹ�������ѹ�ı�ֵ
uint8_t Flag_WorkMode_Conversion;
uint16_t TimeCnt_WorkMode_Conversion;
uint16_t Delt_VotStart,Delt_VotStop;
/*******************************************************
SystemState���Ծ�������״̬�л����ݶ������£�
0��δ���״̬��1���������״̬��ֻ��������״̬���м�״̬�������⴦��
*******************************************************/

float Cur_Kp,Cur_Ki,Vot_Kp,Vot_Ki;//PI���ƵĲ���

//�����Ǻ��ⷢ�����ز���
uint8_t Flag1,Flag2,BitCnt,Flag_Infray_Done,Jiaoyan_Infray;//��������У���ã���4λ����;
uint8_t Flag_Zhen_No,Flag_Infray_WaitTime;
//ֻ��Flag_Infray_Done��־Ϊ1ʱ��������������ź�
uint32_t Infray_TransDat;//Ҫ���͵�����
uint16_t Period[32];//�洢ÿһλ���ݶ�Ӧ��Pulse��Ϣ
uint16_t TimeCnt_Infray;//���Զ�ʱ3���ӣ�������̬��ÿ��3���ӣ�����һ�κ��⣬һ�η���4֡��ѭ��2��
uint8_t Cnt_InfrayTrans;//���Լ�¼���ⷢ��ѭ���˼���

//�����ǳ���������Ϣ
uint32_t Bike_ID;

//�����Ǵ��ڿ�����Ϣ
uint8_t Flag_StartByte;//�����Ƿ��⵽��ͷ
uint8_t Byte_Cnt;//�Խ����ֽڵ��������м���
uint8_t Data_ReceArray[6];//�洢���յ��Ĵ������ݣ�6���ֽ�Ϊһ�飬��ʱδ���������ȫ������
uint8_t Data_ReceSingle;//���������ֽ�
uint8_t Flag_Trans_Done,Flag_Rece_Done,TimeCnt485;
uint8_t SystInfo_Package[18];
uint16_t Jiaoyan485;

//������FLASH�����Ϣ
uint32_t ReadFlashData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//���ⷢ�亯�����β�ΪҪ���͵�32λ��ֵ
void Ir_Tx(uint32_t Dat);
//���������Ϣ�������
void Infray_InfoPackage_First_First(void);//�����һ��֡��һ��֡
void Infray_InfoPackage_First_Second(void);//�����һ��֡�ڶ���֡
void Infray_InfoPackage_First_Third(void);//�����һ��֡������֡
void Infray_InfoPackage_Second(void);//����ڶ���֡
void Infray_InfoPackage_Third(void);//���������֡
//FLASH��д����
uint32_t ReadFlash(uint32_t Address);//��ָ��λ���ϵ���ֵ��һ�ζ�ȡ4���ֽ�
void WriteFlash(uint32_t Address);//��ָ����ַ��д���ݣ����ֲ�������һ��д�������ֽ�
//������ʼ������
void Variable_Init(void);
//ȫ����Ϣ��������ͺ���
void Info_Trans(void);
void Info_Package(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define FLASH_RW_StartAddress ((uint32_t) 0x0803F804)  //FLASH��127ҳ��ʼ��ַ���ƫ��4���ֽ�
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
	/*********************************������������*********************************/
	
	//���ڳ����Ƶĵ�ѹ��ֵ�͵�����ֵ��ͨ����λ������·�ָ������޸�
	//�����Ƶ���������ΪVot_Limit_Rec��Cur_Limit_Rec����AD�ж���ʹ�ã����е�ѹ��������
	//����״̬ת�����漰��4��������float K_VotStart,K_VotStop�Լ�uint16_t Delt_VotStart,Delt_VotStop;
	//��δ���״̬ʱ����⵽�����ѹVot_Rec_Buck_In����K_VotStart*(0.465*Vot_Rec_Buck_Out-65+Delt_VotStart)ʱ����ʼ�ߵ�ѹ��ʱ���ߵ�ѹ��ʱ����1s��ת��״̬
	//K_VotStart=0.85,Delt_VotStart=100
	//��ʱ�����У���⵽Vot_Rec_Buck_InС��K_VotStart*(0.465*Vot_Rec_Buck_Out-65)ʱ�������ʱ��־�����¼��
	//���������״̬ʱ����⵽�����ѹVot_Rec_Buck_InС��K_VotStop*��0.465*Vot_Rec_Buck_Out-Delt_VotStop��ʱ����ʼ�͵�ѹ��ʱ���͵�ѹ��ʱ����30s��ת��״̬
	//K_VotStop=0.7,Delt_VotStop=100
	//��ʱ�����У���⵽Vot_Rec_Buck_In����0.465*Vot_Rec_Buck_Out*K_VotStopʱ�������ʱ��־�����¼��
	K_VotStart=0.55;Delt_VotStart=100;//����Delt_VotStart��Delt_VotStop��Ҫ�Ǳ��������׼�Ͷ��������������
	K_VotStop=0.25;Delt_VotStop=100;
	//���ڿ����¶ȱ����󣬵��¶ȸ���һ��ֵ��ϵͳ��������������ֹ��磬�ϱ����ϣ����¶ȱ�������ֵ��Temp_Limit_Recȷ������ֵԽС���¶�Խ��
	Temp_Limit_Rec=620;

	/*********************************������������*********************************/
	
		
	//������ʼ����������ѹ/�����޶�ֵ������ģʽ������ID�Ķ�ȡ��״̬�������趨��PI���Ʋ����ĳ�ʼ��
	Variable_Init();
		
	Rece_485_En();
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
	
	//����PI���ƣ���û���������ʱ�����Զ�����Duty_Rec_Buck������
	//����BUCK�������ܹ���Ч���������ѹ���ͣ����豸��ȫ��Ϊ��Ҫ����ˣ�����״̬�£�������ر�BUCK���ƣ����ǲ���ָ���е�ȫ��ָ��
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_4);
	
	Flag_Infray_Done=1;//�������⹦��
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		
		if(Flag_StartByte)
		{
			if(TimeCnt485>50)//�Ѽ�⵽��ͷ����50ms��δ�������6���ֽڣ���λ���½���
			{
				Flag_StartByte=0;
				TimeCnt485=0;
				Byte_Cnt=0;
			}
		}
		
		if(Flag_Rece_Done)//�������6���ֽڵ���Ч���ݣ����д���
		{
			Flag_Rece_Done=0;
			sum=0;//���ֵ������
			for(ss=1;ss<5;ss++)
			{
				sum+=Data_ReceArray[ss];
			}
			if(sum==Data_ReceArray[5])//�ж�У�飬У��ʧ�������κδ���
			{
				switch(Data_ReceArray[0])
				{
					case 0x0F: 
					{
						//�������ն����е�·��Ϣ
						Info_Trans();
						break;
					}
					case 0x1E: 
					{
						//�޸ĵ�ѹ��ֵ��������ֵ��д��FLASH��Ȼ�������е�·��Ϣ
						Vot_Limit_Rec=Data_ReceArray[1]+(Data_ReceArray[3]&0xf0)*16;
						Cur_Limit_Rec=Data_ReceArray[2]+(Data_ReceArray[3]&0x0f)*256;
						WriteFlash(FLASH_RW_StartAddress);
						Info_Trans();						
						break;
					}
					case 0x2D: 
					{
						//�޸�BUCK����ģʽΪȫ����Ȼ�������е�·��Ϣ
						WorkMode|=0x08;//�Ե�4λ��1
						WorkMode&=0xfB;//�Ե�3λ��0
						htim1.Instance->CCR4=3600;
						Info_Trans();
						break;
					}
					case 0x3C: 
					{
						//�޸�BUCK����ģʽΪȫ�أ�Ȼ�������е�·��Ϣ
						WorkMode|=0x0C;//�Ե�3-4λ��1
						htim1.Instance->CCR4=0;
						Info_Trans();
						break;
					}
					case 0x4B: 
					{
						//�ָ�BUCK����ģʽΪPI���ƣ�Ȼ�������е�·��Ϣ
						WorkMode&=0xf3;//�Ե�3-4λ����
						Info_Trans();
						break;
					}
					case 0x5A: 
					{
						//�޸ĳ����豸��ţ�д��FLASH��Ȼ�������е�·��Ϣ
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
						//ʧ���¶ȿ��ƣ�д��FLASH��Ȼ�������е�·��Ϣ
						WorkMode&=0xfc;//��1-2λ����
						WriteFlash(FLASH_RW_StartAddress);
						Info_Trans();
						break;
					}
					case 0x78: 
					{
						//ʹ���¶ȿ��ƣ�д��FLASH��Ȼ�������е�·��Ϣ
						WorkMode|=0x03;//��1-2λ��1
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
				//������̬����Ҫ���ͺ����һ��֡���ڶ���֡����4֡�����3�뷢��һ��
				//����״̬���л��������ѹ���
				
				if(Flag_Infray_WaitTime)
				{
					//Flag_Zhen_No��ת���ڶ�ʱ���ж���ʵ��
					if(Flag_Infray_Done)
					{
						Flag_Infray_Done=0;
						HAL_Delay(100);
						if(Flag_Zhen_No==0)//���͵�һ��֡��һ��֡
						{
							if(Cnt_InfrayTrans)
								Infray_InfoPackage_First_First();
							Ir_Tx(Infray_TransDat);							
						}
						else if(Flag_Zhen_No==1)//���͵�һ��֡�ڶ���֡
						{
							if(Cnt_InfrayTrans)
								Infray_InfoPackage_First_Second();
							Ir_Tx(Infray_TransDat);
						}
						else if(Flag_Zhen_No==2)//���͵�һ��֡������֡
						{
							if(Cnt_InfrayTrans)
								Infray_InfoPackage_First_Third();
							Ir_Tx(Infray_TransDat);
						}
						else if(Flag_Zhen_No==3)//���͵ڶ���֡
						{
							if(Cnt_InfrayTrans)
								Infray_InfoPackage_Second();
							Ir_Tx(Infray_TransDat);
							if(Cnt_InfrayTrans)
								Cnt_InfrayTrans--;//Flag_Zhen_No��ʼ����0��ʼ�������������֡��ѭ��������1
							else
								Flag_Infray_WaitTime=0;//SystTick����1�������������֡������ѭ������2������㣬���������һ�η��ͣ��ȴ��´ζ�ʱʱ�䵽�ٴ���1
						}
					}
				}
				if(Vot_Rec_Buck_In>(K_VotStart*0.465*Vot_Rec_Buck_Out+Delt_VotStart))//��������ѹ����״̬�л�
				{
					if(!Flag_WorkMode_Conversion)//�״μ�⵽�����ѹ�����趨��ֵ
					{
						Flag_WorkMode_Conversion=1;//���ñ�־λ
					}
					else//֮ǰ�Ѽ�⵽�����ѹ���ߣ��˴��Դ��ڸߵ�ѹ��ʱ����м�ʱ�ж�
					{
						if(Flag_WorkMode_Conversion>2)////ֵ-1������ʱ���������˴��ж��Ƿ��ѹ����״̬�Ƿ��ѳ���1s
						{
							SystemState=1;
							Flag_WorkMode_Conversion=0;
							TimeCnt_WorkMode_Conversion=0;
						}//�������ʱ��δ�����򲻶���
					}
				}
				//��ѹֵ���ߣ�������Flag_WorkMode_Conversion��־λ������ʼ����
				//��ѹֵ���ͣ��������־λ�����¼�⣬ֻ�е�ѹֵ�������ߣ��Ż�ʹ��Flag_WorkMode_Conversion��������
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
					if(Flag_Zhen_No==0)//���͵ڶ���֡
					{
						Infray_InfoPackage_Second();
						Ir_Tx(Infray_TransDat);
					}
					else//���͵�����֡
					{
						Infray_InfoPackage_Third();
						Ir_Tx(Infray_TransDat);
					}
				}
				if(Vot_Rec_Buck_In<200)//��������ѹ������״̬�л�
				{
					if(!Flag_WorkMode_Conversion)//�״μ�⵽�����ѹС���趨��ֵ
					{
						LED2_TOGGLE();
						Flag_WorkMode_Conversion=1;//���ñ�־λ
					}
					else//֮ǰ�Ѽ�⵽�����ѹ���ͣ��˴��Դ��ڵ͵�ѹ��ʱ����м�ʱ�ж�
					{
						if(Flag_WorkMode_Conversion>6)//ֵ-1������ʱ���������˴��ж��Ƿ��ѹ����״̬�Ƿ��ѳ���30s
						{							
							SystemState=0;//����30s���ͣ���ת��״̬��������������
							Flag_WorkMode_Conversion=0;
							TimeCnt_WorkMode_Conversion=0;
							
							//�趨����Ҫ��������
							Flag_Zhen_No=0;
							Flag_Infray_Done=1;
							Flag_Infray_WaitTime=1;
							
						}//���ʱ��δ��30s���򲻶���
					}
				}
				//��ѹֵ���ͣ�������Flag_WorkMode_Conversion��־λ������ʼ����
				//��ѹֵ���ߣ��������־λ�����¼�⣬ֻ�е�ѹֵ�������ͣ��Ż�ʹ��Flag_WorkMode_Conversion��������
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
//SystemTick 1ms�ж�
void HAL_SYSTICK_Callback(void)
{
	//485ͨ����ؼ�ʱ����־
	if(Flag_StartByte)//��⵽485��ͷ��������ʱ����ʱδ�����꣬������ȫ�����յ������ݣ����¼����ͷ
	{
		if(TimeCnt485<255)
		{
			TimeCnt485++;
		}
	}
	//���ⷢ����ؼ�ʱ����־
	if(SystemState==0)//ֻ������̬��Ҫ�ṩ��ʱ��Ϣ������״̬��������������
	{
		TimeCnt_Infray++;
		if(TimeCnt_Infray>3000)//ÿ��3���ṩһ�ζ�ʱ��Ϣ
		{
			TimeCnt_Infray=0;
			Flag_Infray_WaitTime=1;//��ʱʱ�䵽��־��1
			Cnt_InfrayTrans=1;//ѭ�������趨Ϊ2�Σ������������ȷ��ͼ�1
		}
	}
	//����״̬ת����ؼ�ʱ����־
	if(Flag_WorkMode_Conversion)
	{
		TimeCnt_WorkMode_Conversion++;
		if(TimeCnt_WorkMode_Conversion>1000)//��ʱ�ﵽ1s
		{
			TimeCnt_WorkMode_Conversion=0;
			Flag_WorkMode_Conversion++;
		}
	}
}
//���ⷢ�亯������
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

//FLASH�����ݺ���
uint32_t ReadFlash(uint32_t Address)
{
	uint32_t Result;
	Result=*(__IO uint32_t*)(Address);
	return Result;
}
//FLASHд���ݺ���
void WriteFlash(uint32_t Address)
{
	//FLASH�洢����������Stm32F103RCT6�����ڴ��СΪ256K����128ҳ��ÿҳ2K	
	FLASH_EraseInitTypeDef Flash;
	uint32_t PageError;
	
	HAL_FLASH_Unlock();//����д����
	//�����������
	Flash.TypeErase=FLASH_TYPEERASE_PAGES;
	Flash.PageAddress=FLASH_RW_StartAddress;
	Flash.NbPages=1;
	//����PageError
	PageError=0;//0xFFFFFFFF means that all the pages have been correctly erased
	//���ò�������
	HAL_FLASHEx_Erase(&Flash,&PageError);
	//��дFLASH
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address,Vot_Limit_Rec);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address+2,Cur_Limit_Rec);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Address+4,Bike_ID);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address+8,WorkMode);
	HAL_FLASH_Lock();
}

//���ⷢ����жϴ���
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//��ʱ�������¼��Ļص�����
{
	if(htim->Instance==TIM1)
	{
		//TIM1��BUCK���Ƶ��жϣ���ʱ��������
	}
	//���ƺ����TIM3δ���жϣ���˲���Ҫ����
	if(htim->Instance==TIM7)
	{
		if(Flag1==0)//�������������뷢��
		{
			if(Flag2==0)//���������PWM���䣬��Ҫ�ض�һ��ʱ��
			{
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
				Flag2=1;				
				__HAL_TIM_SET_AUTORELOAD(&htim7,4500);	
			}
			else//����������ߵ͵�ƽ�������
			{
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
				Flag1=1;Flag2=0;				
				__HAL_TIM_SET_AUTORELOAD(&htim7,560);//�ز�560us
			}
		}
		else//��������뷢��
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
					Flag_Infray_Done=1;//���ⷢ����ɱ�־��ֻ����ɷ��䣬���ܽ�����һ֡����
					//HAL_Delay(100);
					//Flag_Zhen_No���л�Ҫ��ϸ����
					if(SystemState)//�������״̬
					{
						if(Flag_Zhen_No==0)
							Flag_Zhen_No=1;
						else
							Flag_Zhen_No=0;//��0-1�м�ѭ��
					}
					else//������״̬
					{
						if(Flag_Zhen_No<3)
							Flag_Zhen_No++;
						else
							Flag_Zhen_No=0;//��0-3�м�ѭ��
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

//BUCK���Ƶ�PWM������Ƚ��жϣ���ʱ��1��ͨ��4����������AD--ADC1-3, ADC1-9, ADC1-14, ADC1-15
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{
		HAL_ADC_Start_DMA(&hadc1,ADC_ConvertedValue,4);
	}
	__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_CC4);
}

//����ΪPID���Ʋ��֣���AD����ж��н��п���
//float inte=0;

#define SCALE_Cur 10000
#define SCALE_Vot 10000
#define TIM2_Pulse_Upper 3600

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//��ò���ֵ
  Vot_Rec_Buck_In=ADC_ConvertedValue[0]&0xfff;//BUCK�����ѹ
	Temp_Rec_Battery=ADC_ConvertedValue[1]&0xfff;//����¶�
	Vot_Rec_Buck_Out=ADC_ConvertedValue[2]&0xfff;//��ص�ѹ
	Cur_Rec_Buck=ADC_ConvertedValue[3]&0xfff;//������
	
	//�����㷨
	
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
		Vot_Err=(Vot_Limit_Rec-Vot_Rec_Buck_Out)*SCALE_Vot;//�趨ֵ��ȥʵ��ֵ
		D_Vot_Err=(Vot_Err-Vot_OldErr);
		Vot_OldErr=Vot_Err;
		
		D_Cur_Ref=D_Vot_Err*Vot_Kp+Vot_Err*Vot_Ki/20000;
		Cur_Ref=Cur_Ref_OldOut+D_Cur_Ref;
		if(Cur_Ref>Cur_Limit_Rec)
		{
			Cur_Ref=Cur_Limit_Rec;//����������ֵ��������Ϊ��
		}
		Cur_Ref_OldOut=Cur_Ref;//ֻ��Cur_Ref��0-Cur_Limit_Rec֮��ʱ���Ÿ���Cur_Ref_OldOut
			
		err=(Cur_Ref-Cur_Rec_Buck)*SCALE_Cur;//�趨ֵ��ȥʵ��ֵ
		
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
			oldOut=intOldOut;//ֻ�������ռ�ձ���0-3600֮��ʱ���Ÿ������
		}	
		Duty_Rec_Buck=out;
	}
	
	//ֻ������PI���ƣ��¶ȿ���δ�������¶ȿ����ѿ����¶�δ����ʱ�򣬲��������PWM����ռ�ձ�
	//������BUCKȫ����BUCKȫ�أ�������PI�����ҿ����¶ȿ��Ƶ��¶ȳ��������£����������PWM��ռ�ձȣ�PWM�������������������趨��ֵ
	/*if((WorkMode&0x0C)==0)//��������PI����ģʽ
	{
		if((WorkMode&0x03)==0)//�¶ȿ���ʧ��
		{
			htim1.Instance->CCR4=Duty_Rec_Buck;
		}
		else if(Temp_Rec_Battery>Temp_Limit_Rec)//WorkMode��1-2λ���㣬����Ϊ�������¶ȿ��ƣ���ʱ��Ҫ�ж��¶��Ƿ񳬱꣬�¶�Խ�ߣ��¶Ȳ���ֵԽС
		{
			htim1.Instance->CCR4=Duty_Rec_Buck;
		}
		else
		{
			htim1.Instance->CCR4=0;//�¶ȳ��꣬�ر�BUCK
		}			
	}*/
	htim1.Instance->CCR4=Duty_Rec_Buck;
	
}
//���ڷ�������ж�
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	Flag_Trans_Done=1;
	Rece_485_En();
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
}

//���ڽ�������ж�
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(!Flag_StartByte)//δ��⵽��ͷ����Ҫ�ȼ����ͷ
	{
		if(Data_ReceSingle==0x55)
		{
			Flag_StartByte=1;//�Ѽ�⵽��ͷ�������1
		}
	}
	else//�Ѽ�⵽��ͷ����Ҫ�����Ч����
	{
		Data_ReceArray[Byte_Cnt]=Data_ReceSingle;
		if(Byte_Cnt<5)//��Ч����6���ֽڣ��±���ൽ5
		{
			Byte_Cnt++;
		}
		else//�ѽ����������¼����ͷ�����485��ʱ���ֽ��±��0���趨��־λ
		{
			Flag_StartByte=0;
			TimeCnt485=0;
			Byte_Cnt=0;
			Flag_Rece_Done=1;
		}
	}
	
	HAL_UART_Receive_IT(&huart1,&Data_ReceSingle,1);
}

//������ʼ������
void Variable_Init()
{
	//��ȡFLASH�д洢�Ŀ�����Ϣ���Լ���������
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress);
	Vot_Limit_Rec=ReadFlashData&0xfff;//��ѹ��ֵ
	Cur_Limit_Rec=(ReadFlashData>>16)&0xfff;//������ֵ
	
	if(Vot_Limit_Rec>4000)
		Vot_Limit_Rec=0;
	if(Cur_Limit_Rec>4000)
		Cur_Limit_Rec=0;
	
	//��Ҫ������ֵ���зǷ��ж����Ժ��ٿ���
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress+4);
	Bike_ID=ReadFlashData;//��������
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress+8);
	WorkMode=ReadFlashData&0x03;//����ģʽ����Чֵֻȡ1-2λ��3-4λĬ��00����ʾ����PI����
	
	Duty_Rec_Buck=3600;//BUCKȫ������ֹ����˹�ѹ
	SystemState=0;//ϵͳ���ڰ�����̬
	Flag_WorkMode_Conversion=0;//����ģʽת����־
	TimeCnt_WorkMode_Conversion=0;//����ģʽת������
		
	//PI������ʼ��
	Cur_Kp=0.001;Cur_Ki=5;
	Vot_Kp=0.001;Vot_Ki=5;
	
	//������ر�����ʼ��
	Flag1=0;Flag2=0;
	BitCnt=0;Flag_Infray_Done=0;
	Flag_Zhen_No=0;Jiaoyan_Infray=0;
	Flag_Infray_WaitTime=1;//��ʼ����Ĭ��������
	TimeCnt_Infray=0;
	
	//������ر�����ʼ��
	Flag_StartByte=0;Byte_Cnt=0;TimeCnt485=0;
	Flag_Trans_Done=0;Flag_Rece_Done=0;
}

//�����������Ϣ����
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
	
	//�洢��FLASH�е�ֵ�������ٴ�FLASH��ֱ�Ӷ�ȡ������֤ȷʵ����ȷ��д����FLASH
	
	SystInfo_Package[0]=0xaa;//��ʼ��ͷ
	SystInfo_Package[1]=Vot_Rec_Buck_Out;//�����ѹ��8λ
	SystInfo_Package[2]=Cur_Rec_Buck;//���������8λ
	SystInfo_Package[3]=(Vot_Rec_Buck_Out>>4)&0xf0;//ȡ�����ѹ9-12λ�������4λ
	SystInfo_Package[3]+=(Cur_Rec_Buck>>8)&0x0f;//ȡ�������9-12λ�������4λ
	SystInfo_Package[4]=Vot_Rec_Buck_In;//�����ѹ��8λ
	SystInfo_Package[5]=Duty_Rec_Buck;//ռ�ձȵ�8λ
	SystInfo_Package[6]=(Vot_Rec_Buck_In>>4)&0xf0;//ȡ�����ѹ9-12λ�������4λ
	SystInfo_Package[6]+=(Duty_Rec_Buck>>8)&0x0f;//ȡռ�ձ�9-12λ�������4λ
	SystInfo_Package[7]=Temp_Rec_Battery;//����¶ȵ�8λ
	SystInfo_Package[8]=(Temp_Rec_Battery>>4)&0xf0;//����¶�9-12λ�������4λ
	
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress+8);
	Temp_ReadInfo=ReadFlashData&0x03;//ֻȡ����λ��Ϣ��BUCKģʽ��Ϣ��ʹ��
	WorkMode=WorkMode&0xfC;//�ѵ���λ��������
	WorkMode=WorkMode+Temp_ReadInfo;//BUCK����ģʽ��д��FLASH������ʼ��ʱ���3-4λ���㣬��ϵͳ��λ��BUCKģʽĬ��Ϊ����PI����
	//������������ʱ����ȡ��BUCK����ģʽ��Ϣ��WorkMode��3-4λ��
	SystInfo_Package[8]+=WorkMode&0x0f;//����ģʽ�������4λ
	
	//��ȡFLASH�д洢�ĵ�ѹ��ֵ�͵�����ֵ
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress);
	Vot_Limit_Rec=ReadFlashData&0xfff;//��ѹ��ֵ
	Cur_Limit_Rec=(ReadFlashData>>16)&0xfff;//������ֵ
	
	SystInfo_Package[9]=Vot_Limit_Rec;//��ѹ��ֵ��8λ
	SystInfo_Package[10]=Cur_Limit_Rec;//������ֵ��8λ
	SystInfo_Package[11]=(Vot_Limit_Rec>>4)&0xf0;//ȡ��ѹ��ֵ9-12λ�������4λ
	SystInfo_Package[11]+=(Cur_Limit_Rec>>8)&0x0f;//ȡ������ֵ9-12λ�������4λ
	
	//��ȡFLASH�д洢��Bike_ID
	ReadFlashData=ReadFlash(FLASH_RW_StartAddress+4);
	Bike_ID=ReadFlashData;//��������
	
	SystInfo_Package[12]=Bike_ID;//���������8λ
	SystInfo_Package[13]=Bike_ID>>8;//��������9-16λ
	SystInfo_Package[14]=Bike_ID>>16;//��������17-24λ
	SystInfo_Package[15]=Bike_ID>>24;//��������25-32λ
	
	//����У��ֵ
	Jiaoyan485=0;
	for(i=1;i<16;i++)
	{
		Jiaoyan485+=SystInfo_Package[i];
	}
	SystInfo_Package[16]=Jiaoyan485;//У��ֵ��8λ
	SystInfo_Package[17]=Jiaoyan485>>8;//У��ֵ��8λ
}

void Infray_InfoPackage_First_First(void)
{
	uint8_t Temp;//��ʱ���������԰��ֽڲ��Bike_ID
	Jiaoyan_Infray=0;//ֻ�ڵ�һ��֡��һ��֡������
	Infray_TransDat=0x33;//������֡ͷ
	
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
	
	Infray_TransDat=0x36;//������֡ͷ
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Bike_ID>>24;
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Vot_Limit_Rec;
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=(Vot_Limit_Rec>>4)&0xf0;//ȡ��ѹ��ֵ��9-12λ�������4λ
	Temp+=(Cur_Limit_Rec>>8)&0x0f;//ȡ������ֵ��9-12λ�������4λ
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
}
void Infray_InfoPackage_First_Third(void)
{
	uint8_t Temp;
	
	Infray_TransDat=0x39;//������֡ͷ
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=Cur_Limit_Rec;
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Temp=WorkMode&0x0f;//ֻȡ��4λ
	Infray_TransDat+=Temp;
	Jiaoyan_Infray+=Temp;
	
	Infray_TransDat=Infray_TransDat<<8;
	Infray_TransDat+=Jiaoyan_Infray;//��Jiaoyanֵд���һ��֡������֡���8λ
}
void Infray_InfoPackage_Second(void)
{
	uint8_t Temp1,Temp2;
	
	Temp1=0;
	
	Infray_TransDat=Cur_Rec_Buck&0xfff;//д���������ֵ��ȡ��12λ
	Infray_TransDat=Infray_TransDat<<12;
	Infray_TransDat+=(Vot_Rec_Buck_Out&0xfff);//д�������ѹֵ��ȡ��12λ
	
	Temp2=Infray_TransDat;
	Temp1+=Temp2;
	
	Temp2=Infray_TransDat>>8;
	Temp1+=Temp2;
	
	Temp2=Infray_TransDat>>16;
	Temp1+=Temp2;
	
	Temp1=Temp1>>4;
	Temp1&=0x0f;
	Temp1+=0xC0;
	
	Infray_TransDat+=Temp1*16777216;//2��24�η����൱������24λ	
}
void Infray_InfoPackage_Third(void)
{
	uint8_t Temp1,Temp2;
	
	Temp1=0;
	
	Infray_TransDat=Duty_Rec_Buck&0xfff;//д��ռ�ձ���Ϣ��ȡ��12λ
	Infray_TransDat=Infray_TransDat<<12;
	Infray_TransDat+=(Vot_Rec_Buck_In&0xfff);//д�������ѹֵ��ȡ��12λ
	
	//����У��
	Temp2=Infray_TransDat;
	Temp1+=Temp2;
	
	Temp2=Infray_TransDat>>8;
	Temp1+=Temp2;
	
	Temp2=Infray_TransDat>>16;
	Temp1+=Temp2;
	
	Temp1=Temp1>>4;
	Temp1&=0x0f;
	Temp1+=0x90;
	
	Infray_TransDat+=Temp1*16777216;//2��24�η����൱������24λ	
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
