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
//�����ǵ���׮������Ϣ
uint8_t Charger_ID;//���׮���
uint8_t Charger_State;//���׮��ǰ״̬
uint8_t Charger_Start_Permission;//����������־
/***********************************************************
����������־��ͨ������ָ����1������������״������ʱ��0��
-----����������
-----����ֹͣ���
-----�����źų�ʱ�ж�
-----Ч�ʳ�������
-----����˵���������������Ϸ���

***********************************************************/
//�����ǽ��ն˵���ز���
uint16_t Vot_Limit_Rec,Cur_Limit_Rec;//���ն˳��ĵ�ѹ������ֵ��ͨ�����ⷢ�͹�����ֵ
uint16_t Vot_Rec_Buck_In,Duty_Rec_Buck,Vot_Rec_Buck_Out,Cur_Rec_Buck,Temp_Rec_Battery;
uint8_t WorkMode_Rec;//�洢���ն˵Ĺ���ģʽ
uint32_t Bike_ID;//�洢���յ��ĳ�������

//�����Ƿ���˵���ز���
uint32_t ADC_ConvertedValue[4];//�洢AD�������
uint16_t Vot_Trans_Buck_In,Duty_Trans_Buck,Vot_Trans_Buck_Out,Cur_Trans_Buck,Temp_Trans_MOS;
uint8_t Relay_State;//�洢�̵���������Ϣ
uint16_t Freq_Value;
uint32_t Fre_Trans_Inverter;
uint8_t Fault_Type;//�������ͣ�ֵΪ1-32�������ȼ��Ĺ������Ϳ��Ը��ǵ����ȼ��Ĺ�������
uint8_t Frequency_Scan_State;//������ǰɨƵ״̬
uint16_t Max_Cur_Trans_Buck;//��¼���ɨƵ����ֵ
uint32_t Max_Cur_Trans_Frequency;//��¼���ɨƵ����ֵ��Ӧ��Ƶ�ʵ�
uint8_t Trans_OverCurrent_Alert;//����˹������澯
uint8_t Trans_OverCurrent_Alert_Read;//��1��ʾ�����澯�ѱ�������֪��
uint16_t Vot_Limit_Trans,Cur_Limit_Trans;//����˳����Ƶĵ�ѹ������ֵ��ͨ�����ָ���·�
uint16_t Charging_ComptCnt;//�����ɼ���
uint8_t Low_Efficiency_Cnt;//��Ч�ʼ���
uint8_t Low_Efficiency_Restart_Cnt;//��Ч�ʳ�������ʱ��ϵͳ������ɨƵ���øñ�����������ɨƵ����������3�κ󣬽������
//�ڿ���������ʱ��Ӧ���ȶ���������ֵ���бȽ��ж�

//�����Ǻ��������ʹ�õı���
uint8_t RmtSta,RDATA;//RDATA���Ա�ʶ��ǰ�������ز������½��ز���
//[7]:�յ��������־
//[6]:���յ���һ�������������Ϣ����־�����һ�ν���
//[5]������
//[4]:����������Ƿ��Ѿ�������
//[3:0]:�����ʱ��
uint16_t Dval[35];//�½���ʱ��������ֵ
uint32_t RmtRec=0;//������յ�������
uint32_t RmtRecTx=0;//�������ڷ���
uint8_t CapCnt=0;//�������
uint16_t Tep;
uint8_t Zhen_Jiaoyan,Data_Jiaoyan;//Zhen_Jiaoyan����ȡ��ͷ���ж��ǵڼ�֡��Data_Jiaoyan���ж��ڲ�������У��
uint8_t Flag_Infrared_Error;//������մ������
uint8_t Infrared_Error_Location;

uint32_t Infrared_Rec_Succ_Cnt;//������ճɹ�����
uint16_t Infrared_Decode_Error_Cnt;//�������������
uint32_t Infrared_Waiting_First_Cnt;//��һ��֡��ʱ����
uint32_t Infrared_Waiting_First_First_Cnt;//��һ��֡��һ��֡��ʱ����
uint32_t Infrared_Waiting_First_Second_Cnt;//��һ��֡�ڶ���֡��ʱ����
uint32_t Infrared_Waiting_First_Third_Cnt;//��һ��֡������֡��ʱ����
uint32_t Infrared_Waiting_Second_Cnt;//�ڶ���֡��ʱ����
uint32_t Infrared_Waiting_Third_Cnt;//������֡��ʱ����
uint32_t Infrared_Rec_Error_Cnt;//������մ������
uint8_t  Flag_Infrared_Data_OK;//�������ݽ���OK���ɸ��ݸñ�־���п���
uint16_t Infrared_Charging_Waiting_Cnt;//����ȴ�ʱ�䣬����3�룬����BUCKֵ������30s���ص�����̬
uint16_t Infrared_Total_Waiting_Cnt;//�����ж೤ʱ��δ���յ��������͵ĺ����źţ�����60s��������ճɹ���������󡢽��մ���ȼ���
uint32_t Infrary_First_Data_First,Infrary_First_Data_Second,Infrary_First_Data_Third;

//485ͨ����ر���
uint8_t Flag_Trans_Done,Flag_Rece_Done,TimeCnt485;
uint8_t Flag_StartByte;//�����Ƿ��⵽��ͷ
uint8_t Byte_Cnt;//�Խ����ֽڵ��������м���
uint8_t Data_ReceArray[8];//�洢���յ��Ĵ������ݣ�8���ֽ�Ϊһ�飬��ʱδ���������ȫ������
uint8_t Data_ReceSingle;//���������ֽ�
uint8_t Flag_485_Reset;//ÿ��5�룬�ж�һ��485��״̬��������Ӧ����

uint8_t All_Data_Package[58];//ȫ����Ϣ���
uint8_t Fault_Data_Package[31];//�������ͼ��ֳ����ݴ��
uint8_t State_Data_Package[11];//״̬��Ϣ���
uint8_t Limit_Data_Package[11];//��ѹ��������ֵ��Ϣ���
uint8_t Frequency_Data_Package[11];//ɨƵ��Ϣ����Ƶ�����ݴ��

//�����ֳ����ݴ洢
uint16_t Field_Vot_Trans_Buck_In,Field_Duty_Trans_Buck,Field_Vot_Trans_Buck_Out,Field_Cur_Trans_Buck;
uint16_t Field_Temp_Trans_MOS,Field_Vot_Rec_Buck_In,Field_Duty_Rec_Buck;
uint16_t Field_Vot_Rec_Buck_Out,Field_Cur_Rec_Buck,Field_Temp_Rec_Battery;
uint32_t Field_Freq_Value;
//����˵�ѹ��ֵ������ֵ�����ն˵�ѹ��ֵ������ֵ�����ն˹���ģʽ�ڳ������в����ı䣬���豣���ֳ�

//����Ч�ʼ���
double Power_Trans,Power_Trans_Refer,Power_Rece,Efficiency,Power_Trans_Refer_Upper;
double K_Power_Trans,K_Power_Rece;

//�����¶�ֵ��ÿ��10s������һ�ι�����ֵ
uint16_t Temp_Refresh_Cnt;
uint8_t  Flag_Temp_Refresh;

//FLASH��ȡ����
uint32_t ReadFlashData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Variable_Init(void);//������ʼ����ֵ

void All_Data_Trans_Packing(void);//ȫ����Ϣ���
void Fault_Data_Trans_Packing(void);//�������ͼ��ֳ����ݴ��
void State_Data_Trans_Packing(void);//״̬��Ϣ���
void Limit_Data_Trans_Packing(void);//��ѹ��������ֵ��Ϣ���
void Frequency_Data_Trans_Packing(void);//Ƶ����Ϣ���

//�������ݽ��ս���
void Data_Decode(void);//�������ݽ���
void Decode_First(void);//��һ��֡����
void Decode_Second(void);//�ڶ���֡����
void Decode_Third(void);//������֡����
void Decode_Fault(void);//����֡����

//ɨƵ��������
void Frequency_Scan(void);//���ν׶β���485��ɨƵ
void Frequency_Scan485(void);//����ָ����Ҫ�ģ���485ͨ�ŵ�ɨƵ
void Charging_Preparing(void);//ɨƵ�ɹ������г��ǰ�ĸ���׼��
void Charging_Stopping(void);//�����жϻ�����ɻ�Ч�ʳ������ͣ���ֹ���

//�����ֳ�����
void Fault_Field_Save(void);

//FLASH��д��������
uint32_t ReadFlash(uint32_t Address);//��ָ��λ���ϵ���ֵ��һ�ζ�ȡ4���ֽ�
void WriteFlash(uint32_t Address);//��ָ����ַ��д���ݣ����ֲ�������һ��д�������ֽ�

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define FLASH_RW_StartAddress ((uint32_t) 0x0803F800)  //FLASH��127ҳ��ʼ��ַ
#define Power_Trans_Refer_Waiting 50
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t ss,sum;//ss���������У��ʱforѭ��ʹ�ã�sum��������ͽ��
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
	
	//������ʱ��2ͨ��1���벶�����Ժ������
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	
	//����BUCK��·��PWM����
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_OC_Start_IT(&htim5,TIM_CHANNEL_1);//��ʼ��ʱ��Ĭ��ռ�ձ�Ϊ0��ȫ��
	HAL_Delay(1);
	
	//����Ƶ�ʿ���
	//HAL_TIM_Base_Start_IT(&htim1);
	//HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);//��ʼ��ʱ��Ĭ��ռ�ձ�Ϊ0��ȫ��
	//HAL_Delay(1);
	
	Rece_485_En();
	HAL_UART_Receive_IT(&huart3,&Data_ReceSingle,1);
	
	//�������Ź�
	HAL_IWDG_Start(&hiwdg);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//LED1_ON();
		//485���յ�����ָ�����ָ��
		if(Flag_StartByte)
		{
			if(TimeCnt485>50)//�Ѽ�⵽��ͷ����50ms��δ�������8���ֽڣ���λ���½���
			{
				Flag_StartByte=0;
				TimeCnt485=0;
				Byte_Cnt=0;
			}
		}
		
		//��ʱʱ�䵽���ж�485״̬
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

		//��ʱʱ�䵽�������¶ȶ�Ӧ�Ĺ�������ֵ
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
		
		if(Flag_Rece_Done)//�������8���ֽڵ���Ч���ݣ����д���
		{
			LED2_TOGGLE();
			HAL_Delay(1);
			Flag_Rece_Done=0;//�����������ɱ�־
			Data_ReceArray[1]=~Data_ReceArray[1];//�Ȱѽ��յ��ı�ŷ���ȡ��
			if(Data_ReceArray[0]==Data_ReceArray[1])//�жϽ��յ��ı����Ƿ�����
			{//������ȷ������£����жϸñ����Ƿ�Ϊ�������
				//LED1_TOGGLE();
				if(Data_ReceArray[0]==Charger_ID)
				{//���������ȷ����Ϊ������룬��ȷ������ѯ���󣬼�������ָ��
					
					switch(Data_ReceArray[2])
					{
						case 0x0F://��ѯ״̬������״̬���
						{
							Trans_485_En();
							State_Data_Trans_Packing();							
							HAL_UART_Transmit_DMA(&huart3,State_Data_Package,sizeof(State_Data_Package));
							break;
						}
						case 0x1E://������磬���ص�ѹ��ֵ��������ֵ�Լ��̵���״̬
						{
							//���ж�У��
							sum=0;
							for(ss=3;ss<7;ss++)
							{
								sum+=Data_ReceArray[ss];
							}
							if(sum==Data_ReceArray[7])
							{
								//�ڽ��г��ģʽת��֮ǰ����ѹ��ֵҪ����ն˽��к˶ԣ����ɹ�����б���
								Vot_Limit_Trans=Data_ReceArray[3]+(Data_ReceArray[5]&0xf0)*16;//��ȡ��ѹ��ֵ����
								Cur_Limit_Trans=Data_ReceArray[4]+(Data_ReceArray[5]&0x0f)*256;//��ȡ������ֵ����
								Relay_State=Data_ReceArray[6];
								if((Charger_State&0x80)&&((Charger_State&0x60)==0x00))//��⵽��������ϵͳ����ֹ̬ͣ�����ܶԸ�ָ��������Ӧ
								{	
									Charger_State&=0xf8;//�����ն˲����޸�����λ���������¼���־λ���Լ������ɱ�־λ���㣬& = 1111, 1000
									if(Fault_Type<16)
										Fault_Type=0;//��������ڷ���˲����޸����ϣ��򽫹���������Ϊ���
									Charging_ComptCnt=0;//�����ɼ�������
									
									if(Vot_Trans_Buck_Out>3000)//ֹ̬ͣ��BUCK�����ѹ���ߣ���ΪBUCK���մ������ù��ϱ�־
									{
										Charger_State|=0x10;//0001 0000��BUCK�մ�����Ӧ����˹����¼�����5λ��1
										if(Fault_Type<FaultLevel_Trans_Buck_Short)
										{
											Fault_Type=FaultLevel_Trans_Buck_Short;
											Fault_Field_Save();
										}										
										Charger_Start_Permission=0;//��������0���ٴγ�����ٴ��·�ָ��
									}
									else
									{									
										Charger_Start_Permission=1;//�����翪��
									}
									
									Trans_485_En();
									Limit_Data_Trans_Packing();//�����ֵ���ݴ��
									HAL_UART_Transmit_DMA(&huart3,Limit_Data_Package,sizeof(Limit_Data_Package));
								}
							}							
							break;
						}
						case 0x2D://ֹͣ��磬����״̬���
						{
						  Charging_Stopping();
							Trans_485_En();
							State_Data_Trans_Packing();
							HAL_UART_Transmit_DMA(&huart3,State_Data_Package,sizeof(State_Data_Package));
							break;
						}
						case 0x3C://����ɨƵ
						{
							if((Charger_State&0x60)==0)//���ж��Ƿ���ֹ̬ͣ��ֻ�д���ֹ̬ͣ���ſ��Խ���ɨƵ����
							{
								//ɨƵռ�ձ���ָ���·��������Ƿ��⵽����������ָ��ռ�ձȽ���ɨƵ
								Data_ReceArray[4]=~Data_ReceArray[4];//�ȰѼ̵�������ȡ��
								if(Data_ReceArray[3]==Data_ReceArray[4])//����̵������������⣬����Ӧ���ܿ�����ָ��
								{
									sum=Data_ReceArray[5]+Data_ReceArray[6];
									if(Data_ReceArray[7]==sum)//�ж�ָ��������У���Ƿ���ȷ
									{
										Duty_Trans_Buck=Data_ReceArray[6]*256;
										Duty_Trans_Buck+=Data_ReceArray[5];//��ȡָ��ָ����ռ�ձ���Ϣ
										htim5.Instance->CCR1=Duty_Trans_Buck;//���趨ɨƵռ�ձ�
										//����״̬λΪָ��ɨƵ״̬
										Charger_State&=0xBF;//& = 1011 1111 �Ե���λ��������
										Charger_State|=0x20;//| = 0010 1111 �Ե���λ������1
										Frequency_Scan485();//��485ͨ�ŵ�ɨƵ
										Duty_Trans_Buck=0;//ɨƵ�������ر�BUCK
										htim5.Instance->CCR1=Duty_Trans_Buck;
										Freq_Value=0xffff;
							      TIM1->ARR=Freq_Value;//����ʱ����װ�ؼĴ�������������Ƶ��
	                  htim1.Instance->CCR2=0;
	                  HAL_TIM_Base_Stop_IT(&htim1);
	                  HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2); 
										HAL_Delay(50);
										Charger_State&=0x9F;//ɨƵ��ɣ��Ե��ߺ͵���λ�������㣬�ָ�״̬λΪֹ̬ͣ
									}
								}								
							}
							//ɨƵ֮ǰӦ�����ж��Ƿ��⵽�˳���������ʹ�ô�ռ�ձȣ�����ʹ��Сռ�ձ�
							break;
						}
						case 0x4B://��ȡ����׮ȫ����Ϣ������ȫ������
						{
							Trans_485_En();
							All_Data_Trans_Packing();
							HAL_UART_Transmit_DMA(&huart3,All_Data_Package,sizeof(All_Data_Package));
							break;
						}
						case 0x5A://��ȡ����׮�������ͼ������ֳ����ݣ����ع�������
						{
							Fault_Data_Trans_Packing();
							Trans_485_En();
							HAL_UART_Transmit_DMA(&huart3,Fault_Data_Package,sizeof(Fault_Data_Package));
							break;
						}
						case 0x69://���㸴λ��־������״̬��Ϣ
						{
							Charger_State&=0xf7;//���㸴λ��־
							Trans_485_En();
							State_Data_Trans_Packing();
							HAL_UART_Transmit_DMA(&huart3,State_Data_Package,sizeof(State_Data_Package));
							break;
						}
						default: break;
					}
				}
				else if(Data_ReceArray[0]==0xff)//�޸ĵ���׮����ID��ָ��
				{
					Data_ReceArray[3]=~Data_ReceArray[3];
					if(Data_ReceArray[2]==Data_ReceArray[3])//��У���趨��IDֵ�Ƿ�����
					{
						//�趨��IDֵͨ��У�飬�����޸ļ�Ӧ��
						Charger_ID=Data_ReceArray[2];
						if(Charger_ID>200)
							Charger_ID=0;
						WriteFlash(FLASH_RW_StartAddress);
						Trans_485_En();
						State_Data_Trans_Packing();							
						HAL_UART_Transmit_DMA(&huart3,State_Data_Package,sizeof(State_Data_Package));
					}
				}
				/*else if(Data_ReceArray[0]==0x00)//����ָ�����˶Ա�ţ����ɶ�ȡ����׮ȫ����Ϣ
				{
					Trans_485_En();
					All_Data_Trans_Packing();
					HAL_UART_Transmit_DMA(&huart3,All_Data_Package,sizeof(All_Data_Package));
				}*/
				else
					;
			}
		}
	
		//������ճɹ�����������
		if(RmtSta&(1<<6))
		{
			if(~Infrared_Rec_Succ_Cnt)//���ñ����ﵽ��ֵȫ1ʱ��ȡ����Ϊ0�������ͻ�ֹͣ����
				Infrared_Rec_Succ_Cnt++;
			Infrared_Total_Waiting_Cnt=0;//һ�����ճɹ����ͽ���������ܵȴ���������
			RmtSta&=~(1<<6);//���ճɹ���־λ����
			Data_Decode();//���ݽ���
		}		
		//������մ������Ӵ����־
		if(Flag_Infrared_Error)
		{
			Flag_Infrared_Error=0;
			if(Infrared_Rec_Error_Cnt<0xffffffff)//�ﵽ��1ʱȡ��Ϊ0.ֹͣ����
				Infrared_Rec_Error_Cnt++;
		}
		if(Infrared_Waiting_First_Cnt>10000)//ֻ�д���ֹ̬ͣʱ���Ż���е�һ��֡���������ǵ����ն��л���ֹ̬ͣ��Ҫ30s����ʱ��������˸�ʱ��Ӧ���ڽ��ն�����ʱ��
		{
			Charger_State&=0x7f;//����10sδ�������յ�һ��֡����Ϊδ��⵽���������ñ�־λ
		}
		//�����ʱ��δ���յ���������źţ��򽫽��ճɹ�����������ʧ�ܼ������������������㣬׼�����¼�������ʱ���Ӧ�����л���ֹ̬ͣ����Ҫ�ĺ�����ʱ����
		if(Infrared_Total_Waiting_Cnt>60000)//����60sδ���յ���������ź�
		{
			Infrared_Rec_Succ_Cnt=0;
			Infrared_Decode_Error_Cnt=0;
			Infrared_Rec_Error_Cnt=0;
		}
		
		//δ��⵽������LED1Ϩ��
		if(Charger_State&0x80)//ȡ���λ���ж����޳���
		{
			LED1_ON();
		}
		else
		{
			LED1_OFF();
		}
				
		//�Ѽ�⵽������LED1��
		
		//״̬�жϼ��л�
		/******************************************
		���������Ƶļ���������
		-----ϵͳ�޹���
		-----��⵽�˳�������
		-----�����˳������ָ��
		-----ɨƵ����ͨ��
		******************************************/
		switch(Charger_State&0x60)//��ȡ��ǰ����״̬����״̬������6-7λ
		{
			case 0x00://0000��0000 ֹ̬ͣ����״̬�н��������жϣ������������л�������״̬
			{
				
				Power_Trans_Refer=0;//�ο�������Ϊ��
				
				/*if(Trans_OverCurrent_Alert)
				{
					Trans_OverCurrent_Alert_Read=1;
					Freq_Value=0xffff;
					TIM1->ARR=Freq_Value;//��װ�ؼĴ�������������Ƶ��
					htim1.Instance->CCR2=0;
					HAL_TIM_Base_Stop_IT(&htim1);
					HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
					Charger_State|=0x10;//0001 0000��ֹ̬ͣ��������Ӧ����˹����¼�����5λ��1
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
						Charger_State&=0xef;//1110, 1111 �������λ����˹��ϱ�־λ
					}
				}*/
				
				if(Charger_State&0x10)//��ȡ����λ���ж��Ƿ���ڷ���˲����޸����ϣ���λ��1��ʾ�й���
				{//���ڹ���
					Charger_Start_Permission=0;//���ڷ���˹��ϣ��رճ�翪�������־;
				}
				else
				{//�޹���
					if(Charger_State&0x80)//��ȡ��������־λ����⵽�������ܿ������
					{
						if(Charger_Start_Permission)//�Ƿ���ָ��������
						{
							if((Vot_Limit_Trans==Vot_Limit_Rec)&&(Cur_Limit_Trans==Cur_Limit_Rec))//ָ���·�����ֵ�ͺ����ϴ�����ֵӦ����һ��
							{	
								Duty_Trans_Buck=Duty_Frequency_Scan;
								htim5.Instance->CCR1=Duty_Trans_Buck;//���趨ɨƵռ�ձȣ��Ѽ�⵽��������Ϊָ����⵽����ʱ��ָ��ռ�ձ�
								HAL_Delay(50);//�տ���BUCK����Ҫ����������磬�ȴ�������������Ӱ��ɨƵ���
								//������ϵͳ״̬λ����ɨƵ̬
								Charger_State|=0x40;//����λ��1, |=0100 0000
								Charger_State&=0xDF;//����λ��0, &=1101 1111
								//�����䲿���մ�����ɨƵ�ʼ�ͻ���ֹ�����������ù��ϻ�������ɨƵ������
								Frequency_Scan();//ɨƵ���ԣ�����ͣ��������Ҳ����ͣ�Ź��࣬Ҳ���ܹ���
								//ɨƵ֮���״̬Ҫת��Ϊ�������״̬����Charging_Preparing()�����н�������
								//ֻ��ͣ������ʱ�����������������
								//ͣ�Ź�������״��ʱ���������Ϲ�����������״̬ת���������������־����Ҫ���µ���������ָ��
							}
							else//ָ���·�����ֵ�ͺ����ϴ�����ֵ��һ�£�������˽������ˣ����ù���λ��
							{
								Charger_Start_Permission=0;//�������־���㣬�����������¿���
								Charger_State|=0x04;//�������ն˲����޸����ϣ����ñ�־λ���������·�ָ��ʱ����
								if(Fault_Type<FaultLevel_LimitError)//���ж����ȼ��������ȼ����ϲ��ɱ����
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
			case 0x60://0110��0000 �������״̬����״̬�н��������жϣ������������л���ֹ̬ͣ
			{
				//�жϺ������������ɱ�־��Ȼ����п���
				//�жϺ�����ʱ���������ж��Ƿ�������ϸ澯
				//�жϵ��������ж��Ƿ�����澯
				//�ж�Ч�ʣ����ж��Ƿ�����澯
				//�жϹ��ʣ����ж��Ƿ�����澯
				//�жϽ��ն˵�ѹ���������ж��Ƿ���ɳ��
				//����״̬���
				if(Infrared_Charging_Waiting_Cnt<3000)//�ڶ���֡�͵�����֡����1s����ʱ���ŻὫInfrared_Charging_Waiting_Cnt���㣬SysTickÿ��1ms��1
				{
					if(Flag_Infrared_Data_OK)
					{
						Flag_Infrared_Data_OK=0;//ÿ�ο������㣬�ٴν��յ��µ���������1
						/*****************************���ն˹��ʲ������**************************/
						if(Duty_Rec_Buck==3600)//���ն����������ӷ��书��
						{
							if(Vot_Rec_Buck_Out<Vot_Limit_Rec-100)//������
							{	
								//��Ҫ�ȸ��ݽ��ն˵���ֵ�жϷ����ռ�ձ�Ҫ���ӵ���
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
							else//��ѹ��
							{
								Power_Trans_Refer+=1;
							}
							if(Power_Trans_Refer>Power_Trans_Refer_Upper)
								Power_Trans_Refer=Power_Trans_Refer_Upper;
						}
						/*******************************���ն˹��ص���****************************/
						if(Duty_Rec_Buck<3600)//���ն˽����˱��������ͷ��书��
						{
							//��Ҫ�ȸ��ݽ��ն�ռ�ձ��жϷ����ռ�ձ�Ӧ�½�����
							if(Duty_Rec_Buck<1000)
							{
								Duty_Trans_Buck=1000;//���ն����ع��أ����̽��ͷ����ռ�ձȣ����µ���
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
						/********************************�������ж�******************************/
						//����˼·�ǣ�ÿ�κ�������������У��ɹ����жϳ���ѹֵ��������ֵ�Ƿ���������ɱ�־
						//������㣬�������1����������㣬��������㣬���������㣨��������100��������Ϊ������
						if(Vot_Rec_Buck_Out>Vot_Limit_Rec-250)//ʵ���ѹ����2000����49V
						{
							if(Cur_Rec_Buck<Cur_Limit_Rec*0.2)//ϵ����0.35��Ϊ0.2
							{
								if(Charging_ComptCnt<100)
									Charging_ComptCnt++;
								else
								{
									Charger_State|=0x02;//Charger_State�ڶ�λ��1����־��������˳�磬���·����ָ��ʱ����
									Charging_Stopping();
								}
							}
							else
								Charging_ComptCnt=0;//���ַ������������
						}
						else
							Charging_ComptCnt=0;//���ַ������������
						/********************************�������ж�******************************/
						
						/********************************��繦�ʡ�Ч�ʼ���******************************/
						
						Power_Rece=Vot_Rec_Buck_Out*Cur_Rec_Buck;
						Power_Rece=Power_Rece*K_Power_Rece;
						Efficiency=Power_Rece/Power_Trans;
						
						/********************************��繦�ʡ�Ч�ʼ���******************************/
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
									Charger_State|=0x01;//�������¼���־λ��1
									//���ϼ�¼���ȼ�¼�ֳ����ٹر�ϵͳ
									if(Fault_Type<FaultLevel_EffiencyLow)//���ж����ȼ��������ȼ����ϲ��ɱ����
									{
										Fault_Type=FaultLevel_EffiencyLow;
										Fault_Field_Save();
									}
									//��ֹ���
									Charging_Stopping();
									//��Ч��������������
									Low_Efficiency_Restart_Cnt=0;//�����ڳ��׼�������㣬��Ȼ��Զ�޷��ﵽ3�����ֹͣʱ����
								}
								else
								{
									Charger_State&=0x9f;//&=1001 1111�������6-7λ����Ϊֹ̬ͣ�����������������־����ѭ�������¿�ʼɨƵ
								}
							}
						}
						else
						{
							Low_Efficiency_Cnt=0;//���ַ�������������
						}
						
						//����ڷ��书�ʽϴ�ʱ������Ч�ʳ������ͣ�Ӧ���ù��ϱ�־����������ɨƵ���������ɨƵ����ȻЧ�ʳ������ͣ���ܾ����
						/********************************��繦�ʡ�Ч�ʼ���******************************/
					}
				}
				else if(Infrared_Charging_Waiting_Cnt<30000)//������ʱ��3-30s֮�䣬����ռ�ձ�
				{
					Power_Trans_Refer=Power_Trans_Refer_Waiting;
				}
				else//������ʱ��30s���⣬Ӧ�ص�����̬
				{
					Charger_State|=0x01;//�������¼���־λ��1
					//���ϼ�¼���ȼ�¼�ֳ����ٹر�ϵͳ
					if(Fault_Type<FaultLevel_InfraryStop)//���ж����ȼ��������ȼ����ϲ��ɱ����
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
		HAL_IWDG_Refresh(&hiwdg);//ι��
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
//SystemTick 1ms�ж�
void HAL_SYSTICK_Callback(void)
{
	//485ͨ����ؼ�ʱ����־
	if(Flag_StartByte)//��⵽485��ͷ��������ʱ����ʱδ�����꣬������ȫ�����յ������ݣ����¼����ͷ
	{
		if(TimeCnt485<255)
		{
			TimeCnt485++;
			if(TimeCnt485>50)//�Ѽ�⵽��ͷ����50ms��δ�������6���ֽڣ���λ���½���
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
	
	if(Temp_Refresh_Cnt<0xffff)//10s�Ӹ���һ�ι�����ֵ
	{
		Temp_Refresh_Cnt++;
		if(Temp_Refresh_Cnt>5000)
		{
			Flag_485_Reset=1;//ÿ��5���ж�һ��
		}
		if(Temp_Refresh_Cnt>10000)
		{
			Temp_Refresh_Cnt=0;
			Flag_Temp_Refresh=1;
		}
	}
	
	if((Charger_State&0x60)==0)//ֹ̬ͣ�����е�һ��֡�����͵ڶ���֡����
	{
		if(Infrared_Waiting_First_Cnt<0xffffffff)//�ü�����1ʱȡ��Ϊ0�����ټ�����δ��������
		{
			Infrared_Waiting_First_Cnt++;
		}
		if(Infrared_Waiting_First_First_Cnt<0xffffffff)//�ü�����1ʱȡ��Ϊ0�����ټ�����δ��������
		{
			Infrared_Waiting_First_First_Cnt++;
		}
		if(Infrared_Waiting_First_Second_Cnt<0xffffffff)//�ü�����1ʱȡ��Ϊ0�����ټ�����δ��������
		{
			Infrared_Waiting_First_Second_Cnt++;
		}
		if(Infrared_Waiting_First_Third_Cnt<0xffffffff)//�ü�����1ʱȡ��Ϊ0�����ټ�����δ��������
		{
			Infrared_Waiting_First_Third_Cnt++;
		}
		if(Infrared_Waiting_Second_Cnt<0xffffffff)
		{
			Infrared_Waiting_Second_Cnt++;
		}
	}
	//SysTick��ֻ���м����Լӣ��ں�����뺯���н�������
	if((Charger_State&0x60)==0x60)//�������״̬�����еڶ���֡�����͵�����֡����
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
//��������жϴ�����ʱ��2ͨ��1
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)//����������ʱ��2ͨ��1������ж�
	{
		if(__HAL_TIM_GET_IT_SOURCE(htim,TIM_IT_UPDATE) !=RESET)
		{
			RmtSta&=~0x10;//ȡ���������ѱ�������
			RmtSta&=~(1<<7);//һ�����ֹ�����ʱ�������������־�����µȴ�������
			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
			RDATA=1;
			CapCnt=0;//���¼�λ
			if((RmtSta&0x0f)<14)
				RmtSta++;//��¼�������  
			else
			{
				RmtSta&=0xf0;//��ռ�����
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_UPDATE);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)//����������TIM2ͨ��1�����벶���ж�
	{
		if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) !=RESET)
		{
			if(RDATA)//�����ز����־
			{
				//�½��ز���
				__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
				//��ռ�����
				__HAL_TIM_SET_COUNTER(htim,0);
				RmtSta|=0x10;
				RDATA=0;
			}
			else//�½��ز���
			{				
				__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
				RDATA=1;
				if(RmtSta&0x10)//�Ѳ���һ�θߵ�ƽ
				{	
					RmtSta&=~(1<<4);//������Ѳ��������ر�־���´β���������ʱ�ٱ��
					Dval[CapCnt]=__HAL_TIM_GET_COUNTER(htim);
					if(RmtSta&0x80)//���յ�������
					{				
						if((Dval[CapCnt]>400)&&(Dval[CapCnt]<800))//560Ϊ��׼ֵ,560us
						{							
							if(CapCnt<32)
							{
								RmtRec<<=1;//����һλ����λ��0
								CapCnt++;
								if(CapCnt>=32)
								{
									RmtSta|=1<<6;//��־�����һ�ν���
									RmtSta&=~(1<<7);//�ѽ��յ������룬���Ѵ洢32λ��Ϣ��ֹͣ�����������ɽ���
									RmtRecTx=RmtRec;
								}
							}
						}
						else if((Dval[CapCnt]>1000)&&(Dval[CapCnt]<1800))//1680Ϊ��׼ֵ,1680us
						{
							if(CapCnt<32)
							{
								RmtRec<<=1;
								RmtRec+=1;//����һλ����λ��1
								CapCnt++;
								if(CapCnt>=32)
								{
									RmtSta|=1<<6;//��־�����һ�ν���
									RmtSta&=~(1<<7);//�ѽ��յ������룬���Ѵ洢32λ��Ϣ��ֹͣ�����������ɽ���
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
					else if(Dval[CapCnt]>4000&&Dval[CapCnt]<5000) //4500Ϊ��׼ֵ4.5 ms���ж�������
					{
						RmtSta|=1<<7;//��ǳɹ����յ�������
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
	Charger_ID=ReadFlashData;//ȡ��8λ������127ҳ��һ���ֽ�
	if(Charger_ID>200)
		Charger_ID=101;
	if(Charger_ID==0)
		Charger_ID=101;
	//Charger_ID����Ϊ��
		
	Charger_State=0x08;//δ��⵽������ֹ̬ͣ���޹��ϣ��и�λ
	
	//�����������
	RDATA=1;
	
	//485��־������
	Byte_Cnt=0;
	
	//���ʼ���ϵ��
	K_Power_Trans=0.0000313;
	K_Power_Rece=0.00002;
	
	//�����ɼ�������
	Charging_ComptCnt=0;
	//��Ч�ʼ�������
	Low_Efficiency_Cnt=0;
	//��Ч��������������
	Low_Efficiency_Restart_Cnt=0;
	//��������ֵ��ʼ��Ϊ200W
	Power_Trans_Refer_Upper=200;
  //�¶ȸ��±�־����
	Temp_Refresh_Cnt=0;
	Flag_Temp_Refresh=0;
	//485״̬�жϱ�־����
	Flag_485_Reset=0;
	//ռ�ձȹض�
	Duty_Trans_Buck=0;
	htim5.Instance->CCR1=Duty_Trans_Buck;
	//Ƶ����Ϊ0
	Freq_Value=0xffff;
	TIM1->ARR=Freq_Value;//��װ�ؼĴ�������������Ƶ��
	htim1.Instance->CCR2=0;
	
}

//���ڷ�������ж�
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	Flag_Trans_Done=1;//�ṩ������ɱ�־λ
	Rece_485_En();//����ʹ��
	HAL_UART_Receive_IT(&huart3,&Data_ReceSingle,1);//���ж�ģʽһ�ν���һ���ֽ�
}
//���ڽ�������ж�
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t temp;
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
		if(Byte_Cnt<7)//��Ч����8���ֽڣ��±���ൽ7
		{
			Byte_Cnt++;
		}
		else//�ѽ����������¼����ͷ�����485��ʱ���ֽ��±��0���趨��־λ
		{
			Flag_StartByte=0;
			TimeCnt485=0;
			Byte_Cnt=0;
			temp=Charger_State&0x60;
			if((temp==0x00)||(temp==0x60))//������ֹ̬ͣ����̬
			{
				Flag_Rece_Done=1;//�������ָ��������������ָ�ɨƵ̬��������Ӧ��
			}			
		}
	}	
	HAL_UART_Receive_IT(&huart3,&Data_ReceSingle,1);
}
//������ս���
void Data_Decode(void)
{
	Zhen_Jiaoyan=(RmtRecTx>>28)&0x0f;//ȡ֡ͷ
	switch(Zhen_Jiaoyan)
	{
		case 3://֡ͷΪ0011
		{
			Infrared_Total_Waiting_Cnt=0;//���յ�����һ֡���ݣ����������ܼ������㣬��ʾ�г�������
			Decode_First();
			break;
		}
		case 12://֡ͷΪ1100
		{
			Infrared_Total_Waiting_Cnt=0;
			Decode_Second();
			break;
		}
		case 9://֡ͷΪ1001
		{
			Infrared_Total_Waiting_Cnt=0;
			Decode_Third();
			break;
		}
		case 6://֡ͷΪ0110
		{
			Infrared_Total_Waiting_Cnt=0;
			Decode_Fault();
			break;
		}
		default: 
		{
			if(Infrared_Decode_Error_Cnt<0xffff)//֡ͷ��ƥ�䣬�������
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
		Infrared_Waiting_First_First_Cnt=0;//�����һ��֡��һ��֡
		Infrary_First_Data_First=RmtRecTx;
	}
	else if(Zhen_Jiaoyan==6)
	{
		Infrared_Waiting_First_Second_Cnt=0;//�����һ��֡�ڶ���֡
		Infrary_First_Data_Second=RmtRecTx;
	}
	else if(Zhen_Jiaoyan==9)
	{
		Infrared_Waiting_First_Third_Cnt=0;//�����һ��֡������֡
		Infrary_First_Data_Third=RmtRecTx;
	}
	else
	{
		if(Infrared_Decode_Error_Cnt<0xffff)//����1���Լ�
		{
			Infrared_Decode_Error_Cnt++;
		}
	}
	if((Infrared_Waiting_First_First_Cnt<1000)&&(Infrared_Waiting_First_Second_Cnt<1000)&&(Infrared_Waiting_First_Third_Cnt<1000))
	{
		jiaoyan=Infrary_First_Data_Third;//ȡ������֡��8λ��ΪУ��ֵ
		
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

		if(jiaoyan==sum)//������֡��ͬУ��ɹ�
		{
			Infrared_Waiting_First_Cnt=0;//������֡���ݼ�ʱ�Ծ�����Ҫ��1s���ڵ��������ʶȣ�������Ϊ��һ��֡����������
			if(Charger_State&0x80)//ȡ���λ��������־���Ѽ�⵽���������޸�
			{
				;
			}
			else//δ��⵽���������ȡ��ţ����ó������״ֵ̬
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
				
				Charger_State|=0x80;//����Ϊ�Ѽ�⵽����
			}		
		}		
	}
}

void Decode_Second(void)
{
	uint8_t temp1,temp2;
	
	LED3_TOGGLE();
	
	Data_Jiaoyan=(RmtRecTx>>20)&0xf0;//ȡ��25-28λ����
	
	temp1=0;temp2=RmtRecTx;temp1+=temp2;
	temp2=RmtRecTx>>8;temp1+=temp2;	
	temp2=RmtRecTx>>16;temp1+=temp2;
	temp1&=0xf0;
	
	if(Data_Jiaoyan==temp1)
	{
		//У��ɹ�����������
		Vot_Rec_Buck_Out=RmtRecTx&0xfff;
		Cur_Rec_Buck=(RmtRecTx>>12)&0xfff;
		Infrared_Waiting_Second_Cnt=0;//����ڶ���֡����
	}
	else
	{
		if(Infrared_Decode_Error_Cnt<0xffff)//����1���Լ�
		{
			Infrared_Decode_Error_Cnt++;
		}
	}	
}

void Decode_Third(void)
{
	uint8_t temp1,temp2;
	
	LED4_TOGGLE();
	
	Data_Jiaoyan=(RmtRecTx>>20)&0xf0;//ȡ��25-28λ����
	
	temp1=0;temp2=RmtRecTx;temp1+=temp2;
	temp2=RmtRecTx>>8;temp1+=temp2;	
	temp2=RmtRecTx>>16;temp1+=temp2;
	temp1&=0xf0;
	
	if(Data_Jiaoyan==temp1)
	{
		//У��ɹ�����������
		Vot_Rec_Buck_In=RmtRecTx&0xfff;
		Duty_Rec_Buck=(RmtRecTx>>12)&0xfff;
		Infrared_Waiting_Third_Cnt=0;//���������֡����
		if(Infrared_Waiting_Second_Cnt<1000)//�жϵڶ���֡��ʵʱ��
		{
			Infrared_Charging_Waiting_Cnt=0;//�������ȴ�����
			Flag_Infrared_Data_OK=1;//��������OK���ɽ���һ�ο���
		}
	}
	else
	{
		if(Infrared_Decode_Error_Cnt<0xffff)//����1���Լ�
		{
			Infrared_Decode_Error_Cnt++;
		}
	}
}

void Decode_Fault(void)
 {
	;
}
//ȫ����Ϣ���
void All_Data_Trans_Packing(void)
{
	uint8_t i,temp;
	uint16_t Jiaoyan;
	
	All_Data_Package[0]=0xaa;//����һ֡�Ŀ�ʼ
	All_Data_Package[1]=0x0f;//�����֡����Ϊ����׮ȫ����Ϣ֡
	All_Data_Package[2]=Charger_ID;
	All_Data_Package[3]=~Charger_ID;
	All_Data_Package[4]=Charger_State;
	All_Data_Package[5]=~Charger_State;
	All_Data_Package[6]=Vot_Trans_Buck_In;
	All_Data_Package[7]=Duty_Trans_Buck;
	All_Data_Package[8]=(Vot_Trans_Buck_In>>4)&0xf0;//ȡ�����ѹ9-12λ�������4λ
	All_Data_Package[8]+=(Duty_Trans_Buck>>8)&0x0f;//ȡ�����ռ�ձ�9-12λ�������4λ
	All_Data_Package[9]=Vot_Trans_Buck_Out;
	All_Data_Package[10]=Cur_Trans_Buck;
	All_Data_Package[11]=(Vot_Trans_Buck_Out>>4)&0xf0;//ȡ�����ѹ9-12λ�������4λ
	All_Data_Package[11]+=(Cur_Trans_Buck>>8)&0x0f;//ȡ�������9-12λ�������4λ
	All_Data_Package[12]=Relay_State;
	All_Data_Package[13]=Temp_Trans_MOS;
	
	Fre_Trans_Inverter=Freq_Value;
	All_Data_Package[14]=Fre_Trans_Inverter;
	All_Data_Package[15]=Fre_Trans_Inverter>>8;
	All_Data_Package[16]=(Fre_Trans_Inverter>>12)&0xf0;//ȡƵ�ʵ�17-20λ�������4λ
	All_Data_Package[16]+=(Temp_Trans_MOS>>8)&0x0f;//ȡ�¶ȵ�9-12λ�������4λ
	
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
	All_Data_Package[39]=(Vot_Limit_Trans>>4)&0xf0;//ȡ����˵�ѹ��ֵ9-12λ�������4λ
	All_Data_Package[39]+=(Cur_Limit_Trans>>8)&0x0f;//ȡ����˵�����ֵ9-12λ�������4λ
	All_Data_Package[40]=Fault_Type;
	All_Data_Package[41]=Bike_ID;
	All_Data_Package[42]=Bike_ID>>8;
	All_Data_Package[43]=Bike_ID>>16;
	All_Data_Package[44]=Bike_ID>>24;
	All_Data_Package[45]=Vot_Limit_Rec;
	All_Data_Package[46]=Cur_Limit_Rec;
	All_Data_Package[47]=(Vot_Limit_Rec>>4)&0xf0;//ȡ��ѹ��ֵ9-12λ�������4λ
	All_Data_Package[47]+=(Cur_Limit_Rec>>8)&0x0f;//ȡ������ֵ9-12λ�������4λ
	All_Data_Package[48]=Vot_Rec_Buck_In;
	All_Data_Package[49]=Duty_Rec_Buck;
	All_Data_Package[50]=(Vot_Rec_Buck_In>>4)&0xf0;//ȡ���ն������ѹֵ9-12λ�������4λ
	All_Data_Package[50]+=(Duty_Rec_Buck>>8)&0x0f;//ȡ���ն�ռ�ձ�9-12λ�������4λ
	All_Data_Package[51]=Vot_Rec_Buck_Out;
	All_Data_Package[52]=Cur_Rec_Buck;
	All_Data_Package[53]=(Vot_Rec_Buck_Out>>4)&0xf0;//ȡ���ն������ѹֵ9-12λ�������4λ
	All_Data_Package[53]+=(Cur_Rec_Buck>>8)&0x0f;//ȡ���ն��������9-12λ�������4λ
	All_Data_Package[54]=Temp_Rec_Battery;
	All_Data_Package[55]=(Temp_Rec_Battery>>4)&0xf0;//ȡ����¶�9-12λ�������4λ
	temp=WorkMode_Rec&0x0f;//ֻȡ��4λ��Ч����
	All_Data_Package[55]+=temp;
	
	Jiaoyan=0;
	for(i=6;i<56;i++)
	{
		Jiaoyan+=All_Data_Package[i];
	}
	All_Data_Package[56]=Jiaoyan;
	All_Data_Package[57]=Jiaoyan>>8;
}

//�������ͼ��ֳ����ݴ��
void Fault_Data_Trans_Packing(void)
{
	uint8_t i;
	uint16_t Jiaoyan;
	
	Fault_Data_Package[0]=0xaa;//����һ֡�Ŀ�ʼ
	Fault_Data_Package[1]=0x1E;//�����֡����Ϊ����֡����
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
	//�жϹ����Ƿ�����ն��й�
	//
	Jiaoyan=0;
	for(i=4;i<29;i++)
	{
		Jiaoyan+=Fault_Data_Package[i];
	}
	Fault_Data_Package[29]=Jiaoyan;
	Fault_Data_Package[30]=Jiaoyan>>8;
}

//״̬��Ϣ���
void State_Data_Trans_Packing(void)
{
	uint8_t i,temp;
	
	State_Data_Package[0]=0xaa;//����һ֡�Ŀ�ʼ
	State_Data_Package[1]=0x2D;//�����֡����Ϊ״̬֡����
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
//��ѹ��������ֵ��Ϣ���
void Limit_Data_Trans_Packing(void)
{
	uint8_t i,Jiaoyan;
	
	Limit_Data_Package[0]=0xaa;//����һ��Ŀ�ʼ
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
//Ƶ����Ϣ���
void Frequency_Data_Trans_Packing(void)
{
	uint8_t i,Jiaoyan;
	
	Frequency_Data_Package[0]=0xAA;
	Frequency_Data_Package[1]=0x4B;
	Frequency_Data_Package[2]=Charger_ID;
	Frequency_Data_Package[3]=~Charger_ID;
	Frequency_Data_Package[4]=Frequency_Scan_State;
	Frequency_Data_Package[5]=Cur_Trans_Buck;
	Frequency_Data_Package[6]=(Cur_Trans_Buck>>8)&0x0f;//ȡ����ֵ��9-12λ�������4λ
	
	Fre_Trans_Inverter=Freq_Value-Frequency_Scan_Delt;
	Frequency_Data_Package[7]=Fre_Trans_Inverter;
	Frequency_Data_Package[8]=Fre_Trans_Inverter>>8;
	Frequency_Data_Package[9]=(Fre_Trans_Inverter>>16)&0x0f;//ȡƵ��ֵ��17-20λ�������4λ
	
	Jiaoyan=0;
	for(i=5;i<10;i++)
	{
		Jiaoyan+=Frequency_Data_Package[i];
	}
	Frequency_Data_Package[10]=Jiaoyan;
}

void Frequency_Scan(void)
{
	//��ʼ��ɨƵ�����������ֵ��������ֵ��Ӧ��Ƶ��ֵ������Ϊ0
	Max_Cur_Trans_Buck=0;
	Max_Cur_Trans_Frequency=0;
	Frequency_Scan_State=0;//��ʼ��ɨƵ״̬��δ��ʼɨƵ
	
	for(Freq_Value=Frequency_Start;;)
	{
		TIM1->ARR=Freq_Value;
	  htim1.Instance->CCR2=Freq_Value/2;
	  HAL_TIM_Base_Start_IT(&htim1);
	  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
		HAL_Delay(50);
		if(Trans_OverCurrent_Alert)
		{
			Trans_OverCurrent_Alert_Read=1;//���Ķ��澯��־
			Freq_Value=0xffff;
			TIM1->ARR=Freq_Value;//��װ�ؼĴ�������������Ƶ��
	    htim1.Instance->CCR2=0;
	    HAL_TIM_Base_Stop_IT(&htim1);
	    HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
			//�ж����ѹر�BUCK���˴������ٴιر�
			//Duty_Trans_Buck=0;
			//htim5.Instance->CCR1=Duty_Trans_Buck;//�ر�BUCK
			Charger_Start_Permission=0;//�������־��0��������Ϻ��ٴ��·�ָ��
			Charger_State|=0x01;//���������¼������޸����ٴ��·�ָ��ʱ����
			if(Fault_Type<FaultLevel_CurOver_Short)
			{
				Fault_Type=FaultLevel_CurOver_Short;
				Fault_Field_Save();
			}
			Frequency_Scan_State=0x2d;
			Charger_State&=0x9F;//ɨƵ�������Ե��ߺ͵���λ��������
			return;//�˳�������������ѭ����������״̬ת����������ѭ���в��л�״̬
		}
		else
		{
			if(Cur_Trans_Buck>Max_Cur_Trans_Buck)
			{
				Max_Cur_Trans_Buck=Cur_Trans_Buck;
				Max_Cur_Trans_Frequency=Freq_Value;
			}
			
			Frequency_Scan_State=0x1E;//����ɨƵ״̬
			
			//�ж�ɨƵ���������ɨƵ������˵��ͣ�ž����Զ���ܾ����
			if(Cur_Trans_Buck>Frequency_Scan_CurLimit)
			{
				//��ɨƵ������ָ������δ�ﵽ�ܴ�ļ���ֵ������������ɨƵ���ܵ�ֵ�����ֵ�ԭ���ǳ���ͣ�Ź�Զ�����������⣬�����޸���
				//LED1_TOGGLE();
				Frequency_Scan_State=0x2D;
				Freq_Value=0xffff;
				TIM1->ARR=Freq_Value;//����װ�ؼĴ�������������Ƶ��ֵ
	      htim1.Instance->CCR2=0;
	      HAL_TIM_Base_Stop_IT(&htim1);
	      HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
				Duty_Trans_Buck=0;
				htim5.Instance->CCR1=Duty_Trans_Buck;//�ر�BUCK
				Charger_State|=0x01;//���������¼������޸��������λ��1���ñ�־λ��ÿ����ָ������ϵͳʱ����
				if(Fault_Type<FaultLevel_FreqScanError)//ɨƵ�������ϵȼ�Ϊ4���ڵ�ǰ�������ȼ�С��4ʱ���ɸ��¹������ͼ������ֳ�
				{
					Fault_Type=FaultLevel_FreqScanError;//Ҫ��������ֳ�
					Fault_Field_Save();
				}
				Charger_Start_Permission=0;//�����������־��������Ϻ������·�ָ���ٴο���
				Charger_State&=0x9F;//ɨƵ�������Ե��ߺ͵���λ��������
				return;
			}		
			Freq_Value+=Frequency_Scan_Delt;//����Ƶ��ֵ
			
			if(Freq_Value>Frequency_Stop)//��ʾ���������ɨƵ
			{
				Frequency_Scan_State=0xf0;//0xf0��ʾɨƵ�������
				Charging_Preparing();
				return;
			}
		}
	}
}

void Frequency_Scan485(void)
{
	//��ʼ��ɨƵ�����������ֵ��������ֵ��Ӧ��Ƶ��ֵ������Ϊ0
	Max_Cur_Trans_Buck=0;
	Max_Cur_Trans_Frequency=0;
	Frequency_Scan_State=0;//��ʼ��ɨƵ״̬��δ��ʼɨƵ
	
	for(Freq_Value=Frequency_Start;;)
	{
		TIM1->ARR=Freq_Value;
	  htim1.Instance->CCR2=Freq_Value/2;
	  HAL_TIM_Base_Start_IT(&htim1);
	  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
		HAL_Delay(50);

		if(Trans_OverCurrent_Alert)
		{
			Trans_OverCurrent_Alert_Read=1;//���Ķ��澯��־
			Freq_Value=0xffff;
			TIM1->ARR=Freq_Value;//����װ�ؼĴ�������������Ƶ��ֵ
	    htim1.Instance->CCR2=0;
	    HAL_TIM_Base_Stop_IT(&htim1);
	    HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
			//�ж����ѹر�BUCK���˴������ٴιر�
			//Duty_Trans_Buck=0;
			//htim5.Instance->CCR1=Duty_Trans_Buck;//�ر�BUCK
			Charger_Start_Permission=0;//�������־��0��������Ϻ��ٴ��·�ָ��
			Charger_State|=0x01;//���������¼������޸����ٴ��·�ָ��ʱ����
			if(Fault_Type<FaultLevel_CurOver_Short)
			{
				Fault_Type=FaultLevel_CurOver_Short;
				Fault_Field_Save();
			}
			Frequency_Scan_State=0x2d;//����ɨƵ�����־
			Trans_485_En();
			Frequency_Data_Trans_Packing();
			HAL_UART_Transmit_DMA(&huart3,Frequency_Data_Package,sizeof(Frequency_Data_Package));
			return;//�˳�������������ѭ����������״̬ת����������ѭ���в��л�״̬
		}
		else
		{
			if(Cur_Trans_Buck>Max_Cur_Trans_Buck)
			{
				Max_Cur_Trans_Buck=Cur_Trans_Buck;
				Max_Cur_Trans_Frequency=Freq_Value;
			}
			
			Frequency_Scan_State=0x1E;//����ɨƵ״̬
			
			//�ж�ɨƵ���������ɨƵ������˵��ͣ�ž����Զ���ܾ����
			if(Cur_Trans_Buck>Frequency_Scan_CurLimit)
			{
				//��ɨƵ������ָ������δ�ﵽ�ܴ�ļ���ֵ������������ɨƵ���ܵ�ֵ�����ֵ�ԭ���ǳ���ͣ�Ź�Զ�����������⣬�����޸���
				Frequency_Scan_State=0x2D;//����ɨƵ�����־
				Freq_Value=0xffff;
				TIM1->ARR=Freq_Value;//����װ�ؼĴ�������������Ƶ��ֵ
	      htim1.Instance->CCR2=0;
	      HAL_TIM_Base_Stop_IT(&htim1);
	      HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
				Duty_Trans_Buck=0;
				htim5.Instance->CCR1=Duty_Trans_Buck;//�ر�BUCK
				Charger_State|=0x01;//���������¼������޸��������λ��1���ñ�־λ��ÿ����ָ������ϵͳʱ����
				if(Fault_Type<FaultLevel_FreqScanError)//ɨƵ�������ϵȼ�Ϊ4���ڵ�ǰ�������ȼ�С��4ʱ���ɸ��¹������ͼ������ֳ�
				{
					Fault_Type=FaultLevel_FreqScanError;//Ҫ��������ֳ�
					Fault_Field_Save();
				}		
				//Charger_Start_Permission=0;//�����������־��������Ϻ������·�ָ���ٴο���
				//��ɨƵ��ָ��ɨƵ���������δ�������ƺ����������������־
				Trans_485_En();
				Frequency_Data_Trans_Packing();
				HAL_UART_Transmit_DMA(&huart3,Frequency_Data_Package,sizeof(Frequency_Data_Package));
				return;
			}		
			Freq_Value+=Frequency_Scan_Delt;//����Ƶ��ֵ
			
			if(Freq_Value>Frequency_Stop)//��ʾ���������ɨƵ
			{
				Frequency_Scan_State=0xf0;//0xf0��ʾɨƵ�������
			}
			Trans_485_En();
			Frequency_Data_Trans_Packing();
			HAL_UART_Transmit_DMA(&huart3,Frequency_Data_Package,sizeof(Frequency_Data_Package));
			if(Frequency_Scan_State==0xf0)
				return;
		}
	}
}

//PWM����Ƚ��жϣ�����AD����
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM5)
	{
		HAL_ADC_Start_DMA(&hadc1,ADC_ConvertedValue,4);
	}
	__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_CC1);
}
//AD����ж�
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//��ò���ֵ
	Temp_Trans_MOS=ADC_ConvertedValue[0]&0xfff;
	Vot_Trans_Buck_In=ADC_ConvertedValue[1]&0xfff;
	Cur_Trans_Buck=ADC_ConvertedValue[2]&0xfff;
	Vot_Trans_Buck_Out=ADC_ConvertedValue[3]&0xfff;
	Power_Trans=Vot_Trans_Buck_Out*Cur_Trans_Buck;//�Ȱ�����ת��Ϊdouble��
	Power_Trans=Power_Trans*K_Power_Trans;//���б�������
	
	if((Charger_State&0x60)==0x60)//ֻ�д��ڳ��̬��ִ��ռ�ձȵ���
	{
		if(Power_Trans>Power_Trans_Refer)
		{
			if(Duty_Trans_Buck>2)
				Duty_Trans_Buck-=2;//Ϊ��ֹ���Ʋ������¹��ʳ����������ʵĵ��ڿ��ʵ����ӷ���
			else
				Duty_Trans_Buck=0;
			htim5.Instance->CCR1=Duty_Trans_Buck;
		}
		else
		{
			if(Vot_Trans_Buck_Out<4000)//��ֹ�迹����ʱ�������ѹ���ߣ��ﵽAD��ֵ��Ӱ�칦�ʼ��㣬����ֹ��ѹ����
			{				
				if(Cur_Trans_Buck<3440)//4A--�������ܹ���
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
				
				if(Duty_Trans_Buck<20)//���̬��BUCKֵ��С�������ѹ�ܴ󣬿���Ϊ��BUCK�մ�
				{
					Freq_Value=0xffff;
					TIM1->ARR=Freq_Value;//��װ�ؼĴ�������������Ƶ��
					htim1.Instance->CCR2=0;
					HAL_TIM_Base_Stop_IT(&htim1);
					HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
					Charger_State|=0x10;//0001 0000��BUCK�մ�����Ӧ����˹����¼�����5λ��1
					if(Fault_Type<FaultLevel_Trans_Buck_Short)
					{
						Fault_Type=FaultLevel_Trans_Buck_Short;
						Fault_Field_Save();
					}
					
					Charger_Start_Permission=0;//��������0���ٴγ�����ٴ��·�ָ��
					Charger_State&=0x1f;//&=0001 1111�������6-7λ����Ϊֹ̬ͣ��ͬʱ���㳵������־���ȴ��������ٴε���
				}
			}							
			htim5.Instance->CCR1=Duty_Trans_Buck;
		}		
	}
	
	if(Cur_Trans_Buck>4000)//�澯�ٽ�ֵ��Ҫ�ٴ�ȷ����AD�ĵ����Ŵ����Ҳ��Ҫ�ٴ�ȷ��
	{
		htim5.Instance->CCR1=0;//�ر�BUCK
		Trans_OverCurrent_Alert=1;//�������󣬷����澯
		Duty_Trans_Buck=0;
	}
	if(Cur_Trans_Buck<3600)
	{
		//����ֵ�½����ɿط�Χ���Ҹ澯��־�ѱ��������Ķ������Ӧȡ���澯��־
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
	
	//����Ƶ��ֵ
	Freq_Value=Max_Cur_Trans_Frequency-Frequency_Waiting_Delt;
  TIM1->ARR=Freq_Value;
	htim1.Instance->CCR2=Freq_Value/2;
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
	
	//����״̬ת����ת��Ϊ���״̬
	Charger_State|=0x60;//���ߺ͵���λ��1��|=0110 0000
	//���ú���ȴ�������һ��ʼ�ʹ��ڵȴ���������Ϣ״̬�������յ������Ϣ��Infrared_Charging_Waiting_Cnt�Իᱻ����
	Infrared_Charging_Waiting_Cnt=3001;
	
	//�����ɼ�������
	Charging_ComptCnt=0;
	//��Ч�ʼ�������
	Low_Efficiency_Cnt=0;
}

void Charging_Stopping()
{
	Charger_Start_Permission=0;//��������0���ٴγ�����ٴ��·�ָ��
	Charger_State&=0x1f;//&=0001 1111�������6-7λ����Ϊֹ̬ͣ��ͬʱ���㳵������־���ȴ��������ٴε���
	
	//ռ�ձ���Ϊ0
	Duty_Trans_Buck=0;
	htim5.Instance->CCR1=Duty_Trans_Buck;
	HAL_Delay(50);
	//Ƶ����Ϊ0
	Freq_Value=0xffff;
	TIM1->ARR=Freq_Value;//��װ�ؼĴ�������������Ƶ��
	htim1.Instance->CCR2=0;
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2);
	
	
	
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
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address,Charger_ID);
	HAL_FLASH_Lock();
}
//�����ֳ�����
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
