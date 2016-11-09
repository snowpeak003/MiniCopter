#include "usart.h"
#include "config.h"
#include "stdio.h"
#include "struct_all.h"
#include "protocol.h"
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

//static uint8_t TxCount=0;
//static uint8_t Count=0;

//static  uint8_t TxBuff[256];//���ڷ��ͻ�����
uint8_t RxBuff[2][50];		//���ڽ��ջ�����
uint8_t Line0,Line1;	//���ڽ���˫�����л�


typedef union {unsigned char byte[4];float num;}t_floattobyte;
t_floattobyte floattobyte;
//u16 USART_RX_STA=0;
///*******************************************************************************
//* Function Name  : USART_Configuration
//* Description    : Configure Open_USARTx 
//* Input          : None
//* Output         : None
//* Return         : None
//* Attention		 : None
//*******************************************************************************/
u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
void Usart1_Init(u32 baudrate)
{ 												
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

  //Open_USARTx_TX -> PA9 , Open_USARTx_RX -PA10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  /* Enable the Open_USART Transmit interrupt: this interrupt is generated when the 
     Open_USARTx transmit data register is empty */
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

  USART_Cmd(USART1, ENABLE);
}

void USART1_NVIC_Configuration(void)				//���ô����ж����ȼ�
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	if(USART1->SR & USART_SR_ORE)//ORE�ж�
	{
		u8 com_data = USART1->DR;//USART_ClearFlag(USART1,USART_IT_ORE);
	}
	//�����ж�
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART1->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�ж�//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}
	//�����ж� (���ռĴ����ǿ�) /////////////////////////////////////////////////////////////////////////////////////////
	if(USART1->SR & (1<<5))//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    
	{
		static uint8_t Head = 0;
		static uint8_t Line = 0;
		uint8_t com_data = USART1->DR;
		
		if(Head==0)	//Ѱ��֡ͷ
		{
			if(com_data=='$')	
			{
				RxBuff[Line][0] = com_data;
				Head = 1;
			}
		}
		else if(Head==1)
		{
			if(com_data=='M')	
			{
				RxBuff[Line][1] = com_data;
				Head = 2;
			}
			else
				Head = 0;
		}
		else if(Head==2)
		{
			if(com_data=='<')//��λ�����͸�MWC
			{
				RxBuff[Line][2] = com_data;
				Head = 3;
			}
			else
				Head = 0;
		}
		else
		{
			RxBuff[Line][Head] = com_data;
			Head ++;
		}
		
		if(Head==RxBuff[Line][3]+6)	//���ݽ������
		{
			Head = 0;
			if(Line)
			{ 
				Line = 0; //�л�����
				Line1 = 1;
			}
			else
			{
				Line = 1;
				Line0 = 1;
			}
		}
	}
}




/******************************************************************************
����ԭ��:	void Uart_Check(void)
��������:	���ڻ������ݴ���
*******************************************************************************/ 
void Uart_Check(void)
{
	uint8_t Line_address;
	if(Line0==1 || Line1==1)
	{
		if(Line0==1)//ȷ���������
		{
			Line0 = 0;Line_address = 0;
		}
		else
		{
			Line1 = 0;Line_address = 1;
		}
		
		switch (RxBuff[Line_address][4])
		{
			case MSP_SET_PID : //����PID����
					pid[0].kp = RxBuff[Line_address][5];
					pid[0].ki = RxBuff[Line_address][6];
					pid[0].kd = RxBuff[Line_address][7];

					pid[1].kp = RxBuff[Line_address][8];
					pid[1].ki = RxBuff[Line_address][9];
					pid[1].kd = RxBuff[Line_address][10];

					pid[2].kp = RxBuff[Line_address][11];
					pid[2].ki = RxBuff[Line_address][12];
					pid[2].kd = RxBuff[Line_address][13];
					Print_MSP_SET_PID();
			   Print_MSP_PID();

				break;
			case MSP_PID :	//��ȡPID����
				Print_MSP_PID();
				break;
			case MSP_ACC_CALIBRATION : 	//У�����ٶȼ�
				Do_ACC_Offset();
				break;
			case MSP_MAG_CALIBRATION : 	//ԭ��У�������ƣ���������У��������
				Do_GYRO_Offset();
				break;
			case MSP_RESET_CONF : 		//����PID
			{
				PID_Reset();
				Print_MSP_PID();
				//NRF_Send_TX(RxBuff[Line_address],32);
				break;
			}
		}
	}
}



/**************************ʵ�ֺ���********************************************
*******************************************************************************/
uint8_t Uart1_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;
  if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
	return DataToSend;
}
void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num)
{ u8 i;
	for(i=0;i<data_num;i++)
		TxBuffer[count++] = *(DataToSend+i);
	if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}
uint8_t Uart1_Put_Int16(uint16_t DataToSend)
{
	uint8_t sum = 0;
	TxBuffer[count++] = BYTE1(DataToSend);
	TxBuffer[count++] = BYTE0(DataToSend);
	if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	sum += BYTE1(DataToSend);
	sum += BYTE0(DataToSend);
	return sum;
}
uint8_t Uart1_Put_Float(float DataToSend)
{
	uint8_t sum = 0;
	floattobyte.num=DataToSend;
	TxBuffer[count++] = floattobyte.byte[3];  
	TxBuffer[count++] = floattobyte.byte[2];  
	TxBuffer[count++] = floattobyte.byte[1];  
	TxBuffer[count++] = floattobyte.byte[0];  
	if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	sum += BYTE3(DataToSend);
	sum += BYTE2(DataToSend);
	sum += BYTE1(DataToSend);
	sum += BYTE0(DataToSend);
	return sum;	
}

void Uart1_Put_String(unsigned char *Str)
{
	//�ж�Strָ��������Ƿ���Ч.
	while(*Str)
	{
	//�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
	if(*Str=='\r')Uart1_Put_Char(0x0d);
		else if(*Str=='\n')Uart1_Put_Char(0x0a);
			else Uart1_Put_Char(*Str);
	//ָ��++ ָ����һ���ֽ�.
	Str++;
	}
}


