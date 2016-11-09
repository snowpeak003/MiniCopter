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

//static  uint8_t TxBuff[256];//串口发送缓冲区
uint8_t RxBuff[2][50];		//串口接收缓冲区
uint8_t Line0,Line1;	//串口接收双缓冲切换


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
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
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

void USART1_NVIC_Configuration(void)				//设置串口中断优先级
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{
		u8 com_data = USART1->DR;//USART_ClearFlag(USART1,USART_IT_ORE);
	}
	//发送中断
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART1->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE中断//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}
	//接收中断 (接收寄存器非空) /////////////////////////////////////////////////////////////////////////////////////////
	if(USART1->SR & (1<<5))//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    
	{
		static uint8_t Head = 0;
		static uint8_t Line = 0;
		uint8_t com_data = USART1->DR;
		
		if(Head==0)	//寻找帧头
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
			if(com_data=='<')//上位机发送给MWC
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
		
		if(Head==RxBuff[Line][3]+6)	//数据接收完毕
		{
			Head = 0;
			if(Line)
			{ 
				Line = 0; //切换缓存
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
函数原型:	void Uart_Check(void)
功　　能:	串口缓冲数据处理
*******************************************************************************/ 
void Uart_Check(void)
{
	uint8_t Line_address;
	if(Line0==1 || Line1==1)
	{
		if(Line0==1)//确定缓冲队列
		{
			Line0 = 0;Line_address = 0;
		}
		else
		{
			Line1 = 0;Line_address = 1;
		}
		
		switch (RxBuff[Line_address][4])
		{
			case MSP_SET_PID : //设置PID参数
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
			case MSP_PID :	//读取PID参数
				Print_MSP_PID();
				break;
			case MSP_ACC_CALIBRATION : 	//校正加速度计
				Do_ACC_Offset();
				break;
			case MSP_MAG_CALIBRATION : 	//原是校正磁力计，这里用来校正陀螺仪
				Do_GYRO_Offset();
				break;
			case MSP_RESET_CONF : 		//重置PID
			{
				PID_Reset();
				Print_MSP_PID();
				//NRF_Send_TX(RxBuff[Line_address],32);
				break;
			}
		}
	}
}



/**************************实现函数********************************************
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
	//判断Str指向的数据是否有效.
	while(*Str)
	{
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')Uart1_Put_Char(0x0d);
		else if(*Str=='\n')Uart1_Put_Char(0x0a);
			else Uart1_Put_Char(*Str);
	//指针++ 指向下一个字节.
	Str++;
	}
}


