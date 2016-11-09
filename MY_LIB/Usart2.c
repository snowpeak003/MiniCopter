#include "usart2.h"	 



////接收缓存区 	
//u8 USART2_RX_BUF[64];  	//接收缓冲,最大64个字节.
////接收到的数据长度
//u8 USART2_RX_CNT=0; 

int16_t RC_CH[18];
u8 SBUS_DATA[25]={0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};	
u8 SBUS_DATAIndex=0;
u8 RC_SBUS_captured=0;

//初始化IO 串口2
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率	  
void USART2_Init(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  /*
  *  Open_USARTx_TX -> PA2 , Open_USARTx_RX -PA3
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = 100000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//两个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式
  USART_Init(USART2, &USART_InitStructure); ; //初始化串口
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
  USART_Cmd(USART2, ENABLE);                    //使能串口 

}

void USART2_NVIC_Configuration(void)				//设置串口中断优先级
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//响应3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void USART2_IRQHandler(void)
{ u8 inData=0;
if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
 {
	 inData=USART_ReceiveData(USART2);
	 if(inData==0x0f){
		 SBUS_DATAIndex=0;
		 SBUS_DATA[SBUS_DATAIndex]=inData;
		 SBUS_DATA[24]=0xff;
	 }
	 else{
		 SBUS_DATAIndex++;
		 SBUS_DATA[SBUS_DATAIndex]=inData;
	 }
	 if(SBUS_DATA[0]==0x0f&&SBUS_DATA[24]==0x00)
	 {RC_SBUS_captured=1;
	 }
	 else RC_SBUS_captured=0;
 }
} 

////发送len个字节.
////buf:发送区首地址
////len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
//void USART2_Send_Data(u8 *buf,u8 len)
//{
//	u8 t;
//	USART2_TX_EN=1;			//设置为发送模式
//  	for(t=0;t<len;t++)		//循环发送数据
//	{		   
//		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
//		USART_SendData(USART2,buf[t]);
//	}	 
// 
//	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);		
//	RS485_RX_CNT=0;	  
//	RS485_TX_EN=0;				//设置为接收模式	
//}
//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
//void USART_Receive_Data(u8 *buf,u8 *len)
//{
//	u8 rxlen=USART2_RX_CNT;
//	u8 i=0;
//	*len=0;				//默认为0
// if(rxlen==USART2_RX_CNT&&rxlen)//接收到了数据,且接收完成了
//	{
//		for(i=0;i<rxlen;i++)
//		{
//			buf[i]=USART2_RX_BUF[i];	
//		}		
//		*len=USART2_RX_CNT;	//记录本次数据长度
//		USART2_RX_CNT=0;		//清零
//	}
//}l

//u8还是u16
void FUTABA_SBUS_UpdateChannels(void)
{
  RC_CH[0]  = ((uint16_t)(SBUS_DATA[1]|(uint16_t)SBUS_DATA[2]<< 8) & 0x07FF);
  RC_CH[1]  = ((uint16_t)(SBUS_DATA[2]>>3|(uint16_t)SBUS_DATA[3]<<5) & 0x07FF);
  RC_CH[2]  = ((uint16_t)(SBUS_DATA[3]>>6|(uint16_t)SBUS_DATA[4]<<2|SBUS_DATA[5]<<10) & 0x07FF);
  RC_CH[3]  = ((uint16_t)(SBUS_DATA[5]>>1|(uint16_t)SBUS_DATA[6]<<7) & 0x07FF);
  RC_CH[4]  = ((uint16_t)(SBUS_DATA[6]>>4|(uint16_t)SBUS_DATA[7]<<4) & 0x07FF);
  RC_CH[5]  = ((uint16_t)(SBUS_DATA[7]>>7|(uint16_t)SBUS_DATA[8]<<1|SBUS_DATA[9]<<9) & 0x07FF);
  RC_CH[6]  = ((uint16_t)(SBUS_DATA[9]>>2|(uint16_t)SBUS_DATA[10]<<6) & 0x07FF);
  RC_CH[7]  = ((uint16_t)(SBUS_DATA[10]>>5|(uint16_t)SBUS_DATA[11]<<3) & 0x07FF); // & the other 8 + 2 channels if you need them

//  channels[8]  = ((sbusData[12]|sbusData[13]<< 8) & 0x07FF);
//  channels[9]  = ((sbusData[13]>>3|sbusData[14]<<5) & 0x07FF);
//  channels[10] = ((sbusData[14]>>6|sbusData[15]<<2|sbusData[16]<<10) & 0x07FF);
//  channels[11] = ((sbusData[16]>>1|sbusData[17]<<7) & 0x07FF);
//  channels[12] = ((sbusData[17]>>4|sbusData[18]<<4) & 0x07FF);
//  channels[13] = ((sbusData[18]>>7|sbusData[19]<<1|sbusData[20]<<9) & 0x07FF);
//  channels[14] = ((sbusData[20]>>2|sbusData[21]<<6) & 0x07FF);
//  channels[15] = ((sbusData[21]>>5|sbusData[22]<<3) & 0x07FF);

}



















