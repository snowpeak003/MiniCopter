#include "usart6.h"	 
//u8 RC_UART_OriData[20]={0};
//u8 RC_UART_OriDataIndex=0;
///*******************************************************************************
//* Function Name  : USART_Configuration
//* Description    : Configure Open_USARTx 
//* Input          : None
//* Output         : None
//* Return         : None
//* Attention		 : None
//*******************************************************************************/
u8 UART6_TxBuffer[256];
u8 UART6_TxCounter=0;
u8 UART6_count=0; 
uint8_t UART6_RxBuff[2][50];		//���ڽ��ջ�����
uint8_t UART6_Line0,UART6_Line1;	//���ڽ���˫�����л�

extern uint8_t Rc_Lock;

//��ʼ��IO ����6
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������	  
void USART6_Init(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

  /*
  *  Open_USART6_TX -> PC6 , Open_USART6_RX -PC7
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ
  USART_Init(USART6, &USART_InitStructure); ; //��ʼ������
	
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//���������ж�
//	USART_ITConfig(USART6, USART_IT_TXE, ENABLE);//���������ж� �������ݼĴ������ж�TXE
  USART_Cmd(USART6, ENABLE);                    //ʹ�ܴ��� 

}

void USART6_NVIC_Configuration(void)				//���ô����ж����ȼ�
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//u8 inData_last=0;
void USART6_IRQHandler(void)
{ 
	if(USART_GetITStatus(USART6,USART_IT_TXE)!=RESET)
	{
		USART6->DR = UART6_TxBuffer[UART6_TxCounter++]; //дDR����жϱ�־          
		if(UART6_TxCounter == UART6_count)
		{
				USART_ITConfig(USART6,USART_IT_TXE,DISABLE);		//�ر�TXE�ж�
		}
	}
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	 {
		static uint8_t Head = 0;
		static uint8_t Line = 0;
		uint8_t com_data;
		com_data = USART_ReceiveData(USART6);//		uint8_t com_data = USART1->DR;
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);//��������жϱ�־		
		if(Head==0)	//Ѱ��֡ͷ
		{
			if(com_data=='$')	
			{
				UART6_RxBuff[Line][0] = com_data;
				Head = 1;
			}
		}
		else if(Head==1)
		{
			if(com_data=='M')	
			{
				UART6_RxBuff[Line][1] = com_data;
				Head = 2;
			}
			else
				Head = 0;
		}
		else if(Head==2)
		{
			if(com_data=='<')//��λ�����͸�MWC
			{
				UART6_RxBuff[Line][2] = com_data;
				Head = 3;
			}
			else
				Head = 0;
		}
		else
		{
			UART6_RxBuff[Line][Head] = com_data;
			Head ++;
		}
		
		if(Head==UART6_RxBuff[Line][3]+6)	//���ݽ������
		{
			Head = 0;
			if(Line)
			{ 
				Line = 0; //�л�����
				UART6_Line1 = 1;
			}
			else
			{
				Line = 1;
				UART6_Line0 = 1;
			}
		}
	 }
} 

void Uart6_Check(void)
{
	uint8_t Line_address;
	if(UART6_Line0==1 || UART6_Line1==1)
	{
		if(UART6_Line0==1)//ȷ���������
		{
			UART6_Line0 = 0;Line_address = 0;
		}
		else
		{
			UART6_Line1 = 0;Line_address = 1;
		}
		
		switch (UART6_RxBuff[Line_address][4])
		{
		 case MSP_SET_4CON:	//UdataBuf
			 Rc_Pwm_In[2]=UART6_RxBuff[Line_address][6]<<8 | UART6_RxBuff[Line_address][5];
			 Rc_Pwm_In[3]=UART6_RxBuff[Line_address][8]<<8 | UART6_RxBuff[Line_address][7];
			 Rc_Pwm_In[1]=UART6_RxBuff[Line_address][10]<<8 | UART6_RxBuff[Line_address][9];
			 Rc_Pwm_In[0]=UART6_RxBuff[Line_address][12]<<8 | UART6_RxBuff[Line_address][11];
		 break;
		 case MSP_ARM_IT://arm,����
			 Rc_Lock=0;
			 break;
		 case MSP_DISARM_IT://disarm,����
			 Rc_Lock=1;
			 break;
		 case MSP_ACC_CALIBRATION://У׼���ٶȼ�
			 Do_ACC_Offset();
			 break;
//		 case MSP_FLY_STATE:	
////			 flyLogApp=1;
//			 break;
//		 case MSP_HEAD_FREE:
////			 SetHeadFree(1);
//			 break;
//		 case MSP_STOP_HEAD_FREE:
////			 SetHeadFree(0);
//			 break;
//		 case MSP_LAND_DOWN:		//�Զ�����
////			 altCtrlMode=LANDING;
//			 break;
		}
	}
}


///**************************ʵ�ֺ���********************************************
//*******************************************************************************/
//uint8_t Uart6_Put_Char(unsigned char DataToSend)
//{
//	UART6_TxBuffer[UART6_count++] = DataToSend;
//  if(!(USART6->CR1 & USART_CR1_TXEIE))
//		USART_ITConfig(USART6, USART_IT_TXE, ENABLE); 
//	return DataToSend;
//}

//void Uart6_Put_String(unsigned char *Str)
//{
//	//�ж�Strָ��������Ƿ���Ч.
//	while(*Str)
//	{
//	//�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
//	if(*Str=='\r')Uart6_Put_Char(0x0d);
//		else if(*Str=='\n')Uart6_Put_Char(0x0a);
//			else Uart6_Put_Char(*Str);
//	//ָ��++ ָ����һ���ֽ�.
//	Str++;
//	}
//}














