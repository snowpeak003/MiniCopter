#include "usart3.h"	 

int16_t GyroData[3], AccData[3], AngleData[3], MagData[3];
float Gyro[3]={0.0f},Acc[3]={0.0f},Angle[3]={0.0f},Mag[3]={0.0f};
u8 M3C_OriData[11]={0};
u8 M3C_OriDataIndex=0;
int M3C_Data[9];
u8 M3C_captured=0;

//初始化IO 串口3
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率	  
void USART3_Init(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART2);

  /*
  *  Open_USART3_TX -> PB10 , Open_USART3_RX -PB11
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式
  USART_Init(USART3, &USART_InitStructure); ; //初始化串口
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断
  USART_Cmd(USART3, ENABLE);                    //使能串口 

}

void USART3_NVIC_Configuration(void)				//设置串口中断优先级
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void USART3_IRQHandler(void)
{ u8 inData=0;
if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
 {
	 inData=USART_ReceiveData(USART3);
	 if(inData==0xa7){
		 M3C_OriDataIndex=0;
		 M3C_OriData[M3C_OriDataIndex]=inData;
  	 M3C_OriData[10]=0xff;
	 }
	 else{
		 M3C_OriDataIndex++;
		 M3C_OriData[M3C_OriDataIndex]=inData;
	 }
	 
	 if(M3C_OriData[0]==0xa7&&M3C_OriData[10]!=0xff)
	 {M3C_captured=1;
	 }
	 else M3C_captured=0;
 }
} 


void M3C_Update(void)
{
 if(M3C_OriData[0]==0xA7)  //判断是否为帧 头
          {switch(M3C_OriData[2]) //帧类型判断
          {
            case 0x70: //标识为角速度帧
              GyroData[0] = (int16_t)(((uint16_t)M3C_OriData[5]| (uint16_t)M3C_OriData[6] << 8) );      //X轴
              GyroData[1] = (int16_t)(((uint16_t)M3C_OriData[7]| (uint16_t)M3C_OriData[8] << 8) );      //Y轴
              GyroData[2] = (int16_t)(((uint16_t)M3C_OriData[9]| (uint16_t)M3C_OriData[10] << 8) );     //Z轴
            break;
            case 0x71: //标识为加速度帧          
              AccData[0] = (int16_t)(((uint16_t)M3C_OriData[5]| (uint16_t)M3C_OriData[6] << 8) );      //X轴
              AccData[1] = (int16_t)(((uint16_t)M3C_OriData[7]| (uint16_t)M3C_OriData[8] << 8) );      //Y轴
              AccData[2] = (int16_t)(((uint16_t)M3C_OriData[9]| (uint16_t)M3C_OriData[10] << 8) );     //Z轴
            break;
            case 0x72: //标识为姿态帧
							
              AngleData[0] = (int16_t)(((uint16_t)M3C_OriData[5]| (uint16_t)M3C_OriData[6] << 8) );    //X轴角度(横滚)
              AngleData[1] = (int16_t)(((uint16_t)M3C_OriData[7]| (uint16_t)M3C_OriData[8] << 8) );    //Y轴角度(俯仰)
              AngleData[2] = (int16_t)(((uint16_t)M3C_OriData[9]| (uint16_t)M3C_OriData[10] << 8) );   //Z轴角度(偏航)
            break;
            case 0x73: //标识为 地磁帧

              MagData[0] = (int16_t)(((uint16_t)M3C_OriData[5]| (uint16_t)M3C_OriData[6] << 8) );      //X轴
              MagData[1] = (int16_t)(((uint16_t)M3C_OriData[7]| (uint16_t)M3C_OriData[8] << 8) );      //Y轴
              MagData[2] = (int16_t)(((uint16_t)M3C_OriData[9]| (uint16_t)M3C_OriData[10] << 8) );     //Z轴
            default:  break;
          }
        }
      }
void M3c_Cal(void)
{
Gyro[0]=GyroData[0]*4/16.4;
Gyro[1]=GyroData[1]*4/16.4;	
Gyro[2]=-GyroData[2]*4/16.4;
Angle[0]=AngleData[0]*0.01;
Angle[1]=AngleData[1]*0.01;
Angle[2]=AngleData[2]*0.01;	
}

	
  



















