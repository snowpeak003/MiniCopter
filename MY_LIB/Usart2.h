#include "sys.h"	
#include "stm32f4xx.h"

	  		  	
extern u8 USART2_RX_BUF[64]; 		//接收缓冲,最大64个字节
extern u8 USART2_RX_CNT;   			//接收到的数据长度
extern u8 RC_SBUS_captured;
extern int16_t RC_CH[18];
//模式控制
#define SBUS_EN		PGout(9)	//485模式控制.0,接收;1,发送.
//如果想串口中断接收，请不要注释以下宏定义
//#define EN_USART2_RX 	1			//0,不接收;1,接收.




void USART2_Init(void);
//void USART2_Send_Data(u8 *buf,u8 len);
//void USART2_Receive_Data(u8 *buf,u8 *len);
void FUTABA_SBUS_UpdateChannels(void);

//#endif	   
















