#include "sys.h"	
#include "stm32f4xx.h"

	  		  	
extern u8 USART2_RX_BUF[64]; 		//���ջ���,���64���ֽ�
extern u8 USART2_RX_CNT;   			//���յ������ݳ���
extern u8 RC_SBUS_captured;
extern int16_t RC_CH[18];
//ģʽ����
#define SBUS_EN		PGout(9)	//485ģʽ����.0,����;1,����.
//����봮���жϽ��գ��벻Ҫע�����º궨��
//#define EN_USART2_RX 	1			//0,������;1,����.




void USART2_Init(void);
//void USART2_Send_Data(u8 *buf,u8 len);
//void USART2_Receive_Data(u8 *buf,u8 *len);
void FUTABA_SBUS_UpdateChannels(void);

//#endif	   
















