#include "stm32f4xx.h"
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

void Usart1_Init(u32 baudrate);
void USART1_NVIC_Configuration(void);
void USART1_IRQHandler(void);
void Uart1_Put_String(unsigned char *Str);
uint8_t Uart1_Put_Char(unsigned char DataToSend);
uint8_t Uart1_Put_Int16(uint16_t DataToSend);
uint8_t Uart1_Put_Float(float DataToSend);
void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num);
void Data_Send_Temp(long temp,unsigned char Series);
//void Data_Send_Vcan(u8 temp);
//void Send_Data(u8 *buf,u8 len);
void Uart_Check(void);

