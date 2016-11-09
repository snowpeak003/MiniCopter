#include "sys.h"	
#include "stm32f4xx.h"
#include "string.h"
#include "protocol.h"
#include "rc.h"
#include "mpu6050.h"
extern u8 Rc_captured;
extern u8 RC_UART_OriData[20];
void USART6_Init(void);
void USART6_NVIC_Configuration(void);
void USART6_IRQHandler(void);
void Uart6_Put_String(unsigned char *Str);
void Uart6_Check(void);
//#endif	   
















