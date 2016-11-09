#include "sys.h"	
#include "stm32f4xx.h"
extern float Gyro[3],Acc[3],Angle[3],Mag[3];
extern u8 M3C_captured;
void USART3_Init(void);
void USART3_NVIC_Configuration(void);
void USART3_IRQHandler(void);
void M3C_Update(void);
void M3c_Cal(void);
//#endif	   
















