#include "rc.h"
float RC_Target_ROL=0.0f,RC_Target_PIT=0.0f,RC_Target_YAW=0.0f;
u8 ARMED = 0;

void Update_RcData(void)
{
//	if(Rc_captured==1)
//		{
//		Rc_DataCal_UART();
//		Rc_DataAnl_PWMIn();
//		Rc_captured=0;
//		}
	Rc_DataAnl_PWMIn();
}

void Rc_DataAnl_PWMIn(void)
{
	Rc.ROLL			  =	Rc_Pwm_In[0];
	Rc.PITCH			=	Rc_Pwm_In[1];
	Rc.THROTTLE  	=	Rc_Pwm_In[2];
	Rc.YAW				=	Rc_Pwm_In[3];
	
	Rc.AUX1			=	Rc_Pwm_In[4];
	Rc.AUX2			=	Rc_Pwm_In[5];
	Rc.AUX3			=	Rc_Pwm_In[6];
	Rc.AUX4			=	Rc_Pwm_In[7];
//	Rc.AUX5			=	Rc_Pwm_In[8];
//	Rc.AUX6			=	Rc_Pwm_In[9];
}
//void Rc_DataAnl_UART(void)
//{
//	Rc.ROLL			  =	Rc_Pwm_In[0];
//	Rc.PITCH			=	Rc_Pwm_In[1];
//	Rc.THROTTLE  	=	Rc_Pwm_In[2];
//	Rc.YAW				=	Rc_Pwm_In[3];
//	
//	Rc.AUX1			=	Rc_Pwm_In[4];
//	Rc.AUX2			=	Rc_Pwm_In[5];
//	Rc.AUX3			=	Rc_Pwm_In[6];
//	Rc.AUX4			=	Rc_Pwm_In[7];
////	Rc.AUX5			=	Rc_Pwm_In[8];
////	Rc.AUX6			=	Rc_Pwm_In[9];
//}
//矫正或者补偿在此函数
void Rc_DataCal(void)
{
	RC_Target_ROL = (Rc.ROLL-1523)*0.04;
	RC_Target_PIT = (Rc.PITCH-1523)*0.04;
	RC_Target_YAW = (Rc.YAW-1523)*0.04;
}

//void Rc_DataCal_UART(void)
//{
//		Rc_Pwm_In[0] = (int16_t)(((uint16_t)RC_UART_OriData[2]| (uint16_t)RC_UART_OriData[3] << 8 ) );      
//		Rc_Pwm_In[1] = (int16_t)(((uint16_t)RC_UART_OriData[4]| (uint16_t)RC_UART_OriData[5] << 8 ) );   
//		Rc_Pwm_In[2] = (int16_t)(((uint16_t)RC_UART_OriData[6]| (uint16_t)RC_UART_OriData[7] << 8 ) );   
//		Rc_Pwm_In[3] = (int16_t)(((uint16_t)RC_UART_OriData[8]| (uint16_t)RC_UART_OriData[9] << 8 ) );   
//		Rc_Pwm_In[4] = (int16_t)(((uint16_t)RC_UART_OriData[10]| (uint16_t)RC_UART_OriData[11] << 8 ) );   
//		Rc_Pwm_In[5] = (int16_t)(((uint16_t)RC_UART_OriData[12]| (uint16_t)RC_UART_OriData[13] << 8 ) );   
//		Rc_Pwm_In[6] = (int16_t)(((uint16_t)RC_UART_OriData[14]| (uint16_t)RC_UART_OriData[15] << 8 ) );   
//   	Rc_Pwm_In[7] = (int16_t)(((uint16_t)RC_UART_OriData[16]| (uint16_t)RC_UART_OriData[17] << 8 ) );   
//}



