#include "config.h"
int main(void)
{	
	Led_Init();
	led_delay(12  00);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Usart1_Init(115200);
	USART1_NVIC_Configuration();
//	USART3_Init();
//	USART3_NVIC_Configuration();
	USART6_Init();
	USART6_NVIC_Configuration();
	Tim3_Pwm_Out_Init();
//	Tim4_Pwm_Out_Init();
//	ppm_hw_init();
//	Tim_Pwm_In_Init(); 
	Timer_Sys_Init(999);//1000hz
//	SPI1_Init();//与 Nano pi 通信
  printf("SystemInitialized\r\n");
	MPU6050_Init();
	Calculate_FilteringCoefficient(0.001f,10.f);//计算IIR滤波器参数
	Control_Init();
//  ESC_Calibration();//电调油门校准
	PWMWriteServo(100,100,100,100);
//  Uart6_Put_String("AT+NAME DroneFXF\r\n");	
	led_delay(500);
	PWMWriteServo(0,0,0,0);	

	while (1)
	{
    Uart_Check();
    Uart6_Check();
		if(Count_1ms>=1)
		{	
			Count_1ms = 0;
			Task_1000HZ();
			//printf("%d,%d,%d\r\n",filter_acc.x,filter_acc.y,filter_acc.z);
		}
		if(Count_2ms>=2)
		{
			Count_2ms = 0;
			Task_500HZ();
		}
		if(Count_4ms>=4)
		{
			Count_4ms = 0;
			Task_250HZ();
//       printf("%f,%f,%f\r\n",out_angle.yaw,out_angle.pitch,out_angle.roll);
//			 printf("%d,%d,%d,%d\r\n",Rc.ROLL,Rc.PITCH,Rc.THROTTLE,Rc.YAW);
//			if(spi_cnt==0)
//			printf("%x,%x,%x,%x,%x\r\n",spi_buff[0],spi_buff[1],spi_buff[2],spi_buff[3],spi_buff[4]);
		}
		if(Count_20ms>=20)
		{
			Count_20ms = 0;
			Task_50HZ();

		}
		if(Count_100ms>=100)
		{
			Count_100ms = 0;
			Task_10HZ();
		}
		
	}
}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
////	void Update_SBUSData(void)
////{
////	if(RC_SBUS_captured)
////	{
////		FUTABA_SBUS_UpdateChannels();
////		
////	}
////}

//void Update_RcData(void)
//{
//	if(Rc_captured==1)
//		{
//		Rc_DataAnl();
//		Rc_DataCal();
//		Rc_captured=0;
//		}
//}
//	void Update_IMUData(void)
//{
//	if(M3C_captured)
//	{
//	 M3C_Update();
//	 M3c_Cal();
////	Protocol();//
//	 M3C_captured=0;
//	}
//}

//void Controller_PID(void)
//{
//	RowOut=(PID_P*(RC_Target_ROL-Angle[0])-PID_D*Gyro[0]);
//	PitchOut=(PID_P*(RC_Target_PIT-Angle[1])-PID_D*Gyro[1]);
//  YawOut=(5*RC_Target_YAW-0.6f*	Gyro[2]);
//}

//void Controller_MtoTheta(void)
//{
//	Theta[0]=RowOut  -YawOut;
//	Theta[1]=PitchOut-YawOut;
//	Theta[2]=RowOut  +YawOut;
//	Theta[3]=PitchOut+YawOut;
//}

//void UpdateServo(void)
//{
//	ServoPWM[0]=(u16)(400*( Theta[0]/93.04f+1.5f+ServoErr[0]));
//	ServoPWM[1]=(u16)(400*( Theta[1]/93.04f+1.5f+ServoErr[1]));
//	ServoPWM[2]=(u16)(400*(-Theta[0]/93.04f+1.5f+ServoErr[2]));
//	ServoPWM[3]=(u16)(400*(-Theta[0]/93.04f+1.5f+ServoErr[3]));
//	PWMWriteServo(ServoPWM[0],ServoPWM[1],ServoPWM[2],ServoPWM[3]);
//	Protocol();//
//}

//void Protocol(void)
//{
//	//printf("%d,%d,%d,%d\r\n",Rc_Data.ROLL,Rc_Data.PITCH,Rc_Data.THROTTLE,Rc_Data.YAW);
//	 //printf("%f,%f,%f\r\n",Angle[0],Angle[1],Angle[2]);
//	//printf("%d,%d,%d,%d\r\n",ServoPWM[0],ServoPWM[1],ServoPWM[2],ServoPWM[3]);
//}

void led_delay(u16 _ms)
{
	u16 i,j,k;
	for(i=_ms;i>0;i--)
	{
		for(j=5000;j>0;j--)
			for(k=2;k>0;k--)
			;
	}
}

void ESC_Calibration(void)
{
	PWMWriteServo(2000,2000,2000,2000);
	led_delay(9999);
	PWMWriteServo(1000,1000,1000,1000);	
	led_delay(9999);
}


