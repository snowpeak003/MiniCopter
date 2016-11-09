#include "IMU.h"
#include "Tasks.h"
#include "Control.h"
#include "struct_all.h"
#include "MPU6050.h"
#include "Filter.h"
#include "Mymath.h"
#include "rc.h"
#include "led.h"
#include "usart6.h"
#include "tim_pwm_out.h"
#include "config.h"
#include "struct_all.h"
uint8_t Bsp_Int_Ok = 0;

struct _acc  acc;			//原始数据
struct _gyro gyro;
////////////////////////////////////////////
struct _acc  filter_acc;	//滤波后数据
struct _gyro filter_gyro;
////////////////////////////////////////////
struct _acc  offset_acc={0, 0, 0};	//零偏数据
struct _gyro offset_gyro={-480, 22, -97};
////////////////////////////////////////////
struct _SI_float  SI_acc;	//加速度数据（m/s2）
struct _SI_float  SI_gyro;	//角速度数据（rad）
////////////////////////////////////////////
struct _Rc Rc;				//遥控通道
struct _out_angle out_angle;//姿态解算-角度值

/* pid */
struct _pid pitch;
struct _pid roll;
struct _pid yaw;
struct _pid gyro_pitch;
struct _pid gyro_roll;
struct _pid gyro_yaw;

/******************************************************************************
函数原型：	void Task_1000HZ(void)
功    能：	主循环中运行频率为1000HZ任务
*******************************************************************************/ 
void Task_1000HZ(void)
{
  MPU6050_DataANL();
	ACC_IIR_Filter(&acc,&filter_acc);//对acc做IIR滤波
	Gyro_Filter(&gyro,&filter_gyro);//对gyro做窗口滤波
	Get_Radian(&filter_gyro,&SI_gyro);//角速度数据转为弧度
	IMUupdate(SI_gyro.x,SI_gyro.y,SI_gyro.z,filter_acc.x,filter_acc.y,filter_acc.z);//姿态解算
}

/******************************************************************************
函数原型：	void Task_500HZ(void)
功    能：	主循环中运行频率为500HZ任务
*******************************************************************************/ 
void Task_500HZ(void)
{
	Control_Gyro(&SI_gyro,&Rc,Rc_Lock);//内环控制

}

/******************************************************************************
函数原型：	void Task_250HZ(void)
功    能：	主循环中运行频率为250HZ任务
*******************************************************************************/ 
void Task_250HZ(void)
{

   Update_RcData();
	 Get_Eulerian_Angle(&out_angle);//四元数转欧拉角
   Control_Angle(&out_angle,&Rc);//外环控制

}
/******************************************************************************
函数原型：	void Task_50HZ(void)
功    能：	主循环中运行频率为50HZ任务
*******************************************************************************/ 
void Task_50HZ(void)
{
	static uint8_t turn_count;
	//	Print_MSP_PID();
	turn_count++;
	switch(turn_count)
	{
		case 1: 	  Print_MSP_RAW_IMU();      break;
	  case 3:     Print_MSP_ATTITUDE();     break;
    case 5:     Print_MSP_RC();           break;
    case 7:     Print_MSP_MOTOR();		    break;
		case 9:     Print_MSP_IDENT();        break;
//    case 6:    	Print_MSP_PID();	      break;		
		case 11:     Print_MSP_MOTOR_PINS();	  break;


		case 12:    turn_count=0; 	          break;//随便加的
	}

}			

/******************************************************************************
函数原型：	void Task_10HZ(void)
功    能：	主循环中运行频率为10HZ任务
*******************************************************************************/ 
void Task_10HZ(void)
{
	static uint8_t turn_count_10Hz;
	static uint8_t led_count;
	//2Hz事件
	turn_count_10Hz++;
  if(turn_count_10Hz>=5)
	{
		turn_count_10Hz=0;
		if(Rc_Lock==1)
		Led_Single(led_count%5,1);
		else 
		Led_Control(5,led_count%2);
		led_count++;
	}
		
}			
