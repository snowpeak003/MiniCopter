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

struct _acc  acc;			//ԭʼ����
struct _gyro gyro;
////////////////////////////////////////////
struct _acc  filter_acc;	//�˲�������
struct _gyro filter_gyro;
////////////////////////////////////////////
struct _acc  offset_acc={0, 0, 0};	//��ƫ����
struct _gyro offset_gyro={-480, 22, -97};
////////////////////////////////////////////
struct _SI_float  SI_acc;	//���ٶ����ݣ�m/s2��
struct _SI_float  SI_gyro;	//���ٶ����ݣ�rad��
////////////////////////////////////////////
struct _Rc Rc;				//ң��ͨ��
struct _out_angle out_angle;//��̬����-�Ƕ�ֵ

/* pid */
struct _pid pitch;
struct _pid roll;
struct _pid yaw;
struct _pid gyro_pitch;
struct _pid gyro_roll;
struct _pid gyro_yaw;

/******************************************************************************
����ԭ�ͣ�	void Task_1000HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ1000HZ����
*******************************************************************************/ 
void Task_1000HZ(void)
{
  MPU6050_DataANL();
	ACC_IIR_Filter(&acc,&filter_acc);//��acc��IIR�˲�
	Gyro_Filter(&gyro,&filter_gyro);//��gyro�������˲�
	Get_Radian(&filter_gyro,&SI_gyro);//���ٶ�����תΪ����
	IMUupdate(SI_gyro.x,SI_gyro.y,SI_gyro.z,filter_acc.x,filter_acc.y,filter_acc.z);//��̬����
}

/******************************************************************************
����ԭ�ͣ�	void Task_500HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ500HZ����
*******************************************************************************/ 
void Task_500HZ(void)
{
	Control_Gyro(&SI_gyro,&Rc,Rc_Lock);//�ڻ�����

}

/******************************************************************************
����ԭ�ͣ�	void Task_250HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ250HZ����
*******************************************************************************/ 
void Task_250HZ(void)
{

   Update_RcData();
	 Get_Eulerian_Angle(&out_angle);//��Ԫ��תŷ����
   Control_Angle(&out_angle,&Rc);//�⻷����

}
/******************************************************************************
����ԭ�ͣ�	void Task_50HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ50HZ����
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


		case 12:    turn_count=0; 	          break;//���ӵ�
	}

}			

/******************************************************************************
����ԭ�ͣ�	void Task_10HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ10HZ����
*******************************************************************************/ 
void Task_10HZ(void)
{
	static uint8_t turn_count_10Hz;
	static uint8_t led_count;
	//2Hz�¼�
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
