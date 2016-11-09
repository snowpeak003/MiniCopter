#include "Mymath.h"
#include "Control.h"
#include "struct_all.h"
#include "tim_pwm_out.h"
uint8_t Rc_Lock=0;//1������0����
int16_t throttle1,throttle2,throttle3,throttle4;
void Control_Init(void)
{
	roll.kp=4.9f; pitch.kp=4.9f; yaw.kp=0;
	roll.ki=0.02f; pitch.ki=0.02f;  yaw.ki=0;
	roll.kd=8.0f; pitch.kd=8.0f;  yaw.kd=0;   
	gyro_roll.kp=1.4f;	gyro_pitch.kp=1.4f; gyro_yaw.kp=0.5f;
	gyro_roll.ki=0.0f;	gyro_pitch.ki=0.0f; gyro_yaw.ki=0.007f;
	gyro_roll.kd=2.0f; gyro_pitch.kd=2.0f; gyro_yaw.kd=0.0f;
}

/******************************************************************************
����ԭ�ͣ�	void RC_Limit(struct _Rc *rc)
��    �ܣ�	����ң��ָ�Χ
*******************************************************************************/ 
void RC_Limit(struct _Rc *rc)
{
	rc->THROTTLE = (rc->THROTTLE<=1000)?1000:rc->THROTTLE; 
	rc->THROTTLE = (rc->THROTTLE>=2000)?2000:rc->THROTTLE; 
	rc->PITCH = (rc->PITCH<=1000)?1000:rc->PITCH; 
	rc->PITCH = (rc->PITCH>=2000)?2000:rc->PITCH; 
	rc->ROLL = (rc->ROLL<=1000)?1000:rc->ROLL; 
	rc->ROLL = (rc->ROLL>=2000)?2000:rc->ROLL; 
	rc->YAW  = (rc->YAW<=1000)?1000:rc->YAW; 
	rc->YAW  = (rc->YAW>=2000)?2000:rc->YAW; 
	rc->AUX1 = (rc->AUX1<=1000)?1000:rc->AUX1; 
	rc->AUX1 = (rc->AUX1>=2000)?2000:rc->AUX1; 
	rc->AUX2 = (rc->AUX2<=1000)?1000:rc->AUX2; 
	rc->AUX2 = (rc->AUX2>=2000)?2000:rc->AUX2; 
	rc->AUX3 = (rc->AUX3<=1000)?1000:rc->AUX3; 
	rc->AUX3 = (rc->AUX3>=2000)?2000:rc->AUX3; 
}
	
/******************************************************************************
����ԭ�ͣ�	void RC_LOCK(void)
��    �ܣ�	ң������ָ�������������ע����������֣�ң��Ĭ�ϵľ��������֣�
*******************************************************************************/ 
//void RC_LOCK(void)
//{
//	static uint8_t count0,count1,count2;
//	if(Rc.THROTTLE<1300 )//&& Rc.YAW>1700 && Rc.PITCH>1400 && Rc.PITCH<1600)
//		count0++;
//	else
//		count0 = 0;
//	
//	if(count0>15 && Rc_Lock==1)
//	{
//		Rc_Lock = 0;//���������ֵ�ҡ�˴������½ǣ�����ҡ�˲���������
////		LED3_OFF;
////		Delay_led(100);
////		LED3_ON;
////		Delay_led(100);
//	}
//////////////////////////////////////////////////	
//	if(Rc.THROTTLE<1300 && Rc.YAW<1300 && Rc.PITCH>1400 && Rc.PITCH<1600)
//		count1++;
//	else
//		count1 = 0;
//	
//	if(count1>150 && Rc_Lock==0)
//	{
//		Rc_Lock = 1;//���������ֵ�ҡ�˴������½ǣ�����ҡ�˲���������
////		LED3_OFF;
////		Delay_led(100);
////		LED3_ON;
////		Delay_led(100);
//	}
//////////////////////////////////////////////////	
//	if(Rc.THROTTLE<1300 && Rc.YAW<1300 && Rc.PITCH<1300)
//		count2++;
//	else
//		count2 = 0;
//	
//	if(count2>100 )//&& Rc_Lock)//����״̬����У��
//	{
////		count2=0;
////		Do_GYRO_Offset();//���������ֵ�ҡ�˴������½ǣ�����ҡ�˸�������������ף�У��������
////		Do_ACC_Offset(); //���������ֵ�ҡ�˴������½ǣ�����ҡ�˸�������������ף�У�����ٶȼ�
////		
////		LED3_OFF;
////		Delay_led(50);
////		LED3_ON;
////		Delay_led(50);
////		LED3_OFF;
////		Delay_led(50);
////		LED3_ON;
//	}
//}


#define angle_max 	 		10.0f	
#define angle_integral_max 	1000.0f	
/******************************************************************************
����ԭ�ͣ�	void Control_Angle(struct _out_angle *angle,struct _Rc *rc)
��    �ܣ�	PID������(�⻷)
*******************************************************************************/ 
void Control_Angle(struct _out_angle *angle,struct _Rc *rc)
{
	static struct _out_angle control_angle;
	static struct _out_angle last_angle;
//////////////////////////////////////////////////////////////////
//			����Ϊ�ǶȻ�
//////////////////////////////////////////////////////////////////
	if(rc->ROLL>1480 && rc->ROLL<1530)	
		rc->ROLL=1500;
	if(rc->PITCH>1480 && rc->PITCH<1530)	
		rc->PITCH=1500;
		if(rc->YAW>1400 && rc->YAW<1600)
		rc->YAW=1500;
//////////////////////////////////////////////////////////////////
	if(rc->AUX1>1495 && rc->AUX1<1505)	
		rc->AUX1=1500;
	if(rc->AUX2>1495 && rc->AUX2<1505)	
		rc->AUX2=1500;
//////////////////////////////////////////////////////////////////
	control_angle.roll  = angle->roll  - (rc->ROLL  -1500)/13.0f; //+ (rc->AUX2 -1500)/100.0f;
	control_angle.pitch = angle->pitch - (rc->PITCH -1500)/13.0f ;//- (rc->AUX1 -1500)/100.0f;
	control_angle.yaw   = angle->yaw - (rc->YAW -1500)/13.0f  ;//+ (rc->AUX3 -1500)/50.0f;//����βģʽ
//////////////////////////////////////////////////////////////////
	if(control_angle.roll >  angle_max)	//ROLL
		roll.integral +=  angle_max;
	if(control_angle.roll < -angle_max)
		roll.integral += -angle_max;
	else
		roll.integral += control_angle.roll;
	
	if(roll.integral >  angle_integral_max)
	   roll.integral =  angle_integral_max;
	if(roll.integral < -angle_integral_max)
	   roll.integral = -angle_integral_max;
//////////////////////////////////////////////////////////////////
	if(control_angle.pitch >  angle_max)//PITCH
	   pitch.integral +=  angle_max;
	if(control_angle.pitch < -angle_max)
	   pitch.integral += -angle_max;
	else
		pitch.integral += control_angle.pitch;

	if(pitch.integral >  angle_integral_max)
	   pitch.integral =  angle_integral_max;
	if(pitch.integral < -angle_integral_max)
	   pitch.integral = -angle_integral_max;
//////////////////////////////////////////////////////////////////
	if(rc->THROTTLE<1200)//���Ž�Сʱ����������
	{
		roll.integral  = 0;
		pitch.integral = 0;
	}
//////////////////////////////////////////////////////////////////
	roll.output  = roll.kp *control_angle.roll  + roll.ki *roll.integral  + roll.kd *(control_angle.roll -last_angle.roll );
	pitch.output = pitch.kp*control_angle.pitch + pitch.ki*pitch.integral + pitch.kd*(control_angle.pitch-last_angle.pitch);
 // yaw.output   = yaw.kp*control_angle.yaw                                   + yaw.kd*(control_angle.yaw-last_angle.yaw);

	last_angle.roll =control_angle.roll;
	last_angle.pitch=control_angle.pitch;
	last_angle.yaw=control_angle.yaw;
}

#define gyro_max 	 		50.0f	
#define gyro_integral_max 	5000.0f
/******************************************************************************
����ԭ�ͣ�	void Control_Gyro(struct _SI_float *gyro,struct _Rc *rc,uint8_t Lock)
��    �ܣ�	PID������(�ڻ�)
*******************************************************************************/ 
void Control_Gyro(struct _SI_float *gyro,struct _Rc *rc,uint8_t Lock)
{
	static struct _out_angle control_gyro;
	static struct _out_angle last_gyro;

//////////////////////////////////////////////////////////////////
//			����Ϊ���ٶȻ�
//////////////////////////////////////////////////////////////////
	if(rc->YAW>1400 && rc->YAW<1600)
		rc->YAW=1500;
	if(rc->AUX3>1495 && rc->AUX3<1505)	
		rc->AUX3=1500;
//////////////////////////////////////////////////////////////////
	control_gyro.roll  = -roll.output - gyro->y*Radian_to_Angle;
	control_gyro.pitch = pitch.output - gyro->x*Radian_to_Angle;
	
//	if(rc->AUX4 & Lock_Mode)
//		control_gyro.yaw   = - gyro->z*Radian_to_Angle - (rc->AUX3 -1500)/100.0f;//��βģʽ
//	else
		control_gyro.yaw   = -(rc->YAW-1500)/2.0f - gyro->z*Radian_to_Angle; //+ (rc->AUX3 -1500)/50.0f;//����βģʽ
//////////////////////////////////////////////////////////////////
	if(control_gyro.roll >  gyro_max)	//GYRO_ROLL
		gyro_roll.integral +=  gyro_max;
	if(control_gyro.roll < -gyro_max)
		gyro_roll.integral += -gyro_max;
	else
		gyro_roll.integral += control_gyro.roll;
	
	if(gyro_roll.integral >  gyro_integral_max)
	   gyro_roll.integral =  gyro_integral_max;
	if(gyro_roll.integral < -gyro_integral_max)
	   gyro_roll.integral = -gyro_integral_max;
//////////////////////////////////////////////////////////////////
	if(control_gyro.pitch >  gyro_max)//GYRO_PITCH
		gyro_pitch.integral +=  gyro_max;
	if(control_gyro.pitch < -gyro_max)
		gyro_pitch.integral += -gyro_max;
	else
		gyro_pitch.integral += control_gyro.pitch;
	
	if(gyro_pitch.integral >  gyro_integral_max)
	   gyro_pitch.integral =  gyro_integral_max;
	if(gyro_pitch.integral < -gyro_integral_max)
	   gyro_pitch.integral = -gyro_integral_max;
////////////////////////////////////////////////////////////////
	if(control_gyro.yaw >  gyro_max)//GYRO_YAW
		gyro_yaw.integral +=  gyro_max;
	if(control_gyro.yaw < -gyro_max)
		gyro_yaw.integral += -gyro_max;
	else
		gyro_yaw.integral += control_gyro.yaw;
	
	if(gyro_yaw.integral >  gyro_integral_max)
	   gyro_yaw.integral =  gyro_integral_max;
	if(gyro_yaw.integral < -gyro_integral_max)
	   gyro_yaw.integral = -gyro_integral_max;
//////////////////////////////////////////////////////////////////
	if(rc->THROTTLE<1200)//���Ž�Сʱ����������
	{
		gyro_yaw.integral  = 0;
	}
//////////////////////////////////////////////////////////////////
	gyro_roll.output  = gyro_roll.kp *control_gyro.roll  + gyro_roll.ki *gyro_roll.integral  + gyro_roll.kd *(control_gyro.roll -last_gyro.roll );
	gyro_pitch.output = gyro_pitch.kp*control_gyro.pitch + gyro_pitch.ki*gyro_pitch.integral + gyro_pitch.kd*(control_gyro.pitch-last_gyro.pitch);
	gyro_yaw.output   = gyro_yaw.kp  *control_gyro.yaw   + gyro_yaw.ki  *gyro_yaw.integral   + gyro_yaw.kd  *(control_gyro.yaw  -last_gyro.yaw  );
//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
//3      1
//   x
//2      4
//
	
		if(rc->THROTTLE>1100&&Rc_Lock==0)//if(rc->THROTTLE>1200 && Lock==0)
	{
		throttle1 = (int)(rc->THROTTLE-950  - gyro_roll.output+ gyro_pitch.output  - gyro_yaw.output);
		throttle2 = (int)(rc->THROTTLE-950  + gyro_roll.output- gyro_pitch.output  - gyro_yaw.output);
		throttle3 = (int)(rc->THROTTLE-950  + gyro_roll.output+ gyro_pitch.output  + gyro_yaw.output);
		throttle4 = (int)(rc->THROTTLE-950  - gyro_roll.output- gyro_pitch.output  + gyro_yaw.output);
	}
	else
	{
		throttle1=1;
		throttle2=2;
		throttle3=3;
		throttle4=4;
	}
	PWMWriteServo(throttle1,throttle2,throttle3,throttle4);
	//////////////////////////////////////////////////////////////////
	last_gyro.roll =control_gyro.roll;
	last_gyro.pitch=control_gyro.pitch;
	last_gyro.yaw  =control_gyro.yaw;

}
