
#include "mpu6050.h"
#include "i2c_soft.h"
#include "led.h"

u8 mpu6050_buffer[14];	              // 读出来的数据
uint8_t ACC_OFFSET_OK = 0;	          // 是否需要ACC调零标志
uint8_t GYRO_OFFSET_OK = 0;           // 是否需要GYRO调零标志

void MPU6050_DataANL(void)
{
	MPU6050_Read();
	acc.x=((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - offset_acc.x;
	acc.y=((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - offset_acc.y;
	acc.z=((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) - offset_acc.z;
	//跳过温度ADC
	gyro.x=((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9])   - offset_gyro.x;
	gyro.y=((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - offset_gyro.y;
	gyro.z=((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - offset_gyro.z;
	
	if(!GYRO_OFFSET_OK)//陀螺仪 零偏计算
	{
		static int32_t tempgx=0,tempgy=0,tempgz=0;
		static uint8_t cnt_g=0;
		if(cnt_g==0)//刚进入 寄存器清零
		{
			offset_gyro.x=0;
			offset_gyro.y=0;
			offset_gyro.z=0;
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_g = 1;
			return;
		}
		//6050数据累加
		tempgx+= gyro.x;
		tempgy+= gyro.y;
		tempgz+= gyro.z;
		if(cnt_g==200)//加两百次 求平均
		{
			offset_gyro.x=tempgx/cnt_g;
			offset_gyro.y=tempgy/cnt_g;
			offset_gyro.z=tempgz/cnt_g;
			cnt_g = 0;
			GYRO_OFFSET_OK = 1;//计算完成标志
			return;
		}
		cnt_g++;
	}
	if(!ACC_OFFSET_OK)//加速度传感器数据 零偏数据计算
	{
		static int32_t	tempax=0,tempay=0,tempaz=0;
		static uint8_t cnt_a=0;
		if(cnt_a==0)
		{
			offset_acc.x = 0;
			offset_acc.y = 0;
			offset_acc.z = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
			return;
		}
		tempax+= acc.x;
		tempay+= acc.y;
		//tempaz+= MPU6050_ACC_LAST.Z;
		if(cnt_a==200)
		{
			offset_acc.x=tempax/cnt_a;
			offset_acc.y=tempay/cnt_a;
			offset_acc.z=tempaz/cnt_a;
			cnt_a = 0;
			ACC_OFFSET_OK = 1;
			return;
		}
		cnt_a++;		
	}
}

void MPU6050_Read(void)
{
	IIC_Read_nByte(devAddr,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
}


/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
失败为0
*******************************************************************************/ 
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
	u8 b;
	IIC_Read_nByte(dev, reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	IIC_Write_1Byte(dev, reg, b);
}
/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
reg	   寄存器地址
bitStart  目标字节的起始位
length   位长度
data    存放改变目标字节位的值
返回   成功 为1 
失败为0
*******************************************************************************/ 
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{
	
	u8 b,mask;
	IIC_Read_nByte(dev, reg, 1, &b);
	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	b &= mask;
	b |= data;
	IIC_Write_1Byte(dev, reg, b);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
	IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
	
}
/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
	IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
	IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
enabled =1   睡觉
enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_setDLPF(uint8_t mode)
{
	IICwriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_Init(void)
{
	IIC_Init();
	
	MPU6050_setSleepEnabled(0); //进入工作状态
  _led_delay(10);
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟  0x6b   0x01
	_led_delay(10);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
	_led_delay(10);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_16);	//加速度度最大量程 +-16G
  _led_delay(10);
	MPU6050_setDLPF(MPU6050_DLPF_BW_42);
//	MPU6050_setDLPF(MPU6050_DLPF_BW_256);
	_led_delay(10);
//	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
//	_led_delay(10);
//	MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
//	_led_delay(10);
}


/******************************************************************************
函数原型：	void Do_ACC_Offset(void)
功    能：	MPU6050加速度零偏校正
*******************************************************************************/ 
void Do_ACC_Offset(void)
{
	ACC_OFFSET_OK = 0;
}

/******************************************************************************
函数原型：	void Do_GYRO_Offset(void)
功    能：	MPU6050角速度零偏校正
*******************************************************************************/ 
void Do_GYRO_Offset(void)
{
	GYRO_OFFSET_OK = 0;
}



