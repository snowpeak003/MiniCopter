#ifndef __IOI2C_H
#define __IOI2C_H

#include "stm32f4xx.h"
#include "sys.h"

//IO操作函数	 
#define IIC_SCL    PBout(14)         // SCL//TIM12CH1
#define IIC_SDA    PBout(15)         // SDA	//TIM12CH2
#define READ_SDA   PBin(15)          // 输入SDA 

//IIC所有操作函数
void IIC_Init(void);                // 初始化IIC的IO口				 
void IIC_Start(void);				        // 发送IIC开始信号
void IIC_Stop(void);	  			      // 发送IIC停止信号
void IIC_Send_Byte(u8 txd);			    // IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);// IIC读取一个字节
u8 IIC_Wait_Ack(void); 				      // IIC等待ACK信号
void IIC_Ack(void);					        // IIC发送ACK信号
void IIC_NAck(void);				        // IIC不发送ACK信号

u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#endif

//------------------End of File----------------------------
