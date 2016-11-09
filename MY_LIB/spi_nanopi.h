#ifndef __SPINANOPI_H
#define __SPINANOPI_H
#include "sys.h"
extern u8 spi_buff[5];
extern u8 spi_cnt;
 	    													  
void SPI1_Init(void);			 //初始化SPI1口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI1速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI1总线读写一个字节
		 
#endif

