#ifndef __SPINANOPI_H
#define __SPINANOPI_H
#include "sys.h"
extern u8 spi_buff[5];
extern u8 spi_cnt;
 	    													  
void SPI1_Init(void);			 //��ʼ��SPI1��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI1���߶�дһ���ֽ�
		 
#endif

