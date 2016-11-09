#include "spi_nanopi.h"
//////////////////////////////SPI_Slave//////////////////////////////////////////////
//以下是SPI模块的初始化代码，配置成从机模式 						  
//SPI口初始化
//这里针是对SPI1的初始化
void SPI1_Init(void)
{	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1时钟
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//使能SPI3时钟
 
	//APB1 =42M ; APB2 =84M ;
	/*SPI3_SCK PC10  SPI3_MISO PC11  SPI3_MOSI PC12*/
	/*对应板子上的标号（部分被TIM3占用）
	  //SPI1_SCK PA5 PA5 
	  SPI1_SCK PB3  SWO
	  SPI1_MISO PA6 TIM13_CH1 
	  SPI1_MOSI PA7 TIM14_CH1
	  //SPI1_NSS PA4 SBUS_IN
	  SPI1_NSS PA15 TIM2_CH1
	*/	
  //GPIO A6 MISO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_15;//PA6,7,15复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	 //GPIO B3 SCK
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PB3复用	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	//GPIO A7 A15OSI NSS
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_15;//PA7,15复用功能
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//开漏
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); //PA6复用为 SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //PA7复用为 SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_SPI1); //PA15复用为 SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1); //PB3复用为 SPI1
 
	//这里只针对SPI口初始化
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;		//设置SPI工作模式:设置为从SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第2个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
    
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);//定义接收中断
	
 	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//响应2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
 	NVIC_Init(&NVIC_InitStructure);
	
    /* Enable SPI_SLAVE */	
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
	SPI1_ReadWriteByte(0xff);//启动传输		 
}   
//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2时钟一般为84Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
	SPI1->CR1&=0XFFC7;//位3-5清零，用来设置波特率
	SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度 
	SPI_Cmd(SPI1,ENABLE); //使能SPI1
} 
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{		 			 
 
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
	
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  
 
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据	
 		    
}
u8 spi_buff[5];
u8 spi_cnt=0;
u8 spi_sendBuff[5]={0x08,0x04,0x02,0x06,0x81};
void SPI1_IRQHandler(void)
{         
	//u8 buff;
	if(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==SET)
	{   		
		spi_buff[spi_cnt]=SPI_I2S_ReceiveData(SPI1);
		//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) ;
		SPI_I2S_SendData(SPI1,spi_sendBuff[spi_cnt]);
		
		spi_cnt++;
		if(spi_cnt==5) 
			spi_cnt=0;
		
		
		SPI_I2S_ClearFlag(SPI1,SPI_I2S_IT_RXNE);
	}
}






