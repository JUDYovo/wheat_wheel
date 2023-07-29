/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c2;

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
    IIC_SCL=1;
	IIC_SDA=1;
  /* USER CODE END I2C2_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**************************************************************************
Function: Simulate IIC start signal
Input   : none
Output  : none
函数功能：模拟IIC起始信号
入口参数：无
返回  值：无
**************************************************************************/
void I2C_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;
	if(!READ_SDA)return ;	
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return ;
	delay_us(1);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
	return ;
}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
函数功能：模拟IIC结束信号
入口参数：无
返回  值：无
**************************************************************************/
void I2C_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(1);	
}



bool I2C_WaiteForAck(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			I2C_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//时钟输出0 	   
	return 1;
}

/**************************************************************************
Function: IIC response
Input   : none
Output  : none
函数功能：IIC应答
入口参数：无
返回  值：无
**************************************************************************/
void I2C_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}

/**************************************************************************
Function: IIC don't reply
Input   : none
Output  : none
函数功能：IIC不应答
入口参数：无
返回  值：无
**************************************************************************/ 
void I2C_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}



bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
    uint8_t Dat;
    
    Dat =I2C_ReadOneByte(DevAddr, RegAddr);
    Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}




bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{

    uint8_t Dat, Mask;
    
	Dat = I2C_ReadOneByte(DevAddr, RegAddr);
    Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
    Data <<= (8 - Length);
    Data >>= (7 - BitStart);
    Dat &= Mask;
    Dat |= Data;
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}

/**************************************************************************
Function: IIC sends a bit
Input   : none
Output  : none
函数功能：IIC发送一个位
入口参数：无
返回  值：无
**************************************************************************/
void I2C_WriteByte(uint8_t Data)
{
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(Data&0x80)>>7;
        Data<<=1; 	  
		delay_us(1);   
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
}


u8 I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_WriteByte(Data);
	I2C_WaiteForAck();
	I2C_Stop();
	return 1;
}


bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	
	for(i = 0; i < Num; i ++)
	{
		I2C_WriteByte(*(pBuff + i));
		I2C_WaiteForAck();
	}
	I2C_Stop();

	return true;
}

/**************************************************************************
Function: IIC reads a bit
Input   : none
Output  : none
函数功能：IIC读取一个位
入口参数：无
返回  值：无
**************************************************************************/
uint8_t I2C_ReadByte(uint8_t Ack)
{
	uint8_t i, RecDat = 0;

	SDA_IN();
	for(i = 0; i < 8; i ++)
	{
	//	I2C_SCL_Clr();
		IIC_SCL=0;
		delay_us(1);
//		I2C_SCL_Set();
				IIC_SCL=1;
		RecDat <<= 1;
		if(READ_SDA)
			RecDat |= 0x01;
		else
			RecDat &= ~0x01;
		delay_us(1);
	}
	if(I2C_ACK == Ack)
		I2C_Ack();
	else
		I2C_NAck();

	return RecDat;
}




uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t TempVal = 0;
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();
	TempVal = I2C_ReadByte(I2C_NACK);
	I2C_Stop();
	
	return TempVal;
}

bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();

	for(i = 0; i < Num; i ++)
	{
		if((Num - 1) == i)
		{
			*(pBuff + i) = I2C_ReadByte(I2C_NACK);
		}
		else
		{
			*(pBuff + i) = I2C_ReadByte(I2C_ACK);
		}
	}

	I2C_Stop();
	
	return true;
}


///**************************************************************************
//Function: IIC continuous reading data
//Input   : dev：Target device IIC address；reg:Register address；
//					length：Number of bytes；*data:The pointer where the read data will be stored
//Output  : count：Number of bytes read out-1
//函数功能：IIC连续读数据
//入口参数：dev：目标设备IIC地址；reg:寄存器地址；length：字节数；
//					*data:读出的数据将要存放的指针
//返回  值：count：读出来的字节数量-1
//**************************************************************************/ 
//u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
//    u8 count = 0;
//	
//	IIC_Start();
//	IIC_Send_Byte(dev);	   //发送写命令
//	IIC_Wait_Ack();
//	IIC_Send_Byte(reg);   //发送地址
//  IIC_Wait_Ack();	  
//	IIC_Start();
//	IIC_Send_Byte(dev+1);  //进入接收模式	
//	IIC_Wait_Ack();
//	
//    for(count=0;count<length;count++){
//		 
//		 if(count!=length-1)   data[count]=IIC_Read_Byte(1);  //带ACK的读数据
//		 else                  data[count]=IIC_Read_Byte(0);  //最后一个字节NACK
//	}
//    IIC_Stop();//产生一个停止条件
//    return count;
//}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
