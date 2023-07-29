#include "dma.h"
#include "sys.h"

DMA_InitTypeDef DMA_InitStructure;
u16 DMA1_MEM_LEN2;//保存DMA每次数据传送的长度

///**************************************************************************
//函数功能：开启一次DMA传输
//入口参数：DMA通道，数据的目标地址，数据的起始地址，发送的数据数量
//返回  值：无
//**************************************************************************/
//void MYDMA_Enable2(DMA_Channel_TypeDef*DMA_CHx)
//{ 
//	DMA_Cmd(DMA_CHx, DISABLE );  //关闭USART1 TX DMA1 所指示的通道      
// 	DMA_SetCurrDataCounter(DMA1_Channel7,DMA1_MEM_LEN2);//DMA通道的DMA缓存的大小
// 	DMA_Cmd(DMA_CHx, ENABLE);  //使能USART1 TX DMA1 所指示的通道 
//}	  

/////**************************************************************************
////函数功能：DMA方式的printf
////入口参数：要打印的内容
////返回  值：无
////**************************************************************************/
//void DMA_printf(const char *format,...)
//{
//	u32 length;
//	va_list args;
//	
//	va_start(args, format);
//	length = vsnprintf((char*)t2xbuf, sizeof(t2xbuf), (char*)format, args);
//    va_end(args);
//	USART_SendBuffer((const char*)t2xbuf,length);
//}

//u32 USART_SendBuffer(const char* buffer, u32 length)
//{
//	if( (buffer==NULL) || (length==0) )
//	{
//		return 0;
//	}
// 
//	DMA_Cmd(DMA1_Channel7, DISABLE); //失能DMA通道
//	DMA_SetCurrDataCounter(DMA1_Channel7, length);
//	DMA_Cmd(DMA1_Channel7, ENABLE); //使能DMA通道
//	while(1)
//	{
//		if(DMA_GetITStatus(DMA1_IT_TC7)!=RESET)	//判断通道7（uart2_tx）传输完成
//		{
//			DMA_ClearFlag(DMA1_IT_TC7);//清除通道7传输完成标志
//			break;
//		}
//	}
//	return length;
//}
