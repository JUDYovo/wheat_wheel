#include "dma.h"
#include "sys.h"

DMA_InitTypeDef DMA_InitStructure;
u16 DMA1_MEM_LEN2;//����DMAÿ�����ݴ��͵ĳ���

///**************************************************************************
//�������ܣ�����һ��DMA����
//��ڲ�����DMAͨ�������ݵ�Ŀ���ַ�����ݵ���ʼ��ַ�����͵���������
//����  ֵ����
//**************************************************************************/
//void MYDMA_Enable2(DMA_Channel_TypeDef*DMA_CHx)
//{ 
//	DMA_Cmd(DMA_CHx, DISABLE );  //�ر�USART1 TX DMA1 ��ָʾ��ͨ��      
// 	DMA_SetCurrDataCounter(DMA1_Channel7,DMA1_MEM_LEN2);//DMAͨ����DMA����Ĵ�С
// 	DMA_Cmd(DMA_CHx, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 
//}	  

/////**************************************************************************
////�������ܣ�DMA��ʽ��printf
////��ڲ�����Ҫ��ӡ������
////����  ֵ����
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
//	DMA_Cmd(DMA1_Channel7, DISABLE); //ʧ��DMAͨ��
//	DMA_SetCurrDataCounter(DMA1_Channel7, length);
//	DMA_Cmd(DMA1_Channel7, ENABLE); //ʹ��DMAͨ��
//	while(1)
//	{
//		if(DMA_GetITStatus(DMA1_IT_TC7)!=RESET)	//�ж�ͨ��7��uart2_tx���������
//		{
//			DMA_ClearFlag(DMA1_IT_TC7);//���ͨ��7������ɱ�־
//			break;
//		}
//	}
//	return length;
//}
