#include "led.h"

int Led_Count=500; //LED flicker time control //LED��˸ʱ�����
/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
�������ܣ�LED�ӿڳ�ʼ��
��ڲ������� 
����  ֵ����
**************************************************************************/

/**************************************************************************
Function: The LED flashing
Input   : none
Output  : blink time
�������ܣ�LED��˸
��ڲ�������˸ʱ��
�� �� ֵ����
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) LED=1;
	  else		if(++temp==time)	LED=~LED,temp=0;
}

void Buzzer_Alarm(u16 count)
{
	int count_time;
	if(0 == count)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);//�õ͵�ƽ������������
	}
	else if(++count_time >= count)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);//�øߵ�ƽ����������
		count_time = 0;	
	}
}

