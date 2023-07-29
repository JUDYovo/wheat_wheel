
#ifndef __MOTOR_H
#define __MOTOR_H

#include "system.h"

/*--------Motor_A control pins--------*/



void Enable_Pin(void);
void Servo_PWM_Init(u16 arr,u16 psc);
void TIM1_PWM_Init(u16 arr,u16 psc);
void TIM9_PWM_Init(u16 arr,u16 psc);
void TIM10_PWM_Init(u16 arr,u16 psc);
void TIM11_PWM_Init(u16 arr,u16 psc);			
	


#endif
