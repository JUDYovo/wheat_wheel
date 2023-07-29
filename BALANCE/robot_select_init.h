#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//�����˲����ṹ��
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, Mec_Car is half wheelspacing //�־� ���ֳ�Ϊ���־�
  float AxleSpacing;       //Axlespacing, Mec_Car is half axlespacing //��� ���ֳ�Ϊ�����	
  int GearRatio;           //Motor_gear_ratio //������ٱ�
  int EncoderAccuracy;     //Number_of_encoder_lines //����������(����������)
  float WheelDiameter;     //Diameter of driving wheel //������ֱ��	
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional trolley //ȫ����С����ת�뾶	
}Robot_Parament_InitTypeDef;

// Encoder structure
//�������ṹ��
typedef struct  
{
  int A;      
  int B; 
	int C; 
	int D; 
}Encoder;

//The minimum turning radius of Ackermann models is determined by the mechanical structure: 
//the maximum Angle of the wheelbase, wheelbase and front wheels
//���������͵���Сת��뾶���ɻ�е�ṹ�������־ࡢ��ࡢǰ�����ת��
#define MINI_AKM_MIN_TURN_RADIUS 0.350f 

//Wheelspacing, Mec_Car is half wheelspacing
//�־� ������һ��
//#define MEC_wheelspacing         0.109
#define MEC_wheelspacing         0.0930 //����2021.03.30


//Axlespacing, Mec_Car is half axlespacing
//��� ������һ��
#define MEC_axlespacing           0.085


//Motor_gear_ratio
//������ٱ�
#define   HALL_30F    30


//Number_of_encoder_lines
//����������
#define		Photoelectric_500 500
#define	  Hall_13           13

//Mecanum wheel tire diameter series
//������ֱ̥��
#define		Mecanum_60  0.060f
#define		Mecanum_75  0.075f
#define		Mecanum_100 0.100f
#define		Mecanum_127 0.127f
#define		Mecanum_152 0.152f
 


//The encoder octave depends on the encoder initialization Settings
//��������Ƶ����ȡ���ڱ�������ʼ������
#define   EncoderMultiples  4
//Encoder data reading frequency
//���������ݶ�ȡƵ��
#define   CONTROL_FREQUENCY 100

//#define PI 3.1415f  //PI //Բ����

void Robot_Select(void);
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter);

#endif