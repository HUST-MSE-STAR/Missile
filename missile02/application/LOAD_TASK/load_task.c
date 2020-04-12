#include "load_task.h"
#include "main.h"

uint32_t give_current0=0;
uint32_t angel;	
uint32_t angle_contorl=0;
int16_t give_current1=20; //����Ϊ����������ĵ���������ʵ���޸�

chassis_move_t chassis_move[2];

uint32_t chassis_feedback_update(chassis_move_t *chassis_move_update)//�Ƕȸ���
{
return chassis_move_update->chassis_motor_measure->ecd;
}

void init(chassis_move_t *chassis_move_init,uint8_t i)
{
chassis_move_init->chassis_motor_measure = get_chassis_motor_measure_point(i);//��ֱַ�Ӵ���ò�ƻᱨ��ֻ���ú���
}

void Load()
{
	give_current0=10;//����Ϊ�����ŵ���ĵ���������ʵ���޸�
	while(give_current0){//main���п����жϺ���
		CAN_cmd_chassis(give_current0, 0, 0, 0);
	}
	Servo_Control(70);//�����ת35�� mainz���ѳ�ʼ����35��
	
	angle_contorl+=5376;//ת21/32Ȧ  ��3508���ؽǶȷ�ΧֵΪ0-8191��
	if(angle_contorl>8191)
		angle_contorl-=8191;
	while((angel<angle_contorl+100)&&(angel>angle_contorl-100)){	//angel��angle_contorl+-100�ڼ�ͣת��������һ������һֱת��100�ɸ�
		angel=chassis_feedback_update(&chassis_move[0]);
		CAN_cmd_chassis (0,give_current1,0,0);
	}
}

void Fire()
{
	Servo_Control(0);//�����ת35��
	void LOAD();
}




