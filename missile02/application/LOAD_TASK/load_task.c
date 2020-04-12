#include "load_task.h"
#include "main.h"

uint32_t give_current0=0;
uint32_t angel;	
uint32_t angle_contorl=0;
int16_t give_current1=20; //此项为给换弹电机的电流，根据实际修改

chassis_move_t chassis_move[2];

uint32_t chassis_feedback_update(chassis_move_t *chassis_move_update)//角度更新
{
return chassis_move_update->chassis_motor_measure->ecd;
}

void init(chassis_move_t *chassis_move_init,uint8_t i)
{
chassis_move_init->chassis_motor_measure = get_chassis_motor_measure_point(i);//地址直接传递貌似会报错，只能用函数
}

void Load()
{
	give_current0=10;//此项为给上膛电机的电流，根据实际修改
	while(give_current0){//main中有开关中断函数
		CAN_cmd_chassis(give_current0, 0, 0, 0);
	}
	Servo_Control(70);//舵机正转35度 mainz中已初始化到35度
	
	angle_contorl+=5376;//转21/32圈  （3508传回角度范围值为0-8191）
	if(angle_contorl>8191)
		angle_contorl-=8191;
	while((angel<angle_contorl+100)&&(angel>angle_contorl-100)){	//angel在angle_contorl+-100内既停转，否则往一个方向一直转，100可改
		angel=chassis_feedback_update(&chassis_move[0]);
		CAN_cmd_chassis (0,give_current1,0,0);
	}
}

void Fire()
{
	Servo_Control(0);//舵机反转35度
	void LOAD();
}




