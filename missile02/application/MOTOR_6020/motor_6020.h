#ifndef __MOTOR_6020_H	
#define __MOTOR_6020_H	 
#include "main.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "load_task.h"
//控制步6020电机位置

#define give_current3  ((uint16_t)20)	  //默认转速

#define INIT_6020  ((uint16_t)4095)		//初始化位置
#define FIX_ANGLE  ((uint16_t)15)		//固定方向旋转速度，需修改
#define FIX_SPEED  ((uint16_t)200)		//固定方向旋转速度，需修改

#define ROLL_1   ((uint16_t)2000)		//前哨站方向，需修改
#define ROLL_2   ((uint16_t)5000)		//基地方向
#define ROLL_FIX   ((uint16_t)50)		//右摇杆旋转固定值，需修改   摇杆左右需在main中修改！！

extern chassis_move_t chassis_move[2];

void MOTOR_6020_Init(void);//初始化
void MOTOR_6020_POS (uint16_t position);//电机旋转到固定位置   参数：旋转位置0-8191
void MOTOR_6020_ANGLE (uint8_t RL);//电机旋转固定角度 FIX_ANGLE (约0.6度)  参数：旋转方向  RL是0是1需根据实际修改
void MOTOR_6020_FIX (uint8_t RL);//电机按某方向运行，速度固定为FIX_SPEED   参数：旋转方向



#endif
