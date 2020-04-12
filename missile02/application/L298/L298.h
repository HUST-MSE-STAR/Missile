#ifndef __L298_H
#define __L298_H	 
#include "main.h"
#include "struct_typedef.h"
#include "tim.h"
#include "imu_communication.h"

//通过L298控制推杆

#define LINEAR_MIN   ((short)3000)		//角度最小值，根据推杆最长最短确定 +-32768映射到+-180
#define LINEAR_MAX   ((short)20000)		//角度最大值
#define PITCH_SPEED  ((short)200)		//固定伸缩速度，需修改  

#define res_a   ((short)40)		//角度预留值 约0.2度  下面三个都需要根据实际调整
#define res_v   ((short)2)		//速度预留值 
#define div   ((short)15)		//电机速度控制 p

#define PITCH_1   ((short)8000)		//前哨站方向，需修改
#define PITCH_2   ((short)15000)		//基地方向
#define PITCH_FIX   ((short)100)		//右摇杆 伸缩固定值，需修改  此时约0.5度


extern imu_info imu_data;

extern void imu_init(void);

void Linear_Speed(short speed);//通过PWM设置推杆方向与速度 -99~99  0时为保持

void Linear_Init(void);//电动推杆初始化
void Linear_Angel(short angel);//设置角度 angel参数范围：+-32768
		 				    
#endif
