#include "L298.h"



void Linear_Speed(short speed)
{
	if(speed>=0){
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, speed);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	}
	else {
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, speed);
	}
	
}

void Linear_Init(void)
{
	Linear_Angel(LINEAR_MIN);
}


void Linear_Angel(short angel)
{
	short speed=0;
	receive_imu_data();
	while(angel>imu_data.stcAngle.Angle[1]+res_a){
		speed=(angel-imu_data.stcAngle.Angle[1])/div+res_v;
		if(speed>99){
			speed=99;
		}
		Linear_Speed(speed);   //根据实际方向需改speed正负
		receive_imu_data();
	}
	while(angel<imu_data.stcAngle.Angle[1]-res_a){
		speed=(imu_data.stcAngle.Angle[1]-angel)/div+res_v;
		if(speed>99){
			speed=99;
		}
		Linear_Speed(-speed);   //根据实际方向需改speed正负
		receive_imu_data();
	}
	
}
