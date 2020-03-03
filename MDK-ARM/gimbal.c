#include "gimbal.h"
#include "stdint.h"
#include "system.h"
#define Mech_Mid_Yaw       4093 //记得到时候测量

extern int Cloud_Angle_Measure[2][2];

int16_t GIMBAL_GetOffsetAngle(void)
{
	int16_t sAngleError = 0;

	sAngleError = (Cloud_Angle_Measure[YAW][MECH] - Mech_Mid_Yaw)*YAW_POSITION; //YAW_POSITION记得到时候测量


	//过零处理,统一成劣弧
	if (sAngleError > 8192 / 2)
	{
		return (sAngleError - 8192) ;
	}
	else if (sAngleError < -8192 / 2)
	{
		return (sAngleError + 8192);
	}
	else
	{
		return  sAngleError;
	}
}
