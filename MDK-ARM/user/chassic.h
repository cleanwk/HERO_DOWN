#ifndef __CHASSIC_H
#define __CHASSIC_H

//底盘模式选择
typedef enum
{
	MECH = 0,//机械
	GYRO = 1,//陀螺仪,底盘跟随云台
} eChassisCtrlMode;

typedef enum
{
	CHASSIS_NORMAL   = 0,//普通模式,正常前进
	CHASSIS_CORGI    = 1,//扭屁股模式
	CHASSIS_ROSHAN   = 2,//打符模式
	CHASSIS_SLOW     = 3,//补弹低速模式
	CHASSIS_SZUPUP   = 4,//爬坡模式
	CHASSIS_MISS     = 5,//自动闪避模式
	CHASSIS_PISA     = 6,//45°模式
	
} eChassisAction;
typedef enum
{
	LEFT_FRON_201 = 0,  // 左前
	RIGH_FRON_202 = 1,  // 右前
	LEFT_BACK_203 = 2,  // 左后
	RIGH_BACK_204 = 3,  // 右后
	
}eChassisWheel;
void Task_Chassis(void);
void test_horizontal(void);
void test_vertical(void);
void test_rotate(void);
//void test_chassis_speed(void);//(之前写的运动逻辑，现在先不用）
void CHASSIS_REST(void);
void CHAS_Key_Ctrl(void);
void Chassis_Omni_Move_Calculate(void);
void Chassis_Motor_Speed_PID( eChassisWheel eWheel );
void Chassis_MotorOutput(void);
void Chassis_Power_Limit(void);
void Chassis_NORMAL_Mode_Ctrl(void);
void Chassis_Keyboard_Move_Calculate(int16_t sMoveMax, int16_t sMoveRamp);
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp);
void CHASSIS_CORGI_Mode_Ctrl_Time(int16_t sRevolMax, int16_t sRevolRamp);
void CHAS_Rc_Ctrl(void);
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec );
extern float traget_speed[4];
void CHASSIS_InitArgument(void);
int16_t GIMBAL_GetOffsetAngle(void);
float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp);
void CHASSIS_SZUPUP_Mode_Ctrl(void);
#endif

