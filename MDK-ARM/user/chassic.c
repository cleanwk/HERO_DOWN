#include "bsp_uart.h"
#include "control.h"
#include "chassic.h"
#include "tim.h"
#include "math.h"
#include "system.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "judgement.h"
#include <math.h>
#include "judgement.h"
#include "kalman.h"

#define ABS(x)		((x>0)? (x): (-x)) 

//��ͬģʽ�µ�����ٶ�
#define    LIMIT_CHASSIS_MAX         4000     //������������µ��̵������������
#define    CHAS_CURRENT_LIMIT        36000//40000    //�ĸ����ӵ��ٶ��ܺ����ֵ,�������*4,�޹��ʵ���������
#define    REVOLVE_MAX_CORGI         9000//5000     //����Ťƨ������ٶ�,̫����ýǶȹ���

//��ͬģʽ��,б�º�����Ӧ��ʱ��ֵ,һ������ͨ�ٶȾ���
#define    TIME_INC_NORMAL           2//10//6	  //����б��,Խ�������ٶ�Խ��,���ʱ��Խ��
#define    TIME_INC_SALTATION        1        //ͻȻ����������ٶȱ仯����
#define    TIME_DEC_NORMAL           3//180        //����б��,Խ���С�ٶ�Խ��(һ��Ҫ��INC��һ��,�����ɿ������ܸ���Ϊ0,̫�������ɵ���ͣ������ʱ����Ծ)

#define    REVOLVE_SLOPE_NORMAL      80       //������ͨģʽб��,Խ��Խ��,���ʱ��Խ��
#define    REVOLVE_SLOPE_CORGI       150      //����Ťƨ��ģʽб��,Խ��Խ��,���ʱ��Խ��


//���̵����޷�
#define iTermChassis_Max             3000     //΢���޷�
#define KP 0
#define KI 1
#define KD 2

#define    FALSE    0
#define    TRUE     1
#define   WHEELBASE  300     //ǰ�����
#define   WHEELTRACK 350//398     //�����־�
#define   GIMBAL_X_OFFSET 0//75  //��̨��Ե������ĵ�ǰ��ƫ����
#define   GIMBAL_Y_OFFSET 0 //��̨��Ե������ĵ�����ƫ��?
#define RADIAN_COEF 57.3f  
#define PERIMETER  478     //�����ܳ�
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)  //���������
#define 	Omni_Speed_Max          8000     //����ˮƽ�ƶ��ٶ��޷�,��ֹ����ģʽ���ٶȳ������ֵ
#define		STANDARD_MAX_NORMAL       8000//ƽ�ؿ�������ٶȣ���ֹҡ�˱���*660�������ֵ
#define		REVOLVE_MAX_NORMAL        5000     //ƽ��Ťͷ����ٶ�(��ת��
#define  REVOLVE_KD                (350.f)
#define     REVOLVE_ANGLE             35
#define Omni_SupCap_Max				 10000
#define    STANDARD_MAX_SZUPUP       3000//5000//6000//4000//3600	  //�ֶ�����ģʽ��ˮƽ�ƶ��ٶ�


/**************�޷�**************/
float Chassis_Standard_Move_Max;//����ǰ������ƽ������
float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX;//����PID�����������ֵ,���̹�������,���ݵ��ת��ʵʱ�仯
float Chassis_Revolve_Move_Max=5000;//����������ת����,���ݲ�ͬ�˶�ģʽʵʱ����,���Բ����ú궨��
float Chassis_Limit_Output_Max=LIMIT_CHASSIS_MAX;//���̹����޷�
extKalman_t Chassis_Error_Kalman;//����һ��kalmanָ��

//Chassis_Revolve_Move_Max=5000;
/**************б��***************/
float Slope_Chassis_Move_Z;//б�¼�������ƶ�����,����Ŀ���������ʵʱб��ֵ
//��еģʽ�µ��̱���ϵ��,���Ƽ���б�±仯��
float kKey_Mech_Chassis_Standard;//ƽ�ƣ���ת
//������ģʽ�µ��̱���ϵ��,����ҡ����Ӧ�ٶ�,�����СҲ���������ת��,max = ��ϵ�� *660
float kRc_Gyro_Chassis_Standard,kRc_Gyro_Chassis_Revolve;//ƽ�ƣ���ת
uint16_t timeInc;//б�����ӱ仯ʱ��
uint16_t timeInc_Saltation;//ǰ����ͻ���µ�б��������,�����������ҪС
int16_t   timeXFron,    timeXBack,    timeYLeft,    timeYRigh, timeZRrotate,timeZLrotate;//����  s  w  d   a

//����ģʽ��ȫ���ƶ�����,б����
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;
float Slope_Chassis_Move_Lrotate, Slope_Chassis_Move_Rrotate;
//����ģʽ��Ťͷб��,��Ҫ����Ťƨ��ģʽ��
float Slope_Chassis_Revolve_Move;
float  traget_speed[4]={0};
eChassisAction  actChassis;//ģʽ�л�����
eChassisCtrlMode modeChassis;

/**************ȫ���ƶ�����****************/
float Chassis_Move_X;//ǰ��
float Chassis_Move_Y;//����ƽ��
float Chassis_Move_Z;//������ת

/***********PID����***************/
//���������ٶ�
float Chassis_Speed_Target[4];//ID
//�����ٶ����
float Chassis_Speed_Error[4];//ID
//���̲����ٶ�
int16_t Chassis_Speed_Measure[4];
//�����ٶ�����
float Chassis_Speed_Error_Sum[4];//ID
float Chassis_Speed_Error_NOW[4], Chassis_Speed_Error_LAST[4];
//������ģʽ�µ���ƫ��(���YAW�����ķ����е�Ƕ�)
int16_t Chassis_Gyro_Error;
//�����Ƕ�
float Cloud_Angle_Measure[2][2];//  pitch/yaw    mech/gyro


//����PID����
float Chassis_Speed_kpid[4][3];//	motorID kp/ki/kd

float   pTermChassis[4], iTermChassis[4], dTermChassis[4];//ID
float	pidTermChassis[4];//ID,���������

//���̵�������
float Chassis_Final_Output[4];



//����Ťͷ
float Chassis_Z_kpid[3] = {-11, 0, 5};
float pTermChassisZ;
float dTermChassisZ;
//PID ����
//float yys[4];

 /*****************���̸���ģʽ�ĸ�������***********/

//Ťƨ��
bool Chass_Switch_F = 1;
u8 	 Chass_Key_F_Change = 0;

/**************����ϵͳ������������*****************/
//#if JUDGE_VERSION == JUDGE_18
//	float WARNING_REMAIN_POWER = 50;//����ϵͳʣ�ཹ���������������ֵ��ʼ�޹���,40Ťƨ�ɻᳬ����,ƽ�ؿ����ᳬ
//#elif JUDGE_VERSION == JUDGE_19
float WARNING_REMAIN_POWER = 60;


float fChasCurrentLimit = CHAS_CURRENT_LIMIT;//����4�����ӵ��ٶ��ܺ�
float fTotalCurrentLimit;//��������,ƽ��ģʽ�·����Ǿ��ȵ�
float constrain_float(float amt, float low, float high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

//�޷�
int constrain(int amt, int low, int high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}


void Task_Chassis(void)//ÿ��msִ��һ������
{
 //for(;;)
	//{
		//Blue_On;//��δ���庯��
	
	    //Chassis_Speed_Measure[0] = M3508[3].rotor_speed;
		if(0/*SYSTEM_GetSystemState() == SYSTEM_STARTING*/)//��ʼ��ģʽ
		{
			CHASSIS_REST();
			actChassis = CHASSIS_NORMAL;
		}
		else if (rc_t.kb.bit.F)
		{
		 CHASSIS_REST();
		 actChassis = CHASSIS_CORGI;
		}
		else
   {           
		actChassis = CHASSIS_NORMAL;
		if(IF_RC_SW2_UP) 
            CHAS_Key_Ctrl();//���̿��Ƶ����˶�
    else
			   		CHAS_Rc_Ctrl();
		}
	
		Chassis_Omni_Move_Calculate();//�ƶ�����
		
    Chassis_Speed_Measure[0]=M3508[0].rotor_speed;
		Chassis_Speed_Measure[1]=M3508[1].rotor_speed;
		Chassis_Speed_Measure[2]=M3508[2].rotor_speed;
		Chassis_Speed_Measure[3]=M3508[3].rotor_speed;
		
		Chassis_MotorOutput();//����PID����
	//	Chassis_Power_Limit();//��������,�������·���
        Chassis_set_voltage(Chassis_Final_Output[0],Chassis_Final_Output[1],Chassis_Final_Output[2],Chassis_Final_Output[3]);
		

		 
}
//* @brief  ���̲�����ʼ��
//  * @param  void
//  * @retval void
//  * @attention 
//  */
void CHASSIS_InitArgument(void)
{
	
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 1.6;
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 0.1;//0.08;
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_FRON_202][KP] =1.6;
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 0.1;//0.08;
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 0;
	
	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 1.6;
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 0.1;//0.08;
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 1.6;
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 0.1;//0.08;
	Chassis_Speed_kpid[RIGH_BACK_204][KD] = 0;
	
}
//int CHASSIS_CORGI_Mode_Ctrl_Time(REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI)
//{

//}
/**
  * @brief  ����ʧ��ֹͣ
  * @param  void
  * @retval void
  * @attention �����ֵΪ��,�ǵ�Ҫ���㷢�ͳ�ȥ
  */
void CHASSIS_StopMotor(void)
{
}
/**
  * @brief  ��������״̬
  * @param  void
  * @retval void
  * @attention ״ֵ̬0
  */
void CHASSIS_REST(void)
{	
	Slope_Chassis_Move_Z = 0;//Ťƨ��ʵʱ���б��
	Chassis_Move_X = 0;
	Chassis_Move_Y = 0;
	Chassis_Move_Z = 0;
	
}
// * @brief  ң�ؿ��Ƶ����ƶ�
//  * @param  void
//  * @retval void
//  * @attention �ڴ˼����ƶ�����
//  */
void CHAS_Rc_Ctrl(void)
{
    float k_rc_z = 1;//����Z�ٶȵ���ǰ������ƽ�����ٱ�
	
	if (IF_RC_SW2_DOWN)//S2��������
	{
		modeChassis = GYRO;
	}
	else
	{
		modeChassis = MECH;
	}
	
	//Ťͷб���л�����ͨģʽ(Ťƨ��ģʽб�¸�λ)
	//Slope_Chassis_Revolve_Move = REVOLVE_SLOPE_NORMAL;
	//�ƶ��ٶ�����
	Chassis_Standard_Move_Max = STANDARD_MAX_NORMAL;//ƽ���޷�
	Chassis_Revolve_Move_Max  = REVOLVE_MAX_NORMAL;//Ťͷ�޷�
	
	//��¼ҡ���ƶ�����
	if(modeChassis == MECH)//��еģʽ
	{	
		Chassis_Move_Z = constrain_float(10*RC_CH0_RLR_OFFSET, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);//��ת
		
		if(fabs(Chassis_Move_Z) > 800)//Ťͷ�ٶ�Խ��,ǰ���ٶ�Խ��,��ֹת��뾶����
		{
			k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
						/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
			
			k_rc_z = constrain_float(k_rc_z,0,1);
		}
		else
		{
			k_rc_z = 1;
		}
		
		Chassis_Move_X = constrain_float( 14*RC_CH3_LUD_OFFSET, -Chassis_Standard_Move_Max*k_rc_z, Chassis_Standard_Move_Max*k_rc_z);//ǰ��
		
		Chassis_Move_Y = constrain_float( 14*RC_CH2_LLR_OFFSET, -Chassis_Standard_Move_Max*k_rc_z, Chassis_Standard_Move_Max*k_rc_z);//ˮƽ
	
	}
	else if(modeChassis == GYRO)//������ģʽ
	{
		Chassis_Gyro_Error = GIMBAL_GetOffsetAngle( );//�������YAW��ƫ��

		Chassis_Move_Z = Chassis_SpeedZ_PID(Chassis_Gyro_Error, kRc_Gyro_Chassis_Revolve);
	
		if(fabs(Chassis_Move_Z) > 800)//Ťͷ�ٶ�Խ��,ǰ���ٶ�Խ��,��ֹת��뾶����
		{
			k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
						/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
			
			k_rc_z = constrain_float(k_rc_z,0,1);
		}
		else
		{
			k_rc_z = 1;
		}
		Chassis_Move_X = constrain_float( kRc_Gyro_Chassis_Standard*RC_CH3_LUD_OFFSET, -Chassis_Standard_Move_Max * k_rc_z, Chassis_Standard_Move_Max * k_rc_z);//ǰ��
			
		Chassis_Move_Y = constrain_float( kRc_Gyro_Chassis_Standard*RC_CH2_LLR_OFFSET, -Chassis_Standard_Move_Max * k_rc_z, Chassis_Standard_Move_Max * k_rc_z);//ˮƽ

	}
}
/**
  * @brief  ���̿��Ƶ����ƶ�
  * @param  void
  * @retval void
  * @attention ģʽѡ��,����ĳģʽ��ǵ�д�˳�����ͨģʽ���ж�
  * �ް������»�һֱ�����Զ�����ģʽ,����ģʽ�л���İ�����������ģʽ�л�ѡ��ģʽ
  */
void CHAS_Key_Ctrl(void)
{
//	//����Ӧ�ü����еģʽ��������ģʽ�ж�
	switch (actChassis) 
	{
		/*------------------��ͨģʽ,����ģʽ�л��ж�-------------------*/	
		case CHASSIS_NORMAL:		
			Chassis_NORMAL_Mode_Ctrl();	
		break;
	//  Chassis_NORMAL_Mode_Ctrl();
		
		/*------------------Ťƨ��ģʽ--------------*/
		case CHASSIS_CORGI:	
			if(!rc_t.kb.bit.F)//F�ɿ�
			{
				Chass_Switch_F = 1;
			}
			
			if(rc_t.kb.bit.F && Chass_Switch_F == 1)
			{
				Chass_Switch_F = 0;
				Chass_Key_F_Change ++;
				Chass_Key_F_Change %= 2;
			}
	//����ǰ���ƶ�,��������ƽ�ơ�QE���л���еģʽ�˳�
			if(Chass_Key_F_Change)
			{
			 	modeChassis = GYRO;//������ģʽ,���̸�����̨��
   // 	Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
				CHASSIS_CORGI_Mode_Ctrl( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);
// 	  	CHASSIS_CORGI_Mode_Ctrl_Time( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);				
			}
			else
			{
				actChassis = CHASSIS_NORMAL;//�˳�Ťƨ��ģʽ
			}
		}
}
/*************************���̼���ģʽ����ģʽС����****************************/

/**
  * @brief  ����ģʽ�µ����˶�����
  * @param  �ٶ���������    �����ٶ�(���293)
  * @retval void
  * @attention  ���̿���ǰ������ƽ��,ƽ���޻�е��������ģʽ֮��
  *             ��Ҫ��ȡʱ��������б�º�������
  */
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp )
{

  float k_rc_z=1;//����z�ٶȵ���ǰ������ƽ�����ٱ�
  Chassis_Standard_Move_Max=sMoveMax;//�����ٶ��޷���ˮƽ�ƶ�
	timeInc=sMoveRamp;
//	if(fabs(Chassis_Move_Z)>800)//Ťͷ�ٶ�Խ�죬ǰ���ٶ�Խ������ֹת��뾶����
//	{
//		k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
//					/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
//		
//		k_rc_z = constrain_float(k_rc_z,0,1);
//	}
//    else 
	//{
//		k_rc_z=1;
//	}
//	delay(100);//��ʱʮ����
	if(actChassis == CHASSIS_NORMAL)//ֻ��һ��ģʽ�²��ж��ٶ�ͻ�����,��ֹ��,��ʱû������shift����
	{
		if(rc_t.kb.bit.W)//w������
		{
			timeXBack=0;//����ǰ���󣬺���б�¹��㣬�����´μ������б��
			if( Chassis_Move_X < sMoveMax/2.5 )//ת��ͻ��,�տ�ʼ��һС��ʱ��б�½���,��ֹ���Ӵ��˷ѹ���
				{//�����ٶ��Ƿ�����ٶȵ�1/5���жϲ�֪���Ƿ����
					timeInc_Saltation = TIME_INC_SALTATION;//������ģʽ�����ٶȷ���ͻ��
				}
				else			//�Ѿ����˴�ʱ������������һ�����ٶ�
				{
					timeInc_Saltation = sMoveRamp;
				}
			
		}
		if (rc_t.kb.bit.S)
			{
				timeXFron = 0;//ͬ��
				//����X�Ǹ�
				if( Chassis_Move_X > (-sMoveMax)/2.5 )//ת��ͻ��,�տ�ʼ��һС��ʱ��б�½���,��ֹ���Ӵ��˷ѹ���
				{
					timeInc_Saltation = TIME_INC_SALTATION;//������ģʽ�����ٶȷ���ͻ��
				}
				else			//�Ѿ����˴�ʱ������������һ�����ٶ�
				{
					timeInc_Saltation = sMoveRamp;
				}
			}
		if(rc_t.kb.bit.D)
			{
			   timeYLeft=0;
				
			}
		if(rc_t.kb.bit.A)
		{
		       timeYRigh=0;
		}
		if(rc_t.kb.bit.Q)
		{
		timeZRrotate=0;
		}
		if(rc_t.kb.bit.E)
		{
		timeZLrotate=0;
		}
		//����ģʽ��ȫ���ƶ�,б��������,ע������,��������*б�±����õ��������ӵ�ֵ,ģ��ҡ��
			//ǰ�������б���Ǳ仯��
			Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.W, &timeXFron, timeInc_Saltation, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.S, &timeXBack, timeInc_Saltation, TIME_DEC_NORMAL ) );

			//���ҵ�����б�¸�ǰ��һ��,����
			Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.A, &timeYLeft, timeInc/1.5, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.D, &timeYRigh,timeInc/1.5, TIME_DEC_NORMAL ));
		   Slope_Chassis_Move_Lrotate = (int16_t)( -REVOLVE_MAX_NORMAL * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.Q, &timeZLrotate, timeInc/1.5, TIME_DEC_NORMAL ) );
	 Slope_Chassis_Move_Rrotate = (int16_t)( REVOLVE_MAX_NORMAL * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.E, &timeZRrotate, timeInc/1.5, TIME_DEC_NORMAL ) );


			Chassis_Move_X  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//ǰ�����
			Chassis_Move_Y  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//���Ҽ���
			Chassis_Move_Z  = (Slope_Chassis_Move_Lrotate + Slope_Chassis_Move_Rrotate) * k_rc_z;//��ת����
		}
	else		//����ģʽ����Ҫ�����ٶȷ���ͻ�����⴦��
		{
			if(rc_t.kb.bit.W)
			{
				timeXBack=0;//����ǰ�������б�¹���,�����´μ������б��
			}
			if(rc_t.kb.bit.S)
			{
				timeXFron=0;//ͬ��
			}
				if(rc_t.kb.bit.A)
			{
				timeYRigh=0;//ͬ��
			}	if(rc_t.kb.bit.D)
			{
				timeYLeft=0;//ͬ��
			}
			Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.W, &timeXFron, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.S, &timeXBack, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.A, &timeYRigh, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.D, &timeYLeft, timeInc, TIME_DEC_NORMAL ) );
            Slope_Chassis_Move_Lrotate = (int16_t)( -REVOLVE_MAX_NORMAL * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.Q, &timeZLrotate, timeInc/1.5, TIME_DEC_NORMAL ) );
	 Slope_Chassis_Move_Rrotate = (int16_t)( REVOLVE_MAX_NORMAL * 
					Chassis_Key_MoveRamp( rc_t.kb.bit.E, &timeZRrotate, timeInc/1.5, TIME_DEC_NORMAL ) );
			if(actChassis != CHASSIS_CORGI)
			{
				Chassis_Move_X  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//ǰ�����
				Chassis_Move_Y  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//���Ҽ���
				Chassis_Move_Z  = (Slope_Chassis_Move_Lrotate + Slope_Chassis_Move_Rrotate) * k_rc_z;//��ת����
			}
		}
}


/**
  * @brief  �����Ƶ�����ת,����QEC���ƿ���תȦ
  * @param  �ٶ��������� 
  * @retval void
  * @attention  ������������ת
  */
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax )
{
//	Chassis_Move_Z = constrain_float( rc_t.kb.bit.F*kKey_Mech_Chassis_Revolve, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);
	
}
/**
  * @brief  ���̼���ģʽѡ��,������Ӧ
  * @param  void
  * @retval void
  * @attention ���̼��̿���״̬�µ�����ģʽ�л�������
  */
void Chassis_NORMAL_Mode_Ctrl(void)
{
            Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);//�����ٶ����ֵ��б��ʱ��	
	          Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);	//���ͨ���ı�chassic_move_z������������ת
}

/**
  * @brief  Ťƨ��ģʽ(λ�ò����)
  * @param  �ٶ���������    ���ӵ����������ʱ��
  * @retval void
  * @attention  ����ʱ�䣬Ť��λ�˾ͻ���
  */
//Ťƨ�ɻ���ѡ��
#define    CORGI_BEGIN    0    
#define    CORGI_LEFT     1
#define    CORGI_RIGH     2

uint16_t  stateCorgi = CORGI_BEGIN;//�������Ť,Ĭ�ϲ�Ť
bool    IfCorgiChange = FALSE;//�Ƿ�Ť������һ��
int16_t  corgi_angle_target = 0;//����Ŀ��Ƕ�
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp)
{
	int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	float    angle         = 0;


	Chassis_Revolve_Move_Max = sRevolMax;//����ٶ�����
	Slope_Chassis_Revolve_Move = sRevolRamp;//Ťͷб������

	//sAngleError = GIMBAL_GetOffsetAngle();//����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//����Ƕ�ƫ��,��е�Ƕ�ת����ŷ����,����ǰ���ٶȲ���
	angle = -(float)sAngleError / (float)8192 * 6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//�ݴ�ʵʱX�仯
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	Chassis_Move_X = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	Chassis_Move_Y = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	//�ؼ�:��������......
	switch (stateCorgi)
	{
		case CORGI_BEGIN:	//�Ժ���������ø����(��־λ��ͣȡ��),���ÿ�ʼŤͷ�ķ������	  
			corgi_angle_target = -900;//�ɸ�����ƶ��Ƕ�,�Զ�����ģʽ�±�����ʱ��Ť���Ƕ�
			IfCorgiChange = FALSE;
			stateCorgi    = CORGI_LEFT;
		break;
		
		case CORGI_LEFT:
			corgi_angle_target = -1024;//�ɸ�����ƶ��Ƕ�			
			IfCorgiChange = FALSE;

			if (sAngleError < -700)//�Ƕ�������700
			{
					stateCorgi = CORGI_RIGH;
				  IfCorgiChange = TRUE;//��ǿ��Ի���
			}			
		break;
			
		case CORGI_RIGH:		
			corgi_angle_target = 1024;
			IfCorgiChange = FALSE;

			if (sAngleError > 700)//�Ƕ�������700
			{
				stateCorgi = CORGI_LEFT;
				IfCorgiChange = TRUE;//��ǿ��Ի���
			}			
		break;
	}

 	//Chassis_Move_Z = Chassis_SpeedZ_PID( (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);//Ťƨ��ģʽ�������������
}

///**
//  * @brief  Ťƨ��ģʽ(ʱ�䲻���)
//  * @param  �ٶ���������    ���ӵ����������ʱ��
//  * @retval void
//  * @attention  ����ŤûŤ��λ��ʱ�䵽�˾ͻ���
//  */
//void CHASSIS_CORGI_Mode_Ctrl_Time(int16_t sRevolMax, int16_t sRevolRamp)
//{
//	static uint32_t twist_count;
//	static float  chass_yaw_set;
//	float    angle         = 0;
//	int16_t  sAngleError   = 0;
//	float    vectorXBuffer = 0;
//	float    vectorYBuffer = 0;
//	
//	
//	static int16_t twist_period = 800;//500*2msΪһ��Ť������
//	static int16_t twist_angle  = 40;//����������ŷ���Ƕ�����
//	
//	
//	twist_count++; 

//	Chassis_Revolve_Move_Max = sRevolMax;//����ٶ�����
//	Slope_Chassis_Revolve_Move = sRevolRamp;//Ťͷб������

//	//sAngleError = GIMBAL_GetOffsetAngle();//����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

//	//����Ƕ�ƫ��,��е�Ƕ�ת����ŷ����,����ǰ���ٶȲ���
//	angle = -(float)sAngleError / (float)8192 * 360;//6.283f;

//	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//�ݴ�ʵʱX�仯
//	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;
//	
//	//���Һ���Ťƨ��
//	Chassis_Move_X = vectorXBuffer * cos( angle/57.3f ) - vectorYBuffer * sin( angle/57.3f );
//	Chassis_Move_Y = vectorXBuffer * sin( angle/57.3f ) + vectorYBuffer * cos( angle/57.3f );

//	//chass_yaw_set = twist_angle * sin(2 * PI / twist_period * twist_count);//�������Ŀ�����Ƕ�
//	//Chassis_Move_Z = -Chassis_Z _Corgi(angle, chass_yaw_set);
//}



//**********************�����������**************************/

/**
  * @brief  ����ȫ���㷨,��������ת��
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  *              	Xǰ(+)��(-)     Y��(-)��(+)     ZŤͷ
  */




void Chassis_Omni_Move_Calculate(void)//�ƶ�����
{
	static float rotate_ratio_fl;//ǰ��
	static float rotate_ratio_fr;//ǰ��
  	static float rotate_ratio_bl;//����
  	static float rotate_ratio_br;//����
  	static float wheel_rpm_ratio;
	
	
	float speed_max;
	
	if(1)//��̨���ڵ�������
	{
		rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET)/RADIAN_COEF;
		rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET)/RADIAN_COEF;
		rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET)/RADIAN_COEF;
		rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET)/RADIAN_COEF;
	}
	else
	{
		rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
		rotate_ratio_fl = rotate_ratio_fr;
		rotate_ratio_bl = rotate_ratio_fr;
		rotate_ratio_br = rotate_ratio_fr;
	}
	
	wheel_rpm_ratio = 60.0f/(PERIMETER * CHASSIS_DECELE_RATIO); //	60/�ܳ�*������
	
	//ȫ���㷨
	Chassis_Speed_Target[LEFT_FRON_201] = +( +Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_fl ) * wheel_rpm_ratio;
	Chassis_Speed_Target[RIGH_FRON_202] = -( -Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) -Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_fr ) * wheel_rpm_ratio;
	Chassis_Speed_Target[LEFT_BACK_203] = +( -Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_bl ) * wheel_rpm_ratio;
	Chassis_Speed_Target[RIGH_BACK_204] = -( +Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) -Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_br ) * wheel_rpm_ratio;
	
	//�޷�
	if(0/*Cap_Out_Can_Open() == TRUE*/)//���ݷŵ�(����û�е��ݣ��Ȳ��жϣ����ϵ��ݺ󣬼ǵø�һ���߼�)
	{
		speed_max = Omni_SupCap_Max;
	}
	else
	{
		speed_max = Omni_Speed_Max;
	}
	//������ǰ�ٶ���Ŀ���ٶȵĴ�С
  Chassis_Speed_Target[LEFT_FRON_201] = constrain_float( Chassis_Speed_Target[LEFT_FRON_201], -speed_max, speed_max);
	Chassis_Speed_Target[RIGH_FRON_202] = constrain_float( Chassis_Speed_Target[RIGH_FRON_202], -speed_max, speed_max);
	Chassis_Speed_Target[LEFT_BACK_203] = constrain_float( Chassis_Speed_Target[LEFT_BACK_203], -speed_max, speed_max);
	Chassis_Speed_Target[RIGH_BACK_204] = constrain_float( Chassis_Speed_Target[RIGH_BACK_204], -speed_max, speed_max);
	
	if(actChassis == CHASSIS_SZUPUP)//����ǰ�ֻ�򻬣������޵ñȺ���ҪС
	{
		//ǰ���޷�����ʵ����������������ֲ��ù�
		Chassis_Speed_Target[LEFT_FRON_201] = constrain_float( Chassis_Speed_Target[LEFT_FRON_201], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
		Chassis_Speed_Target[RIGH_FRON_202] = constrain_float( Chassis_Speed_Target[RIGH_FRON_202], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
		Chassis_Speed_Target[LEFT_BACK_203] = constrain_float( Chassis_Speed_Target[LEFT_BACK_203], -Omni_Speed_Max, Omni_Speed_Max);
		Chassis_Speed_Target[RIGH_BACK_204] = constrain_float( Chassis_Speed_Target[RIGH_BACK_204], -Omni_Speed_Max, Omni_Speed_Max);
	}
		
}
/**
  * @brief  ���̵��PID����,����
  * @param  ���ID
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  */
void Chassis_Motor_Speed_PID( eChassisWheel eWheel ) 
{
	//�����ٶ����
	Chassis_Speed_Error[eWheel] = Chassis_Speed_Target[eWheel] - Chassis_Speed_Measure[eWheel];
//	Chassis_Speed_Error[eWheel] = KalmanFilter(&Chassis_Speed_Kalman[eWheel], Chassis_Speed_Error[eWheel]);
	Chassis_Speed_Error_Sum[eWheel] += Chassis_Speed_Error[eWheel];
	
	pTermChassis[eWheel] =  Chassis_Speed_Error[eWheel]*Chassis_Speed_kpid[eWheel][KP];
	iTermChassis[eWheel] =  Chassis_Speed_Error_Sum[eWheel]*Chassis_Speed_kpid[eWheel][KI] * 0.002f;
	//�����޷�
	iTermChassis[eWheel] = constrain_float(iTermChassis[eWheel],-iTermChassis_Max,iTermChassis_Max);
	
	Chassis_Speed_Error_NOW[eWheel] = Chassis_Speed_Error[eWheel];
	dTermChassis[eWheel] = (Chassis_Speed_Error_NOW[eWheel] - Chassis_Speed_Error_LAST[eWheel])*Chassis_Speed_kpid[eWheel][KD];
	Chassis_Speed_Error_LAST[eWheel] = Chassis_Speed_Error_NOW[eWheel];

	//���������С,��ֹ���Ϊ0ʱͻȻʧ��
	if( pTermChassis[eWheel] * iTermChassis[eWheel] < 0 )
	{
		Chassis_Speed_Error_Sum[eWheel] = constrain_float(Chassis_Speed_Error_Sum[eWheel],
															-(iTermChassis_Max/Chassis_Speed_kpid[eWheel][KI]/5.f),
															 (iTermChassis_Max/Chassis_Speed_kpid[eWheel][KI]/5.f));
	}
	
	pidTermChassis[eWheel] = pTermChassis[eWheel] + iTermChassis[eWheel] + dTermChassis[eWheel];

	pidTermChassis[eWheel] = constrain_float(pidTermChassis[eWheel],-Chassis_Final_Output_Max,Chassis_Final_Output_Max);	
	
	//��¼�������
	Chassis_Final_Output[eWheel] = pidTermChassis[eWheel];
}


	
	/**
  * @brief  �ֱ��4�������PID����,���������������(���͸������ֵ)
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  */
void Chassis_MotorOutput(void)
{
	Chassis_Motor_Speed_PID(LEFT_FRON_201);
	Chassis_Motor_Speed_PID(RIGH_FRON_202);
	Chassis_Motor_Speed_PID(LEFT_BACK_203);
	Chassis_Motor_Speed_PID(RIGH_BACK_204);
}

float Chassis_Z_Speed_PID(void)
{
	static float error[3];
	
	error[NOW] = Chassis_Gyro_Error;
	
	Chassis_Move_Z = error[NOW]*Chassis_Z_kpid[KP] + Chassis_Z_kpid[KD]*(error[NOW] - error[LAST]);
	Chassis_Move_Z = constrain(Chassis_Move_Z, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);

	error[LAST] = error[NOW];
	
	return Chassis_Move_Z;
}





///*****************���̹���*************************/

///**
//  * @brief  ���̹�������
//  * @param  void
//  * @retval void
//  * @attention  �ڵ��������������,��Ҫ�Ǳ������㷨,ICRA
//  */
//void Chassis_Power_Limit(void)
//{
//	float    kLimit = 1;//��������ϵ��
//	float    chassis_totaloutput = 0;//ͳ�����������
//	float    Joule_Residue = 0;//ʣ�ཹ����������
//	int16_t  judgDataCorrect = 0;//����ϵͳ�����Ƿ����	
//	static int32_t judgDataError_Time = 0;
//	judgDataCorrect = JUDGE_sGetDataState();//����ϵͳ�����Ƿ����
//	Joule_Residue = JUDGE_fGetRemainEnergy();//ʣ�ཹ������	
//	
//	//ͳ�Ƶ��������
//	chassis_totaloutput = ABS(Chassis_Final_Output[0]) + ABS(Chassis_Final_Output[1])
//							+ ABS(Chassis_Final_Output[2]) + ABS(Chassis_Final_Output[3]);
//	
//		if(judgDataCorrect == JUDGE_DATA_ERROR)//����ϵͳ��Чʱǿ������
//	{
//		judgDataError_Time++;
//		if(judgDataError_Time > 100)
//		{
//			fTotalCurrentLimit = 8000;//��Ϊ����1/4
//		}
//	}
//	
//	else
//	{
////		judgDataError_Time = 0;
////		//ʣ�ཹ������С,��ʼ�������,����ϵ��Ϊƽ����ϵ
////		if(Joule_Residue < WARNING_REMAIN_POWER)
////		{
////			kLimit = (float)(Joule_Residue / WARNING_REMAIN_POWER)
////						* (float)(Joule_Residue / WARNING_REMAIN_POWER);
////			
////			fTotalCurrentLimit = kLimit * fChasCurrentLimit;
////		}
////		else   //���������ָ���һ����ֵ
////		{
////			fTotalCurrentLimit = fChasCurrentLimit;
////		}
//        if(PowerHeatData.chassis_power>70)
//        {
//            kLimit=((PowerHeatData.chassis_power-70)/70)*((PowerHeatData.chassis_power-70)/70);
//            fTotalCurrentLimit = kLimit * fChasCurrentLimit;
//        }
//        else   //���������ָ���һ����ֵ
//		{
//			fTotalCurrentLimit = fChasCurrentLimit;
//		}
//	}
//	
//	//���̸�����������·���
//	if (chassis_totaloutput > fTotalCurrentLimit)
//	{
//		Chassis_Final_Output[0] = (int16_t)(Chassis_Final_Output[0] / chassis_totaloutput * fTotalCurrentLimit);
//		Chassis_Final_Output[1] = (int16_t)(Chassis_Final_Output[1] / chassis_totaloutput * fTotalCurrentLimit);
//		Chassis_Final_Output[2] = (int16_t)(Chassis_Final_Output[2] / chassis_totaloutput * fTotalCurrentLimit);
//		Chassis_Final_Output[3] = (int16_t)(Chassis_Final_Output[3] / chassis_totaloutput * fTotalCurrentLimit);	
//	}
//}
/**************����ģʽ��������********************/

/**
  * @brief  ���̼���б�º���
  * @param  �жϰ����Ƿ񱻰���, ʱ����, ÿ�����ӵ���, һ��Ҫ��С����
  * @retval б�±���ϵ��
  * @attention  0~1
  */
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
	float  factor = 0;

	
	factor = 0.15 * sqrt( 0.15 * (*time) );  //�����ٶ�б��,time�ۼӵ�296.3б�¾����
	
	if (status == 1)//����������
	{
		if (factor < 1)//��ֹtime̫��
		{
			*time += inc;
		}
	}
	else		//�����ɿ�
	{
		if (factor > 0)
		{
			*time -= dec;

			if (*time < 0)
			{
				*time = 0;
			}
		}
	}

	factor = constrain_float( factor, 0, 1 );//ע��һ����float�����޷�

	return factor;  //ע�ⷽ��
}

/**
  * @brief  ��ȡ�����ƶ�ģʽ
  * @param  void
  * @retval TRUE:��еģʽ    false:������ģʽ
  * @attention  
  */
bool CHASSIS_IfActiveMode(void)
{
	if (modeChassis == MECH)
	{
		return TRUE;//��е
	}
	else
	{
		return FALSE;//������
	}
}


/**
  * @brief  Ťƨ��ר�ã�ʱ�䲻��棩
  * @param  ��ǰ��̨ƫ������Ŀ��ƫ����
  * @retval ��ת�ٶ�
  * @attention 
  */
/*float Chassis_Z_Corgi(float get, float set)
{
	float error[2];
	float z = 0;
	
	error[NOW] = set - get;
	
	z = 15*((error[NOW]*8) + 10*(error[NOW] - error[LAST]));//PD����
	z = constrain_float(z, -5000, 5000);

	error[LAST] = error[NOW];
	
	return z;
}*/


	
/**
  * @brief  ��ת�ٶ�PID����
  * @param  ��ǰ��̨ƫ������Ŀ��ƫ����
  * @retval ��ת�ٶ�
  * @attention 
  */
float speed_z_pterm = 0;
float speed_z_iterm = 0;
float speed_z_dterm = 0;
float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp)
{
	static int16_t ErrorPrev = 0;//�ϴ�ƫ�����
	static int32_t ErrorSum = 0;//�ϴ�ƫ�����
	static int32_t ErrorPR = 0;
	static int32_t ErrorPR_KF = 0;
	
	float speed_z = 0;

	ErrorPR_KF = KalmanFilter(&Chassis_Error_Kalman, ErrorReal);
	
	//P
	speed_z_pterm = ErrorReal * kp;//����yawƫ�����ļ������
	speed_z_pterm = constrain_float(speed_z_pterm, -REVOLVE_MAX_NORMAL, REVOLVE_MAX_NORMAL);
	
	//I
	ErrorSum -= ErrorPR_KF;
	speed_z_iterm = ErrorSum*3*0.002f;
	if( abs(ErrorReal) <= 10)
	{
		ErrorSum = 0;
	}
	//�����޷�
	speed_z_iterm = constrain_float(speed_z_iterm,-5000,5000);
	
	//D
	ErrorPR = ErrorPR_KF - ErrorPrev;
	
	if( abs(ErrorPR_KF) > REVOLVE_ANGLE )
	{
		speed_z_dterm = -(ErrorPR) * REVOLVE_KD;//600;//650;//125.f;
	}
	else
	{
		speed_z_dterm = 0;
	}
	//Ťͷ����ٶ��޷�
	speed_z = speed_z_pterm + speed_z_dterm;// + speed_z_iterm;//// + ;
	speed_z = constrain(speed_z, -Chassis_Revolve_Move_Max, +Chassis_Revolve_Move_Max);

	ErrorPrev = ErrorPR_KF;//��¼�ϴ����
	
	return speed_z;
}