#include "control.h"
/**********************ϵͳ�����жϼ�����******************/


//����ģʽ
eRemoteMode remoteMode = RC;

//ϵͳ״̬
eSystemState systemState = SYSTEM_STARTING;


//���ؿ���ģʽ
eRemoteMode SYSTEM_GetRemoteMode( )
{
	return remoteMode;
}

//����ϵͳ״̬
eSystemState SYSTEM_GetSystemState( )
{
	return systemState;
}
