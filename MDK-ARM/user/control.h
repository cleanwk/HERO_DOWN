#ifndef _CONTROL_H
#define _CONTROL_H
typedef enum
{
    RC   = 0,  
    KEY  = 1,  

} eRemoteMode;  // Ò£¿Ø·½Ê½


typedef enum
{
	  SYSTEM_STARTING  = 0,
	  SYSTEM_RUNNING   = 1,

} eSystemState;
eRemoteMode SYSTEM_GetRemoteMode(void);
eSystemState SYSTEM_GetSystemState(void);
#endif
