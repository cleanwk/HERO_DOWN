#ifndef __SYSTEM_H
#define __SYSTEM_H

#define System_base_tim &htim3
#define INFANTRY_ID 3
#include "main.h"

#include "JY901.h"
#include "chassic.h"
#include "mytype.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "pid.h"
#include "user_tim.h"
#include <stdint.h>
#include "usart3_dma.h"
#include "judgement.h"
#include "freecars_usart.h"
#include "Calculate.h"

#define ABS(x)		((x>0)? (x): (-x)) 

#define gpio_on HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET)
#define gpio_off HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET)
#define  error_num 1//´ý²â
typedef enum
{
    RC_offline=0
}error;
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
extern uint16_t system_tick;
extern  uint8_t Rx[44];

void Task_Dispatch_tim(TIM_HandleTypeDef *htim);
void SystemClock_Config(void);
void init_system(void);
void Task_error(void);
void System_Run(void);
void System_Stop(void);
uint8_t SYSTEM_UpdateSystemState(void);
#endif
