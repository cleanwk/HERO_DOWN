/**
  ******************************************************************************
  * @file    FreeCars_uart.h
  * @author  FreeCars雄哥
  * @version NULL
  * @date    2014/11/11
  * @brief 	 FreeCars_uart H file.(For XS128)
  *   site:   Http://FreeCars.taobao.com
  *   QQ群：  384273254，149168724
  *   版权：  此代码为FreeCars上位机协议代码，允许任意使用，也允许用于商业用途，但请保留此段文字！
  *   tips:   强烈建议小伙伴们使用FreeCars出品的蓝牙模块套装，无线上位机从这里开始！
  ******************************************************************************
  */
#ifndef __FREECARS_USART_H__
#define __FREECARS_USART_H__

#include "stdint.h"


//以下不要修改
#define UartRxBufferLen  100
#define UartRxDataLen    41           //FreeCars上位机发送浮点数据MCU接收，不要改
#define UartRxCmdLen     7	      //FreeCars上位机接收命令数据长度，不要改

#define UartDataNum      17           //FreeCars上位机接收通道数，按照上位机设置改变!!!

#define UartCmdNum  SerialPortRx.Buffer[SerialPortRx.Stack-3]//命令号
#define UartCmdData SerialPortRx.Buffer[SerialPortRx.Stack-2]//命令数据

typedef struct
{
  int32_t Stack;
  uint8_t Data;
  uint8_t PreData;
  uint8_t Buffer[UartRxBufferLen];
  uint8_t Enable;
  uint8_t Check;
}SerialPortType;

//Export Varibales
extern uint8_t uSendBuf[UartDataNum*2];
extern SerialPortType SerialPortRx;
extern double UartData[9];

//Export Functions
void FreeCarsTool_Init(void);
void sendDataToScope(void);
void push(uint8_t adr,uint16_t date);
extern void FreeCars_Send(void);
extern void FreeCars_OnInterrupt(void);

/***********************user***************************/
#define Uartserive   UART4


#endif
