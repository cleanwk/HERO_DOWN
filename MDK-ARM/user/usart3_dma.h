#ifndef BSP_USART_H
#define BSP_USART_H

#include "stdint.h"
#include "stm32f4xx_hal_uart.h"

#define USART_Judegebuff_SIZE      (200u)
//#define DR16_DATA_LEN            (18u)

extern uint8_t USART_Judegebuff[USART_Judegebuff_SIZE];

extern void usart3_dma_init(void);
extern uint8_t usart3_rx_handle(UART_HandleTypeDef *huart);//USART3中断处理函数(串口空闲中断)
extern void UART_IdleRxCallback(UART_HandleTypeDef *huart);
extern int UART_Receive_DMA_No_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size);








#endif
