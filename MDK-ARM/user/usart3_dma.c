#include "stm32f4xx_hal.h"
#include "usart.h"
#include "usart3_dma.h"
#include "judgement.h"
#include "string.h"

uint8_t USART_Judegebuff[USART_Judegebuff_SIZE];
	
//初始化，使用见main.c
void usart3_dma_init(void)
{
  UART_Receive_DMA_No_IT(&huart3, USART_Judegebuff,USART_Judegebuff_SIZE );
  
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}
//不开启中断的DMA，大致同Code1.1遥控器，区别在__HAL_LOCK，__HAL_UNLOCK
int UART_Receive_DMA_No_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
  uint32_t tmp = 0;

  tmp = huart->RxState;
  if (tmp == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;

    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* Enable the DMA Stream */
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,
                  (uint32_t)pData, Size);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

//USART3中断处理函数(串口空闲中断),stm32f4xx_it.c中usart3中断处理函数调用

void UART_IdleRxCallback(UART_HandleTypeDef *huart);
uint8_t usart3_rx_handle(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);// 停止本次DMA传输

    /* handle dbus data dbus_buf from DMA */
   
      UART_IdleRxCallback(huart);//空闲回调，读取数据

    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, USART_Judegebuff_SIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);//// 重启开始DMA传输 
  }
  return 0;
		}
	

//自定义空闲中断回调，数据处理
void UART_IdleRxCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
    {
      
      Judge_Read_Data(USART_Judegebuff);		//读取裁判系统数据
			memset(USART_Judegebuff, 0,USART_Judegebuff_SIZE);//清零
			
    }
	
}

