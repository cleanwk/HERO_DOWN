#include "stm32f4xx_hal.h"
#include "usart.h"
#include "usart3_dma.h"
#include "judgement.h"
#include "string.h"

uint8_t USART_Judegebuff[USART_Judegebuff_SIZE];
	
//��ʼ����ʹ�ü�main.c
void usart3_dma_init(void)
{
  UART_Receive_DMA_No_IT(&huart3, USART_Judegebuff,USART_Judegebuff_SIZE );
  
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}
//�������жϵ�DMA������ͬCode1.1ң������������__HAL_LOCK��__HAL_UNLOCK
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

//USART3�жϴ�����(���ڿ����ж�),stm32f4xx_it.c��usart3�жϴ���������

void UART_IdleRxCallback(UART_HandleTypeDef *huart);
uint8_t usart3_rx_handle(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);// ֹͣ����DMA����

    /* handle dbus data dbus_buf from DMA */
   
      UART_IdleRxCallback(huart);//���лص�����ȡ����

    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, USART_Judegebuff_SIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);//// ������ʼDMA���� 
  }
  return 0;
		}
	

//�Զ�������жϻص������ݴ���
void UART_IdleRxCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
    {
      
      Judge_Read_Data(USART_Judegebuff);		//��ȡ����ϵͳ����
			memset(USART_Judegebuff, 0,USART_Judegebuff_SIZE);//����
			
    }
	
}

