#include "bsp_can.h"
#include "bsp_uart.h"
#include "judgement.h"

uint32_t TxMailbox1=0;
uint16_t num1=0;
uint16_t num2=0;
moto_handle M3508[4];
moto_handle GM6020[2];


uint32_t can1_error=0;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern int yaw_angle_offset;

void drv_can_init()
{
	CAN_FilterTypeDef can_filter_st;//筛选器结构体变量
	
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;  //32位ID
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;  //32位MASK
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;//过滤器0
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;///过滤器0关联至FIFO0
	can_filter_st.FilterActivation = ENABLE;
	
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//挂起中断允许
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	
	can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
	
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//挂起中断允许
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
}


/***************接收回调函数*******************/
/**
  * @brief  RX_handle 将接收的数据进行转换
  * @brief  CAN1  motor_info[]    CAN2 UWB数据
  * @param  CAN_HandleTypeDef *hcan
  * @retval None
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_StatusTypeDef	HAL_RetVal;
	CAN_RxHeaderTypeDef header;
    uint8_t rx_data[8];
	uint8_t i;
	

	if (hcan == &hcan1)
  {
		HAL_RetVal=HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rx_data);
	
    if ( HAL_OK==HAL_RetVal)
		{			
			                  // get motor index by can_id
            switch (header.StdId)
            {    
                case M3508_A:
                case M3508_B:
                case M3508_C:
                case M3508_D:
                    i=header.StdId-0x201;
                    M3508[i].rotor_angle           = ((rx_data[0] << 8) | rx_data[1]);
                    M3508[i].rotor_speed           = ((rx_data[2] << 8) | rx_data[3]);
                    M3508[i].torque_current        = ((rx_data[4] << 8) | rx_data[5]);
                    M3508[i].temp                  =   rx_data[6];
                    break;
                case GM6020_YAW:
                case GM6020_PITCH:
                    i=header.StdId-0x209;
                    GM6020[i].rotor_angle           = ((rx_data[0] << 8) | rx_data[1]);
                    GM6020[i].rotor_speed           = ((rx_data[2] << 8) | rx_data[3]);
                    GM6020[i].torque_current        = ((rx_data[4] << 8) | rx_data[5]);
                    GM6020[i].temp                  =   rx_data[6];
                    break;
                case GIMBAL_DATA:
                    yaw_angle_offset                =((rx_data[0] << 8) | rx_data[1]);
                break;
            }
		}
	else 
	{
		can1_error++;
	}
	
  }
	
}

/********************发送函数********************/
/**
  * @brief  set_motor_voltage 发送数据
  * @param  id_range  数据发送报头
  * @param  v1 0-1位数据
  * @param  v2 2-3位数据
  * @param  v3 4-5位数据
  * @param  v4 6-7位数据
  * @retval None
  */


void Chassis_set_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];
	
	tx_header.StdId = 0x200;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;

	tx_data[0] = (v1>>8)&0xff;
	tx_data[1] =    (v1)&0xff;
	tx_data[2] = (v2>>8)&0xff;
	tx_data[3] =    (v2)&0xff;
	tx_data[4] = (v3>>8)&0xff;
	tx_data[5] =    (v3)&0xff;
	tx_data[6] = (v4>>8)&0xff;
	tx_data[7] =    (v4)&0xff;
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}

void Gimbaol_set_voltage(int16_t v1, int16_t v2)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];
	
	tx_header.StdId = 0x2ff;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 4;

	tx_data[0] = (v1>>8)&0xff;
	tx_data[1] =    (v1)&0xff;
	tx_data[2] = (v2>>8)&0xff;
	tx_data[3] =    (v2)&0xff;
	
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}
void CAN2_send_massage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
	CAN_TxHeaderTypeDef tx_header;
     uint8_t             tx_data[8];
    //can2_send_ffff++;
  tx_header.StdId = 0x300;//!!!
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}

void CAN2_send_RM1(void)
{
	CAN_TxHeaderTypeDef tx_header;
     uint8_t             tx_data[8];
    //can2_send_ffff++;
  tx_header.StdId = CAN_RM_Rx1;//!!!
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;
    
   	tx_data[0] = (rc_t.rc.ch1>>8)&0xff;
	tx_data[1] =    (rc_t.rc.ch1)&0xff;
	tx_data[2] = (rc_t.rc.ch2>>8)&0xff;
	tx_data[3] =    (rc_t.rc.ch2)&0xff;
	tx_data[4] = (rc_t.rc.ch3>>8)&0xff;
	tx_data[5] =    (rc_t.rc.ch3)&0xff;
	tx_data[6] = (rc_t.rc.ch4>>8)&0xff;
	tx_data[7] =    (rc_t.rc.ch4)&0xff;
    
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}
void CAN2_send_RM2(void)
{
	CAN_TxHeaderTypeDef tx_header;
     uint8_t             tx_data[8];
    //can2_send_ffff++;
  tx_header.StdId = CAN_RM_Rx2;//!!!
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;
    
   	tx_data[0] = (rc_t.mouse.x>>8)&0xff;
	tx_data[1] =    (rc_t.mouse.x)&0xff;
	tx_data[2] = (rc_t.mouse.y>>8)&0xff;
	tx_data[3] =    (rc_t.mouse.y)&0xff;
	tx_data[4] = (rc_t.mouse.z>>8)&0xff;
	tx_data[5] =    (rc_t.mouse.z)&0xff;
	tx_data[6] = (rc_t.kb.key_code>>8)&0xff;
	tx_data[7] =    (rc_t.kb.key_code)&0xff;
    
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
num2++;
}

void CAN2_send_RM3(void)
{
	CAN_TxHeaderTypeDef tx_header;
     uint8_t             tx_data[8];
    //can2_send_ffff++;
  tx_header.StdId = CAN_RM_Rx3;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 3;
    
   	tx_data[0] = ((rc_t.rc.sw1&0x03)|((rc_t.rc.sw2&0x03)<<3)|((rc_t.mouse.l&0x01)<<6)|((rc_t.mouse.r&0x01)<<7))&0xff;
	
	tx_data[1] = (rc_t.wheel>>8)&0xff;
	tx_data[2] =    (rc_t.wheel)&0xff;
	
    
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

void CAN2_send_judgement(void)
{

	CAN_TxHeaderTypeDef tx_header;
     uint8_t             tx_data[8];
    //can2_send_ffff++;
  tx_header.StdId = CAN_RM_judgement;//!!!
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;
    
   	tx_data[0] = (PowerHeatData.shooter_heat0>>8)&0xff;
	tx_data[1] =    (PowerHeatData.shooter_heat0)&0xff;
	tx_data[2] = (ShootNum>>8)&0xff;
	tx_data[3] =    (ShootNum)&0xff;
	tx_data[4] = ((uint16_t)ShootData.bullet_speed>>8)&0xff;
	tx_data[5] =    ((uint16_t)ShootData.bullet_speed)&0xff;
	tx_data[6] = (RobotState.shooter_heat0_cooling_limit>>8)&0xff;
	tx_data[7] =    (RobotState.shooter_heat0_cooling_limit)&0xff;
	

  
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
   num1++;
	}

	void CAN1_send_RM1(void)
{
    uint8_t send;
	CAN_TxHeaderTypeDef tx_header;
     uint8_t             tx_data[8];
    //can2_send_ffff++;
	if(IF_RC_SW1_MID)
		send=1;
  tx_header.StdId = 0X00000011;//!!!
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;
    
   	tx_data[0] = (send)&0xff;
	tx_data[1] =   0&0xff;
	tx_data[2] = 0&0xff;
	tx_data[3] =    0&0xff;
	tx_data[4] = 0&0xff;
	tx_data[5] = 0&0xff;
	tx_data[6] =0&0xff;
	tx_data[7] =  0&0xff;
    
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}

void CAN2_send_RM(void)
{
    CAN2_send_RM1();
    CAN2_send_RM2();
    CAN2_send_RM3();
	CAN1_send_RM1();
    RM_Update_Flag=0;
}



/*******************发送邮箱回调*******************/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{ 
	
	if (hcan == &hcan1)
  {
		
  }
    
  else if (hcan == &hcan2)
 {
  }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	TxMailbox1++;
	if (hcan == &hcan1)
  {
	
  }
else if (hcan == &hcan2)
 {
 }
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	
	if (hcan == &hcan1)
  {
		
  }
  else if (hcan == &hcan2)
 {
 }
}
