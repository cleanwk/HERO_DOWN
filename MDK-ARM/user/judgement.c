#include "judgement.h"
#include "string.h"

#include "crc.h"
#include "usart.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "stdint.h"
#include "string.h"
/*-------------------------2019--------------------------------*/
#if JUDGE_VERSION == JUDGE_19

/*****************ϵͳ���ݶ���**********************/
ext_game_state_t       				GameState;					//0x0001
ext_game_result_t            		GameResult;					//0x0002
ext_game_robot_survivors_t          GameRobotSurvivors;			//0x0003
ext_event_data_t        			EventData;					//0x0101
ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
ext_supply_projectile_booking_t		SupplyProjectileBooking;	//0x0103
ext_game_robot_state_t			  	GameRobotStat;				//0x0201
ext_power_heat_data_t		  		PowerHeatData;				//0x0202
ext_game_robot_pos_t				GameRobotPos;				//0x0203
ext_buff_musk_t						BuffMusk;					//0x0204
aerial_robot_energy_t				AerialRobotEnergy;			//0x0205
ext_robot_hurt_t					RobotHurt;					//0x0206
ext_shoot_data_t					ShootData;					//0x0207
ext_game_robot_state_t              RobotState;

xFrameHeader              FrameHeader;		//����֡ͷ��Ϣ
ext_SendClientData_t      ShowData;			//�ͻ�����Ϣ
ext_SendClientData_t      Graphic;			//�ͻ���ͼ��
ext_CommunatianData_t     CommuData;		//����ͨ����Ϣ
/****************************************************/

bool Judge_Data_TF = FALSE;//���������Ƿ����,������������
uint8_t Judge_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID


/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = FALSE;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������
#define BLUE  0
#define RED   1

/**
  * @brief  ��ȡ��������,�ж��ж�ȡ��֤�ٶ�
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
	
	uint16_t judge_length;//ͳ��һ֡���ݳ��� 
	
	int CmdID = 0;//�������������
	
	/***------------------*****/
	//�����ݰ��������κδ���
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch(CmdID)
				{
					case ID_game_state:        			//0x0001
						memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
					break;
					
					case ID_game_result:          		//0x0002
						memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_survivors:       //0x0003
						memcpy(&GameRobotSurvivors, (ReadFromUsart + DATA), LEN_game_robot_survivors);
					break;
					
					case ID_event_data:    				//0x0101
						memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:   //0x0102
						memcpy(&SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_supply_projectile_booking:  //0x0103
						memcpy(&SupplyProjectileBooking, (ReadFromUsart + DATA), LEN_supply_projectile_booking);
					break;
					
					case ID_game_robot_state:      		//0x0201
						memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;
					
					case ID_power_heat_data:      		//0x0202
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
					break;
					
					case ID_game_robot_pos:      		//0x0203
						memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;
					
					case ID_buff_musk:      			//0x0204
						memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205
						memcpy(&AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
//					case ID_robot_hurt:      			//0x0206
//						memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
//						if(RobotHurt.hurt_type == 0)//��װ�װ���������˺�
//						{	Hurt_Data_Update = TRUE;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
//					break;
//					
					case ID_shoot_data:      			//0x0207
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
						JUDGE_ShootNumCount();//������ͳ
					break;
				}
				//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}
		}
		//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
			Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//����������
	}
	else		//ֻҪCRC16У�鲻ͨ����ΪFALSE
	{
		Judge_Data_TF = FALSE;//����������
	}
	CAN2_send_judgement();
	return retval_tf;//����������������
}

/**
  * @brief  �������ݸ�����
  * @param  void
  * @retval void
  * @attention  �������ڱ�����
  */
#define Teammate_max_len     200
unsigned char TeammateTxBuffer[Teammate_max_len];
bool Send_Color = 0;
bool First_Time_Send_Commu = FALSE;
uint16_t send_time = 0;
void Send_to_Teammate(void)
{
	static u8 datalength,i;
	
	extern void usart3_dma_init(void);//ʹ��USART3��DMA
	
	Send_Color = is_red_or_blue();//�жϷ��͸��ڱ�����ɫ,17�ڱ�(��),7�ڱ�(��)��
	
	memset(TeammateTxBuffer,0,200);
	
	CommuData.txFrameHeader.SOF = 0xA5;
	CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
	CommuData.txFrameHeader.Seq = 0;
	memcpy(TeammateTxBuffer, &CommuData.txFrameHeader, sizeof(xFrameHeader));
	Append_CRC8_Check_Sum(TeammateTxBuffer, sizeof(xFrameHeader));	
	
	CommuData.CmdID = 0x0301;
	
	   
	CommuData.dataFrameHeader.send_ID = Judge_Self_ID;//�����ߵ�ID
	
	
//	if( Senty_Run() == TRUE)
//	{
//		Senty_Run_Clean_Flag();
//		First_Time_Send_Commu = TRUE;
//	}
	
	if( First_Time_Send_Commu == TRUE )
	{
		CommuData.dataFrameHeader.data_cmd_id = 0x0292;//��0x0200-0x02ff֮��ѡ��
		send_time++;
		if(send_time >= 20)
		{
			First_Time_Send_Commu = FALSE;
		}
		if(Send_Color == BLUE)//�Լ��������������ڱ�
		{
			CommuData.dataFrameHeader.receiver_ID = 17;//������ID
		}
		else if(Send_Color == RED)//�Լ��Ǻ죬�������ڱ�
		{
			CommuData.dataFrameHeader.receiver_ID = 7;//������ID
		}
	}
	else
	{
		CommuData.dataFrameHeader.data_cmd_id = 0x0255;
		send_time = 0;
		CommuData.dataFrameHeader.receiver_ID = 88;//������ID��������
	}
	
	CommuData.interactData.data[0] = 0;//���͵����� //��С��Ҫ���������ı�������   
	
	memcpy(TeammateTxBuffer+5,(uint8_t *)&CommuData.CmdID,(sizeof(CommuData.CmdID)+sizeof(CommuData.dataFrameHeader)+sizeof(CommuData.interactData)));		
	Append_CRC16_Check_Sum(TeammateTxBuffer,sizeof(CommuData));
	
	datalength = sizeof(CommuData); 
	if( First_Time_Send_Commu == TRUE )
	{
		for(i = 0;i < datalength;i++)
		{
//			USART_SendData(UART5,(uint16_t)TeammateTxBuffer[i]);			//�д���DMA�ķ��ͺ���HAL_UART_Transmit_DMA�滻
//			while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);		//�д���DMA��flag��־��ȡ����__HAL_UART_GET_FLAG�滻
		}	 
	}
}

/**
  * @brief  �ϴ��Զ�������
  * @param  void
  * @retval void
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
#define send_max_len     200
unsigned char CliendTxBuffer[send_max_len];
void JUDGE_Show_Data(void)
{
//	static u8 datalength,i;
//	uint8_t judge_led = 0xff;//��ʼ��ledΪȫ��
//	static uint8_t auto_led_time = 0;
//	static uint8_t buff_led_time = 0;
	
	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
	
	ShowData.txFrameHeader.SOF = 0xA5;
	ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);
	ShowData.txFrameHeader.Seq = 0;
	memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//д��֡ͷCRC8У����
	
	ShowData.CmdID = 0x0301;
	
	ShowData.dataFrameHeader.data_cmd_id = 0xD180;//�����ͻ��˵�cmd,�ٷ��̶�
	//ID�Ѿ����Զ���ȡ����
	ShowData.dataFrameHeader.send_ID 	 = Judge_Self_ID;//�����ߵ�ID
	ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	
	/*- �Զ������� -*/
//	ShowData.clientData.data1 = (float)Capvoltage_Percent();//����ʣ�����

}

/**
  * @brief  �ϴ��Զ���ͼ��
  * @param  void
  * @retval void
  * @attention  ���Զ���UI�����ĵ��滮ͼ�κ�,�ɻ����˽����ݴ��������Ӧ�ͻ��ˣ������˺ͳ����ID�ɳ��������жϣ�
  */
unsigned char ClientGraphicTxBuffer[send_max_len];

void JUDGE_Show_Graphic(void)
{
	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID 
	Graphic.txFrameHeader.SOF = 0xA5;
	Graphic.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_UIcustom_reference_line);
	Graphic.txFrameHeader.Seq = 0;											//?????????????????????��0����1������
	memcpy(ClientGraphicTxBuffer, &Graphic.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(ClientGraphicTxBuffer, sizeof(xFrameHeader));//д��֡ͷCRC8У����

	Graphic.CmdID = 0x0005;//������ID���Ӳ���ϵͳ�û��ӿ�˵���е�֪

	Graphic.dataFrameHeader.data_cmd_id = 0x100;//�ͻ��˵�����ID,�ٷ��̶�
	Graphic.dataFrameHeader.send_ID 	 = Judge_Self_ID;//�����˵�ID
	Graphic.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID
	ShowData.txFrameHeader.Seq = 0;//	???????????????????????????????????????????????????????�������ǲ��Ƕ���ģ�ȥ�����ԣ�
}

/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//��ȡ��ǰ������ID
	
	if(GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/**
  * @brief  �ж�����ID��ѡ��ͻ���ID
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//����ͻ���ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
	}
}
/********************�������ݸ����жϺ���***************************/

/**
  * @brief  �����Ƿ����
  * @param  void
  * @retval  TRUE����   FALSE������
  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  ��ȡ˲ʱ����
  * @param  void
  * @retval ʵʱ����ֵ
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassis_power);
}
/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役������(���60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassis_power_buffer);
}

float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
	Shoot_Speed_Now = ShootData.bullet_speed;
	if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		ShootNum++;
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
}

#endif
