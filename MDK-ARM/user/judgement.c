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

/*****************系统数据定义**********************/
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

xFrameHeader              FrameHeader;		//发送帧头信息
ext_SendClientData_t      ShowData;			//客户端信息
ext_SendClientData_t      Graphic;			//客户端图形
ext_CommunatianData_t     CommuData;		//队友通信信息
/****************************************************/

bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID


/**************裁判系统数据辅助****************/
uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = FALSE;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用
#define BLUE  0
#define RED   1

/**
  * @brief  读取裁判数据,中断中读取保证速度
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析
	
	/***------------------*****/
	//无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//帧头CRC8校验
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//都校验过了则说明数据可用
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
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
//						if(RobotHurt.hurt_type == 0)//非装甲板离线造成伤害
//						{	Hurt_Data_Update = TRUE;	}//装甲数据每更新一次则判定为受到一次伤害
//					break;
//					
					case ID_shoot_data:      			//0x0207
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
						JUDGE_ShootNumCount();//发弹量统
					break;
				}
				//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//如果一个数据包出现了多帧数据,则再次读取
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}
		}
		//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//如果一个数据包出现了多帧数据,则再次读取
			Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//辅助函数用
	}
	else		//只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE;//辅助函数用
	}
	CAN2_send_judgement();
	return retval_tf;//对数据正误做处理
}

/**
  * @brief  发送数据给队友
  * @param  void
  * @retval void
  * @attention  步兵向哨兵发送
  */
#define Teammate_max_len     200
unsigned char TeammateTxBuffer[Teammate_max_len];
bool Send_Color = 0;
bool First_Time_Send_Commu = FALSE;
uint16_t send_time = 0;
void Send_to_Teammate(void)
{
	static u8 datalength,i;
	
	extern void usart3_dma_init(void);//使能USART3的DMA
	
	Send_Color = is_red_or_blue();//判断发送给哨兵的颜色,17哨兵(蓝),7哨兵(红)；
	
	memset(TeammateTxBuffer,0,200);
	
	CommuData.txFrameHeader.SOF = 0xA5;
	CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
	CommuData.txFrameHeader.Seq = 0;
	memcpy(TeammateTxBuffer, &CommuData.txFrameHeader, sizeof(xFrameHeader));
	Append_CRC8_Check_Sum(TeammateTxBuffer, sizeof(xFrameHeader));	
	
	CommuData.CmdID = 0x0301;
	
	   
	CommuData.dataFrameHeader.send_ID = Judge_Self_ID;//发送者的ID
	
	
//	if( Senty_Run() == TRUE)
//	{
//		Senty_Run_Clean_Flag();
//		First_Time_Send_Commu = TRUE;
//	}
	
	if( First_Time_Send_Commu == TRUE )
	{
		CommuData.dataFrameHeader.data_cmd_id = 0x0292;//在0x0200-0x02ff之间选择
		send_time++;
		if(send_time >= 20)
		{
			First_Time_Send_Commu = FALSE;
		}
		if(Send_Color == BLUE)//自己是蓝，发给蓝哨兵
		{
			CommuData.dataFrameHeader.receiver_ID = 17;//接收者ID
		}
		else if(Send_Color == RED)//自己是红，发给红哨兵
		{
			CommuData.dataFrameHeader.receiver_ID = 7;//接收者ID
		}
	}
	else
	{
		CommuData.dataFrameHeader.data_cmd_id = 0x0255;
		send_time = 0;
		CommuData.dataFrameHeader.receiver_ID = 88;//随便给个ID，不发送
	}
	
	CommuData.interactData.data[0] = 0;//发送的内容 //大小不要超过变量的变量类型   
	
	memcpy(TeammateTxBuffer+5,(uint8_t *)&CommuData.CmdID,(sizeof(CommuData.CmdID)+sizeof(CommuData.dataFrameHeader)+sizeof(CommuData.interactData)));		
	Append_CRC16_Check_Sum(TeammateTxBuffer,sizeof(CommuData));
	
	datalength = sizeof(CommuData); 
	if( First_Time_Send_Commu == TRUE )
	{
		for(i = 0;i < datalength;i++)
		{
//			USART_SendData(UART5,(uint16_t)TeammateTxBuffer[i]);			//有待用DMA的发送函数HAL_UART_Transmit_DMA替换
//			while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);		//有待用DMA的flag标志获取函数__HAL_UART_GET_FLAG替换
		}	 
	}
}

/**
  * @brief  上传自定义数据
  * @param  void
  * @retval void
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
#define send_max_len     200
unsigned char CliendTxBuffer[send_max_len];
void JUDGE_Show_Data(void)
{
//	static u8 datalength,i;
//	uint8_t judge_led = 0xff;//初始化led为全绿
//	static uint8_t auto_led_time = 0;
//	static uint8_t buff_led_time = 0;
	
	determine_ID();//判断发送者ID和其对应的客户端ID
	
	ShowData.txFrameHeader.SOF = 0xA5;
	ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);
	ShowData.txFrameHeader.Seq = 0;
	memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	
	ShowData.CmdID = 0x0301;
	
	ShowData.dataFrameHeader.data_cmd_id = 0xD180;//发给客户端的cmd,官方固定
	//ID已经是自动读取的了
	ShowData.dataFrameHeader.send_ID 	 = Judge_Self_ID;//发送者的ID
	ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;//客户端的ID，只能为发送者机器人对应的客户端
	
	/*- 自定义内容 -*/
//	ShowData.clientData.data1 = (float)Capvoltage_Percent();//电容剩余电量

}

/**
  * @brief  上传自定义图形
  * @param  void
  * @retval void
  * @attention  按自定义UI操作文档规划图形后,由机器人将数据打包传给对应客户端（机器人和程序的ID由程序自行判断）
  */
unsigned char ClientGraphicTxBuffer[send_max_len];

void JUDGE_Show_Graphic(void)
{
	determine_ID();//判断发送者ID和其对应的客户端ID 
	Graphic.txFrameHeader.SOF = 0xA5;
	Graphic.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_UIcustom_reference_line);
	Graphic.txFrameHeader.Seq = 0;											//?????????????????????是0还是1？？？
	memcpy(ClientGraphicTxBuffer, &Graphic.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(ClientGraphicTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码

	Graphic.CmdID = 0x0005;//命令码ID，从裁判系统用户接口说明中得知

	Graphic.dataFrameHeader.data_cmd_id = 0x100;//客户端的内容ID,官方固定
	Graphic.dataFrameHeader.send_ID 	 = Judge_Self_ID;//机器人的ID
	Graphic.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;//客户端的ID
	ShowData.txFrameHeader.Seq = 0;//	???????????????????????????????????????????????????????这句代码是不是多余的？去掉试试？
}

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//读取当前机器人ID
	
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
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//计算客户端ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//计算客户端ID
	}
}
/********************裁判数据辅助判断函数***************************/

/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassis_power);
}
/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量(最大60)
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
	if(Shoot_Speed_Last != Shoot_Speed_Now)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
	{
		ShootNum++;
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
}

#endif
