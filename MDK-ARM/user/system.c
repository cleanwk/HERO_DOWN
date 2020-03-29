#include "system.h"
#include "chassic.h"
int SP1=0,SP2=0,SP3=0;//����ָ��
uint16_t system_tick;
int error_register[error_num];
int a,b,c,d;
int set_spd[4];
 uint8_t Rx[44];
/**
  * @brief  ��Ƭ��ʱ����
  * @param  void
  * @retval void
  * @attention 
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief  ϵͳ�ĸ��ֳ�ʼ������ֵ
  * @param  void
  * @retval void
  * @attention 
  */
void init_system(void)
{
    HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  usart3_dma_init();//�򿪿����жϣ���ʼ������ϵͳusart3_DMA����  

    dbus_uart_init();
	drv_can_init();
    HAL_UART_Receive_DMA(&huart6, Rx, 44);//����6
    __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);

    system_tick=0;
}

/**
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention ����while(1)
  */
void Task_error(void)
{
 
}

/**
  * @brief  ϵͳ����
  * @param  void
  * @retval void
  * @attention 
  */
void System_Run(void)
{
    __HAL_TIM_ENABLE_IT(System_base_tim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(System_base_tim);
}

/**
  * @brief  ϵͳ�ر�
  * @param  void
  * @retval void
  * @attention 
  */
void System_Stop(void)
{
    __HAL_TIM_DISABLE_IT(System_base_tim, TIM_IT_UPDATE);


    __HAL_TIM_DISABLE(System_base_tim);
}

/**
  * @brief  ʱ��Ƭ������Ⱥ���
  * @param  void
  * @retval void
  * @attention ����TIM3��ʹ�ã���ʱ���ж����ȼ�Ӧ���������жϣ���ʱ��ʱ��Ӧƫ��
  */
void Task_Dispatch_tim(TIM_HandleTypeDef *htim)
{
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET)
        {
        __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);//����жϣ����¼���
        gpio_on;//����������
        system_tick++;
        switch(SP1)
        {
            
            case 0:
            SP1=SYSTEM_UpdateSystemState();           
            break;
            case 1:
            Task_Chassis();   
            break;
        }
        gpio_off;//����������
		}    
	}
}

/**********************ϵͳ�����жϼ�����******************/


//����ģʽ
eRemoteMode remoteMode = RC;

//ϵͳ״̬
eSystemState systemState = SYSTEM_STARTING;


/**
  * @brief  ϵͳ����
  * @param  void
  * @retval void
  * @attention 
  */
void SYSTEM_Reset( void )
{
	systemState = SYSTEM_STARTING;
}

/**
  * @brief  ʧ�ر���
  * @param  void
  * @retval void
  * @attention 
  */
//void SYSTEM_OutCtrlProtect(void)
//{
//    SYSTEM_Reset();//ϵͳ�ָ�������״̬
//	REMOTE_vResetData();//ң�����ݻָ���Ĭ��״̬
//	
//	
//	Laser_Off;//�����
//	CHASSIS_StopMotor();//���̹�
//	GIMBAL_StopMotor();//��̨��
//	Magazine_StopCtrl();//���ֹͣת��
//	REVOLVER_StopMotor();//����ֹͣת��
//	FRICTION_StopMotor();//Ħ���ֹ�
//	Super_Cap_StopCtrl();//���ݹرճ�ŵ�
//}

/**
  * @brief  ����ϵͳ״̬
  * @param  void
  * @retval void
  * @attention 1kHz,��LOOPѭ������
  */
uint8_t SYSTEM_UpdateSystemState(void)
{
	static uint32_t  ulInitCnt  =  0;
	
	if (systemState == SYSTEM_STARTING)
	{
		ulInitCnt++;

		if (ulInitCnt > 25)//������ʱ,1ms*2k=2s,Ϊ�˸�MPU����ʱ��
		{
			ulInitCnt = 0;

			systemState = SYSTEM_RUNNING;//�������,ת������ͨģʽ
            return 1;
		}
        
	}
    return 0;
}

/**
  * @brief  ����ģʽѡ��
  * @param  void
  * @retval void
  * @attention ���̻����,���ڴ��Զ���ģʽѡ��ʽ,1msִ��һ��
  */
void SYSTEM_UpdateRemoteMode( void )
{ 
    if (IF_RC_SW2_UP)
	{
		remoteMode = KEY;
	}
	
	else
	{
		remoteMode = RC;
	}
}


//���ؿ���ģʽ
eRemoteMode SYSTEM_GetRemoteMode()
{
	return remoteMode;
}

//����ϵͳ״̬
eSystemState SYSTEM_GetSystemState()
{
	return systemState;

}
