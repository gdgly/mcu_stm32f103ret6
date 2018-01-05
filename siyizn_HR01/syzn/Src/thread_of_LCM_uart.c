#include "stm32f1xx_hal.h"
#include "misc.h"
#include "cmsis_os.h"
#include "thread_of_LCM_uart.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "string.h"


void thread_of_LCM_uart (void const *argument);                             // thread function
void crc16( unsigned char *ptr,unsigned int len);




osThreadId tid_LCM_uart;                                          // thread id
osThreadDef (thread_of_LCM_uart, osPriorityRealtime, 1, 0);                   // thread object

#define EVENT_LOOP_TIME_IN_MILLI_SECOND 300
#define DELAY_1S 1000

struct lcm_parameter_t{
		uint8_t pic_id;
		uint16_t all_times;
		uint16_t current_times;
		uint8_t hand_m;													//防止多次进入中断导致手动模式异常
		uint8_t hand_state;
		uint32_t time_now[3];
		uint32_t time_old[3];
}user_parameter;

int SendAndRecv(struct lcm_parameter_t *p,uint32_t millisec);
void setpic(uint8_t pic);
void lcm_int(uint8_t times);
									
										
int init_thread_of_LCM_uart (void) {

	tid_LCM_uart = osThreadCreate (osThread(thread_of_LCM_uart), NULL);
	if (!tid_LCM_uart) {
		return 0;
	}
  
	return 1;
}




void thread_of_LCM_uart (void const *argument) {
	user_signal_info_t user_timer_info = { osThreadGetId(), SIG_USER_TIMER };
	osTimerDef(main_timer, SetUserSignal);
	osTimerId main_timer_id = osTimerCreate(osTimer(main_timer), osTimerPeriodic, &user_timer_info);
	osTimerStart(main_timer_id, EVENT_LOOP_TIME_IN_MILLI_SECOND);

//	while(1)
//	{	
//		
//		osSignalWait(SIG_USER_TIMER, osWaitForever);
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
//		static int cnt = 0;
//		cnt ++;
//		printf("cnt = %04x\r\n",cnt);
//	}	
//	
	
	while(1)
	{		
																	
//lcm屏会在上电时发送  Serial screen to start successfully!  因此需要先将这段数据读取到才能进行后续的操作
	static int lmc_reday = 1,hd_fq = 0;
	while(lmc_reday)
	{
		uint8_t buf[128];
		GetDataFromUart(4,buf,50,500);
		if(buf[0]==0x53)
		{
			lmc_reday= 0;
   		lcm_int(2);			//初始化LCM全部次数为200
			user_parameter.hand_state = 0;
			osDelay(1000);
		}
		
	}


	
	osSignalWait(SIG_USER_TIMER, osWaitForever);	

	//按键输入切换开始/结束	
	if(user_parameter.hand_state == 1)
	{

		setpic((user_parameter.pic_id<5) ? user_parameter.pic_id+4:user_parameter.pic_id-4);
		osDelay(200);
		user_parameter.hand_state = 0;
	}	
	
	SendAndRecv(&user_parameter,300);	
	switch(user_parameter.pic_id)
	{
		case 0x00:		//首页
			
			Vacuum_Pump_Reset();
			setpic(2);
			//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_8);	
			break;
		
		case 0x01:		//自动模式，手套未工作
		case 0x02:
		case 0x03:

			user_parameter.current_times = 0x0000;
			Vacuum_Pump_Reset();	
			Vacuum_Valve_Reset();
			break;
		
		case 0x05:		//自动模式且手套已开始工作
		case 0x06:
		case 0x07:
			
			hd_fq = 4*(user_parameter.pic_id - 4);
			if(user_parameter.all_times == user_parameter.current_times)
			{
				Vacuum_Pump_Reset();
				setpic(user_parameter.pic_id-4);
			}
			if(user_parameter.all_times != user_parameter.current_times)
			{
				static int lcm_cnt=0;																				//lcm_cnt决定手套伸缩的频率  500ms为单位
				Vacuum_Pump_Set();	
				lcm_cnt++;
				if(lcm_cnt >= hd_fq)
				{
				lcm_cnt = 0;
				Vacuum_Valve_Toggle();
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET)
				{
					user_parameter.current_times++;
				}				
				}
			}		
			break;
			
		case 0x04:		//手动模式，手套未工作
			
			user_parameter.current_times = 0x0000;
			Vacuum_Pump_Reset();	
			Vacuum_Valve_Reset();
			break;
		
		case 0x08:		//手动模式
			Vacuum_Pump_Set();
			if(user_parameter.all_times == user_parameter.current_times)
			{
				Vacuum_Pump_Reset();
				setpic(user_parameter.pic_id-4);
			}
			if(user_parameter.all_times != user_parameter.current_times)
			{
				osEvent evt = osSignalWait(SIG_USER_2,1000);									//等待按键触发
				if (evt.status == osEventSignal)	
				{
					Vacuum_Valve_Toggle();
					if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET)
					{
					user_parameter.current_times++;
					}					
					user_parameter.hand_m = 0;
				}
			}	
			break;
			
		default:
			break;
	}
	}
	
}

int SendAndRecv(struct lcm_parameter_t *p,uint32_t millisec)
{
	uint8_t buf[64],data[32];
				//PIC_ID
				buf[0] = FRAME_HEAD_H;
				buf[1] = FRAME_HEAD_L;
				buf[2] = 0x03;
				buf[3] = RD_CTL_REG;
				buf[4] = PIC_ID;
				buf[5] = 0x02;	
				//ALL_TIMES:
				buf[6] = FRAME_HEAD_H;
				buf[7] = FRAME_HEAD_L;
				buf[8] = 0x04;
				buf[9] = RD_DATA_REG;
				buf[10] = (ALL_TIMES&0xff00)>>8;
				buf[11] = ALL_TIMES&0x00ff;	
				buf[12] = 0x01;
				//CURRENT_TIMES-1
				buf[13] = FRAME_HEAD_H;
				buf[14] = FRAME_HEAD_L;
				buf[15] = 0x05;
				buf[16] = WR_DATA_REG;
				buf[17] = (CURRENT_TIMES_RD1&0xff00)>>8;
				buf[18] = CURRENT_TIMES_RD1&0x00ff;	
				buf[19] = (p->current_times&0xff00)>>8;
				buf[20] = p->current_times&0x00ff;				
				//ALL_TIMES-2
				buf[21] = FRAME_HEAD_H;
				buf[22] = FRAME_HEAD_L;
				buf[23] = 0x05;
				buf[24] = WR_DATA_REG;
				buf[25] = (ALL_TIMES_RD1&0xff00)>>8;
				buf[26] = ALL_TIMES_RD1&0x00ff;	
				buf[27] = (p->all_times&0xff00)>>8;
				buf[28] = p->all_times&0x00ff;		

//					buf[12] = 0x00;
//					buf[13] = 0x01;
//					//CURRENT_TIMES-1
//					buf[14] = FRAME_HEAD_H;
//					buf[15] = FRAME_HEAD_L;
//					buf[16] = 0x05;
//					buf[17] = WR_DATA_REG;
//					buf[18] = (CURRENT_TIMES_RD1&0xff00)>>8;
//					buf[19] = CURRENT_TIMES_RD1&0x00ff;	
//					buf[20] = (p->current_times&0xff00)>>8;
//					buf[21] = p->current_times&0x00ff;				
//					//ALL_TIMES-2
//					buf[22] = FRAME_HEAD_H;
//					buf[23] = FRAME_HEAD_L;
//					buf[24] = 0x05;
//					buf[25] = WR_DATA_REG;
//					buf[26] = (ALL_TIMES_RD1&0xff00)>>8;
//					buf[27] = ALL_TIMES_RD1&0x00ff;	
//					buf[28] = (p->all_times&0xff00)>>8;
//					buf[29] = p->all_times&0x00ff;		
				memset(data,0,sizeof(data));
		int ret =	SendReqAndRecvResDataWithUart(4,buf,29,data,18,millisec);			
				
		if(ret)
		{
		LED3_Toggle();
		p->pic_id = data[7];
		p->all_times =100*(data[15]<<8|data[16]);//p->all_times =100*(data[16]<<8|data[17]);
		}
	return 0;
}      


void lcm_int(uint8_t times)
{
		uint8_t buf[32];
		buf[0] = FRAME_HEAD_H;
		buf[1] = FRAME_HEAD_L;
		buf[2] = 0x05;
		buf[3] = WR_DATA_REG;
		buf[4] = (ALL_TIMES&0xff00)>>8;
		buf[5] = ALL_TIMES&0x00ff;	
		buf[6] = 0x00;
		buf[7] = times;
		SendDataToUart(4,buf,8,100);
}


//设置当前显示图片
void setpic(uint8_t pic)
{
		uint8_t buf[16];	
		buf[0] = FRAME_HEAD_H;
		buf[1] = FRAME_HEAD_L;
		buf[2] = 0x04;
		buf[3] = WR_CTL_REG;
		buf[4] = PIC_ID;
		buf[5] = 0x00;
		buf[6] = pic;
		SendDataToUart(4,buf,7,100);
}



void crc16( unsigned char *ptr,unsigned int len)
{
  unsigned long wcrc=0XFFFF;
  unsigned char temp;
  int i=0,j=0;
  for(i=0;i<len;i++)
  {
    temp=*ptr&0X00FF;
    ptr++;
    wcrc^=temp;
    for(j=0;j<8;j++)
    {
      if(wcrc&0X0001)
      {
        wcrc>>=1;
        wcrc^=0XA001;
      }
      else
      {
        wcrc>>=1;
      }
    }
  }
  temp=wcrc;
	*(ptr+len) = wcrc;
	*(ptr+len+1) = wcrc>>8;
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//		switch(GPIO_Pin)	
//	{
//			case GPIO_PIN_1:
//				break;
//			case GPIO_PIN_4:
//				user_parameter.time_now[0] = HAL_GetTick();
//				if(user_parameter.time_now[0]-user_parameter.time_old[0]>2*DELAY_1S)
//				{
//					user_parameter.time_old[0] = user_parameter.time_now[0]; 
//					osSignalSet(tid_LCM_uart,SIG_USER_2);
//					user_parameter.hand_m = 1;
//				}			
//				break;	
//			case GPIO_PIN_10:
//				user_parameter.time_now[1] = HAL_GetTick();
//				if(user_parameter.time_now[1]-user_parameter.time_old[1]>2*DELAY_1S)
//				{					
//					osDelay(10);
//					if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10) == GPIO_PIN_RESET)
//					{
//					user_parameter.time_old[1] = user_parameter.time_now[1]; 
//					user_parameter.hand_state = 1;
//					}					
//				}
//				break;
//			case GPIO_PIN_11:
//				break;
//	}
//}




