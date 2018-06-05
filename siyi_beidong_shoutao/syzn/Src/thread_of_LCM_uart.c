#include "stm32f1xx_hal.h"
#include "misc.h"
#include "cmsis_os.h"
#include "thread_of_LCM_uart.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "string.h"
#include "bsp_EEPROM.h"

void thread_of_LCM_uart (void const *argument);                             // thread function
void crc16( unsigned char *ptr,unsigned int len);


osThreadId tid_LCM_uart;                                          // thread id
osThreadDef (thread_of_LCM_uart, osPriorityNormal, 1, 0);                   // thread object

#define EVENT_LOOP_TIME_IN_MILLI_SECOND 250
#define DELAY_1S 1000
#define time_reset_n 2.5
#define time_start_n 3
#define time_stop_n  2

struct lcm_parameter_t{
	uint8_t pic_id;
	uint16_t all_times;
	uint16_t current_times;
	uint16_t period;											//自动模式下的运行周期
	uint8_t hand_m;													//防止多次进入中断导致手动模式异常
	uint16_t reset_sys;										//复位量
	uint8_t hand_state;
	uint32_t time_now[4];
	uint32_t time_old[4];
}user_parameter;


struct eeprom_parameter_t{
	uint8_t data[64];
	uint8_t buff[64];
	uint8_t	pic_id;
	uint16_t all_times;
	uint16_t period;
	uint32_t usage_times;
}eeprom_parameter;

osMailQId lcm_to_iot_mail;
osMailQDef(lcm_to_iot_mail,10,struct lcm_to_iot_t);

uint8_t pic_old;
uint32_t pic_old_mark[2];									//增加pic_old and pic_old_mark 防止在图片切换时，由于关断泵导致的中断误触发
uint8_t reset_mark;
uint8_t reset_tmp[3]={1,1,1};
uint32_t time_sum[3];
 


int SendAndRecv(struct lcm_parameter_t *p,uint32_t millisec);
void setpic(uint8_t pic);
void lcm_int(uint8_t times);
void lcm_setvalue(uint16_t addr, uint16_t val );									
										
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
	reset_tmp[1]=1;
  lcm_to_iot_mail = osMailCreate(osMailQ(lcm_to_iot_mail), NULL);
	
	while(1){
		static int hd_fq = 0; //lmc_reday = 1,
				
	//lcm屏会在上电时发送  Serial screen to start successfully!  因此需要先将这段数据读取到才能进行后续的操作
		
		
		osSignalWait(SIG_USER_TIMER, osWaitForever);	

		
		//按键输入切换开始/结束	

		if(user_parameter.hand_state == 1){
			//setpic((user_parameter.pic_id <3)?user_parameter.pic_id+2:user_parameter.pic_id -2 );
			switch(user_parameter.pic_id)
			{
				case 0x02: setpic(4);break ;
				case 0x03: setpic(6);break ;
				case 0x04: setpic(9);break ;
				case 0x05: setpic(9);break ;
				case 0x06: setpic(9);break ;
				case 0x07: setpic(9);break ;
				default : break ;
			}
			osDelay(200);
			user_parameter.hand_state = 0;
		}	
	
		SendAndRecv(&user_parameter,200);	

		switch(user_parameter.pic_id)
		{
			case 0x00:		//首页
				
				EEPROM_CheckOk();			
				EEPROM_ReadBytes(eeprom_parameter.data, 0, EEPROM_BUFF_SIZE);
				if((eeprom_parameter.data[0] == EEPROM_HEAD_S)&&(eeprom_parameter.data[1] == EEPROM_HEAD_Y)){
					eeprom_parameter.pic_id = eeprom_parameter.data[2];
					eeprom_parameter.all_times = eeprom_parameter.data[3]<<8|eeprom_parameter.data[4];
					eeprom_parameter.period = eeprom_parameter.data[5]<<8|eeprom_parameter.data[6];	
					eeprom_parameter.usage_times = eeprom_parameter.data[10]<<24|eeprom_parameter.data[9]<<16|eeprom_parameter.data[8]<<8|eeprom_parameter.data[7];	
					
					lcm_setvalue(PERIOD,eeprom_parameter.period);			//进入图片首次设置初始化值，注意：只进入一次
					lcm_setvalue(ALL_TIMES,eeprom_parameter.all_times);	
					Vacuum_Pump_Reset();				//泵 复位停止工作 
					if(reset_tmp[1]){
						reset_tmp[1]=0;
						time_sum[1]=HAL_GetTick();
					}
					if(HAL_GetTick()-time_sum[1]>=time_start_n*DELAY_1S){
					setpic(eeprom_parameter.pic_id);
					}
			}
			else{
					lcm_setvalue(PERIOD,PERIOD_DEFAULT);			//进入图片首次设置初始化值，注意：只进入一次
					lcm_setvalue(ALL_TIMES,ALL_TIMES_DEFAULT);	
					Vacuum_Pump_Reset();				//泵 复位停止工作 
					setpic(1);								//转跳到第二张图片		
					if(reset_tmp[1]){						//首页停止3秒后，再切换到图片1
						 reset_tmp[1]=0;
						 time_sum[1] = HAL_GetTick(); 
					}
					if(HAL_GetTick()-time_sum[1]>=time_start_n*DELAY_1S ){
						setpic(1);			//停止3秒后，切换图片
					}					
				}		
			break;
			case 0x02:					//自动模式，未工作
					if(pic_old > 3){
						pic_old_mark[0] = HAL_GetTick();
						pic_old_mark[1] = HAL_GetTick();
					}				
					user_parameter.current_times = 0x0000;			//当前次数清零			
					Vacuum_Pump_Reset();	
					Vacuum_Valve_Reset();
					reset_mark = 2;							//自动复位标记
					reset_tmp[0]=1;
					reset_tmp[2]=1;
//					if((user_parameter.pic_id==0x01)&&(user_parameter.reset_sys == 1)){
//						//static uint32_t mark_now,mark_old;
//						//mark_now = HAL_GetTick();
//						uint8_t a_tmp;
//						if(a_tmp == 0){
//							user_parameter.time_old[2] = HAL_GetTick();
//							a_tmp =1;
//						}
//						if(HAL_GetTick()-user_parameter.time_old[2]>time_n*DELAY_1S){
//							Vacuum_Pump_Reset();
//							user_parameter.reset_sys = 0;
//							//user_parameter.time_old[2] = user_parameter.time_now[2];
//							lcm_setvalue(RESET_SYS,0);					//更新复位地址数据  0正常 1复位启动
//							a_tmp = 0;
//						}
//						else{
//							Vacuum_Valve_Set();						//伸展-吸气
//							Vacuum_Pump_Set();						//泵启动
//						}
//					}
//					else{	
//						  Vacuum_Pump_Reset();					//泵停止			
//					}
				break ;
			case 0x03:				 //手动模式，未工作
				
				if(pic_old > 4){				
					pic_old_mark[0] = HAL_GetTick();
					pic_old_mark[1] = HAL_GetTick();
				}
					
				user_parameter.current_times = 0x0000;
				Vacuum_Pump_Reset();		
				Vacuum_Valve_Reset();
				reset_mark =3;				//手动复位标记
				reset_tmp[0]=1;
				reset_tmp[2]=1;
//				if((user_parameter.pic_id==0x02)&&(user_parameter.reset_sys == 1)){
//						//static uint32_t mark_now,mark_old;
//						//mark_now = HAL_GetTick();
//						uint8_t b_tmp;
//						if(b_tmp == 0){
//							user_parameter.time_old[2] = HAL_GetTick();
//							b_tmp =1;
//						}
//						if(HAL_GetTick()-user_parameter.time_old[2]>time_n*DELAY_1S){
//							Vacuum_Pump_Reset();
//							user_parameter.reset_sys = 0;
//							//user_parameter.time_old[2] = user_parameter.time_now[2];
//							lcm_setvalue(RESET_SYS,0);					//更新复位地址数据  0正常 1复位启动
//							b_tmp = 0;
//						}
//						else{
//							Vacuum_Valve_Set();						//伸展-吸气
//							Vacuum_Pump_Set();						//泵启动
//						}
//					}
//					else{	
//						  Vacuum_Pump_Reset();					//泵停止			
//					}	
				break;
			case 0x04:				//自动模式，开始工作 伸展			
				if((pic_old != 0x05)&&(pic_old != 0x04)){
					HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);							//led3
					eeprom_parameter.usage_times++;
					eeprom_parameter.buff[0] = eeprom_parameter.usage_times&0xff;
					eeprom_parameter.buff[1] = eeprom_parameter.usage_times>>8&0xff;
					eeprom_parameter.buff[2] = eeprom_parameter.usage_times>>16&0xff;
					eeprom_parameter.buff[3] = eeprom_parameter.usage_times>>24&0xff;
					EEPROM_WriteBytes(eeprom_parameter.buff, 7, 4);	
				}		  
				Vacuum_Pump_Set();		//泵 工作
				Vacuum_Valve_Reset();
				if(user_parameter.current_times >= user_parameter.all_times){
					Vacuum_Pump_Reset();
					setpic(9);		
				}
				//hd_fq = 2*user_parameter.period;											//4*250ms = 1000ms 
				hd_fq = 4*user_parameter.period;												//250ms是程序运行时间周期 hd_fq = 1S * period
				if(user_parameter.all_times != user_parameter.current_times){
					//static int lcm_cnt=0,cnt_tmp = 0;
						static int lcm_cnt=0;		 
					  Vacuum_Pump_Set();	
					  lcm_cnt++;
						if(lcm_cnt>=hd_fq){
							lcm_cnt = 0;						
							//Vacuum_Valve_Reset();						
							setpic(5);
						//	user_parameter.current_times++;																			
						}
				}
				break;
			case 0x05:				//自动模式，开始工作 缩握
				if((pic_old != 0x05)&&(pic_old != 0x04)){
					HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);							//led3
					eeprom_parameter.usage_times++;
					eeprom_parameter.buff[0] = eeprom_parameter.usage_times&0xff;
					eeprom_parameter.buff[1] = eeprom_parameter.usage_times>>8&0xff;
					eeprom_parameter.buff[2] = eeprom_parameter.usage_times>>16&0xff;
					eeprom_parameter.buff[3] = eeprom_parameter.usage_times>>24&0xff;
					EEPROM_WriteBytes(eeprom_parameter.buff, 7, 4);	
				}
				Vacuum_Pump_Set();		//泵 工作
				 Vacuum_Valve_Set();	
				if(user_parameter.current_times >= user_parameter.all_times){
					Vacuum_Pump_Reset();
					setpic(9);		
				}
			  hd_fq = 4*user_parameter.period;
				if(user_parameter.all_times != user_parameter.current_times){
					//static int lcm_cnt=0,cnt_tmp = 0;
						static int lcm_cnt=0;		 
					  Vacuum_Pump_Set();					//泵工作
					  lcm_cnt++;
						if(lcm_cnt>=hd_fq){
							lcm_cnt = 0;						
							//Vacuum_Valve_Set();																						
							user_parameter.current_times++;			
							setpic(4);
						}
				}
				break;
				
			case 0x06:			//手动模式，开始工作 伸展		
					 if((pic_old != 0x07)&&(pic_old != 0X06)){												//数据实时存到e2Prom
							HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);							//led3
							eeprom_parameter.usage_times++;
							eeprom_parameter.buff[0] = eeprom_parameter.usage_times&0xff;
							eeprom_parameter.buff[1] = eeprom_parameter.usage_times>>8&0xff;
							eeprom_parameter.buff[2] = eeprom_parameter.usage_times>>16&0xff;
							eeprom_parameter.buff[3] = eeprom_parameter.usage_times>>24&0xff;
							EEPROM_WriteBytes(eeprom_parameter.buff, 7, 4);	
						}
			
						Vacuum_Pump_Set();
						Vacuum_Valve_Reset();	
						if(user_parameter.current_times >= user_parameter.all_times){					  //计满次数，转跳图片
								Vacuum_Pump_Reset();
								setpic(9);
						}

						if(user_parameter.all_times != user_parameter.current_times){					//按键检测
								osEvent evt = osSignalWait(SIG_USER_2,1000);									//等待按键触发
								if (evt.status == osEventSignal){
										//Vacuum_Valve_Reset();																	//手套伸展  -吸气
										setpic(7);
//						if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==GPIO_PIN_RESET){				//PA4 					
//								user_parameter.current_times++;
//							
//							}					
								user_parameter.hand_m = 0;
						}
				}	
					break;
				
				case 0x07:		  //手动模式，开始工作 缩握
						 if((pic_old != 0x07)&&(pic_old != 0x05)){
								HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);							//led3
								eeprom_parameter.usage_times++;
								eeprom_parameter.buff[0] = eeprom_parameter.usage_times&0xff;
								eeprom_parameter.buff[1] = eeprom_parameter.usage_times>>8&0xff;
								eeprom_parameter.buff[2] = eeprom_parameter.usage_times>>16&0xff;
								eeprom_parameter.buff[3] = eeprom_parameter.usage_times>>24&0xff;
								EEPROM_WriteBytes(eeprom_parameter.buff, 7, 4);	
						}
						Vacuum_Pump_Set();
						Vacuum_Valve_Set();
						if(user_parameter.current_times >= user_parameter.all_times){					  //计满次数，转跳图片
							Vacuum_Pump_Reset();
							setpic(9);
						}
							
						if(user_parameter.all_times != user_parameter.current_times){					//按键检测
								osEvent evt = osSignalWait(SIG_USER_2,1000);									//等待按键触发
								if (evt.status == osEventSignal){
									//Vacuum_Valve_Set();														//手套缩握 -吹气
									if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==GPIO_PIN_RESET){				//PA4 								
									user_parameter.current_times++;		
									setpic(6);
									}					
								user_parameter.hand_m = 0;
								}
						}	
					break;
				case 0x08:
					
					  if(reset_tmp[0]){
							reset_tmp[0]=0;
							time_sum[0] = HAL_GetTick();
							
					  }			
				    if(HAL_GetTick()-time_sum[0]>=time_reset_n*DELAY_1S ){
								Vacuum_Pump_Reset();
															//reset_mark为图片标志位
								setpic(reset_mark);
						 }
						else{
							Vacuum_Pump_Set();
							Vacuum_Valve_Reset();				//手套伸展
						}
				
				break;
				case 0x09:
					if(reset_tmp[2]){
						 reset_tmp[2]=0;
						 time_sum[2] = HAL_GetTick(); 
					}
					if(HAL_GetTick()-time_sum[2]>=time_stop_n*DELAY_1S ){
						setpic(reset_mark);			//停止3秒后，切换图片
					}
				  
					Vacuum_Pump_Reset();
					Vacuum_Valve_Reset();
				break;
			  default:
				break;
		}
		pic_old = user_parameter.pic_id;
	}
}

int SendAndRecv(struct lcm_parameter_t *p,uint32_t millisec)
{
	uint8_t buf[64],data[36];
	//读取当前照片 PIC_ID		返回2个byte 例如：0X0000，最后2个是图片编号  [8]byte
	buf[0] = FRAME_HEAD_H;
	buf[1] = FRAME_HEAD_L;
	buf[2] = 0x03;
	buf[3] = RD_CTL_REG;
	buf[4] = PIC_ID;
	buf[5] = 0x02;			//2个word
	//	读取设置的总次数ALL_TIMES:		返回9个byte，最后2个字节是all——times的值	[9]byte
	buf[6] = FRAME_HEAD_H;
	buf[7] = FRAME_HEAD_L;
	buf[8] = 0x04;					//数据长度4个byte
	buf[9] = RD_DATA_REG;
	buf[10] = (ALL_TIMES&0xff00)>>8;
	buf[11] = ALL_TIMES&0x00ff;	
	buf[12] = 0x01;					//读取1个word			
	//写入CURRENT_TIMES-1
	buf[13] = FRAME_HEAD_H;
	buf[14] = FRAME_HEAD_L;
	buf[15] = 0x05;
	buf[16] = WR_DATA_REG;
	buf[17] = (CURRENT_TIMES_RD1&0xff00)>>8;
	buf[18] = CURRENT_TIMES_RD1&0x00ff;	
	buf[19] = (p->current_times&0xff00)>>8;
	buf[20] = p->current_times&0x00ff;		
  //读取周期	PERIOD的数据				返回[9]byte
	buf[29] = FRAME_HEAD_H;
	buf[30] = FRAME_HEAD_L;
	buf[31] = 0x04;
	buf[32] = RD_DATA_REG;
	buf[33] = (PERIOD&0XFF00)>>8;
	buf[34] = PERIOD&0X00FF;
	buf[35] = 0x01;								//读取1个word
//	//	读取设置的复位地址量RESET_SYSY :		返回9个byte，最后2个字节是reset_sys的值	[9] byte
//	buf[36] = FRAME_HEAD_H;
//	buf[37] = FRAME_HEAD_L;
//	buf[38] = 0x04;					//数据长度4个byte
//	buf[39] = RD_DATA_REG;
//	buf[40] = (RESET_SYS&0xff00)>>8;
//	buf[41] = RESET_SYS&0x00ff;	
//	buf[42] = 0x01;					//读取1个word		
	memset(data,0,sizeof(data));
	
	int ret =	SendReqAndRecvResDataWithUart(4,buf,36,data,26,millisec);			//发送43个byte 接受8+9+9=26个数据
	if(ret){
		p->pic_id = data[7];												//取当前图片编号8-1=7
		p->all_times = (data[15]<<8|data[16]);			//8+9-1=16
		//	p->all_times =100*(data[16]<<8|data[17]);		//取运行总次数的值
		p->period = (data[24]<<8|data[25]);					//8+9+9-1=25
	//	p->reset_sys = (data[33]<<8|data[34]);			//8+9+9+9-1=34
		if((p->pic_id == 1)||(p->pic_id == 2)){
			
			eeprom_parameter.buff[0] = EEPROM_HEAD_S; 
			eeprom_parameter.buff[1] = EEPROM_HEAD_Y;
			eeprom_parameter.buff[2] = p->pic_id;
			eeprom_parameter.buff[3] = data[15];
			eeprom_parameter.buff[4] = data[16];
			eeprom_parameter.buff[5] = data[24];
			eeprom_parameter.buff[6] = data[25];

			EEPROM_WriteBytes(eeprom_parameter.buff, 0, 7);	
		}
		
		static int is_send_iot = 0;
		
		if((p->current_times%2 == 0)&&(p->current_times != 0)){
			if(is_send_iot == 0){
				is_send_iot = 1;
				struct lcm_to_iot_t *s = osMailAlloc(lcm_to_iot_mail, 0);			
				if(s){
					s->count = p->current_times;
					s->usage_times = eeprom_parameter.usage_times;
					if((p->pic_id == 1)||(p->pic_id == 3)||(p->pic_id == 4)){
						s->hand_state = 1;																	//自动模式状态标记
					}else if((p->pic_id == 2)||(p->pic_id == 5)||(p->pic_id == 6)){
						s->hand_state = 0;															    //手动模式状态标记
					}
					osMailPut(lcm_to_iot_mail,s);
				}
			}
		}else{
			is_send_iot = 0;
		}
	}else{
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
//写入当前地址变量值
void lcm_setvalue(uint16_t addr, uint16_t val)
{
	uint8_t buf[32];
	buf[0] = FRAME_HEAD_H;
	buf[1] = FRAME_HEAD_L;
	buf[2] = 0x05;
	buf[3] = WR_DATA_REG;
	buf[4] = (addr&0xff00)>>8;
	buf[5] = addr&0x00ff;
	buf[6] = (val&0xff00)>>8;
	buf[7] = val&0x00ff;
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)	
	{
			case GPIO_PIN_1:			

				break;
			case GPIO_PIN_4:
				user_parameter.time_now[0] = HAL_GetTick();					//获取当前时钟时间
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);								//清楚gpio—Pin4的外部中断寄存器IT
//				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);							//led3
				if((user_parameter.time_now[0]-user_parameter.time_old[0]>2*DELAY_1S)&&(user_parameter.time_now[0] - pic_old_mark[0] > 500))
				{					
					int is_pin_cnt;
					for(is_pin_cnt=0;is_pin_cnt<10;is_pin_cnt++)
					{
						osDelay(2);
						if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == GPIO_PIN_RESET)
						{
								if(is_pin_cnt == 5)
								{ 
									user_parameter.time_old[0] = user_parameter.time_now[0]; 		//更新时钟时间
									osSignalSet(tid_LCM_uart,SIG_USER_2);								
									user_parameter.hand_m = 1;
									break;
								}
						}
						else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == GPIO_PIN_SET)
						{
							break;
						}					
					}		
				}				
				break;	
//				case GPIO_PIN_2:
//					user_parameter.time_now[1] = HAL_GetTick();
//					__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);						//清楚gpio Pin4 外部中断寄存器IT
//					HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);					//led2 
//					if(user_parameter.time_now[1]-user_parameter.time_old[1]>2*DELAY_1S)
//					{					
//						osDelay(10);
////						if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10) == GPIO_PIN_RESET)
////						{
////						osDelay(10);
//						if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2) == GPIO_PIN_RESET)
//							{							
//								user_parameter.time_old[1] = user_parameter.time_now[1]; 
//								user_parameter.hand_state = 1;
//							}
//						}				
//								
//					}
//				break;
			case GPIO_PIN_12:					//PA12 开关按键
				
					user_parameter.time_now[1] = HAL_GetTick();
					__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);						//清楚gpio Pin4 外部中断寄存器IT
					HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);					//led2 
					if((user_parameter.time_now[1]-user_parameter.time_old[1]>0.1*DELAY_1S)&&(user_parameter.time_now[1] - pic_old_mark[1] > 500))
					{					
						osDelay(10);
//						if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10) == GPIO_PIN_RESET)
//						{
//						osDelay(10);
						if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12) == GPIO_PIN_RESET)
							{							
								user_parameter.time_old[1] = user_parameter.time_now[1]; 
								user_parameter.hand_state = 1;
							}
						}				
				break;
	}
}




