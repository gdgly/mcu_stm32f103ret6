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

osMutexDef(lcm_mutex);
osMutexId lcm_mutex_id;

struct mutex_parameter_t mutex_parameter;
struct mutex_parameter_t mutex_parameter_temp;

extern TIM_HandleTypeDef htim1;

#define EVENT_LOOP_TIME_IN_MILLI_SECOND 250
#define DELAY_1S 1000
#define TIME_FOR_GAS_TO_FILL 10*DELAY_1S													//2.3s 对应   -75kpa - 120kpa
#define TIME_FOR_GAS_TO_EXTRACTION 10*DELAY_1S


osMailQId lcm_to_iot_mail;
osMailQDef(lcm_to_iot_mail,10,struct lcm_to_iot_t);

uint8_t pic_old;
uint8_t pic_mark = 0;
struct lcm_parameter_t user_parameter;
struct eeprom_parameter_t eeprom_parameter;
struct glove_adc_press_t glove_adc_press = {50, 65, 80, 87, 70, 40, 100};

int init_thread_of_LCM_uart (void) {

	tid_LCM_uart = osThreadCreate (osThread(thread_of_LCM_uart), NULL);
	if (!tid_LCM_uart) {
		return 0;
	}
  
	return 1;
}

uint32_t pic_switch_mark;

void thread_of_LCM_uart (void const *argument) {
	user_signal_info_t user_timer_info = { osThreadGetId(), SIG_USER_TIMER };
	osTimerDef(main_timer, SetUserSignal);
	osTimerId main_timer_id = osTimerCreate(osTimer(main_timer), osTimerPeriodic, &user_timer_info);
	osTimerStart(main_timer_id, EVENT_LOOP_TIME_IN_MILLI_SECOND);
  lcm_to_iot_mail = osMailCreate(osMailQ(lcm_to_iot_mail), NULL);
	
	
	lcm_mutex_id = osMutexCreate(osMutex(lcm_mutex));

	EEPROM_CheckOk();
	EEPROM_init();

	Lcd_init();
		
	for(;;)
	{
		osSignalWait(SIG_USER_TIMER, osWaitForever);
		lcd_read(&user_parameter,200);
		update_eeprom();
		check_start_stop_button();

		osMutexWait(lcm_mutex_id,osWaitForever);
		mutex_parameter_temp = mutex_parameter;
		osMutexRelease(lcm_mutex_id);				
		
		if(pic_old != user_parameter.common.pic_id){
			pic_switch_mark = HAL_GetTick();
		}

		switch(user_parameter.common.pic_id){
			case boot_animation_1:
				Vacuum_Pump_Reset();
				Vacuum_Valve_stop();
				setpic(boot_animation_2);
			break;

			case boot_animation_2:
				osDelay(500);		
				setpic(boot_animation_3);
			break;
			
			case boot_animation_3:
				osDelay(500);
				setpic(selection_interface);
			break;
			
			case selection_interface:
				
			break;
			
			case automatic_mode:
				if(pic_old != automatic_mode){
					clear_motion_cnt();
				}
			break;
				
			case manual_mode:
				if(pic_old != manual_mode){
					clear_motion_cnt();
				}
			break;
				
			case automatic_motion_1:
				if((pic_old != automatic_motion_1)&&(pic_old != automatic_motion_2)){
					update_usage_times();
				}
				motion_auto_mode(user_parameter.common.period,0);
			break;
				
			case automatic_motion_2:
				if((pic_old != automatic_motion_1)&&(pic_old != automatic_motion_2)){
					update_usage_times();
				}
				motion_auto_mode(user_parameter.common.period,1);
			break;
				
			case manual_motion_1:
				if((pic_old != manual_motion_1)&&(pic_old != manual_motion_2)){
					update_usage_times();
				}
				if(mutex_parameter_temp.mode == 0){
					motion_hand_mode(0);			
				}else if(mutex_parameter_temp.mode == 1){
					motion_gloves_mode();
				}
			break;
				
			case manual_motion_2:
				if((pic_old != manual_motion_1)&&(pic_old != manual_motion_2)){
					update_usage_times();
				}
				if(mutex_parameter_temp.mode == 0){
					motion_hand_mode(1);			
				}else if(mutex_parameter_temp.mode == 1){
					motion_gloves_mode();
				}		
			break;
				
			case calibration_animation_1:
			case calibration_animation_2:
			case calibration_animation_3:		
			case calibration_animation_4:
			case calibration_animation_5:
			case calibration_animation_6:	
				if(mutex_parameter_temp.mode == 0){
					setpic(gloves_not_connected);
				}else if(mutex_parameter_temp.mode == 1){
					motion_reset_mode();				
				}

			break;
			
			case gloves_not_connected:
				osDelay(3000);
				setpic(manual_mode);
			break;
				
			case calibration_failed:
				osDelay(1500);
				setpic(manual_mode);				
			break;
			
			case calibration_successed:
				osDelay(1500);
				setpic(manual_mode);				
			break;
			
			case motion_ending:
				motion_transition_mode1();
			break;
			
			case selection_interface_fake:
				motion_transition_mode2();
			break;
			
			case 0x30:
				air_pressure_init();
			break;
			
			case 0x31:
				restore_factory_setting();
			break;
			
			case 0x32:
				if(pic_old == 0x00){
					eeprom_glove_adc_press();
				}
				read_glove_adc_press();
			break;
				
			case 0x33:
				motion_gloves_mode();
			break;
			default:
			
			break;
		}
		send_to_iot();
		pic_old = user_parameter.common.pic_id;
	}
}

int lcd_read(struct lcm_parameter_t *p,uint32_t millisec)
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
//写入TREAMENT——TIME
	buf[21] = FRAME_HEAD_H;
	buf[22] = FRAME_HEAD_L;
	buf[23] = 0x05;
	buf[24] = WR_DATA_REG;
	buf[25] = (TREATMENT_TIME&0xff00)>>8;
	buf[26] = TREATMENT_TIME&0x00ff;	
	buf[27] = (p->treatment_time&0xff00)>>8;
	buf[28] = p->treatment_time&0x00ff;
  //读取周期	PERIOD的数据				返回[9]byte
	buf[29] = FRAME_HEAD_H;
	buf[30] = FRAME_HEAD_L;
	buf[31] = 0x04;
	buf[32] = RD_DATA_REG;
	buf[33] = (PERIOD&0XFF00)>>8;
	buf[34] = PERIOD&0X00FF;
	buf[35] = 0x02;								//读取1个word
	memset(data,0,sizeof(data));
	
	int ret =	SendReqAndRecvResDataWithUart(5,buf,36,data,28,millisec);			//发送43个byte 接受8+9+9=26个数据
	if(ret){
		p->common.pic_id = data[7];												//取当前图片编号8-1=7
		p->common.all_times = (data[15]<<8|data[16]);			//8+9-1=16
		p->common.period = (data[24]<<8|data[25]);					//8+9+9-1=25
		p->common.motion_time = (data[26]<<8|data[27]);
		return 1;
	}else{
		return 0;
	}
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
	SendDataToUart(5,buf,8,100);
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
	SendDataToUart(5,buf,7,100);
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

void EEPROM_init(void)
{
	EEPROM_ReadBytes(eeprom_parameter.data, 0, EEPROM_BUFF_SIZE);
		if((eeprom_parameter.data[0] == EEPROM_HEAD_S)&&(eeprom_parameter.data[1] == EEPROM_HEAD_Y)){
		eeprom_parameter.common.pic_id = eeprom_parameter.data[2];
		if(eeprom_parameter.common.pic_id < automatic_mode){
			eeprom_parameter.common.pic_id = automatic_mode;
		}
		eeprom_parameter.common.all_times = eeprom_parameter.data[3]<<8|eeprom_parameter.data[4];
		eeprom_parameter.common.all_times = 0xFFFF;
		eeprom_parameter.common.period = eeprom_parameter.data[5]<<8|eeprom_parameter.data[6];	
		eeprom_parameter.usage_times = eeprom_parameter.data[10]<<24|eeprom_parameter.data[9]<<16|eeprom_parameter.data[8]<<8|eeprom_parameter.data[7];
		eeprom_parameter.common.pos_pressure = eeprom_parameter.data[12]<<8|eeprom_parameter.data[13];
		eeprom_parameter.common.neg_pressure = eeprom_parameter.data[14]<<8|eeprom_parameter.data[15];
		eeprom_parameter.common.motion_time = eeprom_parameter.data[16]<<8|eeprom_parameter.data[17];
	}else{
		eeprom_parameter.buff[0] = EEPROM_HEAD_S; 
		eeprom_parameter.buff[1] = EEPROM_HEAD_Y;
		eeprom_parameter.buff[2] = 1;
		eeprom_parameter.buff[3] = 0;
		eeprom_parameter.buff[4] = 6;
		eeprom_parameter.buff[5] = 0;
		eeprom_parameter.buff[6] = 200;
		eeprom_parameter.common.pic_id = automatic_mode;
		eeprom_parameter.common.period = 6;
		eeprom_parameter.common.all_times = 200;
		eeprom_parameter.common.motion_time = 20;
		eeprom_parameter.usage_times = 0;
		eeprom_parameter.common.pos_pressure = 120;
		eeprom_parameter.common.neg_pressure = -70;
		EEPROM_WriteBytes(eeprom_parameter.buff, 0, 7);
	}
//	eeprom_glove_adc_press();
}

void Lcd_init(void)
{
	osDelay(1000);
	lcm_setvalue(PERIOD,eeprom_parameter.common.period);
	lcm_setvalue(ALL_TIMES,eeprom_parameter.common.all_times);
	lcm_setvalue(MOTION_TIME,eeprom_parameter.common.motion_time);
	lcm_setvalue(POS_PRESSURE,eeprom_parameter.common.pos_pressure);
	lcm_setvalue(NEG_PRESSURE,eeprom_parameter.common.neg_pressure);
	lcm_setvalue(ADDR_MECHINE_NUM1,mechine_num1);
	lcm_setvalue(ADDR_MECHINE_NUM2,mechine_num2);
	lcm_setvalue(ADDR_VERSION_NUM,version_num);
	osDelay(500);	
}

void check_start_stop_button(void)
{
	if(user_parameter.hand_state == 1){
		switch(user_parameter.common.pic_id)
		{
			case selection_interface: 
				if(pic_mark == 0){
					setpic(eeprom_parameter.common.pic_id);
				}else if((pic_mark == automatic_motion_1)||(pic_mark == automatic_motion_2)){
					setpic(automatic_mode);
				}else if((pic_mark == manual_motion_1)||(pic_mark == manual_motion_2)){
					setpic(manual_mode);
				}
			break;
			case automatic_mode: setpic(automatic_motion_1);break ;
			case manual_mode: setpic(manual_motion_1);break ;
			case automatic_motion_1: setpic(motion_ending);break ;
			case automatic_motion_2: setpic(motion_ending);break ;
			case manual_motion_1: setpic(motion_ending);break ;
			case manual_motion_2: setpic(motion_ending);break ;
			default : break ;
		}
		osDelay(50);
		user_parameter.hand_state = 0;
	}
}

void clear_motion_cnt(void)
{
	user_parameter.current_times = 0x0000;
	user_parameter.treatment_time = 0x0000;	
	Vacuum_Pump_Reset();	
	Vacuum_Valve_stop();
}

void update_usage_times(void)
{
	eeprom_parameter.usage_times++;
	eeprom_parameter.buff[0] = eeprom_parameter.usage_times&0xff;
	eeprom_parameter.buff[1] = eeprom_parameter.usage_times>>8&0xff;
	eeprom_parameter.buff[2] = eeprom_parameter.usage_times>>16&0xff;
	eeprom_parameter.buff[3] = eeprom_parameter.usage_times>>24&0xff;
	EEPROM_WriteBytes(eeprom_parameter.buff, 7, 4);	
}

uint32_t lcd_time_mark,motion_start_mark;
void motion_auto_mode(const uint8_t period, const uint8_t direction){
	
	if(pic_old != user_parameter.common.pic_id){
		lcd_time_mark = HAL_GetTick();
		if(pic_old == automatic_mode){
			motion_start_mark = HAL_GetTick();
		}
	}
	
	int pressure_now = get_air_pressure();
	
	set_valve(direction, pressure_now, lcd_time_mark);
	
	user_parameter.treatment_time =(HAL_GetTick() - motion_start_mark)/60/DELAY_1S;
		
	if((user_parameter.current_times >= user_parameter.common.all_times)||(user_parameter.treatment_time > user_parameter.common.motion_time)){
		Vacuum_Pump_Reset();
		setpic(motion_ending);		
	}

	if(user_parameter.common.all_times != user_parameter.current_times){		 
		if(HAL_GetTick() - lcd_time_mark > period*1000){			
			if(direction == 0){
				user_parameter.current_times++;				
				setpic(automatic_motion_2);
			}else if(direction == 1){
				setpic(automatic_motion_1);				
			}
		}
	}
}

void motion_hand_mode(const uint8_t direction)
{	
	if(pic_old != user_parameter.common.pic_id){
		lcd_time_mark = HAL_GetTick();
		if(pic_old == manual_mode){
			motion_start_mark = HAL_GetTick();
		}
	}
	
	int pressure_now = get_air_pressure();
	
	set_valve(direction, pressure_now, lcd_time_mark);
	
	user_parameter.treatment_time =(HAL_GetTick() - motion_start_mark)/60/DELAY_1S;
	
	if((user_parameter.current_times >= user_parameter.common.all_times)||(user_parameter.treatment_time > user_parameter.common.motion_time)){				
		Vacuum_Pump_Reset();
		setpic(motion_ending);
	}
		
	if(user_parameter.common.all_times != user_parameter.current_times){					
		osEvent evt = osSignalWait(SIG_USER_2,100);									
		if (evt.status == osEventSignal){											
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==GPIO_PIN_RESET){											
				if(direction == 0){
					user_parameter.current_times++;						
					setpic(manual_motion_2);
				}else if(direction == 1){
					setpic(manual_motion_1);
				}
			}					
		}
	}		
}


uint8_t curvature_old = 0,pre_curvature = 0,index_old =0;
uint8_t cur_index;
int is_curvature_work = 1;
int curvature_direction = 0;

void motion_gloves_mode(void)
{
	if(pic_old == manual_mode){
		motion_start_mark = HAL_GetTick();
	}
	
	if(curvature_old != mutex_parameter_temp.curvature){
		pre_curvature = curvature_old;
		is_curvature_work = 1;		
	}
	if(pre_curvature < mutex_parameter_temp.curvature){	
		curvature_direction = 1;
	}else if(pre_curvature > mutex_parameter_temp.curvature){
		if(mutex_parameter_temp.curvature < 2){
			curvature_direction = 0;		
		}else{
		  is_curvature_work = 0;	
		}
	}
	int pressure_now = get_air_pressure();
	int pressure_err = 0;
	cur_index = 0;
	
  if((mutex_parameter_temp.curvature == 2)||(mutex_parameter_temp.curvature == 4)){
		cur_index = pre_curvature;
	}else{
		cur_index = mutex_parameter_temp.curvature;
	}
	
	if(curvature_direction == 0){
		cur_index = 1;
	}
	
	switch(cur_index){
		case 1:
			if(index_old != cur_index){
				user_parameter.current_times++;	
			}
			pressure_err = -glove_adc_press.press1 - pressure_now;
		break;
		case 3:
			pressure_err = glove_adc_press.press2 - pressure_now;
		break;
		case 5:
			pressure_err = glove_adc_press.press3 - pressure_now;
		break;
		default:
			
		break;
	}
	index_old = cur_index;
	
	curvature_old = mutex_parameter_temp.curvature;
  set_valve_pid(pressure_err);

	user_parameter.treatment_time =(HAL_GetTick() - motion_start_mark)/60/DELAY_1S;
	if(user_parameter.treatment_time > user_parameter.common.motion_time){				
		Vacuum_Pump_Reset();
		setpic(motion_ending);
	}
}


void set_valve_pid(int err)
{
	if(is_curvature_work == 1){
		if(err > 15){
			Vacuum_Pump_Set(100);
			Vacuum_Valve_Set();
			setpic(manual_motion_2);
		}else if(err > 10){
			
		}else if(err > -10){
			Vacuum_Pump_Reset();
			is_curvature_work = 0;
		}else if(err > -15){
		
		}else{
			if((cur_index == 3)||(cur_index == 5)){
				Vacuum_Pump_Set(0);
			}else{
				Vacuum_Pump_Set(100);
				Vacuum_Valve_Reset();
				setpic(manual_motion_1);			
			}
		}	
	}else{
		Vacuum_Pump_Reset();
	}
}

extern uint8_t is_glove_reset_flag;
void motion_reset_mode(void)
{
	if((pic_old >= calibration_animation_1)&&(pic_old <= calibration_animation_6)){

	}else{
		pic_mark = pic_old;
		lcd_time_mark = HAL_GetTick();
		osMutexWait(lcm_mutex_id,osWaitForever);
		mutex_parameter.is_reset = 1;
		osMutexRelease(lcm_mutex_id);	
	}

	uint8_t npic = 0;
	npic = (HAL_GetTick() - lcd_time_mark)/1000;
	if(npic < 6){
		setpic(calibration_animation_1 + npic);			
	}
	
	if(HAL_GetTick() - lcd_time_mark > 6000){
		osMutexWait(lcm_mutex_id,osWaitForever);
		mutex_parameter.is_reset = 0;
		osMutexRelease(lcm_mutex_id);
		Vacuum_Pump_Reset();
		if(is_glove_reset_flag == 1){
			setpic(calibration_successed);		
		}else{
			setpic(calibration_failed);
		}
	}else{
		Vacuum_Pump_Set(100);
		Vacuum_Valve_Reset();
	}	
}

void motion_transition_mode1(void)
{
	if(pic_old != motion_ending){
		pic_mark = pic_old;
		lcd_time_mark = HAL_GetTick();
		Vacuum_Pump_Reset();
		Vacuum_Valve_stop();
	}
	
	if(HAL_GetTick() - lcd_time_mark > 3000){
		setpic(selection_interface);
	}
	

}

void motion_transition_mode2(void)
{
	if(pic_mark == 0){
		setpic(eeprom_parameter.common.pic_id);
	}else if((pic_mark == automatic_motion_1)||(pic_mark == automatic_motion_2)){
		setpic(automatic_mode);
	}else if((pic_mark == manual_motion_1)||(pic_mark == manual_motion_2)){
		setpic(manual_mode);
	}
}

int is_work = 0;
void set_valve(const uint8_t direction, const int pressure, const uint32_t mark)
{
	
	//0为正向 1为反向 2为双向
	
	if((eeprom_parameter.common.pos_pressure <= 80)||(eeprom_parameter.common.neg_pressure > -40)){
		eeprom_parameter.common.pos_pressure = 80;
		eeprom_parameter.common.neg_pressure = -40;
	}
	if(pressure > eeprom_parameter.common.pos_pressure){
		is_work = 0;
	}else if(pressure > eeprom_parameter.common.pos_pressure - 40){
				
	}else if(pressure > eeprom_parameter.common.neg_pressure + 40){
		is_work = 2;
	}else if(pressure > eeprom_parameter.common.neg_pressure){
			
	}else{
		is_work = 1;
	}

	if((direction == 0)){
		Vacuum_Pump_Set(100);
		Vacuum_Valve_Reset();
	}else if((direction == 1)){
		Vacuum_Pump_Set(100);
		Vacuum_Valve_Set();
	}else{
		Vacuum_Pump_Reset();
	}
}


void air_pressure_init(void)
{
	//
	uint8_t buf[32],data[32];
	buf[0] = FRAME_HEAD_H;
	buf[1] = FRAME_HEAD_L;
	buf[2] = 0x04;
	buf[3] = RD_DATA_REG;
	buf[4] = (POS_PRESSURE&0XFF00)>>8;
	buf[5] = POS_PRESSURE&0X00FF;
	buf[6] = 0x01;
	//
	buf[7] = FRAME_HEAD_H;
	buf[8] = FRAME_HEAD_L;
	buf[9] = 0x04;
	buf[10] = RD_DATA_REG;
	buf[11] = (NEG_PRESSURE&0XFF00)>>8;
	buf[12] = NEG_PRESSURE&0X00FF;
	buf[13] = 0x01;	
	
	int ret =	SendReqAndRecvResDataWithUart(5,buf,14,data,18,200);
	if(ret){
		eeprom_parameter.common.pos_pressure = (data[7]<<8|data[8]);
		eeprom_parameter.common.neg_pressure = (data[16]<<8|data[17]);	
		eeprom_parameter.buff[0] = eeprom_parameter.common.pos_pressure>>8&0xff;
		eeprom_parameter.buff[1] = eeprom_parameter.common.pos_pressure&0xff;
		eeprom_parameter.buff[2] = eeprom_parameter.common.neg_pressure>>8&0xff;
		eeprom_parameter.buff[3] = eeprom_parameter.common.neg_pressure&0xff;		
		EEPROM_WriteBytes(eeprom_parameter.buff, 12, 4);
	}	
}


extern uint32_t ADC_ConvertedValue[2];
int get_air_pressure(void)
{
	int pressure;
	float adc_temp;
	adc_temp = (float)(ADC_ConvertedValue[1]&0xFFF)*3.3/4096;
	pressure = (int)((adc_temp*1.51 - 1.5)*100);
	if(pressure < -100){
		return 0;
	}
	return pressure;
}

void update_eeprom(void)
{
	if((user_parameter.common.pic_id == automatic_mode)||(user_parameter.common.pic_id == manual_mode)){
		eeprom_parameter.buff[0] = EEPROM_HEAD_S; 
		eeprom_parameter.buff[1] = EEPROM_HEAD_Y;
		eeprom_parameter.buff[2] = user_parameter.common.pic_id;
		eeprom_parameter.buff[3] = user_parameter.common.all_times>>8&0xff;
		eeprom_parameter.buff[4] = user_parameter.common.all_times&0xff;
		eeprom_parameter.buff[5] = user_parameter.common.period>>8&0xff;
		eeprom_parameter.buff[6] = user_parameter.common.period&0xff;
		EEPROM_WriteBytes(eeprom_parameter.buff, 0, 7);
		eeprom_parameter.buff[0] = user_parameter.common.motion_time>>8&0xff;
		eeprom_parameter.buff[1] = user_parameter.common.motion_time&0xff;
		EEPROM_WriteBytes(eeprom_parameter.buff, 16, 2);
	}
}

int is_send_iot = 0;
void send_to_iot(void)
{

	if((user_parameter.current_times%2 == 0)&&(user_parameter.current_times != 0)){
		if(is_send_iot == 0){
			is_send_iot = 1;
			struct lcm_to_iot_t *s = osMailAlloc(lcm_to_iot_mail, 0);
			if(s){
				s->count = user_parameter.current_times;
				s->usage_times = eeprom_parameter.usage_times;
				if((user_parameter.common.pic_id == automatic_motion_1)||(user_parameter.common.pic_id == automatic_motion_2)){
					s->hand_state = 1;																	//自动模式状态标记
				}else if((user_parameter.common.pic_id == manual_motion_1)||(user_parameter.common.pic_id == manual_motion_2)){
					s->hand_state = 0;															    //手动模式状态标记
				}
				osMailPut(lcm_to_iot_mail,s);			
			}
		}
	}else{
		is_send_iot = 0;
	}
}

void sy02_set_valve(const uint8_t valve1, const uint8_t valve2)
{
	//set VCC_ULN1 CN4
	(valve1)?HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	//set VCC_ULN1 CN5
	(valve2)?HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void eeprom_glove_adc_press(void)
{
	uint8_t buf[64];
	int ret = EEPROM_ReadBytes(eeprom_parameter.data, 25, 14);
	if(ret){
		glove_adc_press.adc1_end = (eeprom_parameter.data[0]<<8|eeprom_parameter.data[1]);
		glove_adc_press.adc2_start = (eeprom_parameter.data[2]<<8|eeprom_parameter.data[3]);
		glove_adc_press.adc2_end = (eeprom_parameter.data[4]<<8|eeprom_parameter.data[5]);
		glove_adc_press.adc3_start = (eeprom_parameter.data[6]<<8|eeprom_parameter.data[7]);
		glove_adc_press.press1 = (eeprom_parameter.data[8]<<8|eeprom_parameter.data[9]);
		glove_adc_press.press2 = (eeprom_parameter.data[10]<<8|eeprom_parameter.data[11]);
		glove_adc_press.press3 = (eeprom_parameter.data[12]<<8|eeprom_parameter.data[13]);
	
		buf[0] = FRAME_HEAD_H;
		buf[1] = FRAME_HEAD_L;
		buf[2] = 0x11;
		buf[3] = WR_DATA_REG;
		buf[4] = (ADDR_ADC1_END&0xff00)>>8;
		buf[5] = ADDR_ADC1_END&0x00ff;
		memcpy(buf+6, eeprom_parameter.data, 14);
		
		SendDataToUart(5,buf,20,100);
	}	
}



void read_glove_adc_press(void)
{
	uint8_t buf[64],data[64];
	buf[0] = FRAME_HEAD_H;
	buf[1] = FRAME_HEAD_L;
	buf[2] = 0x04;
	buf[3] = RD_DATA_REG;
	buf[4] = (ADDR_ADC1_END&0XFF00)>>8;
	buf[5] = ADDR_ADC1_END&0X00FF;
	buf[6] = 0x07;								//读取7个word	
	
	memset(data,0,sizeof(data));
	int ret =	SendReqAndRecvResDataWithUart(5,buf,7,data,21,200);
	if(ret){
		glove_adc_press.adc1_end = (data[7]<<8|data[8]);
		glove_adc_press.adc2_start = (data[9]<<8|data[10]);
		glove_adc_press.adc2_end = (data[11]<<8|data[12]);
		glove_adc_press.adc3_start = (data[13]<<8|data[14]);
		glove_adc_press.press1 = (data[15]<<8|data[16]);
		glove_adc_press.press2 = (data[17]<<8|data[18]);
		glove_adc_press.press3 = (data[19]<<8|data[20]);	
		memcpy(eeprom_parameter.buff, data+7,14);
		EEPROM_WriteBytes(eeprom_parameter.buff, 25, 14);		
	}
}

void restore_factory_setting(void)
{
	eeprom_parameter.buff[0] = EEPROM_HEAD_S; 
	eeprom_parameter.buff[1] = EEPROM_HEAD_Y;
	eeprom_parameter.buff[2] = 1;
	eeprom_parameter.buff[3] = ALL_TIMES_DEFAULT>>8&0XFF;
	eeprom_parameter.buff[4] = ALL_TIMES_DEFAULT&0XFF;
	eeprom_parameter.buff[5] = PERIOD_DEFAULT>>8&0XFF;
	eeprom_parameter.buff[6] = PERIOD_DEFAULT&0XFF;
	eeprom_parameter.buff[7] = USAGE_TIMES_DEFAULT&0XFF;
	eeprom_parameter.buff[8] = USAGE_TIMES_DEFAULT>>8&0XFF;
	eeprom_parameter.buff[9] = USAGE_TIMES_DEFAULT>>16&0XFF;
	eeprom_parameter.buff[10] = USAGE_TIMES_DEFAULT>>24&0XFF;
	eeprom_parameter.buff[12] = POS_PRESSURE_DEFAULT>>8&0XFF;
	eeprom_parameter.buff[13] = POS_PRESSURE_DEFAULT&0XFF;
	eeprom_parameter.buff[14] = NEG_PRESSURE_DEFAULT>>8&0XFF;
	eeprom_parameter.buff[15] = NEG_PRESSURE_DEFAULT&0XFF;
	EEPROM_WriteBytes(eeprom_parameter.buff, 0, 16);
	osDelay(1000);
	EEPROM_init();
	osDelay(1000);
	Lcd_init();
	setpic(eeprom_parameter.common.pic_id);
}

void Vacuum_Pump_Set(uint8_t cycle)
{
	uint16_t pmw = 4096 - cycle*0.01*4096;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,pmw);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);			//PB0 控制继电器->控制泵

}
void Vacuum_Pump_Reset(void)
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,4096);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);			//PB0 控制继电器->控制泵
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)	
	{
			case GPIO_PIN_4:			

				break;
			case GPIO_PIN_3:
				user_parameter.time_now[0] = HAL_GetTick();					//获取当前时钟时间
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);								//清楚gpio—Pin4的外部中断寄存器IT
				if((user_parameter.time_now[0]-user_parameter.time_old[0]>2*DELAY_1S)&&(user_parameter.time_now[0] - pic_switch_mark > 0.5*DELAY_1S)){					
					int is_pin_cnt;
					for(is_pin_cnt=0;is_pin_cnt<10;is_pin_cnt++){
						osDelay(2);
						if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3) == GPIO_PIN_RESET){
								if(is_pin_cnt == 5){ 
									user_parameter.time_old[0] = user_parameter.time_now[0]; 		//更新时钟时间
									osSignalSet(tid_LCM_uart,SIG_USER_2);								
									break;
								}
						}
						else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3) == GPIO_PIN_SET){
							break;
						}					
					}		
				}				
			break;	
			case GPIO_PIN_15:					//PA12 开关按键
				
				user_parameter.time_now[1] = HAL_GetTick();
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);						//清楚gpio Pin4 外部中断寄存器IT
				if((user_parameter.time_now[1]-user_parameter.time_old[1]>2*DELAY_1S)&&(user_parameter.time_now[1] - pic_switch_mark > 0.2*DELAY_1S)){					
					osDelay(10);
					if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) == GPIO_PIN_RESET){							
							user_parameter.time_old[1] = user_parameter.time_now[1]; 
							user_parameter.hand_state = 1;
					}
				}				
			break;
	}
}




