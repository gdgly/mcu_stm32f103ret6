#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "IOI2C.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "thread_of_LCM_uart.h"
#include "device_ctrl.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void thread_of_lcm_uart (void const *argument);                             // thread function
osThreadId tid_thread_of_lcm_uart;                                          // thread id
osThreadDef (thread_of_lcm_uart, osPriorityNormal, 1, 0);                   // thread object

#define EVENT_LOOP_TIME_IN_MILLI_SECOND 100

uint16_t ctrl_register_list[32];
uint16_t data_register_list[96] = LCM_DATA_REGISTER_DEFUALTS;

uint8_t pic_old;
struct lcm_parameter_t user_parameter;
struct lcm_uart_len_t lcm_uart_len = {0 , 0};

#define DELAY_1S 1000

uint8_t lcm_uart_send_buff[128];
uint8_t *lcm_uart_send_pointer = lcm_uart_send_buff;


uint8_t lcm_ctrl_register[][2] = {
  {0x01, 0x01},
  {0x03, 0x02},
};

uint16_t lcm_data_register[] = {
	0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30
};

int init_thread_of_lcm_uart (void) {

  tid_thread_of_lcm_uart = osThreadCreate (osThread(thread_of_lcm_uart), NULL);
  if (!tid_thread_of_lcm_uart) return(-1);
  
  return(0);
}



void thread_of_lcm_uart (void const *argument) {
	user_signal_info_t user_timer_info = { osThreadGetId(), SIG_USER_TIMER };
	osTimerDef(main_timer, SetUserSignal);
	osTimerId main_timer_id = osTimerCreate(osTimer(main_timer), osTimerPeriodic, &user_timer_info);
	osTimerStart(main_timer_id, EVENT_LOOP_TIME_IN_MILLI_SECOND);
  while (1) {
		for(;;){
			osSignalWait(SIG_USER_TIMER, osWaitForever);
			lcm_uart_len.rx_len = 0;
			lcm_uart_len.tx_len = 0;
			read_ctrl_register(1, lcm_uart_send_pointer, &lcm_uart_len);
			read_ctrl_register(0, lcm_uart_send_pointer, &lcm_uart_len);
			
			switch(ctrl_register_list[pic_id]){
			case boot_animation_1:
				setpic(boot_animation_2);
			break;

			case boot_animation_2:
				osDelay(500);		
			  write_data_registers(length_of_training, lcm_uart_send_pointer, 22, &data_register_list[length_of_training], &lcm_uart_len);
				setpic(boot_animation_3);
			break;
			
			case boot_animation_3:
				osDelay(500);
				setpic(selection_interface);
			break;
			
			case selection_interface:
				if(pic_old != selection_interface){
					data_register_list[start_and_stop] = 0;
					write_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &data_register_list[start_and_stop], &lcm_uart_len);						
				}
				sy08_set_valve(0);
				sy08_pump_reset();	
			break;
			
			case flexion_and_extension_interface:
				if(pic_old != flexion_and_extension_interface){
				
				}
				finger_flexion_and_extension();
			break;
			
			case gloves_exercises_interface:
				function_gloves_exercises();
			break;
			
			case settings_interface:
			  read_data_registers(length_of_training, lcm_uart_send_pointer, 22, &lcm_uart_len);
			  if(data_register_list[flexion_and_extension_all] == 1){
					int i;
					for(i=0;i<5;i++){
						data_register_list[flexion_and_extension_thumb + i] = 1;
					}
					write_data_registers(flexion_and_extension_thumb, lcm_uart_send_pointer, 5, &data_register_list[flexion_and_extension_thumb], &lcm_uart_len);
				}
			break;
			
			case motion_ending:
				data_register_list[motion_time] = 0;
			  data_register_list[motion_cnt] = 0;
			  write_data_registers(motion_time, lcm_uart_send_pointer, 2, &data_register_list[motion_time], &lcm_uart_len);
				sy08_set_valve(0);
				sy08_pump_reset();			
			break;
				
			}
			pic_old = ctrl_register_list[pic_id];
			lcd_read(&user_parameter,80);
		}
  }
}

uint32_t lcd_time_mark,motion_start_mark,training_time_mark;
uint8_t gloves_direction;												//1 for 握拳   ， 1 for 张开
void finger_flexion_and_extension(void)
{
	uint8_t valve;
	if(pic_old != ctrl_register_list[pic_id]){
		lcd_time_mark = HAL_GetTick();
		motion_start_mark = HAL_GetTick();
		training_time_mark = 0;
		sy08_pump_set(60 + 10*data_register_list[training_strength]);
		valve = ((data_register_list[flexion_and_extension_little_thumb]&VALVE_SET)<<4)| 	\
						((data_register_list[flexion_and_extension_ring_finger]&VALVE_SET)<<3)| 		\
						((data_register_list[flexion_and_extension_middle_finger]&VALVE_SET)<<2)| 	\
						((data_register_list[flexion_and_extension_index_finger]&VALVE_SET)<<1)| 	\
						(data_register_list[flexion_and_extension_thumb]&VALVE_SET);
		sy08_set_valve(valve);
		gloves_direction = 1;
		data_register_list[flexion_and_extension_icon] = 0;
		write_data_registers(flexion_and_extension_icon, lcm_uart_send_pointer, 1, data_register_list, &lcm_uart_len);
		read_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &lcm_uart_len);
	}else{
		if(!data_register_list[start_and_stop]){
			
			training_time_mark = HAL_GetTick() - motion_start_mark;
			data_register_list[motion_time] = training_time_mark/(60*DELAY_1S);
			if(data_register_list[motion_time] > data_register_list[length_of_training]){
				setpic(motion_ending);
				return ;
			}
			write_data_registers(motion_time, lcm_uart_send_pointer, 1, &data_register_list[motion_time], &lcm_uart_len);
			
			sy08_pump_set(60 + 10*data_register_list[training_strength]);
			if(((HAL_GetTick() - lcd_time_mark) > data_register_list[training_interval]*DELAY_1S)&&gloves_direction == 1){
				lcd_time_mark = HAL_GetTick();
				
				valve = ((data_register_list[flexion_and_extension_little_thumb]&VALVE_RESET)<<4)| 	\
								((data_register_list[flexion_and_extension_ring_finger]&VALVE_RESET)<<3)| 		\
								((data_register_list[flexion_and_extension_middle_finger]&VALVE_RESET)<<2)| 	\
								((data_register_list[flexion_and_extension_index_finger]&VALVE_RESET)<<1)| 	\
								(data_register_list[flexion_and_extension_thumb]&VALVE_RESET);
				sy08_set_valve(valve);
				gloves_direction = 0;
				data_register_list[motion_cnt]++;
				write_data_registers(motion_cnt, lcm_uart_send_pointer, 1, &data_register_list[motion_cnt], &lcm_uart_len);
				data_register_list[flexion_and_extension_icon] = 1;
				write_data_registers(flexion_and_extension_icon, lcm_uart_send_pointer, 1, &data_register_list[flexion_and_extension_icon], &lcm_uart_len);	
			}

			if(((HAL_GetTick() - lcd_time_mark) > data_register_list[training_interval]*DELAY_1S)&&gloves_direction == 0){
				lcd_time_mark = HAL_GetTick();
				valve = ((data_register_list[flexion_and_extension_little_thumb]&VALVE_SET)<<4)| 	\
								((data_register_list[flexion_and_extension_ring_finger]&VALVE_SET)<<3)| 		\
								((data_register_list[flexion_and_extension_middle_finger]&VALVE_SET)<<2)| 	\
								((data_register_list[flexion_and_extension_index_finger]&VALVE_SET)<<1)| 	\
								(data_register_list[flexion_and_extension_thumb]&VALVE_SET);
				sy08_set_valve(valve);
				gloves_direction = 1;
				data_register_list[flexion_and_extension_icon] = 0;
				write_data_registers(flexion_and_extension_icon, lcm_uart_send_pointer, 1, &data_register_list[flexion_and_extension_icon], &lcm_uart_len);
			}
		}else{
			motion_start_mark = HAL_GetTick() - training_time_mark;
			sy08_pump_reset();
			lcd_time_mark = HAL_GetTick();
		}
		read_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &lcm_uart_len);
	}
}

uint8_t exercises_training_list[5];
uint8_t exercises_list_cnt = 0;
void function_gloves_exercises(void)
{
	uint8_t valve;
	if(pic_old != ctrl_register_list[pic_id]){
		lcd_time_mark = HAL_GetTick() - data_register_list[training_interval]*DELAY_1S;
		motion_start_mark = HAL_GetTick();
		training_time_mark = 0;
		
		int i;
		for(i=0;i<5;i++){
			exercises_training_list[i] = data_register_list[gloves_exercises_index_finger + i];
		}
		read_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &lcm_uart_len);
		gloves_direction = 0;
		exercises_list_cnt = 0;
	}else{
		if(!data_register_list[start_and_stop]){
			
			training_time_mark = HAL_GetTick() - motion_start_mark;
			data_register_list[motion_time] = training_time_mark/(60*DELAY_1S);
			if(data_register_list[motion_time] > data_register_list[length_of_training]){
				setpic(motion_ending);
				return ;
			}
			write_data_registers(motion_time, lcm_uart_send_pointer, 1, &data_register_list[motion_time], &lcm_uart_len);
			
			
			sy08_pump_set(60 + 10*data_register_list[training_strength]);
			if(exercises_training_list[exercises_list_cnt]){
				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[training_interval]*DELAY_1S)&&gloves_direction == 0){
					lcd_time_mark = HAL_GetTick();
					
					valve = ((((exercises_list_cnt==3)||(exercises_list_cnt==4))?VALVE_SET:VALVE_RESET)<<4)| 	\
									((((exercises_list_cnt==2)||(exercises_list_cnt==4))?VALVE_SET:VALVE_RESET)<<3)| 	\
									((((exercises_list_cnt==1)||(exercises_list_cnt==4))?VALVE_SET:VALVE_RESET)<<2)| 	\
									((((exercises_list_cnt==0)||(exercises_list_cnt==4))?VALVE_SET:VALVE_RESET)<<1)| 	\
									(VALVE_SET);
					sy08_set_valve(valve);
					gloves_direction = 1;	
					data_register_list[gloves_exercises_icon] = exercises_list_cnt;
					write_data_registers(gloves_exercises_icon, lcm_uart_send_pointer, 1, &data_register_list[gloves_exercises_icon], &lcm_uart_len);
				}
				
				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[training_interval]*DELAY_1S)&&gloves_direction == 1){
					lcd_time_mark = HAL_GetTick();
					
					exercises_list_cnt++;
					if(exercises_list_cnt == 5){
						exercises_list_cnt = 0;
					}			
					
					valve = (VALVE_RESET<<4)| 	\
									(VALVE_RESET<<3)| 	\
									(VALVE_RESET<<2)| 	\
									(VALVE_RESET<<1)| 	\
									(VALVE_RESET);
					sy08_set_valve(valve);
					gloves_direction = 0;
					
					data_register_list[motion_cnt]++;
					write_data_registers(motion_cnt, lcm_uart_send_pointer, 1, &data_register_list[motion_cnt], &lcm_uart_len);					
					data_register_list[gloves_exercises_icon] = 0x05;
					write_data_registers(gloves_exercises_icon, lcm_uart_send_pointer, 1, &data_register_list[gloves_exercises_icon], &lcm_uart_len);
				}
			}else{
				exercises_list_cnt++;
				if(exercises_list_cnt == 5){
					exercises_list_cnt = 0;
				}
			}			
		}else{
			motion_start_mark = HAL_GetTick() - training_time_mark;
			sy08_pump_reset();
			lcd_time_mark = HAL_GetTick();		
		}
		read_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &lcm_uart_len);
	}
}


uint16_t *nctrl_register = ctrl_register_list;
uint16_t *ndata_register = data_register_list;

int lcd_read(struct lcm_parameter_t *p,uint32_t millisec)
{
	uint8_t data[64];
	uint8_t *nrx = data;
	
//	write_data_registers(0, s, 5, nwrite, &t);
//	read_data_registers(0, lcm_uart_send_pointer, 30, &lcm_uart_len);
	memset(data,0,sizeof(data));
	int ret =	SendReqAndRecvResDataWithUart(3,lcm_uart_send_buff,lcm_uart_len.tx_len,data,lcm_uart_len.rx_len,millisec);
	if(ret){
		while(nrx < data + lcm_uart_len.rx_len){
			while((*nrx != FRAME_HEAD_H)&&(*(nrx + 1) != FRAME_HEAD_L)){
				nrx++;
			}
			nrx+=2;	
			nrx++;
			if(*nrx == RD_CTL_REG){
				nrx++;
				memset(nctrl_register+*nrx,0,1+((*(nrx+1)-1)/2));																						//将相应的地址数据写0
				int k;
				for(k=0;k<*(nrx + 1);k++){
					*(nctrl_register + *nrx + k/2) = *(nrx + k + 2)|*(nctrl_register + *nrx+ k/2)<<8;					//将结果写入寄存器
				}
			}
			
			if(*nrx == RD_DATA_REG){
				uint16_t j = *(nrx+1)<<8|*(nrx+2);
				nrx+=3;
				uint8_t z = *nrx;
				nrx++;
				int u;
				for(u=0;u<z;u++){
					*(ndata_register + u + j) = *nrx<<8|*(nrx+1);
					nrx+=2;
				}
			}
		}
		p->common.pic_id = ctrl_register_list[pic_id];
		return 1;
	}
	return 0;
}

//设置当前显示图片
void setpic(uint16_t pic)
{
	uint8_t datagram[16];	
	struct lcm_uart_len_t t = {0,0};
	write_ctrl_register(1, datagram, &pic, &t);
	SendDataToUart(3,datagram,t.tx_len,80);
}

//写控制寄存器
//uint8_t list      : 寄存器地址
//uint8_t *p        : 串口发送 数组 指针
//uint16_t *value   : 写入 数组 指针
//struct lcm_uart_len_t *t  :  发送长度 接收长度
void write_ctrl_register(const uint8_t list, uint8_t *p, uint16_t *value, struct lcm_uart_len_t *t)
{
	uint8_t *s = p + t->tx_len;
	*s++ = FRAME_HEAD_H;
	*s++ = FRAME_HEAD_L;
	*s++ = 0x02 + lcm_ctrl_register[list][REGISTER_LEN];
	*s++ = WR_CTL_REG;	
	*s++ = lcm_ctrl_register[list][REGISTER_ADDR]&0x00ff;
	if(lcm_ctrl_register[list][REGISTER_LEN] == 1){
		*s++ = *value&0x00ff;
	}else if(lcm_ctrl_register[list][REGISTER_LEN] == 2){
		*s++ = (*value&0xff00)>>8;
		*s++ = *value&0x00ff;
	}
	t->tx_len = s - p;
	t->rx_len += 0;
}

//读控制寄存器
//uint8_t list      : 寄存器地址
//uint8_t *p        : 串口发送 数组 指针
//struct lcm_uart_len_t *t  :  发送长度 接收长度
void read_ctrl_register(const uint8_t list, uint8_t *p, struct lcm_uart_len_t *t)
{
	uint8_t *s = p + t->tx_len;
	*s++ = FRAME_HEAD_H;
	*s++ = FRAME_HEAD_L;
	*s++ = 0x03;
	*s++ = RD_CTL_REG;
	*s++ = lcm_ctrl_register[list][REGISTER_ADDR]&0x00ff;
	*s++ = lcm_ctrl_register[list][REGISTER_LEN]&0x00ff;
	t->tx_len = s - p;
	t->rx_len += 6 + lcm_ctrl_register[list][REGISTER_LEN];
	return;
}


//写数据寄存器，可以写多个
//uint8_t list      : 寄存器地址
//uint8_t *p        : 串口发送 数组 指针
//uint8_t len       ：写寄存器的长度
//uint16_t *value   : 写入 数组 指针
//struct lcm_uart_len_t *t  :  发送长度 接收长度
void write_data_registers(const uint8_t list, uint8_t *p, const uint8_t len, uint16_t *value, struct lcm_uart_len_t *t)
{
	uint8_t *s = p + t->tx_len;
	*s++ = FRAME_HEAD_H;
	*s++ = FRAME_HEAD_L;
	*s++ = 0x03 + 2*len;
	*s++ = WR_DATA_REG;
	*s++ = (lcm_data_register[list]&0xff00)>>8;
	*s++ = lcm_data_register[list]&0x00ff;
	int i;
	for(i=0;i<len;i++){
		*s++ = (*(value+i)&0xff00)>>8;
		*s++ = *(value+i)&0x00ff;
	}
	t->tx_len = s - p;
//	t->rx_len += 0;
}

//读寄存器， 可以读多个
//uint8_t list      : 寄存器地址
//uint8_t *p        : 串口发送 数组 指针
//uint8_t len       ：读寄存器的长度
//struct lcm_uart_len_t *t  :  发送长度 接收长度
void read_data_registers(const uint8_t list, uint8_t *p, const uint8_t len, struct lcm_uart_len_t *t)
{
	uint8_t *s = p + t->tx_len;
	*s++ = FRAME_HEAD_H;
	*s++ = FRAME_HEAD_L;
	*s++ = 0x04;
	*s++ = RD_DATA_REG;	
	*s++ = (lcm_data_register[list]&0xff00)>>8;
	*s++ = lcm_data_register[list]&0x00ff;
	*s++ = len;
	t->tx_len = s - p;
	t->rx_len += 7 + 2*len;
}

uint8_t read_panel_buttons(void)
{
	uint8_t buttons;
	buttons = (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)<<2)|(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)<<1)|HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7);
	return buttons;
}

void read_data_register(const uint8_t list, uint8_t *p, struct lcm_uart_len_t *t)
{
	read_data_registers(list, p, 1, t);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin){
		case GPIO_PANEL_BUTTONS:			
			read_panel_buttons();
		break;
		
		case GPIO_ENCODE_PB:			

		break;

		case GPIO_LTC_INT:			
			MCU_KILL_RESET();
		break;
		
	}
}
