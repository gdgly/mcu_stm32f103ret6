#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "IOI2C.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "thread_of_LCM_uart.h"
#include "thread_of_sensor_glove.h"
#include "thread_of_android_uart.h"
#include "device_ctrl.h"
#include "bsp_EEPROM.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void thread_of_lcm_uart (void const *argument);                             // thread function
osThreadId tid_thread_of_lcm_uart;                                          // thread id
osThreadDef (thread_of_lcm_uart, osPriorityNormal, 1, 0);                   // thread object

#define EVENT_LOOP_TIME_IN_MILLI_SECOND 100
uint16_t ctrl_register_list[32];																						//控制寄存器  与屏幕寄存器地址相对应
uint16_t data_register_list[data_register_list_max] = LCM_DATA_REGISTER_DEFUALTS;								//数据寄存器  与屏幕寄存器相对应  for detail look lcm_data_register_list in thread_of_LCM_uart.h
#define AUDIO_BASE_TIME 500
uint8_t lcm_music_list[][2] = {{0,0},{0,0},{0,0},{0,1},{2,1},{4,1},{6,1},{8,1},{10,1},{12,1},{14,1},{16,1},{18,1},{20,1},{22,1},{24,1} \
													,{26,1},{28,1},{30,1},{32,1},{34,1},{36,1},{38,1},{40,1},{42,1},{44,1},{46,1},{48,1},{50,5}};										//屏幕存储的音频位置、长度  
//uint8_t	lcm_music_list[] = {0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50};
//uint8_t	lcm_music_len[] = {1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,5};
	
uint8_t pic_old;
uint8_t panel_button_num;
struct lcm_parameter_t user_parameter;
struct lcm_uart_len_t lcm_uart_len = {0 , 0};

extern struct sensor_glove_para_t sensor_glove_para;
extern uint32_t gloves_access_mark;

extern osThreadId tid_thread_of_android_rx; 
extern osThreadId tid_thread_of_android_tx; 

osMutexDef(glove_sensor_mutex);
osMutexId glove_sensor_mutex_id;

#define DELAY_1S 1000

uint8_t lcm_uart_send_buff[128];
uint8_t *lcm_uart_send_pointer = lcm_uart_send_buff;

uint8_t lcm_ctrl_register[][3] = {
  {0x82, 0x01, 0x00},
  {0x14, 0x01, 0x01},
  {0x84, 0x02, 0x00},
};

//表示哪些数据需要被更新
//uint8_t eeprom_data_list[data_register_list_max] = {0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1};
//uint8_t eeprom_data_buff[data_register_list_max];
	
int init_thread_of_lcm_uart (void) {

  tid_thread_of_lcm_uart = osThreadCreate (osThread(thread_of_lcm_uart), NULL);
  if (!tid_thread_of_lcm_uart) return(-1);
  
  return(0);
}

uint32_t pic_switch_mark = 0;
uint8_t pic_mark,lcm_type;
void thread_of_lcm_uart (void const *argument) {
	user_signal_info_t user_timer_info = { osThreadGetId(), SIG_USER_TIMER };
	osTimerDef(main_timer, SetUserSignal);
	osTimerId main_timer_id = osTimerCreate(osTimer(main_timer), osTimerPeriodic, &user_timer_info);
	osTimerStart(main_timer_id, EVENT_LOOP_TIME_IN_MILLI_SECOND);
  while(1){
		EEPROM_CheckOk();	
//		uint8_t music_cnt = 0;
//		osDelay(3000);
//		while(1){
//			osSignalWait(SIG_USER_TIMER, osWaitForever);
//			lcm_uart_len.rx_len = 0;
//			lcm_uart_len.tx_len = 0;
//			setled(50);
//			read_ctrl_register(0, lcm_uart_send_pointer, &lcm_uart_len);
//			read_data_registers(start_and_stop, lcm_uart_send_pointer, 33, &lcm_uart_len);
//			data_register_list[start_and_stop] = 3;
//			write_data_registers(7, lcm_uart_send_pointer, 1, &data_register_list[start_and_stop], &lcm_uart_len);	
//			read_ctrl_register(0, lcm_uart_send_pointer, &lcm_uart_len);
//			play_music(6, lcm_uart_send_pointer, &lcm_uart_len);
//			setpic(3);
//			lcd_read(&user_parameter,80);	
//			music_cnt ++;
//			if(music_cnt == 26){
//				music_cnt = 0;
//			}			
//			osDelay(5000);
			
//			write_data_to_eeprom(data_register_list, data_register_list_min, sizeof(data_register_list));
////			uint8_t ret = read_data_from_eeprom(data_register_list, data_register_list_min, sizeof(data_register_list));
////			if(ret){
////				osDelay(50);
////			}
//		}

		int check_lcm_android_try = 50;
		while(check_lcm_android_try--){
			osSignalWait(SIG_USER_TIMER, osWaitForever);
			int resualt = check_device_lcm_android();
			if(resualt == 1){
				break;
			}				
		}
		
		//设备为安卓板
		if(check_lcm_android_try < 0){
			osSignalSet(tid_thread_of_android_tx, SIG_USER_2);
			osSignalSet(tid_thread_of_android_rx, SIG_USER_2);
			lcm_type = DEVICE_ANDROID;
			//手套断开监测程序，如果断开则通知安卓上位机一次
			uint8_t is_notice_android = 0;
			while(1){
				osSignalWait(SIG_USER_TIMER, osWaitForever);
				if((HAL_GetTick() - gloves_access_mark > 0.2*DELAY_1S)&&(is_notice_android == 0)){
					struct mcu_hand_type_t *p = AndroidDatagramEvtAlloc(sizeof (*p));
					if(p){
						ANDROID_DATAGRAM_INIT((*p), mcu_hand_type);
						if(HAL_GetTick()	- gloves_access_mark > 200){
							p->left_type = 0;
							p->right_type = 0;
						}else{
							p->left_type = 0x03;
							p->right_type = 0x00;
						}
						AndroidDatagramEvtSend(p);			
						is_notice_android = 1;
					}					
				}else if(HAL_GetTick() - gloves_access_mark < 0.2*DELAY_1S){
					is_notice_android = 0;				
				}
				
				osEvent pb_event = osSignalWait(SIG_USER_2, 0);
				if(pb_event.status == osEventSignal){
					panel_button_porcess();
				}else{
				
				}			
			}
		}
		
		//设备为迪文屏
		lcm_type = DEVICE_DIWEN;
		read_data_from_eeprom(data_register_list, data_register_list_min, sizeof(data_register_list));
		
		for(;;){
			osSignalWait(SIG_USER_TIMER, osWaitForever);
			lcm_uart_len.rx_len = 0;
			lcm_uart_len.tx_len = 0;
				
			switch(ctrl_register_list[pic_id]){
			case boot_animation_1:
				setled(data_register_list[user_led_set]);
				setpic(boot_animation_2);				
			break; 

			case boot_animation_2:
				osDelay(500);		
			  write_data_registers(length_of_training, lcm_uart_send_pointer, 27, &data_register_list[length_of_training], &lcm_uart_len);
			setpic(boot_animation_3);				
			break;
			
			case boot_animation_3:
				osDelay(500);
				setpic(selection_interface);
				play_music(audio_welcome, lcm_uart_send_pointer, &lcm_uart_len);
			break;
			
			case selection_interface:
				if(pic_old != selection_interface){
					data_register_list[start_and_stop] = 0;
					write_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &data_register_list[start_and_stop], &lcm_uart_len);	
					data_register_list[selection_flexion_botton] = 0;
					write_data_registers(selection_flexion_botton, lcm_uart_send_pointer, 1, &data_register_list[selection_flexion_botton], &lcm_uart_len);
					sy08_set_valve(0);
					sy08_pump_reset();
				}else{
					read_data_registers(selection_flexion_botton, lcm_uart_send_pointer, 4, &lcm_uart_len);
					if(data_register_list[selection_flexion_botton] == 1){											//屈伸训练
						if(data_register_list[training_hand] == LEFT_HAND){																		
							setpic(flexion_and_extension_left_open);
						}else if(data_register_list[training_hand] == RIGHT_HAND){								
							setpic(flexion_and_extension_right_open);
						}
					}else if(data_register_list[selection_flexion_botton] == 2){								//手套操
						if(data_register_list[training_hand] == LEFT_HAND){									
							setpic(gloves_exercises_left_open);
						}else if(data_register_list[training_hand] == RIGHT_HAND){			
							setpic(gloves_exercises_right_open);
						}					
					}else if(data_register_list[selection_flexion_botton] == 3){					//对侧训练
						if(data_register_list[training_hand] == LEFT_HAND){										
							setpic(contralateral_training_left);
						}else if(data_register_list[training_hand] == RIGHT_HAND){				
							setpic(contralateral_training_right);
						}						
					}else if(data_register_list[selection_flexion_botton] == 4){						//功能训练
						if(data_register_list[training_hand] == LEFT_HAND){								
							setpic(functional_training_left_open);
						}else if(data_register_list[training_hand] == RIGHT_HAND){		
							setpic(functional_training_right_open);
						}							
					}				
				}
			break;
			
			case flexion_and_extension_right_fists:
			case flexion_and_extension_right_open:
			case flexion_and_extension_left_fists:
			case flexion_and_extension_left_open:
				finger_flexion_and_extension();
			break;
			
			case gloves_exercises_right_fists:
			case gloves_exercises_right_open:
			case gloves_exercises_right_index:
			case gloves_exercises_right_middle:
			case gloves_exercises_right_ring:
			case gloves_exercises_right_little:
			case gloves_exercises_left_fists:
			case gloves_exercises_left_open:
			case gloves_exercises_left_index:
			case gloves_exercises_left_middle:
			case gloves_exercises_left_ring:
			case gloves_exercises_left_little:
				function_gloves_exercises();
			break;
			
			case contralateral_training_right:
			case contralateral_training_left:
				contralateral_training();
			break;
			
			case functional_training_left_open:
			case functional_training_left_catching_ball:
			case functional_training_left_two_fingers:
			case functional_training_left_three_fingers:
			case functional_training_right_open:
			case functional_training_right_catching_ball:
			case functional_training_right_two_fingers:
			case functional_training_right_three_fingers:	
				functional_training();
			break;
			
			case settings_interface:
				if(pic_old != settings_interface){
					play_music(audio_settings_interface, lcm_uart_send_pointer, &lcm_uart_len);
				}
			  read_data_registers(length_of_training, lcm_uart_send_pointer, 27, &lcm_uart_len);
				setled(data_register_list[user_led_set]);
//			  if(data_register_list[flexion_and_extension_all] == 1){
//					int i;
//					for(i=0;i<5;i++){
//						data_register_list[flexion_and_extension_thumb + i] = 1;
//					}
//					write_data_registers(flexion_and_extension_thumb, lcm_uart_send_pointer, 5, &data_register_list[flexion_and_extension_thumb], &lcm_uart_len);
//				}
				write_data_to_eeprom(data_register_list, data_register_list_min, sizeof(data_register_list));
			break;
			
			case calibration_interface:
				if(pic_old != calibration_interface){
					data_register_list[gloves_calibration_button] = 0;
					write_data_registers(gloves_calibration_button, lcm_uart_send_pointer, 1, &data_register_list[gloves_calibration_button], &lcm_uart_len);				
				}else{
					read_data_registers(gloves_calibration_button, lcm_uart_send_pointer, 1, &lcm_uart_len);
					read_data_registers(calibration_return, lcm_uart_send_pointer, 1, &lcm_uart_len);
					if(data_register_list[calibration_return] != 0){
						data_register_list[calibration_return] = 0;						
						if(data_register_list[training_hand] == LEFT_HAND){
							setpic(contralateral_training_left);	
						}else if(data_register_list[training_hand] == RIGHT_HAND){
							setpic(contralateral_training_right);		
						}
						write_data_registers(calibration_return, lcm_uart_send_pointer, 1, &data_register_list[calibration_return], &lcm_uart_len);
					}
					
					if(data_register_list[gloves_calibration_button] == 1){
						if(HAL_GetTick() - gloves_access_mark > 0.2*DELAY_1S){
							setpic(gloves_not_connected);
						}else{
							if(data_register_list[training_hand] == LEFT_HAND){																				//0为左手
								setpic(in_calibration_left_1);
							}else if(data_register_list[training_hand] == RIGHT_HAND){																	//1为右手
								setpic(in_calibration_right_1);
							}		
						}
					}
				}
			break;
				
			case gloves_is_connected:
				if(pic_old != gloves_is_connected){
					pic_switch_mark = HAL_GetTick();
				}else{
					if((HAL_GetTick() - pic_switch_mark) > 0.2*DELAY_1S){
						setpic(pic_mark);
					}
				}
			break;
			
			case gloves_not_connected:
				if(pic_old != gloves_not_connected){
					pic_mark = pic_old;
					pic_switch_mark = HAL_GetTick();
					sy08_set_valve(0);
					sy08_pump_reset();
					data_register_list[start_and_stop] = 0;
					write_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &data_register_list[start_and_stop], &lcm_uart_len);
				}else{
					if((HAL_GetTick() - pic_switch_mark) > 1.5*DELAY_1S){
						setpic(calibration_interface);
					}
				}
				if(HAL_GetTick() - gloves_access_mark < 0.2*DELAY_1S){
					setpic(gloves_is_connected);
				}
			break;
				
			case in_calibration_right_1:
			case in_calibration_right_2:
			case in_calibration_right_3:
			case in_calibration_right_4:
			case in_calibration_left_1:
			case in_calibration_left_2:
			case in_calibration_left_3:
			case in_calibration_left_4:
				gloves_calibrantion_process();
			break;
			
			case calibration_successed:
				if(pic_old != calibration_successed){
					pic_switch_mark = HAL_GetTick();
					play_music(audio_successful_calibration, lcm_uart_send_pointer, &lcm_uart_len);
				}else{
					if((HAL_GetTick() - pic_switch_mark) > 2*DELAY_1S){
						setpic(calibration_interface);
					}
				}
			break;
			
			case calibration_failed:
				if(pic_old != calibration_failed){
					pic_switch_mark = HAL_GetTick();
					play_music(audio_calibration_failed, lcm_uart_send_pointer, &lcm_uart_len);
				}else{
					if((HAL_GetTick() - pic_switch_mark) > 2*DELAY_1S){
						setpic(calibration_interface);
					}
				}
			break;
			
			case motion_ending:
				if(pic_old != motion_ending){
					pic_switch_mark = HAL_GetTick();
					data_register_list[motion_time] = 0;
					data_register_list[motion_cnt] = 0;
					write_data_registers(motion_time, lcm_uart_send_pointer, 2, &data_register_list[motion_time], &lcm_uart_len);
					sy08_set_valve(0);
					sy08_pump_reset();	
					play_music(audio_end_of_training, lcm_uart_send_pointer, &lcm_uart_len);
				}else{
					if((HAL_GetTick() - pic_switch_mark) > 10*DELAY_1S){
						setpic(selection_interface);
					}
				}							
			break;
				
			}
			pic_old = ctrl_register_list[pic_id];
			
			osEvent pb_event = osSignalWait(SIG_USER_2, 0);
			if(pb_event.status == osEventSignal){
				panel_button_porcess();
			}else{
//				read_ctrl_register(1, lcm_uart_send_pointer, &lcm_uart_len);
//				read_ctrl_register(0, lcm_uart_send_pointer, &lcm_uart_len);			
//				lcd_read(&user_parameter,80);					
			}
			read_ctrl_register(1, lcm_uart_send_pointer, &lcm_uart_len);
//			read_ctrl_register(0, lcm_uart_send_pointer, &lcm_uart_len);			
			lcd_read(&user_parameter,80);			
		}
  }
}


//手指屈伸训练
//valve的0-4位为电磁阀的控制位，example: 10101 is for thumb, middle finger, little thumb on
uint32_t lcd_time_mark,motion_start_mark,training_time_mark;					//training time mark 是为了在暂停时记录训练计时
uint8_t gloves_direction,order_cnt = 0;												//1 for 握拳   ， 1 for 张开
void finger_flexion_and_extension(void)
{
	uint8_t valve;
	if(pic_old == selection_interface){
		lcd_time_mark = HAL_GetTick() - data_register_list[training_interval]*DELAY_1S;
		motion_start_mark = HAL_GetTick();
		training_time_mark = 0;
		
		play_music(audio_finger_flexion_and_extension, lcm_uart_send_pointer, &lcm_uart_len);
		
		gloves_direction = 0;
		read_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &lcm_uart_len);
	}else{
		if(data_register_list[start_and_stop]){
			
			training_time_mark = HAL_GetTick() - motion_start_mark;
			data_register_list[motion_time] = training_time_mark/(60*DELAY_1S);
			if(data_register_list[motion_time] > data_register_list[length_of_training]){
				setpic(motion_ending);
				return ;
			}
			write_data_registers(motion_time, lcm_uart_send_pointer, 1, &data_register_list[motion_time], &lcm_uart_len);
			
			
			if(data_register_list[flexion_and_extension_all] == 1){
				//顺序模式
				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[motion_close_interval]*DELAY_1S)&&(order_cnt < 5)){
					lcd_time_mark = HAL_GetTick();
					for(int i=0;i<5;i++){
						if(i<=order_cnt){
							valve = (data_register_list[flexion_and_extension_thumb + i]&VALVE_SET)|(valve<<1);
						}else{
							valve = VALVE_RESET|(valve<<1);
						}
					}
					sy08_pump_intermittent_set(lcd_time_mark);
					sy08_set_valve(valve);
					order_cnt++;
					data_register_list[motion_cnt]++;
					write_data_registers(motion_cnt, lcm_uart_send_pointer, 1, &data_register_list[motion_cnt], &lcm_uart_len);
					if(data_register_list[training_hand] == LEFT_HAND){
						setpic(flexion_and_extension_left_fists);
					}else if(data_register_list[training_hand] == RIGHT_HAND){
						setpic(flexion_and_extension_right_fists);
					}
					play_music(audio_clenched_hands, lcm_uart_send_pointer, &lcm_uart_len);					
				}

				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[motion_open_interval]*DELAY_1S)&&(order_cnt == 5)){
					lcd_time_mark = HAL_GetTick();
					valve = (VALVE_RESET<<4)| 	\
									(VALVE_RESET<<3)| 	\
									(VALVE_RESET<<2)| 	\
									(VALVE_RESET<<1)| 	\
									(VALVE_RESET);
					sy08_pump_intermittent_set(lcd_time_mark);
					sy08_set_valve(valve);					
					order_cnt = 0;
					if(data_register_list[training_hand] == LEFT_HAND){
						setpic(flexion_and_extension_left_open);
					}else if(data_register_list[training_hand] == RIGHT_HAND){
						setpic(flexion_and_extension_right_open);
					}
					play_music(audio_open_hands, lcm_uart_send_pointer, &lcm_uart_len);					
				}
			}else if(data_register_list[flexion_and_extension_all] == 0){
				//非顺序模式
				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[motion_close_interval]*DELAY_1S)&&gloves_direction == 1){
					lcd_time_mark = HAL_GetTick();
					
					valve = ((data_register_list[flexion_and_extension_little_thumb]&VALVE_RESET)<<4)| 	\
									((data_register_list[flexion_and_extension_ring_finger]&VALVE_RESET)<<3)| 		\
									((data_register_list[flexion_and_extension_middle_finger]&VALVE_RESET)<<2)| 	\
									((data_register_list[flexion_and_extension_index_finger]&VALVE_RESET)<<1)| 	\
									(data_register_list[flexion_and_extension_thumb]&VALVE_RESET);
					sy08_pump_intermittent_set(lcd_time_mark);
					sy08_set_valve(valve);
					gloves_direction = 0;
					data_register_list[motion_cnt]++;
					write_data_registers(motion_cnt, lcm_uart_send_pointer, 1, &data_register_list[motion_cnt], &lcm_uart_len);
					if(data_register_list[training_hand] == LEFT_HAND){
						setpic(flexion_and_extension_left_open);
					}else if(data_register_list[training_hand] == RIGHT_HAND){
						setpic(flexion_and_extension_right_open);
					}
					
					play_music(audio_open_hands, lcm_uart_send_pointer, &lcm_uart_len);
	//				data_register_list[flexion_and_extension_icon] = 1;
	//				write_data_registers(flexion_and_extension_icon, lcm_uart_send_pointer, 1, &data_register_list[flexion_and_extension_icon], &lcm_uart_len);	
				}

				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[motion_open_interval]*DELAY_1S)&&gloves_direction == 0){
					lcd_time_mark = HAL_GetTick();
					valve = ((data_register_list[flexion_and_extension_little_thumb]&VALVE_SET)<<4)| 	\
									((data_register_list[flexion_and_extension_ring_finger]&VALVE_SET)<<3)| 		\
									((data_register_list[flexion_and_extension_middle_finger]&VALVE_SET)<<2)| 	\
									((data_register_list[flexion_and_extension_index_finger]&VALVE_SET)<<1)| 	\
									(data_register_list[flexion_and_extension_thumb]&VALVE_SET);
					sy08_pump_intermittent_set(lcd_time_mark);
					sy08_set_valve(valve);
					gloves_direction = 1;
					if(data_register_list[training_hand] == LEFT_HAND){
						setpic(flexion_and_extension_left_fists);
					}else if(data_register_list[training_hand] == RIGHT_HAND){
						setpic(flexion_and_extension_right_fists);
					}
					play_music(audio_clenched_hands, lcm_uart_send_pointer, &lcm_uart_len);
					
	//				data_register_list[flexion_and_extension_icon] = 0;
	//				write_data_registers(flexion_and_extension_icon, lcm_uart_send_pointer, 1, &data_register_list[flexion_and_extension_icon], &lcm_uart_len);
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


//功能训练 ( 对指训练 )
//valve的0-4位为电磁阀的控制位，example: 10101 is for thumb, middle finger, little thumb on
uint8_t exercises_training_list[5];   //功能训练列表，从0到4分别为：对食指，对中指，对无名指，对小指，握拳， 相应的位为1表示选中
uint8_t exercises_list_cnt = 0;
void function_gloves_exercises(void)
{
	uint8_t valve;
	if(pic_old == selection_interface){
		lcd_time_mark = HAL_GetTick() - data_register_list[training_interval]*DELAY_1S;
		motion_start_mark = HAL_GetTick();
		training_time_mark = 0;
		
		play_music(audio_gloves_exercises, lcm_uart_send_pointer, &lcm_uart_len);
		int i;
		for(i=0;i<5;i++){
			exercises_training_list[i] = data_register_list[gloves_exercises_index_finger + i];
		}
		read_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &lcm_uart_len);
		
		gloves_direction = 0;
		exercises_list_cnt = 0;
	}else{
		if(data_register_list[start_and_stop]){
			
			
			
			training_time_mark = HAL_GetTick() - motion_start_mark;
			data_register_list[motion_time] = training_time_mark/(60*DELAY_1S);
			if(data_register_list[motion_time] > data_register_list[length_of_training]){
				setpic(motion_ending);
				return ;
			}
			write_data_registers(motion_time, lcm_uart_send_pointer, 1, &data_register_list[motion_time], &lcm_uart_len);
			if(exercises_training_list[exercises_list_cnt]){
				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[motion_open_interval]*DELAY_1S)&&gloves_direction == 0){
					lcd_time_mark = HAL_GetTick();
					
					valve = ((((exercises_list_cnt==3)||(exercises_list_cnt==4))?VALVE_SET:VALVE_RESET)<<4)| 	\
									((((exercises_list_cnt==2)||(exercises_list_cnt==4))?VALVE_SET:VALVE_RESET)<<3)| 	\
									((((exercises_list_cnt==1)||(exercises_list_cnt==4))?VALVE_SET:VALVE_RESET)<<2)| 	\
									((((exercises_list_cnt==0)||(exercises_list_cnt==4))?VALVE_SET:VALVE_RESET)<<1)| 	\
									(VALVE_SET);
					sy08_pump_intermittent_set(lcd_time_mark);
					sy08_set_valve(valve);
					gloves_direction = 1;	
					play_music(audio_gloves_exercises_index + exercises_list_cnt, lcm_uart_send_pointer, &lcm_uart_len);			
					
					if(data_register_list[training_hand] == LEFT_HAND){
						setpic(gloves_exercises_left_index + exercises_list_cnt);
					}else if(data_register_list[training_hand] == RIGHT_HAND){
						setpic(gloves_exercises_right_index + exercises_list_cnt);
					}


			
//					data_register_list[gloves_exercises_icon] = exercises_list_cnt;
//					write_data_registers(gloves_exercises_icon, lcm_uart_send_pointer, 1, &data_register_list[gloves_exercises_icon], &lcm_uart_len);
				}
				
				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[motion_close_interval]*DELAY_1S)&&gloves_direction == 1){
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
					sy08_pump_intermittent_set(lcd_time_mark);
					sy08_set_valve(valve);
					gloves_direction = 0;
					play_music(audio_open_hands, lcm_uart_send_pointer, &lcm_uart_len);
					data_register_list[motion_cnt]++;
					write_data_registers(motion_cnt, lcm_uart_send_pointer, 1, &data_register_list[motion_cnt], &lcm_uart_len);					
					if(data_register_list[training_hand] == LEFT_HAND){
						setpic(gloves_exercises_left_open);
					}else if(data_register_list[training_hand] == RIGHT_HAND){
						setpic(gloves_exercises_right_open);
					}				
					

//					data_register_list[gloves_exercises_icon] = 0x05;
//					write_data_registers(gloves_exercises_icon, lcm_uart_send_pointer, 1, &data_register_list[gloves_exercises_icon], &lcm_uart_len);
					
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


//对侧训练
uint8_t contralateral_valve;
void contralateral_training(void)
{
	uint8_t degree[5];
	int i;
	
	if(pic_old == selection_interface){
		motion_start_mark = HAL_GetTick();
		training_time_mark = 0;		
		play_music(audio_contralateral_training, lcm_uart_send_pointer, &lcm_uart_len);
		
	}else{
		if(data_register_list[start_and_stop]){
			if(HAL_GetTick() - gloves_access_mark > 0.2*DELAY_1S){
				setpic(gloves_not_connected);
				return;
			}
			training_time_mark = HAL_GetTick() - motion_start_mark;
			data_register_list[motion_time] = training_time_mark/(60*DELAY_1S);
			if(data_register_list[motion_time] > data_register_list[length_of_training]){
				setpic(motion_ending);
				return ;
			}
			write_data_registers(motion_time, lcm_uart_send_pointer, 1, &data_register_list[motion_time], &lcm_uart_len);
			
			sy08_pump_set(60 + 10*data_register_list[training_strength]);
			
			osMutexWait(glove_sensor_mutex_id,osWaitForever);
			memcpy(degree,sensor_glove_para.degree,sizeof(degree));
			osMutexRelease(glove_sensor_mutex_id);	
			
			if(degree[0] < 60){
				contralateral_valve = contralateral_valve&0x1e;
				contralateral_valve = (VALVE_SET)|contralateral_valve;
			}else if(degree[0] < 130){
				
			}else if(degree[0] < 180){
				contralateral_valve = contralateral_valve&0x1e;
				contralateral_valve = (VALVE_RESET)|contralateral_valve;
			}else{
				contralateral_valve = contralateral_valve&0x1e;
				contralateral_valve = (VALVE_RESET)|contralateral_valve;
			}	
			
			for(i=1;i<4;i++){
				if(degree[i] < 45){
					contralateral_valve = contralateral_valve&(0x1f - (1<<i));
					contralateral_valve = (VALVE_SET<<i)|contralateral_valve;
				}else if(degree[i] < 130){
					
				}else if(degree[i] < 180){
					contralateral_valve = contralateral_valve&(0x1f - (1<<i));
					contralateral_valve = (VALVE_RESET<<i)|contralateral_valve;
				}else{
					contralateral_valve = contralateral_valve&(0x1f - (1<<i));
					contralateral_valve = (VALVE_RESET<<i)|contralateral_valve;
				}	
			}
			
			if(degree[4] < 60){
				contralateral_valve = contralateral_valve&(0x1f - (1<<4));
				contralateral_valve = (VALVE_SET<<4)|contralateral_valve;
			}else if(degree[4] < 130){
				
			}else if(degree[4] < 180){
				contralateral_valve = contralateral_valve&(0x1f - (1<<4));
				contralateral_valve = (VALVE_RESET<<4)|contralateral_valve;
			}else{
				contralateral_valve = contralateral_valve&(0x1f - (1<<4));
				contralateral_valve = (VALVE_RESET<<4)|contralateral_valve;
			}
			
			sy08_set_valve(contralateral_valve);			
		}else{
			motion_start_mark = HAL_GetTick() - training_time_mark;
			sy08_pump_reset();
		}
		read_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &lcm_uart_len);
	}

}

//功能训练
uint8_t functional_training_list[3];
const uint8_t functional_training_func[][5] = {{0x01,0x01,0x01,0x01,0x01},{0x00,0x00,0x00,0x01,0x01},{0x00,0x00,0x01,0x01,0x01}}; //必须加const 才能保证数组能用
uint8_t functional_training_list_cnt;
void functional_training(void)
{
	uint8_t valve;
	if(pic_old == selection_interface){
		lcd_time_mark = HAL_GetTick() - data_register_list[training_interval]*DELAY_1S;
		motion_start_mark = HAL_GetTick();
		training_time_mark = 0;
		
		int i;
		for(i=0;i<3;i++){
			functional_training_list[i] = data_register_list[functional_training_catching_ball + i];
		}
		read_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &lcm_uart_len);
		play_music(audio_functional_training, lcm_uart_send_pointer, &lcm_uart_len);
		gloves_direction = 0;
		functional_training_list_cnt = 0;
	}else{
		if(data_register_list[start_and_stop]){
			
			training_time_mark = HAL_GetTick() - motion_start_mark;
			data_register_list[motion_time] = training_time_mark/(60*DELAY_1S);
			if(data_register_list[motion_time] > data_register_list[length_of_training]){
				setpic(motion_ending);
				return ;
			}
			write_data_registers(motion_time, lcm_uart_send_pointer, 1, &data_register_list[motion_time], &lcm_uart_len);
			
			sy08_pump_set(60 + 10*data_register_list[training_strength]);
			
			if(functional_training_list[functional_training_list_cnt]){
				
				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[motion_open_interval]*DELAY_1S)&&gloves_direction == 0){
					lcd_time_mark = HAL_GetTick();
					
					valve = (((functional_training_func[functional_training_list_cnt][0])?VALVE_SET:VALVE_RESET)<<4)| 	\
									(((functional_training_func[functional_training_list_cnt][1])?VALVE_SET:VALVE_RESET)<<3)| 	\
									(((functional_training_func[functional_training_list_cnt][2])?VALVE_SET:VALVE_RESET)<<2)| 	\
									(((functional_training_func[functional_training_list_cnt][3])?VALVE_SET:VALVE_RESET)<<1)| 	\
									((functional_training_func[functional_training_list_cnt][4])?VALVE_SET:VALVE_RESET);
					sy08_set_valve(valve);
					gloves_direction = 1;
					if(data_register_list[training_hand] == LEFT_HAND){
						setpic(functional_training_left_catching_ball + functional_training_list_cnt);
					}else if(data_register_list[training_hand] == RIGHT_HAND){
						setpic(functional_training_right_catching_ball + functional_training_list_cnt);
					}	
					
					play_music(audio_functional_training_catching_ball + functional_training_list_cnt, lcm_uart_send_pointer, &lcm_uart_len);
//					data_register_list[functional_training_icon] = 2*functional_training_list_cnt;
//					write_data_registers(functional_training_icon, lcm_uart_send_pointer, 1, &data_register_list[functional_training_icon], &lcm_uart_len);
				}
				
//				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[motion_close_interval]*DELAY_1S)&&gloves_direction == 1){
//					lcd_time_mark = HAL_GetTick();
//					
//					valve = (((functional_training_func[functional_training_list_cnt][0])?VALVE_SET:VALVE_RESET)<<4)| 	\
//									(((functional_training_func[functional_training_list_cnt][1])?VALVE_SET:VALVE_RESET)<<3)| 	\
//									(((functional_training_func[functional_training_list_cnt][2])?VALVE_SET:VALVE_RESET)<<2)| 	\
//									(((functional_training_func[functional_training_list_cnt][3])?VALVE_SET:VALVE_RESET)<<1)| 	\
//									((functional_training_func[functional_training_list_cnt][4])?VALVE_SET:VALVE_RESET);
//					
//					sy08_set_valve(valve);
//					gloves_direction = 2;
//					
//					data_register_list[functional_training_icon] = 2*functional_training_list_cnt + 1;
//					write_data_registers(functional_training_icon, lcm_uart_send_pointer, 1, &data_register_list[functional_training_icon], &lcm_uart_len);				
//				}				
				
				if(((HAL_GetTick() - lcd_time_mark) > data_register_list[motion_close_interval]*DELAY_1S)&&gloves_direction == 1){
					lcd_time_mark = HAL_GetTick();
					
					functional_training_list_cnt++;
					if(functional_training_list_cnt == 3){
						functional_training_list_cnt = 0;
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
					if(data_register_list[training_hand] == LEFT_HAND){
						setpic(functional_training_left_open);
					}else if(data_register_list[training_hand] == RIGHT_HAND){
						setpic(functional_training_right_open);
					}	

					play_music(audio_hand_release, lcm_uart_send_pointer, &lcm_uart_len);
//					data_register_list[functional_training_icon] = 2*functional_training_list_cnt;
//					write_data_registers(functional_training_icon, lcm_uart_send_pointer, 1, &data_register_list[functional_training_icon], &lcm_uart_len);					
				}
			}else{
				functional_training_list_cnt++;
				if(functional_training_list_cnt == 3){
					functional_training_list_cnt = 0;
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


//数据手套校准
void gloves_calibrantion_process(void)
{
	if(pic_old == calibration_interface){
		play_music(audio_start_calibration, lcm_uart_send_pointer, &lcm_uart_len);
		motion_start_mark = HAL_GetTick();
		struct calibration_cmd_t *p = SerialDatagramEvtAlloc(sizeof (*p));
		if(p){
			SERIAL_DATAGRAM_INIT((*p), calibration_cmd);
			p->cmd = 1;
			SerialDatagramEvtSend(p);
		}
	}else{
		uint8_t icon = ((HAL_GetTick() - motion_start_mark)/1500);
		if(icon < 8){
			if(data_register_list[training_hand] == LEFT_HAND){
				setpic(in_calibration_left_1 + icon%4);
			}else if(data_register_list[training_hand] == RIGHT_HAND){
				setpic(in_calibration_right_1 + icon%4);
			}
			play_music(audio_please_clenched_hands + icon%2, lcm_uart_send_pointer, &lcm_uart_len);
			return;
		}else if(icon == 8){
			struct calibration_cmd_t *p = SerialDatagramEvtAlloc(sizeof (*p));
			if(p){
				SERIAL_DATAGRAM_INIT((*p), calibration_cmd);
				p->cmd = 2;
				SerialDatagramEvtSend(p);
			}		
		}else{			
			uint8_t resualt = 0;
			osMutexWait(glove_sensor_mutex_id,osWaitForever);
			resualt = sensor_glove_para.calibration_resualt;
			osMutexRelease(glove_sensor_mutex_id);
			
			if(resualt == 1){
				setpic(calibration_failed);	
			}else if(resualt == 2){
				setpic(calibration_successed);
			}
			return;
		}
		
//		if(icon > 3){
//			icon = icon - 4;
//		}
//		
//		if(data_register_list[training_hand] == LEFT_HAND){
//			setpic(in_calibration_left_1 + icon);
//		}else if(data_register_list[training_hand] == RIGHT_HAND){
//			setpic(in_calibration_right_1 + icon);
//		}
		
//		data_register_list[gloves_calibration_icon] = icon;
//		write_data_registers(gloves_calibration_icon, lcm_uart_send_pointer, 1, &data_register_list[gloves_calibration_icon], &lcm_uart_len);
	}
}


//lcd read is for communication with diwen
//通讯周期为100ms，在此100ms内，read_ctrl_register, write_ctrl_register, write_data_registers, read_data_registers
//会将需要串口发送的内容存入lcm_uart_send_buff，需要发送和接收的数据长度存入lcm_uart_len
//因此可以根据不同状态下，调整串口发送和接收的长度
//SendReqAndRecvResDataWithUart会发送、接收串口内容
//if(ret){}里会处理串口反馈的数据，依据迪文串口通讯格式，分别处理控制寄存器和数据寄存器的值，将结果存入ctrl_register_list和data_register_list中
uint16_t *nctrl_register = ctrl_register_list;
uint16_t *ndata_register = data_register_list;

int lcd_read(struct lcm_parameter_t *p,uint32_t millisec)
{
	uint8_t data[128];
	uint8_t *nrx = data;
	
//	write_data_registers(0, s, 5, nwrite, &t);
//	read_data_registers(0, lcm_uart_send_pointer, 30, &lcm_uart_len);
	memset(data,0,sizeof(data));
	int ret =	SendReqAndRecvResDataWithUart(3,lcm_uart_send_buff,lcm_uart_len.tx_len,data,lcm_uart_len.rx_len,millisec);
	if(ret){
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_11);	
		while(nrx < data + lcm_uart_len.rx_len){
			while((*nrx != FRAME_HEAD_H)&&(*(nrx + 1) != FRAME_HEAD_L)){
				nrx++;
			}
			nrx+=3;	
//			nrx++;
			
			if(*nrx == WR_DATA_REG){
				continue;
			}
			
			if(*nrx == RD_DATA_REG){
				nrx++;
				if(*nrx == CTRL_REG_ADDR_H){
					nrx++;
					for (int i = 0; i < sizeof(ctrl_register_list); i++){
						if(lcm_ctrl_register[i][0] == *nrx){
							nrx++;
							uint8_t ln = *nrx;
							nrx++;
							int u;
							for(u=0;u<ln;u++){
								*(nctrl_register + u + lcm_ctrl_register[i][2]) = *nrx<<8|*(nrx+1);
								nrx+=2;
							}
							break;
						}
					}
				}
				
				if(*nrx == DATA_REG_ADDR_H){
					nrx++;
					uint8_t addr = *nrx;
					nrx++;
					uint8_t ln = *nrx;
					nrx++;
					int u;
					for(u=0;u<ln;u++){
						*(ndata_register + u + addr) = *nrx<<8|*(nrx+1);
						nrx+=2;
					}
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
	uint16_t buff[3];
	buff[0] = 0x5a01;
	buff[1] = pic;
	write_ctrl_register(pic_set, lcm_uart_send_pointer, buff, &lcm_uart_len);
}

//设置当前显示背光亮度
void setled(uint16_t valve)
{
	uint16_t led = valve<<8|0x0064;
	write_ctrl_register(led_set, lcm_uart_send_pointer, &led, &lcm_uart_len);
}

//检查设备，确定是串口屏/安卓板的哪一个
int check_device_lcm_android(void)
{
	uint8_t buff[32],data[32];
	buff[0] = FRAME_HEAD_H;
	buff[1] = FRAME_HEAD_L;
	buff[2] = 0x04;
	buff[3] = RD_DATA_REG;
	buff[4] = 0x00;
	buff[5] = 0x00;
	buff[6] = 0x04;
	memset(data, 0 , sizeof(data));
	int ret = SendReqAndRecvResDataWithUart(3, buff, 7, data, 15, 100);
	if(ret){
		uint16_t version = (data[5]<<8)|(data[6]);
		if(!version){
			return -1;
		}
		return 1;
	}
	return -1;
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
	*s++ = 0x03 + lcm_ctrl_register[list][REGISTER_LEN]*2;
	*s++ = WR_DATA_REG;	
	*s++ = 0x00;
	*s++ = lcm_ctrl_register[list][REGISTER_ADDR]&0x00ff;
	uint8_t i = 0;
	while(i < lcm_ctrl_register[list][REGISTER_LEN]){
		*s++ = (*(value + i)&0xff00)>>8;
		*s++ = *(value + i)&0x00ff;
		i++;
	}
	t->tx_len = s - p;
	t->rx_len += 6;
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
	*s++ = 0x04;
	*s++ = RD_DATA_REG;
	*s++ = 0x00;
	*s++ = lcm_ctrl_register[list][REGISTER_ADDR]&0x00ff;
	*s++ = lcm_ctrl_register[list][REGISTER_LEN]&0x00ff;
	t->tx_len = s - p;
	t->rx_len += 7 + lcm_ctrl_register[list][REGISTER_LEN]*2;
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
	*s++ = 0x10;
	*s++ = list;
//	*s++ = len;
//	*s++ = lcm_data_register[list]&0x00ff;
	int i;
	for(i=0;i<len;i++){
		*s++ = (*(value+i)&0xff00)>>8;
		*s++ = *(value+i)&0x00ff;
	}
	t->tx_len = s - p;
	t->rx_len += 6;
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
	*s++ = 0x10;
	*s++ = list;
	*s++ = len;
	t->tx_len = s - p;
	t->rx_len += 7 + 2*len;
}

uint32_t audio_play_interval = 0;
uint32_t audio_play_mark = 0;
uint8_t audio_old = 0xff;
//播放音乐
void play_music(const uint8_t music, uint8_t *p, struct lcm_uart_len_t *t)
{
	if(((HAL_GetTick() - audio_play_mark) > audio_play_interval)&&(audio_old != music)){
		uint8_t *s = p + t->tx_len;
		*s++ = FRAME_HEAD_H;
		*s++ = FRAME_HEAD_L;
		*s++ = 0x07;
		*s++ = WR_DATA_REG;
		*s++ = 0x00;
		*s++ = PLAY_MUSIC_SET;
		*s++ = lcm_music_list[music][0];																						//音乐的起始地址
		*s++ = lcm_music_list[music][1];																						//音乐的长度
		if(data_register_list[audio_switch] == 0){
			*s++ = data_register_list[volume_set];			
		}else if(data_register_list[audio_switch] == 1){
			*s++ = 0;
		}
		*s++ = 0;
//		*s++ = data_register_list[volume_set];	
		t->tx_len = s - p;
		t->rx_len += 6;
		audio_old = music;
		audio_play_mark = HAL_GetTick();
		audio_play_interval = lcm_music_list[music][1]*AUDIO_BASE_TIME;		
	}
}

void panel_button_porcess(void)
{
	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_11);	
	if(lcm_type == DEVICE_ANDROID){
		struct mcu_noti_scene_t *mcu_noti_scene_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_noti_scene_evt));
		if(mcu_noti_scene_evt){
			ANDROID_DATAGRAM_INIT((*mcu_noti_scene_evt), mcu_noti_scene);
			switch(panel_button_num){
				case button_selection_interface:
					mcu_noti_scene_evt->scene = 0x00;
					break;
				case button_motion_stop:
					mcu_noti_scene_evt->scene = 0x01;
					break;
				case button_audio_decrease:
					mcu_noti_scene_evt->scene = 0x03;
					break;
				case button_settings_interface:
					mcu_noti_scene_evt->scene = 0x02;
					break;
				case button_start_stop:
					mcu_noti_scene_evt->scene = 0x06;
					break;
				case button_audio_increase:
					mcu_noti_scene_evt->scene = 0x04;
					break;
				case button_reserve:
					mcu_noti_scene_evt->scene = 0x05;
					break;
			}
			AndroidDatagramEvtSend(mcu_noti_scene_evt);
		}		
	}else if(lcm_type == DEVICE_DIWEN){
		switch(panel_button_num){
			case button_selection_interface:
				data_register_list[motion_time] = 0;
				data_register_list[motion_cnt] = 0;
				write_data_registers(motion_time, lcm_uart_send_pointer, 2, &data_register_list[motion_time], &lcm_uart_len);
				sy08_set_valve(0);
				sy08_pump_reset();
				setpic(selection_interface);
				break;
			case button_motion_stop:
				if((ctrl_register_list[pic_id] >= flexion_and_extension_right_fists)&&(ctrl_register_list[pic_id] <= functional_training_right_three_fingers)){
					setpic(motion_ending);						
				}
				break;
			case button_audio_decrease:
				if(data_register_list[volume_set] > 10){
					data_register_list[volume_set] = data_register_list[volume_set] - 10;						
				}else{
					data_register_list[volume_set] = 0;
				}
				write_data_registers(volume_set, lcm_uart_send_pointer, 1, &data_register_list[volume_set], &lcm_uart_len);
				break;
			case button_settings_interface:
				data_register_list[motion_time] = 0;
				data_register_list[motion_cnt] = 0;
				write_data_registers(motion_time, lcm_uart_send_pointer, 2, &data_register_list[motion_time], &lcm_uart_len);
				sy08_set_valve(0);
				sy08_pump_reset();						
				setpic(settings_interface);
				break;
			case button_start_stop:
				if((ctrl_register_list[pic_id] >= flexion_and_extension_right_fists)&&(ctrl_register_list[pic_id] <= functional_training_right_three_fingers)){
					data_register_list[start_and_stop] = !data_register_list[start_and_stop];
					write_data_registers(start_and_stop, lcm_uart_send_pointer, 1, &data_register_list[start_and_stop], &lcm_uart_len);					
				}						
				break;
			case button_audio_increase:
				if(data_register_list[volume_set] < 90){
					data_register_list[volume_set] = data_register_list[volume_set] + 10;						
				}else{
					data_register_list[volume_set] = 100;
				}
				write_data_registers(volume_set, lcm_uart_send_pointer, 1, &data_register_list[volume_set], &lcm_uart_len);						
				break;
			case button_reserve:
				break;
		}
	}
}


uint8_t read_panel_buttons(void)
{
	panel_button_num = (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)<<2)|(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)<<1)|HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7);
	osSignalSet(tid_thread_of_lcm_uart, SIG_USER_2);
	return 0;
}

void read_data_register(const uint8_t list, uint8_t *p, struct lcm_uart_len_t *t)
{
	read_data_registers(list, p, 1, t);
}

void sy08_pump_intermittent_set(uint32_t mark)
{
	if(data_register_list[training_strength] >= 3){
		sy08_pump_set(100);
	}else if(data_register_list[training_strength] == 2){
		if((HAL_GetTick() - mark) < 5*DELAY_1S){
			sy08_pump_set(100);
		}else{
			sy08_pump_reset();
		}
	}else if(data_register_list[training_strength] <= 1){
		if((HAL_GetTick() - mark) < 3*DELAY_1S){
			sy08_pump_set(100);
		}else{
			sy08_pump_reset();
		}	
	}
}


uint32_t panel_buttons_mark;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin){
		case GPIO_PANEL_BUTTONS:
			if(HAL_GetTick() - panel_buttons_mark > 0.5*DELAY_1S){
				panel_buttons_mark = HAL_GetTick();			
				read_panel_buttons();
			}
		break;
		
		case GPIO_ENCODE_PB:			

		break;

		case GPIO_LTC_INT:			
			MCU_KILL_RESET();
		break;
		
	}
}
