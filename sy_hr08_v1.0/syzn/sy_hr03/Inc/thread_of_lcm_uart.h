#ifndef __THREAD_OF_LCM_UART_H
#define __THREAD_OF_LCM_UART_H
#include "stm32f1xx_hal.h"

int init_thread_of_lcm_uart (void);

#define GPIO_LTC_INT GPIO_PIN_12								//电源按键中断
#define GPIO_ENCODE_PB GPIO_PIN_11							//旋钮按键中断
#define GPIO_PANEL_BUTTONS GPIO_PIN_6						//面板按键 74LS148的触发中断

#define LEFT_HAND 1															//1为左手
#define RIGHT_HAND 2														//2为右手

struct user_common_t{
	uint8_t pic_id;
	uint16_t all_times;
	uint16_t motion_time;
	uint16_t training_interval;
};

struct lcm_parameter_t{
	struct user_common_t common;
	uint16_t current_times;
	uint16_t treatment_time;
	uint16_t reset_sys;			
	uint8_t hand_state;
};


//屏幕控制寄存器地址
enum lcm_ctrl_register_list{
	led_set,	
	pic_id,
	pic_set,
};

//屏幕数据寄存器地址
enum lcm_data_register_list{
	data_register_list_min,
	start_and_stop = 1,
	motion_time,
	motion_cnt,
	flexion_and_extension_icon,
	gloves_exercises_icon,
	functional_training_icon,
	length_of_training,
	training_mode,
	training_interval,
	training_strength,														//0x0a
	training_hand,
	flexion_and_extension_thumb,
	flexion_and_extension_index_finger,
	flexion_and_extension_middle_finger,
	flexion_and_extension_ring_finger,
	flexion_and_extension_little_thumb,						//0x10
	flexion_and_extension_all,
	gloves_exercises_index_finger,
	gloves_exercises_middle_finger,
	gloves_exercises_ring_finger,
	gloves_exercises_little_thumb,
	gloves_exercises_all,
	functional_training_catching_ball,
	functional_training_two_fingers,
	functional_training_three_fingers,
	gloves_calibration_icon,											//0x1a
	gloves_calibration_button,
	audio_switch,																		
	volume_set,
	user_led_set,																	//0x1e
	selection_flexion_botton,
	motion_open_interval,	
	motion_close_interval,												//0x21
	calibration_return = 0x23,
	data_register_list_max = 40,
};


//图片编号
enum lcm_pic_id_t{
	boot_animation_1 = 0,
	boot_animation_2,
	boot_animation_3,
	selection_interface,
	selection_interface_fake,
	flexion_and_extension_right_fists,
	flexion_and_extension_right_open,
	flexion_and_extension_left_fists,
	flexion_and_extension_left_open,
	gloves_exercises_right_open,	
	gloves_exercises_right_index,
	gloves_exercises_right_middle,
	gloves_exercises_right_ring,
	gloves_exercises_right_little,
	gloves_exercises_right_fists,	
	gloves_exercises_left_open,	
	gloves_exercises_left_index,
	gloves_exercises_left_middle,
	gloves_exercises_left_ring,
	gloves_exercises_left_little,
	gloves_exercises_left_fists,	
	contralateral_training_right,
	contralateral_training_left,
	functional_training_left_open,
	functional_training_left_catching_ball,
	functional_training_left_two_fingers,
	functional_training_left_three_fingers,
	functional_training_right_open,
	functional_training_right_catching_ball,
	functional_training_right_two_fingers,
	functional_training_right_three_fingers,
	settings_interface,
	calibration_interface,
	gloves_is_connected,													//
	gloves_not_connected,
	in_calibration_right_1,
	in_calibration_right_2,
	in_calibration_right_3,
	in_calibration_right_4,
	in_calibration_left_1,
	in_calibration_left_2,
	in_calibration_left_3,
	in_calibration_left_4,
	calibration_successed,
	calibration_failed,
	motion_ending,																//
};

//音乐播放列表
enum lcm_audio_id_t{
	audio_settings_interface  = 3,	
	audio_finger_flexion_and_extension,
	audio_open_hands,
	audio_clenched_hands,
	audio_gloves_exercises,
	audio_gloves_exercises_index,
	audio_gloves_exercises_middle,
	audio_gloves_exercises_ring,	
	audio_gloves_exercises_little,
	audio_make_a_fist,
	audio_contralateral_training,
	audio_functional_training,
	audio_hand_release,	
	audio_functional_training_catching_ball,
	audio_functional_training_two_fingers,	
	audio_functional_training_three_fingers,		
	audio_start_calibration,
	audio_please_clenched_hands,
	audio_please_open,	
	audio_successful_calibration,
	audio_calibration_failed,
	audio_start_training,
	audio_end_of_training,
	audio_audio_on,	
	audio_please_choose,
	audio_welcome,
};


enum panel_button_type_t{
	button_selection_interface,
	button_motion_stop,
	button_audio_decrease,	
	button_settings_interface,
	button_start_stop = 5,	
	button_audio_increase,	
	button_reserve,
};
//defualt valve for lcm data register
//1 - 27 is in use
#define LCM_DATA_REGISTER_DEFUALTS {0,1,0,0,0,0,0,10,1,4,2,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,0,0,100,100,0,4,5}

#define REGISTER_ADDR  0
#define REGISTER_LEN  1

/*数据格式定义*/
#define WR_CTL_REG 0x80				//写控制寄存器
#define RD_CTL_REG 0x81				//读控制寄存器
#define WR_DATA_REG 0x82			//写数据寄存器
#define RD_DATA_REG 0x83			//读数据寄存器
#define FRAME_HEAD_H 0x5A
#define FRAME_HEAD_L 0xA5			//数据帧头为A55A
#define DATA_REG_ADDR_H 0x10
#define CTRL_REG_ADDR_H 0X00
	/*寄存器接口定义*/
#define VERSION 0x0000				//固件版本号
#define LTG_INT 0x0001 				//背光亮度控制 0x00-0x40
#define BUZ_TIME 0x0002				//触控蜂鸣器响起响应时间  单位10ms
#define PIC_ID 0x03						//写：设置当前需要显示图片  读：获取当前显示图片编号
#define TP_FLAG 0x0005					//0xAA-触控坐标更新;其他-触屏坐标未更新
#define TP_STATUS 0x0006				//0x01:第一次按下;0x03:一直按压中;0x02:抬起;其他:无效
#define TP_POSITION 0x000B			//触屏按压位置:X_H,X_L,Y_H,Y_L
#define TPC_ENABLE 0x000B				//0x00:触屏不启用 其他:触屏启用(默认0xFF)
#define PLAY_MUSIC_SET 0xA0

/*数据寄存器定义*/
#define ICON_1 0x0001									//Icon1(图标变量)地址
#define ICON_2 0x0002									//Icon2地址

struct lcm_uart_len_t
{
	uint8_t tx_len;
	uint8_t rx_len;
};

void finger_flexion_and_extension(void);
void function_gloves_exercises(void);
void functional_training(void);
void contralateral_training(void);
void gloves_calibrantion_process(void);
void panel_button_porcess(void);
int check_device_lcm_android(void);
int lcd_read(struct lcm_parameter_t *p,uint32_t millisec);
void setpic(uint16_t pic);
void setled(uint16_t valve);
void write_ctrl_register(const uint8_t list, uint8_t *p, uint16_t *value, struct lcm_uart_len_t *t);
void read_ctrl_register(const uint8_t list, uint8_t *p, struct lcm_uart_len_t *t);
void write_data_registers(const uint8_t list, uint8_t *p, const uint8_t len, uint16_t *value, struct lcm_uart_len_t *t);
void read_data_registers(const uint8_t list, uint8_t *p, const uint8_t len, struct lcm_uart_len_t *t);
void read_data_register(const uint8_t list, uint8_t *p, struct lcm_uart_len_t *t);
void play_music(const uint8_t music, uint8_t *p, struct lcm_uart_len_t *t);
void sy08_pump_intermittent_set(uint32_t mark);


#endif
