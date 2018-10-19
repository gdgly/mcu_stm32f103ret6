#ifndef __THREAD_OF_LCM_UART_H
#define __THREAD_OF_LCM_UART_H
#include "stm32f1xx_hal.h"

int init_thread_of_lcm_uart (void);

#define GPIO_LTC_INT GPIO_PIN_12								//电源按键中断
#define GPIO_ENCODE_PB GPIO_PIN_11							//旋钮按键中断
#define GPIO_PANEL_BUTTONS GPIO_PIN_6						//面板按键 74LS148的触发中断

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
	led_set = 0x01,	
	pic_id = 0x03,
};

//屏幕数据寄存器地址
enum lcm_data_register_list{
	start_and_stop = 1,
	motion_time,
	motion_cnt,
	flexion_and_extension_icon,
	gloves_exercises_icon,
	functional_training_icon,
	length_of_training,
	training_mode,
	training_interval,
	training_strength,
	training_hand,
	flexion_and_extension_thumb,
	flexion_and_extension_index_finger,
	flexion_and_extension_middle_finger,
	flexion_and_extension_ring_finger,
	flexion_and_extension_little_thumb,
	flexion_and_extension_all,
	gloves_exercises_index_finger,
	gloves_exercises_middle_finger,
	gloves_exercises_ring_finger,
	gloves_exercises_little_thumb,
	gloves_exercises_all,
	functional_training_catching_ball,
	functional_training_two_fingers,
	functional_training_three_fingers,
	audio_switch,
	volume_set,
	user_led_set,
};


//图片编号
enum lcm_pic_id_t{
	boot_animation_1 = 0,
	boot_animation_2,
	boot_animation_3,
	selection_interface,
	flexion_and_extension_interface,
	gloves_exercises_interface,
	contralateral_training_interface,
	functional_training_interface,
	settings_interface,
	calibration_interface,
	gloves_is_connected,
	gloves_not_connected,
	calibration_successed,
	calibration_failed,
	motion_ending,
};

//defualt valve for lcm data register
//1 - 27 is in use
#define LCM_DATA_REGISTER_DEFUALTS {0,1,0,0,0,0,0,10,0,4,2,0,1,1,1,1,1,0,1,1,0,1,1,0,0,0,0,0,10}

#define REGISTER_ADDR  0
#define REGISTER_LEN  1

/*数据格式定义*/
#define WR_CTL_REG 0x80				//写控制寄存器
#define RD_CTL_REG 0x81				//读控制寄存器
#define WR_DATA_REG 0x82			//写数据寄存器
#define RD_DATA_REG 0x83			//读数据寄存器
#define FRAME_HEAD_H 0xA5
#define FRAME_HEAD_L 0x5A			//数据帧头为A55A
/*寄存器接口定义*/
#define VERSION 0x0000				//固件版本号
#define LTG_INT 0x0001 				//背光亮度控制 0x00-0x40
#define BUZ_TIME 0x0002				//触控蜂鸣器响起响应时间  单位10ms
#define PIC_ID 0x03						//写：设置当前需要显示图片  读：获取当前显示图片编号
#define TP_FLAG 0x0005					//0xAA-触控坐标更新;其他-触屏坐标未更新
#define TP_STATUS 0x0006				//0x01:第一次按下;0x03:一直按压中;0x02:抬起;其他:无效
#define TP_POSITION 0x000B			//触屏按压位置:X_H,X_L,Y_H,Y_L
#define TPC_ENABLE 0x000B				//0x00:触屏不启用 其他:触屏启用(默认0xFF)
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

int lcd_read(struct lcm_parameter_t *p,uint32_t millisec);
void setpic(uint16_t pic);
void write_ctrl_register(const uint8_t list, uint8_t *p, uint16_t *value, struct lcm_uart_len_t *t);
void read_ctrl_register(const uint8_t list, uint8_t *p, struct lcm_uart_len_t *t);
void write_data_registers(const uint8_t list, uint8_t *p, const uint8_t len, uint16_t *value, struct lcm_uart_len_t *t);
void read_data_registers(const uint8_t list, uint8_t *p, const uint8_t len, struct lcm_uart_len_t *t);
void read_data_register(const uint8_t list, uint8_t *p, struct lcm_uart_len_t *t);
#endif
