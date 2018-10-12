#include "stm32f1xx_hal.h"


int init_thread_of_LCM_uart (void);
int init_thread_of_iot_uart (void);

enum pic_id_m{
	boot_animation_1 = 0,
	boot_animation_2,
	boot_animation_3,
	selection_interface,
	selection_interface_fake = 6,
	automatic_mode,
	automatic_motion_1,
	automatic_motion_2,
	manual_mode,
	manual_motion_1,
	manual_motion_2,
	calibration_animation_1,
	calibration_animation_2,
	calibration_animation_3,
	calibration_animation_4,
	calibration_animation_5,
	calibration_animation_6,
	gloves_not_connected,
	calibration_failed,
	calibration_successed,
	motion_ending,	
};


struct lcm_to_iot_t{
	uint32_t usage_times;
  int hand_state;
  uint32_t count;
};

struct user_common_t{
	uint8_t pic_id;
	uint16_t all_times;
	uint16_t motion_time;
	uint16_t period;											//自动模式下的运行周期
	int16_t neg_pressure;
	int16_t pos_pressure;
};

struct lcm_parameter_t{
	struct user_common_t common;
	uint16_t current_times;
	uint16_t treatment_time;
	uint16_t reset_sys;										//复位量
	uint8_t hand_state;
	uint32_t time_now[4];
	uint32_t time_old[4];
};


struct eeprom_parameter_t{
	struct user_common_t common;	
	uint8_t data[64];
	uint8_t buff[64];
	uint32_t usage_times;
};

struct mutex_parameter_t{
	uint8_t mode;
	uint8_t direction;
	uint8_t curvature;
	uint8_t is_reset;
};

struct glove_adc_press_t{
	uint16_t adc1_end;
	uint16_t adc2_start;
	uint16_t adc2_end;
	uint16_t adc3_start;
	uint16_t press1;
	uint16_t press2;
	uint16_t press3;
};


#define EEPROM_HEAD_S 0x53
#define EEPROM_HEAD_Y 0x59
#define EEPROM_BUFF_SIZE 32

/*数据格式定义*/
#define WR_CTL_REG 0x80				//写控制寄存器
#define RD_CTL_REG 0x81				//读控制寄存器
#define WR_DATA_REG 0x82			//写数据寄存器
#define RD_DATA_REG 0x83			//读数据寄存器
#define FRAME_HEAD_H 0xA5
#define FRAME_HEAD_L 0x5A			//数据帧头为5AA5
/*寄存器接口定义*/
#define VERSION 0x0000				//固件版本号
#define LTG_INT 0x0001 					//背光亮度控制 0x00-0x40
#define BUZ_TIME 0x0002					//触控蜂鸣器想起时间寄存器  单位10ms
#define PIC_ID 0x03						//写：设置当前所需要显示的图片编号，读：获取当前显示的图片编号。
#define TP_FLAG 0x0005					//0xAA-触屏坐标有更新；其他-触屏坐标未更新
#define TP_STATUS 0x0006				//0x01:第一次按下；0x03:一直按压中；0x02:抬起；其他：无效
#define TP_POSITION 0x000B			//触摸屏按压位置：X_H,X_L,Y_H,Y_L
#define TPC_ENABLE 0x000B				//0x00:触屏不启用 其他：触屏启用（默认0xFF）
/*变量地址*/
#define ALL_TIMES 0x0014				//全部次数
#define ALL_TIMES_RD1 0x0015		//全部次数
#define ALL_TIMES_RD2 0x0015		//全部次数
#define CURRENT_TIMES_RD1 0x0013		//当前次数
#define CURRENT_TIMES_RD2 0x0013		//当前次数
#define PERIOD		0X0017					//运行周期	
#define MOTION_TIME     0x0018
#define TREATMENT_TIME 0x0019		//当前治疗时间

#define PERIOD_DEFAULT	0X0006				//自动模式的运行周期默认值   6
#define ALL_TIMES_DEFAULT 0X00C8      //总次数默认200 =0xC8
#define USAGE_TIMES_DEFAULT 0
#define NEG_PRESSURE_DEFAULT 0XFFBA   //0XFFBA=-70
#define POS_PRESSURE_DEFAULT 0X0078   //0X78=120

#define NEG_PRESSURE 0x0012
#define POS_PRESSURE 0x0016

#define ADDR_MECHINE_NUM1 0x0020
#define ADDR_MECHINE_NUM2 0x0021
#define ADDR_VERSION_NUM  0X0022

#define mechine_num1 0x00B4
#define mechine_num2 0x1B5D
#define version_num  0x0031


#define ADDR_ADC1_END 0x0025
#define ADDR_ADC2_START 0x0026
#define ADDR_ADC2_END 0x0027
#define ADDR_ADC3_START 0x0028
#define ADDR_PRESS1 0x0029
#define ADDR_PRESS2 0x002A
#define ADDR_PRESS3 0x002B

#define DATABASE_ADDR_H 0x0048
#define DATABASE_ADDR_L 0x0000

//#define Vacuum_Pump_Set() \
//__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);\
//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET)			//PB0 控制继电器->控制泵

//#define Vacuum_Pump_Reset() \
//__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,4096);\
//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET)

#define Vacuum_Pump_Toggle() HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_11)

#define Vacuum_Valve_Set() \
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);\
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)

#define Vacuum_Valve_Reset() \
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);\
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)

#define	Vacuum_Valve_Toggle() \
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);\
HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5)

#define Vacuum_Valve_stop() \
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);\
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)

#define LED3_Toggle() HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10)

void Vacuum_Pump_Set(uint8_t cycle);
void Vacuum_Pump_Reset(void);

int lcd_read(struct lcm_parameter_t *p,uint32_t millisec);
void setpic(uint8_t pic);
void lcm_setvalue(uint16_t addr, uint16_t val );

void EEPROM_init(void);
void Lcd_init(void);
void update_eeprom(void);
void check_start_stop_button(void);
void clear_motion_cnt(void);
void update_usage_times(void);
void motion_auto_mode(const uint8_t period, const uint8_t direction);
void motion_hand_mode(const uint8_t direction);
void motion_gloves_mode(void);
void motion_reset_mode(void);
void motion_transition_mode1(void);
void motion_transition_mode2(void);
void set_valve(const uint8_t direction, const int pressure, const uint32_t mark);
void air_pressure_init(void);
int get_air_pressure(void);
void send_to_iot(void);
void set_valve_pid(int err);
void sy02_set_valve(const uint8_t valve1, const uint8_t valve2);
void read_glove_adc_press(void);
void eeprom_glove_adc_press(void);
void restore_factory_setting(void);

