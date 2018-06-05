#include "stm32f1xx_hal.h"


int init_thread_of_LCM_uart (void);
int init_thread_of_iot_uart (void);


struct lcm_to_iot_t{
	uint32_t usage_times;
  int hand_state;
  uint32_t count;
};


#define EEPROM_HEAD_S 0x53
#define EEPROM_HEAD_Y 0x59
#define EEPROM_BUFF_SIZE 16

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
#define PERIOD_DEFAULT	0X0006				//自动模式的运行周期默认值   6
//#define PERIOD_DEFAULT_S	0X0001				//手动模式的运行周期默认值  1
#define ALL_TIMES_DEFAULT 0X00C8      //总次数默认200 =0xc8

#define Vacuum_Pump_Set() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET)			//PB0 控制继电器->控制泵
#define Vacuum_Pump_Reset() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)
#define Vacuum_Pump_Toggle() HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0)

#define Vacuum_Valve_Set() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET)				//PC5 控制阀门切换
#define Vacuum_Valve_Reset() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET)
#define	Vacuum_Valve_Toggle() HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5)

#define LED3_Toggle() HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14)





