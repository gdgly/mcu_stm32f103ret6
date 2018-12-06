#ifndef __DEVICE_CTRL_H
#define __DEVICE_CTRL_H
#include "stm32f1xx_hal.h"


// datagram description
#define SERIAL_DATAGRAM_START_CHR '\r'
#define SERIAL_DATAGRAM_END_CHR   '\n'

#define MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT 128

#define DEVICE_DIWEN 1
#define DEVICE_ANDROID 2

extern TIM_HandleTypeDef htim3;

#define SET_MOTOR1_PWM(pmw)      __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,pmw);   
#define SET_MOTOR2_PWM(pmw)      __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,pmw);

#define DEVICE_5V_SET()          HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET)
#define DEVICE_5V_RESET()        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET)

#define MCU_KILL_SET()           HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET)
#define MCU_KILL_RESET()         HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET)

#define PWR_12V_SET()            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET)
#define PWR_12V_RESET()          HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET)

#define DEVICE_12V_SET()         HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET)
#define DEVCIE_12V_RESET()       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET)

#define MOTOR1_SET()             HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)
#define MOTOR1_RESET()           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)

#define MOTOR2_SET()             HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define MOTOR2_RESET()           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)

#define LED_PWR_SET()            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)
#define LED_PWR_RESET()          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)

#define LED_YLD_SET()            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET)
#define LED_YLD_RESET()          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET)

#define LED_RCD_SET()            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)
#define LED_RCD_RESET()          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)

#define LED_LED1_SET()           HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_SET)
#define LED_LED1_RESET()         HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_RESET)

#define LED_LED2_SET()           HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET)
#define LED_LED2_RESET()         HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_RESET)

#define LED_LED3_SET()           HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET)
#define LED_LED3_RESET()         HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET)

#define VALVE_CN1_SET()          HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET)
#define VALVE_CN1_RESET()        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET)

#define VALVE_CN2_SET()          HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET)
#define VALVE_CN2_RESET()        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET)

#define VALVE_CN3_SET()          HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET)
#define VALVE_CN3_RESET()        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET)

#define VALVE_CN4_SET()          HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET)
#define VALVE_CN4_RESET()        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET)

#define VALVE_CN5_SET()          HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET)
#define VALVE_CN5_RESET()        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET)

#define VALVE_CN6_SET()          HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET)
#define VALVE_CN6_RESET()        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET)

#define VALVE_CNREV1_SET()       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET)
#define VALVE_CNREV1_RESET()     HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET)

//#define VALVE_CNREV2_SET       HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET)
//#define VALVE_CNREV2_RESET     HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET)

void sy08_pump_set(uint8_t cycle);
void sy08_pump_reset(void);
//void sy08_set_valve(const uint8_t valve1, const uint8_t valve2, const uint8_t valve3, const uint8_t valve4, const uint8_t valve5);
void sy08_set_valve(const uint8_t valve);

int get_msg_from_serial(uint8_t No, uint8_t *raw_datagram, size_t max_size, size_t *actual_size_ptr, size_t *skipped_byte_count_ptr);
int send_raw_datagram_to_serial(uint8_t No, const void *raw_datagram, size_t raw_datagram_len);
uint8_t read_data_from_eeprom(void *msg, uint16_t Address, uint8_t msg_len);
uint8_t write_data_to_eeprom(void *msg, uint16_t Address, uint8_t msg_len);

#define VALVE_RESET 0
#define VALVE_SET 1

#endif
