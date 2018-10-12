/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"     // CMSIS RTOS header file
#include <stdlib.h>
#include <string.h>
#include "device_ctrl.h"




void sy08_pump_set(uint8_t cycle)
{
	uint16_t pmw = 4096 - cycle*0.01*4096;
	SET_MOTOR1_PWM(pmw);
	SET_MOTOR2_PWM(pmw);
	MOTOR1_SET();
	MOTOR2_SET();
}

void sy08_pump_reset(void)
{
	SET_MOTOR1_PWM(4096);
	SET_MOTOR2_PWM(4096);
	MOTOR1_RESET();
	MOTOR2_RESET();
}

void sy08_set_valve(const uint8_t valve)
{
	(valve&0x01)?VALVE_CN5_SET():VALVE_CN5_RESET();

	((valve&0x02)>>1)?VALVE_CN4_SET():VALVE_CN4_RESET();

	((valve&0x04)>>2)?VALVE_CN3_SET():VALVE_CN3_RESET();

	((valve&0x08)>>3)?VALVE_CN2_SET():VALVE_CN2_RESET();	

	((valve&0x10)>>4)?VALVE_CN1_SET():VALVE_CN1_RESET();	
}


