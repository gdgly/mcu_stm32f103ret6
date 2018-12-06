/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"     // CMSIS RTOS header file
#include <stdlib.h>
#include <string.h>
#include "device_ctrl.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "bsp_EEPROM.h"
#include "thread_of_LCM_uart.h"


void sy08_pump_set(uint8_t cycle)
{
	uint16_t pmw = 4096 - cycle*0.01*4096;
	SET_MOTOR1_PWM(pmw);
	SET_MOTOR2_PWM(pmw);
	MOTOR2_RESET();
	MOTOR1_RESET();
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

int get_msg_from_serial(uint8_t No, uint8_t *raw_datagram, size_t max_size, size_t *actual_size_ptr, size_t *skipped_byte_count_ptr)
{
	struct AsyncIoResult_t IoResult = { 0, osThreadGetId() };
	int i;
	
	*skipped_byte_count_ptr = 0;
	
	for(i = 0;; i++) {
		osStatus status = StartUartRx(No, raw_datagram, max_size, SERIAL_DATAGRAM_END_CHR, NotifyAsyncIoFinished, &IoResult);
		size_t len;
		uint8_t *start_pos;
		size_t offset;
		
		if (status != osOK) {
			continue;
		}
		osSignalWait(SIG_SERVER_FINISHED, osWaitForever);
		// TODO: Wait time should not be 'forever'. if it's out of time, should Call StopUartXX.
		len = IoResult.IoResult;
		if (len < 2) {
			*skipped_byte_count_ptr += len;
			continue;
		}
		if (raw_datagram[len - 1] != SERIAL_DATAGRAM_END_CHR) {
			*skipped_byte_count_ptr += len;
			continue;
		}

		start_pos = memchr(raw_datagram, SERIAL_DATAGRAM_START_CHR, len - 1);
		if (start_pos == NULL) {
			*skipped_byte_count_ptr += len;
			continue;
		}
		
		// we found it.
		offset = start_pos - raw_datagram;
		*skipped_byte_count_ptr += offset;
		*actual_size_ptr = len - 2 - offset;
		memcpy(raw_datagram, start_pos + 1, *actual_size_ptr);
		raw_datagram[*actual_size_ptr] = 0;
		if (i) {
			// easy to set a breakpoint when debugging.
			i = 0;
		}
		return 1;
	}
}


int send_raw_datagram_to_serial(uint8_t No, const void *raw_datagram, size_t raw_datagram_len)
{
	struct AsyncIoResult_t IoResult = { 0, osThreadGetId() };
	osStatus status = StartUartTx(No, raw_datagram, raw_datagram_len, NotifyAsyncIoFinished, &IoResult);
	if (status != osOK) {
		return 0;
	}
	osSignalWait(SIG_SERVER_FINISHED, osWaitForever);
	// TODO: Wait time should not be 'forever'. if it's out of time, should Call StopUartXX.
	return 1;
}

//extern uint8_t eeprom_data_list[data_register_list_max];

uint8_t write_data_to_eeprom(void *msg, uint16_t Address, uint8_t msg_len)
{
//	uint8_t buff[64];
//	uint8_t *s = buff;
//	uint16_t *p = msg;

//	while(p < (uint16_t *)msg + msg_len/2){
//		if(eeprom_data_list[p - (uint16_t *)msg]){
//			*s = *p;		
//			s++;			
//		}
//		p++;	
//	}
//	uint8_t len = s - buff;
//	return EEPROM_WriteBytes(buff, Address, len);
	uint8_t *p = msg;
	return EEPROM_WriteBytes(p, Address, msg_len);
}

uint8_t read_data_from_eeprom(void *msg, uint16_t Address, uint8_t msg_len)
{
	uint8_t *p = msg;
	return EEPROM_ReadBytes(p, Address, msg_len);
	return 1;
}


