#include "cmsis_os.h"     // CMSIS RTOS header file
#include <stdlib.h>
#include <string.h>
#include "thread_of_development.h"
#include "thread_of_android_uart.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "IOI2C.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void thread_of_development (void const *argument);                             // thread function
osThreadId tid_thread_of_development;                                          // thread id
osThreadDef (thread_of_development, osPriorityNormal, 1, 0);                   // thread object
extern TIM_HandleTypeDef htim3;
MCP4728_TypeDef dac1;
MCP4728_TypeDef dac2;

//uint8_t timer_cnt = 0;

//osTimerId my_timer_id;
//void my_timer(void const *argument){
//	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
//	timer_cnt ++ ;
//	if(timer_cnt == 10){
//		timer_cnt = 0;
//		osTimerStop(my_timer_id);
//	}
//	return;
//}
//osTimerDef (timer1, my_timer);

int init_thread_of_development(void) {

  tid_thread_of_development = osThreadCreate (osThread(thread_of_development), NULL);
  if (!tid_thread_of_development) return(-1);
  
  return(0);
}

void thread_of_development (void const *argument) {
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
	osDelay(200);
//	dac1.addr = MCP4728_DEVICE_ADDR;
//	dac2.addr = MCP4728_DEVICE_ADDR;
//	MCP4728WriteVref(2, &dac1, 1);
//	MCP4728WriteVref(3, &dac2, 0);
	osDelay(100);
	

//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
//	osDelay(3000);
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	
//	struct mcu_scene_t *mcu_scene_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_scene_evt));
//	if(mcu_scene_evt){
//		ANDROID_DATAGRAM_INIT((*mcu_scene_evt), mcu_scene);				
//		mcu_scene_evt->scene = 0x01;
//		AndroidDatagramEvtSend(mcu_scene_evt);			
//	}	
//  uint8_t cnt;

//	my_timer_id = osTimerCreate(osTimer(timer1), osTimerPeriodic, NULL);
//	osTimerStart(my_timer_id, 1000);
	while(1){
//	cnt ++;
//	struct mcu_hand_angle_t *mcu_angle_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_angle_evt));
//	if(mcu_angle_evt){
//		ANDROID_DATAGRAM_INIT((*mcu_angle_evt), mcu_hand_angle);
//		mcu_angle_evt->angle[0] = 20;
//		mcu_angle_evt->angle[1] = 20;
//		mcu_angle_evt->angle[2] = 20;
//		mcu_angle_evt->angle[3] = 20;
//		mcu_angle_evt->angle[4] = cnt;
//		AndroidDatagramEvtSend(mcu_angle_evt);
//		if(cnt == 100){
//			cnt = 0;
//		}
//	}
//	osDelay(20);	
	
//	struct mcu_speed_t *mcu_speed_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_speed_evt));
//	if(mcu_speed_evt){
//		ANDROID_DATAGRAM_INIT((*mcu_speed_evt), mcu_speed);		
//		mcu_speed_evt->head.cnt = cnt;
//		mcu_speed_evt->speed = 03;
//		AndroidDatagramEvtSend(mcu_speed_evt);
//	}
//	osDelay(20);		
//	
//	struct mcu_version_t *mcu_version_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_version_evt));
//	if(mcu_version_evt){
//		ANDROID_DATAGRAM_INIT((*mcu_version_evt), mcu_version);				
//		mcu_version_evt->head.cnt = cnt;
//		mcu_version_evt->version = 0x1003;
//		AndroidDatagramEvtSend(mcu_version_evt);
//	}
//	osDelay(20);			
//	
//		dac1.dac[0] = 0.4*MCP4728_VREF;
//		dac1.dac[1] = 0.5*MCP4728_VREF;
//		dac1.dac[2] = 0.6*MCP4728_VREF;
//		MCP4728FastWrite(2, &dac1);
//		dac2.dac[0] = 0.99*MCP4728_VREF;
//		dac2.dac[1] = 0.1*MCP4728_VREF;
//		MCP4728FastWrite(3, &dac2);
//		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,2048);
//		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,2048);
//		
//	
		osDelay(2000);
	
	}
//	uint8_t datagram[128];
//	uint8_t buf[128];
//	size_t datagram_len;
//	size_t buf_size;
//	size_t skipped_count;
//	for(;;){
//		while (1) {
//				int ret = get_raw_datagram_from_serial(buf, sizeof buf, &buf_size, &skipped_count);
//				if (!ret) {
//					break;
//				}
//				uint32_t *d = (uint32_t *)datagram;
//				uint8_t *s = buf;
//				while (s < buf + buf_size && d < (uint32_t *)(datagram + sizeof datagram) ) {
//					if (sscanf((char *)s, "%08X", d) != 1) {
//						break;
//					}
//					d++;
//					s+= 9; // format likes '00000000 '
//				}
//				datagram_len = ((uint8_t *)d) - datagram;			
//		}	
//	}

}

void set_valve(void)
{


	return;
}


//int get_raw_datagram_from_serial(uint8_t *raw_datagram, size_t max_size, size_t *actual_size_ptr, size_t *skipped_byte_count_ptr)
//{
//	struct AsyncIoResult_t IoResult = { 0, osThreadGetId() };
//	int i;
//	
//	*skipped_byte_count_ptr = 0;
//	
//	for(i = 0;; i++) {
//		osStatus status = StartUartRx(UART_NO_TO_MAINBOARD, raw_datagram, max_size, SERIAL_DATAGRAM_END_CHR, NotifyAsyncIoFinished, &IoResult);
//		size_t len;
//		uint8_t *start_pos;
//		size_t offset;
//		
//		if (status != osOK) {
//			continue;
//		}
//		osSignalWait(SIG_SERVER_FINISHED, osWaitForever);
//		// TODO: Wait time should not be 'forever'. if it's out of time, should Call StopUartXX.
//		len = IoResult.IoResult;
//		if (len < 2) {
//			*skipped_byte_count_ptr += len;
//			continue;
//		}
//		if (raw_datagram[len - 1] != SERIAL_DATAGRAM_END_CHR) {
//			*skipped_byte_count_ptr += len;
//			continue;
//		}

//		start_pos = memchr(raw_datagram, SERIAL_DATAGRAM_START_CHR, len - 1);
//		if (start_pos == NULL) {
//			*skipped_byte_count_ptr += len;
//			continue;
//		}
//		
//		// we found it.
//		offset = start_pos - raw_datagram;
//		*skipped_byte_count_ptr += offset;
//		*actual_size_ptr = len - 2 - offset;
//		memcpy(raw_datagram, start_pos + 1, *actual_size_ptr);
//		raw_datagram[*actual_size_ptr] = 0;
//		if (i) {
//			// easy to set a breakpoint when debugging.
//			i = 0;
//		}
//		return 1;
//	}
//}

