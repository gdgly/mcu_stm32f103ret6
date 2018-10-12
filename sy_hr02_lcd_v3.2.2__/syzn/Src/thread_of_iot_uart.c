#include "stm32f1xx_hal.h"
#include "misc.h"
#include "cmsis_os.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "string.h"
#include "thread_of_LCM_uart.h"

void thread_of_iot_uart (void const *argument);                             // thread function


osThreadId tid_iot_uart;                                          // thread id
osThreadDef (thread_of_iot_uart, osPriorityNormal, 1, 0);                   // thread object


int init_thread_of_iot_uart (void) {

	tid_iot_uart = osThreadCreate (osThread(thread_of_iot_uart), NULL);
	if (!tid_iot_uart) {
		return 0;
	}
  
	return 1;
}


extern osMailQId lcm_to_iot_mail;


void thread_of_iot_uart (void const *argument){
	
	char send_to_iot_char[32];
	uint8_t send_to_iot_buff[64];
	while(1){
		osEvent evt = osMailGet(lcm_to_iot_mail,osWaitForever);
//		struct lcm_to_iot_t *p = osMailAlloc(lcm_to_iot_mail, 0);
		struct lcm_to_iot_t *p = evt.value.p;
		if(evt.status == osEventMail){
			sprintf(send_to_iot_char,"@@event=%u,%d,%d###",p->usage_times,p->hand_state,p->count);
			int ln = strlen(send_to_iot_char);
			memcpy(send_to_iot_buff,(uint8_t*)send_to_iot_char,ln);
			SendDataToUart(4,send_to_iot_buff,ln,20);		
		}
		osMailFree(lcm_to_iot_mail, p);
	}
}

