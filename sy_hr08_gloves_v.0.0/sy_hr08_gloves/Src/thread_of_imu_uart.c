#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "thread_of_imu_uart.h"
#include "thread_of_host_uart.h"
#include <stdlib.h>
#include <string.h>
#include "uart-API.h"
#include "uart-line-IO.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/

extern ADC_HandleTypeDef hadc1;
__IO float ADC_ConvertedValueLocal[16];
uint32_t ADC_ConvertedValue[5];
uint16_t adc_transformed_resualt[5];

struct sensor_calibration_resualt_t{
	uint16_t max[5];
	uint16_t min[5];
}sensor_calibration_resualt;

void thread_of_imu_uart (void const *argument);                             // thread function
osThreadId tid_thread_of_imu_uart;                                          // thread id
osThreadDef (thread_of_imu_uart, osPriorityNormal, 1, 0);                   // thread object

int init_thread_of_imu_uart (void) {

  tid_thread_of_imu_uart = osThreadCreate (osThread(thread_of_imu_uart), NULL);
  if (!tid_thread_of_imu_uart) return(-1);
  
  return(0);
}

#if 0
void thread_of_imu_uart (void const *argument) {

  while (1) {

  }
}
#endif



#if 1
#define EVENT_LOOP_TIME_IN_MILLI_SECOND 10

uint8_t is_calibration,is_glove_calibration;

void thread_of_imu_uart (void const *argument) {

  while (1) {	
		user_signal_info_t user_timer_info = { osThreadGetId(), SIG_USER_TIMER };
		osTimerDef(main_timer, SetUserSignal);
		osTimerId main_timer_id = osTimerCreate(osTimer(main_timer), osTimerPeriodic, &user_timer_info);
		osTimerStart(main_timer_id, EVENT_LOOP_TIME_IN_MILLI_SECOND);
		osDelay(100);
		struct software_version_t *version = SerialDatagramEvtAlloc(sizeof (*version));
		SERIAL_DATAGRAM_INIT((*version), software_version);
		version->tx_buff[0] = 0x0000;
		version->tx_buff[1] = (uint16_t)((*(__IO uint32_t *)(0X1FFFF7E8))&0x0000ffff);
		version->tx_buff[2] = (uint16_t)(((*(__IO uint32_t *)(0X1FFFF7E8))&0xffff0000)>>16);
		SerialDatagramEvtSend(version);
		osDelay(100);
		
		struct calibration_resualt_t *calibration_resualt_p = SerialDatagramEvtAlloc(sizeof (*calibration_resualt_p));
		SERIAL_DATAGRAM_INIT((*calibration_resualt_p), calibration_resualt);
		
		memset(&sensor_calibration_resualt, 0, sizeof(struct sensor_calibration_resualt_t));
		sensor_calibration_resualt.max[0] = 0x0001;
		
		
		struct calibration_data_t *calibration = SerialDatagramEvtAlloc(sizeof (*calibration));
		SERIAL_DATAGRAM_INIT((*calibration), calibration_data);
		memcpy(calibration->tx_buff, &sensor_calibration_resualt, sizeof(sensor_calibration_resualt));
		SerialDatagramEvtSend(calibration);
		osDelay(100);			
		
		for(;;){
			osEvent event = osSignalWait(0, osWaitForever);
			int32_t signal = event.value.signals;
			if (event.status != osEventSignal) {
				signal = 0;
			}
//			switch (signal) {						//switch后面不能有指针定义？会提示warnning
//				case SIG_USER_0:
//					event = osMailGet(mail_queue_id_for_cmd_calibration_cmd, osWaitForever);
//					struct serial_calibration_cmd_t *cmd = event.value.p;
//					is_calibration = cmd->cmd;				
//				break;
//				
//				case SIG_USER_TIMER:
//				
//				break;
//			
//			}	
			if(signal == SIG_USER_0){
				event = osMailGet(mail_queue_id_for_cmd_calibration_cmd, osWaitForever);
				struct serial_calibration_cmd_t *s = event.value.p;
				is_calibration = s->cmd;				
			}else if(signal == SIG_USER_TIMER){
			
			
			}
			
			HAL_ADC_Start_DMA(&hadc1, ADC_ConvertedValue, 5);
			int i;
			for(i=0;i<5;i++){
				ADC_ConvertedValueLocal[i] =(float)(ADC_ConvertedValue[i]&0xFFF)*3.3/4096;
				adc_transformed_resualt[i] = (uint16_t)(ADC_ConvertedValueLocal[i]*1000);
			}
			struct data_host_uart_tx_t *p = SerialDatagramEvtAlloc(sizeof (*p));			
			if(p){
				SERIAL_DATAGRAM_INIT((*p), data_host_uart_tx);		
				main_data_encode(p, ADC_ConvertedValueLocal);
				SerialDatagramEvtSend(p);
			}
			if(is_calibration == 1){
				glove_calibration_process();
			}else if(is_calibration == 2){
				calibration_resualt_p->tx_buff = glove_calibration_check();
				SerialDatagramEvtSend(calibration_resualt_p);
				osDelay(20);				
				is_glove_calibration = 0;
				memcpy(calibration->tx_buff, &sensor_calibration_resualt, sizeof(sensor_calibration_resualt));
				SerialDatagramEvtSend(calibration);
				osDelay(50);
			}			
		}
  }
}

void main_data_encode(struct data_host_uart_tx_t *p, __IO float *ptr)
{
	uint8_t *s = p->tx_buff;
	uint8_t cnt = 0;
	
	while(s+1 < p->tx_buff + sizeof(p->tx_buff)){
		*s = ((uint16_t)((*(ptr + cnt))*1000))&0x00ff;
		s++;
		*s = (((uint16_t)((*(ptr + cnt))*1000))&0xff00)>>8;
		s++;
		cnt++;
	}
	return;
}

struct sensor_calibration_resualt_t sensor_calibrantion_register;

void glove_calibration_process(void)
{
	if(is_glove_calibration == 0){
		is_glove_calibration = 1;
		memcpy(&sensor_calibrantion_register,&sensor_calibration_resualt,sizeof(struct sensor_calibration_resualt_t));
		
		memcpy(&sensor_calibrantion_register.max,adc_transformed_resualt,sizeof(adc_transformed_resualt));
		memcpy(&sensor_calibrantion_register.min,adc_transformed_resualt,sizeof(adc_transformed_resualt));
	}	

	int i;
	for(i=0;i<5;i++){
		if(sensor_calibrantion_register.max[i] <= adc_transformed_resualt[i]){
			sensor_calibrantion_register.max[i] = adc_transformed_resualt[i];
		}

		if(sensor_calibrantion_register.min[i] >= adc_transformed_resualt[i]){
			sensor_calibrantion_register.min[i] = adc_transformed_resualt[i];
		}
	}
}

int glove_calibration_check(void)
{
	int i;
	for(i=0;i<5;i++){
		if(sensor_calibrantion_register.max[i] - sensor_calibrantion_register.min[i] < 200){
			continue;
		}
		memcpy(&sensor_calibration_resualt,&sensor_calibrantion_register,sizeof(struct sensor_calibration_resualt_t));
		return 1;
	}
	return 0;
}

#endif

