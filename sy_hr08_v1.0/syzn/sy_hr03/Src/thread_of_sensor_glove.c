#include "cmsis_os.h"     // CMSIS RTOS header file
#include <stdlib.h>
#include <string.h>
#include "thread_of_sensor_glove.h"
#include "thread_of_android_uart.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "device_ctrl.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
osMailQDef(sensor_glove_tx, 30, uint8_t[MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT]); 
static osMailQId mail_queue_id_for_glove_tx; 
 
void thread_of_sensor_glove_rx (void const *argument);                             // thread function
osThreadId tid_thread_of_sensor_glove_rx;                                          // thread id
osThreadDef (thread_of_sensor_glove_rx, osPriorityNormal, 1, 0);                   // thread object

void thread_of_sensor_glove_tx (void const *argument);                             // thread function
osThreadId tid_thread_of_sensor_glove_tx;                                          // thread id
osThreadDef (thread_of_sensor_glove_tx, osPriorityNormal, 1, 0);                   // thread object

extern osMutexId glove_sensor_mutex_id;

struct sensor_glove_para_t sensor_glove_para;
static void calculation_glove_degree(const uint16_t *adc);

uint32_t gloves_access_mark;
extern uint8_t lcm_type;
extern struct android_share_t android_share;


int init_thread_of_sensor_glove_rx (void) {

  tid_thread_of_sensor_glove_rx = osThreadCreate (osThread(thread_of_sensor_glove_rx), NULL);
  if (!tid_thread_of_sensor_glove_rx) return(-1);
  
  return(0);
}

int init_thread_of_sensor_glove_tx (void) {

	mail_queue_id_for_glove_tx = osMailCreate(osMailQ(sensor_glove_tx), NULL);
	
  tid_thread_of_sensor_glove_tx = osThreadCreate (osThread(thread_of_sensor_glove_tx), NULL);
  if (!tid_thread_of_sensor_glove_tx) return(-1);
  
  return(0);
}

void thread_of_sensor_glove_rx (void const *argument) {

  while (1) {
		uint8_t datagram[128];
		uint8_t buf[128];
		size_t datagram_len;
		size_t buf_size;
		size_t skipped_count;                                         // suspend thread
		while (1) {
			int ret = get_msg_from_serial(GLOVES_UART_NO, buf, sizeof buf, &buf_size, &skipped_count);
			if (!ret) {
				break;
			}
			gloves_access_mark = HAL_GetTick();
			
			uint16_t *d = (uint16_t *)datagram;
			uint8_t *s = buf;
			
			if (sscanf((char *)s, "%02X", (uint32_t *)d) != 1) {
				break;
			}
			d++;
			s+= 3;
			while (s < buf + buf_size && d < (uint16_t *)(datagram + sizeof datagram) ) {
				if (sscanf((char *)s, "%04X", (uint32_t *)d) != 1) {
					break;
				}
				d++;
				s+= 5; // format likes '00000000'
			}
			datagram_len = ((uint8_t *)d) - datagram;
			
			update_sensor_glove_para(datagram, datagram_len);
//			for(i=0;i<1;i++){
//				if(head->type == msg_process_func_list[i].id){
//					serial_datagram_msg_process_common_func(datagram, datagram_len, msg_process_func_list[i].ptr);
//				}			
//			}
		}
  }
}

void thread_of_sensor_glove_tx (void const *argument) {
	while (1) {
		for(;;){
			uint8_t len;
			osEvent evt = osMailGet(mail_queue_id_for_glove_tx, osWaitForever);
			struct uart_head_t *head = evt.value.p;
			if (evt.status == osEventMail && head != NULL) {
				len = head->body_len;
				if (len <= MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT) {
					sensor_glove_datagram_send(head, len);
				}			
			}
			osMailFree(mail_queue_id_for_glove_tx, head);
		}		
	}
}

void *SerialDatagramEvtAlloc(size_t size)
{
	void *ret = NULL;
	if (size <= MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT) {
		ret = osMailAlloc(mail_queue_id_for_glove_tx, 0);
	}
	if (!ret) {
		return NULL;
	}
	return ret;
}

int SerialDatagramEvtSend(void *ptr)
{
	osStatus status = osMailPut(mail_queue_id_for_glove_tx, ptr);
	return status == osOK;
}

void SerialDatagramEvtFree(void *ptr)
{
	osMailFree(mail_queue_id_for_glove_tx, ptr);
}

int sensor_glove_datagram_send(void *msg, const size_t msg_len)
{
	uint8_t buf[128];
	uint8_t *p = buf;
	int ret;	
	
//	struct serial_datagram_head_t *head = msg;
	
	*p++ = SERIAL_DATAGRAM_START_CHR;
	uint8_t *s = msg;
	size_t len = sprintf((char *) p, "%02X ", *s);
	p += len;
	s++;
	s++;
	while ((uint8_t *) s < (uint8_t *) msg + msg_len && p < buf + (sizeof buf) - 9) {
		len = sprintf((char *) p, "%02X", *(s+1));
		p += len;
		len = sprintf((char *) p, "%02X ", *s);
		s += 2;
		p += len;
	}
	
	
//	uint32_t *s = msg;
//	while ((uint8_t *) s < (uint8_t *) msg + msg_len && p < buf + (sizeof buf) - 9) { // 9 is length of a text word
//		size_t len = sprintf((char *) p, "%08X ", *s);
//		s++;
//		p += len;
//	}
	if ((uint8_t *) s != (uint8_t *) msg + msg_len) {
		return 0; // too long datagram
	}	
	p--;
	*p++ = SERIAL_DATAGRAM_END_CHR;	
	
	ret = send_raw_datagram_to_serial(GLOVES_UART_NO, buf, p - buf);
	return ret;
}



uint16_t sensor_adc_standard[5];
uint16_t sensor_adc_width[5];

void update_sensor_glove_para(const void *msg, size_t msg_len)
{
	int i;
	const struct uart_head_t *head = msg;
	if(head->type == data_host_uart_tx){
		const struct data_host_uart_rx_t *adc_now = msg;
		calculation_glove_degree(adc_now->glove_adc);
	}else if(head->type == calibration_resualt){
		osMutexWait(glove_sensor_mutex_id,osWaitForever);
		const struct calibration_resualt_t *calibration_resualt = msg;			
		sensor_glove_para.calibration_resualt = calibration_resualt->resualt;
		osMutexRelease(glove_sensor_mutex_id);
		
		struct mcu_proof_t *glove_proof_evt = AndroidDatagramEvtAlloc(sizeof (*glove_proof_evt));
		if(glove_proof_evt){
			ANDROID_DATAGRAM_INIT((*glove_proof_evt), mcu_proof);
			if(calibration_resualt->resualt == 1){
				glove_proof_evt->proof = 2;
			}else if(calibration_resualt->resualt == 2){
				glove_proof_evt->proof = 1;
			}else if(calibration_resualt->resualt == 0){
				glove_proof_evt->proof = 2;
			}
			AndroidDatagramEvtSend(glove_proof_evt);
		}
		
	}else if(head->type == calibration_data){
		const struct calibration_data_t *calibration_data = msg;
		for(i=0;i<5;i++){
			sensor_adc_standard[i] = calibration_data->min[i];
			sensor_adc_width[i] = calibration_data->max[i] - calibration_data->min[i];
		}
	}else if(head->type == software_version){
		osMutexWait(glove_sensor_mutex_id,osWaitForever);
		const struct software_version_t *version = msg;
		sensor_glove_para.machinary_id = version->machinary_id;
		sensor_glove_para.version = version->version;
		osMutexRelease(glove_sensor_mutex_id);
		struct mcu_hand_type_t *mcu_hand_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_hand_evt));
		if(mcu_hand_evt){
			ANDROID_DATAGRAM_INIT((*mcu_hand_evt), mcu_hand_type);
			mcu_hand_evt->left_type = 0x03;
			mcu_hand_evt->right_type = 0x00;
			AndroidDatagramEvtSend(mcu_hand_evt);
		}
	}
}

uint8_t degree_cnt = 0;
static void calculation_glove_degree(const uint16_t *adc)
{
	int i;
	osMutexWait(glove_sensor_mutex_id,osWaitForever);
  for(i=0;i<5;i++){
		sensor_glove_para.degree[i] = (uint16_t)(((*(adc + i) - sensor_adc_standard[i])*180)/sensor_adc_width[i]);	
	}
	osMutexRelease(glove_sensor_mutex_id);
	if(lcm_type == DEVICE_ANDROID){
		if(android_share.scene == 2){
			if(degree_cnt == 2){
				struct mcu_noti_hand_angle_t *mcu_noti_hand_angle_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_noti_hand_angle_evt));
				if(mcu_noti_hand_angle_evt){
					ANDROID_DATAGRAM_INIT((*mcu_noti_hand_angle_evt), mcu_noti_hand_angle);
					mcu_noti_hand_angle_evt->angle[0] =180 - sensor_glove_para.degree[0];
					mcu_noti_hand_angle_evt->angle[1] =180 - sensor_glove_para.degree[1];
					mcu_noti_hand_angle_evt->angle[2] =180 - sensor_glove_para.degree[2];
					mcu_noti_hand_angle_evt->angle[3] =180 - sensor_glove_para.degree[3];
					mcu_noti_hand_angle_evt->angle[4] =180 - sensor_glove_para.degree[4];
					AndroidDatagramEvtSend(mcu_noti_hand_angle_evt);		
				}			
			}else if(degree_cnt == 3){
				degree_cnt = 0;
			}else{
				degree_cnt++;
			}
		}
	}	
}



