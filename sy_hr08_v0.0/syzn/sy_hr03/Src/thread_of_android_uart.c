#include "cmsis_os.h"     // CMSIS RTOS header file
#include <stdlib.h>
#include <string.h>
#include "thread_of_android_uart.h"
#include "thread_of_sensor_glove.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "device_ctrl.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
extern struct sensor_glove_para_t sensor_glove_para;
struct android_share_t android_share;
extern uint32_t gloves_access_mark;
extern osMutexId glove_sensor_mutex_id;

osMailQDef(sensor_android_tx, 30, uint8_t[MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT]); 
static osMailQId mail_queue_id_for_android_tx; 

void thread_of_android_rx (void const *argument);                             // thread function
osThreadId tid_thread_of_android_rx;                                          // thread id
osThreadDef (thread_of_android_rx, osPriorityNormal, 1, 0);                   // thread object

void thread_of_android_tx (void const *argument);                             // thread function
osThreadId tid_thread_of_android_tx;                                          // thread id
osThreadDef (thread_of_android_tx, osPriorityNormal, 1, 0);                   // thread object

int init_tid_thread_of_android_rx (void) {

  tid_thread_of_android_rx = osThreadCreate (osThread(thread_of_android_rx), NULL);
  if (!tid_thread_of_android_rx) return(-1);
  
  return(0);
}

int init_tid_thread_of_android_tx (void) {

	mail_queue_id_for_android_tx = osMailCreate(osMailQ(sensor_android_tx), NULL);
	
  tid_thread_of_android_tx = osThreadCreate (osThread(thread_of_android_tx), NULL);
  if (!tid_thread_of_android_tx) return(-1);
  
  return(0);
}

void thread_of_android_rx (void const *argument) {
	
	osSignalWait(SIG_USER_2,osWaitForever);
  
	while (1) {
		uint8_t datagram[128];
		uint8_t buf[128];
		size_t datagram_len;
		size_t buf_size;
		size_t skipped_count;                                         // suspend thread
		
		while (1) {
			int ret = get_msg_from_serial(ANDROID_UART_NO, buf, sizeof buf, &buf_size, &skipped_count);
			if (!ret) {
				break;
			}
			uint8_t *d = (uint8_t *)datagram;
			uint8_t *s = buf;
		
			while (s < buf + buf_size && d < (uint8_t *)(datagram + sizeof datagram) ) {
				if (sscanf((char *)s, "%02X", (uint32_t *)d) != 1) {
					break;
				}
				d++;
				
				s+= 3; // format likes '00000000'
			}
			datagram_len = ((uint8_t *)d) - datagram;
			
			android_rev_process(datagram, datagram_len);
//			for(i=0;i<1;i++){
//				if(head->type == msg_process_func_list[i].id){
//					serial_datagram_msg_process_common_func(datagram, datagram_len, msg_process_func_list[i].ptr);
//				}			
//			}
		}
  }
}

void thread_of_android_tx (void const *argument) {
	
	osSignalWait(SIG_USER_2,osWaitForever);

  while (1) {
		for(;;){
			uint8_t len;
			osEvent evt = osMailGet(mail_queue_id_for_android_tx, osWaitForever);
			struct android_head_t *head = evt.value.p;
			if (evt.status == osEventMail && head != NULL) {
				len = head->body_len;
				if (len <= MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT) {
					mcu_to_android_datagram_send(head, len);
				}			
			}
			osMailFree(mail_queue_id_for_android_tx, head);
		}	
  }
}

void *AndroidDatagramEvtAlloc(size_t size)
{
	void *ret = NULL;
	if (size <= MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT) {
		ret = osMailAlloc(mail_queue_id_for_android_tx, 0);
	}
	if (!ret) {
		return NULL;
	}
	return ret;
}

int AndroidDatagramEvtSend(void *ptr)
{
	osStatus status = osMailPut(mail_queue_id_for_android_tx, ptr);
	return status == osOK;
}

void AndroidDatagramEvtFree(void *ptr)
{
	osMailFree(mail_queue_id_for_android_tx, ptr);
}


int mcu_to_android_datagram_send(void *msg, const size_t msg_len)
{
	uint8_t buf[128];
	uint8_t *p = buf;
	int ret;	
	
//	struct serial_datagram_head_t *head = msg;
	
//	*p++ = SERIAL_DATAGRAM_START_CHR;
	uint8_t *s = msg;
//	size_t len = 0;
//	while ((uint8_t *) s < (uint8_t *) msg + msg_len && p < buf + (sizeof buf) - 3) {
//		len = sprintf((char *) p, "%02X ", *s);
//		s++;
//		p += len;
//	}
	
	memcpy(p, s, msg_len);
//	uint32_t *s = msg;
//	while ((uint8_t *) s < (uint8_t *) msg + msg_len && p < buf + (sizeof buf) - 9) { // 9 is length of a text word
//		size_t len = sprintf((char *) p, "%08X ", *s);
//		s++;
//		p += len;
//	}
//	if ((uint8_t *) s != (uint8_t *) msg + msg_len) {
//		return 0; // too long datagram
//	}	
//	len = sprintf((char *) p, "%02X", 0xff);
//	p += len;
//	*p++ = SERIAL_DATAGRAM_END_CHR;
	*(p + msg_len) = 0xff;
	ret = send_raw_datagram_to_serial(ANDROID_UART_NO, buf, msg_len + 1);
	return ret;
}

uint16_t motor_pwm = 2;

void android_rev_process(const void *msg, size_t msg_len)
{
//	int i;
	const struct android_head_t *head = msg;
	if(head->type == SetResetMCU){
		
	}else if(head->type == SetShutDownAndroid){
		DEVCIE_12V_RESET();
	}else if(head->type == SetHandAngle){
		const struct android_SetHandAngle_t *hand_angle_cmd = msg;
		uint8_t valve = 0x00;

		if(hand_angle_cmd->angle[0]< 60){
			valve = (VALVE_SET)|valve;
		}else if(hand_angle_cmd->angle[0] < 120){
			
		}else if(hand_angle_cmd->angle[0] < 180){
			valve = (VALVE_RESET)|valve;
		}else{
			valve = (VALVE_RESET)|valve;
		}	
		int i;
		for(i=1;i<5;i++){
			if(hand_angle_cmd->angle[0] < 30){
				valve = (VALVE_SET<<i)|valve;
			}else if(hand_angle_cmd->angle[0] < 120){
				
			}else if(hand_angle_cmd->angle[0] < 180){
				valve = (VALVE_RESET<<i)|valve;
			}else{
				valve = (VALVE_RESET<<i)|valve;
			}	
		}
		sy08_set_valve(valve);	
		
		
	}else if(head->type == SetHandSpeed){
		osDelay(5);
		const struct android_SetHandSpeed_t *hand_speed_cmd = msg;
		struct mcu_speed_t *hand_speed_evt = AndroidDatagramEvtAlloc(sizeof (*hand_speed_evt));
		ANDROID_DATAGRAM_INIT((*hand_speed_evt), mcu_speed);
		hand_speed_evt->speed = hand_speed_cmd->speed;
		AndroidDatagramEvtSend(hand_speed_evt);			
		motor_pwm = hand_speed_cmd->speed;
		
	}else if(head->type == SetScene){
		
		const struct android_SetScene_t *hand_scene_cmd = msg;
		struct mcu_scene_t *hand_scene_evt = AndroidDatagramEvtAlloc(sizeof (*hand_scene_evt));
		ANDROID_DATAGRAM_INIT((*hand_scene_evt), mcu_scene);
		hand_scene_evt->scene = hand_scene_cmd->scene;
		AndroidDatagramEvtSend(hand_scene_evt);
		
		android_share.scene = hand_scene_cmd->scene;
		switch(hand_scene_cmd->scene){
			case 0:
				sy08_pump_reset();
				sy08_set_valve(0);
			break;
			case 1:

			break;
			case 2:
				sy08_pump_set(70 + 10*(4 - motor_pwm));
			break;
			case 3:
				sy08_pump_set(70 + 10*(4 - motor_pwm));
			break;
			case 4:
				sy08_pump_set(70 + 10*(4 - motor_pwm));
			break;
			case 5:

			break;
			case 6:
				
			break;
			
			default:
				sy08_pump_reset();
				sy08_set_valve(0);
			break;
		}				
		
	}else if(head->type == SetProof){
		
		const struct android_SetProof_t *glove_proof_cmd = msg;
		struct calibration_cmd_t *p = SerialDatagramEvtAlloc(sizeof (*p));  
		if(p){
			SERIAL_DATAGRAM_INIT((*p), calibration_cmd);
			p->cmd = glove_proof_cmd->proof;
			SerialDatagramEvtSend(p);
		}
		
//		if(glove_proof_cmd->proof == 2){
//			uint8_t resualt = 0;
//			osMutexWait(glove_sensor_mutex_id,osWaitForever);
//			resualt = sensor_glove_para.calibration_resualt;
//			osMutexRelease(glove_sensor_mutex_id);			

//			struct mcu_proof_t *glove_proof_evt = AndroidDatagramEvtAlloc(sizeof (*glove_proof_evt));
//			if(glove_proof_evt){
//				ANDROID_DATAGRAM_INIT((*glove_proof_evt), mcu_proof);
//				if(resualt == 1){
//					glove_proof_evt->proof = 2;
//				}else if(resualt == 2){
//					glove_proof_evt->proof = 1;
//				}else if(resualt == 0){
//					glove_proof_evt->proof = 2;
//				}
//				AndroidDatagramEvtSend(glove_proof_evt);
//			}
//		}
	}else if(head->type == SetStartStop){
		const struct android_SetStartStop_t *start_stop_cmd = msg;
		if(start_stop_cmd->start_stop_pb == 0){
			sy08_pump_reset();
		}else if(start_stop_cmd->start_stop_pb == 1){
			sy08_pump_set(70 + 10*(4 - motor_pwm));
		}
		android_share.start_stop = start_stop_cmd->start_stop_pb;
		
	}else if(head->type == GetError){

		struct mcu_run_info_t *hand_error_evt = AndroidDatagramEvtAlloc(sizeof (*hand_error_evt));
		ANDROID_DATAGRAM_INIT((*hand_error_evt), mcu_run_info);
		hand_error_evt->error = 0xff;
		AndroidDatagramEvtSend(hand_error_evt);		
		
	}else if(head->type == GetVersion){

		struct mcu_version_t *mcu_version_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_version_evt));
		ANDROID_DATAGRAM_INIT((*mcu_version_evt), mcu_version);
		mcu_version_evt->version = 0x00ff;
		AndroidDatagramEvtSend(mcu_version_evt);
		
	}else if(head->type == GetUsefulLife){
	
		struct mcu_version_t *mcu_usefullife_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_usefullife_evt));
		ANDROID_DATAGRAM_INIT((*mcu_usefullife_evt), mcu_usefullife);
		mcu_usefullife_evt->version = 0xff;
		AndroidDatagramEvtSend(mcu_usefullife_evt);		
		
	}else if(head->type == GetTemperatur){
		
		struct mcu_temperature_t *mcu_temperature_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_temperature_evt));
		ANDROID_DATAGRAM_INIT((*mcu_temperature_evt), mcu_temperature);
		mcu_temperature_evt->temperature = 0x00ff;
		AndroidDatagramEvtSend(mcu_temperature_evt);	
		
	}else if(head->type == GetMCU_ID){
		
		struct mcu_machinary_id_t *mcu_machinary_id_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_machinary_id_evt));
		ANDROID_DATAGRAM_INIT((*mcu_machinary_id_evt), mcu_machinary_id);
		mcu_machinary_id_evt->machinary_id = 0x000000ff;
		AndroidDatagramEvtSend(mcu_machinary_id_evt);		
		
	}else if(head->type == GetHandAngle){
		
		struct mcu_hand_angle_t *mcu_hand_angle_evt = AndroidDatagramEvtAlloc(sizeof (*mcu_hand_angle_evt));
		ANDROID_DATAGRAM_INIT((*mcu_hand_angle_evt), mcu_machinary_id);
		mcu_hand_angle_evt->mode = 0x02;
		AndroidDatagramEvtSend(mcu_hand_angle_evt);	
		
	}else if(head->type == GetHandError){
		
		struct glove_run_info_t *glove_run_info_evt = AndroidDatagramEvtAlloc(sizeof (*glove_run_info_evt));
		ANDROID_DATAGRAM_INIT((*glove_run_info_evt), glove_run_info);
		glove_run_info_evt->error = 0x00000001;
		AndroidDatagramEvtSend(glove_run_info_evt);
		
	}else if(head->type == GetHandID){
		
		struct glove_machinary_id_t *glove_machinary_id_evt = AndroidDatagramEvtAlloc(sizeof (*glove_machinary_id_evt));
		ANDROID_DATAGRAM_INIT((*glove_machinary_id_evt), glove_machinary_id);
		glove_machinary_id_evt->machinary_id = sensor_glove_para.machinary_id;
		AndroidDatagramEvtSend(glove_machinary_id_evt);	
		
	}else if(head->type == GetHandVersion){
		
		struct glove_version_t *glove_version_evt = AndroidDatagramEvtAlloc(sizeof (*glove_version_evt));
		ANDROID_DATAGRAM_INIT((*glove_version_evt), glove_version);
		glove_version_evt->version = sensor_glove_para.version;
		AndroidDatagramEvtSend(glove_version_evt);	
		
	}else if(head->type == GetHandType){
		
		struct mcu_hand_type_t *hand_type_evt = AndroidDatagramEvtAlloc(sizeof (*hand_type_evt));
		if(hand_type_evt){
			ANDROID_DATAGRAM_INIT((*hand_type_evt), mcu_hand_type);
			if(HAL_GetTick()	- gloves_access_mark > 200){
				hand_type_evt->left_type = 0;
				hand_type_evt->right_type = 0;
			}else{
				hand_type_evt->left_type = 0x03;
				hand_type_evt->right_type = 0x00;
			}
			AndroidDatagramEvtSend(hand_type_evt);		
		}
	
		
	}else{
	
	}
}

