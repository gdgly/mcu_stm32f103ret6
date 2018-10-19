#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "uart-API.h"
#include "uart-line-IO.h"
#include "thread_of_host_uart.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
#define EVENT_LOOP_TIME_IN_MILLI_SECOND 10

#define MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT 128
osMailQDef(host_uart_tx, 30, uint8_t[MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT]); 
static osMailQId mail_queue_id_for_host_tx;

extern osThreadId tid_thread_of_imu_uart;

DEFINE_MAIL_QUEUE_FOR_SERIAL_DATAGRAM_CMD(calibration_cmd,1);

void thread_of_host_uart_tx (void const *argument);                             // thread function
osThreadId tid_thread_of_host_uart_tx;                                          // thread id
osThreadDef (thread_of_host_uart_tx, osPriorityNormal, 1, 0);                   // thread object

void thread_of_host_uart_rx (void const *argument);                             // thread function
osThreadId tid_thread_of_host_uart_rx;                                          // thread id
osThreadDef (thread_of_host_uart_rx, osPriorityNormal, 1, 0);                   // thread object

#define FUNC_TAB_ITEM(type) {type, sizeof(struct serial_##type##_t) , &mail_queue_id_for_cmd_##type }

static const struct gloves_msg_process_func_t msg_process_func_list[] = {
	FUNC_TAB_ITEM(calibration_cmd),
};


int init_thread_of_host_uart_tx (void) {
	
	mail_queue_id_for_host_tx = osMailCreate(osMailQ(host_uart_tx), NULL);

  tid_thread_of_host_uart_tx = osThreadCreate (osThread(thread_of_host_uart_tx), NULL);
  if (!tid_thread_of_host_uart_tx) return(-1);
  
  return(0);
}


int init_thread_of_host_uart_rx (void) {

	INIT_MAIL_QUEUE_FOR_SERIAL_DATAGRAM_CMD(calibration_cmd);
	
  tid_thread_of_host_uart_rx = osThreadCreate (osThread(thread_of_host_uart_rx), NULL);
  if (!tid_thread_of_host_uart_rx) return(-1);
  
  return(0);
}


//thread for host tx
void thread_of_host_uart_tx (void const *argument) {

  while (1) {
		for(;;){
			uint8_t len;
			osEvent evt = osMailGet(mail_queue_id_for_host_tx, osWaitForever);
			struct uart_head_t *head = evt.value.p;
			int isOK = 0;
			if (evt.status == osEventMail && head != NULL) {
				len = head->body_len;
				if (len <= MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT) {
					isOK = host_uart_datagram_send(head, len);
				}			
			}
			osMailFree(mail_queue_id_for_host_tx, head);
		}
  }
}


//thread for host rx
void thread_of_host_uart_rx (void const *argument) {

  while (1) {
		uint8_t datagram[128];
		uint8_t buf[128];
		size_t datagram_len;
		size_t buf_size;
		size_t skipped_count;                                         // suspend thread
		while (1) {
			int ret = get_raw_datagram_from_serial(buf, sizeof buf, &buf_size, &skipped_count);
			if (!ret) {
				break;
			}
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
				s+= 5; // format likes '00000000 '
			}
			datagram_len = ((uint8_t *)d) - datagram;
			
			size_t i = 0;
			const struct uart_head_t *head = (const struct uart_head_t *)datagram;
			switch(head->type){
				case calibration_cmd:
					osSignalSet(tid_thread_of_imu_uart,SIG_USER_0);
				break;				
			}
			for(i=0;i<1;i++){
				if(head->type == msg_process_func_list[i].id){
					serial_datagram_msg_process_common_func(datagram, datagram_len, msg_process_func_list[i].ptr);
				}			
			}				
		}		
  }
}


void *SerialDatagramEvtAlloc(size_t size)
{
	void *ret = NULL;
	if (size <= MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT) {
		ret = osMailAlloc(mail_queue_id_for_host_tx, 0);
	}
	if (!ret) {
		return NULL;
	}
	return ret;
}

int SerialDatagramEvtSend(void *ptr)
{
	osStatus status = osMailPut(mail_queue_id_for_host_tx, ptr);
	return status == osOK;
}

void SerialDatagramEvtFree(void *ptr)
{
	osMailFree(mail_queue_id_for_host_tx, ptr);
}

int host_uart_datagram_send(void *msg, const size_t msg_len)
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
	
	ret = send_raw_datagram_to_serial(buf, p - buf);
	return ret;
}


int send_raw_datagram_to_serial(const void *raw_datagram, size_t raw_datagram_len)
{
	struct AsyncIoResult_t IoResult = { 0, osThreadGetId() };
	osStatus status = StartUartTx(1, raw_datagram, raw_datagram_len, NotifyAsyncIoFinished, &IoResult);
	if (status != osOK) {
		return 0;
	}
	osSignalWait(SIG_SERVER_FINISHED, osWaitForever);
	// TODO: Wait time should not be 'forever'. if it's out of time, should Call StopUartXX.
	return 1;
}

int get_raw_datagram_from_serial(uint8_t *raw_datagram, size_t max_size, size_t *actual_size_ptr, size_t *skipped_byte_count_ptr)
{
	struct AsyncIoResult_t IoResult = { 0, osThreadGetId() };
	int i;
	
	*skipped_byte_count_ptr = 0;
	
	for(i = 0;; i++) {
		osStatus status = StartUartRx(1, raw_datagram, max_size, SERIAL_DATAGRAM_END_CHR, NotifyAsyncIoFinished, &IoResult);
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

static int serial_datagram_msg_process_common_func(const void *msg, size_t msg_len, const void *ptr)
{
	// every type of datagram has a mail-queue, then we should put msg into it
	const osMailQId *mq_ptr = ptr;
	osMailQId mq = *mq_ptr;
	void *msg_copy = osMailAlloc(mq, 0);
	if (msg_copy == NULL) {

		// try to discard the 1st command in the queue
		osEvent evt = osMailGet(mq, 0);
		if (evt.status == osEventMail && evt.value.p != NULL) {
			osMailFree(mq, evt.value.p);
		}
		// and re-try to queue it
		msg_copy = osMailAlloc(mq, 0);

		if (msg_copy == NULL) {
			return 0;
		}
	}
	memcpy(msg_copy, msg, msg_len);
	osMailPut(mq, msg_copy);
	return 1;
}

