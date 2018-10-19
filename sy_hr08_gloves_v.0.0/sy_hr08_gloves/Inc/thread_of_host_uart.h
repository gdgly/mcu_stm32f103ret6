#ifndef __THREAD_OF_IMU_HOST_H
#define __THREAD_OF_IMU_HOST_H
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

int init_thread_of_host_uart_tx (void);
int init_thread_of_host_uart_rx (void);

// datagram description
#define SERIAL_DATAGRAM_START_CHR '\r'
#define SERIAL_DATAGRAM_END_CHR   '\n'

enum evt_id_t{
	data_host_uart_tx = 5,
	calibration_resualt,
	calibration_data,
	software_version,
	machinary_id,
	evt_id_max,
};

enum cmd_id_t{
	calibration_cmd = evt_id_max,
};

struct uart_head_t{
	uint8_t type;
	uint8_t body_len;
};

struct data_host_uart_tx_t{
	struct uart_head_t head;
	uint8_t tx_buff[10];
};

struct calibration_resualt_t{
	struct uart_head_t head;
	uint16_t tx_buff;
};

struct calibration_data_t{
	struct uart_head_t head;
	uint16_t tx_buff[10];
};

struct software_version_t{
	struct uart_head_t head;
	uint16_t tx_buff[3];
};

struct machinary_id_t{
	struct uart_head_t head;
	uint16_t tx_buff[2];
};

struct serial_calibration_cmd_t{
	uint16_t cmd;
};

struct gloves_msg_process_func_t{
	uint8_t id;
	uint8_t body_len;
	const void *ptr;
};


#define DECL_MAIL_QUEUE_FOR_SERIAL_DATAGRAM_CMD(type) extern osMailQId mail_queue_id_for_cmd_##type;
DECL_MAIL_QUEUE_FOR_SERIAL_DATAGRAM_CMD(calibration_cmd)

// following should be used in our .c not invoker
#define DEFINE_MAIL_QUEUE_FOR_SERIAL_DATAGRAM_CMD(type,n) osMailQDef(type, (n), struct serial_##type##_t); osMailQId mail_queue_id_for_cmd_##type;
#define INIT_MAIL_QUEUE_FOR_SERIAL_DATAGRAM_CMD(type) mail_queue_id_for_cmd_##type = osMailCreate(osMailQ(type), NULL)



#define SERIAL_DATAGRAM_INIT(name, id)          \
	memset(&name, 0, sizeof name);                \
	name.head.type       = id;                    \
	name.head.body_len	 = sizeof(struct id##_t);

// prepair stage: allocate a mail box slot for event. size is serial-datagram-size. return NULL if failed, non-NULL which pointer to the serial-datagram.
void *SerialDatagramEvtAlloc(size_t size);
// send data into the mail Q, then sending thread(event thread) would sending it 
int SerialDatagramEvtSend(void *ptr);
void SerialDatagramEvtFree(void *ptr);
int host_uart_datagram_send(void *msg, const size_t msg_len);
int send_raw_datagram_to_serial(const void *raw_datagram, size_t raw_datagram_len);

int get_raw_datagram_from_serial(uint8_t *raw_datagram, size_t max_size, size_t *actual_size_ptr, size_t *skipped_byte_count_ptr);
static int serial_datagram_msg_process_common_func(const void *msg, size_t msg_len, const void *ptr);


#endif

