#ifndef __THREAD_OF_SENSOR_GOLVE_H
#define __THREAD_OF_SENSOR_GOLVE_H
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

int init_thread_of_sensor_glove_rx (void);
int init_thread_of_sensor_glove_tx (void);

#define GLOVES_UART_NO 4

struct sensor_glove_para_t{
	uint8_t degree[5];
	uint16_t calibration_resualt;
	uint16_t version;
	uint32_t machinary_id;
};


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

struct data_host_uart_rx_t{
	struct uart_head_t head;
	uint16_t glove_adc[5];
};

struct calibration_resualt_t{
	struct uart_head_t head;
	uint16_t resualt;
};

struct calibration_data_t{
	struct uart_head_t head;
	uint16_t max[5];
	uint16_t min[5];
};

struct software_version_t{
	struct uart_head_t head;
	uint16_t version;
	uint32_t machinary_id;
};

struct machinary_id_t{
	struct uart_head_t head;
	uint32_t machinary_id_t;
};

struct calibration_cmd_t{
	struct uart_head_t head;
	uint16_t cmd;
};

struct host_msg_process_func_t{
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
int sensor_glove_datagram_send(void *msg, const size_t msg_len);

static int sensor_glove_msg_process_common_func(const void *msg, size_t msg_len, const void *ptr);
void update_sensor_glove_para(const void *msg, size_t msg_len);

#endif


