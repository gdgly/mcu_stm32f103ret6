#ifndef __THREAD_OF_ANDROID_UART_H
#define __THREAD_OF_ANDROID_UART_H
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"     // CMSIS RTOS header file

int init_tid_thread_of_android_rx (void);
int init_tid_thread_of_android_tx (void);

struct android_share_t{
	uint8_t scene;
	uint8_t start_stop;
};

#define ANDROID_UART_NO 2

enum android_cmd_id_t{
	SetResetMCU = 1,
	SetShutDownAndroid,
	SetHandAngle,
	SetHandSpeed,
	SetScene,
	SetProof,
	SetStartStop,
	GetError,
	GetVersion,
	GetUsefulLife,
	GetTemperatur,
	GetMCU_ID,
	GetHandAngle,
	NotiHeart = 0x1e,
	NotiHandAngle,
	GetHandType = 0x28,
	GetHandError,
	GetHandID,
	GetHandVersion,
	NotiHandSetup,
	NotiScene,
};

enum mcu_evt_id_t{
	mcu_speed = 0x04,
	mcu_scene,
	mcu_proof,
	mcu_run_info,
	mcu_version,
	mcu_usefullife,
	mcu_temperature,
	mcu_machinary_id,
	mcu_hand_angle,	
	mcu_noti_heart = 0x1e,
	mcu_noti_hand_angle,
	mcu_hand_type = 0x28,
	glove_run_info,
	glove_machinary_id,
	glove_version,
	mcu_noti_glove_setup,
	mcu_noti_scene = 0x2e,
};



struct android_head_t{
	uint8_t serial_tail_h;
	uint8_t serial_tail_l;
	uint8_t type;
	uint8_t body_len;	
};

struct android_SetHandAngle_t{
	struct android_head_t head;
	uint8_t angle[5];
};


struct android_SetHandSpeed_t{
	struct android_head_t head;
	uint8_t speed;
};

struct mcu_speed_t{
	struct android_head_t head;
	uint8_t speed;
};

struct android_SetScene_t{
	struct android_head_t head;
	uint8_t scene;
};

struct mcu_scene_t{
	struct android_head_t head;
	uint8_t scene;
};

struct android_SetProof_t{
	struct android_head_t head;
	uint8_t proof;
};

struct android_SetStartStop_t{
	struct android_head_t head;
	uint8_t start_stop_pb;
};

struct mcu_proof_t{
	struct android_head_t head;
	uint8_t proof;
};

struct mcu_run_info_t{
	struct android_head_t head;
	uint32_t error;
};

struct mcu_version_t{
	struct android_head_t head;
	uint16_t version;
};

struct mcu_usefullife_t{
	struct android_head_t head;
	uint16_t useful_life;
};

struct mcu_temperature_t{
	struct android_head_t head;
	uint16_t temperature;
};

struct mcu_machinary_id_t{
	struct android_head_t head;
	uint32_t machinary_id;
};

struct mcu_hand_angle_t{
	struct android_head_t head;
	uint8_t angle[5];
	uint8_t mode;	
};

struct mcu_noti_heart_t{
	struct android_head_t head;
	uint8_t cnt;
};

struct mcu_noti_hand_angle_t{
	struct android_head_t head;
	uint8_t angle[5];
	uint8_t mode;
};

struct mcu_hand_type_t{
	struct android_head_t head;
	uint8_t left_type;
	uint8_t right_type;
};

struct glove_run_info_t{
	struct android_head_t head;
	uint32_t error;
};

struct glove_machinary_id_t{
	struct android_head_t head;
	uint32_t machinary_id;
};

struct glove_version_t{
	struct android_head_t head;
	uint16_t version;
};

struct mcu_noti_glove_setup_t{
	struct android_head_t head;
	
};

struct mcu_noti_scene_t{
	struct android_head_t head;
	uint8_t scene;
};

#define DECL_MAIL_QUEUE_FOR_ANDROID_DATAGRAM_CMD(type) extern osMailQId mail_queue_id_for_cmd_##type;
DECL_MAIL_QUEUE_FOR_ANDROID_DATAGRAM_CMD(SetProof)


// following should be used in our .c not invoker
#define DEFINE_MAIL_QUEUE_FOR_ANDROID_DATAGRAM_CMD(type,n) osMailQDef(type, (n), struct android_##type##_t); osMailQId mail_queue_id_for_cmd_##type;
#define INIT_MAIL_QUEUE_FOR_ANDROID_DATAGRAM_CMD(type) mail_queue_id_for_cmd_##type = osMailCreate(osMailQ(type), NULL)

#define ANDROID_DATAGRAM_INIT(name, id)         \
	memset(&name, 0, sizeof name);                \
	name.head.serial_tail_h = 0xfe;								\
	name.head.serial_tail_l = 0xfe;								\
	name.head.type       	 	= id;                 \
	name.head.body_len	 	  = sizeof(struct id##_t);


int mcu_to_android_datagram_send(void *msg, const size_t msg_len);
void *AndroidDatagramEvtAlloc(size_t size);
int AndroidDatagramEvtSend(void *ptr);
void AndroidDatagramEvtFree(void *ptr);
void android_rev_process(const void *msg, size_t msg_len);
void glove_angle_timer_process(void const *argument);

#endif

