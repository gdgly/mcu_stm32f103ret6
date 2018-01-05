
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "thread_of_accelnet.h" 
#include "string.h"
#include "thread_of_LCM_uart.h"
#include "misc.h"
#include "stm32f1xx_hal.h"
#include "xbusparser.h"
#include "xbusdef.h"
#include "xbusutility.h"
#include <stdlib.h>
#include "xsdeviceid.h"
#include "uart-line-IO.h"
#include "uart-API.h"
#include "math.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/




void thread_of_accelnet (void const *argument);                             // thread function
osThreadId tid_thread_accelnet;                                          // thread id
osThreadDef (thread_of_accelnet, osPriorityNormal, 1, 0);                   // thread object

extern osThreadId tid_hc05_thread; 

int init_thread_of_accelnet (void) {

  tid_thread_accelnet = osThreadCreate (osThread(thread_of_accelnet), NULL);
  if (!tid_thread_accelnet) return(-1);
  
  return(0);
}

#define MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT 200

osMailQDef(tid_thread_accelnet, 30, uint8_t[MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT]); 
static osMailQId mail_queue_id_for_evt;


osMailQDef(tid_hc05_thread, 30, int32_t[16]); 
osMailQId mail_queue_for_imu;

struct user_t{
	uint8_t status;
	int32_t c_p;
	int32_t a_p;
	int32_t c;
	int32_t v;
	char com[256];
};

struct user_t user_p;

static void* allocateMessageData(size_t bufSize)
{
	return NULL;
}

static void deallocateMessageData(void const* buffer)
{
	
}




static void mtMessageHandler(struct XbusMessage const* message)
{
	uint16_t counter;
	static int cnt=0;
	cnt++;
	if(cnt == 5)
	{
	cnt = 0;
	struct imu_mail_t *p = osMailAlloc(mail_queue_for_imu, 0);
	
	if(XbusMessage_getDataItem(&counter,XDI_PacketCounter,message))
	{
//		printf("Pcnt :%5d", counter);
	}
	
	float acc[3];
	if(XbusMessage_getDataItem(acc,XDI_Acceleration,message))
	{
//		printf(" Acc:(% .3f, % .3f, % .3f)", p->acc[0], p->acc[1],p->acc[2]);
	}
	float gyr[3];
	if (XbusMessage_getDataItem(gyr, XDI_RateOfTurn, message))
	{
//		printf(" Rt:(% .3f, % .3f, % .3f)", p->gyr[0], p->gyr[1], p->gyr[2]);
	}	
	float deltaV[3];
	if (XbusMessage_getDataItem(deltaV, XDI_DeltaV, message))
	{
//		printf(" DV:(% .3f, % .3f0, % .3f)", p->deltaV[0], p->deltaV[1], p->deltaV[2]);
	}	
	float ori[4];
	if(XbusMessage_getDataItem(ori,XDI_Quaternion,message))
	{
//		printf(" Ori:(% .3f, % .3f, % .3f, % .3f)\r\n", p->ori[0], p->ori[1],p->ori[2], p->ori[3]);
	}

	if(p)
	{	
			
//			p->acc[0]  =  (int32_t)(acc[0]*1000);
//			p->acc[1]  =  (int32_t)(acc[1]*1000);
//			p->acc[2]  =  (int32_t)(acc[2]*1000);
//			p->gyr[0]  =  (int32_t)(gyr[0]*1000);
//			p->gyr[1]  =  (int32_t)(gyr[1]*1000);
//			p->gyr[2]  =  (int32_t)(gyr[2]*1000);
//			p->deltaV[0]  =  (int32_t)(deltaV[0]*1000);
//			p->deltaV[1]  =  (int32_t)(deltaV[1]*1000);
//			p->deltaV[2]  =  (int32_t)(deltaV[2]*1000);
			p->ori[0]  =  (int32_t)(ori[0]*10000);
			p->ori[1]  =  (int32_t)(ori[1]*10000);
			p->ori[2]  =  (int32_t)(ori[2]*10000);
			p->ori[3]  =  (int32_t)(ori[3]*10000);
			osMailPut(mail_queue_for_imu, p);
			}
			
	}
	
}


void send2com(struct user_t *p)
{
	if(p->status == 0)
	{
	
	
	
	}
}




void thread_of_accelnet (void const *argument) {
  while (1) {
		mail_queue_id_for_evt = osMailCreate(osMailQ(tid_thread_accelnet), NULL);
		struct XbusParser *xbusParser  = osMailAlloc(mail_queue_id_for_evt, 0);
		xbusParser->state = XBPS_Preamble;
		xbusParser->callbacks.allocateBuffer = allocateMessageData;
		xbusParser->callbacks.deallocateBuffer = deallocateMessageData;
		xbusParser->callbacks.handleMessage = mtMessageHandler;
		
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
//		osDelay(1000);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);		
		uint8_t rxbuffer;
//		HAL_UART_Receive_IT(&huart4,&rxbuffer,1);
		mail_queue_for_imu = osMailCreate(osMailQ(tid_hc05_thread), NULL);
		struct imu_mail_t *p = osMailAlloc(mail_queue_for_imu, 0);
		while(1)
		{

//			osDelay(100);
			GetDataFromUart(4,&rxbuffer,1,20);
			int ret = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
			if(ret)
			{
				XbusParser_parseByte(xbusParser,rxbuffer);
			}
		}

  }
}









