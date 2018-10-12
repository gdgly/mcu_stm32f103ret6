#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "misc.h"
#include "cmsis_os.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "string.h"
#include "thread_of_hc05.h"
#include "HC05.h"
#include <stdlib.h>

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void thread_of_hc05 (void const *argument);                             // thread function
osThreadId tid_hc05_thread;                                          // thread id
osThreadDef (thread_of_hc05, osPriorityNormal, 1, 0);                   // thread object

//extern UART_HandleTypeDef huart1;

#define MAX_SIZE_OF_UART_RECV 1024
struct uart_recv_result_t {
	uint8_t  data[MAX_SIZE_OF_UART_RECV];
	uint16_t len;
};




/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
//#define DEFAULT_HC05_ROLE           0 // 默认从模式
#define DEFAULT_HC05_ROLE           1 // 默认主模式

//unsigned int Task_Delay[2];
struct	task_delay_t
{
	uint32_t task_tick_now[2];
	uint32_t task_tick_old[2];
}Task;

BLTDev bltDevList;
char sendData[1024];
char linebuff[1024];

uint8_t aRxBuffer;
static __IO uint32_t TimingDelay=0;


struct imu_dac_parameter_t{
		float acc[3];
		float gyr[3];
		float deltaV[3];
		float ori[4];												
		float adc[5];
}user_p;

int init_thread_of_hc05 (void) {

  tid_hc05_thread = osThreadCreate (osThread(thread_of_hc05), NULL);
  if (!tid_hc05_thread) return(-1);
  
//	mail_queue_id_for_uart_recv = osMailCreate(osMailQ(uart_recv_queue), NULL);
	
  return(0);
}

void thread_of_hc05 (void const *argument) {
	osDelay(osWaitForever);
}

void Usart_SendString( uint8_t pUSARTx, uint8_t *str)
	{
}		

//中断缓存串口数据
#define UART_BUFF_SIZE      1024
__IO  uint16_t uart_p = 0;
uint8_t   uart_buff[UART_BUFF_SIZE];

/**
  * 函数功能: 接收中断回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  
//  if(uart_p<UART_BUFF_SIZE)
//  {
//    uart_buff[uart_p] =aRxBuffer; 
//    uart_p++; 
//    HAL_UART_Receive_IT(&huart2,&aRxBuffer,1);
//  }
//  else
//  {
//    clean_rebuff();       
//  }
}

/**
  * 函数功能: 获取接收到的数据和长度 
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
char *get_rebuff(uint16_t *len) 
{
    *len = uart_p;
    return (char *)&uart_buff;
}


/**
  * 函数功能: 获取透传的数据和长度 
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
uint8_t *get_data(uint16_t *len) 
{
    *len = uart_p;
    return (uint8_t *)&uart_buff;
}


/**
  * 函数功能: 清空缓冲区
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void clean_rebuff(void)
{
  uint16_t i=uart_p+1;
  
  uart_p = 0;
	while(i)
		uart_buff[--i]=0;
}

///**
//  * 函数功能: 系统滴答定时器中断服务函数
//  * 输入参数: GPIO_Pin：中断引脚
//  * 返 回 值: 无
//  * 说    明: 无
//  */
//void HAL_SYSTICK_Callback(void)
//{
//  unsigned char i;
//	
//	for(i=0;i<2;i++)
//	{
//		if(Task_Delay[i])
//		{
//			Task_Delay[i]--;
//		}
//	}
//}

void hextostr(void *msg,size_t len)
{
	uint8_t buf[256];
	uint8_t *p=buf;
	uint8_t *s = msg;

	*p++ = 'S';
	*p++ = 'Y';

	
//	while((*s != 0x55)&&((uint8_t*)s<(uint8_t*)msg+len))
//	{
//		s++;
//	}
//	while((*s != 0xa5)&&((uint8_t*)s<(uint8_t*)msg+len))
//	{
//		s++;
//	}	
	s++;
	if((uint8_t*)s>=(uint8_t*)msg+len)
	{
		return;
	}
//	while(((uint8_t*)s<(uint8_t*)msg+len)&&((*s != 0x55)||(*(s+1) != 0xa5)))
	while((uint8_t*)s<(uint8_t*)msg+len)
	{
		size_t ln=sprintf((char *) p, " %02X%02X", *(s+1),*s);
		s=s+2;
		p +=ln;
	}
	*p++ = '\n';	
//	HAL_UART_Transmit(&huart1,buf,p-buf,10);	

}	



void hc_05_bytecheck(void*msg,uint8_t len)
{
	static uint8_t buf[256];
	static int state = 0;
	static int cnt;
	uint8_t *s = msg;
	while((uint8_t*)s<(uint8_t*)msg+len)
	{
		switch(state)
		{
			case 0:
				if(*s == 0x55)
				{
					cnt = 0;
					state = 1;			
				}
			break;
			case 1:
				if(*s == 0xa5)
				{
					state = 2;
				}
				else
				{
					state = 0;			
				}
			break;
			case 2:
				buf[cnt] = *(s-1);	
				cnt ++;
				if(cnt == 37)
				{
					cnt = 0;	
					state = 0;
					hextostr(buf,36);
				}
				break;
		}
		s++;
	}
}


char *Getstrfromhc05(uint32_t millisec)
{
	return NULL;
}









