
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "thread_of_accelnet.h" 
#include "string.h"
#include "thread_of_hc05.h"
#include "stm32f1xx_hal.h"
#include "math.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void thread_of_accelnet (void const *argument);                             // thread function
osThreadId tid_thread_accelnet;                                          // thread id
osThreadDef (thread_of_accelnet, osPriorityNormal, 1, 0);                   // thread object
extern UART_HandleTypeDef huart1;

int user_sscanf(void*msg1,void *msg2,uint8_t len);
int Getdatafromsoft(void*msg,uint32_t millisec);
void softdataprocess(void*msg,uint8_t len);



struct user_soft_para_t user_soft_para;


int init_thread_of_accelnet (void) {

  tid_thread_accelnet = osThreadCreate (osThread(thread_of_accelnet), NULL);
  if (!tid_thread_accelnet) return(-1);
  
  return(0);
}

void thread_of_accelnet (void const *argument) {

	
	uint8_t data[256];
	osDelay(1000);
  while (1) {
			int ln ;
			ln = Getdatafromsoft(data,1000);		
			if(ln!=0)
			{
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);		
			softdataprocess(data,ln);
			}			
  }
}

int user_sscanf(void*msg1,void *msg2,uint8_t len)
{
	uint8_t *s = msg1;
	uint8_t *d = msg2;
	int ln;
		while((uint8_t*)s < (uint8_t*)msg1+len)
		{
			if((sscanf((char *)s,"%2x",(uint32_t *)d)!=0)&&(*s != 0x20))
			{
				d++;		
				s=s+2;
			}
			else{
				s++;
			}

		}
		ln = (uint8_t*)d - (uint8_t*)msg2;
		return ln;
}

int Getdatafromsoft(void*msg,uint32_t millisec)
{
		uint8_t buffer[256];		
		uint8_t *s = buffer;	
		uint32_t time_now,time_mark;
		int ln;
		time_mark = time_now = HAL_GetTick();
		while(time_now - time_mark <millisec)
		{
			if(HAL_UART_Receive(&huart1,s,1,100)==HAL_OK)
			{
				if(*s == 0x0a){
					ln = user_sscanf(buffer,msg,s-buffer);
					return ln;
				}
				s++;			
			}
			time_now = HAL_GetTick();
		}
		return 0;
}


void softdataprocess(void*msg,uint8_t len)
{
	uint8_t *s = msg;
	while((uint8_t*)s<(uint8_t*)msg+len)
	{
		if(*s == 0x01)
		{
			s++;
			user_soft_para.whichhand = *s;
			s++;
			int i;
			for(i=0;i<10;i++)
			{
				user_soft_para.adc_calibration[i] = (*(s+2*i)<<8)|(*(s+2*i+1));				
			}
			break;
		}
		if(*s == 0x02)
		{
			s++;
			user_soft_para.soft_cnt = (*s<<8)|(*(s+1));			
			break;
		}			
		if(*s == 0x03)
		{
			s++;
			if((*s == 0xa5)&&(*(s+1) == 0x5a))
			{
				//enterstopmode;
				break;
			}		
		}
		break;
	}
}

