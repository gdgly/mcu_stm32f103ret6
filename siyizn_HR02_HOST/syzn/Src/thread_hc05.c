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


	struct AsyncIoResult_t IoResult = { 0, osThreadGetId() };
  while (1) {

  uint8_t hc05_connect=0; // 0:未连接       1：连接成功，可以进行数据传输
  uint8_t hc05_mode=0;  // 0：SPP规范      1：AT模式
  char hc05_mode_str[10]="SLAVE";
  uint8_t hc05_role=DEFAULT_HC05_ROLE;  // 0：从模式       1：主模式
  uint8_t i=0;
	char* redata;
	uint16_t len;
	char hc05_name[30]="HC05_SLAVE";
  char hc05_nameCMD[40];
  
  HC05_Init();
		
	hc05_connect=0;		
	hc05_mode=0;
	
  HC05_EN_LOW(); 
//  while(hc05_mode==0)
//  {
//		osDelay(500);
//    HC05_EN_HIGHT(); 
//    /* 发送一个AT指令 */
//    Usart_SendString(2,(uint8_t *)"AT\r\n");

//    for(i=0;i<20;++i)
//    {
//      HAL_Delay(20); 
//      redata=Getstrfromhc05(1000);
//      printf("redata=%d",len);
//      if(redata!=0)
//      {
//        HC05_DEBUG("receive=%s",redata);
//        if(strstr((char*)redata,"OK"))		
//        {
//          hc05_mode=1;
//          break;
//        }
//      }
//    } 
//	}  

//  if(hc05_mode==1)  //  AT模式
//  {  
//			
//    /*复位、恢复默认状态*/
////		HC05_Send_CMD("AT+BIND=2017,10,116891\r\n",1);
////		osDelay(osWaitForever);
//    HC05_Send_CMD("AT+RESET\r\n",1);	
//    HAL_Delay(400);
//    
//    HC05_Send_CMD("AT+ORGL\r\n",1);
//    HAL_Delay(200);
//    
//    /*各种命令测试演示，默认不显示。
//     *在bsp_hc05.h文件把HC05_DEBUG_ON 宏设置为1，
//     *即可通过串口调试助手接收调试信息*/	       
//		HC05_Send_CMD("AT+UART=38400,0,1\r\n",1);  
//    HC05_Send_CMD("AT+CMODE?\r\n",1);    
//    if(hc05_role==0) // 从模式
//    {
//      if(HC05_Send_CMD("AT+ROLE=0\r\n",1) == 0)	
//      {				
//        HAL_Delay(100);        
//        sprintf(hc05_mode_str,"SLAVE");
//        HC05_INFO("hc05_mode  = %s",hc05_mode_str);	

//        sprintf(hc05_name,"HC05_%s_%d",hc05_mode_str,(uint8_t)rand());
//        sprintf(hc05_nameCMD,"AT+NAME=%s\r\n",hc05_name);
//        
//        if(HC05_Send_CMD(hc05_nameCMD,1) != 0)
//          HC05_INFO("设备名字被更改为：%s",hc05_name);
//        else
//          HC05_ERROR("更改名字失败");
//      }
//    }
//    else
//    {
////      if(HC05_Send_CMD("AT+ROLE=1\r\n",1) == 0)	
////      {
////        HAL_Delay(100);
//				HC05_Send_CMD("AT+ROLE=1\r\n",1);
//				HC05_Send_CMD("AT+INIT\r\n",1);
//				HC05_Send_CMD("AT+iac=9e8b3f\r\n",1);
//				HC05_Send_CMD("at+inqm=1,1,24\r\n",1);
//        sprintf(hc05_mode_str,"MASTER");
//        HC05_INFO("HC05 mode  = %s",hc05_mode_str);          
//        sprintf(hc05_name,"HC05_%s_%d",hc05_mode_str,(uint8_t)rand());
//        sprintf(hc05_nameCMD,"AT+NAME=%s\r\n",hc05_name);	        
//        if(HC05_Send_CMD(hc05_nameCMD,1) != 0)
//          HC05_INFO("设备名字被更改为：%s",hc05_name);
//        else
//          HC05_ERROR("更改名字失败");
////      }
//    }
//    //清空蓝牙设备列表
//    bltDevList.num = 0;
//    
//    HC05_Send_CMD("AT+INIT\r\n",1);
//    HC05_Send_CMD("AT+CLASS=0\r\n",1);
//    HC05_Send_CMD("AT+INQM=1,9,48\r\n",1);        
//  }
//  else
//  {  
//    hc05_connect=1;
//    printf("使用HC-05上次配置进行通信\n");
//  }
  
	while(1)
	{
//		//搜索蓝牙模块，并进行连接
//		if(hc05_connect==0)
//		{
//      HAL_Delay(100);
//			if(hc05_role == 1)	//主模式
//			{
//				HC05_INFO("正在扫描蓝牙设备...");				
//				while(linkHC05()==1)
//        {          
//        }
//			}
//			else	//从模式
//			{
//				HC05_Send_CMD("AT+INIT\r\n",1);
//				HAL_Delay(100);
//        HC05_Send_CMD("AT+INQ\r\n",1);//模块在查询状态，才能容易被其它设备搜索到
//        HAL_Delay(1000);        
//			}
//      hc05_connect=1;
//      HC05_Send_CMD("AT+CLASS=0\r\n",1);
//      HAL_Delay(100);
//      HC05_Send_CMD("AT+INQM=1,9,48\r\n",1);  
//      HAL_Delay(1000);
//      HC05_EN_LOW();      
////      HC05_SendString("CONNECTED");
//      HAL_Delay(100);   
//		}
		//连接后每隔一段时间检查接收缓冲区
    Task.task_tick_now[0] = HAL_GetTick();
		int ret = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);
		if(ret)
		{
			uint8_t buffer[512];
			while(1)
			{
				int ret;
				ret = HAL_UART_Receive(&huart4,buffer,20,100);
				if(ret == HAL_OK)
				{
					hc_05_bytecheck(buffer,20);
				}		
			}						
		}
		else
		{
			HAL_Delay(20);
		}	
	}				
	}	
}

void Usart_SendString( uint8_t pUSARTx, uint8_t *str)
	{
	unsigned int k=0;
	uint8_t buf[128];
		    do 
    {
				buf[k] = *(str + k);
        k++;
    } while(*(str + k)!='\0');
	HAL_UART_Transmit(&huart4,buf,k,20);
//	SendDataToUart(2,buf,k,20);
}		


///**
//  * 函数功能: 获取接收到的数据和长度 
//  * 输入参数: 无
//  * 返 回 值: 无
//  * 说    明：无
//  */
//char *get_rebuff(uint16_t *len) 
//{
//    *len = uart_p;
//    return (char *)&uart_buff;
//}


///**
//  * 函数功能: 获取透传的数据和长度 
//  * 输入参数: 无
//  * 返 回 值: 无
//  * 说    明：无
//  */
//uint8_t *get_data(uint16_t *len) 
//{
//    *len = uart_p;
//    return (uint8_t *)&uart_buff;
//}


/**
//  * 函数功能: 清空缓冲区
//  * 输入参数: 无
//  * 返 回 值: 无
//  * 说    明：无
//  */
//void clean_rebuff(void)
//{
//  uint16_t i=uart_p+1;
//  
//  uart_p = 0;
//	while(i)
//		uart_buff[--i]=0;
//}


void hextostr(void *msg,size_t len)
{
	uint8_t buf[256];
	uint8_t *p=buf;
	uint16_t *s = msg;

	*p++ = 'S';
	*p++ = 'Y';
	if((uint8_t*)s>=(uint8_t*)msg+len)
	{
		return;
	}
	while((uint16_t*)s<(uint16_t*)msg+len)
	{
		size_t ln=sprintf((char *) p, " %04X", *s);
		s++;
		p +=ln;
	}
	*p++ = '\n';	
	HAL_UART_Transmit(&huart1,buf,p-buf,10);	

}	

extern struct user_soft_para_t user_soft_para;

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
				buf[cnt] = *s;	
				cnt ++;
				if(cnt == len)
				{
					cnt = 0;	
					state = 0;					
					adc_calibration(buf,len);
				}
				break;
		}
		s++;
	}
}


char *Getstrfromhc05(uint32_t millisec)
{
		static uint8_t buffer[512];
		uint32_t time_now,time_mark;
		time_mark = time_now = HAL_GetTick();
		memset(buffer,0,sizeof(buffer));
		uint8_t *s = buffer;
		while(time_now - time_mark <millisec)
		{
			if(HAL_UART_Receive(&huart4,s,1,100)==HAL_OK)
			{
				if((*s == 0x0a)&&((strstr((char *)&buffer, "OK"))||(strstr((char *)&buffer, "ERROR"))))
				{
					return (char *)&buffer;
				}
				s++;			
			}
			time_now = HAL_GetTick();
		}
		return 0;
}


int test(void*msg,uint32_t millisec,uint8_t len)
{
		uint32_t time_now,time_mark;
		time_mark = time_now = HAL_GetTick();
		uint8_t *s = msg;
		while(time_now - time_mark <millisec)
		{
			if(HAL_UART_Receive(&huart4,s,1,100)==HAL_OK)
			{
				if((*s == 0x0a)&&((strstr((char *)msg, "OK"))||(strstr((char *)msg, "ERROR"))))
				{
					len = (uint8_t*)s - (uint8_t*)msg;
					return 1;
				}
				s++;			
			}
			time_now = HAL_GetTick();
		}
		return 0;
}

void adc_calibration(void *msg,uint8_t len)
{
	
	
	
	uint16_t buf[32];
	uint8_t *s = msg;
	uint16_t *p = buf;
	while((uint8_t*)s<(uint8_t*)msg+len)
	{
		*p = *(s+1)<<8|*s;
		p++;
		s=s+2;
	}	
	p = buf;
	if(user_soft_para.whichhand == 0)
	{
		*(p + 4) = 	*(p + 4) - user_soft_para.adc_calibration[0];
		*(p + 5) = 	*(p + 5) - user_soft_para.adc_calibration[1];
		*(p + 6) = 	*(p + 6) - user_soft_para.adc_calibration[2];
		*(p + 7) = 	*(p + 7) - user_soft_para.adc_calibration[3];
		*(p + 8) = 	*(p + 8) - user_soft_para.adc_calibration[4];
		*(p + 9) = 0;
		*(p + 10) = 0;
		*(p + 11) = 0;
		*(p + 12) = 0;
		*(p + 13) = 0;
	}
	if(user_soft_para.whichhand == 1)
	{
		*(p + 9)  = *(p + 8) - user_soft_para.adc_calibration[0];
		*(p + 10) = *(p + 7) - user_soft_para.adc_calibration[1];
		*(p + 11) = *(p + 6) - user_soft_para.adc_calibration[2];
		*(p + 12) = *(p + 5) - user_soft_para.adc_calibration[3];
		*(p + 13) = *(p + 4) - user_soft_para.adc_calibration[4];	
		*(p + 4) = 	0;
		*(p + 5) = 	0;
		*(p + 6) = 	0;
		*(p + 7) = 	0;
		*(p + 8) = 	0;
	}
	hextostr(buf,14);
}

