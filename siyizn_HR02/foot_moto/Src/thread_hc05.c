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
#include "thread_of_accelnet.h" 

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void thread_of_hc05 (void const *argument);                             // thread function
osThreadId tid_hc05_thread;                                          // thread id
osThreadDef (thread_of_hc05, osPriorityNormal, 1, 0);                   // thread object

extern osThreadId tid_thread_accelnet; 
extern osMailQId mail_queue_for_imu;
extern ADC_HandleTypeDef hadc1;


#define MAX_SIZE_OF_UART_RECV 1024
struct uart_recv_result_t {
	uint8_t  data[MAX_SIZE_OF_UART_RECV];
	uint16_t len;
};

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define DEFAULT_HC05_ROLE           0 // 默认从模式
//#define DEFAULT_HC05_ROLE           1 // 默认主模式

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

__IO float ADC_ConvertedValueLocal[5];
uint32_t ADC_ConvertedValue[5];


//osMailQDef(uart_recv_queue, 5, struct uart_recv_result_t); 
//static osMailQId mail_queue_id_for_uart_recv;


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
	char hc05_name[30]="HC05_SLAVE";
  char hc05_nameCMD[40];
  
  
  HC05_Init();
  
  HC05_EN_LOW(); 
	osDelay(500);
  
  while(hc05_mode==0)
  {
//    printf("》》按下KEY1使用HC-05上次配置进行通信\n》》按下HC-05模块内按键进入配置默认\n");	
    HC05_EN_HIGHT(); 
    /* 发送一个AT指令 */
    Usart_SendString(5,(uint8_t *)"AT\r\n");

    for(i=0;i<20;++i)
    {
      HAL_Delay(20); 
      redata=Getstrfromhc05(1000);
      if(redata!=0)
      {
        if(strstr((char*)redata,"OK"))		
        {
          hc05_mode=1;
          break;
        }
      }
    } 
	}   
  if(hc05_mode==1)  //  AT模式
  {  
			
    /*复位、恢复默认状态*/
    HC05_Send_CMD("AT+RESET\r\n",1);	
    HAL_Delay(400);
    
//    HC05_Send_CMD("AT+ORGL\r\n",1);
//    HAL_Delay(200);
//    
    /*各种命令测试演示，默认不显示。
     *在bsp_hc05.h文件把HC05_DEBUG_ON 宏设置为1，
     *即可通过串口调试助手接收调试信息*/	    
//    HC05_Send_CMD("AT+VERSION?\r\n",1);
       
//    HC05_Send_CMD("AT+ADDR?\r\n",1);    
			HC05_Send_CMD("AT+UART=38400,0,1\r\n",1);  
//    HC05_Send_CMD("AT+CMODE?\r\n",1);    
//    HC05_Send_CMD("AT+STATE?\r\n",1);	 
    if(hc05_role==0) // 从模式
    {		
				HC05_Send_CMD("AT+ROLE=0\r\n",1);
				HC05_Send_CMD("AT+iac=9e8b3f\r\n",1);
        HAL_Delay(100);        
//        sprintf(hc05_mode_str,"SLAVE");
////        HC05_INFO("hc05_mode  = %s",hc05_mode_str);	

//        sprintf(hc05_name,"HC05_%s_%d",hc05_mode_str,(uint8_t)rand());
//        sprintf(hc05_nameCMD,"AT+NAME=%s\r\n",hc05_name);
//        
//				HC05_Send_CMD(hc05_nameCMD,1);
////          HC05_INFO("设备名字被更改为：%s",hc05_name);
////        else
    }
    else
    {
      if(HC05_Send_CMD("AT+ROLE=1\r\n",1) == 0)	
      {
        HAL_Delay(100);        
        sprintf(hc05_mode_str,"MASTER");        
        sprintf(hc05_name,"HC05_%s_%d",hc05_mode_str,(uint8_t)rand());
        sprintf(hc05_nameCMD,"AT+NAME=%s\r\n",hc05_name);	        
        HC05_Send_CMD(hc05_nameCMD,1);
      }
    }
    //清空蓝牙设备列表
    bltDevList.num = 0; 
  }
  else
  {  
    hc05_connect=1;
  }
	while(1)
	{
		//搜索蓝牙模块，并进行连接
		if(hc05_connect==0)
		{
      HAL_Delay(100);
			if(hc05_role == 1)	//主模式
			{
				while(linkHC05()==1)
        {          
        }
			}
			else	//从模式
			{
				HC05_Send_CMD("AT+INIT\r\n",1);
        HC05_Send_CMD("AT+INQ\r\n",1);//模块在查询状态，才能容易被其它设备搜索到
        HAL_Delay(1000);        
			}
      hc05_connect=1;
      HC05_Send_CMD("AT+CLASS=0\r\n",1);
      HAL_Delay(100);
      HC05_Send_CMD("AT+INQM=1,9,48\r\n",1);  
      HAL_Delay(1000);
			HC05_EN_LOW();
			HAL_Delay(100);
			Task.task_tick_now[1] = HAL_GetTick();
	    Task.task_tick_old[1] = Task.task_tick_now[1];
		}
		static int ulink_cnt = 0;		
		Task.task_tick_now[1] = HAL_GetTick();
		int ret = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
		if(ret)
		{				
			ulink_cnt = 0;
			Task.task_tick_old[1] = Task.task_tick_now[1];
			osEvent evt = osMailGet(mail_queue_for_imu,osWaitForever);
			struct imu_mail_t *p = evt.value.p;
			
			if (evt.status == osEventMail)
			{
				
				uint8_t data[256],buffer[64];
				uint8_t cnt;
				ADC_ConvertedValueLocal[0] =(float)(ADC_ConvertedValue[0]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]只取最低12有效数据
				ADC_ConvertedValueLocal[1] =(float)(ADC_ConvertedValue[1]&0xFFF)*3.3/4096; // ADC_ConvertedValue[1]只取最低12有效数据
				ADC_ConvertedValueLocal[2] =(float)(ADC_ConvertedValue[2]&0xFFF)*3.3/4096; // ADC_ConvertedValue[2]只取最低12有效数据
				ADC_ConvertedValueLocal[3] =(float)(ADC_ConvertedValue[3]&0xFFF)*3.3/4096; // ADC_ConvertedValue[3]只取最低12有效数据
				ADC_ConvertedValueLocal[4] =(float)(ADC_ConvertedValue[4]&0xFFF)*3.3/4096; // ADC_ConvertedValue[4]只取最低12有效数据				

				data[0] = 0x55;
				data[1] = 0xa5;					
//				for(cnt=0;cnt<3;cnt++){
//					data[2*cnt+2] = (uint8_t)p->acc[cnt]&0x000000ff;
//					data[2*cnt+3] = (uint8_t)((p->acc[cnt]&0x0000ff00)>>8);			
//				}
//				
//				for(cnt=0;cnt<3;cnt++){
//					data[2*cnt+8] = (uint8_t)p->gyr[cnt]&0x000000ff;
//					data[2*cnt+9] = (uint8_t)((p->gyr[cnt]&0x0000ff00)>>8);			
//				}				
//				
//				for(cnt=0;cnt<3;cnt++){
//					data[2*cnt+14] = (uint8_t)p->deltaV[cnt]&0x000000ff;
//					data[2*cnt+15] = (uint8_t)((p->deltaV[cnt]&0x0000ff00)>>8);			
//				}				

//				for(cnt=0;cnt<4;cnt++){
//					data[2*cnt+20] = (uint8_t)p->ori[cnt]&0x000000ff;
//					data[2*cnt+21] = (uint8_t)((p->ori[cnt]&0x0000ff00)>>8);			
//				}
//				
				for(cnt=0;cnt<5;cnt++){
					data[2*cnt+10] = (uint8_t)ADC_ConvertedValue[cnt]&0x00ff;
					data[2*cnt+11] = (uint8_t)((ADC_ConvertedValue[cnt]&0xff00)>>8);										
				}
				
				for(cnt=0;cnt<4;cnt++){
					data[2*cnt+2] = (uint8_t)p->ori[cnt]&0x000000ff;
					data[2*cnt+3] = (uint8_t)((p->ori[cnt]&0x0000ff00)>>8);			
				}

//				int temp;
//				temp = SendReqAndRecvResDataWithUart(5,data,28,buffer,5,80);
//				if(temp)
//				{
//					HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
//				}
				SendDataToUart(5,data,20,30);
			}
				osMailFree(mail_queue_for_imu, p);
		}
		else{
			
			ulink_cnt++;
			if(ulink_cnt == 5000){
				ulink_cnt = 0;
				enter_stop_mode();
			}
			HAL_Delay(20);
		}
	}		
		
	}	
}


unsigned char HexToChar(unsigned char bChar)  
{  
    if((bChar>=0x30)&&(bChar<=0x39))  
    {  
        bChar -= 0x30;  
    }  
    else if((bChar>=0x41)&&(bChar<=0x46)) // Capital  
    {  
        bChar -= 0x37;  
    }  
    else if((bChar>=0x61)&&(bChar<=0x66)) //littlecase  
    {  
        bChar -= 0x57;  
    }  
    else   
    {  
        bChar = 0xff;  
    }  
    return bChar;  
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
	SendDataToUart(5,buf,k,20);
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
			if(GetDataFromUart(5,s,1,100)==1)
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




