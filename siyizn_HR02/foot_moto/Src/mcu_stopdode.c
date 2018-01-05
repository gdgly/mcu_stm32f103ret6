
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "stm32f1xx_hal.h"
#include "thread_of_hc05.h"
#include "misc.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void thread_of_stopmode (void const *argument);                             // thread function
osThreadId tid_thread_of_stopmode;                                          // thread id
osThreadDef (thread_of_stopmode, osPriorityNormal, 1, 0);                   // thread object

void SYSCLKConfig_STOP(void);


#define EVENT_LOOP_TIME_IN_MILLI_SECOND 100
  
int is_in_stop,is_in_interrupt;
uint32_t stop_time,stop_mark;


int init_thread_of_stopmode (void) {

  tid_thread_of_stopmode = osThreadCreate (osThread(thread_of_stopmode), NULL);
  if (!tid_thread_of_stopmode) return(-1);
  
  return(0);
}

void thread_of_stopmode (void const *argument) {
	user_signal_info_t user_timer_info = { osThreadGetId(), SIG_USER_TIMER };
	osTimerDef(main_timer, SetUserSignal);
	osTimerId main_timer_id = osTimerCreate(osTimer(main_timer), osTimerPeriodic, &user_timer_info);
	osTimerStart(main_timer_id, EVENT_LOOP_TIME_IN_MILLI_SECOND);
	
	is_in_stop = 0;
	is_in_interrupt = 0;
  while (1) {
		osEvent evt = osSignalWait(SIG_USER_0,osWaitForever);
		if(evt.status == osEventSignal)
		{
			osDelay(10);
			stop_time = HAL_GetTick();
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)==GPIO_PIN_RESET)
			{
				is_in_interrupt = 1;
				stop_mark = stop_time;
			}
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)==GPIO_PIN_SET)
			{
				if((stop_time - stop_mark >4000)&&is_in_interrupt==1)
				{
					enter_stop_mode();				
				}		
			}				
		}                                        // suspend thread
  }
}

//�򵥵���ʱ����
static void Delay(__IO uint32_t nCount)	
{
	for(; nCount != 0; nCount--);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_12)
	{
		if(is_in_stop == 1)
		{
			SYSCLKConfig_STOP();											//ͣ�����Ѻ���Ҫ����HSE
			Delay(0x7FFFFF);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
			is_in_stop = 0;
			__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);				
		}
		else
		{
			osSignalSet(tid_thread_of_stopmode,SIG_USER_0);
		}

	}
}

/**
  * ��������: ͣ�����Ѻ�����ϵͳʱ��: ʹ�� HSE, PLL
  *           ����ѡ��PLL��Ϊϵͳʱ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SYSCLKConfig_STOP(void)
{
  /* ʹ�� HSE */
  __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);

  /* �ȴ� HSE ׼������ */
  while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET);
  
  /* ʹ�� PLL */ 
  __HAL_RCC_PLL_ENABLE();

  /* �ȴ� PLL ׼������ */
  while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
  {
  }

  /* ѡ��PLL��Ϊϵͳʱ��Դ */
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_PLLCLK);

  /* �ȴ�PLL��ѡ��Ϊϵͳʱ��Դ */
  while(__HAL_RCC_GET_SYSCLK_SOURCE() != 0x08)
  {
  }
}


void enter_stop_mode(void)
{
		is_in_stop = 1;
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
}
