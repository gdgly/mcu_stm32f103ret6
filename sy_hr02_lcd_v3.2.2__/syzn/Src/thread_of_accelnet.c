#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "thread_of_accelnet.h" 
#include "string.h"
#include "thread_of_hc05.h"
#include "stm32f1xx_hal.h"
#include "misc.h"
#include "uart-API.h"
#include "uart-line-IO.h"
#include "thread_of_LCM_uart.h"
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
 #define EVENT_LOOP_TIME_IN_MILLI_SECOND 20
 
 #define GLOVES_ADC_HIGH_LEVEL 2600
 #define GLOVES_ADC_LOW_LEVEL 1900
 
 #define BUTTON_ADC_HIGH_LEVEL 3.2
 #define BUTTON_ADC_LOW_LEVEL 0.2
 
 #define GLOVE_ADC_MAX 2750
 #define GLOVE_ADC_MIN 1800
 
void thread_of_accelnet (void const *argument);                             // thread function
osThreadId tid_thread_accelnet;                                          // thread id
osThreadDef (thread_of_accelnet, osPriorityNormal, 1, 0);                   // thread object

void moving_avergae(void);
void check_motion_mode(void);
uint8_t calculation_curvature(void);
void glove_calibration(void);

__IO float ADC_ConvertedValueLocal[16];
uint32_t adc_moving_resualt[16];
uint32_t ADC_ConvertedValue[2];

uint32_t time_now,time_mark = 0;
extern osThreadId tid_LCM_uart;
extern ADC_HandleTypeDef hadc1;
//extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern struct lcm_parameter_t user_parameter;
extern struct mutex_parameter_t mutex_parameter;
extern struct glove_adc_press_t glove_adc_press;
extern osMutexId lcm_mutex_id;


int init_thread_of_accelnet (void) {

  tid_thread_accelnet = osThreadCreate (osThread(thread_of_accelnet), NULL);
  if (!tid_thread_accelnet) return(-1);
  
  return(0);
}

int diff_adc_temp;
int voltage_level_cnt,button_level_cnt,peak_cnt,increase_cnt,decrease_cnt;
int manual_mode_flag,low_level_flag,manual_mode_flag_old = 0,is_glove_calibration;
uint32_t peak_adc_h,peak_adc_l,peak_adc_temp;
uint8_t is_reset;
uint32_t glov_extremum_max = GLOVE_ADC_MAX,glov_extremum_min = GLOVE_ADC_MIN;

void thread_of_accelnet (void const *argument) {
	user_signal_info_t user_timer_info = { osThreadGetId(), SIG_USER_TIMER };
	osTimerDef(main_timer, SetUserSignal);
	osTimerId main_timer_id = osTimerCreate(osTimer(main_timer), osTimerPeriodic, &user_timer_info);
	osTimerStart(main_timer_id, EVENT_LOOP_TIME_IN_MILLI_SECOND);

	int i = 0;
	while(1){
		osSignalWait(SIG_USER_TIMER, osWaitForever);	
				
		HAL_ADC_Start_DMA(&hadc1, ADC_ConvertedValue, 2);		
		for(i=0;i<15;i++){
			ADC_ConvertedValueLocal[15-i] = ADC_ConvertedValueLocal[14-i];
		}
		ADC_ConvertedValueLocal[0] =(float)(ADC_ConvertedValue[0]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]只取最低12有效数据
		
		voltage_level_cnt = 0;
		peak_cnt = 0;
		increase_cnt = 0;
		decrease_cnt = 0;
		
		moving_avergae();
		check_motion_mode();
		
		time_now = HAL_GetTick();
		button_level_cnt = 0;
				
		
		//0为按键模式 1为对侧手套模式 2为中间态
		switch(manual_mode_flag){
			case 0:
				osMutexWait(lcm_mutex_id,osWaitForever);
				mutex_parameter.mode = 0;
				osMutexRelease(lcm_mutex_id);		
				break;
				if((low_level_flag == 1)&&((time_now - time_mark) > 2000)){
					for(i=1;i<6;i++){
						if(ADC_ConvertedValueLocal[i] < BUTTON_ADC_LOW_LEVEL){
							button_level_cnt++;
						}
					}				
					if(button_level_cnt >= 5){
						HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
						time_mark = HAL_GetTick();
						osSignalSet(tid_LCM_uart,SIG_USER_2);
						low_level_flag = 0;
					}				
				}else{
					for(i=1;i<6;i++){
						if(ADC_ConvertedValueLocal[i] > BUTTON_ADC_HIGH_LEVEL){
							button_level_cnt++;
						}
					}
					if(button_level_cnt >= 5){
						low_level_flag = 1;
					}
				}				
				break;
			case 1:	
				if((user_parameter.common.pic_id >= calibration_animation_1)&&(user_parameter.common.pic_id <= calibration_animation_6)){
					osMutexWait(lcm_mutex_id,osWaitForever);
					mutex_parameter.mode = 1;
					osMutexRelease(lcm_mutex_id);
					glove_calibration();
				}else if(user_parameter.common.pic_id == gloves_not_connected){
					glov_extremum_max = GLOVE_ADC_MAX;
					glov_extremum_min = GLOVE_ADC_MIN;
				}else{
					is_glove_calibration = 0;
					osMutexWait(lcm_mutex_id,osWaitForever);
					mutex_parameter.mode = 1;
					mutex_parameter.curvature = calculation_curvature();
					osMutexRelease(lcm_mutex_id);									
					if((low_level_flag == 1)&&((time_now - time_mark) > 1000)){
						for(i=1;i<6;i++){
							if(adc_moving_resualt[i] > GLOVES_ADC_HIGH_LEVEL){
								button_level_cnt++;
							}
						}				
						if(button_level_cnt >= 5){
							//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12);
							time_mark = HAL_GetTick();
							if(user_parameter.common.pic_id == manual_motion_1){
								osSignalSet(tid_LCM_uart,SIG_USER_2);
							}						
							low_level_flag = 0;
						}				
					}else if((time_now - time_mark) > 1000){
						for(i=1;i<6;i++){
							if(adc_moving_resualt[i] < GLOVES_ADC_LOW_LEVEL){
								button_level_cnt++;
							}
						}
						if(button_level_cnt >= 5){
							//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12);
							time_mark = HAL_GetTick();
							if(user_parameter.common.pic_id == manual_motion_2){
								osSignalSet(tid_LCM_uart,SIG_USER_2);						
							}
							low_level_flag = 1;
						}
					}			
				}						
				break;
			case 2:
				
				break;
			default:
				
				break;
		
		}
		manual_mode_flag_old = manual_mode_flag;
	}
}

int cycle_cnt=0;
void moving_avergae(void)
{
	int i;
	cycle_cnt++;
	if(cycle_cnt > 100){
		cycle_cnt = 100;
		adc_moving_resualt[0] = (uint32_t)(ADC_ConvertedValueLocal[0]*1000);
		adc_moving_resualt[1] = (uint32_t)(((ADC_ConvertedValueLocal[0] + ADC_ConvertedValueLocal[1] + ADC_ConvertedValueLocal[2])/3)*1000);
		for(i=2;i<14;i++){
			adc_moving_resualt[i] = (uint32_t)(((ADC_ConvertedValueLocal[i-2] + ADC_ConvertedValueLocal[i-1] + ADC_ConvertedValueLocal[i] + ADC_ConvertedValueLocal[i+1] + ADC_ConvertedValueLocal[i+2])/5)*1000);
		}
	}else{
//	memcpy(adc_moving_resualt,(const float*)ADC_ConvertedValueLocal,sizeof(adc_moving_resualt));
		for(i=0;i<16;i++){
			adc_moving_resualt[i] = (uint32_t)(ADC_ConvertedValueLocal[i]*1000);
		}
	}
}



void check_motion_mode(void)
{
	int i;
	for(i=0;i<10;i++){
		if((ADC_ConvertedValueLocal[i] > 0.3)&&(ADC_ConvertedValueLocal[i] < 3.0)){
			voltage_level_cnt++;
		}
	}
	
	if(voltage_level_cnt > 8){
		manual_mode_flag = 1;
	}else if(voltage_level_cnt < 2){
		manual_mode_flag = 0;
		if(manual_mode_flag_old != 0){
			low_level_flag = 0;	
		}
	}else{
		manual_mode_flag = 2;
	}
}

uint8_t curvature;
uint8_t calculation_curvature(void)
{
	
	int cnt_temp[16],i;

	uint32_t glove_adc_width;
	glove_adc_width = glov_extremum_max - glov_extremum_min;
	if(glove_adc_width < 0.5){
		glov_extremum_max = GLOVE_ADC_MAX;
		glov_extremum_min = GLOVE_ADC_MIN;
		glove_adc_width = glov_extremum_max - glov_extremum_min;
	}
	
	memset(cnt_temp, 0, sizeof(cnt_temp));
	for(i=1;i<11;i++){
		if(adc_moving_resualt[i] < (glov_extremum_min + glove_adc_width*glove_adc_press.adc1_end*0.01)){									
			cnt_temp[1]++;
		}else if(adc_moving_resualt[i] < (glov_extremum_min + glove_adc_width*glove_adc_press.adc2_start*0.01)){					
			cnt_temp[2]++;
		}else if(adc_moving_resualt[i] < (glov_extremum_min + glove_adc_width*glove_adc_press.adc2_end*0.01)){						
			cnt_temp[3]++;
		}else if(adc_moving_resualt[i] < (glov_extremum_min + glove_adc_width*glove_adc_press.adc3_start*0.01)){		
			cnt_temp[4]++;
		}else{				
			cnt_temp[5]++;
		}
	}

	for(i=1;i<7;i++){
		if(cnt_temp[i] >= 7){
			curvature = i;				
		}
	}
//	int npressure;
//	float nadc_temp;
//	nadc_temp = (float)(ADC_ConvertedValue[1]&0xFFF)*3.3/4096;
//	npressure = (int)((nadc_temp*1.51 - 1.5)*100);
//	
//	uint8_t buff[64];
//	char send_to_iot_char[64];
//	memset((uint8_t*)send_to_iot_char,0,64);
//	memset(buff,0,64);
//	sprintf(send_to_iot_char,"@@event=%02d,%02d,%02d,%02d,%02d,%04d,%02d###\n",cnt_temp[1],cnt_temp[2],cnt_temp[3],cnt_temp[4],cnt_temp[5],npressure,adc_moving_resualt[0]);

//	int ln = strlen(send_to_iot_char);
//	memcpy(buff,(uint8_t*)send_to_iot_char,ln);
//	SendDataToUart(2,buff,ln,20);	

	return curvature;
}


uint8_t is_glove_reset_flag;
void glove_calibration(void)
{
	int i;

	for(i=2;i<11;i++){
		diff_adc_temp = adc_moving_resualt[i] - adc_moving_resualt[i-1];		
		if((diff_adc_temp < 10)&&(diff_adc_temp > -10)){
			peak_cnt++;
		}
		
		if(diff_adc_temp > 100){
			increase_cnt++;
		}
		
		if(diff_adc_temp < -100){
			decrease_cnt++;
		}
	}
	
	if(peak_cnt > 8){
		peak_adc_temp = adc_moving_resualt[1];
	}

	peak_adc_temp = adc_moving_resualt[1];
	
	if(is_glove_calibration == 0){
		is_glove_calibration = 1;
		glov_extremum_min = peak_adc_temp;
		glov_extremum_max = peak_adc_temp;
	}
	
	if(glov_extremum_max < peak_adc_temp){
		glov_extremum_max = peak_adc_temp;
	}

	if(glov_extremum_min > peak_adc_temp){
		glov_extremum_min = peak_adc_temp;
	}
	
	if(glov_extremum_max - glov_extremum_min < 500){
		is_glove_reset_flag = 0;
	}else{
		is_glove_reset_flag = 1;
	}
	
//	uint8_t buff[64];
//	char send_to_iot_char[64];
//	memset((uint8_t*)send_to_iot_char,0,64);
//	memset(buff,0,64);
//	sprintf(send_to_iot_char,"@@event=%04u,%04u,%04u###\n",peak_adc_temp,glov_extremum_max,glov_extremum_min);

//	int ln = strlen(send_to_iot_char);
//	memcpy(buff,(uint8_t*)send_to_iot_char,ln);
//	SendDataToUart(2,buff,ln,20);	
}


