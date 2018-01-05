#ifndef __HC05_H__
#define	__HC05_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
//#include "usart/bsp_usartx.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/*������ַ��������ʽ����NAP��UAP��LAP��*/			
#define BLTDEV_MAX_NUM         5

typedef  struct 
{
	uint16_t  NAP;
	uint8_t 	UAP;
	uint32_t  LAP;
}BLTAddr;

typedef  struct 
{
	uint8_t num;		//ɨ�赽�������豸����	
	BLTAddr addr[BLTDEV_MAX_NUM];	//�����豸��ַ��������ʽ
	char unpraseAddr[BLTDEV_MAX_NUM][50];	//�����豸��ַ���ַ�����ʽ������ɨ��ʱ������ʱʹ��
	char name[BLTDEV_MAX_NUM][50];	//�����豸������
}BLTDev;

enum
{
  HC05_DEFAULT_TIMEOUT = 200,
  HC05_INQUIRY_DEFAULT_TIMEOUT = 10000,
  HC05_PAIRING_DEFAULT_TIMEOUT = 10000,
  HC05_PASSWORD_MAXLEN = 16,
  HC05_PASSWORD_BUFSIZE = HC05_PASSWORD_MAXLEN + 1,
  HC05_NAME_MAXLEN = 32,
  HC05_NAME_BUFSIZE = HC05_NAME_MAXLEN + 1,
  HC05_ADDRESS_MAXLEN = 14,
  HC05_ADDRESS_BUFSIZE = HC05_ADDRESS_MAXLEN + 1,
};
		
/* �궨�� --------------------------------------------------------------------*/
#define HC05_USART          	          USART2

#define HC05_EN_GPIO_CLK() 	            __HAL_RCC_GPIOF_CLK_ENABLE()		/* GPIO�˿�ʱ�� */
#define HC05_EN_GPIO_PORT    	          GPIOC			              /* GPIO�˿� */
#define HC05_EN_GPIO_PIN		            GPIO_PIN_0		          /* ���ӵ�HC05 EN���ŵ�GPIO */
#define HC05_EN_HIGHT()		              HAL_GPIO_WritePin(HC05_EN_GPIO_PORT,HC05_EN_GPIO_PIN,GPIO_PIN_SET);	
#define HC05_EN_LOW()				            HAL_GPIO_WritePin(HC05_EN_GPIO_PORT,HC05_EN_GPIO_PIN,GPIO_PIN_RESET);	


/*��Ϣ���*/
#define HC05_DEBUG_ON                   1
#define HC05_DEBUG_FUNC_ON              0

#define HC05_INFO(fmt,arg...)           printf("<<-HC05-INFO->> "fmt"\n",##arg)
#define HC05_ERROR(fmt,arg...)          printf("<<-HC05-ERROR->> "fmt"\n",##arg)
#define HC05_DEBUG(fmt,arg...)          do{\
                                          if(HC05_DEBUG_ON)\
                                          printf("<<-HC05-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)

#define HC05_DEBUG_FUNC()               do{\
                                         if(HC05_DEBUG_FUNC_ON)\
                                         printf("<<-HC05-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)
														 
/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/																	 
void HC05_Init(void);
char* HC05_Send_CMD(char* cmd,uint8_t clean);
void HC05_SendString(char* str);																			 
void strBLTAddr(BLTDev *bltDev,char delimiter);
uint8_t getRemoteDeviceName(BLTDev *bltDev);
void printBLTInfo(BLTDev *bltDev);
uint8_t linkHC05(void);
int get_line(char* line, char* stream ,int max_size);
void atlinkhc05(void);
																			 
#endif /* __HC05_H__ */


/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
