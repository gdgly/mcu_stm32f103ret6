#include "stm32f1xx_hal.h"


int init_thread_of_hc05 (void);

char *get_rebuff(uint16_t *len);
uint8_t *get_data(uint16_t *len);
void clean_rebuff(void);
void Usart_SendString( uint8_t pUSARTx, uint8_t *str);
void hextostr(void *msg,size_t len);
void hc_05_bytecheck(void*msg,uint8_t len);
char *Getstrfromhc05(uint32_t millisec);
int test(void*msg,uint32_t millisec,uint8_t len);

extern UART_HandleTypeDef huart4;


