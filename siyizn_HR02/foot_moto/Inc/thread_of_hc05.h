#include "stm32f1xx_hal.h"


int init_thread_of_hc05 (void);
int init_thread_of_stopmode (void);
void enter_stop_mode(void);

void Usart_SendString( uint8_t pUSARTx, uint8_t *str);
unsigned char HexToChar(unsigned char bChar);
char *Getstrfromhc05(uint32_t millisec);
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;


