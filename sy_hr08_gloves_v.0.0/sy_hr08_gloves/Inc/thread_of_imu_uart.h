#ifndef __THREAD_OF_IMU_UART_H
#define __THREAD_OF_IMU_UART_H
#include "stm32f1xx_hal.h"
#include "thread_of_host_uart.h"

int init_thread_of_imu_uart (void);
void main_data_encode(struct data_host_uart_tx_t *p, __IO float *ptr);

void glove_calibration_process(void);
int glove_calibration_check(void);

#endif

