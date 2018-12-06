#include "stm32f1xx_hal.h"

int init_thread_of_development(void);
void set_valve(void);

//// datagram description
//#define SERIAL_DATAGRAM_START_CHR '\n'
//#define SERIAL_DATAGRAM_END_CHR   '\r'

#define UART_NO_TO_MAINBOARD 2

int get_raw_datagram_from_serial(uint8_t *raw_datagram, size_t max_size, size_t *actual_size_ptr, size_t *skipped_byte_count_ptr);
