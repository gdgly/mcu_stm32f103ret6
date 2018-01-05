#include "stm32f1xx_hal.h"




int init_thread_of_accelnet (void);
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
static void checksum(unsigned char *ptr,uint8_t *check,unsigned int len);


enum XbusState
{
	xbusPreamble,          /*!< \brief Looking for preamble. */
	xbusBusId,             /*!< \brief Waiting for bus ID. */
	xbusMessageId,         /*!< \brief Waiting for message ID. */
	xbusLength,            /*!< \brief Waiting for length. */
	xbusExtendedLengthMsb, /*!< \brief Waiting for extended length MSB*/
	xbusExtendedLengthLsb, /*!< \brief Waiting for extended length LSB*/
	xbusPayload,           /*!< \brief Reading payload. */
	xbusChecksum           /*!< \brief Waiting for checksum. */
};




struct imu_mail_t
{
//	int32_t acc[3];
//	int32_t gyr[3];
//	int32_t deltaV[3];
	int32_t ori[4];
};


