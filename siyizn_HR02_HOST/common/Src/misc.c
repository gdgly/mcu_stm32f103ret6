#include "misc.h"

void NotifyAsyncIoFinished(void *p, uint32_t value)
{
	struct AsyncIoResult_t *result = p;
	result->IoResult = value;
	osSignalSet(result->CallerId, SIG_SERVER_FINISHED);
}

void SetUserSignal(const void *user_signal_info_ptr)
{
	const user_signal_info_t *p = user_signal_info_ptr;
	osSignalSet(p->tid, p->signal);
}



//void SendStrToHost(const char *msg)
//{
//	struct serial_datagram_evt_str_info_t *p = SerialDatagramEvtAlloc(sizeof (*p)); // send it by string
//	if(p) {
//		SERIAL_DATAGRAM_INIT((*p), evt_str_info);
//		strncpy((char *)(p->info.str_info), msg, sizeof (p->info.str_info));
//		SerialDatagramEvtSend(p);
//	}
//}

#include <stdarg.h>
#include <stdio.h>
//void PrintToHost(const char *format,...)
//{
//	va_list args;
//	char buffer[sizeof (struct evt_str_info_t) + 1];

//	va_start(args, format);
//	vsnprintf(buffer, sizeof buffer, format, args);
//	va_end(args);
//	SendStrToHost(buffer);
//}

