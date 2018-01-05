
#include "cmsis_os.h"                                           // CMSIS RTOS header file

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void thread_hc_05 (void const *argument);                             // thread function
osThreadId tid_Thread_hc_05;                                          // thread id
osThreadDef (thread_hc_05, osPriorityNormal, 1, 0);                   // thread object

int Init_Thread (void) {

  tid_Thread_hc_05 = osThreadCreate (osThread(thread_hc_05), NULL);
  if (!tid_Thread_hc_05) return(-1);
  
  return(0);
}

void Thread (void const *argument) {

  while (1) {
    ; // Insert thread code here...
    osThreadYield ();                                           // suspend thread
  }
}
