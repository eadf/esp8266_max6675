#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_config.h"
#include "user_interface.h"
#include "driver/stdout.h"

#define user_procTaskPrio        0
#define user_procTaskQueueLen    1
os_event_t user_procTaskQueue[user_procTaskQueueLen];
static void user_procTask(os_event_t *events);

static volatile os_timer_t some_timer;

#define TEXTBUFFERSIZE 128
static char txtbuffer[TEXTBUFFERSIZE];

void some_timerfunc(void *arg) {
  int bytesWritten = 0;
  if (max6675_readTempAsString(txtbuffer, TEXTBUFFERSIZE, &bytesWritten, true)) {
    os_printf("max6675 sensor: %s\r\n", txtbuffer);
  } else {
    os_printf("max6675 has no data.\r\n");
  }
}

//Do nothing function
static void ICACHE_FLASH_ATTR
user_procTask(os_event_t *events) {
  os_delay_us(10);
}

//Init function 
void ICACHE_FLASH_ATTR
user_init() {
  // Make os_printf working again. Baud:115200,n,8,1
  stdoutInit();

  //Set station mode
  wifi_set_opmode( NULL_MODE );

  // Initialize the GPIO subsystem.
  gpio_init();
  max6675_init();

  //Disarm timer
  os_timer_disarm(&some_timer);

  //Setup timer
  os_timer_setfn(&some_timer, (os_timer_func_t *) some_timerfunc, NULL);

  //Arm the timer
  //&some_timer is the pointer
  //1000 is the fire time in ms
  //0 for once and 1 for repeating
  os_timer_arm(&some_timer, 1000, 1);

  //Start os task
  system_os_task(user_procTask, user_procTaskPrio, user_procTaskQueue, user_procTaskQueueLen);
}
