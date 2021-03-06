#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_config.h"
#include "user_interface.h"
#include "stdout/stdout.h"
#include "max6675/max6675.h"

#define user_procTaskPrio        0
#define user_procTaskQueueLen    1
os_event_t user_procTaskQueue[user_procTaskQueueLen];
static void user_procTask(os_event_t *events);
static volatile os_timer_t loop_timer;

#define TEXTBUFFERSIZE 128
static char txtbuffer[TEXTBUFFERSIZE];

// forward declaratons
void loop(void);
static void setup(void);
void user_init(void);

//Do nothing function, main system task
static void ICACHE_FLASH_ATTR
user_procTask(os_event_t *events) {
  os_delay_us(10);
}

/**
 * This is the main user program loop
 */
void ICACHE_FLASH_ATTR
loop(void) {
  int bytesWritten = 0;
  if (max6675_readTempAsString(txtbuffer, TEXTBUFFERSIZE, &bytesWritten, true)) {
    os_printf("max6675 sensor: %s\r\n", txtbuffer);
  } else {
    os_printf("max6675 has no data.\r\n");
  }
}

/**
 * Setup program. When user_init() runs the debug printouts will not always
 * show on the serial console. So i run the inits in here, 2 seconds later.
 */
static void ICACHE_FLASH_ATTR
setup(void) {

  // Initialize the GPIO subsystem.
  gpio_init();
  const int ICS_PIN=2;   // !Chip Select: This is GIOP2, connect a 10K pull down resistor to GND to avoid problems with the bootloader starting
  const int CLOCK_PIN=0; // Clock       : This is GIOP0, connect a 10K pull up resistor to Vcc to avoid problems with the bootloader starting
  const int SO_PIN=3;    // Slave Output: This pin is normally RX. So don't have your hardware connected while flashing.

  if (max6675_init(ICS_PIN, CLOCK_PIN, SO_PIN)) {
    // Start repeating loop timer
    os_timer_disarm(&loop_timer);
    os_timer_setfn(&loop_timer, (os_timer_func_t *) loop, NULL);
    os_timer_arm(&loop_timer, 500, 1);
  } else {
    os_printf("max6675_init failed, aborting\n");
  }
}


//Init function 
void ICACHE_FLASH_ATTR
user_init() {
  // Make uart0 work with just the TX pin. Baud:115200,n,8,1
  // The RX pin is now free for GPIO use.
  stdout_init();

  // turn off WiFi for this console only demo
  wifi_station_set_auto_connect(false);
  wifi_station_disconnect();

  // Run setup() 2 seconds from now
  os_timer_disarm(&loop_timer);
  os_timer_setfn(&loop_timer, (os_timer_func_t *) setup, NULL);
  os_timer_arm(&loop_timer, 3000, 0);

  //Start os task
  system_os_task(user_procTask, user_procTaskPrio, user_procTaskQueue, user_procTaskQueueLen);
}
