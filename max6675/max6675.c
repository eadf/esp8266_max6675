/*
 * max6675.c
 *
 *  This is a direct port of the Arduino library found at https://github.com/adafruit/MAX6675-library
 *
 * this library is public domain. enjoy!
 * www.ladyada.net/learn/sensors/thermocouple
 */
#include "max6675/max6675.h"
#include "gpio.h"
#include "ets_sys.h"
#include "osapi.h"
#include "math.h"
#include "easygpio/easygpio.h"

static uint16_t ICS_PIN=2;   // !Chip Select: This is GIOP2, connect a 10K pull down resistor to GND to avoid problems with the bootloader starting
static uint16_t CLOCK_PIN=0; // Clock       : This is GIOP0, connect a 10K pull up resistor to Vcc to avoid problems with the bootloader starting
static uint16_t SO_PIN=3;    // Slave Output: This pin is normally RX. So don't have your hardware connected while flashing.

static bool max6675_isConfigured = false;

void max6675_delay(uint32 microseconds);
void max6675_delay_between_clock(void);
void max6675_digitalWrite(unsigned int pin, bool value);
bool max6675_digitalRead(unsigned int pin);
uint8_t max6675_readByte(void);

// some arduino lookalike methods :)
#define LOW false
#define HIGH true

void ICACHE_FLASH_ATTR
max6675_delay(uint32 microseconds) {
  os_delay_us(1000*microseconds);
}

void ICACHE_FLASH_ATTR
max6675_delay_between_clock(void) {
  os_delay_us(500);
}

void ICACHE_FLASH_ATTR
max6675_digitalWrite(unsigned int pin, bool value) {
  GPIO_OUTPUT_SET(pin, value);
}

bool ICACHE_FLASH_ATTR
max6675_digitalRead(unsigned int pin) {
  return GPIO_INPUT_GET(pin)!=0;
}

uint8_t ICACHE_FLASH_ATTR
max6675_readByte(void) {
  int i;
  uint8_t d = 0;
  for (i=7; i>=0; i--)
  {
    max6675_digitalWrite(CLOCK_PIN, LOW);
    max6675_delay_between_clock();
    if (max6675_digitalRead(SO_PIN)) {
      //set the bit to 0 no matter what
      d |= (1 << i);
    }
    max6675_digitalWrite(CLOCK_PIN, HIGH);
    max6675_delay_between_clock();
  }
  return d;
}


bool ICACHE_FLASH_ATTR
max6675_readTemp(float* sample, bool celcius)
{
  if (!max6675_isConfigured) {
    os_printf("max6675 module is not completely configured.\r\n");
    return false;
  }

  uint16_t v;
  max6675_digitalWrite(ICS_PIN, LOW);
  max6675_delay_between_clock();
  v = max6675_readByte();
  v <<= 8;
  v |= max6675_readByte();
  max6675_digitalWrite(CLOCK_PIN, LOW);
  max6675_delay_between_clock();
  max6675_digitalWrite(ICS_PIN, HIGH);
  if (v & 0x4) {
    // uh oh, no thermocouple attached!
    os_printf("The max6675 chip thinks the thermocouple is lose.\r\n", v);
    *sample = NAN;
    return false;
  }
  v >>= 3;
  *sample = v*0.25;
  if(!celcius) {
    *sample *= 9.0/5.0 + 32.0;
  }
  //os_printf("max6675_readTemp got result %d/100 \r\n" , (int)((*sample)*100.0));
  return true;
}

bool ICACHE_FLASH_ATTR
max6675_readTempAsString(char *buf, int bufLen, int *bytesWritten, bool celcius){
  float sample = 0.0;
  bool rv = max6675_readTemp(&sample, celcius);
  if(rv){
    *bytesWritten = max6675_float2string(100.0*sample, 100, buf, bufLen);
    if (celcius && (bufLen >= *bytesWritten+3)){
      buf[*bytesWritten] = 0xC2;  // UTF8 º
      buf[*bytesWritten+1] = 0xB0;
      buf[*bytesWritten+2] = 'C';
      buf[*bytesWritten+3] = 0;
      *bytesWritten += 3;
    }
  } else {
    *bytesWritten = 0;
    buf[0] = 0;
  }
  return rv;
}

// quick and dirty implementation of sprintf with %f
int ICACHE_FLASH_ATTR
max6675_float2string(float sample, int divisor, char *buf, int bufLen) {
  char localBuffer[256];

  char *sign;
  if (sample>=0){
    sign = "";
  } else {
    sign = "-";
    sample = -sample;
  }

  int h = (int)(sample / divisor);
  int r = (int)(sample - h*divisor);
  int size = 0;

  switch (divisor){
    case 1: size = os_sprintf(localBuffer, "%s%d",sign,h);
      break;
    case 10: size = os_sprintf(localBuffer, "%s%d.%01d",sign,h, r);
      break;
    case 100: size = os_sprintf(localBuffer, "%s%d.%02d",sign,h, r);
      break;
    case 1000: size = os_sprintf(localBuffer, "%s%d.%03d",sign,h, r);
      break;
    case 10000: size = os_sprintf(localBuffer, "%s%d.%04d",sign,h, r);
      break;
    case 100000: size = os_sprintf(localBuffer, "%s%d.%05d",sign,h, r);
      break;
    default:
      os_printf("dro_utils_float_2_string: could not recognize divisor: %d\r\n", divisor);
     return 0;
  }
  int l = size>bufLen?bufLen:size;
  strncpy(buf,localBuffer,l);
  buf[l] = 0;
  return l;
}


/******************************************************************************
 * FunctionName : max6675_init
 * Description  : initialize I2C bus to enable faked i2c operations
 * IMPORTANT    : Make sure that the UART/serial hardware isn't using the RX pin, if you use GPIO3 that is.
 * Parameters   : icsPin: inverted chip select pin
 *              : clockPin: clock pin
 *              : soPin:    slave out pin
 * Returns      : true if init succeeded
*******************************************************************************/
bool ICACHE_FLASH_ATTR
max6675_init(uint16_t icsPin, uint16_t clockPin, uint16_t soPin ) {

 ICS_PIN = icsPin;
 CLOCK_PIN = clockPin;
 SO_PIN = soPin;

  // Define each used pin as a GPIO
  if(easygpio_countBits(1<<ICS_PIN|1<<CLOCK_PIN|1<<SO_PIN)!=3) {
    os_printf("max6675_init:Error: you must provide exactly 3 unique I/O pins\n");
    return false;
  }

  if (!(easygpio_pinMode(ICS_PIN, NOPULL, OUTPUT) &&
        easygpio_pinMode(CLOCK_PIN, NOPULL, OUTPUT) &&
        easygpio_pinMode(SO_PIN, NOPULL, INPUT))) {
    return false;
  }

  max6675_digitalWrite(ICS_PIN, true);
  max6675_digitalWrite(CLOCK_PIN, false);
  GPIO_DIS_OUTPUT(SO_PIN);

  os_printf("\nInitiated max6675\n");
  os_printf(" !CS   = GPIO%d\n", ICS_PIN);
  os_printf(" CLOCK = GPIO%d\n", CLOCK_PIN);
  os_printf(" SO    = GPIO%d\n\n", SO_PIN);

  max6675_isConfigured = true;
  return true;
}
