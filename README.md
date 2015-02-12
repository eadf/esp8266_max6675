# esp8266_max6675
A MAX6675 thermo-couple driver for esp8266.

It is a direct port of the Arduino library found at https://github.com/adafruit/MAX6675-library

It uses 3 GPIO pins so it is possible to use an esp-1.

But you will have to reappropriate your RX pin. Make sure that you don't have your max6675 connected to the RX pin while flashing or it will vent the magic smoke.

### default pins
MAX6675| ESP8266
-------|------------------
!CS | GPIO2
SCK | GPIO0
SO | GPIO3 (RX)



I've made all of the pin assignments configurable, so now you must tell the init method what pins you want to use.
```
void max6675_init(uint16_t icsPin, uint16_t clockPin, uint16_t soPin);
```

###Building and installing:

First you need to install the sdk and the easy way of doing that is to use [esp_open_sdk.](https://github.com/pfalcon/esp-open-sdk)

You can put that anywhere you like (/opt/local/esp-open-sdk, /esptools etc etc)

Then you could create a small setenv.sh file, containing the location of your newly compiled sdk and other platform specific info;
```
export SDK_BASE=/opt/local/esp-open-sdk/sdk
export PATH=${SDK_BASE}/../xtensa-lx106-elf/bin:${PATH}
export ESPPORT=/dev/ttyO0  
```
(or setup your IDE to do the same)

In the root of this project create a soft link Makefile -> Makefile.[mac,linux]
```
ln -s Makefile.linux Makefile
```
You don't *have* to do this, it just makes it more convenient to run make (instead of *make -f Makefile.linux*)

To make a clean build, flash and connect to the esp console you just do this in a shell:
```
source setenv.sh # This is only needed once per session
make clean && make test
```

You won't be needing esptool, my makefiles only uses esptool.py (provided by esp-open-sdk)
(The makefiles under examples still uses esptool. I'll fix that soon)

I have tested this with sdk v0.9.5 and v0.9.4 (linux & mac makefile)
