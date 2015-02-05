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

##Required:

esp_iot_sdk_v0.9.4_14_12_19

I have not tested this with v0.9.5. I tested a [clean sdk 0.9.5 install](https://github.com/pfalcon/esp-open-sdk) with one of the basic examples ([blinky](https://github.com/esp8266/source-code-examples)). It compiled and uploaded fine but the esp had a infinite crash loop with some message about "MEM CHK FAIL" on the console. So i threw the whole sdk out (aint nobody got time fo dat). I will try upgrading the sdk again once [mqtt](https://github.com/tuanpmt/esp_mqtt) upgrades to 0.9.5+.
