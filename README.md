# esp8266_max6675
A MAX6675 thermo-couple driver for esp8266.

It is a direct port of the Arduino library found at https://github.com/adafruit/MAX6675-library

It uses 3 GPIO pins so it is possible to use an esp-1.

But you will have to reappropriate your RX pin.

MAX6675| ESP8266
-------|------------------
!CS | GPIO2
SCK | GPIO0
SO | GPIO3 (RX)



