# Arduino Feather 32u4II-LoRA Sensor Node (DHT22)

This code is the implementation of a simple temperature and humidity measurement
sensor node using the IBM LoRA LMIC (the specific pin mappings are configured to
be compatible with the BSFrance 32u4II-LORA board - if you use this development
board be sure to solder the appropriate bridge(s) for the DIO pins - normally you
are required to solder DIO1 an I/O pin - in this code it's assumed to be
connected to I/O pin 6. If you want to use FSK mode too you also have to connect
DIO2 to any available I/O pin).

## Dependencies

This code depends on:

* The Arduino LMIC (https://github.com/matthijskooijman/arduino-lmic). This is
  licensed mostly under Eclipse Public License and LGPL
* The Adafruit DHT sensor library (https://github.com/adafruit/DHT-sensor-library).
  This library is licensed under MIT license
* Rocketscream's LowPower library (https://github.com/rocketscream/Low-Power) licensed
  under Creative Commons Attribution-ShareAlike 3.0 Unported License

## License

All code in this repository (note the licenses of libraries above though) is licensed
under the BSD license.

## A word of caution

There are many different versions of the mentiones 32u4II-LoRA boards. Even the
same manufacturer is not consistent with the DIO pin mappings. Double check that
these pins are connected correctly if anything doesn't work.