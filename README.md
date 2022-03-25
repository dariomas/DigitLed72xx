DigitLed72xx Controller
=======================

Version 0.0.7

Arduino MAX7219/7221 Library using hardware SPI

From an idea by [Leonardo SAMMARTANO](https://github.com/SaLeeC), made with Leo's help and support.

Copyright (c) 2020 DarioMas & SaLe

Portions from [MAX7219](https://github.com/csdexter/MAX7219), [DigitLedDisplay](https://github.com/ozhantr/DigitLedDisplay), [LedControl](https://github.com/wayoda/LedControl)

Download
--------

The lastest binary version of the Library is always available from the
[DigitLed72xx Release Page](https://github.com/dariomas/DigitLed72xx/releases)

Install
-------

The library can be installed using the [standard Arduino library install procedure](http://arduino.cc/en/Guide/Libraries).

Recent Arduino IDE releases include the Library Manager for easy installation. Otherwise, to download, click the DOWNLOAD ZIP button, uncompress and rename the uncompressed folder DigitLed72xx. Confirm that the DigitLed72xx folder contains DigitLed72xx.cpp and DigitLed72xx.h. Place the DigitLed72xx library folder your ArduinoSketchFolder/Libraries/ folder. You may need to create the Libraries subfolder if its your first library. Restart the IDE.


Usage
-----

 DigitLed72xx(LOAD_PIN, NCHIPS);
   The Constructor initializes communication, takes the display out of test mode, clears the screen and sets intensity.
   Intensity is set at minimum but can be coinfigured by setBright(brightness, nDevice)


Device number (nDevice) start at 0 up to number of devices controlled,
When parameter nDevice >= NCHIPS (number of devices initialised in constructor), the function broadcasts to all devices

 printDigit(number, startDigit, nDevice)
   This method displays a number value (charachter) from position startDigit (0=right most digit, 7=left most digit)
   To print minus sign on negative numbers use:
 #define PRINT_DIGIT_NEG 1
 #include <DigitLed72xx.h>

 write(address, data, nDevice)
   This method displays a single character (symbol) by sending a code directly to the driver


MIT license, all text above must be included in any redistribution
