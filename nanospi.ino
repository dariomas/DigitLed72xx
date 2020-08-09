/*
  Basic code for using Maxim MAX7219/MAX7221 with Arduino.
  Wire the Arduino and the MAX7219/MAX7221 together as follows:
  | Arduino   | MAX7219/MAX7221 |
  | --------- | --------------- |
  | MOSI (11) | DIN (1)         |
  | SCK (13)  | CLK (13)        |
  | I/O (7)*  | LOAD/CS (12)    |
    * - This should match the LOAD_PIN constant defined below.
  
  For the rest of the wiring follow the wiring diagram found in the datasheet.
  
  Datasheet: http://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf
  
 */
#include <SPI.h>
#include "DigitLed72xx.h"

// What pin on the Arduino connects to the LOAD/CS pin on the MAX7219/MAX7221
#define LOAD_PIN 7
#define NCHIP 2

DigitLed72xx ld = DigitLed72xx(LOAD_PIN, NCHIP);
  
void setup() 
{
  ld.on(10);
  
  ld.printDigits(76543210, 1);

}

int n = 0;
void loop() 
{
   n = n % (NCHIP + 1);
  /* Prints data to the display */
  ld.printDigits(12345678, n);
  delay(900);
  ld.clear(n);

  ld.printDigits(22222222, n);
  delay(500);
  ld.clear(n);

  ld.printDigits(44444444, n);
  delay(500);
  ld.clear(n);

  for (int i = 0; i < 1001; i++) {
    ld.printDigits(i, n);

    /* Start From Digit 4 */
    ld.printDigit(i, 4, n);
    delay(20);
  }

  for (int i = 0; i <= 10; i++) {
    /* Display off */
    ld.off(n);
    delay(150);

    /* Display on */
    ld.on(n);
    delay(150);
  }

  /* Clear all display value */
  ld.clear(n);
  delay(500);

  for (long i = 0; i < 2000; i++) {
    ld.printDigits(i, n);
    i += 10;
    delay(50);
  }

  for (int i = 0; i <= 20; i++) {
    /* Select Digit 5 and write B01100011 */
    ld.write(5, B01100011, n);
    delay(200);

    /* Select Digit 5 and write B00011101 */
    ld.write(5, B00011101, n);
    delay(200);
  }

  /* Clear all display value */
  ld.clear(n);
  delay(500);
  
  ++n;
}
