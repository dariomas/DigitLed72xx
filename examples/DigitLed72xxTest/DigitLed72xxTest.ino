/*
  Basic code for using Maxim MAX7219/MAX7221 with Arduino.
  Version 0.0.3
  Wire the Arduino and the MAX7219/MAX7221 together as follows:
  | Arduino   | MAX7219/MAX7221 |
  | --------- | --------------- |
  | MOSI (11) | DIN (1)         |
  | SCK (13)  | CLK (13)        |
  | I/O (7)*  | LOAD/CS (12)    |
    * - This should match the LOAD_PIN constant defined below.
  
  For the rest of the wiring follow the wiring diagram found in the datasheet.
  
  Datasheet: http://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf

   Thankyoy to Leonardo SAMMARTANO for help and support.
 */
#include <SPI.h>
#include <DigitLed72xx.h>

// What pin on the Arduino connects to the LOAD/CS pin on the MAX7219/MAX7221
#define LOAD_PIN 7
#define NCHIP 2

DigitLed72xx ld = DigitLed72xx(LOAD_PIN, NCHIP);
  
void setup() 
{
  ld.on(2);
  
  ld.printDigit(76543210);
  delay(500);
  ld.printDigit(43218765, 1);
  delay(500);

}

int n = 0;
void loop() 
{
   n = n % (NCHIP + 1);
  /* Prints data to the display */
  ld.printDigit(12345678, n);
  delay(900);
  ld.clear(n);

  ld.printDigit(-2222222, n);
  delay(500);
  ld.clear(n);

  ld.printDigit(44444444, n);
  delay(500);
  ld.clear(n);

  for (int i = 0; i < 1001; i++) {
    ld.printDigit(i, n);

    /* Start From Digit 4 */
    ld.printDigit(i, n, 4);
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
    ld.printDigit(i, n);
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
