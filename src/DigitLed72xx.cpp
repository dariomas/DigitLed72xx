/* Arduino MAX7219/7221 Library
 * See the README file for author and licensing information. In case it's
 * missing from your distribution, use the one here as the authoritative
 * version: https://github.com/dariomas/DigitLed72xx/blob/master/README.md
 *
 * This library is for use with Maxim's MAX7219 and MAX7221 LED driver chips.
 * Austria Micro Systems' AS1100/1106/1107 is a pin-for-pin compatible and is
 * also supported.
 *  Thankyoy to Leonardo SAMMARTANO for help and support.
 * 
 * See the example sketches to learn how to use the library in your code.
 *
 * This is the main code file for the library.
 * See the header file for better function documentation.
 * 
 * ---------------------------------------------------------------------------
 * Copyright (c) 2020 Dariomas
 *
 * MIT license, all text here must be included in any redistribution.
 */

// Version 0.0.6

#include "DigitLed72xx.h"
/**
 * @brief This class provied a control interface for MAX7219 and MAX7221 7-seg Led display drivers.
 * @details This Controller Class is mainly target at 7-Segment Led Displays.
 * @warning This object is not thread safe.
 * @note This library implements the 7-segment numeric LED display of 8 digits
 * 
 * @todo ...
 * 
 * The MAX7219/MAX7221 are compact, serial input/output common-cathode display drivers that interface
 * microprocessors (µPs) to 7-segment numeric LED displays of up to 8 digits.
 * Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf
 *
 * Library Description
 *
 *  - The host communicates with the MAX72xx using hardware SPI 
 */

/*!
 * @brief Construct a new DigitLed72xx controller for use with hardware SPI
 * 
 * @param  csPin    CS/LOAD pin for selecting the device 
 * @param  nDevice  number of devices that can be controled
 * @param  spiClass instance of SPI class
 * 
 */
DigitLed72xx::DigitLed72xx(unsigned char csPin, unsigned char nDevice, SPIClass &spiClass) : spi(&spiClass), pinLOAD_CS(csPin), maxDevices(nDevice)
{
  _digitLimit = new byte[maxDevices];
  for (byte i = 0; i < maxDevices; ++i)
  {
    _digitLimit[i] = 8;
  }
  begin();
}

/*!
 *    @brief  Initializes SPI bus and sets CS pin high
 *
 */
inline void DigitLed72xx::begin(void)
{
  // Set load pin to output
  //pinLOAD_CS = csPin;
  pinMode(pinLOAD_CS, OUTPUT);
  digitalWrite(pinLOAD_CS, HIGH);

  // Start SPI
  spi->begin();
#if defined(SPI_HAS_TRANSACTION)
  spi->beginTransaction(SPISettings(SPIMAXSPEED, MSBFIRST, SPI_MODE0));
#else
  spi->setClockDivider(SPI_CLOCK_DIV16); // 1 MHz clock
  spi->setBitOrder(MSBFIRST);
  spi->setDataMode(SPI_MODE0);
#endif // SPI_HAS_TRANSACTION

  spiWrite(DISPLAYTEST_ADDR, OP_OFF);
  //we go into shutdown-mode on startup
  spiWrite(SHUTDOWN_ADDR, OP_OFF);
  //scanlimit is set to max on startup
  spiWrite(SCANLIMIT_ADDR, 7); // show 8 digits
  //decode is done in raw mode
  spiWrite(DECODEMODE_ADDR, 0);
  //clearSegments
  for (unsigned char i = 1; i < 9; ++i)
    spiWrite(i, MAX72b);        // blank
  spiWrite(BRIGHTNESS_ADDR, 0); // min power
                                //  spiWrite(SHUTDOWN_ADDR, OP_ON);
  if (maxDevices > 1)
    shiftAll(); // shiftout no_op

#if defined(SPI_HAS_TRANSACTION)
  spi->endTransaction();
#endif // SPI_HAS_TRANSACTION
}

void DigitLed72xx::end(void)
{
  if (_digitLimit)
    delete[] _digitLimit;
  //sendToAll shutdown mode (ie. turn it off)
  write(SHUTDOWN_ADDR, OP_OFF, maxDevices);
  spi->end();
}

void DigitLed72xx::shiftAll(unsigned char nDevice = 1)
{
  // shiftout no_op
  for (unsigned char i = nDevice; i < maxDevices; ++i) // (maxDevices - nDevice)
    spiWrite(NOOP_ADDR, OP_OFF);
}

void DigitLed72xx::setBright(unsigned char brightness, unsigned char nDevice)
{
  if (brightness > 15)
    return;
  write(BRIGHTNESS_ADDR, brightness, nDevice);
}

/*!
         * @brief Set the number of digits to be displayed.
         * See datasheet for sideeffects of the scanlimit on the brightness
         * of the display.
         * @param limit  number of digits to be displayed (1..8)
         * @param nDevice address of the display to control
         */
void DigitLed72xx::setDigitLimit(unsigned char limit, unsigned char nDevice)
{
  if ((limit == 0) || (limit > 8))
    return;
  for (unsigned char i = 0; i < maxDevices; ++i)
    if ((nDevice >= maxDevices) || (i == nDevice))
    {
      _digitLimit[i] = limit;
      spiTransfer(SCANLIMIT_ADDR, limit - 1, i);
    }
}

void DigitLed72xx::clear(unsigned char nDevice)
{
  for (unsigned char j = 0; j < maxDevices; ++j)
    if ((nDevice >= maxDevices) || (j == nDevice))
      //clearSegments
      for (unsigned char i = 1; i <= _digitLimit[j]; ++i)
        spiTransfer(i, MAX72b, j);
}

void DigitLed72xx::on(unsigned char nDevice)
{
  write(SHUTDOWN_ADDR, OP_ON, nDevice);
}

void DigitLed72xx::off(unsigned char nDevice)
{
  write(SHUTDOWN_ADDR, OP_OFF, nDevice);
}

void DigitLed72xx::setDigit(unsigned char digit, byte value, byte dp, unsigned char nDevice)
{
  if (digit > 8)
    return;
  if (dp > 0)
    dp = DP_FLAG;
  write(digit, value | dp, nDevice);
}

void DigitLed72xx::printDigit(long number, unsigned char nDevice, byte startDigit)
{
  unsigned long num = abs(number);
  byte digit = startDigit + 1;
  byte parsed = B01111110; //charTable[ 0 ];
  // shiftout no_op
  for (unsigned char i = 1; i < maxDevices; ++i)
  {
    spi->transfer(NOOP_ADDR);
    spi->transfer(OP_OFF);
  }
  if (num == 0)
  {
    if (nDevice >= maxDevices)
    {
#if defined(SPI_HAS_TRANSACTION)
      spi->beginTransaction(SPISettings(SPIMAXSPEED, MSBFIRST, SPI_MODE0));
#endif // SPI_HAS_TRANSACTION
      spiWrite(digit, parsed);
      shiftAll();
#if defined(SPI_HAS_TRANSACTION)
      spi->endTransaction();
#endif // SPI_HAS_TRANSACTION
    }
    else
      spiTransfer(digit, parsed, nDevice);
  }
  else
  {
    for (; num > 0; ++digit)
    {
      unsigned long temp = num / 10;
      parsed = charTable[num - 10 * temp];
      num = temp;
      if (((digit - startDigit) == 4) || ((digit - startDigit) == 7))
        parsed |= DP_FLAG;
      if (nDevice >= maxDevices)
      {
#if defined(SPI_HAS_TRANSACTION)
        spi->beginTransaction(SPISettings(SPIMAXSPEED, MSBFIRST, SPI_MODE0));
#endif // SPI_HAS_TRANSACTION
        spiWrite(digit, parsed);
        shiftAll();
#if defined(SPI_HAS_TRANSACTION)
        spi->endTransaction();
#endif // SPI_HAS_TRANSACTION
      }
      else
        spiTransfer(digit, parsed, nDevice);
      if (digit == 8)
        break;
    }
#if defined(PRINT_DIGIT_NEG)
    if ((number < 0) && (digit <= 8))
      write(digit, MAX72d, nDevice);
#endif // PRINT_DIGIT_NEG
  }
}

//inline void DigitLed72xx::printDigits(long number, unsigned char nDevice)
//{
//  printDigit(number, 0, nDevice);
//}

//void DigitLed72xx::setChar(unsigned char digit, byte value, byte dp, unsigned char nDevice)
//{
//  if(nDevice > maxDevices || digit >= _digitLimit[nDevice])
//        return;
//
//}

void DigitLed72xx::write(byte address, byte data, unsigned char nDevice)
{
  if (nDevice >= maxDevices)
  {
#if defined(SPI_HAS_TRANSACTION)
    spi->beginTransaction(SPISettings(SPIMAXSPEED, MSBFIRST, SPI_MODE0));
#endif // SPI_HAS_TRANSACTION \
    // shiftout no_op
    for (unsigned char i = 1; i < maxDevices; ++i)
    {
      spi->transfer(NOOP_ADDR);
      spi->transfer(OP_OFF);
    }
    spiWrite(address, data);
    shiftAll();
#if defined(SPI_HAS_TRANSACTION)
    spi->endTransaction();
#endif // SPI_HAS_TRANSACTION
  }
  else
    spiTransfer(address, data, nDevice);
}

/*! 
 * @brief Write to one of the drivers registers. No-ops are sent to all other 
 * drivers in the chain.
 * @param addr is the driver number in the chain 
*/
void DigitLed72xx::spiTransfer(byte opcode, byte data, unsigned char addr)
{
#if defined(SPI_HAS_TRANSACTION)
  spi->beginTransaction(SPISettings(SPIMAXSPEED, MSBFIRST, SPI_MODE0));
#endif // SPI_HAS_TRANSACTION

  // Set LOAD/CS to LOW
  digitalWrite(pinLOAD_CS, LOW);

  // shiftout no_op
  for (unsigned char i = maxDevices; i > addr; --i)
  {
    spi->transfer(NOOP_ADDR);
    spi->transfer(OP_OFF);
  }

  // Send the register address
  spi->transfer(opcode);
  // Send the value
  spi->transfer(data);

  // shiftout no_op
  for (unsigned char i = 0; i < addr; ++i)
  {
    spi->transfer(NOOP_ADDR);
    spi->transfer(OP_OFF);
  }
  // Tell chip to load in data
  digitalWrite(pinLOAD_CS, HIGH);

#if defined(SPI_HAS_TRANSACTION)
  spi->endTransaction();
#endif // SPI_HAS_TRANSACTION
}

/**
 * Transfers data to a MAX7219/MAX7221 register.
 * 
 * @param opcode The register to load data into
 * @param data   Value to store in the register
 */
void DigitLed72xx::spiWrite(byte opcode, byte data)
{
  //Datasheet calls for 25ns between LOAD/#CS going low and the start of the
  //transfer, an Arduino running at 20MHz has a clock period of 50ns so no action needed.
  // Ensure LOAD/CS is LOW
  digitalWrite(pinLOAD_CS, LOW);

  // Send the register address
  spi->transfer(opcode);

  // Send the value
  spi->transfer(data);

  // Tell chip to load in data
  //__asm("nop");
  digitalWrite(pinLOAD_CS, HIGH);
}
