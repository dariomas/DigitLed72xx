#include "DigitLed72xx.h"
/**
 * @brief This class provied a control interface for MAX7219 and MAX7221 7-seg Led display drivers.
 * @details This Controller Class is mainly target at 7-Segment Led Displays.
 * @warning This object is not thread safe yet.
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
 *  - SPI speed can be configured in DigitLed72xxController.h
 *
 */

        /* 
         * Create a new controler 
         * Params :
         * csPin    CS/LOAD pin for selecting the device 
         * nDevice  number of devices that can be controled
         */

DigitLed72xx::DigitLed72xx(unsigned char csPin, unsigned char nDevice, SPIClass& spiClass):spi(&spiClass), pinLOAD_CS(csPin), maxDevices(nDevice)
{
  _digitLimit = new byte[maxDevices];
  for (byte i=0; i < maxDevices; ++i) _digitLimit[i] = 8;
  // Set load pin to output
  //pinLOAD_CS = csPin;
  pinMode(pinLOAD_CS, OUTPUT);
  digitalWrite(pinLOAD_CS,HIGH);
  
  //spi = &spiClass; 
    
  // Start SPI
  spi->begin();
#if defined(SPI_HAS_TRANSACTION)
  spi->beginTransaction(SPISettings (SPIMAXSPEED, MSBFIRST, SPI_MODE0));
#else
  spi->setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz clock
  spi->setBitOrder(MSBFIRST);
  spi->setDataMode(SPI_MODE0);
#endif // SPI_HAS_TRANSACTION
  
  //we go into shutdown-mode on startup
  spiWrite(SHUTDOWN_ADDR, OP_OFF);
  spiWrite(DISPLAYTEST_ADDR, OP_OFF);
  //scanlimit is set to max on startup
  spiWrite(SCANLIMIT_ADDR, 7); // show 8 digits
  //decode is done in BCD Code B
  spiWrite(DECODEMODE_ADDR,0xFF);
  //clearSegments
  for (unsigned char i = 1; i<9; ++i)
    spiWrite(i, MAX72b); // blank
  // min power
  spiWrite(BRIGHTNESS_ADDR,0);
//  spiWrite(SHUTDOWN_ADDR, OP_ON); 
  if (nDevice > 1)
  {
    //maxDevices = nDevice;
    // shiftout no_op
    shiftAll();
  }

#if defined(SPI_HAS_TRANSACTION)
  spi->endTransaction();
#endif // SPI_HAS_TRANSACTION
}

void DigitLed72xx::shiftAll(unsigned char nDevice = 1)
  {    
    // shiftout no_op
    for(unsigned char i=nDevice;i < maxDevices;++i) // (maxDevices - nDevice)
      spiWrite(NOOP_ADDR, OP_OFF);
  }

inline void DigitLed72xx::end ()
  {
    //sendToAll shutdown mode (ie. turn it off)
    write(SHUTDOWN_ADDR, OP_OFF, maxDevices ); 
    spi->end ();
  }

void DigitLed72xx::setBright(unsigned char brightness, unsigned char nDevice) 
{
    if (brightness > 15)
        return;
    write(BRIGHTNESS_ADDR, brightness, nDevice);
}

        /* 
         * Set the number of digits to be displayed.
         * See datasheet for sideeffects of the scanlimit on the brightness
         * of the display.
         * Params :
         * limit  number of digits to be displayed (1..8)
         * nDevice address of the display to control
         */    
void DigitLed72xx::setDigitLimit(unsigned char limit, unsigned char nDevice) 
{
  if((limit == 0) || (limit > 8)) return;
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

inline void DigitLed72xx::on(unsigned char nDevice) 
{
  write(SHUTDOWN_ADDR, OP_ON, nDevice);
}

inline void DigitLed72xx::off(unsigned char nDevice) 
{
  write(SHUTDOWN_ADDR, OP_OFF, nDevice);
}

void DigitLed72xx::setDigit(unsigned char digit, byte value, byte dp, unsigned char nDevice)
{
//  if(digit > _digitLimit[nDevice])
//        return;
  if (dp>0)
      dp = DP_FLAG;
  write(digit,value|dp, nDevice);
}

void DigitLed72xx::printDigit(long number, byte startDigit, unsigned char nDevice)
{
  unsigned long temp, num;
  num = abs(number);
  char zero = 0;
 
  for ( byte digit=1; num > 0; ++digit ) {
        temp = num / 10 ;
        byte parsed = num-10*temp;
        byte dp = 0;
        if ((digit == 4) || (digit == 7)) parsed|=DP_FLAG;
        setDigit(digit, parsed, 0, nDevice);
        num = temp ;
  }
  
//  String figure = String(number);
//  int figureLength = figure.length();
//
//  int parseInt;
//  char str[2];
//  for(unsigned char i = 0; i < figure.length(); i++) {
//    str[0] = figure[i];
//    str[1] = '\0';
//    parseInt = (int) strtol(str, NULL, 10);
//    //table(figureLength - i + startDigit, parseInt);
//    unsigned char digit = figureLength - i + startDigit;
//    if (digit < 9)
//        setDigit(digit, parseInt, 0, nDevice);
//  }
}

inline void DigitLed72xx::printDigits(long number, unsigned char nDevice)
{
  printDigit(number, 0, nDevice);
}

void DigitLed72xx::setChar(unsigned char digit, byte value, byte dp, unsigned char nDevice)
{
  if(nDevice > maxDevices || digit >= _digitLimit[nDevice])
        return;
  
}

//void DigitLed72xx::showDots(uint8_t dots, uint8_t* digits){
//    for(int i = 0; i < 4; ++i){
//        digits[i] |= (dots & 0x80);
//        dots <<= 1;
//    }
//}
void DigitLed72xx::write(byte address, byte data, unsigned char nDevice)
{
  if(nDevice >= maxDevices)
  {
#if defined(SPI_HAS_TRANSACTION)
      spi->beginTransaction(SPISettings (SPIMAXSPEED, MSBFIRST, SPI_MODE0));
#endif // SPI_HAS_TRANSACTION
      spiWrite(address,data);
      shiftAll();
#if defined(SPI_HAS_TRANSACTION)
      spi->endTransaction();
#endif // SPI_HAS_TRANSACTION
  }
  else
      spiTransfer(address, data, nDevice);
}

/* Write to one of the drivers registers. No-ops are sent to all other 
   drivers in the chain.
   Driver is the driver number in the chain 
   */
void DigitLed72xx::spiTransfer(byte opcode, byte data, unsigned char addr)
{
#if defined(SPI_HAS_TRANSACTION)
  spi->beginTransaction(SPISettings (SPIMAXSPEED, MSBFIRST, SPI_MODE0));
#endif // SPI_HAS_TRANSACTION

  // Set LOAD/CS to LOW
  digitalWrite(pinLOAD_CS, LOW);
  
  // Send the register address
  spi->transfer(opcode);
  // Send the value
  spi->transfer(data);
      
  // shiftout no_op
  for(unsigned char i=0; i < maxDevices; ++i)
  {
      if(i == addr)
      {
        // Tell chip to load in data
        digitalWrite(pinLOAD_CS, HIGH);
        //__asm("nop");
        // Ensure LOAD/CS is LOW
        digitalWrite(pinLOAD_CS, LOW);        
      } 
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
 * @param address The register to load data into
 * @param value   Value to store in the register
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


/* Loads a text string into the output buffer using the seven segment 
   character set */
//void HCMAX7219::print7Seg(char TextString[], unsigned int Offset)
//{
//  unsigned int _StringLength;
//  unsigned int bufferindex;
//  byte charindex;
//  
//  _StringLength = strlen(TextString);
//  
//  /* Set output buffer pointer */
//  if(Offset < DISPLAYBUFFERSIZE)
//  {
//    bufferindex = Offset;
//  }else
//  {
//    bufferindex = DISPLAYBUFFERSIZE;
//  }
//  
//  /* If text runs beyond the output buffer then crop it */
//  charindex = 0;
//  if (Offset > DISPLAYBUFFERSIZE)
//    charindex = Offset - (DISPLAYBUFFERSIZE);
//
//  /* Copy text into output buffer */
//  while(bufferindex != 0 && charindex != _StringLength)
//  {
//     bufferindex--;
//   DisplayBuffer[bufferindex] = SevenSegChar[TextString[charindex] - 32];
//     charindex++;
//  } 
//}

/* Loads an integer into the output buffer using the seven segment 
   character set */
//void HCMAX7219::print7Seg(long number, unsigned int Offset)
//{
//  char Digits[10];
//  byte index = 0;
//  long Temp;
//  unsigned int bufferindex;
//
//  Temp = number;
//  
// /*Is the number negative ? If so then remove the sign */
//  if (Temp < 0)
//    Temp *= -1;
//
//  /* Is the number zero ? */
//  if (Temp == 0)
//  {
//    Digits[index] = '0';
//    index++;
//  }else
//  {
//    /* Convert the number to an ASCII decimal string */
//    while (Temp)
//    {
//      Digits[index] = (Temp % 10) + 48;
//      Temp /= 10;
//      index++;
//    } 
//   
//    /* If the number was negative add the sign */ 
//    if (number < 0)
//    {
//      Digits[index] = '-';
//      index++;
//    }
//  }
//  
//  /* Set output buffer pointer */
//  if(Offset < DISPLAYBUFFERSIZE)
//  {
//    bufferindex = Offset;
//  }else
//  {
//    bufferindex = DISPLAYBUFFERSIZE;
//  } 
//   
//  /* If text runs beyond the output buffer then crop it */ 
//  if (Offset > DISPLAYBUFFERSIZE)
//    index = index - (Offset - DISPLAYBUFFERSIZE);
//   
//  /* Copy text into output buffer */
//  while(index && bufferindex)
//  {
//    index--;
//    bufferindex--;
//    DisplayBuffer[bufferindex] = SevenSegChar[Digits[index]-32];
//  }
//}


/* Loads a decimal number into the output buffer using the seven segment 
   character set */
//void HCMAX7219::print7Seg(long number, byte decimalPlace, unsigned int Offset)
//{
//
//  char Digits[10];
//  byte index = 0;
//  long Temp;
//  unsigned int bufferindex;
//
//  Temp = number;
//  
// /*Is the number negative ? If so then remove the sign */
//  if (Temp < 0)
//    Temp *= -1;
//
//  /* Is the number zero ? */  
//  if (Temp == 0)
//  {
//    Digits[index] = '0';
//    index++;
//  }else
//  {
//  /* Convert the number to an ASCII decimal string */
//    while (Temp)
//    {
//      Digits[index] = (Temp % 10) + 48;
//      Temp /= 10;
//      index++;
//    } 
//  }
//  
//  /* If decimal place is at the beginning of the number then pad it 
//     with a zero */
//  if (decimalPlace == index)
//  {
//    Digits[index] = '0';
//    index++;
//  }
//   
//  /* If the number was negative add the sign */ 
//  if (number < 0)
//  {
//    Digits[index] = '-';
//    index++;
//  }
//  
//  /* Set output buffer pointer */
//  if(Offset < DISPLAYBUFFERSIZE)
//  {
//    bufferindex = Offset;
//  }else
//  {
//    bufferindex = DISPLAYBUFFERSIZE;
//  } 
//   
//  /* If text runs beyond the output buffer then crop it */  
//  if (Offset > DISPLAYBUFFERSIZE)
//    index = index - (Offset - DISPLAYBUFFERSIZE);
//
//  /* Copy text into output buffer */
//  while(index && bufferindex)
//  {
//    index--;
//    bufferindex--;
//    if(decimalPlace !=0 && index == decimalPlace)
//    {
//      DisplayBuffer[bufferindex] = SevenSegChar[Digits[index]-32] |  SevenSegChar[14];
//    }else
//    {
//      DisplayBuffer[bufferindex] = SevenSegChar[Digits[index]-32];
//    } 
//  }
//}
