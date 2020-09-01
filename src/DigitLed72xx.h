/* Arduino MAX7219/7221 Library
 * See the README file for author and licensing information. In case it's
 * missing from your distribution, use the one here as the authoritative
 * version: https://github.com/dariomas/DigitLed72xx/blob/master/README.md
 *
 * This library is for use with Maxim's MAX7219 and MAX7221 LED driver chips.
 * Austria Micro Systems' AS1100/1106/1107 is a pin-for-pin compatible and is
 * also supported.
 * Thankyoy to Leonardo SAMMARTANO for help and support.
 * 
 * See the example sketches to learn how to use the library in your code.
 *
 * This is the main include file for the library.
 * 
 * ---------------------------------------------------------------------------
 * Copyright (c) 2020 Dariomas
 *
 * MIT license, all text here must be included in any redistribution.
 */

// Version 0.0.3

#pragma once

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#ifdef __has_include

#define INCLUDED_PGMSPACE
#if (__has_include(<avr/pgmspace.h>))
#include <avr/pgmspace.h>
#else
#include <pgmspace.h>
#endif

#if (__has_include(<array>))
#include <array>
#define STD_CAPABLE 1
#else
#define STD_CAPABLE 0
#endif

#endif

#ifndef INCLUDED_PGMSPACE
#include <avr/pgmspace.h>
#define INCLUDED_PGMSPACE
#endif

#include <SPI.h>

/**
 * @brief This class provied a control interface for MAX7219 and MAX7221 7-seg Led display drivers.
 * @details This Controller Class is mainly target at 7-Segment Led Displays.
 * @warning This object is not thread safe.
 * @note This library implements the 7-segment numeric LED display of 8 digits
 * 
 * The MAX7219/MAX7221 are compact, serial input/output common-cathode display drivers that interface
 * microprocessors (µPs) to 7-segment numeric LED displays of up to 8 digits, bar-graph displays, or 64 
 * individual LEDs
 * Datasheet  : https://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf
 *
 * Library Description
 *
 *  - The host communicates with the MAX72xx using hardware SPI 
 *  - CS/LOAD Pin can be configured on constructor
 *
 *
 * Usage
 *
 * These methods are exposed for use:
 *  
 *  1. Constructor
 *  The Constructor initializes communication, takes the display out of test mode, clears the screen and sets intensity.
 *  Intensity is set at minimum but can be coinfigured by setBright(brightness, nDevice)
 *  
 *  2. printDigit(number, startDigit, nDevice)
 *  This method displays a number value (charachter) from position startDigit (0=right most digit, 7=left most digit)
 *  
 *  3. write(address, data, nDevice)
 *  This method displays a single character (symbol) by sending a code directly to the driver
 *  
 */

#ifndef SPIMAXSPEED
#define SPIMAXSPEED (20000000)
#endif
// Uncomment to print minus sign on negative numbers
// #define PRINT_DIGIT_NEG 1

// the MAX7219 address map (datasheet table 2)
#define NOOP_ADDR (0x00)
#define DECODEMODE_ADDR (0x09)
#define BRIGHTNESS_ADDR (0x0A)
#define SCANLIMIT_ADDR (0x0B)
#define SHUTDOWN_ADDR (0X0C)
#define DISPLAYTEST_ADDR (0x0F)

#define OP_OFF (0x0)
#define OP_ON (0x1)
#define MAX72d (0x1)        // '-'
#define MAX72b (0x0)        // ' '
#define DP_FLAG (B10000000) // '.'

class DigitLed72xx
{
private:
    /*! 
         * @brief Send out a single command to the device 
         * 
         */
    void spiTransfer(byte opcode, byte data, unsigned char addr);
    void spiWrite(byte opcode, byte data);
    void shiftAll(unsigned char nDevice = 1);

    /*!
        * @brief Stop the SPI and sends a shutdown command to the MAX7219(s).
        * 
        */
    void end(void);

    // Pointer to the SPI class
    SPIClass *spi;
    /* This one is driven LOW for chip selection */
    unsigned char pinLOAD_CS;
    /* The maximum number of devices we use */
    unsigned char maxDevices = 1;

    byte *_digitLimit;
    const byte charTable[10] = {
        B01111110, B00110000, B01101101, B01111001, B00110011, B01011011, B01011111, B01110000, B01111111, B01111011};

public:
    /**
     * @brief Construct a new DigitLed72xx controller for use with hardware SPI
     * 
     * @param csPin The pin to select the device (CS)
     * @param nDevice The number of connected devices that can be controled (defualt 1)
     * @param spiClass The oject that drives the SPI hardware (a SPIClass instance)
     **/
    DigitLed72xx(unsigned char csPin = SS, unsigned char nDevice = 1, SPIClass &spiClass = SPI);

    /**
        * @brief This is the destructor, it simply calls end() to free memory.
        * 
        */
    ~DigitLed72xx()
    {
        end();
    }

    /*!
     *    @brief  Initializes SPI bus and sets CS pin high
     *
     */
    inline void begin(void);

    /**
     * @brief Set the Intensity of the whole display to the given value.
     * @note if you want to save more energy disable segments you don't need or lower the brightness.
     * @param brightness the new brightness of the chain. (0..15)
     * @param nDevice the address of the device to control
     **/
    void setBright(unsigned char brightness, unsigned char nDevice = 0);

    /**
     * @brief Set the number of digits to be displayed.
     * @note See datasheet for side-effects of the scanlimit on the brightness of the display.
     * @param limit The number of digits to be displayed (1..8)
     * @param nDevice The device which should be addressed
     */
    void setDigitLimit(unsigned char limit, unsigned char nDevice = 0);
    inline void setLimit(unsigned char limit, unsigned char nDevice = 0)
    {
        setDigitLimit(limit, nDevice);
    }
    //    inline unsigned char getLimit(unsigned char nDevice = 0) {
    //      return _digitLimit[nDevice];
    //    }

    /** 
         * @brief Display a single digit on a 7-Segment Display
         * @note There are only a few characters that make sense here :
         *  '0','1','2','3','4','5','6','7','8','9',
         *  '-',' ' 
         * Params:
         * @param nDevice The address of the display
         * @param digit  The position of the digit on the display (0..7)
         * @param value  The value to be displayed. (0x00..0x09)
         * @param dp sets The decimal point.
         **/
    void setDigit(unsigned char digit, byte value, byte dp = 0, unsigned char nDevice = 0);

    /** 
         * @brief Display a whole number on a 7-Segment Display
         * @note There are only a few characters that make sense here :
         *  '0','1','2','3','4','5','6','7','8','9',
         *  '-',' ' 
         * Params:
         * @param nDevice Theaddress of the display
         * @param startDigit  The position of the digit on the display (0..7)
         * @param number  The value to be displayed and sets the decimal point every thousands.
         */
    void printDigit(long number, unsigned char nDevice = 0, byte startDigit = 0);
    //    inline void printDigits(long number, unsigned char nDevice = 0);

    /*
 * Light up 'dot' at position.
 * @param position: dot position from range <0, 7>
 */
    //void MAX7219_display_dot(uint8_t position);

    /*
 * Turn off 'dot' at position.
 * @param position: dot position from range <0, 7>
 */
    //void MAX7219_clear_dot(uint8_t position);

    /* 
         * @brief Display a character on a 7-Segment display.
         * @note decode mode raw
         * Params:
         * nDevice address of the display
         * digit  the position of the character on the display (0..7)
         * value  the character to be displayed. 
         * dp sets the decimal point.
         */
    //    void setChar(unsigned char digit, byte value, byte dp = 0, unsigned char nDevice = 0);

    /** 
         * @brief Switch all Leds on the display off. 
         * @param nDevice address of the display to control
         **/
    void clear(unsigned char nDevice = 0);

    /** 
         * @brief Set the shutdown (power saving) mode for the device
         * @param nDevice The address of the display to control
         * @note the device goes into power-down mode or normal operation.
         **/
    void on(unsigned char nDevice = 0);
    void off(unsigned char nDevice = 0);

    /*
         * @brief Gets the number of devices attached
         * Returns :
         * char  the number of devices on this LedControl
         */
    //char getDeviceCount();

    void write(byte address, byte data, unsigned char nDevice = 0);
};
