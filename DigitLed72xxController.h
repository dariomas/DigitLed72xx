#pragma once

#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#ifdef __has_include

    #define INCLUDED_PGMSPACE
    #if(__has_include(<avr/pgmspace.h>))
        #include <avr/pgmspace.h>
    #else
        #include <pgmspace.h>
    #endif

    #if(__has_include(<array>))
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
 * @warning This object is not thread safe yet.
 * @note This library implements the 7-segment numeric LED display of 8 digits
 * 
 * @todo make it threading safe
 * 
 * The MAX7219/MAX7221 are compact, serial input/output common-cathode display drivers that interface
 * microprocessors (µPs) to 7-segment numeric LED displays of up to 8 digits, bar-graph displays, or 64 
 * individual LEDs
 * Datasheet  : https://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf
 *
 * Library Description
 *
 *  - The host communicates with the MAX72xx using hardware SPI 
 *  - CS/LOAD Pin can be configured in .h
 *
 *
 * Usage
 *
 * Three methods are exposed for use:
 *  
 *  1. Begin
 *  This method initializes communication, takes the display out of test mode, clears the screen and sets intensity.
 *  Intensity is set at maximum but can be coinfigured in max7219.h
 *  
 *  2. DisplayChar(Digit, Value, DP)
 *  This method displays a single value (charachter) in position DIGIT (0=right most digit, 7=left most digit)
 *  
 *  3. DisplayText(Text, Justify)
 *  This method displays a text string (Text) either right justified (Justify=0) ot left justified (Justify=1)
 *  
 */

#ifndef SPIMAXSPEED
    #define SPIMAXSPEED (1000000)
#endif

// the MAX7219 address map (datasheet table 2)
#define NOOP_ADDR        (0x00)
#define DECODEMODE_ADDR  (0x09)
#define BRIGHTNESS_ADDR  (0x0A)
#define SCANLIMIT_ADDR   (0x0B)
#define SHUTDOWN_ADDR    (0X0C)
#define DISPLAYTEST_ADDR (0x0F)

#define OP_OFF   (0x0)
#define OP_ON    (0x1)
#define MAX72d      (0xA) // '-'
#define MAX72E      (0xB) // 'E'
#define MAX72H      (0xC) // 'H'
#define MAX72L      (0xD) // 'L'
#define MAX72P      (0xE) // 'P'
#define MAX72b      (0xF) // ' '
#define DP_FLAG  (B10000000) // '.'

class DigitLed72xxController {
    private :
        /* Send out a single command to the device */
        void spiTransfer(byte opcode, byte data, unsigned char addr);
        void spiWrite(byte opcode, byte data);
        void shiftAll(unsigned char nDevice = 1);

       /* This one is driven LOW for chip selection */
        unsigned char pinLOAD_CS;
        /* The maximum number of devices we use */
        unsigned char maxDevices = 1;

        unsigned char _digitLimit = 8;
        
  public:
       /*
        * @brief Construct a new LedController for use with hardware SPI
        * 
        * @param csPin The pin to select the device (CS)
        * @param nDevice The number of connected devices that can be controled (defualt 1)
        */
    DigitLed72xxController(unsigned char csPin, unsigned char nDevice = 1);

        /*
        * Description:
        *   This is the destructor, it simply calls end().
        */
    ~DigitLed72xxController() { end(); };

       /*
        * Description:
        *    Clears the SRAM and sends a shutdown command to the MAX7219(s).
        */
     inline void end(void);

     /**
     * @brief Set the Intensity of the whole display chain to the given value.
     * @note if you want to save more energy disable segments you don't need or lower the brightness.
     * @param brightness the new brightness of the chain. (0..15)
     * @param nDevice the address of the device to control
     */
    void setBright(unsigned char brightness, unsigned char nDevice = 1);
 
    /**
     * @brief Set the number of digits to be displayed.
     * @note See datasheet for sideeffects of the scanlimit on the brightness of the display.
     * @param limit The number of digits to be displayed (1..8)
     * @param nDevice The device which should be addressed
     */
    void setDigitLimit(unsigned char limit, unsigned char nDevice = 1);
    

        /* 
         * @brief Display a single digit on a 7-Segment Display
         * @note There are only a few characters that make sense here :
         *  '0','1','2','3','4','5','6','7','8','9',
         *  '-','E','H','L','P',' ' 
         * Params:
         * nDevice address of the display
         * digit  the position of the digit on the display (0..7)
         * value  the value to be displayed. (0x00..0x0F)
         * dp sets the decimal point.
         */
    void setDigit(unsigned char digit, byte value, byte dp = 0, unsigned char nDevice = 1);
    void printDigit(long number, byte startDigit = 0, unsigned char nDevice = 1);
    inline void printDigits(long number, unsigned char nDevice = 1);
    
/**
 * Light up 'dot' at position.
 * @param position: dot position from range <0, 7>
 */
//void MAX7219_display_dot(uint8_t position);

/**
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
    void setChar(unsigned char digit, byte value, byte dp = 0, unsigned char nDevice = 1);

        /* 
         * @brief Switch all Leds on the display off. 
         * @param nDevice address of the display to control
         */
    void clear(unsigned char nDevice = 1);

        /* 
         * @brief Set the shutdown (power saving) mode for the device
         * @param nDevice The address of the display to control
         * @note the device goes into power-down mode or normal operation.
         */    
    void on(unsigned char nDevice = 1);
    void off(unsigned char nDevice = 1);   



        /*
         * @brief Gets the number of devices attached
         * Returns :
         * char  the number of devices on this LedControl
         */
        //char getDeviceCount();

    void write(byte address, byte data, unsigned char nDevice = 1);
}; 
