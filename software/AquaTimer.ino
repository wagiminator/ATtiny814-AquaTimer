// ===================================================================================
// Project:   AquaTimer - Programmable Timer for Aquariums
// Version:   v1.0
// Year:      2021
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// AquaTimer is a programmable timer for 12V devices such as lighting, solenoid
// valves or pumps not only for aquariums. It has three switchable channels for
// currents up to 2A each and up to 5A in total. Connected lighting can be dimmed
// if desired and slowly faded in and out to simulate sunrises and sunsets. The
// internal RTC of the ATtiny is used as a clockwork in conjunction with a
// 32.768kHz crystal. A backup battery keeps the clock running even if the
// external power supply is interrupted. Settings are made using three buttons
// and the OLED display.
//
// References:
// -----------
// The I²C OLED implementation is based on TinyOLEDdemo:
// https://github.com/wagiminator/ATtiny13-TinyOLEDdemo
//
// The OLED font was adapted from Neven Boyanov and Stephen Denne:
// https://github.com/datacute/Tiny4kOLED
//
// The RTC and battery mode functions are based on USB-RTC:
// https://github.com/wagiminator/ATtiny814-USB-RTC
//
// Wiring:
// -------
//                            +-\/-+
//                      Vcc  1|°   |14  GND
//    OUT2 --- !SS AIN4 PA4  2|    |13  PA3 AIN3 SCK ---- OUT1
//    OUT3 ------- AIN5 PA5  3|    |12  PA2 AIN2 MISO --- BT_UP
//  BT_SET --- DAC AIN6 PA6  4|    |11  PA1 AIN1 MOSI --- BT_DOWN
//         ------- AIN7 PA7  5|    |10  PA0 AIN0 UPDI --- UPDI
// CRYSTAL -------- RXD PB3  6|    |9   PB0 AIN11 SCL --- OLED
// CRYSTAL ---------TXD PB2  7|    |8   PB1 AIN10 SDA --- OLED
//                            +----+
//
// Compilation Settings:
// ---------------------
// Core:    megaTinyCore (https://github.com/SpenceKonde/megaTinyCore)
// Board:   ATtiny1614/1604/814/804/414/404/214/204
// Chip:    ATtiny1614 or ATtiny814 or ATtiny414
// Clock:   5 MHz internal
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// Compile and upload the code.
//
// No Arduino core functions or libraries are used. To compile and upload without
// Arduino IDE download AVR 8-bit toolchain at:
// https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
// and extract to tools/avr-gcc. Use the makefile to compile and upload.
//
// Fuse Settings: 0:0x00 1:0x00 2:0x02 4:0x00 5:0xC5 6:0x04 7:0x00 8:0x00


// ===================================================================================
// Libraries and Definitions
// ===================================================================================

// Libraries
#include <avr/io.h>           // for GPIO
#include <avr/sleep.h>        // for sleep functions
#include <avr/interrupt.h>    // for interrupts
#include <util/delay.h>       // for delays

// Firmware version
#define VERSION     "v1.0"

// Pin definitions
#define BT_UP       PA2       // pin connected to "UP" button
#define BT_DOWN     PA1       // pin connected to "DOWN" button
#define BT_SET      PA6       // pin connected to "SET" button
#define OUT1        PA3       // set HIGH to switch on output channel 1
#define OUT2        PA4       // set HIGH to switch on output channel 2
#define OUT3        PA5       // set HIGH to switch on output channel 3

// Pin manipulation macros
enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB3};    // enumerate pin designators
#define pinOutput(x)      (&VPORTA.DIR)[((x)&8)>>1] |=  (1<<((x)&7))  // set pin to OUTPUT
#define pinLow(x)         (&VPORTA.OUT)[((x)&8)>>1] &= ~(1<<((x)&7))  // set pin to LOW
#define pinHigh(x)        (&VPORTA.OUT)[((x)&8)>>1] |=  (1<<((x)&7))  // set pin to HIGH
#define pinToggle(x)      (&VPORTA.IN )[((x)&8)>>1] |=  (1<<((x)&7))  // TOGGLE pin
#define pinRead(x)        ((&VPORTA.IN)[((x)&8)>>1] &   (1<<((x)&7))) // READ pin
#define pinPullup(x)      (&PORTA.PIN0CTRL)[(((x)&8)<<2)+((x)&7)] |=  PORT_PULLUPEN_bm
#define pinIntFalling(x)  (&PORTA.PIN0CTRL)[(((x)&8)<<2)+((x)&7)] |= PORT_ISC_FALLING_gc
#define pinDisable(x)     (&PORTA.PIN0CTRL)[(((x)&8)<<2)+((x)&7)] |= PORT_ISC_INPUT_DISABLE_gc
#define pinIntFlagClr(x)  (&VPORTA.INTFLAGS)[((x)&8)>>1] |= (1<<((x)&7))

// ===================================================================================
// Global Variables
// ===================================================================================

// Control variables and presets
typedef struct {
  uint8_t  pin;     // pin number of channel
  uint8_t  pwm;     // current pwm value
  uint8_t  fade;    // 0-no fade, 1-fade
  uint8_t  vmin;    // min PWM value (0..255)
  uint8_t  vmax;    // max PWM value (0..255)
  uint16_t on1;     // switch on  time (in min of the day), period 1
  uint16_t off1;    // switch off time (in min of the day), period 1
  uint16_t on2;     // switch on  time (in min of the day), period 2
  uint16_t off2;    // switch off time (in min of the day), period 2
} channeltype;

// Global variables
channeltype channel[] = {
  //PIN PWM FADE VMIN VMAX   ON1       OFF1      ON2       OFF2
  { OUT1, 0,  1,   0,  96,  9*60+ 0, 13*60+ 0, 15*60+ 0, 21*60+ 0 },
  { OUT2, 0,  1,   0,  96,  9*60+ 0, 13*60+ 0, 15*60+ 0, 21*60+ 0 },
  { OUT3, 0,  0,   0, 255,  9*60+30, 12*60+30, 15*60+30, 20*60+30 }
};

// ===================================================================================
// I2C Master Implementation (Write only)
// ===================================================================================

#define I2C_FREQ  400000UL                        // I2C clock frequency in Hz
#define I2C_BAUD  ((F_CPU / I2C_FREQ) - 10) / 2;  // simplified BAUD calculation

// I2C init function
void I2C_init(void) {
  TWI0.MBAUD   = I2C_BAUD;                    // set TWI master BAUD rate
  TWI0.MCTRLA  = TWI_ENABLE_bm;               // enable TWI master
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;        // set bus state to idle
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  TWI0.MADDR = addr;                          // start sending address
}

// I2C stop transmission
void I2C_stop(void) {
  while(~TWI0.MSTATUS & TWI_WIF_bm);          // wait for last transfer to complete
  TWI0.MCTRLB = TWI_MCMD_STOP_gc;             // send stop condition
}

// I2C transmit one data byte to the slave, ignore ACK bit
void I2C_write(uint8_t data) {
  while(~TWI0.MSTATUS & TWI_WIF_bm);          // wait for last transfer to complete
  TWI0.MDATA = data;                          // start sending data byte 
}

// ===================================================================================
// OLED Implementation
// ===================================================================================

// OLED definitions
#define OLED_ADDR       0x78                  // OLED write address
#define OLED_CMD_MODE   0x00                  // set command mode
#define OLED_DAT_MODE   0x40                  // set data mode
#define OLED_INIT_LEN   9                     // length of init command array

// OLED init settings
const uint8_t OLED_INIT_CMD[] = {
  0xC8, 0xA1,                                 // flip screen
  0xA8, 0x1F,                                 // set multiplex ratio
  0xDA, 0x02,                                 // set com pins hardware configuration
  0x8D, 0x14,                                 // set DC-DC enable
  0xAF                                        // display on
};

// Standard ASCII 5x8 font (adapted from Neven Boyanov and Stephen Denne)
const uint8_t OLED_FONT[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62,
  0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, 0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x00, 0xA0, 0x60, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x60, 0x60, 0x00, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00,
  0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x18, 0x14, 0x12, 0x7F, 0x10,
  0x27, 0x45, 0x45, 0x45, 0x39, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03,
  0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x36, 0x36, 0x00, 0x00,
  0x00, 0x56, 0x36, 0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x51, 0x09, 0x06, 0x32, 0x49, 0x59, 0x51, 0x3E,
  0x7C, 0x12, 0x11, 0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
  0x7F, 0x41, 0x41, 0x22, 0x1C, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09, 0x09, 0x09, 0x01,
  0x3E, 0x41, 0x49, 0x49, 0x7A, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00,
  0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40, 0x40,
  0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x3E, 0x41, 0x41, 0x41, 0x3E,
  0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46,
  0x46, 0x49, 0x49, 0x49, 0x31, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x3F, 0x40, 0x40, 0x40, 0x3F,
  0x1F, 0x20, 0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63,
  0x07, 0x08, 0x70, 0x08, 0x07, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x7F, 0x41, 0x41, 0x00,
  0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x04, 0x02, 0x01, 0x02, 0x04,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x01, 0x02, 0x04, 0x00, 0x20, 0x54, 0x54, 0x54, 0x78,
  0x7F, 0x48, 0x44, 0x44, 0x38, 0x38, 0x44, 0x44, 0x44, 0x20, 0x38, 0x44, 0x44, 0x48, 0x7F,
  0x38, 0x54, 0x54, 0x54, 0x18, 0x08, 0x7E, 0x09, 0x01, 0x02, 0x18, 0xA4, 0xA4, 0xA4, 0x7C,
  0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00,
  0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,
  0x7C, 0x08, 0x04, 0x04, 0x78, 0x38, 0x44, 0x44, 0x44, 0x38, 0xFC, 0x24, 0x24, 0x24, 0x18,
  0x18, 0x24, 0x24, 0x18, 0xFC, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x48, 0x54, 0x54, 0x54, 0x20,
  0x04, 0x3F, 0x44, 0x40, 0x20, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x1C, 0x20, 0x40, 0x20, 0x1C,
  0x3C, 0x40, 0x30, 0x40, 0x3C, 0x44, 0x28, 0x10, 0x28, 0x44, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C,
  0x44, 0x64, 0x54, 0x4C, 0x44, 0x08, 0x36, 0x41, 0x41, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00,
  0x00, 0x41, 0x41, 0x36, 0x08, 0x08, 0x04, 0x08, 0x10, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// OLED variables
uint8_t OLED_x, OLED_y;                       // current cursor position
uint8_t OLED_inv = 0;                         // "0xff" = print inverted

// OLED init function
void OLED_init(void) {
  I2C_init();                                 // initialize I2C first
  I2C_start(OLED_ADDR);                       // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                   // set command mode
  for(uint8_t i = 0; i < OLED_INIT_LEN; i++)
    I2C_write(OLED_INIT_CMD[i]);              // send the command bytes
  I2C_stop();                                 // stop transmission
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  I2C_start(OLED_ADDR);                       // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                   // set command mode
  I2C_write(xpos & 0x0F);                     // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));              // set high nibble of start column
  I2C_write(0xB0 | (ypos & 0x07));            // set start page
  I2C_stop();                                 // stop transmission
  OLED_x = xpos; OLED_y = ypos;               // set the cursor variables
}

// OLED clear rest of the current line
void OLED_clearLine(void) {
  I2C_start(OLED_ADDR);                       // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                   // set data mode
  while(OLED_x++ < 128) I2C_write(OLED_inv);  // clear rest of the line
  I2C_stop();                                 // stop transmission
  if(++OLED_y > 3) OLED_y = 0;                // calculate next line
  OLED_setCursor(0, OLED_y);                  // set cursor to star of next line
}

// OLED clear screen
void OLED_clearScreen(void) {
  OLED_setCursor(0, 0);                       // set cursor to home position
  for(uint8_t i=4; i; i--) OLED_clearLine();  // clear all 4 lines
}

// OLED plot a single character
void OLED_plotChar(char c) {
  uint16_t ptr = c - 32;                      // character pointer
  ptr += ptr << 2;                            // -> ptr = (ch - 32) * 5;
  I2C_write(OLED_inv);                        // write space between characters
  for(uint8_t i=5 ; i; i--) {                 // character consists of 5 vertical lines
    if(OLED_inv) I2C_write(~OLED_FONT[ptr++]);// plot inverted character line
    else         I2C_write( OLED_FONT[ptr++]);// plot character line
  }
  OLED_x += 6;                                // update cursor
  if(OLED_x > 122) {                          // line end ?
    I2C_stop();                               // stop data transmission
    OLED_setCursor(0,++OLED_y);               // set next line start
    I2C_start(OLED_ADDR);                     // start transmission to OLED
    I2C_write(OLED_DAT_MODE);                 // set data mode
  }
}

// OLED print a single character
void OLED_printChar(char c) {
  I2C_start(OLED_ADDR);                       // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                   // set data mode
  OLED_plotChar(c);                           // plot the character
  I2C_stop();                                 // stop transmission
}

// OLED print a string (by default from program memory)
void OLED_printString(const char* p) {
  I2C_start(OLED_ADDR);                       // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                   // set data mode
  while(*p) OLED_plotChar(*p++);              // print each character of the string
  I2C_stop();                                 // stop transmission
}

// OLED print 8-bit value as 2-digit decimal (BCD conversion by substraction method)
void OLED_printDec(uint8_t value) {
  I2C_start(OLED_ADDR);                       // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                   // set data mode
  uint8_t digitval = 0;                       // start with digit value 0
  while(value >= 10) {                        // if current divider fits into the value
    digitval++;                               // increase digit value
    value -= 10;                              // decrease value by divider
  }
  OLED_plotChar(digitval + '0');              // print first digit
  OLED_plotChar(value + '0');                 // print second digit
  I2C_stop();                                 // stop transmission
}

// OLED BCD conversion array
const uint16_t DIVIDER[] = {100, 10, 1};

// OLED print 8-bit value as 3-digital decimal (BCD conversion by substraction method)
void OLED_printDec3(uint8_t value) {
  uint8_t leadflag = 0;
  I2C_start(OLED_ADDR);                         // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                     // set data mode
  for(uint8_t digit = 0; digit < 3; digit++) {  // 3 digits
    uint8_t digitval = 0;                       // start with digit value 0
    uint16_t divider = DIVIDER[digit];          // current divider
    while(value >= divider) {                   // if current divider fits into the value
      leadflag = 1;                             // end of leading spaces
      digitval++;                               // increase digit value
      value -= divider;                         // decrease value by divider
    }
    if(leadflag || (digit == 2)) OLED_plotChar('0' + digitval);   // print the digit
    else OLED_plotChar(' ');                    // print leading space
  }
  I2C_stop();                                   // stop transmission
}

// ===================================================================================
// RTC and Time Functions
// ===================================================================================

// Variables
typedef struct {
  uint8_t  second;
  uint8_t  minute;
  uint8_t  hour;
} time;

volatile time t;

// Convert string into integer (2 digits)
uint8_t str2dec(const char *p) {
  return( (*p++ - '0') * 10 + (*p - '0') );
}

// Get compile time to use as initial time
void TIME_init(void) {
  char *ptr = __TIME__;                       // format "23:59:01"
  t.hour = str2dec(ptr); ptr += 3;            // hour
  t.minute = str2dec(ptr); ptr += 3;          // minute
  t.second = str2dec(ptr);                    // second
}

// Setup external 32.768 kHz crystal and periodic interrupt timer (PIT)
void RTC_init(void) {
  _PROTECTED_WRITE(CLKCTRL_XOSC32KCTRLA, CLKCTRL_ENABLE_bm); // enable crystal
  RTC.CLKSEL      = RTC_CLKSEL_TOSC32K_gc;    // select external 32K crystal
  RTC.PITINTCTRL  = RTC_PI_bm;                // enable periodic interrupt
  RTC.PITCTRLA    = RTC_PERIOD_CYC32768_gc    // set period to 1 second
                  | RTC_PITEN_bm;             // enable PIT
}

// Interrupt service routine for PIT (fires every second)
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;                // clear interrupt flag
  if(++t.second > 59) {                       // inc second; >59 ?
    t.second -= 60;                           // reset second
    if(++t.minute > 59) {                     // inc minute; >59 ?
      t.minute -= 60;                         // reset minute
      if(++t.hour > 23) t.hour -= 24;         // inc hour; >23 ? -> reset
    }
  }
}

// ===================================================================================
// PWM Functions using TCA in Split Mode; Pins: PA3, PA4, PA5
// ===================================================================================

// PWM macros
#define PWM_enable(pin)     TCA0.SPLIT.CTRLB |=  (2<<(pin))
#define PWM_disable(pin)    TCA0.SPLIT.CTRLB &= ~(2<<(pin))
#define PWM_duty(pin,duty)  (&TCA0.SPLIT.HCMP0)[((pin)-3)<<1] = (duty)

// PWM init
void PWM_init(void) {
  pinOutput(OUT1);                            // set OUT1 pin as output
  pinOutput(OUT2);                            // set OUT2 pin as output
  pinOutput(OUT3);                            // set OUT3 pin as output
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;     // set split mode
  TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV16_gc// prescaler 16
                   | TCA_SPLIT_ENABLE_bm;     // start the timer
}

// PWM set duty cycle on pin (pin PB3..PB5, duty cycle 0..255)
void PWM_set(uint8_t pin, uint8_t duty) {
  if(duty < 255) {                            // duty cycle < 255 ?
    PWM_enable(pin);                          // enable PWM on pin
    PWM_duty(pin, duty);                      // set duty cycle
  } else {                                    // duty cycle = 255 ?
    pinHigh(pin);                             // preset pin HIGH
    PWM_disable(pin);                         // disable PWM on pin -> steady HIGH
  }
}

// PWM shut off; output steady LOW
void PWM_off(uint8_t pin) {
  pinLow(pin);                                // preset pin LOW
  PWM_disable(pin);                           // disable PWM on pin -> steady LOW
}

// ===================================================================================
// ADC Supply Voltage Measurement
// ===================================================================================

// ADC init for VCC measurements
void ADC_init(void) {
  VREF.CTRLA  = VREF_ADC0REFSEL_1V1_gc;       // select 1.1V reference
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;         // set internal reference as ADC input
  ADC0.CTRLC  = ADC_REFSEL_VDDREF_gc          // set VCC as reference
              | ADC_PRESC_DIV4_gc;            // set prescaler for 1.25 MHz ADC clock
  ADC0.CTRLD  = ADC_INITDLY_DLY64_gc;         // delay to settle internal reference
  ADC0.CTRLA  = ADC_RESSEL_bm                 // select 8-bit resolution
              | ADC_ENABLE_bm;                // enable ADC, single shot
}

// ADC check supply voltage; return "true" if battery powered
uint8_t ADC_isBat(void) {
  ADC0.COMMAND = ADC_STCONV_bm;               // start sampling supply voltage
  while(ADC0.COMMAND & ADC_STCONV_bm);        // wait for ADC sampling to complete
  return(ADC0.RESL > 65);                     // true if VCC < 4.3V (65=256*1.1V/4.3V)
}

// ===================================================================================
// Battery Mode Implementation
// ===================================================================================

void BAT_check(void) {
  if(ADC_isBat()) {                           // in battery mode?
    TWI0.MCTRLA   = 0;                        // disable TWI
    PWM_off(OUT1); PWM_off(OUT2);PWM_off(OUT3); // switch off PWM
    SLPCTRL.CTRLA = SLPCTRL_SMODE_PDOWN_gc    // set sleep mode to power down
                  | SLPCTRL_SEN_bm;           // enable sleep  
    while(ADC_isBat()) sleep_cpu();           // sleep as long as battery powered
    OLED_init();                              // init OLED
    OLED_clearScreen();                       // clear screen
  }
}

// ===================================================================================
// Button Implementation
// ===================================================================================

// Global variables
int16_t count, countMin, countMax, countStep;
uint8_t circ;
#define CNT_MAX 16

// Init button pins
void BT_init(void) {
  pinPullup(BT_UP);                           // set pullup on "UP"   button pin
  pinPullup(BT_DOWN);                         // set pullup on "DOWN" button pin
  pinPullup(BT_SET);                          // set pullup on "SET"  button pin
  pinIntFalling(BT_SET);                      // enable falling edge interrupt
}

// Check SET button; returns "1" only if button is newly pressed
uint8_t SETpressed(void) {
  static uint8_t lastbutton = 1;
  if(lastbutton && !pinRead(BT_SET)) {
    lastbutton = 0;
    _delay_ms(1);
    return 1;
  }
  if(!lastbutton && pinRead(BT_SET)) {
    lastbutton = 1;
    _delay_ms(1);
  }
  return 0;
}

// Set start values for button simulated rotary encoder
void setRotary(int16_t rmin, int16_t rmax, int16_t rstep, int16_t rval, uint8_t circle) {
  countMin  = rmin;
  countMax  = rmax;
  countStep = rstep;
  count     = rval;
  circ      = circle;
}

// Read current value of button simulated rotary encoder
uint8_t getRotary(void) {
  static uint8_t CNT_up   = 0;
  static uint8_t CNT_down = 0;

  if(!pinRead(BT_UP)) {
    if((!CNT_up) || (++CNT_up > (CNT_MAX << 1))) {
      CNT_up = (CNT_up >> 1) + 1;
      count += countStep;
    }
  } else CNT_up = 0;
  
  if(!pinRead(BT_DOWN)) {
    if((!CNT_down) || (++CNT_down > (CNT_MAX << 1))) {
      CNT_down = (CNT_down >> 1) + 1;
      count -= countStep;
    }
  } else CNT_down = 0;

  if(circ) {
    if(count < countMin) count = countMax;
    if(count > countMax) count = countMin;
  } else {
    if(count < countMin) count = countMin;
    if(count > countMax) count = countMax;
  }
  
  return count;
}

// Interrupt service routine for SET button press
ISR(PORTA_PORT_vect) {
  pinIntFlagClr(BT_SET);                      // clear interrupt flag
}

// ===================================================================================
// Info Screens
// ===================================================================================

// Menu items
const char *MainItems[]     = { "Setup Menu", "Set Time", "Setup Channel 1",
                                "Setup Channel 2", "Setup Channel 3",
                                "Direct Control", "Return" };

// Print specified time (hour:minute)
void printTime(uint8_t h, uint8_t m) {
  OLED_printDec(h); OLED_printChar(':'); OLED_printDec(m);
}

// Print specified time (min of the day)
void printMinute(uint16_t m) {
  OLED_printDec(m/60); OLED_printChar(':'); OLED_printDec(m%60);
}

// Print current time (hour:minute:second)
void printCurrentTime(void) {
  OLED_printDec(t.hour);   OLED_printChar(':');
  OLED_printDec(t.minute); OLED_printChar(':');
  OLED_printDec(t.second);
}

// Print channel number
void printChannel(uint8_t channel) {
  OLED_printChar(channel + '1');
  OLED_printString(": ");
}

// Main Screen
void Screen_main(void) {
  OLED_setCursor(0, 0);
  OLED_printString("AquaTimer    ");
  printCurrentTime();
}

// Menu Screen
uint8_t MenuScreen(const char *Items[], uint8_t numberOfItems, uint8_t selected) {
  uint8_t lastselected = selected;
  int8_t  arrow = 0;
  if(selected) arrow = 1;
  setRotary(0, numberOfItems - 2, -1, selected, 0);
  OLED_setCursor(0, 0);
  OLED_printString(Items[0]);
  OLED_printString(" - "); OLED_printString(VERSION);
  OLED_clearLine();

  do {
    selected = getRotary();
    arrow += selected - lastselected;
    if(arrow < 0) arrow = 0;
    if(arrow > 2) arrow = 2;
    lastselected = selected;
    OLED_setCursor(0, 1);
    for(uint8_t i=0; i<3; i++) {
      if(arrow == i) OLED_inv = 0xff;
      uint8_t drawnumber = selected + i + 1 - arrow;
      if(drawnumber < numberOfItems)
        OLED_printString(Items[drawnumber]);
      OLED_clearLine();
      OLED_inv = 0x00;
    }
  } while(!SETpressed());

  return selected;
}

// Set time screen
uint16_t SetTimeScreen(uint16_t dmin, uint8_t xpos, uint8_t ypos) {
  uint8_t h = dmin / 60;
  uint8_t m = dmin % 60;
  OLED_setCursor (xpos, ypos);
  printTime(h, m);
  
  setRotary(0, 23, 1, h, 1);
  OLED_inv = 0xff;
  do {
    h = getRotary();
    OLED_setCursor (xpos, ypos);
    OLED_printDec(h);
    _delay_ms(10);
  } while(!SETpressed());
  OLED_inv = 0x00;
  OLED_setCursor (xpos, ypos);
  OLED_printDec(h);

  setRotary(0, 59, 1, m, 1);
  OLED_inv = 0xff;
  xpos += 6 * 3;
  do {
    m = getRotary();
    OLED_setCursor (xpos, ypos);
    OLED_printDec(m);
    _delay_ms(10);
  } while(!SETpressed());
  OLED_inv = 0x00;
  OLED_setCursor (xpos, ypos);
  OLED_printDec(m);

  return h * 60 + m;
}

// Set value screen
uint8_t SetValueScreen(uint8_t val, uint8_t xpos, uint8_t ypos) {
  setRotary(0, 255, 1, val, 0);
  OLED_inv = 0xff;
  do {
    val = getRotary();
    OLED_setCursor(xpos, ypos);
    OLED_printDec3(val);
    _delay_ms(5);
  } while(!SETpressed());
  OLED_inv = 0x00;
  OLED_setCursor(xpos, ypos);
  OLED_printDec3(val);
  return val;
}

// Set current time screen
void SetCurrenTimeScreen(void) {
  uint16_t cTime = 60*t.hour + t.minute;
  OLED_clearScreen();
  OLED_printString("  Set Current Time  ");
  cTime    = SetTimeScreen(cTime, 46, 2);
  t.hour   = cTime / 60;
  t.minute = cTime % 60;
  t.second = 0;
}

// Setup channel Screen
void SetupChannelScreen(uint8_t chan) {
  OLED_clearScreen();
  OLED_printString("Setup Channel "); OLED_printChar(chan + '1');
  OLED_setCursor(0, 1);
  OLED_printString("Period 1: ");  printMinute(channel[chan].on1);
  OLED_printChar('-');             printMinute(channel[chan].off1);
  OLED_printString("Period 2: ");  printMinute(channel[chan].on2);
  OLED_printChar('-');             printMinute(channel[chan].off2);
  OLED_printString("PWM Range: "); OLED_printDec3(channel[chan].vmin);
  OLED_printString(" -  ");        OLED_printDec3(channel[chan].vmax);

  channel[chan].on1  = SetTimeScreen(channel[chan].on1,  60, 1);
  if(channel[chan].on1 > 23*60+58) {
    channel[chan].on1 = 23*60+58;
    OLED_setCursor(60, 1); printMinute(channel[chan].on1);
  }
  channel[chan].off1 = SetTimeScreen(channel[chan].off1, 96, 1);
  if(channel[chan].off1 < channel[chan].on1) {
    channel[chan].off1 = channel[chan].on1;
    OLED_setCursor(96, 1); printMinute(channel[chan].off1);
  }
  channel[chan].on2  = SetTimeScreen(channel[chan].on2,  60, 2);
  if(channel[chan].on2 <= channel[chan].off1) {
    channel[chan].on2 = channel[chan].off1 + 1;
    OLED_setCursor(60, 2); printMinute(channel[chan].on2);
  }
  channel[chan].off2 = SetTimeScreen(channel[chan].off2, 96, 2);
  if(channel[chan].off2 < channel[chan].on2) {
    channel[chan].off2 = channel[chan].on2;
    OLED_setCursor(96, 2); printMinute(channel[chan].off2);
  }
  channel[chan].vmin = SetValueScreen(channel[chan].vmin, 66, 3);
  channel[chan].vmax = SetValueScreen(channel[chan].vmax,108, 3);
  if(channel[chan].vmin > channel[chan].vmax) {
    channel[chan].vmax = channel[chan].vmin;
    OLED_setCursor(108, 3); OLED_printDec3(channel[chan].vmax);
  }
  while(!SETpressed());

  OLED_clearScreen();
  OLED_printString("Setup Channel "); OLED_printChar(chan + '1');
  OLED_setCursor(0, 2); OLED_printString("Fading:");
  setRotary(0, 1, 1, channel[chan].fade, 1);
  do {
    channel[chan].fade = getRotary();
    OLED_setCursor(48, 2);
    if(channel[chan].fade) OLED_printString("YES");
    else                   OLED_printString("NO ");
    _delay_ms(10);
  } while(!SETpressed());
}

// Control screen
void ControlScreen(void) {
  OLED_clearScreen();
  OLED_printString("Direct Control"); OLED_clearLine();
  OLED_printString("UP:   Switch all on  ");
  OLED_printString("SET:  Return to menu ");
  OLED_printString("DOWN: Switch all off ");
  do {
    if(!pinRead(BT_UP)) {
      for(uint8_t i=0; i<3; i++) PWM_set(channel[i].pin, channel[i].vmax);
    } else if(!pinRead(BT_DOWN)) {
      for(uint8_t i=0; i<3; i++) PWM_set(channel[i].pin, channel[i].vmin);
    }
  } while(!SETpressed());
  for(uint8_t i=0; i<3; i++) PWM_set(channel[i].pin, channel[i].pwm);
}

// ===================================================================================
// Main Function
// ===================================================================================

int main(void) {
  // Setup
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 3);     // set clock frequency to 5 MHz
  TIME_init();                                // set time to compile time
  BT_init();                                  // setup button pins
  ADC_init();                                 // init ADC
  RTC_init();                                 // init RTC
  PWM_init();                                 // init PWM
  OLED_init();                                // init OLED
  OLED_clearScreen();                         // clear screen
  sei();                                      // enable global interrupts
  
  // Loop
  while(1) {                                  // loop until forever                         
    BAT_check();                              // check if battery powered
    Screen_main();                            // draw main screen
    uint16_t cTime = 60*t.hour + t.minute;    // calculate time in min of day

    // Set channels and print info on main screen
    for(uint8_t i=0; i<3; i++) {
      printChannel(i);
      if(  ((cTime >= channel[i].on1) && (cTime < channel[i].off1)) 
        || ((cTime >= channel[i].on2) && (cTime < channel[i].off2))) {
        if(channel[i].fade) {
          if(channel[i].pwm < channel[i].vmax) {
            channel[i].pwm++;
            OLED_printString("Fading in ...     ");
          } else {
            OLED_printString("ON  until    ");
            if(cTime < channel[i].off1) printMinute(channel[i].off1);
            else                        printMinute(channel[i].off2);
          }
        } else {
          channel[i].pwm = channel[i].vmax;
          OLED_printString("ON  until    ");
          if(cTime < channel[i].off1) printMinute(channel[i].off1);
          else                        printMinute(channel[i].off2);
        }
      } else {
        if(channel[i].fade) {
          if(channel[i].pwm > channel[i].vmin) {
            channel[i].pwm--;
            OLED_printString("Fading out ...    ");
          } else {
            OLED_printString("OFF until    ");
            if((cTime < channel[i].on1) || (cTime >= channel[i].on2)) 
                  printMinute(channel[i].on1);
            else  printMinute(channel[i].on2);
          }
        } else {
          channel[i].pwm = channel[i].vmin;
          OLED_printString("OFF until    ");
          if((cTime < channel[i].on1) || (cTime >= channel[i].on2)) 
                  printMinute(channel[i].on1);
          else    printMinute(channel[i].on2);
        }
      }
      PWM_set(channel[i].pin, channel[i].pwm);
    }

    // Check SET button; jump into menu if pressed
    if(SETpressed()) {
      uint8_t repeat = 1;
      uint8_t selection = 0;
      while(repeat) {
        selection = MenuScreen(MainItems, 7, selection);
        switch(selection) {
          case 0:   SetCurrenTimeScreen(); break;
          case 1:   SetupChannelScreen(0); break;
          case 2:   SetupChannelScreen(1); break;
          case 3:   SetupChannelScreen(2); break;
          case 4:   ControlScreen(); break;
          default:  repeat = 0; break;
        }
      }
      OLED_clearScreen();
    }

    // Idle CPU until PIT wakes it up the next second
    else {
      SLPCTRL.CTRLA = SLPCTRL_SMODE_IDLE_gc   // set sleep mode to IDLE
                    | SLPCTRL_SEN_bm;         // enable sleep
      sleep_cpu();                            // sleep until next second
    }
  }
}
