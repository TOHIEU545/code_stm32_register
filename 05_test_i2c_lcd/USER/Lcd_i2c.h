#ifndef Lcd_i2c_h
#define Lcd_i2c_h
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"

extern uint8_t _addr;
extern uint8_t _displayfunction;
extern uint8_t _displaycontrol;
extern uint8_t _displaymode;
extern uint8_t _cols;
extern uint8_t _rows;
extern uint8_t _charsize;
extern uint8_t _backlightval;

// Definition of control bit IC PCF8574T ---
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00
#define ENABLE        0x04
#define RS_DATA       0x01
#define RS_CMD        0x00

// --- commands ---
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// --- flags for display entry mode ---
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// --- flags for display on/off control ---
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// --- flags for display/cursor shift ---
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// --- flags for function set ---
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// -- Delay --
void Delay_I2C_ms(uint16_t ms);
void Delay_I2C_us(uint16_t us);

// --- LCD-specific I2C configuration ---
void LCD_I2C_Init(void);
void LCD_I2C_Start(void);
void LCD_I2C_SendAddress(uint8_t address);
void LCD_I2C_SendData(uint8_t data);
void LCD_I2C_Stop(void);

// --- ****** ---
void send(uint8_t _value, uint8_t mode);
void write4bits(uint8_t _value);
void expanderWrite(uint8_t _data);
void pulseEnable(uint8_t _data);


void LCD_SendData(uint8_t data_);
void LCD_SendCmd(uint8_t cmd);

// -- Functions in LiquidCrystal.h ---
void LCD_Begin(uint8_t lines);
void LCD_SetEntryMode(uint8_t leftToRight, uint8_t shiftDisplay);
void LCD_Clear(void);
void LCD_Home(void);
void LCD_noDisplay(void);
void LCD_display(void);
void LCD_noBlink(void);
void LCD_blink(void);
void LCD_noCursor(void);
void LCD_cursor(void);
void LCD_scrollDisplayLeft(void);
void LCD_scrollDisplayRight(void);
void LCD_SetCursor(uint8_t col, uint8_t row);

void LCD_SendString(const char* str);







#endif