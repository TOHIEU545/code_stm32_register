#include "Lcd_i2c.h"

uint8_t _displaycontrol =  LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
uint8_t _addr = 0x27 << 1;
uint8_t _backlightval = LCD_BACKLIGHT;
uint8_t _cols = 0;
uint8_t _rows = 0;

// -- Delay --
void Delay_I2C_ms(uint16_t ms){
	SysTick->LOAD = 72000u - 1u;
	SysTick->VAL = 0u;
	SysTick->CTRL = 1u | 1u << 2;
	while(ms--){
	while((SysTick->CTRL & 1<<16) == 0);
	}
	SysTick -> CTRL = 0;
}

void Delay_I2C_us(uint16_t us){
	SysTick->LOAD = 72u - 1u;
	SysTick->VAL = 0u;
	SysTick->CTRL = 1u | 1u << 2;
	while(us--){
	while((SysTick->CTRL & 1<<16) == 0);
	}
	SysTick -> CTRL = 0;
}
// --- **************** ---


// --- LCD-specific I2C configuration ---
void LCD_I2C_Init(void){
	// Clock for Port B anh I2C1
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	GPIOB->CRL &= ~((0xF << 24) | (0xF << 28));
	GPIOB->CRL |= ((0xF << 24) | (0xF << 28));
	
	I2C1->CR2 = 36;
	I2C1->CCR = 180;
	I2C1->TRISE = 37;

	I2C1->CR1 |= I2C_CR1_PE; // ON i2C
}
void LCD_I2C_Start(void){
	I2C1->CR1 |= I2C_CR1_START;              // On bit Start
	while(!(I2C1->SR1 & I2C_SR1_SB));        // Waiting for Start to be sent
	(void)I2C1->SR1;                         // Read sr1 clear SB flag
}
void LCD_I2C_SendAddress(uint8_t address){
	I2C1->DR = address;                      // Send addres include bit R/W
	while(!(I2C1->SR1 & I2C_SR1_ADDR));      // Waiting for Address to be sent
	(void)I2C1->SR1;
	(void)I2C1->SR2;                         // Read SR1 and SR2 clear ADDR flag
}
void LCD_I2C_SendData(uint8_t data){
	while(!(I2C1->SR1 & I2C_SR1_TXE));       // Waiting buffer empty
	I2C1->DR = data;
}
void LCD_I2C_Stop(void){
	while(!(I2C1->SR1 & I2C_SR1_TXE));       // Waiting buffer empty
	while(!(I2C1->SR1 & I2C_SR1_BTF));       // Wait for last byte transmission complete
	I2C1->CR1 |= I2C_CR1_STOP;
}
// --- ************************** ---


// --- Send 4-bit LCD data and control bits with EN pulse ---
// This function starts I2C, sends a byte (data + backlight status), then stops I2C.
void expanderWrite(uint8_t _data){
	LCD_I2C_Start();
	LCD_I2C_SendAddress(_addr);
	LCD_I2C_SendData(_data | _backlightval);
	LCD_I2C_Stop();
}

// Create an Enable (EN) pulse to latch data into the LCD
void pulseEnable(uint8_t _data){
	expanderWrite(_data | ENABLE);
	Delay_I2C_ms(1);
	
	expanderWrite(_data & ~ENABLE);
	Delay_I2C_ms(1);
}

// This sends 4 bits data with 4 bits control, then triggers the EN pulse.
void write4bits(uint8_t _value){
	expanderWrite(_value);
	pulseEnable(_value);
}

// Send a full 8-bit command or data to the LCD in 4-bit mode
void send(uint8_t _value, uint8_t mode){
	uint8_t upper = _value & 0xF0;
	uint8_t lower = (_value << 4) & 0xF0;
	write4bits(upper | mode);
	write4bits(lower | mode);
}
// --- ***************************** ---


// --- send Command & Data ---
void LCD_SendData(uint8_t data_){
	send(data_, RS_DATA);
}

void LCD_SendCmd(uint8_t cmd){
	send(cmd, RS_CMD);
}
// --- ******************** ---

// --- Functions rewritten according to arduino library ---
// Send LCD start command string according to HD44780 standard in 4-bit mode.
void LCD_Begin(uint8_t lines){
	Delay_I2C_ms(50);

	write4bits(0x30);
	Delay_I2C_ms(5);
	write4bits(0x30);
	Delay_I2C_ms(1);
	write4bits(0x30);
	Delay_I2C_ms(5);

	write4bits(0x20);
	Delay_I2C_ms(5);
	
	if(lines == 1){
		LCD_SendCmd(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_1LINE);
		Delay_I2C_us(40);
	}
	else if(lines > 1){
		LCD_SendCmd(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE);
		Delay_I2C_us(40);
	}
	LCD_Clear();
	LCD_display();
}

// Clear
void LCD_Clear(void){
	LCD_SendCmd(LCD_CLEARDISPLAY);
	Delay_I2C_us(40);
}

// Move cursor to the first pos (0, 0)
void LCD_Home(void){
	LCD_SendCmd(LCD_RETURNHOME);
	Delay_I2C_ms(2);
}

// OFF Display
void LCD_noDisplay(void){
	_displaycontrol &= ~LCD_DISPLAYON;
	LCD_SendCmd(LCD_DISPLAYCONTROL | _displaycontrol);
	Delay_I2C_us(40);
}

// ON Display
void LCD_display(void){
	_displaycontrol |= LCD_DISPLAYON;
	LCD_SendCmd(LCD_DISPLAYCONTROL | _displaycontrol);
	Delay_I2C_us(40);
}

// OFF blink cursor
void LCD_noBlink(void){
	_displaycontrol &= ~LCD_BLINKON;
	LCD_SendCmd(LCD_DISPLAYCONTROL | _displaycontrol);
	Delay_I2C_us(40);
}

// ON blink cursor
void LCD_blink(void){
	_displaycontrol |= LCD_BLINKON;
	LCD_SendCmd(LCD_DISPLAYCONTROL | _displaycontrol);
	Delay_I2C_us(40);
}

// OFF cursor
void LCD_noCursor(void){
	_displaycontrol &= ~LCD_CURSORON;
	LCD_SendCmd(LCD_DISPLAYCONTROL | _displaycontrol);
	Delay_I2C_us(40);
}

// ON cursor
void LCD_cursor(void){
	_displaycontrol |= LCD_CURSORON;
	LCD_SendCmd(LCD_DISPLAYCONTROL | _displaycontrol);
	Delay_I2C_us(40);
}

// Scroll screen content to the left
void LCD_scrollDisplayLeft(void){
	LCD_SendCmd(0x18);
	Delay_I2C_us(40);
}

// Scroll screen content to the right
void LCD_scrollDisplayRight(void){
	LCD_SendCmd(0x1C);
	Delay_I2C_us(40);
}

// Set auto-increment cursor and auto-shift screen
void LCD_SetEntryMode(uint8_t leftToRight, uint8_t shiftDisplay) {
	uint8_t mode = LCD_ENTRYMODESET;

	if (leftToRight) mode |= LCD_ENTRYLEFT;
	else              mode |= LCD_ENTRYRIGHT;

	if (shiftDisplay) mode |= LCD_ENTRYSHIFTINCREMENT;
	else              mode |= LCD_ENTRYSHIFTDECREMENT;

	LCD_SendCmd(mode);
	Delay_I2C_us(40);
}

// Set cursor position at specific column and row
void LCD_SetCursor(uint8_t col, uint8_t row){
	static const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (row > 3) row = 3;
	LCD_SendCmd(0x80 | (col + row_offsets[row]));
}


void LCD_SendString(const char* str){
	while(*str){
		LCD_SendData(*str++);
	}
}
// --- ******************* ---
