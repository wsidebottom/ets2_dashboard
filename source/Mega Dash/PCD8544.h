#ifndef PCD8544_h
#define PCD8544_h

#include "config.h"
#include "SPI.h"
#include "font_big.h"

/* PCD8544-specific defines: */
#define LCD_COMMAND  0
#define LCD_DATA     1

/* 84x48 LCD Defines: */
#define LCD_WIDTH   84 // Note: x-coordinates go wide
#define LCD_HEIGHT  48 // Note: y-coordinates go high
#define WHITE       0  // For drawing pixels. A 0 draws white.
#define BLACK       1  // A 1 draws black.

class PCD8544
{
	public:
	PCD8544(){};
	void begin(void);

	void setChar(char character, int x, int y, bool bw);
	void setStr(char * dString, int x, int y, bool bw);
	int printf(int x, int y, bool bw, char *fmt, ...);
	void setCharBig(char character, int x, int y, bool bw);
	void setStrBig(char * dString, int x, int y, bool bw);
	int printfBig(int x, int y, bool bw, char *fmt, ...);
	void write(uint8_t data_or_command, uint8_t data);

	void setContrast(uint8_t contrast);
	void invertDisplay();
	void updateDisplay();
	void gotoXY(int x, int y);
	void clearDisplay(bool bw = WHITE);

	void setBitmap(const char * bitArray);
	void setCircle (int x0, int y0, int radius, bool bw, int lineThickness);
	void setRect(int x0, int y0, int x1, int y1, bool fill, bool bw);
	void setLine(int x0, int y0, int x1, int y1, bool bw);
	void clearPixel(int x, int y);
	void setPixel(int x, int y);
	void setPixel(int x, int y, bool bw);
};
#endif
