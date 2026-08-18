#pragma once

#include "../obkdef.h"
#include <stdint.h>


#define rgb565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

#define RED    rgb565(255,   0,   0) // 0xf800
#define GREEN  rgb565(  0, 255,   0) // 0x07e0
#define BLUE   rgb565(  0,   0, 255) // 0x001f
#define BLACK  rgb565(  0,   0,   0) // 0x0000
#define WHITE  rgb565(255, 255, 255) // 0xffff
#define GRAY   rgb565(128, 128, 128) // 0x8410
#define YELLOW rgb565(255, 255,   0) // 0xFFE0
#define CYAN   rgb565(  0, 156, 209) // 0x04FA
#define PURPLE rgb565(128,   0, 128) // 0x8010

typedef enum {DIRECTION0, DIRECTION90, DIRECTION180, DIRECTION270} DIRECTION;

typedef enum {
	SCROLL_RIGHT = 1,
	SCROLL_LEFT = 2,
	SCROLL_DOWN = 3,
	SCROLL_UP = 4,
} SCROLL_TYPE_t;

typedef struct {
	/* interface settings */
	obk_spidevice_t     spidev;
	int16_t 			dc_pin;        /* Data/command line pin */
	int16_t 			bl_pin;        /* Backlight pin */
	int16_t 			rs_pin;        /* Reset pin */
	/* display params */
	uint16_t 			width;
	uint16_t 			height;
	uint16_t 			offsetx;
	uint16_t 			offsety;
	uint16_t            rotation;
	uint16_t 			font_direction;
	uint16_t 			font_scale;
//	uint16_t 			font_fill;
//	uint16_t 			font_fill_color;
//	uint16_t 			font_underline;
//	uint16_t 			font_underline_color;		
	bool 				use_frame_buffer;
	uint16_t 		   *frame_buffer;
} TFT_t;

