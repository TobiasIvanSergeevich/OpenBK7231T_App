#include "../obkdef.h"
#include "../obkhelper.h"
#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"

#include "../hal/hal_pins.h"

#include "drv_spibus.h"
#include "drv_st7789.h"
#include "drv_st7789_font7x15.h"


#define SWAPBYTE(u16) (((u16>>8)&0xFF) | (u16<<8))

static TFT_t tft;

void lcdInit(TFT_t * dev);
void lcdDrawPixel(TFT_t * dev, uint16_t x, uint16_t y, uint16_t color);
void lcdDrawFillRect(TFT_t * dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcdDisplayOn(TFT_t * dev);
void lcdDisplayOff(TFT_t * dev);
void lcdSetBrightness(TFT_t * dev, uint8_t brightness);
void lcdDrawBitmap(TFT_t * dev, uint16_t x,uint16_t y,uint16_t w,uint16_t h, uint16_t *bitmap, uint16_t rcolor, uint16_t bkcolor);
int lcdDrawChar(TFT_t * dev, uint16_t x, uint16_t y, uint8_t ascii, uint16_t charColor, uint16_t bkgColor);
int lcdDrawString(TFT_t * dev, uint16_t x, uint16_t y, uint8_t * ascii, uint16_t charColor, uint16_t bkgColor);

//startDriver HWSPI; startDriver ST7789 hspi NA IO15 IO2 NA|CH4 135 240 52 40 180
// startDriver ST7789 [SPI_NAME] [CS_PIN] [CS_LEVEL] [DC_PIN] [BL_PIN] [RESET_PIN] [DYSP_WIDTH] [DYSP_HEIGHT] [DYSP_W_OFS] [DYSP_H_OFS] [ROTATION]
void st7789_Init() {
	int arg_cnt = Tokenizer_GetArgsCount();
	int arg_i = 0;
	if(arg_cnt < 1) {
    	 ADDLOG_INFO(LOG_FEATURE_CMD, "\"startdriver ST7789\" needs at least one argument <SPI_NAME>, given ony %i" , Tokenizer_GetArgsCount() -1 );
    	 return;
    }
	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ST7789 driver init start...");
	/* TODO: load config from file st7789.json */
	/* spi & interface */
	tft.spidev.config.mode = OBK_SPI_MODE_3|OBK_SPI_MSB|OBK_SPI_3WIRE;
	tft.spidev.config.data_width = 8;
	tft.spidev.config.max_hz = 20000000L;	
	if (arg_i <= arg_cnt) arg_i++; //SPI NAME: hspi or cspi (normally not used) for esp8266, spi1, spi2 and etc. for other
	const char *spi_bus = Tokenizer_GetArg(arg_i);
	if (arg_i <= arg_cnt) arg_i++; //CS_PIN
	int nss_obk_pin = HAL_PIN_Find(Tokenizer_GetArg(arg_i));
	if (nss_obk_pin != -1)
		/* ! nss_pin must be actual GPIO pin, not obk pin index*/
		tft.spidev.config.nss_pin = HAL_GetGPIOPin(nss_obk_pin);
	else 
		tft.spidev.config.nss_pin = -1;
		
	/* TODO: nss_level */ 
	//if (Tokenizer_GetPin(3, 0)) tft.spidev.config.mode |= OBK_SPI_CS_HIGH;
	if (arg_i <= arg_cnt) arg_i++;
	tft.dc_pin = HAL_PIN_Find(Tokenizer_GetArg(arg_i));  /* Data\command line pin (obk pin index)*/
	if (arg_i <= arg_cnt) arg_i++;
	tft.rs_pin = HAL_PIN_Find(Tokenizer_GetArg(arg_i));  /* Reset pin (obk pin index)*/
	if (arg_i <= arg_cnt) arg_i++;
	if (arg_i > arg_cnt) tft.bl_pin = -1;
	else {
		//tft.bl_pin = HAL_PIN_Find(Tokenizer_GetArg(arg_i));  /* Backlight pin (obk pin index) or OBK_Channel*/	
		if (tft.bl_pin == -1) { // try parse it like channel
		
		}
	}
	/* display params */
	if (arg_i <= arg_cnt) arg_i++;
	if (arg_i > arg_cnt) tft.width    = 135;
	else tft.width   = Tokenizer_GetArgIntegerDefault(6, 320); /* display width */
	if (arg_i <= arg_cnt) arg_i++;
	if (arg_i > arg_cnt) tft.height   = 240;
	else tft.height  = Tokenizer_GetArgIntegerDefault(7, 240); /* display height */
	if (arg_i <= arg_cnt) arg_i++;
	if (arg_i > arg_cnt) tft.offsetx  = 52;
	else tft.offsetx = Tokenizer_GetArgIntegerDefault(8, 0); /* display width ofset */
	if (arg_i <= arg_cnt) arg_i++;
	if (arg_i > arg_cnt) tft.offsety  = 40;
	else tft.offsety = Tokenizer_GetArgIntegerDefault(9, 0); /* display height ofset */
	if (arg_i <= arg_cnt) arg_i++;
	if (arg_i > arg_cnt) tft.rotation = 0;
	else tft.rotation = Tokenizer_GetArgIntegerDefault(10, 0); /* display rotaion: 0, 90, 180 */
	
	tft.font_direction = 0;
	tft.font_scale = 2;
	
#if PLATFORM_ESP8266	
	tft.use_frame_buffer = false;
	tft.frame_buffer = NULL;
#else
	
#endif

	if (tft.dc_pin != -1) {
		HAL_PIN_Setup_Output(tft.dc_pin);
		HAL_PIN_SetOutputValue(tft.dc_pin, 0);
	}
	/*
	if (tft.bl_pin != -1) {
		HAL_PIN_Setup_Output(tft.bl_pin);
		HAL_PIN_SetOutputValue(tft.bl_pin, 0);
	}*/
	if (tft.rs_pin != -1) {
		HAL_PIN_Setup_Output(tft.rs_pin);
		HAL_PIN_SetOutputValue(tft.rs_pin, 1);
	}
		
	/* Attach to spi bus */	
	if (obk_spi_bus_attach_device(&tft.spidev, spi_bus, NULL) != OBK_EOK) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "ST7789 can't attach to bus [%s]" , spi_bus);
		return;
	}
	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ST7789 attached to bus [%s], CS=IO%d, DC=IO%d, RS=IO%d" , spi_bus, tft.spidev.config.nss_pin, HAL_GetGPIOPin(tft.dc_pin), HAL_GetGPIOPin(tft.rs_pin));
	
	/* Run display service thread */
	lcdInit(&tft);	
	lcdDrawFillRect(&tft, 0, 0, tft.width-1, tft.height-1, BLACK);	
	lcdDrawString(&tft, 18, 120, "OBK_ESP", BLUE, BLACK);
	lcdDisplayOn(&tft);
	lcdSetBrightness(&tft, 0);
	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ST7789 driver init done.");
}

void st7789_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState)
{
	/*
	if(bPreState)
		return;
	hprintf255(request, "<h2>SHTC3 Temperature=%.1fC, Humidity=%.0f%%</h2>", g_temp, g_humid);
	if (channel_humid == channel_temp) {
		hprintf255(request, "WARNING: You don't have configured target channels for temp and humid results, set the first and second channel index in Pins!");
	}
	*/
}

// stopDriver ST7789
void st7789_Stop() {
	
	
}

/**
 * @brief lcd_write_command 
 *
 * @param TFT_t * dev, uint8_t cmd
 *   
 * @return obk_err_t
 */
obk_err_t lcd_write_command(TFT_t * dev, uint8_t cmd)
{
	uint8_t xfer_buf = cmd;	
	
	HAL_PIN_SetOutputValue(dev->dc_pin, 0); /* command mode */	
	
	obk_spi_message_t message;
	message.send_buf = (uint8_t*)&xfer_buf;
	message.recv_buf = NULL;
	message.length   = 1;
	message.repeat   = 1;
	message.cs_take    = 0;
	message.cs_release = 0;
	message.next = NULL;
	
	dev->spidev.bus->ops->xfer(&dev->spidev, &message);
	
	return  OBK_EOK;
}

obk_err_t lcd_write_data_08(TFT_t * dev, uint8_t data)
{
	uint8_t xfer_buf = data;	
	
	HAL_PIN_SetOutputValue(dev->dc_pin, 1); /* data mode */
		
	obk_spi_message_t message;
	message.send_buf = (uint8_t*)&xfer_buf;
	message.recv_buf = NULL;
	message.length   = 1;
	message.repeat   = 1;
	message.cs_take    = 0;
	message.cs_release = 0;
	message.next = NULL;
	
	dev->spidev.bus->ops->xfer(&dev->spidev, &message);
	
	return  OBK_EOK;
}

obk_err_t lcd_write_data_16(TFT_t * dev, uint16_t data)
{
	uint16_t xfer_buf = data;	
	
	HAL_PIN_SetOutputValue(dev->dc_pin, 1); /* data mode */
	
	obk_spi_message_t message;
	message.send_buf = (uint8_t*)&xfer_buf;
	message.recv_buf = NULL;
	message.length   = 2;
	message.repeat   = 1;
	message.cs_take    = 0;
	message.cs_release = 0;
	message.next = NULL;
	
	dev->spidev.bus->ops->xfer(&dev->spidev, &message);
	
	return OBK_EOK;
}

obk_err_t lcd_write_data_32(TFT_t * dev, uint32_t data)
{
	uint32_t xfer_buf = data;	
	
	HAL_PIN_SetOutputValue(dev->dc_pin, 1); /* data mode */
	
	obk_spi_message_t message;
	message.send_buf = (uint8_t*)&xfer_buf;
	message.recv_buf = NULL;
	message.length   = 4;
	message.repeat   = 1;
	message.cs_take    = 0;
	message.cs_release = 0;
	message.next = NULL;
	
	dev->spidev.bus->ops->xfer(&dev->spidev, &message);
	
	return OBK_EOK;  
}

obk_err_t lcd_write_addr(TFT_t * dev, uint16_t addr1, uint16_t addr2)
{
	uint8_t xfer_buf[4];
	xfer_buf[0] = (addr1 >> 8) & 0xFF;
	xfer_buf[1] = addr1 & 0xFF;
	xfer_buf[2] = (addr2 >> 8) & 0xFF;
	xfer_buf[3] = addr2 & 0xFF;
	
	HAL_PIN_SetOutputValue(dev->dc_pin, 1); /* data mode */
	
	obk_spi_message_t message;
	message.send_buf = xfer_buf;
	message.recv_buf = NULL;
	message.length   = 4;
	message.repeat   = 1;
	message.cs_take    = 0;
	message.cs_release = 0;
	message.next = NULL;
	
	dev->spidev.bus->ops->xfer(&dev->spidev, &message);
	
	return OBK_EOK;
}

obk_err_t lcd_write_color(TFT_t * dev, uint16_t color, uint16_t size)
{
	uint8_t xfer_buf[2];
	xfer_buf[0] = (color >> 8) & 0xFF;
	xfer_buf[1] = color & 0xFF;
	
	HAL_PIN_SetOutputValue(dev->dc_pin, 1); /* data mode */
	
	obk_spi_message_t message;	
	message.send_buf = xfer_buf;
	message.recv_buf = NULL;
	message.length   = 2;
	message.repeat   = size;
	message.cs_take    = 0;
	message.cs_release = 0;
	message.next = NULL;

	dev->spidev.bus->ops->xfer(&dev->spidev, &message);
	
	return OBK_EOK;
}

obk_err_t lcd_write_colors(TFT_t * dev, uint16_t * colors, uint16_t size)
{
	HAL_PIN_SetOutputValue(dev->dc_pin, 1); /* data mode */
	
	obk_spi_message_t message;
	message.send_buf = colors;
	message.recv_buf = NULL;
	message.length   = size<<1;
	message.repeat   = 1;
	message.cs_take    = 0;
	message.cs_release = 0;
	message.next = NULL;
	
	dev->spidev.bus->ops->xfer(&dev->spidev, &message);
	
	return OBK_EOK;
}

/**
 * @brief lcdInit 
 *
 * @param TFT_t * dev: display device pointer
 *   
 * @return obk_err_t
 */
void lcdInit(TFT_t * dev)
{
	/* take the bus */
	obk_spi_take_bus(&dev->spidev);
	
	if (dev->rs_pin != -1) {
		HAL_PIN_SetOutputValue(dev->rs_pin, 1);
		obk_delay_ms(10);
		HAL_PIN_SetOutputValue(dev->rs_pin, 0);
		obk_delay_ms(50);
		HAL_PIN_SetOutputValue(dev->rs_pin, 1);
		obk_delay_ms(150);
	} 
		
	/* activate slave select signal */
	if (dev->spidev.config.nss_pin != -1)
		obk_spi_take_device(&dev->spidev);
	
	//lcd_write_command(dev, 0x01); //Software Reset
	//obk_delay_ms(150);	

	lcd_write_command(dev, 0x11); //Sleep Out
	obk_delay_ms(10);
		
	lcd_write_command(dev, 0x3A); //Interface Pixel Format
	lcd_write_data_08(dev, 0x55);
		
	lcd_write_command(dev, 0x36); //Memory Data Access Control
	lcd_write_data_08(dev, 0x00);
	
//	lcd_write_command(dev, 0x30);	//Partial sart/end address set
//	lcd_write_data_byte(dev, 0x00);
//	lcd_write_data_byte(dev, 0x00);
//	lcd_write_data_byte(dev, 0x00);
//	lcd_write_data_byte(dev, 0xA0-1);

	lcd_write_command(dev, 0x21);	//Display Inversion Of

	lcd_write_command(dev, 0x13);	//Normal Display Mode On
	//lcd_write_command(dev, 0x12);	//Partial Display Mode On

	lcd_write_command(dev, 0x29);	//Display ON
	
	/* deactivate slave select signal */
	if (dev->spidev.config.nss_pin != -1)
		obk_spi_release_device(&dev->spidev);	
	/* release the bus */
	obk_spi_release_bus(&dev->spidev);
}

/**
 * @brief lcdDrawPixel Draw pixel
 *
 * @param x:X coordinate
          y:Y coordinate
          color:color
 *   
 * @return void
 */
void lcdDrawPixel(TFT_t * dev, uint16_t x, uint16_t y, uint16_t color){
	if (x >= dev->width) return;
	if (y >= dev->height) return;

	if (dev->use_frame_buffer) {
		dev->frame_buffer[y*dev->width+x] = color;
	} else {
		uint16_t _x = x + dev->offsetx;
		uint16_t _y = y + dev->offsety;

		lcd_write_command(dev, 0x2A);	// set column(x) address
		lcd_write_addr(dev, _x, _x);
		lcd_write_command(dev, 0x2B);	// set Page(y) address
		lcd_write_addr(dev, _y, _y);
		lcd_write_command(dev, 0x2C);	// Memory Write
		//spi_master_write_data_word(dev, color);
		lcd_write_color(dev, color, 1);
	}
}

/**
 * @brief lcdDrawFillRect 
 *
 * @param x1:Start X coordinate
          y1:Start Y coordinate
          x2:End X coordinate
          y2:End Y coordinate
          color:color
 *   
 * @return void
 */
void lcdDrawFillRect(TFT_t * dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	if (x1 >= dev->width) return;
	if (x2 >= dev->width) x2=dev->width-1;
	if (y1 >= dev->height) return;
	if (y2 >= dev->height) y2=dev->height-1;

	//ESP_LOGD(TAG,"offset(x)=%d offset(y)=%d",dev->_offsetx,dev->_offsety);
/*
	if (dev->use_frame_buffer) {
		for (int16_t j = y1; j <= y2; j++){
			for(int16_t i = x1; i <= x2; i++){
				dev->frame_buffer[j*dev->width+i] = color;
			}
		}
	} else*/ {
		uint16_t _x1 = x1 + dev->offsetx;
		uint16_t _x2 = x2 + dev->offsetx;
		uint16_t _y1 = y1 + dev->offsety;
		uint16_t _y2 = y2 + dev->offsety;
		
		/* take the bus */
		obk_spi_take_bus(&dev->spidev);
		
		lcd_write_command(dev, 0x2A);	// set column(x) address
		lcd_write_addr(dev, _x1, _x2);
		lcd_write_command(dev, 0x2B);	// set Page(y) address
		lcd_write_addr(dev, _y1, _y2);
		lcd_write_command(dev, 0x2C);	// Memory Write
		for(int i=_x1;i<=_x2;i++){
			uint16_t size = _y2-_y1+1;
			lcd_write_color(dev, color, size);
		}
		
		/* release the bus */
		obk_spi_release_bus(&dev->spidev);
	}
}

/**
 * @brief lcdDrawBitmap  
 *
 * @param x:Start X coordinate
		  y:Start Y coordinate
          w:bitmap width
          h:bitmap height
          bitmap:Pointer to bitmap array
          rcolor:color in bitmap to replace
          bkcolor: new color
 *   
 * @return void
 */
void lcdDrawBitmap(TFT_t * dev, uint16_t x,uint16_t y,uint16_t w,uint16_t h, uint16_t *bitmap, uint16_t rcolor, uint16_t bkcolor) {
	uint16_t *bitmap_buf = 0;
	uint16_t _rcolor = SWAPBYTE(rcolor);
	uint16_t _bkcolor = SWAPBYTE(bkcolor); 
	if (_rcolor != _bkcolor) {
		bitmap_buf = (uint16_t *)os_malloc(sizeof(uint16_t)*w*h);
		if (bitmap_buf != 0) {
			int16_t px = w*h-1;
			for ( ; px >= 0; px-- ) {
				if (bitmap[px] == _rcolor) bitmap_buf[px] = _bkcolor;
				else                      bitmap_buf[px] = bitmap[px];
			}
			bitmap = bitmap_buf;
		}
	}
	lcd_write_command(dev, 0x2A);	// set column(x) address
	lcd_write_addr(dev, x, x+w-1);
	lcd_write_command(dev, 0x2B);	// set Page(y) address
	lcd_write_addr(dev, y, y+h-1);
	lcd_write_command(dev, 0x2C);	// Memory Write
	lcd_write_colors(dev, bitmap, w*h);
	if (bitmap_buf != 0)
		os_free(bitmap_buf);
}

/**
 * @brief lcdDrawChar Draw ASCII character
 *
 * @param x:X coordinate
          y:Y coordinate
          ascii: ascii code
          charColor: char color
          bkgColor: bkackground color
 *   
 * @return void
 */
int lcdDrawChar(TFT_t * dev, uint16_t x, uint16_t y, uint8_t ascii, uint16_t charColor, uint16_t bkgColor) {
	uint16_t i_x, j_y, sc_x, sc_y, pos_x, pos_y;
	uint16_t _charColor = SWAPBYTE(charColor);
	uint16_t _bkgColor  = SWAPBYTE(bkgColor);
	
	uint16_t *char_buf = (uint16_t *)os_malloc(sizeof(uint16_t)*13*dev->font_scale*7*dev->font_scale);
	
	for (i_x=0;i_x<7;i_x++)	{
		for (sc_x=0;sc_x<dev->font_scale;sc_x++) {
			uint16_t ch = (NewBFontLAT[ ( (ascii-0x20)*14 + i_x+7) ] <<8) + NewBFontLAT[ ( (ascii-0x20)*14 + i_x) ];
			ch = ch<<1;
			for (j_y=0;j_y<13;j_y++) {
				uint16_t _color;
				if (ch & 0x8000) _color=_charColor; else _color=_bkgColor;
				ch = ch<<1;
				for (sc_y=0;sc_y<dev->font_scale;sc_y++)
				{
					if (char_buf == 0)	{
						//pos_x = x + i_x*(dev->font_scale)+sc_x;
						//pos_y = y + (12-j_y)*(dev->font_scale)+sc_y;
						//lcdDrawPixel(dev, pos_x, pos_y, _color);
					} else {
						pos_x = i_x*(dev->font_scale)+sc_x;
						pos_y = (12-j_y)*(dev->font_scale)+sc_y;
						*(uint16_t *)(char_buf+(pos_y*7*(dev->font_scale)+pos_x)) = _color;
						//*(uint16_t *)(char_buf+(pos_x*13*(dev->font_scale)+pos_y)) = _color;
					}
				}
			}
		}
	}
	if (char_buf != 0) {
		uint16_t _x = x + dev->offsetx;
	    uint16_t _y = y + dev->offsety;
		lcd_write_command(dev, 0x2A);	// set column(x) address
		lcd_write_addr(dev, _x, _x+7*dev->font_scale-1);
		lcd_write_command(dev, 0x2B);	// set Page(y) address		
		lcd_write_addr(dev, _y, _y+13*dev->font_scale-1);
		lcd_write_command(dev, 0x2C);	// Memory Write
		lcd_write_colors(dev, char_buf, dev->font_scale*13*dev->font_scale*7);
	}
	if (char_buf != 0) os_free(char_buf);
	return 0;
}
/**
 * @brief lcdDrawString Draw ASCII string
 *
 * @param x:X coordinate
          y:Y coordinate
          *ascii: pointer to ascii string
          charColor: char color
          bkgColor: bkackground color
 *   
 * @return int
 */
int lcdDrawString(TFT_t * dev, uint16_t x, uint16_t y, uint8_t * ascii, uint16_t charColor, uint16_t bkgColor) {
	int length = strlen((char *)ascii);
	for(int i=0;i<length;i++) {
		if (dev->font_direction == 0) {
			lcdDrawChar(dev, x, y, ascii[i], charColor, bkgColor);
			x=x+(7*dev->font_scale)-1;
		}
		if (dev->font_direction == 1) {
			lcdDrawChar(dev, x, y, ascii[i], charColor, bkgColor);
			y=y+(14*dev->font_scale)-1;
		}
		if (dev->font_direction == 2) {
			lcdDrawChar(dev, x, y, ascii[i], charColor, bkgColor);
			x=x-(7*dev->font_scale)-1;
		}
		if (dev->font_direction == 3) {
			y = lcdDrawChar(dev, x, y, ascii[i], charColor, bkgColor);
			y=y-(14*dev->font_scale)-1;
		}
	}
	if (dev->font_direction == 0) return x;
	if (dev->font_direction == 2) return x;
	if (dev->font_direction == 1) return y;
	if (dev->font_direction == 3) return y;
	return 0;
}
/**
 * @brief Display OFF
 *
 * @param dev: pointer to dispay structure
 *   
 * @return 
 */
void lcdDisplayOff(TFT_t * dev) {
	lcd_write_command(dev, 0x28);	// Display off
}
/**
 * @brief Display ON
 *
 * @param dev: pointer to dispay structure
 *   
 * @return 
 */
void lcdDisplayOn(TFT_t * dev) {
	lcd_write_command(dev, 0x29);	// Display on
}

/**
 * @brief lcdSetBrightness Write Display Brightness
 *
 * @param dev: pointer to dispay structure
 *        brightness: brightness         
 *   
 * @return 
 */
void lcdSetBrightness(TFT_t * dev, uint8_t brightness) {
	lcd_write_command(dev, 0x51);	// Write Display Brightness
	lcd_write_data_08(dev, brightness);
}