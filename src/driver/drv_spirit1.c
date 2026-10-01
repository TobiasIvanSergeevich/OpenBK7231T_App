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
#include "drv_spirit1.h"

static obk_spidevice_t spidev;

// startDriver SPIRIT hspi IO2 
// startDriver SPIRIT [SPI_NAME] [CS_PIN] 
void spirit_Init() {
	int arg_cnt = Tokenizer_GetArgsCount();
	int arg_i = 0;
	if(arg_cnt < 2) {
    	ADDLOG_INFO(LOG_FEATURE_CMD, "\"startdriver SPIRIT1\" needs at least two arguments <SPI_NAME> and <CS_PIN>, given ony %i" , Tokenizer_GetArgsCount() -1 );
    	return;
    }
	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "SPIRIT driver init start...");	
	/* spi & interface */
	spidev.config.mode = OBK_SPI_MODE_3|OBK_SPI_MSB|OBK_SPI_3WIRE;
	spidev.config.data_width = 8;
	spidev.config.max_hz = 20000000L;	
	if (arg_i <= arg_cnt) arg_i++; //SPI NAME: hspi or cspi (normally not used) for esp8266, spi1, spi2 and etc. for other
	const char *spi_bus = Tokenizer_GetArg(arg_i);
	if (arg_i <= arg_cnt) arg_i++; //CS_PIN
	int nss_obk_pin = HAL_PIN_Find(Tokenizer_GetArg(arg_i));
	if (nss_obk_pin != -1)
		/* ! nss_pin must be actual GPIO pin, not obk pin index*/
		spidev.config.nss_pin = HAL_GetGPIOPin(nss_obk_pin);
	else { 			
		spidev.config.nss_pin = -1;
		ADDLOG_ERROR(LOG_FEATURE_CMD, "\"driver SPIRIT1\" can't work without <CS_PIN>");
		return;
	}	
	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "SPIRIT1 driver init done.");
}

void spirit_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState)
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

// stopDriver SPIRIT1
void spirit_Stop() {
	
	
}

/**
 * @brief _write_command 
 *
 * @param TFT_t * dev, uint8_t cmd
 *   
 * @return obk_err_t
 */
obk_err_t _write_command(TFT_t * dev, uint8_t cmd)
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