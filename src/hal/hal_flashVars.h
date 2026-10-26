#ifndef __HALK_FLASH_VARS_H__
#define __HALK_FLASH_VARS_H__

//#define DISABLE_FLASH_VARS_VARS
#include "../new_common.h"

#define BOOT_COMPLETE_SECONDS 30
#define MAX_RETAIN_CHANNELS 12

/* Fixed size 32 bytes */
typedef struct ENERGY_METERING_DATA {
	float TotalConsumption;
	float TodayConsumpion;
	float YesterdayConsumption;
	long save_counter;
#if ENABLE_BL_TWIN
	float TotalConsumption_b;
	float TodayConsumpion_b;
#else
	float ConsumptionHistory[2];
#endif
	time_t ConsumptionResetTime;
	unsigned char reseved[3];
	char actual_mday;
} ENERGY_METERING_DATA;

// size 8 bytes
typedef struct {
	float Import;
	float Export;
} ENERGY_DATA;

typedef enum {
	ENERGY_CHANNEL_A = 0,
	ENERGY_CHANNEL_B = 1,
} ENERGY_CHANNEL;

#if defined(ENABLE_BLv2_SHARED)

#if ENABLED_TARIFED_ENERGY
	#ifndef TARIFED_TIMEZONE_MAX
		#error "Timezone count for tarifed energy (TARIFED_TIMEZONE_MAX) is not defined. Refer to obk_config.h"
	#endif
	#if (TARIFED_TIMEZONE_MAX > 4) 
		#error "Timezone count must not exceed 4. TARIFED_TIMEZONE_MAX must be corrected in obk_config.h"
	#endif
	#if (TARIFED_TIMEZONE_MAX < 2)
		#error "Timezone count (TARIFED_TIMEZONE_MAX) can not be less 2. Refer to obk_config.h"
	#endif
#endif

typedef struct {	
	float 		WA_Import_Total;
	float 		WA_Export_Total;
#if ENABLED_TARIFED_ENERGY
	float 		WA_Import[TARIFED_TIMEZONE_MAX];
#if (TARIFED_TIMEZONE_MAX < 4)
	float		res_i[4-TARIFED_TIMEZONE_MAX];
#endif	
	float 		WA_Export[TARIFED_TIMEZONE_MAX];
#if (TARIFED_TIMEZONE_MAX < 4)
	float		res_e[4-TARIFED_TIMEZONE_MAX];	
#endif	
#else 
	float		res_i[4];	
	float		res_e[4];
#endif	
} ENERGY_DATA_V2;

typedef struct {
	long 				save_counter;
#if ENABLED_TARIFED_ENERGY		
	time_t 				timezones[TARIFED_TIMEZONE_MAX];
#if (TARIFED_TIMEZONE_MAX < 4)
	time_t 				rez_tz[4-TARIFED_TIMEZONE_MAX];
#endif	
#else 
	time_t				res_tz[4];
#endif
	ENERGY_DATA_V2      energy_total;
#if ENABLED_MONTH_ENERGY	
	char 				actual_month;
	ENERGY_DATA_V2 		month_energy[12];
#else
	char 				res0;
	float				res1[12*(sizeof(ENERGY_DATA_V2)/sizeof(float))];
#endif	
} ENERGY_METERING_DATA_V2;

#endif

typedef struct flash_vars_structure
{
	// offset  0
	unsigned short boot_count; // number of times the device has booted
	unsigned short boot_success_count; // if a device boots completely (>30s), will equal boot_success_count
	// offset  4
	short savedValues[MAX_RETAIN_CHANNELS];
	// offset 28
	ENERGY_METERING_DATA emetering;
	// offset 60
	unsigned char rgb[3];
	unsigned char len; // length of the whole structure (i.e. 2+2+1 = 5)  MUST NOT BE 255
	// size   64
} FLASH_VARS_STRUCTURE;

#define MAGIC_FLASHVARS_SIZE 64

// call at startup
void HAL_FlashVars_IncreaseBootCount();
// call once started (>30s?)
void HAL_FlashVars_SaveBootComplete();
// call to return the number of boots since a HAL_FlashVars_SaveBootComplete
int HAL_FlashVars_GetBootFailures();
int HAL_FlashVars_GetBootCount();
void HAL_FlashVars_SaveChannel(int index, int value);
void HAL_FlashVars_SaveLED(byte mode, short brightness, short temperature, byte r, byte g, byte b, byte bEnableAll);
void HAL_FlashVars_ReadLED(byte* mode, short* brightness, short* temperature, byte* rgb, byte* bEnableAll);
int HAL_FlashVars_GetChannelValue(int ch);
int HAL_GetEnergyMeterStatus(ENERGY_METERING_DATA* data);
int HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA* data);
void HAL_FlashVars_SaveTotalConsumption(float total_consumption);
void HAL_FlashVars_SaveEnergyExport(float f);
float HAL_FlashVars_GetEnergyExport();

#if defined(ENABLE_DRIVER_HLW8112SPI) || defined(ENABLE_DRIVER_BL0939SPI)
void HAL_FlashVars_SaveEnergy(ENERGY_DATA** data, int channel_count);
void HAL_FlashVars_GetEnergy(ENERGY_DATA* data, ENERGY_CHANNEL channel);
#endif

#if defined(ENABLE_BLv2_SHARED) 
void HAL_FlashVars_SaveEnergy_BLv2(ENERGY_METERING_DATA_V2* data, int meter);
void HAL_FlashVars_GetEnergy_BLv2(ENERGY_METERING_DATA_V2* data, int meter);
#endif

#endif /* __HALK_FLASH_VARS_H__ */

