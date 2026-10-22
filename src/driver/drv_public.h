#ifndef __DRV_PUBLIC_H__
#define __DRV_PUBLIC_H__

#include "../httpserver/new_http.h"

typedef enum energySensor_e {
	OBK__FIRST = 0,
	OBK_VOLTAGE = OBK__FIRST, // must match order in cmd_public.h

#ifdef ENABLE_BLv2_SHARED
	OBK_VOLTAGE_L1 = OBK_VOLTAGE,
	OBK_VOLTAGE_L2,
	OBK_VOLTAGE_L3,
#endif

	OBK_CURRENT,

#ifdef ENABLE_BLv2_SHARED
	OBK_CURRENT_L1 = OBK_CURRENT,
	OBK_CURRENT_L2,
	OBK_CURRENT_L3,
#endif

#ifdef ENABLE_BLv2_SHARED
	OBK_POWER_L1,
	OBK_POWER_L2,
	OBK_POWER_L3,
#endif

	OBK_POWER,
	OBK_FREQUENCY,
	OBK_POWER_APPARENT,
	OBK_POWER_REACTIVE,
	OBK_POWER_FACTOR,
	OBK_CONSUMPTION_TOTAL,
#ifdef ENABLE_BLv2_SHARED
	OBK_CONSUMPTION_WAImp_TOTAL = OBK_CONSUMPTION_TOTAL,
	OBK_CONSUMPTION_WAExp_TOTAL,
	//OBK_CONSUMPTION_WR_TOTAL,
#ifdef ENABLED_TARIFED_ENERGY
#if TARIFED_TIMEZONE_MAX == 2
	OBK_CONSUMPTION_WAImp_T1, // Timezone Tarif 1
	OBK_CONSUMPTION_WAImp_T2, // Timezone Tarif 2
#endif
#if TARIFED_TIMEZONE_MAX == 3
	OBK_CONSUMPTION_WAImp_T3, // Timezone Tarif 3
#endif
#if TARIFED_TIMEZONE_MAX == 4
	OBK_CONSUMPTION_WAImp_T4, // Timezone Tarif 4
#endif
#if TARIFED_TIMEZONE_MAX == 2
	OBK_CONSUMPTION_WAExp_T1, // Timezone Tarif 1
	OBK_CONSUMPTION_WAExp_T2, // Timezone Tarif 2
#endif
#if TARIFED_TIMEZONE_MAX == 3
	OBK_CONSUMPTION_WAExp_T3, // Timezone Tarif 3
#endif
#if TARIFED_TIMEZONE_MAX == 4
	OBK_CONSUMPTION_WAExp_T4, // Timezone Tarif 4
#endif
#endif //ENABLED_TARIFED_ENERGY

#endif //ENABLE_BLv2_SHARED	
	OBK__NULL,
	OBK__NUM_MEASUREMENTS = OBK__NULL-1,
	// TODO OBK_CONSUMPTION_LAST_HOUR is actally "sum of consumption stats recording period"
	// and won't correspond to 'last hour' unless cmd SetupEnergyStats is enabled and configured to record one hour
	// e.g. 'SetupEnergyStats 1 60 60 0': 60 sec intervals, 60 samples
	OBK_CONSUMPTION_LAST_HOUR,	
	//OBK_CONSUMPTION_STATS, // represents a variable size array of energy samples, not a sensor
	// below here are sensors that are assumed to require NTP driver
	OBK_CONSUMPTION__DAILY_FIRST, //daily consumptions are assumed to be in chronological order
	OBK_CONSUMPTION_TODAY = OBK_CONSUMPTION__DAILY_FIRST,
	OBK_CONSUMPTION_YESTERDAY,
	OBK_CONSUMPTION_2_DAYS_AGO,
	OBK_CONSUMPTION_3_DAYS_AGO,
	OBK_CONSUMPTION__DAILY_LAST = OBK_CONSUMPTION_3_DAYS_AGO,

	OBK_CONSUMPTION_CLEAR_DATE,
	OBK__LAST = OBK_CONSUMPTION_CLEAR_DATE,
	OBK__NUM_SENSORS,
} energySensor_t;

#if ENABLE_BL_TWIN
extern const int OBK_CONSUMPTION_STORED_LAST[2];
#else
extern const int OBK_CONSUMPTION_STORED_LAST[1];
#endif

typedef struct energySensorNames_s {
	const char* const hass_dev_class;
	const char* const units;
	const char* const name_friendly;
	const char* const name_mqtt;
	const char* const hass_uniq_id_suffix; //keep identifiers persistent in case OBK_ENERG_SENSOR changes
} energySensorNames_t;

extern int g_dhtsCount;

void DRV_Generic_Init();
void DRV_OnHassDiscovery(const char *topic);
void DRV_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState);
void DRV_OnEverySecond();
void DHT_OnEverySecond();
void DHT_OnPinsConfigChanged();
void DRV_RunQuickTick();
void DRV_StartDriver(const char* name);
void DRV_StopDriver(const char* name);
// right now only used by simulator
void DRV_ShutdownAllDrivers();
bool DRV_IsRunning(const char* name);
void DRV_SavePowerMeterDriverStatistics(void);
void DRV_OnChannelChanged(int channel, int iVal);
#if PLATFORM_BK7231N
void Strip_setMultiplePixel(uint32_t pixel, uint8_t *data, bool push);
#endif
void DRV_GosundSW2_Write(float* rgbcw);
void SM2135_Write(float* rgbcw);
void BP5758D_Write(float* rgbcw);
void BP1658CJ_Write(float* rgbcw);
void SM2235_Write(float* rgbcw);
void KP18058_Write(float *rgbcw);
void DRV_DGR_OnLedDimmerChange(int iVal);
void DRV_DGR_OnLedEnableAllChange(int iVal);
void DRV_DGR_OnLedFinalColorsChange(byte rgbcw[5]);

// OBK_POWER etc
#ifdef ENABLE_BLv2_SHARED

float DRV_GetReading_v2(int asensdatasetix, energySensor_t type);
#define DRV_GetReading(type) DRV_GetReading_v2(0, type)

int BLv2_HasEnergySensorReading(int asensdatasetix, energySensor_t type);
#define BL_HasEnergySensorReadingEx(asensdatasetix, type) BLv2_HasEnergySensorReading(asensdatasetix, type)
#define BL_HasEnergySensorReading(type) BLv2_HasEnergySensorReading(0, type)

energySensorNames_t* DRV_GetEnergySensorNames_v2(int asensdatasetix, energySensor_t type);
#define DRV_GetEnergySensorNames(type) DRV_GetEnergySensorNames_v2(0, type)
#define DRV_GetEnergySensorNamesEx(asensdatasetix, type) DRV_GetEnergySensorNames_v2(asensdatasetix, type)

#else

float DRV_GetReading(energySensor_t type);
int BL_HasEnergySensorReadingEx(int asensdatasetix, energySensor_t type);
int BL_HasEnergySensorReading(energySensor_t type);
energySensorNames_t* DRV_GetEnergySensorNames(energySensor_t type);
energySensorNames_t* DRV_GetEnergySensorNamesEx(int asensdatasetix, energySensor_t type);

#endif

bool DRV_IsMeasuringPower();
bool DRV_IsMeasuringBattery();
bool DRV_IsSensor();

// TuyaMCU exports for LED
void TuyaMCU_OnRGBCWChange(const float *rgbcw, int bLightEnableAll, int iLightMode, float brightnessRange01, float temperatureRange01);
bool TuyaMCU_IsLEDRunning();

void Shutter_MoveByIndex(int index, float frac, bool bStopOnDuplicate);

#endif /* __DRV_PUBLIC_H__ */

