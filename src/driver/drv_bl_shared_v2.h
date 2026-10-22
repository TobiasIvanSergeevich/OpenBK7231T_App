#pragma once

#include "../httpserver/new_http.h"
#include "../obk_config.h"

#if ENABLE_BLv2_SHARED

typedef enum energySensorMode_e {
	es_1PhaseMode = 0,
	es_3PhaseMode,
} energySensorMode_t;
/* Mesuared by meter sensors*/
#define msVoltage  				(0x1 << 0)
#define msCurrent  				(0x1 << 1)
#define msPowerActiveSumm		(0x1 << 2)
#define msPowerActivePhase 		(0x1 << 3)
#define msPowerReactiveSumm		(0x1 << 4)
#define msPowerReactivePhase	(0x1 << 5)
#define msPowerApparentSumm		(0x1 << 6)
#define msPowerApparentPhase	(0x1 << 7)
#define msPowerFactorSumm  		(0x1 << 8)
#define msPowerFactorPhase 		(0x1 << 9)
#define msFrequency    			(0x1 << 10)
#define msConsumptionTotalAp	(0x1 << 11)
// for output to grid (export)
#define msConsumptionTotalAm	(0x1 << 12)


void BLv2_Shared_Init(int meter_count, energySensorMode_t mode, int MeasuredSensors);
void BLv2_ProcessUpdateEnergy(int meter, float energyWAp, float energyWRp, float energyWAm, float energyWRm, int line);
void BLv2_ProcessUpdateMesuaments(int meter, float voltage, float current, float power_active, float power_reactive, float power_apparent, float power_factor, float frequency, int line);
//void BLv2_ProcessUpdate(int meter, float voltage, float current, float power, float frequency, float energyWh);
void BLv2_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);
void BLv2_SaveEmeteringStatistics();

/* for compatable */
static inline void BL09XX_SaveEmeteringStatistics(void) {
	BLv2_SaveEmeteringStatistics();
}
static inline void BL09XX_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {
 	BLv2_AppendInformationToHTTPIndexPage(request, bPreState);
}

#define BL_SENSORS_IX_0 0
#define BL_SENSORS_IX_1 1

#endif

