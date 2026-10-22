#include "../obk_config.h"

#if ENABLE_DRIVER_ATMS100133

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include "drv_public.h"
#include "drv_bl_shared_v2.h"
#include <math.h>
#include "..\cJSON\cJSON.h"

#include "drv_tuyaMCU_atms100133.h"
#include "drv_tuyaMCU.h"

#define atms_dp_VCP_L1  			6
#define atms_dp_VCP_L2  			7
#define atms_dp_VCP_L3  			8
#define atms_dp_PowerFactor  		50
#define atms_dp_PowerActive3Phase  	29
#define atms_dp_PowerReactive3Phase 30
#define atms_dp_EnergyWAImport		1
#define atms_dp_EnergyWAExport 		23
#define atms_dp_EnergyWA			24

typedef struct {
	uint8_t 	voltage[2];
	uint8_t 	current[3];
	uint8_t 	power[3];	
} dp_VCP_Lx_t;

enum atms_StateMachine_e {
	sm_Error = -1,
	sm_CheckTuayMCUDriver = 0,
	sm_StartTuayMCUDriver,
	sm_CheckProductInformation,
	sm_QueryProductInformation,	
	sm_NormalMeasure
};

static int drv_atms_init = 0;
static int drv_atms_state   = 0;
static int drv_atms_startTuyaMCU_attempts   = 0;
static portTickType drv_atms_delay;
static portTickType drv_atms_timestamp;
static uint32_t prev_energyWA = -1;
//static uint32_t prev_energyWAImport = -1;
//static uint32_t prev_energyWAExport = -1;

static void ATMS_dpParser(int dpId, int dpType, int dpSize, byte *payload) {
	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ATMS_dpParser dpId=%d, dpType=%d dpSize=%d", dpId, dpType, dpSize);
	switch (dpId) {
		case atms_dp_VCP_L1:
		case atms_dp_VCP_L2:
		case atms_dp_VCP_L3:
		{
			/* 	Info:TuyaMCU:Received: 55 AA 03 07 00 0C 08 00 00 08 04 4B 00 01 F6 00 00 30 9B
				Info:TuyaMCU:ProcessIncoming[v=3]: cmd 7 (State) len 19
				Info:TuyaMCU:ParseState: id 8 type 0-raw len 8
			*/
			dp_VCP_Lx_t *dp_VCP = (dp_VCP_Lx_t *)payload;
			uint16_t voltage = dp_VCP->voltage[0]<<8 | dp_VCP->voltage[1];
			uint32_t current = dp_VCP->current[0]<<16 | dp_VCP->current[1]<<8 | dp_VCP->current[2];
			uint32_t power   = dp_VCP->power[  0]<<16 | dp_VCP->power[  1]<<8 | dp_VCP->power[  2];
			BLv2_ProcessUpdateMesuaments(0, (float)voltage/10.0, (float)current/1000.0, (float)power, NAN, NAN, NAN, NAN, dpId-atms_dp_VCP_L1+1);
			break;
		}
		case atms_dp_PowerFactor: {
			/* 	Info:TuyaMCU:Received: 55 AA 03 07 00 08 32 02 00 04 00 00 00 64 AD
				Info:TuyaMCU:ProcessIncoming[v=3]: cmd 7 (State) len 15
				Info:TuyaMCU:ParseState: id 50 type 2-val len 4
				Info:TuyaMCU:ParseState: int32 100
			*/
			//BLv2_ProcessUpdateMesuaments(int meter, float voltage, float current, float power_active, float power_reactive, float power_apparent, float power_factor, float frequency, int line);
			uint32_t power_factor_i32 = payload[0] << 24 | payload[1] << 16 | payload[2] << 8 | payload[3];
			BLv2_ProcessUpdateMesuaments(0, NAN, NAN, NAN, NAN, NAN, (float)power_factor_i32/100.0, NAN, 0);
			break;
		}
		case atms_dp_PowerActive3Phase: {
			//BLv2_ProcessUpdateMesuaments(int meter, float voltage, float current, float power_active, float power_reactive, float power_apparent, float power_factor, float frequency, int line);
			uint32_t powerA_i32 = payload[0] << 24 | payload[1] << 16 | payload[2] << 8 | payload[3];
			BLv2_ProcessUpdateMesuaments(0, NAN, NAN, (float)powerA_i32, NAN, NAN, NAN, NAN, 0);
			break;
		}
		case atms_dp_PowerReactive3Phase: {
			//BLv2_ProcessUpdateMesuaments(int meter, float voltage, float current, float power_active, float power_reactive, float power_apparent, float power_factor, float frequency, int line);
			uint32_t powerR_i32 = payload[0] << 24 | payload[1] << 16 | payload[2] << 8 | payload[3];
			BLv2_ProcessUpdateMesuaments(0, NAN, NAN, NAN, (float)powerR_i32, NAN, NAN, NAN, 0);
			break;
		}		
		case atms_dp_EnergyWA: {
			// TODO: if MCU will have restarted and start count from zero?...
			float energyWAp = 0;
			uint32_t energy_u32 = payload[0] << 24 | payload[1] << 16 | payload[2] << 8 | payload[3];
			if (prev_energyWA != (uint32_t)-1) {
				int diff = (energy_u32 < prev_energyWA
					? energy_u32 + (0xFFFFFF - prev_energyWA) + 1
					: energy_u32 - prev_energyWA);
				energyWAp = (float)diff;
			}
			prev_energyWA = energy_u32;
			//BLv2_ProcessUpdateEnergy(int meter, float energyWAp, float energyWRp, float energyWAm, float energyWRm, int line);
			BLv2_ProcessUpdateEnergy(0, energyWAp, NAN, NAN, NAN, 0);
			break;
		}
		case atms_dp_EnergyWAImport: {
			break;
		}
		case atms_dp_EnergyWAExport: {
			break;
		}			
	}
}
// StopDriver ATMS
void ATMS_StopDriver() {
	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ATMS: Stopping Driver");
	TuyaMCU_Set_dpParser(NULL);
	drv_atms_init = 0;
}

// startDriver ATMS
void ATMS_Init() {

	BLv2_Shared_Init(1, es_3PhaseMode, msVoltage|msCurrent|msPowerActivePhase|msPowerActiveSumm|msPowerReactiveSumm|msPowerFactorSumm|msConsumptionTotalAp);

    //PwrCal_Init(PWR_CAL_DIVIDE, DEFAULT_VOLTAGE_CAL, DEFAULT_CURRENT_CAL,
    //           DEFAULT_POWER_CAL);
	
	drv_atms_state = sm_CheckTuayMCUDriver;
	drv_atms_timestamp = xTaskGetTickCount();
	drv_atms_delay     = 10/portTICK_PERIOD_MS;
	drv_atms_init = 1;
}

void ATMS_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {
	BLv2_AppendInformationToHTTPIndexPage(request, bPreState);
}

void ATMS_RunEverySecond()
{

}

void ATMS_runQuickTick()
{
	portTickType drv_atms_now = xTaskGetTickCount();
	if (drv_atms_now >= drv_atms_timestamp) {	 
		if ((drv_atms_now - drv_atms_timestamp) < drv_atms_delay) return;
	} else {
		drv_atms_timestamp = drv_atms_now;
		return;
	}
	drv_atms_timestamp = drv_atms_now;
	drv_atms_delay = 10/portTICK_PERIOD_MS;
	switch (drv_atms_state) {
		case sm_CheckTuayMCUDriver: {
			addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ATMS: Checking TuyaMCU if driver is running.");
			// check if "TuyaMCU" driver started
			if (DRV_IsRunning("TuyaMCU") == false) {
				if (drv_atms_startTuyaMCU_attempts == 10) {
					addLogAdv(LOG_ERROR, LOG_FEATURE_DRV, "ATMS: Driver TuyaMCU required, but it can't be launched. Check your firmware." );
					drv_atms_state = sm_Error;
					return;
				}
				drv_atms_state = sm_StartTuayMCUDriver;
				return;
			}
			addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ATMS: Driver TuyaMCU OK, settting up TuyaMCU.");
			// setup TuyaMCU baudrate
			CMD_ExecuteCommand("tuyaMcu_setBaudRate 115200", 0);
			drv_atms_state = sm_CheckProductInformation;
			break;
		};
		case sm_StartTuayMCUDriver: {
			// start it
			addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ATMS: Try to start driver TuyaMCU.");
			//DRV_StartDriver("TuyaMCU");
			CMD_ExecuteCommand("startDriver TuyaMCU", 0);
			drv_atms_state = sm_CheckTuayMCUDriver;			
			drv_atms_startTuyaMCU_attempts++;
			drv_atms_delay = 500/portTICK_PERIOD_MS;
			return;
			break;		
		}
		case sm_CheckProductInformation: {	
			addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ATMS: Checking product information.");		
			cJSON *info_json = TuyaMCU_GetProductInformation();
			if (info_json) {
				cJSON *pid_json, *version_json, *mode_json;
				pid_json     = cJSON_GetObjectItem(info_json, "p");
				version_json = cJSON_GetObjectItem(info_json, "v");
				mode_json    = cJSON_GetObjectItem(info_json, "m");
				if (!pid_json || !version_json || !mode_json) {
					addLogAdv(LOG_ERROR, LOG_FEATURE_DRV, "ATMS: Wrong product information string." );
					drv_atms_state = sm_Error;
					return;
				}				
				char *pid_str = cJSON_GetStringValue(pid_json);
				if (!pid_str) {
					addLogAdv(LOG_ERROR, LOG_FEATURE_DRV, "ATMS: Wrong product information string." );
					drv_atms_state = sm_Error;
					return;					
				}
				if (strcmp(pid_str, "ab1sqlmvkyfkll6v")) {
					addLogAdv(LOG_ERROR, LOG_FEATURE_DRV, "ATMS: Wrong PID from TuyaMCU \"%s\" != \"ab1sqlmvkyfkll6v\".",  pid_str);
					drv_atms_state = sm_Error;
					return;
				}
				char *ver_str = cJSON_GetStringValue(version_json);
				if (!ver_str || strcmp(ver_str, "1.0.1")) {
					addLogAdv(LOG_WARN, LOG_FEATURE_DRV, "ATMS: Software version is not 1.0.1.");
				}
			} else {
				addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ATMS: Product information not ready.");
				drv_atms_state = sm_QueryProductInformation;
				return;
			}
			// setup TuyaMCU dpID parser
			TuyaMCU_Set_dpParser(ATMS_dpParser);
			drv_atms_delay = 10000/portTICK_PERIOD_MS;						
			drv_atms_state = sm_NormalMeasure;			
			addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ATMS: Ready to measure.");
			return; 
			break;
		}
		case sm_QueryProductInformation: {
			addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ATMS: Try to get MCU product information.");
			CMD_ExecuteCommand("tuyaMcu_sendProductInformation", 0);
			drv_atms_delay = 500/portTICK_PERIOD_MS;
			drv_atms_state = sm_CheckProductInformation;			
			return;
			break;
		}
		case sm_NormalMeasure: {			
			drv_atms_delay = 10000/portTICK_PERIOD_MS;
			break;
		}
	}
}

#endif //ENABLE_DRIVER_ATMS100133