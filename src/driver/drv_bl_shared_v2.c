#include "drv_bl_shared_v2.h"
#include "../obk_config.h"

#if ENABLE_BLv2_SHARED

#include "../new_cfg.h"
#include "../new_pins.h"
#include "../cJSON/cJSON.h"
#include "../hal/hal_flashVars.h"
#include "../logging/logging.h"
#include "../mqtt/new_mqtt.h"
#include "../hal/hal_ota.h"
#include "drv_local.h"
#include "drv_deviceclock.h"
#include "drv_public.h"
#include "drv_uart.h"
#include "../cmnds/cmd_public.h" //for enum EventCode
#include <math.h>
#include "../libraries/obktime/obktime.h"	// for time functions

#define BLV2_EMETERS_MAX 2

#if ENABLE_BL_TWIN
const int OBK_CONSUMPTION_STORED_LAST[2] = { OBK_CONSUMPTION_YESTERDAY,OBK_CONSUMPTION_TODAY };
#else
const int OBK_CONSUMPTION_STORED_LAST[1] = { OBK_CONSUMPTION__DAILY_LAST };
#endif

/*IMPORTANT: keep the same like in drv_public.h energySensorNames_t*/
typedef struct  {
	const char* hass_dev_class;
	const char* units;
	char* name_friendly;
	char* name_mqtt;
	char* hass_uniq_id_suffix; //keep identifiers persistent in case OBK_ENERG_SENSOR changes
} blv2_energySensorNames_t;

typedef struct {
	portTickType 	energyCounterStamp;
	float 			lastSavedEnergyImpCounterValue;
	float 			lastSavedEnergyExpCounterValue;
	int 			stat_updatesSkipped;
	int 			stat_updatesSent;
	bool 			sensors_reciveddata;  //1 if data received
	int 			actual_mday;
#if ENABLED_TARIFED_ENERGY
	bool			tarif_tz_ok;
	time_t		 	tarif_tz[TARIFED_TIMEZONE_MAX];
#endif
#if ENABLED_MONTH_ENERGY
	int 			actual_month;
	ENERGY_DATA_V2	month_energy[12];
#endif
} blv2_energy_meter_data_t;

// Order corrsponds to enums OBK_VOLTAGE - OBK__LAST
// note that Wh/kWh units are overridden in hass_init_energy_sensor_device_info()
const char UNIT_WAH[] = "Wh";
const char UNIT_WRH[] = "VArh";
struct energysensor {
	blv2_energySensorNames_t names;
	byte rounding_decimals;
	// Variables below are for optimization
	// We can't send a full MQTT update every second.
	// It's too much for Beken, and it's too much for LWIP 2 MQTT library,
	// especially when actively browsing site and using JS app Log Viewer.
	// It even fails to publish with -1 error (can't alloc next packet)
	// So we publish when value changes from certain threshold or when a certain time passes.
	float changeSendThreshold;
	double lastReading; //double only needed for energycounter i.e. OBK_CONSUMPTION_TOTAL to avoid rounding issues as value becomes high
	double lastSentValue; // what are the last values we sent over the MQTT?
	int noChangeFrame; // how much update frames has passed without sending MQTT update of read values?
};

typedef struct energysensor energysensor_t;
typedef struct {
  energysensor_t sensors[OBK__NUM_SENSORS];
} energysensdataset_t;

enum hass_dev_class_names_e {
	hass_dev_class_voltage,
	hass_dev_class_current,
	hass_dev_class_power,
	hass_dev_class_frequency,
	hass_dev_class_apparent_power,
	hass_dev_class_reactive_power,
	hass_dev_class_power_factor,
	hass_dev_class_energy,
	hass_dev_class_timestamp};
const char* hass_dev_class_names[] = {"voltage", "current", "power", "frequency", "apparent_power", "reactive_power", "power_factor", "energy", "timestamp"};

enum units_names_e {
	unit_voltage,
	unit_current,
	unit_power,
	unit_frequency,
	unit_apparent_power,
	unit_reactive_power,
	unit_power_factor,
	unit_energy_wa,
	unit_energy_wr,
	unit_timestamp};
const char* units_names[] = {"V", "A", "W", "Hz", "VA", "VAr", "", UNIT_WAH, UNIT_WRH, ""};

typedef struct {
	int 				 		meter_count;
	energySensorMode_t 	 		meter_mode;
	int 						measuredsensors; /* mesuared sensors */
	uint32_t                    outputsensors;   /* mesuared and calculated sensors*/
	energysensdataset_t  	 	*datasetlist;
	blv2_energy_meter_data_t 	*energy_meter_data;
	portTickType        		*energyCounterStamp;
	bool 				 		energyCounterStatsEnable; // = false;
	int                  		energyCounterSampleCount; // = 60;
	int                  		energyCounterSampleInterval; // = 60;
	float 			    		*energyCounterMinutes;// = NULL;
	portTickType 		 		energyCounterMinutesStamp;
	long                 		energyCounterMinutesIndex;
	bool                 		energyCounterStatsJSONEnable;// = false;

	float                		changeSavedThresholdEnergy; // = 1000.0f;
	long                 		ConsumptionSaveCounter;// = 0;
	portTickType         		lastConsumptionSaveStamp;
	time_t               		ConsumptionResetTime; // = 0;

	int                  		changeSendAlwaysFrames;// = 60;
	int                  		changeDoNotSendMinFrames; // = 5;

} blv2_energy_meter_t;

static int            		blv2_energy_meter_init = 0;
static blv2_energy_meter_t 	blv2_energy_meter;


/**
 * @brief Reset recived data flag
 */
void BLv2_ResetRecivedDataBool() {
  for (int i = 0; i < blv2_energy_meter.meter_count; i++) blv2_energy_meter.energy_meter_data[i].sensors_reciveddata = 0;
}

/**
 * @brief Append measurment information to html page (actual actions)
 */
void BLv2_AppendInformationToHTTPIndexPageEx(int meter, http_request_t *request)
{
	if ((meter < 0) || (meter > (blv2_energy_meter.meter_count-1))) return;  //to avoid bad index on data[BL_SENSDATASETS_COUNT]
	energysensdataset_t* sensdataset = &blv2_energy_meter.datasetlist[meter];

    int i;
    const char *mode;
//    struct tm *ltm;

    if(DRV_IsRunning("BL0937")) {
        mode = "BL0937";
    } else if(DRV_IsRunning("BL0942")) {
        mode = "BL0942";
    } else if (DRV_IsRunning("BL0942SPI")) {
        mode = "BL0942SPI";
    } else if(DRV_IsRunning("CSE7766")) {
        mode = "CSE7766";
    } else if(DRV_IsRunning("RN8209")) {
        mode = "RN8209";
    } else if(DRV_IsRunning("ATMS100133")) {
        mode = "ATMS100133";
    } else {
        mode = "PWR";
    }

    poststr(request, "<hr><table style='width:100%'>");

	for (int i = OBK__FIRST; i <= OBK_CONSUMPTION__DAILY_LAST; i++) {
		// conditions for frequency
		if (i == OBK_FREQUENCY && (meter != BL_SENSORS_IX_0 || isnan(sensdataset->sensors[i].lastReading))) continue;
		if ((blv2_energy_meter.energyCounterMinutes == NULL) && (i == OBK_CONSUMPTION_LAST_HOUR)) {
			continue;
		}
		if (BLv2_HasEnergySensorReading(meter, i) == false) continue;
		if (i <= OBK__NUM_MEASUREMENTS || TIME_IsTimeSynced()) {
			poststr(request, "<tr><td><b>");
			poststr(request, sensdataset->sensors[i].names.name_friendly);
			poststr(request, "</b></td><td style='text-align: right;'>");
			hprintf255(request, "%.*f</td><td>%s</td>", sensdataset->sensors[i].rounding_decimals,
					(((i >= OBK_CONSUMPTION_TOTAL) && (i <= OBK__NUM_MEASUREMENTS))? 0.001 : 1) * sensdataset->sensors[i].lastReading, //always display OBK_CONSUMPTION_TOTAL in kwh
					 ((i >= OBK_CONSUMPTION_TOTAL) && (i <= OBK__NUM_MEASUREMENTS))? "kWh": sensdataset->sensors[i].names.units);
		}
	};

    poststr(request, "</table>");

    hprintf255(request, "(changes sent %i, skipped %i, saved %li) - %s<hr>",
		blv2_energy_meter.energy_meter_data[meter].stat_updatesSent,
		blv2_energy_meter.energy_meter_data[meter].stat_updatesSkipped,
		blv2_energy_meter.ConsumptionSaveCounter,
		mode);
#if ENABLED_TARIFED_ENERGY
	poststr(request, "<h5>Timezones:");
	if  (blv2_energy_meter.energy_meter_data[meter].tarif_tz_ok) {
		for (int tarif_i=0; tarif_i<TARIFED_TIMEZONE_MAX; tarif_i++) {
			TimeComponents tarif_tc = calculateComponents(blv2_energy_meter.energy_meter_data[meter].tarif_tz[tarif_i]);
			hprintf255(request, " T%i: %i:%i;", tarif_i+1, tarif_tc.hour, tarif_tc.minute);
		}
	} else {
		poststr(request, "not set or invalid.");
	}
	poststr(request, "</h5>");
#endif //ENABLED_TARIFED_ENERGY

#if ENABLED_MONTH_ENERGY
	char* months_names[12] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
	poststr(request, "<h5>Energy at the end of month:<hr><table style='width:100%'><tr><td>Month</td>");
	if (blv2_energy_meter.outputsensors&(0x01 << OBK_CONSUMPTION_WAExp_TOTAL)) {
		poststr(request, "<td style='text-align: center;'>Total+,kWh</td><td style='text-align: center;'>Total-,kWh</td>");
	} else {
		poststr(request, "<td style='text-align: center;'>Total,kWh</td>");
	}
#if ENABLED_TARIFED_ENERGY
#if TARIFED_TIMEZONE_MAX > 1
	if (blv2_energy_meter.outputsensors&(0x01 << OBK_CONSUMPTION_WAExp_T1)) {
		poststr(request, "<td style='text-align: center;'>T1+,kWh</td><td style='text-align: center;'>T1-,kWh</td><td style='text-align: center;'>T2+,kWh</td><td style='text-align: center;'>T2-,kWh</td>");
	} else {
		poststr(request, "<td style='text-align: center;'>T1,kWh</td><td style='text-align: center;'>T2,kWh</td>");
	}
#endif
#if TARIFED_TIMEZONE_MAX > 2
	if (blv2_energy_meter.outputsensors&(0x01 << OBK_CONSUMPTION_WAExp_T3)) {
		poststr(request, "<td style='text-align: center;'>T3+,kWh</td><td style='text-align: center;'>T3-,kWh</td>");
	} else {
		poststr(request, "<td style='text-align: center;'>T3,kWh</td>");
	}
#endif
#if TARIFED_TIMEZONE_MAX > 3
	if (blv2_energy_meter.outputsensors&(0x01 << OBK_CONSUMPTION_WAExp_T4)) {
		poststr(request, "<td style='text-align: center;'>T4+,kWh</td><td style='text-align: center;'>T4-,kWh</td>");
	} else {
		poststr(request, "<td style='text-align: center;'>T4,kWh</td>");
	}
#endif
#endif //ENABLED_TARIFED_ENERGY
	for (int month_i=0; month_i<12; month_i++) {
		poststr(request, "<tr><td>");
		poststr(request, months_names[month_i]);
		poststr(request, "</td><td style='text-align: center;'>");
		hprintf255(request, "%.*f</td>", sensdataset->sensors[OBK_CONSUMPTION_TOTAL].rounding_decimals,
					blv2_energy_meter.energy_meter_data[meter].month_energy[month_i].WA_Import_Total * 0.001);
#if ENABLED_TARIFED_ENERGY
#if TARIFED_TIMEZONE_MAX > 1
		poststr(request, "<td style='text-align: center;'>");
		hprintf255(request, "%.*f</td>", sensdataset->sensors[OBK_CONSUMPTION_TOTAL].rounding_decimals,
					blv2_energy_meter.energy_meter_data[meter].month_energy[month_i].WA_Import[0] * 0.001);
		poststr(request, "<td style='text-align: center;'>");
		hprintf255(request, "%.*f</td>", sensdataset->sensors[OBK_CONSUMPTION_TOTAL].rounding_decimals,
					blv2_energy_meter.energy_meter_data[meter].month_energy[month_i].WA_Import[1] * 0.001);
#endif
#if TARIFED_TIMEZONE_MAX > 2
		poststr(request, "<td style='text-align: center;'>");
		hprintf255(request, "%.*f</td>", sensdataset->sensors[OBK_CONSUMPTION_TOTAL].rounding_decimals,
					blv2_energy_meter.energy_meter_data[meter].month_energy[month_i].WA_Import[2] * 0.001);
#endif
#if TARIFED_TIMEZONE_MAX > 3
		poststr(request, "<td style='text-align: center;'>");
		hprintf255(request, "%.*f</td>", sensdataset->sensors[OBK_CONSUMPTION_TOTAL].rounding_decimals,
					blv2_energy_meter.energy_meter_data[meter].month_energy[month_i].WA_Import[3] * 0.001);
#endif
#endif //ENABLED_TARIFED_ENERGY
	}
	poststr(request, "</table>");
#endif //ENABLED_MONTH_ENERGY

    if (meter == BL_SENSORS_IX_0) {
		poststr(request, "<h5>Energy Clear Date: ");
		if (blv2_energy_meter.ConsumptionResetTime) {
			hprintf255(request, "%s",TS2STR(blv2_energy_meter.ConsumptionResetTime,TIME_FORMAT_LONG));
		} else {
			poststr(request, "(not set)");
		}

		hprintf255(request, "<br>");
		if(TIME_IsTimeSynced()==false) {
			hprintf255(request,"Device time not in sync, daily energy stats disabled.");
		}
		hprintf255(request, "</h5>");

		if (blv2_energy_meter.energyCounterStatsEnable == true)
		{
			/********************************************************************************************************************/
			hprintf255(request,"<h2>Periodic Statistics</h2><h5>Consumption (during this period): ");
			hprintf255(request,"%1.*f Wh<br>", sensdataset->sensors[OBK_CONSUMPTION_LAST_HOUR].rounding_decimals, DRV_GetReading_v2(meter, OBK_CONSUMPTION_LAST_HOUR));
			hprintf255(request,"Sampling interval: %d sec<br>History length: ",blv2_energy_meter.energyCounterSampleInterval);
			hprintf255(request,"%d samples<br>History per samples:<br>",blv2_energy_meter.energyCounterSampleCount);
			if (blv2_energy_meter.energyCounterMinutes != NULL) {
				for(i=0; i<blv2_energy_meter.energyCounterSampleCount; i++) {
					if ((i%20)==0) {
						hprintf255(request, "%1.1f", blv2_energy_meter.energyCounterMinutes[i]);
					} else {
						hprintf255(request, ", %1.1f", blv2_energy_meter.energyCounterMinutes[i]);
					}
					if ((i%20)==19){
						hprintf255(request, "<br>");
					}
				}
				// energyCounterMinutesIndex is a long type, we need to use %ld instead of %d
				if ((i%20)!=0)
					hprintf255(request, "<br>");
				hprintf255(request, "History Index: %ld<br>JSON Stats: %s <br>", blv2_energy_meter.energyCounterMinutesIndex,
					(blv2_energy_meter.energyCounterStatsJSONEnable == true) ? "enabled" : "disabled");
			}

			hprintf255(request, "</h5>");
		} else {
			hprintf255(request,"<h5>Periodic Statistics disabled. Use startup command SetupEnergyStats to enable function.</h5>");
      }
      /********************************************************************************************************************/
    }
}

/**
 * @brief Append measurment information to html page
 */
void BLv2_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState) {
	if (bPreState) return;
	if (!blv2_energy_meter_init) return; // Driver not init yet
	for (int meter_i = 0; meter_i < blv2_energy_meter.meter_count; meter_i++) {
		if (blv2_energy_meter.energy_meter_data[meter_i].sensors_reciveddata) {
			BLv2_AppendInformationToHTTPIndexPageEx(meter_i, request);
		}
	}
}
/**
 * @brief Save energy to flash. Only for two first meter channels!
 */
void BLv2_SaveTarifAndMonthStatistics() {
	ENERGY_METERING_DATA_V2 energy_data_new;
	for (int meter_i=0; meter_i < blv2_energy_meter.meter_count; meter_i++) {
#if ENABLED_TARIFED_ENERGY
		for (int tarif_i = 0; tarif_i < TARIFED_TIMEZONE_MAX; tarif_i++) {
			// copy values to store
			energy_data_new.energy_total.WA_Import[tarif_i] = blv2_energy_meter.datasetlist[meter_i].sensors[OBK_CONSUMPTION_WAImp_T1+tarif_i].lastReading;
			energy_data_new.energy_total.WA_Export[tarif_i] = blv2_energy_meter.datasetlist[meter_i].sensors[OBK_CONSUMPTION_WAExp_T1+tarif_i].lastReading;
			// copy time zones
			energy_data_new.timezones[tarif_i] = blv2_energy_meter.energy_meter_data[meter_i].tarif_tz[tarif_i];
		}
#endif
#if ENABLED_MONTH_ENERGY
		energy_data_new.actual_month = blv2_energy_meter.energy_meter_data[meter_i].actual_month;
		memmove((void*)&energy_data_new.month_energy, (void*)&blv2_energy_meter.energy_meter_data[meter_i].month_energy, 12*sizeof(ENERGY_DATA_V2));
#endif
		HAL_FlashVars_SaveEnergy_BLv2(&energy_data_new, meter_i);
	}
}
/**
 * @brief Save energy to flash. Only for two first meter channels!
 */
void BLv2_SaveEmeteringStatistics()
{
	energysensdataset_t* sensdataset_chA = &blv2_energy_meter.datasetlist[BL_SENSORS_IX_0];
	energysensdataset_t* sensdataset_chB = NULL;
	if (blv2_energy_meter.meter_count > 1) {
		sensdataset_chB = &blv2_energy_meter.datasetlist[BL_SENSORS_IX_1];
	}

	ENERGY_METERING_DATA data;

    memset(&data, 0, sizeof(ENERGY_METERING_DATA));

    data.TotalConsumption = (float)sensdataset_chA->sensors[OBK_CONSUMPTION_TOTAL].lastReading;
    data.TodayConsumpion = (float)sensdataset_chA->sensors[OBK_CONSUMPTION_TODAY].lastReading;
    data.YesterdayConsumption = (float)sensdataset_chA->sensors[OBK_CONSUMPTION_YESTERDAY].lastReading;
    data.actual_mday = blv2_energy_meter.energy_meter_data[BL_SENSORS_IX_0].actual_mday;	//one in flashvars is enough, I assume that both channels are synchronized
	if (sensdataset_chB) {
#if ENABLE_BL_TWIN
		data.TotalConsumption_b = (float)sensdataset_chB->sensors[OBK_CONSUMPTION_TOTAL].lastReading;
		data.TodayConsumpion_b  = (float)sensdataset_chB->sensors[OBK_CONSUMPTION_TODAY].lastReading;
#else
		data.ConsumptionHistory[0] = (float)sensdataset_chB->sensors[OBK_CONSUMPTION_TOTAL].lastReading;
		data.ConsumptionHistory[1] = (float)sensdataset_chB->sensors[OBK_CONSUMPTION_TODAY].lastReading;
#endif
	} else {
		data.ConsumptionHistory[0] = (float)sensdataset_chA->sensors[OBK_CONSUMPTION_2_DAYS_AGO].lastReading;
		data.ConsumptionHistory[1] = (float)sensdataset_chA->sensors[OBK_CONSUMPTION_3_DAYS_AGO].lastReading;
	}
    data.ConsumptionResetTime = blv2_energy_meter.ConsumptionResetTime;
    blv2_energy_meter.ConsumptionSaveCounter++;
    data.save_counter = blv2_energy_meter.ConsumptionSaveCounter;

    HAL_SetEnergyMeterStatus(&data);
#if ENABLED_MONTH_ENERGY || ENABLED_TARIFED_ENERGY
	BLv2_SaveTarifAndMonthStatistics();
#endif
}

/**
 * @brief Set energy of given <meter> to new value <pvalue> or reset it ro zero, if NULL
 */
commandResult_t BLv2_ResetEnergyCounterEx(int meter, float* pvalue)
{
	if (!blv2_energy_meter_init) return CMD_RES_ERROR; //energy meter not init yet
	if ((meter < 0) || (meter > (blv2_energy_meter.meter_count-1))) return CMD_RES_ERROR;  //to avoid bad index
	energysensdataset_t* sensdataset = &blv2_energy_meter.datasetlist[meter];
    int i;
    if (!pvalue) {
		//addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "ResEC %i", meter);
		blv2_energy_meter.energy_meter_data[meter].lastSavedEnergyImpCounterValue = 0.0; //20250203 reset lastSavedEnergyImpCounterValue, otherwise the values will not be saved until restart BL
		sensdataset->sensors[OBK_CONSUMPTION_TOTAL].lastReading = 0.0;
		blv2_energy_meter.energy_meter_data[meter].energyCounterStamp = xTaskGetTickCount();
        if (blv2_energy_meter.energyCounterStatsEnable == true) {
            if (blv2_energy_meter.energyCounterMinutes != NULL) {
                for(i = 0; i < blv2_energy_meter.energyCounterSampleCount; i++) {
                    blv2_energy_meter.energyCounterMinutes[i] = 0.0;
                }
            }
            blv2_energy_meter.energyCounterMinutesStamp = xTaskGetTickCount();
            blv2_energy_meter.energyCounterMinutesIndex = 0;
        }
        for(i = OBK_CONSUMPTION__DAILY_FIRST; i <= OBK_CONSUMPTION__DAILY_LAST; i++) {
			sensdataset->sensors[i].lastReading = 0.0;
        }
#ifdef ENABLED_TARIFED_ENERGY
		for (int tarif_i = 0; tarif_i < TARIFED_TIMEZONE_MAX; tarif_i++) {
				// copy stored values
				sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1+tarif_i].lastReading = 0.0f;
				sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1+tarif_i].lastReading = 0.0f;
		}
#endif //ENABLED_TARIFED_ENERGY
#ifdef ENABLED_MONTH_ENERGY
		blv2_energy_meter.energy_meter_data[meter].actual_month = TIME_GetMonth();
		memset((void*)&blv2_energy_meter.energy_meter_data[meter].month_energy, 0, 12*sizeof(ENERGY_DATA_V2));
#endif //ENABLED_MONTH_ENERGY
    } else {
		//addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "ResEC %i t=%f", meter,avalue);
		sensdataset->sensors[OBK_CONSUMPTION_TOTAL].lastReading = *pvalue;
		blv2_energy_meter.energy_meter_data[meter].energyCounterStamp = xTaskGetTickCount();
    }
    blv2_energy_meter.ConsumptionResetTime = (time_t)TIME_GetCurrentTime();
    if (OTA_GetProgress()==-1)
    {
		BLv2_SaveEmeteringStatistics();
		blv2_energy_meter.lastConsumptionSaveStamp = xTaskGetTickCount();
    }
    return CMD_RES_OK;
}
/**
 * @brief User command for reset energy counters
 */
commandResult_t BLv2_ResetEnergyCounter(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	if (!blv2_energy_meter_init) return CMD_RES_ERROR; //energy meter not init yet
	Tokenizer_TokenizeString(args, 0);
	int acnt = Tokenizer_GetArgsCount();
	if (acnt == 0) {
		for (int meter_i = 0; meter_i < blv2_energy_meter.meter_count; meter_i++) {
			BLv2_ResetEnergyCounterEx(meter_i, NULL);
		}
	} else {
		float fvalue = Tokenizer_GetArgFloat(0);
		int fsix = BL_SENSORS_IX_0;
		if (acnt>=2) fsix = Tokenizer_GetArgInteger(1);
	    return BLv2_ResetEnergyCounterEx(fsix, &fvalue);
	}
	return CMD_RES_OK;
}

/**
 * @brief User command for setup energy statistics parameters
 */
commandResult_t BLv2_SetupEnergyStatistic(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    // SetupEnergyStats enable sample_time sample_count
    int enable;
    int sample_time;
    int sample_count;
    int json_enable;

    Tokenizer_TokenizeString(args,0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

    enable = Tokenizer_GetArgInteger(0);
    sample_time = Tokenizer_GetArgInteger(1);
    sample_count = Tokenizer_GetArgInteger(2);
    if (Tokenizer_GetArgsCount() >= 4)
        json_enable = Tokenizer_GetArgInteger(3);
    else
        json_enable = 0;

    /* Security limits for sample interval */
    if (sample_time <10)
        sample_time = 10;
    if (sample_time >900)
        sample_time = 900;

    /* Security limits for sample count */
    if (sample_count < 10)
        sample_count = 10;
    if (sample_count > 180)
        sample_count = 180;

    /* process changes */
    if (enable != 0)
    {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Consumption History enabled");
        /* Enable function */
        blv2_energy_meter.energyCounterStatsEnable = true;
        if (blv2_energy_meter.energyCounterSampleCount != sample_count) {
            /* upgrade sample count, free memory */
            if (blv2_energy_meter.energyCounterMinutes != NULL)
                os_free(blv2_energy_meter.energyCounterMinutes);
            blv2_energy_meter.energyCounterMinutes = NULL;
            blv2_energy_meter.energyCounterSampleCount = sample_count;
        }
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Sample Count: %d", blv2_energy_meter.energyCounterSampleCount);
        if (blv2_energy_meter.energyCounterSampleInterval != sample_time)
        {
            /* change sample time */
            blv2_energy_meter.energyCounterSampleInterval = sample_time;
            if (blv2_energy_meter.energyCounterMinutes != NULL)
                memset(blv2_energy_meter.energyCounterMinutes, 0, blv2_energy_meter.energyCounterSampleCount*sizeof(float));
        }

        if (blv2_energy_meter.energyCounterMinutes == NULL)
        {
            /* allocate new memeory */
            blv2_energy_meter.energyCounterMinutes = (float*)os_malloc(sample_count*sizeof(float));
            if (blv2_energy_meter.energyCounterMinutes != NULL)
            {
                memset(blv2_energy_meter.energyCounterMinutes, 0, blv2_energy_meter.energyCounterSampleCount*sizeof(float));
            }
        }
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Sample Interval: %d", blv2_energy_meter.energyCounterSampleInterval);

        blv2_energy_meter.energyCounterMinutesStamp = xTaskGetTickCount();
        blv2_energy_meter.energyCounterMinutesIndex = 0;
    } else {
        /* Disable Consimption Nistory */
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Consumption History disabled");
        blv2_energy_meter.energyCounterStatsEnable = false;
        if (blv2_energy_meter.energyCounterMinutes != NULL)
        {
            os_free(blv2_energy_meter.energyCounterMinutes);
            blv2_energy_meter.energyCounterMinutes = NULL;
        }
        blv2_energy_meter.energyCounterSampleCount = sample_count;
        blv2_energy_meter.energyCounterSampleInterval = sample_time;
    }

    blv2_energy_meter.energyCounterStatsJSONEnable = (json_enable != 0) ? true : false;

    return CMD_RES_OK;
}

/**
 * @brief User command for setup interval to publish MQTT
 */
commandResult_t BLv2_VCPPublishIntervals(const void *context, const char *cmd, const char *args, int cmdFlags)
{
	if (!blv2_energy_meter_init) return CMD_RES_ERROR; //energy meter not init yet
	Tokenizer_TokenizeString(args, 0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	blv2_energy_meter.changeDoNotSendMinFrames = Tokenizer_GetArgInteger(0);
	blv2_energy_meter.changeSendAlwaysFrames = Tokenizer_GetArgInteger(1);

	return CMD_RES_OK;
}

/**
 * @brief User command for setup precision
 */
commandResult_t BLv2_VCPPrecision(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	if (!blv2_energy_meter_init) return CMD_RES_ERROR; //energy meter not init yet
	Tokenizer_TokenizeString(args, 0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	for (int meter_i = 0; meter_i < blv2_energy_meter.meter_count; meter_i++) {
		energysensdataset_t* sensdataset = &blv2_energy_meter.datasetlist[meter_i];
		for (int i = 0; i < Tokenizer_GetArgsCount(); i++) {
			int val = Tokenizer_GetArgInteger(i);
			switch(i) {
			case 0: // voltage
				sensdataset->sensors[OBK_VOLTAGE].rounding_decimals = val;
				sensdataset->sensors[OBK_VOLTAGE_L1].rounding_decimals = val;
				sensdataset->sensors[OBK_VOLTAGE_L2].rounding_decimals = val;
				sensdataset->sensors[OBK_VOLTAGE_L3].rounding_decimals = val;
				break;
			case 1: // current
				sensdataset->sensors[OBK_CURRENT].rounding_decimals = val;
				sensdataset->sensors[OBK_CURRENT_L1].rounding_decimals = val;
				sensdataset->sensors[OBK_CURRENT_L2].rounding_decimals = val;
				sensdataset->sensors[OBK_CURRENT_L3].rounding_decimals = val;
				break;
			case 2: // power
				sensdataset->sensors[OBK_POWER].rounding_decimals = val;
				sensdataset->sensors[OBK_POWER_L1].rounding_decimals = val;
				sensdataset->sensors[OBK_POWER_L2].rounding_decimals = val;
				sensdataset->sensors[OBK_POWER_L3].rounding_decimals = val;
				sensdataset->sensors[OBK_POWER_APPARENT].rounding_decimals = val;
				sensdataset->sensors[OBK_POWER_REACTIVE].rounding_decimals = val;
				break;
			case 3: // energy
				for (int j = OBK_CONSUMPTION__DAILY_FIRST; j <= OBK_CONSUMPTION__DAILY_LAST; j++) {
					sensdataset->sensors[j].rounding_decimals = val;
				};
				break;
			case 4: // frequency
				sensdataset->sensors[OBK_FREQUENCY].rounding_decimals = val;
				break;
			};
		}
	}
	return CMD_RES_OK;
}

/**
 * @brief User command for setup measuarments (voltage, current and etc.) threshold. The same for all meter channels
 */
commandResult_t BLv2_VCPPublishThreshold(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	if (!blv2_energy_meter_init) return CMD_RES_ERROR; //energy meter not init yet
	Tokenizer_TokenizeString(args, 0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	for (int meter_i = 0; meter_i < blv2_energy_meter.meter_count; meter_i++) {
		energysensdataset_t* sensdataset = &blv2_energy_meter.datasetlist[meter_i];
		sensdataset->sensors[OBK_VOLTAGE].changeSendThreshold = Tokenizer_GetArgFloat(0);
		sensdataset->sensors[OBK_CURRENT].changeSendThreshold = Tokenizer_GetArgFloat(1);
		sensdataset->sensors[OBK_POWER].changeSendThreshold = Tokenizer_GetArgFloat(2);
		sensdataset->sensors[OBK_POWER_APPARENT].changeSendThreshold = Tokenizer_GetArgFloatDefault(2,sensdataset->sensors[OBK_POWER_APPARENT].changeSendThreshold);
		sensdataset->sensors[OBK_POWER_REACTIVE].changeSendThreshold = Tokenizer_GetArgFloatDefault(2,sensdataset->sensors[OBK_POWER_REACTIVE].changeSendThreshold);
		//sensdataset->sensors[OBK_POWER_FACTOR].changeSendThreshold = Tokenizer_GetArgFloat(TODO);

		if (Tokenizer_GetArgsCount() >= 4) {
			for (int i = OBK_CONSUMPTION_LAST_HOUR; i <= OBK_CONSUMPTION__DAILY_LAST; i++) {
				sensdataset->sensors[i].changeSendThreshold = Tokenizer_GetArgFloat(3);
			}
		}

		sensdataset->sensors[OBK_FREQUENCY].changeSendThreshold = Tokenizer_GetArgFloatDefault(4,sensdataset->sensors[OBK_FREQUENCY].changeSendThreshold);
	}

	return CMD_RES_OK;
}

/**
 * @brief User command for setup consumption threshold
 */
commandResult_t BLv2_SetupConsumptionThreshold(const void *context, const char *cmd, const char *args, int cmdFlags)
{
	if (!blv2_energy_meter_init) return CMD_RES_ERROR; //energy meter not init yet
    float threshold;
    Tokenizer_TokenizeString(args,0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

    threshold = (float)atof(Tokenizer_GetArg(0));

    if (threshold<1.0f)
        threshold = 1.0f;
    if (threshold>5000.0f)
        threshold = 5000.0f;
    blv2_energy_meter.changeSavedThresholdEnergy = threshold;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "ConsumptionThreshold: %1.1f", blv2_energy_meter.changeSavedThresholdEnergy);

    return CMD_RES_OK;
}
#if ENABLED_TARIFED_ENERGY
bool BLv2_CheckTarifTimezone(int meter) {
	time_t last_tz = 0;
	time_t onedayTime = (time_t)dateToEpoch(1970,1,1,23,59,59);
	for (int tarif_i = 0; tarif_i < TARIFED_TIMEZONE_MAX; tarif_i++) {
		if (blv2_energy_meter.energy_meter_data[meter].tarif_tz[tarif_i] > onedayTime) return false;
		if (blv2_energy_meter.energy_meter_data[meter].tarif_tz[tarif_i] < last_tz)    return false;
		last_tz = blv2_energy_meter.energy_meter_data[meter].tarif_tz[tarif_i];
	}
	return true;
}

/**
 * @brief User command for setup consumption threshold
 */
commandResult_t BLv2_SetTarifTimezone(const void *context, const char *cmd, const char *args, int cmdFlags)
{
	if (!blv2_energy_meter_init) return CMD_RES_ERROR; //energy meter not init yet
    int meter_i, tarif;
    Tokenizer_TokenizeString(args,0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	meter_i = Tokenizer_GetArgInteger(0);
	tarif = Tokenizer_GetArgInteger(1);
	if ((tarif < 1) || (tarif > TARIFED_TIMEZONE_MAX)) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGYMETER, "Invalid tarif number.");
		return CMD_RES_ERROR;
	}
	tarif--;
	const char *timezone_str = Tokenizer_GetArg(2);

	int hh, mm;
    if (timezone_str) {
		if (sscanf(timezone_str, "%d:%d", &hh, &mm) != 2) {
			addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGYMETER, "Invalid time format: %s", timezone_str);
			return CMD_RES_ERROR;
		}
    } else {
		addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGYMETER, "Invalid time format. Time string is NULL.");
	}
	if (hh >= 0 && hh<=23 && mm>=0 && mm<=59) {
		blv2_energy_meter.energy_meter_data[meter_i].tarif_tz[tarif] = (time_t)dateToEpoch(1970,1,1,hh,mm,0);
	} else {
		addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGYMETER, "Invalid time format: hour=%d, minutes=%d", hh, mm);
		return CMD_RES_ERROR;
	}

	if (BLv2_CheckTarifTimezone(meter_i) == true) {
		blv2_energy_meter.energy_meter_data[meter_i].tarif_tz_ok = 1;
		if (OTA_GetProgress()==-1) {
			BLv2_SaveTarifAndMonthStatistics();
		}
	}
	return CMD_RES_OK;
}
#endif
/**
 * @brief Returns true, if there is opened relay, to force current and power to zero. What about second meter channel?
 */
bool Channel_AreAllRelaysOpen() {
	int i, role, ch;

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		role = g_cfg.pins.roles[i];
		ch = g_cfg.pins.channels[i];
		if (role == IOR_Relay) {
			// this channel is high = relay is set
			if (CHANNEL_Get(ch)) {
				return false;
			}
		}
		if (role == IOR_Relay_n) {
			// this channel is low = relay_n is set
			if (CHANNEL_Get(ch)==false) {
				return false;
			}
		}
		if (role == IOR_BridgeForward) {
			// this channel is high = relay is set
			if (CHANNEL_Get(ch)) {
				return false;
			}
		}
	}
	return true;
}

/**
 * @brief Change energy units, if flasg OBK_FLAG_MQTT_ENERGY_IN_KWH is settup
 */
float BL_ChangeEnergyUnitIfNeeded(float Wh) {
	if (CFG_HasFlag(OBK_FLAG_MQTT_ENERGY_IN_KWH)) {
		return Wh * 0.001f;
	}
	return Wh;
}

/**
 * @brief Process Update Energy only
 */
void BLv2_ProcessUpdateEnergy(int meter, float energyWAp, float energyWRp, float energyWAm, float energyWRm, int line) {
	if (!blv2_energy_meter_init) return;
	if (meter < 0) return;
	if (meter > (blv2_energy_meter.meter_count-1)) return;
	energysensdataset_t* sensdataset = &blv2_energy_meter.datasetlist[meter];
	int i;
	cJSON* root;
	cJSON* stats;
	char *msg;
	portTickType interval;
	portTickType nowTickCount = xTaskGetTickCount();
	time_t deviceTime;
	char datetime[64];
	bool needSave = false;
	float energy_val [4] = {energyWAp, energyWRp, energyWAm, energyWRm};
	int   energy_type[4] = {OBK_CONSUMPTION_WAImp_TOTAL, -1, OBK_CONSUMPTION_WAExp_TOTAL, -1};
	for (int energy_i = 0; energy_i<4; energy_i++) {
		if (energy_type[energy_i]==-1) continue;
		if (isnan(energy_val[energy_i])) continue;
		float energy = 0;
		if (energy_val[energy_i] > 0) {
			energy = energy_val[energy_i];
			needSave = true;
		}
		sensdataset->sensors[energy_type[energy_i]].lastReading += (double)energy;
		blv2_energy_meter.energy_meter_data[meter].energyCounterStamp = nowTickCount;

		// Handle daily consumption
		if (energy_type[energy_i] == OBK_CONSUMPTION_WAImp_TOTAL) {
			sensdataset->sensors[OBK_CONSUMPTION_TODAY].lastReading += energy;
		}
		if (TIME_IsTimeSynced()) { // if time is synchronized
			deviceTime = (time_t)TIME_GetCurrentTime();
			TimeComponents tc = calculateComponents(deviceTime);

			uint8_t mday=tc.day;
			if (blv2_energy_meter.ConsumptionResetTime == 0)	blv2_energy_meter.ConsumptionResetTime = (time_t)deviceTime;

			if (blv2_energy_meter.energy_meter_data[meter].actual_mday == -1) {
				blv2_energy_meter.energy_meter_data[meter].actual_mday = mday;
			}

			if (blv2_energy_meter.energy_meter_data[meter].actual_mday != mday) { // day changed
				for (i = OBK_CONSUMPTION__DAILY_LAST; i >= OBK_CONSUMPTION__DAILY_FIRST; i--) {
					sensdataset->sensors[i].lastReading = sensdataset->sensors[i - 1].lastReading;
				}
				sensdataset->sensors[OBK_CONSUMPTION_TODAY].lastReading = 0.0;
				blv2_energy_meter.energy_meter_data[meter].actual_mday = mday;

				//blv2_energy_meter.stat_updatesSent++;
				/*
				if (OTA_GetProgress() == -1) {
					BLv2_SaveEmeteringStatistics();
					blv2_energy_meter.lastConsumptionSaveStamp = xTaskGetTickCount();
				}
				*/
				needSave = true;
			}
#if ENABLED_TARIFED_ENERGY
			if  (blv2_energy_meter.energy_meter_data[meter].tarif_tz_ok) {
				time_t startdayTime = dateToEpoch(tc.year, tc.month, tc.day, 0, 0, 0);
				int current_tarif = 0;
				do {
					time_t tarifzoneTime = startdayTime + blv2_energy_meter.energy_meter_data[meter].tarif_tz[current_tarif];
					if (deviceTime <= tarifzoneTime) {
						current_tarif--;
						if (current_tarif < 0) current_tarif = TARIFED_TIMEZONE_MAX-1;
						break;
					}
					current_tarif++;
				} while (current_tarif < TARIFED_TIMEZONE_MAX);
				if (current_tarif == TARIFED_TIMEZONE_MAX) current_tarif = TARIFED_TIMEZONE_MAX-1;
				if (current_tarif >= 0 && current_tarif < TARIFED_TIMEZONE_MAX) {
					if (energy_type[energy_i] == OBK_CONSUMPTION_WAImp_TOTAL) {
						sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1+current_tarif].lastReading += energy;
						needSave = true;
					}
					if (energy_type[energy_i] == OBK_CONSUMPTION_WAExp_TOTAL) {
						sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1+current_tarif].lastReading += energy;
						needSave = true;
					}
				} else {
					// ????
				}
			}
#endif
#if ENABLED_MONTH_ENERGY
			if (blv2_energy_meter.energy_meter_data[meter].actual_month != tc.month) {
				while (blv2_energy_meter.energy_meter_data[meter].actual_month != tc.month) {
					blv2_energy_meter.energy_meter_data[meter].month_energy[blv2_energy_meter.energy_meter_data[meter].actual_month].WA_Import_Total = sensdataset->sensors[OBK_CONSUMPTION_WAImp_TOTAL].lastReading;
					blv2_energy_meter.energy_meter_data[meter].month_energy[blv2_energy_meter.energy_meter_data[meter].actual_month].WA_Export_Total = sensdataset->sensors[OBK_CONSUMPTION_WAExp_TOTAL].lastReading;
#if ENABLED_TARIFED_ENERGY
#if TARIFED_TIMEZONE_MAX > 1
					blv2_energy_meter.energy_meter_data[meter].month_energy[blv2_energy_meter.energy_meter_data[meter].actual_month].WA_Import[0] = sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1].lastReading;
					blv2_energy_meter.energy_meter_data[meter].month_energy[blv2_energy_meter.energy_meter_data[meter].actual_month].WA_Import[1] = sensdataset->sensors[OBK_CONSUMPTION_WAImp_T2].lastReading;
					blv2_energy_meter.energy_meter_data[meter].month_energy[blv2_energy_meter.energy_meter_data[meter].actual_month].WA_Export[0] = sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1].lastReading;
					blv2_energy_meter.energy_meter_data[meter].month_energy[blv2_energy_meter.energy_meter_data[meter].actual_month].WA_Export[1] = sensdataset->sensors[OBK_CONSUMPTION_WAExp_T2].lastReading;
#endif
#if TARIFED_TIMEZONE_MAX > 2
					blv2_energy_meter.energy_meter_data[meter].month_energy[blv2_energy_meter.energy_meter_data[meter].actual_month].WA_Import[2] = sensdataset->sensors[OBK_CONSUMPTION_WAImp_T3].lastReading;
					blv2_energy_meter.energy_meter_data[meter].month_energy[blv2_energy_meter.energy_meter_data[meter].actual_month].WA_Export[2] = sensdataset->sensors[OBK_CONSUMPTION_WAExp_T3].lastReading;
#endif
#if TARIFED_TIMEZONE_MAX > 3
					blv2_energy_meter.energy_meter_data[meter].month_energy[blv2_energy_meter.energy_meter_data[meter].actual_month].WA_Import[3] = sensdataset->sensors[OBK_CONSUMPTION_WAImp_T4].lastReading;
					blv2_energy_meter.energy_meter_data[meter].month_energy[blv2_energy_meter.energy_meter_data[meter].actual_month].WA_Export[3] = sensdataset->sensors[OBK_CONSUMPTION_WAExp_T4].lastReading;
#endif
					blv2_energy_meter.energy_meter_data[meter].actual_month++;
#endif 	//ENABLED_TARIFED_ENERGY
					if (blv2_energy_meter.energy_meter_data[meter].actual_month >= 12) blv2_energy_meter.energy_meter_data[meter].actual_month = 0;
				}
				needSave = true;
			}

#endif	//ENABLED_MONTH_ENERGY
		} else {
#if ENABLED_TARIFED_ENERGY

#endif	//ENABLED_TARIFED_ENERGY
		}

		// Handle consumption for period - why only for meter channel A(0) ????
		if ((blv2_energy_meter.energyCounterStatsEnable == true) && (meter==BL_SENSORS_IX_0)) {
			interval = blv2_energy_meter.energyCounterSampleInterval;
			interval *= (1000 / portTICK_PERIOD_MS);
			if ((nowTickCount - blv2_energy_meter.energyCounterMinutesStamp) >= interval) {
				if (blv2_energy_meter.energyCounterMinutes != NULL) {
					sensdataset->sensors[OBK_CONSUMPTION_LAST_HOUR].lastReading = 0;
					for (int i = 0; i < blv2_energy_meter.energyCounterSampleCount; i++) {
						sensdataset->sensors[OBK_CONSUMPTION_LAST_HOUR].lastReading += blv2_energy_meter.energyCounterMinutes[i];
					}
				}
#if ENABLE_MQTT
				if ((blv2_energy_meter.energyCounterStatsJSONEnable == true) && (MQTT_IsReady() == true)) {
					root = cJSON_CreateObject();
					cJSON_AddNumberToObject(root, "uptime", g_secondsElapsed);
					cJSON_AddNumberToObject(root, "consumption_total", BL_ChangeEnergyUnitIfNeeded(DRV_GetReading_v2(meter, OBK_CONSUMPTION_TOTAL)));
					cJSON_AddNumberToObject(root, "consumption_last_hour", BL_ChangeEnergyUnitIfNeeded(DRV_GetReading_v2(meter, OBK_CONSUMPTION_LAST_HOUR)));
					cJSON_AddNumberToObject(root, "consumption_stat_index", blv2_energy_meter.energyCounterMinutesIndex);
					cJSON_AddNumberToObject(root, "consumption_sample_count", blv2_energy_meter.energyCounterSampleCount);
					cJSON_AddNumberToObject(root, "consumption_sampling_period", blv2_energy_meter.energyCounterSampleInterval);
					if(TIME_IsTimeSynced() == true) {
						cJSON_AddNumberToObject(root, "consumption_today", BL_ChangeEnergyUnitIfNeeded(DRV_GetReading_v2(meter, OBK_CONSUMPTION_TODAY)));
						cJSON_AddNumberToObject(root, "consumption_yesterday", BL_ChangeEnergyUnitIfNeeded(DRV_GetReading_v2(meter, OBK_CONSUMPTION_YESTERDAY)));
#if PLATFORM_ECR6600
						// for ECR6000 printf("%+03i", 1); will not be "+01" but "0+1" ...
						int ttm = TIME_GetTimesZoneOfsSeconds()/60;	// used to get +/- sign and minutes
						snprintf(datetime, sizeof(datetime), "%.16s%c%02i:%02i",TS2STR(blv2_energy_meter.ConsumptionResetTime, TIME_FORMAT_ISO_8601),
							ttm < 0 ? '-': '+', abs(ttm/60), abs(ttm) % 60);
#else
						snprintf(datetime, sizeof(datetime), "%.16s%+03i:%02i",TS2STR(blv2_energy_meter.ConsumptionResetTime, TIME_FORMAT_ISO_8601),
							TIME_GetTimesZoneOfsSeconds()/3600, (abs(TIME_GetTimesZoneOfsSeconds())/60) % 60);
#endif
						cJSON_AddStringToObject(root, "consumption_clear_date", datetime);
					}

					if (blv2_energy_meter.energyCounterMinutes != NULL)
					{
						stats = cJSON_CreateArray();
						// WARNING - it causes HA problems?
						// See: https://github.com/openshwprojects/OpenBK7231T_App/issues/870
						// Basically HA has 256 chars state limit?
						// Wait, no, it's over 256 even without samples?
						for(i = 0; i < blv2_energy_meter.energyCounterSampleCount; i++) {
							cJSON_AddItemToArray(stats, cJSON_CreateNumber(blv2_energy_meter.energyCounterMinutes[i]));
						}
						cJSON_AddItemToObject(root, "consumption_samples", stats);
					}

					if(TIME_IsTimeSynced() == true) {
						stats = cJSON_CreateArray();
						for(i = OBK_CONSUMPTION__DAILY_FIRST; i <= OBK_CONSUMPTION__DAILY_LAST; i++) {
							cJSON_AddItemToArray(stats, cJSON_CreateNumber(DRV_GetReading_v2(meter, i)));
						}
						cJSON_AddItemToObject(root, "consumption_daily", stats);
					}

					msg = cJSON_PrintUnformatted(root);
					cJSON_Delete(root);

					// addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "JSON Printed: %d bytes", strlen(msg));

					MQTT_PublishMain_StringString("consumption_stats", msg, 0);
					blv2_energy_meter.energy_meter_data[meter].stat_updatesSent++;
					os_free(msg);
				}
#endif
				if (blv2_energy_meter.energyCounterMinutes != NULL) {
					for (i=blv2_energy_meter.energyCounterSampleCount-1;i>0;i--) {
						if (blv2_energy_meter.energyCounterMinutes[i-1]>0.0) {
							blv2_energy_meter.energyCounterMinutes[i] = blv2_energy_meter.energyCounterMinutes[i-1];
						} else {
							blv2_energy_meter.energyCounterMinutes[i] = 0.0;
						}
					}
					blv2_energy_meter.energyCounterMinutes[0] = 0.0;
				}
				blv2_energy_meter.energyCounterMinutesStamp = nowTickCount;
				blv2_energy_meter.energyCounterMinutesIndex++;
			}
			if (blv2_energy_meter.energyCounterMinutes != NULL && energy_type[energy_i] == OBK_CONSUMPTION_WAImp_TOTAL)
				blv2_energy_meter.energyCounterMinutes[0] += energy;
		}
	}
	// Save if need
	if (needSave) {
		if (((sensdataset->sensors[OBK_CONSUMPTION_WAImp_TOTAL].lastReading - blv2_energy_meter.energy_meter_data[meter].lastSavedEnergyImpCounterValue) >= blv2_energy_meter.changeSavedThresholdEnergy) ||
		    ((sensdataset->sensors[OBK_CONSUMPTION_WAExp_TOTAL].lastReading - blv2_energy_meter.energy_meter_data[meter].lastSavedEnergyImpCounterValue) >= blv2_energy_meter.changeSavedThresholdEnergy) ||
			((nowTickCount - blv2_energy_meter.lastConsumptionSaveStamp) >= (6 * 3600 * 1000 / portTICK_PERIOD_MS))) {
			if (OTA_GetProgress() == -1) {
				blv2_energy_meter.energy_meter_data[meter].lastSavedEnergyImpCounterValue = (float)sensdataset->sensors[OBK_CONSUMPTION_WAImp_TOTAL].lastReading;
				blv2_energy_meter.energy_meter_data[meter].lastSavedEnergyExpCounterValue = (float)sensdataset->sensors[OBK_CONSUMPTION_WAExp_TOTAL].lastReading;
				BLv2_SaveEmeteringStatistics();
				blv2_energy_meter.lastConsumptionSaveStamp = nowTickCount;
			}
		}
	}
}

/**
 * @brief Process Update Measuarments only (voltage, current and etc.)
 */
void BLv2_ProcessUpdateMesuaments(int meter, float voltage, float current, float power_active, float power_reactive, float power_apparent, float power_factor, float frequency, int line) {
	if (!blv2_energy_meter_init) return;
	if (meter < 0) return;
	if (meter > (blv2_energy_meter.meter_count-1)) return;
	portTickType xPassedTicks;
	float diff;
	char datetime[64];
	portTickType nowTickCount = xTaskGetTickCount();
	energysensdataset_t* sensdataset = &blv2_energy_meter.datasetlist[meter];
	// I had reports that BL0942 sometimes gives
	// a large, negative peak of current/power
	if (!CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE)) {
		if (!isnan(voltage) && (voltage < 0.0f)) 	voltage = 0.0f;
		if (!isnan(current) && (current < 0.0f)) 	current = 0.0f;
		if (!isnan(power_active) && (power_active < 0.0f)) 		power_active = 0.0f;
		if (!isnan(power_reactive) && (power_reactive < 0.0f)) 		power_reactive = 0.0f;
		if (!isnan(power_apparent) && (power_apparent < 0.0f)) 		power_apparent = 0.0f;
	}
	if (CFG_HasFlag(OBK_FLAG_POWER_FORCE_ZERO_IF_RELAYS_OPEN)) {
		if (Channel_AreAllRelaysOpen()) {
			if (!isnan(power_active)) power_active = 0;
			if (!isnan(power_reactive)) power_reactive = 0;
			if (!isnan(power_apparent)) power_apparent = 0;
			if (!isnan(current)) current = 0;
		}
	}

	#ifdef ENABLE_BL_MOVINGAVG
		if (!isnan(power_active)) {
			switch (line) {
				case 0: power_active = XJ_MovingAverage_float((float)sensdataset->sensors[OBK_POWER].lastReading, power_active); break;
				case 1: power_active = XJ_MovingAverage_float((float)sensdataset->sensors[OBK_POWER_L1].lastReading, power_active); break;
				case 2: power_active = XJ_MovingAverage_float((float)sensdataset->sensors[OBK_POWER_L2].lastReading, power_active); break;
				case 3: power_active = XJ_MovingAverage_float((float)sensdataset->sensors[OBK_POWER_L3].lastReading, power_active); break;
			}
		}
		if (!isnan(power_reactive)) power_reactive = XJ_MovingAverage_float((float)sensdataset->sensors[OBK_POWER_REACTIVE].lastReading, power_reactive);
		if (!isnan(power_apparent)) power_apparent = XJ_MovingAverage_float((float)sensdataset->sensors[OBK_POWER_APPARENT].lastReading, power_apparent);
		if (!isnan(current))  {
			switch (line) {
				case 0: current = XJ_MovingAverage_float((float)sensdataset->sensors[OBK_CURRENT].lastReading, current); break;
				case 1: current = XJ_MovingAverage_float((float)sensdataset->sensors[OBK_CURRENT_L1].lastReading, current); break;
				case 2: current = XJ_MovingAverage_float((float)sensdataset->sensors[OBK_CURRENT_L2].lastReading, current); break;
				case 3: current = XJ_MovingAverage_float((float)sensdataset->sensors[OBK_CURRENT_L3].lastReading, current); break;
			}
		}
	#endif
	double *voltage_lastReading = NULL;
	double *curent_lastReading = NULL;
	double *power_active_lastReading = NULL;
	switch (line) {
		case 0: {
			voltage_lastReading = &sensdataset->sensors[OBK_VOLTAGE].lastReading;
			curent_lastReading = &sensdataset->sensors[OBK_CURRENT].lastReading;
			power_active_lastReading = &sensdataset->sensors[OBK_POWER].lastReading;
			break;
		}
		case 1: {
			voltage_lastReading = &sensdataset->sensors[OBK_VOLTAGE_L1].lastReading;
			curent_lastReading = &sensdataset->sensors[OBK_CURRENT_L1].lastReading;
			power_active_lastReading = &sensdataset->sensors[OBK_POWER_L1].lastReading;
			break;
		}
		case 2: {
			voltage_lastReading = &sensdataset->sensors[OBK_VOLTAGE_L2].lastReading;
			curent_lastReading = &sensdataset->sensors[OBK_CURRENT_L2].lastReading;
			power_active_lastReading = &sensdataset->sensors[OBK_POWER_L2].lastReading;
			break;
		}
		case 3: {
			voltage_lastReading = &sensdataset->sensors[OBK_VOLTAGE_L3].lastReading;
			curent_lastReading = &sensdataset->sensors[OBK_CURRENT_L3].lastReading;
			power_active_lastReading = &sensdataset->sensors[OBK_POWER_L3].lastReading;
			break;
		}
	}
	if ((blv2_energy_meter.measuredsensors&msVoltage) && !isnan(voltage) && voltage_lastReading) *voltage_lastReading = (double)voltage;
	if ((blv2_energy_meter.measuredsensors&msCurrent) && !isnan(current) && curent_lastReading) *curent_lastReading = (double)current;
	if (!isnan(power_active)) {
		if (((!line) && (blv2_energy_meter.measuredsensors&msPowerActiveSumm)) || ((line) && (blv2_energy_meter.measuredsensors&msPowerActivePhase))) {
			if (power_active_lastReading) *power_active_lastReading = (double)power_active;
		}
	}
	if (blv2_energy_meter.measuredsensors&msFrequency) {
		if (!isnan(frequency)) sensdataset->sensors[OBK_FREQUENCY].lastReading = frequency;
	} else {
		// make fake value; TODO: some countries has a 60Hz power grid
		if (blv2_energy_meter.meter_mode == es_1PhaseMode) {
			if (!isnan(sensdataset->sensors[OBK_VOLTAGE].lastReading)) {
				if (sensdataset->sensors[OBK_VOLTAGE].lastReading < 1.0) 	{sensdataset->sensors[OBK_FREQUENCY].lastReading = 0.0;}
				else 												  		{sensdataset->sensors[OBK_FREQUENCY].lastReading = 50.0;}
			}
		} else {
			if (!isnan(sensdataset->sensors[OBK_VOLTAGE_L1].lastReading) ||
			    !isnan(sensdataset->sensors[OBK_VOLTAGE_L2].lastReading) ||
				!isnan(sensdataset->sensors[OBK_VOLTAGE_L3].lastReading)) {

				if ((sensdataset->sensors[OBK_VOLTAGE_L1].lastReading < 1.0) &&
				    (sensdataset->sensors[OBK_VOLTAGE_L2].lastReading < 1.0) &&
					(sensdataset->sensors[OBK_VOLTAGE_L3].lastReading < 1.0)) 	{sensdataset->sensors[OBK_FREQUENCY].lastReading = 0.0;}
				else
																				{sensdataset->sensors[OBK_FREQUENCY].lastReading = 50.0;}
			}
		}
	}

	if (!line) { // for summ measurements
		if (blv2_energy_meter.measuredsensors&msPowerApparentSumm) { // if value supply by meter
			if (!isnan(power_apparent)) sensdataset->sensors[OBK_POWER_APPARENT].lastReading = (double)power_apparent;
		} else { // try calculate it
			if (blv2_energy_meter.meter_mode == es_1PhaseMode) {
				if (!isnan(sensdataset->sensors[OBK_VOLTAGE].lastReading) && !isnan(sensdataset->sensors[OBK_CURRENT].lastReading))
					sensdataset->sensors[OBK_POWER_APPARENT].lastReading = sensdataset->sensors[OBK_VOLTAGE].lastReading * sensdataset->sensors[OBK_CURRENT].lastReading;
			} else {
				if (!isnan(sensdataset->sensors[OBK_VOLTAGE_L1].lastReading) && !isnan(sensdataset->sensors[OBK_CURRENT_L1].lastReading) &&
					!isnan(sensdataset->sensors[OBK_VOLTAGE_L2].lastReading) && !isnan(sensdataset->sensors[OBK_CURRENT_L2].lastReading) &&
				    !isnan(sensdataset->sensors[OBK_VOLTAGE_L3].lastReading) && !isnan(sensdataset->sensors[OBK_CURRENT_L3].lastReading))
					sensdataset->sensors[OBK_POWER_APPARENT].lastReading = sensdataset->sensors[OBK_VOLTAGE_L1].lastReading * sensdataset->sensors[OBK_CURRENT_L1].lastReading +
					                                                       sensdataset->sensors[OBK_VOLTAGE_L2].lastReading * sensdataset->sensors[OBK_CURRENT_L2].lastReading +
					                                                       sensdataset->sensors[OBK_VOLTAGE_L3].lastReading * sensdataset->sensors[OBK_CURRENT_L3].lastReading;
			}
		}
		if (blv2_energy_meter.measuredsensors&msPowerReactiveSumm) { // if value supply by meter
			if (!isnan(power_reactive)) sensdataset->sensors[OBK_POWER_REACTIVE].lastReading = (double)power_reactive;
		} else { // try calculate it
			if (!isnan(sensdataset->sensors[OBK_POWER_APPARENT].lastReading) && !isnan(sensdataset->sensors[OBK_POWER].lastReading)) {
				sensdataset->sensors[OBK_POWER_REACTIVE].lastReading = (sensdataset->sensors[OBK_POWER_APPARENT].lastReading <= fabsf((float)sensdataset->sensors[OBK_POWER].lastReading)
					? 0
					: sqrtf(powf((float)sensdataset->sensors[OBK_POWER_APPARENT].lastReading, 2) -
					powf((float)sensdataset->sensors[OBK_POWER].lastReading, 2)));
			}
		}
		if (blv2_energy_meter.measuredsensors&msPowerFactorSumm) { // if value supply by meter
			if (!isnan(power_factor)) sensdataset->sensors[OBK_POWER_FACTOR].lastReading = (double)power_factor;
		} else { // try calculate it
			if (!isnan(sensdataset->sensors[OBK_POWER_APPARENT].lastReading) && !isnan(sensdataset->sensors[OBK_POWER].lastReading)) {
				sensdataset->sensors[OBK_POWER_FACTOR].lastReading =
					(sensdataset->sensors[OBK_POWER_APPARENT].lastReading == 0 ? 1 : sensdataset->sensors[OBK_POWER].lastReading / sensdataset->sensors[OBK_POWER_APPARENT].lastReading);
			}
		}
		// TODO: if 3phase-meter provides only phase power????
		double energyWAp = NAN;
		double energyWRp = NAN;
		if (!isnan(power_active)) {
			if (!(blv2_energy_meter.measuredsensors&msConsumptionTotalAp)) { // Active Energy Import(Plus) is not provided by meter, calculate it
				xPassedTicks = (int)(nowTickCount - blv2_energy_meter.energy_meter_data[meter].energyCounterStamp);
				// FIXME: Wrong calculation if tick count overflows
				if (xPassedTicks <= 0) xPassedTicks = 1;
				energyWAp = xPassedTicks * power_active / (3600000.0f / portTICK_PERIOD_MS);
			}
		}
/*
		if (!isnan(power_reactive)) {
			if (!(blv2_energy_meter.measuredsensors&msConsumptionTotalRp)) { // Reactive Energy Import(Plus) is not provided by meter, calculate it
				xPassedTicks = (int)(nowTickCount - blv2_energy_meter.energy_meter_data[meter].energyCounterStamp);
				// FIXME: Wrong calculation if tick count overflows
				if (xPassedTicks <= 0) xPassedTicks = 1;
				energyWRp = xPassedTicks * power_reactive / (3600000.0f / portTICK_PERIOD_MS);
			}
		}
*/
		if (!isnan(energyWAp) || !isnan(energyWRp)) {
			BLv2_ProcessUpdateEnergy(meter, energyWAp, energyWRp, NAN, NAN, 0);
		}
	}
	blv2_energy_meter.energy_meter_data[meter].sensors_reciveddata = 1;

	for (int i = OBK__FIRST; i <= OBK__LAST; i++) {
#if ENABLE_BL_TWIN
		//in twin mode, for ix1 is possible to skip OBK_VOLTAGE, dont skip for now
		//if ((meter > 0) && (i== OBK_VOLTAGE)) continue;
		//in twin mode, for ix0 is last OBK_CONSUMPTION_YESTERDAY, for ix1 ,OBK_CONSUMPTION_TODAY
		if ((i > OBK_CONSUMPTION_STORED_LAST[meter]) && (i <= OBK_CONSUMPTION__DAILY_LAST)) continue;
#endif
		if (BLv2_HasEnergySensorReading(meter, i) == false) continue;
		// send update only if there was a big change or if certain time has passed
		// Do not send message with every measurement.
		diff = (float)sensdataset->sensors[i].lastSentValue - (float)sensdataset->sensors[i].lastReading;
		// check for change
		if (((fabsf(diff) > sensdataset->sensors[i].changeSendThreshold) &&
			(sensdataset->sensors[i].noChangeFrame >= blv2_energy_meter.changeDoNotSendMinFrames)) ||
			(sensdataset->sensors[i].noChangeFrame >= blv2_energy_meter.changeSendAlwaysFrames)) {
			sensdataset->sensors[i].noChangeFrame = 0;

			enum EventCode eventChangeCode;
			switch (i) {
				//case OBK_VOLTAGE_L1: - same as OBK_VOLTAGE
				case OBK_VOLTAGE_L2:
				case OBK_VOLTAGE_L3:
				case OBK_VOLTAGE:				eventChangeCode = CMD_EVENT_CHANGE_VOLTAGE;	break;
				//case OBK_CURRENT_L1: - same as OBK_CURRENT
				case OBK_CURRENT_L2:
				case OBK_CURRENT_L3:
				case OBK_CURRENT:				eventChangeCode = CMD_EVENT_CHANGE_CURRENT;	break;
				case OBK_POWER_L1:
				case OBK_POWER_L2:
				case OBK_POWER_L3:
				case OBK_POWER:					eventChangeCode = CMD_EVENT_CHANGE_POWER; break;
				case OBK_FREQUENCY:			    eventChangeCode = CMD_EVENT_CHANGE_FREQUENCY; break;
				case OBK_CONSUMPTION_WAImp_TOTAL:
				case OBK_CONSUMPTION_WAExp_TOTAL:
												eventChangeCode = CMD_EVENT_CHANGE_CONSUMPTION_TOTAL; break;
				case OBK_CONSUMPTION_LAST_HOUR:	eventChangeCode = CMD_EVENT_CHANGE_CONSUMPTION_LAST_HOUR; break;
				default:						eventChangeCode = CMD_EVENT_NONE; break;
			}
			switch (eventChangeCode) {
				case CMD_EVENT_NONE:
					break;
				case CMD_EVENT_CHANGE_FREQUENCY:;
		// the event change comparisons are stored as int types, so frequency compared as *100
		// i.e. 50.10 becomes 5010
					int prev_hz = (int)(sensdataset->sensors[i].lastSentValue * 100);
					int now_hz = (int)(sensdataset->sensors[i].lastReading * 100);
					EventHandlers_ProcessVariableChange_Integer(eventChangeCode, prev_hz, now_hz);
					break;
				case CMD_EVENT_CHANGE_CURRENT:;
					int prev_mA = (int)(sensdataset->sensors[i].lastSentValue * 1000);
					int now_mA = (int)(sensdataset->sensors[i].lastReading * 1000);
					EventHandlers_ProcessVariableChange_Integer(eventChangeCode, prev_mA, now_mA);
					break;
				default:
					EventHandlers_ProcessVariableChange_Integer(eventChangeCode, (int)sensdataset->sensors[i].lastSentValue, (int)sensdataset->sensors[i].lastReading);
					break;
			}
			sensdataset->sensors[i].lastSentValue = sensdataset->sensors[i].lastReading;
#if ENABLE_MQTT
			if (MQTT_IsReady() == true)	{
				if (i == OBK_CONSUMPTION_CLEAR_DATE) {
					{
					sensdataset->sensors[i].lastReading = blv2_energy_meter.ConsumptionResetTime; //Only to make the 'nochangeframe' mechanism work here
#if PLATFORM_ECR6600
					// for ECR6000 printf("%+03i", 1); will not be "+01" but "0+1" ...
					int ttm = TIME_GetTimesZoneOfsSeconds()/60;	// used to get +/- sign and minutes
					snprintf(datetime, sizeof(datetime), "%.16s%c%02i:%02i",TS2STR(blv2_energy_meter.ConsumptionResetTime, TIME_FORMAT_ISO_8601),
						ttm < 0 ? '-': '+', abs(ttm/60), abs(ttm) % 60);
#else
					snprintf(datetime, sizeof(datetime), "%.16s%+03i:%02i",TS2STR(blv2_energy_meter.ConsumptionResetTime, TIME_FORMAT_ISO_8601),
						TIME_GetTimesZoneOfsSeconds()/3600, (abs(TIME_GetTimesZoneOfsSeconds())/60) % 60);
#endif

					MQTT_PublishMain_StringString(sensdataset->sensors[i].names.name_mqtt, datetime, 0);
					}
				} else { //all other sensors
					float val = (float)sensdataset->sensors[i].lastReading;
					if (sensdataset->sensors[i].names.units == UNIT_WAH) val = BL_ChangeEnergyUnitIfNeeded(val);
					MQTT_PublishMain_StringFloat(sensdataset->sensors[i].names.name_mqtt, val, sensdataset->sensors[i].rounding_decimals, OBK_PUBLISH_FLAG_QOS_ZERO);
				}
				blv2_energy_meter.energy_meter_data[meter].stat_updatesSent++;
			}
#endif
		} else {
			// no change frame
			sensdataset->sensors[i].noChangeFrame++;
			blv2_energy_meter.energy_meter_data[meter].stat_updatesSkipped++;
		}
	}

}

#define MAKE_SENSOR_NAME(name, pattern, var) do { 					\
    size_t needed = snprintf(NULL, 0, pattern, var) + 1;			\
	name = os_malloc(needed);						                \
    if (name) {                                                     \
		sprintf((char*)name, pattern, var);	     				    \
		name[needed] = 0;                                           \
	}                                                               \
    } while (0)

/**
 * @brief Init energy meters
 */
void BLv2_Shared_Init(int meter_count, energySensorMode_t mode, int MeasuredSensors) {
	if (blv2_energy_meter_init) {
		// error - already init
		return;
	}
	if ((meter_count < 1) & (meter_count > BLV2_EMETERS_MAX)) return; // now only one or two sensors (when ENABLE_BL_TWIN) support
	blv2_energy_meter.meter_count = meter_count;
	blv2_energy_meter.meter_mode  = mode;
	blv2_energy_meter.measuredsensors = MeasuredSensors;

	blv2_energy_meter.outputsensors = 0;
	if (blv2_energy_meter.measuredsensors&msVoltage) {
		if (blv2_energy_meter.meter_mode == es_1PhaseMode)
			blv2_energy_meter.outputsensors |= (0x01 << OBK_VOLTAGE);
		else
			blv2_energy_meter.outputsensors |= (0x01 << OBK_VOLTAGE_L1)|(0x01 << OBK_VOLTAGE_L2)|(0x01 << OBK_VOLTAGE_L3);
	}
	if (blv2_energy_meter.measuredsensors&msCurrent) {
		if (blv2_energy_meter.meter_mode == es_1PhaseMode)
			blv2_energy_meter.outputsensors |= (0x01 << OBK_CURRENT);
		else
			blv2_energy_meter.outputsensors |= (0x01 << OBK_CURRENT_L1)|(0x01 << OBK_CURRENT_L2)|(0x01 << OBK_CURRENT_L3);
	}
	if (blv2_energy_meter.measuredsensors&msPowerActiveSumm) {
		blv2_energy_meter.outputsensors |= (0x01 << OBK_POWER);
	}
	if (blv2_energy_meter.measuredsensors&msPowerActivePhase) {
		blv2_energy_meter.outputsensors |= (0x01 << OBK_POWER_L1)|(0x01 << OBK_POWER_L2)|(0x01 << OBK_POWER_L3);
	}
	if (blv2_energy_meter.measuredsensors&msPowerApparentSumm || ((blv2_energy_meter.measuredsensors&msCurrent) && (blv2_energy_meter.measuredsensors&msVoltage))) {
		blv2_energy_meter.outputsensors |= (0x01 << OBK_POWER_APPARENT);
	}
	if (blv2_energy_meter.measuredsensors&msPowerReactiveSumm || (blv2_energy_meter.outputsensors&((0x01 << OBK_POWER)|(0x01 << OBK_POWER_APPARENT)))) {
		blv2_energy_meter.outputsensors |= (0x01 << OBK_POWER_REACTIVE);
	}
	if (blv2_energy_meter.measuredsensors&msPowerFactorSumm || (blv2_energy_meter.outputsensors&((0x01 << OBK_POWER)|(0x01 << OBK_POWER_APPARENT)))) {
		blv2_energy_meter.outputsensors |= (0x01 << OBK_POWER_FACTOR);
	}
	if (blv2_energy_meter.measuredsensors&msFrequency || blv2_energy_meter.measuredsensors&msVoltage) {
		blv2_energy_meter.outputsensors |= (0x01 << OBK_FREQUENCY);
	}
	if (blv2_energy_meter.measuredsensors&msConsumptionTotalAp || blv2_energy_meter.measuredsensors&msPowerActiveSumm) {
		blv2_energy_meter.outputsensors |= (0x01 << OBK_CONSUMPTION_WAImp_TOTAL);
#if ENABLED_TARIFED_ENERGY
#if TARIFED_TIMEZONE_MAX > 1
		blv2_energy_meter.outputsensors |= (0x01 << OBK_CONSUMPTION_WAImp_T1)|(0x01 << OBK_CONSUMPTION_WAImp_T2);
#endif
#if TARIFED_TIMEZONE_MAX > 2
		blv2_energy_meter.outputsensors |= (0x01 << OBK_CONSUMPTION_WAImp_T3);
#endif
#if TARIFED_TIMEZONE_MAX > 3
		blv2_energy_meter.outputsensors |= (0x01 << OBK_CONSUMPTION_WAImp_T4);
#endif
#endif 	//ENABLED_TARIFED_ENERGY
		blv2_energy_meter.outputsensors |= (0x01<<OBK_CONSUMPTION_TODAY)|(0x01<<OBK_CONSUMPTION_YESTERDAY)|(0x01<<OBK_CONSUMPTION_2_DAYS_AGO)|(0x01<<OBK_CONSUMPTION_3_DAYS_AGO);
	}
	if (blv2_energy_meter.measuredsensors&msConsumptionTotalAm /*|| blv2_energy_meter.measuredsensors&msPowerActiveSumm*/) { // TODO: Exported energy can be calculated,if active power have a sign
		blv2_energy_meter.outputsensors |= (0x01 << OBK_CONSUMPTION_WAExp_TOTAL);
#if ENABLED_TARIFED_ENERGY
#if TARIFED_TIMEZONE_MAX > 1
		blv2_energy_meter.outputsensors |= (0x01 << OBK_CONSUMPTION_WAExp_T1)|(0x01 << OBK_CONSUMPTION_WAExp_T2);
#endif
#if TARIFED_TIMEZONE_MAX > 2
		blv2_energy_meter.outputsensors |= (0x01 << OBK_CONSUMPTION_WAExp_T3);
#endif
#if TARIFED_TIMEZONE_MAX > 3
		blv2_energy_meter.outputsensors |= (0x01 << OBK_CONSUMPTION_WAExp_T4);
#endif
#endif //ENABLED_TARIFED_ENERGY
	}

	blv2_energy_meter.energyCounterStatsEnable = false;
	blv2_energy_meter.energyCounterSampleCount = 60;
	blv2_energy_meter.energyCounterSampleInterval = 60;
	blv2_energy_meter.energyCounterMinutes = NULL;
	blv2_energy_meter.energyCounterStatsJSONEnable = false;
	blv2_energy_meter.changeSavedThresholdEnergy = 1000.0f;
	blv2_energy_meter.ConsumptionSaveCounter = 0;
	blv2_energy_meter.ConsumptionResetTime = 0;
	blv2_energy_meter.changeSendAlwaysFrames = 60;
	blv2_energy_meter.changeDoNotSendMinFrames = 5;

	/* create datasetlist */
	blv2_energy_meter.datasetlist = (energysensdataset_t*)os_malloc(blv2_energy_meter.meter_count*sizeof(energysensdataset_t));
	memset((void*)blv2_energy_meter.datasetlist,0,blv2_energy_meter.meter_count*sizeof(energysensdataset_t));
	blv2_energy_meter.energy_meter_data = (blv2_energy_meter_data_t*)os_malloc(blv2_energy_meter.meter_count*sizeof(blv2_energy_meter_data_t));
	memset((void*)blv2_energy_meter.energy_meter_data,0,blv2_energy_meter.meter_count*sizeof(blv2_energy_meter_data_t));

	/* init dataset */
	const char *name_suffix = "";
	const char *mqtt_suffix = "";
    const char *uid_suffix  = "";
	for (int meter_i=0; meter_i < meter_count; meter_i++) {
		if (meter_count > 1) {
			if (meter_i == 0) {
				name_suffix = " A";
				mqtt_suffix = "_a";
				uid_suffix  = "a_";
			}
			if (meter_i == 1) {
				name_suffix = " B";
				mqtt_suffix = "_b";
				uid_suffix  = "b_";
			}
		}
		blv2_energy_meter.energy_meter_data[meter_i].actual_mday = -1;
		energysensdataset_t* sensdataset = &blv2_energy_meter.datasetlist[meter_i];
		if (blv2_energy_meter.meter_mode == es_1PhaseMode) {
			sensdataset->sensors[OBK_VOLTAGE].names.hass_dev_class = hass_dev_class_names[hass_dev_class_voltage];
			sensdataset->sensors[OBK_VOLTAGE].names.units          =          units_names[          unit_voltage];
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE].names.name_friendly, "Voltage%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE].names.name_mqtt, "voltage%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE].names.hass_uniq_id_suffix, "%s0", uid_suffix);
			sensdataset->sensors[OBK_VOLTAGE].rounding_decimals = 1;
			sensdataset->sensors[OBK_VOLTAGE].changeSendThreshold = 0.25;

			sensdataset->sensors[OBK_CURRENT].names.hass_dev_class = hass_dev_class_names[hass_dev_class_current];
			sensdataset->sensors[OBK_CURRENT].names.units          =          units_names[          unit_current];
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT].names.name_friendly, "Current%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT].names.name_mqtt, "current%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT].names.hass_uniq_id_suffix, "%s1", uid_suffix);
			sensdataset->sensors[OBK_CURRENT].rounding_decimals = 3;
			sensdataset->sensors[OBK_CURRENT].changeSendThreshold = 0.002;
		} else {
			sensdataset->sensors[OBK_VOLTAGE_L1].names.hass_dev_class = hass_dev_class_names[hass_dev_class_voltage];
			sensdataset->sensors[OBK_VOLTAGE_L2].names.hass_dev_class = hass_dev_class_names[hass_dev_class_voltage];
			sensdataset->sensors[OBK_VOLTAGE_L3].names.hass_dev_class = hass_dev_class_names[hass_dev_class_voltage];
			sensdataset->sensors[OBK_VOLTAGE_L1].names.units          =          units_names[          unit_voltage];
			sensdataset->sensors[OBK_VOLTAGE_L2].names.units          =          units_names[          unit_voltage];
			sensdataset->sensors[OBK_VOLTAGE_L3].names.units          =          units_names[          unit_voltage];
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE_L1].names.name_friendly, "Voltage L1%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE_L2].names.name_friendly, "Voltage L2%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE_L3].names.name_friendly, "Voltage L3%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE_L1].names.name_mqtt, "voltage_l1%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE_L2].names.name_mqtt, "voltage_l2%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE_L3].names.name_mqtt, "voltage_l3%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE_L1].names.hass_uniq_id_suffix, "%s21", uid_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE_L2].names.hass_uniq_id_suffix, "%s22", uid_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_VOLTAGE_L3].names.hass_uniq_id_suffix, "%s23", uid_suffix);
			sensdataset->sensors[OBK_VOLTAGE_L1].rounding_decimals = 1;
			sensdataset->sensors[OBK_VOLTAGE_L1].changeSendThreshold = 0.25;
			sensdataset->sensors[OBK_VOLTAGE_L2].rounding_decimals = 1;
			sensdataset->sensors[OBK_VOLTAGE_L2].changeSendThreshold = 0.25;
			sensdataset->sensors[OBK_VOLTAGE_L3].rounding_decimals = 1;
			sensdataset->sensors[OBK_VOLTAGE_L3].changeSendThreshold = 0.25;

			sensdataset->sensors[OBK_CURRENT_L1].names.hass_dev_class = hass_dev_class_names[hass_dev_class_current];
			sensdataset->sensors[OBK_CURRENT_L2].names.hass_dev_class = hass_dev_class_names[hass_dev_class_current];
			sensdataset->sensors[OBK_CURRENT_L3].names.hass_dev_class = hass_dev_class_names[hass_dev_class_current];
			sensdataset->sensors[OBK_CURRENT_L1].names.units          =          units_names[          unit_current];
			sensdataset->sensors[OBK_CURRENT_L2].names.units          =          units_names[          unit_current];
			sensdataset->sensors[OBK_CURRENT_L3].names.units          =          units_names[          unit_current];
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT_L1].names.name_friendly, "Current L1%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT_L2].names.name_friendly, "Current L2%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT_L3].names.name_friendly, "Current L3%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT_L1].names.name_mqtt, "current_l1%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT_L2].names.name_mqtt, "current_l2%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT_L3].names.name_mqtt, "current_l3%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT_L1].names.hass_uniq_id_suffix, "%s24", uid_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT_L2].names.hass_uniq_id_suffix, "%s25", uid_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CURRENT_L3].names.hass_uniq_id_suffix, "%s26", uid_suffix);
			sensdataset->sensors[OBK_CURRENT_L1].rounding_decimals = 3;
			sensdataset->sensors[OBK_CURRENT_L1].changeSendThreshold = 0.002;
			sensdataset->sensors[OBK_CURRENT_L2].rounding_decimals = 3;
			sensdataset->sensors[OBK_CURRENT_L2].changeSendThreshold = 0.002;
			sensdataset->sensors[OBK_CURRENT_L3].rounding_decimals = 3;
			sensdataset->sensors[OBK_CURRENT_L3].changeSendThreshold = 0.002;

			sensdataset->sensors[OBK_POWER_L1].names.hass_dev_class = hass_dev_class_names[hass_dev_class_power];
			sensdataset->sensors[OBK_POWER_L2].names.hass_dev_class = hass_dev_class_names[hass_dev_class_power];
			sensdataset->sensors[OBK_POWER_L3].names.hass_dev_class = hass_dev_class_names[hass_dev_class_power];
			sensdataset->sensors[OBK_POWER_L1].names.units          =          units_names[          unit_power];
			sensdataset->sensors[OBK_POWER_L2].names.units          =          units_names[          unit_power];
			sensdataset->sensors[OBK_POWER_L3].names.units          =          units_names[          unit_power];
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_L1].names.name_friendly, "Power L1%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_L2].names.name_friendly, "Power L2%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_L3].names.name_friendly, "Power L3%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_L1].names.name_mqtt, "power_l1%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_L2].names.name_mqtt, "power_l2%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_L3].names.name_mqtt, "power_l3%s", mqtt_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_L1].names.hass_uniq_id_suffix, "%s27", uid_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_L2].names.hass_uniq_id_suffix, "%s28", uid_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_L3].names.hass_uniq_id_suffix, "%s29", uid_suffix);
			sensdataset->sensors[OBK_POWER_L1].rounding_decimals = 2;
			sensdataset->sensors[OBK_POWER_L1].changeSendThreshold = 0.25;
			sensdataset->sensors[OBK_POWER_L2].rounding_decimals = 2;
			sensdataset->sensors[OBK_POWER_L2].changeSendThreshold = 0.25;
			sensdataset->sensors[OBK_POWER_L3].rounding_decimals = 2;
			sensdataset->sensors[OBK_POWER_L3].changeSendThreshold = 0.25;
		}
		sensdataset->sensors[OBK_POWER].names.hass_dev_class = hass_dev_class_names[hass_dev_class_power];
		sensdataset->sensors[OBK_POWER].names.units          =          units_names[          unit_power];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER].names.name_friendly, "Power%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER].names.name_mqtt, "power%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER].names.hass_uniq_id_suffix, "%s2", uid_suffix);
		sensdataset->sensors[OBK_POWER].rounding_decimals = 2;
		sensdataset->sensors[OBK_POWER].changeSendThreshold = 0.25;

		sensdataset->sensors[OBK_FREQUENCY].names.hass_dev_class = hass_dev_class_names[hass_dev_class_frequency];
		sensdataset->sensors[OBK_FREQUENCY].names.units          =          units_names[          unit_frequency];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_FREQUENCY].names.name_friendly, "Frequency%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_FREQUENCY].names.name_mqtt, "138%s", mqtt_suffix); //// OBK_FREQUENCY - SPECIAL_CHANNEL_OBK_FREQUENCY is 138
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_FREQUENCY].names.hass_uniq_id_suffix, "%s138", uid_suffix);
		sensdataset->sensors[OBK_FREQUENCY].rounding_decimals = 2;
		sensdataset->sensors[OBK_FREQUENCY].changeSendThreshold = 0.02;
		sensdataset->sensors[OBK_FREQUENCY].lastReading = NAN;

		sensdataset->sensors[OBK_POWER_APPARENT].names.hass_dev_class = hass_dev_class_names[hass_dev_class_apparent_power];
		sensdataset->sensors[OBK_POWER_APPARENT].names.units          =          units_names[          unit_apparent_power];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_APPARENT].names.name_friendly, "Apparent Power%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_APPARENT].names.name_mqtt, "apparent_power%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_APPARENT].names.hass_uniq_id_suffix, "%s9", uid_suffix);
		sensdataset->sensors[OBK_POWER_APPARENT].rounding_decimals = 2;
		sensdataset->sensors[OBK_POWER_APPARENT].changeSendThreshold = 0.25;

		sensdataset->sensors[OBK_POWER_REACTIVE].names.hass_dev_class = hass_dev_class_names[hass_dev_class_reactive_power];
		sensdataset->sensors[OBK_POWER_REACTIVE].names.units          =          units_names[          unit_reactive_power];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_REACTIVE].names.name_friendly, "Reactive Power%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_REACTIVE].names.name_mqtt, "reactive_power%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_REACTIVE].names.hass_uniq_id_suffix, "%s10", uid_suffix);
		sensdataset->sensors[OBK_POWER_REACTIVE].rounding_decimals = 2;
		sensdataset->sensors[OBK_POWER_REACTIVE].changeSendThreshold = 0.25;

		sensdataset->sensors[OBK_POWER_FACTOR].names.hass_dev_class = hass_dev_class_names[hass_dev_class_power_factor];
		sensdataset->sensors[OBK_POWER_FACTOR].names.units          =          units_names[          unit_power_factor];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_FACTOR].names.name_friendly, "Power Factor%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_FACTOR].names.name_mqtt, "power_factor%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_POWER_FACTOR].names.hass_uniq_id_suffix, "%s11", uid_suffix);
		sensdataset->sensors[OBK_POWER_FACTOR].rounding_decimals = 2;
		sensdataset->sensors[OBK_POWER_FACTOR].changeSendThreshold = 0.05;

		sensdataset->sensors[OBK_CONSUMPTION_TOTAL].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_TOTAL].names.units          =          units_names[          unit_energy_wa];
		if (blv2_energy_meter.outputsensors&(0x01 << OBK_CONSUMPTION_WAExp_TOTAL)) {
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_TOTAL].names.name_friendly, "Energy+ Total%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_TOTAL].names.name_mqtt, "energy_imp_total%s", mqtt_suffix);
		} else {
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_TOTAL].names.name_friendly, "Energy Total%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_TOTAL].names.name_mqtt, "energy_total%s", mqtt_suffix);
		}
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_TOTAL].names.hass_uniq_id_suffix, "%s3", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_TOTAL].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_TOTAL].changeSendThreshold = 0.1;

		sensdataset->sensors[OBK_CONSUMPTION_WAExp_TOTAL].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_TOTAL].names.units          =          units_names[          unit_energy_wa];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_TOTAL].names.name_friendly, "Energy- Total%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_TOTAL].names.name_mqtt, "energy_exp_total%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_TOTAL].names.hass_uniq_id_suffix, "%s33", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_TOTAL].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_TOTAL].changeSendThreshold = 0.1;


		sensdataset->sensors[OBK_CONSUMPTION_LAST_HOUR].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_LAST_HOUR].names.units          =          units_names[          unit_energy_wa];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_LAST_HOUR].names.name_friendly, "Energy Last Hour%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_LAST_HOUR].names.name_mqtt, "energy_last_hour%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_LAST_HOUR].names.hass_uniq_id_suffix, "%s4", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_LAST_HOUR].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_LAST_HOUR].changeSendThreshold = 0.1;

		sensdataset->sensors[OBK_CONSUMPTION_TODAY].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_TODAY].names.units          =          units_names[          unit_energy_wa];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_TODAY].names.name_friendly, "Energy Today%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_TODAY].names.name_mqtt, "energy_today%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_TODAY].names.hass_uniq_id_suffix, "%s7", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_TODAY].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_TODAY].changeSendThreshold = 0.1;

		sensdataset->sensors[OBK_CONSUMPTION_YESTERDAY].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_YESTERDAY].names.units          =          units_names[          unit_energy_wa];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_YESTERDAY].names.name_friendly, "Energy Yesterday%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_YESTERDAY].names.name_mqtt, "energy_yesterday%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_YESTERDAY].names.hass_uniq_id_suffix, "%s6", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_YESTERDAY].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_YESTERDAY].changeSendThreshold = 0.1;

		sensdataset->sensors[OBK_CONSUMPTION_2_DAYS_AGO].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_2_DAYS_AGO].names.units          =          units_names[          unit_energy_wa];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_2_DAYS_AGO].names.name_friendly, "Energy 2 Days Ago%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_2_DAYS_AGO].names.name_mqtt, "energy_2_days_ago%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_2_DAYS_AGO].names.hass_uniq_id_suffix, "%s13", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_2_DAYS_AGO].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_2_DAYS_AGO].changeSendThreshold = 0.1;

		sensdataset->sensors[OBK_CONSUMPTION_3_DAYS_AGO].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_3_DAYS_AGO].names.units          =          units_names[          unit_energy_wa];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_3_DAYS_AGO].names.name_friendly, "Energy 3 Days Ago%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_3_DAYS_AGO].names.name_mqtt, "energy_3_days_ago%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_3_DAYS_AGO].names.hass_uniq_id_suffix, "%s14", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_3_DAYS_AGO].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_3_DAYS_AGO].changeSendThreshold = 0.1;

#if ENABLED_TARIFED_ENERGY
#if TARIFED_TIMEZONE_MAX > 1
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1].names.units          =          units_names[          unit_energy_wa];
		if (blv2_energy_meter.outputsensors&(0x01 << OBK_CONSUMPTION_WAExp_TOTAL)) {
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1].names.name_friendly, "Energy+ Tarif 1%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1].names.name_mqtt, "energy_imp_tarif_1%s", mqtt_suffix);
		} else {
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1].names.name_friendly, "Energy Tarif 1%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1].names.name_mqtt, "energy_tarif_1%s", mqtt_suffix);
		}

		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1].names.hass_uniq_id_suffix, "%s15", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1].changeSendThreshold = 0.1;

		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1].names.units          =          units_names[          unit_energy_wa];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1].names.name_friendly, "Energy- Tarif 1%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1].names.name_mqtt, "energy_exp_tarif_1%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1].names.hass_uniq_id_suffix, "%s35", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1].changeSendThreshold = 0.1;

		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T2].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T2].names.units          =          units_names[          unit_energy_wa];
		if (blv2_energy_meter.outputsensors&(0x01 << OBK_CONSUMPTION_WAExp_TOTAL)) {
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T2].names.name_friendly, "Energy+ Tarif 2%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T2].names.name_mqtt, "energy_imp_tarif_2%s", mqtt_suffix);
		} else {
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T2].names.name_friendly, "Energy Tarif 2%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T2].names.name_mqtt, "energy_tarif_2%s", mqtt_suffix);
		}
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T2].names.hass_uniq_id_suffix, "%s16", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T2].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T2].changeSendThreshold = 0.1;

		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T2].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T2].names.units          =          units_names[          unit_energy_wa];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T2].names.name_friendly, "Energy- Tarif 2%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T2].names.name_mqtt, "energy_exp_tarif_2%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T2].names.hass_uniq_id_suffix, "%s36", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T2].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T2].changeSendThreshold = 0.1;
#endif
#if TARIFED_TIMEZONE_MAX > 2
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T3].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T3].names.units          =          units_names[          unit_energy_wa];
		if (blv2_energy_meter.outputsensors&(0x01 << OBK_CONSUMPTION_WAExp_TOTAL)) {
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T3].names.name_friendly, "Energy+ Tarif 3%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T3].names.name_mqtt, "energy_imp_tarif_3%s", mqtt_suffix);
		} else {
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T3].names.name_friendly, "Energy Tarif 3%s", name_suffix);
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T3].names.name_mqtt, "energy_tarif_3%s", mqtt_suffix);
		}
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T3].names.hass_uniq_id_suffix, "%s17", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T3].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T3].changeSendThreshold = 0.1;

		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T3].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T3].names.units          =          units_names[          unit_energy_wa];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T3].names.name_friendly, "Energy- Tarif 3%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T3].names.name_mqtt, "energy_exp_tarif_3%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T3].names.hass_uniq_id_suffix, "%s37", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T3].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T3].changeSendThreshold = 0.1;
#endif
#if TARIFED_TIMEZONE_MAX > 3
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T4].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T4].names.units          =          units_names[          unit_energy_wa];
		if (blv2_energy_meter.outputsensors&(0x01 << OBK_CONSUMPTION_WAExp_TOTAL)) {
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T4].names.name_friendly, "Energy+ Tarif 4%s", name_suffix);
		} else {
			MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T4].names.name_friendly, "Energy Tarif 4%s", name_suffix);
		}
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T4].names.name_mqtt, "energy_imp_tarif_4%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAImp_T4].names.hass_uniq_id_suffix, "%s18", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T4].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_WAImp_T4].changeSendThreshold = 0.1;

		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T4].names.hass_dev_class = hass_dev_class_names[hass_dev_class_energy];
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T4].names.units          =          units_names[          unit_energy_wa];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T4].names.name_friendly, "Energy- Tarif 4%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T4].names.name_mqtt, "energy_exp_tarif_4%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_WAExp_T4].names.hass_uniq_id_suffix, "%s38", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T4].rounding_decimals = 3;
		sensdataset->sensors[OBK_CONSUMPTION_WAExp_T4].changeSendThreshold = 0.1;
#endif
#endif //ENABLED_TARIFED_ENERGY
		sensdataset->sensors[OBK_CONSUMPTION_CLEAR_DATE].names.hass_dev_class = hass_dev_class_names[hass_dev_class_timestamp];
		sensdataset->sensors[OBK_CONSUMPTION_CLEAR_DATE].names.units          = units_names[                   unit_timestamp];
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_CLEAR_DATE].names.name_friendly, "Energy Clear Date%s", name_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_CLEAR_DATE].names.name_mqtt, "energy_clear_date%s", mqtt_suffix);
		MAKE_SENSOR_NAME(sensdataset->sensors[OBK_CONSUMPTION_CLEAR_DATE].names.hass_uniq_id_suffix, "%s8", uid_suffix);
		sensdataset->sensors[OBK_CONSUMPTION_CLEAR_DATE].rounding_decimals = 0;
		sensdataset->sensors[OBK_CONSUMPTION_CLEAR_DATE].changeSendThreshold = 86400;

		for(int i = OBK__FIRST; i <= OBK__LAST; i++) {
			sensdataset->sensors[i].noChangeFrame = 0;
			sensdataset->sensors[i].lastReading = 0;
		}

	}
	/* ?????????????
	if (blv2_energy_meter.energyCounterStatsEnable == true) {
		if (blv2_energy_meter.energyCounterMinutes == NULL){
		  blv2_energy_meter.energyCounterMinutes = (float*)os_malloc(blv2_energy_meter.energyCounterSampleCount*sizeof(float));
		}
		if (blv2_energy_meter.energyCounterMinutes != NULL) {
			for(i = 0; i < blv2_energy_meter.energyCounterSampleCount; i++) {
				blv2_energy_meter.energyCounterMinutes[i] = 0.0;
			}
		}
		blv2_energy_meter.energyCounterMinutesStamp = xTaskGetTickCount();
		blv2_energy_meter.energyCounterMinutesIndex = 0;
	}
	*/
    ENERGY_METERING_DATA data;
	{
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Read ENERGYMETER values sz=%d", sizeof(ENERGY_METERING_DATA));

		HAL_GetEnergyMeterStatus(&data);

		for (int meter_i=0; meter_i < meter_count; meter_i++) {
			energysensdataset_t* sensdataset = &blv2_energy_meter.datasetlist[meter_i];
			blv2_energy_meter.energy_meter_data[meter_i].actual_mday = data.actual_mday;              //one in flashvars is enough, I assume that both channels are synchronized
			portTickType nowTickCount = xTaskGetTickCount();
			switch (meter_i) {
				case BL_SENSORS_IX_0: {
					sensdataset->sensors[OBK_CONSUMPTION_TOTAL].lastReading = data.TotalConsumption;
					sensdataset->sensors[OBK_CONSUMPTION_TODAY].lastReading = data.TodayConsumpion;
					sensdataset->sensors[OBK_CONSUMPTION_YESTERDAY].lastReading = data.YesterdayConsumption;
					sensdataset->sensors[OBK_CONSUMPTION_2_DAYS_AGO].lastReading = data.ConsumptionHistory[0];
					sensdataset->sensors[OBK_CONSUMPTION_3_DAYS_AGO].lastReading = data.ConsumptionHistory[1];
					blv2_energy_meter.energy_meter_data[BL_SENSORS_IX_0].lastSavedEnergyImpCounterValue = data.TotalConsumption;
					blv2_energy_meter.energy_meter_data[BL_SENSORS_IX_0].energyCounterStamp = nowTickCount;
					break;
				};
				case BL_SENSORS_IX_1: {
#if ENABLE_BL_TWIN
					sensdataset->sensors[OBK_CONSUMPTION_TOTAL].lastReading = data.TotalConsumption_b;
					sensdataset->sensors[OBK_CONSUMPTION_TODAY].lastReading = data.TodayConsumpion_b;
					blv2_energy_meter.energy_meter_data[BL_SENSORS_IX_1].lastSavedEnergyImpCounterValue = data.TotalConsumption_b;
#else
					sensdataset->sensors[OBK_CONSUMPTION_TOTAL].lastReading = data.ConsumptionHistory[0];
					sensdataset->sensors[OBK_CONSUMPTION_TODAY].lastReading = data.ConsumptionHistory[1];
					blv2_energy_meter.energy_meter_data[BL_SENSORS_IX_1].lastSavedEnergyImpCounterValue = data.ConsumptionHistory[0];
#endif
					blv2_energy_meter.energy_meter_data[BL_SENSORS_IX_1].energyCounterStamp = nowTickCount;
					break;
				}
			}
			blv2_energy_meter.ConsumptionResetTime     = data.ConsumptionResetTime;
			blv2_energy_meter.ConsumptionSaveCounter   = data.save_counter;
			blv2_energy_meter.lastConsumptionSaveStamp = nowTickCount;
		}
		//int HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA *data);
    }
#if ENABLED_TARIFED_ENERGY || ENABLED_MONTH_ENERGY
	ENERGY_METERING_DATA_V2 energy_data_new;
	{
		for (int meter_i=0; meter_i < meter_count; meter_i++) {
			energysensdataset_t* sensdataset = &blv2_energy_meter.datasetlist[meter_i];
			addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Read ENERGYMETER tarif and month values sz=%d, meter=%d", sizeof(ENERGY_METERING_DATA_V2), meter_i);
			HAL_FlashVars_GetEnergy_BLv2(&energy_data_new, meter_i);
#if ENABLED_TARIFED_ENERGY
			for (int tarif_i = 0; tarif_i < TARIFED_TIMEZONE_MAX; tarif_i++) {
				// copy stored values
				sensdataset->sensors[OBK_CONSUMPTION_WAImp_T1+tarif_i].lastReading = energy_data_new.energy_total.WA_Import[tarif_i];
				sensdataset->sensors[OBK_CONSUMPTION_WAExp_T1+tarif_i].lastReading = energy_data_new.energy_total.WA_Export[tarif_i];
				// copy time zones
				blv2_energy_meter.energy_meter_data[meter_i].tarif_tz[tarif_i] = energy_data_new.timezones[tarif_i];
			}
			// check time zones
			if (BLv2_CheckTarifTimezone(meter_i) == true) {
				blv2_energy_meter.energy_meter_data[meter_i].tarif_tz_ok = 1;
			}
#endif
#if ENABLED_MONTH_ENERGY
			blv2_energy_meter.energy_meter_data[meter_i].actual_month = energy_data_new.actual_month;
			memmove((void*)&blv2_energy_meter.energy_meter_data[meter_i].month_energy, (void*)&energy_data_new.month_energy, 12*sizeof(ENERGY_DATA_V2));
#endif
		}
	}
#endif //ENABLED_TARIFED_ENERGY

	//cmddetail:{"name":"EnergyCntReset","args":"[OptionalNewValue][sensorix]",
	//cmddetail:"descr":"Resets the total Energy Counter, the one that is usually kept after device reboots. After this commands, the counter will start again from 0 (or from the value you specified). sensorix is used in ENABLE_BL_TWIN",
	//cmddetail:"fn":"BLv2_ResetEnergyCounter","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("EnergyCntReset", BLv2_ResetEnergyCounter, NULL);
	//cmddetail:{"name":"SetupEnergyStats","args":"[Enable1or0][SampleTime][SampleCount][JSonEnable]",
	//cmddetail:"descr":"Setup Energy Statistic Parameters: [enable 0 or 1] [sample_time[10..90]] [sample_count[10..180]] [JsonEnable 0 or 1]. JSONEnable is optional.",
	//cmddetail:"fn":"BLv2_SetupEnergyStatistic","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("SetupEnergyStats", BLv2_SetupEnergyStatistic, NULL);
	//cmddetail:{"name":"ConsumptionThreshold","args":"[FloatValue]",
	//cmddetail:"descr":"Setup value for automatic save of consumption data [1..100]",
	//cmddetail:"fn":"BLv2_SetupConsumptionThreshold","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("ConsumptionThreshold", BLv2_SetupConsumptionThreshold, NULL);
	//cmddetail:{"name":"VCPPublishThreshold","args":"[VoltageDeltaVolts][CurrentDeltaAmpers][PowerDeltaWats][EnergyDeltaWh][Frequency]",
	//cmddetail:"descr":"Sets the minimal change between previous reported value over MQTT and next reported value over MQTT. Very useful for BL0942, BL0937, etc. So, if you set, `VCPPublishThreshold 0.25 0.002 0.5 0.25 0.25 0.01` (the default), it will only report voltage again if the delta from previous reported value is larger than 0.25V. Remember, that the device will also ALWAYS force-report values every N seconds (default 60).  The last there arugments are optional",
	//cmddetail:"fn":"BLv2_VCPPublishThreshold","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":"`VCPPublishThreshold 0.5 0.02 1`"}
	CMD_RegisterCommand("VCPPublishThreshold", BLv2_VCPPublishThreshold, NULL);
	//cmddetail:{"name":"VCPPrecision","args":"[VoltageDigits][CurrentDigitsAmpers][PowerDigitsWats][EnergyDigitsWh][Frequency]",
	//cmddetail:"descr":"Sets the number of digits after decimal point for power metering publishes for BL09XX. Default is `VCPPrecision 1 3 2 3 3`. This works for OBK-style publishes.  Only the first argument is required, others are optional.",
	//cmddetail:"fn":"BLv2_VCPPrecision","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("VCPPrecision", BLv2_VCPPrecision, NULL);
	//cmddetail:{"name":"VCPPublishIntervals","args":"[MinDelayBetweenPublishes][ForcedPublishInterval]",
	//cmddetail:"descr":"First argument is minimal allowed interval in second between Voltage/Current/Power/Energy publishes (even if there is a large change), second value is an interval in which V/C/P/E is always published, even if there is no change.  Frequency is published with Voltage",
	//cmddetail:"fn":"BLv2_VCPPublishIntervals","file":"driver/drv_bl_shared_v2.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("VCPPublishIntervals", BLv2_VCPPublishIntervals, NULL);
#if ENABLED_TARIFED_ENERGY
	//cmddetail:{"name":"SetupTarifTimezone","args":"[tarif][time of day]",
	//cmddetail:"descr":"Set start time for select tarif",
	//cmddetail:"fn":"BLv2_SetTarifTimezone","file":"driver/drv_bl_shared_v2.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("SetupTarifTimezone", BLv2_SetTarifTimezone, NULL);
#endif
	blv2_energy_meter_init = 1;
}

/**
 * @brief OBK_POWER etc
 */
float DRV_GetReading_v2(int meter, energySensor_t type)
{
	if (!blv2_energy_meter_init) return NAN;
	if (meter < 0) return NAN;
	if (meter > (blv2_energy_meter.meter_count-1)) return NAN;
	if (!(blv2_energy_meter.outputsensors & (0x01 << type))) return NAN;
	return (float) blv2_energy_meter.datasetlist[meter].sensors[type].lastReading;
}
/**
 * @brief BLv2_HasEnergySensorReading
 */
int BLv2_HasEnergySensorReading(int meter, energySensor_t type)
{
	if (!blv2_energy_meter_init) return false;
	if (meter < 0) return false;
	if (meter > (blv2_energy_meter.meter_count-1)) return false;
	if (type < OBK__FIRST) return false;
	if (type > OBK__LAST) return false;
	if (!(blv2_energy_meter.outputsensors & (0x01 << type))) return false;
	return !isnan(blv2_energy_meter.datasetlist[meter].sensors[type].lastReading);
}

/**
 * @brief BLv2_IsMeteringDeviceIndexActive
 */
int BLv2_IsMeteringDeviceIndexActive(int meter){
	if (!blv2_energy_meter_init) return false;
	if (meter < 0) return false;
	if (meter > (blv2_energy_meter.meter_count-1)) return false;
	return blv2_energy_meter.energy_meter_data[meter].sensors_reciveddata;
}

energySensorNames_t* DRV_GetEnergySensorNames_v2(int meter, energySensor_t type)
{
	// TODO hass.c not checking result !!!
	if (!blv2_energy_meter_init) return 0;
	if (meter < 0) return 0;
	if (meter > (blv2_energy_meter.meter_count-1)) return 0;
	if (!(blv2_energy_meter.outputsensors & (0x01 << type))) return 0;
	return (energySensorNames_t*)&blv2_energy_meter.datasetlist[meter].sensors[type].names;
}

#endif
