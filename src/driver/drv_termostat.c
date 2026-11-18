/*
MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../hal/hal_flashVars.h"
#include "../cmnds/cmd_public.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"
#include "drv_deviceclock.h"
#include "../libraries/obktime/obktime.h"	// for time functions
#include "math.h"
#include "strings.h"

#include "../hal/hal_pins.h"

#define HEATER_MAX_COUNT 3

typedef struct {
	/* setting values */
	int 		switch_channel;         /* current heater switch state */
	int 		block_channel;
	double      heat_power;
	/* statistic */
	uint32_t    switch_count;			/* switch on/off counter */
	time_t      heater_on_total;        /* worked time total */
	/* operates values */
	bool        on;
	bool        blocked;
	uint16_t 	on_ticks; 				/* on-time for current cycle */
	uint16_t 	done_ticks; 			/* сколько секунд уже отработало */
	uint32_t 	last_change; 			/* метка времени последнего переключения */
} heater_t;

static int	  g_heater_count = HEATER_MAX_COUNT;    /* number of heaters */
static int	  g_work_cycle = 360;					/* цикл пропорционирования, с */
static int    g_cycle_tick = 0;
static int    g_min_on_s = 180;
static int    g_min_off_s = 30;

static double g_carry_ws = 0.0;

static int	  g_pump_channel = 1;
//static int    g_error_channel = 6;

//static int	  g_..._channel = 7;

static heater_t  g_heaters_m[HEATER_MAX_COUNT];
static heater_t *g_heaters_l[HEATER_MAX_COUNT];


static int	  g_temp_water_channel = 5;
static double g_temp_water = 0.0;			/* current boiler water temperature (from g_temp_channel)*/
static time_t g_temp_water_update;

static int	  g_temp_out_channel = 15;
static double g_temp_out = 0.0;				/* current outer temperature (from g_temp_out_channel)*/
static time_t g_temp_out_update;

static int	  g_temp_int_channel = 16;
static double g_temp_int = 0.0;				/* current internal temperature (from g_temp_int_channel)*/
static time_t g_temp_int_update;

static double g_U = 400;                    /* */
static double g_C = 10;                     /* */


//static int	  g_temp_channel = 5;
//static double g_temp = 0.0;				/* current boiler water temperature (from g_temp_channel)*/
//static time_t g_temp_update;

static double g_temp_min = 40.0;        /* minimal work boiler water temperature (by Init() must be setup) */
static double g_temp_max = 60.0;        /* maximal work boiler water temperature (by Init() must be setup) */

static double g_temp_fault = 80.0;      /* fault work boiler water temperature (by Init() must be setup) */

//static double g_temp_lvl  = 40.0;
//static int	  g_power_channel = 7;
//static double g_power_lvl = 0.0;
//static time_t g_power_lvl_update;

static time_t g_now_s;
static int8_t g_mday = -1;

static bool g_termostat_state = false;
static bool g_overheat        = false;

static int  g_deficit_channel = -1;
static bool g_deficit         = false;

static bool g_need_save        = false;
static bool g_need_rotate      = false;

static bool g_wait_temp        = false;
static bool g_wait_power       = false;

void Termostat_SaveStatistic() {

	TERMOSTAT_STATISTIC_DATA data;
	int data_len = sizeof(TERMOSTAT_STATISTIC_DATA);
	memset((void*)&data, 0, data_len);

	data.heater[0].switches_cnt = g_heaters_m[0].switch_count;
	data.heater[0].heater_on_tm = g_heaters_m[0].heater_on_total;

	data.heater[1].switches_cnt = g_heaters_m[1].switch_count;
	data.heater[1].heater_on_tm = g_heaters_m[1].heater_on_total;

	data.heater[2].switches_cnt = g_heaters_m[2].switch_count;
	data.heater[2].heater_on_tm = g_heaters_m[2].heater_on_total;
	data.magic = 0x5585;
	data.crc = 0;
	char *data_p = (char*)&data;
	for (int i=0; i<(data_len-1); i++) {
		data.crc ^= data_p[i];
	}
	HAL_SaveTermostatStatistic(&data);
	g_need_save = false;
}

void Termostat_LoadStatistic() {

	TERMOSTAT_STATISTIC_DATA data;

	HAL_LoadTermostatStatistic(&data);

	if (data.magic == 0x5585) {
		g_heaters_m[0].switch_count    = data.heater[0].switches_cnt;
		g_heaters_m[0].heater_on_total = data.heater[0].heater_on_tm;

		g_heaters_m[1].switch_count    = data.heater[1].switches_cnt;
		g_heaters_m[1].heater_on_total = data.heater[1].heater_on_tm;

		g_heaters_m[2].switch_count    = data.heater[2].switches_cnt;
		g_heaters_m[2].heater_on_total = data.heater[2].heater_on_tm;

		int data_len = sizeof(TERMOSTAT_STATISTIC_DATA);
		char *data_p = (char*)&data;
		char data_crc =0;
		for (int i=0; i<(data_len-1); i++) {
			data_crc ^= data_p[i];
		}
		if (data_crc != data.crc) {
			addLogAdv(LOG_WARN, LOG_FEATURE_SENSOR, "Termostat: Invalide checksumm for statistic data.");
		}
	}
}

int Termostat_ParseChannelStr(const char *channel_str) {
	int channel = -1;
	if ((channel_str[0] == 'C') && (channel_str[1] == 'H')) {
		channel = atoi(channel_str+2);
		if (channel >= 0 && channel < 64) {
			return channel;
		}
	}
	return -1;
}

// startDriver Termostat -T:CH5 -PUMP:CH1 -HL:CH2:CH12:4000 -HL:CH3:CH13:4000 -HL:CH4:CH14:4000 -POWER:CH7 -CYCLE:360 -MINON:180 -MINOFF:30 -MINT:40 -MAXT:60 -FAULTT:80
void Termostat_Init() {
	memset(g_heaters_m, 0, g_heater_count*sizeof(heater_t));
	int arg_cnt = Tokenizer_GetArgsCount();
	//ADDLOG_INFO(LOG_FEATURE_DRV, "\"Termostat\" needs at least one argument, given ony %i" , arg_cnt );
	for (int arg_i = 0; arg_i < arg_cnt; arg_i++) {
		const char* arg_str = Tokenizer_GetArg(arg_i);
		if (arg_str == NULL) continue;
		if (arg_i>0 && arg_str[0] != '-') {
			ADDLOG_INFO(LOG_FEATURE_DRV, "Wrong argument %i" , arg_i);
			continue;
		}
		if (strncmp(arg_str, "-T:", 3) == 0) {
			g_temp_channel = Termostat_ParseChannelStr(&arg_str[3]);
			if (g_temp_channel != -1) {
				int ch_type = CHANNEL_GetType(g_temp_channel);
				if ((ch_type == ChType_Temperature)||(ch_type == ChType_Temperature_div10)||(ch_type == ChType_Temperature_div100)||(ch_type == ChType_Default))
					{
					if (ch_type == ChType_Default) CHANNEL_SetType(g_temp_channel, ChType_Temperature);
				} else {
					addLogAdv(LOG_ERROR, LOG_FEATURE_SENSOR, "Termostat: Channel for temperature has not appropriate type.");
					g_temp_channel = -1;
				}
			} else {
				addLogAdv(LOG_ERROR, LOG_FEATURE_SENSOR, "Termostat: Invalide channel number for -T:CHx.");
			}
			continue;
		}
		if (strncmp(arg_str, "-PUMP:", 6) == 0) {
			g_pump_channel = Termostat_ParseChannelStr(&arg_str[6]);
			if (g_pump_channel != -1) {
				int ch_type = CHANNEL_GetType(g_pump_channel);
				if ((ch_type == ChType_Toggle)||(ch_type == ChType_Default)) {
					if (ch_type == ChType_Default) CHANNEL_SetType(g_pump_channel, ChType_Toggle);
				} else {
					addLogAdv(LOG_ERROR, LOG_FEATURE_SENSOR, "Termostat: Channel for pump toggle has not appropriate type.");
					g_pump_channel = -1;
				}
			} else {
				addLogAdv(LOG_ERROR, LOG_FEATURE_SENSOR, "Termostat: Invalide channel number for -PUMP:CHx.");
			}
			continue;
		}
		if (strncmp(arg_str, "-POWER:", 7) == 0) {
			g_power_channel = Termostat_ParseChannelStr(&arg_str[7]);
			if (g_power_channel != -1) {
				int ch_type = CHANNEL_GetType(g_power_channel);
				if ((ch_type == ChType_Power)||(ch_type == ChType_Power_div10)||(ch_type == ChType_Power_div100)||(ch_type == ChType_Default)) {
					if (ch_type == ChType_Default) CHANNEL_SetType(g_power_channel, ChType_Power);
				} else {
					addLogAdv(LOG_ERROR, LOG_FEATURE_SENSOR, "Termostat: Channel for power has not appropriate type.");
					g_power_channel = -1;
				}
				if (ch_type == ChType_Default) {
					CHANNEL_SetType(g_power_channel, ChType_Power);
				}
			} else {
				addLogAdv(LOG_ERROR, LOG_FEATURE_SENSOR, "Termostat: Invalide channel number for -POWER:CHx.");
			}
			continue;
		}
		if (strncmp(arg_str, "-CYCLE:", 7) == 0) {
			g_work_cycle = itoa(&arg_str[7]);
			if ((g_work_cycle < 60) || (g_work_cycle > 720)) g_work_cycle = 360;
			continue;
		}
		if (strncmp(arg_str, "-MINON:", 7) == 0) {
			g_min_on_s = itoa(&arg_str[7]);
			if ((g_min_on_s < 10) || (g_min_on_s > 720)) g_min_on_s = 60;
			continue;
		}
		if (strncmp(arg_str, "-MINOFF:", 8) == 0) {
			g_min_off_s = itoa(&arg_str[8]);
			if ((g_min_off_s < 10) || (g_min_off_s > 720)) g_min_off_s = 60;
			continue;
		}
		if (strncmp(arg_str, "-FAULTT:", 8) == 0) {
			g_temp_fault = itoa(&arg_str[8]);
			if ((g_temp_fault < 80) || (g_temp_fault > 105)) g_temp_fault = 80;
			continue;
		}
		if (strncmp(arg_str, "-MINT:", 6) == 0) {
			g_temp_min = (double)itoa(&arg_str[8]);
			if ((g_temp_min < 20) || (g_temp_min > 50)) g_temp_min = 40;
			continue;
		}
		if (strncmp(arg_str, "-MAXT:", 6) == 0) {
			g_temp_max = (double)itoa(&arg_str[8]);
			if ((g_temp_max < 40) || (g_temp_max > 80)) g_temp_max = 60;
			continue;
		}
		if (strncmp(arg_str, "-HL:", 4) == 0) {

			//g_temp_max = (double)itoa(&arg_str[8]);


			continue;
		}

	}

	g_heaters_m[0].switch_channel = 2;
	g_heaters_m[0].block_channel = 12;
	g_heaters_m[0].heat_power = 4000;
	g_heaters_m[0].blocked =  (bool)CHANNEL_Get(g_heaters_m[0].block_channel);
	g_heaters_l[0] = &g_heaters_m[0];

	g_heaters_m[1].switch_channel = 3;
	g_heaters_m[1].block_channel = 13;
	g_heaters_m[1].heat_power = 4000;
	g_heaters_m[1].blocked =  (bool)CHANNEL_Get(g_heaters_m[1].block_channel);
	g_heaters_l[1] = &g_heaters_m[1];

	g_heaters_m[2].switch_channel = 4;
	g_heaters_m[2].block_channel = 14;
	g_heaters_m[2].heat_power = 4000;
	g_heaters_m[2].blocked =  (bool)CHANNEL_Get(g_heaters_m[2].block_channel);
	g_heaters_l[2] = &g_heaters_m[2];

	Termostat_LoadStatistic();

	g_termostat_state = (bool)CHANNEL_Get(g_pump_channel);
	/* Check arguments */
	if (g_temp_max < g_temp_min) {

	}
	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "Termostat driver init done.");
}

void Termostat_appendInformationToHTTPIndexPage(http_request_t* request, int bPreState)
{
	if(bPreState)
		return;
	if (g_termostat_state == true) {
		const char * state_str = "running";
		if (g_wait_temp == true) {
			state_str = "waiting temperature update";
		} else
		if (g_wait_power == true) {
			state_str = "waiting power update";
		}
		hprintf255(request, "<h2>Termostat is %s (cycle %d of %d).</h2>", state_str, g_cycle_tick, g_work_cycle);
	} else {
		hprintf255(request, "<h2>Termostat:</h2>");
	}
	if (g_overheat == true) {
		hprintf255(request, "<h2>WARNING! Temperature is too high (%0.1f)!</h2>", g_temp);
	}
	if (g_deficit == true) {
		hprintf255(request, "<h2>Not enough heaters (%d watt)!</h2>", (uint32_t)g_carry_ws);
	}
	hprintf255(request, "Heater L1: %d switches, on-time %d h.<br>", g_heaters_m[0].switch_count, g_heaters_m[0].heater_on_total/3600);
	hprintf255(request, "Heater L2: %d switches, on-time %d h.<br>", g_heaters_m[1].switch_count, g_heaters_m[1].heater_on_total/3600);
	hprintf255(request, "Heater L3: %d switches, on-time %d h.", g_heaters_m[2].switch_count, g_heaters_m[2].heater_on_total/3600);
}

void Termostat_RotateHeaters(void);
void Termostat_RePlanCycle(heater_t *hp);
void Termostat_PlanCycle(double req_power);
void Termostat_HeaterOff(heater_t *hp, bool add_carry)
{
    if (hp->on) {
        if (add_carry && hp->done_ticks < hp->on_ticks)
            g_carry_ws += (hp->on_ticks - hp->done_ticks) * hp->heat_power;
		if (hp->done_ticks > 0) {
			hp->heater_on_total += hp->done_ticks;
			g_need_save = true;
		}
        hp->on_ticks = hp->done_ticks = 0;
        hp->on = false;
        hp->last_change = g_now_s;
		hp->switch_count++;
		g_need_save = true;
    }
    CHANNEL_Set(hp->switch_channel, false, 0);
}


void Termostat_RunEverySecond() {

	if (TIME_IsTimeSynced()) { // if time is synchronized
		g_now_s = (time_t)TIME_GetCurrentTime();
	} else {
		g_now_s++;
	}
	TimeComponents tc = calculateComponents(g_now_s);
	bool f_day = false;
	if (g_mday == -1) {
		g_mday = tc.day;
	} else if (tc.day != g_mday) { // day changed
		f_day = true;
		g_mday = tc.day;
	}
	if ((g_need_rotate == true) && (f_day == true)) { /* if need rotate once a day*/
		g_need_rotate = false;
		Termostat_RotateHeaters();
	}
	if (g_termostat_state == true) {
		if ((g_now_s - g_temp_update) > 300) { /* check temperature update */
			/* Switch off all heaters*/
			for (uint8_t h = 0; h < g_heater_count; h++) {
				Termostat_HeaterOff(g_heaters_l[h], false);
			}
			g_cycle_tick = 0;
			g_carry_ws = 0;
			g_wait_temp = true;
			return;
		} else {g_wait_temp = false;}
		if (g_temp > g_temp_fault) {
			addLogAdv(LOG_WARN, LOG_FEATURE_DRV, "Warning! Temperature is too high. Switch off heaters.");
			g_overheat = true;
			g_cycle_tick = 0;
			/* Switch off all heaters*/
			for (uint8_t h = 0; h < g_heater_count; h++) {
				Termostat_HeaterOff(g_heaters_l[h], true);
			}
		}
		if ((g_now_s - g_power_lvl_update) > 3600) { /* check power update */
			/* Switch off all heaters*/
			for (uint8_t h = 0; h < g_heater_count; h++) {
				Termostat_HeaterOff(g_heaters_l[h], false);
			}
			g_cycle_tick = 0;
			g_carry_ws = 0;
			g_wait_power = true;
			return;
		} else {g_wait_power = false;}

		if (g_overheat == true) {
			/* wait, while temperature return to max work value*/
			if (g_temp > g_temp_max) return;
			g_overheat = false;
		}
		if (g_cycle_tick == 0) {
			//addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "Planing new cycle.");
			Termostat_PlanCycle(g_power_lvl);
			g_need_rotate = true;
		}

		for (uint8_t h = 0; h < g_heater_count; h++) {
			heater_t *hp = g_heaters_l[h];
			bool want = !hp->blocked &&
						(hp->done_ticks < hp->on_ticks);
			if (want && !hp->on) {
				if (g_now_s - hp->last_change >= g_min_off_s) {
					hp->on = true;
					hp->last_change = g_now_s;
					CHANNEL_Set(hp->switch_channel, true, 0);
				}
			}
			else if (!want && hp->on) {
				/* блокировка по фазе — гасим немедленно, min_on не удерживает */
				bool force_off = hp->blocked || (g_now_s - hp->last_change >= g_min_on_s);
				if (force_off) {
					Termostat_HeaterOff(g_heaters_l[h], true);
				}
			}
			if (hp->on)
				hp->done_ticks++;
		}
		g_cycle_tick = (int)((g_cycle_tick + 1) % g_work_cycle);
	}
}

void Termostat_onChannelChanged(int channel, int iVal) {
	if (channel == g_temp_channel) { /* boiler water temperature changed */
		g_temp = CHANNEL_GetFinalValue(channel);
		g_temp_update = g_now_s;
		return;
	}
	if (channel == g_power_channel) { /* requared heater power changed */
		g_power_lvl = CHANNEL_GetFinalValue(channel);
		if (g_power_lvl < 0.0) g_power_lvl = 0.0;
		g_power_lvl_update = g_now_s;
		return;
	}
	if (channel == g_pump_channel) { /* pump(=termostat) switch on/off */
		g_termostat_state = CHANNEL_Get(channel);
		if (g_termostat_state == true) {
			g_cycle_tick = 0;
			g_carry_ws = 0;
		} else {
			/* switch off heaters */
			for (int h=0; h< g_heater_count; h++)
				Termostat_HeaterOff(g_heaters_l[h], false);
		}
		return;
	}
//	if (channel == g_error_channel) { /* temperature sensor error */
//		/* what shout it do? */
//		return;
//	}
	for (int h=0; h< g_heater_count; h++) {
		if (channel == g_heaters_m[h].switch_channel) { /* manual heater_L1/L2/L3 switch on/off */
			/* what shout it do? */
			if (g_termostat_state == false) CHANNEL_Set(channel, false, 0); /* насос выключен — ТЭН запрещён */
			g_heaters_m[h].on = CHANNEL_Get(channel);
			g_heaters_m[h].last_change = g_now_s;
			return;
		}
		if (channel == g_heaters_m[h].block_channel) { /* heater_L1/L2/L3 block on/off */
			g_heaters_m[h].blocked = CHANNEL_Get(channel);
			if (g_heaters_m[h].blocked == true) {
				Termostat_HeaterOff(&g_heaters_m[h], true);
				for (int hi=0; hi< g_heater_count; hi++) {
					if (h!= hi) {
						Termostat_RePlanCycle(&g_heaters_m[hi]);
					}
				}
			} else {
				Termostat_RePlanCycle(&g_heaters_m[h]);
			}

			return;
		}
	}
}

// stopDriver Termostat
void Termostat_Stop() {
	if (g_need_save == true) {
		Termostat_SaveStatistic();
	}
}

void Termostat_RotateHeaters(void) {
	heater_t *g_heaters_p = g_heaters_l[0];
	g_heaters_l[0] = g_heaters_l[1];
	g_heaters_l[1] = g_heaters_l[2];
	g_heaters_l[2] = g_heaters_p;
}

void Termostat_RePlanCycle(heater_t *hp) {
	if ((hp->blocked == false) && (g_carry_ws > 0) && (g_termostat_state == true)) { /* Try to plan carry_ws */
		int32_t t = (g_work_cycle - g_cycle_tick + 1);
		if (hp->on) {
			t -= hp->on_ticks - hp->done_ticks;
		} else
		if ((g_now_s > hp->last_change) && (g_now_s - hp->last_change < g_min_off_s)) {
			t -= g_min_off_s - (g_now_s - hp->last_change);
		}
		if (t < 0) t = 0;
		if (t > (g_carry_ws / hp->heat_power)) t = (g_carry_ws / hp->heat_power);
		if (t >= g_min_on_s) { /* Heater can be switch on*/
			//hp->done_ticks = 0;
			hp->on_ticks += t;
			g_carry_ws -= t* hp->heat_power;
			//addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "Heater L%d will be switch on againg (t=%d)", (g_heaters_m[0]-hp)/sizeof(heater_t)+1, t);
		} else {
			//addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "Heater L%d can't be switch on againg (t=%d)", (g_heaters_m[0]-hp)/sizeof(heater_t)+1, t);
		}
	}
}

void Termostat_PlanCycle(double req_power) {

	uint8_t heater_avail_cnt = 0;
	for (uint8_t h = 0; h < g_heater_count; h++)
		if (g_heaters_l[h]->blocked == false)
			heater_avail_cnt++;
	bool deficit = false;
	for (uint8_t h = 0; h < g_heater_count; h++) {
		if (g_heaters_l[h]->on_ticks - g_heaters_l[h]->done_ticks)
			g_carry_ws += (g_heaters_l[h]->on_ticks - g_heaters_l[h]->done_ticks) * g_heaters_l[h]->heat_power;
		g_heaters_l[h]->on_ticks = 0;
		g_heaters_l[h]->done_ticks = 0;
	}
	if (heater_avail_cnt == 0) { /* все заблокированы */
		deficit = true;
		g_carry_ws += req_power * g_work_cycle; /* не теряем энергию, переносим */
		return;
	}
	/* Требуемая энергия за цикл + недоданная в прошлых циклах */
	double energy = req_power * g_work_cycle + g_carry_ws;
	g_carry_ws = 0.0;
	for (uint8_t h = 0; h < g_heater_count; h++) {
		if (g_heaters_l[h]->blocked == true) continue;
		/* max amount of this heater energy for the cycle */
		double cap_per_heater = g_heaters_l[h]->heat_power * g_work_cycle;
		if ((g_now_s > g_heaters_l[h]->last_change) && (g_now_s - g_heaters_l[h]->last_change < g_min_off_s)) {
			cap_per_heater -= g_heaters_l[h]->heat_power *(g_now_s - g_heaters_l[h]->last_change);
		}
		/* actual amount of this heater energy for the cycle */
		double give = (energy < cap_per_heater) ? energy : cap_per_heater;
		/* energy reminder */
		energy -= give;
		/* секунды работы из энергии: округление к ближайшему */
		uint32_t t = (give / g_heaters_l[h]->heat_power);
		if (t > g_work_cycle) t = g_work_cycle;
		if (t < g_min_on_s) {
			/* короче минимального импульса — не включаем,
			энергию переносим на следующие циклы */
			g_carry_ws += give;
		} else {
			g_heaters_l[h]->on_ticks = (uint16_t)t;
		}
	}
	if (energy > 0) { /* весь доступный ТЭН-парк недостаточен */
		g_carry_ws += energy;
		deficit = true;
	}
	if (g_deficit != deficit) {
		g_deficit = deficit;
		if (g_deficit_channel != -1) CHANNEL_Set(g_deficit_channel, g_deficit, 0);
	}

}


#if 0
// x: ΔT (°C), y: dT (°C/h), p: мощность (Вт), n: число валидных сэмплов
// c_prior: последнее значение C (или стартовая оценка), use_c_prior: fallback
bool regress(const float *x, const float *y, const float *p, int n,
             float c_prior, bool fix_c,
             float *U_out, float *C_out, float *sigma_out)
{
    float mx=0, my=0, mp=0;
    for (int i=0;i<n;i++){ mx+=x[i]; my+=y[i]; mp+=p[i]; }
    mx/=n; my/=n; mp/=n;

    float Sxx=0,Syy=0,Sxy=0,SxP=0,SyP=0;
    for (int i=0;i<n;i++){
        float dx=x[i]-mx, dy=y[i]-my, dp=p[i]-mp;
        Sxx+=dx*dx; Syy+=dy*dy; Sxy+=dx*dy;
        SxP+=dx*dp; SyP+=dy*dp;
    }

    float U, C;
    if (fix_c || Syy < 1e-6f) {
        if (Sxx < 1e-6f) return false;           // деградация полная — пропуск
        C = c_prior;
        U = (SxP - C*SyP) / Sxx;
    } else {
        float D = Sxx*Syy - Sxy*Sxy;
        if (fabsf(D) < 1e-6f) return false;       // коллинеарность — fallback
        U = (Syy*SxP - Sxy*SyP) / D;
        C = (Sxx*SyP - Sxy*SxP) / D;
    }

    // выбросы: остатки, 3 сигмы, один проход
    float s2 = 0;
    for (int i=0;i<n;i++){ float r = p[i]-U*x[i]-C*y[i]; s2 += r*r; }
    float sigma = sqrtf(s2/(n>2?n-2:1));
    float lim = 3.0f*sigma;
    // ... второй проход по не-выбросам опционально ...

    *U_out = U; *C_out = C; *sigma_out = sigma;
    return true;
}
#endif