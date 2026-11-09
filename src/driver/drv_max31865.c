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
#include "../cmnds/cmd_public.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"

#include "math.h"
#include "strings.h"

#include "../hal/hal_pins.h"
#include "drv_soft_spi.h"

enum rtd_types {
	rtd_unknown = 0,
    rtd_pt100,
	rtd_pt500,
	rtd_pt1000,
	rtd_cu50,
	rtd_count,
};
float rtd_r0[] = {0.0, 100.0, 500.0, 1000.0, 50.0};
const char *rtd_type_names[] = {"NA", "pt100", "pt500", "pt1000", "cu50"};

enum REG{
    REG_READ_CONFIGURATION = 0x00,
    REG_READ_RTD_MSB,
    REG_READ_RTD_LSB,
    REG_READ_HIGH_FAULT_TH_MSB,
    REG_READ_HIGH_FAULT_TH_LSB,
    REG_READ_LOW_FAULT_TH_MSB,
    REG_READ_LOW_FAULT_TH_LSB,
    REG_READ_FAULT_STATUS,
    REG_WRITE_CONFIGURATION = 0x80,
    REG_WRITE_HIGH_FAULT_TH_MSB = 0x83,
    REG_WRITE_HIGH_FAULT_TH_LSB,
    REG_WRITE_LOW_FAULT_TH_MSB,
    REG_WRITE_LOW_FAULT_TH_LSB
};

enum {
    D0 = 0x01,
    D1 = 0x02,
    D2 = 0x04,
    D3 = 0x08,
    D4 = 0x10,
    D5 = 0x20,
    D6 = 0x40,
    D7 = 0x80
};

static softSPI_t 	spibus;
static uint8_t 		config = D7|D6|D0;
static double 		rtd_rref = 0.0;
static double       rtd_Ohm = 0.0;
static double       g_temp = 0.0;
static int		 	rtd_wire = 4;
static int		 	rtd_type = rtd_unknown;

static int	  g_temp_channel = -1;
static int    g_error_channel = -1;
static byte g_sht_secondsUntilNextMeasurement = 1, g_sht_secondsBetweenMeasurements = 10;
static double g_caltemp = 0.0;



void max31865_read_n_reg(softSPI_t          *spi,
                        uint8_t             start_reg_address,
                        uint8_t*            data,
                        uint8_t             len);

void max31865_write_n_reg(softSPI_t         *spi,
                         uint8_t            start_reg_address,
                         const uint8_t*     data,
                         uint8_t            len);

double rtd_callendar_van_dusen(double R, double R0);
double nsh50M_temperature(double R, double R0);

int max31865_rtd_type(const char *rtd_name) {
	for (int i = 1; i < rtd_count; i++) {
		if (strncmp(rtd_name, rtd_type_names[i], strlen(rtd_type_names[i])) == 0)  return i;
	}
	return rtd_unknown;
}

commandResult_t MAX31865_Calibrate(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_caltemp = Tokenizer_GetArgFloat(0);

	ADDLOG_INFO(LOG_FEATURE_SENSOR, "Calibrate MAX31865: Calibration done temp %f", g_caltemp);

	return CMD_RES_OK;
}

commandResult_t MAX31865_cycle(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_sht_secondsBetweenMeasurements = Tokenizer_GetArgInteger(0);

	ADDLOG_INFO(LOG_FEATURE_CMD, "Measurement will run every %i seconds", g_sht_secondsBetweenMeasurements);

	return CMD_RES_OK;
}

static commandResult_t MAX31865_SetErrorOutput(const void *context, const char *cmd, const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	const char *error_channel_str = Tokenizer_GetArg(0);
	if (!error_channel_str) return CMD_RES_NOT_ENOUGH_ARGUMENTS;

	if ((error_channel_str[0] == 'C') && (error_channel_str[1] == 'H')) {
		g_error_channel = atoi(error_channel_str+2);
		int ch_type = CHANNEL_GetType(g_error_channel);
		if ((ch_type == ChType_Error) || (ch_type == ChType_Default)) {
			CHANNEL_SetType(g_error_channel, ChType_Error);
		} else {
			addLogAdv(LOG_ERROR, LOG_FEATURE_SENSOR, "MAX31865: Channel for sensor error has not appropriate type.");
		}
	}

	return CMD_RES_OK;
}

// startDriver MAX31865 -spi:spi0 TODO
// startDriver MAX31865 -mosi:IO0 -miso:IO23 -sck:IO20 -cs:IO21 -rref:430 -rtd:pt100 -wire:2 -out:CH0
void MAX31865_Init() {
	int arg_cnt = Tokenizer_GetArgsCount();
	//ADDLOG_INFO(LOG_FEATURE_DRV, "\"MAX31865\" needs at least one argument, given ony %i" , arg_cnt );
	int mosi_obk_pin=-1;
	int miso_obk_pin=-1;
	int sck_obk_pin=-1;
	int cs_obk_pin=-1;

	for (int arg_i = 0; arg_i < arg_cnt; arg_i++) {
		const char* arg_str = Tokenizer_GetArg(arg_i);
		if (arg_str == NULL) continue;
		if (arg_str[0] != '-') {
			ADDLOG_INFO(LOG_FEATURE_DRV, "Wrong argument %i" , arg_i);
			continue;
		}
/*
		if (strncmp(arg_str, "-spi:", 5) == 0) {

			continue;
		}
*/
		if (strncmp(arg_str, "-out:", 5) == 0) {
			if ((arg_str[5] == 'C') && (arg_str[6] == 'H')) {
				g_temp_channel = atoi(arg_str+7);
				if (g_temp_channel >= 0 && g_temp_channel < 64) {
					int ch_type = CHANNEL_GetType(g_temp_channel);
					if ((ch_type == ChType_Temperature_div10) || (ch_type == ChType_Default)) {
						CHANNEL_SetType(g_temp_channel, ChType_Temperature_div10);
					} else {
						addLogAdv(LOG_ERROR, LOG_FEATURE_SENSOR, "MAX31865: Channel for sensor temperature has not appropriate type.");
						g_temp_channel = -1;
					}
				} else {
					addLogAdv(LOG_ERROR, LOG_FEATURE_SENSOR, "MAX31865: Invalide channel number.");
				}
			}
			continue;
		}
		if (strncmp(arg_str, "-mosi:", 6) == 0) {
			mosi_obk_pin = HAL_PIN_Find(arg_str+6);
			continue;
		}
		if (strncmp(arg_str, "-miso:", 6) == 0) {
			miso_obk_pin = HAL_PIN_Find(arg_str+6);
			continue;
		}
		if (strncmp(arg_str, "-sck:", 5) == 0) {
			sck_obk_pin = HAL_PIN_Find(arg_str+5);
			continue;
		}
		if (strncmp(arg_str, "-cs:", 4) == 0) {
			cs_obk_pin = HAL_PIN_Find(arg_str+4);
			continue;
		}
/*
		if (strncmp(arg_str, "-ncs:", 5) == 0) {
			ncs_obk_pin = HAL_PIN_Find(arg_str+5);
			continue;
		}
*/
		if (strncmp(arg_str, "-rref:", 6) == 0) {
			rtd_rref = atof(arg_str+6);
			continue;
		}
		if (strncmp(arg_str, "-wire:", 6) == 0) {
			rtd_wire = atoi(arg_str+6);
			continue;
		}
		if (strncmp(arg_str, "-rtd:", 5) == 0) {
			rtd_type = max31865_rtd_type(arg_str+5);
			continue;
		}
	}
	/* Check arguments */
	if ((mosi_obk_pin==-1)||(mosi_obk_pin==-1)||(sck_obk_pin==-1)||(cs_obk_pin==-1)) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "SPI IO-lines are not asigned. Driver not started.");
		return;
	}
	if (rtd_rref == 0) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "Rref is not asigned. Driver not started.");
		return;
	}
	if (rtd_type == rtd_unknown) {
		ADDLOG_WARN(LOG_FEATURE_DRV, "RTD type is not asigned. Temperature can not be calculated.");
	}
	/* Init SPI */
	spibus.sck  = sck_obk_pin;
	spibus.mosi = mosi_obk_pin;
	spibus.miso = miso_obk_pin;
	spibus.ss   = cs_obk_pin;
	SPI_Setup(&spibus);
	if (rtd_wire == 3) config |= D4;
	max31865_write_n_reg(&spibus, REG_WRITE_CONFIGURATION, &config, 1);

	//cmddetail:{"name":"MAX31865_Cycle","args":"[int]",
	//cmddetail:"descr":"This is the interval between measurements in seconds, by default 10. Max is 255.",
	//cmddetail:"fn":"MAX31865_Cycle","file":"driver/drv_shtc3.c","requires":"",
	//cmddetail:"examples":"MAX31865_cycle 60"}
	CMD_RegisterCommand("MAX31865_Cycle", MAX31865_cycle, NULL);
	//cmddetail:{"name":"MAX31865_Calibrate","args":"[DeltaTemp]",
	//cmddetail:"descr":"Calibrate the MAX31865 Sensor as Tolerance is +/-2 degrees C.",
	//cmddetail:"fn":"MAX31865_Calibrate","file":"driver/drv_shtc3.c","requires":"",
	//cmddetail:"examples":"MAX31865_Calibrate -1"}
	CMD_RegisterCommand("MAX31865_Calibrate", MAX31865_Calibrate, NULL);
	//cmddetail:{"name":"MAX31865_SetErrorOutput","args":"CHANNEL",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"MAX31865_SetErrorOutput","file":"drivers/drv_max31865.c","requires":"",
	//cmddetail:"examples":"MAX31865_SetErrorOutput CH10"}
    CMD_RegisterCommand("MAX31865_SetErrorOutput", MAX31865_SetErrorOutput, NULL);

	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "MAX31865 driver init done. SPI:sck=IO%d, si=IO%d, so=IO%d, cs=IO%d.", sck_obk_pin,  mosi_obk_pin, miso_obk_pin, cs_obk_pin);
}

void MAX31865_appendInformationToHTTPIndexPage(http_request_t* request, int bPreState)
{
	if(bPreState)
		return;
	hprintf255(request, "<h2>MAX31865 Temperature=%.1foC</h2>", g_temp);
	hprintf255(request, "RTD type: %s, R(t)=%.1fOhm; R0=%.0fOhm; Rref=%.0fOhm.", rtd_type_names[rtd_type], rtd_Ohm,  rtd_r0[rtd_type], rtd_rref);
}

void MAX31865_RunEverySecond() {
	uint8_t buff[2] = {0,0};
	max31865_read_n_reg(&spibus, REG_READ_RTD_MSB, buff, 2);

	if(buff[1] & 0x01)  {
		addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "ADC result fault.");
		if (g_error_channel != -1) {
			CHANNEL_Set(g_error_channel, 1, 0);
		}
		buff[0] = config|D1;
		max31865_write_n_reg(&spibus, REG_WRITE_CONFIGURATION, buff, 1);
		return;
    }
	uint16_t rtd_adc = (uint16_t)buff[0] << 7 | (uint16_t)buff[1] >> 1;
	rtd_Ohm = ((double)rtd_adc*rtd_rref) / 32768.0;
	//addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "RTD=%0.1f Ohm.", rtd_Ohm);
	switch (rtd_type) {
		case rtd_cu50:
			g_temp = nsh50M_temperature(rtd_Ohm,  rtd_r0[rtd_type])+g_caltemp;
			break;
		case rtd_pt100:
		case rtd_pt500:
		case rtd_pt1000:
			g_temp = rtd_callendar_van_dusen(rtd_Ohm, rtd_r0[rtd_type])+g_caltemp;
			break;
		default:
			return;
			break;
	}
	if (g_temp_channel != -1)
		CHANNEL_Set(g_temp_channel, (int)(g_temp * 10), 0);
	if (g_error_channel != -1) {
		CHANNEL_Set(g_error_channel, 0, 0);
	}
}

// stopDriver MAX31865
void MAX31865_Stop() {


}

void max31865_write_n_reg(softSPI_t         *spi,
                         uint8_t            start_reg_address,
                         const uint8_t*     data,
                         uint8_t            len)
{
    uint8_t index = 0;

    if(len == 0) return;

    SPI_Begin(spi);
    SPI_Send(spi, start_reg_address);

    do{
        SPI_Send(spi, data[index++]);
    } while(index < len);

    SPI_End(spi);
}

void max31865_read_n_reg(softSPI_t          *spi,
                        uint8_t             start_reg_address,
                        uint8_t*            data,
                        uint8_t             len)
{
    uint8_t index = 0;

    if(len == 0) return;

    SPI_Begin(spi);
    SPI_Send(spi, start_reg_address);

    do {
        data[index++] = SPI_Read(spi);
    } while(index < len);

    SPI_End(spi);
}

/* ------------------------------------------------------------
	RTD: уравнение Каллендара–Ван Дюзена (IEC 60751, Pt100)
	T >= 0: R = R0 * (1 + A*T + B*T^2)
	T < 0: R = R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3)
	Коэффициенты — стандартные для платины alpha=0.00385.
------------------------------------------------------------ */
double rtd_callendar_van_dusen(double R, double R0) {
    const double A = 3.9083e-3;
    const double B = -5.775e-7;
    const double C = -4.183e-12;
    /* Относительное сопротивление */
    double r = R / R0;

    if (r >= 1.0) {                       /* T >= 0 °C: квадратное уравнение */
        /* B*T^2 + A*T + (1 - r) = 0 */
        double D = A * A - 4.0 * B * (1.0 - r);
        return (-A + sqrt(D)) / (2.0 * B);
    } else {                              /* T < 0 °C: численное решение (Ньютон) */
        double T = (r - 1.0) / A;         /* стартовое приближение */
        int i;
        for (i = 0; i < 50; i++) {
            double f  = 1.0 + A*T + B*T*T + C*(T - 100.0)*T*T*T - r;
            double df = A + 2.0*B*T + C*(4.0*T*T*T - 300.0*T*T);
            double dT = -f / df;
            T += dT;
            if (fabs(dT) < 1e-9) break;
        }
        return T;
    }
}

/* ------------------------------------------------------------
   НСХ 50М — медный термопреобразователь сопротивления (ТСМ)
   ГОСТ 6651-2009: R0 = 50 Ом при 0 °C, W100 = 1.428,
   диапазон измерений -180...+200 °C
------------------------------------------------------------ */
double nsh50M_temperature(double R, double R0)
{
/* Коэффициенты прямой функции (ГОСТ 6651-2009, п. 5.2.3) */
#define CU_A  (4.28e-3)      /* °C^-1  */
#define CU_B  (-6.2032e-7)   /* °C^-2  */
#define CU_C  (8.5154e-10)   /* °C^-3  */

/* Коэффициенты обратного полинома (ГОСТ 6651-2009, прил. Б) */
#define CU_D1  (233.87)
#define CU_D2  (7.9370)
#define CU_D3  (-2.0062)
#define CU_D4  (-0.3953)

    double x = R / R0 - 1.0;

    if (x >= 0.0) {
        /* 0...200 °C: точная обратная функция t = x / A */
        return x / CU_A;
    }
    /* -180...0 °C: полином ГОСТ, точность ±0.002 °C
       t = D1*x + D2*x^2 + D3*x^3 + D4*x^4  (схема Горнера) */
    return x * (CU_D1 + x * (CU_D2 + x * (CU_D3 + x * CU_D4)));
}
