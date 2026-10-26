
#include "..\cJSON\cJSON.h"

typedef void(*dpParser_t)(int dpId, int dpType, int dpSize, byte *payload);

void TuyaMCU_Init();
void TuyaMCU_RunFrame();
void TuyaMCU_Shutdown();
void TuyaMCU_Send(byte *data, int size);
void TuyaMCU_OnChannelChanged(int channel,int iVal);
void TuyaMCU_Send_RawBuffer(byte *data, int len);
bool TuyaMCU_IsChannelUsedByTuyaMCU(int channelIndex);
void TuyaMCU_ForcePublishChannelValues();
void TuyaMCU_EnableAutomaticSending(bool enable);
void TuyaMCU_BatteryPoweredMode(bool enable);

void TuyaMCU_Set_dpParser(dpParser_t dpParser_func);
cJSON *TuyaMCU_GetProductInformation(void);

// True once tuyaMcu_setupLED has configured an LED (see #2218).
bool TuyaMCU_HasLED();
