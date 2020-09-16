#include <esp_now.h>

typedef struct {
    short espNowDataStructVer=1; 
    int transmitterVcc;  // Transmitter ESP Vcc * 10 (Vcc with one decimal)
    int transmitterTemp; // Temperature sensor value on transmitter ESP * 10 (temperature with one decimal)
    char transmitterUptime[11];
    char meterMake[15];
    char meterModel[25];
    char meterId[25];
    unsigned long meterTimestamp;
    unsigned int activeImportPower;
    unsigned int activeExportPower;
    unsigned int reactiveImportPower;
    unsigned int reactiveExportPower;
    unsigned int L1Current;
    unsigned int L2Current;
    unsigned int L3Current;
    unsigned int L1Voltage;
    unsigned int L2Voltage;
    unsigned int L3Voltage;
    unsigned long meterCounterTimestamp;
    unsigned int activeImportCounter;
    unsigned int activeExportCounter;
    unsigned int reactiveImportCounter;
    unsigned int reactiveExportCounter;
} amsEspNowDataStruct;

static uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0x96, 0x66, 0xCC };   //ESP-NOW receiver MAC adr (TTGO-display)

static void msg_send_cb(const uint8_t* mac, esp_now_send_status_t sendStatus)
{
  switch (sendStatus)
  {
    case ESP_NOW_SEND_SUCCESS:
      Serial.println("Send success"); break;
    case ESP_NOW_SEND_FAIL:
      Serial.println("Send Failure"); break;
    default:
      break;
  }
}

esp_now_peer_info_t peerInfo;