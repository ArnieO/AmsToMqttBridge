#ifndef _FRAMES_H
#define _FRAMES_H

#include <stdint.h>

enum AMSNOW_Frame_Type
{
    AMSNOW_Frame_Type_Info = 0x01,
    AMSNOW_Frame_Type_Bind = 0x02,
    AMSNOW_Frame_Type_Data = 0x03,
    AMSNOW_Frame_Type_Encrypted = 0x09
};

// Send this on interval if not bound to master
typedef struct AMSNOW_Frame_Info
{
    uint8_t type;
    char manufacturer[10];
    char model[20];
    char identifier[20];
    uint8_t pk[32]; // Public key
} __attribute__((packed)) AMSNOW_Frame_Info;

// Master will answer with this frame. Use PK to generate shared key
typedef struct AMSNOW_Frame_Bind
{
    uint8_t type;
    uint8_t pk[32]; // Public key
} __attribute__((packed)) AMSNOW_Frame_Bind;

typedef struct AMSNOW_Frame_Data
{
    uint8_t type;
    char manufacturer[10];
    char model[20];
    char identifier[20];
    uint16_t activeImport;
    uint16_t activeExport;
    int16_t transmitterVcc;  // Transmitter ESP Vcc * 100 (Vcc with two decimals)
    int16_t transmitterTemp; // Temperature sensor value on transmitter ESP * 10 (temperature with one decimal)
    char transmitterUptime[11];
    uint32_t meterTimestamp;
    uint16_t L1Current;
    uint16_t L2Current;
    uint16_t L3Current;
    uint16_t L1Voltage;
    uint16_t L2Voltage;
    uint16_t L3Voltage;
    uint32_t meterCounterTimestamp;
    uint32_t activeImportCounter;
    uint32_t activeExportCounter;
    uint32_t reactiveImportCounter;
    uint32_t reactiveExportCounter;
} __attribute__((packed)) AMSNOW_Frame_Data;

typedef struct AMSNOW_Frame_Encrypted
{
    uint8_t type;
    uint8_t iv[16];    // Initialization vector
    uint8_t data[208]; // Must be a factor of 16
    uint8_t tag[16];   // Tag
} __attribute__((packed)) AMSNOW_Frame_Encrypted;

#endif
