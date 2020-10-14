#ifndef _HWTOOLS_H
#define _HWTOOLS_H

#include "Arduino.h"

#include <WiFi.h>
#include <driver/adc.h>  // Enables IDF function names for ADC
#include <esp_adc_cal.h> // Enables ESP32 calibration functions

//#include <DallasTemperature.h>
//#include <OneWire.h>

#define LED_INTERNAL 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3
#define LED_YELLOW 4

struct TempSensorData
{
    uint8_t address[8];
    char name[32];
    bool common;
    float lastRead;
    float lastValidRead;
};

class HwTools
{
public:
    void setVccResistorGnd(unsigned long VccResistorGnd);
    void setVccResistorVcc(unsigned long VccResistorVcc);
    void setAdcAverageLength(int adcAverageLength);
    void setTempAnalogMillivoltZeroC(int tempAnalogMillivoltZeroC);
    void setTempAnalogMillivoltPerC(double tempAnalogMillivoltPerC);
    void setAdcChannelVcc(adc1_channel_t adcChannelVcc);
    void setAdcChannelTemp(adc1_channel_t adcChannelTemp);
    void setAdcAtten(adc_atten_t adcAtten);
    unsigned int getAdcRaw(adc1_channel_t adcChannel, adc_atten_t adcAtten, int adcAverageLength);
    double getAdcVcc(adc1_channel_t adcChannel, unsigned long resistorGnd, unsigned long resistorVcc, unsigned int averageLength);
    double getAdcTemp(adc1_channel_t adcChannel, int millivoltZeroC, double millivoltPerC, int adcAverageLength);
    double getVcc();
    int getWifiRssi();
    void setLed(uint8_t ledPin, bool ledInverted);
    void setLedRgb(uint8_t ledPinRed, uint8_t ledPinGreen, uint8_t ledPinBlue, bool ledRgbInverted);
    bool ledOn(uint8_t color);
    bool ledOff(uint8_t color);
    bool ledBlink(uint8_t color, uint8_t blink);

    HwTools(){};

private:
    uint8_t tempSensorPin = 0xFF, tempAnalogSensorPin = 0xFF;
    uint8_t vccPin = 0xFF;
    uint8_t ledPin = 0xFF, ledPinRed = 0xFF, ledPinGreen = 0xFF, ledPinBlue = 0xFF;
    bool ledInverted, ledRgbInverted;
    double vccOffset = 0.0;
    double vccMultiplier = 1.0;
    unsigned long VccResistorGnd = 22000; // ohm
    unsigned long VccResistorVcc = 33000; // ohm
    adc1_channel_t adcChannelVcc, adcChannelTemp;
    adc_atten_t adcAtten = ADC_ATTEN_DB_6;
    int adcAverageLength = 10;
    int tempAnalogMillivoltZeroC = 400;
    double tempAnalogMillivoltPerC = 19.5;

    bool writeLedPin(uint8_t color, uint8_t state);
    bool isSensorAddressEqual(uint8_t a[8], uint8_t b[8]);
};

#endif
