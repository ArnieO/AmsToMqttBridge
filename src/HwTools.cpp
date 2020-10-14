#include "HwTools.h"

void HwTools::setVccResistorGnd(unsigned long VccResistorGnd)
{
    this->VccResistorGnd = VccResistorGnd;
}

void HwTools::setVccResistorVcc(unsigned long VccResistorVcc)
{
    this->VccResistorVcc = VccResistorVcc;
}

void HwTools::setAdcChannelVcc(adc1_channel_t adcChannelVcc)
{
    this->adcChannelVcc = adcChannelVcc;
}

void HwTools::setAdcChannelTemp(adc1_channel_t adcChannelTemp)
{
    this->adcChannelTemp = adcChannelTemp;
}

void HwTools::setAdcAtten(adc_atten_t adcAtten)
{
    this->adcAtten = adcAtten;
}

void HwTools::setAdcAverageLength(int adcAverageLength)
{
    this->adcAverageLength = adcAverageLength;
}

void HwTools::setTempAnalogMillivoltZeroC(int tempAnalogMillivoltZeroC)
{
    this->tempAnalogMillivoltZeroC = tempAnalogMillivoltZeroC;
}

void HwTools::setTempAnalogMillivoltPerC(double tempAnalogMillivoltPerC)
{
    this->tempAnalogMillivoltPerC = tempAnalogMillivoltPerC;
}

unsigned int HwTools::getAdcRaw(adc1_channel_t adcChannel, adc_atten_t adcAtten, int adcAverageLength)
{
    unsigned int errorValue = 0;
    adc1_config_width(ADC_WIDTH_BIT_12); // 12 bits ADC
    adc1_config_channel_atten(adcChannel, adcAtten);
    if (adc1_get_raw(adcChannel) == -1)
    { //ADC parameter error
        errorValue = -99;
    }
    double x = 0.0;
    for (int i = 0; i < adcAverageLength; i++)
    { //average over adcAverageLength samples
        x += adc1_get_raw(adcChannel);
    }
    x = x / adcAverageLength;
    return (errorValue == -99 ? errorValue : (unsigned int)x);
}

double HwTools::getAdcVcc(adc1_channel_t adcChannel, unsigned long resistorGnd, unsigned long resistorVcc, unsigned int averageLength)
{
    esp_adc_cal_characteristics_t *adcChars;

    //Characterize ADC
    adcChars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t valType = esp_adc_cal_characterize(ADC_UNIT_1, adcAtten, ADC_WIDTH_BIT_12, 1100, adcChars);

    //Read ADC
    unsigned int avgAdcRaw = getAdcRaw(adcChannel, adcAtten, adcAverageLength);

    return ((double)esp_adc_cal_raw_to_voltage(avgAdcRaw, adcChars) / 1000.0 * (resistorGnd + resistorVcc) / resistorGnd);
}

double HwTools::getAdcTemp(adc1_channel_t adcChannel, int millivoltZeroC, double millivoltPerC, int adcAverageLength)
{
    esp_adc_cal_characteristics_t *adcChars;

    //Characterize ADC
    adcChars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t valType = esp_adc_cal_characterize(ADC_UNIT_1, adcAtten, ADC_WIDTH_BIT_12, 1100, adcChars);

    //Read ADC
    unsigned int avgAdcRaw = getAdcRaw(adcChannel, adcAtten, adcAverageLength);

    return ((double)(esp_adc_cal_raw_to_voltage(avgAdcRaw, adcChars) - millivoltZeroC) / millivoltPerC);
}

int HwTools::getWifiRssi()
{
    int rssi = WiFi.RSSI();
    return isnan(rssi) ? -100.0 : rssi;
}

void HwTools::setLed(uint8_t ledPin, bool ledInverted)
{
    if (ledPin > 0 && ledPin < 40)
    {
        this->ledPin = ledPin;
        this->ledInverted = ledInverted;
        pinMode(ledPin, OUTPUT);
        ledOff(LED_INTERNAL);
    }
    else
    {
        this->ledPin = 0xFF;
    }
}

void HwTools::setLedRgb(uint8_t ledPinRed, uint8_t ledPinGreen, uint8_t ledPinBlue, bool ledRgbInverted)
{
    this->ledRgbInverted = ledRgbInverted;
    if (ledPinRed > 0 && ledPinRed < 40)
    {
        this->ledPinRed = ledPinRed;
        pinMode(ledPinRed, OUTPUT);
        ledOff(LED_RED);
    }
    else
    {
        this->ledPinRed = 0xFF;
    }
    if (ledPinGreen > 0 && ledPinGreen < 40)
    {
        this->ledPinGreen = ledPinGreen;
        pinMode(ledPinGreen, OUTPUT);
        ledOff(LED_GREEN);
    }
    else
    {
        this->ledPinGreen = 0xFF;
    }
    if (ledPinBlue > 0 && ledPinBlue < 40)
    {
        this->ledPinBlue = ledPinBlue;
        pinMode(ledPinBlue, OUTPUT);
        ledOff(LED_BLUE);
    }
    else
    {
        this->ledPinBlue = 0xFF;
    }
}

bool HwTools::ledOn(uint8_t color)
{
    if (color == LED_INTERNAL)
    {
        return writeLedPin(color, ledInverted ? LOW : HIGH);
    }
    else
    {
        return writeLedPin(color, ledRgbInverted ? LOW : HIGH);
    }
}

bool HwTools::ledOff(uint8_t color)
{
    if (color == LED_INTERNAL)
    {
        return writeLedPin(color, ledInverted ? HIGH : LOW);
    }
    else
    {
        return writeLedPin(color, ledRgbInverted ? HIGH : LOW);
    }
}

bool HwTools::ledBlink(uint8_t color, uint8_t blink)
{
    for (int i = 0; i < blink; i++)
    {
        if (!ledOn(color))
            return false;
        delay(50);
        ledOff(color);
        if (i != blink)
            delay(50);
    }
}

bool HwTools::writeLedPin(uint8_t color, uint8_t state)
{
    switch (color)
    {
    case LED_INTERNAL:
        if (ledPin != 0xFF)
        {
            digitalWrite(ledPin, state);
            return true;
        }
        else
        {
            return false;
        }
        break;
    case LED_RED:
        if (ledPinRed != 0xFF)
        {
            digitalWrite(ledPinRed, state);
            return true;
        }
        else
        {
            return false;
        }
        break;
    case LED_GREEN:
        if (ledPinGreen != 0xFF)
        {
            digitalWrite(ledPinGreen, state);
            return true;
        }
        else
        {
            return false;
        }
        break;
    case LED_BLUE:
        if (ledPinBlue != 0xFF)
        {
            digitalWrite(ledPinBlue, state);
            return true;
        }
        else
        {
            return false;
        }
        break;
    case LED_YELLOW:
        if (ledPinRed != 0xFF && ledPinGreen != 0xFF)
        {
            digitalWrite(ledPinRed, state);
            digitalWrite(ledPinGreen, state);
            return true;
        }
        else
        {
            return false;
        }
        break;
    }
    return false;
}
