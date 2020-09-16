#include "HwTools.h"

void HwTools::setTempSensorPin(int tempSensorPin) {
    if(tempSensorPin != this->tempSensorPin) {
        this->tempSensorInit = false;
        if(sensorApi)
            delete sensorApi;
        if(oneWire)
            delete oneWire;
        if(tempSensorPin > 0 && tempSensorPin < 40) {
            this->tempSensorPin = tempSensorPin;
            pinMode(tempSensorPin, INPUT);
        } else {
            this->tempSensorPin = 0xFF;
        }
    }
}

void HwTools::setTempAnalogSensorPin(int tempAnalogSensorPin) {
    if(tempAnalogSensorPin != this->tempAnalogSensorPin) {
        if(tempAnalogSensorPin > 0 && tempAnalogSensorPin < 40) {
            this->tempAnalogSensorPin = tempAnalogSensorPin;
            pinMode(tempAnalogSensorPin, INPUT);
        } else {
            this->tempAnalogSensorPin = 0xFF;
        }
    }
}

void HwTools::setVccPin(int vccPin) {
    if(vccPin > 0 && vccPin < 40) {
        pinMode(vccPin, INPUT);
        this->vccPin = vccPin;
    } else {
        this->vccPin = 0xFF;
    }
}

void HwTools::setVccOffset(double vccOffset) {
    this->vccOffset = vccOffset;
}

void HwTools::setVccMultiplier(double vccMultiplier) {
    this->vccMultiplier = vccMultiplier;
}

void HwTools::setVccResistorGnd(unsigned long VccResistorGnd) {
    this->VccResistorGnd = VccResistorGnd;
}

void HwTools::setVccResistorVcc(unsigned long VccResistorVcc) {
    this->VccResistorVcc = VccResistorVcc;
}

void HwTools::setAdcChannelVcc(adc1_channel_t adcChannelVcc) {
    this->adcChannelVcc = adcChannelVcc;
}

void HwTools::setAdcChannelTemp(adc1_channel_t adcChannelTemp) {
    this->adcChannelTemp = adcChannelTemp;
}

void HwTools::setAdcAtten(adc_atten_t adcAtten) {
    this->adcAtten = adcAtten;
}

void HwTools::setAdcAverageLength(int adcAverageLength) {
    this->adcAverageLength = adcAverageLength;
}

void HwTools::setTempAnalogMillivoltZeroC(int tempAnalogMillivoltZeroC) {
    this->tempAnalogMillivoltZeroC = tempAnalogMillivoltZeroC;
}

void HwTools::setTempAnalogMillivoltPerC(double tempAnalogMillivoltPerC) {
    this->tempAnalogMillivoltPerC = tempAnalogMillivoltPerC;
}

#if defined(ESP32)
unsigned int HwTools::getAdcRaw(adc1_channel_t adcChannel, adc_atten_t adcAtten, int adcAverageLength) {
    unsigned int errorValue = 0;
    adc1_config_width(ADC_WIDTH_BIT_12);    // 12 bits ADC
    adc1_config_channel_atten(adcChannel, adcAtten);
    if (adc1_get_raw(adcChannel) == -1) { //ADC parameter error
        errorValue = -99;
    }
    double x = 0.0;
    for (int i = 0; i < adcAverageLength; i++) { //average over adcAverageLength samples
        x +=  adc1_get_raw(adcChannel);
    }
    x = x / adcAverageLength;
    return (errorValue == -99 ? errorValue : (unsigned int)x);
}

double HwTools::getAdcVcc(adc1_channel_t adcChannel, unsigned long resistorGnd, unsigned long resistorVcc, unsigned int averageLength) {
    esp_adc_cal_characteristics_t *adcChars;
    esp_adc_cal_value_t valType;

    //Characterize ADC
    adcChars = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    valType = esp_adc_cal_characterize(ADC_UNIT_1, adcAtten, ADC_WIDTH_BIT_12, 1100, adcChars);

    //Read ADC
    unsigned int avgAdcRaw = getAdcRaw(adcChannel, adcAtten, adcAverageLength);

    return((double)esp_adc_cal_raw_to_voltage(avgAdcRaw, adcChars) / 1000.0 * (resistorGnd + resistorVcc) / resistorGnd);
}

double HwTools::getAdcTemp(adc1_channel_t adcChannel, int millivoltZeroC, double millivoltPerC, int adcAverageLength) {
    esp_adc_cal_characteristics_t *adcChars;
    esp_adc_cal_value_t valType;

    //Characterize ADC
    adcChars = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    valType = esp_adc_cal_characterize(ADC_UNIT_1, adcAtten , ADC_WIDTH_BIT_12, 1100, adcChars);

    //Read ADC
    unsigned int avgAdcRaw = getAdcRaw(adcChannel, adcAtten, adcAverageLength);

    return((double)(esp_adc_cal_raw_to_voltage(avgAdcRaw, adcChars)-millivoltZeroC)/millivoltPerC);
}

#endif // #if defined(ESP32)

double HwTools::getVcc() { // Egil har rotet med denne funksjonen; gå evt. tilbake til originalen
    double voltsRead = 0.0, vccVolts = 0.0;
    if(vccPin != 0xFF) {
        #if defined(ESP8266)
            vccVolts = (analogRead(vccPin) / 1024.0) * 3.3;
        #elif defined(ESP32)
            // volts = (analogRead(vccPin) / 4095.0) * 3.3;
            voltsRead = analogRead(vccPin) * 0.00085682 + 0.16059; // Calibrated for offset and gain measured on ESP32-SOLO-1
            vccVolts = voltsRead * (VccResistorGnd + VccResistorVcc) / VccResistorGnd ;
        #endif
    } else {
        #if defined(ESP8266)
            vccVolts = ((double) ESP.getVcc()) / 1024.0;
        #endif
    }

    //return vccOffset + (volts > 0.0 ? volts * vccMultiplier : 0.0);
    return (vccVolts > 0.0 ? vccVolts : 0.0);
}

void HwTools::confTempSensor(uint8_t address[8], const char name[32], bool common) {
    bool found = false;
    for(int x = 0; x < sensorCount; x++) {
        TempSensorData *data = tempSensors[x];
        if(isSensorAddressEqual(data->address, address)) {
            found = true;
            strcpy(data->name, name);
            data->common = common;
        }
    }
    if(!found) {
        TempSensorData *data = new TempSensorData();
        memcpy(data->address, address, 8);
        strcpy(data->name, name);
        data->common = common;
        data->lastRead = DEVICE_DISCONNECTED_C;
        data->lastValidRead = DEVICE_DISCONNECTED_C;
        tempSensors[sensorCount] = data;
        sensorCount++;
    }
}

uint8_t HwTools::getTempSensorCount() {
    return sensorCount;
}

TempSensorData* HwTools::getTempSensorData(uint8_t i) {
    return tempSensors[i];
}

bool HwTools::updateTemperatures() {
    if(tempSensorPin != 0xFF) {
        if(!tempSensorInit) {
            oneWire = new OneWire(tempSensorPin);
            sensorApi = new DallasTemperature(this->oneWire);
            sensorApi->begin();
            delay(50);
            tempSensorInit = true;

            DeviceAddress addr;
            sensorApi->requestTemperatures();
            int c = sensorApi->getDeviceCount();
            //Serial.print("Sensors found: ");
            //Serial.println(c);
            for(int i = 0; i < c; i++) {
                bool found = false;
                sensorApi->getAddress(addr, i);
                float t = sensorApi->getTempC(addr);
                for(int x = 0; x < sensorCount; x++) {
                    TempSensorData *data = tempSensors[x];
                    if(isSensorAddressEqual(data->address, addr)) {
                        found = true;
                        data->lastRead = t;
                        if(t > -85) {
                            data->lastValidRead = t;
                        }
                    }
                }
                if(!found) {
                    TempSensorData *data = new TempSensorData();
                    memcpy(data->address, addr, 8);
                    data->common = true;
                    data->lastRead = t;
                    if(t > -85) {
                        data->lastValidRead = t;
                    }

                    tempSensors[sensorCount] = data;
                    sensorCount++;
                }
                delay(10);
            }
        } else {
            sensorApi->requestTemperatures();

            for(int x = 0; x < sensorCount; x++) {
                TempSensorData *data = tempSensors[x];
                float t = sensorApi->getTempC(data->address);
                data->lastRead = t;
                if(t > -85) {
                    data->lastValidRead = t;
                }
            }
        }

        return true;
    }
    return false;
}

bool HwTools::isSensorAddressEqual(uint8_t a[8], uint8_t b[8]) {
    for(int i = 0; i < 8; i++) {
        if(a[i] != b[i]) return false;
    }
    return true;
}

double HwTools::getTemperature() {
    uint8_t c = 0;
    double ret = 0;
    double analogTemp = getTemperatureAnalog();
    if(analogTemp != DEVICE_DISCONNECTED_C) {
        ret += analogTemp;
        c++;
    }
    for(int x = 0; x < sensorCount; x++) {
        TempSensorData data = *tempSensors[x];
        if(data.common && data.lastValidRead > -85) {
            ret += data.lastValidRead;
            c++;
        }
    }
    return c == 0 ? DEVICE_DISCONNECTED_C : ret/c;
}
double HwTools::getTemperatureAnalog() {
    if(tempAnalogSensorPin != 0xFF) {
        float adcCalibrationFactor = 1.06587;
        int adcRead = analogRead(tempAnalogSensorPin);
        int volts;
        #if defined(ESP8266)
            volts = (analogRead(vccPin) / 1024.0) * 3.3;
        #elif defined(ESP32)
            volts = (analogRead(vccPin) / 4095.0) * 3.3;
        #endif
        return ((volts * adcCalibrationFactor) - 0.4) / 0.0195;
    }
    return DEVICE_DISCONNECTED_C;
}

int HwTools::getWifiRssi() {
    int rssi = WiFi.RSSI();
    return isnan(rssi) ? -100.0 : rssi;
}

void HwTools::setLed(uint8_t ledPin, bool ledInverted) {
    if(ledPin > 0 && ledPin < 40) {
        this->ledPin = ledPin;
        this->ledInverted = ledInverted;
        pinMode(ledPin, OUTPUT);
        ledOff(LED_INTERNAL);
    } else {
        this->ledPin = 0xFF;
    }
}

void HwTools::setLedRgb(uint8_t ledPinRed, uint8_t ledPinGreen, uint8_t ledPinBlue, bool ledRgbInverted) {
    this->ledRgbInverted = ledRgbInverted;
    if(ledPinRed > 0 && ledPinRed < 40) {
        this->ledPinRed = ledPinRed;
        pinMode(ledPinRed, OUTPUT);
        ledOff(LED_RED);
    } else {
        this->ledPinRed = 0xFF;
    }
    if(ledPinGreen > 0 && ledPinGreen < 40) {
        this->ledPinGreen = ledPinGreen;
        pinMode(ledPinGreen, OUTPUT);
        ledOff(LED_GREEN);
    } else {
        this->ledPinGreen = 0xFF;
    }
    if(ledPinBlue > 0 && ledPinBlue < 40) {
        this->ledPinBlue = ledPinBlue;
        pinMode(ledPinBlue, OUTPUT);
        ledOff(LED_BLUE);
    } else {
        this->ledPinBlue = 0xFF;
    }
}

bool HwTools::ledOn(uint8_t color) {
    if(color == LED_INTERNAL) {
        return writeLedPin(color, ledInverted ? LOW : HIGH);
    } else {
        return writeLedPin(color, ledRgbInverted ? LOW : HIGH);
    }
}

bool HwTools::ledOff(uint8_t color) {
    if(color == LED_INTERNAL) {
        return writeLedPin(color, ledInverted ? HIGH : LOW);
    } else {
        return writeLedPin(color, ledRgbInverted ? HIGH : LOW);
    }
}

bool HwTools::ledBlink(uint8_t color, uint8_t blink) {
    for(int i = 0; i < blink; i++) {
        if(!ledOn(color)) return false;
        delay(50);
        ledOff(color);
        if(i != blink)
            delay(50);
    }
}

bool HwTools::writeLedPin(uint8_t color, uint8_t state) {
    switch(color) {
        case LED_INTERNAL:
            if(ledPin != 0xFF) {
                digitalWrite(ledPin, state);
                return true;
            } else {
                return false;
            }
            break;
        case LED_RED:
            if(ledPinRed != 0xFF) {
                digitalWrite(ledPinRed, state);
                return true;
            } else {
                return false;
            }
            break;
        case LED_GREEN:
            if(ledPinGreen != 0xFF) {
                digitalWrite(ledPinGreen, state);
                return true;
            } else {
                return false;
            }
            break;
        case LED_BLUE:
            if(ledPinBlue != 0xFF) {
                digitalWrite(ledPinBlue, state);
                return true;
            } else {
                return false;
            }
            break;
        case LED_YELLOW:
            if(ledPinRed != 0xFF && ledPinGreen != 0xFF) {
                digitalWrite(ledPinRed, state);
                digitalWrite(ledPinGreen, state);
                return true;
            } else {
                return false;
            }
            break;
    }
    return false;
}
