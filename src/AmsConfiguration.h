#ifndef _AMSCONFIGURATION_h
#define _AMSCONFIGURATION_h
#include <EEPROM.h>
#include <esp_now.h>
#include "Arduino.h"
#include <driver/adc.h>   // Gjør det mulig å bruke mange IDF funksjonsnavn for ADC

struct ConfigObject {
	uint8_t boardType;
	uint8_t meterType;
	uint8_t meterEncryptionKey[16];
	uint8_t meterAuthenticationKey[16];
	bool substituteMissing;	// If set to "true": Calculate I2 if I2 is empty (happens on meters on IT system). Calculation is done in AmsData.cpp.
	bool sendUnknown;
	bool debugTelnet;
	bool debugSerial;
	uint8_t debugLevel;
	uint8_t hanPin;
	uint8_t apPin;
	uint8_t ledPin;
	bool ledInverted;
	uint8_t ledPinRed;
	uint8_t ledPinGreen;
	uint8_t ledPinBlue;
	bool ledRgbInverted;
	uint8_t vccBootLimit;

	double vccResistorGnd;
	double vccResistorVcc;
	adc1_channel_t adcChannelVcc;
	adc1_channel_t adcChannelTemp;
	adc_atten_t adcAtten;
	int adcAverageLength;
	int tempAnalogMillivoltZeroC;
	double tempAnalogMillivoltPerC;
	uint8_t encryptionKeyArray[32];
	uint8_t isPairedFlag[7]; // Shall contain "Paired" if paired
	esp_now_peer_info_t peerInfo;
	
};

class AmsConfiguration {
public:
	bool hasConfig();
	int getConfigVersion();

	bool load();
	bool save();

	uint8_t getBoardType();
	void setBoardType(uint8_t boardType);
	uint8_t getMeterType();
	void setMeterType(uint8_t meterType);
	bool isSubstituteMissing();
	void setSubstituteMissing(bool substituteMissing);
	bool isSendUnknown();
	void setSendUnknown(bool sendUnknown);
	void clearMeter();

	bool isMeterChanged();
	void ackMeterChanged();

	bool isDebugTelnet();
	void setDebugTelnet(bool debugTelnet);
	bool isDebugSerial();
	void setDebugSerial(bool debugSerial);
	uint8_t getDebugLevel();
	void setDebugLevel(uint8_t debugLevel);

	bool pinUsed(uint8_t pin);

	uint8_t getHanPin();
	void setHanPin(uint8_t hanPin);
	uint8_t getApPin();
	void setApPin(uint8_t apPin);
	uint8_t getLedPin();
	void setLedPin(uint8_t ledPin);
	bool isLedInverted();
	void setLedInverted(bool ledInverted);
	
	uint8_t getLedPinRed();
	void setLedPinRed(uint8_t ledPinRed);
	uint8_t getLedPinGreen();
	void setLedPinGreen(uint8_t ledPinGreen);
	uint8_t getLedPinBlue();
	void setLedPinBlue(uint8_t ledPinBlue);
	bool isLedRgbInverted();
	void setLedRgbInverted(bool ledRgbInverted);

	void setVccResistorGnd(double vccResistorGnd);
	double getVccResistorGnd();
    void setVccResistorVcc(double vccResistorVcc);
	double getVccResistorVcc();
    void setAdcChannelVcc(adc1_channel_t adcChannelVcc);
	adc1_channel_t getAdcChannelVcc();
    void setAdcChannelTemp(adc1_channel_t adcChannelTemp);
	adc1_channel_t getAdcChannelTemp();
    void setAdcAtten(adc_atten_t adcAtten);
	adc_atten_t getAdcAtten();
    void setAdcAverageLength(int adcAverageLength);
	int getAdcAverageLength();
    void setTempAnalogMillivoltZeroC(int tempAnalogMillivoltZeroC);
	int getTempAnalogMillivoltZeroC();
    void setTempAnalogMillivoltPerC(double tempAnalogMillivoltPerC);
	double getTempAnalogMillivoltPerC();
	void setEncryptionKey(uint8_t* key);
	uint8_t* getEncryptionKey();
	boolean isPairedWithReceiver();
	void setIsPairedFlag();
	void setPeerInfo(uint8_t* master);
	esp_now_peer_info_t* getPeerInfo();
	
	double getVccBootLimit();
	void setVccBootLimit(double vccBootLimit);

	void print(Print* debugger);

	uint8_t getTempSensorCount();
	
	void clear();

protected:

private:
	int configVersion = 0;
	ConfigObject config {
		0, // Board type
		0, // Meter type
		{}, // Encryption key
		{}, // Authentication key
		true, // Substitute I2 if I2 is empty (happens on meters on IT system)
		false, // Send unknown
		false, // Debug telnet
		false, // Debug serial
		5, // Debug level
		0x03, // HAN pin
		0xFF, // AP pin
		0x02, // LED pin
		true, // Inverted on-board LED
		0xFF, // Red
		0xFF, // Green
		0xFF, // Blue
		true, // Inverted RGB LED
		0, // Vcc Boot limit
		
		22.0,	// vccResistorGnd
		33.0,	// vccResistorVcc
		ADC1_GPIO35_CHANNEL,	// adcChannelVcc
		ADC1_GPIO34_CHANNEL,	// adcChannelTemp
		ADC_ATTEN_DB_6,			// adcAtten
		100,	// adcAverageLength
		400,	// tempAnalogMillivoltZeroC
		19.5,	// tempAnalogMillivoltPerC
		"",		// encryptionKeyArray[32]
		"",		// isPairedFlag[7] ;  Shall contain "Paired" if paired
		
		// 894 bytes ---> gammelt tall, ER VEL NÅ FEIL?
	};

	bool wifiChanged, mqttChanged, meterChanged = true, domoChanged, ntpChanged;

	uint8_t tempSensorCount = 0;
	
	const int EEPROM_SIZE = 1024 * 3;
	const int EEPROM_CHECK_SUM = 83; // Used to check if config is stored. Change if structure changes
	const int EEPROM_CONFIG_ADDRESS = 0;
	const int EEPROM_TEMP_CONFIG_ADDRESS = 2048;

	void loadTempSensors();
	void saveTempSensors();

	bool loadConfig81(int address);
	bool loadConfig82(int address);

	int readString(int pAddress, char* pString[]);
	int readInt(int pAddress, int *pValue);
	int readBool(int pAddress, bool *pValue);
	int readByte(int pAddress, byte *pValue);
};
#endif
