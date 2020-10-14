#include "AmsConfiguration.h"

uint8_t AmsConfiguration::getBoardType()
{
	return config.boardType;
}

void AmsConfiguration::setBoardType(uint8_t boardType)
{
	config.boardType = boardType;
}

uint8_t AmsConfiguration::getMeterType()
{
	return config.meterType;
}

void AmsConfiguration::setMeterType(uint8_t meterType)
{
	meterChanged |= config.meterType != meterType;
	config.meterType = meterType;
}

bool AmsConfiguration::isSubstituteMissing()
{
	return config.substituteMissing;
}

void AmsConfiguration::setSubstituteMissing(bool substituteMissing)
{
	config.substituteMissing = substituteMissing;
}

bool AmsConfiguration::isSendUnknown()
{
	return config.sendUnknown;
}

void AmsConfiguration::setSendUnknown(bool sendUnknown)
{
	config.sendUnknown = sendUnknown;
}

void AmsConfiguration::clearMeter()
{
	setMeterType(0);
	setSubstituteMissing(false);
	setSendUnknown(false);
}

bool AmsConfiguration::isMeterChanged()
{
	return meterChanged;
}

void AmsConfiguration::ackMeterChanged()
{
	meterChanged = false;
}

bool AmsConfiguration::isDebugTelnet()
{
	return config.debugTelnet;
}

void AmsConfiguration::setDebugTelnet(bool debugTelnet)
{
	config.debugTelnet = debugTelnet;
}

bool AmsConfiguration::isDebugSerial()
{
	return config.debugSerial;
}

void AmsConfiguration::setDebugSerial(bool debugSerial)
{
	config.debugSerial = debugSerial;
}

uint8_t AmsConfiguration::getDebugLevel()
{
	return config.debugLevel;
}

void AmsConfiguration::setDebugLevel(uint8_t debugLevel)
{
	config.debugLevel = debugLevel;
}

bool AmsConfiguration::pinUsed(uint8_t pin)
{
	if (pin == 0xFF)
		return false;
	return pin == config.hanPin ||
		   pin == config.apPin ||
		   pin == config.ledPinRed ||
		   pin == config.ledPinGreen ||
		   pin == config.ledPinBlue;
}

uint8_t AmsConfiguration::getHanPin()
{
	return config.hanPin;
}

void AmsConfiguration::setHanPin(uint8_t hanPin)
{
	if (!pinUsed(hanPin))
	{
		meterChanged |= config.hanPin != hanPin;
		config.hanPin = hanPin;
	}
}

uint8_t AmsConfiguration::getApPin()
{
	return config.apPin;
}

void AmsConfiguration::setApPin(uint8_t apPin)
{
	if (!pinUsed(apPin))
	{
		config.apPin = apPin;
		if (apPin >= 0)
			pinMode(apPin, INPUT_PULLUP);
	}
}

uint8_t AmsConfiguration::getLedPin()
{
	return config.ledPin;
}

void AmsConfiguration::setLedPin(uint8_t ledPin)
{
	if (!pinUsed(ledPin))
	{
		config.ledPin = ledPin;
	}
}

bool AmsConfiguration::isLedInverted()
{
	return config.ledInverted;
}

void AmsConfiguration::setLedInverted(bool ledInverted)
{
	config.ledInverted = ledInverted;
}

uint8_t AmsConfiguration::getLedPinRed()
{
	return config.ledPinRed;
}

void AmsConfiguration::setLedPinRed(uint8_t ledPinRed)
{
	if (!pinUsed(ledPinRed))
	{
		config.ledPinRed = ledPinRed;
	}
}

uint8_t AmsConfiguration::getLedPinGreen()
{
	return config.ledPinGreen;
}

void AmsConfiguration::setLedPinGreen(uint8_t ledPinGreen)
{
	if (!pinUsed(ledPinGreen))
	{
		config.ledPinGreen = ledPinGreen;
	}
}

uint8_t AmsConfiguration::getLedPinBlue()
{
	return config.ledPinBlue;
}

void AmsConfiguration::setLedPinBlue(uint8_t ledPinBlue)
{
	if (!pinUsed(ledPinBlue))
	{
		config.ledPinBlue = ledPinBlue;
	}
}

bool AmsConfiguration::isLedRgbInverted()
{
	return config.ledRgbInverted;
}

void AmsConfiguration::setLedRgbInverted(bool ledRgbInverted)
{
	config.ledRgbInverted = ledRgbInverted;
}

double AmsConfiguration::getVccBootLimit()
{
	return config.vccBootLimit > 0 ? config.vccBootLimit / 10.0 : 0;
}

void AmsConfiguration::setVccBootLimit(double vccBootLimit)
{
	if (vccBootLimit == 0.0)
		config.vccBootLimit = 0;
	else
		config.vccBootLimit = max(25, min((int)(vccBootLimit * 10), 35));
}

void AmsConfiguration::setVccResistorGnd(double vccResistorGnd)
{
	config.vccResistorGnd = vccResistorGnd;
}

double AmsConfiguration::getVccResistorGnd()
{
	return config.vccResistorGnd > 0 ? config.vccResistorGnd : 1;
}

void AmsConfiguration::setVccResistorVcc(double vccResistorVcc)
{
	config.vccResistorVcc = vccResistorVcc;
}

double AmsConfiguration::getVccResistorVcc()
{
	return config.vccResistorVcc > 0 ? config.vccResistorVcc : 1;
}

void AmsConfiguration::setAdcChannelVcc(adc1_channel_t adcChannelVcc)
{
	config.adcChannelVcc = adcChannelVcc;
}

adc1_channel_t AmsConfiguration::getAdcChannelVcc()
{
	return config.adcChannelVcc;
}

void AmsConfiguration::setAdcChannelTemp(adc1_channel_t adcChannelTemp)
{
	config.adcChannelTemp = adcChannelTemp;
}

adc1_channel_t AmsConfiguration::getAdcChannelTemp()
{
	return config.adcChannelTemp;
}

void AmsConfiguration::setAdcAtten(adc_atten_t adcAtten)
{
	config.adcAtten = adcAtten;
}

adc_atten_t AmsConfiguration::getAdcAtten()
{
	return config.adcAtten;
}

void AmsConfiguration::setAdcAverageLength(int adcAverageLength)
{
	config.adcAverageLength = adcAverageLength;
}

int AmsConfiguration::getAdcAverageLength()
{
	return config.adcAverageLength;
}

void AmsConfiguration::setTempAnalogMillivoltZeroC(int tempAnalogMillivoltZeroC)
{
	config.tempAnalogMillivoltZeroC = tempAnalogMillivoltZeroC;
}

int AmsConfiguration::getTempAnalogMillivoltZeroC()
{
	return config.tempAnalogMillivoltZeroC;
}

void AmsConfiguration::setTempAnalogMillivoltPerC(double tempAnalogMillivoltPerC)
{
	config.tempAnalogMillivoltPerC = tempAnalogMillivoltPerC;
}

double AmsConfiguration::getTempAnalogMillivoltPerC()
{
	return config.tempAnalogMillivoltPerC;
}

void AmsConfiguration::setEncryptionKey(uint8_t *key)
{
	for (int i = 0; i < 32; i++)
		config.encryptionKeyArray[i] = key[i];
}

//void AmsConfiguration::getEncryptionKeyArray(uint8_t *key)
uint8_t* AmsConfiguration::getEncryptionKey()
{
	//for (int i = 0; i < 32; i++)
	//	key[i] = config.encryptionKeyArray[i];
	return config.encryptionKeyArray;
}

boolean AmsConfiguration::isPairedWithReceiver()
{
	// Shall be set to TRUE if paired (if isPairedFlag[7] == "Paired")
	return strcmp((const char *)config.isPairedFlag, "Paired") == 0;
}

void AmsConfiguration::setIsPairedFlag()
{
	strcpy((char *)config.isPairedFlag, "Paired");
}

void AmsConfiguration::clear()
{
	clearMeter();

	int address = EEPROM_CONFIG_ADDRESS;

	EEPROM.begin(EEPROM_SIZE);
	while (address < EEPROM_CONFIG_ADDRESS + EEPROM_SIZE)
	{
		EEPROM.put(address++, 0);
	}
	EEPROM.commit();
	EEPROM.end();
}

bool AmsConfiguration::hasConfig()
{
	if (configVersion == 0)
	{
		EEPROM.begin(EEPROM_SIZE);
		configVersion = EEPROM.read(EEPROM_CONFIG_ADDRESS);
		EEPROM.end();
	}
	switch (configVersion)
	{
	case 81:
	case 82:
	case 83:
		return true;
	default:
		configVersion = 0;
		return false;
	}
}

int AmsConfiguration::getConfigVersion()
{
	return configVersion;
}

bool AmsConfiguration::load()
{
	int address = EEPROM_CONFIG_ADDRESS;
	bool success = false;

	EEPROM.begin(EEPROM_SIZE);
	int cs = EEPROM.read(address++);
	switch (cs)
	{
	case 81: // v1.2
		break;
	case 82: // v1.3
		break;
	case 83: // v1.4
		EEPROM.get(address, config);
		success = true;
		break;
	}
	EEPROM.end();

	if (config.apPin >= 0)
		pinMode(config.apPin, INPUT_PULLUP);
	meterChanged = true;

	return success;
}

bool AmsConfiguration::save()
{
	int address = EEPROM_CONFIG_ADDRESS;

	EEPROM.begin(EEPROM_SIZE);
	EEPROM.put(address, EEPROM_CHECK_SUM);
	address++;
	EEPROM.put(address, config);
	bool success = EEPROM.commit();
	EEPROM.end();

	configVersion = EEPROM_CHECK_SUM;
	return success;
}

int AmsConfiguration::readString(int pAddress, char *pString[])
{
	int address = 0;
	byte length = EEPROM.read(pAddress + address);
	address++;

	char *buffer = new char[length];
	for (int i = 0; i < length; i++)
	{
		buffer[i] = EEPROM.read(pAddress + address++);
	}
	*pString = buffer;
	return address;
}

int AmsConfiguration::readInt(int address, int *value)
{
	int lower = EEPROM.read(address);
	int higher = EEPROM.read(address + 1);
	*value = lower + (higher << 8);
	return 2;
}

int AmsConfiguration::readBool(int address, bool *value)
{
	byte y = EEPROM.read(address);
	*value = (bool)y;
	return 1;
}

int AmsConfiguration::readByte(int address, byte *value)
{
	*value = EEPROM.read(address);
	return 1;
}

void AmsConfiguration::print(Print *debugger)
{
	debugger->print("Configuration size: ");
	debugger->println(sizeof(config));
	debugger->println("-----------------------------------------------");
	debugger->printf("Paired with receiver: %s\r\n", this->isPairedWithReceiver() ? "Yes" : "No");
	debugger->printf("meterType:            %i\r\n", this->getMeterType());
	debugger->printf("Substitute missing I2:%s\r\n", this->isSubstituteMissing() ? "Yes" : "No");
	Serial.flush();

	debugger->printf("HAN pin:              %i\r\n", this->getHanPin());
	debugger->printf("LED pin:              %i\r\n", this->getLedPin());
	debugger->printf("Built-in LED inverted:%s\r\n", this->isLedInverted() ? "Yes" : "No");
	debugger->printf("RGB LED red pin:      %i\r\n", this->getLedPinRed());
	debugger->printf("RGB LED green pin:    %i\r\n", this->getLedPinGreen());
	debugger->printf("RGB LED blue pin:     %i\r\n", this->getLedPinBlue());
	debugger->printf("RGB LED inverted:     %s\r\n", this->isLedRgbInverted() ? "Yes" : "No");
	Serial.flush();

	debugger->printf("Vcc resistor GND:     %f\r\n", this->getVccResistorGnd());
	debugger->printf("Vcc resistor Vcc:     %f\r\n", this->getVccResistorVcc());
	gpio_num_t gpio;
	adc1_pad_get_io_num(this->getAdcChannelVcc(), &gpio);
	debugger->printf("Vcc ADC GPIO pin:     %i\r\n", gpio);
	adc1_pad_get_io_num(this->getAdcChannelTemp(), &gpio);
	debugger->printf("Temp ADC GPIO pin:    %i\r\n", gpio);
	debugger->printf("ADC attenuation:      %i\r\n", this->getAdcAtten());
	debugger->printf("ADC average length:   %i\r\n", this->getAdcAverageLength());
	debugger->printf("Temp sensor mV@0degC: %i\r\n", this->getTempAnalogMillivoltZeroC());
	debugger->printf("Temp sensor mV/degC:  %f\r\n", this->getTempAnalogMillivoltPerC());
	Serial.flush();

	debugger->printf("Vcc boot limit:       %f\r\n", this->getVccBootLimit());
	debugger->println("-----------------------------------------------");
}
