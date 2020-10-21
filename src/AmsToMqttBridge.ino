/**
 * @brief ESP32-based program to receive data from AMS electric meters and send to ESP-NOW peer
 * 
 * @details Originally developed by Roar Fredriksen, this program was created to receive data from
 * AMS electric meters via M-Bus, decode and send to a MQTT broker. The data packet structure 
 * supported by this software is specific to Norwegian meters, but may also support data from
 * electricity providers in other countries. It was originally based on ESP8266, but has also been 
 * adapted to work with ESP32.
 * 
 * @author Roar Fredriksen (@roarfred)
 * The original developer for this project
 * https://github.com/roarfred/AmsToMqttBridge
 * 
 * @author Gunnar Skjold (@gskjold)
 * Maintainer of current code
 * https://github.com/gskjold/AmsToMqttBridge
 * 
 * @author Egil Opsahl (@arnieo)
 * Adapted Gunnar Skjold code for use with "Pow bridge", an ESP-32 based board that sends meter data
 * using ESP-NOW protocol to a receiver that ensures processing of the data (and optional transmission
 * to MQTT server).
 * This board shall never connect to Wifi, so all code for Wifi, MQTT and Domoticz is removed.
 * The board is ESP-32 only, so all ESP8266 code is removed.
 * 
 */

#include "AmsToMqttBridge.h"
#include "HwTools.h"
#include "AmsConfiguration.h"
#include "HanReader.h"
#include "AmsData.h"
#include "Aidon.h"
#include "Kaifa.h"
#include "Kamstrup.h"
#include "Omnipower.h"
#include "Uptime.h"

#include <esp_pm.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

#include "frames.h"
#include "encryption.h"

#define WEBSOCKET_DISABLED true
#include "RemoteDebug.h"

#define DEBUG_ESP_HTTP_CLIENT 1
#define DEBUG_ESP_PORT Serial

HwTools hw;

AmsConfiguration config;

RemoteDebug Debug;
#include "EspNowAms.h"

HanReader hanReader;

Stream *hanSerial;

int64_t lastData = 0;
AMSNOW_Frame_Data dataFrame = {AMSNOW_Frame_Type_Data};

// Light sleep period is needed for Kamstrup meter to preserve power
void espLightSleep()
{
	delay(10); //Short delay to finish transmit before esp_now_deinit()
	ESP_ERROR_CHECK(esp_now_deinit());
	WiFi.disconnect(true);
	WiFi.mode(WIFI_OFF);

	// Light sleep needed for Kamstrup meter to preserve power
	if (config.getMeterType() == METER_TYPE_KAMSTRUP)
	{
		// The Kamstrup meter sends a List3 payload inbetween the two List2 payloads at xx:00:55
		// The normal Ligt sleep duration is calibrated for 10 seconds payload interval, to preserve energy.
		// In order to capture the List3 arriving at xx:00:55, the sleep duration of xx:00:40 is set to a longer interval,
		// to skip List2 payload arriving at xx:00:50
		// After that, normal sleep period after the List3 payload, so also List2 payload at xx:01:00 will be lost.
		// Retained payloads will be:
		// List2 at xx:00:40
		// List3 at xx:00:55
		// List2 at xx:01:10
		// Then all List1 payloads each 10 seconds until next xx:00:40

		int second = dataFrame.meterTimestamp % 60;
		int minute = (dataFrame.meterTimestamp / 60) % 60;
		long sleepDuration = 10; // microseconds

		Serial.print("Timestamp: ");
		Serial.printf("%02d", minute);
		Serial.print(":");
		Serial.printf("%02d", second);
		Serial.println();

		if ((minute == 00) && (second == 40))
		{
			sleepDuration = 12000000; //12 seconds
		}
		else
		{
			sleepDuration = 8000000; //8 seconds
		}

		//Serial.print("sleepDuration: "); Serial.println(sleepDuration);

		static long wakeTimeMillis;
		static long sleepTimeMillis;
		sleepTimeMillis = esp_timer_get_time() / 1000;

		debugI("Light sleep start after having worked for %d ms", (int)(sleepTimeMillis - wakeTimeMillis));
		delay(10); //Short delay to finish ongoing Serial.println etc before sleep
		esp_sleep_enable_timer_wakeup(sleepDuration);
		esp_light_sleep_start();
		wakeTimeMillis = esp_timer_get_time() / 1000;
	}
}

void setup()
{
	setCpuFrequencyMhz(80); //Reduces power consumption somewhat compared to default 160 MHz

	if (config.hasConfig())
	{
		config.load();
	}

	config.setHanPin(16);
	config.setApPin(0);
	config.setLedPin(0xFF); //ESP32-SOLO-1 has no built-in LED. When set to 0xFF: No signal to any GPIO when attempting to blink built-in LED
	config.setLedPinRed(13);
	config.setLedPinGreen(14);
	config.setLedPinBlue(0xFF);		// Blue LED not used by ESP
	config.setLedInverted(true);	// LED lights on LOW signal
	config.setLedRgbInverted(true); // LED lights on LOW signal
	config.setAdcChannelVcc(ADC1_GPIO35_CHANNEL);
	config.setAdcChannelTemp(ADC1_GPIO34_CHANNEL);
	config.setAdcAtten(ADC_ATTEN_DB_6);
	config.setAdcAverageLength(100);
	config.setTempAnalogMillivoltZeroC(400); //TMP236 temp sensor
	config.setTempAnalogMillivoltPerC(19.5); //TMP236 temp sensor

	delay(10); //Feed watchdog

	hw.setLed(config.getLedPin(), config.isLedInverted());
	hw.setLedRgb(config.getLedPinRed(), config.getLedPinGreen(), config.getLedPinBlue(), config.isLedRgbInverted());
	hw.ledBlink(LED_INTERNAL, 1);
	hw.ledBlink(LED_RED, 1);
	hw.ledBlink(LED_YELLOW, 1);
	hw.ledBlink(LED_GREEN, 1);
	hw.ledBlink(LED_BLUE, 1);

	delay(10); //Feed watchdog

	if (config.getHanPin() == 3)
	{
		switch (config.getMeterType())
		{
		case METER_TYPE_KAMSTRUP:
		case METER_TYPE_OMNIPOWER:
			Serial.begin(2400, SERIAL_8N1);
			break;
		default:
			Serial.begin(2400, SERIAL_8E1);
			break;
		}
	}
	else
		Serial.begin(115200);

	if (config.hasConfig() && config.isDebugSerial())
		Debug.setSerialEnabled(config.isDebugSerial());
	else
	{
#if DEBUG_MODE
		Debug.setSerialEnabled(true);
#endif
	}
	Serial.println();

	// Everything needing output to Serial must be below this line
	Serial.println();
	Serial.println("****************");
	Serial.println("*  Pow bridge  *");
	Serial.println("****************");

	Serial.print("My MAC-address: ");
	Serial.println(WiFi.macAddress());

	if (config.hasConfig())
	{
		Serial.println("Stored config found, reading from EEPROM");
		config.load();
	}

	// If unit is pared with a receiver, change encryption_key pointer to the array in config, and get PeerInfo from config
	if (config.isPairedWithReceiver())
	{
		encryption_key = config.getEncryptionKey();
		// Display encryption key in console
		debugI("Common key: ");
		mbus_hexdump(encryption_key, 32);

		memcpy(&peerInfo, config.getPeerInfo(), sizeof(peerInfo));
		memcpy(master, peerInfo.peer_addr, 6);
		debugI("peerInfo.peer_addr= ");
		mbus_hexdump(peerInfo.peer_addr, 6);
		mbus_hexdump(master, 6);
		debugI("peerInfo.lmk= ");
		mbus_hexdump(peerInfo.lmk, 16);
		debugI("peerInfo.channel= %d", peerInfo.channel);
		debugI("peerInfo.ifidx= %d", peerInfo.ifidx);
		debugI("peerInfo.encrypt= %d", peerInfo.encrypt);

		hasMaster = true;
	}

	// hasMaster = false;	//Used for debug

	double vcc = hw.getAdcVcc(config.getAdcChannelVcc(), config.getVccResistorGnd(), config.getVccResistorVcc(), config.getAdcAverageLength());

	if (Debug.isActive(RemoteDebug::INFO))
	{
		debugI("AMS bridge started");
		debugI("Voltage: %.2fV", vcc);
	}

	// Module to deepsleep if Vcc has not reached vccBootLimit
	double vccBootLimit = config.getVccBootLimit();
	if (vccBootLimit > 0 && (config.getApPin() == 0xFF || digitalRead(config.getApPin()) == HIGH))
	{ // Skip if user is holding AP button while booting (HIGH = button is released)
		if (vcc < vccBootLimit)
		{
			if (Debug.isActive(RemoteDebug::INFO))
			{
				debugI("Voltage is too low, sleeping");
				Serial.flush();
			}
			ESP.deepSleep(10000000); //Deep sleep to allow output cap to charge up
		}
	}

	WiFi.disconnect(true);
	WiFi.softAPdisconnect(true);
	WiFi.mode(WIFI_OFF);

	if (Debug.isActive(RemoteDebug::INFO))
		config.print(&Debug);

	// xTaskCreate(communicationTask, "Comm", 4*1024, NULL, 5, NULL);
}
// END of setup()

int buttonTimer = 0;
bool buttonActive = false;
unsigned long longPressTime = 5000;
bool longPressActive = false;

//unsigned long lastTemperatureRead = 0;
//float temperatures[32];

unsigned long lastRead = 0;
unsigned long lastSuccessfulRead = 0;

void loop()
{
	Debug.handle();
	unsigned long now = millis();

	if (!hasMaster)
		hw.ledOn(LED_YELLOW);
	else
		hw.ledOff(LED_YELLOW);

	if (config.getApPin() != 0xFF)
	{
		if (digitalRead(config.getApPin()) == LOW)
		{
			if (buttonActive == false)
			{
				buttonActive = true;
				buttonTimer = now;
			}

			if ((now - buttonTimer > longPressTime) && (longPressActive == false))
			{
				longPressActive = true;
				hasMaster = false;
			}
		}
		/*
		else
		{
			if (buttonActive == true)
			{
				if (longPressActive == true)
				{
					longPressActive = false;
				}
				else
				{
					// Single press action
				}
				buttonActive = false;
			}
		}
		*/
	}

	if (config.isMeterChanged())
	{
		setupHanPort(config.getHanPin(), config.getMeterType());
		config.ackMeterChanged();
	}

	if (now - lastRead > 100)
	{
		yield();
		if (readHanPort())
		{
			communication(); // Sends over ESP-NOW
			if (hasMaster)
				espLightSleep();
			lastRead = now;
		};
	}

	delay(1); // Needed for auto modem sleep
}
// End loop()

void setupHanPort(int pin, int newMeterType)
{
	debugI("Setting up HAN on pin %d for meter type %d", pin, newMeterType);

	HardwareSerial *hwSerial = NULL;
	if (pin == 3)
		hwSerial = &Serial;
	if (pin == 9)
		hwSerial = &Serial1;
	if (pin == 16)
		hwSerial = &Serial2;

	if (pin == 0)
	{
		debugE("Invalid GPIO configured for HAN");
		return;
	}

	if (hwSerial != NULL)
	{
		debugD("Hardware serial");
		Serial.flush();
		switch (newMeterType)
		{
		case METER_TYPE_KAMSTRUP:
		case METER_TYPE_OMNIPOWER:
			hwSerial->begin(2400, SERIAL_8N1);
			break;
		default:
			hwSerial->begin(2400, SERIAL_8E1);
			break;
		}
		hanSerial = hwSerial;
	}
	else
	{
		debugD("Software serial");
		Serial.flush();
		SoftwareSerial *swSerial = new SoftwareSerial(pin);

		switch (newMeterType)
		{
		case METER_TYPE_KAMSTRUP:
		case METER_TYPE_OMNIPOWER:
			swSerial->begin(2400, SWSERIAL_8N1);
			break;
		default:
			swSerial->begin(2400, SWSERIAL_8E1);
			break;
		}
		hanSerial = swSerial;

		Serial.begin(115200);
	}

	hanReader.setup(hanSerial, &Debug);

	// Compensate for the known Kaifa bug
	hanReader.compensateFor09HeaderBug = (newMeterType == 1);

	// Empty buffer before starting
	while (hanSerial->available() > 0)
	{
		hanSerial->read();
	}

	if (config.hasConfig() && config.isDebugSerial())
	{
		/*
		if (WiFi.status() == WL_CONNECTED)
		{
			Debug.begin(config.getWifiHostname(), (uint8_t)config.getDebugLevel());
		}
		*/
		Debug.setSerialEnabled(config.isDebugSerial());
		if (!config.isDebugTelnet())
		{
			Debug.stop();
		}
	}
}

// Plumbing ESP-NOW
static void now_start()
{
	//Serial.println("Running now_start()");
	WiFi.mode(WIFI_STA); //ESP_ERROR_CHECK(WiFi.mode(WIFI_STA));
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM)); // Maximum modem power saving. In this mode, interval to receive beacons is determined by the listen_interval parameter in wifi_sta_config_t
	ESP_ERROR_CHECK(esp_now_init());

	ESP_ERROR_CHECK(esp_now_register_recv_cb(now_recv_cb));
	ESP_ERROR_CHECK(esp_now_register_send_cb(now_send_cb));

	//esp_now_peer_info_t peerInfo;
	peerInfo.channel = 0;
	peerInfo.ifidx = ESP_IF_WIFI_STA;
	peerInfo.encrypt = false;
	if (!hasMaster)
	{
		memcpy(peerInfo.peer_addr, bcast, 6);
	}
	else
	{
		memcpy(peerInfo.peer_addr, master, 6);
	}
	ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
}

static void now_stop()
{
	ESP_ERROR_CHECK(esp_now_deinit());
	WiFi.disconnect(true); //ESP_ERROR_CHECK(WiFi.disconnect(true));
	WiFi.mode(WIFI_OFF);   //ESP_ERROR_CHECK(WiFi.mode(WIFI_OFF));
}

static void send_once(uint8_t *peer, const uint8_t *msg, int len, uint16_t wait)
{
	//Serial.println("Running send_once()");
	now_start();
	ESP_ERROR_CHECK(esp_now_send(peer, msg, len));
	if (wait > 0)
	{
		vTaskDelay(wait / portTICK_PERIOD_MS);
	}
	Serial.print("Bytes sent: ");
	Serial.println(len);
	now_stop();
}

// ESP-NOW receive callback
static void now_recv_cb(const uint8_t *mac, const uint8_t *incomingData, int len)
{
	Serial.println("Running now_recv_cb()*************************");
	uint8_t type = *incomingData;
	ESP_LOGD(TAG, "New incoming message!! %d bytes, type %02X", len, type);

	// If we receive a bind response to our discovery announcement, use remote public key to create shared encryption key
	if (type == AMSNOW_Frame_Type_Bind)
	{
		AMSNOW_Frame_Bind bind;
		memcpy(&bind, incomingData, sizeof(bind));

		// Generate encryption key
		encryption_key = ams_key_exchange_get(bind.pk);

		//Copy encryption key to config, then save config to EEPROM
		config.setEncryptionKey(encryption_key);

		// Display encryption key in console
		debugI("Common key: ");
		mbus_hexdump(encryption_key, 32);

		// Set master mac address
		memcpy(master, mac, 6);
		hasMaster = true;

		//Update config before saving to EEPROM
		config.setIsPairedFlag();
		config.setPeerInfo((uint8_t *)master);

		// Copy back to peerInfo
		memcpy(&peerInfo, config.getPeerInfo(), sizeof(peerInfo));

		if (config.save())
			debugI("Successfully saved config incl. enc_key to EEPROM");
		else
			debugI("FAILED to save config incl. enc_key to EEPROM");
	}
}

// Sends data over ESP-NOW
void communication()
{

	//Serial.println("Running communication()");
	int64_t time = esp_timer_get_time();

	// Send data every 2500ms
	if (lastData < (time - 2500000))
	{

		// If we have established connection with a master, send meter data
		if (hasMaster)
		{

			// Create a new dataframe to send to master
			AMSNOW_Frame_Data frame = {AMSNOW_Frame_Type_Data};

			// Copy content from temporary container
			memcpy(&frame, &dataFrame, sizeof dataFrame);

			// If we have an encryption key, wrap the whole frame into a encrypted frame
			if (encryption_key != NULL)
			{
				AMSNOW_Frame_Encrypted encFrame = {AMSNOW_Frame_Type_Encrypted};

				// Copy frame info a buffer with fixed size before encrypting
				uint8_t buf[208];
				memcpy(buf, &frame, sizeof frame);

				// Encrypt content, arguments as follows:
				// 		encryption_key 	- The key to use to encrypt message
				// 		encFrame.iv 	- Pointer to initialization vector. Populated by encryption, needed to decrypt
				// 		buf 			- The data to encrypt
				// 		encFrame.data 	- Pointer where encrypted data will be stored
				// 		208 			- Size of the encrypted payload. Needs to be a factor of 16
				// 		encFrame.tag 	- AES tag. Populated by encryption, needed to decrypt
				ams_encrypt(encryption_key, encFrame.iv, buf, encFrame.data, 208, encFrame.tag);
				send_once(master, (uint8_t *)&encFrame, sizeof encFrame, 0);
			}
			else
			{
				// If no encryption, send frame as is
				send_once(master, (uint8_t *)&frame, sizeof frame, 0);
			}

			// If no master, send a discovery message
		}
		else
		{
			// If no public key have been generated, make one
			if (public_key == NULL)
			{
				public_key = ams_key_exchange_init();
			}
			AMSNOW_Frame_Info info = {AMSNOW_Frame_Type_Info};

			memcpy(&info.manufacturer, dataFrame.manufacturer, 10); // Meter manufacturer
			memcpy(&info.model, dataFrame.model, 10);				// Meter model
			memcpy(&info.identifier, dataFrame.identifier, 16);		// Meter ID
			memcpy(&info.pk, public_key, 32);						// Public key

			// Send frame and keep WiFi online for 1000ms to wait for handshake
			send_once(bcast, (uint8_t *)&info, sizeof info, 1000);
		}
		lastData = time;
	}
}

// ------------------ ESP-NOW END ------------------

int currentMeterType = 0;
boolean readHanPort() // returnerer TRUE dersom det har kommet nye data.
{
	boolean newDataArrived = false;

	//Serial.println("Starting readHanPort()");
	if (hanReader.read())
	{
		newDataArrived = true;
		// Empty serial buffer. For some reason this seems to make a difference. Some garbage on the wire after package?
		while (hanSerial->available())
		{
			hanSerial->read();
		}

		lastSuccessfulRead = millis();

		if (config.getMeterType() > 0)
		{
			if (!hw.ledBlink(LED_GREEN, 1))
				hw.ledBlink(LED_INTERNAL, 1);

			AmsData data(config.getMeterType(), config.isSubstituteMissing(), hanReader);
			Serial.print("data.getListType() = ");
			if (data.getListType() > 0)
			{
				String meterMake;

				switch (config.getMeterType()) //currentMeterType
				{
				case METER_TYPE_AIDON:
					meterMake = "Aidon";
					break;
				case METER_TYPE_KAIFA:
					meterMake = "Kaifa";
					break;
				case METER_TYPE_KAMSTRUP:
					meterMake = "Kamstrup";
					break;
				case METER_TYPE_OMNIPOWER:
					meterMake = "Omnipower";
					break;
				default:
					meterMake = "ERROR";
					break;
				};

				// Calculate HANbridge uptime, deliver as formatted string "hhhh:mm:ss"
				static long uptimeSeconds;
				char uptimeString[11], buf[10];
				uptimeSeconds = (int)(esp_timer_get_time() / 1000000);
				sprintf(buf, "%04d", (int)(uptimeSeconds / 3600));
				strcpy(uptimeString, buf);
				strcat(uptimeString, ":");
				sprintf(buf, "%02d", (int)((uptimeSeconds % 3600) / 60));
				strcat(uptimeString, buf);
				strcat(uptimeString, ":");
				sprintf(buf, "%02d", (int)((uptimeSeconds % 3600) % 60));
				strcat(uptimeString, buf);

				Serial.print("hw.getAdcTemp()= ");
				Serial.println(hw.getAdcTemp(config.getAdcChannelTemp(), config.getTempAnalogMillivoltZeroC(), config.getTempAnalogMillivoltPerC(), config.getAdcAverageLength()));

				dataFrame.transmitterVcc = (int16_t)(100.0 * hw.getAdcVcc(config.getAdcChannelVcc(), config.getVccResistorGnd(), config.getVccResistorVcc(), config.getAdcAverageLength()));
				;
				dataFrame.transmitterTemp = (int16_t)(10.0 * hw.getAdcTemp(config.getAdcChannelTemp(), config.getTempAnalogMillivoltZeroC(), config.getTempAnalogMillivoltPerC(), config.getAdcAverageLength()));
				strcpy(dataFrame.transmitterUptime, uptimeString); // Uptime time format transmitted: "hhhh:mm:ss"
				strcpy(dataFrame.manufacturer, meterMake.c_str());
				strcpy(dataFrame.model, data.getMeterType().c_str());
				strcpy(dataFrame.identifier, data.getMeterId().c_str());
				dataFrame.activeImport = data.getActiveImportPower();
				dataFrame.activeExport = data.getActiveExportPower();

				// Some AMS meters deliver timestamp via "Package timestamp", some via "Meter timestamp".
				// To be sure to get a valid timestamp, use one of them that is not zero.
				if (data.getPackageTimestamp() != 0)
					dataFrame.meterTimestamp = data.getPackageTimestamp();
				else
					dataFrame.meterTimestamp = data.getMeterTimestamp();

				dataFrame.L1Current = data.getL1Current() * 100;
				dataFrame.L2Current = data.getL2Current() * 100;
				dataFrame.L3Current = data.getL2Current() * 100;
				dataFrame.L1Voltage = data.getL1Voltage() * 10;
				dataFrame.L2Voltage = data.getL2Voltage() * 10;
				dataFrame.L3Voltage = data.getL3Voltage() * 10;
				if (data.getActiveImportCounter() > 0)
				{ //Only updated when received counter data
					dataFrame.meterCounterTimestamp = dataFrame.meterTimestamp;
					dataFrame.activeImportCounter = data.getActiveImportCounter();
					dataFrame.activeExportCounter = data.getActiveExportCounter();
					dataFrame.reactiveImportCounter = data.getReactiveImportCounter();
					dataFrame.reactiveExportCounter = data.getReactiveExportCounter();
				}
			}
		}
		else
		{
			// Auto detect meter if not set
			for (int i = 1; i <= 3; i++)
			{
				String list;
				switch (i)
				{
				case 1:
					list = hanReader.getString((int)Kaifa_List1Phase::ListVersionIdentifier);
					break;
				case 2:
					list = hanReader.getString((int)Aidon_List1Phase::ListVersionIdentifier);
					break;
				case 3:
					list = hanReader.getString((int)Kamstrup_List1Phase::ListVersionIdentifier);
					break;
				}
				if (!list.isEmpty())
				{
					list.toLowerCase();
					if (list.startsWith("kfm"))
					{
						config.setMeterType(1);
						if (Debug.isActive(RemoteDebug::INFO))
							debugI("Detected Kaifa meter");
						if (config.save())
							debugI("Successfully saved MeterType to EEPROM");
						else
							debugI("FAILED to save MeterType to EEPROM");
						break;
					}
					else if (list.startsWith("aidon"))
					{
						config.setMeterType(2);
						if (Debug.isActive(RemoteDebug::INFO))
							debugI("Detected Aidon meter");
						if (config.save())
							debugI("Successfully saved MeterType to EEPROM");
						else
							debugI("FAILED to save MeterType to EEPROM");
						break;
					}
					else if (list.startsWith("kamstrup"))
					{
						config.setMeterType(3);
						if (Debug.isActive(RemoteDebug::INFO))
							debugI("Detected Kamstrup meter");
						if (config.save())
							debugI("Successfully saved MeterType to EEPROM");
						else
							debugI("FAILED to save MeterType to EEPROM");
						break;
					}
				}
			}
			hanReader.compensateFor09HeaderBug = (config.getMeterType() == 1);
		}
	}

	// Switch parity if meter is still not detected
	if (config.getMeterType() == 0 && millis() - lastSuccessfulRead > 10000)
	{
		lastSuccessfulRead = millis();
		debugD("No data for current setting, switching parity");
		Serial.flush();
		if (++currentMeterType == 4)
			currentMeterType = 1;
		setupHanPort(config.getHanPin(), currentMeterType);
	}
	return newDataArrived;
}