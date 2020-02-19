#ifndef _AMSWEBSERVER_h
#define _AMSWEBSERVER_h

#define BOOTSTRAP_URL "https://cdnjs.cloudflare.com/ajax/libs/twitter-bootstrap/4.4.1/css/bootstrap.min.css"

#include <ArduinoJson.h>
#include <MQTT.h>
#include "AmsConfiguration.h"
#include "HwTools.h"

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#if defined(ESP8266)
	#include <ESP8266WiFi.h>
#elif defined(ESP32) // ARDUINO_ARCH_ESP32
	#include <WiFi.h>
#else
	#warning "Unsupported board type"
#endif

#include <ESPAsyncWebServer.h>
#include "StringStream.h"

class AmsWebServer {
public:
    void setup(AmsConfiguration* config, Stream* debugger, MQTTClient* mqtt);
	void setJson(StaticJsonDocument<1024> json);

private:
	HwTools hw;
    AmsConfiguration* config;
	Stream* debugger;
	MQTTClient* mqtt;
    StaticJsonDocument<1024> json;
    int maxPwr;
	int p, po;
	double u1, u2, u3, i1, i2, i3, tpi, tpo, tqi, tqo;

	AsyncWebServer server = AsyncWebServer(80);
	String processor(const String& var);

	bool checkSecurity(AsyncWebServerRequest *request, byte level);

	void indexHtml(AsyncWebServerRequest *request);
	void configMeterHtml(AsyncWebServerRequest *request);
	void configWifiHtml(AsyncWebServerRequest *request);
	void configMqttHtml(AsyncWebServerRequest *request);
	void configWebHtml(AsyncWebServerRequest *request);
	void bootCss(AsyncWebServerRequest *request);
	void gaugemeterJs(AsyncWebServerRequest *request);
	void gaugemeterCss(AsyncWebServerRequest *request);
    void dataJson(AsyncWebServerRequest *request);

	void handleSave(AsyncWebServerRequest *request);

   	size_t print(const char* text);
	size_t println(const char* text);
	size_t print(const Printable& data);
	size_t println(const Printable& data);

};

#endif
