#include "AmsWebServer.h"
#include "version.h"

#include "root/index_html.h"
#include "root/configmeter_html.h"
#include "root/configwifi_html.h"
#include "root/configmqtt_html.h"
#include "root/configweb_html.h"
#include "root/boot_css.h"
#include "root/gaugemeter_js.h"
#include "root/gaugemeter_css.h"

#include "Base64.h"


void AmsWebServer::setup(AmsConfiguration* config, Stream* debugger, MQTTClient* mqtt) {
    this->config = config;
    this->debugger = debugger;
	this->mqtt = mqtt;

	server.on("/", HTTP_GET, std::bind(&AmsWebServer::indexHtml, this, std::placeholders::_1));
	server.on("/config-meter", HTTP_GET, std::bind(&AmsWebServer::configMeterHtml, this, std::placeholders::_1));
	server.on("/config-wifi", HTTP_GET, std::bind(&AmsWebServer::configWifiHtml, this, std::placeholders::_1));
	server.on("/config-mqtt", HTTP_GET, std::bind(&AmsWebServer::configMqttHtml, this, std::placeholders::_1));
	server.on("/config-web", HTTP_GET, std::bind(&AmsWebServer::configWebHtml, this, std::placeholders::_1));
	server.on("/boot.css", HTTP_GET, std::bind(&AmsWebServer::bootCss, this, std::placeholders::_1));
	server.on("/gaugemeter.js", HTTP_GET, std::bind(&AmsWebServer::gaugemeterJs, this, std::placeholders::_1)); 
	server.on("/gaugemeter.css", HTTP_GET, std::bind(&AmsWebServer::gaugemeterCss, this, std::placeholders::_1)); 
	server.on("/data.json", HTTP_GET, std::bind(&AmsWebServer::dataJson, this, std::placeholders::_1));

	server.on("/save", HTTP_POST, std::bind(&AmsWebServer::handleSave, this, std::placeholders::_1));

	server.begin(); // Web server start
}

void AmsWebServer::setJson(StaticJsonDocument<1024> json) {
	if(!json.isNull()) {
		p = json["data"]["P"].as<int>();
		po = json["data"]["PO"].as<int>();

		if(json["data"].containsKey("U1")) {
			u1 = json["data"]["U1"].as<double>();
			i1 = json["data"]["I1"].as<double>();
	
			if(json["data"].containsKey("U2")) {
				u2 = json["data"]["U2"].as<double>();
				i2 = json["data"]["I2"].as<double>();

				if(json["data"].containsKey("U3")) {
					u3 = json["data"]["U3"].as<double>();
					i3 = json["data"]["I3"].as<double>();
				}

				// Only way to determine if you have more than one phase is to run this code here
				if(maxPwr == 0 && config->hasConfig() && config->getMainFuse() > 0 && config->getDistributionSystem() > 0) {
					int volt = config->getDistributionSystem() == 2 ? 400 : 230;
					if(u2 > 0) {
						maxPwr = config->getMainFuse() * sqrt(3) * volt;
					} else {
						maxPwr = config->getMainFuse() * 230;
					}
				}
			}

			if(json["data"].containsKey("tPI")) {
				tpi = json["data"]["tPI"].as<double>();
				tpo = json["data"]["tPO"].as<double>();
				tqi = json["data"]["tQI"].as<double>();
				tqo = json["data"]["tQO"].as<double>();
			}
		} else {
			if(po > 0) {
				json["data"]["PO"] = po;
			}
			if(u1 > 0) {
				json["data"]["U1"] = u1;
				json["data"]["I1"] = i1;
			}
			if(u2 > 0) {
				json["data"]["U2"] = u2;
				json["data"]["I2"] = i2;
			}
			if(u3 > 0) {
				json["data"]["U3"] = u3;
				json["data"]["I3"] = i3;
			}
			if(tpi > 0) {
				json["data"]["tPI"] = tpi;
				json["data"]["tPO"] = tpo;
				json["data"]["tQI"] = tqi;
				json["data"]["tQO"] = tqo;
			}
		}
	    this->json = json;
	}
}

bool AmsWebServer::checkSecurity(AsyncWebServerRequest *request, byte level) {
	bool access = WiFi.getMode() == WIFI_AP || !config->hasConfig() || config->getAuthSecurity() < level;
	if(!access && config->getAuthSecurity() >= level && request->hasHeader("Authorization")) {
		println(" forcing web security");
		String expectedAuth = String(config->getAuthUser()) + ":" + String(config->getAuthPassword());

		String providedPwd = request->header("Authorization");
		providedPwd.replace("Basic ", "");
		char inputString[providedPwd.length()];
		providedPwd.toCharArray(inputString, providedPwd.length()+1);

		int inputStringLength = sizeof(inputString);
		int decodedLength = Base64.decodedLength(inputString, inputStringLength);
		char decodedString[decodedLength];
		Base64.decode(decodedString, inputString, inputStringLength);
		print("Received auth: ");
		println(decodedString);
		access = String(decodedString).equals(expectedAuth);
	}

	if(!access) {
		println(" no access, requesting user/pass");
		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", "");
		response->setContentLength(0);
		response->addHeader("WWW-Authenticate", "Basic realm=\"Secure Area\"");
		response->setCode(401);
		request->send(response);
	}
	return access;
}

String AmsWebServer::processor(const String& var) {
	if(var == "VERSION") {
		return VERSION;
	} else if(var == "BOOTSTRAP_CSS") {
		return WiFi.getMode() == WIFI_AP ? "boot.css" : BOOTSTRAP_URL;
	} else if(var == "DATA_P") {
		return String(p);
	} else if(var == "DATA_PO") {
		return String(po);
	} else if(var == "DISPLAY_PRODUCTION") {
		return config->getProductionCapacity() > 0 ? "" : "none";
	} else if(var == "DATA_U1") {
		return u1 > 0 ? String(u1, 1) : "";
	} else if(var == "DATA_I1") {
		return u1 > 0 ? String(i1, 1) : "";
	} else if(var == "DISPLAY_P1") {
		return u1 > 0 ? "" : "none";
	} else if(var == "DATA_U2") {
		return u2 > 0 ? String(u2, 1) : "";
	} else if(var == "DATA_I2") {
		return u2 > 0 ? String(i2, 1) : "";
	} else if(var == "DISPLAY_P2") {
		return u2 > 0 ? "" : "none";
	} else if(var == "DATA_U3") {
		return u3 > 0 ? String(u3, 1) : "";
	} else if(var == "DATA_I3") {
		return u3 > 0 ? String(i3, 1) : "";
	} else if(var == "DISPLAY_P3") {
		return u3 > 0 ? "" : "none";
	} else if(var == "DATA_TPI") {
		return tpi > 0 ? String(tpi, 1) : "";
	} else if(var == "DATA_TPO") {
		return tpi > 0 ? String(tpo, 1) : "";
	} else if(var == "DATA_TQI") {
		return tpi > 0 ? String(tqi, 1) : "";
	} else if(var == "DATA_TQO") {
		return tpi > 0 ? String(tqo, 1) : "";
	} else if(var == "DISPLAY_ACCUMULATIVE") {
		return tpi > 0 ? "" : "none";
	} else if(var == "VCC") {
		double vcc = hw.getVcc();
		return vcc > 0 ? String(vcc, 2) : "";
	} else if(var == "TEMP") {
		double temp = hw.getTemperature();
		return temp > 0 ? String(temp, 1) : "";
	} else if(var == "DISPLAY_TEMP") {
		double temp = hw.getTemperature();
		return temp != DEVICE_DISCONNECTED_C ? "" : "none";
	} else if(var == "WIFI_RSSI") {
		float rssi = WiFi.RSSI();
		rssi = isnan(rssi) ? -100.0 : rssi;
		return rssi > 0 ? String(rssi, 0) : "";
	} else if(var == "WIFI_CHANNEL") {
		return WiFi.channel() > 0 ? String(WiFi.channel()) : "";
	} else if(var == "WIFI_SSID") {
		return !WiFi.SSID().isEmpty() ? String(WiFi.SSID()) : "";
	}
}

void AmsWebServer::indexHtml(AsyncWebServerRequest *request) {
	println("Serving /index.html over http...");

	if(!checkSecurity(request, 2))
		return;

	request->send_P(200, "text/html", INDEX_HTML, std::bind(&AmsWebServer::processor, this, std::placeholders::_1));
}

void AmsWebServer::configMeterHtml(AsyncWebServerRequest *request) {
	println("Serving /config/meter.html over http...");

	if(!checkSecurity(request, 1))
		return;

	String html = String((const __FlashStringHelper*) CONFIGMETER_HTML);
	html.replace("${version}", VERSION);

	if(WiFi.getMode() != WIFI_AP) {
		html.replace("boot.css", BOOTSTRAP_URL);
	}

	html.replace("${config.meterType}", String(config->getMainFuse()));
	for(int i = 0; i<4; i++) {
		html.replace("${config.meterType" + String(i) + "}", config->getMeterType() == i ? "selected"  : "");
	}
	html.replace("${config.distributionSystem}", String(config->getDistributionSystem()));
	for(int i = 0; i<3; i++) {
		html.replace("${config.distributionSystem" + String(i) + "}", config->getDistributionSystem() == i ? "selected"  : "");
	}
	html.replace("${config.mainFuse}", String(config->getMainFuse()));
	for(int i = 0; i<64; i++) {
		html.replace("${config.mainFuse" + String(i) + "}", config->getMainFuse() == i ? "selected"  : "");
	}
	html.replace("${config.productionCapacity}", String(config->getProductionCapacity()));

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html);
	response->setContentLength(html.length());
	response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
	response->addHeader("Pragma", "no-cache");
	response->addHeader("Expires", "-1");
	request->send(response);
}

void AmsWebServer::configWifiHtml(AsyncWebServerRequest *request) {
	println("Serving /config/wifi.html over http...");

	if(!checkSecurity(request, 1))
		return;

	String html = String((const __FlashStringHelper*) CONFIGWIFI_HTML);
	html.replace("${version}", VERSION);

	if(WiFi.getMode() != WIFI_AP) {
		html.replace("boot.css", BOOTSTRAP_URL);
	}

	html.replace("${config.wifiSsid}", config->getWifiSsid());
	html.replace("${config.wifiPassword}", config->getWifiPassword());
	html.replace("${config.wifiIpType1}", config->getWifiIp().isEmpty() ? "" : "selected");
	html.replace("${config.wifiIp}", config->getWifiIp());
	html.replace("${config.wifiGw}", config->getWifiGw());
	html.replace("${config.wifiSubnet}", config->getWifiSubnet());

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html);
	response->setContentLength(html.length());
	response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
	response->addHeader("Pragma", "no-cache");
	response->addHeader("Expires", "-1");
	request->send(response);
}

void AmsWebServer::configMqttHtml(AsyncWebServerRequest *request) {
	println("Serving /config/mqtt.html over http...");

	if(!checkSecurity(request, 1))
		return;

	String html = String((const __FlashStringHelper*) CONFIGMQTT_HTML);
	html.replace("${version}", VERSION);

	if(WiFi.getMode() != WIFI_AP) {
		html.replace("boot.css", BOOTSTRAP_URL);
	}

	html.replace("${config.mqtt}", config->getMqttHost() == 0 ? "" : "checked");
	html.replace("${config.mqttHost}", config->getMqttHost());
	html.replace("${config.mqttPort}", String(config->getMqttPort()));
	html.replace("${config.mqttClientId}", config->getMqttClientId());
	html.replace("${config.mqttPublishTopic}", config->getMqttPublishTopic());
	html.replace("${config.mqttSubscribeTopic}", config->getMqttSubscribeTopic());
	html.replace("${config.mqttUser}", config->getMqttUser());
	html.replace("${config.mqttPassword}", config->getMqttPassword());

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html);
	response->setContentLength(html.length());
	response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
	response->addHeader("Pragma", "no-cache");
	response->addHeader("Expires", "-1");
	request->send(response);
}

void AmsWebServer::configWebHtml(AsyncWebServerRequest *request) {
	println("Serving /config/web.html over http...");

	if(!checkSecurity(request, 1))
		return;

	String html = String((const __FlashStringHelper*) CONFIGWEB_HTML);
	html.replace("${version}", VERSION);

	if(WiFi.getMode() != WIFI_AP) {
		html.replace("boot.css", BOOTSTRAP_URL);
	}

	html.replace("${config.authSecurity}", String(config->getAuthSecurity()));
	for(int i = 0; i<3; i++) {
		html.replace("${config.authSecurity" + String(i) + "}", config->getAuthSecurity() == i ? "selected"  : "");
	}
	html.replace("${config.authUser}", config->getAuthUser());
	html.replace("${config.authPassword}", config->getAuthPassword());

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html);
	response->setContentLength(html.length());
	response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
	response->addHeader("Pragma", "no-cache");
	response->addHeader("Expires", "-1");
	request->send(response);
}

void AmsWebServer::bootCss(AsyncWebServerRequest *request) {
	println("Serving /boot.css over http...");

	AsyncWebServerResponse *response = request->beginResponse_P(200, "text/css", BOOT_CSS);
	response->addHeader("Cache-Control","public, max-age=3600");
	request->send(response);
}

void AmsWebServer::gaugemeterJs(AsyncWebServerRequest *request) {
	println("Serving /gaugemeter.js over http...");

	AsyncWebServerResponse *response = request->beginResponse_P(200, "application/javascript", GAUGEMETER_JS);
	response->addHeader("Cache-Control","public, max-age=3600");
	request->send(response);
}

void AmsWebServer::gaugemeterCss(AsyncWebServerRequest *request) {
	println("Serving /gaugemeter.css over http...");

	AsyncWebServerResponse *response = request->beginResponse_P(200, "text/css", GAUGEMETER_CSS);
	response->addHeader("Cache-Control","public, max-age=3600");
	request->send(response);
}

void AmsWebServer::dataJson(AsyncWebServerRequest *request) {
	println("Serving /data.json over http...");

	if(!checkSecurity(request, 2))
		return;

	StaticJsonDocument<768> json;

    String jsonStr;
	if(!this->json.isNull() && this->json.containsKey("data")) {
		int maxPwr = this->maxPwr;
		if(maxPwr == 0) {
			if(u2 > 0) {
				maxPwr = 20000;
			} else {
				maxPwr = 10000;
			}
		}

		json["up"] = this->json["up"];
		json["t"] = this->json["t"];
		json["data"] = this->json["data"];

		json["p_pct"] = min(p*100/maxPwr, 100);

		if(config->getProductionCapacity() > 0) {
			int maxPrd = config->getProductionCapacity() * 1000;
			json["po_pct"] = min(po*100/maxPrd, 100);
		}
	} else {
		json["p_pct"] = -1;
		json["po_pct"] = -1;
	}

	unsigned long now = millis();
	json["id"] = WiFi.macAddress();
	json["maxPower"] = maxPwr;
	json["meterType"] = config->getMeterType();
	json["currentMillis"] = now;
	double vcc = hw.getVcc();
	json["vcc"] = vcc > 0 ? vcc : 0;

	double temp = hw.getTemperature();
	json["temp"] = temp;

	json.createNestedObject("wifi");
	float rssi = WiFi.RSSI();
	rssi = isnan(rssi) ? -100.0 : rssi;
	json["wifi"]["ssid"] = WiFi.SSID();
	json["wifi"]["channel"] = (int) WiFi.channel();
	json["wifi"]["rssi"] = rssi;

	json.createNestedObject("status");

	String espStatus;
	if(vcc == 0) {
		espStatus = "secondary";
	} else if(vcc > 3.1) {
		espStatus = "success";
	} else if(vcc > 2.8) {
		espStatus = "warning";
	} else {
		espStatus = "danger";
	}
	json["status"]["esp"] = espStatus;

	unsigned long lastHan = json.isNull() ? 0 : json["up"].as<unsigned long>();
	String hanStatus;
	if(config->getMeterType() == 0) {
		hanStatus = "secondary";
	} else if(now - lastHan < 15000) {
		hanStatus = "success";
	} else if(now - lastHan < 30000) {
		hanStatus = "warning";
	} else {
		hanStatus = "danger";
	}
	json["status"]["han"] = hanStatus;

	String wifiStatus;
	if(config->getWifiSsid().isEmpty()) {
		wifiStatus = "secondary";
	} else if(rssi > -75) {
		wifiStatus = "success";
	} else if(rssi > -95) {
		wifiStatus = "warning";
	} else {
		wifiStatus = "danger";
	}
	json["status"]["wifi"] = wifiStatus;

	String mqttStatus;
	if(config->getMqttHost().isEmpty()) {
		mqttStatus = "secondary";
	} else if(mqtt->connected()) {
		mqttStatus = "success";
	} else if(mqtt->lastError() == 0) {
		mqttStatus = "warning";
	} else {
		mqttStatus = "danger";
	}
	json["status"]["mqtt"] = mqttStatus;

	json.createNestedObject("mqtt");
	json["mqtt"]["lastError"] = (int) mqtt->lastError();

	serializeJson(json, jsonStr);

	AsyncWebServerResponse *response = request->beginResponse(200, "application/json", jsonStr);
	response->setContentLength(jsonStr.length());
	response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
	response->addHeader("Pragma", "no-cache");
	response->addHeader("Expires", "-1");
	request->send(response);
}

void AmsWebServer::handleSave(AsyncWebServerRequest *request) {
	String temp;

	if(request->hasArg("meterConfig") && request->arg("meterConfig") == "true") {
		config->setMeterType(request->arg("meterType").toInt());
		config->setDistributionSystem(request->arg("distributionSystem").toInt());
		config->setMainFuse(request->arg("mainFuse").toInt());
		config->setProductionCapacity(request->arg("productionCapacity").toInt());
	}

	if(request->hasArg("wifiConfig") && request->arg("wifiConfig") == "true") {
		config->setWifiSsid(request->arg("wifiSsid"));
		config->setWifiPassword(request->arg("wifiPassword"));
		if(request->hasArg("wifiIpType") && request->arg("wifiIpType").toInt() == 1) {
			config->setWifiIp(request->arg("wifiIp"));
			config->setWifiGw(request->arg("wifiGw"));
			config->setWifiSubnet(request->arg("wifiSubnet"));
		} else {
			config->clearWifiIp();
		}
	}

	if(request->hasArg("mqttConfig") && request->arg("mqttConfig") == "true") {
		if(request->hasArg("mqtt") && request->arg("mqtt") == "true") {
			config->setMqttHost(request->arg("mqttHost"));
			config->setMqttPort(request->arg("mqttPort").toInt());
			config->setMqttClientId(request->arg("mqttClientId"));
			config->setMqttPublishTopic(request->arg("mqttPublishTopic"));
			config->setMqttSubscribeTopic(request->arg("mqttSubscribeTopic"));
			config->setMqttUser(request->arg("mqttUser"));
			config->setMqttPassword(request->arg("mqttPassword"));
			config->setAuthUser(request->arg("authUser"));
			config->setAuthPassword(request->arg("authPassword"));
		} else {
			config->clearMqtt();
		}
	}

	if(request->hasArg("authConfig") && request->arg("authConfig") == "true") {
		config->setAuthSecurity((byte)request->arg("authSecurity").toInt());
		if(config->getAuthSecurity() > 0) {
			config->setAuthUser(request->arg("authUser"));
			config->setAuthPassword(request->arg("authPassword"));
		} else {
			config->clearAuth();
		}
	}

	println("Saving configuration now...");

	if (debugger) config->print(debugger);
	if (config->save()) {
		println("Successfully saved.");
		if(config->isWifiChanged()) {
			String html = "<html><body><h1>Successfully Saved!</h1><a href=\"/\">Go to index</a></form>";
			AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html);
			response->setContentLength(html.length());
			response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
			response->addHeader("Pragma", "no-cache");
			response->addHeader("Expires", "-1");
			request->send(response);

			println("Wifi config changed, rebooting");
#if defined(ESP8266)
			ESP.reset();
#elif defined(ESP32)
			ESP.restart();
#endif
		} else {
			request->redirect("/");
		}
	} else {
		println("Error saving configuration");
		String html = "<html><body><h1>Error saving configuration!</h1></form>";
		AsyncWebServerResponse *response = request->beginResponse(500, "text/html", html);
		response->setContentLength(html.length());
		response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
		response->addHeader("Pragma", "no-cache");
		response->addHeader("Expires", "-1");
		request->send(response);
	}
}


size_t AmsWebServer::print(const char* text)
{
	if (debugger) debugger->print(text);
}
size_t AmsWebServer::println(const char* text)
{
	if (debugger) debugger->println(text);
}
size_t AmsWebServer::print(const Printable& data)
{
	if (debugger) debugger->print(data);
}
size_t AmsWebServer::println(const Printable& data)
{
	if (debugger) debugger->println(data);
}
