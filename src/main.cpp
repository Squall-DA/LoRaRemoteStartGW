/** 
 *  @file                   main.cpp
 *  @brief                  Main source file for remote starter gateway
 *  @copyright              2019 Squall-DA
 *  @date                   12/10/2019
 * 
 *  @remark Author:         Squall-DA
 *  @remark Project Tree:   LoRaRemoteStartGW
 * 
 */

/* Includes */
#include "APP_LoRa.h"

/*
This project uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <stdint.h>
#include <stdlib.h>
#include <WiFi.h>
#include "SSD1306.h"
#include "IotWebConf.h"

/* Self Include */
#include "main.h"

/*========================================================================*
 * SECTION - Local defines                                                *
 * =======================================================================*
 */
#define CONFIG_VERSION "lra1"

/*========================================================================* 
 *  SECTION - Local Defines                                               * 
 *========================================================================* 
 */
#define MQTT_HOST "test.mosquitto.org"
#define MQTT_PORT 8883

/*========================================================================* 
 *  SECTION - External variables that cannot be defined in header files   * 
 *========================================================================* 
 */

/*========================================================================* 
 *  SECTION - Local function prototypes                                   * 
 *========================================================================* 
 */

void handleRoot(void);
void connectToMqtt(void);
void WiFiEvent(WiFiEvent_t event);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttPublish(uint16_t packetId);

/*========================================================================* 
 *  SECTION - Local variables                                             * 
 *========================================================================* 
 */

/* Setup the OLED display at address 0x3c using standard SDA and SCL */
SSD1306 oledDisplay(0x3c,SDA,SCL);

/* LoRa Frequency */
const uint32_t kulFrequency = 915E6;

/* LoRa Pins */ 
const uint8_t kubLoraCsPin = 18;
const uint8_t kubLoraIrqPin = 26;
const uint8_t kubLoraRstPin = 0;  //Throwaway pin Rst not actually connected.

/* DNS and webserver setup */
DNSServer dnsServer;
WebServer webServer(80);

/* IotWebConf specific */
const char thingName[] = "LoRaGW";
const char wifiInitApPass[] = "LoRaPa55word";
IotWebConf iotWebConf(thingName, &dnsServer, &webServer, wifiInitApPass);

char startTruck[2] = {0x01,0x00};
char pszTestMessage[18];

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;



/**
 *  @fn     void setup()
 *
 *  @brief  Runs once at device startup. Is usually used to configure IO
 *  
 *  @return N/A
 *
 *  @author Squall-DA
 *
 *  @note   N/A
 *
 */

void setup() 
{ 
    pinMode(16,OUTPUT);
    digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
    delay(50); 
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、

    /* Init screen */
    oledDisplay.init();
    oledDisplay.flipScreenVertically();  
    oledDisplay.setFont(ArialMT_Plain_10);
    oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);

    // Debug console
    Serial.begin(115200);

    while (!Serial);

    /* init iotWebConf class */ 
    iotWebConf.init();

    webServer.on("/", handleRoot);
    webServer.on("/config", []{ iotWebConf.handleConfig(); });
    webServer.onNotFound([](){ iotWebConf.handleNotFound(); });
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

    WiFi.onEvent(WiFiEvent);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setSecure(true);
    mqttClient.setRootCa("",0);

    /* Configure ESP32 SPI */
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    /* Set lora pins */
    LoRa.setPins(kubLoraCsPin, kubLoraRstPin, kubLoraIrqPin);

    /* Initialize lora frequency */
    if(!LoRa.begin(kulFrequency))
    {
        oledDisplay.drawString(0,0,"LoRa Init Error");
        oledDisplay.display();
        while(true);
    }
    else
    {
      oledDisplay.drawString(0,0,"LoRa Initialized"); 
      oledDisplay.display(); 
    }
    
    LoRa.onReceive(gvAPP_LoRa_OnReceive);
    LoRa.onTxDone(gvAPP_LoRa_OnTxDone);

    gvLoRa_rxMode();
}

/**
 *  @fn     void loop()
 *
 *  @brief  NULL loop of the device. Code placed in this functions
 *          runs as fast as the microcontroller will allow it.
 *  
 *  @return N/A
 *
 *  @author Squall-DA
 *
 *  @note   N/A
 *
 */
void loop() 
{
  if (runEvery(5000)) { // repeat every 5000 millis

    String message = "HeLoRa World! ";
    message += "I'm a Gateway! ";
    message += millis();

    LoRa_sendMessage(message); // send a message

    Serial.println("Send Message!");
  }

    iotWebConf.doLoop();
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        break;
    default:
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe("test/lol", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  mqttClient.publish("test/lol", 0, true, "test 1");
  Serial.println("Publishing at QoS 0");
  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}


/**
 * Handle web requests to "/" path.
 */
void handleRoot(void)
{
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "<title>IotWebConf 01 Minimal</title></head><body>Hello world!";
  s += "Go to <a href='config'>configure page</a> to change settings.";
  s += "</body></html>\n";

  webServer.send(200, "text/html", s);
}
