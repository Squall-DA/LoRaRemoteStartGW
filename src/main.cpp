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
#include "main.h"
#include <WiFi.h>

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
#include "LoRa.h"
#include "SSD1306.h"


/*========================================================================* 
 *  SECTION - Local Defines                                               * 
 *========================================================================* 
 */
#define WIFI_SSID "yourSSID"
#define WIFI_PASSWORD "yourpass"

#define MQTT_HOST "test.mosquitto.com"
#define MQTT_PORT 8883

/*========================================================================* 
 *  SECTION - External variables that cannot be defined in header files   * 
 *========================================================================* 
 */

/*========================================================================* 
 *  SECTION - Local function prototypes                                   * 
 *========================================================================* 
 */

void vLoRa_rxMode(void);
void vLoRa_txMode(void);
void vLoRaOnReceiveMsg(int swPacketSize);
void gvProcessLoRaMessage(char * const kpszLoraMessage, uint8_t ubMsgSize);
void connectToWifi(void);

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

/**
 * @brief The NULL terminated UUID for this device. Stored as individual
 *        chars to save space.
 * 
 */
const char kpszRemoteStartUuid[] = {0x6e,0x2c,0xab,0x63,0xe4,0x64,0x4f,0x57,0xa0,0xe7,0x70,0x6a,0x98,0xe3,0x1a,0xc3,0x00};

/* Setup the OLED display at address 0x3c using standard SDA and SCL */
SSD1306 oledDisplay(0x3c,SDA,SCL);

/* LoRa Frequency */
const uint32_t kulFrequency = 915E6;

/* LoRa Pins */ 
const uint8_t kubLoraCsPin = 18;
const uint8_t kubLoraIrqPin = 26;
const uint8_t kubLoraRstPin = 0;  //Throwaway pin Rst not actually connected.


char startTruck[2] = {0x01,0x00};
char pszTestMessage[18];

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;



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

    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

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

    connectToWifi();

    /* Configure ESP32 SPI */
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    /* Set lora pins */
    LoRa.setPins(kubLoraCsPin, kubLoraRstPin, kubLoraIrqPin);

    /* Initialize lora frequency */
    if(!LoRa.begin(kulFrequency))
    {
        oledDisplay.drawString(0,0,"LoRa Init Error");
        oledDisplay.display();
        while(1);
    }
    else
    {
      oledDisplay.drawString(0,0,"LoRa Initialized"); 
      oledDisplay.display(); 
    }
    

    vLoRa_rxMode();
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
    int packetSize = LoRa.parsePacket();
    if (packetSize) 
    { 
        vLoRaOnReceiveMsg(packetSize);  
    }

    delay(10);
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
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
		xTimerStart(wifiReconnectTimer, 0);
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
 * @fn     void vLoRa_rxMode(void)
 * 
 *  @brief  Sets the Gateway to receive mode
 *  
 *  @return N/A
 *
 *  @author Squall-DA
 *
 *  @note   N/A
 *
 */
void vLoRa_rxMode(void){
    LoRa.disableInvertIQ();               // Normal mode
    LoRa.receive();                       // set receive mode
}

/**
 * @fn  void LoRa_txMode(void)
 * 
 * @brief Sets the gateway back to tx mode 
 * 
 */
void vLoRa_txMode(void){
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();                // active invert I and Q signals
}

/**
 * @fn    void vLoRaOnReceiveMsg(int swPacketSize)
 * 
 * @brief This function is a callback that is called any time
 *        the LoRa radio receives a message.
 * 
 * @param swPacketSize - size of the LoRa packet. 
 */
void vLoRaOnReceiveMsg(int swPacketSize)
{
    /* Create temporary string the size of the message */
    char * pszMsgString = nullptr;
    uint8_t ubCount = 0;

    if(256 >= swPacketSize)
    {
        pszMsgString = (char*)malloc(swPacketSize + 1);
    }
    

    oledDisplay.clear();
    oledDisplay.drawString(0,0, "LoRa msg received.");
    oledDisplay.drawString(0,15, "Size: " + String(swPacketSize,DEC));
    
    if(nullptr != pszMsgString)
    {

        for(ubCount=0; 
            (ubCount < swPacketSize) && LoRa.available();
            ubCount++)
        {
            pszMsgString[ubCount] = LoRa.read();
        }

        /* Process the message then free the message buffer */
        gvProcessLoRaMessage(pszMsgString, swPacketSize);
        free(pszMsgString);
    }

    oledDisplay.display();
}


void gvProcessLoRaMessage(char * const kpszLoraMessage, uint8_t ubMsgSize)
{
    boolean fUuidMatch = false;

    /* Compare the first UUID_LENGTH bytes */
    fUuidMatch = !strncmp(kpszRemoteStartUuid, kpszLoraMessage, UUID_LENGTH);

    if(fUuidMatch)
    {
        switch(kpszLoraMessage[UUID_LENGTH])
        {
            case LoRa_VEH_CMD_START:
                
                oledDisplay.drawString(0,30,"START");
                break;

            case LoRa_VEH_CMD_LOCK:
                
                oledDisplay.drawString(0,30,"LOCK");
                break;

            case LoRa_VEH_CMD_UNLOCK:
                
                oledDisplay.drawString(0,30,"UNLOCK");
                break;

            default:
                break;
        }

        /* Print LoRa Status */
        oledDisplay.drawString(0,45, "RSSI " + String(LoRa.packetRssi(), DEC));

    }
}