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

#include <stdint.h>
#include <stdlib.h>
#include "LoRa.h"
#include "SSD1306.h"
#include "IotWebConf.h"

/*========================================================================*
 * SECTION - Local defines                                                *
 * =======================================================================*
 */
#define CONFIG_VERSION "lra1"

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
void handleRoot(void);

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

/* DNS and webserver setup */
DNSServer dnsServer;
WebServer webServer(80);

/* IotWebConf specific */
const char thingName[] = "LoRaGW";
const char wifiInitApPass[] = "LoRaPa55word";
IotWebConf iotWebConf(thingName, &dnsServer, &webServer, wifiInitApPass);

char startTruck[2] = {0x01,0x00};
char pszTestMessage[18];



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
    
    //LoRa.onReceive(vLoRaOnReceiveMsg);
    //vLoRa_rxMode();

    strcpy(pszTestMessage, kpszRemoteStartUuid);
    strcat(pszTestMessage, startTruck);

    /*Send Response Message */
    vLoRa_txMode();
    LoRa.beginPacket();
    LoRa.print(pszTestMessage);
    LoRa.endPacket();
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

    iotWebConf.doLoop();

    delay(10);
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

    if(20 >= swPacketSize)
    {
        pszMsgString = (char*)malloc(swPacketSize + 1);
    }
    

    oledDisplay.clear();
    oledDisplay.drawString(0,0, "LoRa msg received.");
    oledDisplay.drawString(0,15, "Size: " + String(swPacketSize,DEC));
    
    for(ubCount=0; 
        (ubCount < swPacketSize) && LoRa.available() && (nullptr != pszMsgString);
        ubCount++)
    {
        pszMsgString[ubCount] = LoRa.read();
    }

    /* Process the message then free the message buffer */
    if(nullptr != pszMsgString)
    {
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