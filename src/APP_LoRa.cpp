/** 
 *  @file                   APP_LoRa.cpp
 *  @brief                  Main source file for remote starter gateway
 *  @copyright              2022 Squall-DA
 *  @date                   02/21/2022
 * 
 *  @remark Author:         Squall-DA
 *  @remark Project Tree:   LoRaRemoteStartGW
 * 
 */



/* Self Include */
#include "APP_LoRa.h"

/*========================================================================* 
 *  SECTION - Local Defines                                               * 
 *========================================================================* 
 */

/*========================================================================* 
 *  SECTION - External variables that cannot be defined in header files   * 
 *========================================================================* 
 */

/*========================================================================* 
 *  SECTION - Local function prototypes                                   * 
 *========================================================================* 
 */
void gvProcessLoRaMessage(char * const kpszLoraMessage, uint8_t ubMsgSize);

/*========================================================================* 
 *  SECTION - Local variables                                             * 
 *========================================================================* 
 */

LoraApp::LoraApp(uint8_t ubCsPin = LORA_CS,uint8_t ubIrqPin = LORA_IRQ, uint8_t ubRstPin = LORA_RST, uint8_t ubDio1Pin = 255U) :
radio(new Module(ubCsPin,ubIrqPin,ubRstPin,ubDio1Pin))
{
    SPI.begin();
}

void LoraApp::vRxMode(void){
    radio.invertIQ(false);
    radio.startReceive();
}

void LoraApp::vTxMode(void){
    radio.invertIQ(true);
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void gvAPP_LoRa_onReceive(int16_t swPacketSize)
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

void gvAPP_LoRa_onTxDone(void)
{

}


void gvProcessLoRaMessage(char * const kpszLoraMessage, uint8_t ubMsgSize)
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