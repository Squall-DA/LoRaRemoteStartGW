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

void vLoRa_rxMode(void){
    LoRa.disableInvertIQ();               // Normal mode
    LoRa.receive();                       // set receive mode
}

void vLoRa_txMode(void){
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();                // active invert I and Q signals
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