/** 
 *  @file                   APP_LoRa.h.h 
 *  @brief                  APP_LoRa.h header file
 *  @copyright              2022 Squall-DA
 *  @date                   12/04/2022
 *  
 *  @remark Author:         Squall-DA
 *  @remark Project Tree:   LoRaRemoteStartGW
 *  
 */
#ifndef APP_LORA_MODULE
#define APP_LORA_MODULE 1

#include <stdint.h>
#include <Arduino.h>
#include "RadioLib.h"

/*========================================================================* 
 *  SECTION - Global definitions 
 *========================================================================* 
 */

/*========================================================================* 
 *  SECTION - extern global variables (minimize global variable use)      * 
 *========================================================================* 
 */

/*========================================================================* 
 *  SECTION - extern global functions                                     * 
 *========================================================================* 
 */

class LoraApp {
    public:
        LoraApp(uint8_t _ubCsPin,uint8_t _ubIrqPin, uint8_t _ubRstPin, uint8_t _ubDio1Pin);
        void vTxMode();
        void vRxMode();
        void vSendMessage(String message);

    private:
        SX1276 radio;
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
extern void vLoRa_rxMode(void);

/**
 * @fn  void LoRa_txMode(void)
 * 
 * @brief Sets the gateway back to tx mode 
 * 
 */
extern void vLoRa_txMode(void);

extern void LoRa_sendMessage(String message);

/**
 * @fn    void vLoRaOnReceiveMsg(int swPacketSize)
 * 
 * @brief This function is a callback that is called any time
 *        the LoRa radio receives a message.
 * 
 * @param swPacketSize - size of the LoRa packet. 
 */
extern void gvAPP_LoRa_onReceive(int16_t swPacketSize);


extern void gvAPP_LoRa_onTxDone(void);

#endif  /* #ifndef APP_LORA_MODULE */