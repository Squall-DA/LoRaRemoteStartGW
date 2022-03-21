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

#ifndef LORA_SCK
    #define LORA_SCK    5   // GPIO5 - SX1276 SCK
    #define LORA_MISO   19  // GPIO19 - SX1276 MISO
    #define LORA_MOSI   27  // GPIO27 - SX1276 MOSI
    #define LORA_CS     18  // GPIO18 - SX1276 CS
    #define LORA_RST    12  // GPIO14 - SX1276 RST
    #define LORA_IRQ    26  // GPIO26 - SX1276 IRQ (interrupt request)
#endif

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
        LoraApp(uint8_t ubCsPin = LORA_CS,uint8_t ubIrqPin = LORA_IRQ, uint8_t ubRstPin = LORA_RST, uint8_t ubDio1Pin = 255U,uint8_t ubSckPin = LORA_SCK, uint8_t ubMosiPin = LORA_MOSI, uint8_t ubMisoPin = LORA_MISO);
        void vTxMode();
        void vRxMode();
        void vSendMessage(String message);

    private:
        SX1276 radio;
};
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