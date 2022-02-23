/** 
 *  @file                   main.h 
 *  @brief                  main header file
 *  @copyright              2017 Squall-DA
 *  @date                   12/04/2017
 *  
 *  @remark Author:         Squall-DA
 *  @remark Project Tree:   LoRaRemoteStartGW
 *  
 */
#ifndef MAIN_MODULE
#define MAIN_MODULE 1


/*========================================================================* 
 *  SECTION - Global definitions 
 *========================================================================* 
 */
/* Starter pin connections */
#define START_PIN    25
#define UNLOCK_PIN   32
#define LOCK_PIN     33

/* LoRa Pin connections */
#define LoRa_SCK     5    // GPIO5  -- SX1278's SCK
#define LoRa_MISO    19   // GPIO19 -- SX1278's MISO
#define LoRa_MOSI    27   // GPIO27 -- SX1278's MOSI
#define LoRa_CS      18   // GPIO18 -- SX1278's CS
#define LoRa_RST     23  // GPIO14 -- SX1278's RESET
#define LoRa_DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define LoRa_BAND    904E6

#define UUID_LENGTH  16  /* Length of UUIDs in bytes */
#define LoRa_CMD_START UUID_LENGTH /* LoRa command starts at byte UUID_LENGTH (count from 0) */

typedef enum
{   
    LoRa_VEH_CMD_START = 1
    ,LoRa_VEH_CMD_LOCK
    ,LoRa_VEH_CMD_UNLOCK
}LoRa_VEHICLE_CMDS;

/*========================================================================* 
 *  SECTION - extern global variables (minimize global variable use)      * 
 *========================================================================* 
 */

/*========================================================================* 
 *  SECTION - extern global functions                                     * 
 *========================================================================* 
 */


#endif  /* #ifndef MAIN_MODULE */