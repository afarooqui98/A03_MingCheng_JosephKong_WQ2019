// Author: Ming Cheng, Joseph Kong
#include <stdio.h>
#include <stdbool.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "rom.h"
#include "rom_map.h"

#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "pin.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "spi.h"
#include "uart.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "timer_if.h"
#include "timer.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "test.h"

//Common interface includes
#include "pin_mux_config.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"

#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1


#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Spring2018"
#define SERVER_NAME             "a18y0m5hbm22k5-ats.iot.us-west-2.amazonaws.com"
#define GOOGLE_DST_PORT         8443

#define SL_SSL_CA_CERT "/cert/rootCA.der" //starfield class2 rootca (from firefox) // <-- this one works
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                26    /* Current Date */
#define MONTH               2     /* Month 1-12 */
#define YEAR                2019  /* Current year */
#define HOUR                10    /* Time - hours */
#define MINUTE              39    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define SPI_IF_BIT_RATE  100000
#define MAX_BUFFER  80

#define ZERO  0x08f7 //0000 1000 1111 0111
#define ONE   0x8877 //1000 1000 0111 0111
#define TWO   0x48b7 //0100 1000 1011 0111
#define THREE 0xc837 //1100 1000 0011 0111
#define FOUR  0x28d7 //0010 1000 1101 0111
#define FIVE  0xa857 //1001 1000 0101 0111
#define SIX   0x6897 //0110 1000 1001 0111
#define SEVEN 0xe817 //1110 1000 0001 0111
#define EIGHT 0x18e7 //0001 1000 1110 0111
#define NINE  0x9867 //1001 1000 0110 0111
#define ENTER 0x22dd //0010 0010 1101 1101
#define MUTE  0x906f //1001 0000 0110 1111
#define LAST  0x58a7 //0101 1000 1001 0111

#define Player 1

extern void (* const g_pfnVectors[])(void);

volatile long PIN8_intcount;
volatile unsigned char PIN8_intflag;

volatile unsigned long tmp;
volatile static tBoolean bRxDone;

volatile long currentButton;

char message[MAX_BUFFER];
char player1[MAX_BUFFER] = "15x15";
char player2[MAX_BUFFER] = "15x15";
unsigned long buffer[1000]; // buffer to store IR Data

#define POSTHEADER "POST /things/CC3200_Thing/shadow HTTP/1.1\n\r"
#define GETHEADER "GET /things/CC3200_Thing/shadow HTTP/1.1\n\r"
#define HOSTHEADER "Host: a18y0m5hbm22k5-ats.iot.us-west-2.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

char DATA1[100] = "{\"state\": {\r\n\"desired\" : {\r\n\"player1\" : \"";
char DATA2[100] = "\",\r\n\"player2\" : \"";
char DATA3[100] = "\"\r\n}}}\r\n\r\n";

bool yourTurn;
int Game[15][15];

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;

typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

typedef struct Letter {
    unsigned int x;
    unsigned int y;
    char letter;
} Letter;

typedef struct Position {
   unsigned int x;
   unsigned int y;
} Position;

static PinSetting PIN8  = { .port = GPIOA2_BASE, .pin = 0x2 }; // GPIOPIN8 for IR_OUT

Position Players[2];
Position PlayersMove[2];
int PlayerIndex;
int PlayerColor;

static void BoardInit(void);
unsigned long Decode(unsigned long* buffer);
void MasterMain();
void MoveCursor(unsigned long value, long lRetVal);
void ProcessIR(long lRetVal);

bool CheckHorizontal(int row, int col, int index);
bool CheckVertical(int row, int col, int index);
bool CheckRightDiagonal(int row, int col, int index);
bool CheckLeftDiagonal(int row, int col, int index);
bool EndGame(int row, int col, int index, long lRetVal);

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int);
static int http_get(int);

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE: 
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n", 
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default: 
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End breadcrumb: s18_df
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}




long printErrConvenience(char * msg, long retVal) {
    UART_PRINT(msg);
    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}


//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP;
//    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256;
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_MD5
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_DHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_TLS_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256 // does not work (-340, handshake fails)
    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket 
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }



/////////////////////////////////
// START: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
// END: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
/////////////////////////////////


    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal >= 0) {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal == SL_ESECSNOVERIFY) {
        UART_PRINT("Device has connected to the website (UNVERIFIED):");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal < 0) {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}



int connectToAccessPoint() {
    long lRetVal = -1;
    GPIO_IF_LedConfigure(LED1|LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    UART_PRINT("Opening sl_start\n\r");
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

static void GPIOA0IntHandler(void) {    // PIN61 handler
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (PIN8.port, true);
    MAP_GPIOIntClear(PIN8.port, ulStatus);     // clear interrupts on GPIOA0

    PIN8_intcount++;

    // wave form counter for a button pressed
    if(PIN8_intcount == 36) {
        PIN8_intflag = 1;
        PIN8_intcount = 0;

    }

    // convert to 0 or 1 for waveforms
    tmp = TimerValueGet(TIMERA0_BASE, TIMER_A) >> 17;

    // when still pressing reset to zero
    if(tmp == 58 || tmp == 59) {
        PIN8_intcount = -1;
        PIN8_intflag = 1;

    }

    buffer[PIN8_intcount] = tmp;

    TimerValueSet(TIMERA0_BASE, TIMER_A, 0);
}

unsigned long Decode(unsigned long* buffer) {
    unsigned long value = 0;
    int i;
    for(i = 0; i < 16; i++) {
        value += *(buffer + i) << (15 - i);
    }
    return value;
}

void MoveCursor(unsigned long value, long lRetVal) {
    switch(value) {
        case TWO:
             if(Players[PlayerIndex].y > 0) {
                 drawRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, WHITE);
                 Players[PlayerIndex].y = Players[PlayerIndex].y - 8;
                 drawRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, PlayerColor);
            }
            break;
        case FOUR:
            if(Players[PlayerIndex].x > 0) {
                 drawRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, WHITE);
                 Players[PlayerIndex].x = Players[PlayerIndex].x - 8;
                 drawRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, PlayerColor);
            }
            break;
        case FIVE:
            if(Game[Players[PlayerIndex].x / 8][Players[PlayerIndex].y / 8] == -1) {
                fillRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, PlayerColor);
                Game[Players[PlayerIndex].x / 8][Players[PlayerIndex].y / 8] = PlayerIndex;
                PlayersMove[PlayerIndex].x = Players[PlayerIndex].x / 8;
                PlayersMove[PlayerIndex].y = Players[PlayerIndex].y / 8;

                if(Player == 1) {
                    memset(player1, 0, strlen(player1));
                    if(Players[PlayerIndex].x / 8 > 9) {
                        player1[0] = '1';
                        player1[1] = Players[PlayerIndex].x / 8 - 10 + '0';
                    }
                    else {
                        player1[0] = '0';
                        player1[1] = Players[PlayerIndex].x / 8 + '0';
                    }
                    player1[2] = 'x';
                    if(Players[PlayerIndex].y / 8 > 9) {
                        player1[3] = '1';
                        player1[4] = Players[PlayerIndex].y / 8 - 10 + '0';
                    }
                    else {
                        player1[3] = '0';
                        player1[4] = Players[PlayerIndex].y / 8 + '0';
                    }
                }
                else {
                    memset(player2, 0, strlen(player2));
                    if(Players[PlayerIndex].x / 8 > 9) {
                        player2[0] = '1';
                        player2[1] = Players[PlayerIndex].x / 8 - 10 + '0';
                    }
                    else {
                        player2[0] = '0';
                        player2[1] = Players[PlayerIndex].x / 8 + '0';
                    }
                    player2[2] = 'x';
                    if(Players[PlayerIndex].y / 8 > 9) {
                        player2[3] = '1';
                        player2[4] = Players[PlayerIndex].y / 8 - 10 + '0';
                    }
                    else {
                        player2[3] = '0';
                        player2[4] = Players[PlayerIndex].y / 8 + '0';
                    }
                }
                http_post(lRetVal);
                yourTurn = false;

                // check if game ends
                if(EndGame(PlayersMove[PlayerIndex].y, PlayersMove[PlayerIndex].x, PlayerIndex, lRetVal)) {
                    UART_PRINT("You Win!\n\r");
                }
            }
            break;
        case SIX:
            if(Players[PlayerIndex].x < 112) {
                drawRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, WHITE);
                Players[PlayerIndex].x = Players[PlayerIndex].x + 8;
                drawRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, PlayerColor);
            }
            break;
        case EIGHT:
            if(Players[PlayerIndex].y < 112) {
                drawRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, WHITE);
                Players[PlayerIndex].y = Players[PlayerIndex].y + 8;
                drawRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, PlayerColor);
            }
            break;
        default:
            break;
    }
}

void MasterMain()
{
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Initialize Adafruit
    Adafruit_Init();
}

void ProcessIR(long lRetVal) {
    if (PIN8_intflag) {
        PIN8_intflag = 0;  // clear flag
        currentButton = Decode(buffer + 19);

        // process input from user
        MoveCursor(currentButton, lRetVal);
    }
}

bool CheckHorizontal(int row, int col, int index) {
    int i, j;
    for(i = -4; i < 1; i++) {
        int num = 0;
        for(j = col + i; j < col + i + 5; j++) {
            // out of bounds
            if(j < 0 || j > 112) {
                break;
            }
            else if(Game[j][row] == index) {
                num++;
            }
        }

        if(num == 5) {
            UART_PRINT("Horizontal Win!\n\r");
            return true;
        }
    }
    return false;
}

bool CheckVertical(int row, int col, int index) {
    int i, j;
    for(i = -4; i < 1; i++) {
        int num = 0;
        for(j = row + i; j < row + i + 5; j++) {
            // out of bounds
            if(j < 0 || j > 112) {
                break;
            }
            else if(Game[col][j] == index) {
                num++;
            }
        }

        if(num == 5) {
            UART_PRINT("Vertical Win!\n\r");
            return true;
        }
   }
   return false;
}

bool CheckRightDiagonal(int row, int col, int index) {
    int i, j;
    for(i = -4; i < 1; i++) {
        int num = 0;
        for(j = 0; j < 5; j++) {
            int x = col + i + j;
            int y = row + i + j;
            // out of bounds
            if(x < 0 || x > 112 || y < 0 || y > 112) {
                break;
            }
            else if(Game[x][y] == index) {
                num++;
            }
        }

        if(num == 5) {
            UART_PRINT("Right Diagonal Win!\n\r");
            return true;
        }
   }
   return false;
}

bool CheckLeftDiagonal(int row, int col, int index) {
    int i, j;
    for(i = -4; i < 1; i++) {
        int num = 0;
        for(j = 0; j < 5; j++) {
            int x = col + i + j;
            int y = row - i - j;
            // out of bounds
            if(x < 0 || x > 112 || y < 0 || y > 112) {
                break;
            }
            else if(Game[x][y] == index) {
                num++;
            }
        }

        if(num == 5) {
            UART_PRINT("Left Diagonal Win!\n\r");
            return true;
        }
   }
   return false;
}

bool EndGame(int row, int col, int index, long lRetVal) {
    if(CheckHorizontal(row, col, index) || CheckVertical(row, col, index) ||
           CheckRightDiagonal(row, col, index) || CheckLeftDiagonal(row, col, index))
    {
        // reset player data in web
        memset(player1, 0, strlen(player1));
        player1[0] = '1';
        player1[1] = '5';
        player1[2] = 'x';
        player1[3] = '1';
        player1[4] = '5';
        player2[0] = '1';
        player2[1] = '5';
        player2[2] = 'x';
        player2[3] = '1';
        player2[4] = '5';

        if(Player == 1) {
            http_post(lRetVal);
            yourTurn = true;
        }
        else {
            yourTurn = false;
        }

        // reset player move
        int i;
        for(i = 0; i < 2; i++) {
            PlayersMove[i].x = 15;
            PlayersMove[i].y = 15;
            Players[i].x = 56;
            Players[i].y = 56;
        }

        // reset game matrix
        // initialize game matrix
        int row;
        int col;
        for(col = 0; col < 15; col++) {
            for(row = 0; row < 15; row++) {
                Game[col][row] = -1;
            }
        }

        // clean screen
        fillScreen(BLACK);

        // draw the connect-5 grid: 15x15
        drawGrid();

        // draw cursor
        drawRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, PlayerColor);

        return true;
    }
    return false;

}

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void main() {
    // initialize players
    Position Player1 = {.x = 56, .y = 56};
    Position Player2 = {.x = 56, .y = 56};
    Position Player1Move = {.x = 15, .y = 15};
    Position Player2Move = {.x = 15, .y = 15};
    Players[0] = Player1;
    Players[1] = Player2;
    PlayersMove[0] = Player1Move;
    PlayersMove[1] = Player2Move;
    if(Player == 1) {
        PlayerColor = BLUE;
        PlayerIndex = 0;
        yourTurn = true;
    }
    else {
        PlayerColor = RED;
        PlayerIndex = 1;
        yourTurn = false;
    }

    // initialize game matrix
    int row;
    int col;
    for(col = 0; col < 15; col++) {
        for(row = 0; row < 15; row++) {
            Game[col][row] = -1;
        }
    }


    long lRetVal = -1;
    unsigned long ulStatus;

    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    InitTerm();
    ClearTerm();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    // Reset SPI
    MAP_SPIReset(GSPI_BASE);

    MAP_PRCMPeripheralReset(PRCM_GSPI);

    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(PIN8.port, GPIOA0IntHandler);

    // Configure falling edge interrupts on Pin61 from IR output (remote)
    //
    MAP_GPIOIntTypeSet(PIN8.port, PIN8.pin, GPIO_FALLING_EDGE);   // PIN61

    ulStatus = MAP_GPIOIntStatus (PIN8.port, false);
    MAP_GPIOIntClear(PIN8.port, ulStatus);         // clear interrupts on GPIOA0

    // Enable PIN61 interrupts
    MAP_GPIOIntEnable(PIN8.port, PIN8.pin);;

    // set up Timer interrupt
    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC_UP, TIMER_A, 0);
    TimerEnable(TIMERA0_BASE, TIMER_A);
    TimerValueSet(TIMERA0_BASE, TIMER_A, 0);

    // clear global variables
    PIN8_intcount = 0;
    PIN8_intflag = 0;

    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    //Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }

    http_post(lRetVal);

    MasterMain();
    fillScreen(BLACK);

    // draw the connect-5 grid: 15x15
    drawGrid();

    // main for loop
    drawRect(Players[PlayerIndex].x, Players[PlayerIndex].y, 9, 9, PlayerColor);
    while(1) {
        if(yourTurn) {
            ProcessIR(lRetVal);
        }
        else {
            http_get(lRetVal);
        }
    }
}

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char buffer[1000];
    char* pcBufHeaders;
    int lRetVal = 0;

    strcpy(buffer, DATA1);

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(DATA1) + strlen(DATA2) + strlen(DATA3) + strlen(player1) + strlen(player2);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcat(buffer, player1);
    strcat(buffer, DATA2);
    strcat(buffer, player2);
    strcat(buffer, DATA3);

    strcpy(pcBufHeaders, buffer);
    pcBufHeaders += strlen(buffer);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);

    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

static int http_get(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);

    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';

        int i, j;
        j = 0;
        for(i = 0; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;

        char data1[100];
        char data2[100];
        char x1[2], x2[2], y1[2], y2[2];
        int data_x1 = 0, data_x2 = 0, data_y1 = 0, data_y2 = 0;
        for(; acRecvbuff[i] != ','; i++) {
            data1[j] = acRecvbuff[i];
            j++;
        }
        data1[++j] = '\0';

        for(i = 1; data1[i] != 'x'; i++) {
            x1[i - 1] = data1[i];
        }
        data_x1 = (x1[0] - '0') * 10 + (x1[1] - '0');;
        UART_PRINT("x1: %d\n\r", data_x1);

        int k = 0;
        i++;
        for(; data1[i] != '\0'; i++) {
           y1[k] = data1[i];
           k++;
        }
        data_y1 = (y1[0] - '0') * 10 + (y1[1] - '0');;
        UART_PRINT("y1: %d\n\r", data_y1);

        j = 0;
        i = 0;
        for(i = 0; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;
        for(; acRecvbuff[i] != ':'; i++);
        i++;

        for(; acRecvbuff[i] != ','; i++) {
            data2[j] = acRecvbuff[i];
            j++;
        }

        data2[++j] = '\0';
        for(i = 1; data2[i] != 'x'; i++) {
            x2[i - 1] = data2[i];
        }
        data_x2 = (x2[0] - '0') * 10 + (x2[1] - '0');;
        UART_PRINT("x2: %d\n\r", data_x2);

        k = 0;
        i++;
        for(; data2[i] != '\0'; i++) {
           y2[k] = data2[i];
           k++;
        }
        data_y2 = (y2[0] - '0') * 10 + (y2[1] - '0');;
        UART_PRINT("y2: %d\n\r", data_y2);

        if(Player == 1) {
            if(!(PlayersMove[1].x == data_x2 && PlayersMove[1].y == data_y2)) {
                PlayersMove[1].x = data_x2;
                PlayersMove[1].y = data_y2;

                yourTurn = true;
                UART_PRINT("Your Turn!\n\r");
                Game[PlayersMove[1].x][PlayersMove[1].y] = 1;
                fillRect(PlayersMove[1].x * 8, PlayersMove[1].y * 8, 9, 9, RED);
                memset(player2, 0, strlen(player2));
                if(PlayersMove[1].x > 9) {
                    player2[0] = '1';
                    player2[1] = PlayersMove[1].x - 10 + '0';
                }
                else {
                    player2[0] = '0';
                    player2[1] = PlayersMove[1].x + '0';
                }
                player2[2] = 'x';
                if(PlayersMove[1].y > 9) {
                    player2[3] = '1';
                    player2[4] = PlayersMove[1].y - 10 + '0';
                }
                else {
                    player2[3] = '0';
                    player2[4] = PlayersMove[1].y + '0';
                }
                // check if game ends
                if(EndGame(PlayersMove[1].y, PlayersMove[1].x, 1, lRetVal)) {
                    UART_PRINT("You Lose!\n\r");
                }
            }
        }
        else {
            if(!(PlayersMove[0].x == data_x1 && PlayersMove[0].y == data_y1)) {
                PlayersMove[0].x = data_x1;
                PlayersMove[0].y = data_y1;
                yourTurn = true;
                UART_PRINT("Your Turn!\n\r");
                Game[PlayersMove[0].x][PlayersMove[0].y] = 0;
                fillRect(PlayersMove[0].x * 8, PlayersMove[0].y * 8, 9, 9, BLUE);
                memset(player1, 0, strlen(player1));
                if(PlayersMove[0].x > 9) {
                    player1[0] = '1';
                    player1[1] = PlayersMove[0].x - 10 + '0';
                }
                else {
                    player1[0] = '0';
                    player1[1] = PlayersMove[0].x + '0';
                }
                player1[2] = 'x';
                if(PlayersMove[0].y > 9) {
                    player1[3] = '1';
                    player1[4] = PlayersMove[0].y - 10 + '0';
                }
                else {
                    player1[3] = '0';
                    player1[4] = PlayersMove[0].y + '0';
                }
                // check if game ends
                if(EndGame(PlayersMove[0].y, PlayersMove[0].x, 0, lRetVal)) {
                    UART_PRINT("You Lose!\n\r");
                }
            }
        }
    }

    return 0;
}
