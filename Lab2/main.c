// Standard includes
#include <string.h>
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "test.h"
#include "hw_apps_rcm.h"
#include "gpio.h"
#include "utils.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"

#include "pin_mux_config.h"


#define APPLICATION_VERSION     "1.1.1"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
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

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
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

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    // Reset SPI
    MAP_SPIReset(GSPI_BASE);

    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MasterMain();
    fillScreen(BLACK);
    while(1)
    {
        printf("Testing testfullchar():\n");
        testfullchar();
        printf("Finish testing testfullchar\n");
        fillScreen(BLACK);
        delay(100);

        printf("Testing testhelloworld():\n");
        testhelloworld();
        printf("Finish testing testhelloworld()\n");
        fillScreen(BLACK);
        delay(100);

        printf("Testing testhorizontal():\n");
        testhorizontal();
        printf("Finish testing testhorizontal()\n");
        fillScreen(BLACK);
        delay(100);

        printf("Testing testvertical():\n");
        testvertical();
        printf("Finish testing testvertical()\n");
        fillScreen(BLACK);
        delay(100);

        printf("Testing testlines():\n");
        testlines(BLUE);
        printf("Finish testing testlines()\n");
        fillScreen(BLACK);
        delay(100);

        printf("Testing testfastlines():\n");
        testfastlines(MAGENTA, RED);
        printf("Finish testing testfastlines()\n");
        fillScreen(BLACK);
        delay(100);

        printf("Testing testdrawrects():\n");
        testdrawrects(YELLOW);
        printf("Finish testing testdrawrects()\n");
        fillScreen(BLACK);
        delay(100);

        printf("Testing testfillrects():\n");
        testfillrects(CYAN, GREEN);
        printf("Finish testing testfillrects()\n");
        fillScreen(BLACK);
        delay(100);

        printf("Testing testfillcircles():\n");
        testfillcircles(10, BLUE);
        printf("Finish testing testfillcircles()\n");
        fillScreen(BLACK);
        delay(100);

        fillScreen(BLACK);
        testdrawcircles(12, RED);
        printf("Finish testing testdrawcircles()\n");
        fillScreen(BLACK);
        delay(100);

        printf("Testing testroundrects():\n");
        testroundrects();
        printf("Finish testing testroundrects()\n");
        fillScreen(BLACK);
        delay(100);

        printf("Testing testtriangles():\n");
        testtriangles();
        printf("Finish testing testtriangles()\n");
        fillScreen(BLACK);
        delay(100);
    }
}
