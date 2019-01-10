// Authors: Ming Cheng, Joseph Kong


// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"

#include "pin_mux_config.h"

#define APPLICATION_VERSION     "1.1.1"
#define APP_NAME             "GPIO"

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
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
static void BoardInit(void);
static void DisplayBanner(char * AppName);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************

//*****************************************************************************

static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t        CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
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
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//! This function  
//!    implements polling from SW2 and SW3
//!    SW2 for LEDs on and off in unison
//!    SW3 for binary counting on the LEDs
//!
//! \return None.
//
//****************************************************************************
int
main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();
    
    //
    // Power on the corresponding GPIO port B for 9,10,11.
    // Set up the GPIO lines to mode 0 (GPIO)
    //
    PinMuxConfig();

    // Initializing the Terminal.
    //
    InitTerm();
    //
    // Clearing the Terminal.
    //
    ClearTerm();
    DisplayBanner(APP_NAME);
    Message("\t\t****************************************************\n\r");
    Message("\t\t        Push SW3 to start LED binary counting\n\r");
    Message("\t\t        Push SW2 to blink LEDs on and off\n\r");
    Message("\t\t****************************************************\n\r");

    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_ALL_LED_IND);

    int mode = 0;
    int count = 0;

    while(1)
    {
        //polling SW3
        if((GPIOPinRead(GPIOA1_BASE, 0x20) & 0x20) && (mode == 0 || mode == 1))
        {
            Message("SW3 pressed\n\r");
            printf("SW3 pressed\n\r");
            // set P18 low
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0);
            count = 0;
            mode = 2;
        }

        //polling SW2
        if((GPIOPinRead(GPIOA2_BASE, 0x40) & 0x40) && (mode == 0 || mode == 2))
        {
            Message("SW2 pressed\n\r");
            printf("SW2 pressed\n\r");
            // set P18 high
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);
            count = 0;
            mode = 1;
        }


        //blinking: mode 1
        if (mode == 1)
        {
            // set P18 high
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);

            // use modulus as a flag

            // on when modulus = 1
            if(count % 2 == 1)
                GPIO_IF_LedOn(MCU_ALL_LED_IND);

            // off when modulus = 0
            else
                GPIO_IF_LedOff(MCU_ALL_LED_IND);
            // increase counter
            count++;
            if(count > 7)
                count = 0;
            // delays
             MAP_UtilsDelay(4000000);
        }

        //binary counting: mode 2

        else if(mode == 2)
        {
            // set P18 low
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0);

            // reset LEDs low
            GPIO_IF_LedOff(MCU_ALL_LED_IND);

            // turn LEDs based on counter using addresses
            if(count > 0)
                GPIOPinWrite(GPIOA1_BASE, 0x2 + (count - 1) * 2, 0x2 + (count - 1) * 2);
            // increase counter
            count++;
            if(count > 7)
                count = 0;
            // delays
             MAP_UtilsDelay(4000000);
        }
    }
}
