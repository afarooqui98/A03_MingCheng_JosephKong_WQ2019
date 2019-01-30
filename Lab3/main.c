
//*****************************************************************************
//
// Application Name     - int_sw
// Application Overview - The objective of this application is to demonstrate
//							GPIO interrupts using SW2 and SW3.
//							NOTE: the switches are not debounced!
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup int_sw
//! @{
//
//****************************************************************************

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
#include "uart_if.h"

#include "pin_mux_config.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#define ZERO  0x08f7
#define ONE   0x8877
#define TWO   0x48b7
#define THREE 0xc837
#define FOUR  0x28d7
#define FIVE  0xa857
#define SIX   0x6897
#define SEVEN 0xe817
#define EIGHT 0x18e7
#define NINE  0x9867
#define ENTER 0x22dd
#define LAST  0x58a7

extern void (* const g_pfnVectors[])(void);

volatile unsigned long PIN61_intcount;
volatile unsigned char PIN61_intflag;


//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting PIN61 = { .port = GPIOA0_BASE, .pin = 0x40}; // GPIOPIN61 for IR_OUT

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************
static void GPIOA0IntHandler(void) {	// PIN61 handler
	unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (PIN61.port, true);
    MAP_GPIOIntClear(PIN61.port, ulStatus);		// clear interrupts on GPIOA0
    PIN61_intcount++;
    PIN61_intflag=1;
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
BoardInit(void) {
	MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    
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
//!
//! \return None.
//
//****************************************************************************
int main() {
	unsigned long ulStatus;

    BoardInit();
    
    PinMuxConfig();
    
    InitTerm();

    ClearTerm();

    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(PIN61.port, GPIOA0IntHandler);

    //
    // Configure falling edge interrupts on Pin61 from IR output (remote)
    //
    MAP_GPIOIntTypeSet(PIN61.port, PIN61.pin, GPIO_FALLING_EDGE);	// PIN61

    ulStatus = MAP_GPIOIntStatus (PIN61.port, false);
    MAP_GPIOIntClear(PIN61.port, ulStatus);			// clear interrupts on GPIOA0

    // clear global variables
    PIN61_intcount = 0;
    PIN61_intflag = 0;

    // Enable PIN61 interrupts
    MAP_GPIOIntEnable(PIN61.port, PIN61.pin);


    Message("\t****************************************************\n\r");
    Message("\tPress a button on the Remote to see which button you pressed\n\r");
    Message("\t****************************************************\n\r");
    Message("\n\n\n\r");
	Report("PIN61 ints = %d\r\n", PIN61_intcount);

    while (1) {
    	while (PIN61_intflag == 0) {;}
    	if (PIN61_intflag) {
    		PIN61_intflag = 0;	// clear flag
    		Report("PIN61 ints = %d\r\n", PIN61_intcount);
    	}
    }
}
