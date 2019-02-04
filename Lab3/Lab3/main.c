// Authors: Ming Cheng, Joseph Kong
// Standard includes
#include <stdio.h>
#include <stdbool.h>

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
#include "timer_if.h"
#include "timer.h"

// Common interface includes
#include "uart_if.h"

#include "pin_mux_config.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

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

extern void (* const g_pfnVectors[])(void);

volatile long PIN61_intcount;
volatile unsigned long tmp;
volatile unsigned char PIN61_intflag;
volatile static tBoolean bRxDone;
volatile long currentPress;
volatile long previousPress;
volatile long currentButton;
volatile long previousButton;
unsigned long buffer[1000];
int sameButton = 0;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting PIN61 = { .port = GPIOA0_BASE, .pin = 0x40}; // GPIOPIN61 for IR_OUT

static void BoardInit(void);

static void GPIOA0IntHandler(void) {    // PIN61 handler
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (PIN61.port, true);
    MAP_GPIOIntClear(PIN61.port, ulStatus);     // clear interrupts on GPIOA0

    PIN61_intcount++;

    // wave form counter for a button pressd
    if(PIN61_intcount == 36) {
        PIN61_intflag = 1;
        PIN61_intcount = 0;
        Timer_IF_Start(TIMERA0_BASE, TIMER_A, 500);
    }

    // convert to 0 or 1 for waveforms
    tmp = TimerValueGet(TIMERA1_BASE, TIMER_A) >> 17;

    // when still pressing reset to zero
    if(tmp == 58) {
        PIN61_intcount = -1;
        PIN61_intflag = 1;
        Timer_IF_Start(TIMERA0_BASE, TIMER_A, 500);
    }

    buffer[PIN61_intcount] = tmp;
    TimerValueSet(TIMERA1_BASE, TIMER_A, 0);
}

static void ConsecutivePressingHandler(void)
{
    Timer_IF_InterruptClear(TIMERA0_BASE);
    currentPress++;
}

unsigned long Decode(unsigned long* buffer) {
    unsigned long value = 0;
    int i;
    for(i = 0; i < 16; i++) {
        value += *(buffer + i) << (15 - i);
    }
    return value;
}

void Display(unsigned long value) {
    switch(value) {

        case ZERO:
            Report("You pressed 0.\n\r");
            break;
        case ONE:
            Report("You pressed 1.\n\r");
            break;
        case TWO:
            Report("You pressed 2.\n\r");
            break;
        case THREE:
            Report("You pressed 3.\n\r");
            break;
        case FOUR:
            Report("You pressed 4.\n\r");
            break;
        case FIVE:
            Report("You pressed 5.\n\r");
            break;
        case SIX:
            Report("You pressed 6.\n\r");
            break;
        case SEVEN:
            Report("You pressed 7.\n\r");
            break;
        case EIGHT:
            Report("You pressed 8.\n\r");
            break;
        case NINE:
            Report("You pressed 9.\n\r");
            break;
        case ENTER:
            Report("You pressed Enter.\n\r");
            break;
        case MUTE:
            Report("You pressed MUTE.\n\r");
            break;
        case LAST:
            Report("You pressed LAST.\n\r");
            break;
    }
}

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

    // Configure falling edge interrupts on Pin61 from IR output (remote)
    //
    MAP_GPIOIntTypeSet(PIN61.port, PIN61.pin, GPIO_FALLING_EDGE);   // PIN61

    ulStatus = MAP_GPIOIntStatus (PIN61.port, false);
    MAP_GPIOIntClear(PIN61.port, ulStatus);         // clear interrupts on GPIOA0

    // Enable PIN61 interrupts
    MAP_GPIOIntEnable(PIN61.port, PIN61.pin);;


    // set up Timer interrupt
    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_ONE_SHOT, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, ConsecutivePressingHandler);

    TimerConfigure(TIMERA1_BASE, TIMER_CFG_PERIODIC_UP);
    TimerEnable(TIMERA1_BASE, TIMER_A);
    TimerValueSet(TIMERA1_BASE, TIMER_A, 0);

    // clear global variables
    PIN61_intcount = 0;
    PIN61_intflag = 0;
    currentPress = 0;
    previousPress = 1;
    currentButton = -2;
    previousButton = -1;

    // Display banner on UART
    Message("\t****************************************************\n\r");
    Message("\tPress a button on the Remote to see which button you pressed\n\r");
    Message("\t****************************************************\n\r");
    Message("\n\n\n\r");

    // main for loop
    while (1) {
        while (PIN61_intflag == 0) {
            sameButton = 0;
        }
        if (PIN61_intflag) {
            PIN61_intflag = 0;  // clear flag
            currentButton = Decode(buffer + 19);
            Display(currentButton);
            if(previousButton == currentButton) {
                sameButton = 1;
            }
            if(previousPress == currentPress && sameButton) {
                Report("Pressing the same button consecutively \n\r");
            }
            previousPress = currentPress;
            previousButton = currentButton;
        }
    }
}
