#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_epi.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/epi.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/udma.h"
#include "utils/ustdlib.h"

// define part number for pin_map.h
#define PART_TM4C1294KCPDT

// define epi port pins to be used
#define EPI_PORTA_PINS (GPIO_PIN_7 | GPIO_PIN_6)
#define EPI_PORTC_PINS (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4)
#define EPI_PORTG_PINS (GPIO_PIN_1)
#define EPI_PORTK_PINS (GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1)

//*****************************************************************************
//
// The size of the memory transfer source and destination buffers (in words).
//
//*****************************************************************************
#define MEM_BUFFER_SIZE         1024

//*****************************************************************************
//
// The source and destination buffers used for memory transfers.
//
//*****************************************************************************
static uint32_t g_ui32SrcBuf[MEM_BUFFER_SIZE];
static uint32_t g_ui32DstBuf[MEM_BUFFER_SIZE];

//*****************************************************************************
//
// The count of memory uDMA transfer blocks.  This value is incremented by the
// uDMA interrupt handler whenever a memory block transfer is completed.
//
//*****************************************************************************
static uint32_t g_ui32MemXferCount = 0;

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// global variables for generating "random" signal
uint32_t i = 0;
uint8_t rand[10];
uint8_t F1, F2, F3, G0, L4, L5, L0, L1, L2, L3;

//*****************************************************************************
//
// The count of uDMA errors.  This value is incremented by the uDMA error
// handler.
//
//*****************************************************************************
static uint32_t g_ui32uDMAErrCount = 0;

//*****************************************************************************
//
// The count of times the uDMA interrupt occurred but the uDMA transfer was not
// complete.  This should remain 0.
//
//*****************************************************************************
static uint32_t g_ui32BadISR = 0;

//*****************************************************************************
//
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
    uint32_t ui32Status;

    //
    // Check for uDMA error bit
    //
    ui32Status = ROM_uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status)
    {
        ROM_uDMAErrorStatusClear();
        g_ui32uDMAErrCount++;
    }
}

//*****************************************************************************
//
// The interrupt handler for uDMA interrupts from the memory channel.  This
// interrupt will increment a counter, and then restart another memory
// transfer.
//
//*****************************************************************************
void
uDMAIntHandler(void)
{
    uint32_t ui32Mode;

    //
    // Check for the primary control structure to indicate complete.
    //
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_SW);
    if(ui32Mode == UDMA_MODE_STOP)
    {
        //
        // Increment the count of completed transfers.
        //
        g_ui32MemXferCount++;

        //
        // Configure it for another transfer.
        //
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SW, UDMA_MODE_AUTO,
                                     g_ui32SrcBuf, g_ui32DstBuf,
                                     MEM_BUFFER_SIZE);

        //
        // Initiate another transfer.
        //
        ROM_uDMAChannelEnable(UDMA_CHANNEL_SW);
        ROM_uDMAChannelRequest(UDMA_CHANNEL_SW);
    }

    //
    // If the channel is not stopped, then something is wrong.
    //
    else
    {
        g_ui32BadISR++;
    }
}

// Main program
int main(void) {
	
	uint32_t ui32Period;
	uint32_t ui32SysClkFreq;

	// Set Clock frequency to 120 MHz
	ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	// Enable GPIO and Timer peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	// Enable EPI Peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EPI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

	// Configure GPIO and Timer
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_0);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

	// Configure GPIO pins for EPI mode. Address obtained from pin_map.h
	GPIOPinConfigure(GPIO_PA6_EPI0S8);
	GPIOPinConfigure(GPIO_PA7_EPI0S9);
	GPIOPinConfigure(GPIO_PC4_EPI0S7);
	GPIOPinConfigure(GPIO_PC5_EPI0S6);
	GPIOPinConfigure(GPIO_PC6_EPI0S5);
	GPIOPinConfigure(GPIO_PC7_EPI0S4);
	GPIOPinConfigure(GPIO_PG1_EPI0S10);
	GPIOPinConfigure(GPIO_PK1_EPI0S1);
	GPIOPinConfigure(GPIO_PK2_EPI0S2);
	GPIOPinConfigure(GPIO_PK3_EPI0S3);

    GPIOPinTypeEPI(GPIO_PORTA_BASE, EPI_PORTA_PINS);
    GPIOPinTypeEPI(GPIO_PORTC_BASE, EPI_PORTC_PINS);
    GPIOPinTypeEPI(GPIO_PORTG_BASE, EPI_PORTG_PINS);
    GPIOPinTypeEPI(GPIO_PORTK_BASE, EPI_PORTK_PINS);

    // Select general purpose mode for EPI
    EPIModeSet(EPI0_BASE, EPI_MODE_GENERAL);

	// Set Timer A to have a period of one tenth the clock freq
	ui32Period = ui32SysClkFreq/2;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period/10-1);

	// Set GPIO's to have all zeroes
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0);
	GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5,0);
	GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0,0);

	// Enable timer interrupt and set it to go off at Timer A timeouts
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	// Enable Timer A
	TimerEnable(TIMER0_BASE, TIMER_A);



	while(1)
	{
	}

}

void Timer0IntHandler(void)
{
	// State variable to help change values
	i++;

	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Read the current state of the GPIO pin and
	// write back the opposite state
	if(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1))
	{
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
	}
	else
	{
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 2);
	}

	// Change PF1 every cycle
	if(i%1 == 0)
	{
		if(F1 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1))
		{
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 2);
		}
	}

	// Change PF2 every 2 cycles
	if(i%2 == 0)
	{
		if(F2 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
		{
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
		}
	}

	// Change PF3 every 3 cycles
	if(i%3 == 0)
	{
		if(F3 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3))
		{
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 8);
		}
	}

	// Change PG0 every 4 cyclces
	if(i%4 == 0)
	{
		if(G0 = GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_0))
		{
			GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 1);
		}
	}

	// Change PL4 every 5 cyclces
	if(i%5 == 0)
	{
		if(L4 = GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_4))
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 16);
		}
	}

	// Change PL5 every 6 cyclces
	if(i%6 == 0)
	{
		if(L5 = GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_5))
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5, 32);
		}
	}

	// Change PL0 every 7 cyclces
	if(i%7 == 0)
	{
		if(L0 = GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_0))
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 1);
		}
	}

	// Change PL1 every 8 cyclces
	if(i%8 == 0)
	{
		if(L1 = GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_1))
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 2);
		}
	}

	// Change PL2 every 9 cyclces
	if(i%9 == 0)
	{
		if(L2 = GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_2))
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 4);
		}
	}

	// Change PL3 every 10 cyclces
	if(i%10 == 0)
	{
		if(L3 = GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_3))
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 8);
		}
	}

	rand[0] = F1;
	rand[1] = F2;
	rand[2] = F3;
	rand[3] = G0;
	rand[4] = L4;
	rand[5] = L5;
	rand[6] = L0;
	rand[7] = L1;
	rand[8] = L2;
	rand[9] = L3;
}
