#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_epi.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/epi.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/udma.h"
#include "utils/ustdlib.h"

uint32_t ui32SysClkFreq;
uint32_t *EPIdata, *EPISource;
uint32_t test;
uint8_t EPIReadCount, EPIAvail;
uint8_t Start = 0;

// define epi port pins to be used
#define EPI_PORTA_PINS (GPIO_PIN_6)
#define EPI_PORTC_PINS (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4)
#define EPI_PORTG_PINS (GPIO_PIN_0)
#define EPI_PORTL_PINS (GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0)
#define EPI_PORTM_PINS (GPIO_PIN_3)

// define gpio port points to be used as outputs
#define OUTPUT_PORTD_PINS (GPIO_PIN_0 | GPIO_PIN_1)
#define OUTPUT_PORTE_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4)
#define OUTPUT_PORTF_PINS (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
#define OUTPUT_PORTH_PINS (GPIO_PIN_2 | GPIO_PIN_3)

// global variables for generating "random" signal
uint32_t i = 0, j = 0, f = 0;
uint32_t tri[12];
uint32_t receive[12], total;
uint32_t F1, F2, F3, H2, H3, D1, D0, E4, E0, E1, E2, E3;

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
static uint32_t *g_ui32DstBuf[MEM_BUFFER_SIZE];
uint32_t values[MEM_BUFFER_SIZE], inputs[MEM_BUFFER_SIZE];

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
    ui32Status = uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status)
    {
        uDMAErrorStatusClear();
        g_ui32uDMAErrCount++;
    }
}


////////////////////////////////////////////////////////
// EPI interrupt fuction
//////////////////////////////////////////////////
void
EPIIntHandler(void)
{
    uint32_t ui32Mode;

    //
    // Check for the primary control structure to indicate complete.
    //
    ui32Mode = EPIIntStatus(EPI0_BASE, true);
    if(ui32Mode == EPI_INT_RXREQ)
    {
        //
        // Increment the count of completed transfers.
        //
        g_ui32MemXferCount++;

        //
        // Configure it for another transfer.
        //
        uDMAChannelTransferSet(UDMA_CHANNEL_SW, UDMA_MODE_AUTO,
        						EPISource, g_ui32DstBuf[j],
        						1);

        //
        // Initiate another transfer.
        //
        uDMAChannelEnable(UDMA_CHANNEL_SW);
    }

    //
    // If the channel is not stopped, then something is wrong.
    //
    else
    {
        g_ui32BadISR++;
    }

    EPINonBlockingReadConfigure(EPI0_BASE,0,EPI_NBCONFIG_SIZE_32,0);
    EPINonBlockingReadStart(EPI0_BASE,0,1);

	receive[0] = inputs[j] & 0b00000000000000000000000000010000;
	receive[1] = inputs[j] & 0b00000000000000000000000000100000;
	receive[2] = inputs[j] & 0b00000000000000000000000001000000;
	receive[3] = inputs[j] & 0b00000000000000000000000010000000;
	receive[4] = inputs[j] & 0b00000000000000000000000100000000;
	receive[5] = inputs[j] & 0b00000000000000000000100000000000;
	receive[6] = inputs[j] & 0b00000000000000000001000000000000;
	receive[7] = inputs[j] & 0b00000000000000010000000000000000;
	receive[8] = inputs[j] & 0b00000000000000100000000000000000;
	receive[9] = inputs[j] & 0b00000000000001000000000000000000;
	receive[10] = inputs[j] & 0b00000000000010000000000000000000;
	receive[11] = inputs[j] & 0b00000100000000000000000000000000;
	total = receive[0]/16 + 2*receive[1]/32 + 4*receive[2]/64 + 8*receive[3]/128 + 16*receive[4]/256 + 32*receive[5]/2048 + 64*receive[6]/4096 + 128*receive[7]/65536 + 256*receive[8]/131072 + 512*receive[9]/262144 + 1024*receive[10]/524288;
	if (receive[11]){
		total = total + 2048;
	}

	values[j] = total;

    j++;

	if (j == MEM_BUFFER_SIZE ){
		j = 0;
	}
}

//*****************************************************************************
//
// Initializes the uDMA software channel to perform a memory to memory uDMA
// transfer.
//
//*****************************************************************************
void
InitSWTransfer(void)
{

    //
    // Enable interrupts from the uDMA software channel.
    //
    IntEnable(INT_UDMA);

    //
    // Put the attributes in a known state for the uDMA software channel.
    // These should already be disabled by default.
    //
    uDMAChannelAttributeDisable(UDMA_CHANNEL_SW,
                                    UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                    (UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK));

    //
    // Configure the control parameters for the SW channel.  The SW channel
    // will be used to transfer between two memory buffers, 32 bits at a time.
    // Therefore the data size is 32 bits, and the address increment is 32 bits
    // for both source and destination.  The arbitration size will be set to 8,
    // which causes the uDMA controller to rearbitrate after 8 items are
    // transferred.  This keeps this channel from hogging the uDMA controller
    // once the transfer is started, and allows other channels cycles if they
    // are higher priority.
    //
    uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
                              UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                              UDMA_ARB_1);

    //
    // Set up the transfer parameters for the software channel.  This will
    // configure the transfer buffers and the transfer size.  Auto mode must be
    // used for software transfers.
    //
    uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
                               UDMA_MODE_AUTO, EPISource, g_ui32DstBuf,
                               1);

    //
    // Now the software channel is primed to start a transfer.  The channel
    // must be enabled.  For software based transfers, a request must be
    // issued.  After this, the uDMA memory transfer begins.
    //
    uDMAChannelEnable(UDMA_CHANNEL_SW);
}


// Main program//////////////////////////////////////////////////////////////////////////////
int main(void) {


	//uDMA Stuff/////////////////////////////////////////////////////
    //
    // Enable the uDMA controller at the system level.  Enable it to continue
    // to run while the processor is in sleep.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

    //
    // Enable the uDMA controller error interrupt.  This interrupt will occur
    // if there is a bus error during a transfer.
    //
    IntEnable(INT_UDMAERR);

    //
    // Enable the uDMA controller.
    //
    uDMAEnable();

    EPISource = EPI0_BASE + EPI_O_READFIFO0;

    //
    // Point at the control table to use for channel control structures.
    //
    uDMAControlBaseSet(pui8ControlTable);

    // Map Channel perhiperhal for 30
    uDMAChannelAssign(UDMA_CH30_EPI0RX);

    InitSWTransfer();

	uint32_t ui32Period;

	// Set Clock frequency to 120 MHz
	ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	// Enable GPIO and Timer peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	// Enable EPI Peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EPI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

	// Configure GPIO and Timer
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, OUTPUT_PORTD_PINS);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, OUTPUT_PORTE_PINS);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, OUTPUT_PORTF_PINS);
	GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, OUTPUT_PORTH_PINS);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

	// Configure GPIO pins for EPI mode. Address obtained from pin_map.h
	GPIOPinConfigure(GPIO_PA6_EPI0S8);
	GPIOPinConfigure(GPIO_PC4_EPI0S7);
	GPIOPinConfigure(GPIO_PC5_EPI0S6);
	GPIOPinConfigure(GPIO_PC6_EPI0S5);
	GPIOPinConfigure(GPIO_PC7_EPI0S4);
	GPIOPinConfigure(GPIO_PG0_EPI0S11);
	GPIOPinConfigure(GPIO_PL0_EPI0S16);
	GPIOPinConfigure(GPIO_PL1_EPI0S17);
	GPIOPinConfigure(GPIO_PL2_EPI0S18);
	GPIOPinConfigure(GPIO_PL3_EPI0S19);
	GPIOPinConfigure(GPIO_PL4_EPI0S26);
	GPIOPinConfigure(GPIO_PM3_EPI0S12);

    GPIOPinTypeEPI(GPIO_PORTA_BASE, EPI_PORTA_PINS);
    GPIOPinTypeEPI(GPIO_PORTC_BASE, EPI_PORTC_PINS);
    GPIOPinTypeEPI(GPIO_PORTG_BASE, EPI_PORTG_PINS);
    GPIOPinTypeEPI(GPIO_PORTL_BASE, EPI_PORTL_PINS);
    GPIOPinTypeEPI(GPIO_PORTM_BASE, EPI_PORTM_PINS);

    // Select general purpose mode for EPI
    EPIModeSet(EPI0_BASE, EPI_MODE_GENERAL);

	// Set Timer A to have a period of one tenth the clock freq
	ui32Period = ui32SysClkFreq/2;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period/20000-1);

	// Set GPIO's to have all zeroes
	GPIOPinWrite(GPIO_PORTD_BASE, OUTPUT_PORTD_PINS,0);
	GPIOPinWrite(GPIO_PORTE_BASE, OUTPUT_PORTE_PINS,0);
	GPIOPinWrite(GPIO_PORTF_BASE, OUTPUT_PORTF_PINS,0);
	GPIOPinWrite(GPIO_PORTH_BASE, OUTPUT_PORTH_PINS,0);

	// Enable timer interrupt and set it to go off at Timer A timeouts
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	// Enable Timer A
	TimerEnable(TIMER0_BASE, TIMER_A);

	// Configure EPI settings
	EPIDividerSet(EPI0_BASE,0);
	EPIAddressMapSet(EPI0_BASE,EPI_ADDR_PER_SIZE_256B|EPI_ADDR_PER_BASE_A);
	EPIConfigGPModeSet(EPI0_BASE,EPI_GPMODE_DSIZE_32|EPI_GPMODE_ASIZE_NONE|EPI_GPMODE_CLKPIN,0,0);
	EPINonBlockingReadConfigure(EPI0_BASE,0,EPI_NBCONFIG_SIZE_32,0);
	EPINonBlockingReadStart(EPI0_BASE,0,1);
	//test = 10;
	//g_ui32DstBuf[0] = &test;

	for(f=0;f<MEM_BUFFER_SIZE;f++){
		g_ui32DstBuf[f] = &inputs[f];
	}

	EPIFIFOConfig(EPI0_BASE,EPI_FIFO_CONFIG_RX_1_8);
	EPIIntEnable(EPI0_BASE,EPI_INT_RXREQ);
	IntEnable(INT_EPI0);







	while(1)
	{
		/*// Start another read if the data as been read already
		if (Start == 1)
		{
			EPINonBlockingReadConfigure(EPI0_BASE,0,EPI_NBCONFIG_SIZE_32,0);
			EPINonBlockingReadStart(EPI0_BASE,0,1);
			Start = 0;
		}
		EPIReadCount = EPINonBlockingReadCount(EPI0_BASE,0);
		EPIAvail = EPINonBlockingReadAvail(EPI0_BASE);
		// Read the data from the FIFO if data is available
		if(EPIAvail)
		{
			EPINonBlockingReadGet32(EPI0_BASE,1,EPIdata);
			*/

	}

}

void Timer0IntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// State variable to help change values
	i++;

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

	tri[0] = F1;

	// Change PF2 every 2 cycles after i>1
	if(i > 1 && i%2 == 0)
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

	tri[1] = F2;

	// Change PF3 every 4 cycles after i>3
	if(i > 3 && i%4 == 0)
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

	// Change PH2 every 8 cyclces after i>7
	if(i > 7 && i%8 == 0)
	{
		if(H2 = GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2))
		{
			GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_2, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_2, 4);
		}
	}

	// Change PH3 every 16 cyclces after i>15
	if(i > 15 && i%16 == 0)
	{
		if(H3 = GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_3))
		{
			GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_3, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_3, 8);
		}
	}

	// Change PD1 every 32 cyclces after i>31
	if(i > 31 && i%32 == 0)
	{
		if(D1 = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1))
		{
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 2);
		}
	}

	// Change PD0 every 64 cyclces after i>63
	if(i > 63 && i%64 == 0)
	{
		if(D0 = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0))
		{
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1);
		}
	}

	// Change PE4 every 128 cyclces after i>127
	if(i > 127 && i%128 == 0)
	{
		if(E4 = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4))
		{
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 16);
		}
	}

	// Change PE0 every 256 cyclces after i>255
	if(i > 255 && i%256 == 0)
	{
		if(E0 = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0))
		{
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 1);
		}
	}

	// Change PE1 every 512 cyclces after i>511
	if(i > 511 && i%512 == 0)
	{
		if(E1 = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1))
		{
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2);
		}
	}

	// Change PE2 every 1024 cyclces after i>1023
	if(i > 1023 && i%1024 == 0)
	{
		if(E2 = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2))
		{
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4);
		}
	}

	// Change PE3 every 2048 cyclces after i>2047
	if(i > 2047 && i%2048 == 0)
	{
		if(E3 = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3))
		{
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 8);
		}
	}


	tri[2] = F3;
	tri[3] = H2;
	tri[4] = H3;
	tri[5] = D1;
	tri[6] = D0;
	tri[7] = E4;
	tri[8] = E0;
	tri[9] = E1;
	tri[10] = E2;
	tri[11] = E3;
}
