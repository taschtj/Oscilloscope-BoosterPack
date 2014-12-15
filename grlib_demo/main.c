#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "grlib/grlib.h"
#include "Kentec320x240x16_ssd2119_8bit.h"
uint32_t ui32SysClkFreq;
extern const uint8_t g_pui8Image[];
tContext sContext;
tRectangle sRect;
void ClrScreen(void);
int main(void)
{
ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
SYSCTL_CFG_VCO_480), 120000000);
Kentec320x240x16_SSD2119Init(ui32SysClkFreq);
GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);
ClrScreen();
GrImageDraw(&sContext, g_pui8Image, 0, 0);
GrFlush(&sContext);
SysCtlDelay(ui32SysClkFreq);

// later lab steps are between here
ClrScreen();
sRect. i16XMin = 1;
 sRect. i16YMin = 1;
 sRect. i16XMax = 318;
 sRect. i16YMax = 238;
 GrContextForegroundSet(&sContext, ClrRed);
 GrContextFontSet(&sContext, &g_sFontCmss30b);
 GrStringDraw(&sContext, "Texas", -1, 110, 2, 0);
 GrStringDraw(&sContext, "Instruments", -1, 80, 32, 0);
 GrStringDraw(&sContext, "Graphics", -1, 100, 62, 0);
 GrStringDraw(&sContext, "Lab", -1, 135, 92, 0);
 GrContextForegroundSet(&sContext, ClrWhite);
 GrRectDraw(&sContext, &sRect);
 GrFlush(&sContext);

 SysCtlDelay(ui32SysClkFreq);
 GrContextForegroundSet(&sContext, ClrBlue);
 GrCircleFill(&sContext, 80, 182, 50);
 sRect. i16XMin = 160;
  sRect. i16YMin = 132;
  sRect. i16XMax = 312;
  sRect. i16YMax = 232;
 GrContextForegroundSet(&sContext, ClrGreen);
  GrRectDraw(&sContext, &sRect);
  SysCtlDelay(ui32SysClkFreq);
// and here
ClrScreen();
while(1)
{}
}
void ClrScreen()
{
sRect.i16XMin = 0;
sRect.i16YMin = 0;
sRect.i16XMax = 319;
sRect.i16YMax = 239;
GrContextForegroundSet(&sContext, ClrBlack);
GrRectFill(&sContext, &sRect);
GrFlush(&sContext);
}
