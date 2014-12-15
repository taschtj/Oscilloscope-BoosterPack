#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "grlib/pushbutton.h"
#include "Kentec320x240x16_ssd2119_8bit.h"
#include "touch.h"
#include <math.h>
#include "driverlib/fpu.h"

#include "driverlib/debug.h"
uint32_t ui32SysClkFreq;

//
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define SERIES_LENGTH 319
float gSeriesData[SERIES_LENGTH];
//



extern const uint8_t g_pui8Image[];
extern const uint8_t g_pui9Image[];

tContext sContext;
tRectangle sRect;

extern tCanvasWidget g_sBackground;
extern tPushButtonWidget g_sPushBtn;
int x;
int y;
void ClrScreen(void);

int main(void)
{
	ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
	SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
	SYSCTL_CFG_VCO_480), 120000000);

	Kentec320x240x16_SSD2119Init(ui32SysClkFreq);
	GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

//intro pictures
	GrImageDraw(&sContext, g_pui8Image, 0, 0);
	GrFlush(&sContext);
	SysCtlDelay(ui32SysClkFreq);

	GrImageDraw(&sContext, g_pui9Image, 0, 0);
	GrFlush(&sContext);
	SysCtlDelay(ui32SysClkFreq);
	ClrScreen();

//  design layout

	//Top
	GrContextForegroundSet(&sContext, ClrWhite);
	GrContextFontSet(&sContext, &g_sFontCmss14); //set the font


	sRect. i16XMin = 0;sRect. i16YMin = 0;sRect. i16XMax = 52;sRect. i16YMax = 28;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrRed);
	GrStringDraw(&sContext, " mV/div", -1, 0, 5, 0);

	sRect. i16XMin = 52;sRect. i16YMin = 0;sRect. i16XMax =105 ;sRect. i16YMax = 28;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrYellow);
	GrStringDraw(&sContext, " mV/div", -1, 53, 5, 0);

	sRect. i16XMin = 105;sRect. i16YMin = 0;sRect. i16XMax =158 ;sRect. i16YMax = 28;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "  mS/div", -1, 106, 5, 0);

	sRect. i16XMin = 158;sRect. i16YMin = 0;sRect. i16XMax =211 ;sRect. i16YMax = 28;
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "Trigger", -1, 165,5, 0);

	sRect. i16XMin = 211;sRect. i16YMin = 0;sRect. i16XMax =264 ;sRect. i16YMax = 28;
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "Cursors", -1, 216, 5, 0);

	sRect. i16XMin = 264;sRect. i16YMin = 0;sRect. i16XMax =317 ;sRect. i16YMax = 28;
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "Menu", -1, 273, 5, 0);

	//Bottom
	sRect. i16XMin = 0;sRect. i16YMin = 211;sRect. i16XMax = 52;sRect. i16YMax = 239;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrRed);
	GrStringDraw(&sContext, "  1 Hz", -1, 0, 216, 0);

	sRect. i16XMin = 52;sRect. i16YMin = 211;sRect. i16XMax =105 ;sRect. i16YMax = 239;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrYellow);
	GrStringDraw(&sContext, "  2 Hz", -1, 53, 216, 0);

	sRect. i16XMin = 105;sRect. i16YMin = 211;sRect. i16XMax =158 ;sRect. i16YMax = 239;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrRed);
	GrStringDraw(&sContext, "  1 V", -1, 106, 216, 0);

	sRect. i16XMin = 158;sRect. i16YMin = 211;sRect. i16XMax =211 ;sRect. i16YMax = 239;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrYellow);
	GrStringDraw(&sContext, "  2 V", -1, 165,216, 0);

	GrContextForegroundSet(&sContext, ClrWhite);
	sRect. i16XMin = 211;sRect. i16YMin = 211;sRect. i16XMax =264 ;sRect. i16YMax = 239;
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "  Auto", -1, 211, 216, 0);

	sRect. i16XMin = 264;sRect. i16YMin = 211;sRect. i16XMax =317 ;sRect. i16YMax = 239;
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "Run/Stop", -1, 266, 216, 0);




	 GrFlush(&sContext);
	 SysCtlDelay(ui32SysClkFreq);
	//
	 float fRadians1;
	 float fRadians2;
	  FPULazyStackingEnable();
	  FPUEnable();
	  fRadians1 = ((4 * M_PI) / SERIES_LENGTH);
	  fRadians2 = ((7 * M_PI) / SERIES_LENGTH);
//
//WaveForm

GrContextForegroundSet(&sContext, ClrRed);
for(x=0;x<SERIES_LENGTH;x++){
 GrCircleFill(&sContext, x, 75-sinf(fRadians1*x)*40, 1);

}
GrContextForegroundSet(&sContext, ClrYellow);
for(x=0;x<SERIES_LENGTH;x++){
 GrCircleFill(&sContext, x, 163-sinf(fRadians2*x)*40, 1);

}


//////////////////Draw Grids
GrContextForegroundSet(&sContext, ClrWhite);
GrCircleFill(&sContext, 160,119.5, 3);

for(x=0;x<320;x+=4){
	for (y=29;y<=210;y+=18.1){
	GrPixelDraw(&sContext, x, y);
	}
}
for(x=0;y<320;x+=32){
	for (y=29;y<=210;y+=4){
	GrPixelDraw(&sContext, x, y);
	}
}

///////////////////


 //SysCtlDelay(ui32SysClkFreq);
}










void ClrScreen()
{
 sRect. i16XMin = 0;
 sRect. i16YMin = 0;
 sRect. i16XMax = 319;
 sRect. i16YMax = 239;
 GrContextForegroundSet(&sContext, ClrBlack);
 GrRectFill(&sContext, &sRect);
 GrFlush(&sContext);
}
