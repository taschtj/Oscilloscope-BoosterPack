/*
 * MainTestV2.c
 *
 *  Created on: Dec 14, 2014
 *      Author: zhuangr
 */

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
#include "grlib/container.h"
#include "grlib/radiobutton.h"
#include "Kentec320x240x16_ssd2119_8bit.h"
#include "touch.h"
#include <math.h>
#include "driverlib/fpu.h"

#include "driverlib/debug.h"

// Tommy includes
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "inc/hw_nvic.h"
#include "driverlib/flash.h"
#include "driverlib/udma.h"
#include "utils/ustdlib.h"
#include "images.h"
#include "inc/hw_epi.h"
#include "driverlib/epi.h"
#include "driverlib/timer.h"

uint32_t ui32Mode;

uint32_t *EPISource;
uint8_t below = 0;

// define epi port pins to be used
#define EPI_PORTA_PINS (GPIO_PIN_6)
#define EPI_PORTC_PINS (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4)
#define EPI_PORTG_PINS (GPIO_PIN_0)
#define EPI_PORTL_PINS (GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0)
#define EPI_PORTM_PINS (GPIO_PIN_3)

//*****************************************************************************
//
// The size of the memory transfer source and destination buffers (in words).
//
//*****************************************************************************
#define MEM_BUFFER_SIZE         1024
#define MaxSize					8192 // Must be multiple of MEM_BUFFER_SIZE
#define TriggerLevel			100

//*****************************************************************************
//
// The source and destination buffers used for memory transfers.
//
//*****************************************************************************
static uint32_t *g_ui32DstBuf[MEM_BUFFER_SIZE];
static uint32_t *g_ui32DstBuf2[MEM_BUFFER_SIZE];
uint32_t values[MaxSize], values2[MEM_BUFFER_SIZE],
		total_values[MEM_BUFFER_SIZE * 2];
uint32_t inputs[MEM_BUFFER_SIZE], inputs2[MEM_BUFFER_SIZE], all_values[MaxSize];

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
static uint32_t g_ui32MemXferCount = 0, Done = 0;

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
////////////////////////////////////////////////////////

uint32_t ui32SysClkFreq;

//
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define SERIES_LENGTH 319
float gSeriesData[SERIES_LENGTH];
//

int x, old[SERIES_LENGTH];

// global variables for generating "random" signal
uint32_t i = 0, j = 0, f = 0, k = 0, m = 0, l = 0;
uint32_t receive[12], total, CountSize = 1024, count = 0, pixels[SERIES_LENGTH], pixel_total = 0, pixel_average;
uint8_t pri, alt, TriggerStart = 0, Trigger = 0, NumAvg = 1, GoThrough = 0;

extern const uint8_t g_pui8Image[];
extern const uint8_t g_pui9Image[];

tContext sContext;
tRectangle sRect;

extern tCanvasWidget g_sBackground;
extern tCanvasWidget g_sWaveform;
extern tCanvasWidget g_sTop;
extern tCanvasWidget g_sBottom;
extern tPushButtonWidget g_sPushBtn0;
extern tPushButtonWidget g_sPushBtn1;
extern tPushButtonWidget g_sPushBtn2;
extern tPushButtonWidget g_sPushBtn3;
extern tPushButtonWidget g_sPushBtn4;
extern tPushButtonWidget g_sPushBtn5;
extern tPushButtonWidget g_sPushBtn6;
extern tPushButtonWidget g_sPushBtn7;
extern tPushButtonWidget g_sPushBtn8;
extern tPushButtonWidget g_sPushBtn9;
extern tPushButtonWidget g_sPushBtn10;
extern tPushButtonWidget g_sPushBtn11;
extern tContainerWidget g_sContainer0;
extern tContainerWidget g_sContainer1;
extern tContainerWidget g_sContainerMenu;
extern tContainerWidget g_sContainerMagnitude;
extern tRadioButtonWidget g_sRadioBtn0;
extern tRadioButtonWidget g_sRadioBtn1;
extern tRadioButtonWidget g_sRadioBtn2;
extern tRadioButtonWidget g_sRadioBtn3;
extern tRadioButtonWidget g_sRadioBtnAcquire;
extern tRadioButtonWidget g_sRadioBtnCoupling;
extern tRadioButtonWidget g_sRadioBtnMode;
extern tRadioButtonWidget g_sRadioBtnMath;
extern tRadioButtonWidget g_sRadioBtnMaximum;
extern tRadioButtonWidget g_sRadioBtnMinimum;
extern tRadioButtonWidget g_sRadioBtnAverage;
int x;
int y;
void ClrScreen(void);
void DRadio0(tWidget *pWidgetR);
void DRadio1(tWidget *pWidgetR);
void DRadioMenu(tWidget *pWidgetR);
void DRadioMagnitude(tWidget *pWidgetR);
void DWaveForm(tWidget *pWidgetR, tContext *psContext);
void setup(void);


Canvas(g_sBackground, WIDGET_ROOT, 0, &g_sTop, &g_sKentec320x240x16_SSD2119,
		0, 0, 320,240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

Canvas(g_sWaveform, &g_sBackground, 0,0, &g_sKentec320x240x16_SSD2119,
		0, 29, 320,182, CANVAS_STYLE_APP_DRAWN, ClrBlack, 0, 0, 0, 0, 0, DWaveForm);


Canvas(g_sTop, &g_sBackground, &g_sBottom, &g_sPushBtn0, &g_sKentec320x240x16_SSD2119,
		0, 0, 320, 28, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);
Canvas(g_sBottom, &g_sBackground, 0, &g_sPushBtn6, &g_sKentec320x240x16_SSD2119,
		0, 212, 320, 28, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

///Top Buttons///////////////////////////////////////////////////////
RectangularButton(g_sPushBtn0, &g_sTop, &g_sPushBtn1, 0,
		&g_sKentec320x240x16_SSD2119, 0, 0, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrRed,g_psFontCm16, "V/div", 0, 0,
		0, 0, DRadio0);

RectangularButton(g_sPushBtn1, &g_sTop, &g_sPushBtn2, 0, &g_sKentec320x240x16_SSD2119,
		53, 0, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrYellow, g_psFontCm16, "V/div", 0, 0, 0, 0,
		DRadio1);
RectangularButton(g_sPushBtn2, &g_sTop, &g_sPushBtn3, 0,
		&g_sKentec320x240x16_SSD2119, 106, 0, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrWhite,g_psFontCm16, "s/div", 0, 0,
		0, 0, 0);

RectangularButton(g_sPushBtn3, &g_sTop, &g_sPushBtn4, 0, &g_sKentec320x240x16_SSD2119,
		159, 0, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrWhite, g_psFontCm16, "Trigger", 0, 0, 0, 0,
		0);
RectangularButton(g_sPushBtn4, &g_sTop, &g_sPushBtn5, 0,
		&g_sKentec320x240x16_SSD2119, 212, 0, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrWhite,g_psFontCm16, "Cursors", 0, 0,
		0, 0, 0);

RectangularButton(g_sPushBtn5, &g_sTop, 0, 0, &g_sKentec320x240x16_SSD2119,
		265,0, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrWhite, g_psFontCm16, "Menu", 0, 0, 0, 0,
		DRadioMenu);
///Bottom Buttons///////////////////////////////////////////////////////
RectangularButton(g_sPushBtn6, &g_sBottom, &g_sPushBtn7, 0,
		&g_sKentec320x240x16_SSD2119, 0, 212, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrRed,g_psFontCm16, "Hz", 0, 0,
		0, 0, DRadioMagnitude);

RectangularButton(g_sPushBtn7, &g_sBottom, &g_sPushBtn8, 0, &g_sKentec320x240x16_SSD2119,
		53, 212, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrYellow, g_psFontCm16, "Hz", 0, 0, 0, 0,
		0);
RectangularButton(g_sPushBtn8, &g_sBottom, &g_sPushBtn9, 0,
		&g_sKentec320x240x16_SSD2119, 106, 212, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrRed,g_psFontCm16, "V", 0, 0,
		0, 0, 0);

RectangularButton(g_sPushBtn9, &g_sBottom, &g_sPushBtn10, 0, &g_sKentec320x240x16_SSD2119,
		159, 212, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrYellow, g_psFontCm16, "V", 0, 0, 0, 0,
		0);
RectangularButton(g_sPushBtn10, &g_sBottom, &g_sPushBtn11, 0,
		&g_sKentec320x240x16_SSD2119, 212, 212, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrWhite,g_psFontCm16, "Auto", 0, 0,
		0, 0, 0);

RectangularButton(g_sPushBtn11, &g_sBottom, 0, 0, &g_sKentec320x240x16_SSD2119,
		265,212, 52, 28,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrWhite, g_psFontCmss14, "Run/Stop", 0, 0, 0, 0,
		0);
Container(g_sContainer0,0,0,&g_sRadioBtn0,&g_sKentec320x240x16_SSD2119,0, 28, 52, 60,(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ),ClrBlack, ClrWhite,ClrRed,g_psFontCm14, 0);
Container(g_sContainer1,0,0,&g_sRadioBtn2,&g_sKentec320x240x16_SSD2119,53, 28, 52, 60,(CTR_STYLE_OUTLINE |CTR_STYLE_FILL),ClrBlack, ClrWhite,ClrRed,g_psFontCm14, 0);
Container(g_sContainerMenu,0,0,&g_sRadioBtnAcquire,&g_sKentec320x240x16_SSD2119,265, 28, 52, 115,(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ),ClrBlack, ClrWhite,ClrRed,g_psFontCm14, 0);
Container(g_sContainerMagnitude,0,0,&g_sRadioBtnMaximum,&g_sKentec320x240x16_SSD2119,0, 101, 52, 110,(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ),ClrBlack, ClrWhite,ClrRed,g_psFontCm14, 0);

RadioButton(g_sRadioBtn0,&g_sContainer0,&g_sRadioBtn1,0,&g_sKentec320x240x16_SSD2119,1,40,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"10 mV ",0,0);
RadioButton(g_sRadioBtn1,&g_sContainer0,0,0,&g_sKentec320x240x16_SSD2119,1,61,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"20 mV",0,0);
RadioButton(g_sRadioBtn2,&g_sContainer1,&g_sRadioBtn3,0,&g_sKentec320x240x16_SSD2119,54,40,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"10 mV",0,0);
RadioButton(g_sRadioBtn3,&g_sContainer1,0,0,&g_sKentec320x240x16_SSD2119,54,61,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"20 mV",0,0);

RadioButton(g_sRadioBtnAcquire,&g_sContainerMenu,&g_sRadioBtnCoupling,0,&g_sKentec320x240x16_SSD2119,266,40,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"Acquire",0,0);
RadioButton(g_sRadioBtnCoupling,&g_sContainerMenu,&g_sRadioBtnMode,0,&g_sKentec320x240x16_SSD2119,266,61,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"Coupling",0,0);
RadioButton(g_sRadioBtnMode,&g_sContainerMenu,&g_sRadioBtnMath,0,&g_sKentec320x240x16_SSD2119,266,82,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"Mode",0,0);
RadioButton(g_sRadioBtnMath,&g_sContainerMenu,0,0,&g_sKentec320x240x16_SSD2119,266,103,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"Math",0,0);

RadioButton(g_sRadioBtnMaximum,&g_sContainerMagnitude,&g_sRadioBtnMinimum,0,&g_sKentec320x240x16_SSD2119,1,102,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"Maximum",0,0);
RadioButton(g_sRadioBtnMinimum,&g_sContainerMagnitude,&g_sRadioBtnAverage,0,&g_sKentec320x240x16_SSD2119,1,123,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"Minimum",0,0);
RadioButton(g_sRadioBtnAverage,&g_sContainerMagnitude,0,0,&g_sKentec320x240x16_SSD2119,1,144,48,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss12,"Average",0,0);
//
bool HH=false;

void DRadio0(tWidget *pWidgetR){
	HH=!HH;
	if(HH){
		WidgetPaint((tWidget *)&g_sContainer0);
	}
	else{
		WidgetRemove((tWidget *)&g_sContainer0);
		WidgetPaint((tWidget *)&g_sBackground);
		WidgetPaint((tWidget *)& g_sWaveform);
	}

}
bool BB=false;
void DRadio1(tWidget *pWidgetR){
	BB=!BB;
	if(BB){
		WidgetPaint((tWidget *)&g_sContainer1);
	}
	else{
		WidgetRemove((tWidget *)&g_sContainer1);
		WidgetPaint((tWidget *)&g_sBackground);
		WidgetPaint((tWidget *)& g_sWaveform);
	}

}
bool AA=false;

void DRadioMagnitude(tWidget *pWidgetR){
	AA=!AA;
	if(AA){
		WidgetPaint((tWidget *)&g_sContainerMagnitude);
	}
	else{
		WidgetRemove((tWidget *)&g_sContainerMagnitude);
		WidgetPaint((tWidget *)&g_sBackground);
		WidgetPaint((tWidget *)& g_sWaveform);
	}

}
bool CC=false;
void DRadioMenu(tWidget *pWidgetR){
	CC=!CC;
	if(CC){
		WidgetPaint((tWidget *)&g_sContainerMenu);
	}
	else{
		WidgetRemove((tWidget *)&g_sContainerMenu);
		WidgetPaint((tWidget *)&g_sBackground);
		WidgetPaint((tWidget *)& g_sWaveform);
	}

}
void DWaveForm(tWidget *pWidgetR, tContext *psContext) {
	//float fRadians1;
	//float fRadians2;
	FPULazyStackingEnable();
	FPUEnable();
	//fRadians1 = ((4 * M_PI) / SERIES_LENGTH);
	//fRadians2 = ((7 * M_PI) / SERIES_LENGTH);
	/*GrContextForegroundSet(&sContext, ClrRed);
	for (x = 0; x < SERIES_LENGTH; x++) {
		GrCircleFill(&sContext, x, 75 - sinf(fRadians1 * x) * 40, 1);

	}
	GrContextForegroundSet(&sContext, ClrYellow);
	for (x = 0; x < SERIES_LENGTH; x++) {
		GrCircleFill(&sContext, x, 163 - sinf(fRadians2 * x) * 40, 1);

	}*/

	for (x = 0; x < SERIES_LENGTH; x++) {
		GrContextForegroundSet(&sContext, ClrBlack);
		GrCircleFill(&sContext, x, old[x], 1);
		GrContextForegroundSet(&sContext, ClrYellow);
		GrCircleFill(&sContext, x, old[x] = 150 - pixels[x] / 50, 1);

	}
		GrContextForegroundSet(&sContext, ClrWhite);
		GrCircleFill(&sContext, 160, 119, 3);

		for (x = 0; x <= 320; x += 4) {
			for (y = 29; y <= 210; y += 18.1) {
				GrPixelDraw(&sContext, x, y);
			}
		}
		for (x = 0; x <=320; x += 32) {
			for (y = 29; y <= 210; y += 3.62) {
				GrPixelDraw(&sContext, x, y);
			}
		}
}
//
int main(void) {
	ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
	SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
	SYSCTL_CFG_VCO_480), 120000000);

	Kentec320x240x16_SSD2119Init(ui32SysClkFreq);
	GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);
	TouchScreenInit(ui32SysClkFreq);
	TouchScreenCallbackSet(WidgetPointerMessage);


//intro pictures
//	GrImageDraw(&sContext, g_pui8Image, 0, 0);
//	GrFlush(&sContext);
//	SysCtlDelay(ui32SysClkFreq);
//
//	GrImageDraw(&sContext, g_pui9Image, 0, 0);
//	GrFlush(&sContext);
//	SysCtlDelay(ui32SysClkFreq);
//	ClrScreen();
//
//	float fRadians1;
//	float fRadians2;
//	FPULazyStackingEnable();
//	FPUEnable();
//	fRadians1 = ((4 * M_PI) / SERIES_LENGTH);
//	fRadians2 = ((7 * M_PI) / SERIES_LENGTH);


	WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sBackground);
	WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sWaveform);
	WidgetPaint(WIDGET_ROOT);

	setup();

	while (1) {
		//wave
//		GrContextForegroundSet(&sContext, ClrRed);
//		for (x = 0; x < SERIES_LENGTH; x++) {
//			GrCircleFill(&sContext, x, 75 - sinf(fRadians1 * x) * 40, 1);
//
//		}
//		GrContextForegroundSet(&sContext, ClrYellow);
//		for (x = 0; x < SERIES_LENGTH; x++) {
//			GrCircleFill(&sContext, x, 163 - sinf(fRadians2 * x) * 40, 1);
//
//		}
		//

		if (1) {
					for (f = 0; f < MEM_BUFFER_SIZE; f++) {
						receive[0] = inputs[f] & 0b00000000000000000000000000010000;
						receive[1] = inputs[f] & 0b00000000000000000000000000100000;
						receive[2] = inputs[f] & 0b00000000000000000000000001000000;
						receive[3] = inputs[f] & 0b00000000000000000000000010000000;
						receive[4] = inputs[f] & 0b00000000000000000000000100000000;
						receive[5] = inputs[f] & 0b00000000000000000000100000000000;
						receive[6] = inputs[f] & 0b00000000000000000001000000000000;
						receive[7] = inputs[f] & 0b00000000000000010000000000000000;
						receive[8] = inputs[f] & 0b00000000000000100000000000000000;
						receive[9] = inputs[f] & 0b00000000000001000000000000000000;
						receive[10] = inputs[f] & 0b00000000000010000000000000000000;
						receive[11] = inputs[f] & 0b00000100000000000000000000000000;
						total = receive[0] / 16 + 2 * receive[1] / 32
								+ 4 * receive[2] / 64 + 8 * receive[3] / 128
								+ 16 * receive[4] / 256 + 32 * receive[5] / 2048
								+ 64 * receive[6] / 4096 + 128 * receive[7] / 65536
								+ 256 * receive[8] / 131072 + 512 * receive[9] / 262144
								+ 1024 * receive[10] / 524288;
						if (receive[11])
							total = total + 2048;

						values[f + k*MEM_BUFFER_SIZE] = total;
						if(total <= TriggerLevel)
							below = 0;

						if(total >= TriggerLevel && below == 0){
							TriggerStart = 1;
							Trigger = 1;
							below = 1;
						}

						if(TriggerStart == 1){
							if(j<NumAvg){
								j++;
								pixel_total = pixel_total + values[f+k*MEM_BUFFER_SIZE];
							}
							else{
								j=0;
								pixel_average = pixel_total/NumAvg;
								pixel_total = 0;

								if(m < SERIES_LENGTH){
									pixels[m] = pixel_average;
									m++;
								}
								else{
									m = 0;
									TriggerStart = 0;
								}
							}
						}
					}
				}

				if(k*MEM_BUFFER_SIZE + MEM_BUFFER_SIZE < MaxSize)
					k++;

				else
					k = 0;

				if(Trigger == 1){
					GoThrough = 0;
					Trigger = 0;
				}
				else
					GoThrough++;

				if(GoThrough >= 10){
					for(i=0;i<SERIES_LENGTH;i++){
						pixels[i] = 0;
					}
				}

				if (1) {
					for (f = 0; f < MEM_BUFFER_SIZE; f++) {
						receive[0] = inputs2[f] & 0b00000000000000000000000000010000;
						receive[1] = inputs2[f] & 0b00000000000000000000000000100000;
						receive[2] = inputs2[f] & 0b00000000000000000000000001000000;
						receive[3] = inputs2[f] & 0b00000000000000000000000010000000;
						receive[4] = inputs2[f] & 0b00000000000000000000000100000000;
						receive[5] = inputs2[f] & 0b00000000000000000000100000000000;
						receive[6] = inputs2[f] & 0b00000000000000000001000000000000;
						receive[7] = inputs2[f] & 0b00000000000000010000000000000000;
						receive[8] = inputs2[f] & 0b00000000000000100000000000000000;
						receive[9] = inputs2[f] & 0b00000000000001000000000000000000;
						receive[10] = inputs2[f] & 0b00000000000010000000000000000000;
						receive[11] = inputs2[f] & 0b00000100000000000000000000000000;
						total = receive[0] / 16 + 2 * receive[1] / 32
								+ 4 * receive[2] / 64 + 8 * receive[3] / 128
								+ 16 * receive[4] / 256 + 32 * receive[5] / 2048
								+ 64 * receive[6] / 4096 + 128 * receive[7] / 65536
								+ 256 * receive[8] / 131072 + 512 * receive[9] / 262144
								+ 1024 * receive[10] / 524288;
						if (receive[11]) {
							total = total + 2048;
						}

						values[f + k*MEM_BUFFER_SIZE] = total;

						if(total <= TriggerLevel)
							below = 0;

						if(total >= TriggerLevel && below == 0){
							TriggerStart = 1;
							Trigger = 1;
							below = 1;
						}

						if(TriggerStart == 1){
							if(j<NumAvg){
								j++;
								pixel_total = pixel_total + values[f+k*MEM_BUFFER_SIZE];
							}
							else{
								j=0;
								pixel_average = pixel_total/NumAvg;
								pixel_total = 0;

								if(m < SERIES_LENGTH){
									pixels[m] = pixel_average;
									m++;
								}
								else{
									m = 0;
									TriggerStart = 0;
								}
							}
						}
					}
				}

				if(k*MEM_BUFFER_SIZE + MEM_BUFFER_SIZE < MaxSize)
					k++;

				else
					k = 0;

				if(Trigger == 1){
					Trigger = 0;
					GoThrough = 0;
				}
				else
					GoThrough++;

				if(GoThrough >= 10){
					for(i=0;i<SERIES_LENGTH;i++){
						pixels[i] = 0;
					}
				}

		//WidgetMessageQueueProcess();
	}








///Previous code, doesn't need to be displayed/////////////////////////////////////////////////////////////////////////////
//  design layout
//Top
	GrContextForegroundSet(&sContext, ClrWhite);
	GrContextFontSet(&sContext, &g_sFontCmss14); //set the font

	sRect.i16XMin = 0;
	sRect.i16YMin = 0;
	sRect.i16XMax = 52;
	sRect.i16YMax = 28;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrRed);
	GrStringDraw(&sContext, " mV/div", -1, 0, 5, 0);

	sRect.i16XMin = 52;
	sRect.i16YMin = 0;
	sRect.i16XMax = 105;
	sRect.i16YMax = 28;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrYellow);
	GrStringDraw(&sContext, " mV/div", -1, 53, 5, 0);

	sRect.i16XMin = 105;
	sRect.i16YMin = 0;
	sRect.i16XMax = 158;
	sRect.i16YMax = 28;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "  mS/div", -1, 106, 5, 0);

	sRect.i16XMin = 158;
	sRect.i16YMin = 0;
	sRect.i16XMax = 211;
	sRect.i16YMax = 28;
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "Trigger", -1, 165, 5, 0);

	sRect.i16XMin = 211;
	sRect.i16YMin = 0;
	sRect.i16XMax = 264;
	sRect.i16YMax = 28;
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "Cursors", -1, 216, 5, 0);

	sRect.i16XMin = 264;
	sRect.i16YMin = 0;
	sRect.i16XMax = 317;
	sRect.i16YMax = 28;
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "Menu", -1, 273, 5, 0);

	//Bottom
	sRect.i16XMin = 0;
	sRect.i16YMin = 211;
	sRect.i16XMax = 52;
	sRect.i16YMax = 239;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrRed);
	GrStringDraw(&sContext, "  1Hz", -1, 0, 216, 0);

	sRect.i16XMin = 52;
	sRect.i16YMin = 211;
	sRect.i16XMax = 105;
	sRect.i16YMax = 239;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrYellow);
	GrStringDraw(&sContext, "  2Hz", -1, 53, 216, 0);

	sRect.i16XMin = 105;
	sRect.i16YMin = 211;
	sRect.i16XMax = 158;
	sRect.i16YMax = 239;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrRed);
	GrStringDraw(&sContext, "  1 V", -1, 106, 216, 0);

	sRect.i16XMin = 158;
	sRect.i16YMin = 211;
	sRect.i16XMax = 211;
	sRect.i16YMax = 239;
	GrContextForegroundSet(&sContext, ClrWhite);
	GrRectDraw(&sContext, &sRect);
	GrContextForegroundSet(&sContext, ClrYellow);
	GrStringDraw(&sContext, "  2 V", -1, 165, 216, 0);

	GrContextForegroundSet(&sContext, ClrWhite);
	sRect.i16XMin = 211;
	sRect.i16YMin = 211;
	sRect.i16XMax = 264;
	sRect.i16YMax = 239;
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "  Auto", -1, 211, 216, 0);

	sRect.i16XMin = 264;
	sRect.i16YMin = 211;
	sRect.i16XMax = 317;
	sRect.i16YMax = 239;
	GrRectDraw(&sContext, &sRect);
	GrStringDraw(&sContext, "Run/Stop", -1, 266, 216, 0);

	GrFlush(&sContext);
	SysCtlDelay(ui32SysClkFreq);
// WaveForm
//	float fRadians1;
//	float fRadians2;
//	FPULazyStackingEnable();
//	FPUEnable();
//	fRadians1 = ((4 * M_PI) / SERIES_LENGTH);
//	fRadians2 = ((7 * M_PI) / SERIES_LENGTH);
//
//	GrContextForegroundSet(&sContext, ClrRed);
//	for (x = 0; x < SERIES_LENGTH; x++) {
//		GrCircleFill(&sContext, x, 75 - sinf(fRadians1 * x) * 40, 1);
//
//	}
//	GrContextForegroundSet(&sContext, ClrYellow);
//	for (x = 0; x < SERIES_LENGTH; x++) {
//		GrCircleFill(&sContext, x, 163 - sinf(fRadians2 * x) * 40, 1);
//
//	}

//////////////////Draw Grids
//	GrContextForegroundSet(&sContext, ClrWhite);
//	GrCircleFill(&sContext, 160, 119.5, 3);
//
//	for (x = 0; x < 320; x += 4) {
//		for (y = 29; y <= 210; y += 18.1) {
//			GrPixelDraw(&sContext, x, y);
//		}
//	}
//	for (x = 0; y < 320; x += 32) {
//		for (y = 29; y <= 210; y += 4) {
//			GrPixelDraw(&sContext, x, y);
//		}
//	}

///////////////////

	//SysCtlDelay(ui32SysClkFreq);
}

void ClrScreen() {
	sRect.i16XMin = 0;
	sRect.i16YMin = 0;
	sRect.i16XMax = 319;
	sRect.i16YMax = 239;
	GrContextForegroundSet(&sContext, ClrBlack);
	GrRectFill(&sContext, &sRect);
	GrFlush(&sContext);
}

//Tommy Part///////////////////////////////////////////////////////////////////////////////////////

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
void uDMAErrorHandler(void) {
	uint32_t ui32Status;

	//
	// Check for uDMA error bit
	//
	ui32Status = uDMAErrorStatusGet();

	//
	// If there is a uDMA error, then clear the error and increment
	// the error counter.
	//
	if (ui32Status) {
		uDMAErrorStatusClear();
		g_ui32uDMAErrCount++;
	}
}

////////////////////////////////////////////////////////
// EPI interrupt fuction
//////////////////////////////////////////////////
void EPIIntHandler(void) {

	//EPINonBlockingReadConfigure(EPI0_BASE,0,EPI_NBCONFIG_SIZE_32,0);
	//EPINonBlockingReadStart(EPI0_BASE,0,CountSize);

	//
	// Check for the primary control structure to indicate complete.
	//
	ui32Mode = EPIIntStatus(EPI0_BASE, true);
	uDMAIntClear(UDMA_CHANNEL_SW);
	EPIIntErrorClear(EPI0_BASE, EPI_INT_ERR_DMARDIC);
	if (ui32Mode == EPI_INT_RXREQ) {
		//
		// Increment the count of completed transfers.
		//
		g_ui32MemXferCount++;

		//EPINonBlockingReadConfigure(EPI0_BASE,0,EPI_NBCONFIG_SIZE_32,0);
		//EPINonBlockingReadStart(EPI0_BASE,0,CountSize);

	}
	if (ui32Mode == EPI_INT_DMA_RX_DONE) { //EPI_IM_DMARDIM EPI0_EPI_IM EPI_INT_RXREQ

		Done++;

		pri = pui8ControlTable[488] & 0b11;
		alt = pui8ControlTable[1000] & 0b11;

		if (pri == 0) {
			uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
			UDMA_MODE_PINGPONG, EPISource, g_ui32DstBuf[0],
			MEM_BUFFER_SIZE);

			uDMAChannelEnable(UDMA_CHANNEL_SW);

		 	EPINonBlockingReadConfigure(EPI0_BASE, 0, EPI_NBCONFIG_SIZE_32, 0);
			EPINonBlockingReadStart(EPI0_BASE, 0, CountSize);
		}

		if (alt == 0) {
			uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_ALT_SELECT,
			UDMA_MODE_PINGPONG, EPISource, g_ui32DstBuf2[0],
			MEM_BUFFER_SIZE);

			uDMAChannelEnable(UDMA_CHANNEL_SW);

			EPINonBlockingReadConfigure(EPI0_BASE, 1, EPI_NBCONFIG_SIZE_32, 0);
			EPINonBlockingReadStart(EPI0_BASE, 1, CountSize);
		}

	}

	//
	// If the channel is not stopped, then something is wrong.
	//
	else {
		g_ui32BadISR++;

	}

}

//*****************************************************************************
//
// Initializes the uDMA software channel to perform a memory to memory uDMA
// transfer.
//
//*****************************************************************************
void InitSWTransfer(void) {

	//
	// Enable interrupts from the uDMA software channel.
	//
	IntEnable(INT_UDMA);

	//
	// Put the attributes in a known state for the uDMA software channel.
	// These should already be disabled by default.
	//
	uDMAChannelAttributeDisable(UDMA_CHANNEL_SW,
	UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | (UDMA_ATTR_HIGH_PRIORITY |
	UDMA_ATTR_REQMASK));

	uDMAChannelAttributeEnable(UDMA_CHANNEL_SW, UDMA_ATTR_HIGH_PRIORITY);

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
	UDMA_ARB_8);

	uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_ALT_SELECT,
	UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
	UDMA_ARB_8);

	//
	// Set up the transfer parameters for the software channel.  This will
	// configure the transfer buffers and the transfer size.  Auto mode must be
	// used for software transfers.
	//
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_ALT_SELECT,
	UDMA_MODE_PINGPONG, EPISource, g_ui32DstBuf2[0],
	MEM_BUFFER_SIZE);

	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
	UDMA_MODE_PINGPONG, EPISource, g_ui32DstBuf[0],
	MEM_BUFFER_SIZE);

	//
	// Now the software channel is primed to start a transfer.  The channel
	// must be enabled.  For software based transfers, a request must be
	// issued.  After this, the uDMA memory transfer begins.
	//
	uDMAChannelEnable(UDMA_CHANNEL_SW);
}

// Main program//////////////////////////////////////////////////////////////////////////////
void setup(void) {

	for (f = 0; f < MEM_BUFFER_SIZE; f++) {
		g_ui32DstBuf[f] = &inputs[f];
	}

	for (f = 0; f < MEM_BUFFER_SIZE; f++) {
		g_ui32DstBuf2[f] = &inputs2[f];
	}

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

	//
	// Point at the control table to use for channel control structures.
	//
	uDMAControlBaseSet(pui8ControlTable);

	// Map Channel perhiperhal for 30
	uDMAChannelAssign(UDMA_CH30_EPI0RX);

	EPISource = EPI0_BASE + EPI_O_READFIFO0;

	InitSWTransfer();

	uint32_t ui32Period;

	// Enable GPIO and Timer peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	// Enable EPI Peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EPI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

	// Configure GPIO and Timer
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
	ui32Period = ui32SysClkFreq / 2;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period / 8 - 1);

	// Enable timer interrupt and set it to go off at Timer A timeouts
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	// Enable Timer A
	TimerEnable(TIMER0_BASE, TIMER_A);

	// Configure EPI settings
	EPIDividerSet(EPI0_BASE, 6500);
	EPIAddressMapSet(EPI0_BASE, EPI_ADDR_PER_SIZE_256B | EPI_ADDR_PER_BASE_A);
	EPIConfigGPModeSet(EPI0_BASE,
			EPI_GPMODE_DSIZE_32 | EPI_GPMODE_ASIZE_NONE | EPI_GPMODE_CLKPIN, 0,
			0);
	EPINonBlockingReadConfigure(EPI0_BASE, 0, EPI_NBCONFIG_SIZE_32, 0);
	EPINonBlockingReadStart(EPI0_BASE, 0, CountSize);

	EPIFIFOConfig(EPI0_BASE, EPI_FIFO_CONFIG_RX_1_2);
	EPIIntEnable(EPI0_BASE, EPI_INT_DMA_RX_DONE); //| EPI_INT_RXREQ );
	IntEnable(INT_EPI0);

}

void Timer0IntHandler(void) {
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	//WidgetRemove((tWidget *) &g_sWaveform);

	//
	// Add the first panel to the widget tree.
	//
	//WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sWaveform);

	//
	// Issue the initial paint request to the widgets.
	//
	WidgetPaint((tWidget *)&g_sWaveform);

	//
	// Process any messages in the widget message queue.
	//
	WidgetMessageQueueProcess();

}

//End Tommy Part////////////////////////////////////////////////////////////////////
