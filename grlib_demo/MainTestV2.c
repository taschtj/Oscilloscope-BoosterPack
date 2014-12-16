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


Canvas(g_sBackground, WIDGET_ROOT, 0, &g_sTop, &g_sKentec320x240x16_SSD2119,
		0, 0, 320,240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

Canvas(g_sWaveform, WIDGET_ROOT, 0,0, &g_sKentec320x240x16_SSD2119,
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
	float fRadians1;
	float fRadians2;
	FPULazyStackingEnable();
	FPUEnable();
	fRadians1 = ((4 * M_PI) / SERIES_LENGTH);
	fRadians2 = ((7 * M_PI) / SERIES_LENGTH);
	GrContextForegroundSet(&sContext, ClrRed);
	for (x = 0; x < SERIES_LENGTH; x++) {
		GrCircleFill(&sContext, x, 75 - sinf(fRadians1 * x) * 40, 1);

	}
	GrContextForegroundSet(&sContext, ClrYellow);
	for (x = 0; x < SERIES_LENGTH; x++) {
		GrCircleFill(&sContext, x, 163 - sinf(fRadians2 * x) * 40, 1);

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

		WidgetMessageQueueProcess();
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

