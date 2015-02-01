/*
 * MainTestV4.c
 *
 *  Created on: Jan 28, 2014
 *      Author: zhuangr
 *
 *Update:  Simplify Code and add + and - functions
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
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
#include "driverlib/fpu.h"
#include "driverlib/debug.h"

uint32_t ui32SysClkFreq;

// Float points
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define SERIES_LENGTH 319
float gSeriesData[SERIES_LENGTH];
//
//Intro pictures
extern const uint8_t g_pui8Image[];
extern const uint8_t g_pui9Image[];

tContext sContext;
tRectangle sRect;
extern tCanvasWidget g_sBackground;
extern tCanvasWidget g_sWaveform;
extern tCanvasWidget g_sTop;
extern tCanvasWidget g_sBottom;
extern tCanvasWidget g_sAddMinusC1;
extern tCanvasWidget g_sAddMinusC2;
extern tPushButtonWidget g_sPushBtnAddC1;
extern tPushButtonWidget g_sPushBtnMinusC1;
extern tPushButtonWidget g_sPushBtnAddC2;
extern tPushButtonWidget g_sPushBtnMinusC2;
extern tContainerWidget g_sContainerMenu;
extern tContainerWidget g_sContainerMagnitude;
extern tRadioButtonWidget g_sRadioBtn2;
extern tRadioButtonWidget g_sRadioBtn3;
extern tRadioButtonWidget g_sRadioBtnAcquire;
extern tRadioButtonWidget g_sRadioBtnCoupling;
extern tRadioButtonWidget g_sRadioBtnMode;
extern tRadioButtonWidget g_sRadioBtnMath;
extern tRadioButtonWidget g_sRadioBtnMaximum;
extern tRadioButtonWidget g_sRadioBtnMinimum;
extern tRadioButtonWidget g_sRadioBtnAverage;
//for grid
int x;
int y;
//
char *tempMagVolDivC2;
char magVolDivC2[] = { 32, 32, 50, 109, 86, 47, 100, 105, 118, 0 };
char *tempMagVolDivC1;
char magVolDivC1[] = { 32, 32, 50, 109, 86, 47, 100, 105, 118, 0 };
void ClrScreen(void);
void DRadioMenu(tWidget *pWidgetR);
void DRadioMagnitude(tWidget *pWidgetR);
void DWaveForm(tWidget *pWidgetR, tContext *psContext);
void AddMinusFunctionC1(tWidget *pWidgetR);
void AddMinusFunctionC2(tWidget *pWidgetR);
void AddC1(tWidget *psWidget);
void MinusC1(tWidget *psWidget);
void Add(tWidget *psWidget);
void Minus(tWidget *psWidget);
tPushButtonWidget g_psTopButtons[];
tPushButtonWidget g_psBotButtons[];

Canvas(g_sBackground, WIDGET_ROOT, 0, &g_sTop, &g_sKentec320x240x16_SSD2119, 0,
		0, 320, 240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

Canvas(g_sWaveform, WIDGET_ROOT, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 29, 320,
		182, CANVAS_STYLE_APP_DRAWN, ClrBlack, 0, 0, 0, 0, 0, DWaveForm);

Canvas(g_sTop, &g_sBackground, &g_sBottom, g_psTopButtons,
		&g_sKentec320x240x16_SSD2119, 0, 0, 320, 28, CANVAS_STYLE_FILL,
		ClrBlack, 0, 0, 0, 0, 0, 0);
Canvas(g_sBottom, &g_sBackground, 0, g_psBotButtons,
		&g_sKentec320x240x16_SSD2119, 0, 212, 320, 28, CANVAS_STYLE_FILL,
		ClrBlack, 0, 0, 0, 0, 0, 0);

Canvas(g_sAddMinusC1, 0, 0, &g_sPushBtnAddC1, &g_sKentec320x240x16_SSD2119, 0,
		29, 52, 52,
		CANVAS_STYLE_FILL|CANVAS_STYLE_OUTLINE|CANVAS_STYLE_TEXT_VCENTER|CANVAS_STYLE_TEXT,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, magVolDivC1, 0, 0);

Canvas(g_sAddMinusC2, 0, 0, &g_sPushBtnAddC2, &g_sKentec320x240x16_SSD2119, 53,
		29, 52, 52,
		CANVAS_STYLE_FILL|CANVAS_STYLE_OUTLINE|CANVAS_STYLE_TEXT_VCENTER|CANVAS_STYLE_TEXT,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, magVolDivC2, 0, 0);

///Top Buttons///////////////////////////////////////////////////////
tPushButtonWidget g_psTopButtons[] =
		{
								RectangularButtonStruct(&g_sTop,
										g_psTopButtons + 1, 0,
										&g_sKentec320x240x16_SSD2119, 0, 0, 52,
										28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrRed,
										g_psFontCmss12, "2mV/div", 0, 0, 0, 0,
										AddMinusFunctionC1),

								RectangularButtonStruct(&g_sTop,
										g_psTopButtons + 2, 0,
										&g_sKentec320x240x16_SSD2119, 53, 0, 52,
										28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrYellow,
										g_psFontCmss12, "2mV/div", 0, 0, 0, 0,
										AddMinusFunctionC2),

								RectangularButtonStruct(&g_sTop,
										g_psTopButtons + 3, 0,
										&g_sKentec320x240x16_SSD2119, 106, 0,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCm16, "s/div", 0, 0, 0, 0, 0),

								RectangularButtonStruct(&g_sTop,
										g_psTopButtons + 4, 0,
										&g_sKentec320x240x16_SSD2119, 159, 0,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCm16, "Trigger", 0, 0, 0, 0, 0),

								RectangularButtonStruct(&g_sTop,
										g_psTopButtons + 5, 0,
										&g_sKentec320x240x16_SSD2119, 212, 0,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCm16, "Cursors", 0, 0, 0, 0, 0),

								RectangularButtonStruct(&g_sTop, 0, 0,
										&g_sKentec320x240x16_SSD2119, 265, 0,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCm16, "Menu", 0, 0, 0, 0,
										DRadioMenu) };

RectangularButton(g_sPushBtnAddC1, &g_sAddMinusC1, &g_sPushBtnMinusC1, 0,
		&g_sKentec320x240x16_SSD2119, 1, 30, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrRed, g_psFontCm16, "+", 0, 0, 0, 0,
		AddC1);

RectangularButton(g_sPushBtnMinusC1, &g_sAddMinusC1, 0, 0,
		&g_sKentec320x240x16_SSD2119, 1, 61, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrRed, g_psFontCm16, "-", 0, 0, 0, 0,
		MinusC1);

RectangularButton(g_sPushBtnAddC2, &g_sAddMinusC2, &g_sPushBtnMinusC2, 0,
		&g_sKentec320x240x16_SSD2119, 54, 30, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrYellow, g_psFontCm16, "+", 0, 0, 0, 0,
		Add);

RectangularButton(g_sPushBtnMinusC2, &g_sAddMinusC2, 0, 0,
		&g_sKentec320x240x16_SSD2119, 54, 61, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrYellow, g_psFontCm16, "-", 0, 0, 0, 0,
		Minus);

///Bottom Buttons///////////////////////////////////////////////////////
tPushButtonWidget g_psBotButtons[] =
		{
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 1, 0,
										&g_sKentec320x240x16_SSD2119, 0, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrRed,
										g_psFontCm16, "Hz", 0, 0, 0, 0,
										DRadioMagnitude),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 2, 0,
										&g_sKentec320x240x16_SSD2119, 53, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrYellow,
										g_psFontCm16, "Hz", 0, 0, 0, 0, 0),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 3, 0,
										&g_sKentec320x240x16_SSD2119, 106, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrRed,
										g_psFontCm16, "V", 0, 0, 0, 0, 0),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 4, 0,
										&g_sKentec320x240x16_SSD2119, 159, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrYellow,
										g_psFontCm16, "V", 0, 0, 0, 0, 0),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 5, 0,
										&g_sKentec320x240x16_SSD2119, 212, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCm16, "Auto", 0, 0, 0, 0, 0),
								RectangularButtonStruct(&g_sBottom, 0, 0,
										&g_sKentec320x240x16_SSD2119, 265, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCmss14, "Run/Stop", 0, 0, 0, 0,
										0) };

tRadioButtonWidget g_psRadioBtnMenu[] = {
RadioButtonStruct(&g_sContainerMenu, g_psRadioBtnMenu + 1, 0,
		&g_sKentec320x240x16_SSD2119, 266, 40, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Acquire", 0,
		0),
RadioButtonStruct(&g_sContainerMenu, g_psRadioBtnMenu + 2, 0,
		&g_sKentec320x240x16_SSD2119, 266, 61, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Coupling", 0,
		0),
RadioButtonStruct(&g_sContainerMenu, g_psRadioBtnMenu + 3, 0,
		&g_sKentec320x240x16_SSD2119, 266, 82, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Mode", 0, 0),
RadioButtonStruct(&g_sContainerMenu, 0, 0, &g_sKentec320x240x16_SSD2119,
		266, 103, 48, 20, RB_STYLE_TEXT, 10, ClrBlack, ClrWhite, ClrRed,
		g_psFontCmss12, "Math", 0, 0) };
tRadioButtonWidget g_psRadioBtnFreqMag0[] = {
RadioButtonStruct(&g_sContainerMagnitude, g_psRadioBtnFreqMag0 + 1, 0,
		&g_sKentec320x240x16_SSD2119, 1, 102, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Maximum", 0, 0),
RadioButtonStruct(&g_sContainerMagnitude, g_psRadioBtnFreqMag0 + 2, 0,
		&g_sKentec320x240x16_SSD2119, 1, 123, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Minimum", 0, 0),
RadioButtonStruct(&g_sContainerMagnitude, 0, 0,
		&g_sKentec320x240x16_SSD2119, 1, 144, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Average", 0, 0)

};
Container(g_sContainerMenu, 0, 0, g_psRadioBtnMenu,
		&g_sKentec320x240x16_SSD2119, 265, 28, 52, 115,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);
Container(g_sContainerMagnitude, 0, 0, g_psRadioBtnFreqMag0,
		&g_sKentec320x240x16_SSD2119, 0, 101, 52, 110,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);

#define NUM_RADIO1_BUTTONS      (sizeof(g_psRadioBtnVolDiv0) /   \
                                 sizeof(g_psRadioBtnVolDiv0[0]))
void AddC1(tWidget *psWidget) {
	//make it 25V
	if (magVolDivC1[1] == 50 && magVolDivC1[2] == 48 && magVolDivC1[3] == 32)
		magVolDivC1[2] = 53;
	//stay at 25V
	else if (magVolDivC1[1] == 50 && magVolDivC1[2] == 53
			&& magVolDivC1[3] == 32)
		magVolDivC1[2] = 53;
	// mV to V at 1 V
	else if (magVolDivC1[0] == 53) {
		tempMagVolDivC1 = magVolDivC1 + 2;
		*tempMagVolDivC1 = 49;
		magVolDivC1[0] = 32;
		magVolDivC1[1] = 32;
		magVolDivC1[3] = 32;
	}
	//every time from 5 to 10
	else if (*tempMagVolDivC1 == 53) {
		*tempMagVolDivC1 = 48;
		tempMagVolDivC1--;
		*tempMagVolDivC1 = 49;
	}
	//every time from 1 to 2
	else if (*tempMagVolDivC1 == 49)
		*tempMagVolDivC1 = 50;
	//every time from 2 to 5
	else if (*tempMagVolDivC1 == 50)
		*tempMagVolDivC1 = 53;
	CanvasTextSet(&g_sAddMinusC1, magVolDivC1);
	PushButtonTextSet(&g_psTopButtons[0], magVolDivC1);
	WidgetPaint((tWidget * ) &g_psTopButtons[0]);
	WidgetPaint((tWidget * ) &g_sAddMinusC1);
}
void MinusC1(tWidget *psWidget) {
	//25V to 20V
	if (magVolDivC1[1] == 50 && magVolDivC1[2] == 53 && magVolDivC1[3] == 32) {
		magVolDivC1[2] = 48;
		tempMagVolDivC1 = magVolDivC1 + 1;
	}
	//1V to 500mV
	else if (magVolDivC1[1] == 32 && magVolDivC1[2] == 49
			&& magVolDivC1[3] == 32) {
		tempMagVolDivC1 = magVolDivC1;
		*tempMagVolDivC1 = 53;
		magVolDivC1[1] = 48;
		magVolDivC1[2] = 48;
		magVolDivC1[3] = 109;
	}
	//stay at 2mv/div
	else if (magVolDivC1[2] == 50 && magVolDivC1[3] == 109)
		magVolDivC1[2] = 50;
	else if (*tempMagVolDivC1 == 50)
		*tempMagVolDivC1 = 49;
	else if (*tempMagVolDivC1 == 49) {
		*tempMagVolDivC1 = 32;
		tempMagVolDivC1++;
		*tempMagVolDivC1 = 53;
	} else if (*tempMagVolDivC1 == 53)
		*tempMagVolDivC1 = 50;

	CanvasTextSet(&g_sAddMinusC1, magVolDivC1);
	PushButtonTextSet(&g_psTopButtons[0], magVolDivC1);
	WidgetPaint((tWidget * ) &g_psTopButtons[0]);
	WidgetPaint((tWidget * ) &g_sAddMinusC1);
}
void Add(tWidget *psWidget) {
	//make it 25V
	if (magVolDivC2[1] == 50 && magVolDivC2[2] == 48 && magVolDivC2[3] == 32)
		magVolDivC2[2] = 53;
	//stay at 25V
	else if (magVolDivC2[1] == 50 && magVolDivC2[2] == 53
			&& magVolDivC2[3] == 32)
		magVolDivC2[2] = 53;
	// mV to V at 1 V
	else if (magVolDivC2[0] == 53) {
		tempMagVolDivC2 = magVolDivC2 + 2;
		*tempMagVolDivC2 = 49;
		magVolDivC2[0] = 32;
		magVolDivC2[1] = 32;
		magVolDivC2[3] = 32;
	}
	//every time from 5 to 10
	else if (*tempMagVolDivC2 == 53) {
		*tempMagVolDivC2 = 48;
		tempMagVolDivC2--;
		*tempMagVolDivC2 = 49;
	}
	//every time from 1 to 2
	else if (*tempMagVolDivC2 == 49)
		*tempMagVolDivC2 = 50;
	//every time from 2 to 5
	else if (*tempMagVolDivC2 == 50)
		*tempMagVolDivC2 = 53;
	CanvasTextSet(&g_sAddMinusC2, magVolDivC2);
	PushButtonTextSet(&g_psTopButtons[1], magVolDivC2);
	WidgetPaint((tWidget * ) &g_psTopButtons[1]);
	WidgetPaint((tWidget * ) &g_sAddMinusC2);
}
void Minus(tWidget *psWidget) {
	//25V to 20V
	if (magVolDivC2[1] == 50 && magVolDivC2[2] == 53 && magVolDivC2[3] == 32) {
		magVolDivC2[2] = 48;
		tempMagVolDivC2 = magVolDivC2 + 1;
	}
	//1V to 500mV
	else if (magVolDivC2[1] == 32 && magVolDivC2[2] == 49
			&& magVolDivC2[3] == 32) {
		tempMagVolDivC2 = magVolDivC2;
		*tempMagVolDivC2 = 53;
		magVolDivC2[1] = 48;
		magVolDivC2[2] = 48;
		magVolDivC2[3] = 109;
	}
	//stay at 2mv/div
	else if (magVolDivC2[2] == 50 && magVolDivC2[3] == 109)
		magVolDivC2[2] = 50;
	else if (*tempMagVolDivC2 == 50)
		*tempMagVolDivC2 = 49;
	else if (*tempMagVolDivC2 == 49) {
		*tempMagVolDivC2 = 32;
		tempMagVolDivC2++;
		*tempMagVolDivC2 = 53;
	} else if (*tempMagVolDivC2 == 53)
		*tempMagVolDivC2 = 50;

	CanvasTextSet(&g_sAddMinusC2, magVolDivC2);
	PushButtonTextSet(&g_psTopButtons[1], magVolDivC2);
	WidgetPaint((tWidget * ) &g_psTopButtons[1]);
	WidgetPaint((tWidget * ) &g_sAddMinusC2);
}
//void OnRadioChange(tWidget *psWidget, uint32_t bSelected) {
//	uint32_t ui32Idx;
//
//	//
//	// Find the index of this radio button in the first group.
//	//
//	for (ui32Idx = 0; ui32Idx < NUM_RADIO1_BUTTONS; ui32Idx++) {
//		if (psWidget == (tWidget *) (g_psRadioBtnVolDiv0 + ui32Idx)) {
//			break;
//		}
//	}
//}
bool ButtonTF = false;
void AddMinusFunctionC1(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sAddMinusC1);
		WidgetPaint((tWidget * ) &g_sAddMinusC1);
	} else {
		WidgetRemove((tWidget *) &g_sContainerMenu);
		WidgetRemove((tWidget *) &g_sContainerMagnitude);
		WidgetRemove((tWidget *) &g_sAddMinusC1);
		WidgetRemove((tWidget *) &g_sAddMinusC2);
		WidgetPaint((tWidget * )&g_sBackground);
		WidgetPaint((tWidget * )&g_sWaveform);
	}

}
void AddMinusFunctionC2(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sAddMinusC2);
		WidgetPaint((tWidget * ) &g_sAddMinusC2);
	} else {
		WidgetRemove((tWidget *) &g_sContainerMenu);
		WidgetRemove((tWidget *) &g_sContainerMagnitude);
		WidgetRemove((tWidget *) &g_sAddMinusC1);
		WidgetRemove((tWidget *) &g_sAddMinusC2);
		WidgetPaint((tWidget * )&g_sBackground);
		WidgetPaint((tWidget * )&g_sWaveform);
	}

}
void DRadioMagnitude(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerMagnitude);
		WidgetPaint((tWidget * )&g_sContainerMagnitude);
	} else {
		WidgetRemove((tWidget *) &g_sContainerMenu);
		WidgetRemove((tWidget *) &g_sContainerMagnitude);
		WidgetRemove((tWidget *) &g_sAddMinusC1);
		WidgetRemove((tWidget *) &g_sAddMinusC2);
		WidgetPaint((tWidget * )&g_sBackground);
		WidgetPaint((tWidget * )&g_sWaveform);
	}

}
void DRadioMenu(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerMenu);
		WidgetPaint((tWidget * )&g_sContainerMenu);
	} else {
		WidgetRemove((tWidget *) &g_sContainerMenu);
		WidgetRemove((tWidget *) &g_sContainerMagnitude);
		WidgetRemove((tWidget *) &g_sAddMinusC1);
		WidgetRemove((tWidget *) &g_sAddMinusC2);
		WidgetPaint((tWidget * )&g_sBackground);
		WidgetPaint((tWidget * )&g_sWaveform);
	}

}
void DWaveForm(tWidget *pWidgetR, tContext *psContext) {
///////////////////////////////////////////////////////////////////////
//Fake Waveform
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
//////////////////////////////////////////////////////////////////////
//change this for vertical division
	for (x = 0; x <= 320; x += 4) {
		for (y = 29; y <= 210; y += 15)
			GrPixelDraw(&sContext, x, y);
	}
//change this for horizontal division
	for (x = 0; x < 321; x += 32) {
		for (y = 29; y <= 210; y += 4)
			GrPixelDraw(&sContext, x, y);
	}
}
//
int main(void) {
	ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
	SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
	SYSCTL_CFG_VCO_480), 120000000);
	tempMagVolDivC1 = magVolDivC1 + 2;
	tempMagVolDivC2 = magVolDivC2 + 2;
	Kentec320x240x16_SSD2119Init(ui32SysClkFreq);
	GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);
	TouchScreenInit(ui32SysClkFreq);
	TouchScreenCallbackSet(WidgetPointerMessage);

//intro pictures
/////////////////////////////////////////////
//	GrImageDraw(&sContext, g_pui8Image, 0, 0);
//	GrFlush(&sContext);
//	SysCtlDelay(ui32SysClkFreq);
//
//	GrImageDraw(&sContext, g_pui9Image, 0, 0);
//	GrFlush(&sContext);
//	SysCtlDelay(ui32SysClkFreq);
//	ClrScreen();
//////////////////////////////////////////////
	WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sBackground);
	WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sWaveform);
	WidgetPaint(WIDGET_ROOT);

	while (1) {
		WidgetMessageQueueProcess();
	}
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

