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
#include "driverlib/pwm.h"
#include "inc/hw_nvic.h"
#include "driverlib/flash.h"
#include "driverlib/udma.h"
#include "utils/ustdlib.h"
#include "images.h"
#include "inc/hw_epi.h"
#include "driverlib/epi.h"
#include "driverlib/timer.h"

// define epi port pins to be used
#define EPI_PORTA_PINS (GPIO_PIN_6 | GPIO_PIN_7)
#define EPI_PORTB_PINS (GPIO_PIN_2 | GPIO_PIN_3)
#define EPI_PORTC_PINS (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define EPI_PORTG_PINS (GPIO_PIN_0)
#define EPI_PORTK_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7)
#define EPI_PORTL_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4)
#define EPI_PORTM_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)



/*// define epi port pins to be used
#define EPI_PORTA_PINS (GPIO_PIN_6)
#define EPI_PORTC_PINS (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4)
#define EPI_PORTG_PINS (GPIO_PIN_0)
#define EPI_PORTL_PINS (GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0)
#define EPI_PORTM_PINS (GPIO_PIN_3)*/

// define ADC control pins
#define PADC_Output_Mode (GPIO_PIN_2)
#define PADC_CTRL_1 (GPIO_PIN_5)
#define FADC_CTRL_2 (GPIO_PIN_2)
#define FADC_CTRL_3 (GPIO_PIN_3)
#define FADC_CLK_OUT (GPIO_PIN_1)
#define LADC_CLK_IN (GPIO_PIN_5)

// define gpio pins for DVGA
#define ECh1_DVGA_Mode 	(GPIO_PIN_4)
#define ECh2_DVGA_Mode 	(GPIO_PIN_5)
#define ECh1_DVGA_D0	(GPIO_PIN_1)
#define DCh1_DVGA_D1	(GPIO_PIN_3)
#define ECh1_DVGA_D2	(GPIO_PIN_3)
#define MCh1_DVGA_D3	(GPIO_PIN_4)
#define ECh2_DVGA_D0	(GPIO_PIN_2)
#define ECh2_DVGA_D1	(GPIO_PIN_0)
#define DCh2_DVGA_D2	(GPIO_PIN_7)
#define MCh2_DVGA_D3	(GPIO_PIN_5)

// define gpio pins for multiplexer
#define BCh1_Mult_A0	(GPIO_PIN_4)
#define BCh1_Mult_A1	(GPIO_PIN_5)
#define ACh2_Mult_A0	(GPIO_PIN_4)
#define ACh2_Mult_A1	(GPIO_PIN_4)


//*****************************************************************************
//
// The size of the memory transfer source and destination buffers (in words).
//
//*****************************************************************************
#define MEM_BUFFER_SIZE         1024
#define MaxSize					1024*8 // Must be multiple of MEM_BUFFER_SIZE

//*****************************************************************************
//
// The source and destination buffers used for memory transfers.
//
//*****************************************************************************
static uint32_t *g_ui32DstBuf[MEM_BUFFER_SIZE];
static uint32_t *g_ui32DstBuf2[MEM_BUFFER_SIZE];
uint32_t values[MaxSize], values2[MEM_BUFFER_SIZE],
inputs[MEM_BUFFER_SIZE], inputs2[MEM_BUFFER_SIZE], all_values[MaxSize];

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

// global variables
uint32_t totalA, totalB, pixel_divider = 5;
uint32_t i = 0, j = 0, f = 0, k = 0, m = 0, l = 0, EPIDivide = 5;
uint16_t Max1, Max2, Min1, Min2, Amp1, Amp2, Freq1, Freq2;
uint32_t receive[24], oppreceive[24], total, CountSize = 1024, count = 0, pixel_total = 0, pixel_average, TriggerLevel = 1050;
uint8_t pri, alt, TriggerStart = 0, Trigger = 0, NumAvg = 2, GoThrough = 0, CaptureMode = 1, TriggerMode = 0;
uint16_t NumSkip = 1, TriggerPosition = 0, old[SERIES_LENGTH], pixels[SERIES_LENGTH], midlevel;
uint32_t ui32Mode;
uint32_t *EPISource;
uint8_t below = 0, above = 0, clockout = 0;
uint8_t Mag1 = 0, Mag2 = 0, Time = 0;
uint8_t transfer_done[2] = {0,0};

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
extern tCanvasWidget g_sAddMinusTime;
extern tPushButtonWidget g_sPushBtnAddC1;
extern tPushButtonWidget g_sPushBtnMinusC1;
extern tPushButtonWidget g_sPushBtnAddC2;
extern tPushButtonWidget g_sPushBtnMinusC2;
extern tPushButtonWidget g_sPushBtnAddTime;
extern tPushButtonWidget g_sPushBtnMinusTime;
extern tContainerWidget g_sContainerMenu;
extern tContainerWidget g_sContainerFreMagnitudeC1;
extern tContainerWidget g_sContainerFreMagnitudeC2;
extern tContainerWidget g_sContainerVolMagnitudeC1;
extern tContainerWidget g_sContainerVolMagnitudeC2;
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

char *tempMagVolDivC2;
char magVolDivC2[] = { 32, 32, 50, 109, 86, 47, 100, 105, 118, 0 };
char *tempMagVolDivC1;
char magVolDivC1[] = { 32, 32, 50, 109, 86, 47, 100, 105, 118, 0 };
char *tempTimVolDivC1;
char timVolDivC1[] = { 32, 50, 48, 110, 115, 47, 100, 105, 118, 0 };
void ClrScreen(void);
void DRadioMenu(tWidget *pWidgetR);
void DRadioFreMagnitudeC1(tWidget *pWidgetR);
void DRadioFreMagnitudeC2(tWidget *pWidgetR);
void DRadioVolMagnitudeC1(tWidget *pWidgetR);
void DRadioVolMagnitudeC2(tWidget *pWidgetR);
void DWaveForm(tWidget *pWidgetR, tContext *psContext);
void AddMinusFunctionC1(tWidget *pWidget);
void AddMinusFunctionC2(tWidget *pWidget);
void AddMinusFunctionTime(tWidget *pWidget);
void AddMagDivC1(tWidget *psWidget);
void MinusMagDivC1(tWidget *psWidget);
void AddMagDivC2(tWidget *psWidget);
void MinusMagDivC2(tWidget *psWidget);
void AddTimeDiv(tWidget *psWidget);
void MinusTimeDiv(tWidget *psWidget);
void ClrMyWidget();
double ASCtoDouble(char t[]);


tPushButtonWidget g_psTopButtons[];
tPushButtonWidget g_psBotButtons[];


void setup(void);
void SetupVoltageDivision(uint8_t Scale, uint8_t Channel);
void SetupTimeDivision(uint8_t Scale);
void SetupTrigger(uint8_t Level, uint8_t Start_Position, uint8_t Mode);

void SetupVoltageDivision(uint8_t Scale, uint8_t Channel){


	if(Channel == 1){
		switch (Scale) {
		case 0:
			pixel_divider = 5;
			// Smallest Channel 1
			/*GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D0, 0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D1, 0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2, 0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D3, 0);*/
			break;
		case 1:
			pixel_divider = 20;
			break;
		case 2:
			pixel_divider = 30;
			break;
		case 3:
			pixel_divider = 40;
			break;
		case 4:
			pixel_divider = 50;
			break;
		case 5:
			pixel_divider = 60;
			break;
		case 6:
			pixel_divider = 70;
			break;
		case 7:
			pixel_divider = 80;
			break;
		case 8:
			pixel_divider = 90;
			break;
		case 9:
			pixel_divider = 100;
			break;
		case 10:
			pixel_divider = 110;
			//Largest Channel 1
			/*GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0, BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1, BCh1_Mult_A1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D0, DCh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D1, ECh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2, ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D3, ECh1_DVGA_D3);*/
			break;
		default:
			/*GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1, BCh1_Mult_A1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D0, 0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D1, 0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2, ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D3, ECh1_DVGA_D3);*/
			break;
		}
	}
	else if(Channel == 2){
		switch (Scale) {
		case 0:
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
			break;
		case 1:
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0, BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
			break;
		case 2:
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1, BCh1_Mult_A1);
			break;
		case 3:
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0, BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1, BCh1_Mult_A1);
			break;
		case 4:
			break;
		case 5:
			break;
		case 6:
			break;
		case 7:
			break;
		case 8:
			break;
		case 9:
			break;
		case 10:
			break;
		default:
			break;
		}
	}
}

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
Canvas(g_sAddMinusTime, 0, 0, &g_sPushBtnAddTime, &g_sKentec320x240x16_SSD2119, 106,
		29, 52, 52,
		CANVAS_STYLE_FILL|CANVAS_STYLE_OUTLINE|CANVAS_STYLE_TEXT_VCENTER|CANVAS_STYLE_TEXT,
		ClrBlack, ClrWhite, ClrWhite, g_psFontCmss12, timVolDivC1, 0, 0);

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
										g_psFontCmss12, "20ns/div", 0, 0, 0, 0, AddMinusFunctionTime),

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
////////////////////////////////////////////////////////////////////////////////////////////
////Magnitude Division Buttons//////////////////////////////////////////////////////////////////////////
RectangularButton(g_sPushBtnAddC1, &g_sAddMinusC1, &g_sPushBtnMinusC1, 0,
		&g_sKentec320x240x16_SSD2119, 1, 30, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrRed, g_psFontCm16, "+", 0, 0, 0, 0,
		AddMagDivC1);

RectangularButton(g_sPushBtnMinusC1, &g_sAddMinusC1, 0, 0,
		&g_sKentec320x240x16_SSD2119, 1, 61, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrRed, g_psFontCm16, "-", 0, 0, 0, 0,
		MinusMagDivC1);

RectangularButton(g_sPushBtnAddC2, &g_sAddMinusC2, &g_sPushBtnMinusC2, 0,
		&g_sKentec320x240x16_SSD2119, 54, 30, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrYellow, g_psFontCm16, "+", 0, 0, 0, 0,
		AddMagDivC2);

RectangularButton(g_sPushBtnMinusC2, &g_sAddMinusC2, 0, 0,
		&g_sKentec320x240x16_SSD2119, 54, 61, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrYellow, g_psFontCm16, "-", 0, 0, 0, 0,
		MinusMagDivC2);
//////////////////////////////////////////////////////////////////////////////////////
///Time Division Buttons ////////////////////////////////////////////////////////////
RectangularButton(g_sPushBtnAddTime, &g_sAddMinusTime, &g_sPushBtnMinusTime, 0,
		&g_sKentec320x240x16_SSD2119, 106, 30, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrWhite, g_psFontCm16, "+", 0, 0, 0, 0,
		AddTimeDiv);

RectangularButton(g_sPushBtnMinusTime, &g_sAddMinusTime, 0, 0,
		&g_sKentec320x240x16_SSD2119, 106, 61, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrWhite, g_psFontCm16, "-", 0, 0, 0, 0,
		MinusTimeDiv);
///////////////////////////////////////////////////////////////////////////
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
										DRadioFreMagnitudeC1),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 2, 0,
										&g_sKentec320x240x16_SSD2119, 53, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrYellow,
										g_psFontCm16, "Hz", 0, 0, 0, 0, DRadioFreMagnitudeC2),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 3, 0,
										&g_sKentec320x240x16_SSD2119, 106, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrRed,
										g_psFontCm16, "V", 0, 0, 0, 0, DRadioVolMagnitudeC1),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 4, 0,
										&g_sKentec320x240x16_SSD2119, 159, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrYellow,
										g_psFontCm16, "V", 0, 0, 0, 0, DRadioVolMagnitudeC2),
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
///////////////////////////////////////////////////////////////
///Radio Buttons for the menu//////////////////////////////////////////////
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

////Radio Buttons for Hz and V Pushbuttons////////////////////////////////////
tRadioButtonWidget g_psRadioBtnFreqMagC1[] = {
RadioButtonStruct(&g_sContainerFreMagnitudeC1, g_psRadioBtnFreqMagC1 + 1, 0,
		&g_sKentec320x240x16_SSD2119, 1, 142, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Maximum", 0, 0),
RadioButtonStruct(&g_sContainerFreMagnitudeC1, g_psRadioBtnFreqMagC1+ 2, 0,
		&g_sKentec320x240x16_SSD2119, 1, 163, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Minimum", 0, 0),
RadioButtonStruct(&g_sContainerFreMagnitudeC1, 0, 0,
		&g_sKentec320x240x16_SSD2119, 1, 184, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Average", 0, 0)

};
tRadioButtonWidget g_psRadioBtnFreqMagC2[] = {
RadioButtonStruct(&g_sContainerFreMagnitudeC2, g_psRadioBtnFreqMagC2 + 1, 0,
		&g_sKentec320x240x16_SSD2119, 54, 142, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Maximum", 0, 0),
RadioButtonStruct(&g_sContainerFreMagnitudeC2, g_psRadioBtnFreqMagC2+ 2, 0,
		&g_sKentec320x240x16_SSD2119, 54, 163, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Minimum", 0, 0),
RadioButtonStruct(&g_sContainerFreMagnitudeC2, 0, 0,
		&g_sKentec320x240x16_SSD2119, 54, 184, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Average", 0, 0)

};
tRadioButtonWidget g_psRadioBtnVolMagC1[] = {
RadioButtonStruct(&g_sContainerVolMagnitudeC1, g_psRadioBtnVolMagC1 + 1, 0,
		&g_sKentec320x240x16_SSD2119, 107, 142, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Maximum", 0, 0),
RadioButtonStruct(&g_sContainerVolMagnitudeC1, g_psRadioBtnVolMagC1+ 2, 0,
		&g_sKentec320x240x16_SSD2119, 107, 163, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Minimum", 0, 0),
RadioButtonStruct(&g_sContainerVolMagnitudeC1, 0, 0,
		&g_sKentec320x240x16_SSD2119, 107, 184, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Average", 0, 0)

};
tRadioButtonWidget g_psRadioBtnVolMagC2[] = {
RadioButtonStruct(&g_sContainerVolMagnitudeC2, g_psRadioBtnVolMagC2 + 1, 0,
		&g_sKentec320x240x16_SSD2119, 160, 142, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Maximum", 0, 0),
RadioButtonStruct(&g_sContainerVolMagnitudeC2, g_psRadioBtnVolMagC2+ 2, 0,
		&g_sKentec320x240x16_SSD2119, 160, 163, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Minimum", 0, 0),
RadioButtonStruct(&g_sContainerVolMagnitudeC2, 0, 0,
		&g_sKentec320x240x16_SSD2119, 160, 184, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Average", 0, 0)

};
//////////////////////////////////////////////////////////////////////////////////
Container(g_sContainerMenu, 0, 0, g_psRadioBtnMenu,
		&g_sKentec320x240x16_SSD2119, 265, 28, 52, 115,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);
Container(g_sContainerFreMagnitudeC1, 0, 0, g_psRadioBtnFreqMagC1,
		&g_sKentec320x240x16_SSD2119, 0, 141, 52, 70,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);
Container(g_sContainerFreMagnitudeC2, 0, 0, g_psRadioBtnFreqMagC2,
		&g_sKentec320x240x16_SSD2119, 52, 141, 52, 70,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrYellow,
		g_psFontCm14, 0);
Container(g_sContainerVolMagnitudeC1, 0, 0, g_psRadioBtnVolMagC1,
		&g_sKentec320x240x16_SSD2119, 106, 141, 52, 70,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);
Container(g_sContainerVolMagnitudeC2, 0, 0, g_psRadioBtnVolMagC2,
		&g_sKentec320x240x16_SSD2119, 159, 141, 52, 70,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrYellow,
		g_psFontCm14, 0);

#define NUM_RADIO1_BUTTONS      (sizeof(g_psRadioBtnVolDiv0) /   \
                                 sizeof(g_psRadioBtnVolDiv0[0]))
void AddMagDivC1(tWidget *psWidget) {

	if(Mag1 == 10)
		Mag1 = 10;
	else
		Mag1++;

	SetupVoltageDivision(Mag1, 1);

	midlevel = 1024/pixel_divider + 120;

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
void MinusMagDivC1(tWidget *psWidget) {

	if(Mag1 == 0)
		Mag1 = 0;
	else
		Mag1--;

	SetupVoltageDivision(Mag1, 1);

	midlevel = 1024/pixel_divider + 120;

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
void AddMagDivC2(tWidget *psWidget) {

	if(Mag2 == 10)
		Mag2 = 10;
	else
		Mag2++;

	SetupVoltageDivision(Mag2, 2);

	midlevel = 1024/pixel_divider + 120;

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
void MinusMagDivC2(tWidget *psWidget) {

	if(Mag2 == 0)
		Mag2 = 0;
	else
		Mag2--;

	SetupVoltageDivision(Mag2, 2);

	midlevel = 1024/pixel_divider + 120;

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



void AddTimeDiv(tWidget *psWidget) {

	if(Time == 10)
		Time = 10;
	else
		Time++;

	SetupTimeDivision(Time);

	EPIDividerSet(EPI0_BASE, EPIDivide);

	//make it 50 s
	if (timVolDivC1[1] == 53 && timVolDivC1[2] == 48 && timVolDivC1[3] == 32)
		timVolDivC1[2] = 48;
	//stay at 50 s
	else if (timVolDivC1[1] == 53 && timVolDivC1[2] == 48
			&& timVolDivC1[3] == 32)
		timVolDivC1[1] =53 ;

	// from 500 ns to 1 us
	else if(timVolDivC1[0]==53 && timVolDivC1[3]==110){
		tempTimVolDivC1=timVolDivC1+2;
		*tempTimVolDivC1=49;
		timVolDivC1[0]=32;
		timVolDivC1[1]=32;
		timVolDivC1[3]=117;
	}
	// from 500 us to 1 ms
	else if(timVolDivC1[0]==53 && timVolDivC1[3]==117){
		tempTimVolDivC1=timVolDivC1+2;
		*tempTimVolDivC1=49;
		timVolDivC1[0]=32;
		timVolDivC1[1]=32;
		timVolDivC1[3]=109;
	}
	// from 500 ms to 1 s
	else if(timVolDivC1[0]==53 && timVolDivC1[3]==109){
		tempTimVolDivC1=timVolDivC1+2;
		*tempTimVolDivC1=49;
		timVolDivC1[0]=32;
		timVolDivC1[1]=32;
		timVolDivC1[3]=32;
	}
	//every time from 5 to 10
	else if (*tempTimVolDivC1 == 53) {
		*tempTimVolDivC1 = 48;
		tempTimVolDivC1--;
		*tempTimVolDivC1 = 49;
	}
	//every time from 1 to 2
	else if (*tempTimVolDivC1 == 49)
		*tempTimVolDivC1 = 50;
	//every time from 2 to 5
	else if (*tempTimVolDivC1 == 50)
		*tempTimVolDivC1 = 53;
	CanvasTextSet(&g_sAddMinusTime, timVolDivC1);
	PushButtonTextSet(&g_psTopButtons[2], timVolDivC1);
	WidgetPaint((tWidget * ) &g_psTopButtons[2]);
	WidgetPaint((tWidget * ) &g_sAddMinusTime);
}
void MinusTimeDiv(tWidget *psWidget) {

	if(Time == 0)
		Time = 0;
	else
		Time--;

	SetupTimeDivision(Time);

	EPIDividerSet(EPI0_BASE, EPIDivide);

	if (timVolDivC1[0] == 32 && timVolDivC1[1] == 32
			&& timVolDivC1[2] == 49&& timVolDivC1[3] == 32) {
		tempTimVolDivC1 = timVolDivC1;
		*tempTimVolDivC1 = 53;
		timVolDivC1[1] = 48;
		timVolDivC1[2] = 48;
		timVolDivC1[3] = 109;
	}
	//1ms to 500us
	else if  (timVolDivC1[0] == 32 && timVolDivC1[1] == 32
			&& timVolDivC1[2] == 49 && timVolDivC1[3] == 109)  {
		tempTimVolDivC1 = timVolDivC1;
		*tempTimVolDivC1 = 53;
		timVolDivC1[1] = 48;
		timVolDivC1[2] = 48;
		timVolDivC1[3] = 117;
	}
	//1us to 500ns
	else if  (timVolDivC1[0] == 32 && timVolDivC1[1] == 32
			&& timVolDivC1[2] == 49 && timVolDivC1[3] == 117)  {
		tempTimVolDivC1 = timVolDivC1;
		*tempTimVolDivC1 = 53;
		timVolDivC1[1] = 48;
		timVolDivC1[2] = 48;
		timVolDivC1[3] = 110;
	}
	//stay at 20ns/div
	else if (timVolDivC1[1] == 50 && timVolDivC1[3] == 110)
		timVolDivC1[1] = 50;

	//from 5 to 2
	else if (*tempTimVolDivC1 == 53)
		*tempTimVolDivC1 = 50;
	//from 2 to 1
	else if (*tempTimVolDivC1 == 50)
		*tempTimVolDivC1 = 49;
   //from 1 to 0
	else if (*tempTimVolDivC1 == 49) {
		*tempTimVolDivC1 = 32;
		tempTimVolDivC1++;
		*tempTimVolDivC1 = 53;
	}

	CanvasTextSet(&g_sAddMinusTime, timVolDivC1);
	PushButtonTextSet(&g_psTopButtons[2], timVolDivC1);
	WidgetPaint((tWidget * ) &g_psTopButtons[2]);
	WidgetPaint((tWidget * ) &g_sAddMinusTime);
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
void AddMinusFunctionC1(tWidget *pWidget) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sAddMinusC1);
		WidgetPaint((tWidget * ) &g_sAddMinusC1);
	} else {
		ClrMyWidget();
	}

}
void AddMinusFunctionC2(tWidget *pWidget) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sAddMinusC2);
		WidgetPaint((tWidget * ) &g_sAddMinusC2);
	} else {
		ClrMyWidget();
	}

}
void AddMinusFunctionTime(tWidget *pWidget) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sAddMinusTime);
		WidgetPaint((tWidget * ) &g_sAddMinusTime);
	} else {
		ClrMyWidget();
	}

}

void DRadioFreMagnitudeC1(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerFreMagnitudeC1);
		WidgetPaint((tWidget * )&g_sContainerFreMagnitudeC1);
	} else {
		ClrMyWidget();
	}

}
void DRadioFreMagnitudeC2(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerFreMagnitudeC2);
		WidgetPaint((tWidget * )&g_sContainerFreMagnitudeC2);
	} else {
		ClrMyWidget();
	}

}
void DRadioVolMagnitudeC1(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerVolMagnitudeC1);
		WidgetPaint((tWidget * )&g_sContainerVolMagnitudeC1);
	} else {
		ClrMyWidget();
	}

}
void DRadioVolMagnitudeC2(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerVolMagnitudeC2);
		WidgetPaint((tWidget * )&g_sContainerVolMagnitudeC2);
	} else {
		ClrMyWidget();
	}

}
void DRadioMenu(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerMenu);
		WidgetPaint((tWidget * )&g_sContainerMenu);
	} else {
		ClrMyWidget();
	}

}
void DWaveForm(tWidget *pWidgetR, tContext *psContext) {
///////////////////////////////////////////////////////////////////////
//Fake Waveform
	//float fRadians1;
	//float fRadians2;
	FPULazyStackingEnable();
	FPUEnable();
	/*fRadians1 = ((4 * M_PI) / SERIES_LENGTH);
	fRadians2 = ((7 * M_PI) / SERIES_LENGTH);
	GrContextForegroundSet(&sContext, ClrRed);
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
		if((midlevel - pixels[x] / pixel_divider) < 29)
			GrCircleFill(&sContext, x, old[x] = 29, 1);
		else if ((midlevel - pixels[x] / pixel_divider) > 212)
			GrCircleFill(&sContext, x, old[x] = 212, 1);
		else
			GrCircleFill(&sContext, x, old[x] = midlevel - pixels[x] / pixel_divider, 1);

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
	tempTimVolDivC1 = timVolDivC1 + 1;



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

	setup();

	while (1) {

		if (1) {
					for (f = 0; f < MEM_BUFFER_SIZE; f++) {
						/*receive[0] = inputs[f] & 0b00000000000000000000000000010000;
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
						*/

						receive[0] = (inputs[f] & 0b00000000000000000000000000000001)/0b00000000000000000000000000000001;
						receive[1] = (inputs[f] & 0b00000000000000000000000000000010)/0b00000000000000000000000000000010;
						receive[2] = (inputs[f] & 0b00000000000000000000000000000100)/0b00000000000000000000000000000100;
						receive[3] = (inputs[f] & 0b00000000000000000000000000001000)/0b00000000000000000000000000001000;
						receive[4] = (inputs[f] & 0b00000000000000000000000000010000)/0b00000000000000000000000000010000;
						receive[5] = (inputs[f] & 0b00000000000000000000000000100000)/0b00000000000000000000000000100000;
						receive[6] = (inputs[f] & 0b00000000000000000000000001000000)/0b00000000000000000000000001000000;
						receive[7] = (inputs[f] & 0b00000000000000000000000010000000)/0b00000000000000000000000010000000;
						receive[8] = (inputs[f] & 0b00000000000000000000000100000000)/0b00000000000000000000000100000000;
						receive[9] = (inputs[f] & 0b00000000000000000000001000000000)/0b00000000000000000000001000000000;
						receive[10] = (inputs[f] & 0b00000000000000000000100000000000)/0b00000000000000000000100000000000;
						receive[11] = (inputs[f] & 0b00000000000000000001000000000000)/0b00000000000000000001000000000000;
						receive[12] = (inputs[f] & 0b00000000000000000010000000000000)/0b00000000000000000010000000000000;
						receive[13] = (inputs[f] & 0b00000000000000000100000000000000)/0b00000000000000000100000000000000;
						receive[14] = (inputs[f] & 0b00000000000000001000000000000000)/0b00000000000000001000000000000000;
						receive[15] = (inputs[f] & 0b00000000000000010000000000000000)/0b00000000000000010000000000000000;
						receive[16] = (inputs[f] & 0b00000000000000100000000000000000)/0b00000000000000100000000000000000;
						receive[17] = (inputs[f] & 0b00000000000001000000000000000000)/0b00000000000001000000000000000000;
						receive[18] = (inputs[f] & 0b00000000000010000000000000000000)/0b00000000000010000000000000000000;
						receive[19] = (inputs[f] & 0b00000001000000000000000000000000)/0b00000001000000000000000000000000;
						receive[20] = (inputs[f] & 0b00000010000000000000000000000000)/0b00000010000000000000000000000000;
						receive[21] = (inputs[f] & 0b00000100000000000000000000000000)/0b00000100000000000000000000000000;
						receive[22] = (inputs[f] & 0b00001000000000000000000000000000)/0b00001000000000000000000000000000;
						receive[23] = (inputs[f] & 0b00010000000000000000000000000000)/0b00010000000000000000000000000000;

						if(receive[11] == 0){

							totalA = 1024 + (receive[0] + 2*receive[1] + 4*receive[2] + 8*receive[3]
							         + 16*receive[4] + 32*receive[5] + 64*receive[6] + 128*receive[7]
							         + 1*(256*receive[8] + 512*receive[9] + 1024*receive[10]));
						}
						else{

							oppreceive[0] = !receive[0];
							oppreceive[1] = !receive[1];
							oppreceive[2] = !receive[2];
							oppreceive[3] = !receive[3];

							if(receive[0])
								receive[0] = 0;
							else
								receive[0] = 1;

							if(receive[1])
								receive[1] = 0;
							else
								receive[1] = 1;

							if(receive[2])
								receive[2] = 0;
							else
								receive[2] = 1;

							if(receive[3])
								receive[3] = 0;
							else
								receive[3] = 1;

							if(receive[4])
								receive[4] = 0;
							else
								receive[4] = 1;

							if(receive[5])
								receive[5] = 0;
							else
								receive[5] = 1;

							if(receive[6])
								receive[6] = 0;
							else
								receive[6] = 1;

							if(receive[7])
								receive[7] = 0;
							else
								receive[7] = 1;

							if(receive[8])
								receive[8] = 0;
							else
								receive[8] = 1;

							if(receive[9])
								receive[9] = 0;
							else
								receive[9] = 1;

							if(receive[10])
								receive[10] = 0;
							else
								receive[10] = 1;

							totalA = 1024 - (1 + receive[0] + 2*receive[1] + 4*receive[2] + 8*receive[3]
							         + 16*receive[4] + 32*receive[5] + 64*receive[6] + 128*receive[7]
							         + 1*(256*receive[8] + 512*receive[9] + 1024*receive[10]));
						}

						totalB = receive[12]/8192 + 2*(receive[13]/16384) + 4*(receive[14]/32768)
								 + 8*(receive[15]/65536) + 16*(receive[16]/131072) + 32*(receive[17]/262144)
								 + 64*(receive[18]/524288) + 128*(receive[19]/1048576)
								 + 256*(receive[20]/2097152) + 512*(receive[21]/4194304)
								 + 1024*(receive[22]/8388608) + 2048*(receive[23]/16777216);


						values[f + k*MEM_BUFFER_SIZE] = totalA;

						if(TriggerMode == 0){
							if(totalA <= TriggerLevel && totalA >= (TriggerLevel - 10))
								below = 1;

							if(totalA >= TriggerLevel && totalA >= (TriggerLevel + 10) && below == 1){
								TriggerStart = 1;
								Trigger = 1;
								below = 0;
							}
						}
						else if(TriggerMode == 1){
							if(totalA >= TriggerLevel)
								above = 0;

							if(totalA <= TriggerLevel && above == 0){
								TriggerStart = 1;
								Trigger = 1;
								above = 1;
							}
						}

						if(TriggerStart == 1){
							if(CaptureMode == 0){
								for(i=0;i<TriggerPosition;i++){
									if((int32_t) (f + k*MEM_BUFFER_SIZE - TriggerPosition + i) < 0){
										pixels[i] = values[MaxSize - (f + k*MEM_BUFFER_SIZE - TriggerPosition + i)];
									}
									else{
										pixels[i] = values[f + k*MEM_BUFFER_SIZE - TriggerPosition + i];
									}
								}
								if(j<NumSkip){
									j++;
								}
								else{
									j=0;
									if(m + TriggerPosition < SERIES_LENGTH){
										pixels[m + TriggerPosition] = totalA;
										m++;
									}
									else{
										Max1 = 0;
										Min1 = 4097;
										for(i=0;i<SERIES_LENGTH;i++){
											if(pixels[i] < Min1){
												Min1 = pixels[i];
											}
											if(pixels[i] > Max1){
												Max1 = pixels[i];
											}
										}
										Amp1 = Max1 - Min1;
										m = 0;
										TriggerStart = 0;
									}
								}
							}
							else if(CaptureMode == 1){
								for(i=0;i<TriggerPosition;i++){
									if((int32_t) (f + k*MEM_BUFFER_SIZE - TriggerPosition + i) < 0){
										pixels[i] = values[MaxSize - (f + k*MEM_BUFFER_SIZE - TriggerPosition + i)];
									}
									else{
										pixels[i] = values[f + k*MEM_BUFFER_SIZE - TriggerPosition + i];
									}
								}
								if(j<NumAvg){
									j++;
									pixel_total = pixel_total + totalA;
								}
								else{
									j=0;
									pixel_average = pixel_total/NumAvg;
									pixel_total = 0;

									if(m + TriggerPosition < SERIES_LENGTH){
										pixels[m + TriggerPosition] = pixel_average;
										m++;
									}
									else{
										Max1 = 0;
										Min1 = 4097;
										for(i=0;i<SERIES_LENGTH;i++){
											if(pixels[i] < Min1){
												Min1 = pixels[i];
											}
											if(pixels[i] > Max1){
												Max1 = pixels[i];
											}
										}
										Amp1 = Max1 - Min1;
										m = 0;
										TriggerStart = 0;
									}
								}
							}
						}
					}
					if(Trigger == 1){
						GoThrough = 0;
						Trigger = 0;
					}
					else
						GoThrough++;

					if(GoThrough >= 100){
						for(i=0;i<SERIES_LENGTH;i++){
							pixels[i] = 1024;
						}
					}

				}

				if(k*MEM_BUFFER_SIZE + MEM_BUFFER_SIZE < MaxSize)
					k++;

				else
					k = 0;


				if (1) {
					for (f = 0; f < MEM_BUFFER_SIZE; f++) {
						/*receive[0] = inputs2[f] & 0b00000000000000000000000000010000;
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
						*/

						receive[0] = (inputs2[f] & 0b00000000000000000000000000000001)/0b00000000000000000000000000000001;
						receive[1] = (inputs2[f] & 0b00000000000000000000000000000010)/0b00000000000000000000000000000010;
						receive[2] = (inputs2[f] & 0b00000000000000000000000000000100)/0b00000000000000000000000000000100;
						receive[3] = (inputs2[f] & 0b00000000000000000000000000001000)/0b00000000000000000000000000001000;
						receive[4] = (inputs2[f] & 0b00000000000000000000000000010000)/0b00000000000000000000000000010000;
						receive[5] = (inputs2[f] & 0b00000000000000000000000000100000)/0b00000000000000000000000000100000;
						receive[6] = (inputs2[f] & 0b00000000000000000000000001000000)/0b00000000000000000000000001000000;
						receive[7] = (inputs2[f] & 0b00000000000000000000000010000000)/0b00000000000000000000000010000000;
						receive[8] = (inputs2[f] & 0b00000000000000000000000100000000)/0b00000000000000000000000100000000;
						receive[9] = (inputs2[f] & 0b00000000000000000000001000000000)/0b00000000000000000000001000000000;
						receive[10] = (inputs2[f] & 0b00000000000000000000100000000000)/0b00000000000000000000100000000000;
						receive[11] = (inputs2[f] & 0b00000000000000000001000000000000)/0b00000000000000000001000000000000;
						receive[12] = (inputs2[f] & 0b00000000000000000010000000000000)/0b00000000000000000010000000000000;
						receive[13] = (inputs2[f] & 0b00000000000000000100000000000000)/0b00000000000000000100000000000000;
						receive[14] = (inputs2[f] & 0b00000000000000001000000000000000)/0b00000000000000001000000000000000;
						receive[15] = (inputs2[f] & 0b00000000000000010000000000000000)/0b00000000000000010000000000000000;
						receive[16] = (inputs2[f] & 0b00000000000000100000000000000000)/0b00000000000000100000000000000000;
						receive[17] = (inputs2[f] & 0b00000000000001000000000000000000)/0b00000000000001000000000000000000;
						receive[18] = (inputs2[f] & 0b00000000000010000000000000000000)/0b00000000000010000000000000000000;
						receive[19] = (inputs2[f] & 0b00000001000000000000000000000000)/0b00000001000000000000000000000000;
						receive[20] = (inputs2[f] & 0b00000010000000000000000000000000)/0b00000010000000000000000000000000;
						receive[21] = (inputs2[f] & 0b00000100000000000000000000000000)/0b00000100000000000000000000000000;
						receive[22] = (inputs2[f] & 0b00001000000000000000000000000000)/0b00001000000000000000000000000000;
						receive[23] = (inputs2[f] & 0b00010000000000000000000000000000)/0b00010000000000000000000000000000;


						if(receive[11] == 0){

							totalA = 1024 + (receive[0] + 2*receive[1] + 4*receive[2] + 8*receive[3]
							         + 16*receive[4] + 32*receive[5] + 64*receive[6] + 128*receive[7]
							         + 1*(256*receive[8] + 512*receive[9] + 1024*receive[10]));
						}
						else{
							if(receive[0])
								receive[0] = 0;
							else
								receive[0] = 1;

							if(receive[1])
								receive[1] = 0;
							else
								receive[1] = 1;

							if(receive[2])
								receive[2] = 0;
							else
								receive[2] = 1;

							if(receive[3])
								receive[3] = 0;
							else
								receive[3] = 1;

							if(receive[4])
								receive[4] = 0;
							else
								receive[4] = 1;

							if(receive[5])
								receive[5] = 0;
							else
								receive[5] = 1;

							if(receive[6])
								receive[6] = 0;
							else
								receive[6] = 1;

							if(receive[7])
								receive[7] = 0;
							else
								receive[7] = 1;

							if(receive[8])
								receive[8] = 0;
							else
								receive[8] = 1;

							if(receive[9])
								receive[9] = 0;
							else
								receive[9] = 1;

							if(receive[10])
								receive[10] = 0;
							else
								receive[10] = 1;

							totalA = 1024 - (1 + receive[0] + 2*receive[1] + 4*receive[2] + 8*receive[3]
							         + 16*receive[4] + 32*receive[5] + 64*receive[6] + 128*receive[7]
							         + 1*(256*receive[8] + 512*receive[9] + 1024*receive[10]));
						}

						totalB = receive[12]/8192 + 2*(receive[13]/16384) + 4*(receive[14]/32768)
								 + 8*(receive[15]/65536) + 16*(receive[16]/131072) + 32*(receive[17]/262144)
								 + 64*(receive[18]/524288) + 128*(receive[19]/1048576)
								 + 256*(receive[20]/2097152) + 512*(receive[21]/4194304)
								 + 1024*(receive[22]/8388608) + 2048*(receive[23]/16777216);


						values[f + k*MEM_BUFFER_SIZE] = totalA;

						if(TriggerMode == 0){
							if(totalA <= TriggerLevel && totalA >= (TriggerLevel - 10))
								below = 1;

							if(totalA >= TriggerLevel && totalA >= (TriggerLevel + 10) && below == 1){
								TriggerStart = 1;
								Trigger = 1;
								below = 0;
							}
						}
						else if(TriggerMode == 1){
							if(totalA >= TriggerLevel)
								above = 0;

							if(totalA <= TriggerLevel && above == 0){
								TriggerStart = 1;
								Trigger = 1;
								above = 1;
							}
						}

						if(TriggerStart == 1){
							if(CaptureMode == 0){
								for(i=0;i<TriggerPosition;i++){
									if((int32_t) (f + k*MEM_BUFFER_SIZE - TriggerPosition + i) < 0){
										pixels[i] = values[MaxSize - (f + k*MEM_BUFFER_SIZE - TriggerPosition + i)];
									}
									else{
										pixels[i] = values[f + k*MEM_BUFFER_SIZE - TriggerPosition + i];
									}
								}
								if(j<NumSkip){
									j++;
								}
								else{
									j=0;
									if(m + TriggerPosition< SERIES_LENGTH){
										pixels[m + TriggerPosition] = totalA;
										m++;
									}
									else{
										Max1 = 0;
										Min1 = 4097;
										for(i=0;i<SERIES_LENGTH;i++){
											if(pixels[i] < Min1){
												Min1 = pixels[i];
											}
											if(pixels[i] > Max1){
												Max1 = pixels[i];
											}
										}
										Amp1 = Max1 - Min1;
										m = 0;
										TriggerStart = 0;
									}
								}
							}
							else if(CaptureMode == 1){
								for(i=0;i<TriggerPosition;i++){
									if((int32_t) (f + k*MEM_BUFFER_SIZE - TriggerPosition + i) < 0){
										pixels[i] = values[MaxSize - (f + k*MEM_BUFFER_SIZE - TriggerPosition + i)];
									}
									else{
										pixels[i] = values[f + k*MEM_BUFFER_SIZE - TriggerPosition + i];
									}
								}
								if(j<NumAvg){
									j++;
									pixel_total = pixel_total + totalA;
								}
								else{
									j=0;
									pixel_average = pixel_total/NumAvg;
									pixel_total = 0;

									if(m + TriggerPosition < SERIES_LENGTH){
										pixels[m + TriggerPosition] = pixel_average;
										m++;
									}
									else{
										Max1 = 0;
										Min1 = 4097;
										for(i=0;i<SERIES_LENGTH;i++){
											if(pixels[i] < Min1){
												Min1 = pixels[i];
											}
											if(pixels[i] > Max1){
												Max1 = pixels[i];
											}
										}
										Amp1 = Max1 - Min1;
										m = 0;
										TriggerStart = 0;
									}
								}
							}
						}
					}
					if(Trigger == 1){
						GoThrough = 0;
						Trigger = 0;
					}
					else
						GoThrough++;

					if(GoThrough >= 100){
						for(i=0;i<SERIES_LENGTH;i++){
							pixels[i] = 1024;
						}
					}

				}
				if(k*MEM_BUFFER_SIZE + MEM_BUFFER_SIZE < MaxSize)
					k++;

				else
					k = 0;

		//WidgetMessageQueueProcess();

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

//Clean all the widget running and repaint BackGround and Waveform
void ClrMyWidget(){
	WidgetRemove((tWidget *) &g_sContainerMenu);
	WidgetRemove((tWidget *) &g_sContainerFreMagnitudeC1);
	WidgetRemove((tWidget *) &g_sContainerFreMagnitudeC2);
	WidgetRemove((tWidget *) &g_sContainerVolMagnitudeC1);
	WidgetRemove((tWidget *) &g_sContainerVolMagnitudeC2);
	WidgetRemove((tWidget *) &g_sAddMinusC1);
	WidgetRemove((tWidget *) &g_sAddMinusC2);
	WidgetRemove((tWidget *) &g_sAddMinusTime);

	WidgetPaint((tWidget * )&g_sBackground);
	WidgetPaint((tWidget * )&g_sWaveform);
}

//Transfre the ASCII value to the double value
double ASCtoDouble(char t[]){
	int x;
	int temp=0;
	long double num;
	for(x=0;x<3;x++){
		if(t[x]!=32)
			temp=temp+(int)t[x]*10^(2-x);
	}
	//if the unit is milli
	if(t[3]==109)
		num=temp/1000;
	//if the unit is micro
	else if(t[3]==117)
		 num=temp/1000000;
	//if the unit is nano
	else if(t[3]==110)
		num=temp/1000000000;
	//printf("%d\n",num);

	 return num;
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

	midlevel = 1024/pixel_divider + 120;

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
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

	// Enable Control Periphs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// Enable PWM Periphs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	// Configure Timer
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	//TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

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
	GPIOPinConfigure(GPIO_PK0_EPI0S0);
	GPIOPinConfigure(GPIO_PK1_EPI0S1);
	GPIOPinConfigure(GPIO_PK2_EPI0S2);
	GPIOPinConfigure(GPIO_PK3_EPI0S3);
	GPIOPinConfigure(GPIO_PA7_EPI0S9);
	GPIOPinConfigure(GPIO_PM2_EPI0S13);
	GPIOPinConfigure(GPIO_PM1_EPI0S14);
	GPIOPinConfigure(GPIO_PM0_EPI0S15);
	GPIOPinConfigure(GPIO_PK7_EPI0S24);
	GPIOPinConfigure(GPIO_PK6_EPI0S25);
	GPIOPinConfigure(GPIO_PB2_EPI0S27);
	GPIOPinConfigure(GPIO_PB3_EPI0S28);

	GPIOPinTypeEPI(GPIO_PORTA_BASE, EPI_PORTA_PINS);
	GPIOPinTypeEPI(GPIO_PORTB_BASE, EPI_PORTB_PINS);
	GPIOPinTypeEPI(GPIO_PORTC_BASE, EPI_PORTC_PINS);
	GPIOPinTypeEPI(GPIO_PORTG_BASE, EPI_PORTG_PINS);
	GPIOPinTypeEPI(GPIO_PORTK_BASE, EPI_PORTK_PINS);
	GPIOPinTypeEPI(GPIO_PORTL_BASE, EPI_PORTL_PINS);
	GPIOPinTypeEPI(GPIO_PORTM_BASE, EPI_PORTM_PINS);

	// Set GPIO output and input pins for ADC control
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, FADC_CTRL_2 | FADC_CTRL_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, PADC_Output_Mode | PADC_CTRL_1);
	GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, LADC_CLK_IN);

	// Set inital controls for ADC
	GPIOPinWrite(GPIO_PORTP_BASE, PADC_Output_Mode| PADC_CTRL_1,0);
	GPIOPinWrite(GPIO_PORTF_BASE, FADC_CTRL_2 | FADC_CTRL_3, 0);

	// Set GPIO output pins for multiplexer
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, BCh1_Mult_A0);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, BCh1_Mult_A1);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, ACh2_Mult_A0);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, ACh2_Mult_A1);

	GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
	GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
	GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,0);
	GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,0);

	// Set GPIO output pins for DVGA
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh1_DVGA_Mode);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh2_DVGA_Mode);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh1_DVGA_D0);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, DCh1_DVGA_D1);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh1_DVGA_D2);
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, MCh1_DVGA_D3);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh2_DVGA_D0);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh2_DVGA_D1);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, DCh2_DVGA_D2);
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, MCh2_DVGA_D3);

	GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
	GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
	GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
	GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
	GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
	GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
	GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
	GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);

	// Select general purpose mode for EPI
	EPIModeSet(EPI0_BASE, EPI_MODE_GENERAL);

	// Configure GPIO pins for PWM
	GPIOPinConfigure(GPIO_PF1_M0PWM1);
	GPIOPinTypePWM(GPIO_PORTF_BASE, FADC_CLK_OUT);

	// Set Timer A to have a period of one tenth the clock freq
	ui32Period = ui32SysClkFreq / 2;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period / 8 - 1);
	//TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period / 5000000 - 1);

	// Enable timer interrupt and set it to go off at Timer A timeouts
	IntEnable(INT_TIMER0A);
	//IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	// Enable Timer A and B
	TimerEnable(TIMER0_BASE, TIMER_A);
	//TimerEnable(TIMER1_BASE, TIMER_A);


    // Set the PWM clock to the system clock.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	// Configure PWM for count down mode and immediate updates
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

	// Set PWM period
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 4);

	// Set pulse width of PWM1 for 50% duty cycle
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 2);

	// Start timers in generator 0
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);

	// Enable Outputs
	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

	// Configure EPI settings
	EPIDividerSet(EPI0_BASE, EPIDivide);
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

void Timer1IntHandler(void) {
	// Clear the timer interrupt
	/*TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	if(clockout == 0){
		GPIOPinWrite(GPIO_PORTP_BASE, PADC_CLK_OUT, 0);
		clockout = 1;
	}
	else{
		GPIOPinWrite(GPIO_PORTP_BASE, PADC_CLK_OUT, PADC_CLK_OUT);
		clockout = 0;
	}*/
}



void SetupTimeDivision(uint8_t Scale){

	switch(Scale){
	case 0:
		EPIDivide = 5;
		NumAvg = 1;
		NumSkip = 1;
		break;
	case 1:
		EPIDivide = 10;
		NumAvg = 1;
		NumSkip = 1;
		break;
	case 2:
		EPIDivide = 35;
		NumAvg = 1;
		NumSkip = 1;
		break;
	case 3:
		EPIDivide = 75;
		NumAvg = 1;
		NumSkip = 1;
		break;
	case 4:
		EPIDivide = 100;
		NumAvg = 1;
		NumSkip = 1;
		break;
	case 5:
		EPIDivide = 200;
		NumAvg = 1;
		NumSkip = 1;
		break;
	case 6:
		EPIDivide = 500;
		NumAvg = 1;
		NumSkip = 1;
		break;
	case 7:
		EPIDivide = 1000;
		NumAvg = 1;
		NumSkip = 1;
		break;
	case 8:
		EPIDivide = 2500;
		NumAvg = 1;
		NumSkip = 1;
		break;
	case 9:
		EPIDivide = 6500;
		NumAvg = 1;
		NumSkip = 1;
		break;
	case 10:
		EPIDivide = 6500;
		NumAvg = 5;
		NumSkip = 5;
		break;
	default:
		EPIDivide = 10;
		NumAvg = 1;
		NumSkip = 1;
		break;
	}
}

void SetupTrigger(uint8_t Level, uint8_t Start_Position, uint8_t Mode){

	TriggerLevel = Level;
	TriggerMode = Mode; // 0 - positive edge, 1 - negative edge
	TriggerPosition = Start_Position;
	above = 0;
	below = 0;
	Trigger = 0;
	TriggerStart = 0;
	GoThrough = 0;
}
//End Tommy Part////////////////////////////////////////////////////////////////////
