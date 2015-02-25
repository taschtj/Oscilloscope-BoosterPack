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
uint32_t ui32SysClkFreq;

extern tCanvasWidget g_sBackground;
extern tPushButtonWidget g_sPushBtn;

void OnButtonPress(tWidget *pWidget);

Canvas(g_sHeading, &g_sBackground, 0, &g_sPushBtn, &g_sKentec320x240x16_SSD2119,
		0, 0, 320, 23,
		(CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_TEXT),
		ClrBlack, ClrWhite, ClrRed, g_psFontCm20, "LED Control", 0, 0);
Canvas(g_sBackground, WIDGET_ROOT, 0, &g_sHeading, &g_sKentec320x240x16_SSD2119,
		0, 23, 320, (240 - 23), CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

RectangularButton(g_sPushBtn, &g_sHeading, 0, 0, &g_sKentec320x240x16_SSD2119,
		60, 60, 200, 40,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrRed, ClrRed, g_psFontCmss22b, "Toggle LEDs", 0, 0,
		0, 0, OnButtonPress);

bool g_LedsOn = false;

void OnButtonPress(tWidget *pWidget) {
	g_LedsOn = !g_LedsOn;
	if (g_LedsOn) {
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0xFF);
	} else {
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);
	}
}

int main(void) {
	ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
	SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
	SYSCTL_CFG_VCO_480), 120000000);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);

	Kentec320x240x16_SSD2119Init(ui32SysClkFreq);
	TouchScreenInit(ui32SysClkFreq);
	TouchScreenCallbackSet(WidgetPointerMessage);

	WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sBackground);
	WidgetPaint(WIDGET_ROOT);
	while (1) {
		WidgetMessageQueueProcess();
	}
}

