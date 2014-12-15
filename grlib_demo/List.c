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
#include "grlib/container.h"
#include "grlib/pushbutton.h"
#include "grlib/radiobutton.h"
#include "Kentec320x240x16_ssd2119_8bit.h"
#include "touch.h"
#include "grlib/listbox.h"
#include "test.h" //test for string table



uint32_t ui32SysClkFreq;

extern tCanvasWidget g_sBackground;
extern tPushButtonWidget g_sPushBtn;
extern tPushButtonWidget g_sPushBtn1;
extern tContainerWidget g_sContainer;
extern tRadioButtonWidget g_sRadioBtn0;
extern tRadioButtonWidget g_sRadioBtn1;



extern tListBoxWidget g_sListBox;
extern const uint8_t g_pui8Tabletest[];

const char *StrTable[3] = {
"a","b","c"
};

void OnButtonPress(tWidget *pWidget);
void DListBox(tWidget *pWidgetL);
void DRadio(tWidget *pWidgetR);

Canvas(g_sHeading, &g_sBackground, 0, &g_sPushBtn, &g_sKentec320x240x16_SSD2119,
		0, 0, 320, 23,
		(CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_TEXT),
		ClrBlack, ClrWhite, ClrRed, g_psFontCm20, "LED Control", 0, 0);


Canvas(g_sBackground, WIDGET_ROOT, 0, &g_sHeading, &g_sKentec320x240x16_SSD2119,
		0, 23, 320, (240 - 23), CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

RectangularButton(g_sPushBtn, &g_sHeading, &g_sPushBtn1, 0, &g_sKentec320x240x16_SSD2119,
		60, 60, 100, 20,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrRed, ClrRed, g_psFontCmss18b, "Toggle LEDs", 0, 0,
		0, 0, DListBox);


RectangularButton(g_sPushBtn1, &g_sHeading, 0, 0, &g_sKentec320x240x16_SSD2119,
		200, 60, 100, 20,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrRed, ClrRed, g_psFontCmss18b, "BBB", 0, 0,
		0, 0, DRadio);
ListBox(g_sListBox, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 100, 100,
			100, 100, LISTBOX_STYLE_OUTLINE, ClrBlack,
			ClrWhite, ClrRed, ClrRed, ClrWhite, g_psFontCmss14b, &StrTable,3,3, 0);

Container(g_sContainer,0,0,&g_sRadioBtn0,&g_sKentec320x240x16_SSD2119,200, 80, 100, 100,(CTR_STYLE_OUTLINE |CTR_STYLE_FILL |CTR_STYLE_TEXT),ClrBlack, ClrWhite,ClrRed,g_psFontCmss18b, "TRY");
RadioButton(g_sRadioBtn0,&g_sContainer,&g_sRadioBtn1,0,&g_sKentec320x240x16_SSD2119,210,100,80,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss14b,"   111",0,0);
RadioButton(g_sRadioBtn1,&g_sContainer,0,0,&g_sKentec320x240x16_SSD2119,210,140,80,20,(RB_STYLE_FILL|RB_STYLE_OUTLINE|RB_STYLE_SELECTED|RB_STYLE_TEXT),10,ClrBlack,ClrWhite,ClrRed,g_psFontCmss14b,"   222",0,0);



bool HH=false;
void DRadio(tWidget *pWidgetR){
	HH=!HH;
	if(HH){
	//	WidgetAdd((tWidget *)&g_sPushBtn1, (tWidget *)&g_sListBox);
		//WidgetAdd((tWidget *)&g_sListBox);
		WidgetPaint((tWidget *)&g_sContainer);
	}
	else{
		WidgetRemove((tWidget *)&g_sContainer);
		//WidgetPaint(WIDGET_ROOT);
		WidgetPaint((tWidget *)&g_sBackground);
	}

}





bool GG=false;
void DListBox(tWidget *pWidgetL){
	GG=!GG;
	if(GG){
	//	WidgetAdd((tWidget *)&g_sPushBtn1, (tWidget *)&g_sListBox);
		//WidgetAdd((tWidget *)&g_sListBox);
		WidgetPaint((tWidget *)&g_sListBox);
	}
	else{
		WidgetRemove((tWidget *)&g_sListBox);
		//WidgetPaint(WIDGET_ROOT);
		WidgetPaint((tWidget *)&g_sBackground);
	}

}

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

