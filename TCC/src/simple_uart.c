#include <stdint.h>
#include <stdbool.h>
// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


#include "system_TM4C1294.h" 

uint32_t ui32adcValues[4],ui32Count,ui32adc0Values[1],ui32adc1Values[1],ui32adc2Values[1],ui32adc3Values[1];

void
ConfigureADC(void)
{


		SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOE);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
		SysCtlDelay(10);

	    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0 );

	    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_EIGHTH, 30);

   	ADCSequenceStepConfigure(ADC0_BASE,0,0, ADC_CTL_CH0);
   	ADCSequenceStepConfigure(ADC0_BASE,0,1, ADC_CTL_CH1);
   	ADCSequenceStepConfigure(ADC0_BASE,0,2, ADC_CTL_CH2);
   	ADCSequenceStepConfigure(ADC0_BASE,0,3, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);

   	ADCSequenceEnable(ADC0_BASE, 0);
   	ADCSequenceEnable(ADC0_BASE, 1);
   	ADCSequenceEnable(ADC0_BASE, 2);
   	ADCSequenceEnable(ADC0_BASE, 3);
}

uint32_t
WaitAndReadADC(uint32_t *adcValues)
{

	ADCProcessorTrigger(ADC0_BASE,0);
    // Wait until the sample sequence has completed.

    while(!ADCIntStatus(ADC0_BASE, 0, false)) { }
    // Read the value from the ADC.


    return(ADCSequenceDataGet(ADC0_BASE, 0, adcValues));
}

uint32_t
WaitAndReadADC1(uint32_t *adcValues)
{

	ADCProcessorTrigger(ADC0_BASE,1);
    // Wait until the sample sequence has completed.

    while(!ADCIntStatus(ADC0_BASE, 1, false)) { }
    // Read the value from the ADC.


    return(ADCSequenceDataGet(ADC0_BASE, 1, adcValues));
}

uint32_t
WaitAndReadADC2(uint32_t *adcValues)
{

	ADCProcessorTrigger(ADC0_BASE,2);
    // Wait until the sample sequence has completed.

    while(!ADCIntStatus(ADC0_BASE, 2, false)) { }
    // Read the value from the ADC.


    return(ADCSequenceDataGet(ADC0_BASE, 2, adcValues));
}

uint32_t
WaitAndReadADC3(uint32_t *adcValues)
{

	ADCProcessorTrigger(ADC0_BASE,3);
    // Wait until the sample sequence has completed.

    while(!ADCIntStatus(ADC0_BASE, 3, false)) { }
    // Read the value from the ADC.


    return(ADCSequenceDataGet(ADC0_BASE, 3, adcValues));
}


//*****************************************************************************
//
// Print "Hello World!" to the UART on the Intelligent UART Module.
//
//*****************************************************************************
int
main(void)
{

    //
    // Run from the PLL at 120 MHz.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                SYSCTL_CFG_VCO_480), 120000000);

    //
    // Configure the device pins.
    //
    PinoutSet();

    //
    // Enable the GPIO pins for the LED D1 (PN1).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    //
    // Initialize the UART.
    //
    ConfigureADC();

    //
    // Hello!
    //


    //
    // We are finished.  Hang around flashing D1.
    //
    while(1)
    {



        //
        // Turn on D1.
        //
        //LEDWrite(CLP_D1, 1);
     	//ui32Count = WaitAndReadADC0(ui32adc0Values);
     	//ui32Count = WaitAndReadADC1(ui32adc1Values);
     	//ui32Count = WaitAndReadADC2(ui32adc2Values);
     	ui32Count = WaitAndReadADC(ui32adcValues);



        //
        // Delay for a bit.
        //
       // SysCtlDelay(g_ui32SysClock / 10 / 3);

        //
        // Turn off D1.
        //
        //LEDWrite(CLP_D1, 0);

        //
        // Delay for a bit.
        //
       // SysCtlDelay(g_ui32SysClock / 10 / 3);
    }
}