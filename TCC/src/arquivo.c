//*****************************************************************************
//
//
// Copyright (c) 2012-2018 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"


#define USER_LED1  GPIO_PIN_0
#define USER_LED2  GPIO_PIN_1
void init_TIMER(void);
void init_ADC(void);

uint32_t ui32SysClock;

int
main(void)
{

    //
    // Run from the PLL at 120 MHz.
    //
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_320), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);  //Enable the clock to TIMER0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);    //Enable the clock to ADC module
    //
    // Enable and wait for the port to be ready for access
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }
    
    //
    // Configure the GPIO port for the LED operation.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, (USER_LED1|USER_LED2));

    init_ADC();
    init_TIMER();
    IntMasterEnable();
    TimerEnable(TIMER0_BASE, TIMER_A); // Start everything
    //
    // Loop Forever
    //
    while(1)
    {
//        GPIOPinWrite(GPIO_PORTN_BASE, (USER_LED1|USER_LED2), 0);
    	HWREG(GPIO_PORTN_BASE + 4) = 0;
    }
}

void init_TIMER()
{
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    // Set sample frequency to 2MHz (every 62.5uS)
    TimerLoadSet(TIMER0_BASE, TIMER_A, 40 - 1);   //TODO: Timer Load Value is set here
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    TimerControlStall(TIMER0_BASE, TIMER_A, true); //Assist in debug by stalling timer at breakpoints
}

void init_ADC()
{
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    SysCtlDelay(80u);

    // Use ADC0 sequence 0 to sample channel 0 once for each timer period
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 5);

    SysCtlDelay(10); // Time for the clock configuration to set

    IntDisable(INT_ADC0SS0);
    ADCIntDisable(ADC0_BASE, 0u);
    ADCSequenceDisable(ADC0_BASE,0u);
    // With sequence disabled, it is now safe to load the new configuration parameters

    ADCSequenceConfigure(ADC0_BASE, 0u, ADC_TRIGGER_TIMER, 0u);
    ADCSequenceStepConfigure(ADC0_BASE,0u,0u,ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE);
    ADCSequenceEnable(ADC0_BASE,0u); //Once configuration is set, re-enable the sequencer
    ADCIntClear(ADC0_BASE,0u);
    ADCIntEnable(ADC0_BASE, 0u);
    IntEnable(INT_ADC0SS0);

}

void ADC0SS0IntHandler(void)
{
	HWREG(GPIO_PORTN_BASE + 4) = 1;
//	GPIOPinWrite(GPIO_PORTN_BASE, (USER_LED1|USER_LED2), USER_LED1);
    HWREG(ADC0_BASE + ADC_O_ISC) = 1 << 0;
//    ADCIntClear(ADC0_BASE, 0);
}