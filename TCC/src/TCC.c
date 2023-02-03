/*
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
// includes da biblioteca driverlib
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "grlib/grlib.h"
#include "cfaf128x128x16.h" // Projects/drivers
#include "LM35.h"

typedef enum {InitialAutomaticMode, AwaitingRain, RemovingWasteWater, WaterCollect, WaterRemoval} state_Automatic;
typedef enum {InitialManualMode, Sanitazing, InternalRecircularion, AbsoluteTankLevel, RelativeTankLevel, TemperatureRead, WaterOutput} state_Manual;

uint8_t LED_D1 = 0;
tContext sContext;
float TEMP;
int Cuenta_ADC_ISR;

void ISR_ADC0_SS3(void){
int ADC_Dato;
float Voltaje;

ADC_Dato = (ADC0_SSFIFO3_R & 0xFFF);                                //Guarda FIFO3
Voltaje = ADC_Dato * (3.29/4095.0);                                 //Convierte a voltaje
TEMP=Voltaje*100;                                                   //Convierte a temperatura
Cuenta_ADC_ISR=1;                                                   //Condicion de salida LM35_ReadTemp
ADC0_ISC_R |= 0x0008;                                               //Limpia bandera de interrupción
}

void TemparatureSensor_Init()
{
   SYSCTL_RCGCGPIO_R |= 0x0010;                                    // Habilita reloj para Puerto E
    while( (SYSCTL_PRGPIO_R & 0x010) ==0);
    GPIO_PORTE_AHB_DIR_R &= ~0x10;                                  // PE4 como entrada
    GPIO_PORTE_AHB_AFSEL_R |= 0x10;                                 // Función Alterna de PE4
    GPIO_PORTE_AHB_DEN_R &= 0x10;                                   // Deshabilita Función Digital de PE4
    GPIO_PORTE_AHB_AMSEL_R |= 0x10;                                 // Habilita Función Analógica de PE4

    SYSCTL_RCGCADC_R = 0x01;                                        // Habilita reloj del ADC0
    while((SYSCTL_PRADC_R&0x01)==0);
    ADC0_PC_R = 0x01;                                               // Configura para 125Ks/s
    ADC0_SSPRI_R = 0x0123;                                          // SS3 con la más alta prioridad
    ADC0_ACTSS_R = 0x0000;                                          // Deshabilita SS3 antes de cambiar configuración de registros
    ADC0_SAC_R = 0X6;                                               // 64x Hardware oversampling
    ADC0_EMUX_R = 0x0000;                                           // Iniciar muestreo por software
    ADC0_SSEMUX3_R = 0x00;                                          // Rango de entradas para bit EMUX0: AIN[15:0]
    ADC0_SSMUX3_R = 0X9;                                            // Para bit MUX0: Canal AIN9 => PE4
    ADC0_SSCTL3_R = 0x0006;                                         // Entrada externa, IN, Ultima muestra, no diferencial
    ADC0_ISC_R = 0x0008;                                            // Limpia la bandera RIS del ADC0
    ADC0_IM_R = 0x0008;                                             // Habilita interrupciones de SS3
    ADC0_ACTSS_R |= 0x0008;                                         // Habilita SS3

    SYSCTL_PLLFREQ0_R |= SYSCTL_PLLFREQ0_PLLPWR;                    // Enciende PLL
    while((SYSCTL_PLLSTAT_R&0x01)==0);                              // Espera a que el PLL fije su frecuencia
    SYSCTL_PLLFREQ0_R &= ~SYSCTL_PLLFREQ0_PLLPWR;                   // Apaga PLL
    //ADC0 SS3 -> #17
    NVIC_EN0_R |= 0X20000;                                          // Habilita interrupción general del secuenciador SS3 de ADC0
    NVIC_PRI4_R &=~0x0000E000;
}//TemparatureSensor_Init

float LM35_ReadTemp1(void){
     uint16_t temp = 0x0FFF;
	ADC0_PSSI_R=0x0008;
	while((ADC0_RIS_R & 0x07)!=0x08){}
	temp &= ADC0_SSFIFO3_R;
	ADC0_ISC_R = 0x08;
	return temp;
}//LM35_ReadTemp1

void WaterSensorInit()
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM); // Habilita GPIO M (WaterSensor = PM6)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)); // Aguarda final da habilitação
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_6); // water sensor como estrada (PM6)
  GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}//WaterSensorInit

bool WaterSensorRead()
{
  if(GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_6) == GPIO_PIN_6) // Check push button SW2 state
  {
    return true;
  }
  else{
    return false;
  } 
}//WaterSensorRead

bool isSwitchButton1Pressed()
{
  if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == GPIO_PIN_0) // Check push button SW1 state
  {
    return false;
  }
  else{
    return true;
  }  
}//SwitchButton1Read

bool isSwitchButton2Pressed()
{
  if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == GPIO_PIN_1) // Check push button SW2 state
  {
    return false;
  }
  else{
    return true;
  } 
}//SwitchButton2Read

bool isSwitchButtonKit1Pressed()
{
  if(GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_1) == GPIO_PIN_1) // Check push button SW1 state
  {
    return false;
  }
  else{
    return true;
  }  
}//SwitchButton1Read

bool isSwitchButtonKit2Pressed()
{
  if(GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_2) == GPIO_PIN_2) // Check push button SW2 state
  {
    return false;
  }
  else{
    return true;
  } 
}//SwitchButton2Read

void DisplayInit(){
  cfaf128x128x16Init();
  cfaf128x128x16Clear();
  GrContextInit(&sContext, &g_sCfaf128x128x16);
  GrFlush(&sContext);
  GrContextFontSet(&sContext, g_psFontFixed6x8);
  GrContextForegroundSet(&sContext, ClrWhite);
  GrContextBackgroundSet(&sContext, ClrBlack);  
} // DisplayInit

void ContextInit(){
  GrStringDraw(&sContext, "Sistemas Embarcados", -1, 1, 1, true);
  GrStringDraw(&sContext, "2019/2", -1, 1, 12, true);
} // ContextInit

void SysTick_Handler(void){
  GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, LED_D1); // Acende ou apaga LED D1
  LED_D1 ^= GPIO_PIN_1; // Troca estado do LED D1
} // SysTick_Handler

void ClearDisplay()
{
  cfaf128x128x16Clear();
  GrFlush(&sContext);
}//ClearDisplay

int WaterLevelSensor()
{
  return 2;
}//WaterLevelSensor

void MenuManualMode()
{
  cfaf128x128x16Clear();
  GrFlush(&sContext);
  GrStringDraw(&sContext, "1: Sanitazing", -1, 1, 1, true);
  GrStringDraw(&sContext, "2: Recirculation", -1, 1, 10, true);
  GrStringDraw(&sContext, "3: Absolute Water", -1, 1, 20, true);
  GrStringDraw(&sContext, "Level", -1, 1, 30, true);
  GrStringDraw(&sContext, "4: Relative Water Level", -1, 1, 40, true);
  GrStringDraw(&sContext, "Level", -1, 1, 50, true);
  GrStringDraw(&sContext, "5: Temperature Read", -1, 1, 60, true);
  GrStringDraw(&sContext, "6: Water Output", -1, 1, 70, true);
  GrStringDraw(&sContext, "7: Initial Manual Mode", -1, 1, 80, true);
  GrStringDraw(&sContext, "Mode", -1, 1, 90, true);
  int option=1;
  char greetings[] = "Input ";
  sprintf(greetings, "Input: %d", option);
  GrStringDraw(&sContext, greetings, -1, 1, 100, true); 
    while(1)
    {
      if(WaterLevelSensor() == 1){
        break;
      }
      if(isSwitchButtonKit1Pressed() == true){
        (option > 7) ? option=1 : option++;
        sprintf(greetings, "Input: %d", option);
        GrStringDraw(&sContext, greetings, -1, 1, 100, true);
        SysCtlDelay(5000000);
      }
    }
}
//MenuManualMode

void ClearAndWriteDisplaySingleLine(const char *StringLine1, int xPosition, int yPosition)
{
  ClearDisplay();
  GrStringDraw(&sContext, StringLine1, -1, xPosition, yPosition, true);
}//ClearAndWriteDisplaySingleLine

void ClearAndWriteDisplayTwoLines(const char *StringLine1, int xPosition1, int yPosition1, const char *StringLine2, int xPosition2, int yPosition2)
{
  ClearDisplay();
  GrStringDraw(&sContext, StringLine1, -1, xPosition1, yPosition1, true);
  GrStringDraw(&sContext, StringLine2, -1, xPosition2, yPosition2, true);
}//ClearAndWriteDisplayTwoLines

void ClearAndWriteDisplayThreeLines(const char *StringLine1, int xPosition1, int yPosition1, const char *StringLine2, int xPosition2, int yPosition2, const char *StringLine3, int xPosition3, int yPosition3)
{
  ClearDisplay();
  GrStringDraw(&sContext, StringLine1, -1, xPosition1, yPosition1, true);
  GrStringDraw(&sContext, StringLine2, -1, xPosition2, yPosition2, true);
  GrStringDraw(&sContext, StringLine3, -1, xPosition3, yPosition3, true);
}//ClearAndWriteDisplayThreeLines

void main(void){
  DisplayInit();
  ContextInit();
  
  SysTickPeriodSet(12000000); // f = 1Hz para clock = 24MHz
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // Habilita GPIO N (LED D1 = PN1, LED D2 = PN0)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)); // Aguarda final da habilitação
  
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1); // LEDs D1 e D2 como saída
  GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0); // LEDs D1 e D2 apagados
  GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Habilita GPIO F (LED D3 = PF4, LED D4 = PF0)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Aguarda final da habilitação
    
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4); // LEDs D3 e D4 como saída
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0); // LEDs D3 e D4 apagados
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Habilita GPIO J (push-button SW1 = PJ0, push-button SW2 = PJ1)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)); // Aguarda final da habilitação
    
  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1); // push-buttons SW1 e SW2 como entrada
  GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL); // Habilita GPIO L (push-button SW3 = PL1, push-button SW4 = PL2)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)); // Aguarda final da habilitação
    
  GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, GPIO_PIN_1 | GPIO_PIN_2); // push-buttons SW1 e SW2 como entrada
  GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_1 | GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

  SysTickIntEnable();
  SysTickEnable();
  WaterSensorInit();
  TemparatureSensor_Init();
  
  while(1){
    
    if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == GPIO_PIN_0) // Testa estado do push-button SW1
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); // Apaga LED D3
    else
    {
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); // Acende LED D3
      /*
      cfaf128x128x16Clear();
      GrFlush(&sContext);
      GrStringDraw(&sContext, "1: Sanitazing", -1, 1, 1, true);
      GrStringDraw(&sContext, "2: Recirculation", -1, 1, 10, true);
      GrStringDraw(&sContext, "3: Absolute Water", -1, 1, 20, true);
      GrStringDraw(&sContext, "Level", -1, 1, 30, true);
      GrStringDraw(&sContext, "4: Relative Water Level", -1, 1, 40, true);
      GrStringDraw(&sContext, "Level", -1, 1, 50, true);
      GrStringDraw(&sContext, "5: Temperature Read", -1, 1, 60, true);
      GrStringDraw(&sContext, "6: Water Output", -1, 1, 70, true);
      GrStringDraw(&sContext, "7: Initial Manual Mode", -1, 1, 80, true);
      GrStringDraw(&sContext, "Mode", -1, 1, 90, true);
      int option=1;
      char greetings[] = "Input ";
      sprintf(greetings, "Input: %d", option);
      GrStringDraw(&sContext, greetings, -1, 1, 100, true);
      
      while(1)
      {
        if(WaterLevelSensor() == 1){
          break;
        }
        if(isSwitchButtonKit1Pressed() == true){
          (option > 7) ? option=1 : option++;
          sprintf(greetings, "Input: %d", option);
          GrStringDraw(&sContext, greetings, -1, 1, 100, true);
          SysCtlDelay(5000000);
        }
      }
      
    }
    
    printf("Temp: %.2f",LM35_ReadTemp1());
    
    if(WaterSensorRead() == true)
    {
      cfaf128x128x16Clear();
      GrFlush(&sContext);
      GrStringDraw(&sContext, "Water Detected", -1, 1, 1, true);
      SysCtlDelay(5000000);
    }
    else
    {
      cfaf128x128x16Clear();
      GrFlush(&sContext);
      GrStringDraw(&sContext, "No Water Detected", -1, 1, 1, true);
      SysCtlDelay(5000000);
    }
  } // while
} // main

*/
//*****************************************************************************
//
// temperature_sensor.c - Example demonstrating the internal ADC temperature
//                        sensor.
//
// Copyright (c) 2010-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup adc_examples_list
//! <h1>ADC Temperature Sensor (temperature_sensor)</h1>
//!
//! This example shows how to setup ADC0 to read the internal temperature
//! sensor.
//!
//! NOTE: The internal temperature sensor is not calibrated.  This example
//! just takes the raw temperature sensor sample and converts it using the
//! equation found in the LM3S9B96 datasheet.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - ADC0 peripheral
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of the
//! ADC.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - None.
//
//*****************************************************************************

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Configure ADC0 for the temperature sensor input with a single sample.  Once
// the sample is done, an interrupt flag will be set, and the data will be
// read then displayed on the console via UART0.
//
//*****************************************************************************
int
main(void)
{
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    uint32_t ui32SysClock;
#endif

    //
    // This array is used for storing the data read from the ADC FIFO. It
    // must be as large as the FIFO for the sequencer in use.  This example
    // uses sequence 3 which has a FIFO depth of 1.  If another sequence
    // was used with a deeper FIFO, then the array size must be changed.
    //
    uint32_t pui32ADC0Value[1];

    //
    // These variables are used to store the temperature conversions for
    // Celsius and Fahrenheit.
    //
    uint32_t ui32TempValueC;
    uint32_t ui32TempValueF;

    //
    // Set the clocking to run at 20 MHz (200 MHz / 10) using the PLL.  When
    // using the ADC, you must either use the PLL or supply a 16 MHz clock
    // source.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_480), 20000000);
#else
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
#endif

    //
    // Set up the serial console to use for displaying messages.  This is just
    // for this example program and is not needed for ADC operation.
    //
    InitConsole();

    //
    // Display the setup on the console.
    //
    UARTprintf("ADC ->\n");
    UARTprintf("  Type: Internal Temperature Sensor\n");
    UARTprintf("  Samples: One\n");
    UARTprintf("  Update Rate: 250ms\n");
    UARTprintf("  Input Pin: Internal temperature sensor\n\n");

    //
    // The ADC0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a singal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    //
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample the temperature sensor
    // (ADC_CTL_TS) and configure the interrupt flag (ADC_CTL_IE) to be set
    // when the sample is done.  Tell the ADC logic that this is the last
    // conversion on sequence 3 (ADC_CTL_END).  Sequence 3 has only one
    // programmable step.  Sequence 1 and 2 have 4 steps, and sequence 0 has
    // 8 programmable steps.  Since we are only doing a single conversion using
    // sequence 3 we will only configure step 0.  For more information on the
    // ADC sequences and steps, reference the datasheet.
    //
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCIntClear(ADC0_BASE, 3);

    //
    // Sample the temperature sensor forever.  Display the value on the
    // console.
    //
    while(1)
    {
        //
        // Trigger the ADC conversion.
        //
        ADCProcessorTrigger(ADC0_BASE, 3);

        //
        // Wait for conversion to be completed.
        //
        while(!ADCIntStatus(ADC0_BASE, 3, false))
        {
        }

        //
        // Clear the ADC interrupt flag.
        //
        ADCIntClear(ADC0_BASE, 3);

        //
        // Read ADC Value.
        //
        ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);

        //
        // Use non-calibrated conversion provided in the data sheet.  Make
        // sure you divide last to avoid dropout.
        //
        ui32TempValueC = ((1475 * 1023) - (2250 * pui32ADC0Value[0])) / 10230;

        //
        // Get Fahrenheit value.  Make sure you divide last to avoid dropout.
        //
        ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;

        //
        // Display the temperature value on the console.
        //
        UARTprintf("Temperature = %3d*C or %3d*F\r", ui32TempValueC,
                   ui32TempValueF);

        //
        // This function provides a means of generating a constant length
        // delay.  The function delay (in cycles) = 3 * parameter.  Delay
        // 250ms arbitrarily.
        //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
        SysCtlDelay(ui32SysClock / 12);
#else
        SysCtlDelay(SysCtlClockGet() / 12);
#endif
    }
}
