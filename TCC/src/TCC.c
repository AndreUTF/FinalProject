#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "grlib/grlib.h"
#include "cfaf128x128x16.h" // Projects/drivers

typedef enum {InitialAutomaticMode, AwaitingRain, RemovingWasteWater, WaterCollect, WaterRemoval} state_Automatic;
typedef enum {InitialManualMode, Sanitazing, InternalRecircularion, AbsoluteTankLevel, RelativeTankLevel, TemperatureRead, WaterOutput} state_Manual;

uint8_t LED_D1 = 0;
tContext sContext;

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
  int option=1;
  char greetings[] = "Input ";
  char option_str[] = itoa(option);
  strcat(greetings,option_str);
  ClearDisplay();
  if(WaterLevelSensor() == 1){
    //return -1;
  }
  else{
    GrStringDraw(&sContext, "1: Sanitazing", -1, 1, 1, true);
    GrStringDraw(&sContext, "2: Recirculation", -1, 1, 10, true);
    GrStringDraw(&sContext, "3: Absolute Level Read", -1, 1, 20, true);
    GrStringDraw(&sContext, "4: Relative Level Level", -1, 1, 30, true);
    GrStringDraw(&sContext, "5: Temperature Read", -1, 1, 40, true);
    GrStringDraw(&sContext, "6: Water Output", -1, 1, 50, true);
    GrStringDraw(&sContext, "7: Initial Manual Mode", -1, 1, 60, true);
    GrStringDraw(&sContext, greetings , -1, 1, 70, true);
    while(1)
    {
      if(WaterLevelSensor() == 1)
        break;
      
    }
  }
}//MenuManualMode

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
  
  while(1){
    
    if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == GPIO_PIN_0) // Testa estado do push-button SW1
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); // Apaga LED D3
    else
    {
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); // Acende LED D3
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
      
  /*
    if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == GPIO_PIN_1) // Testa estado do push-button SW2
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); // Apaga LED D4
    else
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); // Acende LED D4
    
    if(isSwitchButton2Pressed() == false)
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); // Apaga LED D4
    else
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); // Acende LED D4
    
    if(isSwitchButton1Pressed() == false)
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); // Apaga LED D3
    else
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); // Acende LED D3
    */  
  } // while
} // main