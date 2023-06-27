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

void init_TIMER(void);
void init_ADC(void);

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


void ADC0SS0IntHandler(void)
{
	HWREG(GPIO_PORTN_BASE + 4) = 1;
//	GPIOPinWrite(GPIO_PORTN_BASE, (USER_LED1|USER_LED2), USER_LED1);
    HWREG(ADC0_BASE + ADC_O_ISC) = 1 << 0;
//    ADCIntClear(ADC0_BASE, 0);
}
//ADC0SS0IntHandler

void init_TIMER()
{
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    // Set sample frequency to 2MHz (every 62.5uS)
    TimerLoadSet(TIMER0_BASE, TIMER_A, 40 - 1);   //TODO: Timer Load Value is set here
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    TimerControlStall(TIMER0_BASE, TIMER_A, true); //Assist in debug by stalling timer at breakpoints
}
//init_TIMER

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
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Habilita GPIO PE4
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)); // Aguarda final da habilitação
  
 
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
