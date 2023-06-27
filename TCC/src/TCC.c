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
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "temp.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "buzzer.h"

#define MAX_TIME 7500
#define ECHO (1U<<5) //PA5(INput)
#define TRIGGER (1U<<4) //PA4(OUTPUT)
#define BLUE_LED (1U<<0)//PF3 onboard Blue LED

uint32_t counter =0;
float distance=0;

uint32_t pui32ADC0Value[1];
uint32_t ui32TempValueC;
uint32_t ui32TempValueF;
float levelBeforeCollection_old;

bool AutomaticMode_Flag = false;
bool ManualMode_Flag = false;

typedef enum {InitialAutomaticMode, RemovingWasteWater, WaterCollection, WaterTreatment, CycleInternalState, Blocked} state_Automatic;
typedef enum {InitialManualMode, SanitazingManualMode, InternalRecircularionManualMode, AbsoluteTankLevelManualMode, RelativeTankLevelManualMode, TemperatureReadManualMode, WaterOutputManualMode, EmptyTank, null} state_Manual;

float area = 0.40;
float height = 26;

uint8_t LED_D1 = 0;
tContext sContext;
float TEMP;
uint32_t timer_treat = 0;

float levelBeforeCollection = 0;
float levelAfterCollection = 0;

float timeRemovingWater = 0;
float timeRemovingWaterActive = 0;

float timeControlLevel = 0;

float manualModeCounter;

float absoluteTemperatureAverage;
float absoluteTemperatureSum;
float absoluteTemperatureValues[10];
float absoluteTemperatureMaximum;
float absoluteTemperatureMinimum;

static void intToString(int64_t value, char * pBuf, uint32_t len, uint32_t base, uint8_t zeros){
	static const char* pAscii = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
	bool n = false;
	int pos = 0, d = 0;
	int64_t tmpValue = value;

	// the buffer must not be null and at least have a length of 2 to handle one
	// digit and null-terminator
	if (pBuf == NULL || len < 2)
			return;

	// a valid base cannot be less than 2 or larger than 36
	// a base value of 2 means binary representation. A value of 1 would mean only zeros
	// a base larger than 36 can only be used if a larger alphabet were used.
	if (base < 2 || base > 36)
			return;

	if (zeros > len)
		return;
	
	// negative value
	if (value < 0)
	{
			tmpValue = -tmpValue;
			value    = -value;
			pBuf[pos++] = '-';
			n = true;
	}

	// calculate the required length of the buffer
	do {
			pos++;
			tmpValue /= base;
	} while(tmpValue > 0);


	if (pos > len)
			// the len parameter is invalid.
			return;

	if(zeros > pos){
		pBuf[zeros] = '\0';
		do{
			pBuf[d++ + (n ? 1 : 0)] = pAscii[0]; 
		}
		while(zeros > d + pos);
	}
	else
		pBuf[pos] = '\0';

	pos += d;
	do {
			pBuf[--pos] = pAscii[value % base];
			value /= base;
	} while(value > 0);
}

static void floatToString(float value, char *pBuf, uint32_t len, uint32_t base, uint8_t zeros, uint8_t precision){
	static const char* pAscii = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
	uint8_t start = 0xFF;
	if(len < 2)
		return;
	
	if (base < 2 || base > 36)
		return;
	
	if(zeros + precision + 1 > len)
		return;
	
	intToString((int64_t) value, pBuf, len, base, zeros);
	while(pBuf[++start] != '\0' && start < len); 

	if(start + precision + 1 > len)
		return;
	
	pBuf[start+precision+1] = '\0';
	
	if(value < 0)
		value = -value;
	pBuf[start++] = '.';
	while(precision-- > 0){
		value -= (uint32_t) value;
		value *= (float) base;
		pBuf[start++] = pAscii[(uint32_t) value];
	}
}

void TemparatureSensor_Init(){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
  GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);
  ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH19 | ADC_CTL_IE |
                             ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 0);
  ADCIntClear(ADC0_BASE, 0);
}//TemparatureSensor_Init

uint32_t readingADCs(){
    ADCProcessorTrigger(ADC0_BASE, 0);
    while(!ADCIntStatus(ADC0_BASE, 0, false))
    {
    }
    ADCIntClear(ADC0_BASE, 0);
    ADCSequenceDataGet(ADC0_BASE, 0, pui32ADC0Value); 
    uint32_t read = pui32ADC0Value[0];
    uint32_t read1 = pui32ADC0Value[1];
    return read;
}

float LM35_ReadTemp1(void){
     uint16_t temp = 0x0FFF;
	ADC0_PSSI_R=0x0008;
	while((ADC0_RIS_R & 0x07)!=0x08){}
	temp &= ADC0_SSFIFO3_R;
	ADC0_ISC_R = 0x08;
	return temp;
}//LM35_ReadTemp1

void RainDetectorSensorInit(){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); // Habilita GPIO K2 (WaterSensor = PK2)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)); // Aguarda final da habilitação
  GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_2); // water sensor como estrada (PM6)
  GPIOPadConfigSet(GPIO_PORTK_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}//RainDetectorSensorInit

bool RainDetectorSensorRead(){
  if(GPIOPinRead(GPIO_PORTK_BASE, GPIO_PIN_2) == GPIO_PIN_2) // Check push button SW2 state
  {
    return true;
  }
  else{
    return false;
  } 
}//RainDetectorSensorRead

void SanitazingLiquidDetectorSensorInit(){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); // Habilita GPIO K2 (WaterSensor = PK2)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)); // Aguarda final da habilitação
  GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_1); // water sensor como estrada (PM6)
  GPIOPadConfigSet(GPIO_PORTK_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}//SanitazingLiquidDetectorSensorInit

bool SanitazingLiquidDetectorSensorRead(){
  if(GPIOPinRead(GPIO_PORTK_BASE, GPIO_PIN_1) == GPIO_PIN_1) // Check push button SW2 state
  {
    return true;
  }
  else{
    return false;
  } 
}//SanitazingLiquidDetectorSensorRead

void PumpLiquidDetectorSensorInit(){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); // Habilita GPIO K2 (WaterSensor = PK2)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)); // Aguarda final da habilitação
  GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_0); // water sensor como estrada (PM6)
  GPIOPadConfigSet(GPIO_PORTK_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}//PumpLiquidDetectorSensorInit

bool PumpLiquidDetectorSensorRead(){
  if(GPIOPinRead(GPIO_PORTK_BASE, GPIO_PIN_0) == GPIO_PIN_0) // Check push button SW2 state
  {
    return true;
  }
  else{
    return false;
  } 
}//PumpLiquidDetectorSensorRead

bool isSwitchButton1Pressed(){
  if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == GPIO_PIN_0) // Check push button SW1 state
  {
    return false;
  }
  else{
    return true;
  }  
}//SwitchButton1Read

bool isSwitchButton2Pressed(){
  if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == GPIO_PIN_1) // Check push button SW2 state
  {
    return false;
  }
  else{
    return true;
  } 
}//SwitchButton2Read

bool isSwitchButtonKit1Pressed(){
  if(GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_1) == GPIO_PIN_1) // Check push button SW1 state
  {
    return false;
  }
  else{
    return true;
  }  
}//SwitchButton1Read

bool isSwitchButtonKit2Pressed(){
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
  //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, LED_D1); // Acende ou apaga LED D1
  //LED_D1 ^= GPIO_PIN_1; // Troca estado do LED D1
} // SysTick_Handler

void ClearDisplay(){
  cfaf128x128x16Clear();
  GrFlush(&sContext);
}//ClearDisplay

int WaterLevelSensor(){
  return 2;
}//WaterLevelSensor

void MenuManualMode(){
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
void ClearAndWriteDisplaySingleLine(const char *StringLine1, int xPosition, int yPosition){
  ClearDisplay();
  GrStringDraw(&sContext, StringLine1, -1, xPosition, yPosition, true);
}//ClearAndWriteDisplaySingleLine
void ClearAndWriteDisplayTwoLines(const char *StringLine1, int xPosition1, int yPosition1, const char *StringLine2, int xPosition2, int yPosition2){
  ClearDisplay();
  GrStringDraw(&sContext, StringLine1, -1, xPosition1, yPosition1, true);
  GrStringDraw(&sContext, StringLine2, -1, xPosition2, yPosition2, true);
}//ClearAndWriteDisplayTwoLines
void ClearAndWriteDisplayThreeLines(const char *StringLine1, int xPosition1, int yPosition1, const char *StringLine2, int xPosition2, int yPosition2, const char *StringLine3, int xPosition3, int yPosition3){
  ClearDisplay();
  GrStringDraw(&sContext, StringLine1, -1, xPosition1, yPosition1, true);
  GrStringDraw(&sContext, StringLine2, -1, xPosition2, yPosition2, true);
  GrStringDraw(&sContext, StringLine3, -1, xPosition3, yPosition3, true);
}//ClearAndWriteDisplayThreeLines

void InitADC() {
  //PIN PD5(AINM=6) will be used to read ADC
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
  SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOD);
  GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_5);
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_5);
  GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_ANALOG);
  SysCtlDelay(80u);
  ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 5);
  SysCtlDelay(10); // Time for the clock configuration to set
  ADCIntDisable(ADC0_BASE, 0u);
  ADCSequenceDisable(ADC0_BASE,0u);
  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0u);
  ADCSequenceStepConfigure(ADC0_BASE,3,0u,ADC_CTL_CH6 | ADC_CTL_END | ADC_CTL_IE);
  ADCSequenceEnable(ADC0_BASE,3); //Once configuration is set, re-enable the sequencer
  ADCIntClear(ADC0_BASE,3);
  ADCIntEnable(ADC0_BASE, 3);
}//InitADC

uint32_t ReadADC() {
  uint32_t temp = 0;
  uint32_t pui32ADC0Value[12];
  ADCProcessorTrigger(ADC0_BASE, 3);
  while(!ADCIntStatus(ADC0_BASE, 3, false))
  {
  }
  ADCIntClear(ADC0_BASE, 3);
  ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
  temp=pui32ADC0Value[0];
  return temp;
}//ReadADC

void IntGlobalEnable(void){
        SysTickIntEnable();
        SysTickEnable();
}

void Interrupt_Init(void){
        
}

void DistanceSensor_Init(void){
    SYSCTL_RCGCGPIO_R |=(1U<<0); //Enable clock for PORTA 
    SYSCTL_RCGCGPIO_R |=(1U<<5); //Enable clock for PORTF 
    GPIO_PORTA_AHB_DIR_R =TRIGGER;
    GPIO_PORTF_AHB_DIR_R =BLUE_LED;
    GPIO_PORTA_AHB_DEN_R |=(ECHO)|(TRIGGER);
    GPIO_PORTF_AHB_DEN_R |= BLUE_LED;
}

void delay_Microsecond(uint32_t time){
    int i;
    SYSCTL_RCGCTIMER_R |=(1U<<1);
    TIMER1_CTL_R=0;
    TIMER1_CFG_R=0x04;
    TIMER1_TAMR_R=0x02;
    TIMER1_TAILR_R= 16-1;
    TIMER1_ICR_R =0x1;
    TIMER1_CTL_R |=0x01;
 
    for(i=0;i<time;i++){ 
        while((TIMER1_RIS_R & 0x1)==0);
        TIMER1_ICR_R = 0x1;
    }
 
}

float measureD(void){
    GPIO_PORTA_AHB_DATA_R &=~TRIGGER;
    //delay_Microsecond(10);
    SysCtlDelay(250); 
    GPIO_PORTA_AHB_DATA_R |= TRIGGER;
    //delay_Microsecond(100);
    SysCtlDelay(25); 
    GPIO_PORTA_AHB_DATA_R &=~TRIGGER;
    counter=0;
    while((GPIO_PORTA_AHB_DATA_R &ECHO)==0)    {}
    while(((GPIO_PORTA_AHB_DATA_R &ECHO )!=0) &(counter < MAX_TIME)) 
 { 
  counter++; 
  //delay_Microsecond(1);
  SysCtlDelay(25); 
  } 
 distance = (float)counter*(float)0.08598;
 return distance; }

float measureLevelSensor(void){
  measureD();
    
    if(measureD() < 10.0)
    { 
      GPIO_PORTF_AHB_DATA_R |=BLUE_LED;
    }
    else
    {
      GPIO_PORTF_AHB_DATA_R &=~BLUE_LED;
    }
    //delay_Microsecond(10);
    return measureD();
}

void initAllActuators(void){
  //Test Relay
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH); // Habilita GPIO H
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH)); // Aguarda final da habilitacao do GPIO H
 
  GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_1 | GPIO_PIN_0); // Pinos PH1 e PH0 como saida
  GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1 | GPIO_PIN_0, 0); // Pinos PH1 e PH0 em nivel 0
  GPIOPadConfigSet(GPIO_PORTH_BASE, GPIO_PIN_1 | GPIO_PIN_0, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM); // Habilita GPIO M
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)); // Aguarda final da habilitacao do GPIO M
 
  GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_2 | GPIO_PIN_1); // Pinos PM2 e PM1 como saida
  GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2 | GPIO_PIN_1, 0); // Pinos PM2 e PM1 em nivel 0
  GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_2 | GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
}

void activateAllActuators(void){
  GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, GPIO_PIN_1); // PH1 ON
  GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, GPIO_PIN_0); // PH0 ON
  GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_PIN_1); // PM1 ON
  GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_PIN_2); // PM2 ON
}

void deactivateAllActuators(void){
  GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, 0); // PH1 OFF
  GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, 0); // PH0 OFF
  GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0); // PM1 OFF
  GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, 0); // PM2 OFF
}

void deactivatePump1(void){
  GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, 0); // PH1 OFF
}

void activatePump1(void){
  GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, GPIO_PIN_1); // PH1 ON
}

void deactivatePump2(void){
  GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, 0); // PH0 OFF
}

void activatePump2(void){
  GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, GPIO_PIN_0); // PH0 ON
}

void deactivatePump3(void){
  GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, 0); // PM2 OFF
}

void activatePump3(void){
  GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_PIN_2); // PM2 ON
}

void deactivatePump4(void){
  GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0); // PM1 ON
}

void activatePump4(void){
  GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_PIN_1); // PM1 ON
}

void deactivateValve(void){
  GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0); // PK6 OFF
}

void activateValve(void){
  GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, GPIO_PIN_7); // PK6 OFF
}

void clearAndPrintMenuAutomatic(char mode[], char state[], int lastRainInterval){
  char printableModeText[50];
  if(((strcmp(state, "Inicial")==0)&&(strcmp(mode, "Automatico")==0))||(strcmp(state, "Bloqueado")==0)||(strcmp(state, "Tratamento")==0)||(strcmp(state, "Saida de agua")==0)||
    (strcmp(state, "Recirculacao da agua")==0)||(strcmp(state, "Leitura absoluta")==0)||(strcmp(state, "Leitura Relativa")==0)||(strcmp(state, "Saida de agua")==0)||(strcmp(state, "Esvaziar")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.3f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    float level1 = measureD();
    float relative1 = (1-(level1/height))*100;
    int absoluteLevel1 = (int)relative1;
    char text2[50];
    sprintf(text2, "Nivel do tanque:%.1f %\r", relative1);
    GrStringDraw(&sContext, text2, -1, 1, 31, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "Nivel Cloro: OK", -1, 1, 41, true);
    
    GrFlush(&sContext);
    char printableLastRainText[50];
    sprintf(printableLastRainText, "Tempo sem chuva: %d", lastRainInterval);  
    GrStringDraw(&sContext, printableLastRainText, -1, 1, 51, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "segundos", -1, 11, 61, true);
    
    GrFlush(&sContext);
    char printableTimeToWaterTreatmentText[50];
    sprintf(printableTimeToWaterTreatmentText, "Tratamento em: %d", 3600-lastRainInterval);  
    GrStringDraw(&sContext, printableTimeToWaterTreatmentText, -1, 1, 71, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "segundos", -1, 11, 81, true);
  }
  if((strcmp(state, "Coleta de Agua")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.3f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    float level1 = measureD();
    float relative1 = (1-(level1/height))*100;
    int absoluteLevel1 = (int)relative1;
    char text2[50];
    sprintf(text2, "Nivel do tanque:%.1f %\r", relative1);
    GrStringDraw(&sContext, text2, -1, 1, 31, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "Nivel Cloro: OK", -1, 1, 41, true);
    
    GrFlush(&sContext);
    char printableLastRainText[50];
    float relative2 = (1-(levelBeforeCollection/height))*100;
    int absoluteLevel2 = (int)relative2;
    sprintf(printableLastRainText, "Nivel inicial:%.1f %\r", relative2);  
    GrStringDraw(&sContext, printableLastRainText, -1, 1, 51, true);
    
    GrFlush(&sContext);
    char printableTimeToWaterTreatmentText[50];
    sprintf(printableTimeToWaterTreatmentText, "Variacao do sistema: %1.f %\r", relative1-relative2);
    GrStringDraw(&sContext, printableLastRainText, -1, 1, 61, true);
  }
  if((strcmp(state, "Remocao do 1mm")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.3f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    float level1 = measureD();
    float relative1 = (1-(level1/height))*100;
    int absoluteLevel1 = (int)relative1;
    char text2[50];
    sprintf(text2, "Nivel do tanque:%.1f %\r", relative1);
    GrStringDraw(&sContext, text2, -1, 1, 31, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "Nivel Cloro: OK", -1, 1, 41, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Tempo de remocao: %.3f", timeRemovingWater);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 51, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Tempo ativo: %.3f", timeRemovingWaterActive);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 61, true);
    
    GrFlush(&sContext);
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Tempo desativo: %.3f", timeRemovingWater-timeRemovingWaterActive);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 71, true);
  }
  if((strcmp(state, "Controle do nivel")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.3f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    float level1 = measureD();
    float relative1 = (1-(level1/height))*100;
    int absoluteLevel1 = (int)relative1;
    char text2[50];
    sprintf(text2, "Nivel do tanque:%.1f %\r", relative1);
    GrStringDraw(&sContext, text2, -1, 1, 31, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "Nivel Cloro: OK", -1, 1, 41, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Tempo em estado: %.3f", timeControlLevel);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 51, true);
  }
  if((strcmp(state, "Tratamento da agua")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.3f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    float level1 = measureD();
    float relative1 = (1-(level1/height))*100;
    int absoluteLevel1 = (int)relative1;
    char text2[50];
    sprintf(text2, "Nivel do tanque:%.1f %\r", relative1);
    GrStringDraw(&sContext, text2, -1, 1, 31, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "Nivel Cloro: OK", -1, 1, 41, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Volume adicionado: 20mL");
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 51, true);
  }
  if((strcmp(state, "Leitura da Temperatura")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.2f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Temp. Media: %.2fC", absoluteTemperatureAverage);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 31, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Temp. Maxima: %.2fC", absoluteTemperatureMaximum);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 41, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Temp. Minima: %.2fC", absoluteTemperatureMinimum);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 51, true);
  }
  
}

void clearAndPrintMenuAutomaticCollection(char mode[], char state[], int lastRainInterval, float initialLevel){
  char printableModeText[50];
  if(((strcmp(state, "Inicial")==0)&&(strcmp(mode, "Automatico")==0))||(strcmp(state, "Bloqueado")==0)||(strcmp(state, "Tratamento")==0)||(strcmp(state, "Saida de agua")==0)||
    (strcmp(state, "Recirculacao da agua")==0)||(strcmp(state, "Leitura absoluta")==0)||(strcmp(state, "Leitura Relativa")==0)||(strcmp(state, "Saida de agua")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.3f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    float level1 = measureD();
    float relative1 = (1-(level1/height))*100;
    int absoluteLevel1 = (int)relative1;
    char text2[50];
    sprintf(text2, "Nivel do tanque:%.1f %\r", relative1);
    GrStringDraw(&sContext, text2, -1, 1, 31, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "Nivel Cloro: OK", -1, 1, 41, true);
    
    GrFlush(&sContext);
    char printableLastRainText[50];
    sprintf(printableLastRainText, "Tempo sem chuva: %d", lastRainInterval);  
    GrStringDraw(&sContext, printableLastRainText, -1, 1, 51, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "segundos", -1, 11, 61, true);
    
    GrFlush(&sContext);
    char printableTimeToWaterTreatmentText[50];
    sprintf(printableTimeToWaterTreatmentText, "Tratamento em: %d", 3600-lastRainInterval);  
    GrStringDraw(&sContext, printableTimeToWaterTreatmentText, -1, 1, 71, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "segundos", -1, 11, 81, true);
  }
  if((strcmp(state, "Coleta de Agua")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.3f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    float level1 = initialLevel;
    float relative1 = (1-(level1/height))*100;
    int absoluteLevel1 = (int)relative1;
    char text2[50];
    sprintf(text2, "Nivel do tanque:%.1f %\r", relative1);
    GrStringDraw(&sContext, text2, -1, 1, 31, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "Nivel Cloro: OK", -1, 1, 41, true);
    
    GrFlush(&sContext);
    char printableLastRainText[50];
    float relative2 = (1-(levelBeforeCollection/height))*100;
    int absoluteLevel2 = (int)relative2;
    sprintf(printableLastRainText, "Nivel inicial:%.1f %\r", relative2);  
    GrStringDraw(&sContext, printableLastRainText, -1, 1, 51, true);
    
    GrFlush(&sContext);
    char printableTimeToWaterTreatmentText[50];
    sprintf(printableTimeToWaterTreatmentText, "Variacao do sistema: %1.f %\r", relative1-relative2);
    GrStringDraw(&sContext, printableLastRainText, -1, 1, 61, true);
  }
  if((strcmp(state, "Remocao do 1mm")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.3f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    float level1 = measureD();
    float relative1 = (1-(level1/height))*100;
    int absoluteLevel1 = (int)relative1;
    char text2[50];
    sprintf(text2, "Nivel do tanque:%.1f %\r", relative1);
    GrStringDraw(&sContext, text2, -1, 1, 31, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "Nivel Cloro: OK", -1, 1, 41, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Tempo de remocao: %.3f", timeRemovingWater);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 51, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Tempo ativo: %.3f", timeRemovingWaterActive);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 61, true);
    
    GrFlush(&sContext);
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Tempo desativo: %.3f", timeRemovingWater-timeRemovingWaterActive);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 71, true);
  }
  if((strcmp(state, "Controle do nivel")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.3f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    float level1 = measureD();
    float relative1 = (1-(level1/height))*100;
    int absoluteLevel1 = (int)relative1;
    char text2[50];
    sprintf(text2, "Nivel do tanque:%.1f %\r", relative1);
    GrStringDraw(&sContext, text2, -1, 1, 31, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "Nivel Cloro: OK", -1, 1, 41, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Tempo em estado: %.3f", timeControlLevel);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 51, true);
  }
  if((strcmp(state, "Tratamento da agua")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.3f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    float level1 = measureD();
    float relative1 = (1-(level1/height))*100;
    int absoluteLevel1 = (int)relative1;
    char text2[50];
    sprintf(text2, "Nivel do tanque:%.1f %\r", relative1);
    GrStringDraw(&sContext, text2, -1, 1, 31, true);
    GrFlush(&sContext);
    GrStringDraw(&sContext, "Nivel Cloro: OK", -1, 1, 41, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Volume adicionado: 20mL");
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 51, true);
  }
  if((strcmp(state, "Leitura da Temperatura")==0))
  {
    sprintf(printableModeText, "Modo: %s", mode);
    GrFlush(&sContext);
    GrStringDraw(&sContext, printableModeText, -1, 1, 1, true);
    
    GrFlush(&sContext);
    char printableStateText[50];
    sprintf(printableStateText, "Estado: %s", state);
    GrStringDraw(&sContext, printableStateText, -1, 1, 11, true);
    
    GrFlush(&sContext);
    char printableTemperatureText[50];
    float temperature12;
    temperature12 = temp_read_celsius();
    sprintf(printableTemperatureText, "Temperatura: %.2f C", temperature12);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 21, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Temp. Media: %.2fC", absoluteTemperatureAverage);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 31, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Temp. Maxima: %.2fC", absoluteTemperatureMaximum);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 41, true);
    
    GrFlush(&sContext);
    sprintf(printableTemperatureText, "Temp. Minima: %.2fC", absoluteTemperatureMinimum);
    GrStringDraw(&sContext, printableTemperatureText, -1, 1, 51, true);
  }
  
}

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
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Habilita GPIO N (LED D1 = PN1, LED D2 = PN0)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Aguarda final da habilitação
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1); // LEDs D1 e D2 como saída
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); // LEDs D1 e D2 apagados
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  
  SysTickIntEnable();
  SysTickEnable();
  RainDetectorSensorInit();
  TemparatureSensor_Init();
  temp_init();
  buzzer_init();
  initAllActuators();
  SanitazingLiquidDetectorSensorInit();
  PumpLiquidDetectorSensorInit();
  
  uint32_t ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_480), 20000000);
  
  uint32_t temp =0;
  DistanceSensor_Init();
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Habilita GPIO F (LED D3 = PF4, LED D4 = PF0)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Aguarda final da habilitação
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4); // LEDs D3 e D4 como saída
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); // LEDs D3 e D4 apagados
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
  uint32_t a = 0;
  
  
  AutomaticMode_Flag = true;
  ManualMode_Flag = false;
  state_Automatic e_state_Automatic = InitialAutomaticMode;
  state_Manual e_state_Manual = null;
  
  int selection = 0;
  int last_rain = 0;
  
  GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
  GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
  
  deactivateAllActuators();
  manualModeCounter = 0;
  while(1){
    if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == true &&
       e_state_Manual==null && ManualMode_Flag == false)
    {
      deactivateAllActuators();
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Automatico", "Inicial", timer_treat);
      SysCtlDelay(8000000); 
      timer_treat++;
      if(RainDetectorSensorRead() == true && SanitazingLiquidDetectorSensorRead() == true)
      {
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
        e_state_Automatic = RemovingWasteWater;
        timer_treat = 0;
      }
      else if (SanitazingLiquidDetectorSensorRead() == false)
      {
        e_state_Automatic = Blocked;
        timer_treat = 0;
      }
      else
      {
        if(timer_treat<3600)
        {
          e_state_Automatic = InitialAutomaticMode;
        }
        if(timer_treat>3600)
        {
          if(RainDetectorSensorRead() == false)
          {
            e_state_Automatic = WaterTreatment;
            timer_treat = 0;
          }
          if(RainDetectorSensorRead() == true)
          {
            timer_treat = 0;
            e_state_Automatic = RemovingWasteWater;
          }
        }
      }
      if(isSwitchButtonKit1Pressed() == true)
      {
        e_state_Manual = InitialManualMode;
        AutomaticMode_Flag = false;
        ManualMode_Flag = true;
      }
    }
    else if(e_state_Automatic == Blocked && AutomaticMode_Flag == true)
    {
      //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); // PF1-Buzzer
      buzzer_write(true);
      buzzer_vol_set(10000);
      buzzer_per_set(10000);
      //activateValve();
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Automatico", "Bloqueado", timer_treat);
      SysCtlDelay(8000000); 
      
      if(PumpLiquidDetectorSensorRead()==true){
        activatePump1();
      }
      else{
        deactivatePump1();
      }
      
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        e_state_Automatic = Blocked;
        timer_treat = 0;
      }
      else{
        deactivateAllActuators();
        e_state_Automatic = InitialAutomaticMode;
        timer_treat++;
        buzzer_write(false);
        buzzer_vol_set(0);
        buzzer_per_set(0);
      }
    }
    else if(e_state_Automatic == RemovingWasteWater && AutomaticMode_Flag == true)
    {
      //activateValve();
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Automatico", "Remocao do 1mm", timer_treat);
      SysCtlDelay(8000000);
      for(int count=0;count<35;count++){
        if(RainDetectorSensorRead() == true){
          clearAndPrintMenuAutomatic("Automatico", "Remocao do 1mm", timer_treat);
          timeRemovingWater++;
          if(PumpLiquidDetectorSensorRead()==true){
            activatePump1();
            timeRemovingWaterActive++;
          }
          else{
            deactivatePump1();
          }
          SysCtlDelay(8000000);
        }
        else{
          e_state_Automatic = InitialAutomaticMode;
          break;
        }
        
      }
      deactivateAllActuators();
      if(RainDetectorSensorRead() == false && SanitazingLiquidDetectorSensorRead() == true)
        {
           e_state_Automatic = InitialAutomaticMode;
           deactivateAllActuators();
           timeRemovingWater = 0;
           timeRemovingWaterActive = 0;
        }
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        e_state_Automatic = Blocked;
        timer_treat = 0;
        timeRemovingWater = 0;
        timeRemovingWaterActive = 0;
      }
      if(RainDetectorSensorRead() == true)
      {
         float level = measureD();
         if (level <= 5)
         {
            e_state_Automatic = CycleInternalState;
            timeRemovingWater = 0;
            timeRemovingWaterActive = 0;
         }
         else
         {
            deactivateAllActuators();
            e_state_Automatic = WaterCollection;
            timeRemovingWater = 0;
            timeRemovingWaterActive = 0;
            levelBeforeCollection_old = measureD();
         }
      }
    }
    else if(e_state_Automatic == WaterCollection && AutomaticMode_Flag == true)
    { 
      deactivateAllActuators();
      cfaf128x128x16Clear();
      //clearAndPrintMenuAutomatic("Automatico", "Coleta de Agua", timer_treat);
      clearAndPrintMenuAutomaticCollection("Automatico", "Coleta de Agua", timer_treat,levelBeforeCollection_old);
      SysCtlDelay(8000000); 
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        e_state_Automatic = Blocked;
        timer_treat = 0;
      }
      if(RainDetectorSensorRead() == true)
      {
         float level = measureD();
         if (level <= 5)
         {
            e_state_Automatic = CycleInternalState;
            levelBeforeCollection = 24;
            levelBeforeCollection_old = 0;
         }
         else
         {
            e_state_Automatic = WaterCollection;
            levelBeforeCollection = measureD();
         }
      }
      else
      {
         e_state_Automatic = WaterTreatment;
         levelBeforeCollection_old = 0;
      }
    }
    else if(e_state_Automatic == CycleInternalState && AutomaticMode_Flag == true)
    {
      timeControlLevel++;
      //activatePump1();
      //activateValve();
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Automatico", "Controle do nivel", timer_treat);
      SysCtlDelay(8000000);
      /*
      activatePump3();
      SysCtlDelay(8000000);
      SysCtlDelay(8000000);
      SysCtlDelay(8000000);
      deactivatePump3();
      */
      while(1)
      {
        if (SanitazingLiquidDetectorSensorRead() == false)
        {
          e_state_Automatic = Blocked;
          timer_treat = 0;
          deactivateAllActuators();
          break;
        }
        if((RainDetectorSensorRead() == true)&&(PumpLiquidDetectorSensorRead()==true))
        {
          activatePump1();
        }
        if((RainDetectorSensorRead() == true)&&(PumpLiquidDetectorSensorRead()==false))
        {
          deactivatePump1();
        }
        if((RainDetectorSensorRead() == false))
        {
          deactivateAllActuators();
          e_state_Automatic = WaterTreatment;
          timeControlLevel = 0;
          break;
        }
        SysCtlDelay(8000000);
      }
      
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        e_state_Automatic = Blocked;
        timer_treat = 0;
      }
      /*
      if(RainDetectorSensorRead() == true)
      {
         float level = measureD();
         if (level <= 5)
         {
            e_state_Automatic = CycleInternalState;
            timeControlLevel = 0;
         }
         else
         {
            e_state_Automatic = WaterCollection;
         }
      }
      */
      else
      {
        deactivateAllActuators();
        e_state_Automatic = WaterTreatment;
        timeControlLevel = 0;
      }
    }
    else if(e_state_Automatic == WaterTreatment && AutomaticMode_Flag == true)
    {
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Automatico", "Tratamento", timer_treat);
      SysCtlDelay(8000000);
      //inject sanitizing liquid
      if(SanitazingLiquidDetectorSensorRead() == true)
      {
        activatePump2();
        SysCtlDelay(8000000);
        SysCtlDelay(8000000);
        deactivatePump2();
      }
      
      
      //inject sanitizing liquid
      activatePump4();
      SysCtlDelay(8000000);
      SysCtlDelay(8000000);
      SysCtlDelay(8000000);
      SysCtlDelay(8000000);
      deactivatePump4();
      
      timer_treat++;
      bool sensor = RainDetectorSensorRead();
      deactivateAllActuators();
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        e_state_Automatic = Blocked;
        timer_treat = 0;
      }
      else if(RainDetectorSensorRead() == true)
      {
          float level = measureD();
         if (level <= 5)
         {
            e_state_Automatic = CycleInternalState;
         }
         else
         {
            e_state_Automatic = WaterCollection;
         }
        
      }
      else{
        e_state_Automatic = InitialAutomaticMode;
      } 
    }
    else if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false && ManualMode_Flag == true && e_state_Manual == InitialManualMode)
    {
      manualModeCounter++;
      if(manualModeCounter> 30.0)
      {
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = InitialAutomaticMode;
        e_state_Manual = null;
        deactivateAllActuators();
        manualModeCounter = 0;
      }
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        timer_treat = 0;
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = Blocked;
        e_state_Manual = null;
      }
      if(RainDetectorSensorRead() == true && SanitazingLiquidDetectorSensorRead() == true)
      {
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = InitialAutomaticMode;
        e_state_Manual = null;
        deactivateAllActuators();
      }
      if(manualModeCounter<30.0){
        cfaf128x128x16Clear();
        GrFlush(&sContext);
        GrStringDraw(&sContext, "Modo: InitialManual", -1, 1, 1, true);
        GrStringDraw(&sContext, "1: Tratamento", -1, 1, 10, true);
        GrStringDraw(&sContext, "2: Recirculacao", -1, 1, 20, true);
        GrStringDraw(&sContext, "3: Nivel", -1, 1, 30, true);
        GrStringDraw(&sContext, "Absoluto", -1, 1, 40, true);
        GrStringDraw(&sContext, "4: Nivel", -1, 1, 50, true);
        GrStringDraw(&sContext, "Relativo", -1, 1, 60, true);
        GrStringDraw(&sContext, "5: Temperatura", -1, 1, 70, true);
        GrStringDraw(&sContext, "6: Saida de A.", -1, 1, 80, true);
        GrStringDraw(&sContext, "7: Esvaziar", -1, 1, 90, true);
        SysCtlDelay(4000000);
        if(isSwitchButtonKit2Pressed()==true){
          SysCtlDelay(400000);
          bool status = isSwitchButtonKit2Pressed();
          if(status = true){
            selection > 8 ? selection=1 : selection++;
          }
        }
        
        cfaf128x128x16Clear();
        GrFlush(&sContext);
        char greetings2[] = "";
        sprintf(greetings2, "Selecao = %d\r", selection);
        GrStringDraw(&sContext, greetings2, -1, 1, 1, true);
        char greetings3[] = "";
        sprintf(greetings3, "Selection = %d\r", selection);
          if(selection == 1){
            GrStringDraw(&sContext, "Tratamento", -1, 1, 10, true);
          }
          else if(selection == 2){
            GrStringDraw(&sContext, "Recirculacao", -1, 1, 10, true);
          }
          else if(selection == 3){
            GrStringDraw(&sContext, "Nivel Abs.", -1, 1, 10, true);
          }
          else if(selection == 4){
            GrStringDraw(&sContext, "Nivel Rel.", -1, 1, 10, true);
          }
          else if(selection == 5){
            GrStringDraw(&sContext, "Temperatura", -1, 1, 10, true);
          }
          else if(selection == 6){
            GrStringDraw(&sContext, "Saida de A.", -1, 1, 10, true);
          }
          else if(selection == 7){
            GrStringDraw(&sContext, "Esvaziar", -1, 1, 10, true);
          }
        
        SysCtlDelay(4000000);
        
        if(isSwitchButtonKit1Pressed() == true){
          if(selection == 1){
            e_state_Manual = SanitazingManualMode;
          }
          else if(selection == 2){
            e_state_Manual = InternalRecircularionManualMode;
          }
          else if(selection == 3){
            e_state_Manual = AbsoluteTankLevelManualMode;
          }
          else if(selection == 4){
            e_state_Manual = RelativeTankLevelManualMode;
          }
          else if(selection == 5){
            e_state_Manual = TemperatureReadManualMode;
          }
          else if(selection == 6){
            e_state_Manual = WaterOutputManualMode;
          }
          else if(selection == 7){
            e_state_Manual = EmptyTank;
          }
        }
      }
      
      
      
      //else{
      //   e_state_Manual = null;
      //}
    }
    
    else if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false &&
            ManualMode_Flag == true && e_state_Manual == SanitazingManualMode)
    {
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Manual", "Tratamento da agua", timer_treat);
      SysCtlDelay(8000000);
      if(SanitazingLiquidDetectorSensorRead() == true)
      {
        activatePump2();
        SysCtlDelay(8000000);
      }
        
      if(RainDetectorSensorRead() == true && SanitazingLiquidDetectorSensorRead() == true)
      {
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = InitialAutomaticMode;
        e_state_Manual = null;
        deactivateAllActuators();
      }
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        timer_treat = 0;
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = Blocked;
        e_state_Manual = null;
      }
      SysCtlDelay(8000000);
      SysCtlDelay(8000000);
      deactivatePump2();
      activatePump4();
      SysCtlDelay(8000000);
      SysCtlDelay(8000000);
      SysCtlDelay(8000000);
      deactivatePump4();
      e_state_Manual = null;
      AutomaticMode_Flag = true;
      ManualMode_Flag = false;
      e_state_Automatic = InitialAutomaticMode;
      selection = 0;
    }
    else if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false &&
            ManualMode_Flag == true && e_state_Manual == InternalRecircularionManualMode)
    {
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Manual", "Recirculacao da agua", timer_treat);
      SysCtlDelay(8000000);
      if(RainDetectorSensorRead() == true && SanitazingLiquidDetectorSensorRead() == true)
      {
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = InitialAutomaticMode;
        e_state_Manual = null;
        deactivateAllActuators();
      }
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        timer_treat = 0;
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = Blocked;
        e_state_Manual = null;
      }
      
      float level4 = measureD();
      if(level4 < 22 && (RainDetectorSensorRead() == false))
      {
        deactivateAllActuators();
        activatePump4();
        SysCtlDelay(8000000);
        SysCtlDelay(8000000);
        SysCtlDelay(8000000);
        SysCtlDelay(8000000);
        deactivatePump4();
        deactivateAllActuators();
      }

      
      e_state_Manual = null;
      AutomaticMode_Flag = true;
      ManualMode_Flag = false;
      e_state_Automatic = InitialAutomaticMode;
      selection = 0;
    }
    else if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false &&
            ManualMode_Flag == true && e_state_Manual == AbsoluteTankLevelManualMode)
    {
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Manual", "Leitura absoluta", timer_treat);
      SysCtlDelay(8000000);
      SysCtlDelay(8000000);
      if(RainDetectorSensorRead() == true && SanitazingLiquidDetectorSensorRead() == true)
      {
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = InitialAutomaticMode;
        e_state_Manual = null;
        deactivateAllActuators();
      }
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        timer_treat = 0;
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = Blocked;
        e_state_Manual = null;
      }
      e_state_Manual = null;
      AutomaticMode_Flag = true;
      ManualMode_Flag = false;
      e_state_Automatic = InitialAutomaticMode;
      selection = 0;
    }
    else if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false &&
            ManualMode_Flag == true && e_state_Manual == RelativeTankLevelManualMode)
    {
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Manual", "Leitura Relativa", timer_treat);
      SysCtlDelay(8000000);
      if(RainDetectorSensorRead() == true && SanitazingLiquidDetectorSensorRead() == true)
      {
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = InitialAutomaticMode;
        e_state_Manual = null;
        deactivateAllActuators();
      }
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        timer_treat = 0;
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = Blocked;
        e_state_Manual = null;
      }
      e_state_Manual = null;
      AutomaticMode_Flag = true;
      ManualMode_Flag = false;
      e_state_Automatic = InitialAutomaticMode;
      selection = 0;
    }
    else if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false &&
            ManualMode_Flag == true && e_state_Manual == TemperatureReadManualMode)
    {
      cfaf128x128x16Clear();
      deactivateAllActuators();
      if(RainDetectorSensorRead() == true && SanitazingLiquidDetectorSensorRead() == true)
      {
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = InitialAutomaticMode;
        e_state_Manual = null;
        deactivateAllActuators();
      }
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        timer_treat = 0;
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = Blocked;
        e_state_Manual = null;
      }
      SysCtlDelay(8000000);
      absoluteTemperatureSum = 0.0;
      absoluteTemperatureAverage = 0.0;
      float temperature13;
      temperature13 = temp_read_celsius();
      absoluteTemperatureMaximum = temperature13;
      absoluteTemperatureMinimum = temperature13;
      for (int h=0;h<20;h++)
      {
        float temperature12;
        temperature12 = temp_read_celsius();
        absoluteTemperatureValues[h]=temperature12;
        absoluteTemperatureSum += temperature12;
        if(absoluteTemperatureValues[h] >= absoluteTemperatureMaximum)
        {
          absoluteTemperatureMaximum = absoluteTemperatureValues[h];
        }
        if(absoluteTemperatureValues[h] <= absoluteTemperatureMinimum)
        {
          absoluteTemperatureMinimum = absoluteTemperatureValues[h];
        }
      }
      absoluteTemperatureAverage = absoluteTemperatureSum/10.0;
      clearAndPrintMenuAutomatic("Manual", "Leitura da Temperatura", timer_treat);
      e_state_Manual = null;
      AutomaticMode_Flag = true;
      ManualMode_Flag = false;
      e_state_Automatic = InitialAutomaticMode;
      selection = 0;
      timer_treat = 0;
    }
    else if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false &&
            ManualMode_Flag == true && e_state_Manual == WaterOutputManualMode)
    {
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Manual", "Saida de agua", timer_treat);
      SysCtlDelay(8000000);
      if(RainDetectorSensorRead() == true && SanitazingLiquidDetectorSensorRead() == true)
      {
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        state_Automatic e_state_Automatic = InitialAutomaticMode;
        state_Manual e_state_Manual = null;
        deactivateAllActuators();
      }
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        timer_treat = 0;
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = Blocked;
        e_state_Manual = null;
      }
      for (int j=0;j<11;j++){
        float level = measureD();
        if(RainDetectorSensorRead() == true && SanitazingLiquidDetectorSensorRead() == true)
        {
          AutomaticMode_Flag = true;
          ManualMode_Flag = false;
          state_Automatic e_state_Automatic = InitialAutomaticMode;
          state_Manual e_state_Manual = null;
          deactivateAllActuators();
        }
        if((level >= 5)&&(level <=20))
        {
          activatePump3();
        }
        else{
          deactivatePump3();
        }
        SysCtlDelay(8000000);
      }
      deactivateAllActuators();
      e_state_Manual = null;
      AutomaticMode_Flag = true;
      ManualMode_Flag = false;
      e_state_Automatic = InitialAutomaticMode;
      selection = 0;
    }
    else if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false &&
            ManualMode_Flag == true && e_state_Manual == EmptyTank)
    {
      cfaf128x128x16Clear();
      clearAndPrintMenuAutomatic("Manual", "Esvaziar", timer_treat);
      SysCtlDelay(8000000);
      if(RainDetectorSensorRead() == true && SanitazingLiquidDetectorSensorRead() == true)
      {
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        state_Automatic e_state_Automatic = InitialAutomaticMode;
        state_Manual e_state_Manual = null;
        deactivateAllActuators();
      }
      if (SanitazingLiquidDetectorSensorRead() == false)
      {
        timer_treat = 0;
        AutomaticMode_Flag = true;
        ManualMode_Flag = false;
        e_state_Automatic = Blocked;
        e_state_Manual = null;
      }
      float level3 = measureD();
      while (1)
      {
        level3 = measureD();
        clearAndPrintMenuAutomatic("Manual", "Esvaziar", timer_treat);
        if(level3 < 22 && (RainDetectorSensorRead() == false))
        {
          activatePump3();
        }
        else
        {
          break;
        }
        SysCtlDelay(2000000);
      }
      deactivateAllActuators();
      e_state_Manual = null;
      AutomaticMode_Flag = true;
      ManualMode_Flag = false;
      e_state_Automatic = InitialAutomaticMode;
      selection = 0;
    }
    
    //Debugging Table (Values will be displayed as binary
    // LED D1,D2,D3,D4
    // 0 - 0000 - Initialization Mode
    // 1 - 1000 - State InitialAutomaticMode
    // 2 - 0100 - State RemovingWasteWater
    // 3 - 1100 - State WaterCollection
    // 4 - 0010 - State WaterTreatment
    // 5 - 1010 - State CycleInternalState
    // 6 - 0110 - State InitialManualMode
    // 7 - 1110 - State SanitazingManualMode
    // 8 - 0001 - State InternalRecircularionManualMode
    // 9 - 1001 - State AbsoluteTankLevelManualMode
    // 10 - 0101 - State RelativeTankLevelManualMode
    // 11 - 1101 - State TemperatureReadManualMode
    // 12 - 0011 - State WaterOutputManualMode
    
    if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == true)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
    }
    if(e_state_Automatic == RemovingWasteWater && AutomaticMode_Flag == true)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
    }
    if(e_state_Automatic == WaterCollection && AutomaticMode_Flag == true)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
    }
    if(e_state_Automatic == WaterTreatment && AutomaticMode_Flag == true)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
    }
    if(e_state_Automatic == CycleInternalState && AutomaticMode_Flag == true)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); 
    }
    if(e_state_Automatic == Blocked && AutomaticMode_Flag == true)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); 
    }
    if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false && ManualMode_Flag == true && e_state_Manual == InitialManualMode)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); 
    }
    if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false && ManualMode_Flag == true && e_state_Manual == SanitazingManualMode)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);  
    }
    if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false && ManualMode_Flag == true && e_state_Manual == InternalRecircularionManualMode)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); 
    }
    if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false && ManualMode_Flag == true && e_state_Manual == AbsoluteTankLevelManualMode)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); 
    }
    if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false && ManualMode_Flag == true && e_state_Manual == RelativeTankLevelManualMode)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); 
    }
    if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false && ManualMode_Flag == true && e_state_Manual == TemperatureReadManualMode)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); 
    }
    if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false && ManualMode_Flag == true && e_state_Manual == WaterOutputManualMode)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); 
    }
    if(e_state_Automatic == InitialAutomaticMode && AutomaticMode_Flag == false && ManualMode_Flag == true && e_state_Manual == EmptyTank)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); 
    }
  } // while
} // main
