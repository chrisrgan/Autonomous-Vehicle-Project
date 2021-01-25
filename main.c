//*****************************************************************************
                              /****** EEC 195B NATCAR ******/
// Luke Alcantara
// Quentin Zhong
// Christopher Gan
// Code for the camera and servo together and motor and bluetooth

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "driverlib/adc.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"

volatile uint32_t pui32ADC0Value[129];  //circular buffer
volatile uint32_t pingBuffer[129];
volatile uint32_t pongBuffer[129];
volatile uint32_t whiteblack[129];
volatile uint32_t threshold=180;
volatile int32_t slope[129];
volatile uint32_t pingFull = 0;
volatile uint32_t pongFull = 0;
volatile uint32_t clk_counter;
volatile int32_t maxIndex = 0;
volatile int32_t minIndex = 0;
volatile int32_t meanIndex = 0;
volatile int32_t cameraIndex = 0;

volatile uint32_t sum = 0;
volatile int32_t mean = 0;
volatile int32_t max = 0;
volatile int32_t min = 3000;
volatile uint32_t Vpp = 0;
volatile uint32_t count = 0;
volatile uint32_t toggle = 0; //check if last buffer written to was ping or pong
char letter;
char letter2;
volatile uint32_t count;
volatile uint32_t run = 1;
volatile uint32_t timer_rate = 500;
char input;
volatile uint32_t ServoStall = 0;
volatile int32_t error = 0;
volatile int32_t lastError = 0;
volatile uint32_t T;
volatile uint32_t kp;
volatile uint32_t kd;
volatile int32_t PValue;
volatile int32_t DValue;
volatile int32_t CValue;
volatile int32_t Cdiff;
volatile int32_t CValueLast;
volatile int32_t servoPos;
volatile int32_t inc = 0;
volatile int32_t errorBuffer[256];
volatile int32_t DValueBuffer[256];
volatile uint32_t eIndex = 0;
volatile uint32_t setSpeed=0;

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
void
Timer0IntHandler(void) //timer interrupt handler
{
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Clear the timer interrupt.

	//UARTprintf("\nTEST1\n");
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2); // assert SI signal high
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // assert test signal high
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // assert CLK signal high
    clk_counter = 1;
    //<<========  UPDATE BUFFER POINTER

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0); // deassert SI signal LOW
    ADCProcessorTrigger(ADC0_BASE, 3); // Start an ADC conversion using software triggering
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0); // deassert CLK signal LOW

    //ROM_IntMasterDisable();
    //UARTprintf("\rTimerTest\n");
    //ROM_IntMasterEnable();

}

//*****************************************************************************
void
adcHandler(void) //adc interrupt handler
{

	ADCIntClear(ADC0_BASE, 3); //clear interrupt request

	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // assert CLK signal HIGH

	ADCSequenceDataGet(ADC0_BASE, 3, &pui32ADC0Value[clk_counter]); //address of element

	if (toggle == 0)// Store in Ping or Pong buffer if empty-Ping has priority
	{
		pingBuffer[clk_counter] = (pui32ADC0Value[clk_counter]);
	}
	else
	{
		pongBuffer[clk_counter] = (pui32ADC0Value[clk_counter]);
	}

	clk_counter = clk_counter + 1; //increment counter


	if (clk_counter < 129)// Check if not full
	{
		ADCProcessorTrigger(ADC0_BASE, 3); // Start an ADC conversion using software triggering
	}
	else
	{
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0); // deassert test signal LOW
		if (toggle == 0){
			pingFull = 1; //switch pingfull and pongfull flags
			pongFull = 0;
			clk_counter = 0; //restart clock counter
			toggle = 1;

		}
		else {
			pongFull = 1; //switch pingfull and pongfull flags
			pingFull = 0;
			clk_counter = 0; //restart clock counter
			toggle = 0;
		}
	}

	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0); // deassert CLK signal LOW
}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
   // ui32Status = ROM_UARTIntStatus(UART0_BASE, true);
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    //ROM_UARTIntClear(UART0_BASE, ui32Status);
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //

    //while(ROM_UARTCharsAvail(UART0_BASE))
    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //


       // ROM_UARTCharPutNonBlocking(UART0_BASE,
         //                          ROM_UARTCharGetNonBlocking(UART0_BASE));

    	//char input = ROM_UARTCharGet(UART0_BASE);
    	input = ROM_UARTCharGet(UART1_BASE);

    	//ROM_UARTCharPutNonBlocking(UART0_BASE,input);

		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // assert ENA signal high Accelerate
		//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, ); // CS
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); // ENB high
		//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // INA high (OLD)****
		//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // INA high
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0); // INA low
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // INB low  //brake

//		if(input == 'a')
//			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // INB high
//		if(input == 's')
//			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // INB low


        if(input == '0'){//STOP
        	 //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // INB low
        	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // INA high
			 //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); 					  //0 Motor stopped
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,4540);//neutral
			 ServoStall = 1;

        }
        if(input == '1'){//START													  //NOT DUTY cycle, this is a hard brake
        	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // INB high
        	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0); // INA low
//        	 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
//        	                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 1 / 10); //10%
        	 ServoStall = 0;
        }
        if(input == '2'){
        	 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
        	                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 3 / 20);  //20%
        	 ServoStall = 0;}
		if(input == '3'){
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 4 / 20);  //30%
			 ServoStall = 0;}
		if(input == '4'){
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 5 / 20); //25%
			 ServoStall = 0;}
		if(input == '5'){
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 6 / 20); //30%
			 ServoStall = 0;}
		if(input == '6'){
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 7 / 20); //35%
			 ServoStall = 0;}
		if(input == '7'){
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 8 / 20); //70%
			 ServoStall = 0;}
		if(input == '7'){
					 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
					                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 8 / 20); //70%
					 ServoStall = 0;}
//		if(input == '7')
//			{
//				maxSpeed = 400
//			}
//		if(input == '8'){
//			if (inc > 0)
//			{
//				inc--;
//			}
//			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
//			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * inc / 20); //80%
//			 ServoStall = 0;}
//
//		if(input == '9'){
//
//			if (inc < 18)
//			{
//				inc++;
//			}
//
//			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
//			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * inc / 20);  //90%
//			 ServoStall = 0;}

        //
        // Blink the LED to show a character transfer is occuring.
        //

        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2) == 0)
        	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        else
        	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        //SysCtlDelay(SysCtlClockGet() / 5 * 3);

        //
        // Turn off the LED
        //
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(*pui8Buffer != '\0')
    {
        //
        // Write the next character to the UART.
        //
        //ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
        ROM_UARTCharPutNonBlocking(UART1_BASE, *pui8Buffer++);
    }
}

void
vt(void)
{
	int j = 1;

	if (pingFull)
	{
		for (j=1; j < 129; j++)
		{
			if (pingBuffer[j]<=threshold) // check if less than theshold, or greater than, and write a 1 or 0
			{
				whiteblack[j]=0;
			}
			else if (pingBuffer[j]>threshold)
			{
				whiteblack[j]=1;
			}
		}

		for (j =1; j < 129; j++)
		{
			UARTprintf("%d",whiteblack[j]); // simply prints out if 1 or 0
		}
		UARTprintf("\n");
	}

	else if (pongFull)
	{
		for (j=1; j < 129; j++)
		{
			if (pongBuffer[j]<=threshold) // check if less than theshold, or greater than, and write a 1 or 0
			{
				whiteblack[j]=0;
			}
			else if (pongBuffer[j]>threshold)
			{
				whiteblack[j]=1;
			}
		}
	for (j =1; j<129; j++)
		{
			UARTprintf("%d",whiteblack[j]); // simply prints out if 1 or 0
		}
		UARTprintf("\n");
	}
}

//*****************************************************************************

void
st(void)
{
	int i = 1;
	max = 0;
	min = 3000;
	maxIndex = 0;
	minIndex = 0;

	if (pingFull)
	{
		for (i=1; i < 129; i++)
		{
			if (i<3 || i > 127)
			{
				slope[i] = 0;
			}
			else
			{

				if (pingBuffer[i-1] > pingBuffer[i+1]) // get value before and after
				{
					slope[i] = ((pingBuffer[i-1]-pingBuffer[i+1]) >> 1)*(-1); //slope equation if a negative #
				}
				else
					{
					slope[i] = (pingBuffer[i+1]-pingBuffer[i-1]) >> 1;
					}
			}

			if (slope[i]>max)
			{
				max = slope[i]; //continuously re-update the max
				maxIndex = i;
			}
			if (slope[i]<min)
			{
				min = slope[i]; //continuously re-update the min
				minIndex = i;
			}
		}
	}
	else if (pongFull)
	{
		for (i=1; i < 129; i++)
		{
			if (i<3 || i > 127)
			{
				slope[i] = 0;
			}

			else
			{
				if (pongBuffer[i-1] > pongBuffer[i+1])
				{
					slope[i] = ((pongBuffer[i-1]-pongBuffer[i+1]) >> 1)*(-1);
				}
				else
				{
					slope[i] = (pongBuffer[i+1]-pongBuffer[i-1]) >> 1;
				}
			}

			if (slope[i]>max)
			{
				max = slope[i]; //continuously re-update the max
				maxIndex = i;

			}
			if (slope[i]<min)
			{
				min = slope[i]; //continuously re-update the min
				minIndex = i;
			}

			//UARTprintf("%i ", slope[i]);
		}
	}

	meanIndex = (minIndex + maxIndex) >> 1;

	for(i = 1; i < 128; i++)
	{
		if (i < minIndex && i > maxIndex)
		{
			//UARTStdioInit(0);
//			UARTprintf("1");
		}
		else
		{
			//UARTStdioInit(0);
//			UARTprintf("0");
		}
	}
	//UARTStdioInit(0);
//	UARTprintf("\n");
	/*if((meanIndex >= 0) && (meanIndex < 15))
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) >> 2); //FULLY CCW
	if(meanIndex >= 15 && meanIndex < 29)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) * 10 / 36); //3/4 FULLY CCW
	if(meanIndex >= 29 && meanIndex < 43)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)* 10 / 31); //1/2 FULLY CCW
	if(meanIndex >= 43 && meanIndex < 57)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)* 5 / 14); //1/4 FULLY CCW
	if(meanIndex >= 57 && meanIndex < 71)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)* 3 >> 3); //NEUTRAL 13
	if(meanIndex >= 71 && meanIndex < 85)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) << 2 / 10); //1/4 FULLY CW 12
	if(meanIndex >= 85 && meanIndex < 99)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)* 10 / 23); //1/2 FULLY CW 11
	if(meanIndex >= 99 && meanIndex < 113)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)* 10 / 21); //3/4 FULLY CW
	if(meanIndex >= 113 && meanIndex < 128)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) >> 1); //FULLY CW*/
//
//	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
//								 (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / (4*(1/((meanIndex + 128) >> 7)))));
	cameraIndex = meanIndex;
	if (meanIndex > 100){  // Prevent strain on the servo from overturning
		meanIndex = 100;}
	else if (meanIndex < 20){
		meanIndex = 20;}

	lastError = error;
	error = cameraIndex-66; 		//Error

	if (((error-lastError)<3) && (error-lastError)>-3)
	{
		error = lastError;
	}
	kp = 40;						//=== Proportional Control ===//
	PValue = (kp*error) + 4540; //error will be positive or negative and will turn accordingly


	int N = 256;					//=== Differential Control ===//
	int numD = 4;


	if (eIndex < 256)
	{
		errorBuffer[eIndex] = error;
		eIndex++;
	}
	if (eIndex >= 256) // if overflow
	{
		eIndex = 0;
	}
	kd = 1;
	T = 500; //1 ms
	if (eIndex < (numD-1)) //Grabbing 4 values, so val less than 3 needs overflow, but when index=3, then all ok.
	{
		DValue = (kd*T/(6))*(errorBuffer[eIndex] - errorBuffer[N-3] + (3*(errorBuffer[N-1])) - (3*(errorBuffer[N-2])));

		//DValue = DValue + ;
		DValue = (-1)*DValue + 4540;
	}
	else //No overflow here
	{
		DValue = (kd*T/(6))*(errorBuffer[eIndex] - errorBuffer[eIndex-3] + (3*(errorBuffer[eIndex-1])) - (3*(errorBuffer[eIndex-2])));
		//DValue = DValue + 4540;
		DValue = (-1)*DValue + 4540;
	}

	DValueBuffer[eIndex] = DValue; //debugging puposes
	servoPos = PWMPulseWidthGet(PWM0_BASE, PWM_OUT_6);

	CValueLast = CValue;
	CValue = (PValue/2) + (DValue/2);
	Cdiff = abs((4540 - CValue)/2);
	if(Cdiff > 400) {
		Cdiff = 400;
	}

	if (CValue <= 3442)
		CValue = 3442;
	if (CValue >= 5370)
		CValue = 5370;


	int maxSpeed = 350; //500 was a little too fast for the sharp turn right after the straight away
	int minSpeed = 200;

	setSpeed = maxSpeed - Cdiff;
	if (setSpeed <= minSpeed)
		setSpeed = minSpeed;
	if (setSpeed >= maxSpeed)
		setSpeed = maxSpeed;
	if(ServoStall == 0){
//	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
//									 ((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) * (meanIndex + 120)) >> 9));
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,CValue);   // P+D
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * setSpeed/1000); //10%
	}
	else
	{
		//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) *3 >> 3);//neutral
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,4540);//neutral
	}
}
//*****************************************************************************
/*void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}*/


//*****************************************************************************
int
main(void)
{

    ROM_FPULazyStackingEnable();

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Initialize the UART and write status.
    //
//    ConfigureUART();


	//UARTprintf("\nLuke Alcantara\nQuentin Zhong\nChristopher Gan\n");

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //motor
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2); // ENA
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3); // INA
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); // ENB
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5); // INB
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); // LED



    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_1);
	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // Enable GPIO Port E/F (and ADC port e pin3)
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);// Enable the ADC
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCIntRegister(ADC0_BASE,3,adcHandler);
	ADCIntEnable(ADC0_BASE,3);
	ADCIntClear(ADC0_BASE, 3);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//PWM peripheral enable
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //enable the timer
    ROM_IntMasterEnable();

    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()/timer_rate);

    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    ROM_TimerEnable(TIMER0_BASE, TIMER_A);

     SysCtlPWMClockSet(SYSCTL_PWMDIV_16);

     SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
     //SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //motor

     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//motor

     GPIOPinConfigure(GPIO_PC4_M0PWM6);
     GPIOPinConfigure(GPIO_PB7_M0PWM1);//motor

     GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
     GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//motor

     PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |PWM_GEN_MODE_NO_SYNC);
     PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |PWM_GEN_MODE_NO_SYNC);//motor

     PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 12500);//250MHz servo
     PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 3125); //62500 1kHz//motor

     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) *3 >> 3);//neutral
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // assert ENA signal high Accelerate
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); // ENB high
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0); // INA low
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // INB high //go
     //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 3 / 10);  //30%

     PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
     PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);//motor

     PWMGenEnable(PWM0_BASE, PWM_GEN_3);
     PWMGenEnable(PWM0_BASE, PWM_GEN_0);//motor

     ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); //bluetooth
     //GPIOPinConfigure(GPIO_PA0_U0RX);
     //GPIOPinConfigure(GPIO_PA1_U0TX);
     GPIOPinConfigure(GPIO_PB0_U1RX); //bluetooth
     GPIOPinConfigure(GPIO_PB1_U1TX); //bluetooth

     //ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
     ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1); //bt

     //ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
      //                           (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
      //                            UART_CONFIG_PAR_NONE));
     ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 115200,
                                     (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                      UART_CONFIG_PAR_NONE));
     //ROM_IntEnable(INT_UART0);
     ROM_IntEnable(INT_UART1);
     //ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
     ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT); //bt
     //UARTSend((uint8_t *)"\033[2JEnter text: ", 16);

    while(run==1)
    {
    	if (pingFull | pongFull)
    	{
    		st();
    	}

    }
}
