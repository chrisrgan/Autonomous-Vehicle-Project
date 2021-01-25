//*****************************************************************************
                              /****** EEC 195B NATCAR ******/
// Luke Alcantara
// Christopher Gan
// Quentin Zhong

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

// Initialization and sampling Globals
volatile uint32_t pui32ADC0Value[129];   //circular buffer
volatile uint32_t pui32ADC1Value[129];  //circular buffer 2
volatile uint32_t pingBuffer[129];
volatile uint32_t pongBuffer[129];
volatile uint32_t pingBuffer2[129];
volatile uint32_t pongBuffer2[129];
volatile uint32_t whiteblack[129];
volatile uint32_t threshold=180;
volatile int32_t slope[129];
volatile int32_t slope2[129];
volatile uint32_t pingFull = 0;
volatile uint32_t pongFull = 0;
volatile uint32_t pingFull2 = 0;
volatile uint32_t pongFull2 = 0;
volatile uint32_t clk_counter;
volatile uint32_t clk_counter2;
volatile int32_t maxIndex = 0;
volatile int32_t minIndex = 0;
volatile int32_t meanIndex = 0;
volatile int32_t lastmeanIndex = 0;
volatile int32_t meanDiff = 0;
volatile int32_t maxIndex2 = 0;
volatile int32_t minIndex2 = 0;
volatile int32_t meanIndex2 = 0;
volatile int32_t cameraIndex = 0;
volatile int32_t lineLost = 0;
volatile uint32_t sum = 0;
volatile int32_t mean = 0;
volatile int32_t max = 0;
volatile int32_t min = 3000;
volatile int32_t max2 = 0;
volatile int32_t min2 = 3000;
volatile uint32_t Vpp = 0;
volatile uint32_t count = 0;
volatile uint32_t toggle = 0; //check if last buffer written to was ping or pong
volatile uint32_t toggle2 = 0; //check if last buffer written to was ping or pong
char letter;
char letter2;
volatile uint32_t count;
volatile uint32_t run = 1;

// Servo/Steering Control Globals
char input;
volatile uint32_t ServoStall = 0;
volatile int32_t error = 0;
volatile int32_t error2 = 0;
volatile int32_t lastError = 0;
volatile uint32_t T;
volatile uint32_t kp;
volatile uint32_t kd;
volatile int32_t PValue;
volatile int32_t IValue;
volatile int32_t DValue;
volatile int32_t CValue;
volatile int32_t Cdiff;
volatile int32_t servoPos;
volatile int32_t inc = 0;
volatile int32_t errorBuffer[256];
volatile int32_t errorBuffer2[256];
volatile int32_t DValueBuffer[256];
volatile uint32_t eIndex = 0;
volatile uint32_t eIndex2 = 0;
volatile uint32_t setSpeed=0;
volatile uint32_t lockLeft = 0;
volatile uint32_t lockRight = 0;
int N = 256;	//size of the buffer ^
int numD = 4;	//number of past terms to sample for D
volatile uint32_t movingAverage = 0;
volatile uint32_t movingAverage2 = 0;
volatile int32_t cameraDiff = 0;

/*		VIP GLOBAL CONSTANTS		*/
volatile uint32_t timer_rate = 700;

volatile uint32_t minSpeed = 350;	// Merely a starter value, change via bluetooth
volatile uint32_t maxSpeed = 500;

volatile uint32_t center = 4615;	// Servo center value

volatile uint32_t minWidth = 6;		// Max and min width boundaries
volatile uint32_t maxWidth = 10;

volatile int32_t confidence = 0;
volatile uint32_t conMax = 30;		// Confidence maximum

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
{	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2); 	// assert SI signal high
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); 	// Clear the timer interrupt.

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);	// assert CLK signal high
    clk_counter = 1;//<<========  UPDATE BUFFER POINTER

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0); 			// deassert SI signal LOW
    ADCProcessorTrigger(ADC0_BASE, 3); 						// Start an ADC conversion using software triggering
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0); 			// deassert CLK signal LOW
}

//*****************************************************************************
void
Timer1IntHandler(void) //timer2 interrupt handler
{	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);	// assert SI signal high
	ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); 	// Clear the timer interrupt.

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6); 	// assert CLK signal high
    clk_counter2 = 1;//<<========  UPDATE BUFFER POINTER

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0); 			// deassert SI signal LOW
    ADCProcessorTrigger(ADC1_BASE, 3); 						// Start an ADC conversion using software triggering
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0); 			// deassert CLK signal LOW
}
//*****************************************************************************
void
adcHandler(void) //adc interrupt handler
{
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // assert CLK signal HIGH
	ADCIntClear(ADC0_BASE, 3); //clear interrupt request

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
		//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0); // deassert test signal LOW
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
void
adcHandler2(void) //adc2 interrupt handler
{
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6); // assert CLK signal HIGH

	ADCIntClear(ADC1_BASE, 3); //clear interrupt request

	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);



	ADCSequenceDataGet(ADC1_BASE, 3, &pui32ADC1Value[clk_counter2]); //address of element


	if (toggle2 == 0)// Store in Ping or Pong buffer if empty-Ping has priority
	{
		pingBuffer2[clk_counter2] = (pui32ADC1Value[clk_counter2]);
	}
	else
	{
		pongBuffer2[clk_counter2] = (pui32ADC1Value[clk_counter2]);
	}

	clk_counter2 = clk_counter2 + 1; //increment counter


	if (clk_counter2 < 129)// Check if not full
	{
		ADCProcessorTrigger(ADC1_BASE, 3); // Start an ADC conversion using software triggering
	}
	else
	{
		//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0); // deassert test signal LOW
		if (toggle2 == 0){
			pingFull2 = 1; //switch pingfull and pongfull flags
			pongFull2 = 0;
			clk_counter2 = 0; //restart clock counter
			toggle2 = 1;

		}
		else {
			pongFull2 = 1; //switch pingfull and pongfull flags
			pingFull2 = 0;
			clk_counter2 = 0; //restart clock counter
			toggle2 = 0;
		}
	}

	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0); // deassert CLK signal LOW
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
    // Get the interrrupt status.
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

    ROM_UARTIntClear(UART1_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(ROM_UARTCharsAvail(UART1_BASE))
    {
    	input = ROM_UARTCharGet(UART1_BASE); // Read the character from UART

		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // ENA high
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // ENB high

        if(input == '0')
        {	//STOP
        	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); 	// INB high
        	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); 	// INA high
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,center);			//neutral
			ServoStall = 1;
        }
        if(input == '1')
        {	//START													//NOT DUTY cycle, this is a hard brake
        	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); 	// INB high
        	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0); 			// INA low
        	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); 	// ENA high
        	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); 	// ENB high
        	ServoStall = 0;
        }
        if(input == '2')
        {
        	maxSpeed = 500;
			minSpeed = 350;
			ServoStall = 0;
        }
		if(input == '3')
		{
			maxSpeed = 550;
			minSpeed = 400;
			ServoStall = 0;
		}
		if(input == '4')
		{
			maxSpeed = 600;
			minSpeed = 450;
			ServoStall = 0;
		}
		if(input == '5')
		{
			maxSpeed = 650;
			minSpeed = 500;
			ServoStall = 0;
		}
		if(input == '6')
		{
			maxSpeed = 700;
			minSpeed = 550;
			ServoStall = 0;
		}
		if(input == '7')
		{
			maxSpeed = 750;
			minSpeed = 600;
			ServoStall = 0;
		}
		if(input == '8')
		{
			maxSpeed = 800;
			minSpeed = 650;
			ServoStall = 0;
		}

        //
        // Blink the LED to show a character transfer is occuring.
        //
        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2) == 0)
        	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        else
        	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
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
    while(*pui8Buffer != '\0')
    {
        ROM_UARTCharPutNonBlocking(UART1_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************

void
st(void)
{
	int i = 1;
	max = 0;	//Initialize the max and min
	min = 3000;
	maxIndex = 0;
	minIndex = 0;

	if (pingFull)
	{
		for (i=1; i < 129; i++)
		{
			if (i<3 || i > 127)
			{
				slope[i] = 0; //TODO: Mess with this value to see if I actually need this or not
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
	else if (pongFull)	// Do the same thing as above, but with other buffer
	{
		for (i=1; i < 129; i++)
		{
			if (i<3 || i > 127)
			{
				slope[i] = 0; // TODO: this too
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
				max = slope[i]; // continuously re-update the max
				maxIndex = i;

			}
			if (slope[i]<min)
			{
				min = slope[i]; // continuously re-update the min
				minIndex = i;
			}

			//UARTprintf("%i ", slope[i]);
		}
	}
	meanIndex = (minIndex + maxIndex) >> 1;

	meanDiff = abs(maxIndex-minIndex);	// Width of the line


	if ((meanDiff >= minWidth) && (meanDiff <= maxWidth)) //width of the line reading
	{
		if (confidence < conMax)
		confidence++;
		if (confidence == conMax)
		{
			lineLost = 0;
			lastmeanIndex = meanIndex;
		}
	}
	else
	{
		if (confidence <= 1)
		{
			lineLost = 1;
		}
		else if (confidence <= (conMax-5))
		{
			lineLost = 1;
			confidence = confidence - 2;
		}
		else
		{
			confidence = confidence - 2;
		}
		meanIndex = lastmeanIndex;
//		lineLost = 1;
	}

	for(i = 1; i < 128; i++)
	{
		if (i < minIndex && i > maxIndex)
		{
//			UARTprintf("0");
		}
		else
		{
//			UARTprintf("1");
		}
	}
//	UARTprintf("0:  %d", meanIndex);
//	UARTprintf("\n");
}

//*****************************************************************************
void
ProportionalControl(void)
{
	//=== Proportional Control ===//
	kp = 55;
	PValue = (kp*(meanIndex-70)) + center; //error will be positive or negative and will turn accordingly


}
//*****************************************************************************
void
IntegrationControl(void)	// Non-Existent
{
	int numI = 2;
	int NI = 512;
	if (eIndex < (numI-1)) //Grabbing 4 values, so val less than 3 needs overflow, but when index=3, then all ok.
	{

	}
	else //No overflow here
	{

	}
}

//*****************************************************************************
void
DifferentialControl(void)
{
	//=== Differential Control ===//
	kd = 300; 		//1 ms
	error = meanIndex-70;

	if (error < -20) // TODO: try changing this to -1 since a right turn needs more weight
	{
		error = error - 20;
	}
	if (eIndex < N) // Conditions for the buffer
	{
		errorBuffer[eIndex] = error;
		eIndex++;
	}
	if (eIndex >= N) // if overflow
	{
		eIndex = 0;
	}

	int lastD = DValue;
	if (eIndex == 0) //Grabbing 4 values, so val less than 3 needs overflow, but when index=3, then all ok.
	{
		DValue = (kd/(6))*(errorBuffer[eIndex] - errorBuffer[N-3] + (3*(errorBuffer[N-1])) - (3*(errorBuffer[N-2])));
		DValue = (-1)*DValue + center;
	}
	else if (eIndex == 1) //Grabbing 4 values, so val less than 3 needs overflow, but when index=3, then all ok.
	{
		DValue = (kd/(6))*(errorBuffer[eIndex] - errorBuffer[N-2] + (3*(errorBuffer[eIndex-1])) - (3*(errorBuffer[N-1])));
		DValue = (-1)*DValue + center;
	}
	else if (eIndex == 2) //Grabbing 4 values, so val less than 3 needs overflow, but when index=3, then all ok.
	{
		DValue = (kd/(6))*(errorBuffer[eIndex] - errorBuffer[N-1] + (3*(errorBuffer[eIndex-1])) - (3*(errorBuffer[eIndex-2])));
		DValue = (-1)*DValue + center;
	}
	else //No overflow here
	{
		DValue = (kd/(6))*(errorBuffer[eIndex] - errorBuffer[eIndex-3] + (3*(errorBuffer[eIndex-1])) - (3*(errorBuffer[eIndex-2])));
		DValue = (-1)*DValue + center;
	}

	if ((DValue > (center-200)) && (DValue < (center+200)))
	{
		DValue = center;			// Helps for oscillations
	//TODO: Check if increasing PValue here will help
	}
	else if (DValue < 3710)			// Check if a sharp right turn (special case since it reads right turns slower)
	{
		DValue -= 400;
	}
}

void
SpeedControl(void)
{
	Cdiff = abs(CValue - center);
	if (Cdiff > 1000) // Half turn, keep max speed
	{
		setSpeed = 200;
	}
	else if (Cdiff > 800) // Half turn, keep max speed
	{
		setSpeed = minSpeed - 50;
	}
	else if (Cdiff > 300)
	{
		setSpeed = (maxSpeed-100);
	}
	else if (Cdiff < 100)
	{
		setSpeed = maxSpeed + 50;
	}
	else
	{
		setSpeed = maxSpeed;
	}

	if ((CValue <= 3510)) // Rightmost value servo can turn 
	{
		CValue = 3510;
		lockLeft = 1;
		lockRight = 0;
	}
	// TODO: Right turn is much further from center than left turn, may cause it to turn slower than left turn
	if ((CValue >= 5520))// Leftmost value servo can turn
	{
		CValue = 5520;
		lockLeft = 0;
		lockRight = 1;
	}
}
//*****************************************************************************
void
FinalControl(void) // Combine the code for camera 1 and 2
{
	st();	// Sample data

	ProportionalControl();
	// IntegrationControl(); (called inside Differential control to use the same buffer of error readings)
	DifferentialControl();

	CValue = (PValue/2) + (DValue/2);

	SpeedControl();

	if(ServoStall == 0)		// Servo Active
	{
		if (lineLost == 0)	// Line not lost
		{
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,CValue);   // P/2 + D/2
			lockLeft = 0;
			lockRight = 0;
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * setSpeed/1000);
		}

		else if (lockLeft)	// Line lost, was turning left
		{
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,3510); // Leftmost value servo can turn
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * minSpeed/1000);
		}

		else if(lockRight)	// Line lost, was turning right
		{
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,5520); // Rightmost value servo can turn
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * minSpeed/1000);
		}
	}

	else					// Servo Inactive
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,center);	//neutral
}
//*****************************************************************************
void
ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
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
}

//*****************************************************************************
void
configADC(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // Enable GPIO Port ADC 1
	ROM_GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_5); // Enable GPIO Port ADC 2

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);// Enable the ADC for camera 1	
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCIntRegister(ADC0_BASE,3,adcHandler);
	ADCIntEnable(ADC0_BASE,3);
	ADCIntClear(ADC0_BASE, 3);

//	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);// Enable the ADC for camera 2
//	ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
//	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH11 | ADC_CTL_IE | ADC_CTL_END);
//	ADCSequenceEnable(ADC1_BASE, 3);
//	ADCIntRegister(ADC1_BASE,3,adcHandler2);
//	ADCIntEnable(ADC1_BASE,3);
//	ADCIntClear(ADC1_BASE, 3);
}
//*****************************************************************************
void
configTimer(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5); // Camera 1 SI, CLK
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6); // Camera 2 SI 2, CLK 2

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //enable the timer for cmaera 1
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); //enable the timer for camera 2
    ROM_IntMasterEnable();

    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
//    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()/timer_rate);
//    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet()/timer_rate); // Camera 2

    ROM_IntEnable(INT_TIMER0A);
//    ROM_IntEnable(INT_TIMER1A); // Camera 2
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
//    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Camera 2

    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
//    ROM_TimerEnable(TIMER1_BASE, TIMER_A); // Camera 2
}

//*****************************************************************************
void
configPWM(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //PWM peripheral Enable	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //motor
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //motor
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2); // ENA
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3); // INA
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7); // ENB
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6); // INB

	SysCtlPWMClockSet(SYSCTL_PWMDIV_16);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //motor

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
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // ENB high
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0); // INA low
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0); // INB low //brake
	//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 3 / 10);  //30%

	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);//motor

	PWMGenEnable(PWM0_BASE, PWM_GEN_3);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);//motor

	ServoStall = 1;		// Only activate servo from bluetooth
}

//*****************************************************************************
void
configBluetooth(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); // Bluetooth Test LED

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); //bluetooth

	GPIOPinConfigure(GPIO_PB0_U1RX); //bluetooth
	GPIOPinConfigure(GPIO_PB1_U1TX); //bluetooth

	ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1); //bt

	ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 115200,
	                             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	                              UART_CONFIG_PAR_NONE));

	ROM_IntEnable(INT_UART1);

	ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT); //bt
}
//*****************************************************************************
int
main(void)
 {

    ROM_FPULazyStackingEnable();

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    ConfigureUART();		// Initialize the UART and write status.

    configADC();			// Initialize the ADC for camera 1 AND camera 2

    configTimer();			// Initialize the timer for camera 1 AND  camera 2

    configPWM(); 			// Initialize the PWM for both the motor AND the servo

    configBluetooth();		// Initialize the UART for bluetooth purposes

    while(run==1)
    {
		if (pingFull | pongFull)
			FinalControl();
    }
}
