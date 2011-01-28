/**************************************************************
Jarvis Schultz and Marcus Hammond

8-25-2010

This is the main section of code for the DC Motor controlled puppeteers 
that were made as part of the Puppet People project in the summer of 2010.
The puppeteers are PIC controlled by a PIC32MX416F512L that is mounted 
on a UBW32 board available from SparkFun.

This main code really does not contain much substance, it simply calls
some initialization functions and sits in an infinite while loop.
The heart of the code can be found in DCMotors_MotorControls.cs
***************************************************************/


/** Includes **************************************************/
#include "DCMotors_FunctionsBroadcast.h"
#include "HardwareProfile.h"
#include "Compiler.h"


/** Global Variables ******************************************/
#define SYS_FREQ 	       (80000000L)	
#define TOGGLES_PER_SEC	       1000
#define CORE_TICK_RATE	       (SYS_FREQ/2/TOGGLES_PER_SEC)
#define UART_TIMEOUT           (2100000)
char ID = '0';

/** User Called Functions *************************************/
void delay(int time)
{
    int i;
    for(i = 0; i<=10000000; i++);
}

/* Will currently only work for robots with a node identifier that is
 * less than two character */
int GetID(void)
{
    INTEnable(INT_U2RX,0);
    INTEnable(INT_U2TX,0);
    char InBuffer[10];
    char ID;
    INTDisableInterrupts();
    delay(1);
    putsUART2("+++");
    delay(1);
    getsUART2(3,InBuffer,UART_TIMEOUT); // Timeout is approx .5 seconds
    putsUART2("ATNI\r");
    getsUART2(2,InBuffer,UART_TIMEOUT);
    ID = InBuffer[0];
    putsUART2("ATCN\r");
    getsUART2(3,InBuffer,UART_TIMEOUT);
    INTEnable(INT_U2RX,1);
    INTEnable(INT_U2TX,1);
    return ID;
}


/** Main Function: ********************************************/
int main()
{
	int PbClk; // Frequency of the peripheral bus clock
	long int j = 0;

	// Let's set the integer PbClk to be the value of the frequency
	// of the peripheral bus clock
	PbClk = SYSTEMConfigPerformance(SYS_FREQ);

	// Turn off JTAG so we get the pins back
 	mJTAGPortEnable(0);
	// Initialize the LED's:
	mInitAllLEDs();		
	// Initialize the PWM pins:
	InitMotorPWM();
	// Initialize UART Communication
	InitUART2(PbClk);

	// Send feedback to the user:
	putsUART2("Program Started\r\n\n");
	while(BusyUART2());
	// Get the robot ID:
	ID = GetID();

	// Initialize the second timer for checking kinematics:
	InitTimer2();
	// Initialize the Encoders:
	InitEncoder();

	while(1)
	{
		// We simply call this function repeatedly, and it executes
		// the main libraries written for the mobile robot
		RuntimeOperation();
		j++;
		if(j%500000 == 0) mLED_4_Toggle();    
	}
	CloseOC1();
	CloseOC2();
	CloseOC3();
	CloseIC1();
	CloseIC3();
	CloseIC5();
}
