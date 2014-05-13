/**************************************************************
Jarvis Schultz

8-27-2010

This is the main section of code for the DC Motor controlled
puppeteers that were made as part of the Puppet People project in the
summer of 2010.  The puppeteers are PIC controlled by a
PIC32MX416F512L that is mounted on a UBW32 board available from
SparkFun.

This main code really does not contain much substance, it simply calls
some initialization functions and sits in an infinite while loop.  The
heart of the code can be found in DCMotors_MotorControls.c.

EDIT: 1-31-2011

This code has been modified so that the PIC now reads the node
identifier address of the XBee.  It stores this value in the global
variable ID.  This variable is used in DCMotors_MotorControls.c to
determine which robot the PIC is attached to.  In other words, it is
used to decide if an instruction sent out over the wireless network
applies to this robot.  This initialization will only run the first
time that the PIC boots up after a re-programming.  If at any time the
user wants to re-run the initialization, all they have to do is hold
the swUser button while re-setting the PIC.  For the initialization to
complete successfully, there should be no data transferred over the
XBee network during initialization.

EDIT: 5-25-2011

Just renamed all of the files involved in this project, and created a
new makefile because we are making the transition over to aluminum
puppeteers with four motors.
***************************************************************/


/** Includes **************************************************/
#include "prototypes_al_pupp.h"
#include "HardwareProfile.h"

/** Global Variables ******************************************/
#define UART_TIMEOUT		(2100000)
#define BASE_PAGE		(0x9D016000)
#define ID_ADDRESS		(BASE_PAGE + 0x400)
#define ID_ADDRESS_FLAG		(BASE_PAGE + 0x800)
#define PUSH_BREAK_COUNT	(250000)
char ID, ID_flag;
unsigned int *ptr_ID, *ptr_ID_flag;

/** User Called Functions *************************************/

// Will currently only work for robots with a node identifier that is
// less than two character 
char GetID(void)
{
    INTEnable(INT_U2RX,0);
    INTEnable(INT_U2TX,0);
    char InBuffer[10];
    char ID;   
    delay();
    putsUART2("+++");
    delay();
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
    // Initialize UART Communication
    InitUART2(PbClk);
    // Initialize motor control pins:
    InitMotorPins();
    
    // Let's set the pointers we initialized to the addresses
    // defined at the beginning
    ptr_ID_flag = (void*)ID_ADDRESS_FLAG;
    ptr_ID = (void*)ID_ADDRESS;
    // Let's read the value in the flag address:
    ID_flag = (char) (*ptr_ID_flag);

    
    if(ID_flag != '1' || !swUser)
    {
        // If either of these are true, then let's read our
        // address from the XBee
        mLED_2_Toggle();
        ID = GetID();
        mLED_3_Toggle();
        // Now, store the newly read ID in its memory address
        delay();
	NVMErasePage((void*) BASE_PAGE);
        NVMWriteWord(ptr_ID , (unsigned int) ID);
        if(NVMIsError())
        {
            mLED_1_Toggle();
            NVMClearError();
        }
        delay();
        // Now, set the flag that says we have read in the memory
        // address
        NVMWriteWord(ptr_ID_flag, (unsigned int) '1');
	if(NVMIsError())
        {
            mLED_2_Toggle();
            NVMClearError();
        }

    }
    else
    {
        // We just read the IDValue
        ID = (char) (*ptr_ID);
    }

    // Send feedback to the user:
    putsUART2("Program Started\r\n\n");
    while(BusyUART2());
    putsUART2("ID = ");
    putcUART2(ID);
    while(BusyUART2());
    putsUART2("\n\r");

    putsUART2("ptr_ID = ");
    putcUART2((char) (*ptr_ID));
    while(BusyUART2());
    putsUART2("\n\r");
    
    while(swProgram)
    {
	mLED_1_On();
	mLED_2_On();
	mLED_3_On();
	mLED_4_On();
    }
    mLED_1_Off();
    mLED_2_Off();
    mLED_3_Off();
    mLED_4_Off();

    // Initialize motor PWM:
    InitMotorPWM();
    // Initialize the second timer for checking kinematics:
    InitTimer2();
    // Initialize the fourth timer for data string timeouts:
    InitTimer4();
    // Initialize the Encoders (and all interrupts):
    InitEncoder();
    // Initialize Watchdog Timer
    ClearEventWDT();
    DisableWDT();
    EnableWDT();

    unsigned int push_count = 0;
    while(1)
    {
	// We simply call this function repeatedly, and it executes
	// the main libraries written for the mobile robot
	RuntimeOperation();
	j++;
	if(j%500000 == 0) mLED_4_Toggle();

	// To ensure that the PIC code is not stuck somehow, we
	// use a watchdog timer. Let's reset it here:
	ClearWDT();

	// break out if we have pushed User button enough:
	if (!swUser)
	    push_count++;
	else
	    push_count = 0;
	if (push_count > PUSH_BREAK_COUNT)
	    break;
    }

    // first we disable timeout timer interrupt
    CloseTimer4();
    ConfigIntTimer4(T4_INT_OFF);

    // loop for contolling winches:
    j = 0;
    mLED_4_On();
    mLED_3_On();
    while(1)
    {
	j++;
	// run manual winch commands:
	manual_winch_runtime();
	// blink
	if(j%500000 == 0) mLED_3_Toggle();
	ClearWDT();
    }

    
    CloseOC5();
    CloseOC1();
    CloseOC2();
    CloseOC3();
    CloseCapture1();
    CloseCapture2();
    CloseCapture4();
    CloseCapture5();
}
