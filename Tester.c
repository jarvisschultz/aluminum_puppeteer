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
#include "HardwareProfile.h"
#include "Compiler.h"


/** Global Variables ******************************************/
#define SYS_FREQ 				(80000000L)	
#define TOGGLES_PER_SEC			1000
#define CORE_TICK_RATE	       (SYS_FREQ/2/TOGGLES_PER_SEC)
#define BAUDRATE (115200)
short int ID = 0;

/** Function Declarations *************************************/

/** Interrupt Handlers ****************************************/

/** User Called Functions *************************************/
void InitUART2(int pbClk)
{
    // define setup Configuration 1 for OpenUARTx
    // Module Enable 
    // Work in IDLE mode 
    // Communication through usual pins 
    // Disable wake-up 
    // Loop back disabled 
    // Input to Capture module from ICx pin 
    // no parity 8 bit 
    // 1 stop bit 
    // IRDA encoder and decoder disabled 
g    // CTS and RTS pins are disabled 
    // UxRX idle state is '1' 
    // 16x baud clock - normal speed
#define config1 	UART_EN | UART_IDLE_CON | UART_RX_TX | UART_DIS_WAKE | UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_IRDA_DIS | UART_DIS_BCLK_CTS_RTS| UART_NORMAL_RX | UART_BRGH_SIXTEEN
	
    // define setup Configuration 2 for OpenUARTx
    // IrDA encoded UxTX idle state is '0'
    // Enable UxRX pin
    // Enable UxTX pin
    // Interrupt on transfer of every character to TSR 
    // Interrupt on every char received
    // Disable 9-bit address detect
    // Rx Buffer Over run status bit clear
#define config2		UART_TX_PIN_LOW | UART_RX_ENABLE | UART_TX_ENABLE | UART_INT_TX | UART_INT_RX_CHAR | UART_ADR_DETECT_DIS | UART_RX_OVERRUN_CLEAR	

    // Open UART2 with config1 and config2
    OpenUART2( config1, config2, pbClk/16/BAUDRATE-1);	// calculate actual BAUD generate value.
		
    // Configure UART2 RX Interrupt with priority 7
    ConfigIntUART2(UART_INT_PR7 | UART_RX_INT_EN);
}


void SendDataBuffer(const char *buffer, UINT32 size) 
{ 
    while(size) 
    { 
        while(!UARTTransmitterIsReady(UART2)) 
            ; 
        UARTSendDataByte(UART2, *buffer); 
        buffer++; 
        size--; 
    } 
    while(!UARTTransmissionHasCompleted(UART2)) 
        ; 
}


void delay(int time)
{
    int i;
    for(i = 0; i<=40000000; i++);
}

/* Will currently only work for robots with a node identifier that is
 * less than two character */

int GetID(void)
{
    char InBuffer[10];
    int ID;
    INTDisableInterrupts();
    delay(1);
    putsUART2("+++");
    delay(1);
    getsUART2(3,InBuffer,2100000); // Timeout is approx .5 seconds
    putsUART2("ATNI\r");
    getsUART2(2,InBuffer,2100000);
    ID = InBuffer[0]-'0';
    putsUART2("ATCN\r");
    getsUART2(3,InBuffer,2100000);
    return ID;
}


/** Main Function: ********************************************/
int main()
{
	int PbClk; // Frequency of the peripheral bus clock
	long int j = 0;
	int i;
	UINT8 data[32];
	
	
	// Let's set the integer PbClk to be the value of the frequency
	// of the peripheral bus clock
	PbClk = SYSTEMConfigPerformance(SYS_FREQ);	
	
	// Turn off JTAG so we get the pins back
 	mJTAGPortEnable(0);
	// Initialize the LED's:
	mInitAllLEDs();		
	// Initialize UART Communication
	InitUART2(PbClk);
	
	// Send feedback to the user:
	putsUART2("Program Started\r\n\n");
	while(BusyUART2());

	ID = GetID();

	while(1)
	{
		// We simply call this function repeatedly, and it executes
		// the main libraries written for the mobile robot
		j++;
		if(j%500000 == 0)
		{
			mLED_4_Toggle();
			sprintf(data,"%d \n\r",j);
			SendDataBuffer(data,strlen(data));	
		}
	}
}













