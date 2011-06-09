/*************************************************************
Jarvis Schultz and Marcus Hammond
8-25-2010

**************************************************************/



/** Includes ************************************************/
#include "HardwareProfile.h"
#include <plib.h>
#include <Compiler.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#ifndef _MOTORS
#include "DCMotors_FunctionsBroadcast.h"
#define _MOTORS
#endif



/** Defines ***************************************************/
#define TRUE            1
#define FALSE		0
#define DIRECTION_PIN_L	LATDbits.LATD3
#define DIRECTION_PIN_R	LATDbits.LATD5
#define DIRECTION_PIN_T	LATCbits.LATC13
#define FORWARD		0
#define REVERSE		1
#define T2options	T2_ON | T2_PS_1_2 | T2_SOURCE_INT  	// These are the setup options for timer two (CheckKinematics)
#define T3options 	T3_ON | T3_PS_1_1 | T3_SOURCE_INT  	// These are the setup options for timer three (Motor PWM Period)
#define T4options 	T4_ON | T4_PS_1_256 | T4_SOURCE_INT  	// These are the setup options for timer three (Motor PWM Period)
#define BAUDRATE	(111111)				// This is the rate that we will communicate over RS232 at
#define CHANNEL_A_L	PORTDbits.RD12
#define CHANNEL_B_L	PORTDbits.RD13
#define CHANNEL_A_R	PORTDbits.RD9
#define CHANNEL_B_R	PORTDbits.RD6
#define CHANNEL_A_T	PORTDbits.RD11
#define CHANNEL_B_T	PORTCbits.RC14
#define MAX_RESOLUTION	3999	       // Proportional to period of PWM (represents the maximum 
				       // number that we can use for PWM function)
#define ERROR_DEADBAND	0.05           // This is a deadband where we do nothing to the motors
#define FULL_PACKET_SIZE     12        // This is the length of a data string
#define SMALL_PACKET_SIZE    3 
#define frequency  1500	       // Let's check the kinematics this many times per second
/* #define GEARRATIO  (19.0*(3.0/4.0))    // The gear ratio of the gearhead on the DC motor */
#define GEARRATIO  (19.0*(46.0/42.0))    // The gear ratio of the gearhead on the DC motor
#define CPR  100.0	               // The number of counts per revolution of the motor's encoder
#define dtbase (1.0/frequency)         // The period of CheckKinematics calls
#define convert (M_PI/(CPR*dtbase*GEARRATIO)) // For converting angular wheel and motor velocities
/* #define converttop (convert*(3.0/4.0))      // For the original puppeteers */
#define converttop (convert*(46.0/42.0))      // For the aluminum puppeteers
#define ticktime (2.0/(80000000.0))    // For calculating times since timer ISR's were initially called
#define MAX_BAD_DATA_TOTAL (10)	       
#define MAX_BAD_DATA	(3)
#define MAX_BAD_COUNTER  (200)
#define timeout_frequency (10)
#define SYS_FREQ	(80000000L)


/** Global Variables **************************************************/
static short int DATA_LENGTH = 12;
static float left_speed = 0.0;		// This variable is the current left motor speed in rad/ second
static float right_speed = 0.0;		// This variable is the current right motor speed in rad/ second
static float top_speed = 0.0;		// This variable is the current top motor speed in rad/ second
static long long left_steps = 0;	// This variable is used for tracking how many encoder pulses the left motor has rotated
static long long right_steps = 0;	// This variable is used for tracking how many encoder pulses the right motor has rotated
static long long top_steps = 0;	        // This variable is used for tracking how many encoder pulses the top motor has rotated
static long long left_steps_last = 0;	// This variable is used for tracking how many encoder pulses the left motor had previously rotated
static long long right_steps_last = 0;  // This variable is used for tracking how many encoder pulses the right motor had previously rotated
static long long top_steps_last = 0;	// This variable is used for tracking how many encoder pulses the top motor had previously rotated

static float x_pos = 0.0;		// This variable is what the PIC sees as its current x-position in some world frame
static float y_pos = 0.0;		// This variable is what the PIC sees as its current y-position in some world frame
static float theta = 0.0;		// This variable is what the PIC sees as its current orientation (in radians) w.r.t 
                                        // the positive global x-axis (between 0 and 2pi)	
static float height = 0.0;		// This is the height of the end of the string.  This height is never negative, it is the
                                        // height relative to the height set by the string when it is all the way down.
static float d_pulley = 0.019049999;           // This is the diameter of the pulley that the string winds onto
/* static float L = 5.95/2;		// This is one-half of the robot's track width in meters */
static float L = 0.132334/2;		// This is one-half of the robot's track width in meters
static float D = 0.07619999999999;	// Diameter of the wheel in meters
static float speed = 6.0;		// This is the default wheel revolution rate when in pose control mode (rad/s).
static unsigned char RS232_In_Buffer[FULL_PACKET_SIZE] = "zzzzzzzzzzzz";//  This is an array that is initialized with 
	                                                    // useless data in it; it is used for temporary 
                                                            // storage of data brought in from the PC on UART2
static unsigned char Command_String[FULL_PACKET_SIZE] = "zzzzzzzzzzzz";
static int i = 0;	        // This is for marking the position in the RS232_In_Buffer that we are writing into.
static int data_flag = 0;  	// This variable is used for telling if we have received updated information about the robot's
                                // current position or current instructions.  If we have, we break out of the current control loop
                                // and recalculate what we need to do.
static int pose_flag = 0;  	// This flag is used for determining when the SetPose function should be called.  If we are just 
                                // sitting in main waiting for an instruction to do, the PoseUpdate function should call SetPose.
                                // if we have reached the PoseUpdate function while executing SetPose, SetPose will recursively call
                                // itself and PoseUpdate does not need to call SetPose
static int header_flag = 0;	// This flag is used for determining if we have received a header on the receive pins.
static int height_flag = 0;	// This flag is used for determining if we are controlling the position of the top motor
static float x_sent = 0.0; 	// This is the value of the x-coordinate received
static float y_sent = 0.0; 	// This is the value of the y-coordinate received
static float ori_sent = 0.0;	// This is the value of the orientation received
static float height_sent = 0.0;	// This is the value of the desired height of the end of the string in inches
static float first_angle = 0.0; 	// This is the angle of the vector from the robots current position towards its goal position
                                        // to begin heading towards the new point sent over RS232
static int exec_state = 0;	        // This variable is to determine which mode of operation we are currently in:
						// 0) Just sitting there
						// 1) Wheel Speed Control
						// 2) Initial rotation towards destination
						// 3) Driving towards destination
						// 4) Rotating towards final orientation
static float left_desired;	// This is the desired wheel speed of the left motor in rad/sec sent over RS232
static float right_desired;	// This is the desired wheel speed of the right motor in rad/sec sent over RS232	
static float top_desired;	// This is the desired wheel speed of the top motor in rad/sec sent over RS232	
//static UINT8 STR[1024];		// An empty string we use for sending data
static float kp = 500;		// Gain on the proportional error term
static float ki = 50;		// Gain on the integral error term
static float kd = 0.1;		// Gain on the derivative error term
extern char ID;
static char BROADCAST = '9';

// Add a bunch of variables for communication safety:
static unsigned char header_list[10]={'p','l','r','h','s','q','t','m','w','e'};
/******************************************************************************/
// Note that the header characters mean the following:
//	'p' = Drive to a desired pose
//	'l' = Re-define the robot's position and orientation
//	'r' = Stop driving, and re-set the global configuration to the robot's
//		current pose
//	'h' = Motor speed command
//	's' = Change default speed for pose control
//	'q' = Stop the robot, and reset movement_flag
//	't' = Controls for the winch motors
//	'm' = Start command; must receive this before driving can begin
//	'w' = Current pose request
//	'e' = Current motor speeds request
/******************************************************************************/
static short int bad_data_total = 0;
static short int bad_data = 0;
static short int movement_flag = 0;
static short int midstring_flag = 0;
static short int timeout_flag = 0;
static short int bad_data_counter = 0;
extern int total_replies;


/** Interrupt Handler Functions:**************************************************************************************************/
// UART 2 interrupt handler
// it is set at priority level 2
void __ISR(_UART2_VECTOR, ipl6) IntUart2Handler(void)
{
    mLED_3_On();
    unsigned char temp;
    // Is this from receiving data?
    if(mU2RXGetIntFlag())
    {
	// Clear the RX interrupt Flag
	mU2RXClearIntFlag();

	// Read a char into a temporary variable:
	temp = ReadUART2();

	// Now we need to determine if the data received is one of
	// our header characters.  If it is, we will reset the value 
	// of the buffer counter and flag that we received a header.
	short int j = 0;
	if (midstring_flag != 1)
	{
	    for(j=0;j<sizeof(header_list);j++)
	    {
		if (temp == header_list[j])
		{
		    i = 0;
		    header_flag = 1;
		    midstring_flag = 1;
		    DATA_LENGTH = FULL_PACKET_SIZE;
		    // If a request, it is a short packet!
		    if (temp == 'w' || temp == 'e')
			DATA_LENGTH = SMALL_PACKET_SIZE;
		    break;
		}
	    }
	}

	// Regardless of what temp is, let's place it in the in buffer
	RS232_In_Buffer[i%DATA_LENGTH] = temp;
		
	// Now let's increment i
	i++;
	// Also increment bad_data_counter
	bad_data_counter++;

	// If we have received a full-length set of data, let's
	// validate its integrity by first making sure that the message
	// is intended for this robot, and then checking the checksum.
	// If this is not a valid set of data, let's find the first header
	// and push it to the front of the buffer and start adding on the new
	// characters at the end.
	if(i == DATA_LENGTH && header_flag == 1)
	{
	    unsigned int checksum = 0;
	    for(j = 0; j < DATA_LENGTH-1; j++)
		checksum += RS232_In_Buffer[j];
	    checksum = 0xFF-(checksum & 0xFF);

	    if (RS232_In_Buffer[1] == ID || RS232_In_Buffer[1] == BROADCAST)
	    {
		// If either of these are true, then we can check the checksum:
		if(checksum == RS232_In_Buffer[DATA_LENGTH-1])
		{
		    data_flag = 1;
		    mLED_1_Toggle();
		    midstring_flag = 0;
		    bad_data = 0;
		    bad_data_counter = 0;
		    timeout_flag = 0;
		    memcpy(Command_String, RS232_In_Buffer, DATA_LENGTH);
		}
		// If checksum is bad, we have found a set of data intended for
		// this robot that is corrupt in some way.  Let's increment the
		// bad data counter, and push the next header to the top of the
		// buffer.  If there is no other headers in the buffer, we clear
	        // the header_flag and the midstring flag so that we can start
		// searching for our next string
		else
		{
		    bad_data_total++;
		    bad_data++;

		    short int new_pos = 0;
		    // Let's check to see if there is a new header in the buffer
		    new_pos = Data_Checker(RS232_In_Buffer);

		    if (new_pos != 0) i = new_pos;
		    else
		    {
			midstring_flag = 0;
			header_flag = 0;
		    }
		}		  
	    }
	    else
	    {
		midstring_flag = 0;
		header_flag = 0;
	    }
	}
	// If we have too much bad data, let's quit!
	if (bad_data_total >= MAX_BAD_DATA_TOTAL || bad_data >= MAX_BAD_DATA)
	{
	    Reset_Robot();
	}

	if (bad_data_counter >= MAX_BAD_COUNTER)
	{
	    Reset_Robot();
	}
    }

    // Is this interrupt from sending data?
    if ( mU2TXGetIntFlag() )
    {
	mU2TXClearIntFlag();
    }
    mLED_3_Off();
}

// This is the ISR that gets called when we detect a rising or falling edge on CHANNEL_A_R
void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl5) CheckPosition_r()
{
    static int tempA, tempB;
    tempA = CHANNEL_A_R;
    tempB = CHANNEL_B_R;
		
    // Let's clear the interrupt flag:
    INTClearFlag(INT_IC2);
    /* mLED_2_Toggle();  */
    // Now we can perform logic to determine which direction we are going and we can increment counter
    if(tempA)
    {
	// So, we know that we just detected a rising edge on channel A.  Let's determine whether
	// channel b is high or low to determine our direction:
	if(tempB) right_steps++;
	else right_steps--;
    }
    else if(!tempA)
    {
	if(tempB) right_steps--;
	else right_steps++;
    }
}

// This is the ISR that gets called when we detect a rising or falling edge on CHANNEL_A_L
void __ISR(_INPUT_CAPTURE_5_VECTOR, ipl5) CheckPosition_l()
{
    static int tempA, tempB;
    tempA = CHANNEL_A_L;
    tempB = CHANNEL_B_L;
		
    // Let's clear the interrupt flag:
    INTClearFlag(INT_IC5);
    /* mLED_3_Toggle(); */
    // Now we can perform logic to determine which direction we are going and we can increment counter
    if(tempA)
    {
	// So, we know that we just detected a rising edge on channel A.  Let's determine whether
	// channel b is high or low to determine our direction:
	if(tempB) left_steps--;
	else left_steps++;
    }
    else if(!tempA)
    {
	if(tempB) left_steps++;
	else left_steps--;
    }
}


// This is the ISR that gets called when we detect a rising or falling edge on CHANNEL_A_T
void __ISR(_INPUT_CAPTURE_4_VECTOR, ipl5) CheckPosition_t()
{
    static int tempA, tempB;
    tempA = CHANNEL_A_T;
    tempB = CHANNEL_B_T;
	
    // Let's clear the interrupt flag:
    INTClearFlag(INT_IC4);
    // Now we can perform logic to determine which direction we are going and we can increment counter
    if(tempA)
    {
	// So, we know that we just detected a rising edge on channel A.  Let's determine whether
	// channel b is high or low to determine our direction:
	if(tempB) top_steps++;
	else top_steps--;
    }
    else if(!tempA)
    {
	if(tempB) top_steps--;
	else top_steps++;
    }
}


void __ISR(_TIMER_4_VECTOR, ipl7) Data_Timeout()
{
    static int timeout_count = 0;
    timeout_count++;
    if (timeout_count%5 == 0)
    {
	// If any motor is running, and timeout_flag is high, we are going to reset:
	if (timeout_flag == 1)
	{
	    if( (fabs(left_desired) >= 0.1) || (fabs(right_desired) >= 0.1)
		|| (fabs(top_desired) >= 0.1))
	    {
		// Reset!
		Reset_Robot();
	    }
	}
	timeout_flag = 1;
	
    }
    mT4ClearIntFlag();
}

//  In the following function, we will be updating the robot's pose 
//  using forward kinematics and odometry data.  We will also decide 
//  if we new controls are needed to be sent to the wheels so that 
//  the robot can continue to do what it is supposed to be doing.
void __ISR(_TIMER_2_VECTOR, ipl4) CheckKinematics()
{
    float Vr = 0.0;
    float Vl = 0.0;
    float Vt = 0.0;
    float omega = 0.0;
    float R = 0.0;
    /* float dt = 0.0; */
    int top_current, left_current, right_current;

    left_current = left_steps;
    right_current = right_steps;
    top_current = top_steps;
            
    // Let's first calculate the current angular velocity of each wheel:
    left_speed = convert*((float) (left_current-left_steps_last));
    right_speed = convert*((float) (right_current-right_steps_last));
    top_speed = converttop*((float) (top_current-top_steps_last));
	
    // Now let's store the current counts for the next CheckKinematics call:
    left_steps_last = left_current;
    right_steps_last = right_current;
    top_steps_last = top_current;
	
    // Now, let's calculate the error between the actual and desired velocities 
    float left_error = left_desired-left_speed;
    float right_error = right_desired-right_speed;
    float top_error = top_desired-top_speed;
    
    // Now let's calculate how long it has been since this function has been called.
    /* dt = ticktime*(((float) ReadTimer2())+1.0)+dtbase; */

    // Now, we set the PWM Values:
    if(fabs(left_error) > ERROR_DEADBAND) SetSpeedLeft(left_error, dtbase);
    if(fabs(right_error) > ERROR_DEADBAND) SetSpeedRight(right_error, dtbase);
    if(fabs(top_error) > ERROR_DEADBAND) SetSpeedTop(top_error, dtbase);

    // Now let's get the wheels speeds and convert them into translational velocities
    Vr = (D/2.0)*(right_speed);
    Vl = (D/2.0)*(left_speed);
    Vt = (d_pulley/2.0)*(top_speed);
		
    // Let's calculate the distance to the instantaneous center of rotation and the angular vel.
    R = L*((Vl+Vr)/(Vr-Vl));
    omega = (Vr-Vl)/(2.0*L);
		
    // Now we do the forward kinematics
    if (fabs(R) > 10000.0) // This would imply that the robot is essentially going straight
    {
	// So, let's perform the straight version kinematics
	x_pos += Vr*dtbase*cosf(theta);
	y_pos += Vr*dtbase*sinf(theta);
    }
    else
    {
	// Now let's perform the general kinematics
	x_pos += cosf(omega*dtbase)*R*sinf(theta)+sinf(omega*dtbase)*R*cosf(theta)-R*sinf(theta);
	y_pos += sinf(omega*dtbase)*R*sinf(theta)-cosf(omega*dtbase)*R*cosf(theta)+R*cosf(theta);
	theta += omega*dtbase;
    }
	
    // Let's force theta to be between 0 and 2pi
    if(theta < 0.0) theta += 2.0*M_PI;
    if(theta >= 2.0*M_PI) theta -= 2.0*M_PI;	
	
    // Now, let's update the height of our string
    height += Vt*dtbase;
	
    if(height_flag == 1)
    {
	if(fabs(height_sent-height) < 0.01)
	{
	    height_flag = 0;
	    top_desired = 0.0;
	}
    }
	
    // Are we just controlling wheel speeds?
    if (exec_state == 1)
    {
	// We don't need to check anything then
    }
    // Are we performing an initial rotation to begin heading towards our destination?
    else if (exec_state == 2)
    {
	// Now, let's determine if we have reached the desired orientation yet
	if(fabs(theta-first_angle) <= 0.01 || fabs(fabs(theta-first_angle)-2.0*M_PI) <= 0.01)
	{
	    theta = first_angle;
	    exec_state = 3;
	    pose_flag = 1;
	    right_desired = 0.0;
	    left_desired = 0.0;
	}
    }
    else if (exec_state == 3)
    {
	// Now, are we there yet?
	if(fabs(x_sent-x_pos) < 1.0 && fabs(y_sent-y_pos) < 1.0)
	{
	    x_pos = x_sent;
	    y_pos = y_sent;
	    right_desired = 0.0;
	    left_desired = 0.0;
	    exec_state = 4;
	    pose_flag = 1;
	}
    }
    else if (exec_state == 4)
    {	
	// Now, let's determine if we have reached the final orientation yet
	if(fabs(theta-ori_sent) <= 0.01 || fabs(fabs(theta-ori_sent)-2.0*M_PI) <= 0.01)
	{
	    theta = ori_sent;
	    // Since we have, we don't need to keep updating the kinematics
	    exec_state = 0;
	    pose_flag = 0;
	    // Also, let's stop the motors:
	    right_desired = 0.0;
	    left_desired = 0.0;
	}
    }
    mT2ClearIntFlag(); 		// clear interrupt flag
}



/** User Called Functions ********************************************************************************/
// Following function returns the number of 
// steps that the left motor has taken:
int GetStepsLeft(void)
{
    return left_steps;
}

// Following function returns the number of 
// steps that the right motor has taken:
int GetStepsRight(void)
{
    return right_steps;
}

// Following function returns the number of 
// steps that the right motor has taken:
int GetStepsTop(void)
{
    return top_steps;
}

// The following function returns the current speed 
// of the left motor
float GetSpeedLeft(void)
{
    return left_speed;
}

// The following function returns the current speed 
// of the left motor
float GetSpeedRight(void)
{
    return right_speed;
}

// The following function returns the current speed 
// of the left motor
float GetSpeedTop(void)
{
    return top_speed;
}

// Next function allows to user to re-set a "home" position by
// manually changing the number of encoder steps for the left motor:
void SetStepsLeft(int set_steps)
{
    INTEnable(INT_OC3, 0);
    left_steps = set_steps;
    INTEnable(INT_OC3, 1);
}

// Next function allows to user to re-set a "home" position by
// manually changing the number of encoder steps for the right motor:
void SetStepsRight(int set_steps)
{
    INTEnable(INT_OC5, 0);
    right_steps = set_steps;
    INTEnable(INT_OC5, 1);
}

// Next function allows to user to re-set a "home" position by
// manually changing the number of encoder steps for the right motor:
void SetStepsTop(int set_steps)
{
    INTEnable(INT_OC2, 0);
    top_steps = set_steps;
    INTEnable(INT_OC2, 1);
}

// This function allows the user to specify the desired left motor speed:
void SetSpeedLeft(float error, float dt)  // motor speed in rad/s
{
    static float sum_error = 0.0;	// This is the integrated error
    static float d_error = 0.0;		// This is the derivative error
    static float last_d_error = 0.0;	// This is the previous derivative error    
    static float last_error = 0.0;	// This is the error sent to SetSpeed functions last time for D-control
    static float total_error = 0.0;	// This is the sum of all three components of the error
    static unsigned int PWMVal = 0;     // The actual value that will get stored in the PMW register
    static float tau = 1.0/(2.0*M_PI*500.0);// Filtering parameter - Value in denominator sets cutoff frequency for LPF

    // Calculate integrated error:
    sum_error += error;
    // Calculate filtered derivative error:
    d_error = (dtbase*error+dtbase*last_error+(2*tau-dtbase)*last_d_error)/(dtbase+2*tau);
    // Calculate total error:
    total_error = error*kp+sum_error*ki+d_error*kd;
    // Get PWM Value:
    PWMVal = (unsigned int) (fabs(total_error));
    // Set stored values:
    last_error = error;
    last_d_error = d_error;

    // Prevent Windup:
    if(PWMVal > MAX_RESOLUTION) 
    {
	PWMVal = MAX_RESOLUTION;
	sum_error -= error;
    }

    // Now set the PWM Values:
    if(total_error < 0)
    {
	DIRECTION_PIN_L = FORWARD;
	SetDCOC3PWM(PWMVal);
    }
    else
    {

	DIRECTION_PIN_L = REVERSE;
	SetDCOC3PWM(PWMVal);
    }
}


// This function allows the user to specify the desired right motor speed:
void SetSpeedRight(float error, float dt)  // motor speed in rad/s
{
    static float sum_error = 0.0;	// This is the integrated error
    static float d_error = 0.0;		// This is the derivative error
    static float last_d_error = 0.0;	// This is the previous derivative error    
    static float last_error = 0.0;	// This is the error sent to SetSpeed functions last time for D-control
    static float total_error = 0.0;	// This is the sum of all three components of the error
    static unsigned int PWMVal = 0;     // The actual value that will get stored in the PMW register
    static float tau = 1.0/(2.0*M_PI*500.0);// Filtering parameter - Value in denominator sets cutoff frequency for LPF

    // Calculate integrated error:
    sum_error += error;
    // Calculate filtered derivative error:
    d_error = (dtbase*error+dtbase*last_error+(2*tau-dtbase)*last_d_error)/(dtbase+2*tau);
    // Calculate total error:
    total_error = error*kp+sum_error*ki+d_error*kd;
    // Get PWM Value:
    PWMVal = (unsigned int) (fabs(total_error));
    // Set stored values:
    last_error = error;
    last_d_error = d_error;
    
    if(PWMVal > MAX_RESOLUTION) 
    {
	PWMVal = MAX_RESOLUTION;
	sum_error -= error;
    }
	
	
    if(total_error > 0)
    {
	DIRECTION_PIN_R = FORWARD;
	SetDCOC5PWM(PWMVal);
    }
    else
    {
	DIRECTION_PIN_R = REVERSE;
	SetDCOC5PWM(PWMVal);
    }
}

// This function allows the user to specify the desired top motor speed:
void SetSpeedTop(float error, float dt)  // motor speed in rad/s
{
    static float sum_error = 0.0;	// This is the integrated error
    static float d_error = 0.0;		// This is the derivative error
    static float last_d_error = 0.0;	// This is the previous derivative error    
    static float last_error = 0.0;	// This is the error sent to SetSpeed functions last time for D-control
    static float total_error = 0.0;	// This is the sum of all three components of the error
    static unsigned int PWMVal = 0;     // The actual value that will get stored in the PMW register
    static float tau = 1.0/(2.0*M_PI*500.0);// Filtering parameter - Value in denominator sets cutoff frequency for LPF

    // Calculate integrated error:
    sum_error += error;
    // Calculate filtered derivative error:
    d_error = (dtbase*error+dtbase*last_error+(2*tau-dtbase)*last_d_error)/(dtbase+2*tau);
    // Calculate total error:
    total_error = error*kp+sum_error*ki+d_error*kd;
    // Get PWM Value:
    PWMVal = (unsigned int) (fabs(total_error));
    // Set stored values:
    last_error = error;
    last_d_error = d_error;
    
    if(PWMVal > MAX_RESOLUTION) 
    {
	PWMVal = MAX_RESOLUTION;
	sum_error -= error;
    }
	
	
    if(total_error > 0)
    {
	DIRECTION_PIN_T = FORWARD;
	SetDCOC2PWM(PWMVal);
    }
    else
    {
	DIRECTION_PIN_T = REVERSE;
	SetDCOC2PWM(PWMVal);
    }
}

void InitMotorPWM(void)
{
    // Set port D pins as low digital outputs:
    LATD |= 0x0000;
    // PWM Pins set to outputs:
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD4 = 0;
    TRISDbits.TRISD1 = 0;
	
    // Direction Pins to outputs:
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD5 = 0;
    TRISCbits.TRISC13 = 0;
	
    // init OC2 module, on pin D1
    OpenOC2( OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
	
    // init OC5 module, on pin D4
    OpenOC5( OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
	
    // init OC3 module, on pin D2
    OpenOC3( OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
	
    // init Timer3 
    OpenTimer3(T3options , MAX_RESOLUTION);
}


void InitEncoder(void)
{	
    // Let's set the Channel_B pins (D11) to be digital inputs:
    // Channel A inputs:
    TRISDbits.TRISD12 = 1;
    TRISDbits.TRISD9 = 1;
    TRISDbits.TRISD11 = 1;
	
    // Channel B inputs:
    TRISDbits.TRISD13 = 1;
    TRISDbits.TRISD6 = 1;
    TRISCbits.TRISC14 = 1;
	
    // Now we can configure the CHANNEL_A_L pin (D12) to be an input capture pin:
    // - Capture Every edge
    // - Enable capture interrupts
    // - Use Timer 3 source
    // - Capture rising edge first
    OpenCapture5( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC |IC_FEDGE_RISE | IC_ON );
    // Configure the interrupt options
    ConfigIntCapture5(IC_INT_ON | IC_INT_PRIOR_5);
	
    // Now we can configure the CHANNEL_A_R pin (D9) to be an input capture pin:
    // - Capture Every edge
    // - Enable capture interrupts
    // - Use Timer 3 source
    // - Capture rising edge first
    OpenCapture2( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC |IC_FEDGE_RISE | IC_ON );
    // Configure the interrupt options
    ConfigIntCapture2(IC_INT_ON | IC_INT_PRIOR_5);
	
    // Now we can configure the CHANNEL_A_T pin (D11) to be an input capture pin:
    // - Capture Every edge
    // - Enable capture interrupts
    // - Use Timer 3 source
    // - Capture rising edge first
    OpenCapture4( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC |IC_FEDGE_RISE | IC_ON );
    // Configure the interrupt options
    ConfigIntCapture4(IC_INT_ON | IC_INT_PRIOR_5);
	
    // Enable system-wide interrupts:
    INTEnableSystemMultiVectoredInt();
} 

// This function is used for initilizing the UART2
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
    // CTS and RTS pins are disabled 
    // UxRX idle state is '1' 
    // 16x baud clock - normal speed
#define config1		UART_EN | UART_IDLE_CON | UART_RX_TX | UART_DIS_WAKE | UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_IRDA_DIS | UART_DIS_BCLK_CTS_RTS| UART_NORMAL_RX | UART_BRGH_SIXTEEN
	
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
		
    // Configure UART2 RX Interrupt with priority 6
    ConfigIntUART2(UART_INT_PR6 | UART_RX_INT_EN);
}								


void InitTimer2(void)
{
    int Per;
    Per = (80000000/frequency)/2-1;  // Set the value in the period register
    OpenTimer2(T2options, Per);	
    mT2SetIntPriority( 4); 	// set Timer 2 Interrupt Priority
    mT2ClearIntFlag(); 		// clear interrupt flag
    mT2IntEnable( 1);		// enable timer 2 interrupts
}

void InitTimer4(void)
{
    int Per;
    Per = (80000000/timeout_frequency)/256-1;  // Set the value in the period register
    OpenTimer4(T4options, Per);	
    mT4SetIntPriority( 7); 	// set Timer 4 Interrupt Priority
    mT4ClearIntFlag(); 		// clear interrupt flag
    mT4IntEnable( 1);		// enable timer 4 interrupts    
}


void SetPose(float xdest,float ydest,float thdest)
{
    static int rot1_flag = 0, rot2_flag = 0, dir_flag = 0;
    int rot1_f = 0, rot2_f = 0, rot1_b = 0, rot2_b = 0;
    float thback = 0.0, tpback = 0.0, costf = 0.0, costb = 0.0;
	
    // Are we performing the initial orientation?
    if(exec_state == 2)
    {
	// First, let's calculate the angle that we need to rotate through to 
	// begin heading towards the desired point:
	if(fabs(xdest-x_pos) < 0.000001 && fabs(ydest-y_pos) < 0.000001)
	{
	    first_angle = theta;
	}	
	else
	{
	    first_angle = (atan2f((ydest-y_pos),(xdest-x_pos)));	
	}

	if(first_angle <= 0.0) first_angle += 2.0*M_PI;
	// Now calculate the orientation of the back of the robot:
	thback = theta+M_PI;
	if(thback >= 2.0*M_PI) thback -= 2.0*M_PI;
	// Also, let's figure out where the back would be oriented at the destination orientation:
	tpback = ori_sent+M_PI;
	if(tpback >= 2.0*M_PI) tpback -= 2.0*M_PI;
		
	// Let's let RuntimeOperation() know that it can stop calling this function now
	pose_flag = 0;
		
	// Now let's determine which direction we want to rotate, and which direction we want to drive:
		
	// Let's assume we want forward operation.
	// Start by calculating the minimum cost of the initial rotation for this case:
	if(((first_angle-theta >= 0)&&(first_angle-theta <= M_PI))||
	   ((first_angle-theta<0)&&(first_angle-theta<-M_PI)))
	{
	    // Positive rotation:
	    rot1_f = 1;
	}
	else
	{
	    // Negative rotation:
	    rot1_f = -1;
	}
	costf += min(fabs(first_angle-theta-2*M_PI),fabs(first_angle-theta));
		
		
	// Now calculate the minimum cost of the final rotation for this case:
	if(((ori_sent-first_angle >= 0)&&(ori_sent-first_angle <= M_PI))||
	   ((ori_sent-first_angle<0)&&(ori_sent-first_angle<-M_PI)))
	{
	    // Positive rotation:
	    rot2_f = 1;
	}
	else
	{
	    // Negative rotation:
	    rot2_f = -1;
	}
	costf += min(fabs(ori_sent-first_angle),fabs(fabs(ori_sent-first_angle)-2*M_PI));
		
	// Now, let's assume backwards operation.
	// Start by calculating the minimum cost of the initial rotation for this case:
	if(((first_angle-thback >= 0)&&(first_angle-thback <= M_PI))||
	   ((first_angle-thback<0)&&(first_angle-thback<-M_PI)))
	{
	    // Positive rotation:
	    rot1_b = 1;
	}
	else
	{
	    // Negative rotation:
	    rot1_b = -1;
	}
	costb += min(fabs(first_angle-thback),fabs(fabs(first_angle-thback)-2*M_PI));
		
	// Now calculate the minimum cost of the final rotation for this case:
	if(((tpback-first_angle >= 0)&&(tpback-first_angle <= M_PI))||
	   ((tpback-first_angle<0)&&(tpback-first_angle<-M_PI)))
	{
	    // Positive rotation:
	    rot2_b = 1;
	}
	else
	{
	    // Negative rotation:
	    rot2_b = -1;
	}
	costb += min(fabs(tpback-first_angle),fabs(fabs(tpback-first_angle)-2*M_PI));

	// Now lets figure out which costs less?
	if (costf <= costb)
	{	
	    // Then let's go forward:
	    rot1_flag = rot1_f;
	    rot2_flag = rot2_f;
	    dir_flag = 1;
	}
	else
	{
	    // Then let's go backwards:
	    rot1_flag = rot1_b;
	    rot2_flag = rot2_b;
	    dir_flag = -1;
	    // If we are going backward, we need to change first_angle by pi so that 
	    // the robot knows when to stop the initial rotation
	    first_angle += M_PI;
	    if(first_angle > 2.0*M_PI) first_angle -= 2.0*M_PI;
	}		
		
	// Now let's set the motor speeds such that we begin the first rotation correctly:
	right_desired = ((float) rot1_flag)*(speed);
	left_desired = -1.0*((float) rot1_flag)*(speed);
    }
    // Are we driving to the destination?
    else if(exec_state == 3)
    {
	// Let's let main know that it can stop calling this function now
	pose_flag = 0;	
	// Now let's set the motor speeds for traversing the distance:
	right_desired = ((float) dir_flag)*(speed);
	left_desired = ((float) dir_flag)*(speed);
    }
    else if(exec_state == 4)
    {
	// Let's let main know that it can stop calling this function now
	pose_flag = 0;	
	// We are going to the final orientation:
	right_desired = ((float) rot2_flag)*(speed);
	left_desired = -1.0*((float) rot2_flag)*(speed);		
    }
}


// This is the function that is called after a complete instruction has been received.  It decodes that instruction
// and then sets important global variables to the correct values.
void PoseUpdate(void)
{
    // The first thing that we do is clear the flags for receiving header bits and for receiving new instructions:
    data_flag = 0;
    header_flag = 0;

    // Now, let's disable both OC interrupts, both IC interrupts and the UART interrupt so that they are not interrupting while 
    //	we are sorting out the data that was received:
    INTEnable(INT_U2RX, 0);
    INTEnable(INT_U2TX, 0);
	
    // Now, we can begin sorting the data:
    char data;  // This is the very first entry in the in buffer (the header byte)
    unsigned char buffer[12];
		
    // Set the current byte of the RS232_In_Buffer to be equal to data:
    data = Command_String[0];

    // First, let's check to see if we are actually moving yet:
    if (movement_flag == 0 && data == 'm')
    {
    	short int j;
    	for(j=2;j<DATA_LENGTH-1;j++)
    	{
    	    if (Command_String[j] != 0)
	    {
		movement_flag = 0;
		mLED_2_Off();
		break;
	    }
	    else
	    {
		movement_flag = 1;
		mLED_2_On();
	    }
    	}
    }
    if (movement_flag == 0)
    {
	INTEnable(INT_U2RX, 1);
	INTEnable(INT_U2TX, 1);
	return;
    }
    
    // If the header is a 'p', we have received an instruction for where to go:
    if (data == 'p')
    {
	exec_state = 2;

	x_sent = InterpNumber(&Command_String[2]);
	y_sent = InterpNumber(&Command_String[5]);
	ori_sent = InterpNumber(&Command_String[8]);
		
	// Let's set pose_flag so that we know we need to call SetPose()
	pose_flag = 1;
    }
	
    // If the header byte is an 'l', we have received updated information about the 
    // robot's current location.
    else if(data == 'l')
    {
	
	x_pos = InterpNumber(&Command_String[2]);
	y_pos = InterpNumber(&Command_String[5]);
	theta = InterpNumber(&Command_String[8]);	
		
	// So, we have just received an updated position and orientation, we need to 
	// decide if it is worth it to fix the error.
		
	// If we are currently executing a point command, and we are currently translating,
	// it is very easy to make an adjustment:
	if (exec_state == 3) 
	{
	    pose_flag = 1;
	    exec_state = 2;
	}
		
	// If we are done executing a point command and basically just sitting waiting for 
	// the next, and our error is bad enough, we can go ahead and try to fix it
	if (exec_state == 0)
	{
	    if((powf((x_pos-x_sent),2.0)+powf((y_pos-y_sent),2.0) > 3.0) || (fabs(theta-ori_sent) > 1.571))
	    {
		pose_flag = 1;
		exec_state = 2;
	    }
	}
    }
    else if (data == 'r')
    {
	exec_state = 1;
	left_desired = 0.0;
	right_desired = 0.0;
	top_desired = 0.0;
		
	x_pos = 0.0;
	y_pos = 0.0;
	theta = 0.0;
	height = 0.0;
	while(BusyUART2());
	putsUART2("Coordinate System Reset\r\n");
    }
    // If we receive an h as the header char, then that means that we will simply control the motor speeds
    else if (data == 'h')
    {
	exec_state = 1;
	left_desired = InterpNumber(&Command_String[2]);
	right_desired = InterpNumber(&Command_String[5]);	
	top_desired = InterpNumber(&Command_String[8]);
	/* mLED_3_Toggle(); */
	
	// We just received commands for explicitly controlling the wheel speeds, let's force the pose control
	// to stop executing:
	height_flag = 0;
	pose_flag = 0;
    }
    else if (data == 's')
    {
	speed = InterpNumber(&Command_String[2]);
	while(BusyUART2());
	putsUART2("Changed Default Speed\r\n");
    }
    else if (data == 'q')
    {
	exec_state = 1;
	left_desired = 0.0;
	right_desired = 0.0;
	top_desired = 0.0;
	movement_flag = 0;
    }
    else if (data == 't')
    {
    	char top_state;
    	// Get what mode we are in:
    	top_state = Command_String[2];
 	top_desired = InterpNumber(&Command_String[4]);
		
    	if(top_state == '0')
    	{
    	    // If this is a zero, we are doing height control.

	    // Get the height:
	    height_sent = InterpNumber(&Command_String[9]);

	    // Now, do we need to move up or down?
    	    if(height_sent <= height) top_desired = -top_desired;
    	    // Let's set the position control flag:
    	    height_flag = 1;
    	}
    	else if(top_state == '1')
    	{
    	    // If this is a one, we are just controlling speed:
       	    height_flag = 0;
    	}
    }
    else if (data == 'w')
    {
    	// This is a request for the robot's current pose, so let's
    	// send it out:
    	DisableWDT();
    	// Make string to send out:
    	MakeString(buffer,'w',x_pos,y_pos,theta,4);
    	// Add checksum and send to master node:
    	CreateAndSendArray(0, buffer);
	ClearEventWDT();
    	EnableWDT();
	total_replies++;
    }
    else if (data == 'e')
    {
    	// This is a request for the robot's current motor speeds,
    	// so let's send it out:
    	DisableWDT();
    	// Make string to send out:
    	MakeString(buffer,'e',left_speed,right_speed,top_speed,3);
    	// Add checksum and send to master node:
    	CreateAndSendArray(0, buffer);
    	ClearEventWDT();
    	EnableWDT();
    }
	
    // Now, let's re-enable all interrupts:
    INTEnable(INT_U2RX, 1);
    INTEnable(INT_U2TX, 1);
}

void RuntimeOperation(void)
{
    // If the buffer has just filled, and we know we received a good
    // set of data (because it began with a header bit and was full
    // length chars and the checksum was correct), let's call our
    // functions that interpret and execute the received data
    if(data_flag == 1) PoseUpdate();
	
    // If we are currently controlling pose, lets call that function
    if(pose_flag == 1) SetPose(x_sent,y_sent,ori_sent);
}

// This function is for determining the minium of two numbers:
float	min(float a, float b)
{
    return (a < b ? a : b);
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

/* This function is for interpreting the number received over UART2 */
/* Right now, three characters make up a full number, the least */
/* significant bits of these chars concantenated form a divisor and
 * the rest are stuck together to form a 21 bit signed integer.  The
 * divisor then converts this to a float. */
float InterpNumber(const unsigned char *data)
{
    unsigned int num1 = 0;
    int num2 = 0;
    float numf = 0.0;
    short int c1 = 0;
    short unsigned int c2, c3;
    short unsigned int divisor = 1;

    // First, let's get the numeric values of each char:
    c1 = (short int) *(data);
    c2 = (short unsigned int) *(data+1);
    c3 = (short unsigned int) *(data+2);
    divisor = (((short unsigned int) *(data+2)) & 0x07);
    num1 = ((((c1<<16)&0xFF0000)+((c2<<8)&0x00FF00)+(c3)))<<11;
    num2 = ((int) (num1))>>14;
    numf = ((float) num2)/(powf((float) 10.0,(float) divisor));

    return numf;
}

int Data_Checker(unsigned char* buff)
{
    short int j = 0;
    short int k = 0;
    short int i = 0;
    unsigned char temp[DATA_LENGTH];

    for (k=1; k<DATA_LENGTH; k++)
    {
	for (j=0; j<sizeof(header_list); j++)
	{
	    if(buff[k] == header_list[j])
	    {
		// Let's set DATA_LENGTH to the correct value:
		DATA_LENGTH = FULL_PACKET_SIZE;
		if (buff[k] == 'w' || buff[k] == 'e')
		    DATA_LENGTH = SMALL_PACKET_SIZE;
		if (k == DATA_LENGTH-1)
		{
		    i = DATA_LENGTH-k;
		    memcpy(temp, &buff[k], i);
		    memset(buff,'z',DATA_LENGTH);
		    memcpy(buff, temp, i);
		    return i;
		}
		else if (buff[k+1] == ID || buff[k+1] == BROADCAST)
		{
		    i = DATA_LENGTH-k;
		    memcpy(temp, &buff[k], i);
		    memset(buff,'z',DATA_LENGTH);
		    memcpy(buff, temp, i);
		    return i;
		}   
	    }
	}
    }
    return i;
}

void Reset_Robot(void)
{
    // First, let's disable all interrupts:
    INTDisableInterrupts();
     
    // Now, shut down all PWM and endcoder pins:
    CloseOC1();
    CloseOC2();
    CloseOC3();
    
    // Now, we can restart robot:
    SoftReset();
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
}

void BuildNumber(unsigned char *destination, float value, short int divisor)
{
    int valint = 0;
    int i = 0;
    short int i1;
    short unsigned int i2, i3;
    // First thing is to move the decimal point of the integer to the
    // right a "divisor" number of times:
    for(i = 0; i < divisor; i++) value = value*10.0;
    valint = (int) value;
    // Now build the three chars:
    i1 = ((valint<<3) & 0xFF0000)>>16;
    i2 = ((valint<<3) & 0x00FF00)>>8;
    i3 = ((valint<<3) & 0x0000FF);
    i3 = (((i3)&0xF0)) + ((i3&0x0F)|divisor);

    // Now, place the chars in the array:
    *(destination) = i1;
    *(destination+1) = i2;
    *(destination+2) = i3;
        
    return;
}

void MakeString(unsigned char *dest, char type, float fval,
		float sval, float tval, int div)
{
    *dest = type;
    BuildNumber((dest+1), fval, div);
    BuildNumber((dest+4), sval, div);
    BuildNumber((dest+7), tval, div);
}

void CreateAndSendArray(unsigned short id, unsigned char *DataString)
{
    unsigned char packet [20];
    int i = 0;
    unsigned int checksum = 0;

    // Initialize packet:
    memset(packet,0,sizeof(packet));

    // Fill packet:
    packet[0] = DataString[0];
    sprintf(&packet[1],"%d",id);
        for(i = 2; i < FULL_PACKET_SIZE-1; i++)
	packet[i] = DataString[i-1];

    // Now, let's calculate a checksum:
    checksum = 0;
    for(i = 0; i < FULL_PACKET_SIZE-1; i++)
	checksum += packet[i];
    checksum = 0xFF-(checksum & 0xFF);
    packet[FULL_PACKET_SIZE-1] = checksum;

    // Now, we can send the data out:
    SendDataBuffer(packet, FULL_PACKET_SIZE);
}

void delay(void)
{
    long unsigned int num_calls = SYS_FREQ/8;
    while(num_calls) num_calls--;
}
