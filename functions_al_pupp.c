/*************************************************************
Jarvis Schultz and Marcus Hammond
8-25-2010

This code contains all of the main functions for controlling the
robotic puppeteers as part of the "puppet people" project.

EDIT: 4-15-2011

Just made significant modifications to add a lot of safety features
related to dangerous situations that may arise if wireless
communication fails.

EDIT: 5-25-2011

More significant changes so that the new aluminum puppeteers can
control four motors.

EDIT: 9-5-2011

Added a new mode of communication where the computer can send a 'd' to
control the robot's translational and rotational velocities. As well
as the winches translational velocity.

EDIT: 10-2-2011

Adding more possible types of communication.  Adding a longer packet
size with 4 floats so that the user can send commands to individaully
control each motor's speed as well as a new pose control scheme where
the robot uses its own odometry as a sensor to run a kinematic
controller for trajectory following.

**************************************************************/

/** Includes ************************************************/
#include "HardwareProfile.h"
#include <plib.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#ifndef _MOTORS
	#include "prototypes_al_pupp.h"
	#define _MOTORS
#endif

/** Defines ***************************************************/
#define TRUE            1
#define FALSE		0
#define DIRECTION_PIN_L	 LATDbits.LATD3
#define DIRECTION_PIN_R	 LATDbits.LATD5
#define DIRECTION_PIN_TL LATCbits.LATC13
#define DIRECTION_PIN_TR LATDbits.LATD10
#define FORWARD		0
#define REVERSE		1
#define T2options	T2_ON | T2_PS_1_2 | T2_SOURCE_INT  	
#define T3options 	T3_ON | T3_PS_1_1 | T3_SOURCE_INT  	
#define T4options 	T4_ON | T4_PS_1_256 | T4_SOURCE_INT  	
#define BAUDRATE	(111111)				
#define CHANNEL_A_L	PORTDbits.RD12
#define CHANNEL_B_L	PORTDbits.RD13
#define CHANNEL_A_R	PORTDbits.RD9
#define CHANNEL_B_R	PORTDbits.RD6
#define CHANNEL_A_TL	PORTDbits.RD11
#define CHANNEL_B_TL	PORTCbits.RC14
#define CHANNEL_A_TR	PORTDbits.RD8
#define CHANNEL_B_TR	PORTDbits.RD7
#define MAX_RESOLUTION	3999  // Proportional to period of PWM (represents the
			      // maximum number that we can use for PWM function)
#define ERROR_DEADBAND	0.05
#define LARGE_PACKET_SIZE	18
#define PACKET_SIZE		12    
#define SMALL_PACKET_SIZE	3 
#define frequency  1500	       // Frequency we check kinematics at
#define controller_freq  300    // Frequency to run kinematic controller at
// Aluminum puppeteer:
/* #define GEARRATIO  (19.0*(42.0/46.0))    // The gear ratio of the drivetrain */
/* #define TOPGEARRATIO (19.0*(10.0/14.0))  // Winch gear ratio */
// Wooden puppeteer:
#define GEARRATIO	(19.0*(3.0/4.0))
#define TOPGEARRATIO	(19.0)
#define CPR  100.0	               // counts per revolution of an encoder
#define dtbase (1.0/frequency)         // The period of CheckKinematics calls
#define convert (M_PI/(CPR*dtbase*GEARRATIO)) 
#define converttop (M_PI/(CPR*dtbase*TOPGEARRATIO))      
#define ticktime (2.0/(80000000.0))    // For calculating times since timer
				       // ISR's were initially called
#define MAX_BAD_DATA_TOTAL (10)	       
#define MAX_BAD_DATA	(5)
#define MAX_BAD_COUNTER  (200)
#define timeout_frequency (10)
#define SYS_FREQ	(80000000L)


/** Global Variables **************************************************/
static short int DATA_LENGTH = 12;

// Current motor speeds in rad/ second:
static float left_speed = 0.0;		
static float right_speed = 0.0;		
static float top_left_speed = 0.0;	
static float top_right_speed = 0.0;

// Total motor encoder counts:
static long long left_steps = 0;	
static long long right_steps = 0;	
static long long top_left_steps = 0;	
static long long top_right_steps = 0;

// Old encoder counts for determining velocities:
static long long left_steps_last = 0;	
static long long right_steps_last = 0;  
static long long top_left_steps_last = 0;
static long long top_right_steps_last = 0;

// Robot's pose estimate:
static float x_pos = 0.0;		
static float y_pos = 0.0;		
static float theta = 0.0;  // w.r.t positive x-axis (clamped between 0 and 2pi)
static float height_left = 0.0; // increases as string length decreases i.e.
static float height_right = 0.0; // up is positive

// Robot constants (in meters):
// Aluminum Puppteer:
/* static float DPULLEY = 0.034924999999999998;  */
/* static float WIDTH = 0.148/2.0; */
/* static float DWHEEL = 0.07619999999999; */
// Wooden Puppeteer:
static float DPULLEY = 0.019049999;
static float WIDTH = 0.145/2.0;
static float DWHEEL = 0.07619999999999;
static float speed = 10.0;	// default driving speed

// Communication buffers and miscellaneous:
static unsigned char RS232_In_Buffer[LARGE_PACKET_SIZE] = "zzzzzzzzzzzzzzzzzz";
static unsigned char Command_String[LARGE_PACKET_SIZE] = "zzzzzzzzzzzzzzzzzz";
static int i = 0;	        // position in RS232_In_Buffer
static int data_flag = 0;  	
static int pose_flag = 0;  	
static int header_flag = 0;
extern char ID;
static char BROADCAST = '9';

// Desired pose variables:
static float x_sent = 0.0;
static float y_sent = 0.0;
static float ori_sent = 0.0;
static float height_left_sent = 0.0;
static float height_right_sent = 0.0;

// Miscellaneous pose control variables:
static float first_angle = 0.0; 	// angle of the vector from the robots
					// current position towards its goal position
static int exec_state = 0;	   // Mode of operation:
					// 0) Just sitting there
					// 1) Wheel Speed Control
				        // 2) Initial rotation towards destination
					// 3) Driving towards destination
					// 4) Rotating towards final orientation
					// 5) Running kinematic controller
// Speed control variables:
static float left_desired;	
static float right_desired;	
static float top_left_desired;	
static float top_right_desired;	

// Controller gains:
static float kp = 250;		// Gain on the proportional error term
static float ki = 50;		// Gain on the integral error term
static float kd = 0.5;		// Gain on the derivative error term

// Add a bunch of variables for communication safety:
static unsigned char header_list[]={'p','l','r','h','s','q','m','w',
				    'e','d','k','t','n','b','a'};
/******************************************************************************/
// Note that the header characters mean the following:
//	'p' = Drive to a desired pose (R)
//	'l' = Re-define the robot's position and orientation (R)
//	'r' = Stop driving, and re-set the global configuration to the robot's
//		current pose (R)
//	'h' = Motor speed command (winches run at same speed) (R)
//	's' = Change default speed for pose control (R)
//	'q' = Stop the robot, and reset movement_flag (R)
//	'm' = Start command; must receive this before driving can begin (R)
//	'w' = Current pose request (S)
//	'e' = Current motor speeds request (S)
//	'd' = Translational and rotational velocity command
//		(both winches the same)(R)
//	'k' = Run kinematic controller for following trajectories (R)
//	't' = Run trajectory kinematic controller and height controller (L)
//	'n' = All motor speed commands (L)
//	'b' = Reset current winch height estimates (R)
//	'a' = Set values for all configuration variables (L)
//	'i' = Translational and rotational velocity command (L)
/******************************************************************************/
static short int bad_data_total = 0;
static short int bad_data = 0;
static short int movement_flag = 0;
static short int midstring_flag = 0;
static short int timeout_flag = 0;
static short int bad_data_counter = 0;

//  Kinematic controller variables:
static float dir_sign = 1.0;
static float tvec[2] = {0.0, 0.0};
static float xvec[3] = {0.0, 0.0, 0.0};
static float yvec[3] = {0.0, 0.0, 0.0};
static float thvec[3] = {0.0, 0.0, 0.0};
static float lftvec[3] = {0.0, 0.0, 0.0};
static float rhtvec[3] = {0.0, 0.0, 0.0};
static float k1 = 0.0;
static float k2 = 0.0;
static float t_sent = 0.0;
static float vd = 0.0;
static float wd = 0.0;
static unsigned int controller_flag = 0;
static unsigned int pause_controller_flag = 0;
static float dt = 0.0;
static float dt2 = 0.0;
static float running_dt = 0.0;
static unsigned short waypoint_index = 0;
static unsigned int winch_controller_flag = 0;
static float xd = 0;
static float xdd = 0;
static float yd = 0;
static float ydd = 0;
/* static float vlff[3] = {0.0, 0.0, 0.0}; */
/* static float vrff[3] = {0.0, 0.0, 0.0}; */

/** Interrupt Handler Functions:**************************************************************************************************/
// UART 2 interrupt handler
// it is set at priority level 2
void __ISR(_UART2_VECTOR, ipl6) IntUart2Handler(void)
{
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
		    DATA_LENGTH = PACKET_SIZE;
		    // If a request, it is a short packet!
		    if (temp == 'w' || temp == 'e')
			DATA_LENGTH = SMALL_PACKET_SIZE;
		    // Is this a long packet?
		    if (temp == 't' || temp == 'n' || temp == 'a')
			DATA_LENGTH = LARGE_PACKET_SIZE;
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
		    new_pos = data_checker(RS232_In_Buffer);

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
	    reset_robot();
	}

	if (bad_data_counter >= MAX_BAD_COUNTER)
	{
	    reset_robot();
	}
    }

    // Is this interrupt from sending data?
    if ( mU2TXGetIntFlag() )
    {
	mU2TXClearIntFlag();
    }
}

// This is the ISR that gets called when we detect a rising or falling edge on CHANNEL_A_R
void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl5) CheckPosition_r()
{
    static int tempA, tempB;
    tempA = CHANNEL_A_R;
    tempB = CHANNEL_B_R;
		
    // Let's clear the interrupt flag:
    INTClearFlag(INT_IC2);

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


// This is the ISR that gets called when we detect a rising or falling edge on CHANNEL_A_TL
void __ISR(_INPUT_CAPTURE_4_VECTOR, ipl5) CheckPosition_t_l()
{
    static int tempA, tempB;
    tempA = CHANNEL_A_TL;
    tempB = CHANNEL_B_TL;
	
    // Let's clear the interrupt flag:
    INTClearFlag(INT_IC4);
    // Now we can perform logic to determine which direction we are going and we can increment counter
    if(tempA)
    {
	// So, we know that we just detected a rising edge on channel A.  Let's determine whether
	// channel b is high or low to determine our direction:
	if(tempB) top_left_steps++;
	else top_left_steps--;
    }
    else if(!tempA)
    {
	if(tempB) top_left_steps--;
	else top_left_steps++;
    }
}


// This is the ISR that gets called when we detect a rising or falling edge on CHANNEL_A_TR
void __ISR(_INPUT_CAPTURE_1_VECTOR, ipl5) CheckPosition_t_r()
{
    static int tempA, tempB;
    tempA = CHANNEL_A_TR;
    tempB = CHANNEL_B_TR;
	
    // Let's clear the interrupt flag:
    INTClearFlag(INT_IC1);
    // Now we can perform logic to determine which direction we are going and we can increment counter
    if(tempA)
    {
	// So, we know that we just detected a rising edge on channel A.  Let's determine whether
	// channel b is high or low to determine our direction:
	if(tempB) top_right_steps--;
	else top_right_steps++;
    }
    else if(!tempA)
    {
	if(tempB) top_right_steps++;
	else top_right_steps--;
    }
}


void __ISR(_TIMER_4_VECTOR, ipl7) Data_Timeout()
{
    float max_speed_err = 10.0;
    float max_dead_err = 0.1;
    // If any motor is running, and timeout_flag is high, we are going to reset:
    if (timeout_flag == 1)
    {
	if((fabsf(left_speed) >= max_speed_err) ||
	   (fabsf(right_speed) >= max_speed_err) ||
	   (fabsf(top_left_speed) >= max_speed_err) ||
	   (fabsf(top_right_speed) >= max_speed_err) ||
	   (fabsf(left_desired) >= max_dead_err) ||
	   (fabsf(right_desired) >= max_dead_err) ||
	   (fabsf(top_left_desired) >= max_dead_err) ||
	   (fabsf(top_right_desired) >= max_dead_err))
	{
	    // Reset!
	    reset_robot();
	}
    }
    timeout_flag = 1;
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
    float Vtl = 0.0;
    float Vtr = 0.0;
    float omega = 0.0;
    float R = 0.0;
    /* float dt = 0.0; */
    int top_left_current, top_right_current, left_current, right_current;
    static unsigned int call_count = 0;
    const unsigned int num = floor(frequency/ controller_freq);
    
    left_current = left_steps;
    right_current = right_steps;
    top_left_current = top_left_steps;
    top_right_current = top_right_steps;    
            
    // Let's first calculate the current angular velocity of each wheel:
    left_speed = convert*((float) (left_current-left_steps_last));
    right_speed = convert*((float) (right_current-right_steps_last));
    top_left_speed = converttop*((float) (top_left_current-top_left_steps_last));
    top_right_speed = converttop*((float) (top_right_current-top_right_steps_last));
	
    // Now let's store the current counts for the next CheckKinematics call:
    left_steps_last = left_current;
    right_steps_last = right_current;
    top_left_steps_last = top_left_current;
    top_right_steps_last = top_right_current; 
	
    // Now, let's calculate the error between the actual and desired velocities 
    float left_error = left_desired-left_speed;
    float right_error = right_desired-right_speed;
    float top_left_error = top_left_desired-top_left_speed;
    float top_right_error = top_right_desired-top_right_speed;
    
    // Now let's calculate how long it has been since this function has been called.
    /* dt = ticktime*(((float) ReadTimer2())+1.0)+dtbase; */

    // Now, we set the PWM Values:
    if(fabsf(left_error) > ERROR_DEADBAND) SetSpeedLeft(left_error, dtbase);
    if(fabsf(right_error) > ERROR_DEADBAND) SetSpeedRight(right_error, dtbase);
    if(fabsf(top_left_error) > ERROR_DEADBAND) SetSpeedTopLeft(top_left_error, dtbase);
    if(fabsf(top_right_error) > ERROR_DEADBAND) SetSpeedTopRight(top_right_error, dtbase);

    // Now let's get the wheels speeds and convert them into translational velocities
    Vr = (DWHEEL/2.0)*(right_speed);
    Vl = (DWHEEL/2.0)*(left_speed);
    Vtl = (DPULLEY/2.0)*(top_left_speed);
    Vtr = (DPULLEY/2.0)*(top_right_speed);
		
    // Let's calculate the distance to the instantaneous center of rotation and the angular vel.
    R = WIDTH*((Vl+Vr)/(Vr-Vl));
    omega = (Vr-Vl)/(2.0*WIDTH);
		
    // Now we do the forward kinematics
    if (fabsf(R) > 10000.0) // This would imply that the robot is essentially going straight
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
    theta = clamp_angle(theta);
	
    // Now, let's update the height of our string
    height_left += Vtl*dtbase;
    height_right += Vtr*dtbase;
	
    // Are we just controlling wheel speeds?
    switch (exec_state)
    {
    case 1:
	break;
    case 2:
	// Are we performing an initial rotation to begin heading towards our destination?
	// Now, let's determine if we have reached the desired orientation yet
	if(fabsf(theta-first_angle) <= 0.01 || fabsf(fabsf(theta-first_angle)-2.0*M_PI) <= 0.01)
	{
	    theta = first_angle;
	    exec_state = 3;
	    pose_flag = 1;
	    right_desired = 0.0;
	    left_desired = 0.0;
	}
	break;
    case 3:
	// Now, are we there yet?
	if(fabsf(x_sent-x_pos) < 0.05 && fabsf(y_sent-y_pos) < 0.05)
	{
	    x_pos = x_sent;
	    y_pos = y_sent;
	    right_desired = 0.0;
	    left_desired = 0.0;
	    exec_state = 4;
	    pose_flag = 1;
	}
	break;
    case 4:
	// Now, let's determine if we have reached the final orientation yet
	if(fabsf(theta-ori_sent) <= 0.01 || fabsf(fabsf(theta-ori_sent)-2.0*M_PI) <= 0.01)
	{
	    theta = ori_sent;
	    // Since we have, we don't need to keep updating the kinematics
	    exec_state = 0;
	    pose_flag = 0;
	    // Also, let's stop the motors:
	    right_desired = 0.0;
	    left_desired = 0.0;
	}
	break;
    case 5:
	running_dt += ticktime*(((float) ReadTimer2())+1.0)+dtbase;
	// this means we are running the kinematic controller
	if (call_count%num == 0 && pause_controller_flag != 1)
	    controller_flag = 1;
	break;
    }
    call_count++;
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
// steps that the top left motor has taken:
int GetStepsTopLeft(void)
{
    return top_left_steps;
}

// Following function returns the number of 
// steps that the top left motor has taken:
int GetStepsTopRight(void)
{
    return top_right_steps;
}

// The following function returns the current speed 
// of the left motor
float GetSpeedLeft(void)
{
    return left_speed;
}

// The following function returns the current speed 
// of the right motor
float GetSpeedRight(void)
{
    return right_speed;
}

// The following function returns the current speed 
// of the top left motor
float GetSpeedTopLeft(void)
{
    return top_left_speed;
}

// The following function returns the current speed 
// of the top right motor
float GetSpeedTopRight(void)
{
    return top_right_speed;
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
// manually changing the number of encoder steps for the top left motor:
void SetStepsTopLeft(int set_steps)
{
    INTEnable(INT_OC2, 0);
    top_left_steps = set_steps;
    INTEnable(INT_OC2, 1);
}

// Next function allows to user to re-set a "home" position by
// manually changing the number of encoder steps for the top right motor:
void SetStepsTopRight(int set_steps)
{
    INTEnable(INT_OC1, 0);
    top_right_steps = set_steps;
    INTEnable(INT_OC1, 1);
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
    PWMVal = (unsigned int) (fabsf(total_error));
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
    PWMVal = (unsigned int) (fabsf(total_error));
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

// This function allows the user to specify the desired top left motor speed:
void SetSpeedTopLeft(float error, float dt)  // motor speed in rad/s
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
    PWMVal = (unsigned int) (fabsf(total_error));
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
	DIRECTION_PIN_TL = FORWARD;
	SetDCOC2PWM(PWMVal);
    }
    else
    {
	DIRECTION_PIN_TL = REVERSE;
	SetDCOC2PWM(PWMVal);
    }
}

void SetSpeedTopRight(float error, float dt)  // motor speed in rad/s
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
    PWMVal = (unsigned int) (fabsf(total_error));
    // Set stored values:
    last_error = error;
    last_d_error = d_error;
    
    if(PWMVal > MAX_RESOLUTION) 
    {
	PWMVal = MAX_RESOLUTION;
	sum_error -= error;
    }
    
    if(total_error < 0)
    {
	DIRECTION_PIN_TR = FORWARD;
	SetDCOC1PWM(PWMVal);
    }
    else
    {
	DIRECTION_PIN_TR = REVERSE;
	SetDCOC1PWM(PWMVal);
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
    TRISDbits.TRISD0 = 0;
	
    // Direction Pins to outputs:
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD5 = 0;
    TRISCbits.TRISC13 = 0;
    TRISDbits.TRISD10 = 0;
    
    // init OC5 module, on pin D4
    OpenOC5( OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
	
    // init OC3 module, on pin D2
    OpenOC3( OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);

    // init OC2 module, on pin D1
    OpenOC2( OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);

    // init OC1 module, on pin D1
    OpenOC1( OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
	
    // init Timer3 
    OpenTimer3(T3options , MAX_RESOLUTION);
}


void InitEncoder(void)
{	
    // Channel A inputs:
    TRISDbits.TRISD12 = 1;
    TRISDbits.TRISD9 = 1;
    TRISDbits.TRISD11 = 1;
    TRISDbits.TRISD8 = 1;    
	
    // Channel B inputs:
    TRISDbits.TRISD13 = 1;
    TRISDbits.TRISD6 = 1;
    TRISCbits.TRISC14 = 1;
    TRISDbits.TRISD7 = 1;
    
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
	
    // Now we can configure the CHANNEL_A_TL pin (D11) to be an input capture pin:
    // - Capture Every edge
    // - Enable capture interrupts
    // - Use Timer 3 source
    // - Capture rising edge first
    OpenCapture4( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC |IC_FEDGE_RISE | IC_ON );
    // Configure the interrupt options
    ConfigIntCapture4(IC_INT_ON | IC_INT_PRIOR_5);

    // Now we can configure the CHANNEL_A_TR pin (D8) to be an input capture pin:
    // - Capture Every edge
    // - Enable capture interrupts
    // - Use Timer 3 source
    // - Capture rising edge first
    OpenCapture1( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC |IC_FEDGE_RISE | IC_ON );
    // Configure the interrupt options
    ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_5);

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
#define config1		UART_EN | UART_IDLE_CON | UART_RX_TX | UART_DIS_WAKE | \
	UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_NO_PAR_8BIT | UART_1STOPBIT | \
	UART_IRDA_DIS | UART_DIS_BCLK_CTS_RTS| UART_NORMAL_RX | UART_BRGH_SIXTEEN
	
    // define setup Configuration 2 for OpenUARTx
    // IrDA encoded UxTX idle state is '0'
    // Enable UxRX pin
    // Enable UxTX pin
    // Interrupt on transfer of every character to TSR 
    // Interrupt on every char received
    // Disable 9-bit address detect
    // Rx Buffer Over run status bit clear
#define config2		UART_TX_PIN_LOW | UART_RX_ENABLE | UART_TX_ENABLE | \
	UART_INT_TX | UART_INT_RX_CHAR | UART_ADR_DETECT_DIS | UART_RX_OVERRUN_CLEAR	

    // Open UART2 with config1 and config2
    OpenUART2( config1, config2, pbClk/16/BAUDRATE-1);
		
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
	if(fabsf(xdest-x_pos) < 0.000001 && fabsf(ydest-y_pos) < 0.000001)
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
	costf += find_min(fabsf(first_angle-theta-2*M_PI),fabsf(first_angle-theta));
		
		
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
	costf += find_min(fabsf(ori_sent-first_angle),fabsf(fabsf(ori_sent-first_angle)-2*M_PI));
		
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
	costb += find_min(fabsf(first_angle-thback),fabsf(fabsf(first_angle-thback)-2*M_PI));
		
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
	costb += find_min(fabsf(tpback-first_angle),fabsf(fabsf(tpback-first_angle)-2*M_PI));

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
void interp_command(void)
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
		break;
	    }
	    else
	    {
		movement_flag = 1;
		mLED_2_Toggle();
	    }
    	}
    }
    if (movement_flag == 0)
    {
	INTEnable(INT_U2RX, 1);
	INTEnable(INT_U2TX, 1);
	return;
    }


    switch (data)
    {
	// If the header is a 'p', we have received an instruction for where to go:
    case 'p':
	exec_state = 2;

	x_sent = interp_number(&Command_String[2]);
	y_sent = interp_number(&Command_String[5]);
	ori_sent = interp_number(&Command_String[8]);
		
	// Let's set pose_flag so that we know we need to call SetPose()
	pose_flag = 1;
	break;

    case 'l':
	// If the header byte is an 'l', we have received updated information about the 
	// robot's current location.
	x_pos = interp_number(&Command_String[2]);
	y_pos = interp_number(&Command_String[5]);
	theta = interp_number(&Command_String[8]);	
	break;

    case 'r':
	exec_state = 1;
	stop_all_motors();
		
	x_pos = 0.0;
	y_pos = 0.0;
	theta = 0.0;
        height_left = 0.0;
	height_right = 0.0;
	while(BusyUART2());
	putsUART2("Coordinate System Reset\r\n");
	break;
	
    case 'h':
	// If we receive an h as the header char, then that means that we will
	// simply control the motor speeds
	exec_state = 1;
	left_desired = interp_number(&Command_String[2]);
	right_desired = interp_number(&Command_String[5]);	
	top_left_desired = interp_number(&Command_String[8]);
	top_right_desired = top_left_desired;
	
	// We just received commands for explicitly controlling the wheel speeds,
	// let's force the pose control to stop executing:
	pose_flag = 0;
	break;

    case 'd':
	exec_state = 1;
	float v_robot = interp_number(&Command_String[2]);
	float w_robot = interp_number(&Command_String[5]);
	float v_winch = interp_number(&Command_String[8]);

	//  Now we need to convert these into angular velocities for the motors:
	left_desired = 2.0*(v_robot-w_robot*WIDTH)/DWHEEL;
	right_desired = 2.0*(v_robot+w_robot*WIDTH)/DWHEEL;
	top_left_desired = 2.0*v_winch/DPULLEY;
	top_right_desired = top_left_desired;

	pose_flag = 0;
	break;

    case 's':
	speed = interp_number(&Command_String[2]);
	while(BusyUART2());
	putsUART2("Changed Default Speed\r\n");
	break;

    case 'q':
	exec_state = 0;
	stop_all_motors();
	movement_flag = 0;
	mLED_2_Toggle();
	break;

    case 'w':
	// This is a request for the robot's current pose, so let's
    	// send it out:
    	DisableWDT();
    	// Make string to send out:
    	make_string(buffer,'w',x_pos,y_pos,theta,4);
    	// Add checksum and send to master node:
    	create_send_array(0, buffer);
	ClearEventWDT();
    	EnableWDT();
	break;

    case 'e':
	// This is a request for the robot's current motor speeds,
    	// so let's send it out:
    	DisableWDT();
    	// Make string to send out:
    	make_string(buffer,'e',left_speed,right_speed,top_left_speed,3);
    	// Add checksum and send to master node:
    	create_send_array(0, buffer);
    	ClearEventWDT();
    	EnableWDT();
	break;

    case 'k':
	// we are going to run the kinematic controller:
	DisableWDT();
	setup_controller();
	pose_flag = 0;
	winch_controller_flag = 0;
	ClearEventWDT();
	EnableWDT();
	break;

    case 't':
	// Kinematic controller for trajectory and winches
	DisableWDT();
	setup_controller();
	setup_winch_controller();
	pose_flag = 0;
	winch_controller_flag = 1;
	ClearEventWDT();
	EnableWDT();
	break;

    case 'n':
	// control all motor speeds
	exec_state = 1;
	left_desired = interp_number(&Command_String[2]);
	right_desired = interp_number(&Command_String[5]);	
	top_left_desired = interp_number(&Command_String[8]);
	top_right_desired = interp_number(&Command_String[11]);
	pose_flag = 0;
	break;

    case 'a':
	// set all configuration variables to a specified value
	x_pos = interp_number(&Command_String[2]);
	y_pos = interp_number(&Command_String[5]);
	theta = interp_number(&Command_String[8]);
	height_left = interp_number(&Command_String[11]);
	height_right = interp_number(&Command_String[14]);
	break;

    case 'b':
	// just reset the height configuration variables to a
	// specified value
	height_left = interp_number(&Command_String[2]);
	height_right = interp_number(&Command_String[5]);
	break;

    case 'i':
	exec_state = 1;
	float v_robot_1 = interp_number(&Command_String[2]);
	float w_robot_1 = interp_number(&Command_String[5]);
	float v_winch_1 = interp_number(&Command_String[8]);
	float v_winch_2 = interp_number(&Command_String[11]);

	//  Now we need to convert these into angular velocities for the motors:
	left_desired = 2.0*(v_robot_1-w_robot_1*WIDTH)/DWHEEL;
	right_desired = 2.0*(v_robot_1+w_robot_1*WIDTH)/DWHEEL;
	top_left_desired = 2.0*v_winch_1/DPULLEY;
	top_right_desired = 2.0*v_winch_2/DPULLEY;

	pose_flag = 0;
	break;
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
    if(data_flag == 1) interp_command();
	
    // If we are currently controlling pose, lets call that function
    if(pose_flag == 1) SetPose(x_sent,y_sent,ori_sent);

    // Run kinematic controller?
    if(controller_flag == 1)  run_controller();
}

// This function is for determining the minium of two numbers:
float find_min(float a, float b)
{
    return ((a < b) ? a : b);
}

// This function is for finding the minimum angle between two angles:
float find_min_angle(float a, float b)
{
    float comps[3];
    float dth = 0.0;
    comps[0] = a-b;
    comps[1] = comps[0]+2.0*M_PI;
    comps[2] = comps[0]-2.0*M_PI;

    if ( fabsf(comps[0]) < fabsf(comps[1]))
    {
	if (fabsf(comps[0]) < fabsf(comps[2]))
	    dth = comps[0];
	else
	    dth = comps[2];
    }
    else
    {
	if (fabsf(comps[1]) < fabsf(comps[2]))
	    dth = comps[1];
	else
	    dth = comps[2];
    }
    return dth;
}

float sign(float x)
{
    return (x < 0) ? -1.0 : ((float)(x > 0));
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
float interp_number(const unsigned char *data)
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

int data_checker(unsigned char* buff)
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
		DATA_LENGTH = PACKET_SIZE;
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

void reset_robot(void)
{
    // First, let's disable all interrupts:
    INTDisableInterrupts();

    // disable any running controllers:
    pose_flag = 0;
    controller_flag = 0;
     
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

void build_number(unsigned char *destination, float value, short int divisor)
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

void make_string(unsigned char *dest, char type, float fval,
		float sval, float tval, int div)
{
    *dest = type;
    build_number((dest+1), fval, div);
    build_number((dest+4), sval, div);
    build_number((dest+7), tval, div);
}

void create_send_array(unsigned short id, unsigned char *DataString)
{
    unsigned char packet [20];
    int i = 0;
    unsigned int checksum = 0;

    // Initialize packet:
    memset(packet,0,sizeof(packet));

    // Fill packet:
    packet[0] = DataString[0];
    sprintf((char*) &packet[1],"%d",id);
    for(i = 2; i < PACKET_SIZE-1; i++)
	packet[i] = DataString[i-1];

    // Now, let's calculate a checksum:
    checksum = 0;
    for(i = 0; i < PACKET_SIZE-1; i++)
	checksum += packet[i];
    checksum = 0xFF-(checksum & 0xFF);
    packet[PACKET_SIZE-1] = checksum;

    // Now, we can send the data out:
    SendDataBuffer((char*) packet, PACKET_SIZE);
}

void delay(void)
{
    long unsigned int num_calls = SYS_FREQ/8;
    while(num_calls) num_calls--;
}

void run_fifo(const float new_val, float *array, const unsigned int size)
{
    int i = 0;
    for(i=size-1; i>0; i--)
	array[i] = array[i-1];
    *array = new_val;
    return;
}
    
void calculate_controller_gains(void)
{
    static float zeta = 0.7;
    static float b = 10;

    k1 = 2*zeta*sqrtf( powf(wd,2.0) + b*powf(vd,2.0) );
    k2 = b*fabsf(vd);

    return;
}

int calculate_feedforward_values(const float k)
{
    static float alphaw = 0.1;
    static float alphav = 0.1;
    static float wd_last = 0.0;
    static float vd_last = 0.0;
    static float dta = 0.0;
    
    // reset the running time because we got a new pt:
    running_dt = 0.0;
    // reset the index marking which point in the buffer we are
    // concerned with
    waypoint_index = 2;
    // let's calculate the velocities and accelerations:
    dt2 = tvec[0];
    dt = tvec[1];
    dta = (dt2+dt)/2.0;

    if ( fabsf(dt)<0.00001 || fabsf(dt2)<=0.00001 ||  fabsf(dta)<=0.00001)
    {
        vd = 0;
        wd = 0;
	return 1;
    }

    /* float xd = (xvec[1]-xvec[2])/(dt); */
    /* float yd = (yvec[1]-yvec[2])/(dt); */
    /* float xdp = (xvec[0]-xvec[1])/(dt2); */
    /* float ydp = (yvec[0]-yvec[1])/(dt2); */
    /* float xdd = (xdp-xd)/(dt); */
    /* float ydd = (ydp-yd)/(dt); */

    xd = (xvec[0]-xvec[2])/(dt+dt2);
    yd = (yvec[0]-yvec[2])/(dt+dt2);
    xdd = (xvec[2]-2*xvec[1]+xvec[0])/(powf(dta,2.0));
    ydd = (yvec[2]-2*yvec[1]+yvec[0])/(powf(dta,2.0));
 
    // now we can calculate the orientation at the new desired
    // pose
    thvec[2] = clamp_angle( atan2f(yd, xd) + k*M_PI);
    if(isnan(thvec[2]) != 0)
    	thvec[2] = theta;
    thvec[1] = thvec[2];
    thvec[0] = thvec[2];
    /* thvec[1] = clamp_angle( atan2f(ydp, xdp) + k*M_PI); */
    /* if(isnan(thvec[1]) != 0) */
    /* 	thvec[1] = theta; */
    /* thvec[0] = thvec[1]; */

    // Now we can calculate the feedforward terms
    float tmp = powf(xd,2.0) + powf(yd,2.0);
    vd = dir_sign*sqrt(tmp);
    vd = alphav*vd+(1-alphav)*vd_last;
    if (tmp < 0.00001)
    {
    	wd = 0;
    	return 1;
    }
    else
    {
        wd = dir_sign*(ydd*xd - xdd*yd)/tmp;
	wd = alphaw*wd+(1-alphaw)*wd_last;
    }
    wd_last = wd;
    vd_last = vd;

    if(!swUser)
    	printf("%f\t%f\n\r",vd,wd);
    return 0;
}

void setup_winch_controller(void)
{
    // Pause controller while running this function
    pause_controller_flag = 1;
    float tmp = 0.0;
    // read string
    tmp = interp_number(&Command_String[11]);
    run_fifo(tmp, lftvec, 3);
    tmp = interp_number(&Command_String[14]);
    run_fifo(tmp, rhtvec, 3);

    pause_controller_flag = 0;
    return;
}    

void setup_controller(void)
{
    // We want to disable the controller from running while we are
    // re-calculating gains and desired pose and such
    pause_controller_flag = 1;
    static int data_count = 0;
    float k = 0.0;
    float tmp = 0.0;
    // let's first interpret the string sent to the robot
    tmp = interp_number(&Command_String[2]);
    run_fifo(tmp, tvec, 2);
    tmp = interp_number(&Command_String[5]);
    run_fifo(tmp, xvec, 3);
    tmp = interp_number(&Command_String[8]);
    run_fifo(tmp, yvec, 3);
    
    // determine if we are currently going forward or backward:
    if(tvec[0] < 0.0)
    {
	// Backwards
	if (dir_sign != -1.0)
	{
	    k = 1.0;
	    dir_sign = -1.0;
	    data_count = 0;
	    exec_state = 0;
	    stop_all_motors();
	}
    }
    else
    {
	// Forwards
	if (dir_sign != 1.0)
	{
	    k = 0.0;    
	    dir_sign = 1.0;
	    data_count = 0;
	    exec_state = 0;
	    stop_all_motors();
	}
    }

    // Should we set the exec_state to 5 yet? i.e. do we have enough
    // data to set up the controller?
    if (exec_state != 5)
    {
	if (data_count >= 2)
	{
	    exec_state = 5;
	    data_count = 0;
	}
    }
    
    // set desired orientation and feedforward terms:
    int bad_flag = calculate_feedforward_values(k);
    if(bad_flag)
    {
        k1 = 5;
        k2 = 5;
    }
    else
        // now get the controller gains:
        calculate_controller_gains();

    data_count++;
    pause_controller_flag = 0;
    return;
}

float clamp_angle(float th)
{
    while(th <= 0)
	th += 2.0*M_PI;
    while(th > 2.0*M_PI)
	th -= 2.0*M_PI;
    return th;
}

void run_controller(void)
{
    int recalc = 0;
    // let's find out which waypoint we should be following:
    if (running_dt >= fabsf(dt) && running_dt <= fabsf(dt2+dt))
    {
    	waypoint_index = 1;
    	recalc = 1;
    }
    else if (running_dt >= fabsf(dt2) && waypoint_index == 1)
    {
    	waypoint_index = 0;
    	recalc = 1;
    }

    /* t_sent = tvec[waypoint_index]; */
    /* x_sent = xvec[waypoint_index]; */
    /* y_sent = yvec[waypoint_index]; */
    /* ori_sent = thvec[waypoint_index]; */
    /* t_sent = tvec[1]; */
    /* x_sent = xvec[1]; */
    /* y_sent = yvec[1]; */

    if (running_dt <= (dt+dt2))
    {
	t_sent = tvec[1]+running_dt*sign(xd);
	x_sent = xvec[1]+running_dt*xd;
	y_sent = yvec[1]+running_dt*yd;
    }
    else
    {
	x_sent = xvec[0];
	y_sent = yvec[0];
    }
    ori_sent = thvec[1];
    
    float dth = find_min_angle(ori_sent, theta);
    float v_robot = vd*cosf(dth) + k1*(cosf(theta)*(x_sent-x_pos) +
				       sinf(theta)*(y_sent-y_pos));
    float w_robot = wd + k2*dir_sign*(cosf(theta)*(y_sent-y_pos) -
				      sinf(theta)*(x_sent-x_pos)) + k1*dth;
    // now convert to wheel velocities
    left_desired = 2.0*(v_robot-w_robot*WIDTH)/DWHEEL;
    right_desired = 2.0*(v_robot+w_robot*WIDTH)/DWHEEL;

    // if necessary, run winch controller:
    if (winch_controller_flag == 1)
	run_winch_controller(recalc);

    check_safety();
    
    controller_flag = 0;
    return;
}

void run_winch_controller(int recalc)
{
    static float vl = 0.0, vr = 0.0;

    height_left_sent = lftvec[waypoint_index];
    height_right_sent = rhtvec[waypoint_index];

    if (recalc == 1)
    {
	if ((fabsf(dt) > 0.0001) && (fabsf(dt2) > 0.0001))
	{
	    if (waypoint_index == 2)
	    {
		vl = (height_left_sent-height_left)/fabsf(dt);
		vr = (height_right_sent-height_right)/fabsf(dt);
	    }
	    else
	    {
		vl = (height_left_sent-height_left)/fabsf(dt2);
		vr = (height_right_sent-height_right)/fabsf(dt2);
	    }
	}
	else
	{
	    vl = 0;
	    vr = 0;
	}
    }

    if ((vl >= 0) && (height_left >= height_left_sent))
	vl = 0;
    if ((vl < 0) && (height_left <= height_left_sent))
	vl = 0;
    if ((vr >= 0) && (height_right >= height_right_sent))
	vr = 0;
    if ((vr < 0) && (height_right <= height_right_sent))
	vr = 0;

    // convert to angular velocities:    
    top_left_desired = 2.0*vl/DPULLEY;
    top_right_desired = 2.0*vr/DPULLEY;

    return;
}


void stop_all_motors(void)
{
    left_desired = 0.0;
    right_desired = 0.0;
    top_left_desired = 0.0;
    top_right_desired = 0.0;
    return;
}
	
void check_safety(void)
{
    /* float max_pos_err = 0.1; */
    /* float max_ori_err = 0.2*180/M_PI; */
    /* float max_winch_err = 0.1; */

    /* if(fabsf(x_pos-x_sent) >= max_pos_err || */
    /*    fabsf(y_pos-y_sent) >= max_pos_err) */
    /* 	reset_robot(); */
    /* if(fabsf(find_min_angle(theta,ori_sent)) >= max_ori_err) */
    /* 	reset_robot(); */
    /* if(fabsf(height_left-height_left_sent) >= max_winch_err) */
    /*    /\* fabsf(height_right-height_right_sent) >= max_winch_err) *\/ */
    /* 	reset_robot(); */

    return;
}
