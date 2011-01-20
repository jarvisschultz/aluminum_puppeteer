/*************************************************************
Jarvis Schultz and Marcus Hammond
8-25-2010

This source file contains the primary code that was written for 
controlling a differential drive mobile robot that was created as a 
"puppeteer" for the puppet people project at Disney Research.  The user 
sends serial commands wirelessly through the use of an XBee 
(1 stop bit, no parity bits, no flow control, 115200 baud), and 
this library can interpret the commands.  It is designed to work 
on a PIC32MX460F512L.

Each string that the user sends to the robot contains fifteen 
eight bit characters.  The most significant character is basically
a header character that lets the robot know what kind of instruction 
it is receiving.  

While the robot is executing the received commands a timer 2 interrupt
service routine is called at some set frequency that updates the robot's
current pose using dead reckoning.  The dead reckoning provides the location
(in x and y coordinates) of the center of the robot, and the angle between the
front of the robot and the positive x-axis (ccw rotation is considered to be 
positive, and the angle is in the range of 0-2pi radians).
The code is designed to be almost entirely inerrupt based.  
This leaves the CPU mostly free to be doing other types of calculations.

There are two overarching "modes" of operation.  In the first mode,
the user simply specifies the direction of rotation and speed that is
desired for each wheel.  This mode is useful if all of the commands 
that are desired for the wheels are being executed on user's end. 
The second mode allows the user to simply specify a desired position and 
orientation that the robot should acheive, and the robot will use 
its dead reckoning in order to drive to that configuration.  During 
either of these modes of operation, the user also has the ability to 
send additional information to the robot.  Corrected position and 
orientation can be sent; this is useful if, for example, a camera is
being used as a feedback device.  Also, the user can send an instruction
that stops the robot where it is and defines its current location as the
new origin of the global coordinate system.  

Below is a summary of each instuction that the user can send along with
information about what the robot will do when it receives the instruction.

1)  Configuration Control:
	This is the mode described above where the user specifies a desired 
	configuration for the robot, and it begins driving there.  The motion
	is broken into three sections, an initial rotation that gets the robot 
	oriented such that it can begin driving towards the final location, a
	straight drive towards the final location, and a final rotation to
	the final desired orientation.  The robot decides which direction to
	rotate at both ends and whether to drive forwards or backwards based 
	on which combination requires the least distance of driving.  If while
	executing one of these point commands, a new instruction is received, 
	the robot immediately disregards the old instruction and starts the new.
	This previous statement is true regardless of what type the new instruction 
	is.  If the robot receives an update as to what its current configuration
	is while executing the configuration command, it immediately recalculates 
	what it must do to finish the command and then continues until it is done.
	The string that should be sent contains the following characters:
	Char 1 -			'p' this tells the robot that this command is a point command
	Char 2 -			'0' or '1' this tells the robot the sign of the x-location desired
							a zero indicates negative, and a one inicates positive
	Char 3,4,5,6 - 		These four characters are the desired x-location in inches.  Char 3 is the
							tens digit, 4 is the ones digit, 5 is the tenths, and 6 is the hundredths
	Char 7 -			'0' or '1' this tells the robot the sign of the y-location desired
							a zero indicates negative, and a one inicates positive
	Char 8,9,10,11-		These four characters are the desired y-location in inches.  Char 3 is the
							tens digit, 4 is the ones digit, 5 is the tenths, and 6 is the hundredths
	Char 12,13,14,15 -	These final four characters are the desired final orientation (between 0 and 
							2pi radians).  Char 12 is the ones digit, and the final three are the 
							tenths, hundredths, and thousandths respectively.
							
2) Wheel Speed Control:
	In this mode, the user simply specifies the direction and speed of each wheel. 
	The robot will continue performing the desired action until told to stop or change.
	The string sent by the user is as follows:
	Char 1 -			'h' This char tells the robot that this is a speed instruction
	Char 2 - 			'0' or '1' direction of the left wheel, a one makes the robot go forward and a zero
							backwards
	Char 3,4,5,6 - 		The next four chars are the angular velocity of the left wheel in rad/s e.g. 1234 = 12.34 rad/s
	Char 6 - 			'0' or '1' direction of the right wheel, a one makes the robot go forward and a zero
							backwards
	Char 7,8,9,10 - 	The next four chars are the angular velocity of the right wheel in rad/s e.g. 1234 = 12.34 rad/s						
	The remaining chars can be anything, but they need to be something so that we fill the 15 char in-buffer.

	

3) Corrected Configuration:
	This simply changes what the robot thinks its current configuration is.  Useful for setting the robot's initial
	location at something other than (0,0,0), or correcting its dead reckoning with feedback.  The first char is an
	'l', and the rest of the string follows the exact same format as the configuration control mode.	
	
4) Reset Coordinate System:
	This stops the robot and resets its odometry (effectively translating the global coordinate system to the 
	robot's center and rotating it to align then positive x-axis with the front of the robot).  This is an 'r'
	followed by 14 random chars.
	
5) Change Default Speed:
	When running location control, the robot simply uses a global variable 'speed' as the speed of its wheels.
	This command changes this speed.  The first char is an 's', the next char is garbage, the next four are the 
	default wheel rotation speed in rad/s.  This uses the same format as the wheel velocity control strings.
	
6) Top Motor Control:
	The puppeteer is equipped with a third motor that actuates a winch that controls a marionette string. There is
	a single type of data string for interfacing with this motor.  A summary of the characters follows:
	Char 1 - This char is a 't' to let the robot know we are controlling the top motor
	Char 2 - Either a '0' or a '1'.  A '0' implies that we want to be doing position control at a default
				rotation speed specified in the data string.  A '1' just means we want to move at a set
				velocity.
	Char 3,4,5,6,7 - This 5 character section contains the speed information. Char 3 is the direction;
						'1' is up, '0' is down.  The other four are the standard rad/s velocity command.
	Char 8,9,10,11,12 - This 5 character section contains the position information.  Char 8 is the sign
							of the location we want the end of the string to end up at; '1' is positive.
							The next four contain the actual position info. 1234 = 12.34 inches.
							If we are just doing velocity control, this data is ignored.
	
7) Stop Command:	
	This command stops the robot from moving.  The primary difference between this
	command and the command for setting the speed to zero is the stopping of the h-bridges.  The difference between
	this and the reset coordinate system is that this leaves the global coordinate system alone.  This string is a 'q', 
	followed by 14 unimportant chars.

**************************************************************/



/** Includes ************************************************/
#include "HardwareProfile.h"
#include "Compiler.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#ifndef _MOTORS
#include "DCMotors_FunctionPrototypes.h"
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
#define BAUDRATE	(115200)				// This is the rate that we will communicate over RS232 at
#define CHANNEL_A_L	PORTDbits.RD12
#define CHANNEL_B_L	PORTDbits.RD13
#define CHANNEL_A_R	PORTDbits.RD9
#define CHANNEL_B_R	PORTDbits.RD6
#define CHANNEL_A_T	PORTDbits.RD11
#define CHANNEL_B_T	PORTCbits.RC14
#define MAX_RESOLUTION	3999	            // Proportional to period of PWM (represents the maximum 
					    // number that we can use for PWM function)
#define ERROR_DEADBAND	0.05
#define DATA_LENGTH     16
#define PI  3.141592653
#define frequency  1500 	       // Let's check the kinematics this many times per second
#define GEARRATIO  (19.0*(3.0/4.0))    // The gear ratio of the gearhead on the DC motor
#define CPR  100.0	               // The number of counts per revolution of the motor's encoder
#define dtbase (1.0/frequency)         // The period of CheckKinematics calls
#define convert (PI/(CPR*dtbase*GEARRATIO)) // For converting angular wheel and motor velocities
#define converttop (convert*(3.0/4.0))
#define ticktime (2.0/(80000000.0))    // For calculating times since timer ISR's were initially called



/** Global Variables **************************************************/
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
static float d_pulley = 0.75;           // This is the diameter of the pulley that the string winds onto
static float L = 5.95/2;		// This is one-half of the robot's track width
static float D = 3.0;			// Diameter of the wheel in inches
static float front_dist = 1.77;		// Perpendicular distance from axle axis to front of robot in inches
static float speed = 3.0;		// This is the default wheel revolution rate when in pose control mode (rad/s).
static float speed_t = 3.0;		// This is the default speed for moving the top motor during position control.
static char RS232_In_Buffer[DATA_LENGTH] = "zzzzzzzzzzzzzzzz";	//  This is an array that is initialized with 
                                                                // useless data in it; it is used for temporary 
                                                                // storage of data brought in from the PC on UART2
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

UINT8 STR[1024];				// An empty string we use for sending data
static float kp = 500;				// Gain on the proportional error term
static float ki = 50;				// Gain on the integral error term
static float kd = 0.1;			// Gain on the derivative error term



/** Interrupt Handler Functions:**************************************************************************************************/
// UART 2 interrupt handler
// it is set at priority level 2
void __ISR(_UART2_VECTOR, ipl7) IntUart2Handler(void)
{
    char temp;
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
	if(temp == 'p'||temp == 'l'||temp == 'r'||temp == 'h'
	   ||temp == 's'||temp == 'q'||temp == 't')
	{
	    i = 0;
	    header_flag = 1;
	}
	// Regardless of what temp is, let's place it in the in buffer
	RS232_In_Buffer[i%DATA_LENGTH] = temp;
		
	// Now let's increment i
	i++;
		
	// If we have received a complete set of data, let's set the flag to
	// zero so that we can deal with the data in main
	if((i-1) == (DATA_LENGTH-1) && header_flag == 1) 
	{
	    data_flag = 1;
	    mLED_1_Toggle();
	}
    }

    // Is this interrupt from sending data?
    if ( mU2TXGetIntFlag() )
    {
	mU2TXClearIntFlag();
    }
}

// This is the ISR that gets called when we detect a rising or falling edge on CHANNEL_A_R
void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl6) CheckPosition_r()
{
    static int tempA, tempB;
    tempA = CHANNEL_A_R;
    tempB = CHANNEL_B_R;
		
    // Let's clear the interrupt flag:
    INTClearFlag(INT_IC2);
    mLED_2_Toggle();
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
void __ISR(_INPUT_CAPTURE_5_VECTOR, ipl6) CheckPosition_l()
{
    static int tempA, tempB;
    tempA = CHANNEL_A_L;
    tempB = CHANNEL_B_L;
		
    // Let's clear the interrupt flag:
    INTClearFlag(INT_IC5);
    mLED_3_Toggle();
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
void __ISR(_INPUT_CAPTURE_4_VECTOR, ipl6) CheckPosition_t()
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


//  In the following function, we will be updating the robot's pose 
//  using forward kinematics and odometry data.  We will also decide 
//  if we new controls are needed to be sent to the wheels so that 
//  the robot can continue to do what it is supposed to be doing.
void __ISR(_TIMER_2_VECTOR, ipl4) CheckKinematics(void)
{
    static int check_count = 0;
    float Vr = 0.0;
    float Vl = 0.0;
    float Vt = 0.0;
    float omega = 0.0;
    float R = 0.0;
    float dt = 0.0;
	
    // Let's first calculate the current angular velocity of each wheel:
    left_speed = convert*((float) (left_steps-left_steps_last));
    right_speed = convert*((float) (right_steps-right_steps_last));
    top_speed = converttop*((float) (top_steps-top_steps_last));
	
    // Now let's store the current counts for the next CheckKinematics call:
    left_steps_last = left_steps;
    right_steps_last = right_steps;
    top_steps_last = top_steps;
	
    // Now, let's calculate the error between the actual and desired velocities 
    float left_error = left_desired-left_speed;
    float right_error = right_desired-right_speed;
    float top_error = top_desired-top_speed;
    
    // Now let's calculate how long it has been since this function has been called.
    dt = ticktime*(((float) ReadTimer2())+1.0)+dtbase;

    // Now, we set the PWM Values:
    if(fabs(left_error) > ERROR_DEADBAND) SetSpeedLeft(left_error, dt);
    if(fabs(right_error) > ERROR_DEADBAND) SetSpeedRight(right_error, dt);
    if(fabs(top_error) > ERROR_DEADBAND) SetSpeedTop(top_error, dt);
	
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
	// First let's calculate how long it has been since this function has been called.
	dt = ticktime*(((float) ReadTimer2())+1.0)+dtbase;
	// So, let's perform the straight version kinematics
	x_pos += Vr*dt*cosf(theta);
	y_pos += Vr*dt*sinf(theta);
    }
    else
    {
	// First let's calculate how long it has been since this function has been called.
	dt = ticktime*(((float) ReadTimer2())+1.0)+dtbase;
	// Now let's perform the general kinematics
	x_pos += cosf(omega*dt)*R*sinf(theta)+sinf(omega*dt)*R*cosf(theta)-R*sinf(theta);
	y_pos += sinf(omega*dt)*R*sinf(theta)-cosf(omega*dt)*R*cosf(theta)+R*cosf(theta);
	theta += omega*dt;
    }
	
    // Let's force theta to be between 0 and 2pi
    if(theta < 0.0) theta += 2.0*PI;
    if(theta >= 2.0*PI) theta -= 2.0*PI;	
	
    // Now, let's update the height of our string
    height += Vt*dt;
/*	
	if(!swUser)
	{
	sprintf(STR,"%f\r\n\r\n",height);
	SendDataBuffer(STR,strlen(STR));
	}
*/
	
	
    if(height_flag == 1)
    {
	if(fabs(height_sent-height) < 0.01)
	{
//    	    sprintf(STR,"%f\r\n\r\n",height);
//	    SendDataBuffer(STR,strlen(STR));
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
	if(fabs(theta-first_angle) <= 0.01 || fabs(fabs(theta-first_angle)-2.0*PI) <= 0.01)
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
	if(fabs(theta-ori_sent) <= 0.01 || fabs(fabs(theta-ori_sent)-2.0*PI) <= 0.01)
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
    static float d_error = 0.0;       // This is the derivative error
    static float last_d_error = 0.0;  // This is the previous derivative error    
    static float last_error = 0.0;    // This is the error sent to SetSpeed functions last time for D-control
    static float total_error = 0.0;   // This is the sum of all three components of the error
    static unsigned int PWMVal = 0;     // The actual value that will get stored in the PMW register
    static float tau = 1.0/(2.0*PI*500.0);// Filtering parameter - Value in denominator sets cutoff frequency for LPF

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
    static float d_error = 0.0;       // This is the derivative error
    static float last_d_error = 0.0;  // This is the previous derivative error    
    static float last_error = 0.0;    // This is the error sent to SetSpeed functions last time for D-control
    static float total_error = 0.0;   // This is the sum of all three components of the error
    static unsigned int PWMVal = 0;     // The actual value that will get stored in the PMW register
    static float tau = 1.0/(2.0*PI*500.0);// Filtering parameter - Value in denominator sets cutoff frequency for LPF

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
    static float d_error = 0.0;       // This is the derivative error
    static float last_d_error = 0.0;  // This is the previous derivative error    
    static float last_error = 0.0;    // This is the error sent to SetSpeed functions last time for D-control
    static float total_error = 0.0;   // This is the sum of all three components of the error
    static unsigned int PWMVal = 0;     // The actual value that will get stored in the PMW register
    static float tau = 1.0/(2.0*PI*500.0);// Filtering parameter - Value in denominator sets cutoff frequency for LPF

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
    ConfigIntCapture5(IC_INT_ON | IC_INT_PRIOR_6);
	
    // Now we can configure the CHANNEL_A_R pin (D9) to be an input capture pin:
    // - Capture Every edge
    // - Enable capture interrupts
    // - Use Timer 3 source
    // - Capture rising edge first
    OpenCapture2( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC |IC_FEDGE_RISE | IC_ON );
    // Configure the interrupt options
    ConfigIntCapture2(IC_INT_ON | IC_INT_PRIOR_6);
	
    // Now we can configure the CHANNEL_A_T pin (D11) to be an input capture pin:
    // - Capture Every edge
    // - Enable capture interrupts
    // - Use Timer 3 source
    // - Capture rising edge first
    OpenCapture4( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC |IC_FEDGE_RISE | IC_ON );
    // Configure the interrupt options
    ConfigIntCapture4(IC_INT_ON | IC_INT_PRIOR_6);
	
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


void InitTimer2(void)
{
    int Per;
    Per = (80000000/frequency)/2-1;  // Set the value in the period register
    OpenTimer2(T2options, Per);	
    mT2SetIntPriority( 4); 	// set Timer 2 Interrupt Priority
    mT2ClearIntFlag(); 		// clear interrupt flag
    mT2IntEnable( 1);		// enable timer 2 interrupts
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

	if(first_angle <= 0.0) first_angle += 2.0*PI;
	// Now calculate the orientation of the back of the robot:
	thback = theta+PI;
	if(thback >= 2.0*PI) thback -= 2.0*PI;
	// Also, let's figure out where the back would be oriented at the destination orientation:
	tpback = ori_sent+PI;
	if(tpback >= 2.0*PI) tpback -= 2.0*PI;
		
	// Let's let RuntimeOperation() know that it can stop calling this function now
	pose_flag = 0;
		
	// Now let's determine which direction we want to rotate, and which direction we want to drive:
		
	// Let's assume we want forward operation.
	// Start by calculating the minimum cost of the initial rotation for this case:
	if(((first_angle-theta >= 0)&&(first_angle-theta <= PI))||
	   ((first_angle-theta<0)&&(first_angle-theta<-PI)))
	{
	    // Positive rotation:
	    rot1_f = 1;
	}
	else
	{
	    // Negative rotation:
	    rot1_f = -1;
	}
	costf += min(fabs(first_angle-theta-2*PI),fabs(first_angle-theta));
		
		
	// Now calculate the minimum cost of the final rotation for this case:
	if(((ori_sent-first_angle >= 0)&&(ori_sent-first_angle <= PI))||
	   ((ori_sent-first_angle<0)&&(ori_sent-first_angle<-PI)))
	{
	    // Positive rotation:
	    rot2_f = 1;
	}
	else
	{
	    // Negative rotation:
	    rot2_f = -1;
	}
	costf += min(fabs(ori_sent-first_angle),fabs(fabs(ori_sent-first_angle)-2*PI));
		
	// Now, let's assume backwards operation.
	// Start by calculating the minimum cost of the initial rotation for this case:
	if(((first_angle-thback >= 0)&&(first_angle-thback <= PI))||
	   ((first_angle-thback<0)&&(first_angle-thback<-PI)))
	{
	    // Positive rotation:
	    rot1_b = 1;
	}
	else
	{
	    // Negative rotation:
	    rot1_b = -1;
	}
	costb += min(fabs(first_angle-thback),fabs(fabs(first_angle-thback)-2*PI));
		
	// Now calculate the minimum cost of the final rotation for this case:
	if(((tpback-first_angle >= 0)&&(tpback-first_angle <= PI))||
	   ((tpback-first_angle<0)&&(tpback-first_angle<-PI)))
	{
	    // Positive rotation:
	    rot2_b = 1;
	}
	else
	{
	    // Negative rotation:
	    rot2_b = -1;
	}
	costb += min(fabs(tpback-first_angle),fabs(fabs(tpback-first_angle)-2*PI));

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
	    first_angle += PI;
	    if(first_angle > 2.0*PI) first_angle -= 2.0*PI;
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
    INTEnable(INT_OC5, 0);
    INTEnable(INT_OC2, 0);	
    INTEnable(INT_OC3, 0);
    INTEnable(INT_IC4, 0);
    INTEnable(INT_IC2, 0);
    INTEnable(INT_IC5, 0);
    INTEnable(INT_U2RX, 0);
	
    // Now, we can begin sorting the data:
    char x_sign; 	// Determines if the received x-coordinate is positive or negative
    char y_sign; 	// Determines if the received y-coordinate is positive or negative
    char data;   	// This is the very first entry in the in buffer (the header byte)
    char dir_left;	// The direction of the left wheel that was sent
    char dir_right; // The direction of the right wheel that was sent
    char dir_top;	
		
    // Set the current byte of the RS232_In_Buffer to be equal to data:
    data = RS232_In_Buffer[0];

    // If the header is a 'p', we have received an instruction for where to go:
    if (data == 'p')
    {
	exec_state = 2;
	// Now let's convert the 15 bytes of chars into useful data
	x_sign = RS232_In_Buffer[1];
	x_sent = ((float)(1000*((int) RS232_In_Buffer[2]-48)+
			  100*((int) RS232_In_Buffer[3]-48)+10*((int) RS232_In_Buffer[4]-48)+
			  ((int) RS232_In_Buffer[5]-48)))/100.0;
	y_sign = RS232_In_Buffer[6];
	y_sent = ((float)(1000*((int) RS232_In_Buffer[7]-48)+
			  100*((int) RS232_In_Buffer[8]-48)+10*((int) RS232_In_Buffer[9]-48)+
			  ((int) RS232_In_Buffer[10]-48)))/100.0;
	ori_sent = ((float)(1000*((int) RS232_In_Buffer[11]-48)+
			    100*((int) RS232_In_Buffer[12]-48)+10*((int) RS232_In_Buffer[13]-48)+
			    ((int) RS232_In_Buffer[14]-48)))/1000.0;

	if(x_sign == '0') x_sent = -x_sent;
	if(y_sign == '0') y_sent = -y_sent;
		
	// Let's set pose_flag so that we know we need to call SetPose()
	pose_flag = 1;
    }
	
    // If the header byte is an 'l', we have received updated information about the 
    // robot's current location.
    else if(data == 'l')
    {
	x_sign = RS232_In_Buffer[1];
	x_pos = ((float)(1000*((int) RS232_In_Buffer[2]-48)+
			 100*((int) RS232_In_Buffer[3]-48)+10*((int) RS232_In_Buffer[4]-48)+
			 ((int) RS232_In_Buffer[5]-48)))/100.0;
	y_sign = RS232_In_Buffer[6];
	y_pos = ((float)(1000*((int) RS232_In_Buffer[7]-48)+
			 100*((int) RS232_In_Buffer[8]-48)+10*((int) RS232_In_Buffer[9]-48)+
			 ((int) RS232_In_Buffer[10]-48)))/100.0;
	theta = ((float)(1000*((int) RS232_In_Buffer[11]-48)+
			 100*((int) RS232_In_Buffer[12]-48)+10*((int) RS232_In_Buffer[13]-48)+
			 ((int) RS232_In_Buffer[14]-48)))/1000.0;

	if(x_sign == '0') x_pos = -x_pos;
	if(y_sign == '0') y_pos = -y_pos;
		
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
    // If we receive an h as the header char, then that means that we will simply control the wheel speeds
    else if (data == 'h')
    {
	exec_state = 1;
	dir_left = RS232_In_Buffer[1];
	left_desired = 10.0*((float) RS232_In_Buffer[2]-48.0)+((float) RS232_In_Buffer[3]-48.0)+0.1*((float) RS232_In_Buffer[4]-48.0)+0.01*((float) RS232_In_Buffer[5]-48.0);
	dir_right = RS232_In_Buffer[6];
	right_desired = 10.0*((float) RS232_In_Buffer[7]-48.0)+((float) RS232_In_Buffer[8]-48.0)+0.1*((float) RS232_In_Buffer[9]-48.0)+0.01*((float) RS232_In_Buffer[10]-48.0);
	dir_top = RS232_In_Buffer[11];
	top_desired = 10.0*((float) RS232_In_Buffer[12]-48.0)+((float) RS232_In_Buffer[13]-48.0)+0.1*((float) RS232_In_Buffer[14]-48.0)+0.01*((float) RS232_In_Buffer[15]-48.0);

	// Here we need to calculate the motor speeds and set them	
	if(dir_left == '0') left_desired = -left_desired;		
	if(dir_right == '0') right_desired = -right_desired;
	if(dir_top == '0') top_desired = -top_desired;			
		
	// We just received commands for explicitly controlling the wheel speeds, let's force the pose control
	// to stop executing:
	height_flag = 0;
	pose_flag = 0;
    }
    else if (data == 's')
    {
	speed = 10.0*((float) RS232_In_Buffer[2]-48.0)+((float) RS232_In_Buffer[3]-48.0)+0.1*((float) RS232_In_Buffer[4]-48.0)+0.01*((float) RS232_In_Buffer[5]-48.0);
	while(BusyUART2());
	putsUART2("Changed Default Speed\r\n");
    }
    else if (data == 'q')
    {
	exec_state = 1;
	left_desired = 0.0;
	right_desired = 0.0;
	top_desired = 0.0;
    }
    else if (data == 't')
    {
    	char top_state;
    	// Get what mode we are in:
    	top_state = RS232_In_Buffer[1];
    	top_desired = 10.0*((float) RS232_In_Buffer[3]-48.0)+((float) RS232_In_Buffer[4]-48.0)+
    	    0.1*((float) RS232_In_Buffer[5]-48.0)+0.01*((float) RS232_In_Buffer[6]-48.0);
		
    	if(top_state == '0')
    	{
    	    // If this is a zero, we are doing height control.
    	    // Get the sign of the height:
    	    dir_top = RS232_In_Buffer[7];
    	    // Get the height:
    	    height_sent = 10.0*((float) RS232_In_Buffer[8]-48.0)+((float) RS232_In_Buffer[9]-48.0)+
    		0.1*((float) RS232_In_Buffer[10]-48.0)+0.01*((float) RS232_In_Buffer[11]-48.0);
    	    // Do we need to get the negative of the height?
    	    if(dir_top == '0') height_sent = -height_sent;
    	    // Now, do we need to move up or down?
    	    if(height_sent <= height) top_desired = -top_desired;
    	    // Let's set the position control flag:
    	    height_flag = 1;
    	}
    	else if(top_state == '1')
    	{
    	    // If this is a one, we are just controlling speed:
    	    dir_top = RS232_In_Buffer[2];
    	    if(dir_top == '0') top_desired = -top_desired;
    	    height_flag = 0;
    	}
    }
	
    // Now, let's re-enable all interrupts:
    INTEnable(INT_OC5, 1);
    INTEnable(INT_OC2, 1);	
    INTEnable(INT_OC3, 1);
    INTEnable(INT_IC4, 1);
    INTEnable(INT_IC2, 1);
    INTEnable(INT_IC5, 1);
    INTEnable(INT_U2RX, 1);
}

void RuntimeOperation(void)
{
    // If the buffer has just filled, and we know we received a good
    // set of data (because it began with a header bit and was 14 chars), 
    // let's call our functions that interpret and execute the received 
    // data
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





