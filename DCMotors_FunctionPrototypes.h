/*************************************************
Jarvis Schultz and Marcus Hammond 

8-25-2010

This file simply contains the function prototypes/ declarations for 
DCMotors_MotorControls.c.
*************************************************/
#ifndef _MOTORS
	#define _MOTORS
#include "Compiler.h"	
void SetStepsLeft(int set_steps);   	// This function manually changes the number stored in the number of steps variable for the left motor
void SetSpeedLeft(float motor_speed, float dt); 	// This function sets the left motor speed at the desired speed and begins driving the motor
float GetSpeedLeft(void);				// This function simply returns the current value of the speed set for the left motor in (steps/sec)
int GetStepsLeft(void);		 	                // This function returns the current left motor position in steps

void SetStepsRight(int set_steps);   	// This function manually changes the number stored in the number of steps variable for the right motor
void SetSpeedRight(float motor_speed, float dt); 	// This function sets the right motor speed at the desired speed and begins driving the motor
float GetSpeedRight(void);				// This function simply returns the current value of the speed set for the left motor in (steps/sec)
int GetStepsRight(void);		 		// This function returns the current right motor position in steps

void SetStepsTop(int set_steps);        // This function manually changes the number stored in the number of steps variable for the right motor
void SetSpeedTop(float motor_speed, float dt); 	        // This function sets the right motor speed at the desired speed and begins driving the motor
float GetSpeedTop(void);				// This function simply returns the current value of the speed set for the left motor in (steps/sec)
int GetStepsTop(void);		 			// This function returns the current right motor position in steps

void InitUART2(int pbClk);				// Function for initializing UART2
void SetPose(float xdest, float ydest, float thdest);	// This function contains all of the logic for the pose control mode of operation
void PoseUpdate(void);					// Once we have received a full set of instructions, this function is called to implement them.
void InitTimer2(void);					// This function initializes timer 2
void RuntimeOperation(void);    			// This function actually executes everything.  It needs to be called repeatedly in the main loop.
float min(float a, float b);			        // This function returns the minimum value of two floats passed into it.
void InitMotorPWM(void);				// This initializes the pins that are used for PWM control of the DC motors
void InitEncoder(void);					// This function initializes the IC pins that are being used for reading encoder pulses
void SendDataBuffer(const char *buffer, UINT32 size);	// This is a function that is used for sending data over UART2
#endif













