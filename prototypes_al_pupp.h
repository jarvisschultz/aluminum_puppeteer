/*************************************************
Jarvis Schultz and Marcus Hammond 

5-25-2011

This file simply contains the function prototypes/ declarations
*************************************************/
#ifndef _MOTORS

#define _MOTORS
#include <plib.h>
#include <p32xxxx.h>
void SetStepsLeft(int set_steps);   	// This function manually changes the number stored in the number of steps variable for the left motor
void SetSpeedLeft(float motor_speed, float dt); 	// This function sets the left motor speed at the desired speed and begins driving the motor
float GetSpeedLeft(void);				// This function simply returns the current value of the speed set for the left motor in (steps/sec)
int GetStepsLeft(void);		 	                // This function returns the current left motor position in steps

void SetStepsRight(int set_steps);   	// This function manually changes the number stored in the number of steps variable for the right motor
void SetSpeedRight(float motor_speed, float dt); 	// This function sets the right motor speed at the desired speed and begins driving the motor
float GetSpeedRight(void);				// This function simply returns the current value of the speed set for the left motor in (steps/sec)
int GetStepsRight(void);		 		// This function returns the current right motor position in steps

void SetStepsTopLeft(int set_steps);        // This function manually changes the number stored in the number of steps variable for the right motor
void SetSpeedTopLeft(float motor_speed, float dt); 	// This function sets the right motor speed at the desired speed and begins driving the motor
float GetSpeedTopLeft(void);				// This function simply returns the current value of the speed set for the left motor in (steps/sec)
int GetStepsTopLeft(void);		 		// This function returns the current right motor position in steps

void SetStepsTopRight(int set_steps);        // This function manually changes the number stored in the number of steps variable for the right motor
void SetSpeedTopRight(float motor_speed, float dt); 	// This function sets the right motor speed at the desired speed and begins driving the motor
float GetSpeedTopRight(void);				// This function simply returns the current value of the speed set for the left motor in (steps/sec)
int GetStepsTopRight(void);		 		// This function returns the current right motor position in steps

void InitUART2(int pbClk);				// Function for initializing UART2
void SetPose(float xdest, float ydest, float thdest);	// This function contains all of the logic for the pose control mode of operation
void PoseUpdate(void);					// Once we have received a full set of instructions, this function is called to implement them.
void InitTimer2(void);					// This function initializes timer 2
void InitTimer4(void);					// This function initializes timer 4
void RuntimeOperation(void);    			// This function actually executes everything.  It needs to be called repeatedly in the main loop.
float find_min(float a, float b);			        // This function returns the minimum value of two floats passed into it.
void InitMotorPWM(void);				// This initializes the pins that are used for PWM control of the DC motors
void InitEncoder(void);					// This function initializes the IC pins that are being used for reading encoder pulses
void SendDataBuffer(const char *buffer, UINT32 size);	// This is a function that is used for sending data over UART2

float InterpNumber(const unsigned char *data);		// This function takes in a pointer to a character array, it then interprets the first three
							// characters found at that location, and returns a float.
int Data_Checker(unsigned char* buff);			// For checking the validity of sent commands
void Reset_Robot(void);
void BuildNumber(unsigned char *destination, float value, short int divisor);
void MakeString(unsigned char *dest, char type, float fval, float sval, float tval, int div);
void CreateAndSendArray(unsigned short id, unsigned char *DataString);
void delay(void);

// Functions for kinematic controller:
void run_filo(const float new_val, float *array);
void calculate_controller_gains(void);
void calculate_feedforward_values(const float k);
void setup_controller(void);
float clamp_angle(float th);
float find_min_angle(float a, float b);
float sign(float x);

#endif













