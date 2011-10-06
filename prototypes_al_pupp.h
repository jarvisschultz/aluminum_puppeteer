/*************************************************
Jarvis Schultz and Marcus Hammond 

5-25-2011

This file simply contains the function prototypes/ declarations
*************************************************/
#ifndef _MOTORS

#define _MOTORS
#include <plib.h>
#include <p32xxxx.h>
/******************************************************************************
 * FUNCTIONS FOR MOTOR CONTROLS ***********************************************
 * ***************************************************************************/
// LEFT
void SetStepsLeft(int set_steps);   	
void SetSpeedLeft(float motor_speed, float dt);
float GetSpeedLeft(void);			
int GetStepsLeft(void);		 	        
// RIGHT
void SetStepsRight(int set_steps);
void SetSpeedRight(float motor_speed, float dt);
float GetSpeedRight(void);			
int GetStepsRight(void);		 	
// TOP LEFT
void SetStepsTopLeft(int set_steps);     
void SetSpeedTopLeft(float motor_speed, float dt); 
float GetSpeedTopLeft(void);				
int GetStepsTopLeft(void);		 		
// TOP RIGHT
void SetStepsTopRight(int set_steps);        
void SetSpeedTopRight(float motor_speed, float dt); 	
float GetSpeedTopRight(void);				
int GetStepsTopRight(void);

void stop_all_motors(void);

/******************************************************************************
 * FUNCTIONS FOR INITIALIZATIONS **********************************************
 * ***************************************************************************/
void InitUART2(int pbClk);				
void InitTimer2(void);					
void InitTimer4(void);					
void InitMotorPWM(void);				
void InitEncoder(void);					

/******************************************************************************
 * FUNCTIONS FOR COMMUNICATION ************************************************
 * ***************************************************************************/
void interp_command(void);				
void SendDataBuffer(const char *buffer, UINT32 size);	
float interp_number(const unsigned char *data);	 	
int data_checker(unsigned char* buff);
void build_number(unsigned char *destination, float value, short int divisor);
void make_string(unsigned char *dest, char type,
		 float fval, float sval, float tval, int div);
void create_send_array(unsigned short id, unsigned char *DataString);

/******************************************************************************
 * FUNCTIONS FOR ROBOT CONTROL ************************************************
 * ***************************************************************************/
// GENERAL
void SetPose(float xdest, float ydest, float thdest);
void RuntimeOperation(void);    			
void reset_robot(void);
// Functions for kinematic controller:
void run_filo(const double new_val, double *array);
void calculate_controller_gains(void);
int calculate_feedforward_values(const float k);
void setup_controller(void);
void run_controller(void);
void setup_winch_controller(void);
void run_winch_controller(int recalc);
void check_safety(void);

/******************************************************************************
 * MISCELLANEOUS FUNCTIONS ****************************************************
 * ***************************************************************************/
float find_min(float a, float b);			
void delay(void);
float clamp_angle(float th);
float find_min_angle(float a, float b);
float sign(float x);


#endif













