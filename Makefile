# MPLAB IDE generated this makefile for use with GNU make.
# Project: PCBVersionRobot.mcp
# Date: Wed Jan 26 10:05:25 2011

AS = pic32-as.exe
CC = pic32-gcc.exe
LD = pic32-ld.exe
AR = pic32-ar.exe
HX = pic32-bin2hex.exe
RM = rm

PCBVersionRobot.hex : PCBVersionRobot.elf
	$(HX) "PCBVersionRobot.elf"

PCBVersionRobot.elf : Objects/DCMotors_MainBroadcast.o Objects/DCMotors_MotorControlsBroadcast.o
	$(CC) -mprocessor=32MX460F512L "Objects\DCMotors_MainBroadcast.o" "Objects\DCMotors_MotorControlsBroadcast.o" -o"PCBVersionRobot.elf" -Wl,--defsym=__MPLAB_BUILD=1,-Map="PCBVersionRobot.map"

Objects/DCMotors_MainBroadcast.o : DCMotors_MainBroadcast.c DCMotors_FunctionsBroadcast.h C:/Microchip\ Solutions/Microchip/Include/Compiler.h HardwareProfile.h
	$(CC) -mprocessor=32MX460F512L -x c -c "DCMotors_MainBroadcast.c" -o".\Objects\DCMotors_MainBroadcast.o" -MMD -MF".\Objects\DCMotors_MainBroadcast.d" -I"C:\Microchip Solutions\Microchip\Include" -I"." -I"..\Microchip_USB_2_7\Include" -g

Objects/DCMotors_MotorControlsBroadcast.o : DCMotors_MotorControlsBroadcast.c HardwareProfile.h C:/Microchip\ Solutions/Microchip/Include/Compiler.h DCMotors_FunctionsBroadcast.h
	$(CC) -mprocessor=32MX460F512L -x c -c "DCMotors_MotorControlsBroadcast.c" -o".\Objects\DCMotors_MotorControlsBroadcast.o" -MMD -MF".\Objects\DCMotors_MotorControlsBroadcast.d" -I"C:\Microchip Solutions\Microchip\Include" -I"." -I"..\Microchip_USB_2_7\Include" -g

clean : 
	$(RM) "Objects\DCMotors_MainBroadcast.o" "Objects\DCMotors_MotorControlsBroadcast.o" "PCBVersionRobot.elf" "PCBVersionRobot.hex"

