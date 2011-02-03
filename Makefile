# My first generated makefile.

CD = /opt/microchip/mplabc32/v1.11a/bin/
AS = pic32-as
CC = pic32-gcc
LD = pic32-ld
AR = pic32-ar
HX = pic32-bin2hex
RM = rm
OBJ = DCMotors_MainBroadcast.o\
	DCMotors_MotorControlsBroadcast.o
HDR = DCMotors_FunctionsBroadcast.h\
	HardwareProfile.h

all : PCBVersionRobot.hex

PCBVersionRobot.hex : PCBVersionRobot.elf
	$(CD)$(HX) "PCBVersionRobot.elf"

PCBVersionRobot.elf : $(OBJ)
	$(CD)$(CC) -mprocessor=32MX460F512L $(OBJ) -o"PCBVersionRobot.elf" -Wl,--defsym=__MPLAB_BUILD=1,-Map="PCBVersionRobot.map"

%.o : %.c $(HDR)
	$(CD)$(CC) -mprocessor=32MX460F512L -c $< -o $@ -I"/opt/microchip/MicrochipSolutions/Microchip/Include/" -I"." -g

clean : 
	$(RM) $(OBJ) "PCBVersionRobot.elf" "PCBVersionRobot.hex"

