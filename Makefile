CD = /opt/microchip/mplabc32/v1.11a/bin/
AS = pic32-as
CC = pic32-gcc
LD = pic32-ld
AR = pic32-ar
HX = pic32-bin2hex
RM = rm
OBJ = functions_al_pupp.o\
	main_al_pupp.o
HDR = prototypes_al_pupp.h\
	HardwareProfile.h

all : Al_Robot.hex

Al_Robot.hex : Al_Robot.elf
	$(CD)$(HX) "Al_Robot.elf"

Al_Robot.elf : $(OBJ)
	$(CD)$(CC) -mprocessor=32MX460F512L $(OBJ) -v -Wall -o"Al_Robot.elf" -Wl,--defsym=__MPLAB_BUILD=1,-Map="Al_Robot.map"

%.o : %.c $(HDR)
	$(CD)$(CC) -mprocessor=32MX460F512L -v -Wall -c $< -o $@ -I"/opt/microchip/MicrochipSolutions/Microchip/Include/" -I"." -g

clean : 
	$(RM) $(OBJ) "Al_Robot.elf" "Al_Robot.hex"

