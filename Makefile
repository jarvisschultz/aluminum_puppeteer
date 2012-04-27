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
PIC = 460
PROC = 32MX$(PIC)F512L
TARGET = Al_Robot

all : $(TARGET).hex

$(TARGET).hex : $(TARGET).elf
	$(HX) $(TARGET).elf

$(TARGET).elf : $(OBJ)
	$(CC) -mprocessor=$(PROC) $(OBJ) -v -Wall -o $(TARGET).elf -Wl,--defsym=__MPLAB_BUILD=1,-Map=$(TARGET).map

%.o : %.c $(HDR)
	$(CC) -mprocessor=$(PROC) -v -Wall -c $< -o $@ -I"." -g

clean : 
	$(RM) $(OBJ) $(TARGET).elf $(TARGET).hex $(TARGET).map

write : all
	ubw32 -write $(TARGET).hex

help : 
	ubw32 -help

reset : 
	ubw32 -reset


