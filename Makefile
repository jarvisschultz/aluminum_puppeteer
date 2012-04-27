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


$(PIC)_$(TARGET).hex : $(PIC)_$(TARGET).elf
	@echo Creating hex file
	$(HX) $(PIC)_$(TARGET).elf

$(PIC)_$(TARGET).elf : $(addprefix $(PIC)_, $(OBJ))
	@echo Linking elf file
	$(CC) -mprocessor=$(PROC) $(addprefix $(PIC)_, $(OBJ)) -Wall -o $(PIC)_$(TARGET).elf -Wl,--defsym=__MPLAB_BUILD=1,-Map=$(PIC)_$(TARGET).map

%.o :  $(patsubst %.o, %.c, $(subst $(PIC)_,,$(OBJ))) $(HDR)
	@echo creating object $@
	$(CC) -mprocessor=$(PROC) -Wall -c $(patsubst %.o, %.c, $(subst $(PIC)_,,$@)) -o $@ -I"." -g

clean : 
	$(RM) *.hex *.map *.o *.elf	

write : 
	ubw32 -write $(PIC)_$(TARGET).hex

help : 
	ubw32 -help

reset : 
	ubw32 -reset


