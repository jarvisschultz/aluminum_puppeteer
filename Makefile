AS := xc32-as
CC := xc32-gcc
LD := xc32-ld
AR := xc32-ar
HX := xc32-bin2hex
RM := rm
OBJ := $(patsubst %.c, %.o,$(wildcard *.c))
HDR := $(wildcard *.h)
PIC := 460
PROC := 32MX$(PIC)F512L
TARGET := Al_Robot


.PHONY : clean

$(PIC)_$(TARGET).hex : $(PIC)_$(TARGET).elf
	@echo Creating hex file
	$(HX) $(PIC)_$(TARGET).elf

$(PIC)_$(TARGET).elf : $(addprefix $(PIC)_, $(OBJ))
	@echo Linking elf file
	$(CC) -mprocessor=$(PROC) $(addprefix $(PIC)_, $(OBJ)) \
		-Wl,-Bstatic -lmchp_peripheral_$(PROC) -Wall \
		-o $(PIC)_$(TARGET).elf -Wl,--defsym=__MPLAB_BUILD=1,-Map=$(PIC)_$(TARGET).map

$(addprefix $(PIC)_, $(OBJ)) : %.o :  $(patsubst %.o, %.c, $(subst $(PIC)_,,$(OBJ))) $(HDR)
	@echo creating object $@
	$(CC) -mprocessor=$(PROC) -Wall -c $(patsubst %.o, %.c, $(subst $(PIC)_,,$@)) \
		-o $@ -I"." -g

clean : 
	$(RM) *.hex *.map *.o *.elf	

write : $(PIC)_$(TARGET).hex
	ubw32 -write $(PIC)_$(TARGET).hex

help : 
	ubw32 -help

reset : 
	ubw32 -reset


