# HLab firmware for the PicoRV32 (RV32IM).
# Works with either toolchain:
#   make                      (clang + ld.lld, default)
#   make TOOLCHAIN=gcc        (riscv64-unknown-elf-gcc or riscv-none-elf-gcc, set CROSS=)
# Output: firmware.hex (load into cpu_subsystem RAM via $readmemh)

TOOLCHAIN ?= clang
CROSS     ?= riscv64-unknown-elf-
ARCH       = -march=rv32im -mabi=ilp32
CFLAGS     = $(ARCH) -O2 -ffreestanding -fno-builtin -nostdlib -Wall -Wextra -std=c11
LDFLAGS    = -nostdlib -T link.ld

ifeq ($(TOOLCHAIN),clang)
CC      = clang --target=riscv32-unknown-elf
LD      = ld.lld
OBJCOPY = llvm-objcopy
OBJDUMP = llvm-objdump
LINK    = $(LD) $(LDFLAGS) -o $@ start.o main.o
else
CC      = $(CROSS)gcc
OBJCOPY = $(CROSS)objcopy
OBJDUMP = $(CROSS)objdump
LINK    = $(CC) $(ARCH) $(LDFLAGS) -o $@ start.o main.o -lgcc
endif

all: firmware.hex

sin_table.h: gen_sin_table.py
	python3 gen_sin_table.py

main.o: main.c hlab.h sin_table.h
	$(CC) $(CFLAGS) -c -o $@ $<

start.o: start.S
	$(CC) $(ARCH) -c -o $@ $<

firmware.elf: start.o main.o link.ld
	$(LINK)

firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $< $@

firmware.hex: firmware.bin mkhex.py
	python3 mkhex.py $< $@ 4096

dis: firmware.elf
	$(OBJDUMP) -d $< > firmware.dis

clean:
	rm -f *.o firmware.elf firmware.bin firmware.hex firmware.dis

.PHONY: all dis clean
