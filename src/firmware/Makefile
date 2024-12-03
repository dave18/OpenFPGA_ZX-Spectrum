OBJS = main.o start.o

all: firmware.hex

%.o: %.c
	clang --target=riscv32 -march=rv32imc -std=c99 -O3 -Os -fno-inline -mno-relax -Wall -Werror -c $< -o $@

%.o: %.S
	clang --target=riscv32 -march=rv32imc -mno-relax -Wall -Werror -c $< -o $@

firmware.elf: $(OBJS) system.ld
	ld.lld -T system.ld $(OBJS) -o $@ -M

firmware.bin: firmware.elf
	llvm-objcopy --keep-section=.text --keep-section=.data --output-target=binary $< $@

firmware.hex: firmware.bin
	hexdump -v -e  '4/4 "%08x " "\n"' $< > $@

clean:
	rm -f $(OBJS) firmware.elf firmware.bin firmware.vh
