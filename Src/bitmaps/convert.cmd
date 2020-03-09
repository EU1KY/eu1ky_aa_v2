@rem arm-none-eabi-objcopy -I binary -O elf32-littlearm -B arm ro.bmp ro.o
@rem arm-none-eabi-objdump -t ro.o
@rem extern uint8_t ro[]      asm("_binary_foo_data_bin_start");
@rem extern uint8_t ro_size[] asm("_binary_foo_data_bin_size");
@rem extern uint8_t foo_data_end[]  asm("_binary_foo_data_bin_end");
start cmd /k bmp2h.py
