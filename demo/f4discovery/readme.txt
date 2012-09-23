FOR THE STM32F4DISCOVERY BOARD USE THIS CONFIGURATION:

+ device_select.h:    Uncomment STM32F4XX
+ Linker script:      stm32f407vg.ld
+ OpenOCD interface:  stlink-v2.cfg
+ OpenOCD target:     stm32f4x_stlink.cfg
+ GDB script:         stm32f407vg.script or any of these:
                      stm32f4_16kb.script,
                      stm32f4_32kb.script,
                      etc
