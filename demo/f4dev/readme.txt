FOR THE F4DEV BOARD USE THIS CONFIGURATION:

+ device_select.h:    Uncomment STM32F4XX
+ Linker script:      stm32f407ve.ld
+ OpenOCD interface:  ujtag.cfg
+ OpenOCD target:     stm32f4x.cfg
+ GDB script:         stm32f407ve.script or any of these:
                      stm32f4_16kb.script,
                      stm32f4_32kb.script,
                      etc
