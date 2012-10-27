FOR THE STM32VLDISCOVERY BOARD USE THIS CONFIGURATION:

+ device_select.h:    Uncomment STM32F1XX and VALUE_LINE
+ Linker script:      stm32f100rb.ld
+ OpenOCD interface:  stlink-v1.cfg
+ OpenOCD target:     stm32f1x_stlink.cfg
+ GDB script:         stm32vldiscovery.script**

** The flash protection of the device must be disabled or this configuration
   won't work.