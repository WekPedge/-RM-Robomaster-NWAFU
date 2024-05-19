CFLAGS := -c --apcs=interwork --cpu Cortex-M4.fp --li --c99 -D__MICROLIB -O0 --split_sections --diag_suppress=1 --diag_suppress=1295 -g -I.\..\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc\Legacy -I.\..\Drivers\CMSIS\Include -I.\..\Drivers\CMSIS\Device\ST\STM32F4xx\Include -I.\..\eMPL -I.\..\HARDWARE\IIC -I.\..\HARDWARE\OLED -I.\..\Judgement\JUDGEMENT -I.\..\Judgement\DATAFIFO -I.\..\Judgement\Heat -I.\..\Judgement\PROTICOL -I.\.cmsis\dsp_lib -I.\.cmsis\include -I.\RTE\_HERO_2017_2 -I.\.eide\deps -DUSE_HAL_DRIVER -DSTM32F427xx
CXXFLAGS := -c --cpp --apcs=interwork --cpu Cortex-M4.fp --li -D__MICROLIB -O0 --split_sections --diag_suppress=1 --diag_suppress=1295 -g -I.\..\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc\Legacy -I.\..\Drivers\CMSIS\Include -I.\..\Drivers\CMSIS\Device\ST\STM32F4xx\Include -I.\..\eMPL -I.\..\HARDWARE\IIC -I.\..\HARDWARE\OLED -I.\..\Judgement\JUDGEMENT -I.\..\Judgement\DATAFIFO -I.\..\Judgement\Heat -I.\..\Judgement\PROTICOL -I.\.cmsis\dsp_lib -I.\.cmsis\include -I.\RTE\_HERO_2017_2 -I.\.eide\deps -DUSE_HAL_DRIVER -DSTM32F427xx
ASMFLAGS := --apcs=interwork --cpu Cortex-M4.fp --li --pd "__MICROLIB SETA 1" -g -I.\..\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc\Legacy -I.\..\Drivers\CMSIS\Include -I.\..\Drivers\CMSIS\Device\ST\STM32F4xx\Include -I.\..\eMPL -I.\..\HARDWARE\IIC -I.\..\HARDWARE\OLED -I.\..\Judgement\JUDGEMENT -I.\..\Judgement\DATAFIFO -I.\..\Judgement\Heat -I.\..\Judgement\PROTICOL -I.\.cmsis\dsp_lib -I.\.cmsis\include -I.\RTE\_HERO_2017_2 -I.\.eide\deps
LDFLAGS := --cpu Cortex-M4.fp --library_type=microlib --scatter "c:/Users/w's'j/Desktop/RM现任代码（2022）/步兵3 V1.1/步兵3 V1.1/MDK-ARM/build/HERO_2017_2/MDK-ARM.sct" --strict --summary_stderr --info summarysizes --map --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --list .\build\HERO_2017_2\MDK-ARM.map
LDLIBS := 
