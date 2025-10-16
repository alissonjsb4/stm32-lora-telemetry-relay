#!/usr/bin/env bash
#
# Script unificado para desproteger, gravar e resetar a RS41
#
make clean
make
/home/danilo-alencar/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI -c port=SWD Mode=UR Reset=HWrst -ob RDP=0xAA WRP=0xFFFF
openocd -f ./openocd_rs41.cfg -c "init; halt; flash protect 0 0 30 off; program RS41HUP.elf verify reset; shutdown"
