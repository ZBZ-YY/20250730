################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/SCIENCE/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/Moon/workspace_ccstheia/adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang" -I"C:/Users/Moon/workspace_ccstheia/adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang/Debug" -I"D:/SCIENCE/CCS/mspm0_sdk_2_05_01_00/source/third_party/CMSIS/Core/Include" -I"D:/SCIENCE/CCS/mspm0_sdk_2_05_01_00/source" -I"C:/Users/Moon/workspace_ccstheia/adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang/User" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-324184289: ../adc12_triggered_by_timer_event.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"D:/SCIENCE/CCS/ccs/utils/sysconfig_1.24.0/sysconfig_cli.bat" --script "C:/Users/Moon/workspace_ccstheia/adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang/adc12_triggered_by_timer_event.syscfg" -o "." -s "D:/SCIENCE/CCS/mspm0_sdk_2_05_01_00/.metadata/product.json" -s "D:/SCIENCE/CCS/mspm0_sdk_2_05_01_00/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-324184289 ../adc12_triggered_by_timer_event.syscfg
device.opt: build-324184289
device.cmd.genlibs: build-324184289
ti_msp_dl_config.c: build-324184289
ti_msp_dl_config.h: build-324184289
Event.dot: build-324184289

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/SCIENCE/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/Moon/workspace_ccstheia/adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang" -I"C:/Users/Moon/workspace_ccstheia/adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang/Debug" -I"D:/SCIENCE/CCS/mspm0_sdk_2_05_01_00/source/third_party/CMSIS/Core/Include" -I"D:/SCIENCE/CCS/mspm0_sdk_2_05_01_00/source" -I"C:/Users/Moon/workspace_ccstheia/adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang/User" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/SCIENCE/CCS/mspm0_sdk_2_05_01_00/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/SCIENCE/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/Moon/workspace_ccstheia/adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang" -I"C:/Users/Moon/workspace_ccstheia/adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang/Debug" -I"D:/SCIENCE/CCS/mspm0_sdk_2_05_01_00/source/third_party/CMSIS/Core/Include" -I"D:/SCIENCE/CCS/mspm0_sdk_2_05_01_00/source" -I"C:/Users/Moon/workspace_ccstheia/adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang/User" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


