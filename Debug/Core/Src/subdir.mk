################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FusionAhrs.c \
../Core/Src/FusionCompass.c \
../Core/Src/FusionOffset.c \
../Core/Src/adxl362_hal.c \
../Core/Src/lis2dw12_reg.c \
../Core/Src/lsm6dsox_reg.c \
../Core/Src/lsm6dsv_reg.c \
../Core/Src/main.c \
../Core/Src/stm32u5xx_hal_msp.c \
../Core/Src/stm32u5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32u5xx.c 

OBJS += \
./Core/Src/FusionAhrs.o \
./Core/Src/FusionCompass.o \
./Core/Src/FusionOffset.o \
./Core/Src/adxl362_hal.o \
./Core/Src/lis2dw12_reg.o \
./Core/Src/lsm6dsox_reg.o \
./Core/Src/lsm6dsv_reg.o \
./Core/Src/main.o \
./Core/Src/stm32u5xx_hal_msp.o \
./Core/Src/stm32u5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32u5xx.o 

C_DEPS += \
./Core/Src/FusionAhrs.d \
./Core/Src/FusionCompass.d \
./Core/Src/FusionOffset.d \
./Core/Src/adxl362_hal.d \
./Core/Src/lis2dw12_reg.d \
./Core/Src/lsm6dsox_reg.d \
./Core/Src/lsm6dsv_reg.d \
./Core/Src/main.d \
./Core/Src/stm32u5xx_hal_msp.d \
./Core/Src/stm32u5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32u5xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32U5A5xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32U5xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/FusionAhrs.cyclo ./Core/Src/FusionAhrs.d ./Core/Src/FusionAhrs.o ./Core/Src/FusionAhrs.su ./Core/Src/FusionCompass.cyclo ./Core/Src/FusionCompass.d ./Core/Src/FusionCompass.o ./Core/Src/FusionCompass.su ./Core/Src/FusionOffset.cyclo ./Core/Src/FusionOffset.d ./Core/Src/FusionOffset.o ./Core/Src/FusionOffset.su ./Core/Src/adxl362_hal.cyclo ./Core/Src/adxl362_hal.d ./Core/Src/adxl362_hal.o ./Core/Src/adxl362_hal.su ./Core/Src/lis2dw12_reg.cyclo ./Core/Src/lis2dw12_reg.d ./Core/Src/lis2dw12_reg.o ./Core/Src/lis2dw12_reg.su ./Core/Src/lsm6dsox_reg.cyclo ./Core/Src/lsm6dsox_reg.d ./Core/Src/lsm6dsox_reg.o ./Core/Src/lsm6dsox_reg.su ./Core/Src/lsm6dsv_reg.cyclo ./Core/Src/lsm6dsv_reg.d ./Core/Src/lsm6dsv_reg.o ./Core/Src/lsm6dsv_reg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32u5xx_hal_msp.cyclo ./Core/Src/stm32u5xx_hal_msp.d ./Core/Src/stm32u5xx_hal_msp.o ./Core/Src/stm32u5xx_hal_msp.su ./Core/Src/stm32u5xx_it.cyclo ./Core/Src/stm32u5xx_it.d ./Core/Src/stm32u5xx_it.o ./Core/Src/stm32u5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32u5xx.cyclo ./Core/Src/system_stm32u5xx.d ./Core/Src/system_stm32u5xx.o ./Core/Src/system_stm32u5xx.su

.PHONY: clean-Core-2f-Src

