cmake_minimum_required (VERSION 3.15)

set(CMAKE_SYSTEM_NAME Generic)

#cross-compile for ARM cpu
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})

#set to 1 because cross compiler will fail cmake compiler tests
set(CMAKE_C_COMPILER_WORKS   1)
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_ASM_COMPILER_WORKS 1)

project (32f0308discovery-demo)
#without enable ASM language cmake will not pass asm file to compiler
enable_language(C ASM)

set (DIR_SRCS
        system_stm32f0xx.c
        startup_stm32f0xx.s
        hw_init.c
    )

set(EXECUTABLE ${PROJECT_NAME}.elf)
add_executable (${EXECUTABLE} ${DIR_SRCS})

#special flags
set(GCC_TARGET_COMPILE_OPTIONS "-mcpu=cortex-m0;-mthumb;-mlittle-endian;")
#path to rtos portable sources (WARNING! path shall be able to find from bootloader folder)
set(RTOS_PORTABLE_SRC rtos/portable/ARM_CM0/port.c)
#path to rtos portable include dir (WARNING! path shall be able to find from bootloader folder)
set(RTOS_PORTABLE_INCLUDE_DIR rtos/portable/ARM_CM0)
#path to  dir with target config (WARNING! path shall be able to find from bootloader folder)
set(TARGET_CONFIG_INCLUDE_DIR ../config/32F0308DISCOVERY)

#path bootloader library to project and extract common compiler flags
add_subdirectory(../../src boot-common)
get_directory_property(GCC_COMMON_COMPILE_OPTIONS DIRECTORY ../../src DEFINITION GCC_COMMON_COMPILE_OPTIONS)

target_link_directories(${EXECUTABLE} PUBLIC ../../src)
target_link_libraries (${EXECUTABLE} sm_bootloader)

set(LINKER_SCRIPT  ${CMAKE_CURRENT_SOURCE_DIR}/STM32F030R8_FLASH.ld) 
set(LINKER_OPTIONS "-mcpu=cortex-m0;-mthumb;-mlittle-endian;--specs=nano.specs;-Wl,--gc-sections;")

#common inclide path
target_include_directories(${EXECUTABLE} PRIVATE
        ./include
        ../../src/rtos/include
        ../../inc
        ../../inc/core/Include
        ../../src/${RTOS_PORTABLE_INCLUDE_DIR}
        ../${TARGET_CONFIG_INCLUDE_DIR}     
        )

target_compile_options(${EXECUTABLE} PRIVATE ${GCC_COMMON_COMPILE_OPTIONS})
target_compile_options(${EXECUTABLE} PRIVATE ${GCC_TARGET_COMPILE_OPTIONS})

target_link_options(${EXECUTABLE} PRIVATE -T${LINKER_SCRIPT})
target_link_options(${EXECUTABLE} PRIVATE ${LINKER_OPTIONS})
