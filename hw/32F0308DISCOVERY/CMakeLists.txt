cmake_minimum_required (VERSION 3.15)

set(CMAKE_SYSTEM_NAME Generic)

#cross-compile for ARM cpu
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})

#set to 1 because cross compiler will fail cmake compiler tests
set(CMAKE_C_COMPILER_WORKS   1)
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_ASM_COMPILER_WORKS 1)

project (test_bootloader)
#without enable ASM language cmake will not pass asm file to compiler
enable_language(C ASM)

set (DIR_SRCS
        hw/32F0308DISCOVERY/system_stm32f0xx.c
        hw/32F0308DISCOVERY/startup_stm32f0xx.s
        hw/32F0308DISCOVERY/hw_init.c
    )

set(EXECUTABLE ${PROJECT_NAME}.elf)
add_executable (${EXECUTABLE} ${DIR_SRCS})

#path bootloader library to project and extract compiler flags
add_subdirectory(src)
get_directory_property(PORTABLE_PATH DIRECTORY src DEFINITION PORTABLE_INCLUDE_DIR)
get_directory_property(TARGET_COMPILE_OPTIONS DIRECTORY src DEFINITION GCC_TARGET_COMPILE_OPTIONS)
get_directory_property(COMMON_COMPILE_OPTIONS DIRECTORY src DEFINITION GCC_COMMON_COMPILE_OPTIONS)

target_link_directories(${EXECUTABLE} PUBLIC src)
target_link_libraries (${EXECUTABLE} bootloader_lib)

set(LINKER_SCRIPT  hw/32F0308DISCOVERY/STM32F030R8_FLASH.ld) 
set(LINKER_OPTIONS "-mcpu=cortex-m0;-mthumb;-mlittle-endian;--specs=nano.specs;-Wl,--gc-sections;")

#common inclide path
target_include_directories(${EXECUTABLE} PRIVATE
        ./
        inc
        inc/core/Include
        src/${PORTABLE_PATH}
        src/rtos/include        
        )
#hw spec path
target_include_directories(${EXECUTABLE} PRIVATE
        hw/32F0308DISCOVERY/include
        )

target_compile_options(${EXECUTABLE} PRIVATE ${COMMON_COMPILE_OPTIONS})
target_compile_options(${EXECUTABLE} PRIVATE ${TARGET_COMPILE_OPTIONS})

target_link_options(${EXECUTABLE} PRIVATE -T${LINKER_SCRIPT})
target_link_options(${EXECUTABLE} PRIVATE ${LINKER_OPTIONS})
