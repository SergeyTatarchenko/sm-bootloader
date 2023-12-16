cmake_minimum_required (VERSION 3.15)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

project (bootloader_lib)

option(ARM_CM0_CORE "build for Cortex-M0 architecture" OFF)

if(ARM_CM0_CORE)
        
        if(NOT DEFINED GCC_TARGET_COMPILE_OPTIONS)
                set(GCC_TARGET_COMPILE_OPTIONS "-mcpu=cortex-m0;-mthumb;-mlittle-endian;")
        else()
        message( SEND_ERROR "target platform already defined")
        endif()
        
        if(NOT DEFINED PORTABLE_SRC)
                set(PORTABLE_SRC
                rtos/portable/ARM_CM0/port.c
                )
        else()
        message( SEND_ERROR "target platform already defined")
        endif()
        
        if(NOT DEFINED PORTABLE_INCLUDE_DIR)
                set(PORTABLE_INCLUDE_DIR rtos/portable/ARM_CM0)
        else()
                message( SEND_ERROR "target platform already defined")
        endif()

endif()

set(RTOS_SRC
    rtos/event_groups.c
    rtos/heap_1.c
    rtos/list.c
    rtos/queue.c
    rtos/stream_buffer.c
    rtos/tasks.c
    rtos/timers.c
)

set(BOOTLOADER_SRC
        bootloader.c
        com.c
)

set(GCC_COMMON_COMPILE_OPTIONS "-fdata-sections;-ffunction-sections;-O1;-Wall;-g;")

add_library (${PROJECT_NAME} STATIC ${RTOS_SRC} ${PORTABLE_SRC} ${BOOTLOADER_SRC})

target_include_directories(${PROJECT_NAME} PRIVATE
        ../
        ../inc
        ../inc/core/Include
        rtos/include
        ${PORTABLE_INCLUDE_DIR}
        )

target_compile_options(${PROJECT_NAME} PRIVATE ${GCC_COMMON_COMPILE_OPTIONS})
target_compile_options(${PROJECT_NAME} PRIVATE ${GCC_TARGET_COMPILE_OPTIONS})
        
