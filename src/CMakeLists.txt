cmake_minimum_required (VERSION 3.15)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

project (sm_bootloader)

#special compiler flags, like CPU/FPU type configuration  
if(NOT DEFINED GCC_TARGET_COMPILE_OPTIONS)
        message( SEND_ERROR "target compiler flags not defined.")
endif()
#path to directory with portable includes for RTOS configuration
if(NOT DEFINED RTOS_PORTABLE_INCLUDE_DIR)
        message( SEND_ERROR "FreeRTOS target platform not defined.")
endif()
#path to portable sources for RTOS configuration
if(NOT DEFINED RTOS_PORTABLE_SRC)
        message( SEND_ERROR "FreeRTOS target platform not defined.")
endif()

#path to directory with target configuration (target_cfg.h and FreeRTOSConfig.h)
if(NOT DEFINED TARGET_CONFIG_INCLUDE_DIR)
        message( SEND_ERROR "target configuration not found.")
endif()

set(BOOTLOADER_SRC
        bootloader.c
        com.c
)


set(RTOS_SRC
    rtos/heap_1.c
    rtos/list.c
    rtos/queue.c
    rtos/tasks.c
)

set(GCC_COMMON_COMPILE_OPTIONS "-fdata-sections;-ffunction-sections;-O1;-Wall;-g;")

add_library (${PROJECT_NAME} STATIC ${RTOS_SRC} ${RTOS_PORTABLE_SRC} ${BOOTLOADER_SRC})

target_include_directories(${PROJECT_NAME} PRIVATE
        ../inc
        ../inc/core/Include
        rtos/include
        ${RTOS_PORTABLE_INCLUDE_DIR}
        ${TARGET_CONFIG_INCLUDE_DIR}
        )

target_compile_options(${PROJECT_NAME} PRIVATE ${GCC_COMMON_COMPILE_OPTIONS})
target_compile_options(${PROJECT_NAME} PRIVATE ${GCC_TARGET_COMPILE_OPTIONS})
