/**
 * @file boot_cfg.h
 *
 * @brief header of the main bootloader configurator, this file use hw specific
 * info from target_cfg.h 
 *
 * @author Siarhei Tatarchanka
 * Contact: zlojdigger@gmail.com
 *
 */

#ifndef BOOT_CFG_H
#define BOOT_CFG_H

/* Includes-------------------------------------------------------------------*/
#include "target_cfg.h"
/* Global define--------------------------------------------------------------*/
//should not exceed 16 bytes
#define BOOT_VERSION                                                     "0.01a"

#if !defined(FLASH_START)
#error  flash start address must be defined
#endif
#if !defined(FLASH_SIZE) && !defined(RAM_SIZE)
#error  flash and ram sizes must be defined
#endif
#if !defined(FLASH_PAGE_SIZE)
#error flash page size must be defined
#endif
#define BOOT_START                                                   FLASH_START
//total offset for bootloader - 12 KB
#define BOOT_OFFSET                                           (uint32_t)(0x3000) 
#define APP_MINIMAL_START                             (BOOT_START + BOOT_OFFSET) 
#if !defined(APP_START)
#define APP_START                                              APP_MINIMAL_START
#endif
// default address for bootloader in Modbus network
#if !defined(TARGET_ADDRESS)
#define TARGET_ADDRESS                                          (uint8_t)(0x01U)
#endif
// default block size (same as record length) for Modbus network 
#if !defined(DATA_BLOCK_SIZE)
#define DATA_BLOCK_SIZE                                         (uint8_t)(0x40U)
#endif

// Schematic for general memory configuration in MCU:
/*
       FLASH
###################
       BOOT       |
------------------|
     app info     |
------------------|
                  |
       APP        |
                  |
###################
*/
// (1) BOOT - flash memory area with bootloader size is 12KB, 
// can be changed if this size is not a multiple of flash page size;
// (2) app info - area with struct with app information, its size is 8 bytes, 
// this area always starts from the new flash page;
// (3) APP - area with application;

#endif /*BOOT_CFG_H*/
/******************************* end of file **********************************/
