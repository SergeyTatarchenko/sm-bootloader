/**
 * @file target_chg.h
 *
 * @brief config for 32F0308DISCOVERY
 *
 * @author Siarhei Tatarchanka
 * Contact: zlojdigger@gmail.com
 *
 */

#ifndef TARGET_CFG_H
#define TARGET_CFG_H

/* Includes-------------------------------------------------------------------*/

/* Global define--------------------------------------------------------------*/


#define FLASH_START                                      (uint32_t)(0x08000000U)
#define FLASH_SIZE                                          (uint32_t)(0x10000U)
#define RAM_SIZE                                             (uint32_t)(0x2000U)
#define FLASH_PAGE_SIZE                                       (uint32_t)(0x400U)
#define DATA_BLOCK_SIZE                                         (uint8_t)(0x40U)
#define TARGET_ADDRESS                                          (uint8_t)(0x01U) 

// name should not exceed 32 bytes
#define TARGET_NAME                                            "32F0308DISCOVERY"

#endif /*TARGET_CFG_H*/

/******************************* end of file **********************************/
