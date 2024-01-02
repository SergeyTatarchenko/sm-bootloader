/**
 * @file bootloader.h
 *
 * @brief header for bootloader.c
 *
 * @author Siarhei Tatarchanka
 * Contact: zlojdigger@gmail.com
 *
 */
#ifndef BOOTLOADER_H
#define BOOTLOADER_H

/* Includes-------------------------------------------------------------------*/
#include <stdint.h>
/* Global typedef-------------------------------------------------------------*/
typedef enum 
{
    STAT_UNKNOWN, // unknown status of bootloader FSM
    STAT_READY,   // normal state, bootloader is waiting for commands
    STAT_BUSY,    // state for app update process
    STAT_ERROR    // error state

}PROCESS_STAT_TypeDef;

/* Public function prototypes ------------------------------------------------*/
extern PROCESS_STAT_TypeDef boot_get_stat         (void);
extern void                 boot_set_stat         (const PROCESS_STAT_TypeDef new_stat);
// functions that can be overridden
extern uint32_t             boot_crc32            (const void *buf, size_t size);
// functions that need to be overridden
extern void                 boot_hw_init          (void);
extern void                 boot_hw_deinit        (void);
extern void                 boot_page_erase       (const uint32_t page_start_addr);
extern bool                 boot_check_stay_in    (void);
extern void                 boot_stay_in_request  (void);
extern void                 boot_stay_in_reset    (void);
extern bool                 boot_flash_write_block(const uint32_t addr,
                                                   const uint8_t* p_data,
                                                   const uint32_t size);
#endif /*BOOTLOADER_H*/

/******************************* end of file **********************************/
