/**
 * @file com.h
 *
 * @brief header for com.c
 *
 * @author Siarhei Tatarchanka
 * Contact: zlojdigger@gmail.com
 *
 */
#ifndef COM_H
#define COM_H

/* Includes-------------------------------------------------------------------*/
#include <stdbool.h>
#include "common.h"
#include "boot_cfg.h"
/* Public typedef ------------------------------------------------------------*/

typedef void (* COM_HANDLER)(void *context);

typedef struct
{
    uint8_t func;
    COM_HANDLER handler;
    
}COM_MODBUS_HANDLER_TypeDef;

typedef struct
{
    uint8_t func;
    uint8_t* p_data;
    uint8_t length;

}COM_MODBUS_SERVICE_TypeDef;

/* Global define--------------------------------------------------------------*/

/*
buffer shall have size the same as length of maximum expected message
1) 1 byte for function
2) 1 byte for data length in packet
3) 1 byte for reference byte
4) 2 bytes for file number
5) 2 bytes for record number
6) 2 bytes for record length
    conclusion : 9 additional bytes in total
*/
#define COM_SERVICE_MAX_SIZE                                                  9
/*
initial expected message length (1 additional byte for function + 4 data bytes)
*/
#define COM_SERVICE_MIN_SIZE                                                  5

#define COM_BUFFER_SIZE (COM_SERVICE_MAX_SIZE + \
                         DATA_BLOCK_SIZE + \
                         COM_ADU_REQUIRED_PART)
#define COM_INIT_SIZE             (COM_SERVICE_MIN_SIZE + COM_ADU_REQUIRED_PART)
#define COM_FILE_RW_REF                                         (uint8_t)(0x06U)
/* Public function prototypes ------------------------------------------------*/
extern void     com_init                   (void);
extern uint16_t com_CRC16_u16              (const uint8_t* data, uint16_t length);
extern bool     com_cmd_bind               (const COM_MODBUS_HANDLER_TypeDef* p_cmd);
extern void     com_set_rec_data_length    (const uint8_t length);
extern void     com_generate_callback      (const COM_MODBUS_SERVICE_TypeDef* service);
extern void     com_response_error         (const uint8_t error,const uint8_t exception);
// functions for hw part
extern void     com_rec_byte               (const uint8_t byte);
// functions that need to be overridden
extern void     com_send                   (const uint8_t* p_data, const uint32_t size);
extern bool     com_is_data_sent           (void);

#endif /*COM_H*/

/******************************* end of file **********************************/
