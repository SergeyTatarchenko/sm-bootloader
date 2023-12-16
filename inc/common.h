/**
 * @file common.h
 *
 * @brief common definitions for Modbus server
 *
 * @author Siarhei Tatarchanka
 * Contact: zlojdigger@gmail.com
 *
 */
#ifndef COMMON_H
#define COMMON_H

/* Includes-------------------------------------------------------------------*/
#include <stdint.h>
/* Global define--------------------------------------------------------------*/

/*============================================================================*/
// quantity of members in ManagerCommands
#define NUM_OF_COM_FUNCTIONS                                                   4
/*============================================================================*/
// id for file with target firmware
#define APP_FILE_ID                                          (uint16_t)(0x0001U)
// id for file with target info
#define APP_INFO_ID                                          (uint16_t)(0x0002U)
/*============================================================================*/
// APP control register address, access R/W 
// return 0x0000 in case of reading
// only one bit can le selected at the same time
#define REG_APP_CONTROL                                      (uint16_t)(0x0000U)
// prepare for reading from file 
#define REG_APP_FILE_PR                                      (uint16_t)(0x0001U)
// prepare for writing to file
#define REG_APP_FILE_PW                                      (uint16_t)(0x0002U)
/*============================================================================*/
// APP size register, access R/W
// contains actual application size in records
// APP size in bytes = number of records * data in REG_BOOT_BLOCK_SIZE
// locked for writing until the application file is removed from memory 
#define REG_APP_SIZE                                         (uint16_t)(0x0001U)
/*============================================================================*/
// APP erase register address, access R/W 
// return 0x0000 in case of reading
#define REG_APP_ERASE                                        (uint16_t)(0x0002U)
// request for erase file with APP_FILE_ID
#define REG_APP_ERASE_REQ                                    (uint16_t)(0x0001U)
/*============================================================================*/
// APP start register, access R/W 
// return 0x0000 in case of reading
#define REG_APP_START                                        (uint16_t)(0x0003U)
// request for bootloader to start the application (if presents)
#define REG_APP_START_REQ                                    (uint16_t)(0x0001U)
/*============================================================================*/
// BOOT control register, access R/W
#define REG_BOOT_CONTROL                                     (uint16_t)(0x0004U)
// request for stay in boot(1 = stay in boot, 0 = load application)
#define REG_BOOT_CONTROL_SIB                                 (uint16_t)(0x0001U)
/*============================================================================*/
// BOOT status register, access R 
//(read only, will return COM_EXCEPTION_4 in case of writing)
#define REG_BOOT_STATUS                                      (uint16_t)(0x0005U)
/*============================================================================*/
// BOOT block size register, access R 
//(read only, will return COM_EXCEPTION_4 in case of writing)
#define REG_BOOT_BLOCK_SIZE                                  (uint16_t)(0x0006U)
/*============================================================================*/
#define REGS_AVAILABLE                                                        7
/*============================================================================*/

/*
                                APP install:
1) start conditions:
    a) REG_BOOT_STATUS contains BT_EMPTY value;
    b) REG_APP_SIZE contains 0x0000U value
    if initial values ​​do not match -> perform APP erase
2) write number of records (N) into REG_APP_SIZE register;
    N = (APP size in bytes)/REG_BOOT_BLOCK_SIZE + 1
3) split APP image into records and write them to file with id APP_FILE_ID

                                APP erase:
1) write REG_APP_ERASE_REQ value into REG_APP_ERASE register
*/

/*============================================================================*/
//***********************general Modbus constants*****************************//
// IBM CRC16
#define CRC_SIZE                                                              2u
// address and function length (common start)
#define COM_ADDR_BYTE_SIZE                                                    1u
#define COM_FUNC_BYTE_SIZE                                                    1u
//**********************MODBUS RTU configuration******************************//
//
// start and stop 4 bytes with zeros
#define COM_START_SEQ_SIZE                                                    4u
#define COM_STOP_SEQ_SIZE                                                     4u
#define COM_RTU_PACKAGE_EDGE            (COM_START_SEQ_SIZE + COM_STOP_SEQ_SIZE)
/*============================================================================*/
#define COM_DATA_START_POS (COM_START_SEQ_SIZE + \
                            COM_ADDR_BYTE_SIZE + COM_FUNC_BYTE_SIZE)

#define COM_ADU_REQUIRED_PART (COM_RTU_PACKAGE_EDGE + \
                               CRC_SIZE + COM_ADDR_BYTE_SIZE)

/* Global typedef-------------------------------------------------------------*/

// General Modbus function codes, for reference see https://modbus.org/ 
enum ManagerCommands 
{
    COM_UNDEFINED  = 0xFFU, // illegal function code
    COM_READ_REGS  = 0x03U, // read holding registers
    COM_WRITE_REG  = 0x06U, // write single register
    COM_READ_FILE  = 0x14U, // read file records
    COM_WRITE_FILE = 0x15U  // write file records 
};
// General Modbus error codes, for reference see https://modbus.org/ 
enum ServerErrors 
{
    ERROR_READ_REGS  = 0x83U, // read holding registers error (0x80 | COM_READ_REGS   )
    ERROR_WRITE_REG  = 0x86U, // write single register error  (0x80 | ERROR_WRITE_REG )
    ERROR_READ_FILE  = 0x94U, // read file records error      (0x80 | ERROR_READ_FILE )
    ERROR_WRITE_FILE = 0x95U  // write file records error     (0x80 | ERROR_WRITE_FILE)
};
// General Modbus exception codes, for reference see https://modbus.org/ 
enum ServerExceptions 
{
    COM_EXCEPTION_1  = 0x01U,
    COM_EXCEPTION_2  = 0x02U,
    COM_EXCEPTION_3  = 0x03U,
    COM_EXCEPTION_4  = 0x04U
};
// bootloader status description
enum BootloaderStatus
{
    BT_UNKNOWN,
    BT_EMPTY,
    BT_APP_READY,
    BT_ERROR
};

#pragma pack(push)
#pragma pack(2)
// struct with file for actual bootloader status
typedef struct 
{
    // array with actual bootloader version in ASCII (max 16 chars + 1 end symbol)
    char boot_version[17];
    // array with bootloader name in ASCII (max 32 chars + 1 end symbol)
    char boot_name[33];
    // how many bytes we have for app
    uint32_t available_rom; 
}BootloaderInfo_TypeDef;
#pragma  pack(pop)

#endif // COMMON_H
