/**
 * @file bootloader.c
 *
 * @brief implementation of bootloader logic and Modbus protocol functions
 *
 * @author Siarhei Tatarchanka
 * Contact: zlojdigger@gmail.com
 *
 */

/* Includes-------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rtos.h"
#include "com.h"
#include "bootloader.h"
#include "cmsis_gcc.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint32_t  app_size; // size in bytes
    uint32_t  app_crc;  // CRC-32-IEEE 802.3

}APP_IMAGE_INFO_TypeDef;

typedef struct
{
    uint16_t num_of_records;
    uint32_t file_size;
    void*    p_data;
    uint16_t counter;

}FILE_CONTROL_TypeDef;

typedef struct
{
    bool stay_in_boot;
    bool app_update_request;
    bool hw_configured;

}BOOT_CONTROL_TypeDef;
/* Private function prototypes -----------------------------------------------*/
static void     boot_init        (void);
static bool     initDataValidate (void);
static void     appEraseMemory   (void);
static bool     appImageVerify   (void);
static void     applicationStart (void);
static bool     appProgramMemory (const uint32_t addr, const uint8_t* p_data, const uint32_t size);
static void     jumpToApplication(const uint32_t new_msp, const uint32_t addr);
static void     fileSetup        (const uint16_t file_id, void* p_file, const uint32_t size);
static void     fileProcUpdate   (const uint16_t file_id);
static bool     commandBind      (COM_HANDLER handler, const uint8_t cmd);
static uint16_t extractHalfWord  (const uint8_t *const p_data);
/*----------------------------------------------------------------------------*/
static void writeRegisterCmdProc(void* context);
static int  writeRegisterCmdExe (const uint16_t address,const uint16_t value);
static void readRegisterCmdProc (void* context);
static int  readRegisterCmdExe  (const uint16_t address,const uint16_t quantity);
static void readFileCmdProc     (void* context);
static int  readFileCmdExe      (const uint16_t file_id,const uint16_t record_id, 
                                 const uint16_t length);
static void writeFileCmdProc    (void* context);
static int  writeFileCmdExe     (const uint16_t file_id,const uint16_t record_id, 
                                 const uint8_t* p_data, const uint16_t length);
/* Private variables ---------------------------------------------------------*/
static PROCESS_STAT_TypeDef    process_stat;
static APP_IMAGE_INFO_TypeDef* app_image_info_ps;
static BootloaderInfo_TypeDef  boot_info;
static BOOT_CONTROL_TypeDef    boot_control;
static FILE_CONTROL_TypeDef    files       [2];
static uint16_t                control_regs[REGS_AVAILABLE];
static uint8_t                 buffer      [COM_SERVICE_MAX_SIZE + DATA_BLOCK_SIZE];
//constants
static const uint8_t  rw_reg_client_pdu_size = 5;
static const uint8_t  r_file_client_pdu_size = 9;
static const uint8_t  w_file_client_pdu_size = 9 + DATA_BLOCK_SIZE;
static const uint32_t file_app_idx           = 0;
static const uint32_t file_info_idx          = 1;
static const uint32_t __app_start            = APP_START + sizeof(APP_IMAGE_INFO_TypeDef);
static const size_t   num_of_files           = sizeof(files)/sizeof(FILE_CONTROL_TypeDef);

//main entry point
int main(void)
{
    boot_init();
    vTaskStartScheduler();
    for(;;){}
    return 0;
}

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/**
 * @brief This is to provide the memory that is used by the RTOS daemon/time task.
 *
 * If configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetTimerTaskMemory() to provide the memory that is
 * used by the RTOS daemon/time task.
 */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
    /* If the buffers to be provided to the Timer task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/**
 * @brief request for actual stat for external indication
 * 
 * @retval actual process status 
*/
PROCESS_STAT_TypeDef boot_get_stat(void)
{
    return process_stat;
}

/**
 * @brief setup process status from external signal
 * 
 * @param new_stat new status
*/
void boot_set_stat(const PROCESS_STAT_TypeDef new_stat)
{
    process_stat = new_stat;
}

/**
 * @brief bootloader main init function
*/
static void boot_init(void)
{
    (void)memset(&boot_info,0,sizeof(boot_info));
    (void)memset(&boot_control,0,sizeof(boot_control));
    (void)memset(&files[0],0,sizeof(files));
    (void)memset(&control_regs[0],0,sizeof(control_regs));
    (void)memset(&buffer[0],0,sizeof(buffer));
    
    // parameters checking from target_cfg.h
    bool state = initDataValidate(); 
    if(state != true){for(;;){}}
    boot_control.stay_in_boot = boot_check_stay_in();
    state                     = appImageVerify();

    if((state                     == true) || 
       (boot_control.stay_in_boot == true))
    {
        boot_hw_init();
        boot_control.hw_configured = true;
        com_init();
        //(5) init functions
        if(!commandBind(writeRegisterCmdProc,(uint8_t)(COM_WRITE_REG))) {for(;;){}}
        if(!commandBind(readRegisterCmdProc, (uint8_t)(COM_READ_REGS))) {for(;;){}}
        if(!commandBind(readFileCmdProc,     (uint8_t)(COM_READ_FILE))) {for(;;){}}
        if(!commandBind(writeFileCmdProc,    (uint8_t)(COM_WRITE_FILE))){for(;;){}}
    }
    else{applicationStart();}
}

/**
 * @brief initial check of memory parameters
 *
 * @retval TRUE in case of success, false in case of error 
*/
static bool initDataValidate(void)
{
    /*The application address must be located after the bootloader address, 
    taking into account the fact that when clearing memory, only the area with 
    the application should be cleared.*/
    if(APP_START < APP_MINIMAL_START){return false;}
    /*The application must be at the beginning of the memory page*/
    if((APP_START % FLASH_PAGE_SIZE) != 0){return false;}
    /*block size must be a multiple of 4*/
    if((DATA_BLOCK_SIZE % 4) != 0){return false;}

    app_image_info_ps = (APP_IMAGE_INFO_TypeDef*)(APP_START);
    process_stat      = STAT_UNKNOWN;
    return true;
}

/**
 * @brief checking for a valid application in memory
 *
 * @retval TRUE if bootloader will be started, false if we are ready to start application 
*/
static bool appImageVerify(void)
{
    bool state = false;
    
    if(app_image_info_ps != NULL)
    {
        control_regs[REG_BOOT_BLOCK_SIZE] = DATA_BLOCK_SIZE;
        boot_info.available_rom           = (uint32_t)(FLASH_START + FLASH_SIZE - APP_START); 
        (void)snprintf(&boot_info.boot_version[0],sizeof(boot_info.boot_version),BOOT_VERSION);
        #if defined(TARGET_NAME)
        (void)snprintf(&boot_info.boot_name[0],sizeof(boot_info.boot_name),TARGET_NAME);
        #endif
        // setup file struct for APP_INFO_ID
        fileSetup(file_info_idx,&boot_info,sizeof(boot_info));
        //check for application
        if(app_image_info_ps->app_size == (uint32_t)(0xFFFFFFFFU))
        {
            //probably we do not have any application here
            control_regs[REG_BOOT_STATUS] = (uint16_t)(BT_EMPTY);
            fileSetup(file_app_idx,(uint32_t*)(__app_start),0);
            state = true;
        }
        else if((app_image_info_ps->app_size > 0U) && 
                (app_image_info_ps->app_size < boot_info.available_rom))
        {
            //looks like we have some file, call crc check
            uint32_t crc = boot_crc32((uint32_t*)(__app_start),app_image_info_ps->app_size);
            if(crc == app_image_info_ps->app_crc)
            {
                control_regs[REG_BOOT_STATUS] = (uint16_t)(BT_APP_READY);
                process_stat = STAT_READY;
                fileSetup(file_app_idx,(uint32_t*)(__app_start),app_image_info_ps->app_size);
                state = false;
            }
            else
            {
                control_regs[REG_BOOT_STATUS] = (uint16_t)(BT_ERROR);
                process_stat = STAT_ERROR;
                fileSetup(file_app_idx,(uint32_t*)(__app_start),0);
                state = true;    
            }
        }
        else
        {
            // data is corrupted 
            control_regs[REG_BOOT_STATUS] = (uint16_t)(BT_ERROR);
            process_stat = STAT_ERROR;
            fileSetup(file_app_idx,(uint32_t*)(__app_start),0);
            state = true;
        }
    }
    else{for(;;){}}//pointer shall be defined at this stage
    return state;
}

/**
 * @brief erase flash memory reserved for application
 *
*/
static void appEraseMemory(void)
{
    uint32_t counter = boot_info.available_rom/FLASH_PAGE_SIZE;
    uint32_t addr    = APP_START;

    while(counter != 0)
    {
        boot_page_erase(addr);
        addr += FLASH_PAGE_SIZE;
        counter--;
    }
}

/**
 * @brief writing to flash memory
 * 
 * @param addr memory start address
 * @param p_data pointer to data to write
 * @param size data size in bytes (!IMPORTANT must be multiple of 4 bytes)
 * 
 * @retval TRUE in case of success, FALSE in case of failure 
*/
static bool appProgramMemory(const uint32_t addr, const uint8_t *p_data, const uint32_t size)
{
    bool stat;
    if((addr + size) > (__app_start + boot_info.available_rom))
    {
        //we cannot write to non-existent memory
        stat = false;
        control_regs[REG_BOOT_STATUS] = (uint16_t)(BT_ERROR);
        com_set_rec_data_length(COM_INIT_SIZE);
    }
    else
    {
        stat = boot_flash_write_block(addr, p_data, size);
        if(stat == true)
        {
            fileProcUpdate(file_app_idx);
        }
        else
        {
            control_regs[REG_BOOT_STATUS] = (uint16_t)(BT_ERROR);
            com_set_rec_data_length(COM_INIT_SIZE);
        }
    }
    return stat;
}

/**
 * @brief function for implementation logic for COM_READ_FILE state diagram
 * 
 * @param context pointer to context from message handler in com.c
*/
static void readFileCmdProc(void* context)
{
    const uint8_t* p_buffer    = (uint8_t*)context;
    const uint8_t data_length  = r_file_client_pdu_size + COM_ADDR_BYTE_SIZE; 
    const uint8_t crc_idx      = r_file_client_pdu_size + COM_ADDR_BYTE_SIZE + COM_START_SEQ_SIZE;
    const uint8_t byte_cnt_idx = COM_DATA_START_POS;
    const uint8_t file_idx     = COM_DATA_START_POS + 2;
    const uint8_t record_idx   = COM_DATA_START_POS + 4;
    const uint8_t length_idx   = COM_DATA_START_POS + 6;
    
    uint16_t rec_crc    = extractHalfWord(&p_buffer[crc_idx]);
    uint16_t actual_crc = com_CRC16_u16(&p_buffer[COM_START_SEQ_SIZE],data_length);
    
    if(rec_crc == actual_crc)
    {
        uint8_t byte_cnt    = p_buffer[byte_cnt_idx];
        uint8_t ref_type    = p_buffer[byte_cnt_idx + 1];
        uint16_t id_file    = extractHalfWord(&p_buffer[file_idx]);
        uint16_t id_record  = extractHalfWord(&p_buffer[record_idx]);
        uint16_t length     = extractHalfWord(&p_buffer[length_idx]);

        if((ref_type != COM_FILE_RW_REF) || (byte_cnt < (uint8_t)(0x07U)) || (byte_cnt > (uint8_t)(0xF5U)) )
        {
            com_response_error((uint8_t)(ERROR_READ_FILE),(uint8_t)(COM_EXCEPTION_3));
        }
        else
        {
            int res = readFileCmdExe(id_file,id_record,length);
            if(res != 0)
            {
                com_response_error((uint8_t)(ERROR_READ_FILE),(uint8_t)(COM_EXCEPTION_2));        
            }
            else
            {
                //success
                //callback shall be executed in readFileCmdExe
            }
        }
    }
    else
    {
        com_response_error((uint8_t)(ERROR_READ_FILE),(uint8_t)(COM_EXCEPTION_3));
    }   
}

/**
 * @brief command processing for Modbus COM_READ_FILE function 
 * 
 * @param file_id file if for reading
 * @param record_id record id for reading
 * @param length amount of words to read
 * 
 * @retval 0 in case of success, -1 in case of error
*/
static int readFileCmdExe(const uint16_t file_id, const uint16_t record_id, const uint16_t length)
{
    int                        stat;
    COM_MODBUS_SERVICE_TypeDef service;

    if((length > (DATA_BLOCK_SIZE/2)) || (file_id == 0)|| (record_id > (uint16_t)(0x270FU)))
    {
        stat = -1;
    }
    else
    {
        int actual_id;
        switch(file_id)
        {
            case APP_FILE_ID:
                actual_id = file_app_idx;
                break;
            case APP_INFO_ID:
                actual_id = file_info_idx;
                break;
            default:
                actual_id = -1;
                break;
        }
        if(actual_id == -1)
        {
            stat = -1;    
        }
        else
        {
            const uint32_t file_size      = files[actual_id].file_size;
            const uint16_t num_of_records = files[actual_id].num_of_records;
            const uint8_t* p_data         = files[actual_id].p_data;
            if(record_id > (num_of_records -1))
            {
                stat = -1;    
            }
            else
            {
                uint8_t cp_length;
                if((record_id + 1) == num_of_records)
                {
                    //last record
                    if(file_size == DATA_BLOCK_SIZE)
                    {
                        cp_length = DATA_BLOCK_SIZE;
                    }
                    else
                    {
                        cp_length = file_size % DATA_BLOCK_SIZE;
                    }
                }
                else
                {
                    // other record
                    cp_length = DATA_BLOCK_SIZE;
                }
                if(cp_length > (length * 2))
                {
                    stat = -1;                
                }
                else
                {
                    //prepare record
                    buffer[0] = cp_length + 1;
                    buffer[1] = cp_length;
                    buffer[2] = COM_FILE_RW_REF;
                    memcpy(&buffer[3],&p_data[record_id * DATA_BLOCK_SIZE],cp_length);
                    service.func   = COM_READ_FILE;
                    service.p_data = buffer;
                    service.length = cp_length + 3;
                    com_generate_callback(&service);
                    fileProcUpdate(actual_id);
                    stat = 0;
                }
            }
        }
    }
    return stat;
}

/**
 * @brief function for implementation logic for COM_WRITE_FILE state diagram
 * 
 * @param context pointer to context from message handler in com.c
*/
static void writeFileCmdProc(void *context)
{
    const uint8_t* p_buffer    = (uint8_t*)context;
    const uint8_t data_length  = w_file_client_pdu_size + COM_ADDR_BYTE_SIZE; 
    const uint8_t crc_idx      = w_file_client_pdu_size + COM_ADDR_BYTE_SIZE + COM_START_SEQ_SIZE;
    const uint8_t byte_cnt_idx = COM_DATA_START_POS;
    const uint8_t file_idx     = COM_DATA_START_POS + 2;
    const uint8_t record_idx   = COM_DATA_START_POS + 4;
    const uint8_t length_idx   = COM_DATA_START_POS + 6;
    const uint8_t data_idx     = COM_DATA_START_POS + 8;

    uint16_t rec_crc    = extractHalfWord(&p_buffer[crc_idx]);
    uint16_t actual_crc = com_CRC16_u16(&p_buffer[COM_START_SEQ_SIZE],data_length);
    if(rec_crc == actual_crc)
    {
        uint8_t byte_cnt    = p_buffer[byte_cnt_idx];
        uint8_t ref_type    = p_buffer[byte_cnt_idx + 1];
        uint16_t id_file    = extractHalfWord(&p_buffer[file_idx]);
        uint16_t id_record  = extractHalfWord(&p_buffer[record_idx]);
        uint16_t length     = extractHalfWord(&p_buffer[length_idx]);

        if((ref_type != COM_FILE_RW_REF) || (byte_cnt < (uint8_t)(0x07U)) || (byte_cnt > (uint8_t)(0xF5U)) )
        {
            com_response_error((uint8_t)(ERROR_READ_FILE),(uint8_t)(COM_EXCEPTION_3));
        }
        else
        {
            int res = writeFileCmdExe(id_file,id_record,&p_buffer[data_idx],length);
            
            if(res != 0)
            {
                com_response_error((uint8_t)(ERROR_READ_FILE),(uint8_t)(COM_EXCEPTION_2));        
            }
            else
            {
                //success
                com_generate_callback(NULL);
            }
        }
    }
    else
    {
        com_response_error((uint8_t)(ERROR_READ_FILE),(uint8_t)(COM_EXCEPTION_3));
    }
}

/**
 * @brief command processing for Modbus COM_WRITE_FILE function 
 * 
 * @param file_id file if for reading
 * @param record_id record id for reading
 * @param p_data pointer to data to write
 * @param length amount of words to write
 * 
 * @retval 0 in case of success, -1 in case of error
*/
static int writeFileCmdExe(const uint16_t file_id, const uint16_t record_id, const uint8_t *p_data, const uint16_t length)
{
    int stat;

    if((length > (DATA_BLOCK_SIZE/2)) || (file_id == 0)|| (record_id > (uint16_t)(0x270FU)))
    {
        stat = -1;
    }
    else
    {
        //logic for APP_FILE_ID
        uint32_t address = __app_start + (record_id * DATA_BLOCK_SIZE);
        uint32_t n_bytes = length * 2;
        if(appProgramMemory(address,p_data,n_bytes) == true){stat = 0;}
        else{stat = -1;}
    }
    return stat;
}

/**
 * @brief function for implementation logic for COM_WRITE_REG state diagram
 * 
 * @param context pointer to context from message handler in com.c
*/
static void writeRegisterCmdProc(void* context)
{
    const uint8_t* p_buffer    = (uint8_t*)context;
    const uint8_t data_length  = rw_reg_client_pdu_size + COM_ADDR_BYTE_SIZE; 
    const uint8_t crc_idx      = rw_reg_client_pdu_size + COM_ADDR_BYTE_SIZE + COM_START_SEQ_SIZE;
    const uint8_t address_idx  = COM_DATA_START_POS;
    const uint8_t value_idx    = COM_DATA_START_POS + 2;

    uint16_t rec_crc    =  extractHalfWord(&p_buffer[crc_idx]);
    uint16_t actual_crc = com_CRC16_u16(&p_buffer[COM_START_SEQ_SIZE],data_length);
    
    if(rec_crc == actual_crc)
    {
        uint16_t address = extractHalfWord(&p_buffer[address_idx]);
        uint16_t value   = extractHalfWord(&p_buffer[value_idx]);
        
        int res = writeRegisterCmdExe(address,value);
        if(res == 0)
        {
            //success
            com_generate_callback(NULL);
        }
        else
        {
            com_response_error((uint8_t)(ERROR_WRITE_REG),(uint8_t)(COM_EXCEPTION_4));
        }
    }
    else
    {
        com_response_error((uint8_t)(ERROR_WRITE_REG),(uint8_t)(COM_EXCEPTION_3));
    }   
}

/**
 * @brief command processing for Modbus COM_WRITE_REG function 
 * 
 * @param address register address for writing
 * @param value new register value
 * 
 * @retval 0 in case of success, -1 in case of error
*/
static int writeRegisterCmdExe(const uint16_t address, const uint16_t value)
{
    int stat;
    if(address > (REGS_AVAILABLE -1))
    {
        stat = -1;
    }
    else
    {
        switch(address)
        {
            case(REG_APP_CONTROL):
                switch(value)
                {
                    case REG_APP_FILE_PR:
                        // new length =  (1 additional byte for function + 8 data bytes)
                        com_set_rec_data_length(COM_ADU_REQUIRED_PART + r_file_client_pdu_size);
                        break;
                    case REG_APP_FILE_PW:
                        // new length =  (1 additional byte for function + 8 data bytes + DATA_BLOCK_SIZE)
                        com_set_rec_data_length(COM_ADU_REQUIRED_PART + w_file_client_pdu_size);
                        break;
                    default:
                        break;
                }
                stat = 0;
                break;

            case(REG_APP_SIZE):
                if(control_regs[REG_BOOT_STATUS] == (uint16_t)(BT_EMPTY))
                {
                    //setup initial values for app install process
                    files[file_app_idx].num_of_records = value;
                    boot_control.app_update_request    = true;
                    stat = 0;
                }
                else
                {
                    stat = -1;
                }
                break;

            case(REG_APP_ERASE):
                switch(value)
                {
                    case REG_APP_ERASE_REQ:
                        appEraseMemory();
                        control_regs[REG_BOOT_STATUS] = (uint16_t)(BT_EMPTY);
                        control_regs[REG_APP_SIZE]    = 0;
                        process_stat                  = STAT_READY;
                        break;
                    default:
                        break;
                }
                stat = 0;
                break;
            
            case(REG_BOOT_CONTROL): 
                control_regs[REG_BOOT_CONTROL] = value;
                switch(value)
                {
                    case 0U:
                        boot_stay_in_reset();
                        boot_control.stay_in_boot = boot_check_stay_in();
                        break;
                    case REG_BOOT_CONTROL_SIB:
                        boot_stay_in_request();
                        boot_control.stay_in_boot = boot_check_stay_in();
                        break;
                    default:
                        break;
                }
                stat = 0;
                break;

            case(REG_APP_START):

                //call app start here if it is possible
                if(control_regs[REG_BOOT_STATUS] == (uint16_t)(BT_APP_READY))
                {
                    applicationStart();
                    //normally unreachable
                    for(;;){}
                }
                else
                {
                    stat = -1;
                }
                break;
            // write is forbidden to these registers
            case(REG_BOOT_STATUS):
            case(REG_BOOT_BLOCK_SIZE):    
            default:
                stat = -1;
                break;
        }
    }
    return stat;
}

/**
 * @brief function for implementation logic for COM_READ_REGS state diagram
 * 
 * @param context pointer to context from message handler in com.c
*/
static void readRegisterCmdProc(void* context)
{
    const uint8_t* p_buffer     = (uint8_t*)context;
    const uint8_t  data_length  = rw_reg_client_pdu_size + COM_ADDR_BYTE_SIZE; 
    const uint8_t  crc_idx      = rw_reg_client_pdu_size + COM_ADDR_BYTE_SIZE + COM_START_SEQ_SIZE;
    const uint8_t  address_idx  = COM_DATA_START_POS;
    const uint8_t  quantity_idx = COM_DATA_START_POS + 2;

    uint16_t rec_crc    = extractHalfWord(&p_buffer[crc_idx]);
    uint16_t actual_crc = com_CRC16_u16(&p_buffer[COM_START_SEQ_SIZE], data_length);
    if(rec_crc == actual_crc)
    {
        uint16_t start_address = extractHalfWord(&p_buffer[address_idx]);
        uint16_t quantity      = extractHalfWord(&p_buffer[quantity_idx]);
        
        if((quantity < 1) && (quantity > 125))
        {
            com_response_error((uint8_t)(ERROR_READ_REGS),(uint8_t)(COM_EXCEPTION_3));            
        }
        else
        {
            if(quantity <= REGS_AVAILABLE) 
            {
                int res = readRegisterCmdExe(start_address,quantity);
                if(res != 0)
                {
                    com_response_error((uint8_t)(ERROR_READ_REGS),(uint8_t)(COM_EXCEPTION_2));        
                }
                else
                {
                    //success
                    //callback shall be executed in readRegisterCmdExe
                }
            }
            else
            {
                com_response_error((uint8_t)(ERROR_READ_REGS),(uint8_t)(COM_EXCEPTION_2));        
            }
        }
    }
    else
    {
        com_response_error((uint8_t)(ERROR_READ_REGS),(uint8_t)(COM_EXCEPTION_3));
    }
}

/**
 * @brief command processing for Modbus COM_READ_REGS function 
 * 
 * @param address starting address for reading
 * @param quantity quantity of registers
 * 
 * @retval 0 in case of success, -1 in case of error
*/
static int readRegisterCmdExe(const uint16_t address, const uint16_t quantity)
{
    COM_MODBUS_SERVICE_TypeDef service;
    int                        stat;
    static const uint16_t tab_size = sizeof(control_regs)/sizeof(uint16_t);
    // quantity shall not exceed REGS_AVAILABLE for reference see common.h
    if((address + quantity) <= tab_size)
    {
        buffer[0] = (uint8_t)((quantity * 2) & 0xFFU);    
        int counter = 1;
        //copy actual reg values to created array
        for(int i = 0; i < quantity; i++)
        {
            buffer[counter]     = (uint8_t)((control_regs[address + i] >> 8)& 0xFFU);
            buffer[counter + 1] = (uint8_t)( control_regs[address + i]      & 0xFFU);
            counter += 2;
        }
        service.func   = (uint8_t)(COM_READ_REGS);
        // amount of half words + 1 byte for length
        service.length = (quantity * 2) + 1;
        service.p_data = buffer;
        com_generate_callback(&service);
        stat = 0;
    }
    else
    {
        stat = -1;
    }
    return stat;
}

/**
 * @brief function to extract a half word from an array (convert from big endian to little endian format)
 * 
 * @param p_data pointer to data array
 * 
 * @retval extracted half word
*/
static uint16_t extractHalfWord(const uint8_t *const p_data)
{
     uint16_t half_word = p_data[1];
              half_word |= ((uint16_t)p_data[0]) << 8;
    
    return half_word;
}

/**
 * @brief binding the handler to the function number received in the modbus command
 * 
 * @param handler pointer to handler for command
 * @param cmd command id number 
 * 
 * @retval true in case of success, false in case of fault
*/
static bool commandBind(COM_HANDLER handler, const uint8_t cmd)
{
    COM_MODBUS_HANDLER_TypeDef handle;
    handle.handler = handler;
    handle.func    =  cmd;
    return com_cmd_bind(&handle);
}

/**
 * @brief updating the write counter in the file and resetting to normal mode when reading/writing is completed
 * 
 * @param file_id id in files array
*/
static void fileProcUpdate(const uint16_t file_id)
{    
    if(file_id > num_of_files - 1){return;}
    else
    {
        if(files[file_id].counter == files[file_id].num_of_records){for(;;){};}// this should not happen
        else
        {
            files[file_id].counter++;
            if(files[file_id].counter == files[file_id].num_of_records)
            {
                APP_IMAGE_INFO_TypeDef img_info;
                const size_t file_size =  files[file_id].num_of_records * DATA_BLOCK_SIZE;
                com_set_rec_data_length(COM_INIT_SIZE);
                files[file_id].counter = 0;
                switch (file_id)
                {
                case file_app_idx:
                    if(boot_control.app_update_request == true)
                    {
                        boot_control.app_update_request = false;
                        img_info.app_size = file_size;
                        img_info.app_crc  = boot_crc32((uint32_t*)(__app_start),file_size);
                        if(boot_flash_write_block(APP_START,(uint8_t*)&img_info,sizeof(img_info))==true)
                        {
                            control_regs[REG_APP_SIZE]    = files[file_id].num_of_records;
                            control_regs[REG_BOOT_STATUS] = (uint16_t)(BT_APP_READY);
                        }
                        else
                        {
                            control_regs[REG_APP_SIZE]    = 0;
                            control_regs[REG_BOOT_STATUS] = (uint16_t)(BT_ERROR);
                        } 
                    }
                    break;

                default:
                    break;
                }
            }
        }
    }
}

/**
 * @brief setup FILE_CONTROL_TypeDef struct in files array with new file info
 * 
 * @param file_id id in files array
 * @param p_file pointer to the file
 * @param size file size in bytes
*/
static void fileSetup(const uint16_t file_id, void *p_file, const uint32_t size)
{
    if(file_id > num_of_files - 1){return;}
    else
    {
        files[file_id].counter        = 0;
        files[file_id].file_size      = size;
        files[file_id].p_data         = p_file;
        files[file_id].num_of_records = size/DATA_BLOCK_SIZE;

        if((size % DATA_BLOCK_SIZE)!= 0){files[file_id].num_of_records++;}
    }
}

/**
 * @brief Function for application start
 * 
 * @param addr application direct start address
*/
static void applicationStart()
{
    const uint32_t new_msp       = *((uint32_t *)(__app_start));                    
    const uint32_t reset_handler = *((uint32_t *)(__app_start + sizeof(uint32_t)));
    
    vTaskSuspendAll();
    if(boot_control.hw_configured == true){boot_hw_deinit();}
    jumpToApplication(new_msp, reset_handler);
}

/**
 * @brief Function that sets the stack pointer and starts executing a particular address.
 * 
 * @param new_msp  the new value to set in the main stack pointer.
 * @param addr the address to execute.
*/
static void jumpToApplication(const uint32_t new_msp, const uint32_t addr)
{
    void (*goToApp)() = ((void (*)(void))addr);
    __set_MSP(new_msp);
    goToApp();
}

/**
 * @brief weak implementation of hw init for linker 
*/
void __attribute__((weak)) boot_hw_init(void){}

/**
 * @brief weak implementation of hw deinit for linker 
*/
void __attribute__((weak)) boot_hw_deinit(void){}

/**
 * @brief weak implementation of cheking if we want to stay in boot
 * 
 * @retval returns always true 
*/
bool __attribute__((weak)) boot_check_stay_in(void){return true;}

/**
 * @brief weak implementation of stay in boot request
 * 
*/
void __attribute__((weak)) boot_stay_in_request(void){}

/**
 * @brief weak implementation of stay in boot resetting 
 * 
*/
void __attribute__((weak)) boot_stay_in_reset(void){}

/**
 * @brief crc32 calculator, software realization 
 * 
 * @param buf pointer to data
 * @param size data size in bytes
 * 
 * @retval actual crc value 
*/
uint32_t __attribute__((weak)) boot_crc32(const void *buf, size_t size)
{
    static const uint32_t tab[] = 
    {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3,	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de,	0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,	0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5,	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,	0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940,	0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,	0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
    };

    const uint8_t *p = buf;
    uint32_t crc;
    crc = ~0U;
    while (size--)
    {
        crc = tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ ~0U;
}

/**
 * @brief weak implementation of page erase for linker
 *
 * @param page_start_addr start address of flash page
*/
void __attribute__((weak)) boot_page_erase(const uint32_t page_start_addr)
{
    (void)(page_start_addr);
}

/**
 * @brief weak implementation of flash write for linker
 *
 * @param addr   direct start flash address (!IMPORTANT not pointer)
 * @param p_data pointer to data to be written
 * @param size   data size in bytes (!IMPORTANT must be multiple of 4 bytes)
 * 
 * @retval true in case of success, false in case of error (always false)
*/
bool __attribute__((weak)) boot_flash_write_block(const uint32_t addr, 
                                                  const uint8_t* p_data, 
                                                  const uint32_t size)
{
    (void)(addr);
    (void)(p_data);
    (void)(size);
    return false;
}