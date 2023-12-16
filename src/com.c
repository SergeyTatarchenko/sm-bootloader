/**
 * @file com.c
 *
 * @brief implementation of the basic logic for sending and 
 * receiving messages according to the Modbus protocol
 *
 * @author Siarhei Tatarchanka
 * Contact: zlojdigger@gmail.com
 *
 */

/* Includes-------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "com.h"
#include "rtos.h"
/* Private define-------------------------------------------------------------*/
#define COM_TASK_FC_STACK_SIZE                                                32
#define COM_TASK_RX_STACK_SIZE                                                64
#define COM_TASK_RX_PRIORITY                          (configMAX_PRIORITIES - 2)
#define COM_TASK_FC_PRIORITY                          (configMAX_PRIORITIES - 1)

// 1s timeout for communication error (rtos tick must be 1ms)
#define COM_TIMER_TICKS                                                      100
#define COM_TIMER_RELOAD_VALUE                                                10
/* Private typedef -----------------------------------------------------------*/

typedef struct
{
    uint8_t length;
    uint8_t* p_buffer;

}COM_BUFFER_INFO_TypeDef;

#pragma pack(push)
#pragma pack(1)

typedef struct
{
    uint8_t  start[4];
    uint8_t  addr;
    uint8_t  func;

}COM_MES_HEADER_TypeDef;

#pragma  pack(pop)

/* Private function prototypes -----------------------------------------------*/

static bool generate_modbus_message(const COM_MODBUS_SERVICE_TypeDef* service);
static void task_incCmdProc        (void *pvParameters);
static void task_flowControlProc   (void *pvParameters);
static void transmitRequest        (void);
static void resetAndFlush          (void);

/* Private variables ---------------------------------------------------------*/

static COM_MODBUS_HANDLER_TypeDef modbus_cmd [NUM_OF_COM_FUNCTIONS];
static uint8_t                    data_buffer[COM_BUFFER_SIZE];
static uint8_t                    last_msg_size;
// resources for task_incCmdProc
static StackType_t       rx_task_stack[COM_TASK_RX_STACK_SIZE];
static StaticTask_t      rx_task_buffer;
static TaskHandle_t      xHandle_rx_task_handle         = NULL;
// resources for task_flowControlProc
static StackType_t       fc_task_stack[COM_TASK_FC_STACK_SIZE];
static StaticTask_t      fc_task_buffer;
static TaskHandle_t      xHandle_fc_task_handle         = NULL;
// resources for xSemaphore_data_received
static StaticSemaphore_t xSemaphore_data_received_buffer;
static SemaphoreHandle_t xSemaphore_data_received       = NULL;

static volatile COM_BUFFER_INFO_TypeDef rx_control;
static volatile COM_BUFFER_INFO_TypeDef tx_control;               
static volatile uint8_t                 byte_counter;
static volatile bool                    rc_in_progress;
static volatile UBaseType_t             wdt;

/**
 * @brief initialization of variables and tasks of the data exchange module
 *
*/
void com_init(void)
{
    (void)memset(&data_buffer[0],0,sizeof(data_buffer));
    (void)memset(&modbus_cmd[0],0,sizeof(modbus_cmd));
    resetAndFlush();
    
    xSemaphore_data_received = xSemaphoreCreateBinaryStatic(&xSemaphore_data_received_buffer);
    configASSERT(xSemaphore_data_received);
    
    xHandle_rx_task_handle = xTaskCreateStatic(task_incCmdProc, 
                                                "modbus rx task", 
                                                COM_TASK_RX_STACK_SIZE, 
                                                NULL, 
                                                COM_TASK_RX_PRIORITY, 
                                                rx_task_stack, 
                                                &rx_task_buffer);
                                                
    xHandle_fc_task_handle = xTaskCreateStatic(task_flowControlProc, 
                                                "modbus flow control", 
                                                COM_TASK_FC_STACK_SIZE, 
                                                NULL, 
                                                COM_TASK_FC_PRIORITY, 
                                                fc_task_stack, 
                                                &fc_task_buffer);
}

/**
 * @brief CRC16_u16 calculate (CRC16 0xA001 poly)
 *
 * @param data pointer to data array
 * @param length array length
 *
 * @retval CRC16 word
*/
uint16_t com_CRC16_u16 (const uint8_t * data, uint16_t length)
{
    static const uint16_t CRCTable[256] = 
    {
    0X0000u, 0XC0C1u, 0XC181u, 0X0140u, 0XC301u, 0X03C0u, 0X0280u, 0XC241u,
    0XC601u, 0X06C0u, 0X0780u, 0XC741u, 0X0500u, 0XC5C1u, 0XC481u, 0X0440u,
    0XCC01u, 0X0CC0u, 0X0D80u, 0XCD41u, 0X0F00u, 0XCFC1u, 0XCE81u, 0X0E40u,
    0X0A00u, 0XCAC1u, 0XCB81u, 0X0B40u, 0XC901u, 0X09C0u, 0X0880u, 0XC841u,
    0XD801u, 0X18C0u, 0X1980u, 0XD941u, 0X1B00u, 0XDBC1u, 0XDA81u, 0X1A40u,
    0X1E00u, 0XDEC1u, 0XDF81u, 0X1F40u, 0XDD01u, 0X1DC0u, 0X1C80u, 0XDC41u,
    0X1400u, 0XD4C1u, 0XD581u, 0X1540u, 0XD701u, 0X17C0u, 0X1680u, 0XD641u,
    0XD201u, 0X12C0u, 0X1380u, 0XD341u, 0X1100u, 0XD1C1u, 0XD081u, 0X1040u,
    0XF001u, 0X30C0u, 0X3180u, 0XF141u, 0X3300u, 0XF3C1u, 0XF281u, 0X3240u,
    0X3600u, 0XF6C1u, 0XF781u, 0X3740u, 0XF501u, 0X35C0u, 0X3480u, 0XF441u,
    0X3C00u, 0XFCC1u, 0XFD81u, 0X3D40u, 0XFF01u, 0X3FC0u, 0X3E80u, 0XFE41u,
    0XFA01u, 0X3AC0u, 0X3B80u, 0XFB41u, 0X3900u, 0XF9C1u, 0XF881u, 0X3840u,
    0X2800u, 0XE8C1u, 0XE981u, 0X2940u, 0XEB01u, 0X2BC0u, 0X2A80u, 0XEA41u,
    0XEE01u, 0X2EC0u, 0X2F80u, 0XEF41u, 0X2D00u, 0XEDC1u, 0XEC81u, 0X2C40u,
    0XE401u, 0X24C0u, 0X2580u, 0XE541u, 0X2700u, 0XE7C1u, 0XE681u, 0X2640u,
    0X2200u, 0XE2C1u, 0XE381u, 0X2340u, 0XE101u, 0X21C0u, 0X2080u, 0XE041u,
    0XA001u, 0X60C0u, 0X6180u, 0XA141u, 0X6300u, 0XA3C1u, 0XA281u, 0X6240u,
    0X6600u, 0XA6C1u, 0XA781u, 0X6740u, 0XA501u, 0X65C0u, 0X6480u, 0XA441u,
    0X6C00u, 0XACC1u, 0XAD81u, 0X6D40u, 0XAF01u, 0X6FC0u, 0X6E80u, 0XAE41u,
    0XAA01u, 0X6AC0u, 0X6B80u, 0XAB41u, 0X6900u, 0XA9C1u, 0XA881u, 0X6840u, 
    0X7800u, 0XB8C1u, 0XB981u, 0X7940u, 0XBB01u, 0X7BC0u, 0X7A80u, 0XBA41u,
    0XBE01u, 0X7EC0u, 0X7F80u, 0XBF41u, 0X7D00u, 0XBDC1u, 0XBC81u, 0X7C40u,
    0XB401u, 0X74C0u, 0X7580u, 0XB541u, 0X7700u, 0XB7C1u, 0XB681u, 0X7640u,
    0X7200u, 0XB2C1u, 0XB381u, 0X7340u, 0XB101u, 0X71C0u, 0X7080u, 0XB041u,
    0X5000u, 0X90C1u, 0X9181u, 0X5140u, 0X9301u, 0X53C0u, 0X5280u, 0X9241u,
    0X9601u, 0X56C0u, 0X5780u, 0X9741u, 0X5500u, 0X95C1u, 0X9481u, 0X5440u,
    0X9C01u, 0X5CC0u, 0X5D80u, 0X9D41u, 0X5F00u, 0X9FC1u, 0X9E81u, 0X5E40u,
    0X5A00u, 0X9AC1u, 0X9B81u, 0X5B40u, 0X9901u, 0X59C0u, 0X5880u, 0X9841u,
    0X8801u, 0X48C0u, 0X4980u, 0X8941u, 0X4B00u, 0X8BC1u, 0X8A81u, 0X4A40u,
    0X4E00u, 0X8EC1u, 0X8F81u, 0X4F40u, 0X8D01u, 0X4DC0u, 0X4C80u, 0X8C41u,
    0X4400u, 0X84C1u, 0X8581u, 0X4540u, 0X8701u, 0X47C0u, 0X4680u, 0X8641u,
    0X8201u, 0X42C0u, 0X4380u, 0X8341u, 0X4100u, 0X81C1u, 0X8081u, 0X4040u 
    };

    uint16_t crc          = (uint16_t)0xFFFFu;
    uint16_t data_counter = length;
    uint16_t mem_counter  = (uint16_t)0;

    for ( ; data_counter > 0u; data_counter--)
    {
        uint8_t temp = data[mem_counter] ^ crc;
        mem_counter++;
        crc >>= 8;
        crc ^= CRCTable[temp];
    }
    return crc;
}

/**
 * @brief create and send a packet with a error message 
 * according to Modbus protocol
 *
 * @param error error number
 * @param exception error exception number
*/
void com_response_error(const uint8_t error, const uint8_t exception)
{
    uint8_t data[] = {exception};
    const COM_MODBUS_SERVICE_TypeDef service =
    {
        .func   = error,
        .length = sizeof(data),
        .p_data = &data[0]
    };
    bool stat = generate_modbus_message(&service);
    if(stat == true)
    {
        transmitRequest();
    }
    else
    {
        //internal error
        for(;;){}
    }
}

/**
 * @brief create and send a packet with a message 
 * according to Modbus protocol
 *
 * @param service pointer to COM_MODBUS_SERVICE_TypeDef struct
*/
void com_generate_callback(const COM_MODBUS_SERVICE_TypeDef* service)
{
    if(service != NULL)
    {
        bool stat = generate_modbus_message(service);
        if(stat != true){for(;;){}}//internal error
    }else
    {
        //we want to recent what we already have in buffer
        tx_control.p_buffer = &data_buffer[0];
        tx_control.length   = last_msg_size;
    }
    transmitRequest();
}

/**
 * @brief receiving a byte in the incoming buffer (should be called in an interrupt)
 *
 * @param byte received byte
*/
void com_rec_byte(const uint8_t byte)
{
    if(rx_control.p_buffer != NULL)
    {
        rx_control.p_buffer[byte_counter] = byte;
        byte_counter++;
        rc_in_progress = true;
    }else{return;}
    if(byte_counter == rx_control.length)
    {
        byte_counter   = 0u;
        rc_in_progress = false;
        xSemaphoreGiveFromISR(xSemaphore_data_received, NULL);
    }else{return;}
}

/**
 * @brief updating the packet size value that is expected to be accepted
 * 
 * @param length new length in bytes
*/
void com_set_rec_data_length(const uint8_t length)
{
    rx_control.p_buffer = &data_buffer[0];
    rx_control.length   = length;
}

/**
 * @brief reset all buffers and variables related to data exchange
 *
*/
static void resetAndFlush(void)
{
    rc_in_progress      = false;
    wdt                 = COM_TIMER_RELOAD_VALUE;
    tx_control.length   = 0u;
    tx_control.p_buffer = NULL;
    rx_control.length   = COM_INIT_SIZE;
    rx_control.p_buffer = &data_buffer[0];
    byte_counter        = 0;
    last_msg_size       = 0;
}

/**
 * @brief request for data transmit
 *
*/
void transmitRequest(void)
{
    com_send(tx_control.p_buffer,tx_control.length);
}

/**
 * @brief binding the handler to the function number received in the modbus command
 *
 * @param p_cmd pointer to the command struct
 *
 * @retval return true in case of success, false in case of error
*/
bool com_cmd_bind (const COM_MODBUS_HANDLER_TypeDef *p_cmd)
{
    bool state = false;
    if(p_cmd != NULL)
    {
        if((p_cmd->func != 0) && (p_cmd->handler != NULL))
        {
            for(int i = 0; i < NUM_OF_COM_FUNCTIONS; i++)
            {
                if(modbus_cmd[i].func == 0)
                {
                    modbus_cmd[i].func    = p_cmd->func;
                    modbus_cmd[i].handler = p_cmd->handler;
                    state = true;
                    break;
                }else{continue;}
            }
        }else{state = false;}
    }else{state = false;}
    
    return state;
}

/**
 * @brief an empty function for sending a data packet 
 * that is linked if the real one is not implemented, does nothing
 * 
 * @param p_data pointer to data array
 * @param size data array size
*/
void __attribute__((weak)) com_send(const uint8_t * p_data, const uint32_t size)
{
    (void)(p_data);
    (void)(size);
}

/**
 * @brief creation of a packet with a message 
 * according to Modbus protocol on an allocated memory area
 *
 * @param service pointer to COM_MODBUS_SERVICE_TypeDef struct
 * 
 * @retval true in case success, false in case not
*/
static bool generate_modbus_message(const COM_MODBUS_SERVICE_TypeDef* service)
{    
    if((service->length + COM_ADU_REQUIRED_PART) > COM_BUFFER_SIZE)
    {
        return false;
    }
    else
    {
        //we can replace data_buffer by second buffer in future, but for now it is ok
        tx_control.p_buffer    = &data_buffer[0];
        tx_control.length      = COM_ADU_REQUIRED_PART + service->length + 1;
        
        (void)memset(&tx_control.p_buffer[0],0,tx_control.length);
        tx_control.p_buffer[4] = TARGET_ADDRESS;
        tx_control.p_buffer[5] = service->func;
        (void)memcpy(&tx_control.p_buffer[6],service->p_data,service->length);
        uint16_t crc           = com_CRC16_u16(&tx_control.p_buffer[4],(service->length + 2));
        //put crc in big endian format
        uint8_t crc_index                  = 6 + service->length;
        tx_control.p_buffer[crc_index]     =  (uint8_t)((crc & (uint16_t)(0xFF00U))>>8);
        tx_control.p_buffer[crc_index + 1] =  (uint8_t)(crc & (uint16_t)(0x00FFU));
    }
    return true;
}

/**
 * @brief data flow control task
 * 
 * @param pvParameters pointer to parameters when creating a task
*/
static void task_flowControlProc(void *pvParameters)
{
    (void)(pvParameters);

    for(;;)
    {
        if(rc_in_progress == true)
        {
            wdt--;
            if(wdt == 0)
            {
                resetAndFlush();
            }
        }
        else
        {
            wdt = COM_TIMER_RELOAD_VALUE;
        }
        vTaskDelay(COM_TIMER_TICKS);
    }
}

/**
 * @brief the task of processing incoming message packets
 * 
 * @param pvParameters pointer to parameters when creating a task
*/
static void task_incCmdProc(void *pvParameters)
{
    (void)(pvParameters);

    for(;;)
    {
        xSemaphoreTake(xSemaphore_data_received,portMAX_DELAY);
        COM_MES_HEADER_TypeDef *mes_header = (COM_MES_HEADER_TypeDef*)rx_control.p_buffer;
    
        if(mes_header->addr == TARGET_ADDRESS)
        {
            bool func_exist = false;
            for(int i = 0; i < NUM_OF_COM_FUNCTIONS; i++)
            {
                if(mes_header->func == modbus_cmd[i].func)
                {
                    last_msg_size = rx_control.length;
                    modbus_cmd[i].handler(mes_header);
                    wdt        = COM_TIMER_RELOAD_VALUE;
                    func_exist = true;
                    break;
                }
                else
                {
                    continue;
                }
            }
            if(func_exist == false)
            {
                com_response_error((mes_header->func | (uint8_t)(0x80U)),(uint8_t)(COM_EXCEPTION_1));
            }
        }else{}//do nothing, address not recognized
    }
}
