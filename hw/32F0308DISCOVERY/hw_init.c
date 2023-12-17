/******************************************************************************
* File Name          : hw_init.c
* Author             : 
* Version            : 
* Description        : implementation for 32F030DISCOVERY 
*******************************************************************************/

#include "stm32f0xx.h"

#include <stddef.h>
#include <string.h> 

#include "rtos.h"
#include "com.h"
#include "bootloader.h"

/* Private define-------------------------------------------------------------*/
#define GREEN_LED_ON                             (GPIOC->BSRR |= GPIO_BSRR_BS_9)
#define GREEN_LED_OFF                            (GPIOC->BSRR |= GPIO_BSRR_BR_9)
#define BLUE_LED_ON                              (GPIOC->BSRR |= GPIO_BSRR_BS_8)
#define BLUE_LED_OFF                             (GPIOC->BSRR |= GPIO_BSRR_BR_8)

#define LED_TASK_STACK_SIZE                                                   32
#define LED_TASK_PRIORITY                             (configMAX_PRIORITIES - 3)

#define SERIAL_DEFAULT_BAUDRATE                                            19200

/* Private function prototypes -----------------------------------------------*/

static void pin_init           (void);
static void pin_deinit         (void);
static void serial_init        (const uint32_t baudrate);
static void task_led           (void *pvParameters);
static void set_serial_baudrate(const uint32_t baudrate);
static void serial_enable      (void);
static void serial_disable     (void);
static void serial_send_byte   (const uint8_t byte);

/* Private variables ---------------------------------------------------------*/

static StaticTask_t  led_task_buffer;
static StackType_t   led_task_stack[LED_TASK_STACK_SIZE];
static TaskHandle_t  xHandle_led_task_handle = NULL;

/**
 * @brief mcu hw init
 *
*/
void boot_hw_init(void)
{
    __disable_irq ();
    serial_disable();
    pin_init();
    serial_init(SERIAL_DEFAULT_BAUDRATE);
    xHandle_led_task_handle = xTaskCreateStatic(task_led, 
                                                "led task", 
                                                LED_TASK_STACK_SIZE, 
                                                NULL, 
                                                LED_TASK_PRIORITY, 
                                                led_task_stack, 
                                                &led_task_buffer);
    serial_enable();
    __enable_irq ();
}

void HardFault_Handler()
{
    for(;;){}
}

void boot_hw_deinit(void)
{
    pin_deinit();
    serial_disable();
    __disable_irq ();
}

void com_send(const uint8_t * p_data, const uint32_t size)
{
    for(uint32_t i = 0; i < size; i++)
    {
        serial_send_byte(p_data[i]);
    }
}

void boot_page_erase(const uint32_t page_start_addr)
{
    //flash unlock
    if((FLASH->CR & FLASH_CR_LOCK) != 0)
    {
        FLASH->KEYR = (uint32_t)(0x45670123U);
        FLASH->KEYR = (uint32_t)(0xCDEF89ABU);
    }
    //request for page erase
    FLASH->CR |= FLASH_CR_PER;
    //page number
    FLASH->AR  = page_start_addr;
    //start erase
    FLASH->CR |= FLASH_CR_STRT;
    while((FLASH->SR & FLASH_SR_BSY) != 0){}
    //disable page erase
    FLASH->CR &= ~FLASH_CR_PER;
    //clear EOP flag (set to 1 according to doc)
    FLASH->SR |= FLASH_SR_EOP;
    //flash lock
    FLASH->CR |= FLASH_CR_LOCK;
}

bool boot_flash_write_block(const uint32_t addr, 
                            const uint8_t* p_data, 
                            const uint32_t size)
{
    bool stat;
    if((p_data != NULL) && 
       ( size > 0)      && 
       (size % 4 == 0))
    {
        //flash unlock
        if((FLASH->CR & FLASH_CR_LOCK) != 0)
        {
            FLASH->KEYR = (uint32_t)(0x45670123U);
            FLASH->KEYR = (uint32_t)(0xCDEF89ABU);
        }
        //request for flash write
        FLASH->CR |= FLASH_CR_PG;
        // write in half words (stm32f030 specific)
        uint32_t counter      = 0;
        uint16_t* mem_pointer = (uint16_t*)(addr);
        for(int i = 0; i < size; i += 2)
        {
            uint16_t half_word = p_data[i];
                     half_word |= ((uint16_t)p_data[i + 1]) << 8;
            mem_pointer[counter] = half_word;
            while((FLASH->SR & FLASH_SR_BSY) != 0){}
            counter++;
        }
        //disable flash write
        FLASH->CR &= ~FLASH_CR_PG;
        //clear EOP flag (set to 1 according to doc)
        FLASH->SR |= FLASH_SR_EOP;
        //flash lock
        FLASH->CR |= FLASH_CR_LOCK;
        if(memcmp(mem_pointer,p_data,size) == 0)
        {
            stat = true;
        }
        else
        {
            stat = false;
        }
    }
    else
    {
        stat = false;
    }

    return stat;
}

void USART1_IRQHandler()
{
    if(USART1->ISR &= USART_ISR_RXNE)
    {
        uint8_t byte = USART1->RDR;
        com_rec_byte(byte);
    }
}

static void serial_send_byte(const uint8_t byte)
{
    while((USART1->ISR & USART_ISR_TXE) == 0u){}
	USART1->TDR = byte;
}

static void pin_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    /*init LEDs on Discovery board */
    GPIOC->MODER |= GPIO_MODER_MODER9_0;
    GPIOC->MODER |= GPIO_MODER_MODER8_0;
    GREEN_LED_OFF;
    BLUE_LED_OFF;
    /*init pins for UART communication,
    PA9 - TX, PA10 -RX */
    GPIOA->MODER   |= (GPIO_MODER_MODER9_1   | GPIO_MODER_MODER10_1);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR9 | GPIO_OSPEEDR_OSPEEDR10);
	GPIOA->AFR[1]  |= ((1<<4)|(1<<8));

}

static void pin_deinit(void)
{
    GREEN_LED_ON;
    BLUE_LED_ON;

    GPIOC->MODER   &= ~GPIO_MODER_MODER9_0;
    GPIOC->MODER   &= ~GPIO_MODER_MODER8_0;
    GPIOA->MODER   &= ~(GPIO_MODER_MODER9_1   | GPIO_MODER_MODER10_1);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR9 | GPIO_OSPEEDR_OSPEEDR10);
	GPIOA->AFR[1]  &= ~((1<<4)|(1<<8));    

    RCC->AHBENR  &= ~RCC_AHBENR_GPIOCEN;
    RCC->AHBENR  &= ~RCC_AHBENR_GPIOAEN;
}

static void serial_init(const uint32_t baudrate)
{
    /*USART1 from stm32f030cbt6 by default*/
    RCC->CFGR3   |=  RCC_CFGR3_USART1SW_0; 
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->CR1  |= (USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE);
    set_serial_baudrate(baudrate);
    NVIC_SetPriority (USART1_IRQn,5);
}

static void serial_enable(void)
{
    NVIC_EnableIRQ(USART1_IRQn);	
    USART1->CR1 |= USART_CR1_UE;
}

static void serial_disable(void)
{
    NVIC_DisableIRQ(USART1_IRQn);	
    USART1->CR1 &= ~USART_CR1_UE;
}

static void set_serial_baudrate(const uint32_t baudrate)
{
    /*USART1 from stm32f030cbt6 by default*/
    const uint32_t pll_freq = configCPU_CLOCK_HZ;
          uint16_t div      = 0;
    if(baudrate != 0)
    {
        div = (uint16_t)(pll_freq/baudrate);
        USART1->BRR = div;
    }
    else{}
}

static void task_led(void *pvParameters)
{
    for(;;)
    {
        GREEN_LED_ON;
        vTaskDelay(100);
        GREEN_LED_OFF;
        vTaskDelay(400);
    }
}
