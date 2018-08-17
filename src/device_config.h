#ifndef __DEVICE_CONFIG_H__
#define __DEVICE_CONFIG_H__

#include "flash_config.h"
#include "flash_loader.h"
#include "flash_loader_extra.h"

#define  TICK_INT_PRIORITY            ((uint32_t)0x0FU) /*!< tick interrupt priority */

extern __IO uint32_t g_uptime;

void uptime_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

void sys_mpu_init(void);
void sys_clk_init(void);
void sys_nvic_init(void);
void sys_uart_init(void);
void flexspi_pin_init(void);

void uart_send(uint8_t *dat);
#endif

