/*
 * PZEM-004T Modbus RTU Library
 * 
 * This library provides an interface for the PZEM-004T energy meter using Modbus RTU protocol over UART for the Raspberry Pi Pico.
 * PZEM-004T datasheet: https://innovatorsguru.com/wp-content/uploads/2019/06/PZEM-004T-V3.0-Datasheet-User-Manual.pdf
 * 
 * Paul Uriarte Vicandi (pauluv@ni.eus)
 * April 2025
 *
*/

#ifndef PZEM_004T_MODBUS_RTU_H
#define PZEM_004T_MODBUS_RTU_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// UART format
#define PZEM_UART_BAUD_RATE 9600
#define PZEM_UART_DATA_BITS 8
#define PZEM_UART_STOP_BITS 1
#define PZEM_UART_PARITY UART_PARITY_NONE

// PZEM-004T Modbus RTU commands
#define PZEM_CMD_RHR 0x03 // Read Holding Register
#define PZEM_CMD_RIR 0x04 // Read Input Register
#define PZEM_CMD_WSR 0x06 // Write Single Register
#define PZEM_CMD_CAL 0x41 // Calibration (for factory use)
#define PZEM_CMD_RST 0x42 // Reset energy

// PZEM-004T Modbus RTU addresses
#define PZEM_SLAVE_ADDR_MIN 0x01
#define PZEM_SLAVE_ADDR_MAX 0xF7
#define PZEM_SLAVE_ADDR_GENERAL 0xF8

// Measurement registers
#define PZEM_REG_VOLTAGE        0x0000
#define PZEM_REG_CURRENT_LOW    0x0001
#define PZEM_REG_CURRENT_HIGH   0x0002
#define PZEM_REG_POWER_LOW      0x0003
#define PZEM_REG_POWER_HIGH     0x0004
#define PZEM_REG_ENERGY_LOW     0x0005
#define PZEM_REG_ENERGY_HIGH    0x0006
#define PZEM_REG_FREQUENCY      0x0007
#define PZEM_REG_POWER_FACTOR   0x0008
#define PZEM_REG_ALARM_STATUS   0x0009

// Error codes
#define PZEM_ERR_RESPONSE         0x86
#define PZEM_ERR_ILLEGAL_FUNCTION 0x01
#define PZEM_ERR_ILLEGAL_ADDRESS  0x02
#define PZEM_ERR_ILLEGAL_DATA     0x03
#define PZEM_ERR_SLAVE_ERROR      0x04

// Write register commands
#define PZEM_WREG_ALARM_THR  0x0001
#define PZEM_WREG_SLAVE_ADDR 0x0002

// Measurement Parameters
// To get the real values in SI units, divide the raw values by the following constants
#define PZEM_VOLTAGE_DIV    10
#define PZEM_CURRENT_DIV    1000
#define PZEM_POWER_DIV      10
#define PZEM_PF_DIV         100
#define PZEM_FREQ_DIV       10
#define PZEM_ENERGY_DIV     1

typedef struct {
    uint32_t current; // In units of 0.001A
    uint32_t power;   // In units of 0.1W
    uint32_t energy; // In units of 1Wh
    uint16_t voltage; // In units of 0.1V
    uint16_t frequency; // In units of 0.1Hz
    uint16_t power_factor; // In units of 0.01
    uint16_t alarm_status;
} pzem004t_data_t;

bool pzem004t_init(uart_inst_t *uart, uint gpio_uart_tx, uint gpio_uart_rx, bool debug);

bool pzem004t_set_alarm_thr(uint16_t slave_addr, uint16_t alarm_thr);
bool pzem004t_set_slave_addr(uint16_t new_slave_addr);

bool pzem004t_read_data(uint16_t slave_addr, pzem004t_data_t *data);

#endif