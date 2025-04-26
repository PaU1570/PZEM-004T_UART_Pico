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

#include "pzem-004t_modbus-rtu.h"
#include "modbus_crc16.h"

#define READ_TIMEOUT (10 * 1000) // us
#define CORRECT_RESPONSE_LENGTH 25
#define ERROR_RESPONSE_LENGTH 5
#define RESPONSE_HEADER_LENGTH 3

#define VOLTAGE_OFFSET      (RESPONSE_HEADER_LENGTH + (PZEM_REG_VOLTAGE * 2))
#define CURRENT_LOW_OFFSET  (RESPONSE_HEADER_LENGTH + (PZEM_REG_CURRENT_LOW * 2))
#define CURRENT_HIGH_OFFSET (RESPONSE_HEADER_LENGTH + (PZEM_REG_CURRENT_HIGH * 2))
#define POWER_LOW_OFFSET    (RESPONSE_HEADER_LENGTH + (PZEM_REG_POWER_LOW * 2))
#define POWER_HIGH_OFFSET   (RESPONSE_HEADER_LENGTH + (PZEM_REG_POWER_HIGH * 2))
#define ENERGY_LOW_OFFSET   (RESPONSE_HEADER_LENGTH + (PZEM_REG_ENERGY_LOW * 2))
#define ENERGY_HIGH_OFFSET  (RESPONSE_HEADER_LENGTH + (PZEM_REG_ENERGY_HIGH * 2))
#define FREQUENCY_OFFSET    (RESPONSE_HEADER_LENGTH + (PZEM_REG_FREQUENCY * 2))
#define POWER_FACTOR_OFFSET (RESPONSE_HEADER_LENGTH + (PZEM_REG_POWER_FACTOR * 2))
#define ALARM_STATUS_OFFSET (RESPONSE_HEADER_LENGTH + (PZEM_REG_ALARM_STATUS * 2))

static uart_inst_t *uart = NULL;

static bool __debug = false;

/**
 * @brief Sends a Modbus RTU command to the specified slave device.
 *
 * This function constructs a Modbus RTU frame and sends it over UART to the specified
 * slave device. The frame includes the slave address, function code, register address,
 * register value/number, and CRC checksum.
 *
 * @param slave_addr The address of the slave device. Must be within the valid range
 *                   or equal to the general broadcast address.
 * @param cmd        The Modbus function code to execute.
 * @param reg_addr   The register address to access.
 * @param reg_num    The register value to write or number of registers to access.
 * @return true if the command was successfully sent, false otherwise.
 */
static bool __send_command(uint8_t slave_addr, uint8_t cmd, uint16_t reg_addr, uint16_t reg_num) {
    if (uart == NULL) {
        return false; // UART not initialized
    }

    if ((slave_addr != PZEM_SLAVE_ADDR_GENERAL) && (slave_addr < PZEM_SLAVE_ADDR_MIN || slave_addr > PZEM_SLAVE_ADDR_MAX)) {
        return false; // Invalid slave address
    }

    uint8_t tx_buffer[8];
    uint16_t crc;

    // Build the Modbus RTU frame
    tx_buffer[0] = slave_addr; // Slave address
    tx_buffer[1] = cmd;        // Function code
    tx_buffer[2] = (reg_addr >> 8) & 0xFF; // Register address high byte
    tx_buffer[3] = reg_addr & 0xFF;        // Register address low byte
    tx_buffer[4] = (reg_num >> 8) & 0xFF;  // Register number/value high byte
    tx_buffer[5] = reg_num & 0xFF;         // Register number/value low byte

    // Calculate CRC
    crc = modbus_CRC16(tx_buffer, 6);
    tx_buffer[6] = crc & 0xFF;         // CRC low byte
    tx_buffer[7] = (crc >> 8) & 0xFF;  // CRC high byte
    
    // Send the command over UART
    if (__debug) {
        printf("PZEM-004T: Sending command: ");
        for (int i = 0; i < 6; i++) {
            printf("%02X ", tx_buffer[i]);
        }
        printf("CRC: %04X\n", crc);
    }
    uart_write_blocking(uart, tx_buffer, sizeof(tx_buffer));

    return true;
}

/**
 * @brief Reads a response from the slave device into the provided buffer.
 *
 * This function reads data from the UART interface into the specified buffer
 * until either the buffer is full or the read timeout expires.
 *
 * @param rx_buffer Pointer to the buffer where the received data will be stored.
 * @param length    The maximum number of bytes to read into the buffer.
 *
 * @return The number of bytes successfully read into the buffer.
 */
static uint16_t __read_response(uint8_t *rx_buffer, uint16_t length) {
    if (uart == NULL) {
        return 0; // UART not initialized
    }

    // Read the response from the slave device
    uint16_t index = 0;
    while (uart_is_readable_within_us(uart, READ_TIMEOUT)) {
        rx_buffer[index++] = uart_getc(uart);
        if (__debug) {
            printf("PZEM-004T: Received byte: %02X\n", rx_buffer[index - 1]);
        }
        if (index >= length) {
            break; // Buffer is full
        }
    }
    return index;
}

/**
 * @brief Initializes the PZEM-004T module with the specified UART instance and GPIO pins.
 *
 * This function configures the UART instance and GPIO pins for communication with the
 * PZEM-004T energy monitoring module. It sets the UART baud rate, data format, and assigns
 * the specified GPIO pins for UART TX and RX functionality.
 * Check the GPIO function select table in the datasheet for information on GPIO assignments.
 *
 * @param uart_instance Pointer to the UART instance to be used for communication.
 * @param gpio_uart_tx  GPIO pin number to be used for UART TX (connected to PZEM-004T's RX).
 * @param gpio_uart_rx  GPIO pin number to be used for UART RX (connected to PZEM-004T's TX).
 * @param debug         Flag to enable or disable debug mode.
 * @return true if initialization is successful, false if the UART instance is invalid.
 */
bool pzem004t_init(uart_inst_t *uart_instance, uint gpio_uart_tx, uint gpio_uart_rx, bool debug) {
    if (uart_instance == NULL) {
        return false; // Invalid UART instance
    }

    uart = uart_instance;
    __debug = debug;

    // Initialize UART
    uart_init(uart, PZEM_UART_BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(gpio_uart_tx, UART_FUNCSEL_NUM(uart, gpio_uart_tx));
    gpio_set_function(gpio_uart_rx, UART_FUNCSEL_NUM(uart, gpio_uart_rx));

    // Set the UART format
    uart_set_format(uart, PZEM_UART_DATA_BITS, PZEM_UART_STOP_BITS, PZEM_UART_PARITY);

    return true;
}

bool pzem004t_set_alarm_thr(uint16_t slave_addr, uint16_t alarm_thr) {
    if (uart == NULL) {
        return false; // UART not initialized
    }

    if ((slave_addr != PZEM_SLAVE_ADDR_GENERAL) && (slave_addr < PZEM_SLAVE_ADDR_MIN || slave_addr > PZEM_SLAVE_ADDR_MAX)) {
        return false; // Invalid slave address
    }

    if (!__send_command(slave_addr, PZEM_CMD_WSR, PZEM_WREG_ALARM_THR, alarm_thr)) {
        return false;
    }

    uint8_t rx_buffer[8];
    uint16_t bytes_read = __read_response(rx_buffer, sizeof(rx_buffer));
    if (bytes_read < 8) {
        if (__debug) {
            if ((bytes_read == ERROR_RESPONSE_LENGTH) && (rx_buffer[1] == PZEM_ERR_RESPONSE)) {
                printf("PZEM-004T: Error %02X\n", rx_buffer[2]);
            } else {
                printf("PZEM-004T: Response too short (%d bytes)\n", bytes_read);       
            }
        }
        return false;
    }

    // Check CRC
    if (!modbus_checkCRC(rx_buffer, bytes_read)) {
        if (__debug) {
            printf("PZEM-004T: CRC error\n");
        }
        return false;
    }
    if (__debug) {
        printf("PZEM-004T: Response OK\n");
    }

    return true;
}

bool pzem004t_set_slave_addr(uint16_t new_slave_addr) {
    if (uart == NULL) {
        return false; // UART not initialized
    }

    if ((new_slave_addr < PZEM_SLAVE_ADDR_MIN) || (new_slave_addr > PZEM_SLAVE_ADDR_MAX)) {
        return false; // Invalid slave address
    }

    if (!__send_command(PZEM_SLAVE_ADDR_GENERAL, PZEM_CMD_WSR, PZEM_WREG_SLAVE_ADDR, new_slave_addr)) {
        return false;
    }

    uint8_t rx_buffer[8];
    uint16_t bytes_read = __read_response(rx_buffer, sizeof(rx_buffer));
    if (bytes_read < 8) {
        if (__debug) {
            if ((bytes_read == ERROR_RESPONSE_LENGTH) && (rx_buffer[1] == PZEM_ERR_RESPONSE)) {
                printf("PZEM-004T: Error %02X\n", rx_buffer[2]);
            } else {
                printf("PZEM-004T: Response too short (%d bytes)\n", bytes_read);       
            }
        }
        return false;
    }

    // Check CRC
    if (!modbus_checkCRC(rx_buffer, bytes_read)) {
        if (__debug) {
            printf("PZEM-004T: CRC error\n");
        }
        return false;
    }
    if (__debug) {
        printf("PZEM-004T: Response OK\n");
    }

    return true;
}

bool pzem004t_read_data(uint16_t slave_addr, pzem004t_data_t *data) {
    if (uart == NULL) {
        return false; // UART not initialized
    }

    if ((slave_addr != PZEM_SLAVE_ADDR_GENERAL) && (slave_addr < PZEM_SLAVE_ADDR_MIN || slave_addr > PZEM_SLAVE_ADDR_MAX)) {
        return false; // Invalid slave address
    }

    if (!__send_command(slave_addr, PZEM_CMD_RIR, 0x0000, 10)) {
        return false;
    }

    uint8_t rx_buffer[CORRECT_RESPONSE_LENGTH];
    uint16_t bytes_read = __read_response(rx_buffer, sizeof(rx_buffer));
    if (bytes_read < CORRECT_RESPONSE_LENGTH) {
        if (__debug) {
            if ((bytes_read == ERROR_RESPONSE_LENGTH) && (rx_buffer[1] == PZEM_ERR_RESPONSE)) {
                printf("PZEM-004T: Error %02X\n", rx_buffer[2]);
            } else {
                printf("PZEM-004T: Response too short (%d bytes)\n", bytes_read);       
            }
        }
        return false;
    }

    // Check CRC
    if (!modbus_checkCRC(rx_buffer, bytes_read)) {
        if (__debug) {
            printf("PZEM-004T: CRC error\n");
        }
        return false;
    }
    if (__debug) {
        printf("PZEM-004T: Response OK\n");
    }

    data->voltage = (rx_buffer[VOLTAGE_OFFSET+1]) |
                    (rx_buffer[VOLTAGE_OFFSET] << 8);

    data->current = (rx_buffer[CURRENT_LOW_OFFSET+1]) |
                    (rx_buffer[CURRENT_LOW_OFFSET] << 8) |
                    (rx_buffer[CURRENT_HIGH_OFFSET+1] << 16) |
                    (rx_buffer[CURRENT_HIGH_OFFSET] << 24);

    data->power = (rx_buffer[POWER_LOW_OFFSET+1]) |
                  (rx_buffer[POWER_LOW_OFFSET] << 8) |
                  (rx_buffer[POWER_HIGH_OFFSET+1] << 16) |
                  (rx_buffer[POWER_HIGH_OFFSET] << 24);

    data->energy = (rx_buffer[ENERGY_LOW_OFFSET+1]) |
                   (rx_buffer[ENERGY_LOW_OFFSET] << 8) |
                   (rx_buffer[ENERGY_HIGH_OFFSET+1] << 16) |
                   (rx_buffer[ENERGY_HIGH_OFFSET] << 24);

    data->frequency = (rx_buffer[FREQUENCY_OFFSET+1]) |
                      (rx_buffer[FREQUENCY_OFFSET] << 8);

    data->power_factor = (rx_buffer[POWER_FACTOR_OFFSET+1]) |
                         (rx_buffer[POWER_FACTOR_OFFSET] << 8);

    data->alarm_status = (rx_buffer[ALARM_STATUS_OFFSET+1]) |
                         (rx_buffer[ALARM_STATUS_OFFSET] << 8);

    return true;
}