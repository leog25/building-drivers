/**
 * @file spi_hal.h
 * @brief Hardware Abstraction Layer for SPI Device Communication
 * 
 * This HAL provides a platform-agnostic interface for SPI communication
 * with sensor devices. It abstracts the underlying SPI hardware details.
 */

#ifndef SPI_HAL_H
#define SPI_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*=============================================================================
 * HAL CONFIGURATION
 *=============================================================================*/
#define SPI_HAL_MAX_TRANSFER_SIZE   64
#define SPI_HAL_TIMEOUT_MS          1000

/*=============================================================================
 * HAL ERROR CODES
 *=============================================================================*/
typedef enum {
    SPI_HAL_SUCCESS = 0,
    SPI_HAL_ERROR_TIMEOUT,
    SPI_HAL_ERROR_INVALID_PARAM,
    SPI_HAL_ERROR_HARDWARE_FAULT,
    SPI_HAL_ERROR_BUSY,
    SPI_HAL_ERROR_NOT_INITIALIZED
} spi_hal_error_t;

/*=============================================================================
 * HAL STRUCTURES
 *=============================================================================*/
typedef struct {
    uint32_t frequency;         /* SPI clock frequency in Hz */
    uint8_t mode;              /* SPI mode (0-3) */
    uint8_t bits_per_word;     /* Bits per SPI word (8, 16, 32) */
    bool cs_active_low;        /* Chip select polarity */
    uint32_t cs_pin;           /* Chip select GPIO pin */
    uint32_t timeout_ms;       /* Transaction timeout in milliseconds */
} spi_hal_config_t;

typedef struct {
    void *hw_handle;           /* Platform-specific hardware handle */
    spi_hal_config_t config;   /* SPI configuration */
    bool initialized;          /* Initialization status */
    bool transaction_active;   /* Transaction in progress flag */
} spi_hal_context_t;

/*=============================================================================
 * HAL FUNCTION PROTOTYPES
 *=============================================================================*/

/**
 * @brief Initialize SPI HAL
 * @param ctx Pointer to HAL context structure
 * @param config Pointer to SPI configuration
 * @return SPI_HAL_SUCCESS on success, error code otherwise
 */
spi_hal_error_t spi_hal_init(spi_hal_context_t *ctx, const spi_hal_config_t *config);

/**
 * @brief Deinitialize SPI HAL
 * @param ctx Pointer to HAL context structure
 * @return SPI_HAL_SUCCESS on success, error code otherwise
 */
spi_hal_error_t spi_hal_deinit(spi_hal_context_t *ctx);

/**
 * @brief Start SPI transaction (assert chip select)
 * @param ctx Pointer to HAL context structure
 * @return SPI_HAL_SUCCESS on success, error code otherwise
 */
spi_hal_error_t spi_hal_transaction_start(spi_hal_context_t *ctx);

/**
 * @brief End SPI transaction (deassert chip select)
 * @param ctx Pointer to HAL context structure
 * @return SPI_HAL_SUCCESS on success, error code otherwise
 */
spi_hal_error_t spi_hal_transaction_end(spi_hal_context_t *ctx);

/**
 * @brief Transfer data over SPI
 * @param ctx Pointer to HAL context structure
 * @param tx_data Pointer to transmit data buffer
 * @param rx_data Pointer to receive data buffer
 * @param length Number of bytes to transfer
 * @return SPI_HAL_SUCCESS on success, error code otherwise
 */
spi_hal_error_t spi_hal_transfer(spi_hal_context_t *ctx, 
                                 const uint8_t *tx_data, 
                                 uint8_t *rx_data, 
                                 size_t length);

/**
 * @brief Write data over SPI (transmit only)
 * @param ctx Pointer to HAL context structure
 * @param data Pointer to data buffer to transmit
 * @param length Number of bytes to transmit
 * @return SPI_HAL_SUCCESS on success, error code otherwise
 */
spi_hal_error_t spi_hal_write(spi_hal_context_t *ctx, const uint8_t *data, size_t length);

/**
 * @brief Read data over SPI (receive only)
 * @param ctx Pointer to HAL context structure
 * @param data Pointer to buffer to store received data
 * @param length Number of bytes to receive
 * @return SPI_HAL_SUCCESS on success, error code otherwise
 */
spi_hal_error_t spi_hal_read(spi_hal_context_t *ctx, uint8_t *data, size_t length);

/**
 * @brief Write single byte and read response
 * @param ctx Pointer to HAL context structure
 * @param tx_byte Byte to transmit
 * @param rx_byte Pointer to store received byte
 * @return SPI_HAL_SUCCESS on success, error code otherwise
 */
spi_hal_error_t spi_hal_write_read_byte(spi_hal_context_t *ctx, uint8_t tx_byte, uint8_t *rx_byte);

/**
 * @brief Platform-specific delay function
 * @param ms Delay in milliseconds
 */
void spi_hal_delay_ms(uint32_t ms);

/**
 * @brief Platform-specific microsecond delay function
 * @param us Delay in microseconds
 */
void spi_hal_delay_us(uint32_t us);

/**
 * @brief Get system tick count in milliseconds
 * @return Current system tick count
 */
uint32_t spi_hal_get_tick_ms(void);

/*=============================================================================
 * HAL UTILITY MACROS
 *=============================================================================*/
#define SPI_HAL_DEFAULT_CONFIG() {          \
    .frequency = 1000000,                   \
    .mode = 0,                              \
    .bits_per_word = 8,                     \
    .cs_active_low = true,                  \
    .cs_pin = 0,                            \
    .timeout_ms = SPI_HAL_TIMEOUT_MS        \
}

#define SPI_HAL_INIT_CONTEXT() {            \
    .hw_handle = NULL,                      \
    .config = SPI_HAL_DEFAULT_CONFIG(),     \
    .initialized = false,                   \
    .transaction_active = false             \
}

#endif /* SPI_HAL_H */ 