/**
 * @file spi_device_driver.h
 * @brief SPI Device Driver Interface
 * 
 * This driver provides high-level functions for communicating with SPI devices
 * using the patterns observed in the SPI transfer logs. It builds upon the
 * HAL interface to provide device-specific functionality.
 */

#ifndef SPI_DEVICE_DRIVER_H
#define SPI_DEVICE_DRIVER_H

#include "spi_hal.h"
#include "registers.h"
#include <stdint.h>
#include <stdbool.h>

/*=============================================================================
 * DRIVER CONSTANTS
 *=============================================================================*/
#define DRIVER_MAX_RETRIES          3
#define DRIVER_INIT_TIMEOUT_MS      100
#define DRIVER_DATA_TIMEOUT_MS      50
#define DRIVER_CALIBRATION_TIMEOUT_MS 5000

/*=============================================================================
 * DRIVER ERROR CODES
 *=============================================================================*/
typedef enum {
    DRIVER_SUCCESS = 0,
    DRIVER_ERROR_HAL_FAILURE,
    DRIVER_ERROR_DEVICE_NOT_FOUND,
    DRIVER_ERROR_INVALID_DEVICE_ID,
    DRIVER_ERROR_INITIALIZATION_FAILED,
    DRIVER_ERROR_COMMUNICATION_TIMEOUT,
    DRIVER_ERROR_INVALID_PARAMETER,
    DRIVER_ERROR_DEVICE_BUSY,
    DRIVER_ERROR_CALIBRATION_FAILED,
    DRIVER_ERROR_SELF_TEST_FAILED,
    DRIVER_ERROR_FIFO_OVERFLOW,
    DRIVER_ERROR_DATA_NOT_READY
} driver_error_t;

/*=============================================================================
 * DRIVER STRUCTURES
 *=============================================================================*/
typedef struct {
    spi_hal_context_t hal_ctx;     /* HAL context */
    device_config_t config;        /* Device configuration */
    bool initialized;              /* Driver initialization status */
    bool calibrated;               /* Calibration status */
    uint32_t last_read_time;       /* Last data read timestamp */
    uint8_t error_count;           /* Error counter for diagnostics */
} spi_device_t;

typedef struct {
    bool enable_interrupts;        /* Enable interrupt-driven operation */
    bool use_fifo;                 /* Use FIFO buffering */
    uint8_t fifo_threshold;        /* FIFO threshold level */
    device_range_t range;          /* Measurement range */
    device_resolution_t resolution; /* Data resolution */
    data_rate_t data_rate;         /* Sampling rate */
    bool auto_calibrate;           /* Automatic calibration on init */
    uint32_t calibration_timeout_ms; /* Calibration timeout */
} driver_init_config_t;

/*=============================================================================
 * DRIVER FUNCTION PROTOTYPES
 *=============================================================================*/

/**
 * @brief Initialize SPI device driver
 * @param device Pointer to device structure
 * @param spi_config Pointer to SPI HAL configuration
 * @param init_config Pointer to driver initialization configuration
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_init(spi_device_t *device, 
                               const spi_hal_config_t *spi_config,
                               const driver_init_config_t *init_config);

/**
 * @brief Deinitialize SPI device driver
 * @param device Pointer to device structure
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_deinit(spi_device_t *device);

/**
 * @brief Software reset device
 * @param device Pointer to device structure
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_reset(spi_device_t *device);

/**
 * @brief Read device ID and verify device presence
 * @param device Pointer to device structure
 * @param device_id Pointer to store device ID
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_read_id(spi_device_t *device, uint8_t *device_id);

/**
 * @brief Read single register
 * @param device Pointer to device structure
 * @param reg_addr Register address
 * @param value Pointer to store register value
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_read_register(spi_device_t *device, uint8_t reg_addr, uint8_t *value);

/**
 * @brief Write single register
 * @param device Pointer to device structure
 * @param reg_addr Register address
 * @param value Value to write
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_write_register(spi_device_t *device, uint8_t reg_addr, uint8_t value);

/**
 * @brief Read multiple registers (burst read)
 * @param device Pointer to device structure
 * @param start_addr Starting register address
 * @param data Pointer to buffer to store data
 * @param length Number of bytes to read
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_read_registers(spi_device_t *device, 
                                         uint8_t start_addr, 
                                         uint8_t *data, 
                                         size_t length);

/**
 * @brief Write multiple registers (burst write)
 * @param device Pointer to device structure
 * @param start_addr Starting register address
 * @param data Pointer to data to write
 * @param length Number of bytes to write
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_write_registers(spi_device_t *device, 
                                          uint8_t start_addr, 
                                          const uint8_t *data, 
                                          size_t length);

/**
 * @brief Read device status register
 * @param device Pointer to device structure
 * @param status Pointer to store status value
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_read_status(spi_device_t *device, uint8_t *status);

/**
 * @brief Check if new data is available
 * @param device Pointer to device structure
 * @param data_ready Pointer to store data ready flag
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_is_data_ready(spi_device_t *device, bool *data_ready);

/**
 * @brief Read sensor data (X, Y, Z axes)
 * @param device Pointer to device structure
 * @param data Pointer to store sensor data
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_read_data(spi_device_t *device, sensor_data_t *data);

/**
 * @brief Read temperature data
 * @param device Pointer to device structure
 * @param temperature Pointer to store temperature value
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_read_temperature(spi_device_t *device, int16_t *temperature);

/**
 * @brief Configure device operating mode
 * @param device Pointer to device structure
 * @param mode Operating mode to set
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_set_mode(spi_device_t *device, device_mode_t mode);

/**
 * @brief Configure device measurement range
 * @param device Pointer to device structure
 * @param range Measurement range to set
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_set_range(spi_device_t *device, device_range_t range);

/**
 * @brief Configure device data rate
 * @param device Pointer to device structure
 * @param data_rate Data rate to set
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_set_data_rate(spi_device_t *device, data_rate_t data_rate);

/**
 * @brief Configure interrupts
 * @param device Pointer to device structure
 * @param interrupt_mask Interrupt enable mask
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_configure_interrupts(spi_device_t *device, uint8_t interrupt_mask);

/**
 * @brief Configure FIFO settings
 * @param device Pointer to device structure
 * @param fifo_config FIFO configuration value
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_configure_fifo(spi_device_t *device, uint8_t fifo_config);

/**
 * @brief Read FIFO status
 * @param device Pointer to device structure
 * @param fifo_status Pointer to store FIFO status
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_read_fifo_status(spi_device_t *device, uint8_t *fifo_status);

/**
 * @brief Read data from FIFO
 * @param device Pointer to device structure
 * @param data Pointer to buffer to store FIFO data
 * @param length Number of bytes to read
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_read_fifo(spi_device_t *device, uint8_t *data, size_t length);

/**
 * @brief Set threshold values for motion detection
 * @param device Pointer to device structure
 * @param high_threshold High threshold value
 * @param low_threshold Low threshold value
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_set_thresholds(spi_device_t *device, 
                                         uint8_t high_threshold, 
                                         uint8_t low_threshold);

/**
 * @brief Perform device self-test
 * @param device Pointer to device structure
 * @param test_passed Pointer to store test result
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_self_test(spi_device_t *device, bool *test_passed);

/**
 * @brief Perform device calibration
 * @param device Pointer to device structure
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_calibrate(spi_device_t *device);

/**
 * @brief Enter power-down mode
 * @param device Pointer to device structure
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_power_down(spi_device_t *device);

/**
 * @brief Enter standby mode
 * @param device Pointer to device structure
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_standby(spi_device_t *device);

/**
 * @brief Wake up device from power-down or standby
 * @param device Pointer to device structure
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_wakeup(spi_device_t *device);

/**
 * @brief Get device configuration
 * @param device Pointer to device structure
 * @param config Pointer to store device configuration
 * @return DRIVER_SUCCESS on success, error code otherwise
 */
driver_error_t spi_device_get_config(spi_device_t *device, device_config_t *config);

/**
 * @brief Get error count for diagnostics
 * @param device Pointer to device structure
 * @return Error count
 */
uint8_t spi_device_get_error_count(spi_device_t *device);

/**
 * @brief Clear error count
 * @param device Pointer to device structure
 */
void spi_device_clear_error_count(spi_device_t *device);

/*=============================================================================
 * DRIVER UTILITY MACROS
 *=============================================================================*/
#define DRIVER_DEFAULT_INIT_CONFIG() {       \
    .enable_interrupts = true,               \
    .use_fifo = false,                       \
    .fifo_threshold = 1,                     \
    .range = DEVICE_RANGE_2G,                \
    .resolution = DEVICE_RESOLUTION_12BIT,   \
    .data_rate = DATA_RATE_100HZ,            \
    .auto_calibrate = true,                  \
    .calibration_timeout_ms = DRIVER_CALIBRATION_TIMEOUT_MS \
}

#define DRIVER_INIT_DEVICE() {               \
    .hal_ctx = SPI_HAL_INIT_CONTEXT(),       \
    .config = {0},                           \
    .initialized = false,                    \
    .calibrated = false,                     \
    .last_read_time = 0,                     \
    .error_count = 0                         \
}

/**
 * @brief Check if device is initialized
 */
#define DRIVER_IS_INITIALIZED(device) ((device)->initialized)

/**
 * @brief Check if device is calibrated
 */
#define DRIVER_IS_CALIBRATED(device) ((device)->calibrated)

#endif /* SPI_DEVICE_DRIVER_H */ 