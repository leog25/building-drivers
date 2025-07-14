/**
 * @file spi_device_driver.c
 * @brief SPI Device Driver Implementation
 * 
 * This file implements the SPI device driver functions using the patterns
 * observed in the SPI transfer logs. It demonstrates proper initialization,
 * configuration, data reading, and error handling sequences.
 */

#include "spi_device_driver.h"
#include <string.h>

/*=============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *=============================================================================*/
static driver_error_t driver_wait_for_device_ready(spi_device_t *device, uint32_t timeout_ms);
static driver_error_t driver_poll_status_bit(spi_device_t *device, uint8_t bit_mask, 
                                             bool expected_state, uint32_t timeout_ms);
static driver_error_t driver_perform_power_on_sequence(spi_device_t *device);
static driver_error_t driver_configure_device(spi_device_t *device, 
                                              const driver_init_config_t *init_config);

/*=============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 *=============================================================================*/

driver_error_t spi_device_init(spi_device_t *device, 
                               const spi_hal_config_t *spi_config,
                               const driver_init_config_t *init_config)
{
    driver_error_t result = DRIVER_SUCCESS;
    
    if (!device || !spi_config || !init_config) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Initialize device structure
    memset(device, 0, sizeof(spi_device_t));
    
    // Initialize HAL
    spi_hal_error_t hal_result = spi_hal_init(&device->hal_ctx, spi_config);
    if (hal_result != SPI_HAL_SUCCESS) {
        return DRIVER_ERROR_HAL_FAILURE;
    }
    
    // Perform power-on sequence as shown in SPI log
    result = driver_perform_power_on_sequence(device);
    if (result != DRIVER_SUCCESS) {
        spi_hal_deinit(&device->hal_ctx);
        return result;
    }
    
    // Configure device according to init configuration
    result = driver_configure_device(device, init_config);
    if (result != DRIVER_SUCCESS) {
        spi_hal_deinit(&device->hal_ctx);
        return result;
    }
    
    // Perform calibration if requested
    if (init_config->auto_calibrate) {
        result = spi_device_calibrate(device);
        if (result != DRIVER_SUCCESS) {
            spi_hal_deinit(&device->hal_ctx);
            return result;
        }
        device->calibrated = true;
    }
    
    // Mark device as initialized
    device->initialized = true;
    
    return DRIVER_SUCCESS;
}

driver_error_t spi_device_reset(spi_device_t *device)
{
    if (!device || !device->initialized) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Software reset command as shown in SPI log: FF
    uint8_t reset_cmd = CMD_RESET;
    
    spi_hal_error_t hal_result = spi_hal_transaction_start(&device->hal_ctx);
    if (hal_result != SPI_HAL_SUCCESS) {
        device->error_count++;
        return DRIVER_ERROR_HAL_FAILURE;
    }
    
    hal_result = spi_hal_write(&device->hal_ctx, &reset_cmd, 1);
    spi_hal_transaction_end(&device->hal_ctx);
    
    if (hal_result != SPI_HAL_SUCCESS) {
        device->error_count++;
        return DRIVER_ERROR_HAL_FAILURE;
    }
    
    // Wait for device to be ready after reset
    spi_hal_delay_ms(10);
    
    return driver_wait_for_device_ready(device, DRIVER_INIT_TIMEOUT_MS);
}

driver_error_t spi_device_read_id(spi_device_t *device, uint8_t *device_id)
{
    if (!device || !device_id) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Read device ID as shown in SPI log: 80 00 (read register 0x00)
    return spi_device_read_register(device, REG_DEVICE_ID, device_id);
}

driver_error_t spi_device_read_register(spi_device_t *device, uint8_t reg_addr, uint8_t *value)
{
    if (!device || !value || !IS_READABLE_REGISTER(reg_addr)) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Create read command as shown in SPI log pattern
    uint8_t tx_data[2] = {MAKE_READ_CMD(reg_addr), 0x00};
    uint8_t rx_data[2] = {0};
    
    spi_hal_error_t hal_result = spi_hal_transaction_start(&device->hal_ctx);
    if (hal_result != SPI_HAL_SUCCESS) {
        device->error_count++;
        return DRIVER_ERROR_HAL_FAILURE;
    }
    
    hal_result = spi_hal_transfer(&device->hal_ctx, tx_data, rx_data, 2);
    spi_hal_transaction_end(&device->hal_ctx);
    
    if (hal_result != SPI_HAL_SUCCESS) {
        device->error_count++;
        return DRIVER_ERROR_HAL_FAILURE;
    }
    
    *value = rx_data[1];
    return DRIVER_SUCCESS;
}

driver_error_t spi_device_write_register(spi_device_t *device, uint8_t reg_addr, uint8_t value)
{
    if (!device || !IS_WRITABLE_REGISTER(reg_addr)) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Create write command as shown in SPI log pattern
    uint8_t tx_data[2] = {MAKE_WRITE_CMD(reg_addr), value};
    
    spi_hal_error_t hal_result = spi_hal_transaction_start(&device->hal_ctx);
    if (hal_result != SPI_HAL_SUCCESS) {
        device->error_count++;
        return DRIVER_ERROR_HAL_FAILURE;
    }
    
    hal_result = spi_hal_write(&device->hal_ctx, tx_data, 2);
    spi_hal_transaction_end(&device->hal_ctx);
    
    if (hal_result != SPI_HAL_SUCCESS) {
        device->error_count++;
        return DRIVER_ERROR_HAL_FAILURE;
    }
    
    return DRIVER_SUCCESS;
}

driver_error_t spi_device_read_registers(spi_device_t *device, 
                                         uint8_t start_addr, 
                                         uint8_t *data, 
                                         size_t length)
{
    if (!device || !data || length == 0 || !IS_READABLE_REGISTER(start_addr)) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Burst read command as shown in SPI log: D0 (0xC0|0x10) for data registers
    uint8_t cmd = MAKE_BURST_READ_CMD(start_addr);
    
    spi_hal_error_t hal_result = spi_hal_transaction_start(&device->hal_ctx);
    if (hal_result != SPI_HAL_SUCCESS) {
        device->error_count++;
        return DRIVER_ERROR_HAL_FAILURE;
    }
    
    // Write command byte
    hal_result = spi_hal_write(&device->hal_ctx, &cmd, 1);
    if (hal_result == SPI_HAL_SUCCESS) {
        // Read data bytes
        hal_result = spi_hal_read(&device->hal_ctx, data, length);
    }
    
    spi_hal_transaction_end(&device->hal_ctx);
    
    if (hal_result != SPI_HAL_SUCCESS) {
        device->error_count++;
        return DRIVER_ERROR_HAL_FAILURE;
    }
    
    return DRIVER_SUCCESS;
}

driver_error_t spi_device_read_status(spi_device_t *device, uint8_t *status)
{
    if (!device || !status) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Read status register as frequently shown in SPI log: 84 00
    return spi_device_read_register(device, REG_STATUS, status);
}

driver_error_t spi_device_is_data_ready(spi_device_t *device, bool *data_ready)
{
    if (!device || !data_ready) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    uint8_t status;
    driver_error_t result = spi_device_read_status(device, &status);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    *data_ready = (status & STATUS_DATA_READY) != 0;
    return DRIVER_SUCCESS;
}

driver_error_t spi_device_read_data(spi_device_t *device, sensor_data_t *data)
{
    if (!device || !data) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Read X, Y, Z data using burst read as shown in SPI log
    uint8_t raw_data[6];
    driver_error_t result = spi_device_read_registers(device, REG_DATA_X_LOW, raw_data, 6);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    // Convert raw data to 16-bit signed values
    data->x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    data->y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    data->z = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    
    // Update last read time
    device->last_read_time = spi_hal_get_tick_ms();
    
    return DRIVER_SUCCESS;
}

driver_error_t spi_device_read_temperature(spi_device_t *device, int16_t *temperature)
{
    if (!device || !temperature) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    uint8_t temp_data;
    driver_error_t result = spi_device_read_register(device, REG_TEMPERATURE, &temp_data);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    *temperature = (int16_t)temp_data;
    return DRIVER_SUCCESS;
}

driver_error_t spi_device_calibrate(spi_device_t *device)
{
    if (!device) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Start calibration as shown in SPI log: 0C 01
    driver_error_t result = spi_device_write_register(device, REG_CALIBRATION, 0x01);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    // Wait for calibration to complete
    spi_hal_delay_ms(100);
    
    // Poll calibration status
    result = driver_poll_status_bit(device, STATUS_DEVICE_READY, true, 
                                   DRIVER_CALIBRATION_TIMEOUT_MS);
    if (result != DRIVER_SUCCESS) {
        return DRIVER_ERROR_CALIBRATION_FAILED;
    }
    
    return DRIVER_SUCCESS;
}

driver_error_t spi_device_self_test(spi_device_t *device, bool *test_passed)
{
    if (!device || !test_passed) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Enable self-test as shown in SPI log: 0D 20
    driver_error_t result = spi_device_write_register(device, REG_TEST_MODE, 0x20);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    // Wait for self-test to complete
    spi_hal_delay_ms(100);
    
    // Read status to check self-test result
    uint8_t status;
    result = spi_device_read_status(device, &status);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    *test_passed = (status & STATUS_SELF_TEST_OK) != 0;
    
    // Disable self-test mode: 0D 00
    spi_device_write_register(device, REG_TEST_MODE, 0x00);
    
    return DRIVER_SUCCESS;
}

driver_error_t spi_device_power_down(spi_device_t *device)
{
    if (!device) {
        return DRIVER_ERROR_INVALID_PARAMETER;
    }
    
    // Power down sequence as shown in SPI log
    // 1. Disable interrupts: 05 00
    driver_error_t result = spi_device_write_register(device, REG_INTERRUPT, 0x00);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    // 2. Set power down mode: 03 00
    result = spi_device_write_register(device, REG_MODE, MODE_POWER_DOWN);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    // 3. Set standby power management: 0E 40
    result = spi_device_write_register(device, REG_POWER_MGMT, POWER_STANDBY);
    
    return result;
}

uint8_t spi_device_get_error_count(spi_device_t *device)
{
    if (!device) {
        return 0;
    }
    return device->error_count;
}

void spi_device_clear_error_count(spi_device_t *device)
{
    if (device) {
        device->error_count = 0;
    }
}

/*=============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 *=============================================================================*/

static driver_error_t driver_wait_for_device_ready(spi_device_t *device, uint32_t timeout_ms)
{
    return driver_poll_status_bit(device, STATUS_DEVICE_READY, true, timeout_ms);
}

static driver_error_t driver_poll_status_bit(spi_device_t *device, uint8_t bit_mask, 
                                             bool expected_state, uint32_t timeout_ms)
{
    uint32_t start_time = spi_hal_get_tick_ms();
    
    while ((spi_hal_get_tick_ms() - start_time) < timeout_ms) {
        uint8_t status;
        driver_error_t result = spi_device_read_status(device, &status);
        if (result != DRIVER_SUCCESS) {
            return result;
        }
        
        bool bit_state = (status & bit_mask) != 0;
        if (bit_state == expected_state) {
            return DRIVER_SUCCESS;
        }
        
        spi_hal_delay_ms(1);
    }
    
    return DRIVER_ERROR_COMMUNICATION_TIMEOUT;
}

static driver_error_t driver_perform_power_on_sequence(spi_device_t *device)
{
    driver_error_t result;
    
    // 1. Software reset (FF)
    result = spi_device_reset(device);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    // 2. Check device ID (80 00)
    uint8_t device_id;
    result = spi_device_read_id(device, &device_id);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    if (device_id != DEVICE_ID_EXPECTED) {
        return DRIVER_ERROR_INVALID_DEVICE_ID;
    }
    
    // 3. Read version register (81 00)
    uint8_t version;
    result = spi_device_read_register(device, REG_VERSION, &version);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    device->config.device_id = device_id;
    device->config.version = version;
    
    return DRIVER_SUCCESS;
}

static driver_error_t driver_configure_device(spi_device_t *device, 
                                              const driver_init_config_t *init_config)
{
    driver_error_t result;
    
    // Configure device following the pattern from SPI log
    
    // 1. Set config register (02 81): Enable device with range setting
    uint8_t config_val = CONFIG_ENABLE | (init_config->range & CONFIG_RANGE_MASK);
    result = spi_device_write_register(device, REG_CONFIG, config_val);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    // 2. Set mode register (03 0A): Active mode with resolution
    uint8_t mode_val = MODE_ACTIVE | 
                       ((init_config->resolution << 2) & MODE_RESOLUTION_MASK);
    result = spi_device_write_register(device, REG_MODE, mode_val);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    // 3. Set data rate (08 04): Configure sampling rate
    result = spi_device_write_register(device, REG_DATA_RATE, 
                                      init_config->data_rate & DATA_RATE_MASK);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    // 4. Configure interrupts if enabled (05 81)
    if (init_config->enable_interrupts) {
        uint8_t int_config = INT_DATA_READY_EN | INT_ENABLE;
        result = spi_device_write_register(device, REG_INTERRUPT, int_config);
        if (result != DRIVER_SUCCESS) {
            return result;
        }
    }
    
    // 5. Set power management (0E 00): Normal power mode
    result = spi_device_write_register(device, REG_POWER_MGMT, 
                                      POWER_CLKSEL_INTERNAL);
    if (result != DRIVER_SUCCESS) {
        return result;
    }
    
    // 6. Configure FIFO if enabled (06 05)
    if (init_config->use_fifo) {
        uint8_t fifo_config = FIFO_MODE_FIFO | 
                             ((init_config->fifo_threshold << 2) & FIFO_THRESHOLD_MASK);
        result = spi_device_write_register(device, REG_FIFO_CONFIG, fifo_config);
        if (result != DRIVER_SUCCESS) {
            return result;
        }
    }
    
    // Store configuration
    device->config.range = init_config->range;
    device->config.resolution = init_config->resolution;
    device->config.data_rate = init_config->data_rate;
    device->config.interrupt_enabled = init_config->enable_interrupts;
    device->config.fifo_enabled = init_config->use_fifo;
    
    return DRIVER_SUCCESS;
} 