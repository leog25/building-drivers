/**
 * @file spi_hal_linux.c
 * @brief Linux Implementation of SPI Hardware Abstraction Layer
 * 
 * This file provides a Linux-specific implementation of the SPI HAL using
 * the Linux spidev interface. This demonstrates how the HAL abstracts
 * platform-specific details while providing a consistent interface.
 * 
 * @note This is a sample implementation for educational purposes.
 *       Production code would require additional error handling and features.
 */

#include "spi_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/time.h>
#include <errno.h>

/*=============================================================================
 * LINUX-SPECIFIC DEFINITIONS
 *=============================================================================*/
#define DEFAULT_SPI_DEVICE      "/dev/spidev0.0"
#define MAX_SPI_DEVICE_PATH     64

typedef struct {
    int fd;                     /* SPI device file descriptor */
    char device_path[MAX_SPI_DEVICE_PATH];  /* SPI device path */
    struct spi_ioc_transfer transfer;       /* SPI transfer structure */
} linux_spi_context_t;

/*=============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *=============================================================================*/
static spi_hal_error_t linux_configure_spi(linux_spi_context_t *linux_ctx, 
                                           const spi_hal_config_t *config);
static spi_hal_error_t linux_gpio_control(uint32_t pin, bool state);

/*=============================================================================
 * HAL FUNCTION IMPLEMENTATIONS
 *=============================================================================*/

spi_hal_error_t spi_hal_init(spi_hal_context_t *ctx, const spi_hal_config_t *config)
{
    if (!ctx || !config) {
        return SPI_HAL_ERROR_INVALID_PARAM;
    }
    
    // Allocate Linux-specific context
    linux_spi_context_t *linux_ctx = malloc(sizeof(linux_spi_context_t));
    if (!linux_ctx) {
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    memset(linux_ctx, 0, sizeof(linux_spi_context_t));
    
    // Set default SPI device path
    strncpy(linux_ctx->device_path, DEFAULT_SPI_DEVICE, MAX_SPI_DEVICE_PATH - 1);
    
    // Open SPI device
    linux_ctx->fd = open(linux_ctx->device_path, O_RDWR);
    if (linux_ctx->fd < 0) {
        printf("Failed to open SPI device %s: %s\n", 
               linux_ctx->device_path, strerror(errno));
        free(linux_ctx);
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    // Configure SPI settings
    spi_hal_error_t result = linux_configure_spi(linux_ctx, config);
    if (result != SPI_HAL_SUCCESS) {
        close(linux_ctx->fd);
        free(linux_ctx);
        return result;
    }
    
    // Initialize HAL context
    ctx->hw_handle = linux_ctx;
    ctx->config = *config;
    ctx->initialized = true;
    ctx->transaction_active = false;
    
    printf("SPI HAL initialized: %s @ %u Hz, mode %u\n", 
           linux_ctx->device_path, config->frequency, config->mode);
    
    return SPI_HAL_SUCCESS;
}

spi_hal_error_t spi_hal_deinit(spi_hal_context_t *ctx)
{
    if (!ctx || !ctx->initialized) {
        return SPI_HAL_ERROR_INVALID_PARAM;
    }
    
    linux_spi_context_t *linux_ctx = (linux_spi_context_t *)ctx->hw_handle;
    if (linux_ctx) {
        // Make sure transaction is not active
        if (ctx->transaction_active) {
            spi_hal_transaction_end(ctx);
        }
        
        // Close SPI device
        if (linux_ctx->fd >= 0) {
            close(linux_ctx->fd);
            linux_ctx->fd = -1;
        }
        
        // Free Linux-specific context
        free(linux_ctx);
        ctx->hw_handle = NULL;
    }
    
    ctx->initialized = false;
    printf("SPI HAL deinitialized\n");
    
    return SPI_HAL_SUCCESS;
}

spi_hal_error_t spi_hal_transaction_start(spi_hal_context_t *ctx)
{
    if (!ctx || !ctx->initialized) {
        return SPI_HAL_ERROR_INVALID_PARAM;
    }
    
    if (ctx->transaction_active) {
        return SPI_HAL_ERROR_BUSY;
    }
    
    // Assert chip select (active low)
    spi_hal_error_t result = linux_gpio_control(ctx->config.cs_pin, 
                                                ctx->config.cs_active_low);
    if (result != SPI_HAL_SUCCESS) {
        return result;
    }
    
    ctx->transaction_active = true;
    
    return SPI_HAL_SUCCESS;
}

spi_hal_error_t spi_hal_transaction_end(spi_hal_context_t *ctx)
{
    if (!ctx || !ctx->initialized) {
        return SPI_HAL_ERROR_INVALID_PARAM;
    }
    
    if (!ctx->transaction_active) {
        return SPI_HAL_SUCCESS;  // Already ended
    }
    
    // Deassert chip select
    spi_hal_error_t result = linux_gpio_control(ctx->config.cs_pin, 
                                                !ctx->config.cs_active_low);
    if (result != SPI_HAL_SUCCESS) {
        return result;
    }
    
    ctx->transaction_active = false;
    
    return SPI_HAL_SUCCESS;
}

spi_hal_error_t spi_hal_transfer(spi_hal_context_t *ctx, 
                                 const uint8_t *tx_data, 
                                 uint8_t *rx_data, 
                                 size_t length)
{
    if (!ctx || !ctx->initialized || length == 0) {
        return SPI_HAL_ERROR_INVALID_PARAM;
    }
    
    if (length > SPI_HAL_MAX_TRANSFER_SIZE) {
        return SPI_HAL_ERROR_INVALID_PARAM;
    }
    
    linux_spi_context_t *linux_ctx = (linux_spi_context_t *)ctx->hw_handle;
    if (!linux_ctx || linux_ctx->fd < 0) {
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    // Prepare SPI transfer structure
    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)tx_data,
        .rx_buf = (unsigned long)rx_data,
        .len = length,
        .speed_hz = ctx->config.frequency,
        .bits_per_word = ctx->config.bits_per_word,
        .delay_usecs = 0,
        .cs_change = 0,
    };
    
    // Perform SPI transfer
    int result = ioctl(linux_ctx->fd, SPI_IOC_MESSAGE(1), &transfer);
    if (result < 0) {
        printf("SPI transfer failed: %s\n", strerror(errno));
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    return SPI_HAL_SUCCESS;
}

spi_hal_error_t spi_hal_write(spi_hal_context_t *ctx, const uint8_t *data, size_t length)
{
    // For write-only operations, we don't care about received data
    return spi_hal_transfer(ctx, data, NULL, length);
}

spi_hal_error_t spi_hal_read(spi_hal_context_t *ctx, uint8_t *data, size_t length)
{
    // For read-only operations, we send dummy data (usually 0x00)
    uint8_t dummy_data[SPI_HAL_MAX_TRANSFER_SIZE];
    
    if (length > SPI_HAL_MAX_TRANSFER_SIZE) {
        return SPI_HAL_ERROR_INVALID_PARAM;
    }
    
    memset(dummy_data, 0x00, length);
    return spi_hal_transfer(ctx, dummy_data, data, length);
}

spi_hal_error_t spi_hal_write_read_byte(spi_hal_context_t *ctx, uint8_t tx_byte, uint8_t *rx_byte)
{
    if (!rx_byte) {
        return SPI_HAL_ERROR_INVALID_PARAM;
    }
    
    return spi_hal_transfer(ctx, &tx_byte, rx_byte, 1);
}

void spi_hal_delay_ms(uint32_t ms)
{
    usleep(ms * 1000);
}

void spi_hal_delay_us(uint32_t us)
{
    usleep(us);
}

uint32_t spi_hal_get_tick_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

/*=============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 *=============================================================================*/

static spi_hal_error_t linux_configure_spi(linux_spi_context_t *linux_ctx, 
                                           const spi_hal_config_t *config)
{
    int result;
    
    // Set SPI mode
    uint8_t mode = config->mode;
    result = ioctl(linux_ctx->fd, SPI_IOC_WR_MODE, &mode);
    if (result < 0) {
        printf("Failed to set SPI mode: %s\n", strerror(errno));
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    // Set bits per word
    uint8_t bits = config->bits_per_word;
    result = ioctl(linux_ctx->fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (result < 0) {
        printf("Failed to set bits per word: %s\n", strerror(errno));
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    // Set maximum speed
    uint32_t speed = config->frequency;
    result = ioctl(linux_ctx->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (result < 0) {
        printf("Failed to set SPI speed: %s\n", strerror(errno));
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    // Verify settings by reading them back
    result = ioctl(linux_ctx->fd, SPI_IOC_RD_MODE, &mode);
    if (result < 0 || mode != config->mode) {
        printf("SPI mode verification failed\n");
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    result = ioctl(linux_ctx->fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (result < 0 || bits != config->bits_per_word) {
        printf("SPI bits per word verification failed\n");
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    result = ioctl(linux_ctx->fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (result < 0 || speed != config->frequency) {
        printf("SPI speed verification failed\n");
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    printf("SPI configured: mode=%u, bits=%u, speed=%u Hz\n", 
           mode, bits, speed);
    
    return SPI_HAL_SUCCESS;
}

static spi_hal_error_t linux_gpio_control(uint32_t pin, bool state)
{
    // This is a simplified GPIO control implementation
    // In a real implementation, you would use proper GPIO control methods
    // such as /sys/class/gpio or libgpiod
    
    char gpio_path[64];
    char value_str[2];
    int fd;
    
    // Export GPIO if not already exported
    snprintf(gpio_path, sizeof(gpio_path), "/sys/class/gpio/gpio%u/value", pin);
    fd = open(gpio_path, O_WRONLY);
    if (fd < 0) {
        // Try to export the GPIO
        fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd >= 0) {
            char pin_str[8];
            snprintf(pin_str, sizeof(pin_str), "%u", pin);
            write(fd, pin_str, strlen(pin_str));
            close(fd);
            
            // Set direction to output
            snprintf(gpio_path, sizeof(gpio_path), "/sys/class/gpio/gpio%u/direction", pin);
            fd = open(gpio_path, O_WRONLY);
            if (fd >= 0) {
                write(fd, "out", 3);
                close(fd);
            }
            
            // Try to open value file again
            snprintf(gpio_path, sizeof(gpio_path), "/sys/class/gpio/gpio%u/value", pin);
            fd = open(gpio_path, O_WRONLY);
        }
    }
    
    if (fd < 0) {
        printf("Failed to control GPIO pin %u: %s\n", pin, strerror(errno));
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    // Set GPIO value
    value_str[0] = state ? '1' : '0';
    value_str[1] = '\0';
    
    ssize_t bytes_written = write(fd, value_str, 1);
    close(fd);
    
    if (bytes_written != 1) {
        printf("Failed to set GPIO pin %u to %s\n", pin, state ? "high" : "low");
        return SPI_HAL_ERROR_HARDWARE_FAULT;
    }
    
    return SPI_HAL_SUCCESS;
} 