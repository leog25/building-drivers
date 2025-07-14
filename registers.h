/**
 * @file registers.h
 * @brief Sample SPI Device Register Definitions and Opcodes
 * 
 * This file contains register addresses, opcodes, bit field definitions,
 * and configuration constants for a typical SPI device driver.
 * 
 * @note This is a sample/template file for educational purposes
 */

#ifndef REGISTERS_H
#define REGISTERS_H

#include <stdint.h>

/*=============================================================================
 * DEVICE IDENTIFICATION
 *=============================================================================*/
#define DEVICE_ID_EXPECTED          0x42
#define DEVICE_VERSION_MASK         0x0F
#define MANUFACTURER_ID             0x1A

/*=============================================================================
 * SPI COMMAND OPCODES
 *=============================================================================*/
#define CMD_READ_REG                0x80    /* Read register (OR with address) */
#define CMD_WRITE_REG               0x00    /* Write register (OR with address) */
#define CMD_READ_FIFO               0x3F    /* Read FIFO data */
#define CMD_WRITE_FIFO              0x40    /* Write FIFO data */
#define CMD_RESET                   0xFF    /* Software reset */
#define CMD_NOP                     0x00    /* No operation */

/* Multi-byte operations */
#define CMD_BURST_READ              0xC0    /* Burst read (OR with start address) */
#define CMD_BURST_WRITE             0x80    /* Burst write (OR with start address) */

/*=============================================================================
 * REGISTER ADDRESSES
 *=============================================================================*/
#define REG_DEVICE_ID               0x00    /* Device identification register */
#define REG_VERSION                 0x01    /* Version register */
#define REG_CONFIG                  0x02    /* Main configuration register */
#define REG_MODE                    0x03    /* Operating mode register */
#define REG_STATUS                  0x04    /* Status register */
#define REG_INTERRUPT               0x05    /* Interrupt control register */
#define REG_FIFO_CONFIG             0x06    /* FIFO configuration */
#define REG_FIFO_STATUS             0x07    /* FIFO status */
#define REG_DATA_RATE               0x08    /* Data rate configuration */
#define REG_FILTER                  0x09    /* Digital filter settings */
#define REG_THRESHOLD_HIGH          0x0A    /* High threshold */
#define REG_THRESHOLD_LOW           0x0B    /* Low threshold */
#define REG_CALIBRATION             0x0C    /* Calibration register */
#define REG_TEST_MODE               0x0D    /* Test mode register */
#define REG_POWER_MGMT              0x0E    /* Power management */
#define REG_GPIO_CONFIG             0x0F    /* GPIO configuration */

/* Data registers */
#define REG_DATA_X_LOW              0x10    /* X-axis data low byte */
#define REG_DATA_X_HIGH             0x11    /* X-axis data high byte */
#define REG_DATA_Y_LOW              0x12    /* Y-axis data low byte */
#define REG_DATA_Y_HIGH             0x13    /* Y-axis data high byte */
#define REG_DATA_Z_LOW              0x14    /* Z-axis data low byte */
#define REG_DATA_Z_HIGH             0x15    /* Z-axis data high byte */
#define REG_TEMPERATURE             0x16    /* Temperature data */

/*=============================================================================
 * REGISTER BIT DEFINITIONS
 *=============================================================================*/

/* REG_CONFIG (0x02) */
#define CONFIG_ENABLE               (1 << 7)    /* Device enable */
#define CONFIG_RESET                (1 << 6)    /* Software reset */
#define CONFIG_SELF_TEST            (1 << 5)    /* Self-test enable */
#define CONFIG_INT_ENABLE           (1 << 4)    /* Interrupt enable */
#define CONFIG_DATA_READY           (1 << 3)    /* Data ready mode */
#define CONFIG_RANGE_MASK           (0x07)      /* Range selection mask */
#define CONFIG_RANGE_2G             (0x00)      /* ±2g range */
#define CONFIG_RANGE_4G             (0x01)      /* ±4g range */
#define CONFIG_RANGE_8G             (0x02)      /* ±8g range */
#define CONFIG_RANGE_16G            (0x03)      /* ±16g range */

/* REG_MODE (0x03) */
#define MODE_POWER_DOWN             (0x00)      /* Power down mode */
#define MODE_STANDBY                (0x01)      /* Standby mode */
#define MODE_ACTIVE                 (0x02)      /* Active mode */
#define MODE_CONTINUOUS             (0x03)      /* Continuous mode */
#define MODE_MASK                   (0x03)      /* Mode selection mask */
#define MODE_RESOLUTION_8BIT        (0x00 << 2) /* 8-bit resolution */
#define MODE_RESOLUTION_10BIT       (0x01 << 2) /* 10-bit resolution */
#define MODE_RESOLUTION_12BIT       (0x02 << 2) /* 12-bit resolution */
#define MODE_RESOLUTION_14BIT       (0x03 << 2) /* 14-bit resolution */
#define MODE_RESOLUTION_MASK        (0x03 << 2) /* Resolution mask */

/* REG_STATUS (0x04) */
#define STATUS_DATA_READY           (1 << 7)    /* New data available */
#define STATUS_FIFO_FULL            (1 << 6)    /* FIFO full */
#define STATUS_FIFO_EMPTY           (1 << 5)    /* FIFO empty */
#define STATUS_OVERRUN              (1 << 4)    /* Data overrun */
#define STATUS_THRESHOLD_HIGH       (1 << 3)    /* High threshold exceeded */
#define STATUS_THRESHOLD_LOW        (1 << 2)    /* Low threshold exceeded */
#define STATUS_SELF_TEST_OK         (1 << 1)    /* Self-test passed */
#define STATUS_DEVICE_READY         (1 << 0)    /* Device ready */

/* REG_INTERRUPT (0x05) */
#define INT_DATA_READY_EN           (1 << 7)    /* Data ready interrupt enable */
#define INT_FIFO_FULL_EN            (1 << 6)    /* FIFO full interrupt enable */
#define INT_THRESHOLD_HIGH_EN       (1 << 5)    /* High threshold interrupt enable */
#define INT_THRESHOLD_LOW_EN        (1 << 4)    /* Low threshold interrupt enable */
#define INT_ACTIVE_HIGH             (1 << 3)    /* Interrupt active high */
#define INT_PUSH_PULL               (1 << 2)    /* Push-pull output */
#define INT_LATCHED                 (1 << 1)    /* Latched interrupt */
#define INT_ENABLE                  (1 << 0)    /* Master interrupt enable */

/* REG_FIFO_CONFIG (0x06) */
#define FIFO_MODE_BYPASS            (0x00)      /* FIFO bypass mode */
#define FIFO_MODE_FIFO              (0x01)      /* FIFO mode */
#define FIFO_MODE_STREAM            (0x02)      /* Stream mode */
#define FIFO_MODE_TRIGGER           (0x03)      /* Trigger mode */
#define FIFO_MODE_MASK              (0x03)      /* FIFO mode mask */
#define FIFO_THRESHOLD_MASK         (0x1F << 2) /* FIFO threshold mask */

/* REG_DATA_RATE (0x08) */
#define DATA_RATE_1HZ               (0x00)      /* 1 Hz */
#define DATA_RATE_10HZ              (0x01)      /* 10 Hz */
#define DATA_RATE_25HZ              (0x02)      /* 25 Hz */
#define DATA_RATE_50HZ              (0x03)      /* 50 Hz */
#define DATA_RATE_100HZ             (0x04)      /* 100 Hz */
#define DATA_RATE_200HZ             (0x05)      /* 200 Hz */
#define DATA_RATE_400HZ             (0x06)      /* 400 Hz */
#define DATA_RATE_800HZ             (0x07)      /* 800 Hz */
#define DATA_RATE_MASK              (0x07)      /* Data rate mask */

/* REG_POWER_MGMT (0x0E) */
#define POWER_SLEEP                 (1 << 7)    /* Sleep mode */
#define POWER_STANDBY               (1 << 6)    /* Standby mode */
#define POWER_TEMP_DISABLE          (1 << 5)    /* Temperature sensor disable */
#define POWER_CLKSEL_MASK           (0x07)      /* Clock selection mask */
#define POWER_CLKSEL_INTERNAL       (0x00)      /* Internal oscillator */
#define POWER_CLKSEL_EXTERNAL       (0x01)      /* External clock */

/*=============================================================================
 * CONFIGURATION CONSTANTS
 *=============================================================================*/
#define SPI_MAX_FREQUENCY           10000000    /* Maximum SPI frequency (10 MHz) */
#define SPI_MODE                    0           /* SPI mode (CPOL=0, CPHA=0) */
#define SPI_BITS_PER_WORD           8           /* Bits per SPI word */

#define FIFO_SIZE                   32          /* FIFO depth */
#define DATA_BUFFER_SIZE            64          /* Data buffer size */
#define REGISTER_COUNT              23          /* Total number of registers */

/* Default configuration values */
#define DEFAULT_CONFIG              (CONFIG_ENABLE | CONFIG_RANGE_2G)
#define DEFAULT_MODE                (MODE_ACTIVE | MODE_RESOLUTION_12BIT)
#define DEFAULT_DATA_RATE           DATA_RATE_100HZ
#define DEFAULT_INTERRUPT           (INT_DATA_READY_EN | INT_ENABLE)

/*=============================================================================
 * UTILITY MACROS
 *=============================================================================*/
#define MAKE_READ_CMD(addr)         (CMD_READ_REG | (addr & 0x7F))
#define MAKE_WRITE_CMD(addr)        (CMD_WRITE_REG | (addr & 0x7F))
#define MAKE_BURST_READ_CMD(addr)   (CMD_BURST_READ | (addr & 0x3F))
#define MAKE_BURST_WRITE_CMD(addr)  (CMD_BURST_WRITE | (addr & 0x3F))

#define GET_REGISTER_ADDR(cmd)      (cmd & 0x7F)
#define IS_READ_CMD(cmd)            (cmd & 0x80)
#define IS_WRITE_CMD(cmd)           (!(cmd & 0x80))

#define SET_BITS(reg, mask, value)  ((reg & ~mask) | (value & mask))
#define GET_BITS(reg, mask)         (reg & mask)
#define CLEAR_BITS(reg, mask)       (reg & ~mask)

/*=============================================================================
 * DATA STRUCTURES
 *=============================================================================*/
typedef enum {
    DEVICE_MODE_POWER_DOWN = 0,
    DEVICE_MODE_STANDBY,
    DEVICE_MODE_ACTIVE,
    DEVICE_MODE_CONTINUOUS
} device_mode_t;

typedef enum {
    DEVICE_RANGE_2G = 0,
    DEVICE_RANGE_4G,
    DEVICE_RANGE_8G,
    DEVICE_RANGE_16G
} device_range_t;

typedef enum {
    DEVICE_RESOLUTION_8BIT = 0,
    DEVICE_RESOLUTION_10BIT,
    DEVICE_RESOLUTION_12BIT,
    DEVICE_RESOLUTION_14BIT
} device_resolution_t;

typedef enum {
    DATA_RATE_1HZ = 0,
    DATA_RATE_10HZ,
    DATA_RATE_25HZ,
    DATA_RATE_50HZ,
    DATA_RATE_100HZ,
    DATA_RATE_200HZ,
    DATA_RATE_400HZ,
    DATA_RATE_800HZ
} data_rate_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t temperature;
} sensor_data_t;

typedef struct {
    uint8_t device_id;
    uint8_t version;
    device_mode_t mode;
    device_range_t range;
    device_resolution_t resolution;
    data_rate_t data_rate;
    bool interrupt_enabled;
    bool fifo_enabled;
} device_config_t;

/*=============================================================================
 * REGISTER VALIDATION MACROS
 *=============================================================================*/
#define IS_VALID_REGISTER(addr)     ((addr) <= 0x16)
#define IS_READABLE_REGISTER(addr)  ((addr) <= 0x16)
#define IS_WRITABLE_REGISTER(addr)  ((addr) >= 0x02 && (addr) <= 0x0F)
#define IS_DATA_REGISTER(addr)      ((addr) >= 0x10 && (addr) <= 0x16)

/*=============================================================================
 * ERROR CODES
 *=============================================================================*/
#define ERR_SUCCESS                 0
#define ERR_INVALID_PARAMETER       -1
#define ERR_COMMUNICATION_FAILED    -2
#define ERR_DEVICE_NOT_FOUND        -3
#define ERR_TIMEOUT                 -4
#define ERR_BUFFER_OVERFLOW         -5
#define ERR_INVALID_REGISTER        -6
#define ERR_DEVICE_BUSY             -7
#define ERR_SELF_TEST_FAILED        -8

#endif /* REGISTERS_H */
