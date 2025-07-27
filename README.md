https://building-drivers.vercel.app/

## Table of Contents
- [Introduction](#introduction)
- [The Lego Castle Analogy](#the-lego-castle-analogy)
- [Approaching Zephyr RTOS](#approaching-zephyr-rtos)
- [Know Arduino](#know-arduino)
- [Datasheet](#datasheet)
- [Understanding SPI Communication](#understanding-spi-communication)
- [Understanding Interrupts](#understanding-interrupts)
- [Building Your HAL](#building-your-hal)
- [Let's Get Started](#lets-get-started)

## Introduction

The purpose of this document is to lay out how one may approach building a device driver from the ground up. With the onset of AI tools and agentic workflows, many of the brilliant architecture of systems under the hood is abstracted away. But human curiosity will always persist. When given a black box, where an input results in some output, it is human nature to be curious of the inner workings of the box. This is the same when working with embedded systems.

> **Key Insight**: Understanding the inner workings of embedded systems satisfies our natural curiosity and makes us better developers.

## The Lego Castle Analogy

The key part of firmware development is the idea of **abstraction**. When you create simplicity from complexity. Say you wanted to build a Lego castle, the reality is it'll be much more complex to personally create injection molds, find the right plastic formula, and etc. But if you break it down into repeatable steps and into "Lego bricks" that enables anybody to build a Lego Castle of their own, a daunting task, becomes much more simplistic. By abstracting away the complexities, you can end up with simple "lego brick" programs that build up to create larger and more complex programs.

This is essentially the key idea of building a device driver from the ground up. The **hardware abstraction layer (HAL)** acts like the Lego brick interface. It hides the intricate details of registers, timing, and protocols of the underlying hardware and exposes a consistent, simplified API for higher-level code to interact with. This way, application developers don't need to worry about how SPI works at the bit level or what exact sequence of commands initializes a sensor. They just call a function like `spi_read_data()` or `sensor_init()` and the HAL handles the complexity beneath.


## Approaching Zephyr RTOS

Learning and working with Zephyr is quite a learning curve. For starters, Nordic has beginner and intermediate free courses on using Zephyr RTOS with their NRF Connect SDK: https://academy.nordicsemi.com/ Though it is through NRF Connect, they do establish a good foundation for how to use Zephyr's device tree and KConfig system. Honestly, don't spend too much time trying to learn it. Much of what you learn will be through doing. You will naturally learn and inevitably stumble upon many niche and obscure features of Zephyr.


## Know Arduino

Arduino is a good tool to help you build out the barebone functionality of your driver. It's easy to set up and use. **Trust me. Use Arduino.** This is a lesson a wise man taught me.

### Arduino to Zephyr Workflow:
1. **Prototype** in Arduino to understand the hardware
2. **Test** basic SPI communication and device responses
3. **Document** working code sequences and timing requirements
4. **Wrap** basic SPI transfers into reusable functions
5. **Translate** proven Arduino code patterns to Zephyr driver format 

## Datasheet

Find the datasheet for the device you are trying to build a driver for. This will serve as your **source of truth** and most answers to your problems can be derived from the datasheet. You can read through it, skim it, or whatever, but most importantly, upload it to Gemini or LLM of choice (depending on the context window). Ta da! Your datasheet is now a conversational tool to help you through this journey.


## Understanding SPI Communication

Before diving into driver development, it's crucial to understand how **SPI (Serial Peripheral Interface)** communication works. SPI is one of the most common protocols for communicating with sensors, displays, and other peripherals in embedded systems.

### SPI Basics

SPI is a **synchronous**, **full-duplex** communication protocol that uses a **master-slave** architecture:

- **Master**: Controls the communication (usually your microcontroller)
- **Slave**: Responds to master's requests (your sensor/device)
- **Synchronous**: Data transfer is synchronized by a clock signal
- **Full-duplex**: Data can flow in both directions simultaneously

### SPI Signal Lines

SPI uses four main signal lines:

1. **SCLK (Serial Clock)**: Master generates clock pulses to synchronize data transfer
2. **MOSI (Master Out, Slave In)**: Data line from master to slave
3. **MISO (Master In, Slave Out)**: Data line from slave to master  
4. **CS/SS (Chip Select/Slave Select)**: Master selects which slave to communicate with

### How SPI Transfers Work

SPI is full-duplex, meaning data flows both ways simultaneously during every transfer. In simpler terms, it's the communication protocol that allows your MCU to communicate with your devices. It uses raw SPI transfers which are the raw bits and bytes of data that is being sent. Full duplex essentially means data that you send (TX) is simutaneously sent back throw (RX).

#### TX and RX Explained

- **TX (Transmit)**: Bytes the master sends to the slave via MOSI
- **RX (Receive)**: Bytes the slave sends to the master via MISO

**Critical Point**: Every SPI transfer involves both TX and RX happening at the same time! When you send one byte, you always receive one byte back.

#### Example Transfer Sequence

```
Master wants to read device ID from register 0x00:

Transfer 1: Send command
TX: 0x80  (Read command for register 0x00)
RX: 0xFF  (Slave sends dummy data while processing command)

Transfer 2: Read the actual data  
TX: 0x00  (Master sends dummy byte to generate clock)
RX: 0x42  (Slave responds with device ID)
```

### Common SPI Patterns

**Register Read**: Send register address, then clock out the data
```
TX: [READ_CMD | ADDRESS] [DUMMY_BYTE]
RX: [DUMMY_RESPONSE]     [ACTUAL_DATA]
```

**Register Write**: Send register address, then send data
```
TX: [WRITE_CMD | ADDRESS] [DATA_TO_WRITE]
RX: [DUMMY_RESPONSE]      [DUMMY_RESPONSE]
```

## Understanding Interrupts

Interrupts are one of the most powerful features in embedded systems, but they can seem intimidating at first. Think of interrupts like a doorbell - instead of constantly checking if someone is at the door, you wait for the bell to ring and then respond.

### Why Interrupts Matter

Without interrupts, your microcontroller would spend most of its time **polling** - constantly asking "Is the sensor ready? Is the sensor ready?" This wastes CPU cycles and drains battery power. With interrupts, the sensor says "Hey, I'm ready!" and your microcontroller can do other useful work in the meantime.

### Interrupt Basics

**What triggers an interrupt?**
- **Data ready**: Sensor has new measurement available
- **FIFO threshold**: Buffer is getting full
- **Threshold exceeded**: Sensor value crossed a limit
- **Error conditions**: Communication failure, sensor fault

**What happens during an interrupt?**
1. **Hardware interrupt occurs** (GPIO pin changes state)
2. **CPU stops current task** and saves its state
3. **Interrupt handler function runs** (your code!)
4. **CPU returns** to what it was doing before

### Interrupt Handler Pattern

Here's the typical pattern for sensor interrupt handlers:

```c
void sensor_interrupt_handler(void)
{
    // 1. Read status register to see what happened
    uint8_t status = read_sensor_status();
    
    // 2. Handle different interrupt sources
    if (status & DATA_READY_FLAG) {
        // Data is ready - read it
        sensor_data_t data;
        read_sensor_data(&data);
        
        // Process or queue the data
        add_to_data_queue(&data);
    }
    
    if (status & THRESHOLD_FLAG) {
        // Threshold exceeded - take action
        handle_threshold_event();
    }
    
    // 3. Clear interrupt flags
    clear_interrupt_flags(status);
}
```

### Interrupt Best Practices

**Keep It Short and Sweet**
- Interrupt handlers should be **fast** - do minimal work
- Read the data, set a flag, exit quickly
- Do heavy processing in your main loop

**Don't Block in Interrupts**
- No `delay()` or long calculations
- No `printf()` or serial communication
- No complex algorithms

**Use Volatile Variables**
```c
volatile bool data_ready_flag = false;
volatile sensor_data_t latest_data;

void sensor_interrupt_handler(void)
{
    read_sensor_data(&latest_data);
    data_ready_flag = true;  // Tell main loop data is ready
}

int main(void)
{
    while (1) {
        if (data_ready_flag) {
            data_ready_flag = false;
            process_sensor_data(&latest_data);  // Do heavy work here
        }
        
        // Other main loop tasks
        do_other_work();
    }
}
```

### Arduino Interrupt Example

```c
// Arduino interrupt setup
const int SENSOR_INT_PIN = 2;  // Interrupt pin
volatile bool sensor_data_ready = false;

void setup() {
    attachInterrupt(digitalPinToInterrupt(SENSOR_INT_PIN), 
                    sensor_isr, FALLING);
}

void sensor_isr() {
    // This runs when sensor pulls interrupt pin low
    sensor_data_ready = true;
}

void loop() {
    if (sensor_data_ready) {
        sensor_data_ready = false;
        
        // Read and process sensor data
        sensor_data_t data;
        if (read_sensor_data(&data) == SUCCESS) {
            Serial.print("X: "); Serial.println(data.x);
            Serial.print("Y: "); Serial.println(data.y);
            Serial.print("Z: "); Serial.println(data.z);
        }
    }
    
    // Other loop tasks
    delay(10);
}
```

## Building Your HAL

Now comes the fun part - building your Hardware Abstraction Layer (HAL). This is where you transform those messy register operations into beautiful, simple functions that anyone can use.

### The HAL Philosophy

A good HAL should be like a **good restaurant menu**:
- **Simple choices** - Clear function names that explain what they do
- **Hide complexity** - You don't need to know how the kitchen works
- **Consistent interface** - Similar functions work in similar ways
- **Error handling** - Graceful failure when something goes wrong

### HAL Architecture Layers

Think of your HAL as a layered cake:

```
┌─────────────────────────────────────┐
│     Application Code               │  ← Your main program
├─────────────────────────────────────┤
│     High-Level HAL Functions       │  ← sensor_init(), sensor_read_data()
├─────────────────────────────────────┤
│     Mid-Level HAL Functions        │  ← sensor_read_register(), sensor_write_register()
├─────────────────────────────────────┤
│     Low-Level SPI Functions        │  ← spi_transfer(), spi_read_byte()
├─────────────────────────────────────┤
│     Platform Layer                 │  ← Arduino SPI.h, Zephyr SPI API
└─────────────────────────────────────┘
```

### Step 1: Start with Platform Layer

First, create functions that hide platform differences:

```c
// Platform abstraction - same interface, different implementations
int platform_spi_transfer(uint8_t *tx_data, uint8_t *rx_data, int length);
void platform_delay_ms(int ms);
uint32_t platform_get_time_ms(void);
void platform_gpio_set(int pin, bool state);

// Arduino implementation
#ifdef ARDUINO
int platform_spi_transfer(uint8_t *tx_data, uint8_t *rx_data, int length) {
    digitalWrite(CS_PIN, LOW);
    for (int i = 0; i < length; i++) {
        rx_data[i] = SPI.transfer(tx_data[i]);
    }
    digitalWrite(CS_PIN, HIGH);
    return 0;  // Success
}
#endif

// Zephyr implementation  
#ifdef ZEPHYR
int platform_spi_transfer(uint8_t *tx_data, uint8_t *rx_data, int length) {
    struct spi_buf tx_buf = {.buf = tx_data, .len = length};
    struct spi_buf rx_buf = {.buf = rx_data, .len = length};
    struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};
    
    return spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
}
#endif
```

### Step 2: Build Low-Level HAL Functions

Create functions that handle basic register operations:

```c
typedef enum {
    SENSOR_OK = 0,
    SENSOR_ERROR = -1,
    SENSOR_TIMEOUT = -2
} sensor_result_t;

sensor_result_t sensor_read_register(uint8_t reg_addr, uint8_t *value) {
    uint8_t tx_data[2] = {READ_CMD | reg_addr, 0x00};
    uint8_t rx_data[2];
    
    int result = platform_spi_transfer(tx_data, rx_data, 2);
    if (result != 0) {
        return SENSOR_ERROR;
    }
    
    *value = rx_data[1];  // Second byte contains the data
    return SENSOR_OK;
}

sensor_result_t sensor_write_register(uint8_t reg_addr, uint8_t value) {
    uint8_t tx_data[2] = {WRITE_CMD | reg_addr, value};
    uint8_t rx_data[2];
    
    int result = platform_spi_transfer(tx_data, rx_data, 2);
    return (result == 0) ? SENSOR_OK : SENSOR_ERROR;
}
```

### Step 3: Create Data Structures

Design structures that make sense to users:

```c
typedef struct {
    int16_t x, y, z;        // Raw sensor values
    float x_g, y_g, z_g;    // Values in engineering units (g)
    uint32_t timestamp;     // When this data was captured
    bool valid;             // Is this data good?
} sensor_data_t;

typedef struct {
    bool data_ready;        // New data available
    bool fifo_full;         // FIFO buffer is full
    bool threshold_exceeded; // Measurement exceeded threshold
    bool device_error;      // Hardware error detected
} sensor_status_t;
```

### Step 4: Build High-Level Functions

Create the "Lego brick" functions that users actually want:

```c
sensor_result_t sensor_init(void) {
    // Complete initialization sequence
    
    // 1. Reset device
    if (sensor_write_register(CMD_RESET, 0xFF) != SENSOR_OK) {
        return SENSOR_ERROR;
    }
    platform_delay_ms(10);  // Wait for reset
    
    // 2. Check device ID
    uint8_t device_id;
    if (sensor_read_register(REG_DEVICE_ID, &device_id) != SENSOR_OK) {
        return SENSOR_ERROR;
    }
    if (device_id != EXPECTED_DEVICE_ID) {
        return SENSOR_ERROR;  // Wrong device!
    }
    
    // 3. Configure device
    if (sensor_write_register(REG_CONFIG, CONFIG_ENABLE | CONFIG_2G_RANGE) != SENSOR_OK) {
        return SENSOR_ERROR;
    }
    
    // 4. Enable data ready interrupt
    if (sensor_write_register(REG_INTERRUPT, INT_DATA_READY_EN) != SENSOR_OK) {
        return SENSOR_ERROR;
    }
    
    return SENSOR_OK;
}

sensor_result_t sensor_read_data(sensor_data_t *data) {
    // Check if data is ready
    uint8_t status;
    if (sensor_read_register(REG_STATUS, &status) != SENSOR_OK) {
        return SENSOR_ERROR;
    }
    if (!(status & STATUS_DATA_READY)) {
        return SENSOR_TIMEOUT;  // No new data
    }
    
    // Read raw data (X, Y, Z each 2 bytes)
    uint8_t raw_data[6];
    for (int i = 0; i < 6; i++) {
        if (sensor_read_register(REG_DATA_X_LOW + i, &raw_data[i]) != SENSOR_OK) {
            return SENSOR_ERROR;
        }
    }
    
    // Convert to meaningful data structure
    data->x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    data->y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    data->z = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    
    // Convert to engineering units (assuming ±2g range, 12-bit ADC)
    float scale = 4.0f / 4096.0f;  // 4g total range, 12-bit resolution
    data->x_g = data->x * scale;
    data->y_g = data->y * scale;
    data->z_g = data->z * scale;
    
    data->timestamp = platform_get_time_ms();
    data->valid = true;
    
    return SENSOR_OK;
}
```

### Step 5: Add Convenience Functions

Make common tasks even easier:

```c
bool sensor_is_data_ready(void) {
    uint8_t status;
    if (sensor_read_register(REG_STATUS, &status) != SENSOR_OK) {
        return false;
    }
    return (status & STATUS_DATA_READY) != 0;
}

sensor_result_t sensor_wait_for_data(uint32_t timeout_ms) {
    uint32_t start_time = platform_get_time_ms();
    
    while ((platform_get_time_ms() - start_time) < timeout_ms) {
        if (sensor_is_data_ready()) {
            return SENSOR_OK;
        }
        platform_delay_ms(1);
    }
    
    return SENSOR_TIMEOUT;
}

float sensor_get_magnitude(const sensor_data_t *data) {
    return sqrt(data->x_g * data->x_g + 
                data->y_g * data->y_g + 
                data->z_g * data->z_g);
}
```

### HAL Design Tips

**Error Handling**
- Always return meaningful error codes
- Check every SPI transaction
- Provide error strings for debugging

**Documentation**
- Document what each function does
- Explain parameters and return values
- Give usage examples

**Testing**
- Test with and without hardware connected
- Test error conditions
- Test edge cases (FIFO full, timeouts, etc.)

**Modularity**
- Keep platform code separate
- Make it easy to port to new platforms
- Use consistent naming conventions

### From Arduino to Zephyr

Once your HAL works in Arduino, porting to Zephyr becomes much easier:

1. **Arduino Phase**: Get it working with simple platform functions
2. **Abstraction Phase**: Move platform code to separate functions
3. **Zephyr Phase**: Implement the same platform functions using Zephyr APIs

The beauty is that your high-level HAL functions (`sensor_init()`, `sensor_read_data()`) **don't change at all** - only the platform layer changes. That's the power of abstraction!


## Let's Get Started

With all this in mind, we have reached the point of diminishing returns in terms of what you can get out of trying to understand firmware development. What remains is to begin and take action. It is through trial by fire that real learning happens. The same applies to me and to anyone else; true growth comes through doing. Best of Luck.










