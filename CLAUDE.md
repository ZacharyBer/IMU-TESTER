# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

IMU_TESTER is an STM32U5A5 embedded firmware project that interfaces with multiple IMU sensors and streams their data over UART for visualization. The system supports up to 4 different IMU sensors simultaneously:

- **LSM6DSV**: 6-axis IMU (accel + gyro) over I2C with optional sensor fusion (SFLP)
- **LSM6DSOX**: 6-axis IMU (accel + gyro) over I2C
- **LIS2DW12**: 3-axis accelerometer over I2C
- **ADXL362**: 3-axis accelerometer over SPI

The firmware outputs sensor data via UART at 115200 baud in either CSV or human-readable format. A companion Python application (`imu_visualizer.py`) provides real-time data visualization and analysis.

## Build and Flash Commands

This project is designed for STM32CubeIDE. Build and flash commands:

```bash
# Build the project (from STM32CubeIDE GUI or command line)
# In STM32CubeIDE: Project -> Build Project (or Ctrl+B)

# The build output is placed in Debug/IMU_TESTER.elf

# Flash to target hardware
# In STM32CubeIDE: Run -> Debug (F11) or Run -> Run (Ctrl+F11)
```

**Note**: This is an STM32CubeIDE managed project. The build system is controlled by Eclipse/STM32CubeIDE, not by standalone Makefiles. Direct command-line builds require importing the project into STM32CubeIDE or using STM32CubeMX code generation.

## Python Visualization Tools

The repository includes Python tools for data visualization:

```bash
# Install Python dependencies
pip install -r requirements.txt

# Launch the visualizer GUI
python imu_visualizer.py
```

The visualizer supports:
- Real-time serial streaming from the STM32
- CSV file loading and playback
- Multiple plot types (XYZ, magnitude, 3D)
- Data integration (velocity/position from acceleration)
- Sensor fusion visualization (quaternions and Euler angles)

## Architecture

### Firmware Structure (Core/Src/main.c)

The main firmware follows this architecture:

1. **Compile-time Configuration** (lines 41-58):
   - `IMU_PERFORMANCE_MODE`: 0 = low power, 1 = peak performance
   - `OUTPUT_FORMAT`: 0 = CSV, 1 = human-readable
   - `ENABLE_*` macros: Enable/disable individual IMUs at compile time
   - `ENABLE_LSM6DSV_SENSOR_FUSION`: Enable SFLP quaternion output

2. **Sensor Abstraction Layer**:
   - Platform-agnostic I2C read/write functions (`platform_read()`, `platform_write()`)
   - Per-sensor contexts (`stmdev_ctx_t` for I2C sensors, `adxl362_ctx_t` for SPI)
   - Manufacturer-provided register drivers: `lsm6dsv_reg.c`, `lsm6dsox_reg.c`, `lis2dw12_reg.c`, `adxl362_hal.c`

3. **Runtime Initialization** (in `main()`):
   - Each IMU has an `*_Init()` function that returns 1 if detected, 0 if absent
   - `imu_status` struct tracks which sensors are present at runtime
   - I2C bus scan (`I2C_Scan()`) helps debug connectivity issues

4. **Main Loop**:
   - Reads all present IMUs in sequence
   - Formats output based on `OUTPUT_FORMAT` macro
   - Transmits data over UART via `tx_com()`
   - Uses microsecond timestamp from TIM2 hardware timer

### Key Integration Points

- **STM32CubeMX Configuration** (`IMU_TESTER.ioc`): Defines peripherals, clocks, and pinout. Regenerating code from this file will preserve `/* USER CODE */` sections in generated files.

- **Peripheral Configuration**:
  - I2C2: All I2C-based IMUs share this bus
  - SPI1: ADXL362 uses dedicated SPI with CS pin control
  - TIM2: Configured as microsecond counter for precise timestamping
  - USART1: UART output at 115200 baud (via BSP_COM API)

- **HAL Drivers**: Located in `Drivers/STM32U5xx_HAL_Driver/`. These are ST-provided drivers - avoid modifying unless absolutely necessary.

### Data Flow

```
Sensors (I2C/SPI)
    ↓
Platform read/write functions
    ↓
Manufacturer register drivers (lsm6dsv_reg.c, etc.)
    ↓
Sensor *_ReadData() functions
    ↓
Print_IMU_Data() formatting
    ↓
tx_com() UART transmission
    ↓
Python visualizer (serial or CSV file)
```

## Common Modifications

### Adding a New IMU Sensor

1. Add sensor register driver files to `Core/Src/` and `Core/Inc/`
2. Add `#define ENABLE_NEWSENSOR 1` to configuration section
3. Add I2C address define if using I2C
4. Create sensor context variable in Private Variables section
5. Implement `NEWSENSOR_Init()` function following existing pattern
6. Implement `NEWSENSOR_ReadData()` function
7. Add output formatting in `Print_IMU_Data()`
8. Call init and read functions in main loop

### Changing Sensor Configuration

Sensor parameters (ODR, full scale, filtering) are configured in each `*_Init()` function. The configuration differs based on `IMU_PERFORMANCE_MODE`:
- Mode 0 (Low Power): Lower ODR, reduced power consumption
- Mode 1 (Peak Performance): Maximum ODR for highest data rate

### Switching Output Format

Toggle `OUTPUT_FORMAT` macro:
- 0 = CSV format for Python parsing
- 1 = Human-readable format for debugging in serial terminal

### Debugging Communication Issues

1. Enable I2C bus scan output - already implemented in `I2C_Scan()` function
2. Check UART output in serial terminal at 115200 baud, 8N1
3. For SPI (ADXL362), device ID read debug output is already instrumented
4. Verify pull-up resistors on I2C bus (required for I2C communication)

## Important Notes

- **USER CODE sections**: STM32CubeMX preserves code between `/* USER CODE BEGIN */` and `/* USER CODE END */` markers. All custom code should go in these sections.

- **HAL_Delay limitations**: `HAL_Delay()` is used during initialization but should be avoided in timing-critical paths. Use the TIM2 microsecond counter (`Get_Microseconds()`) for precise timing.

- **Buffer sizes**: UART transmit buffer is stack-allocated (256 bytes in `Print_IMU_Data()`). Larger outputs may require heap allocation or multiple transmissions.

- **Sensor fusion (LSM6DSV)**: The SFLP feature provides quaternion output from onboard sensor fusion. This requires specific firmware configuration and MLC (Machine Learning Core) setup - see LSM6DSV datasheet and application notes.

## Hardware Configuration

- **Target MCU**: STM32U5A5ZJTXQ (Cortex-M33, 144-pin LQFP)
- **Development Board**: STM32U5xx Nucleo (based on BSP includes)
- **I2C Pull-ups**: External pull-ups required on I2C2 (SCL/SDA)
- **SPI CS**: ADXL362 chip select managed manually via GPIO

## Linker Scripts

- `STM32U5A5ZJTXQ_FLASH.ld`: Standard flash execution
- `STM32U5A5ZJTXQ_RAM.ld`: RAM-only execution for debugging
