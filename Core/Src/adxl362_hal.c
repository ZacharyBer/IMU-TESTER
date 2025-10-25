/**
  ******************************************************************************
  * @file           : adxl362_hal.c
  * @brief          : ADXL362 ultra-low power accelerometer driver using STM32 HAL
  ******************************************************************************
  */

#include "adxl362_hal.h"
#include <string.h>

/* Private function prototypes */
static void ADXL362_CS_Low(adxl362_ctx_t *ctx);
static void ADXL362_CS_High(adxl362_ctx_t *ctx);
static uint8_t ADXL362_ReadReg(adxl362_ctx_t *ctx, uint8_t reg);
static void ADXL362_ReadRegs(adxl362_ctx_t *ctx, uint8_t reg, uint8_t *data, uint8_t len);
static uint8_t ADXL362_WriteReg(adxl362_ctx_t *ctx, uint8_t reg, uint8_t value);

/* Private functions */

/**
 * @brief  Assert chip select (active low)
 */
static void ADXL362_CS_Low(adxl362_ctx_t *ctx)
{
  HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_RESET);
  /* Delay for CS setup time (tCSS min 5ns, using ~2us for safety) */
  for (volatile int i = 0; i < 32; i++);  // ~2us delay
}

/**
 * @brief  Deassert chip select
 */
static void ADXL362_CS_High(adxl362_ctx_t *ctx)
{
  /* Delay for CS hold time (tCSH min 10ns, using ~2us for safety) */
  for (volatile int i = 0; i < 32; i++);  // ~2us delay
  HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);
  /* Additional delay between transactions */
  for (volatile int i = 0; i < 32; i++);  // ~2us delay after CS goes high
}

/**
 * @brief  Read single register from ADXL362
 */
static uint8_t ADXL362_ReadReg(adxl362_ctx_t *ctx, uint8_t reg)
{
  /* ADXL362 requires continuous SPI transaction: send command+addr, then receive data */
  uint8_t tx_buf[3] = {ADXL362_READ_REG, reg, 0x00};  // 0x00 is dummy byte to clock out data
  uint8_t rx_buf[3] = {0};

  ADXL362_CS_Low(ctx);
  HAL_SPI_TransmitReceive(ctx->hspi, tx_buf, rx_buf, 3, 100);
  ADXL362_CS_High(ctx);

  return rx_buf[2];  // Data is received on the 3rd byte
}

/**
 * @brief  Read multiple registers from ADXL362
 */
static void ADXL362_ReadRegs(adxl362_ctx_t *ctx, uint8_t reg, uint8_t *data, uint8_t len)
{
  /* ADXL362 requires continuous SPI transaction */
  /* Allocate buffer for command (2 bytes) + data (len bytes) */
  uint8_t tx_buf[32] = {0};  // Max buffer size (sufficient for typical reads)
  uint8_t rx_buf[32] = {0};

  if (len > 30) return;  // Safety check: ensure we don't overflow buffer

  tx_buf[0] = ADXL362_READ_REG;
  tx_buf[1] = reg;
  /* Rest of tx_buf is already zeroed (dummy bytes to clock out data) */

  ADXL362_CS_Low(ctx);
  HAL_SPI_TransmitReceive(ctx->hspi, tx_buf, rx_buf, 2 + len, 100);
  ADXL362_CS_High(ctx);

  /* Copy received data (skip first 2 bytes which are command echo) */
  for (uint8_t i = 0; i < len; i++)
  {
    data[i] = rx_buf[2 + i];
  }
}

/**
 * @brief  Write single register to ADXL362
 */
static uint8_t ADXL362_WriteReg(adxl362_ctx_t *ctx, uint8_t reg, uint8_t value)
{
  uint8_t cmd[3] = {ADXL362_WRITE_REG, reg, value};

  ADXL362_CS_Low(ctx);
  HAL_StatusTypeDef status = HAL_SPI_Transmit(ctx->hspi, cmd, 3, 100);
  ADXL362_CS_High(ctx);

  return (status == HAL_OK) ? 1 : 0;
}

/* Public functions */

/**
 * @brief  Soft reset ADXL362
 */
uint8_t ADXL362_SoftReset(adxl362_ctx_t *ctx)
{
  if (!ADXL362_WriteReg(ctx, ADXL362_REG_SOFT_RESET, ADXL362_RESET_KEY))
  {
    return 0;
  }

  /* Wait for reset to complete (datasheet: 0.5ms required) */
  HAL_Delay(10);

  /* Verify reset by checking register is readable (not stuck at 0x00 or 0xFF)
   * Note: Reset default value varies by chip revision (0x13 on some, different on others)
   * As long as we can read a valid value, reset was successful */
  uint8_t filter_ctl = ADXL362_ReadReg(ctx, ADXL362_REG_FILTER_CTL);
  if (filter_ctl == 0x00 || filter_ctl == 0xFF)
  {
    return 0;  // Communication error
  }

  return 1;
}

/**
 * @brief  Read device ID registers for debugging
 */
void ADXL362_ReadDeviceIDs(adxl362_ctx_t *ctx, uint8_t *devid_ad, uint8_t *devid_mst, uint8_t *partid)
{
  *devid_ad = ADXL362_ReadReg(ctx, ADXL362_REG_DEVID_AD);
  *devid_mst = ADXL362_ReadReg(ctx, ADXL362_REG_DEVID_MST);
  *partid = ADXL362_ReadReg(ctx, ADXL362_REG_PARTID);
}

/**
 * @brief  Check if ADXL362 device is present
 */
uint8_t ADXL362_CheckDevice(adxl362_ctx_t *ctx)
{
  uint8_t devid_ad = ADXL362_ReadReg(ctx, ADXL362_REG_DEVID_AD);
  uint8_t devid_mst = ADXL362_ReadReg(ctx, ADXL362_REG_DEVID_MST);
  uint8_t partid = ADXL362_ReadReg(ctx, ADXL362_REG_PARTID);

  /* Check analog device ID and manufacturer ID (these are fixed) */
  if (devid_ad != ADXL362_DEVICE_AD)
    return 0;
  if (devid_mst != ADXL362_DEVICE_MST)
    return 0;

  /* PARTID can vary between chip revisions (0xF2, 0xF7, etc.)
   * Just verify it's not 0x00 or 0xFF (communication error) */
  if (partid == 0x00 || partid == 0xFF)
    return 0;

  return 1;
}

/**
 * @brief  Initialize ADXL362 with power mode based on IMU_PERFORMANCE_MODE
 * @note   Mode 0: Ultra-low power wakeup mode (6 Hz, ultralow noise)
 *         Mode 1: Moderate performance measurement mode (50 Hz, normal noise)
 *         Mode 2: High performance measurement mode (100 Hz, normal noise)
 */
uint8_t ADXL362_Init(adxl362_ctx_t *ctx)
{
  /* Perform soft reset */
  if (!ADXL362_SoftReset(ctx))
  {
    return 0;
  }

  /* Check device ID */
  if (!ADXL362_CheckDevice(ctx))
  {
    return 0;
  }

#if (IMU_PERFORMANCE_MODE == 0)
  /* Mode 0: Absolute Lowest Power - Wakeup mode (6 Hz) with ultralow noise */
  uint8_t filter_ctl = ADXL362_FILTER_CTL_RANGE(ADXL362_RANGE_2G) |
                       ADXL362_FILTER_CTL_ODR(ADXL362_ODR_12_5_HZ);
  if (!ADXL362_WriteReg(ctx, ADXL362_REG_FILTER_CTL, filter_ctl))
  {
    return 0;
  }

  /* Wakeup mode: ultra-low power ~0.27µA, 6 Hz sampling, ultralow noise */
  uint8_t power_ctl = ADXL362_POWER_CTL_WAKEUP |
                      ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_ULTRALOW) |
                      ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON);
  if (!ADXL362_WriteReg(ctx, ADXL362_REG_POWER_CTL, power_ctl))
  {
    return 0;
  }
#elif (IMU_PERFORMANCE_MODE == 1)
  /* Mode 1: Moderate Performance - 50 Hz ODR, normal noise */
  uint8_t filter_ctl = ADXL362_FILTER_CTL_RANGE(ADXL362_RANGE_2G) |
                       ADXL362_FILTER_CTL_ODR(ADXL362_ODR_50_HZ);
  if (!ADXL362_WriteReg(ctx, ADXL362_REG_FILTER_CTL, filter_ctl))
  {
    return 0;
  }

  /* Measurement mode with normal noise */
  uint8_t power_ctl = ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_NORMAL) |
                      ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON);
  if (!ADXL362_WriteReg(ctx, ADXL362_REG_POWER_CTL, power_ctl))
  {
    return 0;
  }
#elif (IMU_PERFORMANCE_MODE == 2)
  /* Mode 2: High Performance - 100 Hz ODR, normal noise */
  uint8_t filter_ctl = ADXL362_FILTER_CTL_RANGE(ADXL362_RANGE_2G) |
                       ADXL362_FILTER_CTL_ODR(ADXL362_ODR_100_HZ);
  if (!ADXL362_WriteReg(ctx, ADXL362_REG_FILTER_CTL, filter_ctl))
  {
    return 0;
  }

  /* Measurement mode with normal noise for best performance */
  uint8_t power_ctl = ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_NORMAL) |
                      ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON);
  if (!ADXL362_WriteReg(ctx, ADXL362_REG_POWER_CTL, power_ctl))
  {
    return 0;
  }
#endif

  /* Small delay for sensor to stabilize */
  HAL_Delay(10);

  return 1;
}

/**
 * @brief  Read acceleration data from ADXL362 (full 12-bit resolution)
 * @param  accel_raw: Array to store X, Y, Z values [3]
 * @note   ADXL362 12-bit data is positioned in upper bits [15:4] of 16-bit registers
 */
void ADXL362_ReadData(adxl362_ctx_t *ctx, int16_t *accel_raw)
{
  uint8_t data[6];

  /* Read 6 bytes starting from XDATA_L: X_L, X_H, Y_L, Y_H, Z_L, Z_H */
  ADXL362_ReadRegs(ctx, ADXL362_REG_XDATA_L, data, 6);

  /* Combine bytes - note: byte order may be implementation-specific
   * for this particular SPI configuration */
  int16_t x_raw = (int16_t)((data[0] << 8) | data[1]);
  int16_t y_raw = (int16_t)((data[2] << 8) | data[3]);
  int16_t z_raw = (int16_t)((data[4] << 8) | data[5]);

  /* ADXL362 places 12-bit data in bits [15:4] with LSBs in [3:0]
   * Shift right by 4 to extract the meaningful 12-bit value
   * This aligns the scale with 1 mg/LSB sensitivity */
  accel_raw[0] = x_raw >> 4;
  accel_raw[1] = y_raw >> 4;
  accel_raw[2] = z_raw >> 4;
}

/**
 * @brief  Read acceleration data from ADXL362 using 8-bit registers
 * @note   Uses XDATA (0x08), YDATA (0x09), ZDATA (0x0A) - MSB only, lower resolution
 */
void ADXL362_ReadData_8bit(adxl362_ctx_t *ctx, int16_t *accel_raw)
{
  uint8_t data[3];

  /* Read 3 bytes from 8-bit data registers (MSB only of 12-bit values) */
  ADXL362_ReadRegs(ctx, ADXL362_REG_XDATA, data, 3);

  /* These are 8-bit signed values representing the upper 8 bits of 12-bit data
   * Need to shift left by 4 to restore to 12-bit range, then sign-extend */
  int16_t x_raw = (int8_t)data[0];  // Interpret as signed byte
  int16_t y_raw = (int8_t)data[1];
  int16_t z_raw = (int8_t)data[2];

  /* Shift left by 4 to align to 12-bit scale (multiply by 16) */
  x_raw <<= 4;
  y_raw <<= 4;
  z_raw <<= 4;

  accel_raw[0] = x_raw;
  accel_raw[1] = y_raw;
  accel_raw[2] = z_raw;
}

/**
 * @brief  Convert raw value to millig (mg)
 */
float ADXL362_RawToMg(int16_t raw_value, uint8_t range)
{
  float sensitivity;

  switch (range)
  {
    case ADXL362_RANGE_2G:
      sensitivity = ADXL362_2G_SENSITIVITY;
      break;
    case ADXL362_RANGE_4G:
      sensitivity = ADXL362_4G_SENSITIVITY;
      break;
    case ADXL362_RANGE_8G:
      sensitivity = ADXL362_8G_SENSITIVITY;
      break;
    default:
      sensitivity = ADXL362_2G_SENSITIVITY;
      break;
  }

  return (float)raw_value * sensitivity;
}
