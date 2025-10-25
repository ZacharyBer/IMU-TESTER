/**
  ******************************************************************************
  * @file           : adxl362_hal.h
  * @brief          : Header for adxl362_hal.c file.
  *                   ADXL362 ultra-low power accelerometer driver using STM32 HAL
  ******************************************************************************
  */

#ifndef __ADXL362_HAL_H__
#define __ADXL362_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ADXL362Reg.h"

/* ADXL362 Configuration - Uses SPI1_NSS from .ioc */
#define ADXL362_CS_PIN          SPI1_NSS_Pin      // PE12
#define ADXL362_CS_PORT         SPI1_NSS_GPIO_Port // GPIOE

/* ADXL362 Sensitivity (mg per LSB) for different ranges */
#define ADXL362_2G_SENSITIVITY  1.0f    // 1 mg/LSB
#define ADXL362_4G_SENSITIVITY  2.0f    // 2 mg/LSB
#define ADXL362_8G_SENSITIVITY  4.0f    // 4 mg/LSB

/* ADXL362 Context Structure */
typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} adxl362_ctx_t;

/* Function Prototypes */

/**
 * @brief  Initialize ADXL362 with power mode based on IMU_PERFORMANCE_MODE
 * @param  ctx: ADXL362 context containing SPI handle and CS pin info
 * @retval 1 if initialization successful, 0 if device not found or error
 * @note   Mode 0: Ultra-low power wakeup mode (6 Hz, ultralow noise)
 *         Mode 1: Measurement mode (100 Hz, normal noise)
 */
uint8_t ADXL362_Init(adxl362_ctx_t *ctx);

/**
 * @brief  Read acceleration data from ADXL362 (12-bit mode from 16-bit registers)
 * @param  ctx: ADXL362 context
 * @param  accel_raw: Pointer to array for X, Y, Z raw values (3 elements)
 * @retval None
 */
void ADXL362_ReadData(adxl362_ctx_t *ctx, int16_t *accel_raw);

/**
 * @brief  Read acceleration data from ADXL362 (8-bit mode from 8-bit registers)
 * @param  ctx: ADXL362 context
 * @param  accel_raw: Pointer to array for X, Y, Z raw values (3 elements)
 * @retval None
 * @note   Uses 8-bit registers (XDATA/YDATA/ZDATA) instead of 16-bit for debugging
 */
void ADXL362_ReadData_8bit(adxl362_ctx_t *ctx, int16_t *accel_raw);

/**
 * @brief  Soft reset ADXL362
 * @param  ctx: ADXL362 context
 * @retval 1 if successful, 0 on error
 */
uint8_t ADXL362_SoftReset(adxl362_ctx_t *ctx);

/**
 * @brief  Check if ADXL362 device is present
 * @param  ctx: ADXL362 context
 * @retval 1 if device found, 0 otherwise
 */
uint8_t ADXL362_CheckDevice(adxl362_ctx_t *ctx);

/**
 * @brief  Read device ID registers for debugging
 * @param  ctx: ADXL362 context
 * @param  devid_ad: Pointer to store DEVID_AD (expected 0xAD)
 * @param  devid_mst: Pointer to store DEVID_MST (expected 0x1D)
 * @param  partid: Pointer to store PARTID (expected 0xF2)
 * @retval None
 */
void ADXL362_ReadDeviceIDs(adxl362_ctx_t *ctx, uint8_t *devid_ad, uint8_t *devid_mst, uint8_t *partid);

/**
 * @brief  Convert raw value to millig (mg)
 * @param  raw_value: Raw 12-bit signed value from sensor
 * @param  range: Measurement range (0=2g, 1=4g, 2=8g)
 * @retval Acceleration in mg
 */
float ADXL362_RawToMg(int16_t raw_value, uint8_t range);

#ifdef __cplusplus
}
#endif

#endif /* __ADXL362_HAL_H__ */
