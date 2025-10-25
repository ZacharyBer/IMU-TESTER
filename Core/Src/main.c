/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "lsm6dsv_reg.h"
#include "lsm6dsox_reg.h"
#include "lis2dw12_reg.h"
#include "adxl362_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Configuration Macros */
#define IMU_PERFORMANCE_MODE 2  // 0 = Absolute Lowest Power (~2Hz, battery life priority)
                                // 1 = Moderate Performance (~60Hz, balanced power/performance)
                                // 2 = High Performance (~1.5kHz+, real-time control/logging)
#define OUTPUT_FORMAT 0         // 0 = CSV Format, 1 = Human-Readable Format

/* Validate IMU_PERFORMANCE_MODE */
#if (IMU_PERFORMANCE_MODE < 0 || IMU_PERFORMANCE_MODE > 2)
  #error "Invalid IMU_PERFORMANCE_MODE. Valid values are 0 (lowest power), 1 (moderate 60Hz), or 2 (high performance)."
#endif

/* IMU Enable/Disable - Set to 0 to disable a specific IMU at compile time */
#define ENABLE_LSM6DSV  0       // Enable LSM6DSV 6-axis IMU (I2C)
#define ENABLE_LSM6DSOX 1       // Enable LSM6DSOX 6-axis IMU (I2C)
#define ENABLE_LIS2DW12 0       // Enable LIS2DW12 3-axis accelerometer (I2C)
#define ENABLE_ADXL362  0       // Enable ADXL362 3-axis accelerometer (SPI)

/* LSM6DSV Feature Enable/Disable */
#define ENABLE_LSM6DSV_SENSOR_FUSION 0  // Enable SFLP (Sensor Fusion Low Power) for game rotation, gravity vectors, and gyro bias

/* IMU I2C Addresses */
#define LSM6DSV_I2C_ADDR  (0x6B << 1)  // LSM6DSV address (SDO/SA0 pin high)
#define LSM6DSOX_I2C_ADDR (0x6A << 1)  // LSM6DSOX default address
#define LIS2DW12_I2C_ADDR (0x19 << 1)  // LIS2DW12 default address

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* IMU Status Tracking */
typedef struct {
  uint8_t lsm6dsv_present;   // 1 if LSM6DSV detected, 0 otherwise
  uint8_t lsm6dsox_present;  // 1 if LSM6DSOX detected, 0 otherwise
  uint8_t lis2dw12_present;  // 1 if LIS2DW12 detected, 0 otherwise
  uint8_t adxl362_present;   // 1 if ADXL362 detected, 0 otherwise
} IMU_Status_t;

static IMU_Status_t imu_status = {0};

/* Sensor device contexts */
#if ENABLE_LSM6DSV
static stmdev_ctx_t lsm6dsv_ctx;
#endif
#if ENABLE_LSM6DSOX
static stmdev_ctx_t lsm6dsox_ctx;
#endif
#if ENABLE_LIS2DW12
static stmdev_ctx_t lis2dw12_ctx;
#endif
static adxl362_ctx_t adxl362_ctx;

/* Data buffers */
#if ENABLE_LSM6DSV
static int16_t lsm6dsv_accel_raw[3];  // X, Y, Z accelerometer
static int16_t lsm6dsv_gyro_raw[3];   // X, Y, Z gyroscope
#if ENABLE_LSM6DSV_SENSOR_FUSION
static float lsm6dsv_quat[4];         // Quaternion: w, x, y, z (from game rotation vector)
#endif
#endif
#if ENABLE_LSM6DSOX
static int16_t lsm6dsox_accel_raw[3]; // X, Y, Z accelerometer
static int16_t lsm6dsox_gyro_raw[3];  // X, Y, Z gyroscope
#endif
#if ENABLE_LIS2DW12
static int16_t lis2dw12_accel_raw[3]; // X, Y, Z accelerometer
#endif
static int16_t adxl362_accel_raw[3];  // X, Y, Z accelerometer

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* I2C Platform Functions */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/* Sensor Functions */
#if ENABLE_LSM6DSV
static uint8_t LSM6DSV_Init(void);   // Returns 1 if successful, 0 if failed
static void LSM6DSV_ReadData(void);
#if ENABLE_LSM6DSV_SENSOR_FUSION
static void LSM6DSV_ReadSFLP(void);
#endif
#endif
#if ENABLE_LSM6DSOX
static uint8_t LSM6DSOX_Init(void);  // Returns 1 if successful, 0 if failed
static void LSM6DSOX_ReadData(void);
#endif
#if ENABLE_LIS2DW12
static uint8_t LIS2DW12_Init(void);  // Returns 1 if successful, 0 if failed
#endif
#if ENABLE_LIS2DW12
static void LIS2DW12_ReadData(void);
#endif
static void ADXL362_ReadDataWrapper(void);

/* Utility Functions */
static uint32_t Get_Microseconds(void);
static void Print_IMU_Data(void);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void I2C_Scan(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================================
 * I2C Platform Functions
 * ============================================================================ */

/**
 * @brief  Write data to I2C device
 * @param  handle: I2C device address (passed as void*)
 * @param  reg: Register address to write to
 * @param  bufp: Pointer to data buffer
 * @param  len: Number of bytes to write
 * @retval 0 on success, -1 on error
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  uint8_t device_addr = (uint8_t)(uintptr_t)handle;

  if (HAL_I2C_Mem_Write(&hi2c2, device_addr, reg, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t*)bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
  return 0;
}

/**
 * @brief  Read data from I2C device
 * @param  handle: I2C device address (passed as void*)
 * @param  reg: Register address to read from
 * @param  bufp: Pointer to data buffer
 * @param  len: Number of bytes to read
 * @retval 0 on success, -1 on error
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  uint8_t device_addr = (uint8_t)(uintptr_t)handle;

  if (HAL_I2C_Mem_Read(&hi2c2, device_addr, reg, I2C_MEMADD_SIZE_8BIT,
                       bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
  return 0;
}

/* ============================================================================
 * Sensor Initialization Functions
 * ============================================================================ */

#if ENABLE_LSM6DSV
/**
 * @brief  Initialize LSM6DSV 6-axis IMU (Accelerometer + Gyroscope)
 * @retval 1 if initialization successful, 0 if device not found or error
 */
static uint8_t LSM6DSV_Init(void)
{
  uint8_t whoamI = 0;

  /* Initialize sensor context */
  lsm6dsv_ctx.write_reg = platform_write;
  lsm6dsv_ctx.read_reg = platform_read;
  lsm6dsv_ctx.mdelay = HAL_Delay;
  lsm6dsv_ctx.handle = (void*)(uintptr_t)LSM6DSV_I2C_ADDR;

  /* Check device ID */
  if (lsm6dsv_device_id_get(&lsm6dsv_ctx, &whoamI) != 0)
  {
    return 0;  // Communication error
  }

  if (whoamI != LSM6DSV_ID)
  {
    return 0;  // Device not found
  }

  /* Reset device */
  lsm6dsv_reset_t rst;
  lsm6dsv_reset_set(&lsm6dsv_ctx, LSM6DSV_RESTORE_CTRL_REGS);
  do {
    lsm6dsv_reset_get(&lsm6dsv_ctx, &rst);
  } while (rst != LSM6DSV_READY);

  /* Enable Block Data Update */
  lsm6dsv_block_data_update_set(&lsm6dsv_ctx, PROPERTY_ENABLE);

  /* Configure based on performance mode */
#if (IMU_PERFORMANCE_MODE == 0)
  /* Mode 0: Absolute Lowest Power - 1.875 Hz ODR, lowest power settings */
  lsm6dsv_xl_data_rate_set(&lsm6dsv_ctx, LSM6DSV_ODR_AT_1Hz875);
  lsm6dsv_gy_data_rate_set(&lsm6dsv_ctx, LSM6DSV_ODR_AT_1Hz875);
  lsm6dsv_xl_mode_set(&lsm6dsv_ctx, LSM6DSV_XL_LOW_POWER_2_AVG_MD);  // Lowest power with 2 averages
  lsm6dsv_gy_mode_set(&lsm6dsv_ctx, LSM6DSV_GY_LOW_POWER_MD);
#elif (IMU_PERFORMANCE_MODE == 1)
  /* Mode 1: Moderate Performance - 60 Hz ODR, low power mode for balanced operation */
  lsm6dsv_xl_data_rate_set(&lsm6dsv_ctx, LSM6DSV_ODR_AT_60Hz);
  lsm6dsv_gy_data_rate_set(&lsm6dsv_ctx, LSM6DSV_ODR_AT_60Hz);
  lsm6dsv_xl_mode_set(&lsm6dsv_ctx, LSM6DSV_XL_LOW_POWER_4_AVG_MD);  // Low power with 4 averages for better SNR
  lsm6dsv_gy_mode_set(&lsm6dsv_ctx, LSM6DSV_GY_LOW_POWER_MD);
#elif (IMU_PERFORMANCE_MODE == 2)
  /* Mode 2: High Performance - 7.68 kHz ODR, high performance mode */
  lsm6dsv_xl_data_rate_set(&lsm6dsv_ctx, LSM6DSV_ODR_AT_7680Hz);
  lsm6dsv_gy_data_rate_set(&lsm6dsv_ctx, LSM6DSV_ODR_AT_7680Hz);
  lsm6dsv_xl_mode_set(&lsm6dsv_ctx, LSM6DSV_XL_HIGH_PERFORMANCE_MD);
  lsm6dsv_gy_mode_set(&lsm6dsv_ctx, LSM6DSV_GY_HIGH_PERFORMANCE_MD);
#endif

  /* Set full-scale ranges */
  lsm6dsv_xl_full_scale_set(&lsm6dsv_ctx, LSM6DSV_4g);
  lsm6dsv_gy_full_scale_set(&lsm6dsv_ctx, LSM6DSV_2000dps);

#if ENABLE_LSM6DSV_SENSOR_FUSION
  /* Configure SFLP (Sensor Fusion Low Power) for game rotation vector */

  /* Enable SFLP game rotation (6-axis fusion: accel + gyro) */
  lsm6dsv_sflp_game_rotation_set(&lsm6dsv_ctx, 1);

  /* Set SFLP ODR based on performance mode */
#if (IMU_PERFORMANCE_MODE == 0)
  lsm6dsv_sflp_data_rate_set(&lsm6dsv_ctx, LSM6DSV_SFLP_15Hz);   // Mode 0: Minimum power (15 Hz)
#elif (IMU_PERFORMANCE_MODE == 1)
  lsm6dsv_sflp_data_rate_set(&lsm6dsv_ctx, LSM6DSV_SFLP_60Hz);   // Mode 1: Moderate (60 Hz)
#elif (IMU_PERFORMANCE_MODE == 2)
  lsm6dsv_sflp_data_rate_set(&lsm6dsv_ctx, LSM6DSV_SFLP_480Hz);  // Mode 2: Maximum performance (480 Hz)
#endif

  /* Enable SFLP data in FIFO */
  lsm6dsv_fifo_sflp_raw_t sflp_fifo;
  sflp_fifo.game_rotation = 1;  // Enable game rotation vector in FIFO
  sflp_fifo.gravity = 0;        // Disable gravity vector (can enable if needed)
  sflp_fifo.gbias = 0;          // Disable gyro bias output (can enable if needed)
  lsm6dsv_fifo_sflp_batch_set(&lsm6dsv_ctx, sflp_fifo);

  /* Configure FIFO mode to continuous */
  lsm6dsv_fifo_mode_set(&lsm6dsv_ctx, LSM6DSV_STREAM_MODE);
#endif

  return 1;  // Initialization successful
}
#endif

#if ENABLE_LSM6DSOX
/**
 * @brief  Initialize LSM6DSOX 6-axis IMU (Accelerometer + Gyroscope)
 * @retval 1 if initialization successful, 0 if device not found or error
 */
static uint8_t LSM6DSOX_Init(void)
{
  uint8_t whoamI = 0;

  /* Initialize sensor context */
  lsm6dsox_ctx.write_reg = platform_write;
  lsm6dsox_ctx.read_reg = platform_read;
  lsm6dsox_ctx.mdelay = HAL_Delay;
  lsm6dsox_ctx.handle = (void*)(uintptr_t)LSM6DSOX_I2C_ADDR;

  /* Check device ID */
  if (lsm6dsox_device_id_get(&lsm6dsox_ctx, &whoamI) != 0)
  {
    return 0;  // Communication error
  }

  if (whoamI != LSM6DSOX_ID)
  {
    return 0;  // Device not found
  }

  /* Reset device */
  lsm6dsox_reset_set(&lsm6dsox_ctx, PROPERTY_ENABLE);
  uint8_t rst;
  do {
    lsm6dsox_reset_get(&lsm6dsox_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&lsm6dsox_ctx, PROPERTY_ENABLE);

  /* Configure based on performance mode */
#if (IMU_PERFORMANCE_MODE == 0)
  /* Mode 0: Absolute Lowest Power - 12.5 Hz ODR, low normal power */
  lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx, LSM6DSOX_XL_ODR_12Hz5);
  lsm6dsox_gy_data_rate_set(&lsm6dsox_ctx, LSM6DSOX_GY_ODR_12Hz5);
  lsm6dsox_xl_power_mode_set(&lsm6dsox_ctx, LSM6DSOX_LOW_NORMAL_POWER_MD);
  lsm6dsox_gy_power_mode_set(&lsm6dsox_ctx, LSM6DSOX_GY_NORMAL);
#elif (IMU_PERFORMANCE_MODE == 1)
  /* Mode 1: Moderate Performance - 52 Hz ODR, normal power mode */
  lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx, LSM6DSOX_XL_ODR_52Hz);
  lsm6dsox_gy_data_rate_set(&lsm6dsox_ctx, LSM6DSOX_GY_ODR_52Hz);
  lsm6dsox_xl_power_mode_set(&lsm6dsox_ctx, LSM6DSOX_LOW_NORMAL_POWER_MD);
  lsm6dsox_gy_power_mode_set(&lsm6dsox_ctx, LSM6DSOX_GY_NORMAL);
#elif (IMU_PERFORMANCE_MODE == 2)
  /* Mode 2: High Performance - 6.667 kHz ODR, high performance mode */
  lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx, LSM6DSOX_XL_ODR_6667Hz);
  lsm6dsox_gy_data_rate_set(&lsm6dsox_ctx, LSM6DSOX_GY_ODR_6667Hz);
  lsm6dsox_xl_power_mode_set(&lsm6dsox_ctx, LSM6DSOX_HIGH_PERFORMANCE_MD);
  lsm6dsox_gy_power_mode_set(&lsm6dsox_ctx, LSM6DSOX_GY_HIGH_PERFORMANCE);
#endif

  /* Set full-scale ranges */
  lsm6dsox_xl_full_scale_set(&lsm6dsox_ctx, LSM6DSOX_4g);
  lsm6dsox_gy_full_scale_set(&lsm6dsox_ctx, LSM6DSOX_2000dps);

  return 1;  // Initialization successful
}
#endif

#if ENABLE_LIS2DW12
/**
 * @brief  Initialize LIS2DW12 3-axis Accelerometer
 * @retval 1 if initialization successful, 0 if device not found or error
 */
static uint8_t LIS2DW12_Init(void)
{
  uint8_t whoamI = 0;

  /* Initialize sensor context */
  lis2dw12_ctx.write_reg = platform_write;
  lis2dw12_ctx.read_reg = platform_read;
  lis2dw12_ctx.mdelay = HAL_Delay;
  lis2dw12_ctx.handle = (void*)(uintptr_t)LIS2DW12_I2C_ADDR;

  /* Check device ID */
  if (lis2dw12_device_id_get(&lis2dw12_ctx, &whoamI) != 0)
  {
    return 0;  // Communication error
  }

  if (whoamI != LIS2DW12_ID)
  {
    return 0;  // Device not found
  }

  /* Reset device */
  lis2dw12_reset_set(&lis2dw12_ctx, PROPERTY_ENABLE);
  uint8_t rst;
  do {
    lis2dw12_reset_get(&lis2dw12_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lis2dw12_block_data_update_set(&lis2dw12_ctx, PROPERTY_ENABLE);

  /* Configure based on performance mode */
#if (IMU_PERFORMANCE_MODE == 0)
  /* Mode 0: Absolute Lowest Power - 1.6 Hz ODR, continuous low power mode 2 */
  lis2dw12_data_rate_set(&lis2dw12_ctx, LIS2DW12_XL_ODR_1Hz6_LP_ONLY);
  lis2dw12_power_mode_set(&lis2dw12_ctx, LIS2DW12_CONT_LOW_PWR_2);  // Lowest continuous power mode
#elif (IMU_PERFORMANCE_MODE == 1)
  /* Mode 1: Moderate Performance - 50 Hz ODR, low power 12-bit mode */
  lis2dw12_data_rate_set(&lis2dw12_ctx, LIS2DW12_XL_ODR_50Hz);
  lis2dw12_power_mode_set(&lis2dw12_ctx, LIS2DW12_CONT_LOW_PWR_12bit);  // Low power with 12-bit resolution
#elif (IMU_PERFORMANCE_MODE == 2)
  /* Mode 2: High Performance - 1.6 kHz ODR, high performance mode */
  lis2dw12_data_rate_set(&lis2dw12_ctx, LIS2DW12_XL_ODR_1k6Hz);
  lis2dw12_power_mode_set(&lis2dw12_ctx, LIS2DW12_HIGH_PERFORMANCE);
#endif

  /* Set full-scale range */
  lis2dw12_full_scale_set(&lis2dw12_ctx, LIS2DW12_4g);

  return 1;  // Initialization successful
}
#endif

/* ============================================================================
 * Sensor Data Reading Functions
 * ============================================================================ */

#if ENABLE_LSM6DSV
/**
 * @brief  Read data from LSM6DSV (Accelerometer + Gyroscope)
 */
static void LSM6DSV_ReadData(void)
{
  lsm6dsv_all_sources_t status;

  /* Check if new data is available */
  lsm6dsv_all_sources_get(&lsm6dsv_ctx, &status);

  if (status.drdy_xl || status.drdy_gy)
  {
    /* Read accelerometer and gyroscope data (6 bytes each) */
    lsm6dsv_acceleration_raw_get(&lsm6dsv_ctx, lsm6dsv_accel_raw);
    lsm6dsv_angular_rate_raw_get(&lsm6dsv_ctx, lsm6dsv_gyro_raw);
  }
}

#if ENABLE_LSM6DSV_SENSOR_FUSION
/**
 * @brief  Read SFLP (Sensor Fusion) data from LSM6DSV FIFO
 */
static void LSM6DSV_ReadSFLP(void)
{
  lsm6dsv_fifo_status_t fifo_status;
  lsm6dsv_fifo_out_raw_t fifo_data;
  uint16_t fifo_level;

  /* Get FIFO status */
  lsm6dsv_fifo_status_get(&lsm6dsv_ctx, &fifo_status);
  fifo_level = fifo_status.fifo_level;

  /* Read FIFO data and look for SFLP game rotation vector tag */
  while (fifo_level > 0)
  {
    /* Read one FIFO entry (7 bytes: 1 tag + 6 data) */
    lsm6dsv_fifo_out_raw_get(&lsm6dsv_ctx, &fifo_data);
    fifo_level--;

    /* Check if this is SFLP game rotation vector data (tag 0x13) */
    if (fifo_data.tag == LSM6DSV_SFLP_GAME_ROTATION_VECTOR_TAG)
    {
      /* Game rotation vector is a compressed quaternion (3 components)
       * The 6 bytes contain 3x 16-bit signed values representing x, y, z
       * The w component is derived: w = sqrt(1 - x^2 - y^2 - z^2)
       * Values are normalized such that sqrt(x^2 + y^2 + z^2 + w^2) = 1
       */
      int16_t quat_raw[3];

      /* Extract 16-bit quaternion components (little-endian) */
      quat_raw[0] = (int16_t)(fifo_data.data[1] << 8) | fifo_data.data[0];  // x
      quat_raw[1] = (int16_t)(fifo_data.data[3] << 8) | fifo_data.data[2];  // y
      quat_raw[2] = (int16_t)(fifo_data.data[5] << 8) | fifo_data.data[4];  // z

      /* Convert to float (normalized to ±1, scale factor is 2^14 = 16384) */
      lsm6dsv_quat[1] = (float)quat_raw[0] / 16384.0f;  // x
      lsm6dsv_quat[2] = (float)quat_raw[1] / 16384.0f;  // y
      lsm6dsv_quat[3] = (float)quat_raw[2] / 16384.0f;  // z

      /* Calculate w component: w = sqrt(1 - x^2 - y^2 - z^2) */
      float sum_sq = lsm6dsv_quat[1] * lsm6dsv_quat[1] +
                     lsm6dsv_quat[2] * lsm6dsv_quat[2] +
                     lsm6dsv_quat[3] * lsm6dsv_quat[3];

      if (sum_sq < 1.0f)
      {
        lsm6dsv_quat[0] = sqrtf(1.0f - sum_sq);  // w
      }
      else
      {
        lsm6dsv_quat[0] = 0.0f;  // Handle edge case
      }

      /* We found and processed the SFLP data, can break */
      break;
    }
  }
}
#endif

#endif

#if ENABLE_LSM6DSOX
/**
 * @brief  Read data from LSM6DSOX (Accelerometer + Gyroscope)
 */
static void LSM6DSOX_ReadData(void)
{
  lsm6dsox_status_reg_t status;

  /* Check if new data is available */
  lsm6dsox_status_reg_get(&lsm6dsox_ctx, &status);

  if (status.xlda || status.gda)
  {
    /* Read accelerometer and gyroscope data (6 bytes each) */
    lsm6dsox_acceleration_raw_get(&lsm6dsox_ctx, lsm6dsox_accel_raw);
    lsm6dsox_angular_rate_raw_get(&lsm6dsox_ctx, lsm6dsox_gyro_raw);
  }
}
#endif

#if ENABLE_LIS2DW12
/**
 * @brief  Read data from LIS2DW12 (Accelerometer)
 */
static void LIS2DW12_ReadData(void)
{
  lis2dw12_status_t status;

  /* Check if new data is available */
  lis2dw12_status_reg_get(&lis2dw12_ctx, &status);

  if (status.drdy)
  {
    /* Read accelerometer data */
    lis2dw12_acceleration_raw_get(&lis2dw12_ctx, lis2dw12_accel_raw);
  }
}
#endif

/**
 * @brief  Read data from ADXL362 (Accelerometer via SPI)
 */
static void ADXL362_ReadDataWrapper(void)
{
  /* ADXL362 sampling rate depends on IMU_PERFORMANCE_MODE:
   * Mode 0: Wakeup mode (6 Hz, ultralow noise)
   * Mode 1: Measurement mode (100 Hz, normal noise) */
  /* Using full 12-bit resolution from 16-bit registers */
  ADXL362_ReadData(&adxl362_ctx, adxl362_accel_raw);
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief  Get microsecond timestamp from TIM2
 * @retval Current timestamp in microseconds
 */
static uint32_t Get_Microseconds(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}

/**
 * @brief  Transmit data via UART (USART1 at 1Mbit)
 * @param  tx_buffer: Pointer to data buffer
 * @param  len: Number of bytes to transmit
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&huart1, tx_buffer, len, HAL_MAX_DELAY);
}

/**
 * @brief  Scan I2C bus for devices and print results
 */
static void I2C_Scan(void)
{
  char buffer[128];
  uint8_t devices_found = 0;

  snprintf(buffer, sizeof(buffer), "\r\n=== I2C Bus Scan ===\r\n");
  tx_com((uint8_t*)buffer, strlen(buffer));

  snprintf(buffer, sizeof(buffer), "Scanning I2C2 (0x08 to 0x77)...\r\n");
  tx_com((uint8_t*)buffer, strlen(buffer));

  for (uint8_t addr = 0x08; addr <= 0x77; addr++)
  {
    /* Try to communicate with device at this address */
    if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 10) == HAL_OK)
    {
      snprintf(buffer, sizeof(buffer), "  Found device at 0x%02X", addr);
      tx_com((uint8_t*)buffer, strlen(buffer));

      /* Identify known devices */
      if (addr == 0x6B)
      {
        snprintf(buffer, sizeof(buffer), " (LSM6DSV - 6-axis IMU)\r\n");
      }
      else if (addr == 0x19)
      {
        snprintf(buffer, sizeof(buffer), " (LIS2DW12 - 3-axis Accel)\r\n");
      }
      else if (addr == 0x6A || addr == 0x6C)
      {
        snprintf(buffer, sizeof(buffer), " (LSM6DSV/LSM6DSOX - 6-axis IMU)\r\n");
      }
      else if (addr == 0x18)
      {
        snprintf(buffer, sizeof(buffer), " (Possible LIS2DW12 alternate)\r\n");
      }
      else
      {
        snprintf(buffer, sizeof(buffer), " (Unknown device)\r\n");
      }
      tx_com((uint8_t*)buffer, strlen(buffer));
      devices_found++;
    }
  }

  if (devices_found == 0)
  {
    snprintf(buffer, sizeof(buffer), "  No I2C devices found!\r\n");
    tx_com((uint8_t*)buffer, strlen(buffer));
  }
  else
  {
    snprintf(buffer, sizeof(buffer), "\r\nTotal devices found: %d\r\n", devices_found);
    tx_com((uint8_t*)buffer, strlen(buffer));
  }

  snprintf(buffer, sizeof(buffer), "===================\r\n\r\n");
  tx_com((uint8_t*)buffer, strlen(buffer));
}

/**
 * @brief  Print IMU data with timestamp in selected format
 * @note   Each IMU outputs its own line with format: IMU_ID,timestamp,data...
 */
static void Print_IMU_Data(void)
{
  char buffer[256];
  uint32_t timestamp = Get_Microseconds();

#if (OUTPUT_FORMAT == 0)
  /* CSV Format - one line per IMU with identifier */

#if ENABLE_LSM6DSV
  /* LSM6DSV output: LSM6DSV,timestamp,ax,ay,az,gx,gy,gz */
  if (imu_status.lsm6dsv_present)
  {
    float lsm_ax = lsm6dsv_from_fs4_to_mg(lsm6dsv_accel_raw[0]);
    float lsm_ay = lsm6dsv_from_fs4_to_mg(lsm6dsv_accel_raw[1]);
    float lsm_az = lsm6dsv_from_fs4_to_mg(lsm6dsv_accel_raw[2]);
    float lsm_gx = lsm6dsv_from_fs2000_to_mdps(lsm6dsv_gyro_raw[0]);
    float lsm_gy = lsm6dsv_from_fs2000_to_mdps(lsm6dsv_gyro_raw[1]);
    float lsm_gz = lsm6dsv_from_fs2000_to_mdps(lsm6dsv_gyro_raw[2]);

    snprintf(buffer, sizeof(buffer),
             "LSM6DSV,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
             timestamp, lsm_ax, lsm_ay, lsm_az, lsm_gx, lsm_gy, lsm_gz);
    tx_com((uint8_t*)buffer, strlen(buffer));

#if ENABLE_LSM6DSV_SENSOR_FUSION
    /* LSM6DSV_SFLP output: LSM6DSV_SFLP,timestamp,qw,qx,qy,qz */
    snprintf(buffer, sizeof(buffer),
             "LSM6DSV_SFLP,%lu,%.4f,%.4f,%.4f,%.4f\r\n",
             timestamp, lsm6dsv_quat[0], lsm6dsv_quat[1], lsm6dsv_quat[2], lsm6dsv_quat[3]);
    tx_com((uint8_t*)buffer, strlen(buffer));
#endif
  }
#endif

#if ENABLE_LSM6DSOX
  /* LSM6DSOX output: LSM6DSOX,timestamp,ax,ay,az,gx,gy,gz */
  if (imu_status.lsm6dsox_present)
  {
    float lsmox_ax = lsm6dsox_from_fs4_to_mg(lsm6dsox_accel_raw[0]);
    float lsmox_ay = lsm6dsox_from_fs4_to_mg(lsm6dsox_accel_raw[1]);
    float lsmox_az = lsm6dsox_from_fs4_to_mg(lsm6dsox_accel_raw[2]);
    float lsmox_gx = lsm6dsox_from_fs2000_to_mdps(lsm6dsox_gyro_raw[0]);
    float lsmox_gy = lsm6dsox_from_fs2000_to_mdps(lsm6dsox_gyro_raw[1]);
    float lsmox_gz = lsm6dsox_from_fs2000_to_mdps(lsm6dsox_gyro_raw[2]);

    snprintf(buffer, sizeof(buffer),
             "LSM6DSOX,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
             timestamp, lsmox_ax, lsmox_ay, lsmox_az, lsmox_gx, lsmox_gy, lsmox_gz);
    tx_com((uint8_t*)buffer, strlen(buffer));
  }
#endif

#if ENABLE_LIS2DW12
  /* LIS2DW12 output: LIS2DW12,timestamp,ax,ay,az */
  if (imu_status.lis2dw12_present)
  {
    float lis_ax = lis2dw12_from_fs4_to_mg(lis2dw12_accel_raw[0]);
    float lis_ay = lis2dw12_from_fs4_to_mg(lis2dw12_accel_raw[1]);
    float lis_az = lis2dw12_from_fs4_to_mg(lis2dw12_accel_raw[2]);

    snprintf(buffer, sizeof(buffer),
             "LIS2DW12,%lu,%.2f,%.2f,%.2f\r\n",
             timestamp, lis_ax, lis_ay, lis_az);
    tx_com((uint8_t*)buffer, strlen(buffer));
  }
#endif

  /* ADXL362 output: ADXL362,timestamp,ax,ay,az */
  if (imu_status.adxl362_present)
  {
    float adxl_ax = ADXL362_RawToMg(adxl362_accel_raw[0], ADXL362_RANGE_2G);
    float adxl_ay = ADXL362_RawToMg(adxl362_accel_raw[1], ADXL362_RANGE_2G);
    float adxl_az = ADXL362_RawToMg(adxl362_accel_raw[2], ADXL362_RANGE_2G);

    snprintf(buffer, sizeof(buffer),
             "ADXL362,%lu,%.2f,%.2f,%.2f\r\n",
             timestamp, adxl_ax, adxl_ay, adxl_az);
    tx_com((uint8_t*)buffer, strlen(buffer));
  }

#else
  /* Human-Readable Format */

#if ENABLE_LSM6DSV
  if (imu_status.lsm6dsv_present)
  {
    float lsm_ax = lsm6dsv_from_fs4_to_mg(lsm6dsv_accel_raw[0]);
    float lsm_ay = lsm6dsv_from_fs4_to_mg(lsm6dsv_accel_raw[1]);
    float lsm_az = lsm6dsv_from_fs4_to_mg(lsm6dsv_accel_raw[2]);
    float lsm_gx = lsm6dsv_from_fs2000_to_mdps(lsm6dsv_gyro_raw[0]);
    float lsm_gy = lsm6dsv_from_fs2000_to_mdps(lsm6dsv_gyro_raw[1]);
    float lsm_gz = lsm6dsv_from_fs2000_to_mdps(lsm6dsv_gyro_raw[2]);

    snprintf(buffer, sizeof(buffer),
             "[%lu us] LSM6DSV: A(%.2f,%.2f,%.2f)mg G(%.2f,%.2f,%.2f)mdps\r\n",
             timestamp, lsm_ax, lsm_ay, lsm_az, lsm_gx, lsm_gy, lsm_gz);
    tx_com((uint8_t*)buffer, strlen(buffer));

#if ENABLE_LSM6DSV_SENSOR_FUSION
    snprintf(buffer, sizeof(buffer),
             "[%lu us] LSM6DSV_SFLP: Q(%.4f,%.4f,%.4f,%.4f)\r\n",
             timestamp, lsm6dsv_quat[0], lsm6dsv_quat[1], lsm6dsv_quat[2], lsm6dsv_quat[3]);
    tx_com((uint8_t*)buffer, strlen(buffer));
#endif
  }
#endif

#if ENABLE_LSM6DSOX
  if (imu_status.lsm6dsox_present)
  {
    float lsmox_ax = lsm6dsox_from_fs4_to_mg(lsm6dsox_accel_raw[0]);
    float lsmox_ay = lsm6dsox_from_fs4_to_mg(lsm6dsox_accel_raw[1]);
    float lsmox_az = lsm6dsox_from_fs4_to_mg(lsm6dsox_accel_raw[2]);
    float lsmox_gx = lsm6dsox_from_fs2000_to_mdps(lsm6dsox_gyro_raw[0]);
    float lsmox_gy = lsm6dsox_from_fs2000_to_mdps(lsm6dsox_gyro_raw[1]);
    float lsmox_gz = lsm6dsox_from_fs2000_to_mdps(lsm6dsox_gyro_raw[2]);

    snprintf(buffer, sizeof(buffer),
             "[%lu us] LSM6DSOX: A(%.2f,%.2f,%.2f)mg G(%.2f,%.2f,%.2f)mdps\r\n",
             timestamp, lsmox_ax, lsmox_ay, lsmox_az, lsmox_gx, lsmox_gy, lsmox_gz);
    tx_com((uint8_t*)buffer, strlen(buffer));
  }
#endif

#if ENABLE_LIS2DW12
  if (imu_status.lis2dw12_present)
  {
    float lis_ax = lis2dw12_from_fs4_to_mg(lis2dw12_accel_raw[0]);
    float lis_ay = lis2dw12_from_fs4_to_mg(lis2dw12_accel_raw[1]);
    float lis_az = lis2dw12_from_fs4_to_mg(lis2dw12_accel_raw[2]);

    snprintf(buffer, sizeof(buffer),
             "[%lu us] LIS2DW12: A(%.2f,%.2f,%.2f)mg\r\n",
             timestamp, lis_ax, lis_ay, lis_az);
    tx_com((uint8_t*)buffer, strlen(buffer));
  }
#endif

  if (imu_status.adxl362_present)
  {
    float adxl_ax = ADXL362_RawToMg(adxl362_accel_raw[0], ADXL362_RANGE_2G);
    float adxl_ay = ADXL362_RawToMg(adxl362_accel_raw[1], ADXL362_RANGE_2G);
    float adxl_az = ADXL362_RawToMg(adxl362_accel_raw[2], ADXL362_RANGE_2G);

    snprintf(buffer, sizeof(buffer),
             "[%lu us] ADXL362: A(%.2f,%.2f,%.2f)mg\r\n",
             timestamp, adxl_ax, adxl_ay, adxl_az);
    tx_com((uint8_t*)buffer, strlen(buffer));
  }

#endif
}

/**
 * @brief  Retargets the C library printf function to the UART.
 * @param  ch: Character to send
 * @retval Character sent
 * @note   This enables printf() to work via USART1 at 1Mbit
 */
#ifdef __GNUC__
/* For GCC/G++ compiler */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
#elif defined(__ICCARM__)
/* For IAR compiler */
int putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
/* For Keil ARM/ARM Compiler 5/6 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Set ADXL362 CS pin HIGH (idle state) - must be done after GPIO init */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /* Wait for ADXL362 to power up (datasheet: max 6ms settling time) */
  HAL_Delay(50);

  /* Start the microsecond timer */
  HAL_TIM_Base_Start(&htim2);

  /* USART1 already initialized at 1Mbit via MX_USART1_UART_Init() */

  /* Wait for peripherals to stabilize */
  HAL_Delay(100);

  /* Scan I2C bus for connected devices */
  I2C_Scan();

  /* Initialize IMU sensors - attempt each one and track which are present */
  char buffer[256];

#if ENABLE_LSM6DSV
  imu_status.lsm6dsv_present = LSM6DSV_Init();
#else
  imu_status.lsm6dsv_present = 0;
#endif

#if ENABLE_LSM6DSOX
  imu_status.lsm6dsox_present = LSM6DSOX_Init();
#else
  imu_status.lsm6dsox_present = 0;
#endif

#if ENABLE_LIS2DW12
  imu_status.lis2dw12_present = LIS2DW12_Init();
#else
  imu_status.lis2dw12_present = 0;
#endif

#if ENABLE_ADXL362
  /* Initialize ADXL362 context */
  adxl362_ctx.hspi = &hspi1;
  adxl362_ctx.cs_port = ADXL362_CS_PORT;
  adxl362_ctx.cs_pin = ADXL362_CS_PIN;

  /* Debug: Try to read device ID registers */
  snprintf(buffer, sizeof(buffer), "\r\nADXL362 Debug: Attempting SPI communication...\r\n");
  tx_com((uint8_t*)buffer, strlen(buffer));

  uint8_t devid_ad, devid_mst, partid;
  ADXL362_ReadDeviceIDs(&adxl362_ctx, &devid_ad, &devid_mst, &partid);

  snprintf(buffer, sizeof(buffer), "  Read DEVID_AD:  0x%02X (expected 0xAD)\r\n", devid_ad);
  tx_com((uint8_t*)buffer, strlen(buffer));
  snprintf(buffer, sizeof(buffer), "  Read DEVID_MST: 0x%02X (expected 0x1D)\r\n", devid_mst);
  tx_com((uint8_t*)buffer, strlen(buffer));
  snprintf(buffer, sizeof(buffer), "  Read PARTID:    0x%02X (expected 0xF2)\r\n", partid);
  tx_com((uint8_t*)buffer, strlen(buffer));

  /* Try basic device detection */
  if (ADXL362_CheckDevice(&adxl362_ctx))
  {
    snprintf(buffer, sizeof(buffer), "ADXL362 device ID verified! Initializing...\r\n");
    tx_com((uint8_t*)buffer, strlen(buffer));

    imu_status.adxl362_present = ADXL362_Init(&adxl362_ctx);

    if (imu_status.adxl362_present)
    {
      snprintf(buffer, sizeof(buffer), "ADXL362 initialization SUCCESS!\r\n");
      tx_com((uint8_t*)buffer, strlen(buffer));
    }
    else
    {
      snprintf(buffer, sizeof(buffer), "ADXL362 initialization FAILED (reset or config error)\r\n");
      tx_com((uint8_t*)buffer, strlen(buffer));
    }
  }
  else
  {
    snprintf(buffer, sizeof(buffer), "ADXL362 device ID check FAILED - no communication\r\n");
    tx_com((uint8_t*)buffer, strlen(buffer));
    imu_status.adxl362_present = 0;
  }
#else
  imu_status.adxl362_present = 0;
#endif

  /* Print detected IMU status */
  snprintf(buffer, sizeof(buffer), "\r\n=== IMU Detection Results ===\r\n");
  tx_com((uint8_t*)buffer, strlen(buffer));

  snprintf(buffer, sizeof(buffer), "LSM6DSV (6-axis I2C):    %s\r\n",
           imu_status.lsm6dsv_present ? "DETECTED" : "NOT FOUND");
  tx_com((uint8_t*)buffer, strlen(buffer));

  snprintf(buffer, sizeof(buffer), "LSM6DSOX (6-axis I2C):   %s\r\n",
           imu_status.lsm6dsox_present ? "DETECTED" : "NOT FOUND");
  tx_com((uint8_t*)buffer, strlen(buffer));

  snprintf(buffer, sizeof(buffer), "LIS2DW12 (3-axis I2C):   %s\r\n",
           imu_status.lis2dw12_present ? "DETECTED" : "NOT FOUND");
  tx_com((uint8_t*)buffer, strlen(buffer));

  snprintf(buffer, sizeof(buffer), "ADXL362 (3-axis SPI):    %s\r\n",
           imu_status.adxl362_present ? "DETECTED" : "NOT FOUND");
  tx_com((uint8_t*)buffer, strlen(buffer));

  snprintf(buffer, sizeof(buffer), "==============================\r\n\r\n");
  tx_com((uint8_t*)buffer, strlen(buffer));

  /* Check if at least one IMU is present */
  if (!imu_status.lsm6dsv_present && !imu_status.lsm6dsox_present && !imu_status.lis2dw12_present && !imu_status.adxl362_present)
  {
    snprintf(buffer, sizeof(buffer), "ERROR: No IMUs detected! Check connections.\r\n");
    tx_com((uint8_t*)buffer, strlen(buffer));
  }

  /* Print CSV header or startup message */
#if (OUTPUT_FORMAT == 0)
  snprintf(buffer, sizeof(buffer), "Starting data acquisition (CSV format)...\r\n\r\n");
  tx_com((uint8_t*)buffer, strlen(buffer));
#else
  snprintf(buffer, sizeof(buffer), "Starting data acquisition...\r\n\r\n");
  tx_com((uint8_t*)buffer, strlen(buffer));
#endif

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Read data only from detected IMUs */
#if ENABLE_LSM6DSV
    if (imu_status.lsm6dsv_present)
    {
      LSM6DSV_ReadData();
#if ENABLE_LSM6DSV_SENSOR_FUSION
      LSM6DSV_ReadSFLP();
#endif
    }
#endif

#if ENABLE_LSM6DSOX
    if (imu_status.lsm6dsox_present)
    {
      LSM6DSOX_ReadData();
    }
#endif

#if ENABLE_LIS2DW12
    if (imu_status.lis2dw12_present)
    {
      LIS2DW12_ReadData();
    }
#endif

    if (imu_status.adxl362_present)
    {
      ADXL362_ReadDataWrapper();
    }

    /* Print the timestamped data (only outputs data from detected IMUs) */
    if (imu_status.lsm6dsv_present || imu_status.lsm6dsox_present || imu_status.lis2dw12_present || imu_status.adxl362_present)
    {
      Print_IMU_Data();
    }

    /* Small delay to prevent overwhelming the system */
    /* Note: In low power mode, ADXL362 samples at 6 Hz (~167ms period) */
    HAL_Delay(1);  // 1ms delay -> ~1 kHz loop rate

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0010061A;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_HIGH;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UCPD_DBn_GPIO_Port, UCPD_DBn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_DBn_Pin */
  GPIO_InitStruct.Pin = UCPD_DBn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UCPD_DBn_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
