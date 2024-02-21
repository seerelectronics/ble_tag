#ifndef LIS2DS12_H
#define LIS2DS12_H

#include "nrf_drv_spi.h"
#include "lis2ds12_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

// WHO_AM_I register value.
#define LIS2DS12_WHO_AM_I               0x43

/**
 * @brief Output data rate settings.
 */
typedef enum
{
    LIS2DS12_ODR_POWERDOWN = 0,
    LIS2DS12_ODR_1HZ = 8,
    LIS2DS12_ODR_12_5HZ = 9,
    LIS2DS12_ODR_25HZ = 10,
    LIS2DS12_ODR_50HZ = 11,
    LIS2DS12_ODR_100HZ = 12,
    LIS2DS12_ODR_200HZ = 13,
    LIS2DS12_ODR_400HZ = 14,
    LIS2DS12_ODR_800HZ = 15,
} lis2ds12_odr_t;

/**
 * @brief Fifo mode settings.
 */
typedef enum
{
    LIS2DS12_FIFO_BYPASS = 0,
    LIS2DS12_FIFO_STOP_WHEN_FULL = 1,
    LIS2DS12_FIFO_CONTINUOUS_TO_FIFO = 3,
    LIS2DS12_FIFO_BYPASS_TO_CONTINUOUS = 4,
    LIS2DS12_FIFO_CONTINUOUS = 6
} lis2ds12_fifo_mode_t;

/**
 * @brief Accelerometer scale setting.
 */
typedef enum
{
    LIS2DS12_SCALE_2G = 0,
    LIS2DS12_SCALE_16G = 1,
    LIS2DS12_SCALE_4G = 2,
    LIS2DS12_SCALE_8G = 3,
} lis2ds12_scale_t;

#define LIS2DS12_INSTANCE_DEF(_lis2ds12_inst_name, _p_spi) \
    LIS2DS12_INTERNAL_INSTANCE_DEF(_lis2ds12_inst_name, _p_spi)

ret_code_t lis2ds12_init(lis2ds12_instance_t * p_inst);

void lis2ds12_set_odr(lis2ds12_instance_t * p_lis, lis2ds12_odr_t odr);
void lis2ds12_set_step_minths(lis2ds12_instance_t * p_lis, uint8_t minths);
void lis2ds12_get_acc(lis2ds12_instance_t * p_lis, int16_t *acc_xyz);
void lis2ds12_get_acc_fifo(lis2ds12_instance_t * p_lis, int16_t *acc_xyz, uint8_t size, uint8_t avg);
uint16_t lis2ds12_get_step_counter(lis2ds12_instance_t * p_lis);
uint16_t lis2ds12_get_fifo_level(lis2ds12_instance_t *p_lis);
void lis2ds12_set_hp_filter(lis2ds12_instance_t *p_lis, uint8_t state);
void lis2ds12_set_step_debounce(lis2ds12_instance_t * p_lis, uint8_t debounce);
void lis2ds12_set_step_delta(lis2ds12_instance_t * p_lis, uint8_t delta);
void lis2ds12_set_fs(lis2ds12_instance_t * p_lis, uint8_t fs);

#ifdef __cplusplus
}
#endif

#endif // LIS2DS12_H
