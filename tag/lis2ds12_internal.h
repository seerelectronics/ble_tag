#ifndef LIS2DS12_INTERNAL
#define LIS2DS12_INTERNAL

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LIS2DS12 sensor registers.
 */
#define LIS2DS12_REG_SENSORHUB1         0x06
#define LIS2DS12_REG_SENSORHUB2         0x07
#define LIS2DS12_REG_SENSORHUB3         0x08
#define LIS2DS12_REG_SENSORHUB4         0x09
#define LIS2DS12_REG_SENSORHUB5         0x0A
#define LIS2DS12_REG_SENSORHUB6         0x0B
#define LIS2DS12_REG_MOD_8BIT           0x0C
#define LIS2DS12_REG_WHO_AM_I           0x0F
#define LIS2DS12_REG_CTRL1              0x20
#define LIS2DS12_REG_CTRL2              0x21
#define LIS2DS12_REG_CTRL3              0x22
#define LIS2DS12_REG_CTRL4              0x23
#define LIS2DS12_REG_CTRL5              0x24
#define LIS2DS12_REG_FIFO_CTRL          0x25
#define LIS2DS12_REG_OUT_T              0x26
#define LIS2DS12_REG_STATUS             0x27
#define LIS2DS12_REG_OUT_X_L            0x28
#define LIS2DS12_REG_OUT_X_H            0x29
#define LIS2DS12_REG_OUT_Y_L            0x2A
#define LIS2DS12_REG_OUT_Y_H            0x2B
#define LIS2DS12_REG_OUT_Z_L            0x2C
#define LIS2DS12_REG_OUT_Z_H            0x2D
#define LIS2DS12_REG_FIFO_THS           0x2E
#define LIS2DS12_REG_FIFO_SRC           0x2F
#define LIS2DS12_REG_FIFO_SAMPLES       0x30
#define LIS2DS12_REG_TAP_6D_THS         0x31
#define LIS2DS12_REG_INT_DUR            0x32
#define LIS2DS12_REG_WAKE_UP_THS        0x33
#define LIS2DS12_REG_WAKE_UP_DUR        0x34
#define LIS2DS12_REG_FREE_FALL          0x35
#define LIS2DS12_REG_STATUS_DUP         0x36
#define LIS2DS12_REG_WAKE_UP_SRC        0x37
#define LIS2DS12_REG_TAP_SRC            0x38
#define LIS2DS12_REG_6D_SRC             0x39
#define LIS2DS12_REG_STEP_MINTHS        0x3A
#define LIS2DS12_REG_STEP_L             0x3B
#define LIS2DS12_REG_STEP_H             0x3C
#define LIS2DS12_REG_FUNC_CK_GATE       0x3D
#define LIS2DS12_REG_FUNC_SRC           0x3E
#define LIS2DS12_REG_FUNC_CTRL          0x3F

#define LIS2DS12_REG_PEDOMETER_DEBOUNCE 0x2B
#define LIS2DS12_REG_STEP_COUNT_DELTA   0x3A

/**
 * @brief Structure holding sensor instance
 */
typedef struct
{
    nrf_drv_spi_t * const p_spi;

} lis2ds12_instance_t;

/**
 * @brief Macro for defining sensor instance.
 */
#define LIS2DS12_INTERNAL_INSTANCE_DEF(_lis2ds12_inst_name, _p_spi)       \
    static lis2ds12_instance_t _lis2ds12_inst_name =                      \
    {                                                                     \
        .p_spi = _p_spi,                                                  \
    }

#ifdef __cplusplus
}
#endif

#endif // LIS2DS12_INTERNAL
