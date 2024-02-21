#include "lis2ds12.h"

#include "nrf_log.h"

#define RETURN_IF_ERR(err)  \
    if (err != NRF_SUCCESS) \
    {                       \
        return err;         \
    }

static uint8_t m_tx_buf[2];
static uint8_t m_rx_buf[1 + 256 * 6];

extern volatile bool spi_xfer_done;

#define SPI_READ_REG 0x80

static uint8_t read_reg(lis2ds12_instance_t *p_lis, uint8_t reg)
{
    m_tx_buf[0] = SPI_READ_REG | reg;
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(p_lis->p_spi, m_tx_buf, 1, m_rx_buf, 2));
    while (!spi_xfer_done)
    {
        __WFE();
    }
    return m_rx_buf[1];
}

static void write_reg(lis2ds12_instance_t *p_lis, uint8_t reg, uint8_t value)
{
    m_tx_buf[0] = reg;
    m_tx_buf[1] = value;
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(p_lis->p_spi, m_tx_buf, 2, m_rx_buf, 0));
    while (!spi_xfer_done)
    {
        __WFE();
    }
}

void lis2ds12_set_odr(lis2ds12_instance_t * p_lis, lis2ds12_odr_t odr)
{
    uint8_t ctrl1 = read_reg(p_lis, LIS2DS12_REG_CTRL1) & ~(0x0f << 4);
    write_reg(p_lis, LIS2DS12_REG_CTRL1, ((odr & 0x0f) << 4) | ctrl1);
}

void lis2ds12_set_fs(lis2ds12_instance_t *p_lis, uint8_t fs)
{
    uint8_t ctrl1 = read_reg(p_lis, LIS2DS12_REG_CTRL1) & ~(0x03 << 2);
    write_reg(p_lis, LIS2DS12_REG_CTRL1, ((fs & 0x03) << 2) | ctrl1);
}

void lis2ds12_set_step_minths(lis2ds12_instance_t * p_lis, uint8_t minths)
{
    write_reg(p_lis, LIS2DS12_REG_STEP_MINTHS, minths & 0x3f);
}

void lis2ds12_set_step_debounce(lis2ds12_instance_t * p_lis, uint8_t debounce)
{
    uint8_t ctrl2 = read_reg(p_lis, LIS2DS12_REG_CTRL2);
    write_reg(p_lis, LIS2DS12_REG_CTRL2, 0x10 | ctrl2);
    write_reg(p_lis, LIS2DS12_REG_PEDOMETER_DEBOUNCE, debounce);
    write_reg(p_lis, LIS2DS12_REG_CTRL2, ctrl2);
}

void lis2ds12_set_step_delta(lis2ds12_instance_t * p_lis, uint8_t delta)
{
    uint8_t ctrl2 = read_reg(p_lis, LIS2DS12_REG_CTRL2);
    write_reg(p_lis, LIS2DS12_REG_CTRL2, 0x10 | ctrl2);
    write_reg(p_lis, LIS2DS12_REG_STEP_COUNT_DELTA, delta);
    write_reg(p_lis, LIS2DS12_REG_CTRL2, ctrl2);
}

void lis2ds12_set_hp_filter(lis2ds12_instance_t *p_lis, uint8_t state)
{
    uint8_t ctrl2 = read_reg(p_lis, LIS2DS12_REG_CTRL2) & ~(1 << 3);
    write_reg(p_lis, LIS2DS12_REG_CTRL2, ((state & 1) << 3) | ctrl2);
}

ret_code_t lis2ds12_init(lis2ds12_instance_t * p_lis)
{
    ASSERT(p_lis != NULL);

    // Reset
    write_reg(p_lis, LIS2DS12_REG_CTRL2, 0x40);

    uint8_t val = read_reg(p_lis, LIS2DS12_REG_WHO_AM_I);
    if (val != LIS2DS12_WHO_AM_I) {
        return NRF_ERROR_INTERNAL;
    }

    write_reg(p_lis, LIS2DS12_REG_CTRL1, 0x00);
    write_reg(p_lis, LIS2DS12_REG_CTRL2, (1 << 1) | (1 << 2)); // Disable I2C, enable address increment
    write_reg(p_lis, LIS2DS12_REG_CTRL4, (1 << 1)); // INT1 pin: FIFO threshold
    write_reg(p_lis, LIS2DS12_REG_CTRL5, (1 << 2)); // INT2 pin: step detection
    write_reg(p_lis, LIS2DS12_REG_FIFO_CTRL, (LIS2DS12_FIFO_CONTINUOUS << 5));
    write_reg(p_lis, LIS2DS12_REG_FIFO_THS, 60); // Trigger interrupt to fit in BLE packet
    write_reg(p_lis, LIS2DS12_REG_FUNC_CTRL, 1); // enable step counter

    lis2ds12_set_odr(p_lis, LIS2DS12_ODR_25HZ);
    lis2ds12_set_fs(p_lis, LIS2DS12_SCALE_2G);
    lis2ds12_set_hp_filter(p_lis, 0);

    return NRF_SUCCESS;
}

void lis2ds12_get_acc_fifo(lis2ds12_instance_t * p_lis, int16_t *acc_xyz, uint8_t size, uint8_t avg)
{
    int16_t *xyz = acc_xyz;
    if (size > 255) {
        size = 255;
    }
    for (int i = 0; i < size; ++i) {
        spi_xfer_done = false;
        m_tx_buf[0] = SPI_READ_REG | LIS2DS12_REG_OUT_X_L;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(p_lis->p_spi, m_tx_buf, 1, m_rx_buf, 7));
        while (!spi_xfer_done)
        {
            __WFE();
        }
        uint8_t x_low = m_rx_buf[1];
        uint8_t x_high = m_rx_buf[2];
        uint8_t y_low = m_rx_buf[3];
        uint8_t y_high = m_rx_buf[4];
        uint8_t z_low = m_rx_buf[5];
        uint8_t z_high = m_rx_buf[6];
        *xyz = (int16_t)(((uint16_t)x_high << 8) | (x_low));
        *xyz >>= 6;
        ++xyz;
        *xyz = (int16_t)(((uint16_t)y_high << 8) | (y_low));
        *xyz >>= 6;
        ++xyz;
        *xyz = (int16_t)(((uint16_t)z_high << 8) | (z_low));
        *xyz >>= 6;
        ++xyz;
    }
}

void lis2ds12_get_acc(lis2ds12_instance_t * p_lis, int16_t *acc_xyz)
{
    lis2ds12_get_acc_fifo(p_lis, acc_xyz, 1, 0);
}

uint16_t lis2ds12_get_step_counter(lis2ds12_instance_t * p_lis)
{
    m_tx_buf[0] = SPI_READ_REG | LIS2DS12_REG_STEP_L;
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(p_lis->p_spi, m_tx_buf, 1, m_rx_buf, 3));
    while (!spi_xfer_done)
    {
        __WFE();
    }

    uint8_t stepl = m_rx_buf[1];
    uint8_t steph = m_rx_buf[2];
    return ((uint16_t)steph << 8) | stepl;
}

uint16_t lis2ds12_get_fifo_level(lis2ds12_instance_t *p_lis)
{
    uint8_t high = read_reg(p_lis, LIS2DS12_REG_FIFO_SRC);
    high = !!(high & 0x20);
    uint8_t samples = read_reg(p_lis, LIS2DS12_REG_FIFO_SAMPLES);
    return ((uint16_t)high << 8) | samples;
}